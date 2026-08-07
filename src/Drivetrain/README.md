# Rotational drivetrain flow

`RotationalNetwork` is a setup-once, value-owned program for one engine,
one simulated driveshaft, and up to eight driven two-wheel axles. `configure`
copies the validated axle program and builds one fixed primary constraint; solving
does not discover topology or allocate.

## Bodies and constraints

Each `RotationalBody` owns angular velocity, inertia, pending torque, and
effective torque. An axle owns two wheel bodies, a positive drive share, and a
`DifferentialSettings` policy. Shares are normalized as flat weights, so the
primary coordinate is

```text
shaft_velocity - sum(share * (left_velocity + right_velocity) / 2) = 0
```

The corresponding `VelocityConstraint` has coefficient `+1` for the shaft and
`-share/2` for each wheel. Its scalar solve uses
`requested_impulse = -residual / sum(coefficient^2 / inertia)` and converts the
impulse to torque with `dt`; there is no inter-axle differential.

When the signed gearbox ratio is non-zero, the clutch constraint is
`engine_velocity - signed_ratio * shaft_velocity = 0`. Its torque capacity is
the nonnegative clutch limit multiplied by engagement clamped to `[0, 1]`.
`ConstraintSolver::solve_clutch` solves this constraint and the primary
constraint exactly: it clamps the clutch impulse first, then conditions the
primary impulse so the primary residual remains projected.

## Constraint rows and impulses

`VelocityConstraint` is a sparse row of a Jacobian, `J`. Each
`ConstraintTerm` contributes a body pointer and scalar coefficient. If the
bodies' predicted angular velocities form a vector `omega`, the row error is:

```text
e = J * omega = sum(coefficient * angular_velocity)
```

The solver stores one impulse `lambda` for the row. An impulse is angular
momentum (torque multiplied by seconds), not torque. With diagonal inertia
matrix `M`, an uncoupled row requests:

```text
lambda = -e / (J * M^-1 * J^T)
```

The implementation calls the denominator `inverse_mass`. It stages the impulse
in the transaction and reports equivalent torque as `lambda / dt`. A finite
torque capacity is converted to the impulse limit `capacity * dt` before it is
clamped.

## Coupled clutch-primary Schur solve

At the beginning of the clutch phase, both the fixed primary row and transient
clutch row are evaluated against predicted velocities. Write their residuals
as `e_p` and `e_c`, and define:

```text
A = J_p * M^-1 * J_p^T   (primary_mass)
B = J_p * M^-1 * J_c^T   (cross_mass)
C = J_c * M^-1 * J_c^T   (clutch_mass)
```

`cross_inverse_mass` includes only bodies shared by both rows, so `B` is the
shared shaft contribution here. With primary impulse `lambda_p` and clutch
impulse `lambda_c`, exact coupled projection gives:

```text
A * lambda_p + B * lambda_c = -e_p
B * lambda_p + C * lambda_c = -e_c
```

Eliminating the primary impulse produces the Schur complement:

```text
lambda_p = -(e_p + B * lambda_c) / A

(C - B^2 / A) * lambda_c
    = -(e_c - B * e_p / A)
```

This maps directly to `RotationalConstraint.cpp`:

```text
conditioned_mass = clutch_mass
                 - cross_mass * cross_mass / primary_mass;
requested_clutch = -(clutch_residual
                     - cross_mass * primary_residual / primary_mass)
                   / conditioned_mass;
requested_primary = -(primary_residual
                     + cross_mass * requested_clutch)
                   / primary_mass;
```

The solver rejects a non-finite or non-positive conditioned mass: after the
primary direction is removed, the clutch row must still have a usable
independent direction.

### Capacity ordering

The clutch impulse is clamped first to
`[-capacity * dt, +capacity * dt]`. The primary impulse is then recomputed from
the applied clutch impulse:

```text
applied_primary = -(primary_residual
                    + cross_mass * applied_clutch)
                  / primary_mass;
```

This keeps the primary equation exactly projected when the clutch cannot
supply its requested impulse. The clutch row may retain residual slip:

```text
clutch_slip = clutch_residual
             + cross_mass * applied_primary
             + clutch_mass * applied_clutch;
```

With a zero signed ratio there is no clutch row, so the primary is solved as
one unrestricted scalar row. For a coupled solve, both impulses are inserted
into a candidate solver and copied back only when every check succeeds; invalid
arithmetic cannot leave half of the pair staged.

### Numerical example

Suppose `A = 2`, `B = 1`, `C = 1.5`, `e_p = 0.4`, and `e_c = -0.2` in
consistent impulse/velocity units:

```text
C - B^2/A = 1.0
requested_clutch = -(-0.2 - 1*0.4/2) / 1.0 = 0.4
requested_primary = -(0.4 + 1*0.4) / 2 = -0.4
```

If capacity permits only `0.25` impulse, the primary is recomputed as
`-(0.4 + 0.25)/2 = -0.325`. The primary equation remains exact, while the
clutch retains `-0.15` residual slip. At `dt = 0.1`, an impulse of `0.25`
corresponds to `2.5` torque; the other numerical values above are impulses,
not torques.

## Differential phase

After that solve, each axle carrier is computed from the **predicted** left and
right velocities in the transaction (`(left + right) / 2`), and its relative
coordinate is `left - right`. Open axles omit a relative constraint; Locked
axles use unbounded capacity; Limited Slip derives bounded capacity from
preload, power/coast ratio, slip-sensitive gain, and maximum-lock torque.
Relative constraints are transient and exist only for the current solve.

## One transaction, three physical phases

`RotationalNetwork::solve` creates one `ConstraintSolver(dt)` transaction for
the substep. `solve` handles a scalar constraint, `solve_clutch` handles the exact
coupled clutch/primary constraints, and `predict_velocity` exposes the current
predicted wheel speeds for the differential phase. These solve methods only
accumulate impulses in the transaction; they do not mutate body torques.

1. **Clutch-primary:** solve the coupled constraints against the shaft. The
   staged primary impulse is exposed as `primary_applied_torque` (`lambda_p /
   dt`), while telemetry retains requested and applied clutch torque and slip.
2. **Differentials:** use predicted left/right wheel speeds to evaluate each
   axle policy. Transmitted axle torque is
   `(-primary_applied_torque) * share`; solve each non-open relative constraint.
3. **Commit:** `commit` validates every final torque sum and predicted velocity,
   then adds all body torques in one mutation pass. A failed phase leaves bodies
   untouched.

The ordering is significant: differential capacity depends on the primary
reaction and on wheel speeds after the clutch-primary projection. Solving the
differentials first would use stale velocities and a different transmitted
torque for that substep.

`Vehicle::_integrate_forces` advances clutch logic and engine torque, applies
the current wheel reaction torques, calls `VehicleDrivetrain::solve_network`
for each substep, and integrates the engine, shaft, and wheels afterward.
`solve_network` passes the substep `dt`, effective signed ratio, engagement,
and clutch capacity directly to the network.

The transaction covers at most 18 unique bodies (engine, shaft, and sixteen
wheels). Runtime checks protect divisions, products, capacity clamps, and
accumulation from arithmetic failure after setup preconditions have been
established.

## Optional turbo

`VehicleDrivetrain` owns one `Turbo` value, and a valid
`VehicleConfig.turbo_data` resource enables it on the engine. With no turbo
resource, the engine keeps its naturally aspirated air-charge ratio of `1.0`.
`TurboData` exposes only `max_boost_bar` (bar above atmospheric, default
`1.0`), `full_boost_rpm` (RPM, default `3000`), and `lag_seconds` (seconds,
default `0.6`); response constants remain internal to the runtime. Values are
authored within their Inspector ranges and copied from a valid configuration.
`Turbo::update` assumes its caller supplies a positive physics timestep.

For a valid configuration, the runtime maintains normalized shaft energy in
`[0, 1]` and delivered boost in bar. Each substep computes a normalized exhaust
target from effective throttle, normalized base-engine torque, and
`clamp(engine_rpm / full_boost_rpm, 0, 1)`. Base-engine load shapes spool below
the full-boost point, with its influence fading as the RPM ratio approaches
one. Sustained full throttle at `full_boost_rpm` therefore targets shaft energy
`1.0` and `max_boost_bar`, regardless of torque-curve shape.

Shaft energy approaches its target exponentially using `lag_seconds`; on lift,
its decay time doubles so stored energy persists briefly. The compressor target
is `max_boost_bar * shaft_energy^2`, while delivered boost uses separate
internal rise and rapid-vent time constants. Closing the throttle vents boost
much faster than shaft energy decays. The engine converts bounded boost to air
charge with `1.0 + 0.85 * boost_bar` and reuses the sampled base torque for the
same substep.

## Validation ownership

| Boundary | Checks | Why it remains |
| --- | --- | --- |
| `configure` (once) | Distinct valid engine/shaft/wheel bodies, finite positive inertias, 1-8 axles, positive shares, valid differential settings, and fixed-constraint construction | Topology and policy become immutable inputs for every substep. |
| Public `solve` | Configured state plus finite `dt > 0`, signed ratio, engagement, and clutch limit | Rejects invalid external frame/control values at the API boundary. |
| `ConstraintSolver` constraint operations | Trust those preconditions; guard finite residuals/masses, positive conditioned mass, finite impulses/slips, and bounded accumulation | Runtime values can still overflow or become infeasible while constraints are solved. |
| `commit` | Final torque sums and predicted velocities are finite before any mutation | Preserves atomic failure behavior: no partial drivetrain update. |
