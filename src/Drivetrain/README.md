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
`ConstraintSolver::solve_clutch` solves this constraint with the primary constraint exactly:
it clamps the clutch impulse first, then conditions the primary impulse so the
primary residual remains projected.

See [the Schur-complement solve guide](SCHUR_COMPLEMENT_SOLVE.md) for the
coupled equations, capacity ordering, and transaction details.

After that solve, each axle carrier is computed from the **predicted** left and
right velocities in the transaction (`(left + right) / 2`), and its relative
coordinate is `left - right`. Open axles omit a relative constraint; Locked axles use
unbounded capacity; Limited Slip derives bounded capacity from preload,
power/coast ratio, slip-sensitive gain, and maximum-lock torque. Relative constraints
are transient and exist only for the current solve.

## One transaction, three physical phases

`RotationalNetwork::solve` creates one `ConstraintSolver(dt)` transaction for
the substep. `solve` handles a scalar constraint, `solve_clutch` handles the exact
coupled clutch/primary constraints, and `predict_velocity` exposes the current
predicted wheel speeds for the differential phase. These solve methods only
accumulate impulses in the transaction; they do not mutate body torques.

1. **Clutch-primary:** solve the coupled constraints against the shaft and retain
   clutch telemetry.
2. **Differentials:** use predicted left/right wheel speeds to evaluate each
   axle policy and solve each non-open relative constraint.
3. **Commit:** `commit` validates every final torque sum and predicted velocity,
   then adds all body torques in one mutation pass. A failed phase leaves bodies
   untouched.

`Vehicle::_integrate_forces` advances clutch logic and engine torque, applies
the current wheel reaction torques, calls `VehicleDrivetrain::solve_network`
for each substep, and integrates the engine, shaft, and wheels afterward.
`solve_network` passes the substep `dt`, effective signed ratio, engagement,
and clutch capacity directly to the network.

The transaction covers at most 18 unique bodies (engine, shaft, and sixteen
wheels). Runtime checks protect divisions, products, capacity clamps, and
accumulation from arithmetic failure after setup preconditions have been
established.

## Validation ownership

| Boundary | Checks | Why it remains |
| --- | --- | --- |
| `configure` (once) | Distinct valid engine/shaft/wheel bodies, finite positive inertias, 1-8 axles, positive shares, valid differential settings, and fixed-constraint construction | Topology and policy become immutable inputs for every substep. |
| Public `solve` | Configured state plus finite `dt > 0`, signed ratio, engagement, and clutch limit | Rejects invalid external frame/control values at the API boundary. |
| `ConstraintSolver` constraint operations | Trust those preconditions; guard finite residuals/masses, positive conditioned mass, finite impulses/slips, and bounded accumulation | Runtime values can still overflow or become infeasible while constraints are solved. |
| `commit` | Final torque sums and predicted velocities are finite before any mutation | Preserves atomic failure behavior: no partial drivetrain update. |
