# Drivetrain Schur-complement solve

This note explains the coupled clutch/primary projection used by
`ConstraintSolver::solve_clutch`. It assumes the reader knows angular velocity
but has not worked with constraint rows before.

## A constraint row in one paragraph

`VelocityConstraint` is a sparse row of a Jacobian, `J`. Each
`ConstraintTerm` contributes a body pointer and a scalar coefficient. If the
bodies' (predicted) angular velocities are collected in a vector `omega`, the
row's velocity error is

```text
e = J * omega = sum(coefficient * angular_velocity)
```

The solver stores one impulse `lambda` for the row. An impulse is angular
momentum (torque multiplied by seconds), not torque itself. With diagonal
inertia matrix `M`, one row changes the constrained velocity by
`J * M^-1 * J^T * lambda`, where each diagonal entry of `M^-1` is
`1 / body.inertia`. Therefore an uncoupled row requests

```text
lambda = -e / (J * M^-1 * J^T)
```

The implementation calls the denominator `inverse_mass`. It applies the
impulse to the transaction and reports the equivalent torque as
`lambda / dt`; a finite capacity in torque is converted to an impulse limit
`capacity * dt` before clamping.

## The two rows and their A/B/C terms

During `RotationalNetwork::configure`, the fixed **primary** row is built as

```text
shaft - sum(share * (left + right) / 2) = 0
```

Thus its Jacobian has `+1` for the shaft and `-share/2` for each wheel. For a
non-zero signed gearbox ratio, `solve_clutch_primary` creates the transient
**clutch** row

```text
engine - signed_ratio * shaft = 0
```

At the start of the phase, both rows are evaluated against the solver's
predicted velocities (the body's explicit-Euler prediction plus any impulses
already staged in this transaction). Write their residuals as `e_p` and `e_c`.
Define the scalar inverse-mass products

```text
A = J_p * M^-1 * J_p^T   (primary_mass)
B = J_p * M^-1 * J_c^T   (cross_mass)
C = J_c * M^-1 * J_c^T   (clutch_mass)
```

`cross_inverse_mass` only adds terms whose rows reference the same body, so
`B` is the shared shaft contribution for these two rows. With primary impulse
`lambda_p` and clutch impulse `lambda_c`, exact coupled projection is the
two-equation system

```text
A * lambda_p + B * lambda_c = -e_p
B * lambda_p + C * lambda_c = -e_c
```

These signs follow directly from the code's `residual + mass * impulse`
correction and the requested values being the negative residuals.

## Schur complement, matching `RotationalConstraint.cpp`

Eliminate the primary impulse from the first equation:

```text
lambda_p = -(e_p + B * lambda_c) / A
```

Substitution into the clutch equation gives the Schur complement

```text
(C - B^2 / A) * lambda_c
    = -(e_c - B * e_p / A)
```

Consequently the implementation computes

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

The solver rejects a non-finite or non-positive conditioned mass. This is the
usual requirement that the two rows have a usable independent direction after
the primary direction has been removed.

## Why clutch capacity is applied first

The clutch capacity is a torque limit, so `solve_clutch` clamps
`requested_clutch` to `[-capacity * dt, +capacity * dt]`. It then **recomputes**
the primary impulse with the applied clutch impulse:

```text
applied_primary = -(primary_residual
                    + cross_mass * applied_clutch)
                  / primary_mass;
```

This preserves the primary equation exactly even when the clutch cannot supply
its requested impulse. The clutch row may then retain residual slip, which is
reported as

```text
clutch_slip = clutch_residual
             + cross_mass * applied_primary
             + clutch_mass * applied_clutch;
```

If no clutch row is present (zero signed ratio), the primary row is solved as a
single unrestricted row. Both coupled impulses are inserted into a candidate
solver and copied back only after all checks succeed; a malformed term or
non-finite value cannot leave only one side staged.

### Small numerical example

Suppose a particular state has `A = 2`, `B = 1`, `C = 1.5`,
`e_p = 0.4`, and `e_c = -0.2` (arbitrary impulse/velocity units). Then

```text
C - B^2/A = 1.0
requested_clutch = -(-0.2 - 1*0.4/2) / 1.0 = 0.4
requested_primary = -(0.4 + 1*0.4) / 2 = -0.4
```

If the clutch capacity allows only `0.25` impulse, the applied clutch impulse
is `0.25`, and the primary is recomputed as `-(0.4 + 0.25)/2 = -0.325`.
The primary equation remains exact (`2*(-0.325) + 1*0.25 = -0.4`), while the
clutch retains `-0.15` residual slip. For `dt = 0.1`, an impulse of `0.25`
corresponds to `2.5` torque; the values `0.4`, `0.25`, `-0.4`, and `-0.325`
are impulses, not torques.

## Full execution order in `RotationalNetwork::solve`

All phases share one bounded `ConstraintSolver(dt)` transaction. Solve calls
only stage impulses; body torque fields are mutated once, in `commit()`.

1. **Primary/clutch phase.** Build the clutch row when the signed ratio is
   non-zero, derive capacity from non-negative clutch limit times engagement
   clamped to `[0, 1]`, and run the coupled Schur solve above. The staged
   primary impulse is returned as `primary_applied_torque` (`lambda_p / dt`),
   while clutch telemetry records requested/applied clutch torque and slip.
2. **Predicted differential phase.** For each configured axle, read predicted
   left and right wheel velocities (including phase-1 impulses and any prior
   differential impulses). Compute carrier velocity `(left + right) / 2`,
   relative slip `left - right`, and transmitted axle torque
   `(-primary_applied_torque) * share`. Open differentials skip a row; locked
   rows use unbounded capacity; limited-slip rows derive a bounded capacity
   from their policy. Each non-open row is `left - right = 0` and is solved as
   an ordinary scalar constraint.
3. **Atomic commit.** Validate every body's final torque sum and predicted
   velocity. Only then add each staged impulse as torque `impulse / dt` in one
   mutation pass. Any failure in setup, either solve phase, or validation leaves
   the bodies unchanged.

The order matters: differential capacity depends on the primary reaction and
the wheel speeds after the clutch/primary projection. Solving differentials
first would use stale velocities and a different transmitted torque, so it
would no longer represent the configured drivetrain for that substep.
