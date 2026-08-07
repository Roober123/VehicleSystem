# VehicleSystem project architecture

This document expands on the project overview in the main README. Detailed
drivetrain equations and solver invariants live in the
[rotational drivetrain guide](../src/Drivetrain/README.md).

## Component ownership

`Vehicle` is the composition root and performs validated, one-shot setup.
`VehicleRunningGear` owns axles, wheels, suspension, steering, tires, traction
control, and aerodynamics. `VehicleDrivetrain` owns the engine, gearbox, clutch,
driveshaft, rotational network, and optional turbo.

The main production areas are:

```text
src/
  vehicle.{h,cpp}          composition root and frame ordering
  VehicleRunningGear.*     cached running-gear phases
  VehicleDrivetrain.*      drivetrain composition and commands
  Drivetrain/              rotational bodies, constraints, and direct network
  Resources/               VehicleConfig and subsystem configuration resources
  axle.*, wheel.*          axle and wheel physics
```

## Configuration and validation

Each `Vehicle` accepts one `VehicleConfig`. Setup requires:

- engine, gearbox, and suspension resources;
- at least one child `Axle`;
- exactly two distinct wheels with tire data per axle;
- steering data on every steerable axle; and
- at least one positive axle `drive_share` with `DifferentialData`.

One through eight driven two-wheel axles are supported. Positive drive shares
are normalized as flat axle weights; there is no inter-axle differential. Each
axle independently selects an open, locked, or limited-slip left/right policy.
Aerodynamics and turbo resources are optional.

Setup reports all major missing-resource and composition errors together. An
invalid vehicle remains inert, and its configuration cannot be replaced after
successful initialization.

`DifferentialData` defaults to Open mode. Limited-slip defaults are 25 Nm
preload, a 0.35 power-lock ratio, a 0.15 coast-lock ratio, 2 Nm per rad/s of
slip-sensitive gain, and a 250 Nm maximum lock torque. In Limited Slip mode,
capacity is:

```text
min(max_lock_torque,
    preload_torque
    + active_lock_ratio * abs(transmitted_torque)
    + slip_sensitive_gain * abs(relative_speed))
```

Power uses the power ratio when transmitted torque multiplied by carrier speed
is non-negative; coast uses the coast ratio otherwise. These lock settings are
inactive in Open and Locked modes.

## Runtime organization

At each physics substep, `Vehicle::_integrate_forces` advances clutch logic and
engine torque, applies current wheel reaction torques, asks
`VehicleDrivetrain` to solve its rotational network, and then integrates the
engine, driveshaft, and wheels.

The drivetrain is configured once as a fixed, value-owned program.
`RotationalBody` stores bounded angular state for the engine, simulated
driveshaft, and wheels. Each `DrivenAxle` contains its two wheel bodies,
normalized drive share, and copied `DifferentialSettings`.

The rotational solve uses one bounded transaction with three phases:

1. Project the coupled clutch and primary drivetrain constraints.
2. Evaluate and solve axle differential rows using predicted wheel velocities.
3. Validate all final torques and velocities, then commit them atomically.

Solver calls stage impulses rather than modifying body torque immediately. A
failure in any phase leaves all bodies untouched. The runtime iterates fixed
arrays of at most 18 bodies (engine, shaft, and sixteen wheels), so the solve
does not allocate on the hot path. The complete row equations, coupled
Schur-complement derivation, and validation boundaries are documented in the
[drivetrain guide](../src/Drivetrain/README.md).

## Engine and gearbox behavior

Gear changes are command based and support automatic and semi-automatic modes.
The drivetrain solve receives the substep duration, signed gearbox ratio,
clutch engagement, and clutch capacity directly.

`VehicleEngine` samples effective drive torque once per substep and reuses the
value for torque application and telemetry. Its rev limiter cuts combustion at
redline and resumes 150 RPM below redline. Driver throttle remains available
to the turbo while the limiter is active, so limiting does not behave like a
throttle lift. A 110% redline ceiling remains as an emergency bound.

## Running gear, telemetry, and skid marks

Each wheel combines suspension and tire forces with ABS and traction-control
inputs. Its runtime `grip_multiplier` supports surface-dependent or gameplay
grip changes without replacing tire data.

`VehicleTelemetry` consumes snapshots with a cached wheel layout, bounded
per-wheel arrays, and wheel-reference helpers for RPM, force, and compact tire
telemetry dictionaries. Successful drivetrain solves retain scalar clutch
telemetry; neutral reports zero torque and slip with `slipping == false`.

`TireSkid` produces load-normalized marks for wheelspin, braking, lateral slip,
and stationary burnouts. It stores 2,048 cross-sections in an oldest-replacing
ring, samples moving marks every 0.10 m, and renders independent alpha-weighted
quads. Onset filters reject ordinary cornering scrub and brief shift
transients. Marks do not fade with time.
