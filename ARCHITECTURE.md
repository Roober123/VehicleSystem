# VehicleSystem project architecture

This document expands on the project overview in the main README. Detailed
drivetrain equations and solver invariants live in the
[rotational drivetrain guide](../src/Drivetrain/README.md).

## Component ownership

`Vehicle` is the composition root and performs validated, one-shot setup.
`VehicleRunningGear` owns axles, wheels, suspension, steering, tires, traction
control, and aerodynamics. `VehicleDrivetrain` owns the engine, gearbox, clutch,
driveshaft, rotational network, and optional turbo. `Vehicle` also owns the
optional `VehicleStabilityControl`, configured by `ESCData`; aerodynamics
contains only drag and downforce.

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
Aerodynamics, ESC, and turbo resources are optional. Missing `esc_data` or
`ESCData.enabled == false` disables yaw-control torque. ESC parameters are
copied during setup and restart, like the other subsystem resources.

Setup reports all major missing-resource and composition errors together. An
invalid vehicle remains inert, and its configuration cannot be replaced after
successful initialization.

`DifferentialData` defaults to Open mode. Limited-slip defaults are 25 Nm
preload, 70% acceleration locking, 30% engine-braking locking, 20.944 Nm per
100 RPM of speed-sensitive coupling, and a 250 Nm maximum lock torque. These
authoring values compile to the existing 0.35/0.15 torque ratios and 2 Nm per
rad/s speed gain. In Limited Slip mode,
capacity is:

```text
min(max_lock_torque,
    preload_torque
    + active_lock_ratio * abs(transmitted_torque)
    + slip_sensitive_gain * abs(relative_speed))
```

Power uses the acceleration ratio when transmitted torque multiplied by carrier speed
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

`Vehicle` owns an optional `VehicleEngineReaction`, configured by
`EngineReactionData` during setup/restart. The engine caches positive generated
torque (combustion plus idle support) and signed self torque (including braking
and drag) before clutch coupling. Reaction opposes self torque; sinusoidal
vibration scales with generated torque. `Vehicle.engine_axis` defines normalized
chassis-local crankshaft direction; zero disables the effect. Frequency is
independent of vehicle speed; both reaction and vibration strength fade using
smoothstep from 5 to 30 km/h by default, based on absolute body velocity.
The fade thresholds are copied from the resource at setup/restart, and oscillator
phase remains continuous while travelling torque is suppressed. Frequency is
bounded by the chassis tick rate, and symmetric cap headroom preserves zero-mean
vibration at constant load. Each substep analytically averages the oscillator
and accumulates angular impulse; the vehicle applies one frame-average torque
through direct body state. Restart clears phase and telemetry. This is an
engine-block reaction approximation, with no elastic mounts or gearbox housing
reaction model. `get_engine_reaction_telemetry()` exposes the completed sample.

Gear changes are command based and support automatic and semi-automatic modes.
The drivetrain solve receives the substep duration, signed gearbox ratio,
clutch engagement, and clutch capacity directly.

`VehicleEngine` samples effective drive torque once per substep and reuses the
value for torque application and telemetry. Its rev limiter cuts combustion at
redline and resumes 150 RPM below redline. Driver throttle remains available
to the turbo while the limiter is active, so limiting does not behave like a
throttle lift. A 110% redline ceiling remains as an emergency bound.

## Running gear, telemetry, and skid marks

Frame phases apply suspension, aerodynamics, and ESC before drivetrain/tire
substeps. ESC retains the bicycle-model yaw target, lateral-acceleration limit,
yaw inertia estimate, torque ceiling, and two-grounded-wheel requirement.

Each wheel constructs an orthonormal contact frame, filters instantaneous slip
velocity and angle using contact forward speed, then generates and limits tire
forces using the current load and grip. Slip telemetry stays instantaneous;
transient states reset on contact loss. Brake hold shares the combined grip
limit. Each wheel combines suspension and tire forces with ABS and traction-control
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
