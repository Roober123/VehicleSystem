# VehicleSystem - Godot 4 GDExtension

VehicleSystem is a native C++17 vehicle simulation for Godot 4. `Vehicle` is
the composition root for a validated, one-shot setup. `VehicleRunningGear`
owns axles, wheels, suspension, steering, tires, traction control, and
aerodynamics. `VehicleDrivetrain` owns the engine, gearbox, clutch, driveshaft,
rotational network, and optional turbo. No exhaust subsystem is part of the
runtime composition.

## Features

- Configurable engine torque curve, idle/redline behavior, braking, and turbo.
- Command-based gearbox shifting with automatic and semi-automatic modes.
- Direct `RotationalNetwork` configuration through value-owned `DrivenAxle` and
  `DifferentialSettings` records.
- Transactional `RotationalNetwork` solve: one `ConstraintSolver(dt)` uses
  `solve_clutch` for the exact coupled shaft/clutch rows, `solve` for
  differential rows, and `predict_velocity` for current transaction state;
  `commit` validates final torque and predicted velocity before mutating bodies.
- Per-wheel suspension, tire forces, ABS, steering, aerodynamics, and traction
  control. Each `Wheel` exposes a runtime `grip_multiplier` for surface or
  gameplay grip changes.
- `VehicleTelemetry` snapshot consumer with cached wheel layout, bounded
  per-wheel arrays, and wheel-reference helpers for RPM, force, and a compact
  per-tire telemetry dictionary.
- Load-normalized skid marks for wheelspin, braking, lateral slip, and
  stationary burnouts. `TireSkid` keeps 2,048 cross-sections in an
  oldest-replacing ring, samples moving marks every 0.10 m, and renders
  independent alpha-weighted quads. Its internal onset filters ordinary
  cornering scrub and brief shift transients. Marks do not fade with time.

## Getting started

Prerequisites are Godot 4.7+, SCons, and a C++17 compiler. From this template
root, build the supported debug variants with:

```shell
scons -j11 target=template_debug
scons -j11 tests=1 target=template_debug
```

The selected extension is installed at
`project/bin/windows/VehicleSystem.windows.template_debug.x86_64.dll`.

Run the deterministic regression and production trace headlessly with:

```shell
godot --headless --path project Test/drivetrain_regression.tscn
godot --headless --path project Test/production_grounded_trace.tscn
```

## Configuration

Assign one `VehicleConfig` resource to each `Vehicle`. Setup requires engine,
gearbox, and suspension resources; at least one child `Axle`; exactly two
distinct wheels and tire data per axle; steering data on steerable axles; and
at least one positive axle `drive_share` with `DifferentialData`. One through
eight driven two-wheel axles are supported; positive shares are normalized as
flat axle weights with no inter-axle differential. Each axle's differential
mode independently controls its left/right relative row.
Aerodynamics and turbo resources are optional. Validation reports all missing
major resource/composition errors and leaves an invalid vehicle inert.
Configuration cannot be replaced after initialization.

`DifferentialData` defaults to Open mode. Limited-slip defaults are 25 Nm
preload, 0.35 power-lock ratio, 0.15 coast-lock ratio, 2 Nm per rad/s
slip-sensitive gain, and a 250 Nm maximum lock torque. In Limited Slip mode,
capacity is `min(max_lock_torque, preload_torque + active_lock_ratio *
abs(transmitted_torque) + slip_sensitive_gain * abs(relative_speed))`; power
uses the power ratio when transmitted torque times carrier speed is non-negative
and coast uses the coast ratio otherwise. These lock fields are inactive in
Open and Locked modes.

## Project structure

```text
src/                         GDExtension production code
  vehicle.{h,cpp}             composition root and frame ordering
  VehicleRunningGear.*        cached running-gear phases
  VehicleDrivetrain.*         drivetrain composition and commands
  Drivetrain/                 rotational bodies, rows, direct network
  Resources/                  VehicleConfig, DifferentialData, and other resources
  axle.*, wheel.*              axle and wheel physics
Test/                         opt-in native regression
project/Test/                 Godot regression and production trace scenes
doc_classes/                  Godot class documentation inputs
godot-cpp/                    godot-cpp binding submodule
```

`RotationalBody` stores bounded angular state for the engine, simulated
driveshaft, and wheels. `DrivenAxle` carries two wheel bodies, a positive drive
share, and value `DifferentialSettings`; `RotationalNetwork::configure`
receives the engine body, shaft body, fixed axle array, and axle count directly.
Setup accepts one through eight driven axles and normalizes their shares as flat
weights. The primary row distributes torque directly across those axle weights,
with no inter-axle differential. Wheel inertias may be unequal; each body only
needs finite positive inertia.

The detailed row equations, carrier/relative coordinates, three solve phases,
and validation ownership are documented in
[`src/Drivetrain/README.md`](src/Drivetrain/README.md). In brief, the primary
coordinate is `shaft - sum(share * (left + right) / 2)`; a non-neutral clutch adds
the row `engine - signed_ratio * shaft` with engagement-scaled capacity. The
exact coupled solve clamps clutch impulse first, then projects the primary
residual. Axle policies use predicted wheel speeds from the same transaction,
and one atomic commit validates final torque and predicted velocity before
mutating bodies.

At each substep, `RotationalNetwork::solve` runs three physical phases in one
bounded transaction: coupled clutch-primary solving, differential rows from
predicted carrier/relative coordinates, then `commit`. Solver methods only
accumulate impulses; body torques remain unchanged until commit. Setup validates
topology once; the public solve validates its external step and controls; row
operations retain only arithmetic-feasibility guards. The runtime iterates
setup-copied fixed arrays (up to 18 bodies: engine, shaft, and sixteen wheels)
with no hot-path allocation. Successful solves retain scalar `ClutchTelemetry`;
neutral torque and slip are zero and `slipping == false`.
`VehicleEngine` samples effective drive torque once per substep and reuses it
for application and telemetry. The network solve receives `dt`, signed ratio,
clutch engagement, and clutch capacity as direct arguments.
The engine rev limiter cuts combustion torque at redline and resumes 150 RPM
below it. This hysteresis replaces the former fixed-duration cut, while driver
throttle remains available to the turbo so limiter activity does not behave
like throttle lift. A 110% redline ceiling remains only as an emergency bound.
