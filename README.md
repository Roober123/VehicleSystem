# VehicleSystem - Godot 4 GDExtension

VehicleSystem is a native C++17 vehicle simulation for Godot 4. `Vehicle` is
the composition root for a validated, one-shot setup. `VehicleRunningGear`
owns axles, wheels, suspension, steering, tires, traction control, and
aerodynamics. `VehicleDrivetrain` owns the engine, gearbox, clutch, driveshaft,
grouped driven-axle carrier coupling, and optional turbo. No exhaust subsystem
is part of the runtime composition.

## Features

- Configurable engine torque curve, idle/redline behavior, braking, and turbo.
- Command-based gearbox shifting with automatic and semi-automatic modes.
- Grouped shaft-to-axle-carrier coupling; equal wheel impulses preserve
  left/right differential speed.
- Per-wheel suspension, tire forces, ABS, steering, aerodynamics, and traction
  control.
- `VehicleTelemetry` snapshot consumer with cached wheel topology and bounded
  per-wheel arrays.
- `TireSkid` procedural ribbons backed by a fixed-size recycling pool. No fade
  or lifetime visual effect is promised.

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
at least one positive axle `drive_share` with `DifferentialData`. At most two
driven axles are supported; two driven axles additionally require
`center_differential_data`. Aerodynamics and turbo resources are optional.
Validation reports all missing major resources/topology errors and leaves an
invalid vehicle inert. Configuration cannot be replaced after initialization.

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
  Drivetrain/                 engine, gearbox, clutch, differential, coupling
  Resources/                  VehicleConfig, DifferentialData, and other resources
  axle.*, wheel.*              axle and wheel physics
Test/                         opt-in native regression
project/Test/                 Godot regression and production trace scenes
doc_classes/                  Godot class documentation inputs
godot-cpp/                    godot-cpp binding submodule
```

The coupling supports one or two weighted driven-axle carriers. It routes
pending shaft torque by normalized `drive_share`, then solves the primary
shaft/carrier coordinate, an optional two-axle center differential coordinate,
and each axle's left/right differential coordinate in that order. Axle setup
requires equal left/right wheel inertias; the effective axle carrier inertia
is `4 / (1/I_left + 1/I_right)`. Topology and solver snapshots are cached at
setup, and the solve path performs no hot-path allocation. `VehicleEngine`
samples effective drive torque once per substep and reuses it for application
and telemetry.
