# VehicleSystem — Godot 4 GDExtension

A fully simulated vehicle physics system for Godot 4, built as a native GDExtension in C++. Features a complete drivetrain with engine, clutch, gearbox, open differential, suspension, tires, aerodynamics, turbo, traction control, and ABS.

Built on the [godot-cpp](https://github.com/godotengine/godot-cpp) template.

## Features
- **Substepping** - user can use substepping for better stability without bumping up the engine physics ticks. For best quality it is recommended to use 120hz in engine physics and 2 substeps.
- **Engine** — Torque curve, rev-limiter, idle controller, internal friction, engine braking
- **Turbo** — Spool-up/down dynamics, boost pressure curve, configurable via `TurboData`
- **Clutch & Gearbox** — Automatic and semi-automatic modes, configurable ratios, shift timing, clutch engagement simulation
- **Drivetrain** — Driveshaft, per-axle open differential (the only differential implementation currently available), shaft-to-wheel coupling constraint
- **Suspension** — Per-wheel spring-damper, anti-roll bars, configurable via `SuspensionData`
- **Tires** — Custom friction curves, relaxation, self-aligning torque, ABS
- **Steering Rack** — P-D controller with inertia and friction, speed-sensitive, self-aligning torque feedback
- **Aerodynamics** — Drag, downforce (per-axle distribution), yaw damping
- **Traction Control** — Slip-based torque reduction
- **Skid Marks** — Procedural mesh ribbons via `TireSkid` (auto-cleanup, no runtime allocations)
- **Exhaust FX** — Smoke, heat, and flame probability for visual effects
- **Telemetry** — `VehicleTelemetry` node for UI/HUD: RPM, speed, gear, clutch, torque, boost, wheel speeds

## Getting Started

### Prerequisites

- Godot 4.7+
- SCons (`pip install scons`)
- C++17 compiler (MSVC, GCC/MinGW, or Clang)

### Build

```shell
scons -j4
```

The compiled library will be placed in `project/bin/`. Open the `project/` folder in Godot to test.

### Cached debug normal/test variants

Run these commands from `godot-cpp-template/` (the template root):

```shell
scons -j11 target=template_debug
scons -j11 tests=1 target=template_debug
```

`tests` is a SCons `BoolVariable` (and appears in `scons --help`). It is
supported only with `target=template_debug`. The first command selects the
normal debug DLL; the second selects the test-enabled DLL. Both variants are
linked persistently under `bin/windows/` as
`VehicleSystem.windows.template_debug.x86_64.normal.dll` and
`VehicleSystem.windows.template_debug.x86_64.tests.dll`. The selected variant
is copied by `InstallAs` to the canonical
`project/bin/windows/VehicleSystem.windows.template_debug.x86_64.dll` path
referenced by `project/bin/VehicleSystem.gdextension`.

The root build keeps the environment returned by `godot-cpp/SConstruct`
immutable. `godot-cpp` is already a static debug archive at
`godot-cpp/bin/libgodot-cpp.windows.template_debug.x86_64.a` and remains a
normal dependency. VehicleSystem production object
nodes are shared by both variants; only test registration and regression
objects use the test define and the test-only `.tests.os` suffix. After both
variants have been built once, warm normal/test switching performs zero
compile, archive, or link actions and one canonical install/copy.

An optional `SCONS_CACHE`/`CacheDir` can supplement clean rebuilds, but is not
required for variant reuse and does not replace normal dependency tracking.
The workspace path contains a historical `OneDrive` name; synchronization is
disabled for this project. This cache work is deliberately limited to
`template_debug`; no `template_release` command or artifact was touched, and
no release-specific behavior or branch was changed.

To select the normal or test DLL, rerun the corresponding command above. If a
variant or the godot-cpp archive is missing or stale, SCons rebuilds it through
the normal dependency graph. If generated output is malformed, verify the
exact debug target, move only that target aside recoverably, and rerun the
matching command; do not delete the shared godot-cpp archive blindly.

### Drivetrain regression checks

The deterministic drivetrain regression runner is opt-in and is excluded from normal extension builds. From the template root, build the test-enabled debug extension and run its scene headlessly:

```shell
scons -j11 tests=1 target=template_debug
godot --headless --path project Test/drivetrain_regression.tscn
```

### Platform-specific

| Platform | Build target |
|----------|-------------|
| Windows  | `scons platform=windows` |
| Linux    | `scons platform=linux` |
| macOS    | `scons platform=macos` |

## Project Structure

```
├── src/                    # Extension source code
│   ├── vehicle.h/cpp       # Main Vehicle node (RigidBody3D)
│   ├── axle.h/cpp           # Axle node (groups wheels)
│   ├── wheel.h/cpp          # Wheel node (suspension + tire forces)
│   ├── SteeringRack.h/cpp   # Steering rack dynamics
│   ├── TireSkid.h/cpp       # Skid mark system
│   ├── VehicleAerodynamics.h/cpp
│   ├── ExhaustSystem.h/cpp
│   ├── TractionControl.h/cpp
│   ├── VehicleTelemetry.h/cpp
│   ├── Drivetrain/          # Engine, clutch, gearbox, differential, turbo
│   ├── Resources/           # Godot Resource classes (SuspensionData, TireData, etc.)
│   └── register_types.cpp   # GDExtension entry point
├── project/                # Godot test project
├── doc_classes/            # XML documentation for the Godot doc system
└── godot-cpp/              # godot-cpp submodule
```

## Configuration

All vehicle parameters are configured via Godot **Resource** files:

| Resource | Description |
|----------|-------------|
| `SuspensionData` | Spring rate, damping, anti-roll bar |
| `TireData` | Friction, radius, brake power, slip angle, relaxation |
| `VehicleEngineData` | Torque curve, RPM limits, inertia, drag |
| `GearboxData` | Gear ratios, final drive, clutch, shift time, auto/manual |
| `TurboData` | Spool rates, boost pressure, RPM range |
| `VehicleAerodynamicsData` | Drag, downforce, yaw damping |
| `SteeringRackData` | Gains, inertia, friction, max angle |


## Documentation

Generate GDExtension docs with:

```shell
scons doc
```
