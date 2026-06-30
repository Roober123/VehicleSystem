# VehicleSystem — Godot 4 GDExtension

A fully simulated vehicle physics system for Godot 4, built as a native GDExtension in C++. Features a complete drivetrain with engine, clutch, gearbox, differential, suspension, tires  aerodynamics, turbo, traction control, ABS.

Built on the [godot-cpp](https://github.com/godotengine/godot-cpp) template.

## Features
- **Substepping** - user can use substepping for better stability without bumping up the engine physics ticks. For best quality it is recommended to use 120hz in engine physics and 2 substeps.
- **Engine** — Torque curve, rev-limiter, idle controller, internal friction, engine braking
- **Turbo** — Spool-up/down dynamics, boost pressure curve, configurable via `TurboData`
- **Clutch & Gearbox** — Automatic and semi-automatic modes, configurable ratios, shift timing, clutch engagement simulation
- **Drivetrain** — Driveshaft, per-axle differentials (Open, LSD, Torsen), shaft-to-wheel coupling constraint
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
│   ├── Drivetrain/          # Engine, clutch, gearbox, differentials, turbo
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


