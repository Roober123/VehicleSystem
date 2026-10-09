# VehicleSystem

VehicleSystem is a native C++17 vehicle simulation for Godot 4, provided as a GDExtension. It includes a configurable drivetrain, suspension, steering, tire forces, driving assists, aerodynamics, telemetry, and skid marks.

## Features

- Configurable engines, clutches, gearboxes, differentials, and optional turbocharging
- Multi-axle drivetrains with per-wheel suspension, steering, and tire simulation
- ABS, traction control, ESC, and adjustable grip
- Engine reaction and chassis vibration
- Telemetry and skid marks

## Build

Requires Godot 4.7+, SCons, and a C++17 compiler.

```sh
scons -j11 target=template_debug
scons -j11 tests=1 target=template_debug
```

## Setup and tuning

Add a `Vehicle`, assign a `VehicleConfig`, and configure its engine, drivetrain, axles, wheels, and suspension through Godot resources. Optional systems can be added as needed.

Handling and drivetrain behavior are tuned in the Inspector, including tire grip curves. Duplicate shared resources to tune vehicles independently. Some resource changes take effect after calling `Vehicle.restart()`.

See [Project architecture](ARCHITECTURE.md) for an overview of the system and [Rotational drivetrain flow](src/Drivetrain/README.md) for drivetrain details. Godot test scenes and scripts are located in `project/Test/`.

## AI-assisted development

AI tools were used during parts of development and documentation.

## License

See [LICENSE.md](LICENSE.md).
