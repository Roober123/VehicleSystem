# VehicleSystem

VehicleSystem is a native C++17 vehicle simulation for Godot 4. It provides a
validated vehicle composition, multi-axle drivetrain simulation, suspension,
steering, tire forces, driving assists, aerodynamics, telemetry, and skid marks
through GDExtension.

## Highlights

- Configurable engine torque, idle/redline behavior, braking, clutch, gearbox,
  differential, and optional turbo simulation.
- One to eight driven two-wheel axles with open, locked, or limited-slip
  differentials.
- Per-wheel suspension, steering, tire forces, ABS, traction control, and
  runtime grip adjustment.
- Transactional rotational constraint solving with atomic validation and no
  hot-path allocation.
- Vehicle telemetry and load-normalized skid marks for wheelspin, braking,
  lateral slip, and stationary burnouts.
- Designed to run at 120 Hz without substeps for typical vehicles, with
  substeps available for more complex multi-axle setups.

## Build and test

Prerequisites are Godot 4.7+, SCons, and a C++17 compiler. From the repository
root, build the extension and optional native tests with:

```shell
scons -j11 target=template_debug
scons -j11 tests=1 target=template_debug
```

The Windows debug extension is installed at
`project/bin/windows/VehicleSystem.windows.template_debug.x86_64.dll`.

SCons uses `.scons-cache/` as a shared local content cache. Build outputs also
include their platform, target, precision, and architecture in their filenames,
so switching between builds such as Windows and Web preserves both sets of
objects. Each configuration must be compiled once—Windows objects cannot be
used for Web—but switching back reuses that configuration's artifacts. Set the
`SCONS_CACHE` environment variable to use a different cache location.

Run the deterministic Godot regressions headlessly with:

```shell
godot --headless --path project Test/drivetrain_regression.tscn
godot --headless --path project Test/production_grounded_trace.tscn
```

## Configuration at a glance

Add a `Vehicle`, assign it a `VehicleConfig`, and provide engine, gearbox, and
suspension resources. A vehicle also needs at least one `Axle`, two distinct
wheels with tire data per axle, steering data for steerable axles, and at least
one driven axle with `DifferentialData`. Aerodynamics and turbo resources are
optional. Invalid compositions are reported during setup and remain inert.

See [Project architecture](ARCHITECTURE.md) for component ownership,
configuration rules, runtime ordering, limits, and notable simulation behavior.
See [Rotational drivetrain flow](src/Drivetrain/README.md) for the constraint
model, Schur-complement clutch solve, differential policies, transaction
semantics, and turbo model.

## Repository layout

```text
src/             GDExtension production code
Test/            opt-in native regression tests
project/Test/    Godot regression and production trace scenes
doc_classes/     Godot class documentation inputs
godot-cpp/       upstream godot-cpp binding submodule
```

## AI-assisted development

AI-assisted tools have been used in portions of the project's implementation
and documentation. Their output is treated as engineering input and is subject
to the same review and testing expectations as other contributions.

## License

VehicleSystem is provided under the terms in [LICENSE.md](LICENSE.md).
