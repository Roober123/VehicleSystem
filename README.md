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
godot --headless --path project --script Test/tuning_resources.gd
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

## Tuning

The resources use Godot's native Inspector groups, range controls, unit suffixes,
and property tooltips. Advanced settings are in collapsible groups. Differential
LSD controls are only shown when mode is Limited Slip; their saved values survive
mode changes. There is no custom editor plugin.

| Resource | Main controls | Starting values |
|---|---|---|
| Tire grip | Forward/lateral coefficients; coefficient loss at twice reference wheel load | 1.0 / 1.0; 6.7% loss |
| Tire response | Force response time constants at zero speed and 108 km/h | 42 ms / 10 ms |
| Aligning feedback | Total effective aligning lever; lever retained after rolloff | 40 mm; 50% retained |
| Steering | Time to reach 90% of a command; steering sensitivity half-speed; road feedback strength | 160 ms; 50 km/h; 0.5 |
| Limited-slip differential | Acceleration locking; engine-braking locking; preload torque | 70%; 30%; 25 Nm |

Lower response times make forces or steering build faster. The tire time constant
is its 63% response time; its 90% time is about 2.303 times that value. Zero tire
response time applies forces immediately. The rack is critically damped; its
specified 90% time is measured with road feedback and friction disabled. A zero
steering half-speed disables speed-sensitive input.

Load loss describes the grip **coefficient**. For example, 10% loss at twice the
reference load gives 1.8 times the total force capacity, rather than twice the
capacity. The reference load is calculated from equal mass per axle and wheel.

Aligning trail is an effective torque lever: 40 mm gives 40 Nm per 1000 N lateral
force before slip rolloff and rack scaling. Retention controls how much of that
lever remains during a slide, not how much actual torque remains. Road feedback
is normalized against steering stiffness so changing response time preserves its
torque-to-angle scaling. Opposing feedback is capped at the driver's restoring
torque, while the rack's intrinsic damping remains active.

Differential locking percentages describe torque-driven wheel-torque imbalance,
not a percentage of wheel-speed lock. At 40% and 100 Nm axle input, the correction
capacity is 20 Nm, added to one wheel and subtracted from the other. Preload,
speed coupling, and the maximum torque cap also affect the result. Locked mode
remains a separate unbounded policy.

Numeric resource edits take effect after a successful `Vehicle.restart()`.
Duplicate shared resources when tuning one vehicle independently. Hover a field
for its meaning, units, and effects. The checked-in example has been migrated to
the new property names; other saved scenes and scripts using the removed names
must be updated. The example's steering uses a smooth 700 ms response, which is
close to its previous isolated 90% time but has a different transient shape.

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
