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
godot --headless --path project --script Test/engine_reaction.gd
```

## Configuration at a glance

Add a `Vehicle`, assign it a `VehicleConfig`, and provide engine, gearbox, and
suspension resources. A vehicle also needs at least one `Axle`, two distinct
wheels with tire data per axle, steering data for steerable axles, and at least
one driven axle with `DifferentialData`. Aerodynamics, ESC, and turbo resources are
optional. A missing `esc_data` resource disables yaw stabilization; an assigned
`ESCData` can also be disabled with its `enabled` property. Invalid compositions
are reported during setup and remain inert.

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
| Tire curve scales | Longitudinal peak slip ratio; lateral response angle | 0.15 (15%); 10 degrees |
| Tire response | Slip response time constants at zero and 108 km/h contact forward speed | 42 ms / 10 ms |
| Aligning feedback | Total effective aligning lever; lever retained after rolloff | 40 mm; 50% retained |
| Steering | Time to reach 90% of a command; steering sensitivity half-speed; road feedback strength | 160 ms; 50 km/h; 0.5 |
| Limited-slip differential | Acceleration locking; engine-braking locking; preload torque | 70%; 30%; 25 Nm |

Lower response times make tire slip states or steering build faster. The tire
time constant is its 63% slip response time; its 90% time is about 2.303 times
that value. The existing `force_response_*` properties now filter slip before
force generation. Zero response time uses instantaneous slip. Final forces,
including brake hold, always respect the current combined grip capacity. The
rack is critically damped; its specified 90% time is measured with road feedback and friction disabled. A zero
steering half-speed disables speed-sensitive input.

Both friction curves remain ordinary Godot `Curve` resources, editable under
`TireData > Grip Curves`. Set their X domain to 0-2 and use Y as the fraction of
available grip: `(0, 0)` for rolling, `(1, 1)` for peak force, and X=2 for sliding
grip. Forward X is slip ratio divided by `peak_slip_ratio`; lateral X is slip
angle divided by `lateral_response_angle`. Forward slip uses
`(wheel_tread_speed - contact_forward_speed) / sqrt(contact_forward_speed^2 + 9)`
to stay finite and smooth near rest. ABS uses the same ratio and releases above
`peak_slip_ratio`. With no assigned curve, the corresponding response uses tanh.
Existing forward curves must be retuned for this ratio-based X axis.

The example curves have a nonzero starting slope and a rounded peak at X=1,
then retain 80% forward and 85% lateral force at X=2. Its lateral peak is at
9 degrees. Edit the curve points and tangents directly; duplicate the shared
curve resource to tune one axle independently.

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
Assign an optional `EngineReactionData` to `VehicleConfig.engine_reaction_data`
for engine-block reaction and chassis rumble. `Vehicle.engine_axis` is a normalized
chassis-local crankshaft axis: Z for longitudinal engines, X for transverse;
reverse it to reverse spin direction, or set it to zero to disable the effect.
Reaction strength scales signed engine torque before clutch coupling. Vibration
strength scales actual generated torque, including idle support, with no fixed
idle amplitude. Default rumble is 5 Hz at idle and 20 Hz at redline, bounded to
one fifth of the physics tick rate. `idle_vibration_multiplier` defaults to 1.5
at or below `idle_vibration_max_rpm` (1000 RPM), boosting vibration amplitude
before the torque cap and speed fade. Above that RPM, normal strength is used.
Both reaction and vibration retain full
strength through 5 km/h, then fade smoothly to zero at 30 km/h in forward or
reverse. Tune `speed_fade_start_kph` and `speed_fade_end_kph` to adjust that range.
Frequency continues to follow RPM while strength fades. Total torque is capped, averaged across
substeps, and applied once per tick. Inspect it through
`VehicleTelemetry.get_engine_reaction_telemetry()`. The example enables this
resource; vehicles without it retain their previous behavior.

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
