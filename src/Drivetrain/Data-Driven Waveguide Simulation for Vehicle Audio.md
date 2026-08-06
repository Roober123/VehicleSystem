# Data-driven waveguide audio (MVP)

The vehicle-audio MVP is a bounded procedural exhaust signal.  A fixed-memory
waveguide network receives source pressure injections, scatters them at graph
nodes, and exposes one or more node taps.  It is an acoustic signal model for
runtime audio, not a claim that the complete engine, exhaust, or cabin pressure
field is physically solved.

## Runtime contract

The native core uses fixed arrays and no polymorphic acoustic-node objects:

| Limit | Value |
| --- | ---: |
| Nodes | 32 |
| Guides | 48 |
| Pressure sources | 16 |
| Output taps | 4 |
| Delay storage | 32,768 samples (both directions) |
| Render rate used by `VehicleAudio` | 48,000 Hz |

Each guide is authored in metres.  During setup, `length_meters /
propagation_speed * sample_rate` is rounded to the nearest integer and clamped
to a minimum of one sample.  The default propagation speed is 343 m/s.  A
guide's two delay lines must fit the global storage bound; one guide therefore
cannot exceed 16,384 delay samples.  Broadband traversal attenuation is derived
once as `exp(-loss_per_meter * length_meters)`.  The authored
`high_frequency_loss_per_meter` is compiled into
`high_frequency_alpha = exp(-high_frequency_loss_per_meter * length_meters)`.
Each travelling direction keeps its own one-pole HF filter state, so the
compiled field attenuates high-frequency content independently on both guide
directions without adding render-time topology work.

Setup rejects empty or duplicate names, invalid endpoints, self-links,
non-finite or non-positive geometry, invalid gains, over-capacity graphs, and
reflection boundaries that do not have exactly one incident guide.  A source
and tap must point to an incident node.  Disconnected subgraphs are valid when
each subgraph is locally well formed.  A failed replacement leaves the prior
configured network intact.

At each sample the network reads the two travelling waves at every guide end.
For a `Junction`, pressure is the authored source injection plus the
area-weighted incident pressure multiplied by two; each outgoing wave is
`pressure - incoming`.  A `ReflectionBoundary` reports source pressure plus
`(1 + reflection)` times its area-weighted incident wave and returns the
authored reflection through its guide.  Reflection is bounded to -1..1.  A
`NODE_PRESSURE` tap samples post-scattering node pressure.  A
`BOUNDARY_RADIATION` tap is valid only on a single-guide reflection boundary
and reports `(1 - reflection) * incident` wave, both with the authored tap
gain.

Every source event is rendered as a fixed, normalized difference of two
exponentials.  `VehicleAudioFiringData.pulse_width_ms` controls the compiled
decay width (default 1.5 ms); the attack width is 15% of that width.  At event
time, RPM and throttle derive a bounded load
`clamp(0.55 * clamp(rpm / 8000) + 0.45 * clamp(throttle), 0, 1)`, then width
scale `clamp(1 - 0.35 * load, 0.65, 1)` and attack ratio
`clamp(0.15 - 0.04 * load, 0.10, 0.15)`.  The authored width remains the
baseline and scheduler phase/cadence is unchanged.  The Vehicle adapter also
scales event pressure by `0.1 + 0.9 * throttle` and supplies a resettable,
deterministic four-value gain cycle (`0.988, 1.012, 0.996, 1.004`); this is
gain-only variation with no RNG.  The native variation input is clamped to
`0.98..1.02`.  Normalization is derived from the continuous peak, and the two
bounded states are retired after eight authored widths.  This produces a
deterministic multi-sample combustion pulse without allocating in the sample
loop.

Junctions use area-weighted incident-wave scattering.  For a junction with
`nonlinearity = 0`, the linear pressure is unchanged; otherwise the pressure
sent to outgoing waves is `linear / (1 + nonlinearity * abs(linear))` (with a
finite bounded fallback).  The checked-in preset assigns `nonlinearity = 0.35`
only to the `collector` junction; all other junctions are linear.  A
`BOUNDARY_RADIATION` tap remains restricted to a single-guide reflection
boundary and reports `(1 - reflection) * incident`.

`WaveguideTopologyBuilder` is the native authoring helper.  It resolves stable
IDs from non-owning names, accepts junctions, reflection boundaries, guides,
sources, and taps, and can validate into a topology value or configure a
`WaveguideNetwork` directly.  Validation and physical-to-sample derivation are
setup work.  `process_sample`, `render`, and `render_all` use the already
configured fixed storage and caller-owned output buffers; the render path does
not allocate or resize.

## Godot resources and lifecycle

`VehicleAudioTopologyData` stores node and guide resources, source-to-node
name mappings and gains, plus the output node and gain.  `VehicleAudioFiringData`
stores parallel phase, source-name, and gain arrays.  `VehicleAudio::_ready()`
compiles those resources once into the native builder/network and scheduler;
missing resources or malformed arrays disable the node with one diagnostic.
The resource objects are not consulted by the sample loop after compilation.

`VehicleAudio` is an `AudioStreamPlayer3D` using a custom 48 kHz
`AudioStreamGenerator` with a 0.1-second buffer.  It resolves a target
`Vehicle` from the explicit property, parent, or sibling.  Each process pass
reads engine RPM and throttle, advances the firing scheduler, renders bounded
256-frame staging blocks while playback accepts them, and pushes the same tap
sample to left and right channels.  Pulse scale is `0.1 + 0.9 * throttle`.
The generated exterior signal then passes a fixed conditioner: a 20 Hz DC
blocker, a 6.5 kHz low-pass, and `0.6 * tanh(1.5 * sample)`.  The conditioner
owns only fixed scalar state.  Topology validation, delay/loss/HF coefficient
derivation, pulse coefficient compilation, and scheduler setup occur once
during setup; render uses fixed arrays and caller-owned buffers without
allocation or resizing.  `_exit_tree()` stops and clears playback, releases
the generator, and clears staging so scene re-entry compiles a fresh runtime.
Editor previews are not compiled.

## Four-stroke firing and source routing

The scheduler represents one 720-degree cycle.  Up to 16 authored phases are
accepted; each finite phase is normalized modulo 720 degrees and may route to
any valid source with any finite gain.  Crossing detection handles forward
progress and cycle wrap, so uneven phase spacing and arbitrary source routing
are supported.  `VehicleAudioFiringData` defaults to four phases at
0/180/360/540 degrees with source order `cylinder_1`, `cylinder_3`,
`cylinder_4`, `cylinder_2` (the conventional 1-3-4-2 order), all with gain 1.

## Preset and minimal setup

The checked-in preset pair is:

* `project/Audio/four_cylinder_exhaust_topology.tres`: nine nodes, eight
  guides, four cylinder source mappings, and a minimal runner-collector-
  chamber-neck-chamber exterior path.  The collector junction alone has
  `nonlinearity = 0.35`.  Runner HF loss is `0.45` per metre, collector-pipe
  HF loss is `0.28`, and tailpipe HF loss is `0.32`; output is a
  `BOUNDARY_RADIATION` tap at the open `tailpipe_open` boundary
  (`reflection = -1`).  This is a fixed-memory procedural ceiling, not a
  generalized muffler subsystem or a claim of physical muffler fidelity.
* `project/Audio/four_cylinder_firing.tres`: the default four phases and source
  order above, with the 1.5 ms default pulse width.

To use the MVP, add a `VehicleAudio` child to a `Vehicle`, assign topology and
firing resources (or assign equivalent resources in code), and ensure the
topology output node and every firing source name resolve.  Assign `target`
only when automatic parent/sibling discovery is not appropriate.  A valid
setup enables `is_audio_enabled`, installs an `AudioStreamGenerator`, and
starts playback.

## Verification and roadmap

The native `Test/audio_regression.cpp` suite covers bounds, delay derivation,
attenuation, per-direction HF filtering, normalized pulse shape/decay, junction
and boundary behavior, both tap modes, disconnected graphs, replacement
atomicity, resource fields, event-time width/attack shaping, bounded variation,
the fixed output conditioner, and arbitrary scheduler phases.  The Godot
`project/Test/vehicle_audio_smoke.tscn` fixture
validates the two presets, open-tailpipe output, enabled playback, buffered
frames with zero skips, Vehicle restart, and node re-entry.  Normal and test
debug builds, native audio and drivetrain regressions, the deterministic 48 kHz
engine traces, and the production grounded trace pass; verbose smoke reports
zero failures with no skip, leak, buffer, or `FAIL` patterns.  These are
deterministic setup/lifecycle and signal-contract checks; no subjective
listening result or fully physical pressure solution is claimed.  The only
known unrelated console warning is the CA-store certificate warning.

Turbo/compressor and turbine sources, drivetrain and intake sources, muffler
or catalyst models, cabin propagation, and other specialized acoustic elements
are deferred.  The current source model is intentionally limited to generic
junctions, reflection boundaries, pressure injections, and taps.  Future work
can add those inputs and filters behind the same bounded setup boundary without
changing the render contract.
