# Vehicle tuning review

This document records the original inspection and design proposal. The implementation now provides native Inspector groups and unit hints, tire response times and load-loss percentages, total/retained aligning trail, a critically damped steering response time with a configurable half-speed, and differential locking percentages with mode-dependent visibility. Tire smoothing is exponential, and steering uses an exact critically damped update with feedback scaled against stiffness. See README.md for the active interface. Graph previews, calibrated presets, configurable response shapes, and an editor tuning panel remain proposals. Historical source descriptions and calculations below describe the system before this implementation; existing values are not validated real-world specifications.

## Recommended direction

Author parameters in terms of observable behavior, with units and a small number of controls per resource. Compile those controls into bounded runtime settings at setup. Use one authoritative representation; do not expose editable derived coefficients alongside their authoring controls.

The suspension already provides a useful pattern: rest compression and damping ratio are converted into spring stiffness and damping using vehicle mass in `src/axle.cpp`. Apply that approach to steering and tire tuning.

Presets should be editable starting points, with their resulting values visible. Do not make a preset an additional multiplier on individual settings. Validate named road, sport, and drift presets with driving scenarios before describing them as tuned defaults.

## 1. Tire force response

### Current behavior

`src/wheel.cpp::_apply_relaxation` uses a time constant interpolated between `relaxation_low` at zero speed and `relaxation_high` at 30 m/s (108 km/h). Both longitudinal and lateral forces use the same constant. Speed comes from chassis linear-velocity magnitude, rather than local wheel rolling speed.

Defaults are 0.042 s and 0.010 s. The update uses `alpha = min(dt / tau, 1)`, so its actual response changes with timestep. A constant smaller than the substep becomes an immediate response. These controls are force smoothing, not measured tire relaxation lengths.

### Proposed controls

| Control | Meaning | Existing default expressed clearly |
|---|---|---|
| Force response at low speed (ms) | Time constant: approximately 63% of a force change | 42 ms |
| Force response at 108 km/h (ms) | Same measure at and above the current transition speed | 10 ms |

Explain that lower values make force build faster and higher values delay force changes. Show a read-only 90% response time and a force step-response graph. For an exponential first-order response, the 90% time is `ln(10) * tau`: approximately 97 ms and 23 ms for the existing defaults. This is a conversion for the proposed exponential update, not an exact measurement of the current discrete update.

Use `alpha = -expm1(-dt / tau)` for positive `tau`, with an explicit immediate-response case for zero. This gives consistent response to a constant target across substep sizes. It does not make the entire coupled vehicle timestep independent. It also changes current behavior and requires regression verification.

Keep two speed anchors initially: replacing them with one slider would discard useful control. A later physical response mode could use relaxation length and local rolling speed, but that changes the model rather than merely improving labels. Real tire transient models relate time constant to relaxation length divided by forward speed; this relation is documented in the [experimental relaxation-length study](https://link.springer.com/article/10.1007/s11012-023-01684-z). Its bicycle measurements should not be used as car-tire defaults.

## 2. Tire load sensitivity

### Current behavior

`src/wheel.cpp::_get_load_sensitivity_scale` computes:

```text
load_ratio = max(normal_load / reference_load, 0.001)
grip_scale = clamp(load_ratio ^ (-exponent), 0.75, 1.25)
```

The reference load is calculated during setup from equal mass per axle and per wheel (`src/VehicleRunningGear.cpp::setup`, `src/axle.cpp::compute_suspension_parameters`). It is not an independently specified tire nominal load or a calculation of the actual static weight distribution from the center of mass.

The inspector exposes exponent 0-0.3. `doc_classes/TireData.xml` currently lacks a member entry for this property.

### Proposed control

**Grip coefficient loss when wheel load doubles (%)**, specifically relative to the setup reference load. Show that reference load in newtons as read-only context.

```text
loss_fraction = 1 - 2 ^ (-exponent)
exponent = -ln(1 - loss_fraction) / ln(2)
```

| Current exponent | Loss at twice reference load | Total force capacity at twice reference load |
|---|---|---|
| 0.00 | 0% | 2.000 times reference capacity |
| 0.07 (car example) | 4.74% | 1.905 times reference capacity |
| 0.10 (default) | 6.70% | 1.866 times reference capacity |
| 0.20 | 12.94% | 1.741 times reference capacity |
| 0.30 | 18.77% | 1.625 times reference capacity |

The tire still supports more total force when loaded more heavily; the force increases less than proportionally. Do not label this simply “grip loss,” which could suggest that doubling weight decreases total grip force.

Initially preserve the existing exponent range: the equivalent percentage range is 0-18.77%. Extending it requires reviewing the existing 0.75-1.25 clamp. The percentage formula is exact at twice reference load within the current range; it is not the loss for every doubling at arbitrary loads where the clamp may already be active.

Preview both coefficient and force against wheel load, including the clamp. A later nominal-load option could separate tire characteristics from the vehicle on which they are mounted. It should be an explicit model choice.

## 3. Mechanical and pneumatic trail

### Current behavior

`src/wheel.cpp::_compute_sat` computes self-aligning torque from final lateral force and:

```text
u = clamp(abs(slip_angle) / peak_slip_angle, 0, 1)
trail = mechanical_trail + pneumatic_trail * (1 - u)^2
```

The defaults are 20 mm mechanical trail and 20 mm pneumatic trail. Mechanical trail remains after the pneumatic contribution disappears. Pneumatic trail describes the lateral-force resultant offset; see [MathWorks' tire force and torque documentation](https://www.mathworks.com/help/sm/ref/magicformulatireforceandtorque.html). This project's mechanical parameter is an effective constant contribution, not a caster/steering-axis geometry calculation.

The axle sums wheel moments, and the rack multiplies them by `sat_gain`. Consequently, changing total trail and changing rack feedback gain both scale the feedback reaching the rack. Increasing both makes their effects multiply.

### Proposed controls

| Control | Meaning | Existing default expressed clearly |
|---|---|---|
| Effective aligning lever at small slip (mm) | Torque generated per unit lateral force before pneumatic rolloff | 40 mm |
| Aligning lever retained at large slip (%) | Residual lever after pneumatic rolloff | 50% |

```text
mechanical_trail = total_trail * retained_fraction
pneumatic_trail = total_trail * (1 - retained_fraction)
```

Retained percentage is a lever ratio, not the fraction of actual torque retained: lateral force also changes with slip. At 40 mm total trail, 1000 N lateral force produces up to 40 Nm at small normalized slip and 20 Nm after rolloff, before the rack gain and authority cap. At exactly zero slip, the default tire model generates no lateral force and hence no aligning torque.

Show trail and torque versus slip angle. Advanced inspection can show the derived mechanical and pneumatic contributions. For an eventual geometry-based steering model, mechanical trail belongs with steering geometry and pneumatic trail with the tire; that ownership change is broader than this authoring improvement.

Keep raw rack feedback multiplication advanced. A user should have one primary strength control and one shape/retention control, with clear information about their combined effect.

## 4. Steering rack

### Current behavior

`src/SteeringRack.cpp` exposes inertia, proportional gain, derivative gain, damping, friction, maximum angle, and SAT gain. In the continuous linear approximation without SAT and friction:

```text
I * angle_acceleration + (derivative_gain + damping) * angle_velocity
    + proportional_gain * angle = proportional_gain * target_angle
```

Derivative gain and damping therefore both contribute to damping, although the current discrete integration applies them differently. The defaults have a linear damping ratio of approximately 1.37. The example changes proportional gain from 400 to 100 while keeping the default damping coefficients, increasing the ratio to approximately 2.74.

An isolated Python reproduction of the current update at 1/240 s, zero speed, no SAT, default friction, and a full 35-degree step reaches 90% in approximately 163 ms with default gain and 692 ms with gain 100. These are calculations of the inspected rack update, not driving-test results. The car example has a 33-degree maximum angle, so these calculations are illustrative rather than exact measurements of that scene.

The rack also reduces steering command by `1 / (1 + speed_kph * 0.02)`. The sensitivity coefficient is hardcoded: at 50 km/h it halves the target angle, and at 100 km/h it reduces it to one third. This can affect perceived steering responsiveness independently of rack dynamics.

SAT is suppressed when P-D torque is zero. Opposing SAT is capped to that torque's magnitude, so sufficiently strong feedback can cancel the command; assisting SAT is uncapped until travel limits. A setting called “self-centering strength” would therefore overpromise autonomous behavior in the current model.

### Proposed controls

- Maximum steering command at zero speed (degrees), with actual inner/outer wheel angles previewed because Ackermann modifies individual angles.
- Steering response time (ms), defined as time to first reach 90% of a step in the no-road-feedback reference condition.
- Response shape: critically damped as the initial default; advanced damping ratio if needed.
- Steering sensitivity half-speed (km/h): 50 km/h expresses the current hardcoded behavior. Provide an explicit constant-sensitivity choice.
- Road feedback, with a clear preview of actual torque and authority limiting.

Fix an internal inertia convention and derive proportional gain and one total damping coefficient. Natural frequency and damping ratio are the standard two parameters of a second-order response ([MathWorks reference](https://www.mathworks.com/help/control/ref/ord2.html)). For critical damping:

```text
omega = 3.88972017 / response_time_seconds
proportional_gain = I * omega^2
total_damping = 2 * I * omega
```

The constant defines a continuous critically damped response's 90% time. Verify the actual solver against it. Other damping ratios need their own response-time conversion; do not reuse this constant for all shapes. Friction and SAT must be accounted for separately.

Keep inertia and friction advanced; label the current friction value as a resisting torque scale (Nm), since it is multiplied directly by `tanh(angular_velocity * 5)`. Avoid presenting it as a dimensionless material coefficient.

Changing steering response also changes proportional gain and therefore SAT authority. For independent tuning, road feedback could be specified as a desired steering deflection at a documented reference lateral force and slip, with its gain derived from the controller stiffness and aligning lever. That requires verifying the nonlinear authority policy; merely renaming `sat_gain` cannot guarantee independent response and feedback.

## 5. Differential

### Current behavior

`src/Drivetrain/RotationalNetwork.cpp::DifferentialSettings::capacity` computes:

```text
capacity = min(max_lock_torque,
    preload_torque + active_ratio * abs(axle_torque)
    + slip_sensitive_gain * abs(left_speed - right_speed))
```

Power/coast selection follows transmitted torque and carrier velocity, not throttle position. Capacity is a limit, not necessarily the applied torque. The constraint applies equal and opposite correction torques to the wheels.

### Proposed controls

Keep **Open / Limited slip / Locked** as the first control. Hide or disable limited-slip settings in other modes.

For limited slip, expose:

- Acceleration torque-driven locking (%).
- Engine-braking torque-driven locking (%).
- Baseline coupling torque (Nm), currently called preload.

Explain that higher locking resists differences in wheel speed; it does not prescribe a left/right speed ratio or guarantee a particular understeer/oversteer outcome.

There is a factor of two when converting the current ratio. The primary constraint distributes `T/2` to each wheel; a differential correction `q` adds to one and subtracts from the other. Ignoring preload, speed gain, and the cap:

```text
q_capacity = current_ratio * abs(T)
wheel_torques_at_capacity = T/2 +/- q_capacity
torque_imbalance_fraction = 2 * current_ratio
equivalent_bias_ratio = (1 + 2 * current_ratio) / (1 - 2 * current_ratio)
```

Thus current default power ratio 0.35 corresponds to a 70% torque-driven locking effect, or an ideal 5.67:1 bias ratio. Coast ratio 0.15 corresponds to 30%, or 1.86:1. Example power ratio 0.12 corresponds to 24%, or 1.63:1. Display the percentage as the primary value and the equivalent bias ratio as explanatory context. Torque bias ratio is an established measure of differential torque distribution ([Torsen explanation](https://torsen.com/faqs/what-is-tbr/)); this equivalent calculation does not turn the current solver into a Torsen model.

The finite bias-ratio relation requires `current_ratio < 0.5`. At 0.5 the ideal low-side torque is zero; above it there is no finite positive bias-ratio interpretation. A new basic percentage control should cover 0-100%, with Locked retained as a separate unbounded constraint mode. A 100% torque-driven LSD setting is not equivalent to Locked when the finite cap or zero input torque matters.

Preload, slip gain, and the cap alter the actual effect substantially. With default settings and a 10 rad/s wheel-speed difference, capacities at 100, 500, and 1000 Nm axle input are 80, 220, and 250 Nm respectively. These are capacity calculations, not applied-torque measurements.

Put maximum coupling torque and speed-dependent coupling under advanced settings. Express speed gain as additional Nm per 100 RPM wheel-speed difference if displayed; the current gain 2 Nm/(rad/s) equals approximately 20.94 Nm per 100 RPM. Preview capacity versus input torque at selected speed differences, and show when the cap is active. Preload is a physically meaningful tuning dimension in real limited-slip designs ([Drexler](https://www.drexler-automotive.com/en/products/limited-slip-differentials)), so it should remain accessible.

## Supporting authoring changes

- Group controls by grip, force response, aligning feedback, steering response, and differential policy. Add units, a behavior explanation, and a visible reset value.
- Rename `peak_slip_angle` to a lateral force response scale in the default tanh model: it reaches `tanh(1)`, about 76%, rather than a force peak. Custom curves can define a peak independently. The same scale currently controls pneumatic trail rolloff; show that coupling or introduce an independently specified rolloff angle in a later model change.
- Hide custom-curve details until custom curves are enabled. Show the resulting force curve and which default shape controls remain applicable.
- Present combined grip as a curve or an explained blend choice initially; expose the superellipse exponent under advanced settings. Increasing it permits more simultaneous cornering and acceleration/braking force.
- Add validation of finite values and valid domains. Tire and steering setup validation currently checks resource presence, while most setters accept their supplied values directly; inspector ranges do not enforce a valid programmatic configuration.
- Make application of edits explicit. Numeric tire, steering, and differential settings are copied during setup. A tuning panel should offer Apply and restart or a defined safe reconfiguration operation. Preview must identify the values active in runtime. Duplicate shared resources before per-vehicle edits.
- Extend existing telemetry with target/actual rack angle, effective trail, SAT before/after limiting, reference load and load multiplier, and differential capacity/applied correction. Keep solver telemetry in bounded structures; construct presentation data outside the physics hot path.

## Acceptance and verification for a future implementation

1. Every basic parameter has an explicit unit, defined reference condition, default, and observable effect. Equivalent current settings round-trip through authoring conversions. Derived values are read-only.
2. Load sensitivity: verify reference, half, twice, and extreme loads; coefficient versus total force; clamp boundaries; zero sensitivity; nonfinite inputs.
3. Trail: verify total/retained conversion, both slip signs, zero total trail, zero and full retention, SAT sign, and rolloff using final tire forces. Retain the existing SAT authority regression assertions unless that policy is deliberately redesigned.
4. Tire and rack response: measure defined step-response times at relevant physics rates/substeps, with SAT disabled first. Verify angle bounds, shape/overshoot, zero response-time semantics, invalid inputs, and coupled road-feedback behavior separately. Suggested target: within 5% or one substep of the specified reference time, whichever is larger, over the supported tuning range.
5. Differential: verify open/locked/LSD modes, percentage conversion including the factor of two, forward/reverse power/coast, preload, cap saturation, speed gain, zero torque, unequal inertia, and actual applied torque as well as capacity.
6. Editor application: verify serialization, reset behavior, shared-resource isolation, and that displayed active values match settings consumed after successful application.
7. Driving validation: use steering steps at several speeds, steady cornering, corner exit under power, lift-off, and split-grip acceleration. Verify parameter direction and approve presets from observed traces. An isolated formula check cannot establish a good driving feel.

Implement clear labels, units, missing documentation, previews, and exact algebraic conversions first. Introduce the exponential tire update and derived rack dynamics as separate behavior changes with their own verification. This keeps the authoring improvements reviewable and makes any handling changes attributable.

## Original review verification

At the time of the original review, inspected resource definitions, property bindings, setup, tire and rack equations, differential constraints, existing SAT regression coverage, telemetry, and the current example. Independently checked load-loss and bias-ratio conversions and reproduced the isolated rack update with Python. The original review changed no production source or existing tests. The subsequent implementation is described at the start of this document and in README.md; no new preset calibration was performed.
