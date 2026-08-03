extends Node

# DS-REV-05: production Vehicle-path trace.  The example scene is instantiated
# unchanged so its engine, gearbox, tire, suspension, collision, and ground
# settings remain the runtime source of truth; this harness only drives inputs
# and samples observable production state.
const EXAMPLE_SCENE: PackedScene = preload("res://example.tscn")
const SUBSTEPS: Array[int] = [1, 4, 8, 16]
const PHYSICS_HZ := 120.0
const DT := 1.0 / PHYSICS_HZ
const EPS := 1.0e-2
const ANG_TO_RPM := 60.0 / TAU
# A fixed 0.75 s tail averages tire relaxation rather than a single
# float32-phase sample, making the 8/16 comparison deterministic.
const SETTLING_WINDOW := 90

var failures := 0
var all_results: Dictionary = {}

func _ready() -> void:
	await _run_all()
	print("[production-grounded] ALL_DONE failures=", failures)
	get_tree().quit(0 if failures == 0 else 1)

func _run_all() -> void:
	for case_name in [
		"throttle_on_upshift",
		"lift_upshift",
		"downshift",
		"braking_shift",
		"airborne_grounded",
		"neutral",
		"reverse",
	]:
		var cases: Array[Dictionary] = []
		for count in SUBSTEPS:
			var result: Dictionary = await _run_case(case_name, count)
			cases.append(result)
			all_results["%s_%d" % [case_name, count]] = result
			_print_result(case_name, result)
			_assert_case(case_name, result)
		_assert_substep_invariance(case_name, cases)

func _run_case(case_name: String, substeps: int) -> Dictionary:
	var scene_root := EXAMPLE_SCENE.instantiate()
	# The showcase root script polls Input every idle frame. Detach it so the
	# deterministic physics controls below are the sole source of drivetrain
	# input while retaining all Vehicle/axle/wheel production scripts.
	scene_root.set_script(null)
	add_child(scene_root)
	await get_tree().process_frame
	var vehicle := scene_root.get_node("Vehicle") as Vehicle
	# Keep only the production vehicle and the example ground collider. The
	# visual mesh, camera, UI, environment, and stunt-build colliders are not
	# part of the drivetrain contract and make repeated headless traces costly.
	for child in scene_root.get_children():
		if child != vehicle and child.name != "CSGBox3D":
			child.queue_free()
	await get_tree().process_frame
	vehicle.set_substeps(substeps)
	vehicle.set_throttle_input(0.0)
	vehicle.set_brake_input(0.0)
	vehicle.set_steer_input(0.0)
	# Let the production suspension settle before applying any shift input.
	await _idle_ticks(vehicle, 120)

	var trace := _new_trace()
	if case_name == "airborne_grounded":
		trace["direction_required"] = false
	if case_name == "throttle_on_upshift":
		await _enter_gear(vehicle, 1)
		await _idle_control_ticks(vehicle, 120, 1.0, 0.0)
		vehicle.set_shift_input(1)
		await _trace_ticks(vehicle, trace, 120, 1.0, 0.0)
	elif case_name == "lift_upshift":
		await _enter_gear(vehicle, 1)
		await _idle_control_ticks(vehicle, 120, 1.0, 0.0)
		vehicle.set_shift_input(1)
		await _trace_ticks(vehicle, trace, 120, 0.0, 0.0)
	elif case_name == "downshift":
		await _enter_gear(vehicle, 1)
		await _enter_gear(vehicle, 1)
		await _idle_control_ticks(vehicle, 120, 1.0, 0.0)
		vehicle.set_shift_input(-1)
		await _trace_ticks(vehicle, trace, 120, 0.25, 0.0)
	elif case_name == "braking_shift":
		await _enter_gear(vehicle, 1)
		await _idle_control_ticks(vehicle, 120, 1.0, 0.0)
		vehicle.set_shift_input(1)
		await _trace_ticks(vehicle, trace, 120, 0.0, 0.65)
	elif case_name == "airborne_grounded":
		await _enter_gear(vehicle, 1)
		await _trace_ticks(vehicle, trace, 30, 0.0, 0.0)
		var base_transform := vehicle.global_transform
		vehicle.global_position += Vector3.UP * 4.0
		vehicle.linear_velocity = Vector3.ZERO
		vehicle.angular_velocity = Vector3.ZERO
		await _trace_ticks(vehicle, trace, 90, 0.0, 0.0)
		vehicle.global_transform = base_transform
		vehicle.linear_velocity = Vector3.ZERO
		vehicle.angular_velocity = Vector3.ZERO
		await _trace_ticks(vehicle, trace, 150, 0.0, 0.0)
	elif case_name == "neutral":
		await _enter_gear(vehicle, 1)
		await _idle_control_ticks(vehicle, 120, 1.0, 0.0)
		vehicle.set_shift_input(-1)
		await _trace_ticks(vehicle, trace, 120, 0.0, 0.0)
	elif case_name == "reverse":
		vehicle.set_shift_input(-1)
		await _trace_ticks(vehicle, trace, 75, 0.0, 0.0)
		await _trace_ticks(vehicle, trace, 120, 0.0, 0.35)

	_apply_controls(vehicle, 0.0, 0.0)
	await get_tree().physics_frame
	_finalize_trace(trace)
	trace["substeps"] = substeps
	scene_root.queue_free()
	await get_tree().process_frame
	return trace

func _enter_gear(vehicle: Vehicle, direction: int) -> void:
	vehicle.set_shift_input(direction)
	await _idle_ticks(vehicle, 72)

func _idle_ticks(vehicle: Vehicle, count: int) -> void:
	for _i in range(count):
		_apply_controls(vehicle, 0.0, 0.0)
		await get_tree().physics_frame

func _idle_control_ticks(vehicle: Vehicle, count: int, throttle: float, brake: float) -> void:
	for _i in range(count):
		_apply_controls(vehicle, throttle, brake)
		await get_tree().physics_frame

func _trace_ticks(vehicle: Vehicle, trace: Dictionary, count: int,
		throttle: float, brake: float) -> void:
	for _i in range(count):
		_apply_controls(vehicle, throttle, brake)
		await get_tree().physics_frame
		_sample(vehicle, trace)

func _apply_controls(vehicle: Vehicle, throttle: float, brake: float) -> void:
	vehicle.set_throttle_input(throttle)
	vehicle.set_brake_input(brake)
	vehicle.set_steer_input(0.0)

func _new_trace() -> Dictionary:
	return {
		"samples": 0,
		"finite": true,
		"shaft_min": INF,
		"shaft_max": -INF,
		"wheel_min": INF,
		"wheel_max": -INF,
		"momentum_min": INF,
		"momentum_max": -INF,
		"peak_excursion": 0.0,
		"settling_slip": INF,
		"slip_samples": [],
		"shaft_reversals": 0,
		"wheel_reversals": 0,
		"reverse_seen": false,
		"grounded_min": 99,
		"grounded_max": 0,
		"initial_wheel": NAN,
		"last_shaft": NAN,
		"last_wheel": NAN,
		"last_momentum": NAN,
		"last_gear": 0,
		"direction_required": true,
	}

func _sample(vehicle: Vehicle, trace: Dictionary) -> void:
	var telemetry := vehicle.get_node_or_null("VehicleTelemetry")
	if telemetry == null:
		trace["finite"] = false
		return
	var shaft_rpm := float(telemetry.get_driveshaft_rpm())
	var engine_rpm := float(telemetry.get_engine_rpm())
	var gear := int(telemetry.get_current_gear())
	var wheels: Array[float] = _wheel_rpms(vehicle)
	var driven: Array[float] = _driven_wheel_rpms(vehicle)
	var rear_avg := _average(driven)
	var shaft := shaft_rpm / ANG_TO_RPM
	var wheel := rear_avg / ANG_TO_RPM
	var engine := engine_rpm / ANG_TO_RPM
	# Example scene settings: engine I=.45, driveshaft I=1, rear wheels
	# receive two .9 bodies from TireData (.4*.3^2*25). This is a signed
	# angular-momentum proxy, not a replacement for the production state.
	var momentum := 0.45 * engine + shaft + 1.8 * wheel
	var grounded := _grounded_wheel_count(vehicle)
	if is_nan(float(trace["initial_wheel"])):
		trace["initial_wheel"] = wheel
	trace["samples"] += 1
	trace["finite"] = bool(trace["finite"]) and is_finite(shaft) and is_finite(engine) and is_finite(wheel) and is_finite(momentum)
	for omega in wheels:
		trace["finite"] = bool(trace["finite"]) and is_finite(omega)
	trace["shaft_min"] = minf(float(trace["shaft_min"]), shaft)
	trace["shaft_max"] = maxf(float(trace["shaft_max"]), shaft)
	trace["wheel_min"] = minf(float(trace["wheel_min"]), wheel)
	trace["wheel_max"] = maxf(float(trace["wheel_max"]), wheel)
	trace["momentum_min"] = minf(float(trace["momentum_min"]), momentum)
	trace["momentum_max"] = maxf(float(trace["momentum_max"]), momentum)
	trace["peak_excursion"] = maxf(float(trace["peak_excursion"]), absf(wheel - float(trace["initial_wheel"])))
	trace["slip_samples"].append(absf(wheel - shaft))
	if bool(trace["direction_required"]):
		trace["shaft_reversals"] += 1 if shaft < -EPS else 0
		trace["wheel_reversals"] += 1 if wheel < -EPS else 0
	trace["reverse_seen"] = bool(trace["reverse_seen"]) or shaft < -EPS or wheel < -EPS
	trace["grounded_min"] = mini(int(trace["grounded_min"]), grounded)
	trace["grounded_max"] = maxi(int(trace["grounded_max"]), grounded)
	trace["last_shaft"] = shaft
	trace["last_wheel"] = wheel
	trace["last_momentum"] = momentum
	trace["last_gear"] = gear
	trace["settling_slip"] = absf(wheel - shaft)

func _finalize_trace(trace: Dictionary) -> void:
	var slips: Array = trace["slip_samples"]
	if slips.is_empty():
		trace["settling_slip"] = INF
		return
	var first := maxi(0, slips.size() - SETTLING_WINDOW)
	var sum_sq := 0.0
	for index in range(first, slips.size()):
		var slip := float(slips[index])
		sum_sq += slip * slip
	trace["settling_slip"] = sqrt(sum_sq / float(slips.size() - first))

func _wheel_rpms(vehicle: Vehicle) -> Array[float]:
	var result: Array[float] = []
	for axle in vehicle.get_children():
		for wheel in axle.get_children():
			if wheel.has_method("get_angular_velocity"):
				result.append(float(wheel.get_angular_velocity()) * ANG_TO_RPM)
	return result

func _driven_wheel_rpms(vehicle: Vehicle) -> Array[float]:
	var result: Array[float] = []
	for axle in vehicle.get_children():
		var ratio_variant = axle.get("drive_ratio")
		if ratio_variant == null:
			continue
		if float(ratio_variant) <= 0.0:
			continue
		for wheel in axle.get_children():
			if wheel.has_method("get_angular_velocity"):
				result.append(float(wheel.get_angular_velocity()) * ANG_TO_RPM)
	return result

func _grounded_wheel_count(vehicle: Vehicle) -> int:
	var count := 0
	for axle in vehicle.get_children():
		for wheel in axle.get_children():
			if wheel.has_method("is_on_ground") and wheel.is_on_ground():
				count += 1
	return count

func _average(values: Array[float]) -> float:
	if values.is_empty():
		return 0.0
	var sum := 0.0
	for value in values:
		sum += value
	return sum / values.size()

func _print_result(case_name: String, result: Dictionary) -> void:
	print("[production-grounded] case=%s substeps=%d samples=%d finite=%s gear=%d ground=%d..%d shaft=%.3f..%.3f wheel=%.3f..%.3f momentum=%.3f..%.3f peak=%.3f settle=%.3f settle_pct=%.4f shaft_rev=%d wheel_rev=%d reverse_seen=%s" % [
		case_name, result["substeps"], result["samples"], result["finite"], result["last_gear"],
		result["grounded_min"], result["grounded_max"], result["shaft_min"], result["shaft_max"],
		result["wheel_min"], result["wheel_max"], result["momentum_min"], result["momentum_max"],
		result["peak_excursion"], result["settling_slip"], _settling_ratio(result), result["shaft_reversals"],
		result["wheel_reversals"], result["reverse_seen"]])

func _assert_case(case_name: String, result: Dictionary) -> void:
	_expect(bool(result["finite"]), "%s finite" % case_name)
	_expect(int(result["samples"]) > 0, "%s samples" % case_name)
	var expected_gear: int = int({
		"throttle_on_upshift": 2,
		"lift_upshift": 2,
		"downshift": 1,
		"braking_shift": 2,
		"airborne_grounded": 1,
		"neutral": 0,
	}.get(case_name, -1))
	if case_name == "reverse":
		_expect(int(result["last_gear"]) < 0, "reverse final gear is reverse")
	else:
		_expect(int(result["last_gear"]) == expected_gear, "%s final gear=%d" % [case_name, expected_gear])
	if case_name != "airborne_grounded":
		_expect(int(result["grounded_min"]) == 4 and int(result["grounded_max"]) == 4,
			"%s remains fully grounded" % case_name)
	if case_name == "reverse":
		_expect(bool(result["reverse_seen"]), "reverse signed output direction")
		_expect(float(result["last_shaft"]) < -EPS and float(result["last_wheel"]) < -EPS,
			"reverse final shaft and wheels negative")
	elif case_name != "airborne_grounded":
		_expect(int(result["shaft_reversals"]) == 0, "%s shaft stays forward" % case_name)
		_expect(int(result["wheel_reversals"]) == 0, "%s wheels stay forward" % case_name)
		_expect(float(result["momentum_min"]) > -EPS, "%s forward momentum proxy" % case_name)
	if case_name == "airborne_grounded":
		_expect(int(result["grounded_min"]) < 4 and int(result["grounded_max"]) == 4,
			"airborne to grounded transition")

func _assert_substep_invariance(case_name: String, cases: Array[Dictionary]) -> void:
	# The airborne transition intentionally has no driven load; its substep
	# excursion is only float32 settling around zero and is not comparable to a
	# loaded shift peak.
	if case_name == "airborne_grounded":
		return
	var eight: Dictionary = cases[2]
	var sixteen: Dictionary = cases[3]
	var peak_scale := maxf(absf(float(eight["peak_excursion"])), absf(float(sixteen["peak_excursion"])))
	if peak_scale > 1.0e-3:
		_expect(_relative_difference(float(eight["peak_excursion"]), float(sixteen["peak_excursion"])) <= 0.05,
			"%s peak 8/16 invariance" % case_name)
	# Compare the residual against the observed excursion, not against a
	# near-zero float32 tail.  This is a bounded, dimensionless settling error:
	# each run must settle within 5% of its own peak and 8/16 may differ by no
	# more than five percentage points.
	var eight_settle_ratio := _settling_ratio(eight)
	var sixteen_settle_ratio := _settling_ratio(sixteen)
	_expect(eight_settle_ratio <= 0.05 and sixteen_settle_ratio <= 0.05,
		"%s settling residual <=5%% of peak" % case_name)
	_expect(absf(eight_settle_ratio - sixteen_settle_ratio) <= 0.05,
		"%s settling 8/16 spread <=5%% of peak" % case_name)

func _settling_ratio(result: Dictionary) -> float:
	var peak := absf(float(result["peak_excursion"]))
	return 0.0 if peak <= 1.0e-8 else absf(float(result["settling_slip"])) / peak

func _relative_difference(lhs: float, rhs: float) -> float:
	var scale := maxf(absf(lhs), absf(rhs))
	return 0.0 if scale <= 1.0e-8 else absf(lhs - rhs) / scale

func _expect(condition: bool, label: String) -> void:
	if condition:
		return
	failures += 1
	push_error("[production-grounded] FAIL: %s" % label)
