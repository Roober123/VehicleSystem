extends Node

# Runtime diagnostic for the example rear-wheel-drive vehicle.  This is
# intentionally measurement-only: it does not assert handling outcomes or
# modify production code.  Each run instantiates the checked-in example scene
# and samples the public Vehicle/Wheel/VehicleTelemetry bindings.

const PHYSICS_DT := 1.0 / 60.0
const SETTLE_TICKS := 45
const ACCEL_TICKS := 180
const CORNER_TICKS := 90
const SAMPLE_TICKS := 60
const STEER_INPUT := 0.35

var _scenario_id := ""

func _ready() -> void:
	await get_tree().process_frame
	await _run_scenario("current_lsd", false)
	await _run_scenario("open_rear", true)
	print("[grip-diagnostic] ALL_DONE")
	get_tree().quit(0)

func _run_scenario(label: String, force_open: bool) -> void:
	_scenario_id = label
	var scene := load("res://example.tscn") as PackedScene
	if scene == null:
		push_error("[grip-diagnostic] unable to load res://example.tscn")
		return
	var root := scene.instantiate() as Node3D
	if root == null:
		push_error("[grip-diagnostic] example scene did not instantiate")
		return
	# The interactive example script writes inputs every render frame.  Disable
	# only that scene-root process; the Vehicle child remains fully native.
	root.set_process(false)
	root.set_physics_process(false)
	var vehicle := root.get_node("Vehicle") as Vehicle
	if vehicle == null:
		push_error("[grip-diagnostic] %s missing Vehicle" % label)
		root.queue_free()
		return
	var rear := vehicle.get_node("Axle") as Axle
	if rear == null:
		push_error("[grip-diagnostic] %s missing rear axle" % label)
		root.queue_free()
		return
	if force_open:
		var open_diff := rear.get_differential_data().duplicate() as DifferentialData
		open_diff.mode = 0 # DifferentialData.OPEN
		rear.differential_data = open_diff
	else:
		# The current example configuration is mode 1 (limited slip).
		var current_diff := rear.get_differential_data()
		current_diff.mode = 1
	add_child(root)
	# Scene-script defaults are applied on enter_tree, so disable the
	# interactive root process after it has entered the tree as well.
	await get_tree().process_frame
	root.set_process(false)
	root.set_physics_process(false)
	await get_tree().physics_frame
	var telemetry := vehicle.get_node("VehicleTelemetry") as VehicleTelemetry
	if telemetry == null:
		push_error("[grip-diagnostic] %s missing VehicleTelemetry" % label)
		root.queue_free()
		await get_tree().process_frame
		return
	vehicle.select_neutral()
	vehicle.set_throttle_input(0.0)
	vehicle.set_steer_input(0.0)
	vehicle.set_brake_input(0.0)
	await _physics_ticks(SETTLE_TICKS)
	vehicle.select_drive()
	await _physics_ticks(SETTLE_TICKS)

	# Controlled straight-line preload.  The last sample is intentionally
	# retained in the report so cornering metrics are interpreted at a known
	# speed rather than from rest.
	vehicle.set_steer_input(0.0)
	vehicle.set_throttle_input(0.75)
	await _physics_ticks(ACCEL_TICKS)
	var accel_sample := _sample(vehicle, telemetry)
	_print_sample("accel", accel_sample)

	# Steady left corner at coast, moderate power, then full power.  We discard
	# the first part of each segment and aggregate the final SAMPLE_TICKS.
	var corner_modes := [
		{"name": "corner_coast", "throttle": 0.0},
		{"name": "corner_moderate", "throttle": 0.5},
		{"name": "corner_full", "throttle": 1.0},
	]
	for mode in corner_modes:
		vehicle.set_steer_input(STEER_INPUT)
		vehicle.set_throttle_input(float(mode["throttle"]))
		vehicle.set_brake_input(0.0)
		await _physics_ticks(CORNER_TICKS - SAMPLE_TICKS)
		var aggregate := await _aggregate_samples(vehicle, telemetry, SAMPLE_TICKS)
		_print_aggregate(str(mode["name"]), aggregate)

	vehicle.set_throttle_input(0.0)
	vehicle.set_steer_input(0.0)
	vehicle.set_brake_input(1.0)
	await _physics_ticks(15)
	root.queue_free()
	await get_tree().process_frame

func _aggregate_samples(vehicle: Vehicle, telemetry: VehicleTelemetry, count: int) -> Dictionary:
	var sum_speed := 0.0
	var sum_yaw := 0.0
	var sum_lateral_ratio := 0.0
	var max_loss_proxy := 0.0
	var max_rear_delta := 0.0
	var max_front_delta := 0.0
	var sum_rear_delta := 0.0
	var sum_front_delta := 0.0
	var wheel_sum: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
	var wheel_peak: Array[float] = [0.0, 0.0, 0.0, 0.0]
	var first_sample := {}
	var last_sample := {}
	for i in range(count):
		await get_tree().physics_frame
		var sample := _sample(vehicle, telemetry)
		if i == 0:
			first_sample = sample
		last_sample = sample
		sum_speed += float(sample["speed_kph"])
		sum_yaw += absf(float(sample["yaw_rate"]))
		sum_lateral_ratio += float(sample["lateral_ratio"])
		max_loss_proxy = maxf(max_loss_proxy, float(sample["loss_proxy"]))
		var rear_delta := float(sample["rear_rpm_delta"])
		var front_delta := float(sample["front_rpm_delta"])
		sum_rear_delta += rear_delta
		sum_front_delta += front_delta
		max_rear_delta = maxf(max_rear_delta, rear_delta)
		max_front_delta = maxf(max_front_delta, front_delta)
		var forces: Array = sample["forces"]
		for wheel_index in range(4):
			var force: Vector3 = forces[wheel_index]
			wheel_sum[wheel_index] += force
			wheel_peak[wheel_index] = maxf(wheel_peak[wheel_index], force.length())
	var wheel_average: Array[Vector3] = []
	for wheel_index in range(4):
		wheel_average.append(wheel_sum[wheel_index] / float(count))
	return {
		"speed_kph": sum_speed / float(count),
		"yaw_rate_abs": sum_yaw / float(count),
		"lateral_ratio": sum_lateral_ratio / float(count),
		"loss_proxy_max": max_loss_proxy,
		"rear_rpm_delta_avg": sum_rear_delta / float(count),
		"rear_rpm_delta_max": max_rear_delta,
		"front_rpm_delta_avg": sum_front_delta / float(count),
		"front_rpm_delta_max": max_front_delta,
		"wheel_force_avg": wheel_average,
		"wheel_force_peak": wheel_peak,
		"first": first_sample,
		"last": last_sample,
	}

func _sample(vehicle: Vehicle, telemetry: VehicleTelemetry) -> Dictionary:
	var rear_left := vehicle.get_node("Axle/Wheel") as Wheel
	var rear_right := vehicle.get_node("Axle/Wheel2") as Wheel
	var front_left := vehicle.get_node("Axle2/Wheel") as Wheel
	var front_right := vehicle.get_node("Axle2/Wheel2") as Wheel
	var wheels: Array[Wheel] = [front_left, front_right, rear_left, rear_right]
	var rpms: Array[float] = []
	var forces: Array[Vector3] = []
	var grounded := 0
	for wheel in wheels:
		var rpm := telemetry.get_wheel_rpm(wheel)
		rpms.append(rpm)
		forces.append(telemetry.get_tire_force(wheel))
		if wheel.is_on_ground():
			grounded += 1
	var local_velocity := vehicle.global_transform.basis.inverse() * vehicle.linear_velocity
	var forward_speed := -local_velocity.z
	var speed_kph := vehicle.linear_velocity.length() * 3.6
	var lateral_ratio := absf(local_velocity.x) / (absf(forward_speed) + 1.0)
	var yaw_rate := vehicle.angular_velocity.y
	# Loss proxy is deliberately decomposed in the output: lateral velocity
	# ratio plus a bounded yaw-rate contribution, not a pass/fail threshold.
	var loss_proxy := lateral_ratio + minf(absf(yaw_rate) * 0.2, 2.0)
	var telemetry_rpms := telemetry.get_wheel_rpms()
	return {
		"speed_kph": speed_kph,
		"engine_rpm": telemetry.get_engine_rpm(),
		"engine_torque": telemetry.get_engine_torque(),
		"throttle": telemetry.get_engine_throttle(),
		"gear": telemetry.get_current_gear(),
		"clutch": telemetry.get_clutch_engagement(),
		"forward_speed": forward_speed,
		"yaw": vehicle.rotation.y,
		"yaw_rate": yaw_rate,
		"lateral_ratio": lateral_ratio,
		"loss_proxy": loss_proxy,
		"grounded": grounded,
		"rpms": rpms,
		"telemetry_rpms": telemetry_rpms,
		"rear_rpm_delta": absf(rpms[2] - rpms[3]),
		"front_rpm_delta": absf(rpms[0] - rpms[1]),
		"forces": forces,
	}

func _print_sample(phase: String, sample: Dictionary) -> void:
	print("[grip-diagnostic] scenario=%s phase=%s speed_kph=%.3f engine_rpm=%.2f engine_torque=%.2f throttle=%.2f gear=%d clutch=%.2f yaw=%.5f yaw_rate=%.5f lateral_ratio=%.5f loss_proxy=%.5f grounded=%d rear_rpm_delta=%.3f front_rpm_delta=%.3f rpms=%s telemetry_rpms=%s" % [
		_scenario_id, phase, sample["speed_kph"], sample["engine_rpm"], sample["engine_torque"], sample["throttle"], sample["gear"], sample["clutch"], sample["yaw"], sample["yaw_rate"], sample["lateral_ratio"], sample["loss_proxy"], sample["grounded"], sample["rear_rpm_delta"], sample["front_rpm_delta"], sample["rpms"], sample["telemetry_rpms"]])
	_print_forces(sample["forces"])

func _print_aggregate(phase: String, aggregate: Dictionary) -> void:
	var first: Dictionary = aggregate["first"]
	var last: Dictionary = aggregate["last"]
	print("[grip-diagnostic] scenario=%s phase=%s avg_speed_kph=%.3f avg_abs_yaw_rate=%.5f avg_lateral_ratio=%.5f max_loss_proxy=%.5f rear_rpm_delta_avg=%.3f rear_rpm_delta_max=%.3f front_rpm_delta_avg=%.3f front_rpm_delta_max=%.3f first_yaw=%.5f last_yaw=%.5f first_rpms=%s last_rpms=%s" % [
		_scenario_id, phase, aggregate["speed_kph"], aggregate["yaw_rate_abs"], aggregate["lateral_ratio"], aggregate["loss_proxy_max"], aggregate["rear_rpm_delta_avg"], aggregate["rear_rpm_delta_max"], aggregate["front_rpm_delta_avg"], aggregate["front_rpm_delta_max"], first["yaw"], last["yaw"], first["rpms"], last["rpms"]])
	var averages: Array = aggregate["wheel_force_avg"]
	var peaks: Array = aggregate["wheel_force_peak"]
	for wheel_index in range(4):
		var average: Vector3 = averages[wheel_index]
		print("[grip-diagnostic] scenario=%s phase=%s wheel=%d avg_force=(%.3f,%.3f,%.3f) avg_force_mag=%.3f peak_force_mag=%.3f" % [
			_scenario_id, phase, wheel_index, average.x, average.y, average.z, average.length(), peaks[wheel_index]])

func _print_forces(forces: Array) -> void:
	for wheel_index in range(forces.size()):
		var force: Vector3 = forces[wheel_index]
		print("[grip-diagnostic] scenario=%s sample_wheel=%d tire_force=(%.3f,%.3f,%.3f) tire_force_mag=%.3f" % [
			_scenario_id, wheel_index, force.x, force.y, force.z, force.length()])

func _physics_ticks(count: int) -> void:
	for _i in range(count):
		await get_tree().physics_frame
