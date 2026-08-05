extends Node

# Measurement-only diagnostic for direction changes through project/test.gd.

const MAX_TICKS := 180
const SPEED_THRESHOLD_KPH := 0.1
const BRAKE_TICKS := 90


func _ready() -> void:
	await get_tree().process_frame
	await _run_scenario("lerped_residual", 1.0)
	await _run_scenario("snapped_zero", 0.0)
	print("[reverse-diagnostic] ALL_DONE")
	get_tree().quit(0)


func _run_scenario(label: String, initial_forward_throttle: float) -> void:
	var packed := load("res://example.tscn") as PackedScene
	var root := packed.instantiate() as Node3D
	root.set_process(false)
	root.set_physics_process(false)
	var vehicle := root.get_node("Vehicle") as Vehicle
	add_child(root)
	await get_tree().process_frame
	root.set_process(false)
	root.set_physics_process(false)
	await get_tree().physics_frame
	var telemetry := vehicle.get_node("VehicleTelemetry") as VehicleTelemetry

	Input.action_release("ui_up")
	Input.action_release("ui_down")
	vehicle.set_throttle_input(0.0)
	vehicle.set_brake_input(1.0)
	vehicle.select_neutral()
	await _physics_ticks(120)
	vehicle.set_brake_input(0.0)
	await _physics_ticks(2)

	# Re-enable the real example input layer. Its throttle state models the
	# value left behind when forward input is released during a direction change.
	root.forward_throttle = initial_forward_throttle
	root.set_process(true)
	Input.action_press("ui_down", 1.0)
	vehicle.select_reverse()

	var gear_tick := -1
	var residual_below_brake_tick := -1
	var clutch_tick := -1
	var motion_tick := -1
	for tick in range(MAX_TICKS):
		await get_tree().physics_frame
		if gear_tick < 0 and telemetry.get_current_gear() == -1:
			gear_tick = tick
		if residual_below_brake_tick < 0 and float(root.forward_throttle) <= 0.05:
			residual_below_brake_tick = tick
		if clutch_tick < 0 and telemetry.get_current_gear() == -1 and telemetry.get_clutch_engagement() > 0.05:
			clutch_tick = tick
		var local_velocity := vehicle.global_transform.basis.inverse() * vehicle.linear_velocity
		if motion_tick < 0 and -local_velocity.z * 3.6 > SPEED_THRESHOLD_KPH:
			motion_tick = tick
		if motion_tick >= 0 and tick >= motion_tick + 10:
			break

	var speed_before_braking := absf(
		(vehicle.global_transform.basis.inverse() * vehicle.linear_velocity).z * 3.6)
	Input.action_release("ui_down")
	Input.action_press("ui_up", 1.0)
	await _physics_ticks(BRAKE_TICKS)
	var speed_after_braking := absf(
		(vehicle.global_transform.basis.inverse() * vehicle.linear_velocity).z * 3.6)
	var braking_reduced_speed := speed_after_braking < speed_before_braking

	print("[reverse-diagnostic] scenario=%s gear_tick=%d residual_below_0_05_tick=%d clutch_tick=%d motion_tick=%d residual_forward=%.4f gear=%d clutch=%.3f reported_throttle=%.3f shaft_rpm=%.2f reverse_speed_kph=%.3f brake_before_kph=%.3f brake_after_kph=%.3f brake_reduced_speed=%s" % [
		label, gear_tick, residual_below_brake_tick, clutch_tick, motion_tick,
		float(root.forward_throttle), telemetry.get_current_gear(),
		telemetry.get_clutch_engagement(), telemetry.get_engine_throttle(),
		telemetry.get_driveshaft_rpm(),
		-(vehicle.global_transform.basis.inverse() * vehicle.linear_velocity).z * 3.6,
		speed_before_braking, speed_after_braking, braking_reduced_speed])
	if motion_tick < 0 or not braking_reduced_speed:
		push_error("[reverse-diagnostic] scenario=%s reverse braking regression" % label)

	Input.action_release("ui_up")
	Input.action_release("ui_down")
	root.set_process(false)
	vehicle.set_brake_input(0.0)
	root.queue_free()
	await get_tree().process_frame


func _physics_ticks(count: int) -> void:
	for _tick in range(count):
		await get_tree().physics_frame
