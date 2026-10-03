extends SceneTree

# Measurement-only audit. Uses the current example car on an isolated flat floor.
# Run: godot --headless --path project --script Test/handling_audit.gd
# Each variant starts fresh and receives identical timed driver commands.
var results: Array = []
var trace: FileAccess
var diagnostics: FileAccess

class COMProbe extends RigidBody3D:
	var reported := false

	func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
		if reported:
			return
		reported = true
		var expected := state.transform * state.center_of_mass_local
		var correct := state.transform.origin + state.center_of_mass
		var production := state.transform * state.center_of_mass
		print("[handling-audit-com] local=", state.center_of_mass_local,
			" world_offset=", state.center_of_mass,
			" correct_error_m=", correct.distance_to(expected),
			" production_error_m=", production.distance_to(expected))

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	Engine.physics_ticks_per_second = 120
	var probe := COMProbe.new()
	probe.gravity_scale = 0.0
	probe.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	probe.center_of_mass = Vector3(0.3, -0.2, -0.8)
	probe.position = Vector3(4, 5, 6)
	probe.rotation.y = PI / 2.0
	probe.inertia = Vector3.ONE
	root.add_child(probe)
	await _ticks(3)
	probe.queue_free()
	trace = FileAccess.open("res://../handling_audit_trace.csv", FileAccess.WRITE)
	if trace == null:
		push_error("Cannot open handling audit trace")
		quit(1)
		return
	trace.store_line("variant,phase,tick,speed_mps,beta_deg,yaw_rate,rack_deg,front_load,rear_load,max_rear_slip,rear_lateral_force,tcs_throttle_proxy,grounded")
	diagnostics = FileAccess.open("res://../Test/recovery_diagnostics.jsonl", FileAccess.WRITE)
	if diagnostics == null:
		push_error("Cannot open recovery diagnostics")
		trace.close()
		quit(1)
		return
	var variants := ["rwd_steering_only", "rwd_steering_slew", "awd_steering_only", "awd_steering_slew"]
	for variant in variants:
		await _scenario(variant)
	trace.close()
	diagnostics.close()
	var output := FileAccess.open("res://../handling_audit_results.json", FileAccess.WRITE)
	output.store_string(JSON.stringify(results, "\t"))
	output.close()
	print("[handling-audit] ALL_DONE")
	quit(0)

func _scenario(variant: String) -> void:
	var world := Node3D.new()
	var floor_body := StaticBody3D.new()
	var floor_shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(2000, 1, 2000)
	floor_shape.shape = box
	floor_body.position.y = -1.0
	floor_body.add_child(floor_shape)
	world.add_child(floor_body)
	var example := (load("res://example.tscn") as PackedScene).instantiate()
	var vehicle := example.get_node("Vehicle1") as Vehicle
	example.remove_child(vehicle)
	example.free()
	vehicle.position = Vector3(0, 0.6853267, 0)
	vehicle.config = vehicle.config.duplicate(true)
	if variant in ["esc_off", "assists_off"]:
		vehicle.config.esc_data.enabled = false
	# Use TCS for every straight-line preload so the assist comparisons enter
	# the corner with the same speed and rotational drivetrain state.
	vehicle.tcs_enabled = true
	if variant == "substeps_8":
		vehicle.substeps = 8
	var front := vehicle.get_node("Axle2") as Axle
	if variant.begins_with("awd"):
		front.drive_share = 0.5
		(vehicle.get_node("Axle") as Axle).drive_share = 0.5
	vehicle.config.esc_data.torque_smoothing_enabled = variant.ends_with("slew")
	if variant.ends_with("slew"):
		vehicle.config.esc_data.torque_engagement_rate = 4000.0
		vehicle.config.esc_data.torque_release_rate = 12000.0
	diagnostics.store_line(JSON.stringify({"variant": variant, "config": _settings(vehicle.config), "esc_cap": vehicle.config.esc_data.maximum_corrective_torque, "yaw_damping": vehicle.config.esc_data.yaw_damping, "engagement_rate": vehicle.config.esc_data.torque_engagement_rate, "release_rate": vehicle.config.esc_data.torque_release_rate, "rack": _settings(front.steering_rack_data), "tire": _settings(front.tire_data)}))
	if variant in ["feedback_off", "fast_rack"]:
		front.steering_rack_data = front.steering_rack_data.duplicate()
		front.steering_rack_data.road_feedback_strength = 0.0
	if variant == "fast_rack":
		front.steering_rack_data.response_time_ms = 160.0
	world.add_child(vehicle)
	root.add_child(world)
	await _ticks(120)
	vehicle.select_drive()
	await _ticks(60)
	vehicle.set_throttle_input(0.65)
	await _ticks(300)
	if not variant.begins_with("first_gear"):
		vehicle.shift_up()
	await _ticks(300)
	vehicle.tcs_enabled = variant not in ["tcs_off", "assists_off", "first_gear_tcs_off"]
	var telemetry := vehicle.get_node("VehicleTelemetry") as VehicleTelemetry
	var wheels: Array[Wheel] = [vehicle.get_node("Axle2/Wheel"), vehicle.get_node("Axle2/Wheel2"), vehicle.get_node("Axle/Wheel"), vehicle.get_node("Axle/Wheel2")]
	var phases := [
		{"name": "power_corner", "steer": 0.5, "throttle": 1.0, "ticks": 240},
		{"name": "countersteer", "steer": -0.5, "throttle": 0.0, "ticks": 120},
		{"name": "recovery", "steer": 0.0, "throttle": 0.3, "ticks": 240},
	]
	for phase in phases:
		vehicle.set_steer_input(phase.steer)
		vehicle.set_throttle_input(phase.throttle)
		var summary := {"variant": variant, "phase": phase.name, "initial_speed_mps": vehicle.linear_velocity.length(), "max_beta_deg": 0.0, "max_yaw_rate": 0.0, "max_yaw_accel": 0.0, "max_rear_load_step_n": 0.0, "max_rear_lateral_force_step_n": 0.0, "max_rear_slip": 0.0, "min_grounded": 4, "full_tcs_cut_ticks": 0, "ticks": phase.ticks}
		var previous_yaw := vehicle.angular_velocity.y
		var previous_load := 0.0
		var previous_lateral := 0.0
		for tick in range(phase.ticks):
			await physics_frame
			var local_v := vehicle.global_basis.inverse() * vehicle.linear_velocity
			var beta := rad_to_deg(atan2(local_v.x, local_v.z))
			var handling := telemetry.get_handling_telemetry()
			var wheel_samples: Array = []
			var grounded := 0
			var front_load := 0.0
			var rear_load := 0.0
			var rear_slip := 0.0
			var rear_lateral := 0.0
			for index in range(4):
				var sample := telemetry.get_tire_telemetry(wheels[index])
				sample.erase("wheel")
				wheel_samples.append(sample)
				grounded += int(sample.grounded)
				if index < 2:
					front_load += float(sample.normal_load)
				else:
					rear_load += float(sample.normal_load)
					rear_slip = maxf(rear_slip, absf(float(sample.slip_ratio)))
					rear_lateral += vehicle.global_basis.x.dot(sample.tire_force)
			# Proxy only: production telemetry reports driver input, not TCS output.
			var tcs_proxy := float(phase.throttle)
			if vehicle.tcs_enabled and vehicle.linear_velocity.length() >= 5.0 and tcs_proxy >= 0.05:
				tcs_proxy *= 1.0 - clampf((rear_slip - 0.16) / 0.5, 0.0, 1.0)
				if tcs_proxy == 0.0:
					summary.full_tcs_cut_ticks += 1
			summary.max_beta_deg = maxf(summary.max_beta_deg, absf(beta))
			summary.max_yaw_rate = maxf(summary.max_yaw_rate, absf(vehicle.angular_velocity.y))
			summary.max_yaw_accel = maxf(summary.max_yaw_accel, absf(vehicle.angular_velocity.y - previous_yaw) * 120.0)
			summary.max_rear_slip = maxf(summary.max_rear_slip, rear_slip)
			summary.min_grounded = mini(summary.min_grounded, grounded)
			if tick > 0:
				summary.max_rear_load_step_n = maxf(summary.max_rear_load_step_n, absf(rear_load - previous_load))
				summary.max_rear_lateral_force_step_n = maxf(summary.max_rear_lateral_force_step_n, absf(rear_lateral - previous_lateral))
			trace.store_line("%s,%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d" % [variant, phase.name, tick, vehicle.linear_velocity.length(), beta, vehicle.angular_velocity.y, rad_to_deg(front.get_steer_angle()), front_load, rear_load, rear_slip, rear_lateral, tcs_proxy, grounded])
			diagnostics.store_line(JSON.stringify({"variant": variant, "phase": phase.name, "tick": tick, "handling": handling, "wheels": wheel_samples, "position": [vehicle.position.x, vehicle.position.y, vehicle.position.z], "speed_mps": vehicle.linear_velocity.length()}))
			previous_yaw = vehicle.angular_velocity.y
			previous_load = rear_load
			previous_lateral = rear_lateral
		var final_v := vehicle.global_basis.inverse() * vehicle.linear_velocity
		summary.final_speed_mps = vehicle.linear_velocity.length()
		summary.final_beta_deg = rad_to_deg(atan2(final_v.x, final_v.z))
		results.append(summary)
		print("[handling-audit] ", JSON.stringify(summary))
	world.queue_free()
	await process_frame

func _ticks(count: int) -> void:
	for tick in range(count):
		await physics_frame

# Called only once per variant, outside the native physics path.
func _settings(resource: Resource) -> Dictionary:
	var values := {"class": resource.get_class()}
	for property in resource.get_property_list():
		if not (property.usage & PROPERTY_USAGE_STORAGE) or property.name == "script":
			continue
		var value = resource.get(property.name)
		values[property.name] = _settings(value) if value is Resource else value
	return values
