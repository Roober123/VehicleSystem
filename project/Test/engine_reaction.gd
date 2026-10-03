extends SceneTree

# Isolated airborne bodies exercise real Godot torque application without
# suspension/road torques masking engine reaction. Also usable with normal DLL.
var failures := 0

func _initialize() -> void:
	call_deferred("_run")

func _expect(condition: bool, label: String) -> void:
	if not condition:
		failures += 1
		push_error("[engine-reaction] " + label)

func _make_vehicle(axis: Vector3, steps: int, active: bool = true, speed_kph: float = 0.0) -> Vehicle:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 1.0))
	curve.add_point(Vector2(1.0, 1.0))
	var engine := VehicleEngineData.new()
	engine.torque_curve = curve
	engine.idle_rpm = 1000.0
	engine.redline_rpm = 4000.0
	engine.max_torque = 100.0
	engine.engine_drag = 0.1
	var gearbox := GearboxData.new()
	gearbox.auto_mode = false
	var config := VehicleConfig.new()
	config.engine_data = engine
	config.gearbox_data = gearbox
	config.suspension_data = SuspensionData.new()
	if active:
		config.engine_reaction_data = EngineReactionData.new()
		config.engine_reaction_data.reaction_strength = 0.5
		config.engine_reaction_data.vibration_strength = 0.25
	var vehicle := Vehicle.new()
	vehicle.config = config
	vehicle.engine_axis = axis
	vehicle.substeps = steps
	vehicle.gravity_scale = 0.0
	vehicle.linear_velocity = Vector3(0.0, 0.0, speed_kph / 3.6)
	vehicle.can_sleep = false
	vehicle.mass = 100.0
	vehicle.inertia = Vector3(50.0, 50.0, 50.0)
	vehicle.collision_layer = 0
	vehicle.collision_mask = 0
	var shape := CollisionShape3D.new()
	shape.shape = BoxShape3D.new()
	vehicle.add_child(shape)
	var axle := Axle.new()
	axle.drive_share = 1.0
	axle.differential_data = DifferentialData.new()
	axle.tire_data = TireData.new()
	var left := Wheel.new()
	left.position.x = -0.8
	var right := Wheel.new()
	right.position.x = 0.8
	axle.add_child(left)
	axle.add_child(right)
	vehicle.add_child(axle)
	var telemetry := VehicleTelemetry.new()
	telemetry.name = "Telemetry"
	vehicle.add_child(telemetry)
	vehicle.rotation.y = PI / 2.0
	root.add_child(vehicle)
	return vehicle

func _run() -> void:
	var longitudinal := _make_vehicle(Vector3(0.0, 0.0, 10.0), 1)
	var transverse := _make_vehicle(Vector3.RIGHT, 8)
	var reversed := _make_vehicle(Vector3(0.0, 0.0, -1.0), 1)
	var zero := _make_vehicle(Vector3.ZERO, 1)
	var missing := _make_vehicle(Vector3.FORWARD, 1, false)
	var fast_forward := _make_vehicle(Vector3.UP, 1, true, 72.0)
	var fast_reverse := _make_vehicle(Vector3.UP, 8, true, -72.0)
	var vehicles: Array[Vehicle] = [longitudinal, transverse, reversed, zero, missing, fast_forward, fast_reverse]
	var idle_amplitude := 0.0
	var idle_motion := 0.0
	for tick in range(24):
		await physics_frame
		var sample := (longitudinal.get_node("Telemetry") as VehicleTelemetry).get_engine_reaction_telemetry()
		idle_amplitude = maxf(idle_amplitude, float(sample.vibration_amplitude))
		idle_motion = maxf(idle_motion, longitudinal.angular_velocity.length())
	_expect(idle_amplitude > 0.0, "Stable idle has generated-torque vibration")
	_expect(idle_motion > 0.00001, "Idle vibration moves the real chassis")
	for vehicle in vehicles:
		vehicle.set_throttle_input(0.5)
	for tick in range(24):
		await physics_frame
		for vehicle in vehicles:
			var data := (vehicle.get_node("Telemetry") as VehicleTelemetry).get_engine_reaction_telemetry()
			var torque: Vector3 = data.chassis_torque
			_expect(torque.is_finite(), "World torque stays finite")
			_expect(torque.length() <= 250.001, "World torque respects cap")
			if vehicle.engine_axis != Vector3.ZERO and vehicle.config.engine_reaction_data != null:
				var world_axis := vehicle.global_basis.orthonormalized() * vehicle.engine_axis
				_expect(torque.cross(world_axis).length() < 0.02, "Local axis follows rotated chassis")
			else:
				_expect(torque == Vector3.ZERO, "Zero axis or missing resource applies no torque")
	for vehicle in [longitudinal, transverse, reversed]:
		var world_axis: Vector3 = vehicle.global_basis.orthonormalized() * vehicle.engine_axis
		_expect(vehicle.angular_velocity.dot(world_axis) < -0.01, "Throttle reaction rotates opposite engine axis")
		_expect(vehicle.linear_velocity.length() < 0.00001, "Engine torque introduces no pushing force")
	_expect(zero.angular_velocity == Vector3.ZERO and missing.angular_velocity == Vector3.ZERO,
		"Opt-out bodies keep baseline angular motion")
	for vehicle in [fast_forward, fast_reverse]:
		var travel := (vehicle.get_node("Telemetry") as VehicleTelemetry).get_engine_reaction_telemetry()
		_expect(travel.speed_factor == 0.0 and travel.chassis_torque == Vector3.ZERO and travel.vibration_amplitude == 0.0,
			"Forward and reverse travel fully suppress engine chassis torque")
		_expect(vehicle.angular_velocity.length() < 0.00001,
			"An engine yaw axis cannot change direction at travel speed")
		_expect(is_equal_approx(absf(vehicle.linear_velocity.z), 20.0) and
			absf(vehicle.linear_velocity.x) < 0.00001 and absf(vehicle.linear_velocity.y) < 0.00001,
			"Travelling bodies preserve velocity direction under engine load")
	var preserved_velocity := longitudinal.angular_velocity
	longitudinal.config.engine_reaction_data.enabled = false
	_expect(longitudinal.restart(), "Restart accepts disabled reaction tuning")
	_expect(longitudinal.angular_velocity.is_equal_approx(preserved_velocity), "Restart preserves chassis velocity")
	var reset := (longitudinal.get_node("Telemetry") as VehicleTelemetry).get_engine_reaction_telemetry()
	_expect(reset.chassis_torque == Vector3.ZERO and reset.vibration_amplitude == 0.0,
		"Restart clears reaction telemetry immediately")
	for tick in range(8):
		await physics_frame
	var disabled := (longitudinal.get_node("Telemetry") as VehicleTelemetry).get_engine_reaction_telemetry()
	_expect(disabled.chassis_torque == Vector3.ZERO and disabled.vibration_torque == 0.0,
		"Disabled resource keeps applied torque zero after restart")
	for vehicle in vehicles:
		vehicle.queue_free()
	await process_frame
	print("[engine-reaction] ALL_DONE failures=", failures)
	quit(0 if failures == 0 else 1)
