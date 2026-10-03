extends SceneTree

var failures := 0
const ROUNDTRIP_PATH := "res://Test/tuning_resources_roundtrip.tres"

func _initialize() -> void:
	call_deferred("_run")

func _expect(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error("[tuning-resources] " + message)

func _property(resource: Resource, name: String) -> Dictionary:
	for item in resource.get_property_list():
		if item.name == name:
			return item
	return {}

func _roundtrip(resource: Resource) -> Resource:
	_expect(not FileAccess.file_exists(ROUNDTRIP_PATH), "temporary resource path must be unused")
	if FileAccess.file_exists(ROUNDTRIP_PATH):
		return null
	var error := ResourceSaver.save(resource, ROUNDTRIP_PATH)
	_expect(error == OK, "resource saves successfully")
	if error != OK:
		return null
	var loaded := ResourceLoader.load(ROUNDTRIP_PATH, "", ResourceLoader.CACHE_MODE_IGNORE)
	_expect(loaded != null, "resource loads successfully")
	_expect(DirAccess.remove_absolute(ROUNDTRIP_PATH) == OK, "temporary resource is removed")
	return loaded

func _run() -> void:
	var config := VehicleConfig.new()
	_expect(config.engine_reaction_data == null, "Engine reaction is optional and absent by default")
	var reaction := EngineReactionData.new()
	reaction.enabled = false
	reaction.reaction_strength = 0.4
	reaction.vibration_strength = 0.3
	reaction.idle_vibration_max_rpm = 1100.0
	reaction.idle_vibration_multiplier = 1.8
	reaction.idle_vibration_frequency = 6.0
	reaction.redline_vibration_frequency = 18.0
	reaction.maximum_torque = 123.0
	reaction.speed_fade_start_kph = 8.0
	reaction.speed_fade_end_kph = 40.0
	config.engine_reaction_data = reaction
	_expect(config.esc_data == null, "ESC is optional and absent by default")
	var esc := ESCData.new()
	esc.enabled = false
	esc.yaw_damping = 1.25
	esc.minimum_speed = 4.5
	esc.maximum_corrective_torque = 2345.0
	esc.maximum_target_lateral_acceleration = 7.5
	config.esc_data = esc
	var loaded_config := _roundtrip(config) as VehicleConfig
	if loaded_config != null:
		var loaded_reaction := loaded_config.engine_reaction_data
		_expect(loaded_reaction != null, "Engine reaction survives VehicleConfig save/load")
		if loaded_reaction != null:
			_expect(not loaded_reaction.enabled and is_equal_approx(loaded_reaction.reaction_strength, 0.4) and
				is_equal_approx(loaded_reaction.vibration_strength, 0.3) and
				is_equal_approx(loaded_reaction.idle_vibration_max_rpm, 1100.0) and
				is_equal_approx(loaded_reaction.idle_vibration_multiplier, 1.8) and
				is_equal_approx(loaded_reaction.idle_vibration_frequency, 6.0) and
				is_equal_approx(loaded_reaction.redline_vibration_frequency, 18.0) and
				is_equal_approx(loaded_reaction.maximum_torque, 123.0) and
				is_equal_approx(loaded_reaction.speed_fade_start_kph, 8.0) and
				is_equal_approx(loaded_reaction.speed_fade_end_kph, 40.0),
				"All engine reaction settings survive save/load")
		var loaded_esc := loaded_config.esc_data
		_expect(loaded_esc != null, "ESC resource survives VehicleConfig save/load")
		if loaded_esc != null:
			_expect(not loaded_esc.enabled and is_equal_approx(loaded_esc.yaw_damping, 1.25) and
				is_equal_approx(loaded_esc.minimum_speed, 4.5) and
				is_equal_approx(loaded_esc.maximum_corrective_torque, 2345.0) and
				is_equal_approx(loaded_esc.maximum_target_lateral_acceleration, 7.5),
				"All ESC settings survive save/load")
	var aero := VehicleAerodynamicsData.new()
	_expect(_property(aero, "yaw_damping_coefficient").is_empty(), "Aerodynamics no longer exposes yaw control")

	var tire := TireData.new()
	tire.peak_slip_ratio = 0.18
	tire.load_grip_loss_percent = 10.0
	tire.force_response_low_speed_ms = 75.0
	tire.force_response_108_kph_ms = 15.0
	tire.aligning_trail_mm = 80.0
	tire.aligning_trail_retained_percent = 25.0
	var curve := Curve.new()
	curve.add_point(Vector2.ZERO)
	curve.add_point(Vector2.ONE)
	tire.forward_friction_curve = curve
	var loaded_tire := _roundtrip(tire) as TireData
	if loaded_tire != null:
		_expect(is_equal_approx(loaded_tire.peak_slip_ratio, 0.18), "peak slip ratio survives save/load")
		_expect(is_equal_approx(loaded_tire.load_grip_loss_percent, 10.0), "load-loss percentage survives save/load")
		_expect(is_equal_approx(loaded_tire.force_response_low_speed_ms, 75.0) and
			is_equal_approx(loaded_tire.force_response_108_kph_ms, 15.0), "force response times survive save/load")
		_expect(is_equal_approx(loaded_tire.aligning_trail_mm, 80.0) and
			is_equal_approx(loaded_tire.aligning_trail_retained_percent, 25.0), "aligning controls survive save/load")
		_expect(loaded_tire.forward_friction_curve != null and
			is_equal_approx(loaded_tire.forward_friction_curve.sample(1.0), 1.0), "advanced curves survive save/load")
	_expect("suffix:ms" in str(_property(tire, "force_response_low_speed_ms").get("hint_string", "")), "native inspector includes millisecond units")
	_expect("suffix:%" in str(_property(tire, "load_grip_loss_percent").get("hint_string", "")), "native inspector includes percentage units")
	_expect(bool(int(_property(tire, "forward_friction_curve").get("usage", 0)) & PROPERTY_USAGE_EDITOR) and
		bool(int(_property(tire, "lateral_friction_curve").get("usage", 0)) & PROPERTY_USAGE_EDITOR), "both grip curves remain editable in the Inspector")
	var example := load("res://example.tscn") as PackedScene
	_expect(example != null, "example scene loads with the updated curves")
	if example != null:
		var scene_state := example.get_state()
		var tire_count := 0
		for node_index in scene_state.get_node_count():
			for property_index in scene_state.get_node_property_count(node_index):
				if scene_state.get_node_property_name(node_index, property_index) != &"tire_data":
					continue
				var example_tire := scene_state.get_node_property_value(node_index, property_index) as TireData
				if example_tire == null:
					continue
				tire_count += 1
				_expect(is_equal_approx(example_tire.lateral_response_angle, 9.0), "example lateral peak uses nine degrees")
				for grip_curve in [example_tire.forward_friction_curve, example_tire.lateral_friction_curve]:
					_expect(grip_curve != null, "example tire has editable grip curves")
					if grip_curve == null:
						continue
					_expect(is_zero_approx(grip_curve.sample(0.0)) and grip_curve.sample(0.01) > 0.015,
						"example grip builds with a nonzero initial slope")
					_expect(is_equal_approx(grip_curve.sample(1.0), 1.0) and grip_curve.sample(0.5) < 0.9,
						"example grip has a rounded peak instead of an early plateau")
					_expect(grip_curve.sample(1.5) < 1.0 and grip_curve.sample(2.0) >= 0.79 and
						grip_curve.sample(2.0) <= 0.86, "example curves gently decline to sliding grip")
		_expect(tire_count == 5, "all five example axles use the updated tire curves")

	var rack := SteeringRackData.new()
	rack.response_time_ms = 250.0
	rack.steering_half_speed_kph = 80.0
	rack.road_feedback_strength = 0.3
	var loaded_rack := _roundtrip(rack) as SteeringRackData
	if loaded_rack != null:
		_expect(is_equal_approx(loaded_rack.response_time_ms, 250.0) and
			is_equal_approx(loaded_rack.steering_half_speed_kph, 80.0) and
			is_equal_approx(loaded_rack.road_feedback_strength, 0.3), "steering controls survive save/load")

	var differential := DifferentialData.new()
	differential.acceleration_lock_percent = 40.0
	differential.engine_braking_lock_percent = 20.0
	differential.preload_torque = 35.0
	var fields := ["acceleration_lock_percent", "engine_braking_lock_percent", "preload_torque",
		"max_lock_torque", "speed_lock_torque_per_100_rpm"]
	for mode in [DifferentialData.OPEN, DifferentialData.LIMITED_SLIP, DifferentialData.LOCKED]:
		differential.mode = mode
		for field in fields:
			var property := _property(differential, field)
			_expect(not property.is_empty(), "conditional field remains registered: " + field)
			var usage := int(property.get("usage", 0))
			_expect(bool(usage & PROPERTY_USAGE_EDITOR) == (mode == DifferentialData.LIMITED_SLIP),
				"LSD field visibility follows mode: " + field)
			_expect(bool(usage & PROPERTY_USAGE_STORAGE), "hidden LSD fields retain storage: " + field)
	var loaded_diff := _roundtrip(differential) as DifferentialData
	if loaded_diff != null:
		loaded_diff.mode = DifferentialData.LIMITED_SLIP
		_expect(is_equal_approx(loaded_diff.acceleration_lock_percent, 40.0) and
			is_equal_approx(loaded_diff.engine_braking_lock_percent, 20.0) and
			is_equal_approx(loaded_diff.preload_torque, 35.0), "switching modes and saving preserves hidden LSD tuning")

	var smooth_esc := ESCData.new()
	smooth_esc.torque_smoothing_enabled = true
	smooth_esc.torque_engagement_rate = 45000.0
	smooth_esc.torque_release_rate = 90000.0
	_expect(smooth_esc.torque_smoothing_enabled and is_equal_approx(smooth_esc.torque_engagement_rate, 45000.0) and is_equal_approx(smooth_esc.torque_release_rate, 90000.0), "ESC smoothing bindings retain authored settings")
	var authority_rack := SteeringRackData.new()
	authority_rack.minimum_driver_authority = 0.75
	authority_rack.max_feedback_deflection_deg = 4.0
	_expect(is_equal_approx(authority_rack.minimum_driver_authority, 0.75) and is_equal_approx(authority_rack.max_feedback_deflection_deg, 4.0), "Steering authority bindings retain authored settings")
	# Loading the migrated example also validates property names in scene resources.
	var scene := load("res://example.tscn") as PackedScene
	_expect(scene != null, "migrated example loads")
	if scene != null:
		var root := scene.instantiate()
		var car_axle := root.get_node("Vehicle1/Axle") as Axle
		var car_tire := car_axle.get_tire_data()
		var car_config := (root.get_node("Vehicle1") as Vehicle).config
		var truck_config := (root.get_node("Vehicle2") as Vehicle).config
		_expect(car_config.esc_data != null and is_equal_approx(car_config.esc_data.yaw_damping, 1.0) and
			is_equal_approx(car_config.esc_data.maximum_corrective_torque, 5000.0), "Car retains its ESC tuning")
		_expect(truck_config.esc_data != null and is_equal_approx(truck_config.esc_data.yaw_damping, 1.0) and
			is_equal_approx(truck_config.esc_data.maximum_corrective_torque, 3000.0), "Truck retains its ESC tuning")
		_expect(is_equal_approx(car_tire.force_response_low_speed_ms, 60.0), "example retains its tire response setting")
		_expect(is_equal_approx(car_axle.get_differential_data().acceleration_lock_percent, 25.0), "example retains its differential torque effect")
		root.free()
	print("[tuning-resources] ALL_DONE failures=", failures)
	quit(0 if failures == 0 else 1)
