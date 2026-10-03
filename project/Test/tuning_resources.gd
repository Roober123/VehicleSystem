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
	var tire := TireData.new()
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
		_expect(is_equal_approx(loaded_tire.load_grip_loss_percent, 10.0), "load-loss percentage survives save/load")
		_expect(is_equal_approx(loaded_tire.force_response_low_speed_ms, 75.0) and
			is_equal_approx(loaded_tire.force_response_108_kph_ms, 15.0), "force response times survive save/load")
		_expect(is_equal_approx(loaded_tire.aligning_trail_mm, 80.0) and
			is_equal_approx(loaded_tire.aligning_trail_retained_percent, 25.0), "aligning controls survive save/load")
		_expect(loaded_tire.forward_friction_curve != null and
			is_equal_approx(loaded_tire.forward_friction_curve.sample(1.0), 1.0), "advanced curves survive save/load")
	_expect("suffix:ms" in str(_property(tire, "force_response_low_speed_ms").get("hint_string", "")), "native inspector includes millisecond units")
	_expect("suffix:%" in str(_property(tire, "load_grip_loss_percent").get("hint_string", "")), "native inspector includes percentage units")

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

	# Loading the migrated example also validates property names in scene resources.
	var scene := load("res://example.tscn") as PackedScene
	_expect(scene != null, "migrated example loads")
	if scene != null:
		var root := scene.instantiate()
		var car_axle := root.get_node("Vehicle1/Axle") as Axle
		var car_tire := car_axle.get_tire_data()
		_expect(is_equal_approx(car_tire.force_response_low_speed_ms, 60.0), "example retains its tire response setting")
		_expect(is_equal_approx(car_axle.get_differential_data().acceleration_lock_percent, 24.0), "example retains its differential torque effect")
		root.free()
	print("[tuning-resources] ALL_DONE failures=", failures)
	quit(0 if failures == 0 else 1)
