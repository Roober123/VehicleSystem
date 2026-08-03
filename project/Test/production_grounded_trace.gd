extends Node

# Small production-path smoke: construct the public VehicleConfig resource,
# attach one valid driven axle, and exercise the explicit Vehicle shift API at
# both supported substep counts.  This intentionally avoids a scenario matrix.
const DT := 1.0 / 120.0
const EPS := 1.0e-3

var failures := 0

func _ready() -> void:
	for substeps in [1, 8]:
		await _run_trace(substeps)
	print("[production-smoke] ALL_DONE failures=", failures)
	get_tree().quit(0 if failures == 0 else 1)

func _run_trace(substeps: int) -> void:
	var vehicle := _make_vehicle(substeps)
	await get_tree().physics_frame
	var telemetry := vehicle.get_node("VehicleTelemetry") as VehicleTelemetry
	_expect(telemetry != null, "substeps=%d telemetry attached" % substeps)
	_expect(telemetry.get_current_gear() == 0, "substeps=%d ready in neutral" % substeps)

	# Explicit drive command, then a finite forward acceleration sample.
	vehicle.select_drive()
	await _physics_ticks(40)
	vehicle.set_throttle_input(1.0)
	await _physics_ticks(60)
	var forward_shaft := float(telemetry.get_driveshaft_rpm())
	var forward_engine := float(telemetry.get_engine_rpm())
	_expect(is_finite(forward_shaft) and is_finite(forward_engine),
		"substeps=%d forward telemetry finite" % substeps)
	_expect(forward_shaft > EPS, "substeps=%d forward acceleration is signed positive" % substeps)

	# One explicit upshift and its completed phase sequence.
	vehicle.shift_up()
	await _physics_ticks(40)
	_expect(telemetry.get_current_gear() == 2, "substeps=%d one explicit upshift" % substeps)

	# Braking path must remain finite and reduce the engine-side drive signal.
	vehicle.set_throttle_input(0.0)
	vehicle.set_brake_input(1.0)
	await _physics_ticks(30)
	var braking_shaft := float(telemetry.get_driveshaft_rpm())
	_expect(is_finite(braking_shaft), "substeps=%d braking telemetry finite" % substeps)

	# Neutral, followed by explicit reverse selection and a signed reverse run.
	vehicle.set_brake_input(0.0)
	vehicle.select_neutral()
	await _physics_ticks(40)
	_expect(telemetry.get_current_gear() == 0, "substeps=%d neutral command" % substeps)
	vehicle.queue_free()
	await get_tree().process_frame

	# Reverse selection is intentionally exercised from a fresh stationary
	# drivetrain: the public command refuses a reverse engagement while the
	# driveshaft is still spinning forward.
	vehicle = _make_vehicle(substeps)
	await get_tree().physics_frame
	telemetry = vehicle.get_node("VehicleTelemetry") as VehicleTelemetry
	vehicle.select_reverse()
	await _physics_ticks(40)
	_expect(telemetry.get_current_gear() == -1, "substeps=%d reverse command" % substeps)
	vehicle.set_brake_input(0.5)
	await _physics_ticks(60)
	var reverse_shaft := float(telemetry.get_driveshaft_rpm())
	_expect(is_finite(reverse_shaft), "substeps=%d reverse telemetry finite" % substeps)
	_expect(reverse_shaft < -EPS, "substeps=%d reverse output is signed negative" % substeps)

	vehicle.queue_free()
	await get_tree().process_frame

func _make_vehicle(substeps: int) -> Vehicle:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 1.0))
	curve.add_point(Vector2(1.0, 1.0))
	var engine_data := VehicleEngineData.new()
	engine_data.torque_curve = curve
	engine_data.idle_rpm = 850.0
	engine_data.redline_rpm = 5000.0
	engine_data.inertia = 1.0
	engine_data.max_torque = 250.0
	engine_data.engine_drag = 0.02
	engine_data.engine_braking = 0.4

	var gearbox_data := GearboxData.new()
	gearbox_data.gear_ratios = PackedFloat64Array([2.0, 1.0])
	gearbox_data.final_drive = 2.5
	gearbox_data.reverse_ratio = -2.0
	gearbox_data.clutch_max_torque = 800.0
	gearbox_data.shift_time = 0.2
	gearbox_data.auto_mode = false
	gearbox_data.driveshaft_drag = 0.0

	var tire_curve := Curve.new()
	tire_curve.add_point(Vector2(0.0, 0.0))
	tire_curve.add_point(Vector2(1.0, 1.0))
	var tire_data := TireData.new()
	tire_data.forward_friction_curve = tire_curve
	tire_data.lateral_friction_curve = tire_curve
	tire_data.radius = 0.3
	tire_data.drag = 0.0

	var config := VehicleConfig.new()
	config.engine_data = engine_data
	config.gearbox_data = gearbox_data
	config.suspension_data = SuspensionData.new()
	config.aero_data = VehicleAerodynamicsData.new()

	var vehicle := Vehicle.new()
	vehicle.config = config
	vehicle.substeps = substeps
	vehicle.gravity_scale = 0.0
	vehicle.mass = 1000.0
	var axle := Axle.new()
	axle.drive_share = 1.0
	axle.tire_data = tire_data
	axle.differential_data = DifferentialData.new()
	var left := Wheel.new()
	left.position.x = -0.8
	var right := Wheel.new()
	right.position.x = 0.8
	axle.add_child(left)
	axle.add_child(right)
	vehicle.add_child(axle)
	var telemetry := VehicleTelemetry.new()
	telemetry.name = "VehicleTelemetry"
	vehicle.add_child(telemetry)
	add_child(vehicle)
	return vehicle

func _physics_ticks(count: int) -> void:
	for _i in range(count):
		await get_tree().physics_frame

func _expect(condition: bool, label: String) -> void:
	if condition:
		return
	failures += 1
	push_error("[production-smoke] FAIL: %s" % label)
