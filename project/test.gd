extends Node3D


var _telemetry: Node = null

func _ready() -> void:
	_telemetry = $Vehicle/VehicleTelemetry


func _process(_delta: float) -> void:
	if _telemetry == null:
		return

	var ctrl = $Control/VBoxContainer

	# Engine
	ctrl.get_node("EngineRPMLabel").text = "Engine RPM: %d" % int(_telemetry.get_engine_rpm())
	ctrl.get_node("EngineTorqueLabel").text = "Engine Torque: %.1f Nm" % _telemetry.get_engine_torque()
	ctrl.get_node("ThrottleLabel").text = "Throttle: %.2f" % _telemetry.get_engine_throttle()

	# Turbo
	ctrl.get_node("TurboLabel").text = "Turbo Boost: %.2f bar" % _telemetry.get_turbo_boost()

	# Driveshaft
	ctrl.get_node("DriveshaftRPMLabel").text = "Driveshaft RPM: %d" % int(_telemetry.get_driveshaft_rpm())

	# Clutch + Gearbox
	ctrl.get_node("ClutchLabel").text = "Clutch: %.2f" % _telemetry.get_clutch_engagement()
	var gear = _telemetry.get_current_gear()
	var ratio = _telemetry.get_gear_ratio()
	ctrl.get_node("GearLabel").text = "Gear: %d (%.2f:1)" % [gear, ratio]

	# Speed
	ctrl.get_node("SpeedLabel").text = "Speed: %.1f km/h" % _telemetry.get_vehicle_speed_kph()

	# Wheel angular velocities
	var wheel_rpms = _telemetry.get_wheel_angular_velocities()
	for i in range(wheel_rpms.size()):
		var label = ctrl.get_node_or_null("Wheel%dRPMLabel" % (i + 1))
		if label:
			label.text = "Wheel %d: %d RPM" % [i + 1, int(wheel_rpms[i])]

	# Tire forces
	var forces = _telemetry.get_tire_forces()
	for i in range(forces.size()):
		var label = ctrl.get_node_or_null("Wheel%dForceLabel" % (i + 1))
		if label:
			label.text = "Wheel %d Force: (%.0f, %.0f, %.0f)" % [i + 1, forces[i].x, forces[i].y, forces[i].z]
	var th_input : float = Input.is_action_pressed("ui_up")
	for i in get_children():
		if i is Vehicle:
			i.set_throttle_input(th_input)
	$Vehicle.set_brake_input(Input.is_action_pressed("ui_down"))
	var steer : float = -Input.get_axis("ui_left", "ui_right")
	$Vehicle.set_steer_input(steer)
	
	var shift_input : int = 0
	if Input.is_action_just_pressed("shift_up"):
		shift_input = 1
	if Input.is_action_just_pressed("shift_down"):
		shift_input = -1
	$Vehicle.set_shift_input(shift_input)
