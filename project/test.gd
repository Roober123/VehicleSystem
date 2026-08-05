extends Node3D


var _telemetry: Node = null
var _returning_to_menu := false

func _ready() -> void:
	_telemetry = $Vehicle/VehicleTelemetry
	$NavigationLayer/NavigationPanel/BackToMenuButton.pressed.connect(_return_to_menu)

const THROTTLE_RISE_RATE := 3.0

var forward_throttle := 0.0
var reverse_throttle := 0.0

func _process(dt: float) -> void:
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
	var wheel_rpms = _telemetry.get_wheel_rpms()
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

	var forward_pressed := float(Input.is_action_pressed("ui_up"))
	var reverse_pressed := float(Input.is_action_pressed("ui_down"))
	var current_gear: int = _telemetry.get_current_gear()
	if current_gear < 0:
		forward_throttle = 0.0
		reverse_throttle = move_toward(
			reverse_throttle, reverse_pressed, THROTTLE_RISE_RATE * dt)
		# Vehicle swaps the pedal roles in reverse: brake_input drives the
		# engine and throttle_input is passed to the wheel brakes. Keep reverse
		# acceleration smoothed, but apply the opposing pedal as a direct brake.
		$Vehicle.set_throttle_input(forward_pressed)
		$Vehicle.set_brake_input(reverse_throttle)
	else:
		reverse_throttle = 0.0
		forward_throttle = move_toward(
			forward_throttle, forward_pressed, THROTTLE_RISE_RATE * dt)
		$Vehicle.set_throttle_input(forward_throttle)
		$Vehicle.set_brake_input(reverse_pressed)
	var steer : float = -Input.get_axis("ui_left", "ui_right")
	$Vehicle.set_steer_input(steer)
	
	if Input.is_action_just_pressed("shift_up"):
		$Vehicle.shift_up()
	elif Input.is_action_just_pressed("shift_down"):
		$Vehicle.shift_down()


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("return_to_menu") or event.is_action_pressed("ui_cancel"):
		_return_to_menu()
		get_viewport().set_input_as_handled()


func _return_to_menu() -> void:
	if _returning_to_menu:
		return
	_returning_to_menu = true
	var result := get_tree().change_scene_to_file("res://main_menu.tscn")
	if result != OK:
		_returning_to_menu = false
		push_error("Unable to return to the main menu (error %d)." % result)
