extends Control

## Presentation adapter for a VehicleTelemetry snapshot.
##
## The labels remain authored by the scene; only the data source is exported,
## making this display reusable with another Vehicle node.

@export var vehicle: Vehicle

var _telemetry: VehicleTelemetry
@onready var _labels: Node = $VBoxContainer
@onready var _fps_label: Label = $VBoxContainer/FPSLabel


func _ready() -> void:
	_resolve_telemetry()


func _process(_delta: float) -> void:
	_fps_label.text = "FPS: %d" % int(Engine.get_frames_per_second())

	if not is_instance_valid(_telemetry):
		_resolve_telemetry()
	if not is_instance_valid(_telemetry) or not is_instance_valid(_labels):
		return

	# Engine
	_labels.get_node("EngineRPMLabel").text = "Engine RPM: %d" % int(_telemetry.get_engine_rpm())
	_labels.get_node("EngineTorqueLabel").text = "Engine Torque: %.1f Nm" % _telemetry.get_engine_torque()
	_labels.get_node("ThrottleLabel").text = "Throttle: %.2f" % _telemetry.get_engine_throttle()

	# Turbo
	_labels.get_node("TurboLabel").text = "Turbo Boost: %.2f bar" % _telemetry.get_turbo_boost()

	# Driveshaft
	_labels.get_node("DriveshaftRPMLabel").text = "Driveshaft RPM: %d" % int(_telemetry.get_driveshaft_rpm())

	# Clutch + gearbox
	_labels.get_node("ClutchLabel").text = "Clutch: %.2f" % _telemetry.get_clutch_engagement()
	var gear = _telemetry.get_current_gear()
	var ratio = _telemetry.get_gear_ratio()
	_labels.get_node("GearLabel").text = "Gear: %d (%.2f:1)" % [gear, ratio]

	# Speed
	_labels.get_node("SpeedLabel").text = "Speed: %.1f km/h" % _telemetry.get_vehicle_speed_kph()

	# Wheel angular velocities
	var wheel_rpms = _telemetry.get_wheel_rpms()
	for i in range(wheel_rpms.size()):
		var label := _labels.get_node_or_null("Wheel%dRPMLabel" % (i + 1)) as Label
		if label != null:
			label.text = "Wheel %d: %d RPM" % [i + 1, int(wheel_rpms[i])]

	# Tire forces
	var forces = _telemetry.get_tire_forces()
	for i in range(forces.size()):
		var label := _labels.get_node_or_null("Wheel%dForceLabel" % (i + 1)) as Label
		if label != null:
			label.text = "Wheel %d Force: (%.0f, %.0f, %.0f)" % [
				i + 1, forces[i].x, forces[i].y, forces[i].z
			]


func _resolve_telemetry() -> void:
	_telemetry = null
	if not is_instance_valid(vehicle):
		return
	_telemetry = vehicle.get_node_or_null("VehicleTelemetry") as VehicleTelemetry
