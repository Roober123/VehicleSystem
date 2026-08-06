extends Node

## Small keyboard/controller input adapter for a Vehicle.
##
## The target is exported rather than looked up by a fixed scene path. This
## keeps the controls reusable when the example scene grows another vehicle.

const DEFAULT_THROTTLE_RISE_RATE := 3.0

@export var vehicle: Vehicle
@export_range(0.0, 20.0, 0.1) var throttle_rise_rate := DEFAULT_THROTTLE_RISE_RATE

var _telemetry: VehicleTelemetry
var _forward_throttle := 0.0
var _reverse_throttle := 0.0


func _ready() -> void:
	_resolve_telemetry()


func _process(delta: float) -> void:
	if not is_instance_valid(vehicle):
		return
	if not is_instance_valid(_telemetry):
		_resolve_telemetry()
	if not is_instance_valid(_telemetry):
		return

	var forward_pressed := float(Input.is_action_pressed("ui_up"))
	var reverse_pressed := float(Input.is_action_pressed("ui_down"))
	var current_gear: int = _telemetry.get_current_gear()

	if current_gear < 0:
		_forward_throttle = 0.0
		_reverse_throttle = move_toward(
			_reverse_throttle, reverse_pressed, throttle_rise_rate * delta)
		# Vehicle swaps the pedals in reverse: brake_input drives the engine,
		# while throttle_input is passed to the wheel brakes.
		vehicle.set_throttle_input(forward_pressed)
		vehicle.set_brake_input(_reverse_throttle)
	else:
		_reverse_throttle = 0.0
		_forward_throttle = move_toward(
			_forward_throttle, forward_pressed, throttle_rise_rate * delta)
		vehicle.set_throttle_input(_forward_throttle)
		vehicle.set_brake_input(reverse_pressed)

	vehicle.set_steer_input(-Input.get_axis("ui_left", "ui_right"))
	if Input.is_action_just_pressed("shift_up"):
		vehicle.shift_up()
	elif Input.is_action_just_pressed("shift_down"):
		vehicle.shift_down()


func _resolve_telemetry() -> void:
	_telemetry = null
	if not is_instance_valid(vehicle):
		return
	_telemetry = vehicle.get_node_or_null("VehicleTelemetry") as VehicleTelemetry
