class_name VehicleController
extends Node

var vdata : VehicleStaticProperties

@export var is_player_controlled : bool = true
var throttle_input : float
var steer_input : float
var brake_input : float



func _physics_process(delta: float) -> void:
	if !is_player_controlled:
		return
	var veh : Vehicle = vdata.veh_ref
	
	
	throttle_input = lerpf( throttle_input, float(Input.get_action_strength("ui_up")), delta * 10)
	brake_input = Input.get_action_strength("ui_down")
	steer_input = move_toward(steer_input, Input.get_axis("ui_left","ui_right"), delta * 7.5)
	
	if vdata.gearbox_data.is_automatic:
		if vdata.gearbox.gear != 0:
			veh.throttle_input = throttle_input
			veh.brake_input = brake_input
		else:
			veh.throttle_input = brake_input
			veh.brake_input = throttle_input
	else:
		veh.throttle_input = throttle_input
		veh.brake_input = brake_input
		
	veh.steer_input = steer_input
	veh.shift_up = Input.is_action_just_pressed("shift_up")
	veh.shift_dn = Input.is_action_just_pressed("shift_dn")
	if veh.gearbox_data.is_automatic == false:
		veh.clutch_input = lerpf(veh.clutch_input, Input.is_action_pressed("clutch"), 
						   delta * vdata.clutch_data.clutch_speed)
	else:
		if Input.is_action_pressed("clutch"):
			veh.clutch_input = lerpf(veh.clutch_input, 1.0, delta * vdata.gearbox_data.auto_clutch_speed)
	if veh.clutch_input > 0.99:
		veh.clutch_input = 1.0
