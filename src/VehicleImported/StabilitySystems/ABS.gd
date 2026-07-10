class_name ABS
extends Node

@export var engage_slip : float = 7.0
@export var max_slip : float = 12.0


func clamp_brake_torque(tq : float, slip : float)->float:
	slip = absf(slip)
	if slip < engage_slip:
		return tq
	if slip > max_slip:
		return 0.0
	return remap(slip,engage_slip,max_slip,0.0,1.0) * tq
