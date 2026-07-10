class_name TCS
extends Node

@export var engage_slip : float = 3.0
@export var max_slip : float = 6.0

func clamp_torque(tq : float, slip : float)->float:
	if absf(slip) < engage_slip:
		return tq
	if absf(slip) > max_slip:
		return 0.0
	return tq * remap(absf(slip), engage_slip, max_slip, 1.0, 0.0)
