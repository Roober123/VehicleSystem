class_name Gearbox
extends Node

var vdata : VehicleStaticProperties


var gear : int = 1

var real_ratio : float

var is_shifting : bool
var target_gear : int

var unclutched : bool = true
var auto_shift_cool : float = 0.0

func _adjust_shift_number(nr : int)->int:
	var new_gear : int = gear + nr
	new_gear = clampi(new_gear, 0, 
			   vdata.gearbox_data.ratios.size() - 1)
	return new_gear - gear



func shift(nr : int)->void:
	nr = _adjust_shift_number(nr)
	if nr != 0:
		is_shifting = true
		target_gear = gear + nr
		return

func can_shift()->bool:
	return vdata.veh_ref.clutch_input > 0.83 and !is_shifting

func manual_gearbox()->void:
	if vdata.veh_ref.shift_up and can_shift():
		shift(1)
	elif vdata.veh_ref.shift_dn and can_shift():
		shift(-1)

func auto_gearbox(delta : float)->void:
	if gear <= 2:
		auto_shift_cool = move_toward(auto_shift_cool, 0.0, delta)
	elif vdata.veh_ref.clutch_input < 0.2 or \
	vdata.veh_ref.linear_velocity.length() < 2.0:
		auto_shift_cool = move_toward(auto_shift_cool, 0.0, delta)
	
	
	var rpm_ratio : float = vdata.motor.motor_state.rpm \
	/ vdata.motor.redline_rpm
	var shift_nr : int = 0
	if gear > 2:
		if rpm_ratio > vdata.gearbox_data.auto_upshift:
			shift_nr = 1
		elif rpm_ratio < vdata.gearbox_data.auto_downshift:
			shift_nr = -1
	elif gear == 2:
		if vdata.motor.motor_state.rpm < 1100.0 and \
			vdata.veh_ref.brake_input > 0.8:
				shift_nr = -2
		if rpm_ratio > vdata.gearbox_data.auto_upshift:
			shift_nr = 1
	elif gear == 1:
		shift(1)
	else:
		if vdata.veh_ref.brake_input > 0.8:
			shift_nr = 2
	
	if !is_zero_approx(auto_shift_cool):
		shift_nr = 0
	
	
	if can_shift() and shift_nr != 0:
		shift(shift_nr)
		auto_shift_cool = 0.2
	
	
	if shift_nr != 0:
		vdata.veh_ref.clutch_input = move_toward(vdata.veh_ref.clutch_input,
		1.0, delta * vdata.gearbox_data.auto_clutch_speed)
	elif vdata.motor.rpm > 820.0:
		var first_gear_modify : float = 1.0
		if gear == 2:
			first_gear_modify = 0.3
		vdata.veh_ref.clutch_input = move_toward(vdata.veh_ref.clutch_input,
		0.0, delta * vdata.gearbox_data.auto_clutch_speed * first_gear_modify)
		#print(delta * vdata.gearbox_data.auto_clutch_speed * first_gear_modify)


func update_physics(delta : float)->void:
	if is_shifting:
		var ratios : Array[float] = vdata.gearbox_data.ratios
		var old_clutch_torque : float = absf(vdata.clutch.clutch_torque)
		var torque_difference : float = (ratios[target_gear] - real_ratio)
		
		torque_difference = absf(torque_difference) * old_clutch_torque
		
		if  is_equal_approx(real_ratio, ratios[target_gear]):
			gear = target_gear
			real_ratio = ratios[target_gear]
			is_shifting = false
			return
		
		var remaining : float = torque_difference - vdata.gearbox_data.torque_resistance
		remaining = max(remaining, 0)
		if is_zero_approx(torque_difference):
			torque_difference = 1
		real_ratio = lerpf(real_ratio, ratios[target_gear], 1.0 - remaining / torque_difference)
	else:
		if vdata.gearbox_data.is_automatic:
			auto_gearbox(delta)
		else:
			manual_gearbox()
