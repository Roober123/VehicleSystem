class_name MotorData
extends Resource

@export_range(0.8, 10.0, 0.1) var displacement : float = 1.0
@export var cylinders : int = 2

@export_range(0.0, 1.0) var materials_mass_efficiency : float

@export_range(0.3, 1.1) var piston_efficiency : float = 1.0

# small rev bias -> diesel, much torque
# high rev bias -> petrol, high rev, lower torque
@export_range(0.0, 1.0, 0.01)
var rev_bias : float = 0.5

const idle_rpm : float = 800

func get_inertia()->float:
	return displacement * 0.065 * lerpf(1.6, 0.55, rev_bias * rev_bias)

func get_redline_rpm()->float:
	var base_redline : float = 1000
	if cylinders == 2:
		base_redline = 7300.0
	elif cylinders == 3:
		base_redline = 7200.0
	elif cylinders == 4:
		base_redline = 7100.0
	elif cylinders == 6:
		base_redline = 6950.0
	elif cylinders == 8:
		base_redline = 6750.0
	elif cylinders == 10:
		base_redline = 6550.0
	elif cylinders == 12:
		base_redline = 6350.0
	var redline_rpm : float = base_redline * lerpf(0.85,1.15,rev_bias)
	var disp_norm : float = displacement / 10.0
	redline_rpm *= lerpf(1.2, 0.6, disp_norm)
	return redline_rpm

func get_max_torque_rpm()->float:
	var redline : float = get_redline_rpm()
	return lerpf(redline * 0.45, redline * 0.85, rev_bias)

func get_max_torque()->float:
	var base_torque : float = 110 * displacement
	var torque_factor : float = lerpf(1.0, 0.71, rev_bias)
	var torque_multiplier : float = 1.0
	if cylinders == 2:
		torque_multiplier = 0.5
	if cylinders == 3:
		torque_multiplier = 0.75
	if cylinders == 4:
		torque_multiplier = 0.88
	if cylinders == 6:
		torque_multiplier = 1.0
	if cylinders == 8:
		torque_multiplier = 1.2
	if cylinders == 10:
		torque_multiplier = 1.4
	if cylinders == 12:
		torque_multiplier = 1.6
	torque_factor *= torque_multiplier
	var max_torque : float = base_torque * torque_factor * piston_efficiency
	return max_torque

class CurveData:
	var rise : float = 1.0
	var fall : float = 2.0
	var tq_peak : float = 0.5
	var plateau_width : float = 0.0
	var redline : float
	var _extra_plateau_torque : float 
	var _extra_rev_torque : float
	var _extra_torque : float
	var plateau_down : float
	var plateau_up : float
	func _init(data : MotorData) -> void:
		rise = lerpf(0.4, 0.6, data.rev_bias)
		fall = lerpf(1.6, 0.5, data.rev_bias)
		
		var rev_plateau_factor : float = lerpf(0.5, 0.07, data.rev_bias)
		
		
		plateau_width = rev_plateau_factor
		plateau_width *= lerpf(0.8, 1.1, data.cylinders / 12.0)
		
		_extra_plateau_torque = plateau_width * 0.18
		_extra_rev_torque = (1.0 - data.rev_bias) * 0.1
		_extra_torque = _extra_plateau_torque + _extra_rev_torque
		
		
		var pk_tq_rpm : float = data.get_max_torque_rpm()
		redline = data.get_redline_rpm()
		tq_peak = (pk_tq_rpm - idle_rpm) / (redline - idle_rpm)
		tq_peak = clampf(tq_peak, 0.25, 0.80)
		
		
		
		plateau_down = clampf(tq_peak - plateau_width / 2.0, 0.15, 0.85)
		var add : float = 0.0
		if is_equal_approx(plateau_down, 0.15):
			add = 0.15 - (tq_peak - plateau_width / 2.0)
		
		plateau_up = clampf(tq_peak + plateau_width / 2.0 + add, 0.15, 0.85)
		
		#print(plateau_down,' ', plateau_up)
		#print(tq_peak)

func torque_curve(cv : MotorData.CurveData, rpm : float)->float:
	var x : float = (rpm - idle_rpm) / (cv.redline - idle_rpm)
	x = clampf(x, 0.0, 1.0)
	var tq : float 
	if x > cv.plateau_down and x < cv.plateau_up:
		tq = 1.0
	else:
		if x < cv.plateau_down:
			tq = pow(x / cv.plateau_down, cv.rise)
		elif x > cv.plateau_up:
			tq = (x - cv.plateau_up) / (1.0 - cv.plateau_up)
			tq = pow(1.0 - tq, cv.fall)
	
	tq = clampf(tq, 0.32, 1.0)
	
	return tq

const kgs_per_liter : float = 32
const kgs_per_cylinder : float = 8
const diesel_mass_factor : float = 30

var mass : float:
	get():
		return (kgs_per_cylinder * cylinders + \
			   kgs_per_liter * displacement + \
			   diesel_mass_factor * (1.0 - rev_bias) + 40.0) * \
			   (1.0 + (0.5 - materials_mass_efficiency) * 0.5)
