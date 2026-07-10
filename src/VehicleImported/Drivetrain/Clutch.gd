class_name Clutch
extends Node

var vdata : VehicleStaticProperties


var clutch_torque : float
var drivetrain_torque : float


var started : bool = false

var _wheel_inertia_sum : float


func start()->void:
	await  get_tree().create_timer(0.1).timeout
	
	for i : Axle in vdata.axles:
		if !is_zero_approx(i.drive_ratio):
			_wheel_inertia_sum += i.wheels.size() * i.wheels[0].inertia
	
	started = true


func compute_clutch_torque(delta : float)->void:
	if !started:
		return
	
	var engagement : float = 1.0 - vdata.veh_ref.clutch_input
	
	
	var gear_ratio : float = vdata.gearbox.real_ratio * vdata.gearbox_data.final_drive
	
	
	if is_zero_approx(gear_ratio) or engagement < 0.01:
		clutch_torque = lerpf(clutch_torque, 0.0, 0.2)
		return
	
	#gear_ratio = 4
	
	var wheels_inertia_transposed : float = _wheel_inertia_sum *\
											gear_ratio * gear_ratio
	var avg_wh_ang_vel_tr : float = 0.0
	var nr_driven_axles : int = 0
	var wheels_load : float = 0.0
	
	for i : Axle in vdata.axles:
		if !is_zero_approx(i.drive_ratio):
			avg_wh_ang_vel_tr += i.average_angular_velocity * gear_ratio
			nr_driven_axles += 1
			for j : Wheel in  i.wheels:
				wheels_load += j.estimate_load_torque(delta) / gear_ratio * i.drive_ratio
	avg_wh_ang_vel_tr /= nr_driven_axles
	
	if is_zero_approx(avg_wh_ang_vel_tr):
		avg_wh_ang_vel_tr = 0.1
	
	
	var motor_ref : Motor = vdata.motor
	var eng_torque : float = motor_ref.get_engine_torque(motor_ref.motor_state) * \
							vdata.gearbox_data.efficiency
	
	#print(avg_wh_ang_vel_tr,' ', motor_ref.motor_state.angular_velocity)
	
	var avg_system_vel : float = motor_ref.inertia * motor_ref.motor_state.angular_velocity
	avg_system_vel += wheels_inertia_transposed *  avg_wh_ang_vel_tr
	avg_system_vel /= (motor_ref.inertia + wheels_inertia_transposed)
	
	
	var next_avg : float =  avg_system_vel + delta * (eng_torque - wheels_load) / \
							(motor_ref.inertia + wheels_inertia_transposed)
	
	
	
	var required_clutch_torque : float =	(next_avg - motor_ref.motor_state.angular_velocity) \
	* motor_ref.inertia / delta
	
	required_clutch_torque = clampf(required_clutch_torque * engagement, 
							-vdata.clutch_data.max_torque,
							vdata.clutch_data.max_torque)
	
	var clamped_gear_sq : float = clampf(gear_ratio * gear_ratio, 1.0, 600.0)
	var smooth_ratio : float = lerpf(0.10, 0.03, clamped_gear_sq / 600.0)
	
	clutch_torque = lerpf(clutch_torque, required_clutch_torque, smooth_ratio)
