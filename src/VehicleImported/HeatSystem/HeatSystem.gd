class_name HeatSystem
extends Node

const ambient_temperature : float = 20.0

var vdata : VehicleStaticProperties

var motor_temp : float
var brakes_temp : float
var tires_temp : float

var baked_motor_gain : float
var baked_motor_cool : float



func start()->void:
	baked_motor_gain = 0.0015 * vdata.motor.max_torque / vdata.motor_data.displacement * \
								remap(vdata.motor_data.rev_bias,0.0, 1.0, 0.7, 1.0)
	baked_motor_cool = 0.018 * vdata.motor_data.displacement * vdata.motor_data.displacement * \
								lerpf(vdata.motor_data.rev_bias, 1.15, 1.0) /  \
								vdata.motor.max_torque

	
func engine_heat_gain()->float:
	var boost_factor : float = 1.0
	if vdata.turbo_data:
		boost_factor = pow(vdata.turbo.boost + 1.0, 1.4)
	return baked_motor_gain * pow(vdata.motor.motor_state.rpm_factor, 1.3) * \
			boost_factor

func engine_cool_gain()->float:
	return baked_motor_cool 

func get_radiator_cooling()->float:
	if !vdata.radiator_data:
		return 0.0
	var airflow : float = clampf(vdata.veh_ref.linear_velocity.length() / 30.0, 0.1, 4.0)
	
	var output_cool : float = airflow * vdata.radiator_data.efficiency * 1.5
	
	return output_cool

func brakes_heat_gain()->float:
	var add : float = 0.0
	var avg_wh_vel : float = 0.0001
	for wh : Wheel in vdata.wheels:
		add += absf(wh.current_tire_state.applied_brake_torque)
		avg_wh_vel += absf(wh.current_wheel_state.angular_velocity)
	add /= vdata.wheels.size()
	avg_wh_vel /= vdata.wheels.size()
	const heat_base : float = 0.002
	return avg_wh_vel * add * heat_base

func brakes_cool_gain()->float:
	var sp_ms : float = absf(vdata.veh_ref.linear_velocity.length())
	var speed_factor : float = clampf(sp_ms / 30, 0.8, 2.0)
	var temp_dif : float = brakes_temp - ambient_temperature
	temp_dif = maxf(temp_dif, 0.0) * 0.5
	
	return speed_factor * vdata.brakes_data.heat_dissipation * \
			0.18 * temp_dif

func tires_heat_gain()->float:
	var add_heat : float = 0.0
	for i : Wheel in vdata.wheels:
		
		var slip_heat : float = 0.35 * absf(i.current_tire_state.slip_forward) + \
								absf(i.current_tire_state.slip_lateral)
		add_heat += i.current_wheel_state.sustained_mass * 9.8 * \
					slip_heat * vdata.tire_data.tire_temp_gain * 0.0012
		
	return add_heat / vdata.wheels.size()

func tires_cool_gain()->float:
	var sp_ms : float = absf(vdata.veh_ref.linear_velocity.length())
	var airflow : float =clampf(sp_ms /30.0, 0.3, 1.5)
	var temp_diff : float = (tires_temp - ambient_temperature)
	temp_diff = maxf(temp_diff, 0.0)
	var added_cool : float = temp_diff * airflow * vdata.tire_data.tire_temp_cool * 0.02
	return added_cool

func get_brakes_efficiency()->float:
	if brakes_temp <= vdata.brakes_data.heat_rating:
		return 1.0
	return lerpf(1.0, 0.2, minf((brakes_temp - vdata.brakes_data.heat_rating) * 0.01, 1.0))

func get_tires_efficiency()->float:
	if tires_temp < 100.0:
		return 1.0
	return lerpf(1.0, 0.6, minf((tires_temp - 100.0) * 0.02, 1.0))

func get_motor_efficiency()->float:
	var motor_temp_cl : float = clampf(motor_temp, 90.0, 130.0)
	return remap(motor_temp_cl, 90.0, 130.0, 1.0, 0.0)

func update_physics(delta : float)->void:
	motor_temp += delta * engine_heat_gain()
	motor_temp -= engine_cool_gain() * delta
	motor_temp -= get_radiator_cooling() * (motor_temp - ambient_temperature) * delta * 0.018 * 0.1
	
	brakes_temp += (brakes_heat_gain() - brakes_cool_gain()) * delta
	
	tires_temp += (tires_heat_gain() - tires_cool_gain()) * delta
	
	#print(tires_cool_gain())
	
	#print(get_radiator_cooling() * (motor_temp - ambient_temperature) * delta * 0.018)
	
	#print(engine_heat_gain() - engine_cool_gain())
	#print(engine_heat_gain() - engine_cool_gain())
	
	motor_temp = maxf(motor_temp, 90.0)
