class_name Turbo
extends Node

var vdata : VehicleStaticProperties

var turbo_spool : float = 0.0
var boost : float

func update_physics(delta : float)->void:
	if !vdata.turbo_data:
		return
	var exhaust : float = vdata.motor.get_engine_torque(vdata.motor.motor_state) * \
	vdata.veh_ref.throttle_input * vdata.motor.motor_state.rpm
	
	var peak_rpm_torque : float = vdata.motor.torque_curve.tq_peak * \
	vdata.motor.redline_rpm * vdata.motor.max_torque
	
	
	
	exhaust /= peak_rpm_torque
	
	exhaust = clampf(exhaust, 0.0, 1.0)
	
	
	if exhaust > turbo_spool:
		turbo_spool += delta * vdata.turbo_data.spool_up * (exhaust - turbo_spool)
	else:
		turbo_spool += delta * vdata.turbo_data.spool_dn * (exhaust - turbo_spool)
	
	turbo_spool = clampf(turbo_spool, 0.0, 1.0)
	if vdata.veh_ref.throttle_input < 0.1:
		turbo_spool *= 0.9
	boost = vdata.turbo_data.max_boost * smoothstep(0.3, 1.0, turbo_spool)
