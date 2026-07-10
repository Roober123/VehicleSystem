class_name TireModel
extends RefCounted

var peak_slip_vel : float = 4.0
var peak_slip_angle : float = 16


func get_tire_forces(wh : Wheel, old_state : TireState, delta : float)->Dictionary:
	
	var res : Dictionary
	var wh_vel : Vector3 = wh.current_wheel_state.velocity
	var ang : float = wh.current_wheel_state.angular_velocity
	
	var fwd : float = wh.global_basis.z.dot(wh_vel)
	var lat : float = wh.global_basis.x.dot(wh_vel)
	
	var temp_modifier : float = wh.vdata.heat_system.get_tires_efficiency()
	
	var new_mu : float = wh.vdata.tire_data.mu
	var fwd_mu : float = new_mu * temp_modifier
	var lat_mu : float = fwd_mu * temp_modifier * 1.15
	
	if absf(fwd) + absf(lat) < 1.0:
		fwd_mu *= 1.3
		lat_mu *= 1.3
	
	
	#print(absf(rad_to_deg(wh.turn)) / 30.0)
	
	if wh.is_rear:
		lat_mu -= wh.vdata.tire_data.drift_coeff
		lat_mu += 0.12
	var normal : float = maxf(wh.current_wheel_state.sustained_mass * 9.8, 0.001)
	
	
	var slip_vel : float = ang * wh.vdata.tire_data.radius - fwd
	
	res['slip_fwd'] = slip_vel
	
	var fwd_force : float = normal * fwd_mu * tanh(slip_vel / peak_slip_vel)
	
	
	var slip_angle : float = atan2(lat,abs(fwd) + 2.5)
	slip_angle = clampf(slip_angle,-deg_to_rad(32),deg_to_rad(32))
	
	res['slip_lat'] = slip_angle
	
	var lat_force : float = normal * lat_mu * \
							tanh(slip_angle / deg_to_rad(peak_slip_angle))
	
	var nx : float = lat_force / (lat_mu * normal)
	var ny : float = fwd_force / (fwd_mu * normal)
	
	
	var sum : float = sqrt(nx * nx + ny * ny)
	if sum > 1.0:
		var r : float = 1.0/sum
		fwd_force *= r
		lat_force *= r
	
	
	var clamped_vel : float = clampf(wh_vel.length(), -30.0, 30.0)
	var relaxation_time : float = lerpf(0.042, 0.01, clamped_vel / 30.0)
	
	if old_state:
		fwd_force = lerpf(old_state.longitudinal_force, fwd_force, delta / relaxation_time)
		lat_force = lerpf(old_state.lateral_force, lat_force, delta / relaxation_time * 1.3)
	
	res['fwd'] = fwd_force
	res['lat'] = lat_force
	
	var clamped_slip : float = clampf(slip_angle,
							-deg_to_rad(25), deg_to_rad(25))
	var peak_angle_rad : float = deg_to_rad(peak_slip_angle)
	
	
	var trail : float = 0.0
	if absf(clamped_slip) < peak_angle_rad:
		trail = absf(clamped_slip) / peak_angle_rad
	else:
		trail = remap(absf(clamped_slip), peak_angle_rad, deg_to_rad(25), 1.0, 0.0)
	#print(trail)
	trail *= 0.12
	if absf(clamped_slip) < 0.01:
		trail = 0.0
	
	
	trail *= exp(-maxf(0.0, absf(fwd * 3.6) - 50) * 0.04)
	
	var aligning_torque : float = -signf(slip_angle) * absf(lat_force) * trail
	
	if absf(aligning_torque) < 0.3:
		aligning_torque = 0.0
	
	
	
	res['align'] = aligning_torque / wh.vdata.tire_data.radius
	
	return res
