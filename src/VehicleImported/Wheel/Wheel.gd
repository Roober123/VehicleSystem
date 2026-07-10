class_name Wheel
extends Node3D

var vdata : VehicleStaticProperties
var effective_mass : float

var stiffness : float
var damping : float
var inertia : float = 2.0
var is_rear : bool = false
var steerable : bool = false
var wheel_mesh : Node3D

var current_wheel_state : WheelState = WheelState.new()
var current_tire_state : TireState
var tm : TireModel

var ray : RayCast3D

var last_position : Vector3
var turn : float

var turn_target : float
var steer_stiffness : float = 162
var steer_damping : float = 20
var steer_velocity : float
var max_steer_angle : float
var max_steer_angle_default : float 

func get_inverse_effective_mass(axis : Vector3)->float:
	var cr : Vector3 = (global_position - vdata.com.global_position).cross(axis)
	var rotated_inverse_inertia : Basis = vdata.veh_ref.global_basis \
	* vdata.inverse_inertia * vdata.veh_ref.global_basis.transposed()
	var e : float = 1.0 / vdata.veh_ref.mass + cr.dot(rotated_inverse_inertia * cr)
	return e

func start()->void:
	tm = TireModel.new()
	effective_mass = 1.0 / vdata.veh_ref.mass
	var off : Vector3 = (global_position - vdata.com.global_position)
	effective_mass += off.cross(global_basis.y).dot(vdata.inverse_inertia * off.cross(global_basis.y))
	effective_mass = 1.0 / effective_mass
	
	var freq_transformed  : float = vdata.suspension_data.frequency * 2 * PI
	stiffness = effective_mass * freq_transformed * \
				freq_transformed
	damping = 2 * vdata.suspension_data.damping_ratio * \
			  effective_mass * freq_transformed
	
	if ray:
		ray.queue_free()
	ray = RayCast3D.new()
	ray.enabled=false
	ray.hit_back_faces = true
	ray.hit_from_inside = true
	ray.add_exception(vdata.veh_ref)
	ray.target_position.y = -vdata.suspension_data.length
	add_child(ray)
	if wheel_mesh:
		wheel_mesh.queue_free()
	wheel_mesh = vdata.tire_data.mesh_scene.instantiate()
	add_child(wheel_mesh)
	

func update_suspension(delta : float, new_state : WheelState)->void:
	ray.force_raycast_update()
	if ray.is_colliding()==false:
		new_state.on_ground = false
		return
	new_state.on_ground = true
	new_state.collision_point = ray.get_collision_point()
	new_state.compression = 1.0 - ray.global_position.distance_to(new_state.collision_point)\
	/ vdata.suspension_data.length
	
	new_state.compression = clampf(new_state.compression, 0.0, 1.0)
	new_state.collision_normal = ray.get_collision_normal()
	
	var speed : float = (new_state.compression - current_wheel_state.compression) / delta
	var nominator : float = -delta * (stiffness * new_state.compression + damping * speed)
	var denominator : float = 1 + delta * (damping + stiffness * delta) / effective_mass
	var new_rebound : float = -nominator / denominator
	new_state.rebound_impulse = new_rebound
	new_state.sustained_mass = maxf(new_state.rebound_impulse / delta / 9.8 , 0.0)


func estimate_applied_torque(delta : float, cluch_torque : float)->TireState:
	var t : TireState = TireState.new()
	
	
	var brake_torque : float = 0.0
	if vdata.brakes_data:
		brake_torque = vdata.veh_ref.brake_input * vdata.brakes_data.brake_torque
	
	var ang_vel : float = current_wheel_state.angular_velocity
	
	var slip_force_forward : float = 0
	var slip_force_lateral : float = 0
	
	var tiref : Dictionary = tm.get_tire_forces(self, current_tire_state, delta)
	slip_force_forward = tiref['fwd']
	slip_force_lateral = tiref['lat']
	t.slip_forward = tiref['slip_fwd']
	t.slip_lateral = tiref['slip_lat']
	
	if vdata.tcs:
		cluch_torque = vdata.tcs.clamp_torque(cluch_torque, t.slip_forward)
	if vdata.abs_system:
		brake_torque = vdata.abs_system.clamp_brake_torque(brake_torque, t.slip_forward)
	
	var tire_torque : float = slip_force_forward * vdata.tire_data.radius 
	
	brake_torque *= vdata.heat_system.get_brakes_efficiency()
	
	var tire_damping : float = vdata.tire_data.radius * \
	current_wheel_state.sustained_mass * 9.8 * 0.025 * -t.slip_forward
	
	var applied_torque : float = cluch_torque - tire_torque - 0.3 * ang_vel + tire_damping
	var tire_ang_accel : float = applied_torque * delta / inertia
	
	var new_angular : float = ang_vel + tire_ang_accel
	
	brake_torque = clampf(brake_torque, -abs(new_angular) * inertia / delta,
	abs(new_angular) * inertia / delta)
	brake_torque *= -signf(new_angular)
	applied_torque += brake_torque 
	
	t.applied_brake_torque = brake_torque
	t.applied_torque = applied_torque
	t.lateral_force = slip_force_lateral
	t.longitudinal_force = slip_force_forward
	t.algining_force = tiref['align']
	
	return t

func estimate_load_torque(delta : float)->float:
	if !current_wheel_state.on_ground:
		return 0.0
	var tiref : Dictionary = tm.get_tire_forces(self, current_tire_state, delta)
	var slip_force_forward : float = tiref['fwd']
	var tire_torque : float = slip_force_forward * vdata.tire_data.radius 
	return tire_torque

func update_physics(delta : float, clutch_torque : float)->void:
	
	var new_wheel_state : WheelState = WheelState.new()
	update_suspension(delta, new_wheel_state)
	new_wheel_state.velocity = (global_position-last_position)/delta
	last_position = global_position
	
	
	var new_tire_state : TireState = estimate_applied_torque(delta, clutch_torque)
	
	new_wheel_state.angular_velocity = current_wheel_state.angular_velocity
	new_wheel_state.angular_velocity += new_tire_state.applied_torque * delta / inertia
	if !current_wheel_state.on_ground:
		new_tire_state.lateral_force = 0.0
		new_tire_state.longitudinal_force = 0.0
	
	update_steer(delta, new_wheel_state)
	
	current_tire_state = new_tire_state
	current_wheel_state = new_wheel_state

func update_steer(delta : float, new_wh : WheelState)->void:
	if !steerable:
		return
	
	var max_steer_velocity : float = deg_to_rad(25)
	
	#print(rad_to_deg(turn_target))
	
	var input_vel : float = turn_target / deg_to_rad(max_steer_angle_default) * max_steer_velocity
	var target_vel_from_angle : float = (turn_target - turn) * 2.0
	var target_steer_vel : float = input_vel + target_vel_from_angle
	target_steer_vel = clampf(target_vel_from_angle, -max_steer_velocity, max_steer_velocity)
	
	var inp_tq : float = steer_stiffness * (target_steer_vel  - steer_velocity)
	var sat_torque : float = 0.0
	
	if current_tire_state:
		sat_torque = current_tire_state.algining_force * vdata.tire_data.radius / 12.0
		#print(current_tire_state.algining_force)
		var sat_slip_factor  : float = exp(-vdata.tire_data.sat_slip_loss * absf(current_tire_state.slip_lateral))
		sat_torque *= sat_slip_factor
	if current_wheel_state.velocity.length() < 2.5:
		sat_torque = lerpf(0.0,sat_torque, current_wheel_state.velocity.length())
	
	sat_steer_torque = clampf(sat_torque, -absf(inp_tq) * 0.7, absf(inp_tq) * 0.7 )
	var total_torque : float = inp_tq + sat_torque
	sat_steer_torque = sat_torque
	
	new_wh.steer_velocity = current_wheel_state.steer_velocity
	new_wh.steer_velocity += (total_torque - new_wh.steer_velocity * steer_damping) * delta
	turn += new_wh.steer_velocity * delta
	turn = clampf(turn, -max_steer_angle, max_steer_angle)
	if absf(turn) < 0.00001:
		turn = 0.0
	rotation.y = turn

var sat_steer_torque: float 

func update_visuals(delta : float)->void:
	var mesh_tg_y : float = (1.0 - current_wheel_state.compression) * \
	vdata.suspension_data.length * -1 + vdata.tire_data.radius - 0.05
	
	if !current_wheel_state.on_ground:
		wheel_mesh.position.y = move_toward(wheel_mesh.position.y, mesh_tg_y, 9.8 * delta)
	else:
		wheel_mesh.position.y = move_toward(wheel_mesh.position.y, mesh_tg_y, 9.8 * delta * 2.0)
	
	var added_ang_vel : float = clampf(current_wheel_state.angular_velocity,
								-45.0, 45.0)
	
	wheel_mesh.rotation.x += added_ang_vel * delta


func _process(delta: float) -> void:
	update_visuals(delta)
