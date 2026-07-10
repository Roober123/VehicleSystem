class_name Axle
extends Node3D

var vdata : VehicleStaticProperties

var wheels : Array[Wheel]

@export var can_steer : bool
@export_range(15,40) var max_turn_angle : float = 27.0 
@export_range(0.0,1.0) var drive_ratio : float
@export var is_rear : bool = false
var turn : float

var to_update_ackermann : bool = true
var wheelbase : float = 0.0
var trackwidth : float = 0.0

var average_angular_velocity : float


func start()->void:
	wheels.clear()
	vdata.axles.push_back(self)
	for i in get_children():
		if i is Wheel:
			i.vdata = vdata
			i.start()
			wheels.push_back(i)
			vdata.wheels.push_back(i)
			i.is_rear = is_rear
			i.steerable = can_steer
			i.max_steer_angle = deg_to_rad(max_turn_angle)
			i.max_steer_angle_default = max_turn_angle
			
func update_physics(delta : float, engine_power : float)->void:
	var added_torque : float = engine_power * drive_ratio * 0.5
	average_angular_velocity = 0.0
	for i : Wheel in wheels:
		i.update_physics(delta,added_torque)
		average_angular_velocity += i.current_wheel_state.angular_velocity
	average_angular_velocity /= wheels.size()


var last_pos : Vector3
func update_steering(delta : float)->void:
	if !can_steer:
		return
	var speed : float = (global_position-last_pos).length()/delta * 3.6
	speed = absf(speed) 
	last_pos = global_position
	var max_dynamic_turn_angle : float = max_turn_angle / (1 +  speed * 0.02)
	
	
	if to_update_ackermann:
		to_update_ackermann = false
		trackwidth = wheels[0].global_position.distance_to(wheels[1].global_position)
		for i : Axle in vdata.veh_ref.axles:
			for j : Axle in vdata.veh_ref.axles:
				wheelbase = max(wheelbase, i.global_position.distance_to(j.global_position))
	
	turn = deg_to_rad(max_dynamic_turn_angle) * -vdata.veh_ref.steer_input
	
	var wh_turn : Array[float] = get_ackermann(turn)
	
	for ind : int in wheels.size():
		var i : Wheel = wheels[ind]
		i.turn_target = wh_turn[ind]
		i.max_steer_angle = deg_to_rad(max_dynamic_turn_angle)
		

func get_ackermann(new_turn : float)->Array[float]:
	var sgn : float = signf(new_turn)
	new_turn = absf(new_turn)
	
	var radius : float = wheelbase / tan(new_turn)
	
	var inner_radius : float = radius - trackwidth / 2.0
	var outer_radius : float = radius + trackwidth / 2.0
	
	var inner_angle : float = atan2(wheelbase,inner_radius)
	var outer_angle : float = atan2(wheelbase,outer_radius)
	
	if sgn < 0:
		return [-inner_angle,-outer_angle]
	return [outer_angle,inner_angle]




func apply_forces(_delta : float)->void:
	for i : Wheel in wheels:
		var offset : Vector3 = i.global_position - vdata.veh_ref.global_position
		vdata.veh_ref.apply_impulse(i.current_wheel_state.rebound_impulse * \
		i.current_wheel_state.collision_normal, offset)
		vdata.veh_ref.apply_force(i.global_basis.z * \
		i.current_tire_state.longitudinal_force, offset)
		vdata.veh_ref.apply_force(-i.global_basis.x * \
		i.current_tire_state.lateral_force, offset)
		
		## self aligning force
		var rear_coeff : float = (1.0 - int(is_rear))
		#if rear_coeff == 0.0:
		#	rear_coeff = 0.1
		var wheel_front : Vector3 = offset + i.global_basis.z * vdata.tire_data.radius
		vdata.veh_ref.apply_force(i.current_tire_state.algining_force * \
								  i.global_basis.x * rear_coeff, offset)
		vdata.veh_ref.apply_force(-i.current_tire_state.algining_force * \
								  i.global_basis.x * rear_coeff, wheel_front)
		## align torque
		
