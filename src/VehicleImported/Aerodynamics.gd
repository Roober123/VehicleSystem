class_name Aerodyanmics
extends Node3D

var vdata : VehicleStaticProperties
@export var front : Node3D
@export var rear : Node3D
@export_range(0.0,1.0,0.05) var front_ratio : float = 0.5

@export var drag_coefficient : float = 0.25
@export var downforce_multiplier : float = 4.0
@export var yaw_fix : float = 2.0

var downforce_coefficient : float:
	get():
		return drag_coefficient * downforce_multiplier

var drag_force : float
var down_force : float
var yaw_torque : Vector3



## in physics process
func update(_dt : float)->void:
	var sp : float = vdata.veh_ref.linear_velocity.length()
	var sp_2 : float = sp * sp
	
	drag_force = drag_coefficient * sp_2
	down_force = downforce_coefficient * sp_2


func apply_forces()->void:
	var v : Vehicle = vdata.veh_ref
	v.apply_central_force(-v.linear_velocity.normalized() * drag_force)
	var off_front : Vector3 = front.global_position - v.global_position  
	var off_rear : Vector3 =  rear.global_position - v.global_position 
	
	v.apply_force(down_force * Vector3.DOWN * front_ratio,off_front)
	v.apply_force(down_force * Vector3.DOWN * (1.0-front_ratio),off_rear)
	
	var yaw_damping : Vector3 = Vector3.UP * v.angular_velocity * \
	v.angular_velocity.length() 
	
	yaw_torque= -v.mass * yaw_damping 
	v.apply_torque(yaw_torque * yaw_fix)
	
	
	
