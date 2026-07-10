class_name Motor
extends Node

var vdata : VehicleStaticProperties
var motor_data : MotorData
var torque_curve : MotorData.CurveData

var inertia : float
var default_inertia : float
var idle_rpm : float = 800.0
var redline_rpm : float = 4000.0
var max_torque : float = 0.0

const ANG_TO_RPM : float = 60 / (2 * PI)
const RPM_TO_ANG : float = 2 * PI / 60

var horsepower : float:
	get():
		return get_engine_torque(motor_state) * rpm / 5252

var motor_state : MotorState

var rpm : float:
	get():
		return motor_state.angular_velocity * ANG_TO_RPM

func start()->void:
	motor_data = vdata.motor_data
	inertia = motor_data.get_inertia()
	inertia += vdata.flywheel_data.get_inertia()
	default_inertia = inertia
	redline_rpm = motor_data.get_redline_rpm()
	torque_curve = motor_data.CurveData.new(motor_data)
	max_torque = motor_data.get_max_torque()
	
	
	motor_state = MotorState.new()
	motor_state.angular_velocity = RPM_TO_ANG * idle_rpm
	motor_state.rpm = idle_rpm

func estimate_applied_torque(state : MotorState, load_torque : float = 0.0)->float:
	var ebrake_torque : float = -0.015 * (state.angular_velocity - idle_rpm * RPM_TO_ANG)
	if vdata.veh_ref.clutch_input > 0.9 or vdata.gearbox.gear == 1:
		inertia = default_inertia * 0.2
	else:
		inertia = default_inertia
	
	#print(idle_rpm * RPM_TO_ANG, ' ', state.angular_velocity)
	
	var engine_torque : float = get_engine_torque(state)
	
	var stall_torque : float = 0.0
	if state.rpm < idle_rpm:
		var dist : float = idle_rpm - state.rpm
		if dist > 30:
			dist = 30
		if state.rpm < idle_rpm - 150:
			state.rpm = idle_rpm - 150
		
		var clamped_rpm : float = clampf(state.rpm, idle_rpm, redline_rpm)
		
		## will cause problems with turbo
		var new_eng_tq : float = motor_data.torque_curve(torque_curve,clamped_rpm) * \
						max_torque * (dist + 10) / 40.0
		
		engine_torque = maxf(engine_torque, new_eng_tq)
		
	
	#if state.rpm > redline_rpm - 50:
	#	state.rpm -= 300
		#var limiter : float = smoothstep(redline_rpm - 350, redline_rpm, state.rpm)
		#engine_torque *= (1 - limiter)
	
	var total_torque : float = ebrake_torque + stall_torque + \
							   load_torque + engine_torque
	
	return total_torque

func get_engine_torque(state : MotorState)->float:
	var actual_rpm : float = state.rpm
	actual_rpm = clampf(actual_rpm, idle_rpm, redline_rpm)
	var boost : float = vdata.turbo.boost
	
	#print(motor_data.torque_curve(torque_curve,state.rpm))
	return vdata.veh_ref.throttle_input * \
		motor_data.torque_curve(torque_curve,actual_rpm) * \
		max_torque * (1.0 + boost * 0.85) * vdata.heat_system.get_motor_efficiency()

func update_physics(delta : float, load_torque : float)->void:
	var new_state : MotorState = MotorState.new()
	new_state.angular_velocity = 0.0
	if motor_state:
		new_state.angular_velocity = motor_state.angular_velocity
		new_state.rpm = motor_state.rpm
	var torque : float = estimate_applied_torque(new_state, load_torque)
	var accel : float = torque / inertia
	new_state.angular_velocity += accel * delta
	new_state.rpm = new_state.angular_velocity * ANG_TO_RPM
	motor_state = new_state
	if motor_state.rpm < 820.0:
		vdata.veh_ref.clutch_input = 1.0
	elif motor_state.rpm > redline_rpm:
		motor_state.angular_velocity -= 450 * RPM_TO_ANG
	
	motor_state.rpm_factor = remap(clampf(motor_state.rpm,idle_rpm,redline_rpm),
									idle_rpm, redline_rpm, 
									0.0, 1.0)
	
