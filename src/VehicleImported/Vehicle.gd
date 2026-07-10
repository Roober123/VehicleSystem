class_name Vehicle
extends RigidBody3D

var inventory : CarInventory

@export var tire_data : TireData
@export var suspension_data : SuspensionData
@export var motor_data : MotorData
@export var flywheel_data : FlywheelData
@export var gearbox_data : GearboxData
@export var clutch_data : ClutchData
@export var turbo_data : TurboData
@export var brakes_data : BrakesData
@export var radiator_data : RadiatorData

@export var com : Node3D

var vdata : VehicleStaticProperties

var axles : Array[Axle]

var throttle_input : float
var brake_input : float
var steer_input : float
var shift_up : bool
var shift_dn : bool
var clutch_input : float

func load_inventory()->void:
	if !inventory:
		return
	tire_data = inventory.tire
	suspension_data = inventory.suspension
	motor_data = inventory.motor
	flywheel_data = inventory.flywheel
	gearbox_data = inventory.gearbox
	clutch_data = inventory.clutch
	turbo_data = inventory.turbo
	brakes_data = inventory.brakes
	radiator_data = inventory.radiator

func start()->void:
	load_inventory()
	vdata =  VehicleStaticProperties.new()
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = com.position
	vdata.set_params(self)
	for i in get_children():
		if i is Axle:
			axles.push_back(i)
			i.vdata = vdata
		if i is VehicleController:
			vdata.controller = i
			i.vdata = vdata
		if i is Aerodyanmics:
			vdata.aero = i
			i.vdata = vdata
		if i is TCS:
			vdata.tcs = i
		if i is ABS:
			vdata.abs_system = i
		if i is Motor:
			vdata.motor = i
			i.vdata = vdata
			i.start()
		if i is Clutch:
			vdata.clutch = i
			i.vdata = vdata
			i.start()
		if i is Gearbox:
			vdata.gearbox = i
			i.vdata = vdata
		if i is Turbo:
			i.vdata = vdata
			vdata.turbo = i
		if i is HeatSystem:
			i.vdata = vdata
			vdata.heat_system = i

	
	mass += vdata.motor_data.mass
	mass += vdata.gearbox_data.mass
	mass += vdata.flywheel_data.mass
	
	for i : Axle in axles:
		i.start()
	vdata.heat_system.start()

func _ready() -> void:
	start()

func _physics_process(delta: float) -> void:
	if vdata.aero:
		vdata.aero.update(delta)
		vdata.aero.apply_forces()
	
	
	vdata.gearbox.update_physics(delta)
	vdata.clutch.compute_clutch_torque(delta)
	var clutch_torque : float = vdata.clutch.clutch_torque
	
	vdata.motor.update_physics(delta, clutch_torque * 1)
	
	if vdata.turbo_data:
		vdata.turbo.update_physics(delta)
	
	for i : Axle in axles:
		i.update_steering(delta)
		i.update_physics(delta, -clutch_torque * vdata.gearbox.real_ratio * \
						vdata.gearbox_data.final_drive) # * gear_sign
		i.apply_forces(delta)
	
	vdata.heat_system.update_physics(delta)
