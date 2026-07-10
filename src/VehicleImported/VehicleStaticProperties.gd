class_name VehicleStaticProperties
extends Resource

var inverse_inertia : Basis
var tire_data : TireData
var suspension_data : SuspensionData
var motor_data : MotorData
var gearbox_data : GearboxData
var flywheel_data : FlywheelData
var clutch_data : ClutchData
var turbo_data : TurboData
var brakes_data : BrakesData
var radiator_data : RadiatorData

# --- references -- 

var veh_ref : Vehicle
var com : Node3D
var axles : Array[Axle]
var wheels : Array[Wheel]
var controller : VehicleController
var aero : Aerodyanmics
var tcs : TCS
var abs_system : ABS
var motor : Motor
var clutch : Clutch
var gearbox : Gearbox
var turbo : Turbo
var heat_system : HeatSystem


func set_params(veh : Vehicle)->void:
	inverse_inertia = PhysicsServer3D.body_get_direct_state(veh.get_rid()).inverse_inertia_tensor
	tire_data = veh.tire_data
	suspension_data = veh.suspension_data
	gearbox_data = veh.gearbox_data
	flywheel_data = veh.flywheel_data
	clutch_data = veh.clutch_data
	turbo_data = veh.turbo_data
	brakes_data = veh.brakes_data
	veh_ref = veh
	com = veh.com
	motor_data = veh.motor_data
	radiator_data = veh.radiator_data
