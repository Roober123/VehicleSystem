#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "example_class.h"
#include "vehicle.h"
#include "axle.h"
#include "Resources/suspension_data.h"
#include "Resources/vehicle_engine_data.h"
#include "Resources/gearbox_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/tire_data.h"
#include "Resources/steering_rack_data.h"

#include "wheel.h"
#include "VehicleTelemetry.h"
#include "TireSkid.h"
#include "Resources/turbo_data.h"



using namespace godot;

void initialize_gdextension_types(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(TurboData);
	GDREGISTER_CLASS(VehicleAerodynamicsData);
	GDREGISTER_CLASS(SuspensionData);
	GDREGISTER_CLASS(VehicleEngineData);
	GDREGISTER_CLASS(GearboxData);
	GDREGISTER_CLASS(TireData);
	GDREGISTER_CLASS(SteeringRackData);
	GDREGISTER_CLASS(Wheel);
	GDREGISTER_CLASS(Axle);
	GDREGISTER_CLASS(Vehicle);
	GDREGISTER_CLASS(VehicleTelemetry);
	GDREGISTER_CLASS(TireSkid);
	
	

}

void uninitialize_gdextension_types(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C"
{
	// Initialization
	GDExtensionBool GDE_EXPORT init_vehicle_system(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
	{
		GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
		init_obj.register_initializer(initialize_gdextension_types);
		init_obj.register_terminator(uninitialize_gdextension_types);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

		return init_obj.init();
	}
}