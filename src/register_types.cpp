#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "vehicle.h"
#include "axle.h"
#include "Resources/suspension_data.h"
#include "Resources/vehicle_engine_data.h"
#include "Resources/gearbox_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/tire_data.h"
#include "Resources/steering_rack_data.h"
#include "Resources/differential_data.h"

#include "wheel.h"
#include "VehicleTelemetry.h"
#include "TireSkid.h"
#include "Resources/turbo_data.h"
#include "Resources/vehicle_config.h"
#include "Resources/vehicle_audio_data.h"
#include "VehicleAudio.h"

#ifdef VEHICLE_SYSTEM_REGRESSION_TESTS
#include "../Test/drivetrain_regression.h"
#include "../Test/audio_regression.h"
#endif


using namespace godot;

void initialize_gdextension_types(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(TurboData);
	GDREGISTER_CLASS(DifferentialData);
	GDREGISTER_CLASS(VehicleConfig);
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
	GDREGISTER_CLASS(VehicleAudioNodeData);
	GDREGISTER_CLASS(VehicleAudioGuideData);
	GDREGISTER_CLASS(VehicleAudioTopologyData);
	GDREGISTER_CLASS(VehicleAudioFiringData);
	GDREGISTER_CLASS(VehicleAudio);
	#ifdef VEHICLE_SYSTEM_REGRESSION_TESTS
	GDREGISTER_CLASS(DrivetrainRegression);
	GDREGISTER_CLASS(AudioRegression);
	#endif
	
	

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
