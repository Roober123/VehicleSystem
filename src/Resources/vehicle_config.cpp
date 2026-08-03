#include "vehicle_config.h"

namespace godot {

void VehicleConfig::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_engine_data", "data"), &VehicleConfig::set_engine_data);
    ClassDB::bind_method(D_METHOD("get_engine_data"), &VehicleConfig::get_engine_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine_data", PROPERTY_HINT_RESOURCE_TYPE, "VehicleEngineData"),
                 "set_engine_data", "get_engine_data");

    ClassDB::bind_method(D_METHOD("set_gearbox_data", "data"), &VehicleConfig::set_gearbox_data);
    ClassDB::bind_method(D_METHOD("get_gearbox_data"), &VehicleConfig::get_gearbox_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gearbox_data", PROPERTY_HINT_RESOURCE_TYPE, "GearboxData"),
                 "set_gearbox_data", "get_gearbox_data");

    ClassDB::bind_method(D_METHOD("set_suspension_data", "data"), &VehicleConfig::set_suspension_data);
    ClassDB::bind_method(D_METHOD("get_suspension_data"), &VehicleConfig::get_suspension_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "suspension_data", PROPERTY_HINT_RESOURCE_TYPE, "SuspensionData"),
                 "set_suspension_data", "get_suspension_data");

    ClassDB::bind_method(D_METHOD("set_aero_data", "data"), &VehicleConfig::set_aero_data);
    ClassDB::bind_method(D_METHOD("get_aero_data"), &VehicleConfig::get_aero_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "aero_data", PROPERTY_HINT_RESOURCE_TYPE, "VehicleAerodynamicsData"),
                 "set_aero_data", "get_aero_data");

    ClassDB::bind_method(D_METHOD("set_turbo_data", "data"), &VehicleConfig::set_turbo_data);
    ClassDB::bind_method(D_METHOD("get_turbo_data"), &VehicleConfig::get_turbo_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "turbo_data", PROPERTY_HINT_RESOURCE_TYPE, "TurboData"),
                 "set_turbo_data", "get_turbo_data");

    ClassDB::bind_method(D_METHOD("set_center_differential_data", "data"), &VehicleConfig::set_center_differential_data);
    ClassDB::bind_method(D_METHOD("get_center_differential_data"), &VehicleConfig::get_center_differential_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "center_differential_data", PROPERTY_HINT_RESOURCE_TYPE, "DifferentialData"),
                 "set_center_differential_data", "get_center_differential_data");
}

} // namespace godot
