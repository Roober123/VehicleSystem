#include "differential_data.h"

namespace godot {

void DifferentialData::_bind_methods() {
    BIND_ENUM_CONSTANT(OPEN);
    BIND_ENUM_CONSTANT(LIMITED_SLIP);
    BIND_ENUM_CONSTANT(LOCKED);

    ClassDB::bind_method(D_METHOD("set_mode", "mode"), &DifferentialData::set_mode);
    ClassDB::bind_method(D_METHOD("get_mode"), &DifferentialData::get_mode);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "mode", PROPERTY_HINT_ENUM, "Open,Limited Slip,Locked"),
                 "set_mode", "get_mode");

    ClassDB::bind_method(D_METHOD("set_preload_torque", "value"), &DifferentialData::set_preload_torque);
    ClassDB::bind_method(D_METHOD("get_preload_torque"), &DifferentialData::get_preload_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "preload_torque", PROPERTY_HINT_RANGE, "0.0,5000.0,1.0"),
                 "set_preload_torque", "get_preload_torque");

    ClassDB::bind_method(D_METHOD("set_power_lock_ratio", "value"), &DifferentialData::set_power_lock_ratio);
    ClassDB::bind_method(D_METHOD("get_power_lock_ratio"), &DifferentialData::get_power_lock_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "power_lock_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"),
                 "set_power_lock_ratio", "get_power_lock_ratio");

    ClassDB::bind_method(D_METHOD("set_coast_lock_ratio", "value"), &DifferentialData::set_coast_lock_ratio);
    ClassDB::bind_method(D_METHOD("get_coast_lock_ratio"), &DifferentialData::get_coast_lock_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "coast_lock_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"),
                 "set_coast_lock_ratio", "get_coast_lock_ratio");

    ClassDB::bind_method(D_METHOD("set_slip_sensitive_gain", "value"), &DifferentialData::set_slip_sensitive_gain);
    ClassDB::bind_method(D_METHOD("get_slip_sensitive_gain"), &DifferentialData::get_slip_sensitive_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slip_sensitive_gain", PROPERTY_HINT_RANGE, "0.0,100.0,0.1"),
                 "set_slip_sensitive_gain", "get_slip_sensitive_gain");

    ClassDB::bind_method(D_METHOD("set_max_lock_torque", "value"), &DifferentialData::set_max_lock_torque);
    ClassDB::bind_method(D_METHOD("get_max_lock_torque"), &DifferentialData::get_max_lock_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_lock_torque", PROPERTY_HINT_RANGE, "0.0,5000.0,1.0"),
                 "set_max_lock_torque", "get_max_lock_torque");
}

} // namespace godot
