#include "differential_data.h"
#include <algorithm>
#include <cmath>

namespace godot {

void DifferentialData::_bind_methods() {
    BIND_ENUM_CONSTANT(OPEN);
    BIND_ENUM_CONSTANT(LIMITED_SLIP);
    BIND_ENUM_CONSTANT(LOCKED);
    ClassDB::bind_method(D_METHOD("set_mode", "value"), &DifferentialData::set_mode);
    ClassDB::bind_method(D_METHOD("get_mode"), &DifferentialData::get_mode);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "mode", PROPERTY_HINT_ENUM, "Open,Limited Slip,Locked"), "set_mode", "get_mode");
    ADD_GROUP("Limited Slip", "");
    ClassDB::bind_method(D_METHOD("set_acceleration_lock_percent", "value"), &DifferentialData::set_acceleration_lock_percent);
    ClassDB::bind_method(D_METHOD("get_acceleration_lock_percent"), &DifferentialData::get_acceleration_lock_percent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "acceleration_lock_percent", PROPERTY_HINT_RANGE, "0,100,1,suffix:%"), "set_acceleration_lock_percent", "get_acceleration_lock_percent");
    ClassDB::bind_method(D_METHOD("set_engine_braking_lock_percent", "value"), &DifferentialData::set_engine_braking_lock_percent);
    ClassDB::bind_method(D_METHOD("get_engine_braking_lock_percent"), &DifferentialData::get_engine_braking_lock_percent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "engine_braking_lock_percent", PROPERTY_HINT_RANGE, "0,100,1,suffix:%"), "set_engine_braking_lock_percent", "get_engine_braking_lock_percent");
    ClassDB::bind_method(D_METHOD("set_preload_torque", "value"), &DifferentialData::set_preload_torque);
    ClassDB::bind_method(D_METHOD("get_preload_torque"), &DifferentialData::get_preload_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "preload_torque", PROPERTY_HINT_RANGE, "0,500,1,or_greater,suffix:Nm"), "set_preload_torque", "get_preload_torque");
    ADD_GROUP("Advanced", "");
    ClassDB::bind_method(D_METHOD("set_max_lock_torque", "value"), &DifferentialData::set_max_lock_torque);
    ClassDB::bind_method(D_METHOD("get_max_lock_torque"), &DifferentialData::get_max_lock_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_lock_torque", PROPERTY_HINT_RANGE, "0,500,1,or_greater,suffix:Nm"), "set_max_lock_torque", "get_max_lock_torque");
    ClassDB::bind_method(D_METHOD("set_speed_lock_torque_per_100_rpm", "value"), &DifferentialData::set_speed_lock_torque_per_100_rpm);
    ClassDB::bind_method(D_METHOD("get_speed_lock_torque_per_100_rpm"), &DifferentialData::get_speed_lock_torque_per_100_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed_lock_torque_per_100_rpm", PROPERTY_HINT_RANGE, "0,100,0.1,or_greater,suffix:Nm/100 RPM"), "set_speed_lock_torque_per_100_rpm", "get_speed_lock_torque_per_100_rpm");
}

void DifferentialData::set_mode(Mode value) {
    mode = value >= OPEN && value <= LOCKED ? value : OPEN;
    notify_property_list_changed();
}

void DifferentialData::_validate_property(PropertyInfo &property) const {
    const String name = property.name;
    if (mode != LIMITED_SLIP && (name == "acceleration_lock_percent" ||
            name == "engine_braking_lock_percent" || name == "preload_torque" ||
            name == "max_lock_torque" || name == "speed_lock_torque_per_100_rpm")) {
        // Keep storage: switching modes must not discard the user's LSD tune.
        property.usage &= ~PROPERTY_USAGE_EDITOR;
    }
}

void DifferentialData::set_acceleration_lock_percent(real_t value) {
    acceleration_lock_percent = std::isfinite(value) ? std::clamp(value, real_t{0.0}, real_t{100}) : real_t{70.0};
}
void DifferentialData::set_engine_braking_lock_percent(real_t value) {
    engine_braking_lock_percent = std::isfinite(value) ? std::clamp(value, real_t{0.0}, real_t{100}) : real_t{30.0};
}
void DifferentialData::set_preload_torque(real_t value) {
    preload_torque = std::isfinite(value) ? std::clamp(value, real_t{0.0}, real_t{1000000}) : real_t{25.0};
}
void DifferentialData::set_max_lock_torque(real_t value) {
    max_lock_torque = std::isfinite(value) ? std::clamp(value, real_t{0.0}, real_t{1000000}) : real_t{250.0};
}
void DifferentialData::set_speed_lock_torque_per_100_rpm(real_t value) {
    speed_lock_torque_per_100_rpm = std::isfinite(value) ? std::clamp(value, real_t{0.0}, real_t{1000000}) : real_t{20.943951024};
}

real_t DifferentialData::get_slip_sensitive_gain() const {
    constexpr real_t hundred_rpm = real_t{100.0} * real_t{2.0} * Math_PI / real_t{60.0};
    return speed_lock_torque_per_100_rpm / hundred_rpm;
}

} // namespace godot
