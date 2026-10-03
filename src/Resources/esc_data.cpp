#include "esc_data.h"

#include <algorithm>
#include <cmath>

namespace godot {

void ESCData::_bind_methods() {
    ADD_GROUP("Recovery Smoothness", "");
    ClassDB::bind_method(D_METHOD("set_torque_smoothing_enabled", "value"), &ESCData::set_torque_smoothing_enabled);
    ClassDB::bind_method(D_METHOD("get_torque_smoothing_enabled"), &ESCData::get_torque_smoothing_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "torque_smoothing_enabled"), "set_torque_smoothing_enabled", "get_torque_smoothing_enabled");
    ClassDB::bind_method(D_METHOD("set_torque_engagement_rate", "value"), &ESCData::set_torque_engagement_rate);
    ClassDB::bind_method(D_METHOD("get_torque_engagement_rate"), &ESCData::get_torque_engagement_rate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque_engagement_rate", PROPERTY_HINT_RANGE, "1,1000000,100,or_greater,suffix:Nm/s"), "set_torque_engagement_rate", "get_torque_engagement_rate");
    ClassDB::bind_method(D_METHOD("set_torque_release_rate", "value"), &ESCData::set_torque_release_rate);
    ClassDB::bind_method(D_METHOD("get_torque_release_rate"), &ESCData::get_torque_release_rate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "torque_release_rate", PROPERTY_HINT_RANGE, "1,1000000,100,or_greater,suffix:Nm/s"), "set_torque_release_rate", "get_torque_release_rate");

    ADD_GROUP("Yaw Control", "");
    ClassDB::bind_method(D_METHOD("set_enabled", "value"), &ESCData::set_enabled);
    ClassDB::bind_method(D_METHOD("get_enabled"), &ESCData::get_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "get_enabled");

    ClassDB::bind_method(D_METHOD("set_yaw_damping", "value"), &ESCData::set_yaw_damping);
    ClassDB::bind_method(D_METHOD("get_yaw_damping"), &ESCData::get_yaw_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "yaw_damping", PROPERTY_HINT_RANGE, "0.0,3.0,0.1,or_greater"), "set_yaw_damping", "get_yaw_damping");

    ClassDB::bind_method(D_METHOD("set_minimum_speed", "value"), &ESCData::set_minimum_speed);
    ClassDB::bind_method(D_METHOD("get_minimum_speed"), &ESCData::get_minimum_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "minimum_speed", PROPERTY_HINT_RANGE, "0.0,20.0,0.1,or_greater,suffix:m/s"), "set_minimum_speed", "get_minimum_speed");

    ClassDB::bind_method(D_METHOD("set_maximum_corrective_torque", "value"), &ESCData::set_maximum_corrective_torque);
    ClassDB::bind_method(D_METHOD("get_maximum_corrective_torque"), &ESCData::get_maximum_corrective_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "maximum_corrective_torque", PROPERTY_HINT_RANGE, "0.0,20000.0,100.0,or_greater,suffix:Nm"), "set_maximum_corrective_torque", "get_maximum_corrective_torque");

    ClassDB::bind_method(D_METHOD("set_maximum_target_lateral_acceleration", "value"), &ESCData::set_maximum_target_lateral_acceleration);
    ClassDB::bind_method(D_METHOD("get_maximum_target_lateral_acceleration"), &ESCData::get_maximum_target_lateral_acceleration);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "maximum_target_lateral_acceleration", PROPERTY_HINT_RANGE, "0.1,30.0,0.1,or_greater,suffix:m/s²"), "set_maximum_target_lateral_acceleration", "get_maximum_target_lateral_acceleration");
}

void ESCData::set_yaw_damping(real_t value) {
    if (std::isfinite(value)) yaw_damping = std::max(value, real_t{0.0});
}

void ESCData::set_minimum_speed(real_t value) {
    if (std::isfinite(value)) minimum_speed = std::max(value, real_t{0.0});
}

void ESCData::set_maximum_corrective_torque(real_t value) {
    if (std::isfinite(value)) maximum_corrective_torque = std::max(value, real_t{0.0});
}

void ESCData::set_maximum_target_lateral_acceleration(real_t value) {
    if (std::isfinite(value)) maximum_target_lateral_acceleration = std::max(value, real_t{0.1});
}


void ESCData::set_torque_smoothing_enabled(bool value) { torque_smoothing_enabled = value; }

void ESCData::set_torque_engagement_rate(real_t value) { if (std::isfinite(value) && value > real_t{0.0}) torque_engagement_rate = value; }

void ESCData::set_torque_release_rate(real_t value) { if (std::isfinite(value) && value > real_t{0.0}) torque_release_rate = value; }

} // namespace godot
