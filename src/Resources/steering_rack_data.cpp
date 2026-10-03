#include "steering_rack_data.h"
#include <algorithm>
#include <cmath>

namespace godot {

void SteeringRackData::_bind_methods() {
    ADD_GROUP("Steering Response", "");
    ClassDB::bind_method(D_METHOD("set_max_angle", "value"), &SteeringRackData::set_max_angle);
    ClassDB::bind_method(D_METHOD("get_max_angle"), &SteeringRackData::get_max_angle);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_angle", PROPERTY_HINT_RANGE, "0,50,0.1,degrees"), "set_max_angle", "get_max_angle");
    ClassDB::bind_method(D_METHOD("set_response_time_ms", "value"), &SteeringRackData::set_response_time_ms);
    ClassDB::bind_method(D_METHOD("get_response_time_ms"), &SteeringRackData::get_response_time_ms);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "response_time_ms", PROPERTY_HINT_RANGE, "10,1000,1,or_greater,suffix:ms"), "set_response_time_ms", "get_response_time_ms");
    ClassDB::bind_method(D_METHOD("set_steering_half_speed_kph", "value"), &SteeringRackData::set_steering_half_speed_kph);
    ClassDB::bind_method(D_METHOD("get_steering_half_speed_kph"), &SteeringRackData::get_steering_half_speed_kph);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steering_half_speed_kph", PROPERTY_HINT_RANGE, "0,200,1,or_greater,suffix:km/h"), "set_steering_half_speed_kph", "get_steering_half_speed_kph");
    ClassDB::bind_method(D_METHOD("set_road_feedback_strength", "value"), &SteeringRackData::set_road_feedback_strength);
    ClassDB::bind_method(D_METHOD("get_road_feedback_strength"), &SteeringRackData::get_road_feedback_strength);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "road_feedback_strength", PROPERTY_HINT_RANGE, "0,1,0.01,or_greater"), "set_road_feedback_strength", "get_road_feedback_strength");
    ADD_GROUP("Advanced", "");
    ClassDB::bind_method(D_METHOD("set_friction_torque", "value"), &SteeringRackData::set_friction_torque);
    ClassDB::bind_method(D_METHOD("get_friction_torque"), &SteeringRackData::get_friction_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_torque", PROPERTY_HINT_RANGE, "0,5,0.01,or_greater,suffix:Nm"), "set_friction_torque", "get_friction_torque");
}

void SteeringRackData::set_max_angle(real_t value) {
    max_angle = std::isfinite(value) ? std::clamp(value, real_t{0}, real_t{80}) : real_t{35.0};
}
void SteeringRackData::set_response_time_ms(real_t value) {
    response_time_ms = std::isfinite(value) ? std::clamp(value, real_t{10}, real_t{10000}) : real_t{160.0};
}
void SteeringRackData::set_steering_half_speed_kph(real_t value) {
    steering_half_speed_kph = std::isfinite(value) ? std::clamp(value, real_t{0}, real_t{10000}) : real_t{50.0};
}
void SteeringRackData::set_road_feedback_strength(real_t value) {
    road_feedback_strength = std::isfinite(value) ? std::clamp(value, real_t{0}, real_t{10}) : real_t{0.5};
}
void SteeringRackData::set_friction_torque(real_t value) {
    friction_torque = std::isfinite(value) ? std::clamp(value, real_t{0}, real_t{1000}) : real_t{0.3};
}

} // namespace godot
