#include "tire_data.h"
#include <algorithm>
#include <cmath>

namespace godot {
namespace {
real_t bounded(real_t value, real_t low, real_t high, real_t fallback) {
    return std::isfinite(value) ? std::clamp(value, low, high) : fallback;
}
} // namespace

void TireData::_bind_methods() {
    ADD_GROUP("Grip", "");
    ClassDB::bind_method(D_METHOD("set_friction_forward", "value"), &TireData::set_friction_forward);
    ClassDB::bind_method(D_METHOD("get_friction_forward"), &TireData::get_friction_forward);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_forward", PROPERTY_HINT_RANGE, "0,2,0.05"), "set_friction_forward", "get_friction_forward");
    ClassDB::bind_method(D_METHOD("set_friction_lateral", "value"), &TireData::set_friction_lateral);
    ClassDB::bind_method(D_METHOD("get_friction_lateral"), &TireData::get_friction_lateral);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_lateral", PROPERTY_HINT_RANGE, "0,2,0.05"), "set_friction_lateral", "get_friction_lateral");
    ClassDB::bind_method(D_METHOD("set_peak_slip_ratio", "value"), &TireData::set_peak_slip_ratio);
    ClassDB::bind_method(D_METHOD("get_peak_slip_ratio"), &TireData::get_peak_slip_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "peak_slip_ratio", PROPERTY_HINT_RANGE, "0.01,1,0.01"), "set_peak_slip_ratio", "get_peak_slip_ratio");
    ClassDB::bind_method(D_METHOD("set_lateral_response_angle", "value"), &TireData::set_lateral_response_angle);
    ClassDB::bind_method(D_METHOD("get_lateral_response_angle"), &TireData::get_lateral_response_angle);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lateral_response_angle", PROPERTY_HINT_RANGE, "0.1,25,0.5,degrees"), "set_lateral_response_angle", "get_lateral_response_angle");
    ClassDB::bind_method(D_METHOD("set_load_grip_loss_percent", "value"), &TireData::set_load_grip_loss_percent);
    ClassDB::bind_method(D_METHOD("get_load_grip_loss_percent"), &TireData::get_load_grip_loss_percent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "load_grip_loss_percent", PROPERTY_HINT_RANGE, "0,18.77476,0.1,suffix:%"), "set_load_grip_loss_percent", "get_load_grip_loss_percent");
    ADD_GROUP("Dimensions and Brakes", "");
    ClassDB::bind_method(D_METHOD("set_radius", "value"), &TireData::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &TireData::get_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.05,1,0.01,suffix:m"), "set_radius", "get_radius");
    ClassDB::bind_method(D_METHOD("set_tire_width", "value"), &TireData::set_tire_width);
    ClassDB::bind_method(D_METHOD("get_tire_width"), &TireData::get_tire_width);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_width", PROPERTY_HINT_RANGE, "0.05,1,0.01,suffix:m"), "set_tire_width", "get_tire_width");
    ClassDB::bind_method(D_METHOD("set_brake_power", "value"), &TireData::set_brake_power);
    ClassDB::bind_method(D_METHOD("get_brake_power"), &TireData::get_brake_power);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake_power", PROPERTY_HINT_RANGE, "0,10000,100,or_greater,suffix:Nm"), "set_brake_power", "get_brake_power");
    ADD_GROUP("Force Response", "");
    ClassDB::bind_method(D_METHOD("set_force_response_low_speed_ms", "value"), &TireData::set_force_response_low_speed_ms);
    ClassDB::bind_method(D_METHOD("get_force_response_low_speed_ms"), &TireData::get_force_response_low_speed_ms);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "force_response_low_speed_ms", PROPERTY_HINT_RANGE, "0,200,1,or_greater,suffix:ms"), "set_force_response_low_speed_ms", "get_force_response_low_speed_ms");
    ClassDB::bind_method(D_METHOD("set_force_response_108_kph_ms", "value"), &TireData::set_force_response_108_kph_ms);
    ClassDB::bind_method(D_METHOD("get_force_response_108_kph_ms"), &TireData::get_force_response_108_kph_ms);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "force_response_108_kph_ms", PROPERTY_HINT_RANGE, "0,50,1,or_greater,suffix:ms"), "set_force_response_108_kph_ms", "get_force_response_108_kph_ms");
    ADD_GROUP("Aligning Feedback", "");
    ClassDB::bind_method(D_METHOD("set_aligning_trail_mm", "value"), &TireData::set_aligning_trail_mm);
    ClassDB::bind_method(D_METHOD("get_aligning_trail_mm"), &TireData::get_aligning_trail_mm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "aligning_trail_mm", PROPERTY_HINT_RANGE, "0,200,1,suffix:mm"), "set_aligning_trail_mm", "get_aligning_trail_mm");
    ClassDB::bind_method(D_METHOD("set_aligning_trail_retained_percent", "value"), &TireData::set_aligning_trail_retained_percent);
    ClassDB::bind_method(D_METHOD("get_aligning_trail_retained_percent"), &TireData::get_aligning_trail_retained_percent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "aligning_trail_retained_percent", PROPERTY_HINT_RANGE, "0,100,1,suffix:%"), "set_aligning_trail_retained_percent", "get_aligning_trail_retained_percent");
    ADD_GROUP("Advanced", "");
    ClassDB::bind_method(D_METHOD("set_drag", "value"), &TireData::set_drag);
    ClassDB::bind_method(D_METHOD("get_drag"), &TireData::get_drag);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_drag", "get_drag");
    ClassDB::bind_method(D_METHOD("set_combined_grip_exponent", "value"), &TireData::set_combined_grip_exponent);
    ClassDB::bind_method(D_METHOD("get_combined_grip_exponent"), &TireData::get_combined_grip_exponent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "combined_grip_exponent", PROPERTY_HINT_RANGE, "1,16,0.1"), "set_combined_grip_exponent", "get_combined_grip_exponent");
    ADD_GROUP("Grip Curves", "");
    ClassDB::bind_method(D_METHOD("set_forward_friction_curve", "value"), &TireData::set_forward_friction_curve);
    ClassDB::bind_method(D_METHOD("get_forward_friction_curve"), &TireData::get_forward_friction_curve);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "forward_friction_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_forward_friction_curve", "get_forward_friction_curve");
    ClassDB::bind_method(D_METHOD("set_lateral_friction_curve", "value"), &TireData::set_lateral_friction_curve);
    ClassDB::bind_method(D_METHOD("get_lateral_friction_curve"), &TireData::get_lateral_friction_curve);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lateral_friction_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_lateral_friction_curve", "get_lateral_friction_curve");
}

void TireData::set_friction_forward(real_t value) {
    friction_forward = bounded(value, real_t{0}, real_t{2}, real_t{1.0});
}
void TireData::set_friction_lateral(real_t value) {
    friction_lateral = bounded(value, real_t{0}, real_t{2}, real_t{1.0});
}
void TireData::set_peak_slip_ratio(real_t value) {
    peak_slip_ratio = bounded(value, real_t{0.01}, real_t{1.0}, real_t{0.15});
}
void TireData::set_lateral_response_angle(real_t value) {
    lateral_response_angle = bounded(value, real_t{0.1}, real_t{90}, real_t{10.0});
}
void TireData::set_load_grip_loss_percent(real_t value) {
    load_grip_loss_percent = bounded(value, real_t{0}, real_t{18.774760364}, real_t{6.696700846});
}
void TireData::set_radius(real_t value) {
    radius = bounded(value, real_t{0.05}, real_t{10}, real_t{0.3});
}
void TireData::set_tire_width(real_t value) {
    tire_width = bounded(value, real_t{0.01}, real_t{10}, real_t{0.25});
}
void TireData::set_brake_power(real_t value) {
    brake_power = bounded(value, real_t{0}, real_t{1000000}, real_t{1500.0});
}
void TireData::set_force_response_low_speed_ms(real_t value) {
    force_response_low_speed_ms = bounded(value, real_t{0}, real_t{10000}, real_t{42.0});
}
void TireData::set_force_response_108_kph_ms(real_t value) {
    force_response_108_kph_ms = bounded(value, real_t{0}, real_t{10000}, real_t{10.0});
}
void TireData::set_aligning_trail_mm(real_t value) {
    aligning_trail_mm = bounded(value, real_t{0}, real_t{400}, real_t{40.0});
}
void TireData::set_aligning_trail_retained_percent(real_t value) {
    aligning_trail_retained_percent = bounded(value, real_t{0}, real_t{100}, real_t{50.0});
}
void TireData::set_drag(real_t value) {
    drag = bounded(value, real_t{0}, real_t{1000}, real_t{0.1});
}
void TireData::set_combined_grip_exponent(real_t value) {
    combined_grip_exponent = bounded(value, real_t{1}, real_t{16}, real_t{2.0});
}

void TireData::set_forward_friction_curve(const Ref<Curve> &value) { forward_friction_curve = value; }
void TireData::set_lateral_friction_curve(const Ref<Curve> &value) { lateral_friction_curve = value; }

real_t TireData::get_load_sensitivity() const {
    return -std::log1p(-load_grip_loss_percent / real_t{100.0}) / std::log(real_t{2.0});
}

real_t TireData::get_mechanical_trail() const {
    return aligning_trail_mm * real_t{0.001} * aligning_trail_retained_percent / real_t{100.0};
}

real_t TireData::get_pneumatic_trail() const {
    return aligning_trail_mm * real_t{0.001} - get_mechanical_trail();
}

} // namespace godot
