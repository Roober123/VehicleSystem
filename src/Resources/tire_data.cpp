#include "tire_data.h"
#include <algorithm>
#include <cmath>

namespace godot {

void TireData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_friction_forward", "value"), &TireData::set_friction_forward);
    ClassDB::bind_method(D_METHOD("get_friction_forward"), &TireData::get_friction_forward);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_forward", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_friction_forward", "get_friction_forward");

    ClassDB::bind_method(D_METHOD("set_friction_lateral", "value"), &TireData::set_friction_lateral);
    ClassDB::bind_method(D_METHOD("get_friction_lateral"), &TireData::get_friction_lateral);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_lateral", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_friction_lateral", "get_friction_lateral");

    ClassDB::bind_method(D_METHOD("set_forward_friction_curve", "value"), &TireData::set_forward_friction_curve);
    ClassDB::bind_method(D_METHOD("get_forward_friction_curve"), &TireData::get_forward_friction_curve);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "forward_friction_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"),
                          "set_forward_friction_curve", "get_forward_friction_curve");

    ClassDB::bind_method(D_METHOD("set_lateral_friction_curve", "value"), &TireData::set_lateral_friction_curve);
    ClassDB::bind_method(D_METHOD("get_lateral_friction_curve"), &TireData::get_lateral_friction_curve);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lateral_friction_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"),
                          "set_lateral_friction_curve", "get_lateral_friction_curve");

    ClassDB::bind_method(D_METHOD("set_radius", "value"), &TireData::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &TireData::get_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.05,1.0,0.05"), "set_radius", "get_radius");

    ClassDB::bind_method(D_METHOD("set_brake_power", "value"), &TireData::set_brake_power);
    ClassDB::bind_method(D_METHOD("get_brake_power"), &TireData::get_brake_power);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake_power", PROPERTY_HINT_RANGE, "0.0,10000.0,100.0"), "set_brake_power", "get_brake_power");

    ClassDB::bind_method(D_METHOD("set_drag", "value"), &TireData::set_drag);
    ClassDB::bind_method(D_METHOD("get_drag"), &TireData::get_drag);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag", PROPERTY_HINT_RANGE, "0.0,2.0,0.001"), "set_drag", "get_drag");

    ClassDB::bind_method(D_METHOD("set_peak_slip_angle", "value"), &TireData::set_peak_slip_angle);
    ClassDB::bind_method(D_METHOD("get_peak_slip_angle"), &TireData::get_peak_slip_angle);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "peak_slip_angle", PROPERTY_HINT_RANGE, "3.0,25.0,0.5,degrees"), "set_peak_slip_angle", "get_peak_slip_angle");

    ClassDB::bind_method(D_METHOD("set_relaxation_low", "value"), &TireData::set_relaxation_low);
    ClassDB::bind_method(D_METHOD("get_relaxation_low"), &TireData::get_relaxation_low);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "relaxation_low", PROPERTY_HINT_RANGE, "0.005,0.2,0.001,or_greater"), "set_relaxation_low", "get_relaxation_low");

    ClassDB::bind_method(D_METHOD("set_relaxation_high", "value"), &TireData::set_relaxation_high);
    ClassDB::bind_method(D_METHOD("get_relaxation_high"), &TireData::get_relaxation_high);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "relaxation_high", PROPERTY_HINT_RANGE, "0.001,0.05,0.001,or_greater"), "set_relaxation_high", "get_relaxation_high");

    ClassDB::bind_method(D_METHOD("set_tire_width", "value"), &TireData::set_tire_width);
    ClassDB::bind_method(D_METHOD("get_tire_width"), &TireData::get_tire_width);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_width", PROPERTY_HINT_RANGE, "0.05,1.0,0.01,metres"), "set_tire_width", "get_tire_width");

    ClassDB::bind_method(D_METHOD("set_load_sensitivity", "value"), &TireData::set_load_sensitivity);
    ClassDB::bind_method(D_METHOD("get_load_sensitivity"), &TireData::get_load_sensitivity);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "load_sensitivity", PROPERTY_HINT_RANGE, "0.0,0.3,0.01"), "set_load_sensitivity", "get_load_sensitivity");

    ClassDB::bind_method(D_METHOD("set_combined_grip_exponent", "value"), &TireData::set_combined_grip_exponent);
    ClassDB::bind_method(D_METHOD("get_combined_grip_exponent"), &TireData::get_combined_grip_exponent);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "combined_grip_exponent", PROPERTY_HINT_RANGE, "1.0,16.0,0.1"),
                 "set_combined_grip_exponent", "get_combined_grip_exponent");
}

void TireData::set_friction_forward(real_t value) { friction_forward = value; }
real_t TireData::get_friction_forward() { return friction_forward; }

void TireData::set_friction_lateral(real_t value) { friction_lateral = value; }
real_t TireData::get_friction_lateral() { return friction_lateral; }

void TireData::set_forward_friction_curve(const Ref<Curve>& value) { forward_friction_curve = value; }
Ref<Curve> TireData::get_forward_friction_curve() const { return forward_friction_curve; }

void TireData::set_lateral_friction_curve(const Ref<Curve>& value) { lateral_friction_curve = value; }
Ref<Curve> TireData::get_lateral_friction_curve() const { return lateral_friction_curve; }

void TireData::set_radius(real_t value) { radius = value; }
real_t TireData::get_radius() { return radius; }

void TireData::set_brake_power(real_t value) { brake_power = value; }
real_t TireData::get_brake_power() { return brake_power; }

void TireData::set_drag(real_t value) { drag = value; }
real_t TireData::get_drag() { return drag; }

void TireData::set_peak_slip_angle(real_t value) { peak_slip_angle = value; }
real_t TireData::get_peak_slip_angle() { return peak_slip_angle; }

void TireData::set_relaxation_low(real_t value) { relaxation_low = value; }
real_t TireData::get_relaxation_low() { return relaxation_low; }

void TireData::set_relaxation_high(real_t value) { relaxation_high = value; }
real_t TireData::get_relaxation_high() { return relaxation_high; }

void TireData::set_tire_width(real_t value) { tire_width = value; }
real_t TireData::get_tire_width() { return tire_width; }

void TireData::set_load_sensitivity(real_t value) { load_sensitivity = std::clamp(value, real_t{0.0}, real_t{0.3}); }
real_t TireData::get_load_sensitivity() const { return load_sensitivity; }

void TireData::set_combined_grip_exponent(real_t value) {
    constexpr real_t default_exponent = real_t{2.0};
    constexpr real_t min_exponent = real_t{1.0};
    constexpr real_t max_exponent = real_t{16.0};

    // NaN cannot be clamped reliably and infinities would leak into the
    // runtime power calculation.  Reset invalid input to the documented
    // default, while finite values remain bounded for stable simulation.
    if (!std::isfinite(static_cast<double>(value))) {
        combined_grip_exponent = default_exponent;
        return;
    }
    combined_grip_exponent = std::clamp(value, min_exponent, max_exponent);
}

real_t TireData::get_combined_grip_exponent() const { return combined_grip_exponent; }

}
