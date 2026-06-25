#include "tire_data.h"

namespace godot {

void TireData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_friction_coefficient", "value"), &TireData::set_friction_coefficient);
    ClassDB::bind_method(D_METHOD("get_friction_coefficient"), &TireData::get_friction_coefficient);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_coefficient", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_friction_coefficient", "get_friction_coefficient");

    ClassDB::bind_method(D_METHOD("set_radius", "value"), &TireData::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &TireData::get_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.05,1.0,0.05"), "set_radius", "get_radius");

    ClassDB::bind_method(D_METHOD("set_longitudinal_stiffness", "value"), &TireData::set_longitudinal_stiffness);
    ClassDB::bind_method(D_METHOD("get_longitudinal_stiffness"), &TireData::get_longitudinal_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "longitudinal_stiffness", PROPERTY_HINT_RANGE, "1000,500000,1000"), "set_longitudinal_stiffness", "get_longitudinal_stiffness");

    ClassDB::bind_method(D_METHOD("set_lateral_stiffness", "value"), &TireData::set_lateral_stiffness);
    ClassDB::bind_method(D_METHOD("get_lateral_stiffness"), &TireData::get_lateral_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lateral_stiffness", PROPERTY_HINT_RANGE, "1000,500000,1000"), "set_lateral_stiffness", "get_lateral_stiffness");

    ClassDB::bind_method(D_METHOD("set_patch_length", "value"), &TireData::set_patch_length);
    ClassDB::bind_method(D_METHOD("get_patch_length"), &TireData::get_patch_length);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "patch_length", PROPERTY_HINT_RANGE, "0.05,1.0,0.05"), "set_patch_length", "get_patch_length");

    ClassDB::bind_method(D_METHOD("set_brake_power", "value"), &TireData::set_brake_power);
    ClassDB::bind_method(D_METHOD("get_brake_power"), &TireData::get_brake_power);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake_power", PROPERTY_HINT_RANGE, "0.0,10000.0,100.0"), "set_brake_power", "get_brake_power");
}

void TireData::set_friction_coefficient(real_t value) {
    friction_coefficient = value;
}

real_t TireData::get_friction_coefficient() {
    return friction_coefficient;
}

void TireData::set_radius(real_t value) {
    radius = value;
}

real_t TireData::get_radius() {
    return radius;
}

void TireData::set_longitudinal_stiffness(real_t value) {
    longitudinal_stiffness = value;
}

real_t TireData::get_longitudinal_stiffness() {
    return longitudinal_stiffness;
}

void TireData::set_lateral_stiffness(real_t value) {
    lateral_stiffness = value;
}

real_t TireData::get_lateral_stiffness() {
    return lateral_stiffness;
}

void TireData::set_patch_length(real_t value) {
    patch_length = value;
}

real_t TireData::get_patch_length() {
    return patch_length;
}

void TireData::set_brake_power(real_t value) {
    brake_power = value;
}

real_t TireData::get_brake_power() {
    return brake_power;
}

} // namespace godot