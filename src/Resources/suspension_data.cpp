#include "suspension_data.h"

namespace godot {

void SuspensionData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_suspension_length", "value"), &SuspensionData::set_suspension_length);
    ClassDB::bind_method(D_METHOD("get_suspension_length"), &SuspensionData::get_suspension_length);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_length", PROPERTY_HINT_RANGE, "0.1,1.0,0.05"), "set_suspension_length", "get_suspension_length");

    ClassDB::bind_method(D_METHOD("set_rest_compression", "value"), &SuspensionData::set_rest_compression);
    ClassDB::bind_method(D_METHOD("get_rest_compression"), &SuspensionData::get_rest_compression);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rest_compression", PROPERTY_HINT_RANGE, "0.1,0.9,0.05"), "set_rest_compression", "get_rest_compression");

    ClassDB::bind_method(D_METHOD("set_damping_ratio", "value"), &SuspensionData::set_damping_ratio);
    ClassDB::bind_method(D_METHOD("get_damping_ratio"), &SuspensionData::get_damping_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping_ratio", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_damping_ratio", "get_damping_ratio");

    ClassDB::bind_method(D_METHOD("set_antiroll_bar_stiffness", "value"), &SuspensionData::set_antiroll_bar_stiffness);
    ClassDB::bind_method(D_METHOD("get_antiroll_bar_stiffness"), &SuspensionData::get_antiroll_bar_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "antiroll_bar_stiffness", PROPERTY_HINT_RANGE, "0.0,60000.0,1000.0"), "set_antiroll_bar_stiffness", "get_antiroll_bar_stiffness");
}

void SuspensionData::set_suspension_length(real_t value) {
    suspension_length = value;
}

real_t SuspensionData::get_suspension_length() {
    return suspension_length;
}

void SuspensionData::set_rest_compression(real_t value) {
    rest_compression = value;
}

real_t SuspensionData::get_rest_compression() {
    return rest_compression;
}

void SuspensionData::set_damping_ratio(real_t value) {
    damping_ratio = value;
}

real_t SuspensionData::get_damping_ratio() {
    return damping_ratio;
}

void SuspensionData::set_antiroll_bar_stiffness(real_t value) {
    antiroll_bar_stiffness = value;
}

real_t SuspensionData::get_antiroll_bar_stiffness() {
    return antiroll_bar_stiffness;
}

} // namespace godot