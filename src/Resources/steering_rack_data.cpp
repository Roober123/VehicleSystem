#include "steering_rack_data.h"

namespace godot {

void SteeringRackData::_bind_methods() {
    // Inertia
    ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &SteeringRackData::set_inertia);
    ClassDB::bind_method(D_METHOD("get_inertia"), &SteeringRackData::get_inertia);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0.001,1,0.001,or_greater"), "set_inertia", "get_inertia");

    // Damping
    ClassDB::bind_method(D_METHOD("set_damping", "damping"), &SteeringRackData::set_damping);
    ClassDB::bind_method(D_METHOD("get_damping"), &SteeringRackData::get_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,15,0.1,or_greater"), "set_damping", "get_damping");

    // Friction coefficient
    ClassDB::bind_method(D_METHOD("set_friction_coefficient", "coefficient"), &SteeringRackData::set_friction_coefficient);
    ClassDB::bind_method(D_METHOD("get_friction_coefficient"), &SteeringRackData::get_friction_coefficient);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction_coefficient", PROPERTY_HINT_RANGE, "0,5,0.001,or_greater"), "set_friction_coefficient", "get_friction_coefficient");

    // Max angle
    ClassDB::bind_method(D_METHOD("set_max_angle", "angle"), &SteeringRackData::set_max_angle);
    ClassDB::bind_method(D_METHOD("get_max_angle"), &SteeringRackData::get_max_angle);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_angle", PROPERTY_HINT_RANGE, "0,50,0.1,degrees"), "set_max_angle", "get_max_angle");

    // Proportional gain
    ClassDB::bind_method(D_METHOD("set_proportional_gain", "gain"), &SteeringRackData::set_proportional_gain);
    ClassDB::bind_method(D_METHOD("get_proportional_gain"), &SteeringRackData::get_proportional_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "proportional_gain", PROPERTY_HINT_RANGE, "100,500,0.001,or_greater"), "set_proportional_gain", "get_proportional_gain");

    // Derivative gain
    ClassDB::bind_method(D_METHOD("set_derivative_gain", "gain"), &SteeringRackData::set_derivative_gain);
    ClassDB::bind_method(D_METHOD("get_derivative_gain"), &SteeringRackData::get_derivative_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "derivative_gain", PROPERTY_HINT_RANGE, "15,35,0.001,or_greater"), "set_derivative_gain", "get_derivative_gain");

    // SAT gain
    ClassDB::bind_method(D_METHOD("set_sat_gain", "gain"), &SteeringRackData::set_sat_gain);
    ClassDB::bind_method(D_METHOD("get_sat_gain"), &SteeringRackData::get_sat_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sat_gain", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_sat_gain", "get_sat_gain");
}

// Inertia
void SteeringRackData::set_inertia(real_t p_inertia) {
    inertia = p_inertia;
}

real_t SteeringRackData::get_inertia() const {
    return inertia;
}

// Damping
void SteeringRackData::set_damping(real_t p_damping) {
    damping = p_damping;
}

real_t SteeringRackData::get_damping() const {
    return damping;
}

// Friction coefficient
void SteeringRackData::set_friction_coefficient(real_t p_coefficient) {
    friction_coefficient = p_coefficient;
}

real_t SteeringRackData::get_friction_coefficient() const {
    return friction_coefficient;
}

// Max angle
void SteeringRackData::set_max_angle(real_t p_angle) {
    max_angle = p_angle;
}

real_t SteeringRackData::get_max_angle() const {
    return max_angle;
}

// Proportional gain
void SteeringRackData::set_proportional_gain(real_t p_gain) {
    proportional_gain = p_gain;
}

real_t SteeringRackData::get_proportional_gain() const {
    return proportional_gain;
}

// Derivative gain
void SteeringRackData::set_derivative_gain(real_t p_gain) {
    derivative_gain = p_gain;
}

real_t SteeringRackData::get_derivative_gain() const {
    return derivative_gain;
}

// SAT gain
void SteeringRackData::set_sat_gain(real_t p_gain) {
    sat_gain = p_gain;
}

real_t SteeringRackData::get_sat_gain() const {
    return sat_gain;
}
}
