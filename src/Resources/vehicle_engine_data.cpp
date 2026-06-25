#include "vehicle_engine_data.h"

namespace godot {

void VehicleEngineData::_bind_methods() {
    // Torque curve
    ClassDB::bind_method(D_METHOD("set_torque_curve", "curve"), &VehicleEngineData::set_torque_curve);
    ClassDB::bind_method(D_METHOD("get_torque_curve"), &VehicleEngineData::get_torque_curve);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "torque_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_torque_curve", "get_torque_curve");

    // Idle RPM
    ClassDB::bind_method(D_METHOD("set_idle_rpm", "rpm"), &VehicleEngineData::set_idle_rpm);
    ClassDB::bind_method(D_METHOD("get_idle_rpm"), &VehicleEngineData::get_idle_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "idle_rpm", PROPERTY_HINT_RANGE, "0,10000,1,or_greater"), "set_idle_rpm", "get_idle_rpm");

    // Redline RPM
    ClassDB::bind_method(D_METHOD("set_redline_rpm", "rpm"), &VehicleEngineData::set_redline_rpm);
    ClassDB::bind_method(D_METHOD("get_redline_rpm"), &VehicleEngineData::get_redline_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "redline_rpm", PROPERTY_HINT_RANGE, "0,20000,1,or_greater"), "set_redline_rpm", "get_redline_rpm");

    // Inertia
    ClassDB::bind_method(D_METHOD("set_inertia", "inertia"), &VehicleEngineData::set_inertia);
    ClassDB::bind_method(D_METHOD("get_inertia"), &VehicleEngineData::get_inertia);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0.001,10,0.001,or_greater"), "set_inertia", "get_inertia");

    // Max torque
    ClassDB::bind_method(D_METHOD("set_max_torque", "torque"), &VehicleEngineData::set_max_torque);
    ClassDB::bind_method(D_METHOD("get_max_torque"), &VehicleEngineData::get_max_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_torque", PROPERTY_HINT_RANGE, "0,10000,1,or_greater"), "set_max_torque", "get_max_torque");
}

// Torque curve
void VehicleEngineData::set_torque_curve(const Ref<Curve> &p_curve) {
    torque_curve = p_curve;
}

Ref<Curve> VehicleEngineData::get_torque_curve() const {
    return torque_curve;
}

// Idle RPM
void VehicleEngineData::set_idle_rpm(real_t p_rpm) {
    idle_rpm = p_rpm;
}

real_t VehicleEngineData::get_idle_rpm() const {
    return idle_rpm;
}

// Redline RPM
void VehicleEngineData::set_redline_rpm(real_t p_rpm) {
    redline_rpm = p_rpm;
}

real_t VehicleEngineData::get_redline_rpm() const {
    return redline_rpm;
}

// Inertia
void VehicleEngineData::set_inertia(real_t p_inertia) {
    inertia = p_inertia;
}

real_t VehicleEngineData::get_inertia() const {
    return inertia;
}

// Max torque
void VehicleEngineData::set_max_torque(real_t p_torque) {
    max_torque = p_torque;
}

real_t VehicleEngineData::get_max_torque() const {
    return max_torque;
}

} // namespace godot