#include "vehicle_aerodynamics_data.h"
#include <algorithm>

namespace godot {

void VehicleAerodynamicsData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_drag_coefficient", "v"), &VehicleAerodynamicsData::set_drag_coefficient);
    ClassDB::bind_method(D_METHOD("get_drag_coefficient"), &VehicleAerodynamicsData::get_drag_coefficient);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag_coefficient", PROPERTY_HINT_RANGE, "0.01,2.0,0.01,or_greater"), "set_drag_coefficient", "get_drag_coefficient");

    ClassDB::bind_method(D_METHOD("set_downforce_coefficient", "v"), &VehicleAerodynamicsData::set_downforce_coefficient);
    ClassDB::bind_method(D_METHOD("get_downforce_coefficient"), &VehicleAerodynamicsData::get_downforce_coefficient);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "downforce_coefficient", PROPERTY_HINT_RANGE, "0.0,5.0,0.01,or_greater"), "set_downforce_coefficient", "get_downforce_coefficient");

    ClassDB::bind_method(D_METHOD("set_yaw_damping_coefficient", "v"), &VehicleAerodynamicsData::set_yaw_damping_coefficient);
    ClassDB::bind_method(D_METHOD("get_yaw_damping_coefficient"), &VehicleAerodynamicsData::get_yaw_damping_coefficient);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "yaw_damping_coefficient", PROPERTY_HINT_RANGE, "0.0,3.0,0.1,or_greater"), "set_yaw_damping_coefficient", "get_yaw_damping_coefficient");

    ClassDB::bind_method(D_METHOD("set_yaw_control_min_speed", "v"), &VehicleAerodynamicsData::set_yaw_control_min_speed);
    ClassDB::bind_method(D_METHOD("get_yaw_control_min_speed"), &VehicleAerodynamicsData::get_yaw_control_min_speed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "yaw_control_min_speed", PROPERTY_HINT_RANGE, "0.0,20.0,0.1,suffix:m/s"), "set_yaw_control_min_speed", "get_yaw_control_min_speed");

    ClassDB::bind_method(D_METHOD("set_yaw_control_max_torque", "v"), &VehicleAerodynamicsData::set_yaw_control_max_torque);
    ClassDB::bind_method(D_METHOD("get_yaw_control_max_torque"), &VehicleAerodynamicsData::get_yaw_control_max_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "yaw_control_max_torque", PROPERTY_HINT_RANGE, "0.0,20000.0,100.0,or_greater,suffix:Nm"), "set_yaw_control_max_torque", "get_yaw_control_max_torque");

    ClassDB::bind_method(D_METHOD("set_yaw_control_max_lateral_acceleration", "v"), &VehicleAerodynamicsData::set_yaw_control_max_lateral_acceleration);
    ClassDB::bind_method(D_METHOD("get_yaw_control_max_lateral_acceleration"), &VehicleAerodynamicsData::get_yaw_control_max_lateral_acceleration);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "yaw_control_max_lateral_acceleration", PROPERTY_HINT_RANGE, "0.1,30.0,0.1,or_greater,suffix:m/s²"), "set_yaw_control_max_lateral_acceleration", "get_yaw_control_max_lateral_acceleration");
}

void VehicleAerodynamicsData::set_drag_coefficient(real_t v) { drag_coefficient = v; }
real_t VehicleAerodynamicsData::get_drag_coefficient() const { return drag_coefficient; }

void VehicleAerodynamicsData::set_downforce_coefficient(real_t v) { downforce_coefficient = v; }
real_t VehicleAerodynamicsData::get_downforce_coefficient() const { return downforce_coefficient; }

void VehicleAerodynamicsData::set_yaw_damping_coefficient(real_t v) { yaw_damping_coefficient = v; }
real_t VehicleAerodynamicsData::get_yaw_damping_coefficient() const { return yaw_damping_coefficient; }

void VehicleAerodynamicsData::set_yaw_control_min_speed(real_t v) { yaw_control_min_speed = std::max(v, real_t{0.0}); }
real_t VehicleAerodynamicsData::get_yaw_control_min_speed() const { return yaw_control_min_speed; }

void VehicleAerodynamicsData::set_yaw_control_max_torque(real_t v) { yaw_control_max_torque = std::max(v, real_t{0.0}); }
real_t VehicleAerodynamicsData::get_yaw_control_max_torque() const { return yaw_control_max_torque; }

void VehicleAerodynamicsData::set_yaw_control_max_lateral_acceleration(real_t v) { yaw_control_max_lateral_acceleration = std::max(v, real_t{0.1}); }
real_t VehicleAerodynamicsData::get_yaw_control_max_lateral_acceleration() const { return yaw_control_max_lateral_acceleration; }

}
