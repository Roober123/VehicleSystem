#pragma once

#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class VehicleAerodynamicsData : public Resource {
    GDCLASS(VehicleAerodynamicsData, Resource);

protected:
    static void _bind_methods();

public:
    VehicleAerodynamicsData() = default;
    ~VehicleAerodynamicsData() override = default;

    void set_drag_coefficient(real_t v);
    real_t get_drag_coefficient() const;

    void set_downforce_coefficient(real_t v);
    real_t get_downforce_coefficient() const;

    void set_yaw_damping_coefficient(real_t v);
    real_t get_yaw_damping_coefficient() const;

    void set_yaw_control_min_speed(real_t v);
    real_t get_yaw_control_min_speed() const;

    void set_yaw_control_max_torque(real_t v);
    real_t get_yaw_control_max_torque() const;

    void set_yaw_control_max_lateral_acceleration(real_t v);
    real_t get_yaw_control_max_lateral_acceleration() const;

    real_t drag_coefficient = 0.35;
    real_t downforce_coefficient = 0.15;
    real_t yaw_damping_coefficient = 2.0;
    real_t yaw_control_min_speed = 3.0;
    real_t yaw_control_max_torque = 6000.0;
    real_t yaw_control_max_lateral_acceleration = 9.81;
};

}
