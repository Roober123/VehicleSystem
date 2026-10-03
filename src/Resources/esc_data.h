#pragma once

#include "godot_cpp/classes/resource.hpp"

namespace godot {

class ESCData : public Resource {
    GDCLASS(ESCData, Resource);
    bool torque_smoothing_enabled = false;
    real_t torque_engagement_rate = 60000.0;
    real_t torque_release_rate = 120000.0;

    bool enabled = true;
    real_t yaw_damping = 2.0;
    real_t minimum_speed = 3.0;
    real_t maximum_corrective_torque = 6000.0;
    real_t maximum_target_lateral_acceleration = 9.81;

protected:
    static void _bind_methods();

public:
    void set_torque_smoothing_enabled(bool value);
    bool get_torque_smoothing_enabled() const { return torque_smoothing_enabled; }
    void set_torque_engagement_rate(real_t value);
    real_t get_torque_engagement_rate() const { return torque_engagement_rate; }
    void set_torque_release_rate(real_t value);
    real_t get_torque_release_rate() const { return torque_release_rate; }
    void set_enabled(bool value) { enabled = value; }
    bool get_enabled() const { return enabled; }
    void set_yaw_damping(real_t value);
    real_t get_yaw_damping() const { return yaw_damping; }
    void set_minimum_speed(real_t value);
    real_t get_minimum_speed() const { return minimum_speed; }
    void set_maximum_corrective_torque(real_t value);
    real_t get_maximum_corrective_torque() const { return maximum_corrective_torque; }
    void set_maximum_target_lateral_acceleration(real_t value);
    real_t get_maximum_target_lateral_acceleration() const { return maximum_target_lateral_acceleration; }
};

} // namespace godot
