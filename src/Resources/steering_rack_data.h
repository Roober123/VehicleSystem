#pragma once
#include "godot_cpp/classes/resource.hpp"

namespace godot {

// Smooth critically damped steering; response time is its no-feedback 90% time.
class SteeringRackData : public Resource {
    GDCLASS(SteeringRackData, Resource);

protected:
    static void _bind_methods();

public:
    void set_max_angle(real_t value);
    real_t get_max_angle() const { return max_angle; }
    void set_response_time_ms(real_t value);
    real_t get_response_time_ms() const { return response_time_ms; }
    void set_steering_half_speed_kph(real_t value);
    real_t get_steering_half_speed_kph() const { return steering_half_speed_kph; }
    void set_road_feedback_strength(real_t value);
    real_t get_road_feedback_strength() const { return road_feedback_strength; }
    void set_friction_torque(real_t value);
    real_t get_friction_torque() const { return friction_torque; }

private:
    real_t max_angle = 35.0;
    real_t response_time_ms = 160.0;
    real_t steering_half_speed_kph = 50.0;
    real_t road_feedback_strength = 0.5;
    real_t friction_torque = 0.3;
};

} // namespace godot
