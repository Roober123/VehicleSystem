#pragma once
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class SteeringRackData : public Resource {
    GDCLASS(SteeringRackData, Resource);
    real_t inertia = 0.3;

    real_t damping = 5.0;
    real_t friction_coefficient = 0.3;
    real_t max_angle = 35.0; // degrees

    real_t proportional_gain = 400.0;
    real_t derivative_gain = 25.0;

    real_t sat_gain = 0.5;

    protected:
    static void _bind_methods();

    public:


    void set_inertia(real_t p_inertia);
    real_t get_inertia() const;
    void set_damping(real_t p_damping);
    real_t get_damping() const;
    void set_friction_coefficient(real_t p_coefficient);
    real_t get_friction_coefficient() const;
    void set_max_angle(real_t p_angle);
    real_t get_max_angle() const;
    void set_proportional_gain(real_t p_gain);
    real_t get_proportional_gain() const;
    void set_derivative_gain(real_t p_gain);
    real_t get_derivative_gain() const;
    void set_sat_gain(real_t p_gain);
    real_t get_sat_gain() const;
};

}