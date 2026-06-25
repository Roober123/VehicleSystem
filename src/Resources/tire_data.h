#pragma once
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class TireData : public Resource {
    GDCLASS(TireData, Resource);

    protected:
    static void _bind_methods();
    
    public:
    TireData() = default;
    ~TireData() override = default;

    void set_friction_coefficient(real_t value);
    real_t get_friction_coefficient();

    void set_radius(real_t value);
    real_t get_radius();

    void set_longitudinal_stiffness(real_t value);
    real_t get_longitudinal_stiffness();

    void set_lateral_stiffness(real_t value);
    real_t get_lateral_stiffness();

    void set_patch_length(real_t value);
    real_t get_patch_length();

    void set_brake_power(real_t value);
    real_t get_brake_power();

    real_t friction_coefficient = 1.0;
    real_t radius = 0.3;
    real_t longitudinal_stiffness = 80000;
    real_t lateral_stiffness = 60000;
    real_t patch_length = 0.4;
    real_t brake_power = 1500.0;
    
};

}