#pragma once
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/classes/curve.hpp"

namespace godot {

class TireData : public Resource {
    GDCLASS(TireData, Resource);

    protected:
    static void _bind_methods();
    
    public:
    TireData() = default;
    ~TireData() override = default;

    void set_friction_forward(real_t value);
    real_t get_friction_forward();

    void set_friction_lateral(real_t value);
    real_t get_friction_lateral();

    void set_forward_friction_curve(const Ref<Curve>& value);
    Ref<Curve> get_forward_friction_curve() const;

    void set_lateral_friction_curve(const Ref<Curve>& value);
    Ref<Curve> get_lateral_friction_curve() const;

    void set_radius(real_t value);
    real_t get_radius();

    void set_brake_power(real_t value);
    real_t get_brake_power();

    void set_drag(real_t value);
    real_t get_drag();

    void set_peak_slip_angle(real_t value);
    real_t get_peak_slip_angle();

    void set_relaxation_low(real_t value);
    real_t get_relaxation_low();

    void set_relaxation_high(real_t value);
    real_t get_relaxation_high();

    void set_tire_width(real_t value);
    real_t get_tire_width();

    real_t friction_forward = 1.0;
    real_t friction_lateral = 1.0;
    Ref<Curve> forward_friction_curve;
    Ref<Curve> lateral_friction_curve;
    real_t radius = 0.3;
    real_t brake_power = 1500.0;
    real_t drag = 0.1;
    real_t peak_slip_angle = 10.0;    // degrees - lower = sharper bite
    real_t relaxation_low = 0.042;    // seconds at 0 m/s
    real_t relaxation_high = 0.01;    // seconds at 30+ m/s
    real_t tire_width = 0.25;         // meters 
};

}