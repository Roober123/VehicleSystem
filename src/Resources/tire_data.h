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
    void set_friction_forward(real_t value);
    real_t get_friction_forward() const { return friction_forward; }
    void set_friction_lateral(real_t value);
    real_t get_friction_lateral() const { return friction_lateral; }
    void set_peak_slip_ratio(real_t value);
    real_t get_peak_slip_ratio() const { return peak_slip_ratio; }
    void set_lateral_response_angle(real_t value);
    real_t get_lateral_response_angle() const { return lateral_response_angle; }
    void set_load_grip_loss_percent(real_t value);
    real_t get_load_grip_loss_percent() const { return load_grip_loss_percent; }
    void set_radius(real_t value);
    real_t get_radius() const { return radius; }
    void set_tire_width(real_t value);
    real_t get_tire_width() const { return tire_width; }
    void set_brake_power(real_t value);
    real_t get_brake_power() const { return brake_power; }
    void set_force_response_low_speed_ms(real_t value);
    real_t get_force_response_low_speed_ms() const { return force_response_low_speed_ms; }
    void set_force_response_108_kph_ms(real_t value);
    real_t get_force_response_108_kph_ms() const { return force_response_108_kph_ms; }
    void set_aligning_trail_mm(real_t value);
    real_t get_aligning_trail_mm() const { return aligning_trail_mm; }
    void set_aligning_trail_retained_percent(real_t value);
    real_t get_aligning_trail_retained_percent() const { return aligning_trail_retained_percent; }
    void set_drag(real_t value);
    real_t get_drag() const { return drag; }
    void set_combined_grip_exponent(real_t value);
    real_t get_combined_grip_exponent() const { return combined_grip_exponent; }

    void set_forward_friction_curve(const Ref<Curve> &value);
    Ref<Curve> get_forward_friction_curve() const { return forward_friction_curve; }
    void set_lateral_friction_curve(const Ref<Curve> &value);
    Ref<Curve> get_lateral_friction_curve() const { return lateral_friction_curve; }

    // Compiled values consumed by Wheel, in SI units.
    real_t get_relaxation_low() const { return force_response_low_speed_ms * real_t{0.001}; }
    real_t get_relaxation_high() const { return force_response_108_kph_ms * real_t{0.001}; }
    real_t get_load_sensitivity() const;
    real_t get_mechanical_trail() const;
    real_t get_pneumatic_trail() const;

private:
    real_t friction_forward = 1.0;
    real_t friction_lateral = 1.0;
    real_t peak_slip_ratio = 0.15;
    real_t lateral_response_angle = 10.0;
    real_t load_grip_loss_percent = 6.696700846;
    real_t radius = 0.3;
    real_t tire_width = 0.25;
    real_t brake_power = 1500.0;
    real_t force_response_low_speed_ms = 42.0;
    real_t force_response_108_kph_ms = 10.0;
    real_t aligning_trail_mm = 40.0;
    real_t aligning_trail_retained_percent = 50.0;
    real_t drag = 0.1;
    real_t combined_grip_exponent = 2.0;
    Ref<Curve> forward_friction_curve;
    Ref<Curve> lateral_friction_curve;
};

} // namespace godot
