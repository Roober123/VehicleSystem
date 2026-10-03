#pragma once
#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/core/class_db.hpp"

namespace godot {

class DifferentialData : public Resource {
    GDCLASS(DifferentialData, Resource);

public:
    enum Mode { OPEN = 0, LIMITED_SLIP = 1, LOCKED = 2 };
    void set_mode(Mode value);
    Mode get_mode() const { return mode; }
    void _validate_property(PropertyInfo &property) const;
    void set_acceleration_lock_percent(real_t value);
    real_t get_acceleration_lock_percent() const { return acceleration_lock_percent; }
    void set_engine_braking_lock_percent(real_t value);
    real_t get_engine_braking_lock_percent() const { return engine_braking_lock_percent; }
    void set_preload_torque(real_t value);
    real_t get_preload_torque() const { return preload_torque; }
    void set_max_lock_torque(real_t value);
    real_t get_max_lock_torque() const { return max_lock_torque; }
    void set_speed_lock_torque_per_100_rpm(real_t value);
    real_t get_speed_lock_torque_per_100_rpm() const { return speed_lock_torque_per_100_rpm; }

    // Runtime constraint coefficients, derived from the authoring controls.
    real_t get_power_lock_ratio() const { return acceleration_lock_percent / real_t{200.0}; }
    real_t get_coast_lock_ratio() const { return engine_braking_lock_percent / real_t{200.0}; }
    real_t get_slip_sensitive_gain() const;

protected:
    static void _bind_methods();

private:
    Mode mode = OPEN;
    real_t acceleration_lock_percent = 70.0;
    real_t engine_braking_lock_percent = 30.0;
    real_t preload_torque = 25.0;
    real_t max_lock_torque = 250.0;
    real_t speed_lock_torque_per_100_rpm = 20.943951024;
};

} // namespace godot

VARIANT_ENUM_CAST(godot::DifferentialData::Mode);
