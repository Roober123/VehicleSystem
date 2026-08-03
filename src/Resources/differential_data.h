#pragma once

#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/core/class_db.hpp"

namespace godot {

/// Configuration for one differential.  Solver code consumes this resource
/// without mutating it at runtime.
class DifferentialData : public Resource {
    GDCLASS(DifferentialData, Resource);

public:
    enum Mode {
        OPEN = 0,
        LIMITED_SLIP = 1,
        LOCKED = 2,
    };

private:
    Mode mode = OPEN;
    real_t preload_torque = 25.0;
    real_t power_lock_ratio = 0.35;
    real_t coast_lock_ratio = 0.15;
    real_t slip_sensitive_gain = 2.0;
    real_t max_lock_torque = 250.0;

protected:
    static void _bind_methods();

public:
    DifferentialData() = default;
    ~DifferentialData() override = default;

    void set_mode(Mode value) { mode = value; }
    Mode get_mode() const { return mode; }

    void set_preload_torque(real_t value) { preload_torque = value; }
    real_t get_preload_torque() const { return preload_torque; }

    void set_power_lock_ratio(real_t value) { power_lock_ratio = value; }
    real_t get_power_lock_ratio() const { return power_lock_ratio; }

    void set_coast_lock_ratio(real_t value) { coast_lock_ratio = value; }
    real_t get_coast_lock_ratio() const { return coast_lock_ratio; }

    void set_slip_sensitive_gain(real_t value) { slip_sensitive_gain = value; }
    real_t get_slip_sensitive_gain() const { return slip_sensitive_gain; }

    void set_max_lock_torque(real_t value) { max_lock_torque = value; }
    real_t get_max_lock_torque() const { return max_lock_torque; }
};

} // namespace godot

VARIANT_ENUM_CAST(godot::DifferentialData::Mode);
