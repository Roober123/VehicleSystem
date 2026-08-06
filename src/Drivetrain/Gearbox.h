#pragma once

#include <vector>

#include "Resources/gearbox_data.h"
#include "VehicleEngine.h"

namespace godot {

/// Gear selection and clutch phase state. Gearbox owns all mutable running-
/// gear state consumed by the compiled rotational network.
class Gearbox {
    VehicleEngine *engine = nullptr;
    RotationalBody *driveshaft = nullptr;

    std::vector<real_t> gear_ratios;
    real_t final_drive = 0.0;
    real_t reverse_ratio = 0.0;
    real_t clutch_capacity = 0.0;

    real_t upshift_ratio = 0.0;
    real_t downshift_ratio = 0.0;
    real_t shift_time = 0.0;
    real_t clutch_engage_speed = 10.0;
    real_t clutch_disengage_speed = 10.0;
    bool automatic = true;

    int current_gear = 0;
    int shift_input = 0;
    int target_gear = 0;

    enum class ShiftState {
        Idle,
        Disengaging,
        GearChange,
        ReEngaging
    } state = ShiftState::Idle;

    real_t clutch_engagement = 0.0;
    real_t shift_timer = 0.0;
    real_t shift_cooldown = 0.0;
    real_t change_duration = 0.0;

    void compute_phase_times();
    void start_shift(int gear);
    void perform_gear_change();
    void shift_automatic(real_t dt);
    void shift_manual();
    real_t get_current_rpm_normalized() const;
    real_t get_driveshaft_rpm_normalized() const;
    int get_gear_count() const { return static_cast<int>(gear_ratios.size()); }

public:
    void set_bodies(VehicleEngine &engine_body, RotationalBody &driveshaft_body) {
        engine = &engine_body;
        driveshaft = &driveshaft_body;
    }
    void configure(const Ref<GearboxData> &data);

    void update_shifting_logic(real_t dt);
    void update_clutch_logic(real_t dt, real_t brake_input, real_t throttle_input);

    void shift_up();
    void shift_down();
    /// Switches command mode without changing the current gear or an active
    /// clutch/gear-change phase.  Any queued manual direction belongs to the
    /// previous mode and is therefore discarded.
    void set_automatic(bool value) {
        automatic = value;
        shift_input = 0;
    }
    bool is_automatic() const { return automatic; }
    bool is_shifting() const { return state != ShiftState::Idle; }

    int get_current_gear() const { return current_gear; }
    real_t get_effective_ratio() const;
    real_t get_clutch_engagement() const { return clutch_engagement; }
    real_t get_clutch_capacity() const { return clutch_capacity; }

    void select_reverse();
    void select_drive();
    void select_neutral();
};

} // namespace godot
