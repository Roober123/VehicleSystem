#pragma once
#include "ClutchGearConstraint.h"
#include "VehicleEngine.h"
#include "Resources/gearbox_data.h"

namespace godot {

class GearboxModule {

    ClutchGearConstraint* gear_constraint = nullptr;
    // Cache engine for quicker access
    VehicleEngine* engine = nullptr;
    

    real_t upshift_ratio = 0.0;       // Normalized RPM (0-1) to upshift at
    real_t downshift_ratio = 0.0;     // Normalized RPM (0-1) to downshift at
    real_t shift_time = 0.0;          // Total duration of a gear change (seconds)
    bool automatic = true;            // true = auto, false = semi-auto

    int shift_input = 0;              // -1 = shift down, 0 = none, 1 = shift up

    enum class ShiftState {
        Idle,           // No shift in progress, clutch fully engaged
        Disengaging,    // Clutch ramping from 1.0 to 0.0
        GearChange,     // Clutch at 0
        ReEngaging      // Clutch ramping from 0.0 to 1.0
    } state = ShiftState::Idle;

    real_t shift_timer = 0.0;
    int target_gear = 0;
    real_t shift_cooldown = 0.0;


    real_t disengage_duration = 0.0;
    real_t change_duration = 0.0;
    real_t reengage_duration = 0.0;

    void compute_phase_times();
    void start_shift(int gear);
    void perform_gear_change();
    real_t get_current_rpm_normalized() const;
    real_t get_driveshaft_rpm_normalized() const;
    int get_gear_count() const;

    void _shift_automatic(real_t dt);
    void _shift_manual();

    public:
    void load_constraint(ClutchGearConstraint* c, const Ref<GearboxData>& g);
    void update(real_t dt, real_t brake_input = 0.0, real_t throttle_input = 0.0);
    void update_shifting_logic(real_t dt);
    void update_clutch_logic(real_t dt, real_t brake_input, real_t throttle_input);

    // Semi-auto commands
    void shift_up();
    void shift_down();
    void set_automatic(bool auto_mode);
    bool is_automatic() const;
    bool is_shifting() const;
    int get_current_gear() const;

    // Gear selection
    void select_reverse();
    void select_drive();
    void select_neutral();
};

} // namespace godot