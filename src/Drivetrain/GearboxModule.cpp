#include "GearboxModule.h"
#include <algorithm>
#include <cmath>

namespace godot {

static inline real_t smooth_step(real_t t) {
    t = std::clamp<real_t>(t, real_t{0.0}, real_t{1.0});
    return t * t * (real_t{3.0} - real_t{2.0} * t);
}

void GearboxModule::load_constraint(ClutchGearConstraint* c, const Ref<GearboxData>& g) {
    gear_constraint = c;
    engine = static_cast<VehicleEngine*>(gear_constraint->get_engine());

    upshift_ratio   = g->get_upshift_rpm();
    downshift_ratio = g->get_downshift_rpm();
    shift_time      = g->get_shift_time();
    automatic       = g->get_auto_mode();

    // Apply driveshaft drag from gearbox data
    if (gear_constraint->get_output() != nullptr)
        gear_constraint->get_output()->set_drag(g->get_driveshaft_drag());

    compute_phase_times();
}

void GearboxModule::compute_phase_times() {
    disengage_duration = shift_time * 0.4;
    change_duration    = shift_time * 0.2;
    reengage_duration  = shift_time * 0.4;
}


void GearboxModule::update(real_t dt, real_t brake_input, real_t throttle_input) {
    update_shifting_logic(dt);
    update_clutch_logic(dt, brake_input, throttle_input);
}


void GearboxModule::update_shifting_logic(real_t dt) {
    if (state != ShiftState::Idle)  return;
    if (engine == nullptr || gear_constraint == nullptr)
        return;

    if (shift_cooldown > 0.0)   shift_cooldown -= dt;

    if (automatic)  _shift_automatic(dt);
    else    _shift_manual();
}


void GearboxModule::_shift_automatic(real_t dt) {
    const int current_gear = gear_constraint->current_gear;
    const real_t engine_rpm_norm = get_current_rpm_normalized();
    const real_t driveshaft_rpm_norm = get_driveshaft_rpm_normalized();

    if (current_gear == 0) {
        if (engine->throttle > 0.1 && engine_rpm_norm < 0.3) {
            start_shift(1);
            return;
        }
        return;
    }

    if (driveshaft_rpm_norm >= upshift_ratio &&
        shift_cooldown <= 0.0 &&
        current_gear > 0 &&
        current_gear < get_gear_count() &&
        engine->throttle > 0.05) {
        start_shift(current_gear + 1);
        return;
    }

    if (engine_rpm_norm <= downshift_ratio && current_gear > 1) {
        start_shift(current_gear - 1);
        return;
    }
}


void GearboxModule::_shift_manual() {
    if (shift_input == 0)   return;

    const int current_gear = gear_constraint->current_gear;
    const real_t driveshaft_omega = std::abs(gear_constraint->get_output()->get_angular_velocity());
    const bool nearly_stopped = driveshaft_omega < real_t{1.5};

    int target = current_gear + shift_input;

    if (shift_input > 0) {
        // Upshift
        if (current_gear == -1) target = 0; 
        else    target = std::min(target, get_gear_count());
        
    } else {
        // Downshift
        if (target < -1)    target = -1; // Clamp to reverse
        else if (target == -1 && !nearly_stopped)
            target = 0; // Block reverse while moving
    }

    if (target != current_gear)
        start_shift(target);

    shift_input = 0;
}


void GearboxModule::update_clutch_logic(real_t dt, real_t brake_input, real_t throttle_input) {
    if (gear_constraint == nullptr || engine == nullptr)
        return;

    switch (state) {

    case ShiftState::Idle: {
        shift_timer = real_t{0.0};

        constexpr real_t both_threshold = real_t{0.1};
        if (brake_input > both_threshold && throttle_input > both_threshold) {
            gear_constraint->clutch_engagement = real_t{0.0};
        } else if (brake_input > real_t{0.05}) {
            real_t driveshaft_speed = std::abs(get_driveshaft_rpm_normalized());
            real_t speed_factor = std::clamp(driveshaft_speed / real_t{0.15}, real_t{0.0}, real_t{1.0});
            real_t brake_clutch = speed_factor + (real_t{1.0} - speed_factor) * (real_t{1.0} - brake_input);
            gear_constraint->clutch_engagement = std::clamp(brake_clutch, real_t{0.05}, real_t{1.0});
        } else
            gear_constraint->clutch_engagement = 1.0;

        break;
    }

    case ShiftState::Disengaging: {
        shift_timer += dt;
        real_t t = std::clamp<real_t>(shift_timer / disengage_duration, real_t{0.0}, real_t{1.0});
        gear_constraint->clutch_engagement = real_t{1.0} - smooth_step(t);

        if (t >= real_t{1.0}) {
            gear_constraint->clutch_engagement = real_t{0.0};
            state = ShiftState::GearChange;
            shift_timer = real_t{0.0};
            perform_gear_change();
        }
        break;
    }

    case ShiftState::GearChange: {
        shift_timer += dt;
        gear_constraint->clutch_engagement = real_t{0.0};

        if (shift_timer >= change_duration) {
            state = ShiftState::ReEngaging;
            shift_timer = real_t{0.0};
        }
        break;
    }

    case ShiftState::ReEngaging: {
        shift_timer += dt;
        real_t t = std::clamp<real_t>(shift_timer / reengage_duration, real_t{0.0}, real_t{1.0});
        gear_constraint->clutch_engagement = smooth_step(t);

        if (t >= real_t{1.0}) {
            gear_constraint->clutch_engagement = real_t{1.0};
            state = ShiftState::Idle;
            shift_timer = real_t{0.0};
            shift_cooldown = 0.2; // Prevent rapid re-shifts
        }
        break;
    }
    }

    const real_t rpm_norm = engine->get_rpm_normalized();

    if (rpm_norm <= 0.01)
        gear_constraint->clutch_engagement = 0.0;
    else if (rpm_norm < 0.3) {
        
        real_t ramp = (rpm_norm - 0.01) / 0.29;
        real_t safe_limit = std::clamp(ramp, real_t{0.1}, real_t{1.0});
        gear_constraint->clutch_engagement = std::min(
            gear_constraint->clutch_engagement, safe_limit);
    }
    
}

void GearboxModule::start_shift(int gear) {
    target_gear = gear;
    state = ShiftState::Disengaging;
    shift_timer = 0.0;
}

void GearboxModule::perform_gear_change() {
    if (gear_constraint == nullptr) return;
    gear_constraint->current_gear = target_gear;
}

void GearboxModule::shift_up() {
    if (automatic)  return;
    shift_input = 1;
}

void GearboxModule::shift_down() {
    if (automatic)  return;
    shift_input = -1;
}

void GearboxModule::set_automatic(bool auto_mode) {
    automatic = auto_mode;
}

bool GearboxModule::is_automatic() const {
    return automatic;
}

bool GearboxModule::is_shifting() const {
    return state != ShiftState::Idle;
}

int GearboxModule::get_current_gear() const {
    return gear_constraint ? gear_constraint->current_gear : 0;
}

real_t GearboxModule::get_current_rpm_normalized() const {
    if (engine == nullptr)  return 0.0;
    return engine->get_rpm_normalized();
}

real_t GearboxModule::get_driveshaft_rpm_normalized() const {
    if (gear_constraint == nullptr || engine == nullptr)
        return 0.0;

    RotationalBody* driveshaft = gear_constraint->get_output();
    if (driveshaft == nullptr)
        return 0.0;

    const real_t ratio = gear_constraint->get_effective_ratio();
    if (ratio == 0.0)
        return 0.0;

    constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
    real_t reflected_omega = driveshaft->get_angular_velocity() * ratio;
    real_t reflected_rpm = reflected_omega * ang_to_rpm;
    real_t range = engine->get_redline_rpm() - engine->get_idle_rpm();
    if (range <= 0.0)
        return 0.0;

    return (reflected_rpm - engine->get_idle_rpm()) / range;
}

int GearboxModule::get_gear_count() const {
    if (gear_constraint == nullptr)
        return 0;
    return static_cast<int>(gear_constraint->gear_ratios.size());
}

void GearboxModule::select_reverse() {
    if (gear_constraint == nullptr)
        return;
    const int current = gear_constraint->current_gear;
    if (current == -1)
        return; // Already in reverse

    const real_t driveshaft_omega = gear_constraint->get_output()->get_angular_velocity();
    if (driveshaft_omega < real_t{1.5}) start_shift(-1);
}

void GearboxModule::select_drive() {
    if (gear_constraint == nullptr)
        return;
    const int current = gear_constraint->current_gear;
    if (current != 1)   start_shift(1);
}

void GearboxModule::select_neutral() {
    if (gear_constraint == nullptr)
        return;
    const int current = gear_constraint->current_gear;
    if (current != 0)   start_shift(0);
}

}