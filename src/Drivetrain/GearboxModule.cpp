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
    compute_phase_times();
}

void GearboxModule::compute_phase_times() {
    disengage_duration = shift_time * 0.4;
    change_duration    = shift_time * 0.2;
    reengage_duration  = shift_time * 0.4;
}


void GearboxModule::update(real_t dt) {
    update_shifting_logic();
    update_clutch_logic(dt);
}


void GearboxModule::update_shifting_logic() {
    if (state != ShiftState::Idle)
        return;
    if (engine == nullptr || gear_constraint == nullptr)
        return;

    const int current_gear = gear_constraint->current_gear;
    const real_t rpm_norm = get_current_rpm_normalized();

    if (automatic) {

        if (current_gear == 0) {
            if (engine->throttle > 0.1 && rpm_norm < 0.3) {
                start_shift(1);
                return;
            }
            return;
        }

        if (rpm_norm >= upshift_ratio &&
            current_gear > 0 &&
            current_gear < get_gear_count() &&
            engine->throttle > 0.05) {
            start_shift(current_gear + 1);
            return;
        }

        if (rpm_norm <= downshift_ratio && current_gear > 1) {
            start_shift(current_gear - 1);
            return;
        }

    } else {
        // --- Semi-automatic: respond to user shift input ---
        if (shift_input == 0)
            return;

        int target = current_gear + shift_input;
        const bool nearly_stopped = rpm_norm < 0.1;

        if (shift_input > 0) {
            // Upshift
            if (current_gear == -1) {
                target = -1; // Can't upshift out of reverse
            } else {
                target = std::min(target, get_gear_count());
            }
        } else {
            // Downshift
            if (target < -1) {
                target = -1; // Clamp to reverse
            } else if (target == -1 && !nearly_stopped) {
                target = 0; // Block reverse while moving
            }
            // target == 0 from gear 1 stays at 0 (neutral), no change needed
        }

        if (target != current_gear)
            start_shift(target);

        shift_input = 0;
    }
}


void GearboxModule::update_clutch_logic(real_t dt) {
    if (gear_constraint == nullptr)
        return;

    switch (state) {

    case ShiftState::Idle: {
        shift_timer = real_t{0.0};
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
        }
        break;
    }
    }


    if (engine != nullptr && gear_constraint != nullptr) {
        if (engine->get_rpm_normalized() <= 0.01)
            gear_constraint->clutch_engagement = 0.0;
        else {
            real_t available = engine->get_available_torque();
            real_t max_safe = (available * real_t{0.85} + real_t{50.0}) / gear_constraint->clutch_max_torque;
            gear_constraint->clutch_engagement = std::min(
                gear_constraint->clutch_engagement,
                std::clamp(max_safe, real_t{0.04}, real_t{1.0})
            );
        }
    }
}

void GearboxModule::start_shift(int gear) {
    target_gear = gear;
    state = ShiftState::Disengaging;
    shift_timer = 0.0;
}

void GearboxModule::perform_gear_change() {
    if (gear_constraint == nullptr)
        return;
    gear_constraint->current_gear = target_gear;
}

void GearboxModule::shift_up() {
    if (automatic)
        return;
    shift_input = 1;
}

void GearboxModule::shift_down() {
    if (automatic)
        return;
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
    if (engine == nullptr)
        return 0.0;
    return engine->get_rpm_normalized();
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

    const real_t rpm_norm = get_current_rpm_normalized();
    if (rpm_norm < 0.1) { // Engine must be near idle to shift into reverse
        start_shift(-1);
    }
}

void GearboxModule::select_drive() {
    if (gear_constraint == nullptr)
        return;
    const int current = gear_constraint->current_gear;
    if (current != 1)
        start_shift(1);
}

void GearboxModule::select_neutral() {
    if (gear_constraint == nullptr)
        return;
    const int current = gear_constraint->current_gear;
    if (current != 0)
        start_shift(0);
}

} // namespace godot