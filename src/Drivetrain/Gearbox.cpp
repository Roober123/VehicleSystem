#include "Gearbox.h"

#include <algorithm>
#include <cmath>

namespace godot {

static inline real_t smooth_step(real_t t) {
    t = std::clamp<real_t>(t, real_t{0.0}, real_t{1.0});
    return t * t * (real_t{3.0} - real_t{2.0} * t);
}

void Gearbox::configure(const Ref<GearboxData> &data) {
    gear_ratios.clear();
    const PackedFloat64Array ratios = data->get_gear_ratios();
    gear_ratios.reserve(ratios.size());
    for (int i = 0; i < ratios.size(); ++i)
        gear_ratios.push_back(ratios[i]);

    final_drive = data->get_final_drive();
    reverse_ratio = data->get_reverse_ratio();
    clutch_capacity = data->get_clutch_max_torque();
    upshift_ratio = data->get_upshift_rpm();
    downshift_ratio = data->get_downshift_rpm();
    shift_time = data->get_shift_time();
    automatic = data->get_auto_mode();
    driveshaft->set_drag(data->get_driveshaft_drag());

    current_gear = 0;
    target_gear = 0;
    shift_input = 0;
    state = ShiftState::Idle;
    clutch_engagement = 0.0;
    shift_timer = 0.0;
    shift_cooldown = 0.0;
    compute_phase_times();
}

void Gearbox::compute_phase_times() {
    disengage_duration = shift_time * real_t{0.4};
    change_duration = shift_time * real_t{0.2};
    reengage_duration = shift_time * real_t{0.4};
}

real_t Gearbox::get_effective_ratio() const {
    if (current_gear == 0)
        return real_t{0.0};
    if (current_gear < 0)
        return reverse_ratio * final_drive;
    const int index = current_gear - 1;
    if (index < 0 || index >= static_cast<int>(gear_ratios.size()))
        return final_drive;
    return gear_ratios[index] * final_drive;
}

void Gearbox::update_shifting_logic(real_t dt) {
    if (!std::isfinite(dt) || dt <= real_t{0.0} || state != ShiftState::Idle)
        return;

    if (shift_cooldown > real_t{0.0})
        shift_cooldown -= dt;

    if (automatic)
        shift_automatic(dt);
    else
        shift_manual();
}

void Gearbox::shift_automatic(real_t dt) {
    (void)dt;
    const real_t engine_rpm_norm = get_current_rpm_normalized();
    const real_t driveshaft_rpm_norm = get_driveshaft_rpm_normalized();

    if (current_gear == 0) {
        if (engine->throttle > real_t{0.1} && engine_rpm_norm < real_t{0.3})
            start_shift(1);
        return;
    }

    if (driveshaft_rpm_norm >= upshift_ratio &&
            shift_cooldown <= real_t{0.0} && current_gear > 0 &&
            current_gear < get_gear_count() && engine->throttle > real_t{0.05}) {
        start_shift(current_gear + 1);
        return;
    }

    if (engine_rpm_norm <= downshift_ratio && current_gear > 1)
        start_shift(current_gear - 1);
}

void Gearbox::shift_manual() {
    if (shift_input == 0)
        return;

    const int requested_direction = shift_input;
    const real_t driveshaft_omega = std::abs(driveshaft->get_angular_velocity());
    const bool nearly_stopped = driveshaft_omega < real_t{1.5};
    int target = current_gear + requested_direction;

    if (requested_direction > 0) {
        if (current_gear == -1)
            target = 0;
        else
            target = std::min(target, get_gear_count());
    } else {
        if (target < -1)
            target = -1;
        else if (target == -1 && !nearly_stopped)
            target = 0;
    }

    if (target != current_gear)
        start_shift(target);
    shift_input = 0;
}

void Gearbox::update_clutch_logic(real_t dt, real_t brake_input, real_t throttle_input) {
    if (!std::isfinite(dt) || dt <= real_t{0.0})
        return;

    switch (state) {
    case ShiftState::Idle: {
        shift_timer = real_t{0.0};
        constexpr real_t both_threshold = real_t{0.1};
        if (brake_input > both_threshold && throttle_input > both_threshold) {
            clutch_engagement = real_t{0.0};
        } else if (brake_input > real_t{0.05}) {
            const real_t driveshaft_speed = std::abs(get_driveshaft_rpm_normalized());
            const real_t speed_factor = std::clamp(
                driveshaft_speed / real_t{0.15}, real_t{0.0}, real_t{1.0});
            const real_t brake_clutch = speed_factor +
                (real_t{1.0} - speed_factor) * (real_t{1.0} - brake_input);
            clutch_engagement = std::clamp(brake_clutch, real_t{0.05}, real_t{1.0});
        } else {
            clutch_engagement = real_t{1.0};
        }
        break;
    }
    case ShiftState::Disengaging: {
        shift_timer += dt;
        const real_t t = std::clamp<real_t>(
            shift_timer / disengage_duration, real_t{0.0}, real_t{1.0});
        clutch_engagement = real_t{1.0} - smooth_step(t);
        if (t >= real_t{1.0}) {
            clutch_engagement = real_t{0.0};
            state = ShiftState::GearChange;
            shift_timer = real_t{0.0};
            perform_gear_change();
        }
        break;
    }
    case ShiftState::GearChange:
        shift_timer += dt;
        clutch_engagement = real_t{0.0};
        if (shift_timer >= change_duration) {
            state = ShiftState::ReEngaging;
            shift_timer = real_t{0.0};
        }
        break;
    case ShiftState::ReEngaging: {
        shift_timer += dt;
        const real_t t = std::clamp<real_t>(
            shift_timer / reengage_duration, real_t{0.0}, real_t{1.0});
        clutch_engagement = smooth_step(t);
        if (t >= real_t{1.0}) {
            clutch_engagement = real_t{1.0};
            state = ShiftState::Idle;
            shift_timer = real_t{0.0};
            shift_cooldown = real_t{0.2};
        }
        break;
    }
    }

    const real_t rpm_norm = engine->get_rpm_normalized();
    if (rpm_norm <= real_t{0.01})
        clutch_engagement = real_t{0.0};
    else if (rpm_norm < real_t{0.3}) {
        const real_t ramp = (rpm_norm - real_t{0.01}) / real_t{0.29};
        const real_t safe_limit = std::clamp(ramp, real_t{0.1}, real_t{1.0});
        clutch_engagement = std::min(clutch_engagement, safe_limit);
    }
}

void Gearbox::start_shift(int gear) {
    target_gear = gear;
    state = ShiftState::Disengaging;
    shift_timer = real_t{0.0};
}

void Gearbox::perform_gear_change() {
    current_gear = target_gear;
}

void Gearbox::shift_up() {
    if (!automatic)
        shift_input = 1;
}

void Gearbox::shift_down() {
    if (!automatic)
        shift_input = -1;
}

real_t Gearbox::get_current_rpm_normalized() const {
    return engine->get_rpm_normalized();
}

real_t Gearbox::get_driveshaft_rpm_normalized() const {
    const real_t ratio = get_effective_ratio();
    if (ratio == real_t{0.0})
        return real_t{0.0};

    constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
    const real_t reflected_rpm = driveshaft->get_angular_velocity() * ratio * ang_to_rpm;
    const real_t range = engine->get_redline_rpm() - engine->get_idle_rpm();
    if (range <= real_t{0.0})
        return real_t{0.0};
    return (reflected_rpm - engine->get_idle_rpm()) / range;
}

void Gearbox::select_reverse() {
    if (current_gear == -1)
        return;
    if (driveshaft->get_angular_velocity() < real_t{1.5})
        start_shift(-1);
}

void Gearbox::select_drive() {
    if (current_gear != 1)
        start_shift(1);
}

void Gearbox::select_neutral() {
    if (current_gear != 0)
        start_shift(0);
}

} // namespace godot
