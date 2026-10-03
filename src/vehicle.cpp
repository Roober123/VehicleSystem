#include "vehicle.h"

#include "vehicle_setup_validation.h"

#include <algorithm>
#include <cmath>

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

namespace godot {

void Vehicle::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_engine_axis", "axis"), &Vehicle::set_engine_axis);
    ClassDB::bind_method(D_METHOD("get_engine_axis"), &Vehicle::get_engine_axis);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "engine_axis"), "set_engine_axis", "get_engine_axis");

    ADD_SIGNAL(MethodInfo("vehicle_ready"));

    ClassDB::bind_method(D_METHOD("set_config", "config"), &Vehicle::set_config);
    ClassDB::bind_method(D_METHOD("get_config"), &Vehicle::get_config);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "config", PROPERTY_HINT_RESOURCE_TYPE, "VehicleConfig"),
                 "set_config", "get_config");

    ClassDB::bind_method(D_METHOD("set_center_of_mass_marker", "marker"),
                         &Vehicle::set_center_of_mass_marker);
    ClassDB::bind_method(D_METHOD("get_center_of_mass_marker"),
                         &Vehicle::get_center_of_mass_marker);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "center_of_mass_marker",
                              PROPERTY_HINT_NODE_TYPE, "Marker3D"),
                 "set_center_of_mass_marker", "get_center_of_mass_marker");

    ClassDB::bind_method(D_METHOD("set_throttle_input", "value"), &Vehicle::set_throttle_input);
    ClassDB::bind_method(D_METHOD("get_throttle_input"), &Vehicle::get_throttle_input);

    ClassDB::bind_method(D_METHOD("set_steer_input", "value"), &Vehicle::set_steer_input);
    ClassDB::bind_method(D_METHOD("get_steer_input"), &Vehicle::get_steer_input);

    ClassDB::bind_method(D_METHOD("set_brake_input", "value"), &Vehicle::set_brake_input);
    ClassDB::bind_method(D_METHOD("get_brake_input"), &Vehicle::get_brake_input);

    ClassDB::bind_method(D_METHOD("shift_up"), &Vehicle::shift_up);
    ClassDB::bind_method(D_METHOD("shift_down"), &Vehicle::shift_down);
    ClassDB::bind_method(D_METHOD("select_drive"), &Vehicle::select_drive);
    ClassDB::bind_method(D_METHOD("select_neutral"), &Vehicle::select_neutral);
    ClassDB::bind_method(D_METHOD("select_reverse"), &Vehicle::select_reverse);

    ClassDB::bind_method(D_METHOD("set_abs_enabled", "value"), &Vehicle::set_abs_enabled);
    ClassDB::bind_method(D_METHOD("get_abs_enabled"), &Vehicle::get_abs_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "abs_enabled"), "set_abs_enabled", "get_abs_enabled");

    ClassDB::bind_method(D_METHOD("set_tcs_enabled", "value"), &Vehicle::set_tcs_enabled);
    ClassDB::bind_method(D_METHOD("get_tcs_enabled"), &Vehicle::get_tcs_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "tcs_enabled"), "set_tcs_enabled", "get_tcs_enabled");

    ClassDB::bind_method(D_METHOD("set_gearbox_automatic", "value"),
                         &Vehicle::set_gearbox_automatic);
    ClassDB::bind_method(D_METHOD("get_gearbox_automatic"),
                         &Vehicle::get_gearbox_automatic);
    ClassDB::bind_method(D_METHOD("restart"), &Vehicle::restart);

    ClassDB::bind_method(D_METHOD("set_substeps", "value"), &Vehicle::set_substeps);
    ClassDB::bind_method(D_METHOD("get_substeps"), &Vehicle::get_substeps);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "substeps", PROPERTY_HINT_RANGE, "1,1024,1"),
                 "set_substeps", "get_substeps");

}

Vehicle::Vehicle() = default;

void Vehicle::_ready() {
    if (Engine::get_singleton()->is_editor_hint())
        return;

    initialize_runtime();
}

bool Vehicle::initialize_runtime() {
    if (initialized)
        return true;

    std::vector<Axle *> setup_axles;
    TypedArray<Node> children = get_children();
    setup_axles.reserve(children.size());
    for (int i = 0; i < children.size(); ++i) {
        Axle *axle = Object::cast_to<Axle>(children[i]);
        if (axle != nullptr)
            setup_axles.push_back(axle);
    }

    String setup_error;
    if (!VehicleSetupValidation::validate(config, setup_axles, setup_error)) {
        UtilityFunctions::printerr(String("Vehicle setup failed: ") + setup_error);
        return false;
    }

    set_linear_damp_mode(DampMode::DAMP_MODE_REPLACE);
    set_angular_damp_mode(DampMode::DAMP_MODE_REPLACE);
    set_linear_damp(0.0);
    set_angular_damp(0.0);

    if (center_of_mass_marker != nullptr) {
        const Vector3 marker_global_position =
                center_of_mass_marker->get_global_position();
        const Vector3 marker_local_position =
                get_global_transform().affine_inverse().xform(marker_global_position);
        set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
        set_center_of_mass(marker_local_position);
    }

    drivetrain.reset_runtime_state();
    running_gear.setup(setup_axles, get_mass(), config->get_suspension_data(),
                       config->get_aero_data());
    stability_control.load_parameters(config->get_esc_data());
    String drivetrain_error;
    if (!drivetrain.setup(config, running_gear.get_axles(), drivetrain_error)) {
        UtilityFunctions::printerr(String("Vehicle drivetrain setup failed: ") +
                                    drivetrain_error);
        return false;
    }

    initialized = true;
    engine_reaction.configure(config->get_engine_reaction_data());
    engine_reaction_sample = EngineReactionSample();
    engine_chassis_torque = Vector3();
    emit_signal("vehicle_ready");
    return true;
}

void Vehicle::_integrate_forces(PhysicsDirectBodyState3D *state) {
    RigidBody3D::_integrate_forces(state);
    if (!initialized)
        return;

    const Vector3 body_origin = state->get_transform().get_origin();
    // Direct body state returns the COM offset in world axes, relative to the body origin.
    const Vector3 com_global = body_origin + state->get_center_of_mass();
    const Vector3 linear_velocity = state->get_linear_velocity();
    const Vector3 angular_velocity = state->get_angular_velocity();

    // Bounded diagnostic history; no allocations in the physics path.
    const real_t sample_dt = state->get_step();
    const Basis frame = state->get_transform().basis.orthonormalized();
    const Vector3 local_velocity = frame.xform_inv(linear_velocity);
    const real_t beta = std::atan2(local_velocity.x, local_velocity.z);
    const real_t yaw = angular_velocity.dot(frame.get_column(1));
    const bool valid_sample = linear_velocity.is_finite() && angular_velocity.is_finite() &&
        std::isfinite(sample_dt) && sample_dt > real_t{0.0};
    const bool beta_valid = valid_sample && local_velocity.z >= real_t{3.0};
    if (valid_sample && handling_history_valid) {
        handling_sample.yaw_acceleration = (yaw - handling_sample.yaw_rate) / sample_dt;
        handling_sample.lateral_acceleration = (linear_velocity - previous_velocity).dot(frame.get_column(0)) / sample_dt;
        if (beta_valid && handling_sample.sideslip_valid) {
            const real_t derivative = std::clamp<real_t>(std::remainder(beta - handling_sample.sideslip, real_t{2.0} * Math_PI) / sample_dt,
                real_t{-10.0}, real_t{10.0});
            handling_sample.sideslip_trend += (real_t{1.0} - std::exp(-sample_dt / real_t{0.1})) * (derivative - handling_sample.sideslip_trend);
        } else {
            handling_sample.sideslip_trend = 0.0;
        }
    } else {
        handling_sample = HandlingTelemetry();
    }
    handling_sample.sideslip = valid_sample ? beta : real_t{0.0};
    handling_sample.yaw_rate = valid_sample ? yaw : real_t{0.0};
    handling_sample.sideslip_valid = beta_valid;
    previous_velocity = linear_velocity;
    handling_history_valid = valid_sample;

    // Suspension, aerodynamics and ESC precede drivetrain/tire substeps.
    running_gear.update_suspension(this, body_origin, com_global,
                                   linear_velocity, angular_velocity);
    running_gear.apply_aerodynamics(this, linear_velocity, body_origin);
    stability_control.load_parameters(config.is_valid() ? config->get_esc_data() : Ref<ESCData>());
    const StabilityState stability_state = stability_control.get_state(
        state->get_transform().basis, linear_velocity, angular_velocity,
        get_mass(), running_gear.get_axles());
    const Vector3 stability_torque = stability_control.update(stability_state, state->get_step());
    if (stability_torque.length_squared() > real_t{1e-8})
        apply_torque(stability_torque);

    drivetrain.handle_auto_gearbox(get_speed_kph(), brake_input, throttle_input);
    if (drivetrain.is_reverse()) {
        drivetrain.set_throttle(brake_input);
    } else {
        real_t modified_throttle = throttle_input;
        if (tcs_enabled)
            modified_throttle = running_gear.apply_traction_control(throttle_input, get_linear_velocity().length());
        drivetrain.set_throttle(modified_throttle);
    }

    const real_t dt = state->get_step();
    const real_t sub_dt = dt / static_cast<real_t>(substeps);
    engine_reaction_sample = EngineReactionSample();
    engine_chassis_torque = Vector3();

    // Gear selection is evaluated once at frame time. Clutch phase progress is
    // advanced inside each drivetrain substep below.
    drivetrain.update_shifting_logic(dt);

    const real_t wheel_brake = drivetrain.is_reverse() ? throttle_input : brake_input;
    const real_t speed_kph = linear_velocity.length() * real_t{3.6};

    for (int substep = 0; substep < substeps; ++substep) {
        drivetrain.update_clutch_logic(sub_dt, wheel_brake, throttle_input);
        drivetrain.accumulate_engine_torque(sub_dt);

        if (!engine_axis.is_zero_approx()) {
            const VehicleEngine &engine = drivetrain.get_engine();
            const EngineReactionSample sample = engine_reaction.update(
                    engine.get_generated_torque(), engine.get_self_torque(),
                    engine.get_rpm(), engine.get_idle_rpm(), engine.get_redline_rpm(),
                    sub_dt, dt, speed_kph);
            // Accumulate angular impulse; convert to frame-average torque below.
            engine_reaction_sample.reaction_torque += sample.reaction_torque * sub_dt;
            engine_reaction_sample.vibration_torque += sample.vibration_torque * sub_dt;
            engine_reaction_sample.vibration_amplitude += sample.vibration_amplitude * sub_dt;
            engine_reaction_sample.vibration_frequency += sample.vibration_frequency * sub_dt;
            engine_reaction_sample.speed_factor += sample.speed_factor * sub_dt;
        }

        // Current tire and brake reaction torques remain pending on wheel
        // bodies so coupling sees exactly the loads integrated below.
        running_gear.solve_steering_and_tires(
            com_global, linear_velocity, angular_velocity, sub_dt,
            steer_input, wheel_brake, speed_kph, abs_enabled);

        drivetrain.solve_network(sub_dt);

        drivetrain.integrate_bodies(sub_dt);
        running_gear.integrate_wheels(sub_dt);
    }

    running_gear.apply_tire_forces(this, body_origin, substeps);
    if (std::isfinite(dt) && dt > real_t{0.0}) {
        engine_reaction_sample.reaction_torque /= dt;
        engine_reaction_sample.vibration_torque /= dt;
        engine_reaction_sample.vibration_amplitude /= dt;
        engine_reaction_sample.vibration_frequency /= dt;
        engine_reaction_sample.speed_factor /= dt;
        engine_chassis_torque = frame.xform(engine_axis) * engine_reaction_sample.total_torque();
        if (engine_chassis_torque.is_finite() && !engine_chassis_torque.is_zero_approx())
            state->apply_torque(engine_chassis_torque);
    }
}

void Vehicle::set_engine_axis(const Vector3 &value) {
    if (!value.is_finite())
        return;
    // Scale before normalization to avoid overflow with large authored vectors.
    const real_t largest = std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
    engine_axis = largest > real_t{0.0} ? (value / largest).normalized() : Vector3();
}

void Vehicle::set_throttle_input(real_t value) {
    throttle_input = std::clamp(value, real_t{0.0}, real_t{1.0});
}

void Vehicle::set_steer_input(real_t value) {
    steer_input = std::clamp(value, real_t{-1.0}, real_t{1.0});
}

void Vehicle::set_brake_input(real_t value) {
    brake_input = std::clamp(value, real_t{0.0}, real_t{1.0});
}

void Vehicle::set_tcs_enabled(bool value) {
    tcs_enabled = value;
}

void Vehicle::set_config(const Ref<VehicleConfig> &value) {
    config = value;
}

void Vehicle::set_gearbox_automatic(bool value) {
    drivetrain.set_gearbox_automatic(value);
}

bool Vehicle::get_gearbox_automatic() const {
    return drivetrain.get_gearbox_automatic();
}

bool Vehicle::restart() {
    if (Engine::get_singleton()->is_editor_hint())
        return false;

    // Inertness is committed before any validation or setup work.  A failed
    // restart must never leave the previous runtime active.
    initialized = false;
    stability_control.reset();
    engine_reaction.reset();
    engine_reaction_sample = EngineReactionSample();
    engine_chassis_torque = Vector3();
    handling_history_valid = false;
    handling_sample = HandlingTelemetry();
    const Transform3D preserved_transform = get_global_transform();
    const Vector3 preserved_linear_velocity = get_linear_velocity();
    const Vector3 preserved_angular_velocity = get_angular_velocity();

    const bool success = initialize_runtime();

    // Setup does not intentionally respawn a body.  Restore these values even
    // on failure so restart cannot alter the rigid-body state as a side effect.
    set_global_transform(preserved_transform);
    set_linear_velocity(preserved_linear_velocity);
    set_angular_velocity(preserved_angular_velocity);
    return success;
}

void Vehicle::shift_up() {
    drivetrain.shift_up();
}

void Vehicle::shift_down() {
    drivetrain.shift_down();
}

void Vehicle::select_drive() {
    drivetrain.select_drive();
}

void Vehicle::select_neutral() {
    drivetrain.select_neutral();
}

void Vehicle::select_reverse() {
    drivetrain.select_reverse();
}

void Vehicle::set_substeps(int value) {
    substeps = std::clamp(value, 1, 1024);
}

real_t Vehicle::get_speed_kph() const {
    return get_linear_velocity().length() * 3.6;
}

VehicleTelemetrySnapshot Vehicle::get_telemetry_snapshot() const {
    constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);

    const VehicleEngine &engine = drivetrain.get_engine();
    const Gearbox &gearbox = drivetrain.get_gearbox();
    const RotationalBody &drive_shaft = drivetrain.get_driveshaft();

    VehicleTelemetrySnapshot snapshot;
    snapshot.engine_reaction = engine_reaction_sample;
    snapshot.engine_chassis_torque = engine_chassis_torque;
    snapshot.engine_generated_torque = engine.get_generated_torque();
    snapshot.stability = stability_control.get_telemetry();
    snapshot.handling = handling_sample;
    snapshot.driver_steering = steer_input;
    snapshot.engine_rpm = engine.get_rpm();
    snapshot.engine_torque = engine.get_torque();
    snapshot.engine_throttle = throttle_input;
    snapshot.driveshaft_rpm = drive_shaft.get_angular_velocity() * ang_to_rpm;
    snapshot.clutch_engagement = gearbox.get_clutch_engagement();
    const ClutchTelemetry &clutch = drivetrain.get_clutch_telemetry();
    if (clutch.present) {
        snapshot.clutch_requested_torque =
                clutch.requested_engine_torque;
        snapshot.clutch_transmitted_torque =
                clutch.transmitted_torque;
        snapshot.clutch_output_torque = clutch.output_torque;
        snapshot.clutch_slip = clutch.slip;
        snapshot.clutch_slipping = clutch.slipping;
    }
    snapshot.current_gear = gearbox.get_current_gear();
    snapshot.gear_ratio = gearbox.get_effective_ratio();
    snapshot.vehicle_speed_kph = get_speed_kph();
    snapshot.turbo_boost = engine.get_turbo_boost();
    return snapshot;
}

} // namespace godot
