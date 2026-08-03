#pragma once

#include <vector>

#include "godot_cpp/core/defs.hpp"

#include "Drivetrain/ClutchConstraint.h"
#include "Drivetrain/Gearbox.h"
#include "Drivetrain/ShaftWheelsCouplingConstraint.h"
#include "Drivetrain/Turbo.h"
#include "Drivetrain/VehicleEngine.h"
#include "Resources/vehicle_config.h"

namespace godot {

/// Value-owned drivetrain composition and its frame/substep state.
///
/// The Vehicle calls the phase methods in the established order.  This keeps
/// all engine, clutch, gearbox, shaft/coupling, and turbo state out of the
/// Godot-facing composition root without changing frame/substep semantics.
class VehicleDrivetrain {
    VehicleEngine engine;
    Turbo turbo;
    ClutchConstraint clutch;
    Gearbox gearbox;
    ShaftWheelsCouplingConstraint shaft_wheels_coupling;
    RotationalBody drive_shaft;

public:
    VehicleDrivetrain();

    void setup(const Ref<VehicleConfig> &config,
               const std::vector<Axle *> &setup_axles);

    const VehicleEngine &get_engine() const { return engine; }
    const Gearbox &get_gearbox() const { return gearbox; }
    const RotationalBody &get_driveshaft() const { return drive_shaft; }

    void set_throttle(real_t value) { engine.throttle = value; }
    void handle_auto_gearbox(real_t speed_kph, real_t brake_input, real_t throttle_input);
    void update_shifting_logic(real_t dt) { gearbox.update_shifting_logic(dt); }
    int get_current_gear() const { return gearbox.get_current_gear(); }
    bool is_reverse() const { return get_current_gear() < 0; }

    void update_clutch_logic(real_t dt, real_t wheel_brake, real_t throttle_input) {
        gearbox.update_clutch_logic(dt, wheel_brake, throttle_input);
    }
    void accumulate_engine_torque(real_t dt) { engine.accumulate_torque(dt); }

    real_t get_aggregate_inertia() const {
        return shaft_wheels_coupling.get_aggregate_inertia();
    }
    real_t get_predicted_aggregate_angular_velocity(real_t dt) const {
        return shaft_wheels_coupling.get_predicted_aggregate_angular_velocity(dt);
    }
    void solve_clutch(real_t dt) {
        const ClutchSolveInput input{
            gearbox.get_effective_ratio(),
            gearbox.get_clutch_engagement(),
            gearbox.get_clutch_capacity(),
            get_aggregate_inertia(),
            get_predicted_aggregate_angular_velocity(dt),
            engine.get_pending_torque()};
        clutch.solve(dt, input);
    }
    void solve_coupling(real_t dt) { shaft_wheels_coupling.solve(dt); }
    void integrate_bodies(real_t dt) {
        engine.integrate(dt);
        drive_shaft.integrate(dt);
    }

    void shift_up() { gearbox.shift_up(); }
    void shift_down() { gearbox.shift_down(); }
    void select_drive() { gearbox.select_drive(); }
    void select_neutral() { gearbox.select_neutral(); }
    void select_reverse() { gearbox.select_reverse(); }

};

} // namespace godot
