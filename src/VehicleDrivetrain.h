#pragma once

#include <vector>

#include "godot_cpp/core/defs.hpp"
#include "godot_cpp/variant/string.hpp"

#include "Drivetrain/Gearbox.h"
#include "Drivetrain/RotationalNetwork.h"
#include "Drivetrain/Turbo.h"
#include "Drivetrain/VehicleEngine.h"
#include "Resources/vehicle_config.h"

namespace godot {

class Axle;

/// Value-owned drivetrain composition and its frame/substep state.
///
/// The Vehicle calls the phase methods in the established order.  This keeps
/// all engine, clutch, gearbox, shaft/coupling, and turbo state out of the
/// Godot-facing composition root without changing frame/substep semantics.
class VehicleDrivetrain {
    VehicleEngine engine;
    Turbo turbo;
    Gearbox gearbox;
    RotationalNetwork network;
    RotationalBody drive_shaft;

public:
    VehicleDrivetrain();

    bool setup(const Ref<VehicleConfig> &config,
               const std::vector<Axle *> &setup_axles, String &error);

    const VehicleEngine &get_engine() const { return engine; }
    const Gearbox &get_gearbox() const { return gearbox; }
    const RotationalBody &get_driveshaft() const { return drive_shaft; }

    void set_gearbox_automatic(bool value) { gearbox.set_automatic(value); }
    bool get_gearbox_automatic() const { return gearbox.is_automatic(); }

    /// Clear value-owned shaft state before a fresh setup transaction.  The
    /// engine and gearbox are replaced/configured by setup itself.
    void reset_runtime_state() {
        drive_shaft.set_angular_velocity(real_t{0.0});
        drive_shaft.clear_torque();
    }

    void set_throttle(real_t value) { engine.throttle = value; }
    void handle_auto_gearbox(real_t speed_kph, real_t brake_input, real_t throttle_input);
    void update_shifting_logic(real_t dt) { gearbox.update_shifting_logic(dt); }
    int get_current_gear() const { return gearbox.get_current_gear(); }
    bool is_reverse() const { return get_current_gear() < 0; }

    void update_clutch_logic(real_t dt, real_t wheel_brake, real_t throttle_input) {
        gearbox.update_clutch_logic(dt, wheel_brake, throttle_input);
    }
    void accumulate_engine_torque(real_t dt) { engine.accumulate_torque(dt); }

    void solve_network(real_t dt) {
        network.solve(dt, gearbox.get_effective_ratio(),
                gearbox.get_clutch_engagement(), gearbox.get_clutch_capacity());
    }
    const ClutchTelemetry &get_clutch_telemetry() const {
        return network.get_clutch_telemetry();
    }
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
