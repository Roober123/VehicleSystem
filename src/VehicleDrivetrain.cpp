#include "VehicleDrivetrain.h"

#include <array>

#include "axle.h"
#include "wheel.h"

namespace godot {

VehicleDrivetrain::VehicleDrivetrain() {
    gearbox.set_bodies(engine, drive_shaft);
}

bool VehicleDrivetrain::setup(const Ref<VehicleConfig> &config,
                              const std::vector<Axle *> &setup_axles,
                              String &error) {
    error = String();
    if (config.is_null() || config->get_engine_data().is_null() ||
            config->get_gearbox_data().is_null()) {
        error = "drivetrain requires engine_data and gearbox_data";
        return false;
    }
    engine = VehicleEngine(config->get_engine_data());
    if (config->get_turbo_data().is_valid()) {
        engine.set_turbo(&turbo);
        turbo.configure(config->get_turbo_data());
    }

    gearbox.set_bodies(engine, drive_shaft);
    gearbox.configure(config->get_gearbox_data());

    std::array<DrivenAxle, RotationalNetwork::MAX_AXLES> axle_configs{};
    std::size_t driven_count = 0;
    for (Axle *axle : setup_axles) {
        if (axle == nullptr || axle->get_drive_share() <= real_t{0.0})
            continue;
        if (driven_count == RotationalNetwork::MAX_AXLES) {
            error = "drivetrain supports at most eight driven axles";
            return false;
        }
        const auto &wheels = axle->get_wheels();
        if (wheels.size() != 2 || wheels[0] == nullptr || wheels[1] == nullptr) {
            error = "driven axle requires exactly two wheels";
            return false;
        }
        const Ref<DifferentialData> differential = axle->get_differential_data();
        if (differential.is_null()) {
            error = "driven axle requires differential_data";
            return false;
        }
        DrivenAxle &config_entry = axle_configs[driven_count];
        config_entry.left = wheels[0]->get_rotational_body_for_setup();
        config_entry.right = wheels[1]->get_rotational_body_for_setup();
        config_entry.share = axle->get_drive_share();
        config_entry.differential =
                DifferentialSettings::from_resource(differential);
        ++driven_count;
    }
    if (!network.configure(engine, drive_shaft, axle_configs, driven_count)) {
        error = "invalid driven axle configuration (bodies, inertia, shares, or differential policy)";
        return false;
    }
    return true;
}

void VehicleDrivetrain::handle_auto_gearbox(real_t speed_kph,
                                            real_t brake_input,
                                            real_t throttle_input) {
    if (!gearbox.is_automatic() || gearbox.is_shifting())
        return;
    if (speed_kph >= real_t{0.5})
        return;

    const int gear = gearbox.get_current_gear();
    if (gear > 0 && brake_input > real_t{0.3} && throttle_input < real_t{0.05})
        gearbox.select_reverse();
    else if (gear < 0 && throttle_input > real_t{0.1} && brake_input < real_t{0.05})
        gearbox.select_drive();
}

} // namespace godot
