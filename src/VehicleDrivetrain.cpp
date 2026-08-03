#include "VehicleDrivetrain.h"

namespace godot {

VehicleDrivetrain::VehicleDrivetrain() {
    clutch.set_bodies(engine, drive_shaft);
    gearbox.set_bodies(engine, drive_shaft);
}

void VehicleDrivetrain::setup(const Ref<VehicleConfig> &config,
                              const std::vector<Axle *> &setup_axles) {
    engine = VehicleEngine(config->get_engine_data());
    if (config->get_turbo_data().is_valid()) {
        engine.set_turbo(&turbo);
        turbo.configure(config->get_turbo_data());
    }

    clutch.set_bodies(engine, drive_shaft);
    gearbox.set_bodies(engine, drive_shaft);
    gearbox.configure(config->get_gearbox_data());

    shaft_wheels_coupling.load_bodies(
        &drive_shaft, setup_axles, config->get_center_differential_data());
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
