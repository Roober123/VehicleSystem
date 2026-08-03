#include "drivetrain_regression.h"

#include <cmath>
#include <limits>
#include <vector>

#include "godot_cpp/classes/curve.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

#include "Drivetrain/ClutchConstraint.h"
#include "Drivetrain/differential_solver.h"
#include "Drivetrain/Gearbox.h"
#include "Drivetrain/ShaftWheelsCouplingConstraint.h"
#include "Drivetrain/VehicleEngine.h"
#include "Resources/gearbox_data.h"
#include "Resources/differential_data.h"
#include "Resources/suspension_data.h"
#include "Resources/tire_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/vehicle_config.h"
#include "Resources/vehicle_engine_data.h"
#include "axle.h"
#include "vehicle_setup_validation.h"
#include "wheel.h"

namespace godot {
namespace {

constexpr real_t kEpsilon = real_t{1e-4};
constexpr real_t kPi = real_t{3.14159265358979323846};

bool near(real_t actual, real_t expected, real_t tolerance = kEpsilon) {
    return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}

real_t rpm_to_omega(real_t rpm) {
    return rpm * real_t{2.0} * kPi / real_t{60.0};
}

struct TestState {
    int failures = 0;

    void expect(bool condition, const char *label) {
        if (condition)
            return;
        ++failures;
        UtilityFunctions::printerr("[drivetrain-regression] FAIL: ", label);
    }
};

Ref<Curve> make_flat_curve() {
    Ref<Curve> curve = memnew(Curve);
    curve->add_point(Vector2(0.0, 1.0));
    curve->add_point(Vector2(1.0, 1.0));
    return curve;
}

Ref<VehicleEngineData> make_engine_data() {
    Ref<VehicleEngineData> data = memnew(VehicleEngineData);
    data->set_torque_curve(make_flat_curve());
    data->set_idle_rpm(real_t{1000.0});
    data->set_redline_rpm(real_t{4000.0});
    data->set_inertia(real_t{2.0});
    data->set_max_torque(real_t{100.0});
    data->set_engine_drag(real_t{0.5});
    data->set_engine_braking(real_t{0.2});
    return data;
}

Ref<TireData> make_tire_data(real_t radius = real_t{0.3}) {
    Ref<TireData> data = memnew(TireData);
    data->set_radius(radius);
    data->set_drag(real_t{0.0});
    data->set_forward_friction_curve(make_flat_curve());
    data->set_lateral_friction_curve(make_flat_curve());
    return data;
}

Ref<DifferentialData> make_differential_data(
        DifferentialData::Mode mode,
        real_t preload = real_t{0.0},
        real_t power_lock = real_t{0.0},
        real_t coast_lock = real_t{0.0},
        real_t slip_gain = real_t{0.0},
        real_t max_lock = real_t{100.0}) {
    Ref<DifferentialData> data = memnew(DifferentialData);
    data->set_mode(mode);
    data->set_preload_torque(preload);
    data->set_power_lock_ratio(power_lock);
    data->set_coast_lock_ratio(coast_lock);
    data->set_slip_sensitive_gain(slip_gain);
    data->set_max_lock_torque(max_lock);
    return data;
}

Axle *make_axle(real_t drive_share, real_t radius = real_t{0.3},
                const Ref<DifferentialData> &differential_data =
                        Ref<DifferentialData>()) {
    Axle *axle = memnew(Axle);
    axle->set_drive_share(drive_share);
    if (drive_share > real_t{0.0}) {
        axle->set_differential_data(differential_data.is_valid()
                ? differential_data
                : make_differential_data(DifferentialData::OPEN));
    }
    axle->set_tire_data(make_tire_data(radius));
    axle->add_child(memnew(Wheel));
    axle->add_child(memnew(Wheel));
    axle->_ready();
    return axle;
}

void set_wheel_angular_velocity(Wheel *wheel, real_t angular_velocity,
                                real_t dt = real_t{0.1}) {
    // make_tire_data(0.3) gives Wheel's rotational body an inertia of 0.9.
    // Use only public Wheel operations so the fixture remains independent of
    // the private body representation.
    constexpr real_t wheel_inertia = real_t{0.9};
    wheel->add_drive_torque(angular_velocity * wheel_inertia / dt);
    wheel->integrate_rotation(dt);
}

Ref<GearboxData> make_gearbox_data(bool automatic = false) {
    Ref<GearboxData> data = memnew(GearboxData);
    PackedFloat64Array ratios;
    ratios.push_back(2.0);
    ratios.push_back(1.0);
    data->set_gear_ratios(ratios);
    data->set_final_drive(real_t{3.0});
    data->set_reverse_ratio(real_t{-2.0});
    data->set_clutch_max_torque(real_t{100.0});
    data->set_shift_time(real_t{0.1});
    data->set_upshift_rpm(real_t{0.8});
    data->set_downshift_rpm(real_t{0.25});
    data->set_auto_mode(automatic);
    data->set_driveshaft_drag(real_t{0.0});
    return data;
}

Ref<VehicleConfig> make_valid_config() {
    Ref<VehicleConfig> config = memnew(VehicleConfig);
    config->set_engine_data(make_engine_data());
    config->set_gearbox_data(make_gearbox_data());
    config->set_suspension_data(memnew(SuspensionData));
    config->set_aero_data(memnew(VehicleAerodynamicsData));
    return config;
}

void test_rotational_body(TestState &state) {
    RotationalBody body;
    body.set_inertia(real_t{2.0});
    body.set_drag(real_t{0.5});
    body.set_angular_velocity(real_t{3.0});
    body.add_torque(real_t{4.0});

    const real_t predicted = body.predict_angular_velocity(real_t{0.25});
    state.expect(near(predicted, real_t{3.3125}),
                 "RotationalBody drag-aware prediction");
    body.integrate(real_t{0.25});
    state.expect(near(body.get_angular_velocity(), predicted),
                 "RotationalBody integration matches prediction and clears torque");
    state.expect(near(body.get_pending_torque(), real_t{0.0}),
                 "RotationalBody integration clears pending torque");
}

void test_clutch_constraint(TestState &state) {
    struct Case { real_t ratio; real_t engine_omega; real_t output_omega; };
    const Case cases[] = {
        {real_t{2.0}, real_t{6.0}, real_t{2.0}},
        {real_t{-2.0}, real_t{-6.0}, real_t{2.0}},
    };
    for (const Case &test : cases) {
        RotationalBody engine;
        RotationalBody output;
        engine.set_inertia(real_t{1.0});
        output.set_inertia(real_t{4.0});
        engine.set_angular_velocity(test.engine_omega);
        output.set_angular_velocity(test.output_omega);
        ClutchConstraint clutch;
        clutch.set_bodies(engine, output);
        clutch.solve(real_t{0.1}, ClutchSolveInput{
            test.ratio, real_t{1.0}, real_t{100.0}, real_t{4.0},
            test.output_omega, real_t{0.0}});
        const real_t engine_torque = engine.get_pending_torque();
        const real_t output_torque = output.get_pending_torque();
        engine.integrate(real_t{0.1});
        output.integrate(real_t{0.1});
        const real_t slip = engine.get_angular_velocity() -
                            test.ratio * output.get_angular_velocity();
        state.expect(std::abs(slip) <= real_t{2.0} * kEpsilon &&
                         std::abs(engine_torque) > real_t{0.0} &&
                         near(output_torque, -engine_torque * test.ratio),
                     "ClutchConstraint forward/reverse synchronization");
    }

    RotationalBody capacity_engine;
    RotationalBody capacity_output;
    capacity_engine.set_inertia(real_t{1.0});
    capacity_output.set_inertia(real_t{1.0});
    capacity_engine.set_angular_velocity(real_t{2.0});
    ClutchConstraint limited;
    limited.set_bodies(capacity_engine, capacity_output);
    limited.solve(real_t{0.1}, ClutchSolveInput{
        real_t{1.0}, real_t{0.5}, real_t{2.0}, real_t{1.0},
        real_t{0.0}, real_t{0.0}});
    state.expect(std::abs(capacity_engine.get_pending_torque()) <= real_t{1.0} + kEpsilon &&
                     near(capacity_output.get_pending_torque(),
                          -capacity_engine.get_pending_torque()),
                 "ClutchConstraint engagement capacity bound");

    RotationalBody neutral_engine;
    RotationalBody neutral_output;
    neutral_engine.add_torque(real_t{7.0});
    neutral_output.add_torque(real_t{-3.0});
    ClutchConstraint neutral;
    neutral.set_bodies(neutral_engine, neutral_output);
    neutral.solve(real_t{0.1}, ClutchSolveInput{
        real_t{0.0}, real_t{1.0}, real_t{100.0}, real_t{1.0},
        real_t{0.0}, real_t{100.0}});
    state.expect(near(neutral_engine.get_pending_torque(), real_t{7.0}) &&
                     near(neutral_output.get_pending_torque(), real_t{-3.0}),
                 "ClutchConstraint neutral no-op");
}

void advance_shift(Gearbox &gearbox, real_t dt, int steps) {
    for (int i = 0; i < steps; ++i)
        gearbox.update_clutch_logic(dt, real_t{0.0}, real_t{1.0});
}

void test_gearbox(TestState &state) {
    VehicleEngine engine(make_engine_data());
    engine.set_angular_velocity(rpm_to_omega(real_t{2000.0}));
    RotationalBody shaft;
    Gearbox gearbox;
    gearbox.set_bodies(engine, shaft);
    Ref<GearboxData> gearbox_data = make_gearbox_data(false);
    gearbox.configure(gearbox_data);

    state.expect(gearbox.get_current_gear() == 0 && near(gearbox.get_effective_ratio(), real_t{0.0}),
                 "Gearbox configuration starts neutral");
    gearbox.shift_up();
    gearbox.update_shifting_logic(real_t{0.01});
    gearbox.update_clutch_logic(real_t{0.01}, real_t{0.0}, real_t{1.0});
    state.expect(gearbox.is_shifting() && gearbox.get_clutch_engagement() < real_t{1.0},
                 "Gearbox manual shift enters disengaging phase");
    advance_shift(gearbox, real_t{0.01}, 14);
    state.expect(gearbox.get_current_gear() == 1 && !gearbox.is_shifting() &&
                     near(gearbox.get_effective_ratio(), real_t{6.0}) &&
                     near(gearbox.get_clutch_engagement(), real_t{1.0}),
                 "Gearbox manual shift completes all phases");

    gearbox.shift_up();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 14);
    state.expect(gearbox.get_current_gear() == 2 && near(gearbox.get_effective_ratio(), real_t{3.0}),
                 "Gearbox second manual selection");

    gearbox.select_neutral();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 14);
    state.expect(gearbox.get_current_gear() == 0 && near(gearbox.get_effective_ratio(), real_t{0.0}),
                 "Gearbox neutral selection");

    gearbox.select_reverse();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 14);
    state.expect(gearbox.get_current_gear() == -1 && near(gearbox.get_effective_ratio(), real_t{-6.0}),
                 "Gearbox reverse selection and signed ratio");

    VehicleEngine automatic_engine(make_engine_data());
    automatic_engine.throttle = real_t{1.0};
    RotationalBody automatic_shaft;
    Gearbox automatic;
    automatic.set_bodies(automatic_engine, automatic_shaft);
    automatic.configure(make_gearbox_data(true));
    automatic.update_shifting_logic(real_t{0.01});
    state.expect(automatic.is_shifting(), "Gearbox automatic neutral-to-drive selection");
    advance_shift(automatic, real_t{0.01}, 14);
    automatic_shaft.set_angular_velocity(real_t{100.0});
    for (int i = 0; i < 25; ++i)
        automatic.update_shifting_logic(real_t{0.01});
    state.expect(automatic.is_shifting(), "Gearbox automatic upshift threshold");
}

void test_coupling(TestState &state) {
    auto carrier = [](const Axle *axle) {
        const auto &wheels = axle->get_wheels();
        return real_t{0.5} * (wheels[0]->get_angular_velocity() +
                             wheels[1]->get_angular_velocity());
    };
    auto relative = [](const Axle *axle) {
        const auto &wheels = axle->get_wheels();
        return wheels[0]->get_angular_velocity() -
               wheels[1]->get_angular_velocity();
    };
    auto axle_momentum = [](const Axle *axle) {
        const auto &wheels = axle->get_wheels();
        constexpr real_t wheel_inertia = real_t{0.9};
        return wheel_inertia * (wheels[0]->get_angular_velocity() +
                                wheels[1]->get_angular_velocity());
    };
    auto axle_energy = [](const Axle *axle) {
        const auto &wheels = axle->get_wheels();
        constexpr real_t wheel_inertia = real_t{0.9};
        return real_t{0.5} * wheel_inertia *
               (std::pow(wheels[0]->get_angular_velocity(), 2) +
                std::pow(wheels[1]->get_angular_velocity(), 2));
    };

    // One driven axle: OPEN couples only the carrier to the shaft and leaves
    // its left/right speed difference intact. The non-driven axle is ignored.
    RotationalBody shaft;
    shaft.set_inertia(real_t{1.0});
    shaft.set_angular_velocity(real_t{4.0});
    Axle *open_axle = make_axle(real_t{1.0}, real_t{0.3},
                                make_differential_data(DifferentialData::OPEN));
    Axle *free_axle = make_axle(real_t{0.0});
    const auto &open_wheels = open_axle->get_wheels();
    const auto &free_wheels = free_axle->get_wheels();
    set_wheel_angular_velocity(open_wheels[0], real_t{8.0});
    set_wheel_angular_velocity(open_wheels[1], real_t{2.0});
    set_wheel_angular_velocity(free_wheels[0], real_t{5.0});
    set_wheel_angular_velocity(free_wheels[1], real_t{5.0});
    const real_t initial_open_momentum = shaft.get_inertia() * shaft.get_angular_velocity() +
        axle_momentum(open_axle);
    const real_t initial_open_energy = real_t{0.5} * shaft.get_inertia() *
        shaft.get_angular_velocity() * shaft.get_angular_velocity() +
        axle_energy(open_axle);
    ShaftWheelsCouplingConstraint single_coupling;
    single_coupling.load_bodies(&shaft, std::vector<Axle *>{open_axle, free_axle},
                                Ref<DifferentialData>());
    state.expect(near(single_coupling.get_aggregate_inertia(), real_t{2.8}),
                 "Single axle coupling uses shaft plus carrier inertia");
    single_coupling.solve(real_t{0.1});
    shaft.integrate(real_t{0.1});
    open_axle->integrate(real_t{0.1});
    free_axle->integrate(real_t{0.1});
    const real_t final_open_energy = real_t{0.5} * shaft.get_inertia() *
        shaft.get_angular_velocity() * shaft.get_angular_velocity() +
        axle_energy(open_axle);
    state.expect(near(shaft.get_angular_velocity(), real_t{4.642857142857143}) &&
                     near(carrier(open_axle), shaft.get_angular_velocity()) &&
                     near(relative(open_axle), real_t{6.0}),
                 "Single axle OPEN reaches shaft carrier without spool behavior");
    state.expect(near(free_wheels[0]->get_angular_velocity(), real_t{5.0}) &&
                     near(free_wheels[1]->get_angular_velocity(), real_t{5.0}) &&
                     near(shaft.get_inertia() * shaft.get_angular_velocity() +
                              axle_momentum(open_axle), initial_open_momentum, real_t{2e-3}) &&
                     final_open_energy <= initial_open_energy + kEpsilon,
                 "Single axle excludes non-driven wheels and preserves invariants");

    // Axle-local modes are exercised independently of shaft projection.
    Axle *locked_axle = make_axle(real_t{1.0}, real_t{0.3},
                                  make_differential_data(DifferentialData::LOCKED));
    Axle *lsd_axle = make_axle(real_t{1.0}, real_t{0.3},
                               make_differential_data(DifferentialData::LIMITED_SLIP,
                                                      real_t{2.0}, real_t{0.25},
                                                      real_t{0.25}, real_t{0.0},
                                                      real_t{20.0}));
    set_wheel_angular_velocity(locked_axle->get_wheels()[0], real_t{8.0});
    set_wheel_angular_velocity(locked_axle->get_wheels()[1], real_t{2.0});
    set_wheel_angular_velocity(lsd_axle->get_wheels()[0], real_t{8.0});
    set_wheel_angular_velocity(lsd_axle->get_wheels()[1], real_t{2.0});
    locked_axle->get_differential().solve_relative(real_t{0.1});
    lsd_axle->get_differential().add_carrier_torque(real_t{40.0});
    lsd_axle->get_differential().solve_relative(real_t{0.1});
    locked_axle->integrate(real_t{0.1});
    lsd_axle->integrate(real_t{0.1});
    state.expect(near(carrier(locked_axle), real_t{5.0}) &&
                     near(relative(locked_axle), real_t{0.0}),
                 "Axle LOCKED reaches equal wheel speed");
    state.expect(std::abs(relative(lsd_axle)) < real_t{6.0} &&
                     relative(lsd_axle) > real_t{0.0},
                 "Axle LIMITED_SLIP reduces bounded wheel slip without crossing");
    state.expect(axle_momentum(locked_axle) > real_t{0.0} &&
                     std::isfinite(axle_energy(locked_axle)) &&
                     std::isfinite(axle_energy(lsd_axle)),
                 "Axle differential outputs remain finite and energized physically");

    // AWD with 40/60 shares. Choose shaft speed equal to the weighted carrier
    // average so the primary constraint is idle and OPEN center behavior is
    // observed directly: front/rear relative speed remains nonzero.
    RotationalBody awd_shaft;
    awd_shaft.set_inertia(real_t{1.0});
    awd_shaft.set_angular_velocity(real_t{3.2});
    Axle *front = make_axle(real_t{0.4}, real_t{0.3},
                            make_differential_data(DifferentialData::OPEN));
    Axle *rear = make_axle(real_t{0.6}, real_t{0.3},
                           make_differential_data(DifferentialData::OPEN));
    set_wheel_angular_velocity(front->get_wheels()[0], real_t{8.0});
    set_wheel_angular_velocity(front->get_wheels()[1], real_t{2.0});
    set_wheel_angular_velocity(rear->get_wheels()[0], real_t{4.0});
    set_wheel_angular_velocity(rear->get_wheels()[1], real_t{0.0});
    const real_t initial_awd_momentum = awd_shaft.get_inertia() *
        awd_shaft.get_angular_velocity() + axle_momentum(front) + axle_momentum(rear);
    ShaftWheelsCouplingConstraint awd_open;
    awd_open.load_bodies(&awd_shaft, std::vector<Axle *>{front, rear},
                         make_differential_data(DifferentialData::OPEN));
    awd_open.solve(real_t{0.1});
    awd_shaft.integrate(real_t{0.1});
    front->integrate(real_t{0.1});
    rear->integrate(real_t{0.1});
    const real_t weighted_open = real_t{0.4} * carrier(front) +
                                 real_t{0.6} * carrier(rear);
    state.expect(near(weighted_open, awd_shaft.get_angular_velocity()) &&
                     std::abs(carrier(front) - carrier(rear)) > real_t{2.0} &&
                     near(relative(front), real_t{6.0}) &&
                     near(relative(rear), real_t{4.0}),
                 "AWD OPEN center preserves weighted shaft constraint and relative speed");
    state.expect(near(awd_shaft.get_inertia() * awd_shaft.get_angular_velocity() +
                          axle_momentum(front) + axle_momentum(rear),
                      initial_awd_momentum, real_t{2e-3}),
                 "AWD OPEN center conserves momentum");

    Axle *locked_front = make_axle(real_t{0.4}, real_t{0.3},
                                   make_differential_data(DifferentialData::LOCKED));
    Axle *locked_rear = make_axle(real_t{0.6}, real_t{0.3},
                                  make_differential_data(DifferentialData::LOCKED));
    RotationalBody locked_shaft;
    locked_shaft.set_inertia(real_t{1.0});
    locked_shaft.set_angular_velocity(real_t{3.2});
    set_wheel_angular_velocity(locked_front->get_wheels()[0], real_t{8.0});
    set_wheel_angular_velocity(locked_front->get_wheels()[1], real_t{2.0});
    set_wheel_angular_velocity(locked_rear->get_wheels()[0], real_t{4.0});
    set_wheel_angular_velocity(locked_rear->get_wheels()[1], real_t{0.0});
    const real_t initial_locked_energy = real_t{0.5} * locked_shaft.get_inertia() *
        locked_shaft.get_angular_velocity() * locked_shaft.get_angular_velocity() +
        axle_energy(locked_front) + axle_energy(locked_rear);
    ShaftWheelsCouplingConstraint awd_locked;
    awd_locked.load_bodies(&locked_shaft,
                           std::vector<Axle *>{locked_front, locked_rear},
                           make_differential_data(DifferentialData::LOCKED));
    awd_locked.solve(real_t{0.1});
    locked_shaft.integrate(real_t{0.1});
    locked_front->integrate(real_t{0.1});
    locked_rear->integrate(real_t{0.1});
    const real_t final_locked_energy = real_t{0.5} * locked_shaft.get_inertia() *
        locked_shaft.get_angular_velocity() * locked_shaft.get_angular_velocity() +
        axle_energy(locked_front) + axle_energy(locked_rear);
    state.expect(near(carrier(locked_front), carrier(locked_rear), real_t{2e-3}) &&
                     near(locked_shaft.get_angular_velocity(), carrier(locked_front), real_t{2e-3}) &&
                     near(relative(locked_front), real_t{0.0}) &&
                     near(relative(locked_rear), real_t{0.0}) &&
                     final_locked_energy <= initial_locked_energy + kEpsilon,
                 "AWD LOCKED center and axle differentials enforce equality");

    Axle *lsd_front = make_axle(real_t{0.4}, real_t{0.3},
                                make_differential_data(DifferentialData::OPEN));
    Axle *lsd_rear = make_axle(real_t{0.6}, real_t{0.3},
                               make_differential_data(DifferentialData::OPEN));
    RotationalBody lsd_shaft;
    lsd_shaft.set_inertia(real_t{1.0});
    lsd_shaft.set_angular_velocity(real_t{3.2});
    set_wheel_angular_velocity(lsd_front->get_wheels()[0], real_t{8.0});
    set_wheel_angular_velocity(lsd_front->get_wheels()[1], real_t{2.0});
    set_wheel_angular_velocity(lsd_rear->get_wheels()[0], real_t{4.0});
    set_wheel_angular_velocity(lsd_rear->get_wheels()[1], real_t{0.0});
    const real_t initial_lsd_center_slip = carrier(lsd_front) - carrier(lsd_rear);
    const real_t initial_lsd_energy = real_t{0.5} * lsd_shaft.get_inertia() *
        lsd_shaft.get_angular_velocity() * lsd_shaft.get_angular_velocity() +
        axle_energy(lsd_front) + axle_energy(lsd_rear);
    ShaftWheelsCouplingConstraint awd_lsd;
    awd_lsd.load_bodies(&lsd_shaft, std::vector<Axle *>{lsd_front, lsd_rear},
                        make_differential_data(DifferentialData::LIMITED_SLIP,
                                               real_t{4.0}, real_t{0.0}, real_t{0.0},
                                               real_t{0.0}, real_t{20.0}));
    awd_lsd.solve(real_t{0.1});
    lsd_shaft.integrate(real_t{0.1});
    lsd_front->integrate(real_t{0.1});
    lsd_rear->integrate(real_t{0.1});
    const real_t final_lsd_center_slip = carrier(lsd_front) - carrier(lsd_rear);
    const real_t final_lsd_energy = real_t{0.5} * lsd_shaft.get_inertia() *
        lsd_shaft.get_angular_velocity() * lsd_shaft.get_angular_velocity() +
        axle_energy(lsd_front) + axle_energy(lsd_rear);
    state.expect(std::abs(final_lsd_center_slip) < std::abs(initial_lsd_center_slip) &&
                     initial_lsd_center_slip * final_lsd_center_slip >= -kEpsilon &&
                     std::isfinite(final_lsd_center_slip) &&
                     final_lsd_energy <= initial_lsd_energy + kEpsilon,
                 "AWD LIMITED_SLIP center reduces bounded carrier slip without crossing");
    state.expect(near(real_t{0.4} + real_t{0.6}, real_t{1.0}) &&
                     near(relative(lsd_front), real_t{6.0}) &&
                     near(relative(lsd_rear), real_t{4.0}),
                 "AWD LIMITED_SLIP keeps normalized torque shares and axle differentials");

    // A pending shaft torque is routed by normalized 40/60 shares and excludes
    // the free axle. Equal weighted carrier state makes the expected torque
    // increments directly observable after one integration step.
    RotationalBody route_shaft;
    route_shaft.set_inertia(real_t{1.0});
    route_shaft.set_angular_velocity(real_t{3.2});
    Axle *route_front = make_axle(real_t{0.4});
    Axle *route_rear = make_axle(real_t{0.6});
    Axle *route_free = make_axle(real_t{0.0});
    set_wheel_angular_velocity(route_front->get_wheels()[0], real_t{5.0});
    set_wheel_angular_velocity(route_front->get_wheels()[1], real_t{5.0});
    set_wheel_angular_velocity(route_rear->get_wheels()[0], real_t{5.0});
    set_wheel_angular_velocity(route_rear->get_wheels()[1], real_t{5.0});
    set_wheel_angular_velocity(route_free->get_wheels()[0], real_t{7.0});
    set_wheel_angular_velocity(route_free->get_wheels()[1], real_t{7.0});
    ShaftWheelsCouplingConstraint route_coupling;
    route_coupling.load_bodies(&route_shaft,
                               std::vector<Axle *>{route_front, route_rear, route_free},
                               make_differential_data(DifferentialData::OPEN));
    const real_t route_initial_momentum = route_shaft.get_inertia() *
        route_shaft.get_angular_velocity() + axle_momentum(route_front) +
        axle_momentum(route_rear);
    route_shaft.add_torque(real_t{12.0});
    route_coupling.solve(real_t{0.1});
    route_shaft.integrate(real_t{0.1});
    route_front->integrate(real_t{0.1});
    route_rear->integrate(real_t{0.1});
    route_free->integrate(real_t{0.1});
    const real_t front_increment = carrier(route_front) - real_t{5.0};
    const real_t rear_increment = carrier(route_rear) - real_t{5.0};
    state.expect(near(front_increment / rear_increment, real_t{2.0 / 3.0}, real_t{2e-3}) &&
                     near(route_shaft.get_inertia() * route_shaft.get_angular_velocity() +
                              axle_momentum(route_front) + axle_momentum(route_rear),
                          route_initial_momentum + real_t{1.2}, real_t{2e-3}) &&
                     near(route_free->get_wheels()[0]->get_angular_velocity(), real_t{7.0}) &&
                     near(route_free->get_wheels()[1]->get_angular_velocity(), real_t{7.0}),
                 "AWD torque route follows 40/60 shares and excludes free axle");

    memdelete(open_axle);
    memdelete(free_axle);
    memdelete(locked_axle);
    memdelete(lsd_axle);
    memdelete(front);
    memdelete(rear);
    memdelete(locked_front);
    memdelete(locked_rear);
    memdelete(lsd_front);
    memdelete(lsd_rear);
    memdelete(route_front);
    memdelete(route_rear);
    memdelete(route_free);
}

void test_differential_solver(TestState &state) {
    using Mode = DifferentialData::Mode;
    using Input = DifferentialSolver::Input;

    const Input base{
        real_t{2.0}, real_t{6.0}, real_t{8.0}, real_t{2.0},
        real_t{0.0}, real_t{3.5}, real_t{0.1}};
    constexpr real_t relative_inertia = real_t{1.5};
    constexpr real_t ideal_impulse = real_t{-9.0};
    constexpr real_t initial_momentum = real_t{28.0};
    constexpr real_t initial_energy = real_t{76.0};

    auto apply_impulse = [](const Input &input, real_t impulse,
                            real_t &left_velocity, real_t &right_velocity) {
        left_velocity = input.left_free_velocity + impulse / input.left_inertia;
        right_velocity = input.right_free_velocity - impulse / input.right_inertia;
    };
    auto momentum = [](const Input &input, real_t left_velocity,
                       real_t right_velocity) {
        return input.left_inertia * left_velocity +
               input.right_inertia * right_velocity;
    };
    auto energy = [](const Input &input, real_t left_velocity,
                     real_t right_velocity) {
        return real_t{0.5} * input.left_inertia * left_velocity * left_velocity +
               real_t{0.5} * input.right_inertia * right_velocity * right_velocity;
    };

    Ref<DifferentialData> default_data = memnew(DifferentialData);
    DifferentialSolver default_solver(**default_data);
    const DifferentialSolver::Snapshot &default_snapshot =
            default_solver.get_snapshot();
    state.expect(default_snapshot.mode == Mode::OPEN &&
                     near(default_snapshot.preload_torque, real_t{25.0}) &&
                     near(default_snapshot.power_lock_ratio, real_t{0.35}) &&
                     near(default_snapshot.coast_lock_ratio, real_t{0.15}) &&
                     near(default_snapshot.slip_sensitive_gain, real_t{2.0}) &&
                     near(default_snapshot.max_lock_torque, real_t{250.0}),
                 "DifferentialData defaults snapshot into DifferentialSolver");
    state.expect(near(default_solver.solve(base), real_t{0.0}),
                 "DifferentialData OPEN default remains relative no-op");
    default_data->set_mode(Mode::LIMITED_SLIP);
    default_solver.set_snapshot(DifferentialSolver::Snapshot(**default_data));
    const real_t default_lsd_impulse = default_solver.solve(base);
    state.expect(std::isfinite(default_lsd_impulse) &&
                     std::abs(default_lsd_impulse) > real_t{0.0} &&
                     std::abs(default_lsd_impulse) <=
                         default_snapshot.max_lock_torque * base.dt + kEpsilon &&
                     default_lsd_impulse < real_t{0.0},
                 "DifferentialData defaults switch to bounded LIMITED_SLIP");

    DifferentialSolver open(DifferentialSolver::Snapshot(Mode::OPEN));
    const real_t open_impulse = open.solve(base);
    real_t open_left = 0.0;
    real_t open_right = 0.0;
    apply_impulse(base, open_impulse, open_left, open_right);
    state.expect(near(open_impulse, real_t{0.0}) &&
                     near(open_left, base.left_free_velocity) &&
                     near(open_right, base.right_free_velocity) &&
                     near(momentum(base, open_left, open_right), initial_momentum) &&
                     near(energy(base, open_left, open_right), initial_energy),
                 "DifferentialSolver OPEN preserves relative motion and energy");

    DifferentialSolver locked(DifferentialSolver::Snapshot(Mode::LOCKED));
    const real_t locked_impulse = locked.solve(base);
    state.expect(near(locked_impulse, ideal_impulse) &&
                     near(-ideal_impulse / base.dt, real_t{90.0}),
                 "DifferentialSolver LOCKED uses ideal relative impulse");
    real_t locked_left = 0.0;
    real_t locked_right = 0.0;
    apply_impulse(base, locked_impulse, locked_left, locked_right);
    state.expect(near(locked_left, real_t{3.5}) &&
                     near(locked_right, real_t{3.5}) &&
                     near(momentum(base, locked_left, locked_right), initial_momentum),
                 "DifferentialSolver LOCKED equalizes unequal inertias conservatively");

    DifferentialSolver preload(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{10.0}, real_t{0.0}, real_t{0.0},
            real_t{0.0}, real_t{100.0}));
    state.expect(near(preload.solve(base), real_t{-1.0}),
                 "DifferentialSolver LSD preload torque bounds impulse");

    DifferentialSolver power(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{0.0}, real_t{0.5}, real_t{0.0},
            real_t{0.0}, real_t{100.0}));
    Input power_input = base;
    power_input.transmitted_torque = real_t{40.0};
    state.expect(near(power.solve(power_input), real_t{-2.0}),
                 "DifferentialSolver LSD uses power lock ratio");

    DifferentialSolver coast(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{0.0}, real_t{0.0}, real_t{0.25},
            real_t{0.0}, real_t{100.0}));
    Input coast_input = base;
    coast_input.transmitted_torque = real_t{-40.0};
    state.expect(near(coast.solve(coast_input), real_t{-1.0}),
                 "DifferentialSolver LSD uses coast lock ratio");

    DifferentialSolver slip_gain(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{0.0}, real_t{0.0}, real_t{0.0},
            real_t{2.0}, real_t{100.0}));
    state.expect(near(slip_gain.solve(base), real_t{-1.2}),
                 "DifferentialSolver LSD adds slip-sensitive capacity");

    DifferentialSolver max_capacity(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{100.0}, real_t{2.0}, real_t{2.0},
            real_t{10.0}, real_t{3.0}));
    const real_t capped_impulse = max_capacity.solve(power_input);
    state.expect(near(capped_impulse, real_t{-0.3}) &&
                     std::abs(capped_impulse) / base.dt <= real_t{3.0} + kEpsilon,
                 "DifferentialSolver LSD obeys maximum lock torque");

    Input reverse_drive = power_input;
    reverse_drive.transmitted_torque = real_t{-40.0};
    reverse_drive.carrier_velocity = real_t{-3.5};
    state.expect(near(power.solve(reverse_drive), real_t{-2.0}),
                 "DifferentialSolver LSD classifies negative-torque reverse drive as power");

    Input reverse_slip = coast_input;
    reverse_slip.left_free_velocity = real_t{2.0};
    reverse_slip.right_free_velocity = real_t{8.0};
    const real_t reverse_impulse = coast.solve(reverse_slip);
    real_t reverse_left = 0.0;
    real_t reverse_right = 0.0;
    apply_impulse(reverse_slip, reverse_impulse, reverse_left, reverse_right);
    state.expect(near(reverse_impulse, real_t{1.0}) &&
                     near(reverse_left, real_t{2.5}) &&
                     near(reverse_right, real_t{7.833333333333333}) &&
                     reverse_left - reverse_right < real_t{0.0},
                 "DifferentialSolver LSD reverses impulse with slip direction");

    DifferentialSolver combined(DifferentialSolver::Snapshot(
            Mode::LIMITED_SLIP, real_t{5.0}, real_t{0.5}, real_t{0.25},
            real_t{1.0}, real_t{100.0}));
    const real_t combined_impulse = combined.solve(power_input);
    real_t combined_left = 0.0;
    real_t combined_right = 0.0;
    apply_impulse(base, combined_impulse, combined_left, combined_right);
    const real_t initial_slip = base.left_free_velocity - base.right_free_velocity;
    const real_t final_slip = combined_left - combined_right;
    state.expect(near(combined_impulse, real_t{-3.1}) &&
                     std::abs(final_slip) < std::abs(initial_slip) &&
                     initial_slip * final_slip >= -kEpsilon &&
                     near(momentum(base, combined_left, combined_right), initial_momentum) &&
                     energy(base, combined_left, combined_right) <=
                         initial_energy + kEpsilon,
                 "DifferentialSolver LSD preserves momentum and dissipates slip energy");
    state.expect(near(relative_inertia,
                      (base.left_inertia * base.right_inertia) /
                          (base.left_inertia + base.right_inertia)) &&
                     near((base.left_inertia * base.left_free_velocity +
                           base.right_inertia * base.right_free_velocity) /
                              (base.left_inertia + base.right_inertia),
                          real_t{3.5}),
                 "DifferentialSolver relative effective inertia and carrier are stable");

    Input invalid = base;
    invalid.dt = real_t{0.0};
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects zero timestep");
    invalid.dt = real_t{-0.1};
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects negative timestep");
    invalid.dt = std::numeric_limits<real_t>::quiet_NaN();
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects non-finite timestep");
    invalid = base;
    invalid.left_inertia = real_t{0.0};
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects zero left inertia");
    invalid = base;
    invalid.right_inertia = real_t{-1.0};
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects negative right inertia");
    invalid = base;
    invalid.left_free_velocity = std::numeric_limits<real_t>::quiet_NaN();
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects non-finite output velocity");
    invalid = base;
    invalid.transmitted_torque = std::numeric_limits<real_t>::infinity();
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects non-finite transmitted torque");
    invalid = base;
    invalid.carrier_velocity = std::numeric_limits<real_t>::quiet_NaN();
    state.expect(near(combined.solve(invalid), real_t{0.0}),
                 "DifferentialSolver rejects non-finite carrier velocity");
    DifferentialSolver::Snapshot invalid_snapshot(
            static_cast<Mode>(99), real_t{0.0}, real_t{0.0}, real_t{0.0},
            real_t{0.0}, real_t{100.0});
    DifferentialSolver invalid_solver(invalid_snapshot);
    state.expect(near(invalid_solver.solve(base), real_t{0.0}),
                 "DifferentialSolver rejects invalid mode");
    DifferentialSolver::Snapshot nan_snapshot(
            Mode::LIMITED_SLIP,
            std::numeric_limits<real_t>::quiet_NaN(), real_t{0.0}, real_t{0.0},
            real_t{0.0}, real_t{100.0});
    DifferentialSolver nan_solver(nan_snapshot);
    state.expect(near(nan_solver.solve(base), real_t{0.0}),
                 "DifferentialSolver rejects non-finite configuration");
}

void test_engine(TestState &state) {
    VehicleEngine engine(make_engine_data());
    engine.set_angular_velocity(rpm_to_omega(real_t{700.0}));
    engine.throttle = real_t{0.0};
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() > real_t{0.0}, "VehicleEngine idle controller torque");

    engine.clear_torque();
    engine.set_angular_velocity(rpm_to_omega(real_t{2000.0}));
    engine.throttle = real_t{1.0};
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() > real_t{0.0} &&
                     near(engine.get_turbo_boost(), real_t{0.0}) &&
                     near(engine.get_torque(), real_t{100.0}),
                 "VehicleEngine caches effective drive torque");

    engine.clear_torque();
    engine.throttle = real_t{0.0};
    const real_t before_braking = engine.get_angular_velocity();
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() < real_t{0.0} &&
                     near(engine.get_torque(), real_t{0.0}),
                 "VehicleEngine throttle cut updates cached drive torque");
    engine.integrate(real_t{0.01});
    state.expect(engine.get_angular_velocity() < before_braking, "VehicleEngine drag/braking slows engine");

    engine.clear_torque();
    engine.set_angular_velocity(rpm_to_omega(real_t{5000.0}));
    engine.throttle = real_t{1.0};
    engine.integrate(real_t{0.001});
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_rpm() <= real_t{4200.0} + real_t{1.0},
                 "VehicleEngine rev limit hard bound");
    state.expect(engine.get_pending_torque() <= real_t{0.0},
                 "VehicleEngine rev limit cuts throttle torque");
}

void test_setup_validation(TestState &state) {
    String error;
    state.expect(!VehicleSetupValidation::validate(Ref<VehicleConfig>(), {}, error) &&
                     error.find("config is required") >= 0,
                 "Vehicle setup validation rejects missing config");

    auto expect_missing_resource = [&](Ref<VehicleConfig> candidate,
                                       const std::vector<Axle *> &topology,
                                       const char *needle,
                                       const char *label) {
        state.expect(!VehicleSetupValidation::validate(candidate, topology, error) &&
                         error.find(needle) >= 0,
                     label);
    };

    Ref<VehicleConfig> missing_engine = make_valid_config();
    missing_engine->set_engine_data(Ref<VehicleEngineData>());
    Axle *engine_axle = make_axle(real_t{1.0});
    expect_missing_resource(missing_engine, {engine_axle}, "engine_data is required",
                            "Vehicle setup validation rejects missing engine data");
    memdelete(engine_axle);

    Ref<VehicleConfig> missing_gearbox = make_valid_config();
    missing_gearbox->set_gearbox_data(Ref<GearboxData>());
    Axle *gearbox_axle = make_axle(real_t{1.0});
    expect_missing_resource(missing_gearbox, {gearbox_axle}, "gearbox_data is required",
                            "Vehicle setup validation rejects missing gearbox data");
    memdelete(gearbox_axle);

    Ref<VehicleConfig> missing_suspension = make_valid_config();
    missing_suspension->set_suspension_data(Ref<SuspensionData>());
    Axle *suspension_axle = make_axle(real_t{1.0});
    expect_missing_resource(missing_suspension, {suspension_axle}, "suspension_data is required",
                            "Vehicle setup validation rejects missing suspension data");
    memdelete(suspension_axle);

    Ref<VehicleConfig> empty_topology = make_valid_config();
    expect_missing_resource(empty_topology, {}, "at least one axle is required",
                            "Vehicle setup validation rejects empty axle topology");

    Ref<VehicleConfig> null_topology = make_valid_config();
    expect_missing_resource(null_topology, {nullptr}, "null axle",
                            "Vehicle setup validation rejects null axle topology");

    Ref<VehicleConfig> config = make_valid_config();
    Axle *axle = make_axle(real_t{1.0});
    state.expect(VehicleSetupValidation::validate(config, {axle}, error),
                 "Vehicle setup validation accepts centralized valid config");
    config->get_gearbox_data()->set_final_drive(real_t{0.0});
    state.expect(VehicleSetupValidation::validate(config, {axle}, error),
                 "Vehicle setup validation leaves scalar ranges to the editor contract");
    memdelete(axle);

    Ref<VehicleConfig> missing_curve = make_valid_config();
    missing_curve->get_engine_data()->set_torque_curve(Ref<Curve>());
    Axle *curve_axle = make_axle(real_t{1.0});
    state.expect(!VehicleSetupValidation::validate(missing_curve, {curve_axle}, error) &&
                     error.find("torque_curve") >= 0,
                 "Vehicle setup validation requires the engine torque curve");
    memdelete(curve_axle);

    Ref<VehicleConfig> valid_config = make_valid_config();
    Axle *one_wheel = memnew(Axle);
    one_wheel->set_drive_share(real_t{1.0});
    one_wheel->set_tire_data(make_tire_data());
    one_wheel->add_child(memnew(Wheel));
    one_wheel->_ready();
    state.expect(!VehicleSetupValidation::validate(valid_config, {one_wheel}, error) &&
                     error.find("exactly two wheels") >= 0,
                 "Vehicle setup validation requires exactly two wheels per axle");
    memdelete(one_wheel);

    Axle *missing_tire = memnew(Axle);
    missing_tire->set_drive_share(real_t{1.0});
    missing_tire->add_child(memnew(Wheel));
    missing_tire->add_child(memnew(Wheel));
    missing_tire->_ready();
    state.expect(!VehicleSetupValidation::validate(valid_config, {missing_tire}, error) &&
                     error.find("requires tire_data") >= 0,
                 "Vehicle setup validation requires tire data per axle");
    memdelete(missing_tire);

    Axle *missing_steering = make_axle(real_t{1.0});
    missing_steering->set_steerable(true);
    state.expect(!VehicleSetupValidation::validate(valid_config, {missing_steering}, error) &&
                     error.find("requires steering_rack_data") >= 0,
                 "Vehicle setup validation requires steering data for steerable axles");
    memdelete(missing_steering);

    Axle *free_axle = make_axle(real_t{0.0});
    state.expect(!VehicleSetupValidation::validate(valid_config, {free_axle}, error) &&
                     error.find("at least one driven axle") >= 0,
                 "Vehicle setup validation requires a driven axle");
    memdelete(free_axle);

    Axle *missing_differential = memnew(Axle);
    missing_differential->set_drive_share(real_t{1.0});
    missing_differential->set_tire_data(make_tire_data());
    missing_differential->add_child(memnew(Wheel));
    missing_differential->add_child(memnew(Wheel));
    missing_differential->_ready();
    state.expect(!VehicleSetupValidation::validate(
                         valid_config, {missing_differential}, error) &&
                     error.find("requires differential_data") >= 0,
                 "Vehicle setup validation requires driven-axle differential data");
    memdelete(missing_differential);

    Axle *first_driven = make_axle(real_t{0.4});
    Axle *second_driven = make_axle(real_t{0.6});
    state.expect(!VehicleSetupValidation::validate(
                         valid_config, {first_driven, second_driven}, error) &&
                     error.find("center_differential_data") >= 0,
                 "Vehicle setup validation requires a center differential for AWD");
    valid_config->set_center_differential_data(
            make_differential_data(DifferentialData::OPEN));
    state.expect(VehicleSetupValidation::validate(
                         valid_config, {first_driven, second_driven}, error),
                 "Vehicle setup validation accepts complete two-axle AWD topology");

    Axle *third_driven = make_axle(real_t{0.2});
    state.expect(!VehicleSetupValidation::validate(
                         valid_config, {first_driven, second_driven, third_driven}, error) &&
                     error.find("at most two driven axles") >= 0,
                 "Vehicle setup validation rejects more than two driven axles");
    memdelete(first_driven);
    memdelete(second_driven);
    memdelete(third_driven);
}

} // namespace

void DrivetrainRegression::_bind_methods() {
    ClassDB::bind_method(D_METHOD("run"), &DrivetrainRegression::run);
}

bool DrivetrainRegression::run() {
    TestState state;
    test_rotational_body(state);
    test_clutch_constraint(state);
    test_gearbox(state);
    test_coupling(state);
    test_differential_solver(state);
    test_engine(state);
    test_setup_validation(state);
    if (state.failures != 0)
        UtilityFunctions::printerr("[drivetrain-regression] ", state.failures,
                                   " assertion(s) failed");
    else
        UtilityFunctions::print("[drivetrain-regression] all assertions passed");
    return state.failures == 0;
}

} // namespace godot
