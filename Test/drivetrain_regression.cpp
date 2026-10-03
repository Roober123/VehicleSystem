#include "drivetrain_regression.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "godot_cpp/classes/curve.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/window.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/packed_color_array.hpp"
#include "godot_cpp/variant/packed_int32_array.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

#include "Drivetrain/RotationalConstraint.h"
#include "Drivetrain/RotationalNetwork.h"
#include "Drivetrain/Gearbox.h"
#include "Drivetrain/Turbo.h"
#include "Drivetrain/VehicleEngine.h"
#include "Resources/gearbox_data.h"
#include "Resources/differential_data.h"
#include "Resources/suspension_data.h"
#include "Resources/tire_data.h"
#include "Resources/turbo_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/vehicle_config.h"
#include "Resources/vehicle_engine_data.h"
#include "axle.h"
#include "SteeringRack.h"
#include "TireSkid.h"
#include "VehicleTelemetry.h"
#include "vehicle.h"
#include "vehicle_setup_validation.h"
#include "wheel.h"
#include "godot_cpp/classes/marker3d.hpp"

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

Ref<TurboData> make_turbo_data(real_t max_boost_bar = real_t{1.0},
                               real_t full_boost_rpm = real_t{3000.0},
                               real_t lag_seconds = real_t{0.6}) {
    Ref<TurboData> data = memnew(TurboData);
    data->set_max_boost_bar(max_boost_bar);
    data->set_full_boost_rpm(full_boost_rpm);
    data->set_lag_seconds(lag_seconds);
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

Ref<TireData> make_grip_tire(real_t exponent) {
    Ref<TireData> data = make_tire_data();
    data->set_friction_forward(real_t{1.0});
    data->set_friction_lateral(real_t{1.0});
    data->set_load_grip_loss_percent(real_t{0.0});
    data->set_force_response_low_speed_ms(real_t{1.0});
    data->set_force_response_108_kph_ms(real_t{1.0});
    data->set_combined_grip_exponent(exponent);
    return data;
}

Wheel *make_combined_grip_wheel(const Ref<TireData> &tire) {
    Wheel *wheel = memnew(Wheel);
    wheel->set_tire(tire);
    wheel->on_ground = true;
    wheel->collision_point = Vector3();
    wheel->collision_normal = Vector3(0.0, 1.0, 0.0);
    wheel->forward_vector = Vector3(0.0, 0.0, 1.0);
    wheel->right_vector = Vector3(1.0, 0.0, 0.0);
    wheel->set_normal_force(real_t{10.0});
    wheel->tire_force = Vector3();
    return wheel;
}

SkidContactSample make_skid_sample(real_t longitudinal_force,
                                   real_t longitudinal_slip,
                                   real_t lateral_force = real_t{0.0},
                                   real_t lateral_slip = real_t{0.0}) {
    SkidContactSample sample;
    sample.contact_normal = Vector3(0.0, 1.0, 0.0);
    sample.tire_direction = Vector3(0.0, 0.0, 1.0);
    sample.longitudinal_force = longitudinal_force;
    sample.longitudinal_slip_velocity = longitudinal_slip;
    sample.lateral_force = lateral_force;
    sample.lateral_slip_velocity = lateral_slip;
    sample.normal_load = real_t{1000.0};
    return sample;
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
    data->set_acceleration_lock_percent(power_lock * real_t{200.0});
    data->set_engine_braking_lock_percent(coast_lock * real_t{200.0});
    data->set_speed_lock_torque_per_100_rpm(slip_gain * rpm_to_omega(real_t{100.0}));
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

Axle *make_unready_axle(real_t drive_share) {
    Axle *axle = memnew(Axle);
    axle->set_drive_share(drive_share);
    if (drive_share > real_t{0.0})
        axle->set_differential_data(make_differential_data(DifferentialData::OPEN));
    axle->set_tire_data(make_tire_data());
    axle->add_child(memnew(Wheel));
    axle->add_child(memnew(Wheel));
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

void test_rotational_constraint(TestState &state) {
    // Scalar hard row: equalize two bodies while conserving angular momentum
    // and dissipating (never adding) kinetic energy.
    RotationalBody first;
    RotationalBody second;
    first.set_inertia(real_t{2.0});
    second.set_inertia(real_t{3.0});
    first.set_angular_velocity(real_t{4.0});
    second.set_angular_velocity(real_t{-1.0});
    VelocityConstraint row;
    row.add_term(first, real_t{1.0});
    row.add_term(second, real_t{-1.0});
    const real_t initial_momentum = first.get_inertia() * first.get_angular_velocity() +
            second.get_inertia() * second.get_angular_velocity();
    const real_t initial_energy = real_t{0.5} * first.get_inertia() *
            first.get_angular_velocity() * first.get_angular_velocity() +
            real_t{0.5} * second.get_inertia() * second.get_angular_velocity() *
            second.get_angular_velocity();
    ConstraintSolver hard_solver(real_t{0.1});
    const ConstraintSolution hard = hard_solver.solve(row);
    const bool hard_committed = hard_solver.commit();
    first.integrate(real_t{0.1});
    second.integrate(real_t{0.1});
    const real_t final_momentum = first.get_inertia() * first.get_angular_velocity() +
            second.get_inertia() * second.get_angular_velocity();
    const real_t final_energy = real_t{0.5} * first.get_inertia() *
            first.get_angular_velocity() * first.get_angular_velocity() +
            real_t{0.5} * second.get_inertia() * second.get_angular_velocity() *
            second.get_angular_velocity();
    state.expect(hard.valid && hard_committed && !hard.slipping &&
                         near(hard.requested_torque, real_t{-60.0}) &&
                         near(hard.applied_torque, real_t{-60.0}) &&
                         near(hard.slip, real_t{0.0}) &&
                         near(first.get_angular_velocity(), real_t{1.0}) &&
                         near(second.get_angular_velocity(), real_t{1.0}) &&
                         near(final_momentum, initial_momentum) &&
                         final_energy <= initial_energy + kEpsilon,
                     "RotationalConstraint scalar conservation and energy");

    // Capacity is a torque magnitude; the impulse bound is capacity * dt.
    RotationalBody capacity_left;
    RotationalBody capacity_right;
    capacity_left.set_inertia(real_t{1.0});
    capacity_right.set_inertia(real_t{1.0});
    capacity_left.set_angular_velocity(real_t{4.0});
    VelocityConstraint capacity_row;
    capacity_row.add_term(capacity_left, real_t{1.0});
    capacity_row.add_term(capacity_right, real_t{-1.0});
    ConstraintSolver capacity_solver(real_t{0.1});
    const ConstraintSolution limited =
            capacity_solver.solve(capacity_row, real_t{5.0});
    const bool capacity_committed = capacity_solver.commit();
    state.expect(limited.valid && capacity_committed && limited.slipping &&
                         near(limited.requested_torque, real_t{-20.0}) &&
                         near(limited.applied_torque, real_t{-5.0}) &&
                         near(limited.slip, real_t{3.0}) &&
                         near(capacity_left.get_pending_torque(), real_t{-5.0}) &&
                         near(capacity_right.get_pending_torque(), real_t{5.0}),
                     "RotationalConstraint capacity clamp and slip result");

    // Every term is validated before mutation.  A non-finite row leaves all
    // existing pending torques untouched.
    RotationalBody invalid_body;
    invalid_body.set_inertia(real_t{1.0});
    invalid_body.set_angular_velocity(real_t{2.0});
    invalid_body.add_torque(real_t{7.0});
    VelocityConstraint invalid_row;
    state.expect(!invalid_row.add_term(
                         invalid_body,
                         std::numeric_limits<real_t>::quiet_NaN()),
                 "VelocityConstraint rejects non-finite coefficient");
    ConstraintSolver invalid_solver(real_t{0.1});
    const ConstraintSolution invalid = invalid_solver.solve(invalid_row);
    state.expect(!invalid.valid && near(invalid_body.get_pending_torque(), real_t{7.0}),
                 "RotationalConstraint invalid input has no partial mutation");

    // Rows and solver transactions are fixed-capacity.  Fill a row to its 18-term
    // bound, then verify the 19th term is rejected without changing the row.
    std::array<RotationalBody, MAX_CONSTRAINT_TERMS + 1> bounded_bodies{};
    VelocityConstraint bounded_row;
    bool all_terms_added = true;
    for (std::size_t i = 0; i < MAX_CONSTRAINT_TERMS; ++i)
        all_terms_added = all_terms_added &&
                bounded_row.add_term(bounded_bodies[i], real_t{1.0});
    const bool over_bound_rejected =
            !bounded_row.add_term(bounded_bodies[MAX_CONSTRAINT_TERMS],
                                  real_t{1.0});
    ConstraintSolver bounded_solver(real_t{0.1});
    const ConstraintSolution bounded_solution = bounded_solver.solve(bounded_row);
    const bool bounded_committed = bounded_solver.commit();
    state.expect(all_terms_added && over_bound_rejected &&
                         bounded_row.term_count == MAX_CONSTRAINT_TERMS &&
                         bounded_solution.valid && bounded_committed,
                 "RotationalConstraint enforces fixed row and batch bounds");

    // Coupled analytic rows.  The primary row is hard; the clutch row is
    // solved exactly when unbounded and leaves only its bounded residual when
    // its capacity is exhausted.
    RotationalBody coupled_a;
    RotationalBody coupled_b;
    RotationalBody coupled_c;
    coupled_a.set_angular_velocity(real_t{3.0});
    coupled_b.set_angular_velocity(real_t{0.0});
    coupled_c.set_angular_velocity(real_t{-2.0});
    VelocityConstraint primary_constraint;
    primary_constraint.add_term(coupled_a, real_t{1.0});
    primary_constraint.add_term(coupled_b, real_t{-1.0});
    VelocityConstraint clutch_constraint;
    clutch_constraint.add_term(coupled_b, real_t{1.0});
    clutch_constraint.add_term(coupled_c, real_t{-1.0});
    ConstraintSolver analytic_solver(real_t{0.1});
    const ClutchSolution analytic =
            analytic_solver.solve_clutch(primary_constraint, clutch_constraint);
    const bool analytic_committed = analytic_solver.commit();
    coupled_a.integrate(real_t{0.1});
    coupled_b.integrate(real_t{0.1});
    coupled_c.integrate(real_t{0.1});
    state.expect(analytic.valid && analytic_committed &&
                         near(analytic.clutch_requested_torque,
                              real_t{-70.0 / 3.0}) &&
                         near(analytic.clutch_applied_torque,
                              real_t{-70.0 / 3.0}) &&
                         near(analytic.primary_applied_torque,
                              real_t{-80.0 / 3.0}) &&
                         near(coupled_a.get_angular_velocity(),
                              coupled_b.get_angular_velocity()) &&
                         near(coupled_b.get_angular_velocity(),
                              coupled_c.get_angular_velocity()),
                     "RotationalConstraint coupled analytic solve");

    RotationalBody bounded_a;
    RotationalBody bounded_b;
    RotationalBody bounded_c;
    bounded_a.set_angular_velocity(real_t{3.0});
    bounded_c.set_angular_velocity(real_t{-2.0});
    VelocityConstraint bounded_primary_constraint;
    bounded_primary_constraint.add_term(bounded_a, real_t{1.0});
    bounded_primary_constraint.add_term(bounded_b, real_t{-1.0});
    VelocityConstraint bounded_clutch_constraint;
    bounded_clutch_constraint.add_term(bounded_b, real_t{1.0});
    bounded_clutch_constraint.add_term(bounded_c, real_t{-1.0});
    ConstraintSolver bounded_solver_2(real_t{0.1});
    const ClutchSolution bounded = bounded_solver_2.solve_clutch(
            bounded_primary_constraint, bounded_clutch_constraint,
            real_t{10.0});
    const bool bounded_committed_2 = bounded_solver_2.commit();
    bounded_a.integrate(real_t{0.1});
    bounded_b.integrate(real_t{0.1});
    bounded_c.integrate(real_t{0.1});
    state.expect(bounded.valid && bounded_committed_2 && bounded.clutch_slipping &&
                         near(bounded.clutch_applied_torque, real_t{-10.0}) &&
                         near(bounded_a.get_angular_velocity(),
                              bounded_b.get_angular_velocity()) &&
                         std::abs(bounded_b.get_angular_velocity() -
                                          bounded_c.get_angular_velocity()) >
                                 real_t{1.0},
                     "RotationalConstraint bounded clutch conditions primary");
}

void test_rotational_network(TestState &state) {
    auto make_config = [](RotationalBody &left, RotationalBody &right,
                          real_t drive_share,
                          DifferentialData::Mode mode = DifferentialData::OPEN,
                          real_t preload = real_t{0.0},
                          real_t power_lock = real_t{0.0},
                          real_t coast_lock = real_t{0.0},
                          real_t slip_gain = real_t{0.0},
                          real_t max_lock = real_t{100.0}) {
        DrivenAxle config;
        config.left = &left;
        config.right = &right;
        config.share = drive_share;
        config.differential.mode = mode;
        config.differential.preload_torque = preload;
        config.differential.power_lock_ratio = power_lock;
        config.differential.coast_lock_ratio = coast_lock;
        config.differential.slip_sensitive_gain = slip_gain;
        config.differential.max_lock_torque = max_lock;
        return config;
    };

    RotationalBody engine;
    RotationalBody shaft;
    engine.set_inertia(real_t{1.0});
    shaft.set_inertia(real_t{2.0});
    std::array<RotationalBody, RotationalNetwork::MAX_WHEELS> wheels{};
    for (std::size_t i = 0; i < wheels.size(); ++i)
        wheels[i].set_inertia(real_t{1.0} + static_cast<real_t>(i % 3));

    std::array<DrivenAxle, RotationalNetwork::MAX_AXLES>
            configs{};
    for (std::size_t i = 0; i < RotationalNetwork::MAX_AXLES; ++i)
        configs[i] = make_config(wheels[2 * i], wheels[2 * i + 1],
                                 real_t{1.0} + static_cast<real_t>(i));

    RotationalNetwork network;
    for (std::size_t count = 1; count <= RotationalNetwork::MAX_AXLES;
         ++count) {
        state.expect(network.configure(engine, shaft, configs, count) &&
                             network.get_axle_count() == count,
                     "RotationalNetwork configures one through eight axles");
    }
    const std::size_t preserved_count = network.get_axle_count();
    state.expect(!network.configure(engine, shaft, configs, 0) &&
                         network.get_axle_count() == preserved_count,
                 "RotationalNetwork rejects zero axles atomically");
    state.expect(!network.configure(engine, shaft, configs,
                                    RotationalNetwork::MAX_AXLES + 1) &&
                         network.get_axle_count() == preserved_count,
                 "RotationalNetwork rejects nine axles atomically");

    auto expect_rejected_preserving_config = [&](const char *label,
                                                  const DrivenAxle &bad_config) {
        auto candidate = configs;
        candidate[0] = bad_config;
        state.expect(!network.configure(engine, shaft, candidate,
                                        preserved_count) &&
                             network.get_axle_count() == preserved_count,
                     label);
    };
    auto bad = configs[0];
    bad.left = nullptr;
    expect_rejected_preserving_config(
            "RotationalNetwork rejects null wheel without mutation", bad);
    bad = configs[0];
    bad.right = bad.left;
    expect_rejected_preserving_config(
            "RotationalNetwork rejects duplicate wheel without mutation", bad);
    bad = configs[0];
    bad.left = &engine;
    expect_rejected_preserving_config(
            "RotationalNetwork rejects reused engine body without mutation", bad);
    bad = configs[0];
    bad.left->set_inertia(real_t{0.0});
    expect_rejected_preserving_config(
            "RotationalNetwork rejects invalid wheel inertia without mutation", bad);
    bad.left->set_inertia(real_t{1.0});
    bad = configs[0];
    bad.share = real_t{0.0};
    expect_rejected_preserving_config(
            "RotationalNetwork rejects non-positive share without mutation", bad);
    bad = configs[0];
    bad.share = std::numeric_limits<real_t>::quiet_NaN();
    expect_rejected_preserving_config(
            "RotationalNetwork rejects non-finite share without mutation", bad);
    bad = configs[0];
    bad.differential.mode = static_cast<DifferentialData::Mode>(99);
    expect_rejected_preserving_config(
            "RotationalNetwork rejects invalid differential mode without mutation", bad);
    bad = configs[0];
    bad.differential.max_lock_torque =
            std::numeric_limits<real_t>::quiet_NaN();
    expect_rejected_preserving_config(
            "RotationalNetwork rejects invalid differential values without mutation", bad);

    // Count preservation alone would miss a partially committed row or body
    // pointer.  Re-run the old eight-axle program and verify its share table
    // remains intact after every rejected configure attempt above.
    engine.clear_torque();
    shaft.clear_torque();
    for (auto &wheel : wheels)
        wheel.clear_torque();
    shaft.set_angular_velocity(real_t{5.0});
    const bool preserved_solve = network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    const real_t preserved_reference = wheels[0].get_pending_torque() +
            wheels[1].get_pending_torque();
    const real_t preserved_last =
            wheels[14].get_pending_torque() + wheels[15].get_pending_torque();
    state.expect(preserved_solve && near(preserved_last / preserved_reference,
                                         real_t{8.0}, real_t{2e-3}),
                 "RotationalNetwork rejected configure leaves prior program intact");

    // The direct primary row normalizes arbitrary shares.  Inspecting the
    // committed wheel torques proves both the 40/60 case and all eight axles.
    configs[0] = make_config(wheels[0], wheels[1], real_t{4.0});
    configs[1] = make_config(wheels[2], wheels[3], real_t{6.0});
    state.expect(network.configure(engine, shaft, configs, 2),
                 "RotationalNetwork configures 40/60 axle shares");
    engine.clear_torque();
    shaft.clear_torque();
    for (auto &wheel : wheels)
        wheel.clear_torque();
    shaft.set_angular_velocity(real_t{5.0});
    const bool split_result = network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    const real_t split_front_torque = wheels[0].get_pending_torque() +
            wheels[1].get_pending_torque();
    const real_t split_rear_torque = wheels[2].get_pending_torque() +
            wheels[3].get_pending_torque();
    state.expect(split_result &&
                         near(split_rear_torque / split_front_torque,
                              real_t{1.5}),
                 "RotationalNetwork normalizes 40/60 torque shares");

    state.expect(network.configure(engine, shaft, configs,
                                   RotationalNetwork::MAX_AXLES),
                 "RotationalNetwork configures all eight normalized axles");
    for (std::size_t i = 0; i < RotationalNetwork::MAX_AXLES; ++i)
        configs[i] = make_config(wheels[2 * i], wheels[2 * i + 1],
                                 real_t{1.0} + static_cast<real_t>(i));
    state.expect(network.configure(engine, shaft, configs,
                                   RotationalNetwork::MAX_AXLES),
                 "RotationalNetwork rebuilds eight-axle share table");
    engine.clear_torque();
    shaft.clear_torque();
    for (auto &wheel : wheels)
        wheel.clear_torque();
    shaft.set_angular_velocity(real_t{5.0});
    const bool eight_result = network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    bool eight_shares_match = eight_result;
    for (std::size_t i = 0; i < RotationalNetwork::MAX_AXLES; ++i) {
        const real_t axle_torque = wheels[2 * i].get_pending_torque() +
                wheels[2 * i + 1].get_pending_torque();
        const real_t reference_torque = wheels[0].get_pending_torque() +
                wheels[1].get_pending_torque();
        eight_shares_match = eight_shares_match &&
                near(axle_torque / reference_torque,
                     (real_t{1.0} + static_cast<real_t>(i)), real_t{2e-3});
    }
    state.expect(eight_shares_match,
                 "RotationalNetwork distributes normalized torque across eight axles");

    // Eight axles plus a non-neutral clutch touches all 18 fixed batch body
    // slots (engine, shaft, and sixteen wheels) in one network solve.
    engine.clear_torque();
    shaft.clear_torque();
    for (auto &wheel : wheels)
        wheel.clear_torque();
    engine.set_angular_velocity(real_t{10.0});
    shaft.set_angular_velocity(real_t{0.0});
    for (auto &wheel : wheels)
        wheel.set_angular_velocity(real_t{0.0});
    state.expect(network.solve(real_t{0.1}, real_t{2.0}, real_t{1.0},
                               real_t{100.0}),
                 "RotationalNetwork fits eight-axle clutch solve in fixed batch");

    // Reconfigure a simple one-axle fixture for clutch and differential
    // behavior checks.  Unequal wheel inertias are intentional and supported.
    auto open_config = make_config(wheels[0], wheels[1], real_t{1.0});
    std::array<DrivenAxle, RotationalNetwork::MAX_AXLES>
            simple_configs{};
    simple_configs[0] = open_config;
    state.expect(network.configure(engine, shaft, simple_configs, 1),
                 "RotationalNetwork accepts unequal wheel inertias");

    engine.set_angular_velocity(real_t{10.0});
    shaft.set_angular_velocity(real_t{0.0});
    wheels[0].set_angular_velocity(real_t{0.0});
    wheels[1].set_angular_velocity(real_t{0.0});
    const bool launch = network.solve(
            real_t{0.1}, real_t{2.0}, real_t{1.0}, real_t{100.0});
    const ClutchTelemetry launch_telemetry = network.get_clutch_telemetry();
    state.expect(launch && launch_telemetry.present &&
                         !launch_telemetry.slipping,
                 "RotationalNetwork forward clutch commits at capacity");
    state.expect(launch_telemetry.requested_engine_torque > real_t{0.0} &&
                         launch_telemetry.transmitted_torque > real_t{0.0} &&
                         launch_telemetry.output_torque > real_t{0.0} &&
                         near(launch_telemetry.output_torque,
                              real_t{2.0} * launch_telemetry.transmitted_torque),
                 "RotationalNetwork forward clutch telemetry signs");
    engine.integrate(real_t{0.1});
    shaft.integrate(real_t{0.1});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});
    state.expect(near(engine.get_angular_velocity(),
                      real_t{2.0} * shaft.get_angular_velocity()),
                 "RotationalNetwork launch reaches signed ratio");

    const bool hold = network.solve(
            real_t{0.1}, real_t{2.0}, real_t{1.0}, real_t{100.0});
    const ClutchTelemetry hold_telemetry = network.get_clutch_telemetry();
    state.expect(hold && hold_telemetry.present && !hold_telemetry.slipping,
                 "RotationalNetwork synchronized clutch holds without slip");
    engine.integrate(real_t{0.1});
    shaft.integrate(real_t{0.1});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});

    engine.set_angular_velocity(real_t{10.0});
    shaft.set_angular_velocity(real_t{0.0});
    wheels[0].set_angular_velocity(real_t{0.0});
    wheels[1].set_angular_velocity(real_t{0.0});
    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    const bool capped = network.solve(
            real_t{0.1}, real_t{2.0}, real_t{2.0}, real_t{1.0});
    const ClutchTelemetry capped_telemetry = network.get_clutch_telemetry();
    state.expect(capped &&
                         std::abs(capped_telemetry.transmitted_torque) <=
                                 real_t{1.0} + kEpsilon &&
                         capped_telemetry.slipping,
                 "RotationalNetwork clamps over-engagement to hardware capacity");

    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    engine.set_angular_velocity(real_t{6.0});
    engine.add_torque(real_t{7.0});
    const bool neutral = network.solve(
            real_t{0.1}, real_t{0.0}, real_t{1.0}, real_t{100.0});
    const ClutchTelemetry neutral_telemetry = network.get_clutch_telemetry();
    state.expect(neutral && !neutral_telemetry.present &&
                         near(engine.get_pending_torque(), real_t{7.0}) &&
                         near(neutral_telemetry.requested_engine_torque,
                              real_t{0.0}) &&
                         near(neutral_telemetry.transmitted_torque,
                              real_t{0.0}) &&
                         near(neutral_telemetry.output_torque, real_t{0.0}),
                 "RotationalNetwork neutral omits clutch mutation");
    engine.clear_torque();
    engine.set_angular_velocity(real_t{-10.0});
    shaft.set_angular_velocity(real_t{0.0});
    wheels[0].set_angular_velocity(real_t{0.0});
    wheels[1].set_angular_velocity(real_t{0.0});
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    const bool reverse = network.solve(
            real_t{0.1}, real_t{-2.0}, real_t{1.0}, real_t{100.0});
    const ClutchTelemetry reverse_telemetry = network.get_clutch_telemetry();
    engine.integrate(real_t{0.1});
    shaft.integrate(real_t{0.1});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});
    state.expect(reverse && reverse_telemetry.present &&
                         near(engine.get_angular_velocity() +
                                      real_t{2.0} * shaft.get_angular_velocity(),
                              real_t{0.0}),
                 "RotationalNetwork reverse signed ratio");
    state.expect(near(reverse_telemetry.requested_engine_torque,
                      reverse_telemetry.transmitted_torque) &&
                         near(reverse_telemetry.output_torque,
                              real_t{-2.0} * reverse_telemetry.transmitted_torque),
                 "RotationalNetwork reverse clutch telemetry signs");
    // Open axle couples the carrier but leaves left/right relative speed free.
    wheels[0].set_inertia(real_t{1.0});
    wheels[1].set_inertia(real_t{1.0});
    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    shaft.set_angular_velocity(real_t{5.0});
    wheels[0].set_angular_velocity(real_t{3.0});
    wheels[1].set_angular_velocity(real_t{-1.0});
    const real_t open_relative_before =
            wheels[0].get_angular_velocity() - wheels[1].get_angular_velocity();
    const bool open_runtime = network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    shaft.integrate(real_t{0.1});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});
    const real_t open_relative_after =
            wheels[0].get_angular_velocity() - wheels[1].get_angular_velocity();
    const real_t open_carrier_after = real_t{0.5} *
            (wheels[0].get_angular_velocity() + wheels[1].get_angular_velocity());
    state.expect(open_runtime &&
                         near(open_relative_after, open_relative_before) &&
                         near(shaft.get_angular_velocity(), open_carrier_after),
                 "RotationalNetwork open axle preserves relative slip while coupling carrier");

    // Locked axle equalizes wheel speeds while conserving momentum.
    simple_configs[0] = make_config(wheels[0], wheels[1], real_t{1.0},
                                     DifferentialData::LOCKED);
    RotationalNetwork locked_network;
    state.expect(locked_network.configure(engine, shaft, simple_configs, 1),
                 "RotationalNetwork configures locked runtime program");
    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    shaft.set_angular_velocity(real_t{0.0});
    wheels[0].set_angular_velocity(real_t{6.0});
    wheels[1].set_angular_velocity(real_t{-2.0});
    const real_t locked_momentum_before =
            shaft.get_inertia() * shaft.get_angular_velocity() +
            wheels[0].get_inertia() * wheels[0].get_angular_velocity() +
            wheels[1].get_inertia() * wheels[1].get_angular_velocity();
    const real_t locked_energy_before = real_t{0.5} *
            (shaft.get_inertia() * shaft.get_angular_velocity() *
                     shaft.get_angular_velocity() +
             wheels[0].get_inertia() * wheels[0].get_angular_velocity() *
                     wheels[0].get_angular_velocity() +
             wheels[1].get_inertia() * wheels[1].get_angular_velocity() *
                     wheels[1].get_angular_velocity());
    const bool locked_runtime = locked_network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    shaft.integrate(real_t{0.1});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});
    const real_t locked_momentum_after =
            shaft.get_inertia() * shaft.get_angular_velocity() +
            wheels[0].get_inertia() * wheels[0].get_angular_velocity() +
            wheels[1].get_inertia() * wheels[1].get_angular_velocity();
    const real_t locked_energy_after = real_t{0.5} *
            (shaft.get_inertia() * shaft.get_angular_velocity() *
                     shaft.get_angular_velocity() +
             wheels[0].get_inertia() * wheels[0].get_angular_velocity() *
                     wheels[0].get_angular_velocity() +
             wheels[1].get_inertia() * wheels[1].get_angular_velocity() *
                     wheels[1].get_angular_velocity());
    state.expect(locked_runtime &&
                         near(shaft.get_angular_velocity(),
                              wheels[0].get_angular_velocity()) &&
                         near(wheels[0].get_angular_velocity(),
                              wheels[1].get_angular_velocity()) &&
                         near(locked_momentum_after, locked_momentum_before) &&
                         locked_energy_after <= locked_energy_before + kEpsilon,
                 "RotationalNetwork locked axle equalizes wheels with conserved momentum and dissipated energy");

    simple_configs[0] = make_config(wheels[0], wheels[1], real_t{1.0},
                                     DifferentialData::LIMITED_SLIP,
                                     real_t{0.0}, real_t{0.0}, real_t{0.0},
                                     real_t{0.25}, real_t{1.0});
    RotationalNetwork lsd_network;
    state.expect(lsd_network.configure(engine, shaft, simple_configs, 1),
                 "RotationalNetwork configures limited-slip axle");
    shaft.set_angular_velocity(real_t{2.0});
    wheels[0].set_angular_velocity(real_t{4.0});
    wheels[1].set_angular_velocity(real_t{0.0});
    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    const real_t lsd_relative_before =
            wheels[0].get_angular_velocity() - wheels[1].get_angular_velocity();
    const bool lsd_result = lsd_network.solve(
            real_t{0.1}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    wheels[0].integrate(real_t{0.1});
    wheels[1].integrate(real_t{0.1});
    const real_t lsd_relative_after =
            wheels[0].get_angular_velocity() - wheels[1].get_angular_velocity();
    const real_t lsd_capacity = simple_configs[0].differential.capacity(
            real_t{20.0}, real_t{2.0}, real_t{4.0});
    const DifferentialSettings open_settings{};
    const real_t open_capacity = open_settings.capacity(
            real_t{1.0}, real_t{1.0}, real_t{1.0});
    const real_t locked_capacity = make_config(
            wheels[0], wheels[1], real_t{1.0}, DifferentialData::LOCKED)
                                         .differential
                                         .capacity(real_t{1.0}, real_t{1.0},
                                                   real_t{1.0});
    state.expect(lsd_result && near(lsd_capacity, real_t{1.0}) &&
                         near(open_capacity, real_t{0.0}) &&
                         !std::isfinite(locked_capacity) &&
                         lsd_relative_after > real_t{0.0} &&
                         lsd_relative_after < lsd_relative_before,
                 "RotationalNetwork LSD reduces slip without crossing or exceeding capacity");

    // Invalid public controls are rejected before staging.  Each case first
    // seeds valid clutch telemetry, then verifies both body torque atomicity
    // and the documented reset-on-entry telemetry contract.
    const auto telemetry_is_clear = [](const ClutchTelemetry &telemetry) {
        return !telemetry.present && !telemetry.slipping &&
                near(telemetry.requested_engine_torque, real_t{0.0}) &&
                near(telemetry.transmitted_torque, real_t{0.0}) &&
                near(telemetry.output_torque, real_t{0.0}) &&
                near(telemetry.slip, real_t{0.0});
    };
    auto expect_invalid_public_solve = [&](const char *label,
                                                real_t dt,
                                                real_t signed_ratio,
                                                real_t engagement,
                                                real_t clutch_max_torque) {
        engine.clear_torque();
        shaft.clear_torque();
        wheels[0].clear_torque();
        wheels[1].clear_torque();
        engine.set_angular_velocity(real_t{10.0});
        shaft.set_angular_velocity(real_t{0.0});
        wheels[0].set_angular_velocity(real_t{0.0});
        wheels[1].set_angular_velocity(real_t{0.0});
        const bool seeded = network.solve(
                real_t{0.1}, real_t{2.0}, real_t{1.0}, real_t{100.0});
        const ClutchTelemetry seeded_telemetry = network.get_clutch_telemetry();
        state.expect(seeded && seeded_telemetry.present,
                     "RotationalNetwork invalid-input fixture seeds telemetry");
        const real_t engine_torque_before = engine.get_pending_torque();
        const real_t shaft_torque_before = shaft.get_pending_torque();
        const real_t left_torque_before = wheels[0].get_pending_torque();
        const real_t right_torque_before = wheels[1].get_pending_torque();
        const bool result = network.solve(
                dt, signed_ratio, engagement, clutch_max_torque);
        const ClutchTelemetry cleared_telemetry = network.get_clutch_telemetry();
        state.expect(!result && telemetry_is_clear(cleared_telemetry) &&
                             near(engine.get_pending_torque(), engine_torque_before) &&
                             near(shaft.get_pending_torque(), shaft_torque_before) &&
                             near(wheels[0].get_pending_torque(), left_torque_before) &&
                             near(wheels[1].get_pending_torque(), right_torque_before),
                     label);
    };
    expect_invalid_public_solve(
            "RotationalNetwork rejects zero dt and clears telemetry",
            real_t{0.0}, real_t{2.0}, real_t{1.0}, real_t{100.0});
    expect_invalid_public_solve(
            "RotationalNetwork rejects non-finite dt and clears telemetry",
            std::numeric_limits<real_t>::quiet_NaN(), real_t{2.0},
            real_t{1.0}, real_t{100.0});
    expect_invalid_public_solve(
            "RotationalNetwork rejects non-finite ratio and clears telemetry",
            real_t{0.1}, std::numeric_limits<real_t>::infinity(),
            real_t{1.0}, real_t{100.0});
    expect_invalid_public_solve(
            "RotationalNetwork rejects non-finite engagement and clears telemetry",
            real_t{0.1}, real_t{2.0},
            std::numeric_limits<real_t>::quiet_NaN(), real_t{100.0});
    expect_invalid_public_solve(
            "RotationalNetwork rejects non-finite capacity and clears telemetry",
            real_t{0.1}, real_t{2.0}, real_t{1.0},
            std::numeric_limits<real_t>::infinity());

    // Stage a finite primary impulse against a body whose existing pending
    // torque makes the final accumulated torque overflow.  The consolidated
    // commit boundary must reject the solve without mutating any body.
    engine.clear_torque();
    shaft.clear_torque();
    wheels[0].clear_torque();
    wheels[1].clear_torque();
    engine.set_angular_velocity(real_t{0.0});
    shaft.set_angular_velocity(-std::numeric_limits<real_t>::max());
    wheels[0].set_angular_velocity(real_t{0.0});
    wheels[1].set_angular_velocity(real_t{0.0});
    const real_t overflow_torque = std::numeric_limits<real_t>::max();
    shaft.add_torque(overflow_torque);
    const real_t engine_torque_before = engine.get_pending_torque();
    const real_t shaft_torque_before = shaft.get_pending_torque();
    const real_t left_torque_before = wheels[0].get_pending_torque();
    const real_t right_torque_before = wheels[1].get_pending_torque();
    const bool overflow_result = network.solve(
            real_t{1.0}, real_t{0.0}, real_t{0.0}, real_t{0.0});
    state.expect(!overflow_result &&
                         near(engine.get_pending_torque(), engine_torque_before) &&
                         near(shaft.get_pending_torque(), shaft_torque_before) &&
                         near(wheels[0].get_pending_torque(), left_torque_before) &&
                         near(wheels[1].get_pending_torque(), right_torque_before) &&
                         telemetry_is_clear(network.get_clutch_telemetry()),
                 "RotationalNetwork final torque overflow rejects atomically");

}

void advance_shift(Gearbox &gearbox, real_t dt, int steps) {
    for (int i = 0; i < steps; ++i)
        gearbox.update_clutch_logic(dt, real_t{0.0}, real_t{1.0});
}

void test_gearbox(TestState &state) {
    Ref<GearboxData> speed_defaults = memnew(GearboxData);
    state.expect(near(speed_defaults->get_clutch_engage_speed(), real_t{10.0}) &&
                         near(speed_defaults->get_clutch_disengage_speed(), real_t{10.0}),
                 "GearboxData clutch phase speed defaults are ten units per second");
    speed_defaults->set_clutch_engage_speed(real_t{4.5});
    speed_defaults->set_clutch_disengage_speed(real_t{17.25});
    state.expect(near(speed_defaults->get_clutch_engage_speed(), real_t{4.5}) &&
                         near(speed_defaults->get_clutch_disengage_speed(), real_t{17.25}),
                 "GearboxData clutch phase speed setters round-trip authored values");

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
    advance_shift(gearbox, real_t{0.01}, 24);
    state.expect(gearbox.get_current_gear() == 1 && !gearbox.is_shifting() &&
                     near(gearbox.get_effective_ratio(), real_t{6.0}) &&
                     near(gearbox.get_clutch_engagement(), real_t{1.0}),
                 "Gearbox manual shift completes all phases");

    gearbox.shift_up();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 24);
    state.expect(gearbox.get_current_gear() == 2 && near(gearbox.get_effective_ratio(), real_t{3.0}),
                 "Gearbox second manual selection");

    gearbox.select_neutral();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 24);
    state.expect(gearbox.get_current_gear() == 0 && near(gearbox.get_effective_ratio(), real_t{0.0}),
                 "Gearbox neutral selection");

    gearbox.select_reverse();
    gearbox.update_shifting_logic(real_t{0.01});
    advance_shift(gearbox, real_t{0.01}, 24);
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

    // Mode changes are command-policy changes, not drivetrain resets.  An
    // active shift must finish under the new policy while the current gear is
    // retained, and a queued manual command from the old policy must be
    // discarded.
    VehicleEngine mode_engine(make_engine_data());
    mode_engine.set_angular_velocity(rpm_to_omega(real_t{2000.0}));
    RotationalBody mode_shaft;
    Gearbox mode_probe;
    mode_probe.set_bodies(mode_engine, mode_shaft);
    mode_probe.configure(make_gearbox_data(false));
    mode_probe.shift_up();
    mode_probe.update_shifting_logic(real_t{0.01});
    state.expect(mode_probe.is_shifting() && mode_probe.get_current_gear() == 0,
                 "Gearbox manual command starts an active shift before mode switch");
    mode_probe.set_automatic(true);
    state.expect(mode_probe.is_automatic() && mode_probe.is_shifting() &&
                         mode_probe.get_current_gear() == 0,
                 "Gearbox manual-to-automatic switch preserves active shift and gear");
    for (int i = 0; mode_probe.is_shifting() && i < 100; ++i)
        mode_probe.update_clutch_logic(real_t{0.01}, real_t{0.0}, real_t{1.0});
    state.expect(mode_probe.get_current_gear() == 1 && !mode_probe.is_shifting(),
                 "Gearbox active shift completes after manual-to-automatic switch");
    mode_probe.set_automatic(false);
    mode_probe.shift_down();
    mode_probe.set_automatic(true);
    mode_probe.update_shifting_logic(real_t{0.01});
    state.expect(mode_probe.is_automatic() && !mode_probe.is_shifting() &&
                         mode_probe.get_current_gear() == 1,
                 "Gearbox mode switch clears stale queued manual input");

    VehicleEngine reverse_mode_engine(make_engine_data());
    reverse_mode_engine.throttle = real_t{1.0};
    RotationalBody reverse_mode_shaft;
    Gearbox reverse_mode_probe;
    reverse_mode_probe.set_bodies(reverse_mode_engine, reverse_mode_shaft);
    reverse_mode_probe.configure(make_gearbox_data(true));
    reverse_mode_probe.update_shifting_logic(real_t{0.01});
    reverse_mode_probe.set_automatic(false);
    state.expect(!reverse_mode_probe.is_automatic() && reverse_mode_probe.is_shifting() &&
                         reverse_mode_probe.get_current_gear() == 0,
                 "Gearbox automatic-to-manual switch preserves active shift and gear");
    for (int i = 0; reverse_mode_probe.is_shifting() && i < 100; ++i)
        reverse_mode_probe.update_clutch_logic(real_t{0.01}, real_t{0.0}, real_t{1.0});
    state.expect(reverse_mode_probe.get_current_gear() == 1 && !reverse_mode_probe.is_shifting(),
                 "Gearbox active shift completes after automatic-to-manual switch");

    // Engage and disengage rates are independent phase controls.  Use an
    // engine RPM above the low-speed clutch safety clamp, then count the
    // fixed-step calls required by each phase and verify both shifts finish.
    auto measure_shift = [](real_t engage_speed, real_t disengage_speed) {
        VehicleEngine phase_engine(make_engine_data());
        phase_engine.set_angular_velocity(rpm_to_omega(real_t{2000.0}));
        RotationalBody phase_shaft;
        Gearbox phase_gearbox;
        phase_gearbox.set_bodies(phase_engine, phase_shaft);
        Ref<GearboxData> phase_data = make_gearbox_data(false);
        phase_data->set_shift_time(real_t{0.1});
        phase_data->set_clutch_engage_speed(engage_speed);
        phase_data->set_clutch_disengage_speed(disengage_speed);
        phase_gearbox.configure(phase_data);
        phase_gearbox.shift_up();
        phase_gearbox.update_shifting_logic(real_t{0.01});

        int disengage_steps = 0;
        while (phase_gearbox.get_current_gear() == 0 && disengage_steps < 1000) {
            phase_gearbox.update_clutch_logic(real_t{0.01}, real_t{0.0}, real_t{1.0});
            ++disengage_steps;
        }

        int engage_steps = 0;
        bool reengaging = false;
        int total_steps = disengage_steps;
        while (phase_gearbox.is_shifting() && total_steps < 1000) {
            phase_gearbox.update_clutch_logic(real_t{0.01}, real_t{0.0}, real_t{1.0});
            ++total_steps;
            if (!reengaging && phase_gearbox.get_clutch_engagement() > real_t{0.0}) {
                reengaging = true;
                engage_steps = 1;
            } else if (reengaging) {
                ++engage_steps;
            }
        }
        return std::array<int, 3>{disengage_steps, engage_steps, total_steps};
    };

    const std::array<int, 3> slow_disengage_fast_engage =
            measure_shift(real_t{20.0}, real_t{5.0});
    const std::array<int, 3> fast_disengage_slow_engage =
            measure_shift(real_t{5.0}, real_t{20.0});
    state.expect(slow_disengage_fast_engage[0] > fast_disengage_slow_engage[0] &&
                         slow_disengage_fast_engage[1] < fast_disengage_slow_engage[1] &&
                         slow_disengage_fast_engage[2] > 0 &&
                         fast_disengage_slow_engage[2] > 0,
                 "Gearbox authored clutch rates produce distinct phase timing");
    state.expect(slow_disengage_fast_engage[0] > 0 &&
                         fast_disengage_slow_engage[0] > 0 &&
                         slow_disengage_fast_engage[2] < 1000 &&
                         fast_disengage_slow_engage[2] < 1000,
                 "Gearbox authored-rate shifts complete within bounded time");
}

void test_turbo(TestState &state) {
    // TurboData intentionally exposes exactly three authoring values.
    Ref<TurboData> defaults = memnew(TurboData);
    state.expect(near(defaults->get_max_boost_bar(), real_t{1.0}) &&
                         near(defaults->get_full_boost_rpm(), real_t{3000.0}) &&
                         near(defaults->get_lag_seconds(), real_t{0.6}),
                 "TurboData has three stable property defaults");

    defaults->set_max_boost_bar(real_t{2.0});
    defaults->set_full_boost_rpm(real_t{3500.0});
    defaults->set_lag_seconds(real_t{0.75});
    state.expect(near(defaults->get_max_boost_bar(), real_t{2.0}) &&
                         near(defaults->get_full_boost_rpm(), real_t{3500.0}) &&
                         near(defaults->get_lag_seconds(), real_t{0.75}),
                 "TurboData setters round-trip authored values");

    Ref<TurboData> configuration = make_turbo_data(real_t{2.0}, real_t{3000.0},
                                                    real_t{0.6});
    Turbo zero_rpm;
    Turbo full_rpm;
    zero_rpm.configure(configuration);
    full_rpm.configure(configuration);
    zero_rpm.update(real_t{0.1}, real_t{0.0}, real_t{1.0}, real_t{1.0});
    full_rpm.update(real_t{0.1}, real_t{3000.0}, real_t{1.0}, real_t{1.0});
    state.expect(full_rpm.get_shaft_energy() > zero_rpm.get_shaft_energy() +
                         real_t{0.05} &&
                         full_rpm.get_boost() > zero_rpm.get_boost(),
                 "Turbo spool responds to engine RPM load");

    Turbo low_load;
    Turbo high_load;
    low_load.configure(configuration);
    high_load.configure(configuration);
    for (int i = 0; i < 5; ++i) {
        low_load.update(real_t{0.1}, real_t{1500.0}, real_t{1.0}, real_t{0.2});
        high_load.update(real_t{0.1}, real_t{1500.0}, real_t{1.0}, real_t{1.0});
    }
    state.expect(high_load.get_shaft_energy() > low_load.get_shaft_energy() +
                         real_t{0.05},
                 "Turbo spool below full-boost RPM responds to normalized engine load");

    Turbo authored_full_boost;
    authored_full_boost.configure(configuration);
    for (int i = 0; i < 400; ++i)
        authored_full_boost.update(real_t{0.02}, real_t{3000.0},
                                   real_t{1.0}, real_t{0.2});
    state.expect(near(authored_full_boost.get_shaft_energy(), real_t{1.0},
                      real_t{1e-4}) &&
                         near(authored_full_boost.get_boost(),
                              configuration->get_max_boost_bar(), real_t{1e-4}),
                 "Turbo reaches configured maximum at full-boost RPM");

    Turbo bounded;
    bounded.configure(configuration);
    for (int i = 0; i < 400; ++i)
        bounded.update(real_t{0.1}, real_t{3000.0}, real_t{1.0}, real_t{1.0});
    state.expect(bounded.get_shaft_energy() >= real_t{0.0} &&
                         bounded.get_shaft_energy() <= real_t{1.0} &&
                         bounded.get_boost() >= real_t{0.0} &&
                         bounded.get_boost() <= configuration->get_max_boost_bar(),
                 "Turbo shaft and boost remain bounded under full load");
    state.expect(near(bounded.get_air_charge_ratio(),
                      real_t{1.0} + real_t{0.85} * bounded.get_boost(),
                      real_t{1e-5}),
                 "Turbo air-charge ratio follows bounded boost");

    Turbo venting;
    venting.configure(configuration);
    for (int i = 0; i < 200; ++i)
        venting.update(real_t{0.02}, real_t{3000.0}, real_t{1.0}, real_t{1.0});
    const real_t shaft_before_lift = venting.get_shaft_energy();
    const real_t boost_before_lift = venting.get_boost();
    venting.update(real_t{0.1}, real_t{3000.0}, real_t{0.0}, real_t{1.0});
    const real_t shaft_after_lift = venting.get_shaft_energy();
    const real_t boost_after_lift = venting.get_boost();
    const real_t shaft_drop = (shaft_before_lift - shaft_after_lift) /
                              shaft_before_lift;
    const real_t boost_drop = (boost_before_lift - boost_after_lift) /
                              boost_before_lift;
    state.expect(boost_before_lift > real_t{0.5} && shaft_before_lift > real_t{0.5} &&
                         boost_after_lift < boost_before_lift * real_t{0.3} &&
                         shaft_after_lift > shaft_before_lift * real_t{0.7} &&
                         boost_drop > shaft_drop + real_t{0.5},
                 "Turbo boost vents much faster than retained shaft energy on lift");

    Turbo fresh;
    fresh.configure(configuration);
    const real_t fresh_boost = fresh.update(real_t{0.01}, real_t{3000.0},
                                            real_t{1.0}, real_t{1.0});
    const real_t retained_boost = venting.update(real_t{0.01}, real_t{3000.0},
                                                 real_t{1.0}, real_t{1.0});
    state.expect(venting.get_shaft_energy() > fresh.get_shaft_energy() &&
                         retained_boost > fresh_boost + real_t{0.05},
                 "Turbo retained shaft improves immediate reapplication");

    const auto simulate = [&](real_t dt, int steps) {
        Turbo candidate;
        candidate.configure(configuration);
        for (int i = 0; i < steps; ++i)
            candidate.update(dt, real_t{3000.0}, real_t{1.0}, real_t{1.0});
        return std::pair<real_t, real_t>(candidate.get_shaft_energy(),
                                         candidate.get_boost());
    };
    const auto fine_step = simulate(real_t{0.01}, 100);
    const auto common_step = simulate(real_t{0.02}, 50);
    state.expect(std::abs(fine_step.first - common_step.first) < real_t{0.01} &&
                         std::abs(fine_step.second - common_step.second) < real_t{0.03},
                 "Turbo response is timestep-consistent at 10 ms and 20 ms");

    Turbo engine_turbo;
    engine_turbo.configure(make_turbo_data(real_t{1.0}, real_t{3000.0}, real_t{0.6}));
    VehicleEngine boosted_engine(make_engine_data());
    boosted_engine.set_turbo(&engine_turbo);
    boosted_engine.set_angular_velocity(rpm_to_omega(real_t{3000.0}));
    boosted_engine.throttle = real_t{1.0};
    for (int i = 0; i < 100; ++i) {
        boosted_engine.clear_torque();
        boosted_engine.accumulate_torque(real_t{0.02});
    }
    const real_t expected_boosted_torque = real_t{100.0} *
                                            engine_turbo.get_air_charge_ratio();
    state.expect(engine_turbo.get_boost() > real_t{0.0} &&
                         near(boosted_engine.get_torque(), expected_boosted_torque,
                              real_t{1e-4}) &&
                         boosted_engine.get_torque() > real_t{100.0},
                 "VehicleEngine integrates turbo air charge into boosted torque");

    VehicleEngine naturally_aspirated(make_engine_data());
    naturally_aspirated.set_angular_velocity(rpm_to_omega(real_t{3000.0}));
    naturally_aspirated.throttle = real_t{1.0};
    naturally_aspirated.accumulate_torque(real_t{0.02});
    state.expect(near(naturally_aspirated.get_turbo_boost(), real_t{0.0}) &&
                         near(naturally_aspirated.get_torque(), real_t{100.0}) &&
                         near(naturally_aspirated.get_pending_torque(), real_t{100.0}),
                 "VehicleEngine naturally aspirated path remains baseline");
}

void test_engine(TestState &state) {
    VehicleEngine idle_engine(make_engine_data());
    const real_t idle_omega = rpm_to_omega(idle_engine.get_idle_rpm());
    const real_t idle_dt = real_t{0.05};
    idle_engine.set_angular_velocity(idle_omega);
    idle_engine.throttle = real_t{0.0};
    idle_engine.accumulate_torque(idle_dt);
    const real_t expected_idle_feedforward = real_t{0.5} * idle_omega;
    state.expect(near(idle_engine.get_pending_torque(), expected_idle_feedforward) &&
                         near(idle_engine.get_effective_torque(), real_t{0.0}) &&
                         near(idle_engine.predict_angular_velocity(idle_dt), idle_omega),
                 "VehicleEngine idle feed-forward cancels inherited drag at target");
    idle_engine.integrate(idle_dt);
    state.expect(near(idle_engine.get_angular_velocity(), idle_omega) &&
                         near(idle_engine.get_rpm(), idle_engine.get_idle_rpm()),
                 "VehicleEngine integration holds configured idle speed");

    idle_engine.set_angular_velocity(rpm_to_omega(real_t{800.0}));
    const real_t initial_recovery_rpm = idle_engine.get_rpm();
    for (int i = 0; i < 3000; ++i) {
        idle_engine.clear_torque();
        idle_engine.throttle = real_t{0.0};
        idle_engine.accumulate_torque(real_t{0.01});
        idle_engine.integrate(real_t{0.01});
    }
    const real_t recovery_rpm = idle_engine.get_rpm();
    state.expect(recovery_rpm > real_t{990.0} &&
                         recovery_rpm > initial_recovery_rpm,
                 "VehicleEngine below-idle recovery converges without steady-state droop");

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
    engine.set_angular_velocity(rpm_to_omega(real_t{4000.0}));
    engine.throttle = real_t{1.0};
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() <= real_t{0.0},
                 "VehicleEngine rev limiter cuts torque at redline");

    engine.clear_torque();
    engine.set_angular_velocity(rpm_to_omega(real_t{3900.0}));
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() <= real_t{0.0},
                 "VehicleEngine rev limiter remains cut inside RPM hysteresis");

    engine.clear_torque();
    engine.set_angular_velocity(rpm_to_omega(real_t{3850.0}));
    engine.accumulate_torque(real_t{0.01});
    state.expect(engine.get_pending_torque() > real_t{0.0} &&
                         near(engine.get_torque(), real_t{100.0}),
                 "VehicleEngine rev limiter resumes below RPM hysteresis");

    engine.clear_torque();
    engine.set_angular_velocity(rpm_to_omega(real_t{5000.0}));
    engine.integrate(real_t{0.001});
    state.expect(engine.get_rpm() <= real_t{4400.0} + real_t{1.0},
                 "VehicleEngine emergency RPM ceiling remains bounded");

    Turbo limiter_turbo;
    limiter_turbo.configure(make_turbo_data());
    VehicleEngine turbo_limited(make_engine_data());
    turbo_limited.set_turbo(&limiter_turbo);
    turbo_limited.set_angular_velocity(rpm_to_omega(real_t{4000.0}));
    turbo_limited.throttle = real_t{1.0};
    for (int i = 0; i < 100; ++i) {
        turbo_limited.clear_torque();
        turbo_limited.accumulate_torque(real_t{0.02});
    }
    state.expect(near(turbo_limited.get_torque(), real_t{0.0}) &&
                         turbo_limited.get_turbo_boost() > real_t{0.5},
                 "VehicleEngine limiter cut preserves open-throttle turbo boost");
}

void test_tire_combined_grip(TestState &state) {
    Ref<TireData> defaults = memnew(TireData);
    state.expect(near(defaults->get_combined_grip_exponent(), real_t{2.0}),
                 "TireData combined-grip exponent defaults to p=2");

    defaults->set_combined_grip_exponent(real_t{0.25});
    state.expect(near(defaults->get_combined_grip_exponent(), real_t{1.0}),
                 "TireData combined-grip exponent clamps finite lower bound");
    defaults->set_combined_grip_exponent(real_t{32.0});
    state.expect(near(defaults->get_combined_grip_exponent(), real_t{16.0}),
                 "TireData combined-grip exponent clamps finite upper bound");
    defaults->set_combined_grip_exponent(std::numeric_limits<real_t>::quiet_NaN());
    state.expect(near(defaults->get_combined_grip_exponent(), real_t{2.0}),
                 "TireData combined-grip exponent resets NaN to default");
    defaults->set_combined_grip_exponent(std::numeric_limits<real_t>::infinity());
    state.expect(near(defaults->get_combined_grip_exponent(), real_t{2.0}),
                 "TireData combined-grip exponent resets infinity to default");

    // Flat friction curves make both normalized force components exactly one,
    // so the established p=2 path has a retained vector magnitude of 10 N.
    // p=4 must retain more simultaneous force, and the wheel copies the
    // resource exponent at set_tire time rather than observing later edits.
    Ref<TireData> p2_tire = make_grip_tire(real_t{2.0});
    Wheel *p2_wheel = make_combined_grip_wheel(p2_tire);
    p2_wheel->solve_tire(Vector3(), Vector3(3.0, 0.0, 3.0), Vector3(),
                         real_t{0.1}, real_t{0.0}, false);
    const Vector3 p2_force = p2_wheel->tire_force;
    const real_t p2_magnitude = p2_force.length();
    state.expect(near(p2_force.x, real_t{-7.0710678}, real_t{2e-4}) &&
                     near(p2_force.z, real_t{-7.0710678}, real_t{2e-4}) &&
                     near(p2_magnitude, real_t{10.0}, real_t{2e-4}),
                 "Wheel combined-grip p=2 preserves established ellipse result");
    memdelete(p2_wheel);

    Ref<TireData> p4_tire = make_grip_tire(real_t{4.0});
    Wheel *p4_wheel = make_combined_grip_wheel(p4_tire);
    p4_tire->set_combined_grip_exponent(real_t{2.0});
    p4_wheel->solve_tire(Vector3(), Vector3(3.0, 0.0, 3.0), Vector3(),
                         real_t{0.1}, real_t{0.0}, false);
    const Vector3 p4_force = p4_wheel->tire_force;
    const real_t p4_magnitude = p4_force.length();
    const real_t expected_p4_magnitude = real_t{10.0} * std::pow(real_t{2.0}, real_t{0.25});
    state.expect(p4_magnitude > p2_magnitude + real_t{1.0} &&
                     near(p4_magnitude, expected_p4_magnitude, real_t{2e-4}) &&
                     std::abs(p4_force.x) > std::abs(p2_force.x) &&
                     std::abs(p4_force.z) > std::abs(p2_force.z),
                 "Wheel combined-grip p>2 retains more simultaneous force and copies exponent");
    memdelete(p4_wheel);

    Ref<TireData> grip_tire = make_grip_tire(real_t{2.0});
    Wheel *full_grip = make_combined_grip_wheel(grip_tire);
    Wheel *half_grip = make_combined_grip_wheel(grip_tire);
    Wheel *zero_grip = make_combined_grip_wheel(grip_tire);
    half_grip->set_grip_multiplier(real_t{0.5});
    zero_grip->set_grip_multiplier(real_t{-1.0});
    full_grip->solve_tire(Vector3(), Vector3(0.0, 0.0, 3.0), Vector3(),
                          real_t{0.1}, real_t{0.0}, false);
    half_grip->solve_tire(Vector3(), Vector3(0.0, 0.0, 3.0), Vector3(),
                          real_t{0.1}, real_t{0.0}, false);
    zero_grip->solve_tire(Vector3(), Vector3(0.0, 0.0, 3.0), Vector3(),
                          real_t{0.1}, real_t{0.0}, false);
    state.expect(near(half_grip->tire_force.length(),
                      full_grip->tire_force.length() * real_t{0.5}) &&
                         near(zero_grip->get_grip_multiplier(), real_t{0.0}) &&
                         near(zero_grip->tire_force.length(), real_t{0.0}),
                 "Wheel grip multiplier proportionally scales force and supports zero grip");
    memdelete(full_grip);
    memdelete(half_grip);
    memdelete(zero_grip);
}

void test_skid_marks(TestState &state) {
    TireSkid *ordinary = memnew(TireSkid);
    ordinary->submit_sample(make_skid_sample(real_t{1000.0}, real_t{0.0}),
                            real_t{0.02});
    state.expect(ordinary->get_buffer().size() == 0,
                 "Skid force without dissipative slip produces no mark");
    memdelete(ordinary);

    TireSkid *normal_transients = memnew(TireSkid);
    normal_transients->submit_sample(
            make_skid_sample(real_t{1000.0}, real_t{1.0}), real_t{0.02});
    normal_transients->submit_sample(
            make_skid_sample(real_t{0.0}, real_t{0.0}, real_t{1000.0}, real_t{1.0}),
            real_t{0.02});
    normal_transients->submit_sample(
            make_skid_sample(real_t{700.0}, real_t{1.0}, real_t{700.0}, real_t{1.0}),
            real_t{0.02});
    state.expect(normal_transients->get_buffer().size() == 0,
                 "Skid onset rejects shift transients and ordinary turning scrub");
    memdelete(normal_transients);

    TireSkid *independent_modes = memnew(TireSkid);
    independent_modes->submit_sample(
            make_skid_sample(real_t{-1000.0}, real_t{-3.0}), real_t{0.02});
    state.expect(independent_modes->get_buffer().size() == 2,
                 "Skid braking slip independently starts a mark");
    independent_modes->stop_skid();
    SkidContactSample lateral =
            make_skid_sample(real_t{0.0}, real_t{0.0}, real_t{1000.0}, real_t{3.0});
    lateral.contact_position.z = real_t{0.2};
    independent_modes->submit_sample(lateral, real_t{0.02});
    state.expect(independent_modes->get_buffer().size() == 4 &&
                         !independent_modes->get_buffer().section_at(2).connected_to_previous,
                 "Skid lateral slip independently starts a disconnected mark");
    memdelete(independent_modes);

    TireSkid *stationary = memnew(TireSkid);
    SkidContactSample burnout = make_skid_sample(real_t{1000.0}, real_t{3.0});
    stationary->submit_sample(burnout, real_t{0.02});
    const real_t initial_patch_intensity = stationary->get_buffer().section_at(1).intensity;
    for (int i = 0; i < 20; ++i)
        stationary->submit_sample(burnout, real_t{0.02});
    state.expect(stationary->get_buffer().size() == 2 &&
                         stationary->get_buffer().section_at(1).intensity >
                                 initial_patch_intensity,
                 "Stationary burnout reuses and darkens one short patch");
    memdelete(stationary);

    TireSkid *moving = memnew(TireSkid);
    SkidContactSample moving_sample = make_skid_sample(real_t{1000.0}, real_t{3.0});
    moving->submit_sample(moving_sample, real_t{0.02});
    moving_sample.ground_velocity = Vector3(0.0, 0.0, 1.0);
    moving_sample.contact_position.z = real_t{0.05};
    moving->submit_sample(moving_sample, real_t{0.02});
    moving_sample.contact_position.z = real_t{0.10};
    moving->submit_sample(moving_sample, real_t{0.02});
    state.expect(moving->get_buffer().size() == 3 &&
                         near(moving->get_buffer().section_at(2).center.z,
                              real_t{0.10}),
                 "Moving skid appends cross-sections at 0.10 metre spacing");
    memdelete(moving);

    TireSkid *breaks = memnew(TireSkid);
    SkidContactSample active = make_skid_sample(real_t{1000.0}, real_t{3.0});
    breaks->submit_sample(active, real_t{0.02});
    breaks->stop_skid();
    active.contact_position.z = real_t{0.2};
    breaks->submit_sample(active, real_t{0.02});
    SkidContactSample released = active;
    released.longitudinal_force = real_t{0.0};
    released.longitudinal_slip_velocity = real_t{0.0};
    breaks->submit_sample(released, real_t{0.13});
    active.contact_position.z = real_t{0.4};
    breaks->submit_sample(active, real_t{0.02});
    active.contact_position.z = real_t{3.0};
    breaks->submit_sample(active, real_t{0.02});
    state.expect(breaks->get_buffer().size() == 8 &&
                         !breaks->get_buffer().section_at(2).connected_to_previous &&
                         !breaks->get_buffer().section_at(4).connected_to_previous &&
                         !breaks->get_buffer().section_at(6).connected_to_previous,
                 "Skid contact loss, release timeout, and teleport create segment breaks");
    memdelete(breaks);

    SkidMarkBuffer ring;
    for (std::size_t i = 0; i < SkidMarkBuffer::CAPACITY + 2; ++i) {
        SkidMarkSection section;
        section.center.x = static_cast<real_t>(i);
        section.connected_to_previous = true;
        ring.push(section);
    }
    state.expect(ring.size() == SkidMarkBuffer::CAPACITY &&
                         near(ring.section_at(0).center.x, real_t{2.0}) &&
                         !ring.section_at(0).connected_to_previous &&
                         near(ring.section_at(ring.size() - 1).center.x,
                              real_t{2049.0}),
                 "Skid ring deterministically replaces the oldest cross-section");

    TireSkid *rendered = memnew(TireSkid);
    rendered->_ready();
    SkidContactSample half_intensity =
            make_skid_sample(real_t{1000.0}, real_t{4.0});
    rendered->submit_sample(half_intensity, real_t{0.02});
    rendered->_process(0.0);
    rendered->_process(0.0);
    rendered->_process(0.0);
    MeshInstance3D *mesh_instance = Object::cast_to<MeshInstance3D>(
            rendered->get_node_or_null(NodePath("SkidMesh")));
    bool mesh_matches = false;
    if (mesh_instance != nullptr && mesh_instance->get_mesh().is_valid() &&
        mesh_instance->get_mesh()->get_surface_count() == 1) {
        const Array arrays = mesh_instance->get_mesh()->surface_get_arrays(0);
        const PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
        const PackedColorArray colors = arrays[Mesh::ARRAY_COLOR];
        const PackedInt32Array indices = arrays[Mesh::ARRAY_INDEX];
        mesh_matches = vertices.size() == 4 && colors.size() == 4 &&
                       indices.size() == 6 &&
                       near(vertices[0].distance_to(vertices[1]), real_t{0.15}) &&
                       near(colors[0].a, real_t{0.5}, real_t{1.0 / 255.0});
        if (!mesh_matches) {
            UtilityFunctions::printerr(
                    "[drivetrain-regression] skid mesh detail: vertices=", vertices.size(),
                    " colors=", colors.size(), " indices=", indices.size(),
                    " width=", vertices.size() >= 2
                            ? vertices[0].distance_to(vertices[1])
                            : real_t{-1.0},
                    " alpha=", colors.size() >= 1 ? colors[0].a : real_t{-1.0});
        }
    }
    state.expect(mesh_matches,
                 "Skid mesh emits one tire-width quad with vertex-alpha intensity");
    memdelete(rendered);

    Ref<TireData> falling_tire = make_grip_tire(real_t{2.0});
    Ref<Curve> falling_curve = memnew(Curve);
    falling_curve->add_point(Vector2(0.0, 0.0));
    falling_curve->add_point(Vector2(1.0, 1.0));
    falling_curve->add_point(Vector2(2.0, 0.4));
    falling_tire->set_forward_friction_curve(falling_curve);
    Wheel *extreme_wheel = memnew(Wheel);
    extreme_wheel->set_suspension(real_t{1.0}, real_t{1.0}, real_t{1.0},
                                  real_t{1000.0});
    extreme_wheel->set_tire(falling_tire);
    extreme_wheel->on_ground = true;
    extreme_wheel->collision_normal = Vector3(0.0, 1.0, 0.0);
    extreme_wheel->forward_vector = Vector3(0.0, 0.0, 1.0);
    extreme_wheel->right_vector = Vector3(1.0, 0.0, 0.0);
    extreme_wheel->set_normal_force(real_t{1000.0});
    set_wheel_angular_velocity(extreme_wheel, real_t{30.0});
    extreme_wheel->solve_tire(Vector3(), Vector3(), Vector3(), real_t{0.1},
                              real_t{0.0}, false);
    state.expect(extreme_wheel->skid != nullptr &&
                         extreme_wheel->skid->get_buffer().size() == 2,
                 "Wheel extreme slip marks below the tire curve peak");
    memdelete(extreme_wheel);

    Turbo high_power_turbo;
    high_power_turbo.configure(make_turbo_data(real_t{2.0}, real_t{3000.0},
                                                real_t{0.05}));
    for (int i = 0; i < 20; ++i)
        high_power_turbo.update(real_t{0.02}, real_t{3000.0}, real_t{1.0});

    Wheel *powered_wheel = memnew(Wheel);
    powered_wheel->set_suspension(real_t{1.0}, real_t{1.0}, real_t{1.0},
                                  real_t{1000.0});
    powered_wheel->set_tire(falling_tire);
    powered_wheel->on_ground = true;
    powered_wheel->collision_normal = Vector3(0.0, 1.0, 0.0);
    powered_wheel->forward_vector = Vector3(0.0, 0.0, 1.0);
    powered_wheel->right_vector = Vector3(1.0, 0.0, 0.0);
    powered_wheel->set_normal_force(real_t{1000.0});
    for (int i = 0; i < 12; ++i) {
        powered_wheel->collision_point.z = static_cast<real_t>(i) * real_t{0.1};
        powered_wheel->add_drive_torque(real_t{1000.0} *
                                        high_power_turbo.get_air_charge_ratio());
        powered_wheel->integrate_rotation(real_t{0.02});
        powered_wheel->tire_force = Vector3();
        powered_wheel->solve_tire(Vector3(), Vector3(0.0, 0.0, 1.0), Vector3(),
                                  real_t{0.02}, real_t{0.0}, false);
    }
    state.expect(high_power_turbo.get_boost() > real_t{1.0} &&
                         powered_wheel->skid->get_buffer().size() >= 8,
                 "Boosted wheel torque sustains high-slip skid sections");
    memdelete(powered_wheel);
}

void test_tuning_resources(TestState &state) {
    Ref<TireData> tire = memnew(TireData);
    state.expect(near(tire->get_load_sensitivity(), real_t{0.1}) &&
                     near(tire->get_relaxation_low(), real_t{0.042}) &&
                     near(tire->get_relaxation_high(), real_t{0.01}) &&
                     near(tire->get_mechanical_trail(), real_t{0.02}) &&
                     near(tire->get_pneumatic_trail(), real_t{0.02}),
                 "Readable tire defaults compile to the established SI settings");
    tire->set_load_grip_loss_percent(real_t{10.0});
    state.expect(near(std::pow(real_t{2.0}, -tire->get_load_sensitivity()), real_t{0.9}),
                 "10 percent load loss gives 90 percent coefficient at twice reference load");
    tire->set_load_grip_loss_percent(real_t{100.0});
    state.expect(near(tire->get_load_sensitivity(), real_t{0.3}),
                 "Load-loss percentage remains within the supported exponent range");
    tire->set_aligning_trail_mm(real_t{80.0});
    tire->set_aligning_trail_retained_percent(real_t{25.0});
    state.expect(near(tire->get_mechanical_trail(), real_t{0.02}) &&
                     near(tire->get_pneumatic_trail(), real_t{0.06}),
                 "Aligning lever and retained fraction compile to mechanical and pneumatic trail");
    tire->set_aligning_trail_retained_percent(real_t{100.0});
    state.expect(near(tire->get_mechanical_trail(), real_t{0.08}) &&
                     near(tire->get_pneumatic_trail(), real_t{0.0}),
                 "Full aligning retention removes pneumatic rolloff");
    tire->set_aligning_trail_retained_percent(real_t{0.0});
    state.expect(near(tire->get_mechanical_trail(), real_t{0.0}) &&
                     near(tire->get_pneumatic_trail(), real_t{0.08}),
                 "Zero aligning retention removes residual mechanical trail");
    tire->set_aligning_trail_mm(real_t{0.0});
    state.expect(near(tire->get_mechanical_trail() + tire->get_pneumatic_trail(), real_t{0.0}),
                 "Zero aligning lever produces no aligning torque");
    tire->set_force_response_low_speed_ms(-real_t{1.0});
    tire->set_force_response_108_kph_ms(std::numeric_limits<real_t>::quiet_NaN());
    tire->set_lateral_response_angle(real_t{0.0});
    state.expect(near(tire->get_relaxation_low(), real_t{0.0}) &&
                     near(tire->get_relaxation_high(), real_t{0.01}) &&
                     tire->get_lateral_response_angle() > real_t{0.0},
                 "Tire controls reject invalid domains and reset nonfinite response values");

    for (int steps : {1, 2, 8}) {
        Ref<TireData> response_tire = make_grip_tire(real_t{2.0});
        response_tire->set_lateral_friction_curve(Ref<Curve>());
        response_tire->set_force_response_low_speed_ms(real_t{42.0});
        response_tire->set_force_response_108_kph_ms(real_t{42.0});
        Wheel *wheel = make_combined_grip_wheel(response_tire);
        for (int i = 0; i < steps; ++i) {
            wheel->solve_tire(Vector3(), Vector3(0.0, 0.0, 3.0), Vector3(),
                             real_t{0.042} / static_cast<real_t>(steps), real_t{0.0}, false);
        }
        state.expect(near(wheel->prev_longitudinal_force,
                          -real_t{10.0} * (real_t{1.0} - std::exp(-real_t{1.0})), real_t{2e-4}),
                     "Tire force reaches 63 percent in one time constant across substeps");
        memdelete(wheel);
    }
    Ref<TireData> immediate_tire = make_grip_tire(real_t{2.0});
    immediate_tire->set_lateral_friction_curve(Ref<Curve>());
    immediate_tire->set_force_response_low_speed_ms(real_t{0.0});
    immediate_tire->set_force_response_108_kph_ms(real_t{0.0});
    Wheel *immediate_wheel = make_combined_grip_wheel(immediate_tire);
    immediate_wheel->solve_tire(Vector3(), Vector3(0.0, 0.0, 3.0), Vector3(),
                               real_t{0.001}, real_t{0.0}, false);
    state.expect(near(immediate_wheel->prev_longitudinal_force, -real_t{10.0}),
                 "Zero tire response time explicitly applies force immediately");
    memdelete(immediate_wheel);

    Ref<SteeringRackData> steering = memnew(SteeringRackData);
    steering->set_friction_torque(real_t{0.0});
    steering->set_road_feedback_strength(real_t{0.0});
    for (int steps : {1, 2, 8, 40}) {
        for (real_t input : {-real_t{0.4}, real_t{0.4}}) {
            SteeringRack rack;
            rack.load(steering);
            for (int i = 0; i < steps; ++i)
                rack.solve(input, real_t{0.0}, real_t{0.16} / static_cast<real_t>(steps));
            state.expect(near(rack.get_angle(), input * Math::deg_to_rad(real_t{35.0}) * real_t{0.9}),
                         "Steering reaches 90 percent at its response time across substeps and signs");
            for (int i = 0; i < 20; ++i) {
                rack.solve(input, real_t{0.0}, real_t{0.1});
                state.expect(std::abs(rack.get_angle()) <= std::abs(input) * Math::deg_to_rad(real_t{35.0}) + kEpsilon,
                             "Smooth steering does not overshoot its command");
            }
        }
    }
    SteeringRack speed_rack;
    speed_rack.load(steering);
    speed_rack.solve(real_t{1.0}, real_t{0.0}, real_t{2.0}, real_t{50.0});
    state.expect(near(speed_rack.get_angle(), Math::deg_to_rad(real_t{17.5})),
                 "Configured half-speed halves steering command");
    steering->set_steering_half_speed_kph(real_t{0.0});
    SteeringRack constant_rack;
    constant_rack.load(steering);
    constant_rack.solve(real_t{1.0}, real_t{0.0}, real_t{2.0}, real_t{200.0});
    state.expect(near(constant_rack.get_angle(), Math::deg_to_rad(real_t{35.0})),
                 "Zero half-speed disables speed-sensitive steering");
    steering->set_road_feedback_strength(real_t{0.5});
    for (real_t response_ms : {real_t{100.0}, real_t{500.0}}) {
        steering->set_response_time_ms(response_ms);
        SteeringRack feedback_rack;
        feedback_rack.load(steering);
        for (int i = 0; i < 600; ++i)
            feedback_rack.solve(real_t{1.0}, -real_t{10.0}, real_t{1.0} / real_t{120.0});
        state.expect(near(feedback_rack.get_angle(), Math::deg_to_rad(real_t{35.0}) - real_t{0.0125}),
                     "Changing steering response time preserves road-feedback deflection scaling");
    }
    steering->set_response_time_ms(std::numeric_limits<real_t>::infinity());
    state.expect(near(steering->get_response_time_ms(), real_t{160.0}),
                 "Nonfinite steering response time resets to a safe default");

    Ref<DifferentialData> differential = memnew(DifferentialData);
    differential->set_mode(DifferentialData::LIMITED_SLIP);
    state.expect(near(differential->get_power_lock_ratio(), real_t{0.35}) &&
                     near(differential->get_coast_lock_ratio(), real_t{0.15}) &&
                     near(differential->get_slip_sensitive_gain(), real_t{2.0}),
                 "Readable differential defaults compile to the established constraint settings");
    differential->set_acceleration_lock_percent(real_t{40.0});
    differential->set_engine_braking_lock_percent(real_t{20.0});
    differential->set_preload_torque(real_t{0.0});
    differential->set_speed_lock_torque_per_100_rpm(real_t{0.0});
    const DifferentialSettings settings = DifferentialSettings::from_resource(differential);
    state.expect(near(settings.capacity(real_t{100.0}, real_t{1.0}, real_t{0.0}), real_t{20.0}) &&
                     near(settings.capacity(-real_t{100.0}, real_t{1.0}, real_t{0.0}), real_t{10.0}) &&
                     near(settings.capacity(-real_t{100.0}, -real_t{1.0}, real_t{0.0}), real_t{20.0}),
                 "Lock percentages include the factor of two and preserve reverse power/coast selection");
    differential->set_acceleration_lock_percent(real_t{1000.0});
    differential->set_speed_lock_torque_per_100_rpm(-real_t{1.0});
    state.expect(near(differential->get_power_lock_ratio(), real_t{0.5}) &&
                     near(differential->get_slip_sensitive_gain(), real_t{0.0}),
                 "Differential percentages and speed coupling enforce valid domains");
}

void test_steering_sat(TestState &state) {
    Ref<TireData> tire = make_tire_data();
    tire->set_forward_friction_curve(Ref<Curve>());
    tire->set_lateral_friction_curve(Ref<Curve>());
    tire->set_friction_forward(real_t{1.0});
    tire->set_friction_lateral(real_t{1.0});
    tire->set_load_grip_loss_percent(real_t{0.0});
    tire->set_lateral_response_angle(real_t{10.0});
    // Keep relaxation slower than the test step so SAT must consume the
    // final relaxed lateral force rather than the raw tire force.
    tire->set_force_response_low_speed_ms(real_t{1000.0});
    tire->set_force_response_108_kph_ms(real_t{1000.0});
    tire->set_aligning_trail_mm(real_t{70.0});
    tire->set_aligning_trail_retained_percent(real_t{100.0} / real_t{7.0});

    Wheel *wheel = make_combined_grip_wheel(tire);
    Wheel *negative_wheel = make_combined_grip_wheel(tire);
    const auto solve_at_slip = [&](Wheel *target_wheel, real_t slip_angle_degrees) {
        const real_t slip_angle_radians = slip_angle_degrees * kPi / real_t{180.0};
        target_wheel->tire_force = Vector3();
        target_wheel->solve_tire(Vector3(),
                                 Vector3(std::tan(slip_angle_radians) * real_t{2.5}, 0.0, 0.0),
                                 Vector3(), real_t{0.1}, real_t{0.0}, false);
        // Wheel's point-force convention applies -right_tangent*lateral_force;
        // this world-space force is the value used by the Mz sign equation.
        const real_t final_lateral_force = target_wheel->tire_force.x;
        const real_t normalized_slip = std::clamp(
            std::abs(target_wheel->slip_angle) / tire->get_lateral_response_angle(),
            real_t{0.0}, real_t{1.0});
        const real_t expected_trail = tire->get_mechanical_trail() +
                                      tire->get_pneumatic_trail() *
                                          (real_t{1.0} - normalized_slip) *
                                          (real_t{1.0} - normalized_slip);
        return std::pair<real_t, real_t>(
            target_wheel->self_aligning_torque, -final_lateral_force * expected_trail);
    };

    const auto low_slip = solve_at_slip(wheel, real_t{2.0});
    const auto mid_slip = solve_at_slip(wheel, real_t{5.0});
    const auto high_slip = solve_at_slip(wheel, real_t{20.0});
    const auto negative_high_slip = solve_at_slip(negative_wheel, real_t{-20.0});
    state.expect(near(low_slip.first, low_slip.second, real_t{2e-4}) &&
                     near(mid_slip.first, mid_slip.second, real_t{2e-4}) &&
                     near(high_slip.first, high_slip.second, real_t{2e-4}),
                 "Wheel SAT follows final lateral force and quadratic pneumatic-trail rolloff");
    state.expect(near(negative_high_slip.first, negative_high_slip.second, real_t{2e-4}),
                 "Wheel SAT sign equation also holds for negative slip");
    state.expect(std::abs(high_slip.first) > real_t{0.0},
                 "Wheel SAT retains residual mechanical trail at and beyond peak slip");
    state.expect(high_slip.first * wheel->tire_force.x < real_t{0.0},
                 "Wheel SAT opposes positive world lateral force toward center");
    state.expect(negative_high_slip.first * negative_wheel->tire_force.x < real_t{0.0},
                 "Wheel SAT opposes negative world lateral force toward center");

    memdelete(wheel);
    memdelete(negative_wheel);

    Ref<SteeringRackData> rack_data = memnew(SteeringRackData);
    rack_data->set_friction_torque(real_t{0.0});
    rack_data->set_max_angle(real_t{30.0});
    rack_data->set_response_time_ms(real_t{200.0});
    rack_data->set_road_feedback_strength(real_t{1.0});

    SteeringRack positive_rack;
    positive_rack.load(rack_data);
    positive_rack.solve(real_t{1.0}, real_t{-100000.0}, real_t{0.01}, real_t{0.0});
    state.expect(near(positive_rack.get_angle(), real_t{0.0}),
                 "Opposing SAT caps at positive driver restoring authority");

    SteeringRack negative_rack;
    negative_rack.load(rack_data);
    negative_rack.solve(real_t{-1.0}, real_t{100000.0}, real_t{0.01}, real_t{0.0});
    state.expect(near(negative_rack.get_angle(), real_t{0.0}),
                 "Opposing SAT caps at negative driver restoring authority");

    SteeringRack neutral_rack;
    neutral_rack.load(rack_data);
    neutral_rack.solve(real_t{1.0}, real_t{0.0}, real_t{0.01}, real_t{0.0});

    SteeringRack assisting_rack;
    assisting_rack.load(rack_data);
    assisting_rack.solve(real_t{1.0}, real_t{100000.0}, real_t{0.01}, real_t{0.0});
    state.expect(near(assisting_rack.get_angle(), kPi / real_t{6.0}),
                 "Assisting SAT remains unrestricted up to rack travel limit");

    SteeringRack zero_command_rack;
    zero_command_rack.load(rack_data);
    zero_command_rack.solve(real_t{0.0}, real_t{100000.0}, real_t{0.01}, real_t{0.0});
    state.expect(near(zero_command_rack.get_angle(), real_t{0.0}),
                 "SAT cannot initiate rack motion without driver restoring torque");
}

void test_vehicle_center_of_mass_marker(TestState &state) {
    const Transform3D vehicle_transform(
            Basis(Vector3(0.0, 1.0, 0.0), real_t{0.6}),
            Vector3(10.0, 2.0, -3.0));
    const Vector3 marker_local(1.25, -0.5, 0.75);

    SceneTree *scene_tree = Object::cast_to<SceneTree>(
            Engine::get_singleton()->get_main_loop());
    Window *scene_root = scene_tree != nullptr ? scene_tree->get_root() : nullptr;
    state.expect(scene_root != nullptr,
                 "Vehicle COM fixture has a live SceneTree root");
    if (scene_root == nullptr)
        return;

    Vehicle *marked_vehicle = memnew(Vehicle);
    marked_vehicle->set_config(make_valid_config());
    marked_vehicle->set_mass(real_t{1200.0});
    marked_vehicle->set_global_transform(vehicle_transform);
    Marker3D *marker = memnew(Marker3D);
    marker->set_position(marker_local);
    marked_vehicle->add_child(marker);
    marked_vehicle->add_child(make_unready_axle(real_t{1.0}));
    marked_vehicle->set_center_of_mass_marker(marker);
    scene_root->add_child(marked_vehicle);
    const Vector3 marked_com = marked_vehicle->get_center_of_mass();
    state.expect(marked_vehicle->get_center_of_mass_mode() == RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM &&
                     near(marked_com.x, marker_local.x) && near(marked_com.y, marker_local.y) &&
                     near(marked_com.z, marker_local.z),
                 "Vehicle marker assigns transformed global position as local custom COM");
    scene_root->remove_child(marked_vehicle);
    memdelete(marked_vehicle);

    Vehicle *manual_vehicle = memnew(Vehicle);
    manual_vehicle->set_config(make_valid_config());
    manual_vehicle->set_mass(real_t{1200.0});
    const Vector3 manual_com(0.2, -0.3, 0.4);
    manual_vehicle->set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
    manual_vehicle->set_center_of_mass(manual_com);
    manual_vehicle->add_child(make_unready_axle(real_t{1.0}));
    scene_root->add_child(manual_vehicle);
    const Vector3 retained_com = manual_vehicle->get_center_of_mass();
    state.expect(manual_vehicle->get_center_of_mass_marker() == nullptr &&
                     manual_vehicle->get_center_of_mass_mode() == RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM &&
                     near(retained_com.x, manual_com.x) && near(retained_com.y, manual_com.y) &&
                     near(retained_com.z, manual_com.z),
                 "Vehicle null marker preserves manual custom COM");
    scene_root->remove_child(manual_vehicle);
    memdelete(manual_vehicle);
}

void test_vehicle_runtime_restart(TestState &state) {
    SceneTree *scene_tree = Object::cast_to<SceneTree>(
            Engine::get_singleton()->get_main_loop());
    Window *scene_root = scene_tree != nullptr ? scene_tree->get_root() : nullptr;
    state.expect(scene_root != nullptr,
                 "Vehicle restart fixture has a live SceneTree root");
    if (scene_root == nullptr)
        return;

    Ref<VehicleConfig> initial_config = make_valid_config();
    Ref<VehicleConfig> replacement_config = make_valid_config();
    replacement_config->get_engine_data()->set_idle_rpm(real_t{1800.0});
    replacement_config->get_engine_data()->set_redline_rpm(real_t{5000.0});
    replacement_config->get_gearbox_data()->set_auto_mode(true);

    Ref<SteeringRackData> rack_data = memnew(SteeringRackData);
    rack_data->set_friction_torque(real_t{0.0});
    rack_data->set_max_angle(real_t{35.0});
    rack_data->set_response_time_ms(real_t{120.0});
    rack_data->set_road_feedback_strength(real_t{0.0});

    Vehicle *vehicle = memnew(Vehicle);
    vehicle->set_config(initial_config);
    vehicle->set_mass(real_t{1200.0});
    Axle *axle = make_unready_axle(real_t{1.0});
    axle->set_steerable(true);
    axle->set_steering_rack_data(rack_data);
    vehicle->add_child(axle);
    scene_root->add_child(vehicle);

    state.expect(!vehicle->get_gearbox_automatic(),
                 "Vehicle public gearbox mode starts from copied manual config");
    vehicle->set_gearbox_automatic(true);
    state.expect(vehicle->get_gearbox_automatic(),
                 "Vehicle public gearbox mode setter enables automatic mode");
    vehicle->set_gearbox_automatic(false);
    state.expect(!vehicle->get_gearbox_automatic(),
                 "Vehicle public gearbox mode setter disables automatic mode");

    const VehicleTelemetrySnapshot initial_snapshot = vehicle->get_telemetry_snapshot();
    state.expect(near(initial_snapshot.engine_rpm, real_t{1000.0}),
                 "Vehicle runtime copies initial engine values at setup");

    const Transform3D preserved_transform(
            Basis(Vector3(0.0, 1.0, 0.0), real_t{0.35}),
            Vector3(4.0, 2.0, -6.0));
    const Vector3 preserved_linear(real_t{3.0}, real_t{-1.0}, real_t{2.0});
    const Vector3 preserved_angular(real_t{0.2}, real_t{-0.4}, real_t{0.6});
    vehicle->set_global_transform(preserved_transform);
    vehicle->set_linear_velocity(preserved_linear);
    vehicle->set_angular_velocity(preserved_angular);

    Callable first_ready = Callable(vehicle, "set_meta").bind(
            StringName("restart_ready_first"), true);
    state.expect(vehicle->connect("vehicle_ready", first_ready) == OK,
                 "Vehicle restart fixture connects vehicle_ready observer");

    vehicle->set_config(replacement_config);
    const VehicleTelemetrySnapshot before_restart = vehicle->get_telemetry_snapshot();
    state.expect(near(before_restart.engine_rpm, real_t{1000.0}) &&
                         !vehicle->get_gearbox_automatic(),
                 "Replacing Vehicle config leaves copied live engine and gearbox unchanged");

    axle->get_tire_data()->set_radius(real_t{0.6});
    rack_data->set_max_angle(real_t{5.0});
    state.expect(vehicle->restart() && vehicle->has_meta("restart_ready_first"),
                 "Vehicle restart applies replacement config and emits vehicle_ready");

    const VehicleTelemetrySnapshot after_restart = vehicle->get_telemetry_snapshot();
    const Transform3D restored_transform = vehicle->get_global_transform();
    const Vector3 restored_linear = vehicle->get_linear_velocity();
    const Vector3 restored_angular = vehicle->get_angular_velocity();
    state.expect(near(after_restart.engine_rpm, real_t{1800.0}) &&
                         vehicle->get_gearbox_automatic(),
                 "Successful Vehicle restart applies new engine and gearbox values");
    state.expect(near(restored_transform.origin.x, preserved_transform.origin.x) &&
                         near(restored_transform.origin.y, preserved_transform.origin.y) &&
                         near(restored_transform.origin.z, preserved_transform.origin.z) &&
                         near(restored_linear.x, preserved_linear.x) &&
                         near(restored_linear.y, preserved_linear.y) &&
                         near(restored_linear.z, preserved_linear.z) &&
                         near(restored_angular.x, preserved_angular.x) &&
                         near(restored_angular.y, preserved_angular.y) &&
                         near(restored_angular.z, preserved_angular.z),
                 "Vehicle restart preserves transform and linear/angular velocity");

    const std::vector<Axle *> &runtime_axles = vehicle->get_wheel_views();
    state.expect(runtime_axles.size() == 1 && runtime_axles[0] == axle &&
                         axle->get_wheels().size() == 2,
                 "Vehicle restart preserves authored axle topology");
    if (runtime_axles.size() == 1 && axle->get_wheels().size() == 2) {
        Wheel *wheel = axle->get_wheels()[0];
        wheel->add_drive_torque(real_t{9.0});
        wheel->integrate_rotation(real_t{0.1});
        state.expect(near(wheel->get_angular_velocity(), real_t{0.25}),
                     "Vehicle restart reapplies in-place authored tire radius");

        axle->solve_steering(real_t{1.0}, real_t{0.1}, real_t{0.0});
        state.expect(axle->get_steer_angle() <= kPi / real_t{36.0} + real_t{1e-4},
                     "Vehicle restart reapplies in-place authored steering rack limits");
    }

    vehicle->disconnect("vehicle_ready", first_ready);
    Callable second_ready = Callable(vehicle, "set_meta").bind(
            StringName("restart_ready_second"), true);
    state.expect(vehicle->connect("vehicle_ready", second_ready) == OK,
                 "Vehicle restart fixture reconnects vehicle_ready observer");

    Ref<GearboxData> valid_replacement_gearbox =
            replacement_config->get_gearbox_data();
    Ref<VehicleConfig> invalid_config = replacement_config;
    invalid_config->set_gearbox_data(Ref<GearboxData>());
    vehicle->set_config(invalid_config);
    const bool invalid_restart = vehicle->restart();
    const Vector3 inert_linear = vehicle->get_linear_velocity();
    const Vector3 inert_angular = vehicle->get_angular_velocity();
    state.expect(!invalid_restart && !vehicle->has_meta("restart_ready_second") &&
                         near(inert_linear.x, preserved_linear.x) &&
                         near(inert_linear.y, preserved_linear.y) &&
                         near(inert_linear.z, preserved_linear.z) &&
                         near(inert_angular.x, preserved_angular.x) &&
                         near(inert_angular.y, preserved_angular.y) &&
                         near(inert_angular.z, preserved_angular.z),
                 "Invalid Vehicle restart fails inertly without emitting ready");

    invalid_config->set_gearbox_data(valid_replacement_gearbox);
    vehicle->set_config(invalid_config);
    state.expect(vehicle->restart() && vehicle->has_meta("restart_ready_second"),
                 "Corrected Vehicle config restarts successfully and emits vehicle_ready again");

    scene_root->remove_child(vehicle);
    memdelete(vehicle);
}

void test_vehicle_tire_telemetry(TestState &state) {
    SceneTree *scene_tree = Object::cast_to<SceneTree>(
            Engine::get_singleton()->get_main_loop());
    Window *scene_root = scene_tree != nullptr ? scene_tree->get_root() : nullptr;
    state.expect(scene_root != nullptr,
                 "Vehicle tire telemetry fixture has a live SceneTree root");
    if (scene_root == nullptr)
        return;

    Vehicle *vehicle = memnew(Vehicle);
    vehicle->set_config(make_valid_config());
    vehicle->set_mass(real_t{1200.0});
    vehicle->add_child(make_unready_axle(real_t{1.0}));
    VehicleTelemetry *telemetry = memnew(VehicleTelemetry);
    vehicle->add_child(telemetry);
    scene_root->add_child(vehicle);
    telemetry->_process(0.0);

    Wheel *right_tire = vehicle->get_wheel_views()[0]->get_wheels()[1];
    state.expect(telemetry->get_wheel_rpms().size() == 2 &&
                         telemetry->get_tire_forces().size() == 2 &&
                         near(telemetry->get_wheel_rpm(right_tire), real_t{0.0}) &&
                         telemetry->get_tire_force(right_tire) == right_tire->get_tire_force(),
                 "Vehicle telemetry exposes bulk arrays and wheel-reference accessors");
    state.expect(near(telemetry->get_wheel_rpm(nullptr), real_t{0.0}) &&
                         telemetry->get_tire_force(nullptr) == Vector3() &&
                         telemetry->get_tire_telemetry(nullptr).is_empty(),
                 "Vehicle telemetry handles a null wheel reference at the public boundary");

    right_tire->set_grip_multiplier(real_t{0.6});
    const Dictionary tire_data = telemetry->get_tire_telemetry(right_tire);
    state.expect(tire_data.has("wheel") && tire_data.has("wheel_rpm") &&
                         tire_data.has("tire_force") && tire_data.has("normal_load") &&
                         tire_data.has("slip_ratio") &&
                         tire_data.has("slip_angle_degrees") &&
                         tire_data.has("grounded") && tire_data.has("sliding") &&
                         tire_data.has("abs_active") &&
                         tire_data.has("contact_position") &&
                         tire_data.has("contact_normal") &&
                         tire_data.has("grip_multiplier") &&
                         near(static_cast<real_t>(tire_data["grip_multiplier"]),
                              real_t{0.6}),
                 "Vehicle telemetry returns one tire's complete public runtime view");

    scene_root->remove_child(vehicle);
    memdelete(vehicle);
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
    state.expect(VehicleSetupValidation::validate(
                         valid_config, {first_driven, second_driven}, error),
                 "Vehicle setup validation accepts two-axle AWD without center configuration");

    Axle *third_driven = make_axle(real_t{0.2});
    state.expect(VehicleSetupValidation::validate(
                         valid_config,
                         {first_driven, second_driven, third_driven}, error),
                 "Vehicle setup validation accepts up to eight driven axles");

    std::vector<Axle *> eight_driven;
    eight_driven.reserve(RotationalNetwork::MAX_AXLES + 1);
    for (std::size_t i = 0; i < RotationalNetwork::MAX_AXLES; ++i)
        eight_driven.push_back(make_axle(real_t{1.0}));
    state.expect(VehicleSetupValidation::validate(valid_config, eight_driven,
                                                  error),
                 "Vehicle setup validation accepts exactly eight driven axles");
    eight_driven.push_back(make_axle(real_t{1.0}));
    state.expect(!VehicleSetupValidation::validate(valid_config, eight_driven,
                                                   error) &&
                         error.find("at most eight driven axles") >= 0,
                 "Vehicle setup validation rejects nine driven axles");
    for (Axle *driven_axle : eight_driven)
        memdelete(driven_axle);
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
    test_rotational_constraint(state);
    test_rotational_network(state);
    test_gearbox(state);
    test_turbo(state);
    test_engine(state);
    test_tire_combined_grip(state);
    test_skid_marks(state);
    test_tuning_resources(state);
    test_steering_sat(state);
    test_vehicle_center_of_mass_marker(state);
    test_vehicle_runtime_restart(state);
    test_vehicle_tire_telemetry(state);
    test_setup_validation(state);
    if (state.failures != 0)
        UtilityFunctions::printerr("[drivetrain-regression] ", state.failures,
                                   " assertion(s) failed");
    else
        UtilityFunctions::print("[drivetrain-regression] all assertions passed");
    return state.failures == 0;
}

} // namespace godot
