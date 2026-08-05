#include "drivetrain_regression.h"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "godot_cpp/classes/curve.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/window.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

#include "Drivetrain/RotationalConstraint.h"
#include "Drivetrain/RotationalNetwork.h"
#include "Drivetrain/Gearbox.h"
#include "Drivetrain/VehicleEngine.h"
#include "Resources/gearbox_data.h"
#include "Resources/differential_data.h"
#include "Resources/suspension_data.h"
#include "Resources/tire_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/vehicle_config.h"
#include "Resources/vehicle_engine_data.h"
#include "axle.h"
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
    data->set_load_sensitivity(real_t{0.0});
    data->set_relaxation_low(real_t{0.001});
    data->set_relaxation_high(real_t{0.001});
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
    test_engine(state);
    test_tire_combined_grip(state);
    test_vehicle_center_of_mass_marker(state);
    test_setup_validation(state);
    if (state.failures != 0)
        UtilityFunctions::printerr("[drivetrain-regression] ", state.failures,
                                   " assertion(s) failed");
    else
        UtilityFunctions::print("[drivetrain-regression] all assertions passed");
    return state.failures == 0;
}

} // namespace godot
