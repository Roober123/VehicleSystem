#include "drivetrain_regression.h"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

#include "axle.h"
#include "wheel.h"
#include "vehicle.h"
#include "Drivetrain/ClutchGearConstraint.h"
#include "Drivetrain/RotationalBody.h"
#include "Drivetrain/ShaftWheelsCouplingConstraint.h"
#include "Drivetrain/VehicleDifferential.h"

namespace godot {

struct DrivetrainRegressionAccess {
    static void configure(Vehicle *vehicle) {
        vehicle->clutch_gearbox.gear_ratios = {real_t{2.0}, real_t{1.0}};
        vehicle->clutch_gearbox.final_drive = real_t{1.0};
        vehicle->clutch_gearbox.current_gear = 1;
    }

    static void set_gear(Vehicle *vehicle, int gear) {
        vehicle->clutch_gearbox.current_gear = gear;
    }

    static void set_final_drive(Vehicle *vehicle, real_t final_drive) {
        vehicle->clutch_gearbox.final_drive = final_drive;
    }

    static void set_axle_drive_ratio(Axle *axle, real_t ratio) {
        axle->drive_ratio = ratio;
    }

    static void set_cache(Vehicle *vehicle, real_t cache, real_t ratio, bool valid) {
        vehicle->reflected_load_cache = cache;
        vehicle->reflected_load_ratio = ratio;
        vehicle->reflected_load_valid = valid;
    }

    static real_t get_cache(const Vehicle *vehicle) {
        return vehicle->reflected_load_cache;
    }

    static bool is_valid(const Vehicle *vehicle) {
        return vehicle->reflected_load_valid;
    }

    static void begin(Vehicle *vehicle) {
        vehicle->_begin_reflected_load_frame();
    }

    static void refresh(Vehicle *vehicle) {
        vehicle->_refresh_reflected_load_cache();
    }
};

namespace {

constexpr real_t kTolerance = real_t{1e-5};

bool approx_equal(real_t actual, real_t expected, real_t tolerance = kTolerance) {
    return std::abs(actual - expected) <= tolerance;
}

struct TestState {
    int failures = 0;

    void expect(bool condition, const char *name) {
        if (condition)
            return;
        ++failures;
        UtilityFunctions::printerr("[drivetrain-regression] FAIL: ", name);
    }
};

/// Construct an axle using only public Godot APIs.  Calling _ready() discovers
/// the child wheel and installs the open differential, so this fixture does
/// not depend on private production state or test-only production accessors.
Axle *make_single_wheel_axle(real_t wheel_inertia) {
    Axle *axle = memnew(Axle);
    Wheel *wheel = memnew(Wheel);
    wheel->body.set_inertia(wheel_inertia);
    axle->set_drive_ratio(real_t{1.0});
    axle->add_child(wheel);
    axle->_ready();
    return axle;
}

bool shaft_projection_case(real_t &final_shaft_velocity,
                           real_t &final_wheel_velocity,
                           real_t &predicted_aggregate_velocity,
                           real_t &predicted_momentum,
                           real_t &final_momentum,
                           real_t &free_energy,
                           real_t &final_energy) {
    RotationalBody shaft;
    shaft.set_inertia(real_t{2.0});
    Axle *axle = make_single_wheel_axle(real_t{4.0});
    Wheel *wheel = axle->get_wheels()[0];
    constexpr real_t dt = real_t{0.5};

    shaft.set_angular_velocity(real_t{4.0});
    wheel->body.set_angular_velocity(real_t{2.0});
    shaft.add_torque(real_t{6.0});
    wheel->body.add_torque(real_t{-2.0});

    ShaftWheelsCouplingConstraint constraint;
    constraint.load_bodies(&shaft, std::vector<Axle *>{axle});
    // The read-only prediction includes the current wheel reaction before
    // clutch solve, without mutating pending torques. With shaft inertia 2,
    // wheel inertia 4, and torques 6/-2, it predicts a 3 rad/s aggregate.
    predicted_aggregate_velocity =
        constraint.get_predicted_aggregate_angular_velocity(dt);
    // The normalized primary route moves four of the six pending shaft Nm to
    // the wheel. Free velocities are therefore 4.5 and 2.25, with a rigid
    // common target of 3 rad/s.
    predicted_momentum = real_t{2.0} * (real_t{4.0} + dt * real_t{2.0} / real_t{2.0}) +
                         real_t{4.0} * (real_t{2.0} + dt * real_t{2.0} / real_t{4.0});
    const real_t free_shaft_velocity = real_t{4.5};
    const real_t free_wheel_velocity = real_t{2.25};
    free_energy = real_t{0.5} * real_t{2.0} * free_shaft_velocity * free_shaft_velocity +
                  real_t{0.5} * real_t{4.0} * free_wheel_velocity * free_wheel_velocity;
    constraint.solve(dt);

    const real_t shaft_correction_torque = shaft.get_torque();
    const real_t wheel_correction_torque = wheel->body.get_torque();
    const bool finite_torques = std::isfinite(shaft_correction_torque) &&
                                std::isfinite(wheel_correction_torque);
    shaft.integrate(dt);
    axle->integrate(dt);
    final_shaft_velocity = shaft.get_angular_velocity();
    final_wheel_velocity = wheel->body.get_angular_velocity();
    final_momentum = real_t{2.0} * final_shaft_velocity +
                     real_t{4.0} * final_wheel_velocity;
    final_energy = real_t{0.5} * real_t{2.0} * final_shaft_velocity * final_shaft_velocity +
                   real_t{0.5} * real_t{4.0} * final_wheel_velocity * final_wheel_velocity;
    memdelete(axle); // Child wheel ownership is released by Axle/Node.
    return finite_torques && std::isfinite(final_shaft_velocity) &&
           std::isfinite(final_wheel_velocity) && std::isfinite(final_momentum) &&
           std::isfinite(final_energy);
}

bool unequal_drag_projection_case(real_t &predicted_aggregate_velocity,
                                  real_t &final_shaft_velocity,
                                  real_t &final_wheel_velocity,
                                  real_t &predicted_momentum,
                                  real_t &final_momentum) {
    RotationalBody shaft;
    shaft.set_inertia(real_t{2.0});
    shaft.set_drag(real_t{0.5});
    Axle *axle = make_single_wheel_axle(real_t{4.0});
    Wheel *wheel = axle->get_wheels()[0];
    wheel->body.set_drag(real_t{1.5});
    constexpr real_t dt = real_t{0.5};

    shaft.set_angular_velocity(real_t{4.0});
    wheel->body.set_angular_velocity(real_t{2.0});
    shaft.add_torque(real_t{6.0});
    wheel->body.add_torque(real_t{-2.0});

    ShaftWheelsCouplingConstraint constraint;
    constraint.load_bodies(&shaft, std::vector<Axle *>{axle});
    predicted_aggregate_velocity =
        constraint.get_predicted_aggregate_angular_velocity(dt);
    // Explicit drag is part of each predicted free velocity:
    // shaft: 4 + .5 * (6 - .5*4) / 2 = 5, wheel:
    // 2 + .5 * (-2 - 1.5*2) / 4 = 1.375. Their aggregate target is 31/12.
    predicted_momentum = real_t{2.0} * real_t{5.0} +
                         real_t{4.0} * real_t{1.375};
    constraint.solve(dt);
    shaft.integrate(dt);
    axle->integrate(dt);

    final_shaft_velocity = shaft.get_angular_velocity();
    final_wheel_velocity = wheel->body.get_angular_velocity();
    final_momentum = real_t{2.0} * final_shaft_velocity +
                     real_t{4.0} * final_wheel_velocity;
    const bool finite = std::isfinite(predicted_aggregate_velocity) &&
                        std::isfinite(final_shaft_velocity) &&
                        std::isfinite(final_wheel_velocity) &&
                        std::isfinite(predicted_momentum) &&
                        std::isfinite(final_momentum);
    memdelete(axle);
    return finite;
}

void test_shaft_constraint(TestState &state) {
    real_t final_shaft_velocity = real_t{0.0};
    real_t final_wheel_velocity = real_t{0.0};
    real_t predicted_aggregate_velocity = real_t{0.0};
    real_t predicted_momentum = real_t{0.0};
    real_t final_momentum = real_t{0.0};
    real_t free_energy = real_t{0.0};
    real_t final_energy = real_t{0.0};
    state.expect(shaft_projection_case(final_shaft_velocity,
                                       final_wheel_velocity,
                                       predicted_aggregate_velocity,
                                       predicted_momentum,
                                       final_momentum,
                                       free_energy,
                                       final_energy),
                 "shaft/wheel projection finite");
    state.expect(approx_equal(predicted_aggregate_velocity, real_t{3.0}),
                 "predicted aggregate includes pending wheel torque");
    state.expect(approx_equal(final_shaft_velocity, real_t{3.0}) &&
                     approx_equal(final_wheel_velocity, real_t{3.0}),
                 "shaft/wheel projection reaches common target");
    state.expect(approx_equal(predicted_momentum, final_momentum, real_t{1e-4}),
                 "shaft/wheel projection conserves momentum");
    state.expect(final_energy <= free_energy + real_t{1e-4},
                 "shaft/wheel projection does not gain energy");

    real_t drag_predicted = real_t{0.0};
    real_t drag_final_shaft = real_t{0.0};
    real_t drag_final_wheel = real_t{0.0};
    real_t drag_predicted_momentum = real_t{0.0};
    real_t drag_final_momentum = real_t{0.0};
    state.expect(unequal_drag_projection_case(
                     drag_predicted, drag_final_shaft, drag_final_wheel,
                     drag_predicted_momentum, drag_final_momentum),
                 "unequal-drag projection finite");
    state.expect(approx_equal(drag_predicted, real_t{31.0} / real_t{12.0}) &&
                     approx_equal(drag_final_shaft, drag_predicted) &&
                     approx_equal(drag_final_wheel, drag_predicted),
                 "unequal-drag projection reaches common velocity");
    state.expect(approx_equal(drag_final_momentum,
                              drag_predicted_momentum, real_t{1e-4}),
                 "unequal-drag projection momentum accounting");
}

real_t clutch_slip_after_step(real_t ratio, real_t inertia_engine,
                              real_t inertia_output, real_t dt,
                              real_t engine_omega, real_t output_omega,
                              real_t engine_torque, real_t reflected_load) {
    RotationalBody engine;
    RotationalBody output;
    engine.set_inertia(inertia_engine);
    output.set_inertia(inertia_output);
    engine.set_angular_velocity(engine_omega);
    output.set_angular_velocity(output_omega);
    // VehicleEngine::accumulate_torque() has already placed this torque on the
    // engine body when ClutchGearConstraint::solve() is called.
    engine.add_torque(engine_torque);

    ClutchGearConstraint clutch;
    clutch.set_engine(&engine);
    clutch.set_output(&output);
    clutch.clutch_engagement = real_t{1.0};
    clutch.clutch_max_torque = real_t{5000.0};
    clutch.final_drive = real_t{1.0};
    clutch.gear_ratios = {std::abs(ratio)};
    clutch.reverse_ratio = ratio;
    clutch.current_gear = ratio < real_t{0.0} ? -1 : 1;
    // Apply the reflected wheel load to the output body as well as passing its
    // engine-side estimate into the solver.  This keeps the fixture's free
    // relative motion identical to the production drivetrain ordering.
    output.add_torque(-reflected_load * ratio);
    clutch.solve(dt, engine_torque, reflected_load);

    engine.integrate(dt);
    output.integrate(dt);
    return engine.get_angular_velocity() - ratio * output.get_angular_velocity();
}

Vehicle *make_reflected_load_vehicle(real_t reaction_torque, bool grounded = true) {
    Vehicle *vehicle = memnew(Vehicle);
    DrivetrainRegressionAccess::configure(vehicle);

    Axle *axle = make_single_wheel_axle(real_t{2.0});
    axle->set_drive_ratio(real_t{0.5});
    Wheel *wheel = axle->get_wheels()[0];
    wheel->on_ground = grounded;
    wheel->reaction_torque = static_cast<float>(reaction_torque);
    vehicle->add_child(axle);
    vehicle->axles.push_back(axle);
    return vehicle;
}

void test_clutch_constraint(TestState &state) {
    // These deterministic load cases drive the unconstrained legacy solver
    // through zero slip.  Both signs are covered in forward and reverse
    // ratios; the expected invariant is monotonic approach, not overshoot.
    const real_t forward_engine_omega = real_t{-5.219};
    const real_t forward_output_omega = real_t{2.505};
    const real_t forward_initial = forward_engine_omega -
                                   real_t{1.0} * forward_output_omega;
    const real_t forward_after = clutch_slip_after_step(
        real_t{1.0}, real_t{1.675}, real_t{1.286}, real_t{0.091},
        forward_engine_omega, forward_output_omega, real_t{-596.4}, real_t{-21.3});
    state.expect(std::abs(forward_after) <= kTolerance &&
                         forward_initial * forward_after >= real_t{0.0},
                 "clutch forward-ratio synchronized slip");

    const real_t reverse_engine_omega = real_t{-2.905};
    const real_t reverse_output_omega = real_t{-4.639};
    const real_t reverse_initial = reverse_engine_omega -
                                   real_t{-2.0} * reverse_output_omega;
    const real_t reverse_after = clutch_slip_after_step(
        real_t{-2.0}, real_t{2.282}, real_t{4.316}, real_t{0.081},
        reverse_engine_omega, reverse_output_omega, real_t{625.2}, real_t{-847.0});
    state.expect(std::abs(reverse_after) <= kTolerance &&
                         reverse_initial * reverse_after >= real_t{0.0},
                 "clutch reverse-ratio synchronized slip");

    // At synchronization the clutch still needs to carry the engine-side
    // holding torque.  Supplying equal engine/load torques must not collapse
    // the clutch impulse to zero merely because current slip is zero.
    RotationalBody engine;
    RotationalBody output;
    engine.set_inertia(real_t{1.0});
    output.set_inertia(real_t{2.0});
    engine.set_angular_velocity(real_t{4.0});
    output.set_angular_velocity(real_t{2.0});
    engine.add_torque(real_t{100.0});

    ClutchGearConstraint clutch;
    clutch.set_engine(&engine);
    clutch.set_output(&output);
    clutch.clutch_engagement = real_t{1.0};
    clutch.clutch_max_torque = real_t{500.0};
    clutch.final_drive = real_t{1.0};
    clutch.gear_ratios = {real_t{2.0}};
    clutch.current_gear = 1;
    clutch.solve(real_t{0.02}, real_t{100.0}, real_t{100.0});

    state.expect(approx_equal(engine.get_torque(), real_t{0.0}),
                 "clutch synchronized net engine accumulator");
    state.expect(approx_equal(output.get_torque(), real_t{200.0}),
                 "clutch synchronized holding output torque");

    // Partial engagement must bound the applied torque while preserving the
    // sign of a nonzero predicted slip.
    RotationalBody partial_engine;
    RotationalBody partial_output;
    partial_engine.set_inertia(real_t{1.0});
    partial_output.set_inertia(real_t{1.0});
    partial_engine.set_angular_velocity(real_t{2.0});
    partial_output.set_angular_velocity(real_t{0.0});
    ClutchGearConstraint partial;
    partial.set_engine(&partial_engine);
    partial.set_output(&partial_output);
    partial.gear_ratios = {real_t{1.0}};
    partial.final_drive = real_t{1.0};
    partial.current_gear = 1;
    partial.clutch_engagement = real_t{0.5};
    partial.clutch_max_torque = real_t{5.0};
    partial.solve(real_t{0.1}, real_t{0.0}, real_t{0.0});
    state.expect(std::abs(partial_engine.get_torque()) <= real_t{2.5} + kTolerance,
                 "clutch partial engagement capacity");
    partial_engine.integrate(real_t{0.1});
    partial_output.integrate(real_t{0.1});
    const real_t partial_slip = partial_engine.get_angular_velocity() -
                                partial_output.get_angular_velocity();
    state.expect(partial_slip >= real_t{0.0} &&
                         std::abs(partial_slip) < real_t{2.0},
                 "clutch partial engagement no-overshoot");

    // Invalid/non-positive dt is a no-op and must not inject NaN/Inf torque.
    RotationalBody invalid_engine;
    RotationalBody invalid_output;
    ClutchGearConstraint invalid;
    invalid.set_engine(&invalid_engine);
    invalid.set_output(&invalid_output);
    invalid.gear_ratios = {real_t{1.0}};
    invalid.final_drive = real_t{1.0};
    invalid.current_gear = 1;
    invalid.solve(real_t{0.0}, real_t{100.0}, real_t{0.0});
    state.expect(approx_equal(invalid_engine.get_torque(), real_t{0.0}) &&
                         approx_equal(invalid_output.get_torque(), real_t{0.0}),
                 "clutch invalid dt guard");

    // Neutral and zero effective ratios are topology no-ops even with a valid
    // timestep; they must not inject coupling torque into either body.
    RotationalBody neutral_engine;
    RotationalBody neutral_output;
    neutral_engine.set_inertia(real_t{1.0});
    neutral_output.set_inertia(real_t{1.0});
    neutral_engine.add_torque(real_t{25.0});
    ClutchGearConstraint neutral;
    neutral.set_engine(&neutral_engine);
    neutral.set_output(&neutral_output);
    neutral.gear_ratios = {real_t{1.0}};
    neutral.current_gear = 0;
    neutral.solve(real_t{0.02}, real_t{25.0}, real_t{50.0});
    state.expect(approx_equal(neutral_engine.get_torque(), real_t{25.0}) &&
                         approx_equal(neutral_output.get_torque(), real_t{0.0}),
                 "clutch neutral no-op");

    neutral.final_drive = real_t{0.0};
    neutral.current_gear = 1;
    neutral.solve(real_t{0.02}, real_t{25.0}, real_t{50.0});
    state.expect(approx_equal(neutral_engine.get_torque(), real_t{25.0}) &&
                         approx_equal(neutral_output.get_torque(), real_t{0.0}),
                 "clutch zero-ratio no-op");
}

void test_reflected_load_cache(TestState &state) {
    constexpr real_t reaction_torque = real_t{-240.0};
    constexpr real_t expected_load = real_t{60.0}; // -(-240) / 2 * 0.5

    for (const int substeps : {1, 3, 8}) {
        Vehicle *vehicle = make_reflected_load_vehicle(reaction_torque);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load starts invalid");

        // The first tire refresh reads the real wheel reaction state; the
        // following frame begins from that production-backed estimate.
        DrivetrainRegressionAccess::refresh(vehicle);
        state.expect(DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), expected_load),
                     "reflected load refreshed from wheel reaction");
        DrivetrainRegressionAccess::begin(vehicle);
        for (int step = 0; step < substeps; ++step) {
            state.expect(DrivetrainRegressionAccess::is_valid(vehicle) &&
                                 approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), expected_load),
                         "reflected load seeded at substep count");
            DrivetrainRegressionAccess::refresh(vehicle);
        }

        // A ratio/gear change invalidates an engine-side estimate expressed in
        // the old coordinate, even while the wheel remains grounded.
        DrivetrainRegressionAccess::set_gear(vehicle, 2);
        DrivetrainRegressionAccess::begin(vehicle);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load reset on gear change");

        // Neutral and airborne topology both explicitly clear stale feedback.
        DrivetrainRegressionAccess::set_gear(vehicle, 0);
        DrivetrainRegressionAccess::begin(vehicle);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load reset in neutral");
        DrivetrainRegressionAccess::set_gear(vehicle, 1);
        vehicle->axles[0]->get_wheels()[0]->on_ground = false;
        DrivetrainRegressionAccess::set_cache(vehicle, expected_load, real_t{2.0}, true);
        DrivetrainRegressionAccess::begin(vehicle);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load reset airborne");

        // Zero effective ratio and no driven axle topology both clear stale
        // engine-side load estimates, even if a previous valid cache exists.
        vehicle->axles[0]->get_wheels()[0]->on_ground = true;
        DrivetrainRegressionAccess::set_cache(vehicle, expected_load, real_t{2.0}, true);
        DrivetrainRegressionAccess::set_final_drive(vehicle, real_t{0.0});
        DrivetrainRegressionAccess::begin(vehicle);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load reset zero ratio");

        DrivetrainRegressionAccess::set_final_drive(vehicle, real_t{1.0});
        DrivetrainRegressionAccess::set_axle_drive_ratio(vehicle->axles[0], real_t{0.0});
        DrivetrainRegressionAccess::set_cache(vehicle, expected_load, real_t{2.0}, true);
        DrivetrainRegressionAccess::begin(vehicle);
        state.expect(!DrivetrainRegressionAccess::is_valid(vehicle) &&
                             approx_equal(DrivetrainRegressionAccess::get_cache(vehicle), real_t{0.0}),
                     "reflected load reset no driven axle");

        memdelete(vehicle);
    }
}

void test_open_differential(TestState &state) {
    Open_Differential differential;
    for (size_t count = 0; count <= 4; ++count) {
        std::vector<Wheel *> wheels(count, nullptr);
        const std::array<float, 4> &distribution = differential.update(wheels);
        float sum = 0.0f;
        for (size_t i = 0; i < 4; ++i) {
            const float expected = i < count ? 1.0f / static_cast<float>(count) : 0.0f;
            state.expect(std::abs(distribution[i] - expected) <= 1e-6f,
                         "open differential equal torque fractions");
            sum += distribution[i];
        }
        state.expect(count == 0 ? std::abs(sum) <= 1e-6f
                                : std::abs(sum - 1.0f) <= 1e-6f,
                     "open differential fractions sum to one");
    }
}

constexpr int kShiftTraceCapacity = 1024;
constexpr real_t kShiftFrameRate = real_t{120.0};
constexpr real_t kShiftFrameDt = real_t{1.0} / kShiftFrameRate;
constexpr real_t kShiftDuration = real_t{0.5};
constexpr real_t kShiftStartOmega = real_t{4000.0} * real_t{2.0} * Math_PI / real_t{60.0};
constexpr real_t kRepresentativeEngineTorque = real_t{350.0};
// Ignore float32 cancellation-scale sign chatter at synchronization. The
// original coordinate defect crossed by tens of rad/s, far above this gate.
constexpr real_t kReceivingCrossingTolerance = real_t{1e-3};

struct ShiftTrace {
    int samples = 0;
    int sign_crossings = 0;
    int wheel_direction_reversals = 0;
    int free_slip_overshoots = 0;
    real_t first_crossing_before = 0.0;
    real_t first_crossing_after = 0.0;
    real_t first_crossing_ratio = 0.0;
    real_t first_crossing_engagement = 0.0;
    bool finite = true;
    real_t shaft_inertia = 0.0;
    real_t max_abs_slip = 0.0;
    real_t peak_wheel_excursion = 0.0;
    real_t settling_slip = 0.0;
    real_t wheel_settling_slip = 0.0;
    real_t max_impulse_error = 0.0;
    real_t engine_omega[kShiftTraceCapacity]{};
    real_t shaft_omega[kShiftTraceCapacity]{};
    real_t aggregate_omega[kShiftTraceCapacity]{};
    real_t front_omega[kShiftTraceCapacity]{};
    real_t rear_omega[kShiftTraceCapacity]{};
    real_t wheel_omega[4][kShiftTraceCapacity]{};
    real_t engagement[kShiftTraceCapacity]{};
    real_t receiving_slip[kShiftTraceCapacity]{};
    real_t shaft_slip[kShiftTraceCapacity]{};
    real_t wheel_slip[kShiftTraceCapacity]{};
    real_t predicted_free_slip[kShiftTraceCapacity]{};
};

struct ShiftFixture {
    RotationalBody engine;
    RotationalBody driveshaft;
    ClutchGearConstraint clutch;
    ShaftWheelsCouplingConstraint coupling;
    std::array<Axle *, 2> axles{};
    std::array<Wheel *, 4> wheels{};
    real_t initial_wheel_omega = 0.0;

    ShiftFixture(bool upshift) {
        // Match the production example fixture: 0.65 kg m^2 engine inertia,
        // 5:1/3:1 gear ratios, and 3:1 final drive (effective 15 and 9).
        engine.set_inertia(real_t{0.65});
        driveshaft.set_inertia(real_t{1.0});

        for (size_t axle_index = 0; axle_index < axles.size(); ++axle_index) {
            Axle *axle = memnew(Axle);
            axle->set_drive_ratio(real_t{0.5});
            for (size_t wheel_index = 0; wheel_index < 2; ++wheel_index) {
                Wheel *wheel = memnew(Wheel);
                wheel->body.set_inertia(real_t{0.9});
                axle->add_child(wheel);
                wheels[axle_index * 2 + wheel_index] = wheel;
            }
            axle->_ready();
            axles[axle_index] = axle;
        }

        clutch.set_engine(&engine);
        clutch.set_output(&driveshaft);
        clutch.gear_ratios = {real_t{5.0}, real_t{3.0}};
        clutch.reverse_ratio = real_t{-2.0};
        clutch.final_drive = real_t{3.0};
        clutch.current_gear = upshift ? 1 : 2;
        clutch.clutch_engagement = real_t{1.0};
        clutch.clutch_max_torque = real_t{2106.0};

        initial_wheel_omega = kShiftStartOmega / clutch.get_effective_ratio();
        engine.set_angular_velocity(kShiftStartOmega);
        driveshaft.set_angular_velocity(initial_wheel_omega);
        for (Wheel *wheel : wheels)
            wheel->body.set_angular_velocity(initial_wheel_omega);

        coupling.load_bodies(&driveshaft,
                             std::vector<Axle *>{axles[0], axles[1]});
    }

    ~ShiftFixture() {
        for (Axle *axle : axles)
            memdelete(axle);
    }

    real_t wheel_average() const {
        real_t sum = 0.0;
        real_t inertia_sum = 0.0;
        for (Wheel *wheel : wheels) {
            if (wheel == nullptr)
                continue;
            const real_t wheel_inertia = wheel->body.get_inertia();
            if (!std::isfinite(wheel_inertia) || wheel_inertia <= real_t{0.0})
                continue;
            sum += wheel_inertia * wheel->body.get_angular_velocity();
            inertia_sum += wheel_inertia;
        }
        return inertia_sum > real_t{0.0} ? sum / inertia_sum : real_t{0.0};
    }
};

real_t shift_engagement(real_t elapsed, bool &gear_changed, bool upshift) {
    constexpr real_t disengage_duration = real_t{0.1};
    constexpr real_t gear_change_duration = real_t{0.05};
    constexpr real_t reengage_duration = real_t{0.1};
    if (elapsed < disengage_duration) {
        const real_t t = elapsed / disengage_duration;
        return real_t{1.0} - t * t * (real_t{3.0} - real_t{2.0} * t);
    }
    if (elapsed < disengage_duration + gear_change_duration)
        return real_t{0.0};

    if (!gear_changed) {
        gear_changed = true;
    }
    const real_t t = std::clamp(
        (elapsed - disengage_duration - gear_change_duration) /
            reengage_duration,
        real_t{0.0}, real_t{1.0});
    const real_t smooth = t * t * (real_t{3.0} - real_t{2.0} * t);
    // Keep the parameter in the fixture contract explicit: this is a single
    // production-ratio transition; direction only selects the initial gear.
    (void)upshift;
    return smooth;
}

ShiftTrace run_shift_trace(bool upshift, int substeps) {
    ShiftFixture fixture(upshift);
    ShiftTrace trace;
    trace.shaft_inertia = fixture.driveshaft.get_inertia();
    bool gear_changed = false;
    real_t previous_slip = 0.0;
    bool have_previous_slip = false;
    const int frame_count = static_cast<int>(kShiftDuration * kShiftFrameRate);

    for (int frame = 0; frame < frame_count; ++frame) {
        for (int substep = 0; substep < substeps; ++substep) {
            const real_t dt = kShiftFrameDt / static_cast<real_t>(substeps);
            const real_t elapsed = static_cast<real_t>(frame) * kShiftFrameDt +
                                   static_cast<real_t>(substep) * dt;
            const real_t engagement = shift_engagement(elapsed, gear_changed, upshift);
            if (gear_changed)
                fixture.clutch.current_gear = upshift ? 2 : 1;
            fixture.clutch.clutch_engagement = engagement;
            const real_t ratio = fixture.clutch.get_effective_ratio();
            const real_t aggregate_inertia = fixture.coupling.get_aggregate_inertia();
            const real_t aggregate_omega_before =
                fixture.coupling.get_aggregate_angular_velocity();
            fixture.clutch.set_aggregate_output_state(
                aggregate_inertia, aggregate_omega_before);

            // This is the same ordering used by Vehicle::_run_drivetrain_substeps:
            // aggregate prediction, engine torque accumulation, clutch solve,
            // simultaneous shaft/wheel projection, then integration of
            // engine/shaft/wheels.
            const real_t current_clutch_slip =
                fixture.engine.get_angular_velocity() -
                ratio * aggregate_omega_before;
            const real_t predicted_free_slip =
                current_clutch_slip + dt *
                (kRepresentativeEngineTorque / fixture.engine.get_inertia());
            fixture.engine.add_torque(kRepresentativeEngineTorque);
            fixture.clutch.solve(dt, kRepresentativeEngineTorque, real_t{0.0});
            const real_t engine_torque_after_clutch =
                fixture.engine.get_torque() - kRepresentativeEngineTorque;
            const real_t shaft_torque_from_clutch = fixture.driveshaft.get_torque();
            const real_t impulse_error =
                (engine_torque_after_clutch +
                 shaft_torque_from_clutch / ratio) * dt;
            trace.max_impulse_error =
                std::max(trace.max_impulse_error, std::abs(impulse_error));
            fixture.clutch.clear_aggregate_output_state();
            fixture.coupling.solve(dt);

            // Tire solve in production applies the wheel reaction after shaft
            // coupling. A fixed reaction is intentionally omitted here so the
            // trace isolates the clutch/shaft receiving-coordinate transfer.

            fixture.engine.integrate(dt);
            fixture.driveshaft.integrate(dt);
            for (Axle *axle : fixture.axles)
                axle->integrate(dt);

            if (trace.samples < kShiftTraceCapacity) {
                const int sample = trace.samples++;
                const real_t wheel_average = fixture.wheel_average();
                // Clutch receiving-coordinate slip is measured against the
                // physical shaft that receives the clutch impulse.  The
                // wheel-side slip remains recorded separately to expose any
                // residual shaft-to-wheel lag without redefining the clutch
                // coordinate.
                const real_t aggregate_omega_after =
                    fixture.coupling.get_aggregate_angular_velocity();
                const real_t receiving_slip =
                    fixture.engine.get_angular_velocity() - ratio * aggregate_omega_after;
                const real_t shaft_slip =
                    fixture.engine.get_angular_velocity() -
                    ratio * fixture.driveshaft.get_angular_velocity();
                const real_t wheel_slip =
                    fixture.engine.get_angular_velocity() - ratio * wheel_average;
                trace.engine_omega[sample] = fixture.engine.get_angular_velocity();
                trace.shaft_omega[sample] = fixture.driveshaft.get_angular_velocity();
                trace.aggregate_omega[sample] = aggregate_omega_after;
                trace.front_omega[sample] = fixture.axles[0]->get_average_wheel_omega();
                trace.rear_omega[sample] = fixture.axles[1]->get_average_wheel_omega();
                for (size_t wheel = 0; wheel < fixture.wheels.size(); ++wheel)
                    trace.wheel_omega[wheel][sample] =
                        fixture.wheels[wheel]->body.get_angular_velocity();
                trace.engagement[sample] = engagement;
                trace.receiving_slip[sample] = receiving_slip;
                trace.shaft_slip[sample] = shaft_slip;
                trace.wheel_slip[sample] = wheel_slip;
                trace.predicted_free_slip[sample] = predicted_free_slip;

                trace.finite = trace.finite && std::isfinite(receiving_slip) &&
                    std::isfinite(wheel_slip) &&
                    std::isfinite(shaft_slip) &&
                    std::isfinite(aggregate_omega_after) &&
                    std::isfinite(predicted_free_slip) &&
                    std::isfinite(trace.engine_omega[sample]) &&
                    std::isfinite(trace.shaft_omega[sample]) &&
                    std::isfinite(trace.front_omega[sample]) &&
                    std::isfinite(trace.rear_omega[sample]);
                for (size_t wheel = 0; wheel < fixture.wheels.size(); ++wheel)
                    trace.finite = trace.finite &&
                        std::isfinite(trace.wheel_omega[wheel][sample]);
                trace.max_abs_slip = std::max(trace.max_abs_slip,
                                              std::abs(receiving_slip));
                if ((predicted_free_slip > kReceivingCrossingTolerance &&
                     receiving_slip < -kReceivingCrossingTolerance) ||
                    (predicted_free_slip < -kReceivingCrossingTolerance &&
                     receiving_slip > kReceivingCrossingTolerance))
                    ++trace.free_slip_overshoots;
                trace.peak_wheel_excursion = std::max(
                    trace.peak_wheel_excursion,
                    std::abs(wheel_average - fixture.initial_wheel_omega));
                if (have_previous_slip &&
                    std::abs(previous_slip) > kReceivingCrossingTolerance &&
                    std::abs(receiving_slip) > kReceivingCrossingTolerance &&
                    ((previous_slip > real_t{0.0}) !=
                     (receiving_slip > real_t{0.0}))) {
                    ++trace.sign_crossings;
                    if (trace.sign_crossings == 1) {
                        trace.first_crossing_before = previous_slip;
                        trace.first_crossing_after = receiving_slip;
                        trace.first_crossing_ratio = ratio;
                        trace.first_crossing_engagement = engagement;
                    }
                }
                for (size_t wheel = 0; wheel < fixture.wheels.size(); ++wheel) {
                    if (fixture.initial_wheel_omega * trace.wheel_omega[wheel][sample] <
                        real_t{0.0})
                        ++trace.wheel_direction_reversals;
                }
                if (receiving_slip != real_t{0.0}) {
                    previous_slip = receiving_slip;
                    have_previous_slip = true;
                }
            }
        }
    }

    trace.settling_slip = trace.samples > 0
        ? std::abs(trace.receiving_slip[trace.samples - 1])
        : std::numeric_limits<real_t>::infinity();
    trace.wheel_settling_slip = trace.samples > 0
        ? std::abs(trace.wheel_slip[trace.samples - 1])
        : std::numeric_limits<real_t>::infinity();
    return trace;
}

void test_shift_regression(TestState &state) {
    std::array<ShiftTrace, 3> up{};
    std::array<ShiftTrace, 3> down{};
    const std::array<int, 3> substeps = {1, 4, 8};

    for (size_t index = 0; index < substeps.size(); ++index) {
        up[index] = run_shift_trace(true, substeps[index]);
        down[index] = run_shift_trace(false, substeps[index]);
        UtilityFunctions::print(
            "[drivetrain-regression] shift trace up substeps=", substeps[index],
            " crossings=", up[index].sign_crossings,
            " free_slip_overshoots=", up[index].free_slip_overshoots,
            " first_crossing=", up[index].first_crossing_before,
            "->", up[index].first_crossing_after,
            " ratio=", up[index].first_crossing_ratio,
            " engagement=", up[index].first_crossing_engagement,
            " wheel_reversals=", up[index].wheel_direction_reversals,
            " max_slip=", up[index].max_abs_slip,
            " peak_wheel_excursion=", up[index].peak_wheel_excursion,
            " settling_slip=", up[index].settling_slip,
            " wheel_settling_slip=", up[index].wheel_settling_slip,
            " impulse_error=", up[index].max_impulse_error,
            " shaft_inertia=", up[index].shaft_inertia,
            " final_engine=", up[index].engine_omega[up[index].samples - 1],
            " final_shaft=", up[index].shaft_omega[up[index].samples - 1],
            " final_aggregate=", up[index].aggregate_omega[up[index].samples - 1],
            " final_front=", up[index].front_omega[up[index].samples - 1],
            " final_rear=", up[index].rear_omega[up[index].samples - 1],
            " final_wheels=", up[index].wheel_omega[0][up[index].samples - 1],
            ",", up[index].wheel_omega[1][up[index].samples - 1],
            ",", up[index].wheel_omega[2][up[index].samples - 1],
            ",", up[index].wheel_omega[3][up[index].samples - 1],
            " wheel_slip=", up[index].wheel_slip[up[index].samples - 1],
            " shaft_slip=", up[index].shaft_slip[up[index].samples - 1],
            " engagement=", up[index].engagement[up[index].samples - 1]);
        UtilityFunctions::print(
            "[drivetrain-regression] shift trace down substeps=", substeps[index],
            " crossings=", down[index].sign_crossings,
            " free_slip_overshoots=", down[index].free_slip_overshoots,
            " first_crossing=", down[index].first_crossing_before,
            "->", down[index].first_crossing_after,
            " ratio=", down[index].first_crossing_ratio,
            " engagement=", down[index].first_crossing_engagement,
            " wheel_reversals=", down[index].wheel_direction_reversals,
            " max_slip=", down[index].max_abs_slip,
            " peak_wheel_excursion=", down[index].peak_wheel_excursion,
            " settling_slip=", down[index].settling_slip,
            " wheel_settling_slip=", down[index].wheel_settling_slip,
            " impulse_error=", down[index].max_impulse_error,
            " shaft_inertia=", down[index].shaft_inertia,
            " final_engine=", down[index].engine_omega[down[index].samples - 1],
            " final_shaft=", down[index].shaft_omega[down[index].samples - 1],
            " final_aggregate=", down[index].aggregate_omega[down[index].samples - 1],
            " final_front=", down[index].front_omega[down[index].samples - 1],
            " final_rear=", down[index].rear_omega[down[index].samples - 1],
            " final_wheels=", down[index].wheel_omega[0][down[index].samples - 1],
            ",", down[index].wheel_omega[1][down[index].samples - 1],
            ",", down[index].wheel_omega[2][down[index].samples - 1],
            ",", down[index].wheel_omega[3][down[index].samples - 1],
            " wheel_slip=", down[index].wheel_slip[down[index].samples - 1],
            " shaft_slip=", down[index].shaft_slip[down[index].samples - 1],
            " engagement=", down[index].engagement[down[index].samples - 1]);

        state.expect(up[index].finite && down[index].finite,
                     "shift trace finite engine/shaft/wheel state");
        state.expect(approx_equal(up[index].shaft_inertia, real_t{1.0}) &&
                         approx_equal(down[index].shaft_inertia, real_t{1.0}),
                     "shaft inertia remains independently owned");
        state.expect(up[index].max_impulse_error <= real_t{1e-4} &&
                         down[index].max_impulse_error <= real_t{1e-4},
                     "shift clutch impulse conservation");
        // A gear-ratio change may legitimately reverse the state slip target;
        // the limiter invariant is that clutch correction never crosses the
        // *predicted free-slip* zero within one substep.
        state.expect(up[index].free_slip_overshoots == 0 &&
                         down[index].free_slip_overshoots == 0,
                     "clutch correction does not cross predicted free slip");
        state.expect(up[index].wheel_direction_reversals == 0 &&
                         down[index].wheel_direction_reversals == 0,
                     "driven wheel direction remains forward");
        state.expect(up[index].wheel_settling_slip <= real_t{5.0} &&
                         down[index].wheel_settling_slip <= real_t{5.0},
                     "wheel-side slip settles within 5 rad/s");
    }

    const auto relative_difference = [](real_t lhs, real_t rhs) {
        const real_t scale = std::max(std::abs(lhs), std::abs(rhs));
        // Residual settling values are float32 quantized around zero. Treat
        // sub-milliradian-speed differences as numerically settled rather
        // than turning 0 versus one representable tick into a 100% ratio.
        return scale > real_t{1e-3} ? std::abs(lhs - rhs) / scale : real_t{0.0};
    };
    state.expect(relative_difference(up[1].peak_wheel_excursion,
                                     up[2].peak_wheel_excursion) <= real_t{0.05},
                 "upshift substep peak excursion invariance");
    state.expect(relative_difference(down[1].peak_wheel_excursion,
                                     down[2].peak_wheel_excursion) <= real_t{0.05},
                 "downshift substep peak excursion invariance");
    // Settling invariance is an output/wheel gate.  The shaft receiving slip
    // remains recorded above because it is the clutch coordinate, while the
    // wheel-side residual captures the physical tire/output response.
    state.expect(relative_difference(up[1].wheel_settling_slip,
                                     up[2].wheel_settling_slip) <= real_t{0.05},
                 "upshift substep settling invariance");
    state.expect(relative_difference(down[1].wheel_settling_slip,
                                     down[2].wheel_settling_slip) <= real_t{0.05},
                 "downshift substep settling invariance");
}

// DS-REV-01: deterministic grounded-load fixture.  This deliberately keeps
// the front axle present but undriven, matching the rear-only production
// topology that exposed physical driveshaft reversal.  The rear axle has two
// 0.9 kg m^2 wheels (1.8 kg m^2 total), while the shaft and engine retain their
// independent 1.0 and 0.45 kg m^2 inertias.
constexpr real_t kGroundedEngineInertia = real_t{0.45};
constexpr real_t kGroundedShaftInertia = real_t{1.0};
constexpr real_t kGroundedWheelInertia = real_t{0.9};
constexpr real_t kGroundedRearInertia = real_t{1.8};
constexpr real_t kGroundedFirstRatio = real_t{15.0};
constexpr real_t kGroundedSecondRatio = real_t{9.0};
constexpr real_t kGroundedClutchCapacity = real_t{1000.0};
constexpr real_t kGroundedShiftDuration = real_t{0.25};
constexpr real_t kGroundedStartOmega =
    real_t{4000.0} * real_t{2.0} * Math_PI / real_t{60.0};
constexpr real_t kGroundedFrameRate = real_t{120.0};
constexpr real_t kGroundedFrameDt = real_t{1.0} / kGroundedFrameRate;
constexpr real_t kGroundedDirectionTolerance = real_t{1e-4};

struct GroundedShiftTrace {
    int samples = 0;
    bool finite = true;
    bool shaft_direction_reversed = false;
    bool wheel_direction_reversed = false;
    real_t shaft_inertia = 0.0;
    real_t aggregate_inertia = 0.0;
    real_t rear_inertia = 0.0;
    real_t min_driveline_momentum = std::numeric_limits<real_t>::infinity();
    real_t min_total_momentum = std::numeric_limits<real_t>::infinity();
    real_t max_abs_correction = 0.0;
    real_t max_abs_shaft_torque = 0.0;
    real_t max_abs_rear_torque = 0.0;
    real_t max_abs_shaft_omega = 0.0;
    real_t max_abs_wheel_omega = 0.0;
    real_t min_shaft_omega = std::numeric_limits<real_t>::infinity();
    real_t min_rear_omega = std::numeric_limits<real_t>::infinity();
    real_t shaft_omega[kShiftTraceCapacity]{};
    real_t aggregate_omega[kShiftTraceCapacity]{};
    real_t front_omega[kShiftTraceCapacity]{};
    real_t rear_omega[kShiftTraceCapacity]{};
    real_t wheel_omega[4][kShiftTraceCapacity]{};
    real_t engagement[kShiftTraceCapacity]{};
    real_t clutch_torque[kShiftTraceCapacity]{};
    real_t shaft_torque[kShiftTraceCapacity]{};
    real_t rear_torque[kShiftTraceCapacity]{};
    real_t coupling_correction[kShiftTraceCapacity]{};
    real_t driveline_momentum[kShiftTraceCapacity]{};
    real_t total_momentum[kShiftTraceCapacity]{};
};

struct GroundedShiftFixture {
    RotationalBody engine;
    RotationalBody driveshaft;
    ClutchGearConstraint clutch;
    ShaftWheelsCouplingConstraint coupling;
    std::array<Axle *, 2> axles{};
    std::array<Wheel *, 4> wheels{};
    real_t reaction_torque = 0.0;
    real_t initial_wheel_omega = 0.0;

    GroundedShiftFixture(bool upshift, real_t reaction, bool preload) :
            reaction_torque(reaction) {
        engine.set_inertia(kGroundedEngineInertia);
        driveshaft.set_inertia(kGroundedShaftInertia);

        // Front axle remains in the topology but is not driven.  The rear
        // axle is the sole receiving axle, with 1.8 kg m^2 total inertia.
        for (size_t axle_index = 0; axle_index < axles.size(); ++axle_index) {
            Axle *axle = memnew(Axle);
            axle->set_drive_ratio(axle_index == 0 ? real_t{0.0} : real_t{1.0});
            for (size_t wheel_index = 0; wheel_index < 2; ++wheel_index) {
                Wheel *wheel = memnew(Wheel);
                wheel->body.set_inertia(kGroundedWheelInertia);
                wheel->on_ground = axle_index == 1;
                wheel->reaction_torque = axle_index == 1
                    ? static_cast<float>(reaction / real_t{2.0})
                    : 0.0f;
                axle->add_child(wheel);
                wheels[axle_index * 2 + wheel_index] = wheel;
            }
            axle->_ready();
            axles[axle_index] = axle;
        }

        clutch.set_engine(&engine);
        clutch.set_output(&driveshaft);
        clutch.gear_ratios = {kGroundedFirstRatio / real_t{3.0},
                              kGroundedSecondRatio / real_t{3.0}};
        clutch.reverse_ratio = real_t{-2.0};
        clutch.final_drive = real_t{3.0};
        clutch.current_gear = upshift ? 1 : 2;
        clutch.clutch_engagement = real_t{1.0};
        clutch.clutch_max_torque = kGroundedClutchCapacity;

        const real_t initial_ratio = clutch.get_effective_ratio();
        initial_wheel_omega = kGroundedStartOmega / initial_ratio;
        engine.set_angular_velocity(kGroundedStartOmega);
        driveshaft.set_angular_velocity(initial_wheel_omega);
        for (Wheel *wheel : wheels)
            wheel->body.set_angular_velocity(initial_wheel_omega);

        // A steady reaction load preloads the residual twist with the
        // balancing correction T_preload = T_reaction; theta_preload =
        // T_preload / K, K = 9 * I_eff.  Angles are seeded
        // through the public integration API because RotationalBody exposes
        // no test-only angle setter.
        if (preload && reaction_torque != real_t{0.0}) {
            const real_t I_eff = (kGroundedShaftInertia * kGroundedRearInertia) /
                                 (kGroundedShaftInertia + kGroundedRearInertia);
            const real_t coupling_stiffness = real_t{9.0} * I_eff;
            const real_t preload_torque = reaction_torque;
            const real_t preload_angle = preload_torque / coupling_stiffness;
            for (size_t wheel_index = 2; wheel_index < wheels.size(); ++wheel_index) {
                wheels[wheel_index]->body.set_angular_velocity(preload_angle);
                wheels[wheel_index]->body.integrate(real_t{1.0});
                wheels[wheel_index]->body.set_angular_velocity(initial_wheel_omega);
            }
        }

        coupling.load_bodies(&driveshaft,
                             std::vector<Axle *>{axles[0], axles[1]});
    }

    ~GroundedShiftFixture() {
        for (Axle *axle : axles)
            memdelete(axle);
    }

    real_t output_momentum() const {
        real_t momentum = driveshaft.get_inertia() *
                          driveshaft.get_angular_velocity();
        // Only the driven rear axle belongs to this output coordinate.  The
        // front wheels are intentionally present as an undriven topology
        // check and must not make a positive-momentum guard less strict.
        for (size_t wheel_index = 2; wheel_index < wheels.size(); ++wheel_index)
            momentum += wheels[wheel_index]->body.get_inertia() *
                        wheels[wheel_index]->body.get_angular_velocity();
        return momentum;
    }

    real_t total_momentum() const {
        return engine.get_inertia() * engine.get_angular_velocity() +
               output_momentum();
    }
};

real_t grounded_shift_engagement(real_t elapsed, bool &gear_changed,
                                 bool upshift) {
    constexpr real_t disengage_duration = real_t{0.1};
    constexpr real_t gear_change_duration = real_t{0.05};
    constexpr real_t reengage_duration = real_t{0.1};
    if (elapsed < disengage_duration) {
        const real_t t = elapsed / disengage_duration;
        return real_t{1.0} - t * t * (real_t{3.0} - real_t{2.0} * t);
    }
    if (elapsed < disengage_duration + gear_change_duration)
        return real_t{0.0};
    gear_changed = true;
    const real_t t = std::clamp(
        (elapsed - disengage_duration - gear_change_duration) /
            reengage_duration,
        real_t{0.0}, real_t{1.0});
    const real_t smooth = t * t * (real_t{3.0} - real_t{2.0} * t);
    (void)upshift;
    return smooth;
}

GroundedShiftTrace run_grounded_shift_trace(bool upshift, int substeps,
                                             real_t reaction, bool preload) {
    GroundedShiftFixture fixture(upshift, reaction, preload);
    GroundedShiftTrace trace;
    trace.shaft_inertia = fixture.driveshaft.get_inertia();
    trace.aggregate_inertia = fixture.coupling.get_aggregate_inertia();
    trace.rear_inertia = fixture.wheels[2]->body.get_inertia() +
                         fixture.wheels[3]->body.get_inertia();
    bool gear_changed = false;
    const int frame_count = static_cast<int>(kGroundedShiftDuration *
                                             kGroundedFrameRate);
    for (int frame = 0; frame < frame_count; ++frame) {
        for (int substep = 0; substep < substeps; ++substep) {
            const real_t dt = kGroundedFrameDt / static_cast<real_t>(substeps);
            const real_t elapsed = static_cast<real_t>(frame) * kGroundedFrameDt +
                                   static_cast<real_t>(substep) * dt;
            const real_t engagement = grounded_shift_engagement(
                elapsed, gear_changed, upshift);
            fixture.clutch.clutch_engagement = engagement;
            if (gear_changed)
                fixture.clutch.current_gear = upshift ? 2 : 1;

            // No engine drive torque is needed: the external reaction and
            // preloaded initial twist isolate the grounded coupling mode.
            // Match Vehicle::_run_drivetrain_substeps ordering: current tire
            // reaction is pending before the simultaneous coupling projection.
            const real_t reaction_per_wheel = reaction / real_t{2.0};
            for (size_t wheel_index = 2; wheel_index < fixture.wheels.size();
                 ++wheel_index)
                fixture.wheels[wheel_index]->body.add_torque(reaction_per_wheel);

            // Predict the aggregate after this pending wheel reaction, as the
            // production substep does before asking the clutch to synchronize
            // against the receiving bodies.
            const real_t aggregate_before =
                fixture.coupling.get_predicted_aggregate_angular_velocity(dt);
            fixture.clutch.set_aggregate_output_state(
                trace.aggregate_inertia, aggregate_before);

            const real_t shaft_torque_before = fixture.driveshaft.get_torque();
            fixture.clutch.solve(dt, real_t{0.0}, real_t{0.0});
            const real_t clutch_torque = fixture.driveshaft.get_torque() -
                                         shaft_torque_before;
            fixture.clutch.clear_aggregate_output_state();
            const real_t correction = fixture.coupling.solve(dt);
            const real_t shaft_torque_before_integration =
                fixture.driveshaft.get_torque();
            real_t rear_torque_before_integration = real_t{0.0};
            for (size_t wheel_index = 2; wheel_index < fixture.wheels.size();
                 ++wheel_index)
                rear_torque_before_integration +=
                    fixture.wheels[wheel_index]->body.get_torque();

            fixture.engine.integrate(dt);
            fixture.driveshaft.integrate(dt);
            for (Axle *axle : fixture.axles)
                axle->integrate(dt);

            if (trace.samples >= kShiftTraceCapacity)
                continue;
            const int sample = trace.samples++;
            const real_t shaft_omega = fixture.driveshaft.get_angular_velocity();
            const real_t aggregate_omega =
                fixture.coupling.get_aggregate_angular_velocity();
            const real_t front_omega = fixture.axles[0]->get_average_wheel_omega();
            const real_t rear_omega = fixture.axles[1]->get_average_wheel_omega();
            const real_t driveline_momentum = fixture.output_momentum();
            const real_t total_momentum = fixture.total_momentum();
            const real_t rear_torque = rear_torque_before_integration;

            trace.shaft_omega[sample] = shaft_omega;
            trace.aggregate_omega[sample] = aggregate_omega;
            trace.front_omega[sample] = front_omega;
            trace.rear_omega[sample] = rear_omega;
            for (size_t wheel_index = 0; wheel_index < fixture.wheels.size();
                 ++wheel_index)
                trace.wheel_omega[wheel_index][sample] =
                    fixture.wheels[wheel_index]->body.get_angular_velocity();
            trace.engagement[sample] = engagement;
            trace.clutch_torque[sample] = clutch_torque;
            trace.shaft_torque[sample] = shaft_torque_before_integration;
            trace.rear_torque[sample] = rear_torque;
            trace.coupling_correction[sample] = correction;
            trace.driveline_momentum[sample] = driveline_momentum;
            trace.total_momentum[sample] = total_momentum;

            trace.finite = trace.finite && std::isfinite(shaft_omega) &&
                std::isfinite(aggregate_omega) && std::isfinite(front_omega) &&
                std::isfinite(rear_omega) && std::isfinite(clutch_torque) &&
                std::isfinite(shaft_torque_before_integration) &&
                std::isfinite(rear_torque) && std::isfinite(correction) &&
                std::isfinite(driveline_momentum) &&
                std::isfinite(total_momentum);
            for (size_t wheel_index = 0; wheel_index < fixture.wheels.size();
                 ++wheel_index)
                trace.finite = trace.finite &&
                    std::isfinite(trace.wheel_omega[wheel_index][sample]);

            trace.shaft_direction_reversed = trace.shaft_direction_reversed ||
                shaft_omega < -kGroundedDirectionTolerance;
            for (size_t wheel_index = 2; wheel_index < fixture.wheels.size();
                 ++wheel_index)
                trace.wheel_direction_reversed =
                    trace.wheel_direction_reversed ||
                    trace.wheel_omega[wheel_index][sample] <
                        -kGroundedDirectionTolerance;
            trace.min_driveline_momentum = std::min(
                trace.min_driveline_momentum, driveline_momentum);
            trace.min_total_momentum = std::min(
                trace.min_total_momentum, total_momentum);
            trace.max_abs_correction = std::max(
                trace.max_abs_correction, std::abs(correction));
            trace.max_abs_shaft_torque = std::max(
                trace.max_abs_shaft_torque,
                std::abs(trace.shaft_torque[sample]));
            trace.max_abs_rear_torque = std::max(
                trace.max_abs_rear_torque, std::abs(rear_torque));
            trace.max_abs_shaft_omega = std::max(
                trace.max_abs_shaft_omega, std::abs(shaft_omega));
            trace.max_abs_wheel_omega = std::max(
                trace.max_abs_wheel_omega, std::abs(rear_omega));
            trace.min_shaft_omega = std::min(trace.min_shaft_omega, shaft_omega);
            trace.min_rear_omega = std::min(trace.min_rear_omega, rear_omega);
        }
    }
    return trace;
}

void test_grounded_load_regression(TestState &state) {
    struct Scenario {
        real_t reaction;
        bool preload;
        const char *label;
    };
    constexpr std::array<Scenario, 5> scenarios = {{
        {real_t{0.0}, false, "zero"},
        {real_t{-900.0}, false, "live-900"},
        {real_t{-1800.0}, false, "live-1800"},
        {real_t{-900.0}, true, "preloaded-900"},
        {real_t{-1800.0}, true, "preloaded-1800"},
    }};
    constexpr std::array<int, 4> substeps = {1, 4, 8, 16};

    for (const Scenario &scenario : scenarios) {
        for (const int count : substeps) {
            const GroundedShiftTrace up = run_grounded_shift_trace(
                true, count, scenario.reaction, scenario.preload);
            const GroundedShiftTrace down = run_grounded_shift_trace(
                false, count, scenario.reaction, scenario.preload);
            UtilityFunctions::print(
                "[drivetrain-regression] grounded case=", scenario.label,
                " substeps=", count,
                " up shaft_reverse=", up.shaft_direction_reversed,
                " wheel_reverse=", up.wheel_direction_reversed,
                " min_output_momentum=", up.min_driveline_momentum,
                " max_correction=", up.max_abs_correction,
                " max_shaft_torque=", up.max_abs_shaft_torque,
                " max_rear_torque=", up.max_abs_rear_torque,
                " min_shaft=", up.min_shaft_omega,
                " min_rear=", up.min_rear_omega,
                " final_shaft=", up.samples > 0 ? up.shaft_omega[up.samples - 1] : real_t{0.0},
                " final_rear=", up.samples > 0 ? up.rear_omega[up.samples - 1] : real_t{0.0},
                " down shaft_reverse=", down.shaft_direction_reversed,
                " wheel_reverse=", down.wheel_direction_reversed,
                " min_output_momentum=", down.min_driveline_momentum,
                " max_correction=", down.max_abs_correction,
                " min_shaft=", down.min_shaft_omega,
                " min_rear=", down.min_rear_omega,
                " final_shaft=", down.samples > 0 ? down.shaft_omega[down.samples - 1] : real_t{0.0},
                " final_rear=", down.samples > 0 ? down.rear_omega[down.samples - 1] : real_t{0.0});

            state.expect(up.finite && down.finite,
                         "grounded load trace finite");
            state.expect(approx_equal(up.shaft_inertia, kGroundedShaftInertia) &&
                             approx_equal(down.shaft_inertia,
                                          kGroundedShaftInertia),
                         "grounded shaft inertia remains 1.0");
            state.expect(approx_equal(up.aggregate_inertia,
                                      kGroundedShaftInertia + kGroundedRearInertia) &&
                             approx_equal(down.aggregate_inertia,
                                          kGroundedShaftInertia + kGroundedRearInertia),
                         "grounded rear aggregate inertia is 2.8");
            state.expect(approx_equal(up.rear_inertia, kGroundedRearInertia) &&
                             approx_equal(down.rear_inertia,
                                          kGroundedRearInertia),
                         "grounded rear wheel inertia is 1.8");
            if (scenario.reaction == real_t{0.0}) {
                state.expect(!up.shaft_direction_reversed &&
                                 !down.shaft_direction_reversed,
                             "zero-load shaft remains forward");
                state.expect(!up.wheel_direction_reversed &&
                                 !down.wheel_direction_reversed,
                             "zero-load wheels remain forward");
            }

            // The -900 Nm preloaded case starts with positive shaft+wheel
            // momentum even after the full reaction impulse.  A physical
            // coupling must therefore keep both directions forward.  The
            // The former residual angle spring deterministically violated this
            // gate by reversing the low-inertia shaft; the rigid projection is
            // required to keep the positive-momentum case forward.
            if (scenario.reaction == real_t{-900.0} && scenario.preload) {
                state.expect(up.min_driveline_momentum > real_t{0.0} &&
                                 down.min_driveline_momentum > real_t{0.0},
                             "preloaded -900 retains positive output momentum");
                state.expect(!up.shaft_direction_reversed &&
                                 !down.shaft_direction_reversed,
                             "positive output momentum prevents shaft reversal");
                state.expect(!up.wheel_direction_reversed &&
                                 !down.wheel_direction_reversed,
                             "positive output momentum prevents wheel reversal");
            }
        }
    }
}

} // namespace

void DrivetrainRegression::_bind_methods() {
    ClassDB::bind_method(D_METHOD("run"), &DrivetrainRegression::run);
}

bool DrivetrainRegression::run() {
    TestState state;
    test_shaft_constraint(state);
    test_clutch_constraint(state);
    test_reflected_load_cache(state);
    test_open_differential(state);
    test_shift_regression(state);
    test_grounded_load_regression(state);

    if (state.failures != 0)
        UtilityFunctions::printerr("[drivetrain-regression] ", state.failures, " assertion(s) failed");
    else
        UtilityFunctions::print("[drivetrain-regression] all assertions passed");
    return state.failures == 0;
}

} // namespace godot
