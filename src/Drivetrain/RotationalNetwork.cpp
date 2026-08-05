#include "RotationalNetwork.h"

#include <algorithm>
#include <cmath>

namespace godot {
namespace {

bool finite_positive(real_t value) {
	return std::isfinite(value) && value > real_t{0.0};
}

bool finite_nonnegative(real_t value) {
	return std::isfinite(value) && value >= real_t{0.0};
}

bool valid_mode(DifferentialData::Mode mode) {
	return mode == DifferentialData::OPEN ||
			mode == DifferentialData::LIMITED_SLIP ||
			mode == DifferentialData::LOCKED;
}

bool valid_body(const RotationalBody *body) {
	return body != nullptr && finite_positive(body->get_inertia()) &&
			std::isfinite(body->get_angular_velocity()) &&
			std::isfinite(body->get_pending_torque()) &&
			std::isfinite(body->get_effective_torque());
}

bool add_unique_body(
		std::array<RotationalBody *, RotationalNetwork::MAX_BODIES> &bodies,
		std::size_t &count, RotationalBody *body) {
	if (count == bodies.size())
		return false;
	for (std::size_t i = 0; i < count; ++i)
		if (bodies[i] == body)
			return false;
	bodies[count++] = body;
	return true;
}

// Build into a local bounded value so failure leaves the caller's constraint
// unchanged, matching the network's atomic setup and solve transactions.
bool make_clutch_constraint(RotationalBody *engine, RotationalBody *shaft,
		real_t signed_ratio, VelocityConstraint &constraint) {
	VelocityConstraint candidate;
	if (!candidate.add_term(engine, real_t{1.0}) ||
			!candidate.add_term(shaft, -signed_ratio))
		return false;
	constraint = candidate;
	return true;
}

// Build the equal-angular-velocity differential constraint without allocation.
bool make_equal_velocity_constraint(RotationalBody *left,
		RotationalBody *right, VelocityConstraint &constraint) {
	VelocityConstraint candidate;
	if (!candidate.add_term(left, real_t{1.0}) ||
			!candidate.add_term(right, -real_t{1.0}))
		return false;
	constraint = candidate;
	return true;
}

bool solve_clutch_primary(
		RotationalBody *engine, RotationalBody *shaft,
		const VelocityConstraint &primary_constraint, real_t signed_ratio,
		real_t engagement, real_t clutch_max_torque,
		ConstraintSolver &solver, real_t &primary_applied_torque,
		ClutchTelemetry &telemetry) {
	// Preconditions: solve() has validated the external step and controls;
	// configure() has validated all bodies and the primary constraint.
	if (signed_ratio == real_t{0.0}) {
		const ConstraintSolution primary_result = solver.solve(
				primary_constraint, UNBOUNDED_CAPACITY);
		if (!primary_result.valid)
			return false;
		primary_applied_torque = primary_result.applied_torque;
		return true;
	}

	VelocityConstraint clutch_constraint;
	if (!make_clutch_constraint(engine, shaft, signed_ratio,
			clutch_constraint))
		return false;
	const real_t clamped_engagement = std::clamp(
			engagement, real_t{0.0}, real_t{1.0});
	const real_t clutch_capacity =
			std::max(clutch_max_torque, real_t{0.0}) * clamped_engagement;
	if (!std::isfinite(clutch_capacity))
		return false;
	const ClutchSolution coupled = solver.solve_clutch(
				primary_constraint, clutch_constraint, clutch_capacity);
	if (!coupled.valid)
		return false;
	primary_applied_torque = coupled.primary_applied_torque;
	telemetry.present = true;
	telemetry.requested_engine_torque = -coupled.clutch_requested_torque;
	telemetry.transmitted_torque = -coupled.clutch_applied_torque;
	telemetry.output_torque = signed_ratio * telemetry.transmitted_torque;
	telemetry.slip = coupled.clutch_slip;
	telemetry.slipping = coupled.clutch_slipping;
	return std::isfinite(telemetry.output_torque);
}

bool solve_differentials(
		const std::array<DrivenAxle, RotationalNetwork::MAX_AXLES> &axles,
		std::size_t axle_count, real_t primary_applied_torque,
		ConstraintSolver &solver) {
	// Preconditions: setup validated axle count, shares, bodies, and policies;
	// solve_clutch_primary has already applied the primary reaction to solver.
	const real_t primary_reaction_torque = -primary_applied_torque;

	for (std::size_t i = 0; i < axle_count; ++i) {
		const DrivenAxle &axle = axles[i];
		const real_t left_velocity = solver.predict_velocity(axle.left);
		const real_t right_velocity = solver.predict_velocity(axle.right);
		const real_t carrier_velocity = real_t{0.5} *(left_velocity + right_velocity);

		const real_t axle_torque = primary_reaction_torque * axle.share;

		const real_t relative_slip =
				axle.differential.mode == DifferentialData::OPEN ? real_t{0.0} :
				left_velocity - right_velocity;

		const real_t capacity = axle.differential.capacity(axle_torque, carrier_velocity, relative_slip);
		if (std::isnan(capacity) || capacity < real_t{0.0})
			return false;
		if (axle.differential.mode == DifferentialData::OPEN)
			continue;
		VelocityConstraint differential_constraint;
		if (!make_equal_velocity_constraint(axle.left, axle.right,
				differential_constraint) ||
				!solver.solve(differential_constraint, capacity).valid)
			return false;
	}
	return true;
}

} // namespace

DifferentialSettings DifferentialSettings::from_resource(
		const Ref<DifferentialData> &data) {
	DifferentialSettings result;
	if (data.is_null())
		return result;
	result.mode = data->get_mode();
	result.preload_torque = data->get_preload_torque();
	result.power_lock_ratio = data->get_power_lock_ratio();
	result.coast_lock_ratio = data->get_coast_lock_ratio();
	result.slip_sensitive_gain = data->get_slip_sensitive_gain();
	result.max_lock_torque = data->get_max_lock_torque();
	return result;
}

bool DifferentialSettings::is_valid() const {
	return valid_mode(mode) && finite_nonnegative(preload_torque) &&
			finite_nonnegative(power_lock_ratio) &&
			finite_nonnegative(coast_lock_ratio) &&
			finite_nonnegative(slip_sensitive_gain) &&
			finite_nonnegative(max_lock_torque);
}

real_t DifferentialSettings::capacity(real_t transmitted_torque,
		real_t carrier_velocity, real_t relative_slip) const {
	if (mode == DifferentialData::OPEN)
		return real_t{0.0};
	if (mode == DifferentialData::LOCKED)
		return UNBOUNDED_CAPACITY;
	const bool same_direction = transmitted_torque == real_t{0.0} ||
			carrier_velocity == real_t{0.0} ||
			(transmitted_torque > real_t{0.0} &&
					carrier_velocity > real_t{0.0}) ||
			(transmitted_torque < real_t{0.0} &&
					carrier_velocity < real_t{0.0});
	const real_t active_ratio = same_direction ? power_lock_ratio : coast_lock_ratio;
	const real_t result = std::min(max_lock_torque,
			preload_torque + active_ratio * std::abs(transmitted_torque) +
			slip_sensitive_gain * std::abs(relative_slip));
	return result;
}

bool DrivenAxle::is_valid() const {
	return left != nullptr && right != nullptr && left != right &&
			finite_positive(share) && valid_body(left) &&
			valid_body(right) && differential.is_valid();
}

bool RotationalNetwork::configure(
		RotationalBody *engine_body, RotationalBody *shaft_body,
		const std::array<DrivenAxle, MAX_AXLES> &input_axles,
		std::size_t input_axle_count) {
	if (!valid_body(engine_body) || !valid_body(shaft_body) ||
			engine_body == shaft_body || input_axle_count == 0 ||
			input_axle_count > MAX_AXLES)
		return false;

	std::array<RotationalBody *, MAX_BODIES> candidate_bodies{};
	std::size_t candidate_body_count = 0;
	if (!add_unique_body(candidate_bodies, candidate_body_count, engine_body) ||
			!add_unique_body(candidate_bodies, candidate_body_count, shaft_body))
		return false;

	real_t share_sum = real_t{0.0};
	for (std::size_t i = 0; i < input_axle_count; ++i) {
		const DrivenAxle &axle = input_axles[i];
		if (!axle.is_valid() ||
				!add_unique_body(candidate_bodies, candidate_body_count, axle.left) ||
				!add_unique_body(candidate_bodies, candidate_body_count, axle.right))
			return false;
		share_sum += axle.share;
	}
	if (!finite_positive(share_sum))
		return false;

	VelocityConstraint candidate_primary_constraint;
	if (!candidate_primary_constraint.add_term(shaft_body, real_t{1.0}))
		return false;
	std::array<DrivenAxle, MAX_AXLES> candidate_axles = input_axles;
	for (std::size_t i = 0; i < input_axle_count; ++i) {
		DrivenAxle &axle = candidate_axles[i];
		axle.share /= share_sum;
		if (!candidate_primary_constraint.add_term(axle.left,
					-real_t{0.5} * axle.share) ||
			!candidate_primary_constraint.add_term(axle.right,
					-real_t{0.5} * axle.share))
			return false;
	}

	// Commit only after all validation and constraint construction have succeeded.
	engine = engine_body;
	shaft = shaft_body;
	primary_constraint = candidate_primary_constraint;
	axles = candidate_axles;
	axle_count = input_axle_count;
	configured = true;
	clutch_telemetry = ClutchTelemetry{};
	return true;
}

bool RotationalNetwork::solve(real_t dt, real_t signed_ratio,
		real_t engagement, real_t clutch_max_torque) {
	clutch_telemetry = ClutchTelemetry{};
	if (!configured || !std::isfinite(dt) || dt <= real_t{0.0} ||
			!std::isfinite(signed_ratio) || !std::isfinite(engagement) ||
			!std::isfinite(clutch_max_torque))
		return false;

	ConstraintSolver solver(dt);
	real_t primary_applied_torque = real_t{0.0};
	ClutchTelemetry telemetry;

	// Phase 1: solve the exact coupled clutch-primary constraints against the shaft.
	if (!solve_clutch_primary(engine, shaft, primary_constraint, signed_ratio,
			engagement, clutch_max_torque, solver, primary_applied_torque,
			telemetry))
		return false;
	// Phase 2: solve each configured differential from predicted wheel speeds.
	if (!solve_differentials(axles, axle_count,
			primary_applied_torque, solver))
		return false;
	// Phase 3: validate all final torque sums, then mutate every body once.
	if (!solver.commit())
		return false;
	clutch_telemetry = telemetry;
	return true;
}

} // namespace godot
