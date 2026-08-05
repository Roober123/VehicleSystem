#include "RotationalConstraint.h"

#include <algorithm>
#include <cmath>

namespace godot {
namespace {

constexpr real_t kConnectionEpsilon = real_t{1e-7};

bool valid_capacity(real_t capacity) {
	return !std::isnan(capacity) && capacity >= real_t{0.0};
}

real_t clamp_impulse(real_t requested, real_t dt, real_t capacity) {
	if (!std::isfinite(capacity))
		return requested;
	const real_t limit = capacity * dt;
	if (!std::isfinite(limit))
		return std::numeric_limits<real_t>::quiet_NaN();
	return std::clamp(requested, -limit, limit);
}

// Evaluate one velocity constraint against the solver's predicted velocities.
bool constraint_values(const VelocityConstraint &constraint,
		const ConstraintSolver &solver,
		real_t &residual, real_t &inverse_mass) {
	residual = real_t{0.0};
	inverse_mass = real_t{0.0};
	for (std::size_t i = 0; i < constraint.term_count; ++i) {
		const auto &term = constraint.terms[i];
		const real_t velocity = solver.predict_velocity(term.body);
		const real_t inverse_inertia = real_t{1.0} / term.body->get_inertia();
		residual += term.coefficient * velocity;
		inverse_mass += term.coefficient * term.coefficient * inverse_inertia;
	}
	return std::isfinite(residual) && std::isfinite(inverse_mass) &&
			inverse_mass > real_t{0.0};
}

// Accumulate shared inverse-mass terms used by the coupled projection.
real_t cross_inverse_mass(const VelocityConstraint &first,
		const VelocityConstraint &second) {
	real_t result = real_t{0.0};
	for (std::size_t i = 0; i < first.term_count; ++i) {
		const auto &a = first.terms[i];
		for (std::size_t j = 0; j < second.term_count; ++j) {
			const auto &b = second.terms[j];
			if (a.body == b.body)
				result += a.coefficient * b.coefficient /
						a.body->get_inertia();
		}
	}
	return result;
}

void fill_result(ConstraintSolution &result, real_t residual,
		real_t inverse_mass, real_t requested, real_t applied, real_t dt) {
	result.valid = true;
	result.requested_torque = requested / dt;
	result.applied_torque = applied / dt;
	result.slip = residual + applied * inverse_mass;
	const real_t tolerance = kConnectionEpsilon *
			std::max(real_t{1.0}, std::abs(requested));
	result.slipping = std::abs(applied - requested) > tolerance ||
			std::abs(result.slip) > tolerance;
}

} // namespace

ConstraintSolver::ConstraintSolver(real_t solve_dt) :
		dt(solve_dt), valid_dt(std::isfinite(solve_dt) &&
				solve_dt > real_t{0.0}) {}

bool ConstraintSolver::merge_impulse(RotationalBody *body, real_t impulse) {
	if (body == nullptr || !std::isfinite(impulse))
		return false;
	for (std::size_t i = 0; i < entry_count; ++i) {
		if (bodies[i] != body)
			continue;
		const real_t merged = impulses[i] + impulse;
		if (!std::isfinite(merged))
			return false;
		impulses[i] = merged;
		return true;
	}
	if (entry_count == bodies.size())
		return false;
	bodies[entry_count] = body;
	impulses[entry_count++] = impulse;
	return true;
}

bool ConstraintSolver::insert_constraint(const VelocityConstraint &constraint, real_t impulse) {
	if (!std::isfinite(impulse))
		return false;
	// Build into a copy so a malformed later term cannot leave this solver
	// partially updated.
	ConstraintSolver candidate = *this;
	for (std::size_t i = 0; i < constraint.term_count; ++i) {
		const auto &term = constraint.terms[i];
		if (!candidate.merge_impulse(term.body, term.coefficient * impulse))
			return false;
	}
	*this = candidate;
	return true;
}

real_t ConstraintSolver::pending_impulse(const RotationalBody *body) const {
	for (std::size_t i = 0; i < entry_count; ++i)
		if (bodies[i] == body)
			return impulses[i];
	return real_t{0.0};
}

real_t ConstraintSolver::predict_velocity(const RotationalBody *body) const {
	if (body == nullptr)
		return 0.0;
	return body->predict_angular_velocity(dt) +
			pending_impulse(body) / body->get_inertia();
}

bool ConstraintSolver::commit() {
	if (!valid_dt)
		return false;
	for (std::size_t i = 0; i < entry_count; ++i) {
		const real_t torque = impulses[i] / dt;
		if (!std::isfinite(torque) ||
				!std::isfinite(bodies[i]->get_pending_torque() + torque) ||
				!std::isfinite(bodies[i]->predict_angular_velocity(dt) +
						impulses[i] / bodies[i]->get_inertia()))
			return false;
	}
	for (std::size_t i = 0; i < entry_count; ++i)
		bodies[i]->add_torque(impulses[i] / dt);
	return true;
}

bool VelocityConstraint::add_term(RotationalBody *body, real_t coefficient) {
	if (term_count >= MAX_CONSTRAINT_TERMS || body == nullptr ||
			!std::isfinite(coefficient))
		return false;
	terms[term_count++] = ConstraintTerm{body, coefficient};
	return true;
}

// Project one scalar velocity error, clamp its impulse, and retain the update.
ConstraintSolution ConstraintSolver::solve(const VelocityConstraint &constraint, real_t capacity_torque) {
	ConstraintSolution result;
	if (!valid_dt || !valid_capacity(capacity_torque))
		return result;
	real_t residual = real_t{0.0};
	real_t inverse_mass = real_t{0.0};
	if (!constraint_values(constraint, *this, residual, inverse_mass))
		return result;
	const real_t requested = -residual / inverse_mass;
	const real_t applied = clamp_impulse(requested, dt, capacity_torque);
	if (!std::isfinite(requested) || !std::isfinite(applied) ||
			!std::isfinite(requested / dt) || !std::isfinite(applied / dt) ||
			!insert_constraint(constraint, applied))
		return result;
	const real_t slip = residual + applied * inverse_mass;
	if (!std::isfinite(slip))
		return ConstraintSolution{};
	fill_result(result, residual, inverse_mass, requested, applied, dt);
	return result;
}

// Solve clutch and primary together, conditioning the primary after clutch
// capacity is applied so the coupled projection remains exact.
ClutchSolution ConstraintSolver::solve_clutch(
		const VelocityConstraint &primary_constraint,
		const VelocityConstraint &clutch_constraint,
		real_t clutch_capacity_torque) 
{
	ClutchSolution result;
	if (!valid_dt || !valid_capacity(clutch_capacity_torque))
		return result;
	if (clutch_constraint.is_empty()) {
		const ConstraintSolution primary_solution = solve(
				primary_constraint, UNBOUNDED_CAPACITY);
		result.valid = primary_solution.valid;
		if (result.valid)
			result.primary_applied_torque = primary_solution.applied_torque;
		return result;
	}
	real_t primary_residual = real_t{0.0};
	real_t primary_mass = real_t{0.0};
	real_t clutch_residual = real_t{0.0};
	real_t clutch_mass = real_t{0.0};
	if (!constraint_values(primary_constraint, *this, primary_residual,primary_mass) ||
		!constraint_values(clutch_constraint, *this, clutch_residual, clutch_mass))
		return result;
	const real_t cross_mass = cross_inverse_mass(
			primary_constraint, clutch_constraint);
	const real_t conditioned_mass = clutch_mass -
			cross_mass * cross_mass / primary_mass;
	if (!std::isfinite(cross_mass) || !std::isfinite(conditioned_mass) ||
			conditioned_mass <= real_t{0.0})
		return result;
	const real_t requested_clutch = -(clutch_residual -
			cross_mass * primary_residual / primary_mass) / conditioned_mass;
	const real_t applied_clutch = clamp_impulse(
			requested_clutch, dt, clutch_capacity_torque);
	const real_t requested_primary = -(primary_residual +
			cross_mass * requested_clutch) / primary_mass;
	const real_t applied_primary = -(primary_residual +
			cross_mass * applied_clutch) / primary_mass;
	if (!std::isfinite(requested_clutch) || !std::isfinite(applied_clutch) ||
			!std::isfinite(requested_primary) || !std::isfinite(applied_primary) ||
			!std::isfinite(requested_clutch / dt) || !std::isfinite(applied_clutch / dt) ||
			!std::isfinite(requested_primary / dt) || !std::isfinite(applied_primary / dt))
		return result;
	// Commit both constraint impulses together so a capacity or topology failure does
	// not leave only one side of the coupled solve in this solver.
	ConstraintSolver candidate = *this;
	if (!candidate.insert_constraint(primary_constraint, applied_primary) ||
			!candidate.insert_constraint(clutch_constraint, applied_clutch))
		return result;
	const real_t clutch_slip = clutch_residual + cross_mass * applied_primary +
			clutch_mass * applied_clutch;
	if (!std::isfinite(clutch_slip))
		return result;
	const real_t clutch_tolerance = kConnectionEpsilon *
			std::max(real_t{1.0}, std::abs(requested_clutch));
	result.primary_applied_torque = applied_primary / dt;
	result.clutch_requested_torque = requested_clutch / dt;
	result.clutch_applied_torque = applied_clutch / dt;
	result.clutch_slip = clutch_slip;
	result.clutch_slipping = std::abs(clutch_slip) > clutch_tolerance ||
			std::abs(applied_clutch - requested_clutch) > clutch_tolerance;
	result.valid = true;
	*this = candidate;
	return result;
}

} // namespace godot
