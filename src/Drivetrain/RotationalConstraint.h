#pragma once

#include <array>
#include <cstddef>
#include <limits>

#include "Drivetrain/RotationalBody.h"

namespace godot {

/// Direct drivetrain constraints are bounded by the maximum eight-axle topology:
/// one simulated engine, one shaft, and sixteen wheel bodies.
constexpr std::size_t MAX_CONSTRAINT_TERMS = 18;
constexpr real_t UNBOUNDED_CAPACITY =
		std::numeric_limits<real_t>::infinity();

struct ConstraintTerm {
	RotationalBody *body = nullptr;
	real_t coefficient = real_t{0.0};
};

struct VelocityConstraint;
struct ConstraintSolution;
struct ClutchSolution;

/// Bounded rotational constraint solver. Solve operations update only this
/// fixed impulse set; commit() validates every aggregate pending torque before
/// one mutation pass, preserving atomic failure for a whole substep.
class ConstraintSolver {
	std::array<RotationalBody *, MAX_CONSTRAINT_TERMS> bodies{};
	std::array<real_t, MAX_CONSTRAINT_TERMS> impulses{};
	std::size_t entry_count = 0;
	real_t dt = real_t{0.0};
	bool valid_dt = false;

	bool merge_impulse(RotationalBody *body, real_t impulse);
	bool insert_constraint(const VelocityConstraint &constraint, real_t impulse);
	real_t pending_impulse(const RotationalBody *body) const;

public:
	/// Bind one validated substep; an invalid step leaves all solve operations
	/// inert and commit() unsuccessful.
	explicit ConstraintSolver(real_t solve_dt);
	ConstraintSolver() = delete;

	/// Solve one scalar constraint against current predicted velocities. The result is
	/// kept in this solver until commit().
	ConstraintSolution solve(const VelocityConstraint &constraint,
			real_t capacity_torque = UNBOUNDED_CAPACITY);
	/// Solve the primary constraint together with an optional bounded clutch constraint.
	ClutchSolution solve_clutch(const VelocityConstraint &primary_constraint,
			const VelocityConstraint &clutch_constraint,
			real_t clutch_capacity_torque = UNBOUNDED_CAPACITY);

	real_t predict_velocity(const RotationalBody *body) const;
	/// Validate every final torque sum before the one mutation pass.
	bool commit();
};

/// Sparse scalar rotational velocity constraint. The residual is
/// `sum(coefficient * angular_velocity)` and its effective inverse mass is
/// `sum(coefficient^2 / inertia)`.
/// For Jacobian J and angular velocities omega, velocity error is e = J*omega;
/// coupled projection solves J*M^-1*J^T*lambda = -e before bounded impulses apply.
struct VelocityConstraint {
	std::array<ConstraintTerm, MAX_CONSTRAINT_TERMS>
			terms{};
	std::size_t term_count = 0;

	bool add_term(RotationalBody *body, real_t coefficient);
	bool add_term(RotationalBody &body, real_t coefficient) {
		return add_term(&body, coefficient);
	}
	bool is_empty() const { return term_count == 0; }
};

struct ConstraintSolution {
	bool valid = false;
	real_t requested_torque = real_t{0.0};
	real_t applied_torque = real_t{0.0};
	real_t slip = real_t{0.0};
	bool slipping = false;
};

struct ClutchSolution {
	bool valid = false;
	real_t primary_applied_torque = real_t{0.0};
	real_t clutch_requested_torque = real_t{0.0};
	real_t clutch_applied_torque = real_t{0.0};
	real_t clutch_slip = real_t{0.0};
	bool clutch_slipping = false;
};

} // namespace godot
