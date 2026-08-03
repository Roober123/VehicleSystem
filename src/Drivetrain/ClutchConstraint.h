#pragma once

#include <cmath>

#include "RotationalBody.h"

namespace godot {

/// Immutable inputs for one clutch impulse solve.
struct ClutchSolveInput {
    real_t ratio = 0.0;
    real_t engagement = 0.0;
    real_t capacity = 0.0;
    real_t aggregate_inertia = 0.0;
    real_t aggregate_angular_velocity = 0.0;
    real_t pending_engine_torque = 0.0;
};

/// Physics-only clutch constraint. Gear selection and clutch phase state live
/// in Gearbox; this class only applies one bounded impulse between two explicit
/// rotational bodies from a supplied immutable solve input.
class ClutchConstraint {
    RotationalBody *engine = nullptr;
    RotationalBody *output = nullptr;

public:
    void set_bodies(RotationalBody &engine_body, RotationalBody &output_body) {
        engine = &engine_body;
        output = &output_body;
    }

    void solve(real_t dt, const ClutchSolveInput &input);
};

} // namespace godot
