#pragma once

#include "Drivetrain/RotationalBody.h"
#include "Drivetrain/differential_solver.h"

namespace godot {

class AxleDifferential {
    DifferentialSolver solver;
    RotationalBody *left = nullptr;
    RotationalBody *right = nullptr;
    real_t left_inertia = 0.0;
    real_t right_inertia = 0.0;
    real_t carrier_inertia = 0.0;
    real_t transmitted_torque = 0.0;
    bool configured = false;

public:
    AxleDifferential() = default;

    bool configure(const Ref<DifferentialData> &data,
                   RotationalBody *left_body,
                   RotationalBody *right_body);
    void reset();

    bool is_configured() const { return configured; }
    real_t get_carrier_inertia() const { return carrier_inertia; }
    real_t get_carrier_velocity() const;
    real_t get_predicted_carrier_velocity(real_t dt) const;

    void add_carrier_torque(real_t total_torque);
    void add_carrier_impulse(real_t total_impulse, real_t dt);
    real_t solve_relative(real_t dt);
};

} // namespace godot
