#pragma once
#include <vector>
#include <cmath>
#include "RotationalBody.h"
#include "wheel.h"
#include "axle.h"

namespace godot {

class ShaftWheelsCouplingConstraint {
    RotationalBody *driveshaft = nullptr;
    std::vector<Axle*> axles;

    real_t coupling_stiffness = 0.0;
    real_t coupling_damping   = 0.0;

    real_t omega_n = 3.0;   // Natural frequency (rad/s) — softened for tire slip compliance
    real_t zeta    = 1.0;   // Damping ratio — critically damped, no oscillation

    public:
        void load_bodies(RotationalBody* veh_driveshaft, std::vector<Axle*> veh_axles);
        void solve();
    
};

}