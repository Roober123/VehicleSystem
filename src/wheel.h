#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/ray_cast3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/classes/ref.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/physics_direct_body_state3d.hpp"
#include "godot_cpp/classes/physics_direct_body_state3d_extension.hpp"
#include "Drivetrain/RotationalBody.h"

#include "Resources/suspension_data.h"
#include "Resources/tire_data.h"

namespace godot {

class Wheel : public Node3D {
    GDCLASS(Wheel, Node3D);

    real_t stiffness = 1.0;
    real_t damping = 1.0;
    real_t suspension_length = 1.0;
    real_t suspension_rebound_force = 0.0;
    real_t radius = 0.3;
    real_t patch_length = 0.4;
    real_t friction_coefficient = 1.0;
    real_t longitudinal_stiffness = 80000;
    real_t lateral_stiffness = 60000;
    real_t brake_power = 1500.0;

    Vector2 deflection[4] = {};
    real_t offsets[4] = {0.5, 0.2, -0.2, -0.5};

    Vector3 patch_position[4] = {};


    void add_physics();
    protected:
    static void _bind_methods();

    public:
    Wheel() = default;
    ~Wheel() override = default;

    

    RotationalBody body;

    RayCast3D *ray = nullptr;
    void set_suspension(real_t suspension_length, real_t stiffness, real_t damping);
    void set_tire(const Ref<TireData>& t);
    real_t get_suspension_rebound_force() const { return suspension_rebound_force; }
    real_t get_angular_velocity() const { return body.get_angular_velocity(); }
    Vector3 get_tire_force() const { return cached_tire_force; }
    float get_reaction_torque() const { return reaction_torque; }

    real_t sustained_mass = 0.0;
    Vector3 collision_point;
    Vector3 collision_normal;
    Vector3 forward_vector;
    Vector3 right_vector;
    Vector3 up_vector;
    bool on_ground = false;

    void update_suspension(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                           const Vector3 &linear_velocity, const Vector3 &angular_velocity);

    void solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                    const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                    real_t brake_input);
    
    Vector3 tire_force;
    Vector3 cached_tire_force;
    float reaction_torque;
    Vector3 patch_torque;
    real_t self_aligning_torque = 0.0;

    
};

} // namespace godot