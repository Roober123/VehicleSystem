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
#include "TireSkid.h"

#include "godot_cpp/classes/curve.hpp"
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
    real_t friction_forward = 1.0;
    real_t friction_lateral = 1.0;
    real_t brake_power = 1500.0;
    real_t reference_load = 1.0;
    real_t load_sensitivity = 0.10;

    // tanh tire model parameters
    real_t peak_slip_angle = 10.0;     // degrees
    real_t relaxation_low = 0.042;     // seconds at 0 m/s
    real_t relaxation_high = 0.01;     // seconds at 30+ m/s

    // Friction curve resources (sampled with normalized slip)
    Ref<Curve> forward_friction_curve;
    Ref<Curve> lateral_friction_curve;

    void add_physics();
    void _compute_sat(real_t lateral_force);

    // Tangent vectors relative to contact surface
    void _compute_tangents(Vector3& fwd_tangent, Vector3& right_tangent) const;

    // Slip computation (sets slip_ratio, slip_angle, is_sliding)
    void _compute_slip(real_t fwd_speed, real_t lat_speed, real_t& slip_vel, real_t& slip_angle_rad);

    // Normal force from sustained mass
    real_t _compute_normal_force() const;

    // Raw friction forces using tanh model
    void _compute_raw_forces(real_t normal, real_t slip_vel, real_t slip_angle_rad,
                             real_t fwd_speed, real_t lat_speed,
                             real_t& raw_fwd_force, real_t& raw_lat_force,
                             real_t& fwd_mu, real_t& lat_mu) const;

    // Elliptic friction circle limiting
    real_t _combine_forces(real_t raw_fwd, real_t raw_lat, real_t normal,
                           real_t fwd_mu, real_t lat_mu,
                           real_t& out_fwd, real_t& out_lat) const;

    // Lerp-based relaxation with adaptive time constant
    void _apply_relaxation(real_t& longitudinal_force, real_t& lateral_force,
                           real_t dt, const Vector3& linear_velocity);

    // Apply combined tire forces to body
    void _apply_tire_forces(const Vector3& fwd_tangent, const Vector3& right_tangent,
                            real_t longitudinal_force, real_t lateral_force);

    // Update tire skid marks
    void _update_skidmarks(const Vector3& vel_point, real_t friction_sum, real_t dt);

    // Apply brakes with optional ABS
    void _apply_brakes(real_t brake_input, bool abs_enabled, real_t fwd_speed,
                       real_t dt, real_t normal, const Vector3& fwd_tangent);

    protected:
    static void _bind_methods();

    public:
    Wheel() = default;
    ~Wheel() override = default;

    

    RotationalBody body;

    RayCast3D *ray = nullptr;
    TireSkid *skid = nullptr;
    void set_suspension(real_t suspension_length, real_t stiffness, real_t damping, real_t reference_load);
    void set_tire(const Ref<TireData>& t);
    /// Returns the final non-tensile contact force, including anti-roll load transfer.
    real_t get_suspension_rebound_force() const { return suspension_rebound_force; }
    /// Sets the final normal contact force used by both suspension and tire forces.
    void set_normal_force(real_t force);
    real_t get_stiffness() const { return stiffness; }
    /// Returns the wheel's angular velocity in rad/s.
    real_t get_angular_velocity() const { return body.get_angular_velocity(); }
    /// Returns the average tire force (world-space) applied last frame.
    Vector3 get_tire_force() const { return cached_tire_force; }
    /// Returns the drivetrain reaction torque at this wheel.
    float get_reaction_torque() const { return reaction_torque; }
    /// Returns the world-space position where the wheel contacts the ground.
    Vector3 get_collision_point() const { return collision_point; }
    /// Returns the collision normal at the contact point.
    Vector3 get_collision_normal() const { return collision_normal; }
    /// Returns true if the wheel raycast is currently touching a surface.
    bool is_on_ground() const { return on_ground; }
    /// Returns the effective mass (kg) supported by this wheel from suspension.
    real_t get_sustained_mass() const { return sustained_mass; }
    /// Returns the suspension compression ratio (0 = fully extended, 1 = fully compressed).
    real_t get_compression() const { return compression; }
    /// Returns the self-aligning torque (Nm) at the steering axis from tire forces.
    real_t get_self_aligning_torque() const { return self_aligning_torque; }
    /// Returns the longitudinal slip ratio (tire_speed - road_speed) / road_speed.
    /// 0 = rolling, 1 = spinning, -1 = locked.
    real_t get_slip_ratio() const { return slip_ratio; }
    /// Returns the lateral slip angle in degrees. 0 = straight, >0 = sliding.
    real_t get_slip_angle() const { return slip_angle; }
    /// Returns true if ABS is actively modulating brake pressure this frame.
    bool get_abs_active() const { return abs_active; }
    /// Returns true if the wheel is currently sliding (excessive slip ratio or slip angle).
    bool get_is_sliding() const { return is_sliding; }

    real_t sustained_mass = 0.0;
    real_t compression = 0.0;
    Vector3 collision_point;
    Vector3 collision_normal;
    Vector3 forward_vector;
    Vector3 right_vector;
    Vector3 up_vector;
    bool on_ground = false;
    bool is_sliding = false;

    void update_suspension(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                           const Vector3 &linear_velocity, const Vector3 &angular_velocity);

    void solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                    const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                    real_t brake_input, bool abs_enabled = true);
    
    Vector3 tire_force;
    Vector3 cached_tire_force;
    float reaction_torque = 0.0f;
    real_t self_aligning_torque = 0.0;
    real_t slip_ratio = 0.0;
    real_t slip_angle = 0.0; // degrees

    real_t prev_longitudinal_force = 0.0;
    real_t prev_lateral_force = 0.0;

    real_t skid_stop_cooldown = real_t{0.0};

    real_t instability_cooldown = 0.0;
    int oscillation_count = 0;
    void _detect_tire_instability(real_t total_lateral_force, real_t dt);

    bool abs_active = false;
    real_t abs_accumulator = 0.0;
    real_t _apply_abs(real_t brake_input, real_t fwd_speed, real_t dt);
    real_t _get_load_sensitivity_scale(real_t normal_load) const;
};

}
