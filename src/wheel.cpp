#include "wheel.h"

namespace godot {

void Wheel::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_suspension_rebound_force"), &Wheel::get_suspension_rebound_force);
    ClassDB::bind_method(D_METHOD("get_angular_velocity"), &Wheel::get_angular_velocity);
    ClassDB::bind_method(D_METHOD("get_tire_force"), &Wheel::get_tire_force);
    ClassDB::bind_method(D_METHOD("get_reaction_torque"), &Wheel::get_reaction_torque);
}

void Wheel::set_suspension(real_t suspension_length, real_t stiffness, real_t damping) {
    this->suspension_length = suspension_length;
    this->stiffness = stiffness;
    this->damping = damping;
    add_physics();

} 
void Wheel::set_tire(const Ref<TireData>& t) {
    if (t == nullptr)
        return;
    this->friction_coefficient = t->friction_coefficient;
    this->radius = t->radius;
    this->longitudinal_stiffness = t->longitudinal_stiffness;
    this->lateral_stiffness = t->lateral_stiffness;
    this->patch_length = t->patch_length;
    this->brake_power = t->brake_power;
}

void Wheel::add_physics() {
    if (ray == nullptr) {
        ray = memnew(RayCast3D);
        add_child(ray);
        ray->set_enabled(false);
    }
    ray->set_target_position(Vector3(0, -suspension_length, 0));
}

void Wheel::update_suspension(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity) {
    ray->force_raycast_update();
    if (!ray->is_colliding()) {
        on_ground = false;
        sustained_mass = 0.0;
        suspension_rebound_force = 0.0;
        return;
    }
    on_ground = true;
    collision_point = ray->get_collision_point();
    collision_normal = ray->get_collision_normal();
    real_t distance = suspension_length -  ray->get_global_position().distance_to(collision_point);
    
    Vector3 velocity_at_point = linear_velocity + angular_velocity.cross(
                                (collision_point - com_global));
    real_t velocity_along_normal = velocity_at_point.dot(collision_normal);

    suspension_rebound_force = distance * stiffness - damping * velocity_along_normal;
    sustained_mass = suspension_rebound_force / 9.81;
    if (sustained_mass < 0.0) sustained_mass = 0.0;

    forward_vector = get_global_transform().basis.get_column(2); // z
    right_vector = get_global_transform().basis.get_column(1); // x
    up_vector = get_global_transform().basis.get_column(0); // y

}


void Wheel::solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt, real_t brake_input) {
    if (!on_ground)
        return;

    reaction_torque = 0.0f;
    self_aligning_torque = 0.0;

    // tangents because tilt from suspension messes normal
    const Vector3 fwd_tangent = (forward_vector - collision_normal * forward_vector.dot(collision_normal)).normalized();
    const Vector3 right_tangent = (right_vector - collision_normal * right_vector.dot(collision_normal)).normalized();

    Vector3 vel_tire = fwd_tangent * get_angular_velocity() * radius;
    real_t point_load = sustained_mass * 9.81 / 4;

    for (int i = 0; i < 3; i++) {
        real_t lateral_force = 0.0;
        real_t longitudinal_force = 0.0;
        patch_position[i] = collision_point + fwd_tangent * offsets[i] * patch_length;
        Vector3 vel_point = linear_velocity + angular_velocity.cross(patch_position[i] - com_global);
        Vector3 error_vel = vel_tire - vel_point;
        float vx = error_vel.dot(fwd_tangent);
        float vy = error_vel.dot(right_tangent);
        deflection[i] += Vector2(vx, vy) * dt;
        constexpr real_t relaxation_rate = 20.0;
        deflection[i] *= exp(-relaxation_rate * dt);

        longitudinal_force = deflection[i].x * longitudinal_stiffness;
        lateral_force = deflection[i].y * lateral_stiffness;

        real_t l = Vector2(longitudinal_force, lateral_force).length();
        if (l > point_load * friction_coefficient) {
            real_t scale = point_load * friction_coefficient / l;
            longitudinal_force *= scale;
            lateral_force *= scale;
            deflection[i] *= scale;
        }
        Vector3 point_force = fwd_tangent * longitudinal_force + right_tangent * lateral_force;
        tire_force += point_force;
        reaction_torque -= longitudinal_force * radius;
        patch_torque += (patch_position[i] - collision_point).cross(point_force);

    }
    
    self_aligning_torque = patch_torque.dot(collision_normal);
    body.add_torque(reaction_torque);

    if (brake_input > 0.0f) {
        const real_t omega = get_angular_velocity();
        const real_t brake_torque = brake_input * brake_power;
        const real_t max_stopping = std::abs(omega) * body.get_inertia() / std::max(dt, 1e-6f);

        if (brake_torque >= max_stopping) {
            body.set_angular_velocity(0.0);
            body.clear_torque();
        } else {
            body.add_torque((omega > 0.0f) ? -brake_torque : brake_torque);
        }
    }
}

} // namespace godot