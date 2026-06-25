#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/ray_cast3d.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/classes/ref.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/physics_direct_body_state3d.hpp"
#include "godot_cpp/classes/physics_direct_body_state3d_extension.hpp"
#include "Resources/suspension_data.h"
#include "wheel.h"
#include <vector>

namespace godot {

class Axle : public Node3D {
    GDCLASS(Axle, Node3D);

protected:

    std::vector<Wheel*> wheels;


    static void _bind_methods();
    void find_children_wheels();
    

public:
    Axle();
	~Axle() override = default;

    virtual void _ready() override;
    
    bool is_steerable = false;
    void set_steerable(bool value);
    bool get_steerable() const;

    real_t drive_ratio = 0.0;
    void set_drive_ratio(real_t value);
    real_t get_drive_ratio() const;

    void compute_suspension_parameters(real_t mass, const Ref<SuspensionData>& s);

    void update_physics(PhysicsDirectBodyState3D *vehicle_state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity);
    void solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                    const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                    real_t brake_input);

    void add_torque(real_t torque);
    void integrate(real_t dt);
    real_t get_average_wheel_omega() const;
    real_t get_total_sat() const;

    const std::vector<Wheel*>& get_wheels() const;
};

} // namespace godot