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
#include "Resources/steering_rack_data.h"
#include "Resources/tire_data.h"
#include "wheel.h"
#include "SteeringRack.h"
#include <vector>
#include "Drivetrain/VehicleDifferential.h"
#include <memory>

namespace godot {

class Axle : public Node3D {
    GDCLASS(Axle, Node3D);

protected:

    std::vector<Wheel*> wheels;
    Ref<SteeringRackData> steering_rack_data;
    Ref<TireData> tire_data = nullptr;
    SteeringRack steering_rack;
    std::unique_ptr<BaseDifferential> differential;

    static void _bind_methods();
    void find_children_wheels();
    

public:
    Axle();
	~Axle() override = default;

    virtual void _ready() override;

    bool is_steerable = false;
    void set_steerable(bool value) { is_steerable = value; }
    bool get_steerable() const { return is_steerable; }

    real_t drive_ratio = 0.0;
    real_t downforce_ratio = 0.5;
    real_t arb_stiffness = 0.0;
    void set_drive_ratio(real_t value) { drive_ratio = value; }
    real_t get_drive_ratio() const { return drive_ratio; }

    void set_downforce_ratio(real_t value);
    real_t get_downforce_ratio() const { return downforce_ratio; }

    real_t get_antiroll_bar_force() const;

    void set_tire_data(const Ref<TireData>& t) { tire_data = t; }
    Ref<TireData> get_tire_data() const { return tire_data; }

    void set_steering_rack_data(const Ref<SteeringRackData> &p_data) { steering_rack_data = p_data; }
    Ref<SteeringRackData> get_steering_rack_data() const { return steering_rack_data; }

    // Differential type: "Open", "LSD", "Torsen"
    void set_differential_type(const String &p_type);
    String get_differential_type() const;

    void compute_suspension_parameters(real_t mass, const Ref<SuspensionData>& s);

    void update_physics(PhysicsDirectBodyState3D *vehicle_state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity);
    void solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                    const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                    real_t brake_input, bool abs_enabled = true);
    
    void solve_steering(real_t steer_input, real_t dt, real_t speed_kph = 0.0);

    void set_wheels_rotation();

    void add_torque(real_t torque);
    void integrate(real_t dt);
    real_t get_average_wheel_omega() const;
    real_t get_total_sat() const;

    const std::vector<Wheel*>& get_wheels() const;

    // Current steering angle in radians
    real_t get_steer_angle() const;

    void set_trackwidth(real_t value) { trackwidth = value; }
    real_t get_trackwidth() const { return trackwidth; }
    void set_wheelbase(real_t value) { wheelbase = value; }
    real_t get_wheelbase() const { return wheelbase; }

private:
    String differential_type = "Open";

    real_t filtered_sat = 0.0;
    real_t trackwidth = 0.0;
    real_t wheelbase = 0.0;

    void _instantiate_differential();
};

}