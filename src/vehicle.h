#pragma once

#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/classes/ray_cast3d.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>
#include "godot_cpp/classes/engine.hpp"
#include "Resources/suspension_data.h"
#include "godot_cpp/classes/physics_direct_body_state3d.hpp"
#include "godot_cpp/classes/physics_direct_body_state3d_extension.hpp"
#include "axle.h"
#include <vector>
#include "Drivetrain/VehicleEngine.h"
#include "Resources/vehicle_engine_data.h"
#include "Resources/gearbox_data.h"
#include "Drivetrain/ClutchGearConstraint.h"
#include "Drivetrain/GearboxModule.h"
#include "Drivetrain/ShaftWheelsCouplingConstraint.h"
#include "VehicleAerodynamics.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "TractionControl.h"
#include "Drivetrain/Turbo.h"
#include "Resources/turbo_data.h"
#include "ExhaustSystem.h"

namespace godot {

class Vehicle : public RigidBody3D {
    GDCLASS(Vehicle, RigidBody3D);

	VehicleEngine engine;
	Turbo turbo;

	ClutchGearConstraint clutch_gearbox;
	GearboxModule gearbox_module;
	ShaftWheelsCouplingConstraint shaft_wheels_coupling;

	RotationalBody drive_shaft;

	VehicleAerodynamics aerodynamics;
	TractionControl tcs;
	ExhaustSystem exhaust_system;

protected:
	static void _bind_methods();
	void _apply_aerodynamics(const Vector3 &linear_velocity, const Vector3 &angular_velocity, const Vector3 &body_origin);
	void _apply_downforce(real_t total_downforce, const Vector3 &body_origin);
	void _update_suspension(PhysicsDirectBodyState3D *state, const Vector3 &body_origin, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity);
	void _handle_auto_gearbox();
	void _run_drivetrain_substeps(PhysicsDirectBodyState3D *state, const Vector3 &com_global, const Vector3 &linear_velocity, 
		const Vector3 &angular_velocity, real_t sub_dt, real_t wheel_brake, real_t speed_kph, bool abs_enabled);
	real_t _compute_reflected_load() const;
	void _compute_axle_dimensions();

	real_t smoothed_steer = 0.0;

public:
    Vehicle();
	~Vehicle() override = default;
	virtual void _ready() override;
	virtual void _integrate_forces(PhysicsDirectBodyState3D *state) override;

	std::vector<Axle*> axles;

	// Exported variables
	Ref<SuspensionData> suspension_data = nullptr;
	void set_suspension_data(const Ref<SuspensionData>& r) { suspension_data = r; }
	Ref<SuspensionData> get_suspension_data() { return suspension_data; }

	Ref<VehicleEngineData> engine_data = nullptr;
	void set_engine_data(const Ref<VehicleEngineData>& r) { engine_data = r; }
	Ref<VehicleEngineData> get_engine_data() { return engine_data; }

	Ref<GearboxData> gearbox_data = nullptr;
	void set_gearbox_data(const Ref<GearboxData>& r) { gearbox_data = r; }
	Ref<GearboxData> get_gearbox_data() { return gearbox_data; }

	Ref<VehicleAerodynamicsData> aero_data = nullptr;
	void set_aero_data(const Ref<VehicleAerodynamicsData>& a);
	Ref<VehicleAerodynamicsData> get_aero_data() { return aero_data; }

	Ref<TurboData> turbo_data = nullptr;
	void set_turbo_data(const Ref<TurboData>& t) { turbo_data = t; }
	Ref<TurboData> get_turbo_data() {return turbo_data; }


	real_t throttle_input = 0.0;
	real_t steer_input = 0.0;
	real_t brake_input = 0.0;
	int shift_input = 0;

	bool abs_enabled = true;
	bool tcs_enabled = true;

	void set_throttle_input(real_t value);
	real_t get_throttle_input() const { return throttle_input; }
	void set_steer_input(real_t value);
	real_t get_steer_input() const { return steer_input; }
	void set_brake_input(real_t value);
	real_t get_brake_input() const { return brake_input; }
	void set_shift_input(int value);
	int get_shift_input() const { return shift_input; }
	void set_abs_enabled(bool value) { abs_enabled = value; }
	bool get_abs_enabled() const { return abs_enabled; }
	void set_tcs_enabled(bool value);
	bool get_tcs_enabled() const { return tcs_enabled; }

	int substeps = 1;
	void set_substeps(int value);
	int get_substeps() const { return substeps; }

	// Telemetry accessors
	VehicleEngine& get_engine() { return engine; }
	const ClutchGearConstraint& get_clutch_gearbox() const { return clutch_gearbox; }
	const RotationalBody& get_driveshaft() const { return drive_shaft; }
	int get_axle_count() const { return static_cast<int>(axles.size()); }
	real_t get_speed_kph() const;

	// Exhaust system accessors
	real_t get_exhaust_smoke() const { return exhaust_system.get_data().smoke; }
	real_t get_exhaust_heat() const { return exhaust_system.get_data().heat; }
	real_t get_exhaust_flame_probability() const { return exhaust_system.get_data().flame_probability; }
    
};

} // namespace godot
