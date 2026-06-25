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
#include "Drivetrain/DriveShaft.h"
#include "Drivetrain/ClutchGearConstraint.h"
#include "Drivetrain/GearboxModule.h"
#include "Drivetrain/ShaftWheelsCouplingConstraint.h"
#include "Resources/tire_data.h"

namespace godot {

class Vehicle : public RigidBody3D {
    GDCLASS(Vehicle, RigidBody3D);

	VehicleEngine engine;

	ClutchGearConstraint clutch_gearbox;
	GearboxModule gearbox_module;
	ShaftWheelsCouplingConstraint shaft_wheels_coupling;

	RotationalBody drive_shaft;



protected:
	static void _bind_methods();

public:
    Vehicle();
	~Vehicle() override = default;
	virtual void _ready() override;
	virtual void _integrate_forces(PhysicsDirectBodyState3D *state) override;

	std::vector<Axle*> axles;

	// Exported variables
	Ref<SuspensionData> suspension_data = nullptr;
	void set_suspension_data(const Ref<SuspensionData>& r);
	Ref<SuspensionData> get_suspension_data();

	Ref<VehicleEngineData> engine_data = nullptr;
	void set_engine_data(const Ref<VehicleEngineData>& r);
	Ref<VehicleEngineData> get_engine_data();

	Ref<GearboxData> gearbox_data = nullptr;
	void set_gearbox_data(const Ref<GearboxData>& r);
	Ref<GearboxData> get_gearbox_data();

	Ref<TireData> tire_data = nullptr;
	void set_tire_data(const Ref<TireData>& t);
	Ref<TireData> get_tire_data();

	real_t throttle_input = 0.0;
	real_t steer_input = 0.0;
	real_t brake_input = 0.0;
	int shift_input = 0;

	void set_throttle_input(real_t value);
	real_t get_throttle_input() const;
	void set_steer_input(real_t value);
	real_t get_steer_input() const;
	void set_brake_input(real_t value);
	real_t get_brake_input() const;
	void set_shift_input(int value);
	int get_shift_input() const;

	// Telemetry accessors
	VehicleEngine& get_engine() { return engine; }
	const ClutchGearConstraint& get_clutch_gearbox() const { return clutch_gearbox; }
	const RotationalBody& get_driveshaft() const { return drive_shaft; }
	int get_axle_count() const { return static_cast<int>(axles.size()); }
	real_t get_speed_kph() const;
    
};

} // namespace godot