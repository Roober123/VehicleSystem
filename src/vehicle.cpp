#include "vehicle.h"
#include <algorithm>

namespace godot {

void Vehicle::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_suspension_data", "data"), &Vehicle::set_suspension_data);
    ClassDB::bind_method(D_METHOD("get_suspension_data"), &Vehicle::get_suspension_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "suspension_data", PROPERTY_HINT_RESOURCE_TYPE, "SuspensionData"),
				 "set_suspension_data", "get_suspension_data");

    ClassDB::bind_method(D_METHOD("set_engine_data", "data"), &Vehicle::set_engine_data);
    ClassDB::bind_method(D_METHOD("get_engine_data"), &Vehicle::get_engine_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine_data", PROPERTY_HINT_RESOURCE_TYPE, "VehicleEngineData"),
				 "set_engine_data", "get_engine_data");

    ClassDB::bind_method(D_METHOD("set_gearbox_data", "data"), &Vehicle::set_gearbox_data);
    ClassDB::bind_method(D_METHOD("get_gearbox_data"), &Vehicle::get_gearbox_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "gearbox_data", PROPERTY_HINT_RESOURCE_TYPE, "GearboxData"),
				 "set_gearbox_data", "get_gearbox_data");

    ClassDB::bind_method(D_METHOD("set_tire_data", "data"), &Vehicle::set_tire_data);
    ClassDB::bind_method(D_METHOD("get_tire_data"), &Vehicle::get_tire_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "tire_data", PROPERTY_HINT_RESOURCE_TYPE, "TireData"),
				 "set_tire_data", "get_tire_data");

    ClassDB::bind_method(D_METHOD("set_throttle_input", "value"), &Vehicle::set_throttle_input);
    ClassDB::bind_method(D_METHOD("get_throttle_input"), &Vehicle::get_throttle_input);
    

    ClassDB::bind_method(D_METHOD("set_steer_input", "value"), &Vehicle::set_steer_input);
    ClassDB::bind_method(D_METHOD("get_steer_input"), &Vehicle::get_steer_input);
    

    ClassDB::bind_method(D_METHOD("set_brake_input", "value"), &Vehicle::set_brake_input);
    ClassDB::bind_method(D_METHOD("get_brake_input"), &Vehicle::get_brake_input);
    

    ClassDB::bind_method(D_METHOD("set_shift_input", "value"), &Vehicle::set_shift_input);
    ClassDB::bind_method(D_METHOD("get_shift_input"), &Vehicle::get_shift_input);
}

Vehicle::Vehicle() {
	axles = std::vector<Axle*>();
	engine = VehicleEngine();

	clutch_gearbox.set_engine(&engine);
	clutch_gearbox.set_output(&drive_shaft);
}

void Vehicle::_ready() {
	if (Engine::get_singleton()->is_editor_hint())
        return;

	engine = VehicleEngine(engine_data);

	clutch_gearbox.set_engine(&engine);

	if (gearbox_data.is_valid()) {
		PackedFloat64Array ratios = gearbox_data->get_gear_ratios();
		clutch_gearbox.gear_ratios.clear();
		for (int i = 0; i < ratios.size(); i++) {
			clutch_gearbox.gear_ratios.push_back(ratios[i]);
		}
		clutch_gearbox.final_drive = gearbox_data->get_final_drive();
		clutch_gearbox.clutch_max_torque = gearbox_data->get_clutch_max_torque();
		clutch_gearbox.reverse_ratio = gearbox_data->get_reverse_ratio();

		gearbox_module.load_constraint(&clutch_gearbox, gearbox_data);
	}

	TypedArray<Node> arr = get_children();
	for (int i = 0; i < arr.size(); i++) {
		Axle *iter = Object::cast_to<Axle>(arr[i]);
		if (iter != nullptr) {
			axles.push_back(iter);
		}
	}
	int sz = axles.size();
	real_t mass_per_axle = get_mass() / sz;
	for (auto& i : axles)
		i->compute_suspension_parameters(mass_per_axle, suspension_data);

	// Add rotational drag to wheels to prevent drivetrain oscillation
	for (auto& axle : axles) {
		for (auto* wh : axle->get_wheels()) {
			wh->body.set_drag(real_t{0.05});
			wh->set_tire(tire_data);
		}
	}

	shaft_wheels_coupling.load_bodies(&drive_shaft, axles);
}

void Vehicle::_integrate_forces(PhysicsDirectBodyState3D *state) {
	RigidBody3D::_integrate_forces(state);
	if (suspension_data == nullptr) return;
	if (engine_data == nullptr) return;
	
	// suspension
	Vector3 body_origin = state->get_transform().get_origin();
	Vector3 com_global = state->get_transform().xform(state->get_center_of_mass());
	Vector3 linear_velocity = state->get_linear_velocity();
	Vector3 angular_velocity = state->get_angular_velocity();
	for (auto& axle : axles) {
		axle->update_physics(state, com_global, linear_velocity, angular_velocity);
		for(auto &wh : axle->get_wheels()) {
			Vector3 offset = wh->collision_point - body_origin;
			apply_force(wh->collision_normal * wh->get_suspension_rebound_force(), offset);
		}

	}

	
	if (gearbox_module.is_automatic() && !gearbox_module.is_shifting()) {
		const real_t speed = get_speed_kph();
		const bool nearly_stopped = speed < 0.5;
		if (nearly_stopped) {
			const int gear = gearbox_module.get_current_gear();
			if (gear > 0 && brake_input > 0.3 && throttle_input < 0.05)
				gearbox_module.select_reverse();
			else if (gear < 0 && throttle_input > 0.1 && brake_input < 0.05)
				gearbox_module.select_drive();
		}
	}

	
	if (gearbox_module.get_current_gear() < 0)
		engine.throttle = brake_input;
	else 
		engine.throttle = throttle_input;  
	

	const real_t dt = state->get_step();
	
	engine.accumulate_torque();
	gearbox_module.update(dt);
	clutch_gearbox.solve(dt);

	shaft_wheels_coupling.solve();

	const real_t wheel_brake = (gearbox_module.get_current_gear() < 0) ? throttle_input : brake_input;

	for (auto &ax : axles) {
		ax->solve_tire(state, com_global, linear_velocity, angular_velocity, dt, wheel_brake);
		
	}

	// Integrate
	engine.integrate(dt);
	drive_shaft.integrate(dt);

	for (auto &axle : axles)
		axle->integrate(dt);

	for (auto &axle : axles)
		for (auto& wh : axle->get_wheels()) {
			Vector3 offset = wh->collision_point - body_origin;
			apply_force(wh->tire_force, offset);
			wh->cached_tire_force = wh->tire_force;
			wh->tire_force = Vector3(0,0,0);
			wh->patch_torque = Vector3(0,0,0);
		}
	
}


void Vehicle::set_suspension_data(const Ref<SuspensionData>& r) {
    suspension_data = r;
}

Ref<SuspensionData> Vehicle::get_suspension_data() {
    return suspension_data;
}

void Vehicle::set_engine_data(const Ref<VehicleEngineData>& r) {
    engine_data = r;
}

Ref<VehicleEngineData> Vehicle::get_engine_data() {
    return engine_data;
}

void Vehicle::set_gearbox_data(const Ref<GearboxData>& r) {
    gearbox_data = r;
}

Ref<GearboxData> Vehicle::get_gearbox_data() {
    return gearbox_data;
}

void Vehicle::set_tire_data(const Ref<TireData>& t) {
    tire_data = t;
}

Ref<TireData> Vehicle::get_tire_data() {
    return tire_data;
}

void Vehicle::set_throttle_input(real_t value) {
    throttle_input = std::clamp<real_t>(value, real_t{0.0}, real_t{1.0});
}

real_t Vehicle::get_throttle_input() const {
    return throttle_input;
}

void Vehicle::set_steer_input(real_t value) {
    steer_input = std::clamp<real_t>(value, real_t{-1.0}, real_t{1.0});
}

real_t Vehicle::get_steer_input() const {
    return steer_input;
}


real_t Vehicle::get_speed_kph() const {
    return get_linear_velocity().length() * 3.6;
}

void Vehicle::set_brake_input(real_t value) {
    brake_input = std::clamp<real_t>(value, real_t{0.0}, real_t{1.0});
}

real_t Vehicle::get_brake_input() const {
    return brake_input;
}

void Vehicle::set_shift_input(int value) {
    shift_input = std::clamp(value, -1, 1);
    if (shift_input == 1)
        gearbox_module.shift_up();
    else if (shift_input == -1)
        gearbox_module.shift_down();
}

int Vehicle::get_shift_input() const {
    return shift_input;
}

} // namespace godot
