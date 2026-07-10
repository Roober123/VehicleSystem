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

    ClassDB::bind_method(D_METHOD("set_throttle_input", "value"), &Vehicle::set_throttle_input);
    ClassDB::bind_method(D_METHOD("get_throttle_input"), &Vehicle::get_throttle_input);
    

    ClassDB::bind_method(D_METHOD("set_steer_input", "value"), &Vehicle::set_steer_input);
    ClassDB::bind_method(D_METHOD("get_steer_input"), &Vehicle::get_steer_input);
    

    ClassDB::bind_method(D_METHOD("set_brake_input", "value"), &Vehicle::set_brake_input);
    ClassDB::bind_method(D_METHOD("get_brake_input"), &Vehicle::get_brake_input);
    

    ClassDB::bind_method(D_METHOD("set_shift_input", "value"), &Vehicle::set_shift_input);
    ClassDB::bind_method(D_METHOD("get_shift_input"), &Vehicle::get_shift_input);

	
    ClassDB::bind_method(D_METHOD("set_aero_data", "data"), &Vehicle::set_aero_data);
    ClassDB::bind_method(D_METHOD("get_aero_data"), &Vehicle::get_aero_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "aero_data", PROPERTY_HINT_RESOURCE_TYPE, "VehicleAerodynamicsData"), "set_aero_data", "get_aero_data");

    ClassDB::bind_method(D_METHOD("set_turbo_data", "data"), &Vehicle::set_turbo_data);
    ClassDB::bind_method(D_METHOD("get_turbo_data"), &Vehicle::get_turbo_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "turbo_data", PROPERTY_HINT_RESOURCE_TYPE, "TurboData"), "set_turbo_data", "get_turbo_data");

    ClassDB::bind_method(D_METHOD("set_abs_enabled", "value"), &Vehicle::set_abs_enabled);
    ClassDB::bind_method(D_METHOD("get_abs_enabled"), &Vehicle::get_abs_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "abs_enabled"), "set_abs_enabled", "get_abs_enabled");

    ClassDB::bind_method(D_METHOD("set_tcs_enabled", "value"), &Vehicle::set_tcs_enabled);
    ClassDB::bind_method(D_METHOD("get_tcs_enabled"), &Vehicle::get_tcs_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "tcs_enabled"), "set_tcs_enabled", "get_tcs_enabled");


    ClassDB::bind_method(D_METHOD("set_substeps", "value"), &Vehicle::set_substeps);
    ClassDB::bind_method(D_METHOD("get_substeps"), &Vehicle::get_substeps);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "substeps", PROPERTY_HINT_RANGE, "1,1024,1"), "set_substeps", "get_substeps");


    ClassDB::bind_method(D_METHOD("get_exhaust_smoke"), &Vehicle::get_exhaust_smoke);
    ClassDB::bind_method(D_METHOD("get_exhaust_heat"), &Vehicle::get_exhaust_heat);
    ClassDB::bind_method(D_METHOD("get_exhaust_flame_probability"), &Vehicle::get_exhaust_flame_probability);

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
	set_linear_damp_mode(DampMode::DAMP_MODE_REPLACE);
	set_angular_damp_mode(DampMode::DAMP_MODE_REPLACE);
	set_linear_damp(0.0);
	set_angular_damp(0.0);
	engine = VehicleEngine(engine_data);
	if (turbo_data != nullptr) {
		engine.set_turbo(&turbo);
		turbo.configure(turbo_data);
	}

	clutch_gearbox.set_engine(&engine);

	if (gearbox_data.is_valid()) {
		PackedFloat64Array ratios = gearbox_data->get_gear_ratios();
		clutch_gearbox.gear_ratios.clear();
		for (int i = 0; i < ratios.size(); i++)
			clutch_gearbox.gear_ratios.push_back(ratios[i]);
		
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
		}
	}

	shaft_wheels_coupling.load_bodies(&drive_shaft, axles);

	_compute_axle_dimensions();
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

	_update_suspension(state, body_origin, com_global, linear_velocity, angular_velocity);

	aerodynamics.compute(linear_velocity);
	_apply_aerodynamics(linear_velocity, angular_velocity, body_origin, steer_input);

	_handle_auto_gearbox();

	if (gearbox_module.get_current_gear() < 0)
		engine.throttle = brake_input;
	else 
		engine.throttle = tcs_enabled ? tcs.apply(throttle_input, get_linear_velocity().length(), axles) : throttle_input;  
	

	const real_t dt = state->get_step();
	const real_t sub_dt = dt / substeps;

	gearbox_module.update(dt, brake_input, throttle_input);

	const real_t wheel_brake = (gearbox_module.get_current_gear() < 0) ? throttle_input : brake_input;
	const real_t speed_kph = get_speed_kph();

	_run_drivetrain_substeps(state, com_global, linear_velocity, angular_velocity, 
		sub_dt, wheel_brake, speed_kph, abs_enabled);


	exhaust_system.update(dt, engine, gearbox_module, turbo);
	


	const real_t inv_substeps = real_t{1.0} / substeps;
	for (auto &axle : axles)
		for (auto& wh : axle->get_wheels()) {
			const Vector3 avg_force = wh->tire_force * inv_substeps;
			Vector3 offset = wh->collision_point - body_origin;
			apply_force(avg_force, offset);
			wh->cached_tire_force = avg_force;
			wh->tire_force = Vector3(0,0,0);
		}
	
}


void Vehicle::_apply_aerodynamics(const Vector3 &linear_velocity, const Vector3 &angular_velocity, const Vector3 &body_origin, real_t steer_input) {
	real_t speed_mag = linear_velocity.length();
	if (speed_mag > real_t{0.1}) {
		Vector3 drag_dir = -linear_velocity / speed_mag;
		apply_central_force(drag_dir * aerodynamics.get_drag_force());
	}

	real_t yaw_rate = angular_velocity.y;
	if (std::abs(yaw_rate) > real_t{0.001}) {
		real_t blend = real_t{1.0} - std::abs(steer_input);
		blend *= blend;
		real_t yaw_torque_y = -get_mass() * yaw_rate * std::abs(yaw_rate)
		                    * aerodynamics.get_yaw_damping_coefficient() * blend;
		apply_torque(Vector3(0.0, yaw_torque_y, 0.0));
	}
	

	real_t total_df = aerodynamics.get_downforce();
	if (total_df > real_t{0.01}) {
		for (auto *axle : axles) {
			real_t axle_df = total_df * axle->downforce_ratio;
			if (axle_df < real_t{0.01}) continue;
			const auto &wheels = axle->get_wheels();
			size_t grounded_wheels = 0;
			for (auto *wh : wheels)
				if (wh->is_on_ground()) ++grounded_wheels;
			if (grounded_wheels == 0) continue;
			real_t per_wheel = axle_df / static_cast<real_t>(grounded_wheels);
			for (auto *wh : wheels) {
				if (!wh->is_on_ground()) continue;
				Vector3 off = wh->collision_point - body_origin;
				apply_force(wh->collision_normal * -per_wheel, off);
			}
		}
	}
}


void Vehicle::_update_suspension(PhysicsDirectBodyState3D *state, const Vector3 &body_origin, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity) {
	for (auto& axle : axles) {
		axle->update_physics(state, com_global, linear_velocity, angular_velocity);
		for (auto &wh : axle->get_wheels()) {
			if (!wh->is_on_ground()) continue;
			Vector3 offset = wh->collision_point - body_origin;
			apply_force(wh->collision_normal * wh->get_suspension_rebound_force(), offset);
		}
		// anti roll
		const auto& wh = axle->get_wheels();
		if (wh.size() >= 2 && wh[0]->is_on_ground() && wh[1]->is_on_ground()) {
			real_t arb_force = axle->get_antiroll_bar_force();
			if (std::abs(arb_force) > 0.01) {
				Vector3 offL = wh[0]->collision_point - body_origin;
				Vector3 offR = wh[1]->collision_point - body_origin;
				apply_force(wh[0]->collision_normal * -arb_force, offL);
				apply_force(wh[1]->collision_normal * arb_force, offR);
			}
		}
	}
}

void Vehicle::_run_drivetrain_substeps(PhysicsDirectBodyState3D *state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t sub_dt, real_t wheel_brake, real_t speed_kph, bool abs_enabled) {
	real_t prev_reflected_load = 0.0; // estimate from previous substep (0 for first)

	for (int s = 0; s < substeps; ++s) {
		engine.accumulate_torque(sub_dt);
		clutch_gearbox.solve(sub_dt, engine.get_torque(), prev_reflected_load);
		shaft_wheels_coupling.solve();

		for (auto &ax : axles) {
			if (ax->is_steerable) {
				ax->solve_steering(steer_input, sub_dt, speed_kph);
				ax->set_wheels_rotation();
			}
			ax->solve_tire(state, com_global, linear_velocity, angular_velocity, sub_dt, wheel_brake, abs_enabled);
		}

		// for clutch in next frame
		prev_reflected_load = _compute_reflected_load();

		engine.integrate(sub_dt);
		drive_shaft.integrate(sub_dt);

		for (auto &axle : axles)
			axle->integrate(sub_dt);
	}
}

real_t Vehicle::_compute_reflected_load() const {
	const real_t ratio = clutch_gearbox.get_effective_ratio();
	if (ratio == 0.0)	return 0.0;

	real_t load = 0.0;
	for (auto &ax : axles) {
		if (ax->drive_ratio > 0.0)
			for (auto* wh : ax->get_wheels())
				load += (-wh->reaction_torque) / ratio * ax->drive_ratio;
	}
	return load;
}

void Vehicle::_handle_auto_gearbox() {
	if (!gearbox_module.is_automatic() || gearbox_module.is_shifting())
		return;

	const real_t speed = get_speed_kph();
	if (speed >= 0.5)
		return;

	const int gear = gearbox_module.get_current_gear();
	if (gear > 0 && brake_input > 0.3 && throttle_input < 0.05)
		gearbox_module.select_reverse();
	else if (gear < 0 && throttle_input > 0.1 && brake_input < 0.05)
		gearbox_module.select_drive();
}

void Vehicle::_compute_axle_dimensions() {
    // Compute trackwidth for each axle (distance between its two wheels)
    for (auto* axle : axles) {
        const auto& wheels = axle->get_wheels();
        if (wheels.size() >= 2) {
            real_t tw = wheels[0]->get_global_position()
                            .distance_to(wheels[1]->get_global_position());
            axle->set_trackwidth(tw);
        }
    }

    // Compute wheelbase (max distance between any two axles)
    if (axles.size() >= 2) {
        real_t wb = 0.0;
        for (size_t i = 0; i < axles.size(); ++i) {
            for (size_t j = i + 1; j < axles.size(); ++j) {
                real_t dist = axles[i]->get_global_position()
                                  .distance_to(axles[j]->get_global_position());
                wb = std::max(wb, dist);
            }
        }
        for (auto* axle : axles)
            axle->set_wheelbase(wb);
    }
}

void Vehicle::set_throttle_input(real_t value) {
    throttle_input = std::clamp(value, real_t{0.0}, real_t{1.0});
}

void Vehicle::set_steer_input(real_t value) {
    steer_input = std::clamp(value, real_t{-1.0}, real_t{1.0});
}

real_t Vehicle::get_speed_kph() const {
    return get_linear_velocity().length() * 3.6;
}

void Vehicle::set_brake_input(real_t value) {
    brake_input = std::clamp(value, real_t{0.0}, real_t{1.0});
}

void Vehicle::set_tcs_enabled(bool value) {
    tcs_enabled = value;
}

void Vehicle::set_shift_input(int value) {
    shift_input = std::clamp(value, -1, 1);
    if (shift_input == 1)
        gearbox_module.shift_up();
    else if (shift_input == -1)
        gearbox_module.shift_down();
}

void Vehicle::set_substeps(int value) {
    substeps = std::clamp(value, 1, 1024);
}

void Vehicle::set_aero_data(const Ref<VehicleAerodynamicsData>& a) {
    aero_data = a;
    aerodynamics.load_parameters(aero_data);
}



}
