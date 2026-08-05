#pragma once

#include <array>
#include <cstddef>

#include "Drivetrain/RotationalConstraint.h"
#include "Resources/differential_data.h"

namespace godot {

/// Differential policy copied from DifferentialData during setup. Runtime
/// capacity is returned directly; invalid inputs produce NaN.
struct DifferentialSettings {
	DifferentialData::Mode mode = DifferentialData::OPEN;
	real_t preload_torque = real_t{0.0};
	real_t power_lock_ratio = real_t{0.0};
	real_t coast_lock_ratio = real_t{0.0};
	real_t slip_sensitive_gain = real_t{0.0};
	real_t max_lock_torque = real_t{0.0};

	static DifferentialSettings from_resource(const Ref<DifferentialData> &data);
	bool is_valid() const;
	real_t capacity(real_t transmitted_torque, real_t carrier_velocity,
			real_t relative_slip) const;
};

/// Runtime value for one driven two-wheel axle. `share` is the setup weight;
/// RotationalNetwork::configure normalizes the stored copy before using it in
/// the fixed primary constraint.
struct DrivenAxle {
	RotationalBody *left = nullptr;
	RotationalBody *right = nullptr;
	real_t share = real_t{0.0};
	DifferentialSettings differential;

	bool is_valid() const;
};

struct ClutchTelemetry {
	bool present = false;
	real_t requested_engine_torque = real_t{0.0};
	real_t transmitted_torque = real_t{0.0};
	real_t output_torque = real_t{0.0};
	real_t slip = real_t{0.0};
	bool slipping = false;
};

/// Value-owned direct rotational network. Setup stores only the primary constraint,
/// axle bodies, normalized share copies, and differential settings. Carrier
/// and relative coordinates are computed from the two wheel bodies at each
/// solve; no per-axle constraints are retained.
class RotationalNetwork {
public:
	static constexpr std::size_t MAX_AXLES = 8;
	static constexpr std::size_t MAX_WHEELS = MAX_AXLES * 2;
	static constexpr std::size_t MAX_BODIES = MAX_WHEELS + 2;

	bool configure(RotationalBody *engine_body, RotationalBody *shaft_body,
			const std::array<DrivenAxle, MAX_AXLES> &axles,
			std::size_t axle_count);
	bool configure(RotationalBody &engine_body, RotationalBody &shaft_body,
			const std::array<DrivenAxle, MAX_AXLES> &axles,
			std::size_t axle_count) {
		return configure(&engine_body, &shaft_body, axles, axle_count);
	}

	std::size_t get_axle_count() const { return axle_count; }

	/// Solve one direct drivetrain substep and atomically commit all staged
	/// torques. The clutch telemetry is reset on entry and retained on success.
	bool solve(real_t dt, real_t signed_ratio, real_t engagement,
			real_t clutch_max_torque);
	const ClutchTelemetry &get_clutch_telemetry() const {
		return clutch_telemetry;
	}

private:
	RotationalBody *engine = nullptr;
	RotationalBody *shaft = nullptr;
	VelocityConstraint primary_constraint;
	std::array<DrivenAxle, MAX_AXLES> axles{};
	std::size_t axle_count = 0;
	bool configured = false;
	ClutchTelemetry clutch_telemetry;
};

} // namespace godot
