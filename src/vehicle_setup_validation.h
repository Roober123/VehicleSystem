#pragma once

#include <vector>

#include "godot_cpp/classes/ref.hpp"
#include "godot_cpp/variant/string.hpp"

#include "Resources/vehicle_config.h"

namespace godot {

class Axle;

/// Validates the complete Vehicle composition before any runtime state is
/// configured.  Validation is intentionally centralized so a failed setup
/// produces one consolidated diagnostic and no partially initialized vehicle.
class VehicleSetupValidation {
public:
    static bool validate(const Ref<VehicleConfig> &config,
                         const std::vector<Axle *> &axles,
                         String &error);
};

} // namespace godot
