#pragma once

#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/core/class_db.hpp"

#include "Resources/gearbox_data.h"
#include "Resources/suspension_data.h"
#include "Resources/turbo_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "Resources/vehicle_engine_data.h"
#include "Resources/differential_data.h"

namespace godot {

/// Immutable-at-runtime aggregate of the data resources needed to initialize a
/// Vehicle.  The Vehicle consumes this once from _ready(); changing the
/// resource afterwards does not reconfigure an initialized Vehicle.
class VehicleConfig : public Resource {
    GDCLASS(VehicleConfig, Resource);

    Ref<VehicleEngineData> engine_data;
    Ref<GearboxData> gearbox_data;
    Ref<SuspensionData> suspension_data;
    Ref<VehicleAerodynamicsData> aero_data;
    Ref<TurboData> turbo_data;
    Ref<DifferentialData> center_differential_data;

protected:
    static void _bind_methods();

public:
    VehicleConfig() = default;
    ~VehicleConfig() override = default;

    void set_engine_data(const Ref<VehicleEngineData> &value) { engine_data = value; }
    Ref<VehicleEngineData> get_engine_data() const { return engine_data; }

    void set_gearbox_data(const Ref<GearboxData> &value) { gearbox_data = value; }
    Ref<GearboxData> get_gearbox_data() const { return gearbox_data; }

    void set_suspension_data(const Ref<SuspensionData> &value) { suspension_data = value; }
    Ref<SuspensionData> get_suspension_data() const { return suspension_data; }

    void set_aero_data(const Ref<VehicleAerodynamicsData> &value) { aero_data = value; }
    Ref<VehicleAerodynamicsData> get_aero_data() const { return aero_data; }

    void set_turbo_data(const Ref<TurboData> &value) { turbo_data = value; }
    Ref<TurboData> get_turbo_data() const { return turbo_data; }

    void set_center_differential_data(const Ref<DifferentialData> &value) { center_differential_data = value; }
    Ref<DifferentialData> get_center_differential_data() const { return center_differential_data; }
};

} // namespace godot
