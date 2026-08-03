#include "vehicle_setup_validation.h"

#include <vector>

#include "axle.h"

namespace godot {
namespace {

using ErrorList = std::vector<String>;

void add_error(ErrorList &errors, const String &message) {
    errors.push_back(message);
}

String join_errors(const ErrorList &errors) {
    String result;
    for (size_t i = 0; i < errors.size(); ++i) {
        if (i != 0)
            result += "; ";
        result += errors[i];
    }
    return result;
}

String axle_text(const char *prefix, const String &axle_name, const char *suffix) {
    String result = String(prefix);
    result += axle_name;
    result += String(suffix);
    return result;
}

void validate_engine(const Ref<VehicleEngineData> &data, ErrorList &errors) {
    if (data.is_null()) {
        add_error(errors, "engine_data is required");
        return;
    }
    if (data->get_torque_curve().is_null())
        add_error(errors, "engine_data.torque_curve is required");
}

void validate_gearbox(const Ref<GearboxData> &data, ErrorList &errors) {
    if (data.is_null())
        add_error(errors, "gearbox_data is required");
}

void validate_suspension(const Ref<SuspensionData> &data, ErrorList &errors) {
    if (data.is_null())
        add_error(errors, "suspension_data is required");
}

void validate_tire(const Ref<TireData> &data, const String &axle_name, ErrorList &errors) {
    if (data.is_null()) {
        add_error(errors, axle_text("axle '", axle_name, "' requires tire_data"));
        return;
    }
}

void validate_steering(const Ref<SteeringRackData> &data, const String &axle_name, ErrorList &errors) {
    if (data.is_null()) {
        add_error(errors, axle_text("steerable axle '", axle_name, "' requires steering_rack_data"));
        return;
    }
}

} // namespace

bool VehicleSetupValidation::validate(const Ref<VehicleConfig> &config,
                                      const std::vector<Axle *> &axles,
                                      String &error) {
    ErrorList errors;
    if (config.is_null()) {
        error = "config is required";
        return false;
    }

    validate_engine(config->get_engine_data(), errors);
    validate_gearbox(config->get_gearbox_data(), errors);
    validate_suspension(config->get_suspension_data(), errors);

    if (axles.empty())
        add_error(errors, "at least one axle is required");

    size_t driven_axle_count = 0;
    for (const Axle *axle : axles) {
        if (axle == nullptr) {
            add_error(errors, "axle topology contains a null axle");
            continue;
        }

        const String axle_name = axle->get_name().is_empty() ? String("(unnamed)") : String(axle->get_name());
        const auto &wheels = axle->get_wheels();
        if (wheels.size() != 2) {
            add_error(errors, axle_text("axle '", axle_name, "' must contain exactly two wheels"));
        } else {
            if (wheels[0] == nullptr || wheels[1] == nullptr)
                add_error(errors, axle_text("axle '", axle_name, "' contains a null wheel"));
            if (wheels[0] != nullptr && wheels[0] == wheels[1])
                add_error(errors, axle_text("axle '", axle_name, "' wheels must be distinct"));
        }

        validate_tire(axle->get_tire_data(), axle_name, errors);
        if (axle->get_steerable())
            validate_steering(axle->get_steering_rack_data(), axle_name, errors);

        if (axle->get_drive_share() > real_t{0.0}) {
            ++driven_axle_count;
            if (axle->get_differential_data().is_null())
                add_error(errors, axle_text("driven axle '", axle_name, "' requires differential_data"));
        }
    }

    if (driven_axle_count == 0)
        add_error(errors, "at least one driven axle is required (drive_share > 0)");
    else if (driven_axle_count > 2)
        add_error(errors, "at most two driven axles are supported");
    else if (driven_axle_count == 2 && config->get_center_differential_data().is_null())
        add_error(errors, "two driven axles require center_differential_data");

    error = join_errors(errors);
    return errors.empty();
}

} // namespace godot
