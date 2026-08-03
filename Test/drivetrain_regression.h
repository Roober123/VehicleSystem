#pragma once

#include "godot_cpp/classes/ref_counted.hpp"

namespace godot {

/// Deterministic, headless regression surface for the drivetrain constraints.
///
/// This class is compiled and registered only by `scons tests=1`; it is not a
/// gameplay API and is absent from the normal extension build.
class DrivetrainRegression : public RefCounted {
    GDCLASS(DrivetrainRegression, RefCounted);

protected:
    static void _bind_methods();

public:
    bool run();
};

} // namespace godot
