#pragma once

#include "godot_cpp/classes/ref_counted.hpp"

namespace godot {

/// Deterministic, headless regression surface for the native audio core.
/// This class is compiled and registered only by `scons tests=1`.
class AudioRegression : public RefCounted {
	GDCLASS(AudioRegression, RefCounted);

protected:
	static void _bind_methods();

public:
	bool run();
};

} // namespace godot
