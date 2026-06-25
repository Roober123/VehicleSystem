#include "DriveShaft.h"

// Plain RotationalBody — clutch torque added by ClutchGearConstraint::solve().
// Read drive_shaft.get_torque() before integrate() to get wheel drive torque.