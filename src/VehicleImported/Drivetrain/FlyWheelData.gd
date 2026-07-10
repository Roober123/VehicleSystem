class_name FlywheelData
extends Resource

@export_range(3.0,25.0,0.1) var mass : float

func get_inertia()->float:
	var radius : float = 0.16
	return mass * radius * radius * 0.7
