extends Node3D

## Synchronizes simple wheel meshes with a Vehicle's wheel contacts.
##
## Mesh references and rest positions are cached once the native Vehicle has
## completed setup. A different Vehicle can be selected in the Inspector
## without changing the visual node's implementation.

const DEFAULT_WHEEL_RADIUS := 0.3

@export var vehicle: Vehicle

var _wheel_visuals: Array[Dictionary] = []
var _telemetry: VehicleTelemetry


func _enter_tree() -> void:
	_connect_vehicle_ready()


func _ready() -> void:
	_connect_vehicle_ready()
	_resolve_telemetry()
	_cache_wheel_visuals()


func _on_vehicle_ready() -> void:
	_resolve_telemetry()
	_cache_wheel_visuals()


func _connect_vehicle_ready() -> void:
	if not is_instance_valid(vehicle):
		return
	var ready_callback := Callable(self, "_on_vehicle_ready")
	if not vehicle.is_connected("vehicle_ready", ready_callback):
		vehicle.connect("vehicle_ready", ready_callback)


func _resolve_telemetry() -> void:
	_telemetry = null
	if not is_instance_valid(vehicle):
		return
	_telemetry = vehicle.get_node_or_null("VehicleTelemetry") as VehicleTelemetry


func _cache_wheel_visuals() -> void:
	_wheel_visuals.clear()
	if not is_instance_valid(vehicle):
		return

	for axle in vehicle.get_children():
		if not axle is Axle:
			continue

		for wheel in axle.get_children():
			if not wheel is Wheel:
				continue

			var mesh: MeshInstance3D
			for child in wheel.get_children():
				if child is MeshInstance3D:
					mesh = child
					break
			if mesh == null:
				continue

			_wheel_visuals.append({
				"wheel": wheel,
				"mesh": mesh,
				"rest_position": mesh.position,
				"radius": _get_wheel_radius(mesh),
			})


func _process(delta: float) -> void:
	if not is_instance_valid(vehicle):
		return
	if not is_instance_valid(_telemetry):
		_resolve_telemetry()
	if _wheel_visuals.is_empty():
		_cache_wheel_visuals()
	if not is_instance_valid(_telemetry):
		return

	var response_weight: float = minf(9.8 * delta, 1.0)
	for visual in _wheel_visuals:
		var wheel := visual["wheel"] as Wheel
		var mesh := visual["mesh"] as MeshInstance3D
		if not is_instance_valid(wheel) or not is_instance_valid(mesh):
			continue

		var tire_telemetry: Dictionary = _telemetry.get_tire_telemetry(wheel)
		if tire_telemetry.is_empty():
			continue

		var rest_position: Vector3 = visual["rest_position"]
		if tire_telemetry.get("grounded", false):
			var contact_position: Vector3 = tire_telemetry.get("contact_position", Vector3.ZERO)
			var radius: float = visual["radius"]
			var suspension_axis: Vector3 = wheel.global_transform.basis.y.normalized()
			var desired_center_global: Vector3 = contact_position + suspension_axis * radius
			var desired_center_local: Vector3 = wheel.to_local(desired_center_global)
			var target_local := Vector3(rest_position.x, desired_center_local.y, rest_position.z)
			mesh.position = mesh.position.lerp(target_local, response_weight)
		else:
			mesh.position = mesh.position.lerp(rest_position, response_weight)


func _get_wheel_radius(mesh: MeshInstance3D) -> float:
	if mesh.mesh == null:
		return DEFAULT_WHEEL_RADIUS

	var mesh_size := mesh.mesh.get_aabb().size
	var radius := maxf(mesh_size.x, mesh_size.z) * 0.5
	var radial_scale := maxf(absf(mesh.scale.x), absf(mesh.scale.z))
	return maxf(radius * radial_scale, 0.001)
