extends Vehicle

const DEFAULT_WHEEL_RADIUS := 0.3

var _wheel_visuals: Array[Dictionary] = []


func _enter_tree() -> void:
	var ready_callback := Callable(self, "_on_vehicle_ready")
	if not is_connected("vehicle_ready", ready_callback):
		connect("vehicle_ready", ready_callback)


func _on_vehicle_ready() -> void:
	_cache_wheel_visuals()


func _cache_wheel_visuals() -> void:
	_wheel_visuals.clear()

	for axle in get_children():
		if not axle is Axle:
			continue

		for wheel in axle.get_children():
			if not wheel is Wheel:
				continue

			var mesh := wheel.find_child("MeshInstance3D", true, false) as MeshInstance3D
			if mesh == null:
				continue

			_wheel_visuals.append({
				"wheel": wheel,
				"mesh": mesh,
				"rest_position": mesh.position,
				"radius": _get_wheel_radius(mesh),
			})


func _process(delta: float) -> void:
	for visual in _wheel_visuals:
		var wheel := visual["wheel"] as Wheel
		var mesh := visual["mesh"] as MeshInstance3D
		if not is_instance_valid(wheel) or not is_instance_valid(mesh):
			continue

		if wheel.is_on_ground():
			var contact_point: Vector3 = wheel.get_collision_point()
			var contact_normal: Vector3 = wheel.get_collision_normal().normalized()
			var radius: float = visual["radius"]
			mesh.global_position = lerp(mesh.global_position, contact_point + contact_normal * radius, delta * 5)
			#mesh.global_position = contact_point + contact_normal * radius
		else:
			mesh.position = visual["rest_position"]


func _get_wheel_radius(mesh: MeshInstance3D) -> float:
	if mesh.mesh == null:
		return DEFAULT_WHEEL_RADIUS

	var mesh_size := mesh.mesh.get_aabb().size
	var radius := maxf(mesh_size.x, mesh_size.z) * 0.5
	var radial_scale := maxf(absf(mesh.scale.x), absf(mesh.scale.z))
	return maxf(radius * radial_scale, 0.001)
