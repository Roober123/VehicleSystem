extends Camera3D

@export var target: Vehicle = null
@export var follow_speed: float = 5.0         # How fast the camera catches up (lower = smoother)
@export var distance: float = 12.0            # Distance behind the car
@export var height: float = 4.0               # Height above the car

var _current_offset: Vector3 = Vector3.ZERO


func _ready() -> void:
	if target == null:
		# Auto-find the Vehicle if not set
		target = get_parent().get_node_or_null("Vehicle") as Vehicle

	if target == null:
		return

	# Initialize offset to a reasonable starting position
	_current_offset = Vector3(0, height, -distance)


func _process(delta: float) -> void:
	if target == null:
		return

	# Target position: behind and above the car in world space
	var car_pos: Vector3 = target.global_position
	var car_forward: Vector3 = -target.global_transform.basis.z  # Vehicle's forward

	# Desired camera position: behind + above the car
	var desired_pos: Vector3 = car_pos + Vector3.UP * height + car_forward * distance

	# Smoothly interpolate position toward desired
	_current_offset = _current_offset.lerp(desired_pos, follow_speed * delta)
	global_position = _current_offset

	# Always look at the car
	look_at(car_pos, Vector3.UP)
