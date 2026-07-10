class_name TireData
extends Resource

@export var mu : float = 1.0
@export var radius : float = 0.3
@export var mesh_scene : PackedScene
## how fast self-aligning-torque decays when gaining lateral slip
@export var sat_slip_loss : float = 6.0
@export_range(0.0, 0.35) var drift_coeff : float = 0.35

@export_range(0.6, 1.0) var tire_temp_gain : float = 1.0
@export_range(1.0, 3.0) var tire_temp_cool : float = 1.0
