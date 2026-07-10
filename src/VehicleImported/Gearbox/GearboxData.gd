class_name GearboxData
extends Resource

@export_range(0.1, 1.0) var efficiency : float = 0.5
@export var ratios : Array[float]
@export var final_drive : float = 4.0
## in nm / frame
var torque_resistance : float = 20

## manuals are the lightest 20 - 70
## automatics 50 - 150
## very high gear number 200 - 250
@export_range(20.0, 250.0) var mass : float = 50

@export var is_automatic : bool = false
@export var auto_downshift : float = 0.4
@export var auto_upshift : float = 0.8
@export var auto_clutch_speed : float = 5
