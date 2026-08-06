extends Node3D

## Scene-level navigation for the example vehicle.
##
## Driving input, telemetry presentation, and wheel visuals live on their own
## reusable nodes so a second vehicle can be retargeted without changing this
## scene shell.

var _returning_to_menu := false


func _ready() -> void:
	$NavigationLayer/NavigationPanel/BackToMenuButton.pressed.connect(_return_to_menu)


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("return_to_menu") or event.is_action_pressed("ui_cancel"):
		_return_to_menu()
		get_viewport().set_input_as_handled()
	var target : Vehicle
	if event.is_action_pressed("switch_car_1"):
		target= $Vehicle1
		$ChaseCamera1.current = true
		$ChaseCamera2.current = false
		
	if event.is_action_pressed("switch_car_2"):
		target = $Vehicle2
		$ChaseCamera2.current = true
		$ChaseCamera1.current = false
	
	if target:
		$VehicleController.vehicle = target
		$Control.vehicle = target
		$Control._telemetry = null


func _return_to_menu() -> void:
	if _returning_to_menu:
		return
	_returning_to_menu = true
	var result := get_tree().change_scene_to_file("res://main_menu.tscn")
	if result != OK:
		_returning_to_menu = false
		push_error("Unable to return to the main menu (error %d)." % result)
