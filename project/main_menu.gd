extends Control


const TEST_SCENE := "res://example.tscn"
const OPTIONS_SCENE := "res://options.tscn"

@onready var launch_button: Button = $CenterContainer/MenuPanel/MarginContainer/Content/LaunchButton


func _ready() -> void:
	launch_button.grab_focus.call_deferred()


func _on_launch_button_pressed() -> void:
	_change_scene(TEST_SCENE)


func _on_options_button_pressed() -> void:
	_change_scene(OPTIONS_SCENE)


func _on_quit_button_pressed() -> void:
	get_tree().quit()


func _change_scene(scene_path: String) -> void:
	var result := get_tree().change_scene_to_file(scene_path)
	if result != OK:
		push_error("Unable to change to scene '%s' (error %d)." % [scene_path, result])
