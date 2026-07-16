extends Control


const DEFAULT_WINDOW_MODE := DisplayServer.WINDOW_MODE_WINDOWED
const DEFAULT_VSYNC_MODE := DisplayServer.VSYNC_ENABLED

@onready var window_mode_option: OptionButton = $CenterContainer/OptionsPanel/MarginContainer/Content/WindowModeRow/WindowModeOption
@onready var vsync_toggle: CheckButton = $CenterContainer/OptionsPanel/MarginContainer/Content/VSyncRow/VSyncToggle
@onready var back_button: Button = $CenterContainer/OptionsPanel/MarginContainer/Content/BackButton


func _ready() -> void:
	window_mode_option.add_item("Windowed", DisplayServer.WINDOW_MODE_WINDOWED)
	window_mode_option.add_item("Fullscreen", DisplayServer.WINDOW_MODE_FULLSCREEN)
	_refresh_from_display_server()
	back_button.grab_focus.call_deferred()


func _on_window_mode_option_item_selected(index: int) -> void:
	var mode := window_mode_option.get_item_id(index)
	DisplayServer.window_set_mode(mode)


func _on_vsync_toggle_toggled(enabled: bool) -> void:
	DisplayServer.window_set_vsync_mode(
		DisplayServer.VSYNC_ENABLED if enabled else DisplayServer.VSYNC_DISABLED
	)


func _on_reset_button_pressed() -> void:
	DisplayServer.window_set_mode(DEFAULT_WINDOW_MODE)
	DisplayServer.window_set_vsync_mode(DEFAULT_VSYNC_MODE)
	_refresh_from_display_server()


func _on_back_button_pressed() -> void:
	_change_scene("res://main_menu.tscn")


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		_on_back_button_pressed()
		get_viewport().set_input_as_handled()


func _refresh_from_display_server() -> void:
	var current_mode := DisplayServer.window_get_mode()
	var mode_index := window_mode_option.get_item_index(current_mode)
	if mode_index < 0:
		mode_index = window_mode_option.get_item_index(DEFAULT_WINDOW_MODE)
	window_mode_option.select(mode_index)
	vsync_toggle.button_pressed = DisplayServer.window_get_vsync_mode() != DisplayServer.VSYNC_DISABLED


func _change_scene(scene_path: String) -> void:
	var result := get_tree().change_scene_to_file(scene_path)
	if result != OK:
		push_error("Unable to change to scene '%s' (error %d)." % [scene_path, result])
