extends Node


func _ready() -> void:
	var regression := DrivetrainRegression.new()
	var passed: bool = regression.run()
	get_tree().quit(0 if passed else 1)
