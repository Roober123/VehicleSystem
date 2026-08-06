extends Node


func _ready() -> void:
	await get_tree().process_frame
	var regression := AudioRegression.new()
	var passed: bool = regression.run()
	get_tree().quit(0 if passed else 1)
