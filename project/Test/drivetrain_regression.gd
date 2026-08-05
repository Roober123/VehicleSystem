extends Node


func _ready() -> void:
	# Let the fixture scene finish entering the tree before the native runner
	# mounts temporary Vehicle nodes for COM marker coverage.
	await get_tree().process_frame
	var regression := DrivetrainRegression.new()
	var passed: bool = regression.run()
	get_tree().quit(0 if passed else 1)
