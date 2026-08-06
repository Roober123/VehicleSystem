extends Node

const EPS := 0.001

var failures := 0


func _ready() -> void:
	await get_tree().process_frame
	var preset_topology := load("res://Audio/four_cylinder_exhaust_topology.tres") as VehicleAudioTopologyData
	var preset_firing := load("res://Audio/four_cylinder_firing.tres") as VehicleAudioFiringData
	_expect(preset_topology != null, "Four-cylinder topology preset has the expected Resource type")
	_expect(preset_firing != null, "Four-cylinder firing preset has the expected Resource type")
	if preset_topology != null:
		var preset_nodes := preset_topology.nodes
		var preset_guides := preset_topology.guides
		var preset_sources := preset_topology.source_names
		var preset_source_nodes := preset_topology.source_nodes
		_expect(preset_nodes.size() == 9 and preset_guides.size() == 8 and
			preset_sources.size() == 4 and preset_source_nodes.size() == 4 and
			preset_topology.output_node == "tailpipe_open" and
			preset_topology.output_tap_type == 1,
			"Four-cylinder topology preset has 9 nodes, 8 guides, 4 source mappings, and open radiation output")
		if preset_nodes.size() == 9 and preset_guides.size() == 8:
			var collector := preset_nodes[4]
			var nonlinearity_nodes := 0
			for node in preset_nodes:
				if node.nonlinearity > EPS:
					nonlinearity_nodes += 1
			_expect(collector.name == "collector" and collector.nonlinearity > 0.0 and
				nonlinearity_nodes == 1,
				"Four-cylinder preset keeps nonlinearity on the collector junction only")
			var neck := preset_guides[5]
			_expect(preset_guides[4].from == "collector" and
				preset_guides[4].to == "muffler_chamber_1" and
				neck.from == "muffler_chamber_1" and
				neck.to == "muffler_chamber_2" and
				preset_guides[6].from == "muffler_chamber_2" and
				preset_guides[6].to == "output" and
				preset_guides[4].area > neck.area and
				preset_guides[6].area > neck.area,
				"Four-cylinder preset has a clear chamber-to-neck-to-chamber area path")
			_expect(is_equal_approx(preset_guides[7].area, 0.0028),
				"Four-cylinder preset preserves the authored 0.0028 tailpipe area")
			_expect(preset_guides[0].high_frequency_loss_per_meter > 0.0 and
				preset_guides[4].high_frequency_loss_per_meter > 0.0 and
				preset_guides[5].high_frequency_loss_per_meter > 0.0,
				"Four-cylinder guides preserve authored HF-loss fields")
			_expect(preset_topology.source_gains.size() == 4 and
				preset_topology.source_gains[0] == 1.0 and
				preset_topology.source_gains[3] == 1.0,
				"Four-cylinder topology preserves authored source gains")
	if preset_firing != null:
		var preset_phases := preset_firing.phases_degrees
		var preset_firing_sources := preset_firing.source_names
		_expect(preset_phases.size() == 4 and preset_firing_sources.size() == 4 and
			preset_phases[0] == 0.0 and preset_phases[1] == 180.0 and
			preset_phases[2] == 360.0 and preset_phases[3] == 540.0 and
			preset_firing_sources[0] == "cylinder_1" and
			preset_firing_sources[1] == "cylinder_3" and
			preset_firing_sources[2] == "cylinder_4" and
			preset_firing_sources[3] == "cylinder_2",
			"Four-cylinder firing preset has 1-3-4-2 phases and source order")
	var vehicle := _make_vehicle()
	var audio := vehicle.get_node("VehicleAudio") as VehicleAudio
	_expect(audio != null, "VehicleAudio fixture is attached")
	if audio == null:
		get_tree().quit(1)
		return

	_expect(audio.is_audio_enabled(), "VehicleAudio enables with valid resources")
	_expect(audio.stream != null, "VehicleAudio installs an AudioStreamGenerator")
	_expect(audio.is_playing(), "VehicleAudio starts generator playback")
	var playback := audio.get_stream_playback() as AudioStreamGeneratorPlayback
	_expect(playback != null, "VehicleAudio exposes generator playback")
	if playback != null:
		# Allow a bounded number of idle frames for VehicleAudio to fill its
		# staging queue without relying on real-time audio duration.
		for _i in range(8):
			await get_tree().process_frame
		_expect(playback.get_frames_available() > 0,
			"VehicleAudio playback receives bounded buffered frames")
		_expect(playback.get_skips() == 0,
			"VehicleAudio playback reports no buffer skips")

	var restart_ok: bool = vehicle.restart()
	_expect(restart_ok, "Vehicle restart succeeds while audio is attached")
	await get_tree().process_frame
	_expect(audio.is_audio_enabled() and audio.get_stream_playback() != null,
		"VehicleAudio remains enabled after Vehicle restart")

	# Re-entry is public-node behavior: replacing the child with the same
	# authored resources must compile a fresh bounded runtime.
	playback = null
	audio.stop()
	audio.queue_free()
	audio = null
	await get_tree().process_frame
	var replacement := _make_audio(vehicle)
	if preset_topology != null and preset_firing != null:
		replacement.topology = preset_topology
		replacement.firing = preset_firing
	vehicle.add_child(replacement)
	await get_tree().process_frame
	_expect(replacement.is_audio_enabled() and replacement.get_stream_playback() != null,
		"VehicleAudio recompiles successfully on node re-entry")

	replacement.stop()
	replacement = null
	vehicle.queue_free()
	vehicle = null
	await get_tree().process_frame
	await get_tree().process_frame
	await get_tree().create_timer(0.25).timeout
	print("[vehicle-audio-smoke] DONE failures=", failures)
	get_tree().quit(0 if failures == 0 else 1)


func _make_vehicle() -> Vehicle:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 1.0))
	curve.add_point(Vector2(1.0, 1.0))
	var engine_data := VehicleEngineData.new()
	engine_data.torque_curve = curve
	engine_data.idle_rpm = 850.0
	engine_data.redline_rpm = 5000.0
	engine_data.inertia = 1.0
	engine_data.max_torque = 250.0
	engine_data.engine_drag = 0.02
	engine_data.engine_braking = 0.4

	var gearbox_data := GearboxData.new()
	gearbox_data.gear_ratios = PackedFloat64Array([2.0, 1.0])
	gearbox_data.final_drive = 2.5
	gearbox_data.reverse_ratio = -2.0
	gearbox_data.clutch_max_torque = 800.0
	gearbox_data.shift_time = 0.2
	gearbox_data.auto_mode = false
	gearbox_data.driveshaft_drag = 0.0

	var tire_curve := Curve.new()
	tire_curve.add_point(Vector2(0.0, 0.0))
	tire_curve.add_point(Vector2(1.0, 1.0))
	var tire_data := TireData.new()
	tire_data.forward_friction_curve = tire_curve
	tire_data.lateral_friction_curve = tire_curve
	tire_data.radius = 0.3
	tire_data.drag = 0.0

	var config := VehicleConfig.new()
	config.engine_data = engine_data
	config.gearbox_data = gearbox_data
	config.suspension_data = SuspensionData.new()
	config.aero_data = VehicleAerodynamicsData.new()

	var vehicle := Vehicle.new()
	vehicle.config = config
	vehicle.gravity_scale = 0.0
	vehicle.mass = 1000.0
	var axle := Axle.new()
	axle.drive_share = 1.0
	axle.tire_data = tire_data
	axle.differential_data = DifferentialData.new()
	axle.add_child(Wheel.new())
	axle.add_child(Wheel.new())
	vehicle.add_child(axle)
	vehicle.add_child(_make_audio(vehicle))
	add_child(vehicle)
	return vehicle


func _make_audio(vehicle: Vehicle) -> VehicleAudio:
	var topology := VehicleAudioTopologyData.new()
	var nodes: Array[VehicleAudioNodeData] = []
	for i in range(4):
		var runner := VehicleAudioNodeData.new()
		runner.name = "runner_%d" % (i + 1)
		nodes.append(runner)
	var collector := VehicleAudioNodeData.new()
	collector.name = "collector"
	collector.nonlinearity = 0.35
	nodes.append(collector)
	var chamber_1 := VehicleAudioNodeData.new()
	chamber_1.name = "muffler_chamber_1"
	nodes.append(chamber_1)
	var chamber_2 := VehicleAudioNodeData.new()
	chamber_2.name = "muffler_chamber_2"
	nodes.append(chamber_2)
	var output := VehicleAudioNodeData.new()
	output.name = "output"
	nodes.append(output)
	var open_end := VehicleAudioNodeData.new()
	open_end.name = "open_end"
	open_end.node_type = VehicleAudioNodeData.REFLECTION_BOUNDARY
	open_end.reflection = -1.0
	nodes.append(open_end)
	topology.nodes = nodes

	var guides: Array[VehicleAudioGuideData] = []
	for i in range(4):
		var runner_guide := VehicleAudioGuideData.new()
		runner_guide.name = "runner_guide_%d" % (i + 1)
		runner_guide.from = "runner_%d" % (i + 1)
		runner_guide.to = "collector"
		runner_guide.length_meters = 0.01
		runner_guide.area = 1.0
		runner_guide.loss_per_meter = 0.0
		guides.append(runner_guide)
	var collector_guide := VehicleAudioGuideData.new()
	collector_guide.name = "collector_guide"
	collector_guide.from = "collector"
	collector_guide.to = "muffler_chamber_1"
	collector_guide.length_meters = 0.35
	collector_guide.area = 0.0065
	guides.append(collector_guide)
	var neck_guide := VehicleAudioGuideData.new()
	neck_guide.name = "chamber_1_to_chamber_2"
	neck_guide.from = "muffler_chamber_1"
	neck_guide.to = "muffler_chamber_2"
	neck_guide.length_meters = 0.12
	neck_guide.area = 0.0025
	guides.append(neck_guide)
	var chamber_guide := VehicleAudioGuideData.new()
	chamber_guide.name = "chamber_2_to_output"
	chamber_guide.from = "muffler_chamber_2"
	chamber_guide.to = "output"
	chamber_guide.length_meters = 0.55
	chamber_guide.area = 0.0060
	guides.append(chamber_guide)
	var tailpipe_guide := VehicleAudioGuideData.new()
	tailpipe_guide.name = "tailpipe_guide"
	tailpipe_guide.from = "output"
	tailpipe_guide.to = "open_end"
	tailpipe_guide.length_meters = 1.8
	tailpipe_guide.area = 0.0028
	guides.append(tailpipe_guide)
	topology.guides = guides
	topology.source_names = PackedStringArray([
		"cylinder_1", "cylinder_2", "cylinder_3", "cylinder_4"])
	topology.source_nodes = PackedStringArray([
		"runner_1", "runner_2", "runner_3", "runner_4"])
	topology.source_gains = PackedFloat32Array([1.0, 1.0, 1.0, 1.0])
	topology.output_node = "open_end"
	topology.output_tap_type = 1
	topology.output_gain = 1.0

	var firing := VehicleAudioFiringData.new()
	firing.configure_default_four_stroke()
	var audio := VehicleAudio.new()
	audio.name = "VehicleAudio"
	audio.target = vehicle
	audio.topology = topology
	audio.firing = firing
	return audio


func _expect(condition: bool, label: String) -> void:
	if condition:
		return
	failures += 1
	push_error("[vehicle-audio-smoke] FAIL: %s" % label)
