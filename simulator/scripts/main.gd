extends Node3D

const SENSOR_CHANNELS := 16
const REQUIRED_COLUMNS := [
	"t_us", "x", "y", "theta", "line_conf", "valid_count", "v_target", "omega_target",
	"wheel_v_l", "wheel_v_r", "line_found", "safety_faults", "mode", "pwm_l", "pwm_r"
]
const RL_COLUMNS := [
	"rl_action_speed", "rl_action_curvature", "rl_action_applied", "rl_reward", "rl_terminated", "rl_truncated"
]

@onready var camera: Camera3D = $Camera3D

var car: Node3D
var sensor_nodes: Array[MeshInstance3D] = []
var track_root: Node3D
var trace: Array[Dictionary] = []
var track_amplitude_m := 0.0
var track_wavelength_m := 1.0
var track_line_width_m := 0.02
var track_start_x := -1.0
var track_end_x := 4.0
var playback_time_s := 0.0
var playback_speed := 1.0
var current_index := 0
var is_playing := false
var slider_dragging := false

var status_label: Label
var time_label: Label
var speed_label: Label
var line_label: Label
var sensor_label: Label
var rl_label: Label
var pose_label: Label
var safety_label: Label
var play_button: Button
var time_slider: HSlider
var speed_picker: OptionButton
var open_dialog: FileDialog


func _ready() -> void:
	_create_stage()
	_create_track()
	_create_car()
	_create_hud()
	_update_car({"x": 0.0, "y": 0.0, "theta": 0.0})
	_update_hud({})

	var args := OS.get_cmdline_user_args()
	for index in range(args.size()):
		if args[index] == "--trace" and index + 1 < args.size():
			_load_trace(args[index + 1])
			break
		if args[index].begins_with("--trace="):
			_load_trace(args[index].trim_prefix("--trace="))
			break


func _process(delta: float) -> void:
	if not trace.is_empty() and is_playing:
		playback_time_s = minf(playback_time_s + delta * playback_speed, _duration_s())
		if playback_time_s >= _duration_s():
			is_playing = false
			play_button.text = "Play"
		if not slider_dragging:
			time_slider.value = playback_time_s
		_update_playback()

	if car != null:
		var target := car.global_position + Vector3(0.0, 0.08, -0.45)
		var camera_goal := car.global_position + Vector3(0.0, 2.25, 3.35)
		camera.global_position = camera.global_position.lerp(camera_goal, 1.0 - exp(-4.0 * delta))
		camera.look_at(target, Vector3.UP)


func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	match event.keycode:
		KEY_SPACE:
			_toggle_playback()
			get_viewport().set_input_as_handled()
		KEY_R:
			_reset_playback()
			get_viewport().set_input_as_handled()
		KEY_O:
			open_dialog.popup_centered_ratio(0.7)
			get_viewport().set_input_as_handled()


func _create_stage() -> void:
	var world := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color("101a27")
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color("c7d7e3")
	environment.ambient_light_energy = 0.72
	environment.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	world.environment = environment
	add_child(world)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-53.0, -32.0, 0.0)
	sun.light_color = Color("fff2da")
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	add_child(sun)

	var floor := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(80.0, 80.0)
	floor.mesh = plane
	floor.position.y = -0.025
	floor.material_override = _material(Color("e5e9e7"), 0.0, 0.92)
	add_child(floor)

	var grid := MeshInstance3D.new()
	var grid_mesh := ImmediateMesh.new()
	var grid_material := _material(Color("cdd5d3"), 0.0, 1.0)
	grid_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	grid_mesh.surface_begin(Mesh.PRIMITIVE_LINES, grid_material)
	for coordinate in range(-20, 21):
		var tone := 0.42 if coordinate % 5 == 0 else 0.18
		grid_mesh.surface_set_color(Color(tone, tone, tone, 1.0))
		grid_mesh.surface_add_vertex(Vector3(float(coordinate), -0.018, -20.0))
		grid_mesh.surface_add_vertex(Vector3(float(coordinate), -0.018, 20.0))
		grid_mesh.surface_add_vertex(Vector3(-20.0, -0.018, float(coordinate)))
		grid_mesh.surface_add_vertex(Vector3(20.0, -0.018, float(coordinate)))
	grid_mesh.surface_end()
	grid.mesh = grid_mesh
	add_child(grid)


func _create_track() -> void:
	if is_instance_valid(track_root):
		remove_child(track_root)
		track_root.queue_free()

	track_root = Node3D.new()
	track_root.name = "TrackAndTrajectory"
	add_child(track_root)
	_add_ribbon("TrackShoulder", 0.105, Color("bdc6c4"), 0.001)
	_add_ribbon("TrackLine", track_line_width_m * 0.5, Color("17222a"), 0.004)
	_add_trajectory()


func _add_ribbon(node_name: String, half_width: float, tint: Color, lift: float) -> void:
	var segments := maxi(100, int((track_end_x - track_start_x) / 0.025))
	var vertices := PackedVector3Array()
	var indices := PackedInt32Array()
	for index in range(segments + 1):
		var x := lerpf(track_start_x, track_end_x, float(index) / float(segments))
		var center := _world_point(x, _track_y(x))
		var tangent := (_world_point(x + 0.005, _track_y(x + 0.005)) -
			_world_point(x - 0.005, _track_y(x - 0.005))).normalized()
		var normal := Vector3(-tangent.z, 0.0, tangent.x)
		vertices.append(center + normal * half_width + Vector3.UP * lift)
		vertices.append(center - normal * half_width + Vector3.UP * lift)
		if index < segments:
			var base := index * 2
			indices.append_array(PackedInt32Array([base, base + 2, base + 1, base + 1, base + 2, base + 3]))

	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var instance := MeshInstance3D.new()
	instance.name = node_name
	instance.mesh = mesh
	instance.material_override = _material(tint, 0.0, 1.0)
	track_root.add_child(instance)


func _add_trajectory() -> void:
	if trace.size() < 2:
		return
	var vertices := PackedVector3Array()
	for row in trace:
		vertices.append(_world_point(float(row.x), float(row.y), 0.018))
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINE_STRIP, arrays)
	var material := _material(Color("29bfad"), 0.0, 0.5)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.emission_enabled = true
	material.emission = Color("167d72")
	var instance := MeshInstance3D.new()
	instance.name = "LoggedPath"
	instance.mesh = mesh
	instance.material_override = material
	track_root.add_child(instance)


func _create_car() -> void:
	var vehicle_scene := load("res://assets/nncar_vehicle.glb") as PackedScene
	if vehicle_scene == null:
		push_error("Missing Blender vehicle asset. Run simulator/assets/build_vehicle.py with Blender.")
		return
	car = vehicle_scene.instantiate() as Node3D
	car.name = "DifferentialDriveCar"
	add_child(car)
	sensor_nodes.clear()
	for index in range(SENSOR_CHANNELS):
		var sensor := car.find_child("Sensor_%02d" % index, true, false) as MeshInstance3D
		if sensor == null:
			continue
		sensor.material_override = _material(Color("58d5b7"), 0.0, 0.35)
		sensor_nodes.append(sensor)


func _create_hud() -> void:
	var canvas := CanvasLayer.new()
	add_child(canvas)

	var info_panel := PanelContainer.new()
	info_panel.position = Vector2(18.0, 18.0)
	info_panel.custom_minimum_size = Vector2(312.0, 224.0)
	info_panel.add_theme_stylebox_override("panel", _panel_style())
	canvas.add_child(info_panel)
	var info := VBoxContainer.new()
	info.add_theme_constant_override("separation", 8)
	info_panel.add_child(info)

	var title := Label.new()
	title.text = "NN_CAR  /  SIMULATION"
	title.add_theme_color_override("font_color", Color("55d6bd"))
	title.add_theme_font_size_override("font_size", 18)
	info.add_child(title)
	status_label = _add_hud_label(info, "Load a C simulator CSV to begin.", Color("b9c8d2"))
	_add_hud_label(info, "────────────────────────", Color("52636d"))
	time_label = _add_hud_label(info, "TIME          0.000 s", Color("e8f0f3"))
	speed_label = _add_hud_label(info, "SPEED         —", Color("e8f0f3"))
	line_label = _add_hud_label(info, "LINE          —", Color("e8f0f3"))
	sensor_label = _add_hud_label(info, "SENSORS       —", Color("b9c8d2"))
	rl_label = _add_hud_label(info, "RL            —", Color("9ed8ff"))
	pose_label = _add_hud_label(info, "POSE          —", Color("e8f0f3"))
	safety_label = _add_hud_label(info, "SAFETY        —", Color("8de0bf"))

	var controls := PanelContainer.new()
	controls.anchor_left = 0.5
	controls.anchor_top = 1.0
	controls.anchor_right = 0.5
	controls.anchor_bottom = 1.0
	controls.offset_left = -410.0
	controls.offset_top = -92.0
	controls.offset_right = 410.0
	controls.offset_bottom = -18.0
	controls.add_theme_stylebox_override("panel", _panel_style())
	canvas.add_child(controls)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	controls.add_child(row)

	var open_button := _add_button(row, "Open CSV", Color("1d383d"))
	open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	play_button = _add_button(row, "Play", Color("246a61"))
	play_button.pressed.connect(_toggle_playback)
	var reset_button := _add_button(row, "Reset", Color("344550"))
	reset_button.pressed.connect(_reset_playback)

	speed_picker = OptionButton.new()
	speed_picker.custom_minimum_size = Vector2(92.0, 38.0)
	for option in [0.5, 1.0, 2.0, 4.0]:
		speed_picker.add_item("%s×" % str(option))
	speed_picker.select(1)
	speed_picker.item_selected.connect(_on_speed_selected)
	row.add_child(speed_picker)

	time_slider = HSlider.new()
	time_slider.min_value = 0.0
	time_slider.max_value = 1.0
	time_slider.step = 0.002
	time_slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	time_slider.value_changed.connect(_on_time_changed)
	time_slider.drag_started.connect(func() -> void: slider_dragging = true)
	time_slider.drag_ended.connect(func(_changed: bool) -> void: slider_dragging = false)
	row.add_child(time_slider)

	open_dialog = FileDialog.new()
	open_dialog.title = "Open NN_Car simulation CSV"
	open_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	open_dialog.access = FileDialog.ACCESS_FILESYSTEM
	open_dialog.filters = PackedStringArray(["*.csv ; NN_Car simulation log"])
	open_dialog.file_selected.connect(_load_trace)
	canvas.add_child(open_dialog)


func _add_hud_label(parent: VBoxContainer, text: String, color: Color) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_color_override("font_color", color)
	label.add_theme_font_size_override("font_size", 14)
	parent.add_child(label)
	return label


func _add_button(parent: HBoxContainer, text: String, tint: Color) -> Button:
	var button := Button.new()
	button.text = text
	button.custom_minimum_size = Vector2(94.0, 38.0)
	var style := StyleBoxFlat.new()
	style.bg_color = tint
	style.corner_radius_top_left = 6
	style.corner_radius_top_right = 6
	style.corner_radius_bottom_left = 6
	style.corner_radius_bottom_right = 6
	button.add_theme_stylebox_override("normal", style)
	button.add_theme_color_override("font_color", Color("edf4f2"))
	parent.add_child(button)
	return button


func _panel_style() -> StyleBoxFlat:
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.055, 0.085, 0.105, 0.94)
	style.border_color = Color(0.22, 0.36, 0.4, 0.85)
	style.set_border_width_all(1)
	style.set_corner_radius_all(9)
	style.content_margin_left = 14.0
	style.content_margin_top = 12.0
	style.content_margin_right = 14.0
	style.content_margin_bottom = 12.0
	return style


func _material(tint: Color, metal: float, rough: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = tint
	material.metallic = metal
	material.roughness = rough
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	return material


func _load_trace(path: String) -> void:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		status_label.text = "Could not open CSV (error %d)." % FileAccess.get_open_error()
		return

	var header := file.get_csv_line()
	var column_index: Dictionary = {}
	for index in range(header.size()):
		column_index[header[index]] = index
	for column in REQUIRED_COLUMNS:
		if not column_index.has(column):
			status_label.text = "Unsupported CSV: missing '%s'." % column
			file.close()
			return

	var loaded: Array[Dictionary] = []
	var has_sensor_channels := true
	for index in range(SENSOR_CHANNELS):
		var suffix := "%02d" % index
		if not column_index.has("gray_value_" + suffix) or \
				not column_index.has("gray_exist_" + suffix) or \
				not column_index.has("gray_valid_" + suffix):
			has_sensor_channels = false
			break
	var has_rl_fields := true
	for column in RL_COLUMNS:
		if not column_index.has(column):
			has_rl_fields = false
			break
	while not file.eof_reached():
		var values := file.get_csv_line()
		if values.size() < header.size():
			continue
		var sample: Dictionary = {}
		var valid_row := true
		for column in REQUIRED_COLUMNS:
			var value := values[int(column_index[column])]
			if value.is_empty():
				valid_row = false
				break
			if column == "t_us" or column == "line_found" or column == "valid_count" or \
					column == "safety_faults" or column == "mode":
				sample[column] = value.to_int()
			else:
				sample[column] = value.to_float()
		if valid_row and has_sensor_channels:
			for index in range(SENSOR_CHANNELS):
				var suffix := "%02d" % index
				sample["gray_value_" + suffix] = values[int(column_index["gray_value_" + suffix])].to_float()
				sample["gray_exist_" + suffix] = values[int(column_index["gray_exist_" + suffix])].to_int()
				sample["gray_valid_" + suffix] = values[int(column_index["gray_valid_" + suffix])].to_int()
		if valid_row and has_rl_fields:
			for column in RL_COLUMNS:
				var value := values[int(column_index[column])]
				if column == "rl_action_applied" or column == "rl_terminated" or column == "rl_truncated":
					sample[column] = value.to_int()
				else:
					sample[column] = value.to_float()
		if valid_row:
			loaded.append(sample)
	file.close()

	if loaded.size() < 2:
		status_label.text = "CSV contains fewer than two valid samples."
		return

	trace = loaded
	playback_time_s = 0.0
	current_index = 0
	is_playing = false
	play_button.text = "Play"
	time_slider.max_value = _duration_s()
	time_slider.value = 0.0
	track_start_x = float(trace[0].x) - 1.0
	track_end_x = float(trace[-1].x) + 2.0
	_load_manifest(path)
	_create_track()
	_update_playback()
	status_label.text = "%s samples  ·  %s" % [str(trace.size()), path.get_file()]


func _load_manifest(csv_path: String) -> void:
	track_amplitude_m = 0.0
	track_wavelength_m = 1.0
	track_line_width_m = 0.02
	var manifest_path := csv_path.get_basename() + ".manifest.json"
	if not FileAccess.file_exists(manifest_path):
		return
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(manifest_path))
	if not parsed is Dictionary:
		return
	var scenario: Variant = parsed.get("scenario", {})
	if scenario is Dictionary:
		track_amplitude_m = float(scenario.get("amp_m", 0.0))
		track_wavelength_m = maxf(float(scenario.get("wavelength_m", 1.0)), 0.001)
	var simulation: Variant = parsed.get("simulation", {})
	if simulation is Dictionary:
		track_line_width_m = maxf(float(simulation.get("line_width_m", 0.02)), 0.001)


func _update_playback() -> void:
	if trace.is_empty():
		return
	var sample := _sample_at(playback_time_s)
	_update_car(sample)
	_update_hud(sample)


func _sample_at(time_s: float) -> Dictionary:
	var time_us := time_s * 1.0e6
	while current_index + 1 < trace.size() and float(trace[current_index + 1].t_us) < time_us:
		current_index += 1
	while current_index > 0 and float(trace[current_index].t_us) > time_us:
		current_index -= 1
	var first: Dictionary = trace[current_index]
	if current_index + 1 >= trace.size():
		return first
	var second: Dictionary = trace[current_index + 1]
	var span := float(second.t_us) - float(first.t_us)
	var weight := clampf((time_us - float(first.t_us)) / maxf(span, 1.0), 0.0, 1.0)
	var sample: Dictionary = {}
	for column in REQUIRED_COLUMNS:
		if column == "line_found" or column == "valid_count" or \
				column == "safety_faults" or column == "mode":
			sample[column] = first[column]
		elif column == "theta":
			var angle_delta := wrapf(float(second.theta) - float(first.theta), -PI, PI)
			sample[column] = float(first.theta) + angle_delta * weight
		else:
			sample[column] = lerpf(float(first[column]), float(second[column]), weight)
	if first.has("gray_value_00"):
		for index in range(SENSOR_CHANNELS):
			var suffix := "%02d" % index
			var value_key := "gray_value_" + suffix
			var exist_key := "gray_exist_" + suffix
			var valid_key := "gray_valid_" + suffix
			sample[value_key] = lerpf(float(first[value_key]), float(second[value_key]), weight)
			sample[exist_key] = first[exist_key]
			sample[valid_key] = first[valid_key]
	if first.has("rl_action_speed"):
		for column in ["rl_action_speed", "rl_action_curvature", "rl_reward"]:
			sample[column] = lerpf(float(first[column]), float(second[column]), weight)
		for column in ["rl_action_applied", "rl_terminated", "rl_truncated"]:
			sample[column] = first[column]
	return sample


func _update_car(sample: Dictionary) -> void:
	var sim_x := float(sample.get("x", 0.0))
	var sim_y := float(sample.get("y", 0.0))
	car.position = _world_point(sim_x, sim_y)
	car.rotation.y = float(sample.get("theta", 0.0))
	for index in range(mini(sensor_nodes.size(), SENSOR_CHANNELS)):
		var suffix := "%02d" % index
		var color := Color("29353b")
		if sample.has("gray_value_" + suffix):
			var exists := int(sample["gray_exist_" + suffix]) != 0
			var valid := int(sample["gray_valid_" + suffix]) != 0
			if not exists:
				color = Color("263039")
			elif not valid:
				color = Color("f0a45b")
			else:
				var response := clampf((float(sample["gray_value_" + suffix]) + 1.0) * 0.5, 0.0, 1.0)
				color = Color("303d42").lerp(Color("52d7a8"), response)
		elif int(sample.get("line_found", 0)) != 0:
			color = Color("52d7a8")
		var sensor_material := sensor_nodes[index].material_override as StandardMaterial3D
		if sensor_material != null:
			sensor_material.albedo_color = color
			sensor_material.emission_enabled = true
			sensor_material.emission = color * 0.22


func _update_hud(sample: Dictionary) -> void:
	if trace.is_empty():
		time_label.text = "TIME          0.000 s"
		speed_label.text = "SPEED         —"
		line_label.text = "LINE          —"
		sensor_label.text = "SENSORS       —"
		rl_label.text = "RL            —"
		pose_label.text = "POSE          —"
		safety_label.text = "SAFETY        —"
		return

	var actual_speed := 0.5 * (float(sample.wheel_v_l) + float(sample.wheel_v_r))
	var lateral_error := float(sample.y) - _track_y(float(sample.x))
	time_label.text = "TIME          %0.3f s   /   %0.2f×" % [playback_time_s, playback_speed]
	speed_label.text = "SPEED         %0.3f → %0.3f m/s" % [float(sample.v_target), actual_speed]
	line_label.text = "LINE          conf %0.2f   ·   found %d" % [float(sample.line_conf), int(sample.line_found)]
	var present := 0
	var valid_count := 0
	for index in range(SENSOR_CHANNELS):
		var suffix := "%02d" % index
		if sample.has("gray_exist_" + suffix):
			present += int(sample["gray_exist_" + suffix])
			valid_count += int(sample["gray_valid_" + suffix])
	if present > 0:
		sensor_label.text = "SENSORS       %02d present  ·  %02d valid" % [present, valid_count]
	else:
		sensor_label.text = "SENSORS       values not logged"
	if sample.has("rl_action_speed"):
		var done_text := "TERM" if int(sample.rl_terminated) != 0 else (
			"TIME" if int(sample.rl_truncated) != 0 else "RUN")
		var applied_text := "POLICY" if int(sample.rl_action_applied) != 0 else "RECOVERY"
		rl_label.text = "RL            s %+.2f  κ %+.2f  ·  r %+.2f  ·  %s / %s" % [
			float(sample.rl_action_speed), float(sample.rl_action_curvature),
			float(sample.rl_reward), applied_text, done_text
		]
	else:
		rl_label.text = "RL            telemetry not logged"
	pose_label.text = "POSE          x %0.2f m   ·   e %+.3f m" % [float(sample.x), lateral_error]
	var faults := int(sample.safety_faults)
	if faults == 0:
		safety_label.text = "SAFETY        CLEAR   ·   mode %d" % int(sample.mode)
		safety_label.add_theme_color_override("font_color", Color("8de0bf"))
	else:
		safety_label.text = "SAFETY        FAULT 0x%08x" % faults
		safety_label.add_theme_color_override("font_color", Color("ff927f"))


func _toggle_playback() -> void:
	if trace.is_empty():
		open_dialog.popup_centered_ratio(0.7)
		return
	if playback_time_s >= _duration_s():
		playback_time_s = 0.0
		current_index = 0
		is_playing = true
	else:
		is_playing = not is_playing
	play_button.text = "Pause" if is_playing else "Play"


func _reset_playback() -> void:
	is_playing = false
	playback_time_s = 0.0
	current_index = 0
	play_button.text = "Play"
	if not trace.is_empty():
		time_slider.value = 0.0
		_update_playback()


func _on_time_changed(value: float) -> void:
	if trace.is_empty():
		return
	playback_time_s = value
	_update_playback()


func _on_speed_selected(index: int) -> void:
	playback_speed = [0.5, 1.0, 2.0, 4.0][index]


func _duration_s() -> float:
	if trace.is_empty():
		return 0.0
	return float(trace[-1].t_us) / 1.0e6


func _track_y(x: float) -> float:
	if is_zero_approx(track_amplitude_m):
		return 0.0
	return track_amplitude_m * sin(TAU * x / track_wavelength_m)


func _world_point(sim_x: float, sim_y: float, height: float = 0.0) -> Vector3:
	return Vector3(-sim_y, height, -sim_x)
