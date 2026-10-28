extends Node3D

const SENSOR_CHANNELS := 16
const REQUIRED_COLUMNS := [
	"t_us", "x", "y", "theta", "line_conf", "valid_count", "v_target", "omega_target",
	"wheel_v_l", "wheel_v_r", "line_found", "safety_faults", "mode", "pwm_l", "pwm_r"
]
const RL_COLUMNS := [
	"rl_action_speed", "rl_action_curvature", "rl_action_applied", "rl_reward", "rl_terminated", "rl_truncated"
]
const LOG_COLUMNS := [
	"t_us", "mode", "line_conf", "line_centroid_m", "line_found", "valid_count",
	"v_target", "omega_target", "v_max_eff", "lambda", "kappa_target",
	"wheel_v_l_target", "wheel_v_r_target", "wheel_v_l", "wheel_v_r",
	"u_l", "u_r", "pwm_l", "pwm_r", "v_rail", "v_rail_valid",
	"safety_faults", "safety_latched", "safety_derate", "safety_ramp", "safety_stop",
	"x", "y", "theta", "recovery_ms"
]
const DEFAULT_LIVE_PORT := 37621
const MAX_LIVE_STEPS_PER_FRAME := 4

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
var line_dt_s := 0.01
var motor_dt_s := 0.002
var profile_name := "mg513x_12v_4s_default"
var profile_version := "1.0.0"
var run_scenario: Dictionary = {}
var playback_time_s := 0.0
var playback_speed := 1.0
var current_index := 0
var is_playing := false
var slider_dragging := false
var camera_target := Vector3.ZERO
var camera_distance := 4.2
var camera_yaw := 0.0
var camera_pitch := deg_to_rad(34.0)
var camera_follow := true
var orbit_dragging := false
var pan_dragging := false
var dashboard: Control
var plot_dashboard: Control
var plot_window: Window
var plot_zoom_label: Label
var transport_panel: PanelContainer
var live_button: Button
var save_dialog: FileDialog
var live_peer: StreamPeerTCP
var live_port := DEFAULT_LIVE_PORT
var live_rx_buffer := ""
var live_request_pending := false
var live_pending_kind := ""
var live_outstanding_steps := 0
var live_reset_requested := false
var live_mode := false
var live_ready := false
var live_running := false
var live_accumulator := 0.0
var live_control_dt_s := 0.01
var live_duration_s := 60.0
var live_profile_verified := false
var live_scenario: Dictionary = {}

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
	get_tree().root.gui_embed_subwindows = false
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
		if args[index] == "--live-port" and index + 1 < args.size():
			_connect_live(args[index + 1].to_int())
			break
		if args[index].begins_with("--live-port="):
			_connect_live(args[index].trim_prefix("--live-port=").to_int())
			break


func _process(delta: float) -> void:
	if live_mode:
		_process_live(delta)
	elif not trace.is_empty() and is_playing:
		playback_time_s = minf(playback_time_s + delta * playback_speed, _duration_s())
		if playback_time_s >= _duration_s():
			is_playing = false
			play_button.text = "Play"
		if not slider_dragging:
			time_slider.value = playback_time_s
		_update_playback()

	if car != null:
		_update_camera(delta)


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
		KEY_F:
			_toggle_camera_follow()
			get_viewport().set_input_as_handled()
		KEY_V:
			_frame_trace()
			get_viewport().set_input_as_handled()
		KEY_0, KEY_KP_0:
			_reset_camera()
			get_viewport().set_input_as_handled()
		KEY_F11:
			_toggle_fullscreen()
			get_viewport().set_input_as_handled()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			orbit_dragging = event.pressed
			get_viewport().set_input_as_handled()
		elif event.button_index == MOUSE_BUTTON_MIDDLE:
			pan_dragging = event.pressed
			get_viewport().set_input_as_handled()
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
			camera_distance = maxf(1.5, camera_distance * 0.88)
			get_viewport().set_input_as_handled()
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			camera_distance = minf(60.0, camera_distance * 1.14)
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseMotion:
		if orbit_dragging:
			camera_yaw -= event.relative.x * 0.008
			camera_pitch = clampf(camera_pitch + event.relative.y * 0.006,
				deg_to_rad(8.0), deg_to_rad(82.0))
			get_viewport().set_input_as_handled()
		elif pan_dragging:
			var right := Vector3(cos(camera_yaw), 0.0, -sin(camera_yaw))
			var toward_camera := Vector3(sin(camera_yaw), 0.0, cos(camera_yaw))
			var scale := camera_distance * 0.0015
			camera_target += -right * event.relative.x * scale
			camera_target += toward_camera * event.relative.y * scale
			camera_follow = false
			get_viewport().set_input_as_handled()


func _connect_live(port: int = DEFAULT_LIVE_PORT) -> void:
	if live_peer != null:
		return
	live_port = port if port > 0 else DEFAULT_LIVE_PORT
	live_peer = StreamPeerTCP.new()
	var error := live_peer.connect_to_host("127.0.0.1", live_port)
	if error != OK:
		live_peer = null
		status_label.text = "Live connect failed (%s:%d)." % ["127.0.0.1", live_port]
		return
	live_mode = true
	live_ready = false
	live_running = false
	live_rx_buffer = ""
	live_request_pending = false
	live_pending_kind = ""
	live_outstanding_steps = 0
	live_reset_requested = false
	live_accumulator = 0.0
	trace.clear()
	playback_time_s = 0.0
	play_button.text = "Connecting"
	live_button.text = "Disconnect"
	status_label.text = "Connecting to C simulation at 127.0.0.1:%d…" % live_port


func _disconnect_live(send_close: bool) -> void:
	if live_peer != null:
		if send_close and live_peer.get_status() == StreamPeerTCP.STATUS_CONNECTED:
			live_peer.put_data("CLOSE\n".to_utf8_buffer())
		live_peer.disconnect_from_host()
	live_peer = null
	live_mode = false
	live_ready = false
	live_running = false
	live_request_pending = false
	live_pending_kind = ""
	live_outstanding_steps = 0
	if is_instance_valid(live_button):
		live_button.text = "Live"
	if is_instance_valid(play_button):
		play_button.text = "Play"


func _on_live_button() -> void:
	if live_mode:
		_disconnect_live(true)
		status_label.text = "Live link closed."
	else:
		_connect_live(live_port)


func _process_live(delta: float) -> void:
	if live_peer == null:
		return
	live_peer.poll()
	if live_peer.get_status() == StreamPeerTCP.STATUS_ERROR or \
			live_peer.get_status() == StreamPeerTCP.STATUS_NONE:
		status_label.text = "C simulation disconnected."
		_disconnect_live(false)
		return
	if live_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return

	var available := live_peer.get_available_bytes()
	if available > 0:
		var packet := live_peer.get_data(available)
		if packet[0] != OK:
			status_label.text = "Live socket read failed."
			_disconnect_live(false)
			return
		live_rx_buffer += (packet[1] as PackedByteArray).get_string_from_utf8()
	while live_rx_buffer.contains("\n"):
		var newline := live_rx_buffer.find("\n")
		var line := live_rx_buffer.substr(0, newline).strip_edges()
		live_rx_buffer = live_rx_buffer.substr(newline + 1)
		if line.is_empty():
			continue
		var parsed: Variant = JSON.parse_string(line)
		if parsed is Dictionary:
			_handle_live_message(parsed)

	if live_running:
		live_accumulator += delta * playback_speed
		var sent_this_frame := 0
		while live_accumulator >= live_control_dt_s and \
				live_outstanding_steps < MAX_LIVE_STEPS_PER_FRAME and \
				sent_this_frame < MAX_LIVE_STEPS_PER_FRAME:
			if _send_live_command("STEP", "step"):
				live_accumulator -= live_control_dt_s
				live_outstanding_steps += 1
				sent_this_frame += 1
			else:
				break

	if live_reset_requested and live_outstanding_steps == 0 and not live_request_pending:
		live_reset_requested = false
		_send_live_command("RESET", "reset")


func _send_live_command(command: String, kind: String) -> bool:
	if live_peer == null or live_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return false
	var error := live_peer.put_data((command + "\n").to_utf8_buffer())
	if error != OK:
		status_label.text = "Live socket write failed (%d)." % error
		return false
	if kind == "reset":
		live_request_pending = true
		live_pending_kind = kind
	return true


func _handle_live_message(message: Dictionary) -> void:
	var event := str(message.get("event", ""))
	if event == "ready":
		var scenario: Variant = message.get("scenario", {})
		if scenario is Dictionary:
			live_scenario = scenario.duplicate(true)
			run_scenario = scenario.duplicate(true)
			track_amplitude_m = float(scenario.get("amp_m", 0.0))
			track_wavelength_m = maxf(float(scenario.get("wavelength_m", 1.0)), 0.001)
			live_duration_s = float(scenario.get("duration_s", 60.0))
			track_start_x = -1.0
			track_end_x = maxf(8.0, float(scenario.get("v_desired_mps", 0.8)) * live_duration_s * 1.2 + 3.0)
			_create_track()
		live_profile_verified = bool(message.get("profile_verified", false))
		line_dt_s = float(message.get("line_dt_s", 0.01))
		motor_dt_s = float(message.get("motor_dt_s", 0.002))
		live_control_dt_s = maxf(line_dt_s, 0.001)
		profile_name = "mg513x_12v_4s_default"
		profile_version = "1.0.0"
		live_ready = true
		live_reset_requested = false
		_send_live_command("RESET", "reset")
		status_label.text = "C live server ready · fixed step %0.1f ms" % (live_control_dt_s * 1000.0)
	elif event == "reset":
		live_request_pending = false
		live_pending_kind = ""
		live_ready = true
		live_outstanding_steps = 0
		live_accumulator = 0.0
		trace.clear()
		playback_time_s = 0.0
		current_index = 0
		live_running = true
		play_button.text = "Pause"
		time_slider.max_value = live_duration_s
		time_slider.value = 0.0
		_update_car({"x": 0.0, "y": 0.0, "theta": 0.0})
		_set_dashboard_trace(trace, 0.0, live_duration_s)
		_fit_plot_views()
		status_label.text = "LIVE · C fixed-step · profile_verified=%s" % str(live_profile_verified)
	elif event == "state":
		if live_outstanding_steps > 0:
			live_outstanding_steps -= 1
		var sample := _live_state_to_row(message)
		if not trace.is_empty():
			var previous: Dictionary = trace[-1]
			var elapsed := (float(sample.t_us) - float(previous.t_us)) * 1.0e-6
			if elapsed > 0.0:
				sample["omega_actual_radps"] = wrapf(float(sample.theta) -
					float(previous.theta), -PI, PI) / elapsed
		trace.append(sample)
		playback_time_s = float(sample.t_us) * 1.0e-6
		current_index = trace.size() - 1
		if not slider_dragging:
			time_slider.set_value_no_signal(minf(playback_time_s, live_duration_s))
		_update_car(sample)
		_update_hud(sample)
		_set_dashboard_trace(trace, playback_time_s, maxf(playback_time_s, live_control_dt_s))
		if trace.size() % 10 == 0:
			_update_live_trajectory()
		if bool(sample.get("done", false)):
			live_running = false
			play_button.text = "Resume"
			status_label.text = "Live run complete · %d samples" % trace.size()
	elif event == "done":
		live_running = false
		live_outstanding_steps = 0
		play_button.text = "Resume"
	elif event == "error":
		live_request_pending = false
		live_pending_kind = ""
		live_running = false
		status_label.text = "C simulator: %s" % str(message.get("message", "protocol error"))


func _live_state_to_row(message: Dictionary) -> Dictionary:
	var sample := message.duplicate(true)
	var values: Array = message.get("gray_value", [])
	var exists: Array = message.get("gray_exist", [])
	var valid: Array = message.get("gray_valid", [])
	var valid_count := 0
	var present_count := 0
	for channel in range(SENSOR_CHANNELS):
		var suffix := "%02d" % channel
		sample["gray_value_" + suffix] = float(values[channel]) if channel < values.size() else 0.0
		sample["gray_exist_" + suffix] = int(exists[channel]) if channel < exists.size() else 0
		sample["gray_valid_" + suffix] = int(valid[channel]) if channel < valid.size() else 0
		present_count += int(sample["gray_exist_" + suffix])
		valid_count += int(sample["gray_valid_" + suffix])
	sample["sensor_count"] = int(message.get("sensor_count", SENSOR_CHANNELS))
	sample["lateral_error_mm"] = float(message.get("lateral_error_m", 0.0)) * 1000.0
	sample["heading_error_deg"] = rad_to_deg(float(message.get("heading_error_rad", 0.0)))
	sample["speed_actual_mps"] = 0.5 * (float(sample.get("wheel_v_l", 0.0)) +
		float(sample.get("wheel_v_r", 0.0)))
	sample["omega_actual_radps"] = 0.0
	sample["sensor_valid_ratio"] = float(valid_count) / maxf(float(present_count), 1.0)
	return sample


func _update_live_trajectory() -> void:
	if not is_instance_valid(track_root):
		return
	var old_path := track_root.get_node_or_null("LoggedPath")
	if old_path != null:
		track_root.remove_child(old_path)
		old_path.queue_free()
	_add_trajectory()


func _save_trace(path: String) -> void:
	if trace.is_empty():
		status_label.text = "No samples to save."
		return
	var csv_path := path if path.get_extension().to_lower() == "csv" else path + ".csv"
	var columns := LOG_COLUMNS.duplicate()
	for channel in range(SENSOR_CHANNELS):
		var suffix := "%02d" % channel
		columns.append("gray_value_" + suffix)
		columns.append("gray_exist_" + suffix)
		columns.append("gray_valid_" + suffix)
	var include_rl := trace[0].has("rl_action_speed")
	if include_rl:
		columns.append_array(RL_COLUMNS)
	var file := FileAccess.open(csv_path, FileAccess.WRITE)
	if file == null:
		status_label.text = "Could not save CSV (error %d)." % FileAccess.get_open_error()
		return
	file.store_line(",".join(columns))
	var integer_columns := ["t_us", "mode", "line_found", "valid_count", "v_rail_valid",
		"safety_faults", "safety_latched", "safety_stop", "rl_action_applied",
		"rl_terminated", "rl_truncated"]
	for sample in trace:
		var values := PackedStringArray()
		for column in columns:
			var value: Variant = sample.get(column, 0)
			if column in integer_columns or column.begins_with("gray_exist_") or column.begins_with("gray_valid_"):
				values.append(str(int(value)))
			else:
				values.append("%.7f" % float(value))
		file.store_line(",".join(values))
	file.close()
	var scenario := run_scenario.duplicate(true)
	if live_mode:
		scenario = live_scenario.duplicate(true)
	var manifest := {
		"schema_version": "1.1.0", "generator": "nncar_godot_viewer",
		"profile": profile_name, "profile_version": profile_version,
		"profile_verified": live_profile_verified,
		"seed": int(scenario.get("seed", 0)),
		"tick_dt_s": motor_dt_s, "line_dt_s": line_dt_s,
		"scenario": {
			"duration_s": float(scenario.get("duration_s", _duration_s())),
			"v_desired_mps": float(scenario.get("v_desired_mps", 0.0)),
			"offset_m": float(scenario.get("offset_m", 0.0)),
			"heading_deg": float(scenario.get("heading_deg", 0.0)),
			"amp_m": float(scenario.get("amp_m", track_amplitude_m)),
			"wavelength_m": float(scenario.get("wavelength_m", track_wavelength_m)),
			"rail_v": float(scenario.get("rail_v", 12.0))
		}
	}
	var manifest_file := FileAccess.open(csv_path.get_basename() + ".manifest.json", FileAccess.WRITE)
	if manifest_file != null:
		manifest_file.store_string(JSON.stringify(manifest, "\t"))
		manifest_file.close()
	status_label.text = "Saved %d samples · %s" % [trace.size(), csv_path.get_file()]


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
	var indices := PackedInt32Array()
	for index in range(trace.size()):
		var row: Dictionary = trace[index]
		var point := _world_point(float(row.x), float(row.y), 0.026)
		var before: Dictionary = trace[maxi(index - 1, 0)]
		var after: Dictionary = trace[mini(index + 1, trace.size() - 1)]
		var tangent := (_world_point(float(after.x), float(after.y)) -
			_world_point(float(before.x), float(before.y))).normalized()
		var normal := Vector3(-tangent.z, 0.0, tangent.x).normalized()
		vertices.append(point + normal * 0.007)
		vertices.append(point - normal * 0.007)
		if index < trace.size() - 1:
			var base := index * 2
			indices.append_array(PackedInt32Array([base, base + 2, base + 1,
				base + 1, base + 2, base + 3]))
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var material := _material(Color("29bfad"), 0.0, 0.5)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.emission_enabled = true
	material.emission = Color("167d72") * 0.7
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
	dashboard = load("res://scripts/telemetry_dashboard.gd").new() as Control
	dashboard.anchor_left = 0.0
	dashboard.anchor_top = 1.0
	dashboard.anchor_right = 1.0
	dashboard.anchor_bottom = 1.0
	dashboard.offset_left = 12.0
	dashboard.offset_top = -302.0
	dashboard.offset_right = -12.0
	dashboard.offset_bottom = -10.0
	dashboard.mouse_filter = Control.MOUSE_FILTER_STOP
	canvas.add_child(dashboard)
	_create_plot_window()

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
	var camera_help := _add_hud_label(info,
		"VIEW  RMB orbit · wheel zoom · MMB pan\nF follow/free · V fit track · 0 reset\nF11 fullscreen",
		Color("9db3bd"))
	camera_help.add_theme_font_size_override("font_size", 12)

	transport_panel = PanelContainer.new()
	transport_panel.anchor_left = 0.5
	transport_panel.anchor_top = 1.0
	transport_panel.anchor_right = 0.5
	transport_panel.anchor_bottom = 1.0
	transport_panel.offset_left = -570.0
	transport_panel.offset_top = -378.0
	transport_panel.offset_right = 570.0
	transport_panel.offset_bottom = -306.0
	transport_panel.add_theme_stylebox_override("panel", _panel_style())
	canvas.add_child(transport_panel)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	transport_panel.add_child(row)

	var open_button := _add_button(row, "Open CSV", Color("1d383d"))
	open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	play_button = _add_button(row, "Play", Color("246a61"))
	play_button.pressed.connect(_toggle_playback)
	var reset_button := _add_button(row, "Reset", Color("344550"))
	reset_button.pressed.connect(_reset_playback)
	live_button = _add_button(row, "Live", Color("375f79"))
	live_button.pressed.connect(_on_live_button)
	var save_button := _add_button(row, "Save", Color("344550"))
	save_button.tooltip_text = "Save the current run as CSV + manifest"
	save_button.pressed.connect(func() -> void: save_dialog.popup_centered_ratio(0.7))
	var chart_window_button := _add_button(row, "Charts ↗", Color("375f79"))
	chart_window_button.tooltip_text = "Open synchronized telemetry in a separate window"
	chart_window_button.pressed.connect(_open_plot_window)

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

	save_dialog = FileDialog.new()
	save_dialog.title = "Save NN_Car telemetry run"
	save_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	save_dialog.access = FileDialog.ACCESS_FILESYSTEM
	save_dialog.filters = PackedStringArray(["*.csv ; NN_Car simulation log"])
	save_dialog.file_selected.connect(_save_trace)
	canvas.add_child(save_dialog)


func _create_plot_window() -> void:
	plot_window = Window.new()
	plot_window.title = "NN_Car  /  TELEMETRY"
	plot_window.size = Vector2i(1280, 800)
	plot_window.min_size = Vector2i(900, 580)
	plot_window.visible = false
	plot_window.close_requested.connect(_on_plot_window_close_requested)
	add_child(plot_window)

	var layout := VBoxContainer.new()
	layout.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	layout.add_theme_constant_override("separation", 8)
	plot_window.add_child(layout)

	var toolbar_panel := PanelContainer.new()
	toolbar_panel.add_theme_stylebox_override("panel", _panel_style())
	layout.add_child(toolbar_panel)
	var toolbar := HBoxContainer.new()
	toolbar.add_theme_constant_override("separation", 8)
	toolbar_panel.add_child(toolbar)
	var heading := Label.new()
	heading.text = "TELEMETRY  /  SYNCHRONIZED TO RUN CURSOR"
	heading.add_theme_color_override("font_color", Color("54d6bc"))
	heading.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	toolbar.add_child(heading)
	var zoom_out_button := _add_button(toolbar, "−", Color("344550"))
	zoom_out_button.tooltip_text = "Zoom out in time"
	zoom_out_button.pressed.connect(func() -> void: plot_dashboard.zoom_out())
	var zoom_in_button := _add_button(toolbar, "+", Color("246a61"))
	zoom_in_button.tooltip_text = "Zoom in in time"
	zoom_in_button.pressed.connect(func() -> void: plot_dashboard.zoom_in())
	var fit_button := _add_button(toolbar, "Fit", Color("344550"))
	fit_button.tooltip_text = "Fit the full run"
	fit_button.pressed.connect(_fit_plot_views)
	plot_zoom_label = Label.new()
	plot_zoom_label.text = "FIT"
	plot_zoom_label.custom_minimum_size.x = 58.0
	plot_zoom_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	plot_zoom_label.add_theme_color_override("font_color", Color("d2e0e4"))
	toolbar.add_child(plot_zoom_label)
	var hint := Label.new()
	hint.text = "Wheel: zoom  ·  middle-drag: pan"
	hint.add_theme_color_override("font_color", Color("91a6ae"))
	toolbar.add_child(hint)

	plot_dashboard = load("res://scripts/telemetry_dashboard.gd").new() as Control
	plot_dashboard.mouse_filter = Control.MOUSE_FILTER_STOP
	plot_dashboard.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	plot_dashboard.size_flags_vertical = Control.SIZE_EXPAND_FILL
	layout.add_child(plot_dashboard)
	dashboard.view_changed.connect(func() -> void: _sync_plot_views(dashboard, plot_dashboard))
	plot_dashboard.view_changed.connect(func() -> void: _sync_plot_views(plot_dashboard, dashboard))


func _sync_plot_views(source: Control, target: Control) -> void:
	if not is_instance_valid(source) or not is_instance_valid(target):
		return
	target.apply_view_state(source.get_view_state())
	_update_plot_zoom_label()


func _update_plot_zoom_label() -> void:
	if is_instance_valid(plot_zoom_label) and is_instance_valid(plot_dashboard):
		plot_zoom_label.text = plot_dashboard.get_zoom_label()


func _set_dashboard_trace(rows: Array[Dictionary], cursor_s: float, duration_s: float) -> void:
	dashboard.set_trace(rows, cursor_s, duration_s)
	plot_dashboard.set_trace(rows, cursor_s, duration_s)
	_update_plot_zoom_label()


func _set_dashboard_cursor(cursor_s: float) -> void:
	dashboard.set_cursor(cursor_s)
	plot_dashboard.set_cursor(cursor_s)


func _fit_plot_views() -> void:
	dashboard.fit_trace()
	plot_dashboard.fit_trace()
	_update_plot_zoom_label()


func _open_plot_window() -> void:
	if is_instance_valid(plot_window):
		plot_window.popup_centered(Vector2i(1280, 800))
		plot_window.grab_focus()


func _on_plot_window_close_requested() -> void:
	plot_window.hide()


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
	_disconnect_live(true)
	live_mode = false
	live_running = false
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
	_prepare_plot_fields()
	_create_track()
	_frame_trace()
	_set_dashboard_trace(trace, playback_time_s, _duration_s())
	_fit_plot_views()
	_update_playback()
	status_label.text = "%s samples  ·  %s" % [str(trace.size()), path.get_file()]


func _load_manifest(csv_path: String) -> void:
	track_amplitude_m = 0.0
	track_wavelength_m = 1.0
	track_line_width_m = 0.02
	line_dt_s = 0.01
	motor_dt_s = 0.002
	live_profile_verified = false
	run_scenario = {
		"duration_s": _duration_s(), "v_desired_mps": 0.0, "offset_m": 0.0,
		"heading_deg": 0.0, "amp_m": 0.0, "wavelength_m": 1.0, "rail_v": 12.0, "seed": 0
	}
	var manifest_path := csv_path.get_basename() + ".manifest.json"
	if not FileAccess.file_exists(manifest_path):
		return
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(manifest_path))
	if not parsed is Dictionary:
		return
	var scenario: Variant = parsed.get("scenario", {})
	if scenario is Dictionary:
		run_scenario.merge(scenario, true)
		track_amplitude_m = float(scenario.get("amp_m", 0.0))
		track_wavelength_m = maxf(float(scenario.get("wavelength_m", 1.0)), 0.001)
	var simulation: Variant = parsed.get("simulation", {})
	if simulation is Dictionary:
		track_line_width_m = maxf(float(simulation.get("line_width_m", 0.02)), 0.001)
	line_dt_s = float(parsed.get("line_dt_s", line_dt_s))
	motor_dt_s = float(parsed.get("tick_dt_s", motor_dt_s))
	profile_name = str(parsed.get("profile", profile_name))
	profile_version = str(parsed.get("profile_version", profile_version))
	live_profile_verified = bool(parsed.get("profile_verified", false))


func _prepare_plot_fields() -> void:
	var previous_theta := 0.0
	var previous_time_us := 0.0
	for index in range(trace.size()):
		var sample: Dictionary = trace[index]
		var time_us := float(sample.get("t_us", 0))
		var x := float(sample.get("x", 0.0))
		var theta := float(sample.get("theta", 0.0))
		var lateral := float(sample.get("lateral_error_m",
			float(sample.get("y", 0.0)) - _track_y(x)))
		var heading := float(sample.get("heading_error_rad",
			wrapf(theta - _track_heading(x), -PI, PI)))
		var actual_speed := 0.5 * (float(sample.get("wheel_v_l", 0.0)) +
			float(sample.get("wheel_v_r", 0.0)))
		var actual_omega := 0.0
		if index > 0:
			var elapsed := (time_us - previous_time_us) * 1.0e-6
			if elapsed > 0.0:
				actual_omega = wrapf(theta - previous_theta, -PI, PI) / elapsed
		var sensor_valid := 0
		var sensor_present := 0
		for channel in range(SENSOR_CHANNELS):
			var suffix := "%02d" % channel
			var exist_key := "gray_exist_" + suffix
			var valid_key := "gray_valid_" + suffix
			if sample.has(exist_key):
				sensor_present += int(sample[exist_key])
				sensor_valid += int(sample[valid_key])
		if sensor_present == 0:
			sensor_valid = int(sample.get("valid_count", 0))
			sensor_present = int(sample.get("sensor_count", SENSOR_CHANNELS))
		sample["lateral_error_mm"] = lateral * 1000.0
		sample["heading_error_deg"] = rad_to_deg(heading)
		sample["speed_actual_mps"] = actual_speed
		sample["omega_actual_radps"] = actual_omega
		sample["sensor_valid_ratio"] = float(sensor_valid) / maxf(float(sensor_present), 1.0)
		previous_theta = theta
		previous_time_us = time_us


func _update_playback() -> void:
	if trace.is_empty():
		return
	var sample := _sample_at(playback_time_s)
	_update_car(sample)
	_update_hud(sample)
	_set_dashboard_cursor(playback_time_s)


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
	if live_mode:
		if not live_ready:
			return
		live_running = not live_running
		play_button.text = "Pause" if live_running else "Resume"
		return
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
	if live_mode:
		live_running = false
		live_reset_requested = true
		play_button.text = "Resetting"
		return
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
	if live_mode:
		live_running = false
		play_button.text = "Resume"
	playback_time_s = value
	_update_playback()


func _on_speed_selected(index: int) -> void:
	playback_speed = [0.5, 1.0, 2.0, 4.0][index]


func _update_camera(delta: float) -> void:
	if camera_follow:
		camera_target = car.global_position + Vector3(0.0, 0.08, -0.45)
	var horizontal := cos(camera_pitch)
	var offset := Vector3(sin(camera_yaw) * horizontal, sin(camera_pitch),
		cos(camera_yaw) * horizontal) * camera_distance
	var goal := camera_target + offset
	var blend := 1.0 - exp(-6.0 * maxf(delta, 0.016))
	camera.global_position = camera.global_position.lerp(goal, blend)
	camera.look_at(camera_target, Vector3.UP)


func _toggle_camera_follow() -> void:
	camera_follow = not camera_follow
	if camera_follow:
		camera_distance = clampf(camera_distance, 2.0, 6.0)
		camera_pitch = deg_to_rad(36.0)


func _frame_trace() -> void:
	if trace.is_empty():
		return
	camera_follow = false
	camera_yaw = 0.0
	camera_pitch = deg_to_rad(55.0)
	var middle_x := 0.5 * (track_start_x + track_end_x)
	camera_target = _world_point(middle_x, _track_y(middle_x), 0.0)
	camera_distance = maxf(4.5, (track_end_x - track_start_x) * 1.15)


func _reset_camera() -> void:
	camera_follow = true
	camera_distance = 4.2
	camera_yaw = 0.0
	camera_pitch = deg_to_rad(34.0)


func _toggle_fullscreen() -> void:
	if DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_FULLSCREEN:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	else:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_FULLSCREEN)


func _duration_s() -> float:
	if live_mode:
		return live_duration_s
	if trace.is_empty():
		return 0.0
	return float(trace[-1].t_us) / 1.0e6


func _track_y(x: float) -> float:
	if is_zero_approx(track_amplitude_m):
		return 0.0
	return track_amplitude_m * sin(TAU * x / track_wavelength_m)


func _track_heading(x: float) -> float:
	if is_zero_approx(track_amplitude_m):
		return 0.0
	var slope := track_amplitude_m * (TAU / track_wavelength_m) * cos(TAU * x / track_wavelength_m)
	return atan(slope)


func _world_point(sim_x: float, sim_y: float, height: float = 0.0) -> Vector3:
	return Vector3(-sim_y, height, -sim_x)
