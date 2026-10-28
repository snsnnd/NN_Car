extends Node3D

## NN_Car simulation workbench.
## C is the source of truth; this scene is the 3D replay/live view and telemetry console.

const Ui := preload("res://scripts/ui_theme.gd")
const TelemetryDashboard := preload("res://scripts/telemetry_dashboard.gd")
const TimelineBar := preload("res://scripts/timeline_bar.gd")
const SensorStrip := preload("res://scripts/sensor_strip.gd")
const MONO_MEDIUM_FONT := preload("res://assets/fonts/IBMPlexMono-Medium.ttf")

const SENSOR_CHANNELS := 16
const REQUIRED_COLUMNS := [
	"t_us", "x", "y", "theta", "line_conf", "valid_count", "v_target", "omega_target",
	"wheel_v_l", "wheel_v_r", "line_found", "safety_faults", "mode", "pwm_l", "pwm_r"
]
const OPTIONAL_COLUMNS := [
	"v_rail", "v_rail_valid", "safety_latched", "safety_derate", "safety_ramp",
	"safety_stop", "recovery_ms", "line_centroid_m", "wheel_v_l_target",
	"wheel_v_r_target", "u_l", "u_r", "lambda", "v_max_eff", "kappa_target"
]
const INTEGER_COLUMNS := [
	"t_us", "line_found", "valid_count", "safety_faults", "mode", "v_rail_valid",
	"safety_latched", "safety_stop", "rl_action_applied", "rl_terminated", "rl_truncated"
]
const RL_COLUMNS := [
	"rl_action_speed", "rl_action_curvature", "rl_action_applied", "rl_reward",
	"rl_terminated", "rl_truncated"
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
const IMMEDIATE_FAULT_MASK := 0x9C1

const FAULT_NAMES := {
	0: "ESTOP", 1: "STALE TWIST", 2: "STALE WHEEL", 3: "STALE CMD",
	4: "SPEED INVALID", 5: "OVERSPEED", 6: "HARD OVERSPEED", 7: "UNDERVOLT",
	8: "OVERVOLT", 9: "RAIL LOW", 10: "TEMP WARN", 11: "TEMP STOP",
	12: "NN FAULT", 13: "ENCODER", 14: "COMM", 15: "LINE LOST", 16: "CURVATURE"
}

const DEFAULT_SENSOR_POSITIONS := [
	0.0525, 0.0375, 0.0225, 0.0075, -0.0075, -0.0225, -0.0375, -0.0525
]

@onready var camera: Camera3D = $Camera3D

# --- 3D ---
var viewport: SubViewport
var viewport_container: SubViewportContainer
var viewport_holder: Control
var viewport_overlay: Control
var world_root: Node3D
var car: Node3D
var sensor_nodes: Array[MeshInstance3D] = []
var sensor_materials: Array[StandardMaterial3D] = []
var car_beacon: MeshInstance3D
var car_beacon_material: StandardMaterial3D
var car_marker: MeshInstance3D
var car_key_light: SpotLight3D
var track_root: Node3D
var tether_mesh: MeshInstance3D
var tether_immediate: ImmediateMesh
var tether_material: StandardMaterial3D
var tether_label: Label3D
var sun: DirectionalLight3D

# --- run data ---
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
var profile_verified := false
var has_manifest := false
var run_scenario: Dictionary = {}
var sensor_count := 8
var sensor_positions: Array[float] = []
var sensor_to_axle_m := 0.09
var run_file_name := ""
var run_title := "NO RUN"
var run_sample_count := 0

# --- playback ---
var playback_time_s := 0.0
var playback_speed := 1.0
var current_index := 0
var is_playing := false

# --- live ---
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
var live_last_state_ms := 0

# --- camera ---
var cam_mode := "follow"
var follow_yaw := 0.45
var follow_pitch := deg_to_rad(33.0)
var follow_distance := 0.95
var follow_pan := Vector3.ZERO
var track_yaw := deg_to_rad(90.0)
var track_pitch := deg_to_rad(52.0)
var track_zoom := 1.0
var track_pan := Vector3.ZERO
var top_zoom := 1.0
var top_center := Vector3.ZERO
var cam_target := Vector3.ZERO
var cam_position := Vector3.ZERO
var cam_initialized := false
var cam_snap := true
var orbit_dragging := false
var pan_dragging := false

# --- UI ---
var root_ui: Control
var rail_panel: PanelContainer
var rail_scroll: ScrollContainer
var viewport_frame: PanelContainer
var dock_panel: PanelContainer
var run_title_label: Label
var run_sub_label: Label
var status_pill: PanelContainer
var status_dot: Panel
var status_text: Label
var profile_chip: PanelContainer
var profile_chip_label: Label
var rail_run_title: Label
var rail_run_sub: Label
var meta_labels: Dictionary = {}
var ey_value: Label
var ey_state_chip: PanelContainer
var ey_state_label: Label
var heading_value: Label
var speed_value: Label
var speed_sub: Label
var sensor_strip
var safety_status: Label
var safety_mode_chip: PanelContainer
var safety_mode_label: Label
var safety_values: Dictionary = {}
var fault_flow: HFlowContainer
var dashboard
var plot_dashboard
var timeline
var time_label: Label
var time_total_label: Label
var zoom_label: Label
var play_button: Button
var reset_button: Button
var step_back_button: Button
var step_forward_button: Button
var live_button: Button
var save_button: Button
var chart_button: Button
var expand_button: Button
var speed_buttons: Array[Button] = []
var cam_buttons: Dictionary = {}
var empty_card: PanelContainer
var empty_title: Label
var empty_body: Label
var empty_hint: Label
var empty_connect_button: Button
var empty_open_button: Button
var toast_box: VBoxContainer
var help_overlay: Control
var plot_window: Window
var plot_zoom_label: Label
var open_dialog: FileDialog
var save_dialog: FileDialog
var dock_expanded := false
var last_faults := -1
var last_latched := -1
var last_mode := -1
var last_stop := -1


func _ready() -> void:
	get_tree().root.gui_embed_subwindows = false
	get_window().min_size = Vector2i(1180, 760)
	for value in DEFAULT_SENSOR_POSITIONS:
		sensor_positions.append(float(value))
	_build_shell()
	_setup_3d_viewport()
	_create_stage()
	_create_track()
	_create_car()
	_build_diagnostics()
	_build_plot_window()
	_build_dialogs()
	_set_camera_mode("follow", true)
	_update_car({"x": 0.0, "y": 0.0, "theta": 0.0})
	_refresh_run_identity()
	_update_sample_ui({})
	_refresh_empty_state()
	_update_status("offline", "idle")
	_update_playback_buttons()
	_parse_args()


func _parse_args() -> void:
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


# ============================================================== shell / layout

func _build_shell() -> void:
	var canvas := CanvasLayer.new()
	add_child(canvas)

	var backdrop := ColorRect.new()
	backdrop.color = Ui.BG
	backdrop.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	backdrop.mouse_filter = Control.MOUSE_FILTER_IGNORE
	canvas.add_child(backdrop)

	root_ui = Control.new()
	root_ui.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root_ui.theme = Ui.theme()
	root_ui.mouse_filter = Control.MOUSE_FILTER_IGNORE
	canvas.add_child(root_ui)

	var margin := MarginContainer.new()
	margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", 16)
	margin.add_theme_constant_override("margin_right", 16)
	margin.add_theme_constant_override("margin_top", 14)
	margin.add_theme_constant_override("margin_bottom", 14)
	margin.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root_ui.add_child(margin)

	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 11)
	column.mouse_filter = Control.MOUSE_FILTER_IGNORE
	margin.add_child(column)

	column.add_child(_build_top_bar())

	var body := HBoxContainer.new()
	body.add_theme_constant_override("separation", 11)
	body.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.mouse_filter = Control.MOUSE_FILTER_IGNORE
	column.add_child(body)

	rail_panel = _build_rail()
	body.add_child(rail_panel)

	viewport_frame = _build_viewport_frame()
	viewport_frame.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	viewport_frame.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.add_child(viewport_frame)

	dock_panel = _build_dock()
	column.add_child(dock_panel)

	_build_help_overlay()


func _build_top_bar() -> Control:
	var wrapper := VBoxContainer.new()
	wrapper.add_theme_constant_override("separation", 9)
	wrapper.mouse_filter = Control.MOUSE_FILTER_IGNORE

	var bar := HBoxContainer.new()
	bar.add_theme_constant_override("separation", 13)
	bar.custom_minimum_size.y = 34
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	wrapper.add_child(bar)

	var brand := HBoxContainer.new()
	brand.add_theme_constant_override("separation", 9)
	brand.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(brand)

	var mark := PanelContainer.new()
	mark.custom_minimum_size = Vector2(23, 23)
	mark.add_theme_stylebox_override("panel", Ui.flat(Color(0, 0, 0, 0), Color(Ui.ACCENT, 0.85),
		3, 1))
	mark.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	brand.add_child(mark)
	var center := CenterContainer.new()
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	mark.add_child(center)
	var pip := ColorRect.new()
	pip.color = Ui.ACCENT
	pip.custom_minimum_size = Vector2(7, 7)
	pip.mouse_filter = Control.MOUSE_FILTER_IGNORE
	center.add_child(pip)

	var brand_text := VBoxContainer.new()
	brand_text.add_theme_constant_override("separation", -2)
	brand_text.mouse_filter = Control.MOUSE_FILTER_IGNORE
	brand.add_child(brand_text)
	var wordmark := Ui.strong_label("NN·CAR", 16, Ui.TEXT)
	wordmark.mouse_filter = Control.MOUSE_FILTER_IGNORE
	brand_text.add_child(wordmark)
	var sub := Ui.label("LINE-FOLLOWER BENCH", 9, Ui.FAINT)
	sub.mouse_filter = Control.MOUSE_FILTER_IGNORE
	brand_text.add_child(sub)

	bar.add_child(_vrule(22))

	var identity := VBoxContainer.new()
	identity.add_theme_constant_override("separation", 0)
	identity.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	identity.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(identity)
	run_title_label = Ui.strong_label("NO RUN", 13, Ui.TEXT)
	run_title_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	identity.add_child(run_title_label)
	run_sub_label = Ui.label("Connect live or open a CSV log", 9, Ui.MUTED)
	run_sub_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	identity.add_child(run_sub_label)

	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	spacer.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(spacer)

	status_pill = PanelContainer.new()
	status_pill.add_theme_stylebox_override("panel", _status_style(Ui.FAINT))
	status_pill.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(status_pill)
	var status_row := HBoxContainer.new()
	status_row.add_theme_constant_override("separation", 7)
	status_row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	status_pill.add_child(status_row)
	status_dot = Ui.status_dot(Ui.FAINT)
	status_dot.mouse_filter = Control.MOUSE_FILTER_IGNORE
	status_row.add_child(status_dot)
	status_text = Ui.mono_label("OFFLINE", 10, Ui.MUTED)
	status_text.mouse_filter = Control.MOUSE_FILTER_IGNORE
	status_row.add_child(status_text)

	profile_chip = PanelContainer.new()
	profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.ACCENT, 0.08),
		Color(Ui.ACCENT, 0.45), 4, 1, 8.0, 3.0))
	profile_chip.tooltip_text = "profile_verified=false: vehicle and sensor parameters are not hardware-calibrated"
	profile_chip.mouse_filter = Control.MOUSE_FILTER_STOP
	bar.add_child(profile_chip)
	var profile_row := HBoxContainer.new()
	profile_row.add_theme_constant_override("separation", 6)
	profile_row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	profile_chip.add_child(profile_row)
	profile_row.add_child(Ui.status_dot(Ui.ACCENT, 5.0))
	profile_chip_label = Ui.label("MODEL UNVERIFIED", 9, Ui.ACCENT)
	profile_chip_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	profile_row.add_child(profile_chip_label)

	var help := Ui.button("?", "quiet", 26.0)
	help.custom_minimum_size.x = 28.0
	help.tooltip_text = "Controls and shortcuts (? or F1)"
	help.pressed.connect(_toggle_help)
	bar.add_child(help)

	wrapper.add_child(Ui.divider(Color(Ui.LINE, 0.8)))
	return wrapper


func _build_rail() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.custom_minimum_size.x = 308
	panel.size_flags_vertical = Control.SIZE_EXPAND_FILL
	panel.add_theme_stylebox_override("panel", Ui.surface_panel())
	panel.mouse_filter = Control.MOUSE_FILTER_STOP

	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 14)
	margin.add_theme_constant_override("margin_right", 10)
	margin.add_theme_constant_override("margin_top", 13)
	margin.add_theme_constant_override("margin_bottom", 10)
	margin.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.add_child(margin)

	var rail_column := VBoxContainer.new()
	rail_column.add_theme_constant_override("separation", 8)
	rail_column.mouse_filter = Control.MOUSE_FILTER_IGNORE
	margin.add_child(rail_column)

	rail_scroll = ScrollContainer.new()
	rail_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	rail_scroll.vertical_scroll_mode = ScrollContainer.SCROLL_MODE_AUTO
	rail_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	rail_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	rail_scroll.mouse_filter = Control.MOUSE_FILTER_PASS
	rail_column.add_child(rail_scroll)

	var content := VBoxContainer.new()
	content.add_theme_constant_override("separation", 7)
	content.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	content.mouse_filter = Control.MOUSE_FILTER_IGNORE
	rail_scroll.add_child(content)

	content.add_child(Ui.section_header("RUN", "INPUT"))
	rail_run_title = Ui.strong_label("NO RUN", 15, Ui.TEXT)
	content.add_child(rail_run_title)
	rail_run_sub = Ui.label("waiting for data", 9, Ui.MUTED)
	content.add_child(rail_run_sub)

	var meta := GridContainer.new()
	meta.columns = 2
	meta.add_theme_constant_override("h_separation", 12)
	meta.add_theme_constant_override("v_separation", 7)
	meta.mouse_filter = Control.MOUSE_FILTER_IGNORE
	content.add_child(meta)
	for key in ["AMP", "LAMBDA", "SEED", "RAIL"]:
		meta.add_child(_meta_cell(key))

	content.add_child(Ui.divider())
	content.add_child(Ui.section_header("PRIMARY", "CURSOR"))

	var ey_block := VBoxContainer.new()
	ey_block.add_theme_constant_override("separation", 1)
	content.add_child(ey_block)
	ey_block.add_child(Ui.label("CROSS-TRACK ERROR", 8, Ui.MUTED))
	var ey_row := HBoxContainer.new()
	ey_row.add_theme_constant_override("separation", 6)
	ey_row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	ey_block.add_child(ey_row)
	ey_value = Ui.mono_label("—", 24, Ui.TEXT, true)
	ey_value.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	ey_row.add_child(ey_value)
	var ey_unit := Ui.label("mm", 9, Ui.MUTED)
	ey_unit.size_flags_vertical = Control.SIZE_SHRINK_END
	ey_row.add_child(ey_unit)
	var ey_spacer := Control.new()
	ey_spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	ey_row.add_child(ey_spacer)
	ey_state_chip = PanelContainer.new()
	ey_state_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.FAINT, 0.08),
		Color(Ui.FAINT, 0.4), 4, 1, 7.0, 3.0))
	ey_row.add_child(ey_state_chip)
	ey_state_label = Ui.label("NO DATA", 8, Ui.FAINT)
	ey_state_chip.add_child(ey_state_label)

	var twin := HBoxContainer.new()
	twin.add_theme_constant_override("separation", 14)
	content.add_child(twin)
	var heading_block := VBoxContainer.new()
	heading_block.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	heading_block.add_theme_constant_override("separation", 0)
	twin.add_child(heading_block)
	heading_block.add_child(Ui.label("HEADING ERROR", 8, Ui.MUTED))
	var heading_row := HBoxContainer.new()
	heading_row.add_theme_constant_override("separation", 4)
	heading_block.add_child(heading_row)
	heading_value = Ui.mono_label("—", 16, Ui.STEEL, true)
	heading_row.add_child(heading_value)
	heading_row.add_child(_unit_label("deg"))

	var speed_block := VBoxContainer.new()
	speed_block.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	speed_block.add_theme_constant_override("separation", 0)
	twin.add_child(speed_block)
	speed_block.add_child(Ui.label("FORWARD SPEED", 8, Ui.MUTED))
	var speed_row := HBoxContainer.new()
	speed_row.add_theme_constant_override("separation", 4)
	speed_block.add_child(speed_row)
	speed_value = Ui.mono_label("—", 16, Ui.MINT, true)
	speed_row.add_child(speed_value)
	speed_row.add_child(_unit_label("m/s"))
	speed_sub = Ui.label("target —", 8, Ui.FAINT)
	speed_block.add_child(speed_sub)

	content.add_child(Ui.divider())
	content.add_child(Ui.section_header("SENSOR ARRAY", "%02d CH" % sensor_count))
	sensor_strip = SensorStrip.new()
	sensor_strip.set_channel_count(sensor_count)
	content.add_child(sensor_strip)

	rail_column.add_child(Ui.divider())
	var safety_box := VBoxContainer.new()
	safety_box.add_theme_constant_override("separation", 6)
	safety_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	rail_column.add_child(safety_box)
	safety_box.add_child(Ui.section_header("SAFETY", "FAULT BUS"))

	var safety_row := HBoxContainer.new()
	safety_row.add_theme_constant_override("separation", 8)
	safety_box.add_child(safety_row)
	safety_status = Ui.strong_label("CLEAR", 16, Ui.MINT)
	safety_status.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	safety_row.add_child(safety_status)
	var safety_spacer := Control.new()
	safety_spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	safety_row.add_child(safety_spacer)
	safety_mode_chip = PanelContainer.new()
	safety_mode_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.MINT, 0.07),
		Color(Ui.MINT, 0.45), 4, 1, 7.0, 2.0))
	safety_row.add_child(safety_mode_chip)
	safety_mode_label = Ui.label("TRACKING", 8, Ui.MINT)
	safety_mode_chip.add_child(safety_mode_label)

	var safety_grid := GridContainer.new()
	safety_grid.columns = 4
	safety_grid.add_theme_constant_override("h_separation", 8)
	safety_grid.add_theme_constant_override("v_separation", 4)
	safety_box.add_child(safety_grid)
	for key in ["RAIL", "DERATE", "RAMP", "STOP"]:
		var cell := VBoxContainer.new()
		cell.add_theme_constant_override("separation", -1)
		safety_grid.add_child(cell)
		cell.add_child(Ui.label(key, 8, Ui.FAINT))
		var value := Ui.mono_label("—", 9, Ui.TEXT_DIM)
		cell.add_child(value)
		safety_values[key] = value

	fault_flow = HFlowContainer.new()
	fault_flow.add_theme_constant_override("h_separation", 5)
	fault_flow.add_theme_constant_override("v_separation", 5)
	safety_box.add_child(fault_flow)
	_rebuild_faults(0, 0, 0)
	return panel


func _meta_cell(key: String) -> VBoxContainer:
	var cell := VBoxContainer.new()
	cell.add_theme_constant_override("separation", -1)
	cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	cell.mouse_filter = Control.MOUSE_FILTER_IGNORE
	cell.add_child(Ui.label(key, 8, Ui.FAINT))
	var value := Ui.mono_label("—", 10, Ui.TEXT_DIM)
	cell.add_child(value)
	meta_labels[key] = value
	return cell


func _unit_label(text: String) -> Label:
	var label := Ui.label(text, 8, Ui.FAINT)
	label.size_flags_vertical = Control.SIZE_SHRINK_END
	return label


func _build_viewport_frame() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", Ui.flat(Color("0d100e"), Ui.LINE, 8, 1))
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.clip_contents = false

	viewport_holder = Control.new()
	viewport_holder.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	viewport_holder.size_flags_vertical = Control.SIZE_EXPAND_FILL
	viewport_holder.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.add_child(viewport_holder)

	viewport_container = SubViewportContainer.new()
	viewport_container.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	viewport_container.stretch = true
	viewport_container.mouse_filter = Control.MOUSE_FILTER_IGNORE
	viewport_holder.add_child(viewport_container)

	viewport_overlay = Control.new()
	viewport_overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	viewport_overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	viewport_holder.add_child(viewport_overlay)

	var cam_bar := PanelContainer.new()
	cam_bar.anchor_left = 0.0
	cam_bar.anchor_top = 1.0
	cam_bar.anchor_right = 0.0
	cam_bar.anchor_bottom = 1.0
	cam_bar.offset_left = 12.0
	cam_bar.offset_top = -46.0
	cam_bar.offset_right = 386.0
	cam_bar.offset_bottom = -12.0
	cam_bar.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.SURFACE, 0.9), Ui.LINE, 6, 1,
		8.0, 5.0))
	cam_bar.mouse_filter = Control.MOUSE_FILTER_STOP
	viewport_overlay.add_child(cam_bar)
	var cam_row := HBoxContainer.new()
	cam_row.add_theme_constant_override("separation", 5)
	cam_bar.add_child(cam_row)
	var group := ButtonGroup.new()
	group.allow_unpress = false
	for mode in ["follow", "track", "top"]:
		var button := Ui.toggle(mode.to_upper(), 24.0)
		button.custom_minimum_size.x = 56.0
		button.button_group = group
		button.pressed.connect(func() -> void: _set_camera_mode(mode, true))
		cam_row.add_child(button)
		cam_buttons[mode] = button
	var cam_hint := Ui.label("RMB orbit  ·  wheel zoom  ·  MMB pan", 9, Ui.MUTED)
	cam_hint.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	cam_row.add_child(cam_hint)

	toast_box = VBoxContainer.new()
	toast_box.anchor_left = 1.0
	toast_box.anchor_top = 0.0
	toast_box.anchor_right = 1.0
	toast_box.offset_left = -372.0
	toast_box.offset_top = 12.0
	toast_box.offset_right = -12.0
	toast_box.offset_bottom = 320.0
	toast_box.add_theme_constant_override("separation", 6)
	toast_box.alignment = BoxContainer.ALIGNMENT_BEGIN
	toast_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	viewport_overlay.add_child(toast_box)

	var empty_center := CenterContainer.new()
	empty_center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	empty_center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	viewport_overlay.add_child(empty_center)
	empty_card = PanelContainer.new()
	empty_card.custom_minimum_size = Vector2(392, 0)
	empty_card.add_theme_stylebox_override("panel", Ui.surface_panel(Ui.SURFACE, Ui.LINE_STRONG, 10))
	empty_card.mouse_filter = Control.MOUSE_FILTER_STOP
	empty_center.add_child(empty_card)
	var empty_margin := MarginContainer.new()
	empty_margin.add_theme_constant_override("margin_left", 22)
	empty_margin.add_theme_constant_override("margin_right", 22)
	empty_margin.add_theme_constant_override("margin_top", 20)
	empty_margin.add_theme_constant_override("margin_bottom", 18)
	empty_card.add_child(empty_margin)
	var empty_box := VBoxContainer.new()
	empty_box.add_theme_constant_override("separation", 8)
	empty_margin.add_child(empty_box)
	var overline := Ui.mono_label("NO ACTIVE RUN", 9, Ui.ACCENT)
	empty_box.add_child(overline)
	empty_title = Ui.strong_label("Load a run or connect live", 18, Ui.TEXT)
	empty_box.add_child(empty_title)
	empty_body = Ui.label("Replay a C-generated CSV log, or connect to the fixed-step service on 127.0.0.1:37621.", 11, Ui.MUTED)
	empty_body.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	empty_body.custom_minimum_size.x = 348
	empty_box.add_child(empty_body)
	var empty_actions := HBoxContainer.new()
	empty_actions.add_theme_constant_override("separation", 8)
	empty_box.add_child(empty_actions)
	empty_connect_button = Ui.button("CONNECT LIVE", "primary", 32.0)
	empty_connect_button.custom_minimum_size.x = 118
	empty_connect_button.pressed.connect(_on_live_button)
	empty_actions.add_child(empty_connect_button)
	empty_open_button = Ui.button("OPEN CSV", "ghost", 32.0)
	empty_open_button.custom_minimum_size.x = 96
	empty_open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	empty_actions.add_child(empty_open_button)
	empty_hint = Ui.mono_label("bash simulator/run_demo.sh", 8, Ui.FAINT)
	empty_box.add_child(empty_hint)
	return panel


func _build_dock() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.custom_minimum_size.y = 338.0
	panel.add_theme_stylebox_override("panel", Ui.surface_panel())
	panel.mouse_filter = Control.MOUSE_FILTER_STOP

	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 12)
	margin.add_theme_constant_override("margin_right", 12)
	margin.add_theme_constant_override("margin_top", 10)
	margin.add_theme_constant_override("margin_bottom", 10)
	margin.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.add_child(margin)

	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 7)
	column.mouse_filter = Control.MOUSE_FILTER_IGNORE
	margin.add_child(column)

	var transport := HBoxContainer.new()
	transport.add_theme_constant_override("separation", 7)
	transport.custom_minimum_size.y = 30
	column.add_child(transport)

	play_button = Ui.button("PLAY", "primary", 30.0)
	play_button.custom_minimum_size.x = 78
	play_button.pressed.connect(_toggle_playback)
	transport.add_child(play_button)

	reset_button = Ui.button("RESET", "ghost", 30.0)
	reset_button.custom_minimum_size.x = 58
	reset_button.tooltip_text = "Reset to the start (R)"
	reset_button.pressed.connect(_reset_playback)
	transport.add_child(reset_button)

	step_back_button = Ui.button("-1s", "ghost", 30.0)
	step_back_button.custom_minimum_size.x = 40
	step_back_button.tooltip_text = "Step back 1 s (Shift+Left)"
	step_back_button.pressed.connect(func() -> void: _nudge(-1.0))
	transport.add_child(step_back_button)

	step_forward_button = Ui.button("+1s", "ghost", 30.0)
	step_forward_button.custom_minimum_size.x = 40
	step_forward_button.tooltip_text = "Step forward 1 s (Shift+Right)"
	step_forward_button.pressed.connect(func() -> void: _nudge(1.0))
	transport.add_child(step_forward_button)

	var time_box := HBoxContainer.new()
	time_box.add_theme_constant_override("separation", 5)
	time_box.custom_minimum_size.x = 176
	time_box.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	transport.add_child(time_box)
	time_label = Ui.mono_label("0.000", 13, Ui.TEXT, true)
	time_box.add_child(time_label)
	time_total_label = Ui.mono_label("/ 0.000 s", 9, Ui.MUTED)
	time_box.add_child(time_total_label)

	var speed_group := ButtonGroup.new()
	speed_group.allow_unpress = false
	var speed_box := HBoxContainer.new()
	speed_box.add_theme_constant_override("separation", 2)
	transport.add_child(speed_box)
	for index in range(4):
		var factor: float = [0.5, 1.0, 2.0, 4.0][index]
		var button := Ui.toggle("%sx" % str(factor), 24.0)
		button.custom_minimum_size.x = 34
		button.button_group = speed_group
		button.button_pressed = is_equal_approx(factor, 1.0)
		button.pressed.connect(func() -> void: _set_playback_speed(factor))
		speed_box.add_child(button)
		speed_buttons.append(button)

	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	transport.add_child(spacer)

	var open_button := Ui.button("OPEN", "ghost", 30.0)
	open_button.custom_minimum_size.x = 52
	open_button.tooltip_text = "Open a C simulator CSV (O)"
	open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	transport.add_child(open_button)

	live_button = Ui.button("LIVE", "ghost", 30.0)
	live_button.custom_minimum_size.x = 56
	live_button.tooltip_text = "Connect to the C fixed-step service (L)"
	live_button.pressed.connect(_on_live_button)
	transport.add_child(live_button)

	save_button = Ui.button("SAVE", "ghost", 30.0)
	save_button.custom_minimum_size.x = 50
	save_button.tooltip_text = "Save this run as CSV + manifest (S)"
	save_button.pressed.connect(func() -> void: save_dialog.popup_centered_ratio(0.7))
	transport.add_child(save_button)

	chart_button = Ui.button("CHARTS", "ghost", 30.0)
	chart_button.custom_minimum_size.x = 58
	chart_button.tooltip_text = "Open telemetry in a detached window (C)"
	chart_button.pressed.connect(_open_plot_window)
	transport.add_child(chart_button)

	expand_button = Ui.button("EXPAND", "ghost", 30.0)
	expand_button.custom_minimum_size.x = 62
	expand_button.tooltip_text = "Expand telemetry and hide the run rail (D)"
	expand_button.pressed.connect(_toggle_dock)
	transport.add_child(expand_button)

	timeline = TimelineBar.new()
	timeline.custom_minimum_size.y = 26.0
	timeline.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	timeline.seek_requested.connect(_on_seek_requested)
	column.add_child(timeline)

	var chart_header := HBoxContainer.new()
	chart_header.add_theme_constant_override("separation", 8)
	chart_header.custom_minimum_size.y = 16
	column.add_child(chart_header)
	chart_header.add_child(Ui.section_header("TELEMETRY", "STRIP CHART"))
	var chart_spacer := Control.new()
	chart_spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	chart_header.add_child(chart_spacer)
	chart_header.add_child(Ui.label("wheel zoom · drag seek · middle-drag pan", 9, Ui.MUTED))
	zoom_label = Ui.mono_label("FIT", 9, Ui.MUTED)
	zoom_label.visible = false
	chart_header.add_child(zoom_label)
	var fit_button := Ui.button("FIT", "quiet", 20.0)
	fit_button.custom_minimum_size.x = 30
	fit_button.pressed.connect(_fit_plot_views)
	chart_header.add_child(fit_button)

	dashboard = TelemetryDashboard.new()
	dashboard.set_compact(true)
	dashboard.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	dashboard.size_flags_vertical = Control.SIZE_EXPAND_FILL
	dashboard.view_changed.connect(func() -> void: _sync_views(dashboard, plot_dashboard))
	dashboard.seek_requested.connect(_on_seek_requested)
	column.add_child(dashboard)
	return panel


# ============================================================== 3D viewport

func _setup_3d_viewport() -> void:
	viewport = SubViewport.new()
	viewport.own_world_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	viewport.handle_input_locally = false
	viewport.msaa_3d = Viewport.MSAA_2X
	viewport_container.add_child(viewport)

	camera.reparent(viewport)
	camera.current = true
	camera.fov = 42.0
	camera.near = 0.02
	camera.far = 120.0
	camera.projection = Camera3D.PROJECTION_PERSPECTIVE

	world_root = Node3D.new()
	world_root.name = "World"
	viewport.add_child(world_root)


func _create_stage() -> void:
	var world := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Ui.BG
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color("b9c4b2")
	environment.ambient_light_energy = 0.24
	environment.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	environment.fog_enabled = true
	environment.fog_light_color = Color("080b08")
	environment.fog_light_energy = 0.7
	environment.fog_density = 0.0045
	environment.fog_aerial_perspective = 0.0
	world.environment = environment
	world_root.add_child(world)

	sun = DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-49.0, -38.0, 0.0)
	sun.light_color = Color("ffe7c6")
	sun.light_energy = 0.85
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 48.0
	world_root.add_child(sun)

	var fill := DirectionalLight3D.new()
	fill.rotation_degrees = Vector3(-18.0, 142.0, 0.0)
	fill.light_color = Color("9fb6c4")
	fill.light_energy = 0.16
	world_root.add_child(fill)

	var floor_node := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(140.0, 140.0)
	floor_node.mesh = plane
	floor_node.position.y = -0.012
	floor_node.material_override = _material(Color("0b0e0c"), 0.0, 1.0)
	world_root.add_child(floor_node)


func _create_track() -> void:
	if is_instance_valid(track_root):
		track_root.get_parent().remove_child(track_root)
		track_root.queue_free()

	track_root = Node3D.new()
	track_root.name = "TrackAndTrajectory"
	world_root.add_child(track_root)
	_add_ground_grid()
	_add_ribbon("TrackShoulder", 0.078, Color("20251f"), 0.0005)
	_add_ribbon("TrackLine", maxf(track_line_width_m * 0.5, 0.008), Color("b8b3a4"), 0.002)
	_add_track_ticks()
	_add_trajectory()


func _add_ribbon(node_name: String, half_width: float, tint: Color, lift: float) -> void:
	var span := maxf(track_end_x - track_start_x, 1.0)
	var segments := maxi(120, int(span / 0.02))
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
			indices.append_array(PackedInt32Array([base, base + 2, base + 1,
				base + 1, base + 2, base + 3]))

	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var instance := MeshInstance3D.new()
	instance.name = node_name
	instance.mesh = mesh
	var material := _material(tint, 0.0, 0.92)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	instance.material_override = material
	track_root.add_child(instance)


func _add_ground_grid() -> void:
	var line_mesh := ImmediateMesh.new()
	var material := _material(Color.WHITE, 0.0, 1.0)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.vertex_color_use_as_albedo = true
	line_mesh.surface_begin(Mesh.PRIMITIVE_LINES, material)
	var half_side := 1.8
	var x := track_start_x
	var lateral := -half_side
	while lateral <= half_side + 0.001:
		var major := absf(lateral) < 0.001
		var tone := Color(Ui.LINE_STRONG, 0.5) if major else Color(Ui.LINE_STRONG, 0.2)
		line_mesh.surface_set_color(tone)
		line_mesh.surface_add_vertex(Vector3(lateral, -0.004, -track_start_x))
		line_mesh.surface_add_vertex(Vector3(lateral, -0.004, -track_end_x))
		lateral += 0.5
	x = ceilf(track_start_x)
	while x <= track_end_x:
		var major := int(round(x)) % 5 == 0
		var tone := Color(Ui.LINE_STRONG, 0.46) if major else Color(Ui.LINE_STRONG, 0.16)
		line_mesh.surface_set_color(tone)
		line_mesh.surface_add_vertex(Vector3(-half_side, -0.004, -x))
		line_mesh.surface_add_vertex(Vector3(half_side, -0.004, -x))
		x += 1.0
	line_mesh.surface_end()
	var grid := MeshInstance3D.new()
	grid.name = "GroundGrid"
	grid.mesh = line_mesh
	track_root.add_child(grid)


func _add_track_ticks() -> void:
	var line_mesh := ImmediateMesh.new()
	var material := _material(Color(Ui.LINE_STRONG, 0.0), 0.0, 1.0)
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.vertex_color_use_as_albedo = true
	line_mesh.surface_begin(Mesh.PRIMITIVE_LINES, material)
	var start := ceilf(track_start_x)
	var finish := floorf(track_end_x)
	var x := start
	while x <= finish:
		var center := _world_point(x, _track_y(x))
		var half := 0.11 if int(round(x)) % 5 == 0 else 0.085
		var tone := Color(Ui.MUTED, 0.32) if int(round(x)) % 5 == 0 else Color(Ui.MUTED, 0.14)
		line_mesh.surface_set_color(tone)
		line_mesh.surface_add_vertex(center + Vector3(half, 0.004, 0.0))
		line_mesh.surface_add_vertex(center + Vector3(-half, 0.004, 0.0))
		x += 1.0
	line_mesh.surface_end()
	var ticks := MeshInstance3D.new()
	ticks.name = "DistanceTicks"
	ticks.mesh = line_mesh
	track_root.add_child(ticks)


func _build_path_mesh() -> ArrayMesh:
	var vertices := PackedVector3Array()
	var indices := PackedInt32Array()
	var colors := PackedColorArray()
	var half_width := 0.006
	var line_half_mm := maxf(track_line_width_m * 500.0, 8.0)
	for index in range(trace.size()):
		var row: Dictionary = trace[index]
		var x := float(row.x)
		var y := float(row.y)
		var point := _world_point(x, y, 0.021)
		var before: Dictionary = trace[maxi(index - 1, 0)]
		var after: Dictionary = trace[mini(index + 1, trace.size() - 1)]
		var tangent := (_world_point(float(after.x), float(after.y)) -
			_world_point(float(before.x), float(before.y))).normalized()
		var normal := Vector3(-tangent.z, 0.0, tangent.x).normalized()
		var error_mm := absf(float(row.get("lateral_error_mm",
			(y - _track_y(x)) * 1000.0)))
		var color := Ui.MINT
		if error_mm > line_half_mm:
			color = Ui.MINT.lerp(Ui.ACCENT, clampf((error_mm - line_half_mm) / 26.0, 0.0, 1.0))
		if error_mm > line_half_mm + 26.0:
			color = Ui.ACCENT.lerp(Ui.DANGER, clampf((error_mm - line_half_mm - 26.0) / 36.0,
				0.0, 1.0))
		vertices.append(point + normal * half_width)
		vertices.append(point - normal * half_width)
		colors.append(color)
		colors.append(color)
		if index < trace.size() - 1:
			var base := index * 2
			indices.append_array(PackedInt32Array([base, base + 2, base + 1,
				base + 1, base + 2, base + 3]))

	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices
	arrays[Mesh.ARRAY_COLOR] = colors
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func _add_trajectory() -> void:
	if track_root == null:
		return
	var old := track_root.get_node_or_null("LoggedPath")
	if trace.size() < 2:
		if old != null:
			track_root.remove_child(old)
			old.queue_free()
		return
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.vertex_color_use_as_albedo = true
	material.albedo_color = Color.WHITE
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	var instance: MeshInstance3D
	if old is MeshInstance3D:
		instance = old
	else:
		instance = MeshInstance3D.new()
		instance.name = "LoggedPath"
		track_root.add_child(instance)
	instance.mesh = _build_path_mesh()
	instance.material_override = material


func _create_car() -> void:
	var vehicle_scene := load("res://assets/nncar_vehicle.glb") as PackedScene
	if vehicle_scene == null:
		push_error("Missing Blender vehicle asset. Run simulator/assets/build_vehicle.py with Blender.")
		return
	car = vehicle_scene.instantiate() as Node3D
	car.name = "DifferentialDriveCar"
	world_root.add_child(car)

	_style_car()

	sensor_nodes.clear()
	sensor_materials.clear()
	for index in range(SENSOR_CHANNELS):
		var sensor := car.find_child("Sensor_%02d" % index, true, false) as MeshInstance3D
		if sensor == null:
			continue
		var material := _material(Color("1d241f"), 0.0, 0.6)
		material.emission_enabled = true
		material.emission = Color("0f1a15")
		material.emission_energy_multiplier = 0.8
		sensor.material_override = material
		sensor_nodes.append(sensor)
		sensor_materials.append(material)

	var shadow := MeshInstance3D.new()
	var quad := QuadMesh.new()
	quad.size = Vector2(0.34, 0.28)
	var gradient := Gradient.new()
	gradient.colors = PackedColorArray([Color(1, 1, 1, 0.55), Color(1, 1, 1, 0.0)])
	gradient.offsets = PackedFloat32Array([0.0, 1.0])
	var texture := GradientTexture2D.new()
	texture.gradient = gradient
	texture.fill = GradientTexture2D.FILL_RADIAL
	texture.fill_from = Vector2(0.5, 0.5)
	texture.fill_to = Vector2(1.0, 0.5)
	texture.width = 64
	texture.height = 64
	var shadow_material := StandardMaterial3D.new()
	shadow_material.albedo_texture = texture
	shadow_material.albedo_color = Color(0, 0, 0, 0.75)
	shadow_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	shadow_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	shadow_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	shadow.mesh = quad
	shadow.material_override = shadow_material
	shadow.rotation_degrees.x = -90.0
	shadow.position = Vector3(0.0, 0.0015, -0.01)
	car.add_child(shadow)

	var marker := MeshInstance3D.new()
	car_marker = marker
	var torus := TorusMesh.new()
	torus.inner_radius = 0.12
	torus.outer_radius = 0.13
	torus.rings = 48
	torus.ring_segments = 6
	var marker_material := StandardMaterial3D.new()
	marker_material.albedo_color = Ui.ACCENT
	marker_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	marker_material.emission_enabled = true
	marker_material.emission = Ui.ACCENT
	marker_material.emission_energy_multiplier = 1.6
	marker_material.no_depth_test = true
	marker.mesh = torus
	marker.material_override = marker_material
	marker.position.y = 0.004
	marker.visible = false
	car.add_child(marker)

	car_beacon = MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = 0.0048
	sphere.height = 0.0096
	car_beacon_material = _material(Ui.MINT, 0.0, 0.4)
	car_beacon_material.emission_enabled = true
	car_beacon_material.emission = Ui.MINT
	car_beacon_material.emission_energy_multiplier = 2.4
	car_beacon.mesh = sphere
	car_beacon.material_override = car_beacon_material
	car_beacon.position = Vector3(0.0, 0.098, -0.005)
	car.add_child(car_beacon)

	var key_light := SpotLight3D.new()
	car_key_light = key_light
	key_light.position = Vector3(0.0, 0.55, 0.0)
	key_light.rotation_degrees.x = -90.0
	key_light.light_color = Color("e8f0ed")
	key_light.light_energy = 0.5
	key_light.spot_range = 2.5
	key_light.spot_angle = 45.0
	key_light.shadow_enabled = false
	car.add_child(key_light)

	var underglow := OmniLight3D.new()
	underglow.position = Vector3(0.0, 0.025, 0.0)
	underglow.light_color = Color(Ui.ACCENT, 0.8)
	underglow.light_energy = 0.2
	underglow.omni_range = 0.28
	car.add_child(underglow)


func _style_car() -> void:
	var palette := {
		"Chassis": [Color("242a25"), 0.18, 0.5],
		"TopDeck": [Color("2d342d"), 0.12, 0.55],
		"BatteryPack": [Color("3d3324"), 0.05, 0.6],
		"FrontBumper": [Color("333a33"), 0.05, 0.7],
		"DriveWheel_L": [Color("141713"), 0.0, 0.9],
		"DriveWheel_R": [Color("141713"), 0.0, 0.9],
		"WheelHub_L": [Color("c08a4a"), 0.75, 0.3],
		"WheelHub_R": [Color("c08a4a"), 0.75, 0.3],
		"DriveAxle": [Color("7c8178"), 0.6, 0.35],
		"RearCaster": [Color("1c201c"), 0.0, 0.8],
		"GraySensorBoard": [Color("151a15"), 0.0, 0.7]
	}
	for node in car.find_children("*", "MeshInstance3D", true, false):
		var mesh_node := node as MeshInstance3D
		if mesh_node.name.begins_with("Sensor_"):
			continue
		if not palette.has(mesh_node.name):
			continue
		var spec: Array = palette[mesh_node.name]
		mesh_node.material_override = _material(spec[0], float(spec[1]), float(spec[2]))


func _build_diagnostics() -> void:
	tether_immediate = ImmediateMesh.new()
	tether_mesh = MeshInstance3D.new()
	tether_mesh.name = "TrackingTether"
	tether_mesh.mesh = tether_immediate
	tether_material = StandardMaterial3D.new()
	tether_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	tether_material.albedo_color = Color(Ui.ACCENT, 0.9)
	tether_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	tether_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	tether_mesh.material_override = tether_material
	world_root.add_child(tether_mesh)

	tether_label = Label3D.new()
	tether_label.font = MONO_MEDIUM_FONT
	tether_label.font_size = 26
	tether_label.pixel_size = 0.0008
	tether_label.modulate = Ui.ACCENT
	tether_label.outline_modulate = Color(Ui.BG, 0.9)
	tether_label.outline_size = 12
	tether_label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	tether_label.no_depth_test = true
	tether_label.render_priority = 2
	world_root.add_child(tether_label)


func _update_tether(sample: Dictionary) -> void:
	if tether_mesh == null:
		return
	if sample.is_empty() or trace.is_empty():
		tether_mesh.visible = false
		tether_label.visible = false
		return
	var x := float(sample.get("x", 0.0))
	var y := float(sample.get("y", 0.0))
	var center_y := _track_y(x)
	var start := _world_point(x, y, 0.016)
	var end := _world_point(x, center_y, 0.016)
	tether_immediate.clear_surfaces()
	tether_immediate.surface_begin(Mesh.PRIMITIVE_LINES, tether_material)
	tether_immediate.surface_add_vertex(start)
	tether_immediate.surface_add_vertex(end)
	tether_immediate.surface_add_vertex(end + Vector3(0.012, 0.0, 0.0))
	tether_immediate.surface_add_vertex(end - Vector3(0.012, 0.0, 0.0))
	tether_immediate.surface_end()
	var error_mm := float(sample.get("lateral_error_mm", (y - center_y) * 1000.0))
	tether_label.text = "e %+0.1f mm" % error_mm
	tether_label.position = start + Vector3(-0.1, 0.15, -0.02)
	var show := cam_mode == "follow" and int(sample.get("line_found", 1)) != 0 and \
		absf(float(sample.get("line_conf", 1.0))) > 0.0
	tether_mesh.visible = show
	tether_label.visible = show


# ============================================================== camera

func _set_camera_mode(mode: String, reset: bool) -> void:
	cam_mode = mode
	cam_snap = true
	if reset:
		if mode == "follow":
			follow_yaw = 0.45
			follow_pitch = deg_to_rad(33.0)
			follow_distance = 0.95
			follow_pan = Vector3.ZERO
		elif mode == "track":
			track_yaw = deg_to_rad(90.0)
			track_pitch = deg_to_rad(52.0)
			track_zoom = 1.0
			track_pan = Vector3.ZERO
		else:
			top_zoom = 1.0
			top_center = Vector3.ZERO
	for key in cam_buttons:
		var button: Button = cam_buttons[key]
		button.set_pressed_no_signal(key == mode)
	if is_instance_valid(car_marker):
		car_marker.visible = mode != "follow"
	if is_instance_valid(car_key_light):
		car_key_light.visible = mode == "follow"


func _update_camera(delta: float) -> void:
	if not is_instance_valid(camera):
		return
	var target := Vector3.ZERO
	var desired := Vector3.ZERO
	var up := Vector3.UP
	if cam_mode == "follow" and is_instance_valid(car):
		var theta := car.rotation.y + follow_yaw
		var forward := Vector3(-sin(car.rotation.y), 0.0, -cos(car.rotation.y))
		target = car.global_position + forward * 0.28 + Vector3.UP * 0.05 + follow_pan
		var horizontal := cos(follow_pitch)
		var offset := Vector3(sin(theta) * horizontal, sin(follow_pitch),
			cos(theta) * horizontal) * follow_distance
		desired = target + offset
	elif cam_mode == "top":
		target = _track_center() + Vector3.UP * 0.02 + top_center
		desired = target + Vector3(0.0, 14.0, 0.0)
		up = Vector3(1.0, 0.0, 0.0)
		camera.projection = Camera3D.PROJECTION_ORTHOGONAL
		var span := maxf(track_end_x - track_start_x, 1.0)
		var aspect := maxf(float(viewport.size.x) / maxf(float(viewport.size.y), 1.0), 0.6)
		camera.size = maxf(span * 1.1 / aspect, 0.9) * top_zoom
	else:
		target = _track_center() + Vector3.UP * 0.03 + track_pan
		var horizontal := cos(track_pitch)
		var offset := Vector3(sin(track_yaw) * horizontal, sin(track_pitch),
			cos(track_yaw) * horizontal) * _track_fit_distance()
		desired = target + offset
	if cam_mode != "top":
		camera.projection = Camera3D.PROJECTION_PERSPECTIVE
	if cam_snap or not cam_initialized:
		cam_position = desired
		cam_target = target
		cam_initialized = true
		cam_snap = false
	else:
		var blend := 1.0 - exp(-9.0 * maxf(delta, 0.001))
		cam_position = cam_position.lerp(desired, blend)
		cam_target = cam_target.lerp(target, blend)
	camera.global_position = cam_position
	if cam_position.distance_to(cam_target) > 0.001:
		camera.look_at(cam_target, up)


func _track_fit_distance() -> float:
	var span := maxf(track_end_x - track_start_x, 1.0)
	var aspect := maxf(float(viewport.size.x) / maxf(float(viewport.size.y), 1.0), 0.6)
	var fov := deg_to_rad(camera.fov)
	var half_horizontal := atan(tan(fov * 0.5) * aspect)
	var distance := (span * 0.5 * 1.18) / maxf(tan(half_horizontal), 0.01)
	return clampf(distance * 0.95 * track_zoom, 1.5, 90.0)


func _track_center() -> Vector3:
	var middle := 0.5 * (track_start_x + track_end_x)
	return _world_point(middle, _track_y(middle))


func _reset_camera() -> void:
	_set_camera_mode(cam_mode, true)


func _frame_trace() -> void:
	_set_camera_mode("track", true)


func _toggle_camera_follow() -> void:
	_set_camera_mode("follow", true)


func _on_cam_mouse(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT and cam_mode != "top":
			orbit_dragging = event.pressed
			get_viewport().set_input_as_handled()
		elif event.button_index == MOUSE_BUTTON_MIDDLE:
			pan_dragging = event.pressed
			get_viewport().set_input_as_handled()
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_zoom_camera(0.88)
			get_viewport().set_input_as_handled()
		elif event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_zoom_camera(1.14)
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseMotion:
		if orbit_dragging:
			var dx: float = event.relative.x
			var dy: float = event.relative.y
			if cam_mode == "follow":
				follow_yaw -= dx * 0.008
				follow_pitch = clampf(follow_pitch + dy * 0.006, deg_to_rad(8.0),
					deg_to_rad(78.0))
			else:
				track_yaw -= dx * 0.008
				track_pitch = clampf(track_pitch + dy * 0.006, deg_to_rad(18.0),
					deg_to_rad(86.0))
			get_viewport().set_input_as_handled()
		elif pan_dragging:
			_pan_camera(event.relative)
			get_viewport().set_input_as_handled()


func _zoom_camera(factor: float) -> void:
	match cam_mode:
		"follow":
			follow_distance = clampf(follow_distance * factor, 0.7, 6.0)
		"track":
			track_zoom = clampf(track_zoom * factor, 0.5, 2.6)
		_:
			top_zoom = clampf(top_zoom * factor, 0.35, 2.8)


func _pan_camera(relative: Vector2) -> void:
	var right := camera.global_transform.basis.x
	var up_screen := camera.global_transform.basis.y
	var right_flat := Vector3(right.x, 0.0, right.z)
	if right_flat.length() < 0.001:
		right_flat = Vector3.RIGHT
	right_flat = right_flat.normalized()
	var forward_flat := Vector3(up_screen.x, 0.0, up_screen.z)
	if forward_flat.length() < 0.001:
		forward_flat = Vector3.FORWARD
	forward_flat = forward_flat.normalized()
	var scale := 0.0022
	if cam_mode == "follow":
		scale = follow_distance * 0.0018
	elif cam_mode == "track":
		scale = _track_fit_distance() * 0.0018
	elif cam_mode == "top":
		scale = camera.size * 0.0018
	var delta := -right_flat * relative.x * scale + forward_flat * relative.y * scale
	match cam_mode:
		"follow":
			follow_pan += delta
		"track":
			track_pan += delta
		_:
			top_center += delta


# ============================================================== run identity

func _scenario_title() -> String:
	if live_mode:
		return "LIVE SESSION"
	if trace.is_empty():
		return "NO ACTIVE RUN"
	if not has_manifest:
		return "TRACK / NO MANIFEST"
	if track_amplitude_m > 0.0001:
		return "S-CURVE RUN"
	return "STRAIGHT RUN"


func _refresh_run_identity() -> void:
	run_title = _scenario_title()
	run_sample_count = trace.size()
	var source := "live C service" if live_mode else (run_file_name if not run_file_name.is_empty() else "no source")
	run_title_label.text = run_title
	var rate := int(round(1.0 / maxf(line_dt_s, 0.0001)))
	if live_mode:
		run_sub_label.text = "fixed-step link  ·  %d Hz  ·  %.0f s window" % [rate, live_duration_s]
	elif trace.is_empty():
		run_sub_label.text = "connect live or open a CSV log"
	else:
		run_sub_label.text = "%s  ·  %d samples  ·  %d Hz" % [source, run_sample_count, rate]
	rail_run_title.text = run_file_name if not run_file_name.is_empty() else (
		"LIVE SESSION" if live_mode else "NO RUN")
	if live_mode:
		rail_run_sub.text = "C fixed-step link  ·  %d samples" % run_sample_count if run_sample_count > 0 else "waiting for first sample"
	elif not run_file_name.is_empty():
		rail_run_sub.text = "%s  ·  %d samples" % [run_title, run_sample_count]
	else:
		rail_run_sub.text = "waiting for data"

	var scenario := live_scenario if live_mode else run_scenario
	var amp := float(scenario.get("amp_m", track_amplitude_m))
	var wavelength := float(scenario.get("wavelength_m", track_wavelength_m))
	var seed := int(scenario.get("seed", 0))
	var rail := float(scenario.get("rail_v", 0.0))
	if has_manifest or live_mode:
		meta_labels["AMP"].text = "straight" if absf(amp) < 0.0005 else "%0.0f mm" % (amp * 1000.0)
		meta_labels["LAMBDA"].text = "%0.2f m" % wavelength
		meta_labels["SEED"].text = str(seed)
		meta_labels["RAIL"].text = "%0.1f V" % rail
	else:
		meta_labels["AMP"].text = "—"
		meta_labels["LAMBDA"].text = "—"
		meta_labels["SEED"].text = "—"
		meta_labels["RAIL"].text = "—"

	if profile_verified:
		profile_chip_label.text = "MODEL VERIFIED"
		profile_chip_label.add_theme_color_override("font_color", Ui.MINT)
		profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.MINT, 0.08),
			Color(Ui.MINT, 0.45), 4, 1, 8.0, 3.0))
		profile_chip.tooltip_text = "profile_verified=true"
	else:
		profile_chip_label.text = "MODEL UNVERIFIED"
		profile_chip_label.add_theme_color_override("font_color", Ui.ACCENT)
		profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.ACCENT, 0.08),
			Color(Ui.ACCENT, 0.45), 4, 1, 8.0, 3.0))
		profile_chip.tooltip_text = "profile_verified=false: vehicle and sensor parameters are not hardware-calibrated"

	if live_mode:
		rail_run_title.tooltip_text = "live link · C fixed step %0.1f ms · profile_verified=%s" % [
			live_control_dt_s * 1000.0, str(profile_verified)]
	elif has_manifest:
		rail_run_title.tooltip_text = "manifest loaded · line width %0.0f mm · seed %d · profile %s v%s" % [
			track_line_width_m * 1000.0, int(run_scenario.get("seed", 0)), profile_name,
			profile_version]
	else:
		rail_run_title.tooltip_text = "no manifest · straight track assumed"

	_apply_sensor_geometry()
	_refresh_profile_chip()
	_refresh_empty_state()


func _refresh_profile_chip() -> void:
	if is_instance_valid(profile_chip):
		profile_chip.visible = true


func _apply_sensor_geometry() -> void:
	if not is_instance_valid(sensor_strip):
		return
	sensor_strip.set_channel_count(sensor_count)
	sensor_strip.set_positions(sensor_positions)


func _update_status(kind: String, detail: String) -> void:
	var color := Ui.FAINT
	var label := "OFFLINE"
	match kind:
		"replay":
			color = Ui.MUTED
			label = "REPLAY"
		"live":
			color = Ui.ACCENT
			label = "LIVE"
		"connecting":
			color = Ui.ACCENT
			label = "CONNECTING"
		"error":
			color = Ui.DANGER
			label = "LINK ERROR"
		_:
			color = Ui.FAINT
			label = "OFFLINE"
	status_text.text = label if detail.is_empty() else "%s  %s" % [label, detail]
	status_text.add_theme_color_override("font_color", Ui.TEXT_DIM if kind != "offline" else Ui.MUTED)
	status_dot.add_theme_stylebox_override("panel", Ui.flat(color, color, 4, 0))
	status_pill.add_theme_stylebox_override("panel", _status_style(color))


func _status_style(color: Color) -> StyleBoxFlat:
	return Ui.flat(Color(color, 0.05), Color(color, 0.38), 4, 1, 8.0, 3.0)


# ============================================================== sample UI

func _update_sample_ui(sample: Dictionary) -> void:
	if trace.is_empty() or sample.is_empty():
		ey_value.text = "—"
		ey_value.add_theme_color_override("font_color", Ui.TEXT)
		_set_state_chip("NO DATA", Ui.FAINT)
		heading_value.text = "—"
		speed_value.text = "—"
		speed_sub.text = "target —"
		sensor_strip.set_sample({})
		safety_status.text = "—"
		safety_status.add_theme_color_override("font_color", Ui.FAINT)
		safety_mode_label.text = "—"
		safety_mode_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.FAINT, 0.06),
			Color(Ui.FAINT, 0.35), 4, 1, 7.0, 2.0))
		safety_mode_label.add_theme_color_override("font_color", Ui.FAINT)
		for key in safety_values:
			safety_values[key].text = "—"
		_rebuild_faults(0, 0, 0)
		_update_tether({})
		return

	var lateral := float(sample.get("lateral_error_mm", (float(sample.y) - _track_y(float(sample.x))) * 1000.0))
	var line_found := int(sample.get("line_found", 0)) != 0
	ey_value.text = "%+0.1f" % lateral
	var state_color := Ui.FAINT
	var state_text := "NO DATA"
	if line_found:
		if absf(lateral) <= track_line_width_m * 500.0 + 1.0:
			state_text = "ON LINE"
			state_color = Ui.MINT
			ey_value.add_theme_color_override("font_color", Ui.TEXT)
		else:
			state_text = "OFF LINE"
			state_color = Ui.ACCENT
			ey_value.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		state_text = "LINE LOST"
		state_color = Ui.DANGER
		ey_value.add_theme_color_override("font_color", Ui.DANGER)
	_set_state_chip(state_text, state_color)

	var heading := float(sample.get("heading_error_deg",
		rad_to_deg(wrapf(float(sample.theta) - _track_heading(float(sample.x)), -PI, PI))))
	heading_value.text = "%+0.2f" % heading
	var target_speed := float(sample.get("v_target", 0.0))
	var actual_speed := float(sample.get("speed_actual_mps",
		0.5 * (float(sample.get("wheel_v_l", 0.0)) + float(sample.get("wheel_v_r", 0.0)))))
	speed_value.text = "%0.3f" % actual_speed
	speed_sub.text = "target %0.3f m/s" % target_speed

	sensor_strip.set_sample(sample)

	var faults := int(sample.get("safety_faults", 0))
	var latched := int(sample.get("safety_latched", 0))
	var stop := int(sample.get("safety_stop", 0))
	var mode := int(sample.get("mode", 0))
	var derate := float(sample.get("safety_derate", 1.0))
	var ramp := float(sample.get("safety_ramp", 1.0))
	if stop != 0:
		safety_status.text = "STOPPED"
		safety_status.add_theme_color_override("font_color", Ui.DANGER)
	elif faults != 0 and (faults & IMMEDIATE_FAULT_MASK) != 0:
		safety_status.text = "STOP REQUEST"
		safety_status.add_theme_color_override("font_color", Ui.DANGER)
	elif faults != 0:
		safety_status.text = "DERATED" if (derate < 0.999 or ramp < 0.999) else "FAULT"
		safety_status.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		safety_status.text = "CLEAR"
		safety_status.add_theme_color_override("font_color", Ui.MINT)

	safety_mode_label.text = "RECOVERY" if mode == 1 else "TRACKING"
	var mode_color := Ui.ACCENT if mode == 1 else Ui.MINT
	safety_mode_label.add_theme_color_override("font_color", mode_color)
	safety_mode_chip.add_theme_stylebox_override("panel", Ui.flat(Color(mode_color, 0.07),
		Color(mode_color, 0.45), 4, 1, 7.0, 2.0))

	var rail_value := "—"
	if sample.has("v_rail"):
		rail_value = "%0.1f V" % float(sample.v_rail)
		if int(sample.get("v_rail_valid", 1)) == 0:
			rail_value = "invalid"
	safety_values["RAIL"].text = rail_value
	safety_values["DERATE"].text = "%0.0f%%" % (100.0 * float(sample.get("safety_derate", 1.0)))
	safety_values["RAMP"].text = "%0.0f%%" % (100.0 * float(sample.get("safety_ramp", 1.0)))
	safety_values["STOP"].text = "YES" if stop != 0 else "NO"
	safety_values["STOP"].add_theme_color_override("font_color",
		Ui.DANGER if stop != 0 else Ui.TEXT_DIM)

	if faults != last_faults or latched != last_latched or mode != last_mode or stop != last_stop:
		last_faults = faults
		last_latched = latched
		last_mode = mode
		last_stop = stop
		_rebuild_faults(faults, latched, stop)

	if is_instance_valid(car_beacon_material):
		var beacon := Ui.MINT
		if stop != 0 or (faults & IMMEDIATE_FAULT_MASK) != 0:
			beacon = Ui.DANGER
		elif faults != 0:
			beacon = Ui.ACCENT
		car_beacon_material.albedo_color = beacon
		car_beacon_material.emission = beacon
		car_beacon_material.emission_energy_multiplier = 2.6

	_update_tether(sample)


func _set_state_chip(text: String, color: Color) -> void:
	ey_state_label.text = text
	ey_state_label.add_theme_color_override("font_color", color)
	ey_state_chip.add_theme_stylebox_override("panel", Ui.flat(Color(color, 0.08),
		Color(color, 0.45), 4, 1, 7.0, 3.0))


func _rebuild_faults(faults: int, latched: int, stop: int) -> void:
	if fault_flow == null:
		return
	for child in fault_flow.get_children():
		child.queue_free()
	var any := false
	for bit in range(17):
		if (faults & (1 << bit)) == 0:
			continue
		any = true
		var immediate := (faults & (1 << bit) & IMMEDIATE_FAULT_MASK) != 0
		var color := Ui.DANGER if immediate else Ui.ACCENT
		fault_flow.add_child(Ui.chip(str(FAULT_NAMES.get(bit, "FAULT %d" % bit)), color))
	if latched != 0:
		any = true
		fault_flow.add_child(Ui.chip("LATCHED", Ui.DANGER, true))
	if stop != 0:
		any = true
		fault_flow.add_child(Ui.chip("OUTPUT STOP", Ui.DANGER, true))
	if not any:
		fault_flow.add_child(Ui.label("no active faults", 9, Ui.FAINT))


# ============================================================== playback / charts

func _process(delta: float) -> void:
	if live_mode:
		_process_live(delta)
	elif not trace.is_empty() and is_playing:
		playback_time_s = minf(playback_time_s + delta * playback_speed, _duration_s())
		if playback_time_s >= _duration_s():
			is_playing = false
			_update_playback_buttons()
		_update_playback()
	if car != null:
		_update_camera(delta)


func _update_playback() -> void:
	if trace.is_empty():
		return
	var sample := _sample_at(playback_time_s)
	_update_car(sample)
	_update_sample_ui(sample)
	_set_cursor(playback_time_s)
	time_label.text = "%0.3f" % playback_time_s


func _set_cursor(value: float) -> void:
	dashboard.set_cursor(value)
	plot_dashboard.set_cursor(value)
	timeline.set_cursor(value)


func _sample_at(time_s: float) -> Dictionary:
	var time_us := time_s * 1.0e6
	while current_index + 1 < trace.size() and float(trace[current_index + 1].t_us) < time_us:
		current_index += 1
	while current_index > 0 and float(trace[current_index].t_us) > time_us:
		current_index -= 1
	var first: Dictionary = trace[current_index]
	if current_index + 1 >= trace.size():
		return _derive_sample(first)
	var second: Dictionary = trace[current_index + 1]
	var span := float(second.t_us) - float(first.t_us)
	var weight := clampf((time_us - float(first.t_us)) / maxf(span, 1.0), 0.0, 1.0)
	var sample: Dictionary = {}
	for column in REQUIRED_COLUMNS:
		if column in INTEGER_COLUMNS:
			sample[column] = first[column]
		elif column == "theta":
			var angle_delta := wrapf(float(second.theta) - float(first.theta), -PI, PI)
			sample[column] = float(first.theta) + angle_delta * weight
		else:
			sample[column] = lerpf(float(first[column]), float(second[column]), weight)
	for column in OPTIONAL_COLUMNS:
		if first.has(column):
			if column in INTEGER_COLUMNS:
				sample[column] = first[column]
			else:
				sample[column] = lerpf(float(first[column]), float(second.get(column, first[column])),
					weight)
	for column in ["lateral_error_mm", "heading_error_deg", "speed_actual_mps",
			"omega_actual_radps", "sensor_valid_ratio"]:
		if first.has(column):
			sample[column] = lerpf(float(first[column]), float(second.get(column, first[column])),
				weight)
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
	return _derive_sample(sample)


func _derive_sample(sample: Dictionary) -> Dictionary:
	var x := float(sample.get("x", 0.0))
	var y := float(sample.get("y", 0.0))
	var theta := float(sample.get("theta", 0.0))
	sample["lateral_error_mm"] = (y - _track_y(x)) * 1000.0
	sample["heading_error_deg"] = rad_to_deg(wrapf(theta - _track_heading(x), -PI, PI))
	sample["speed_actual_mps"] = 0.5 * (float(sample.get("wheel_v_l", 0.0)) +
		float(sample.get("wheel_v_r", 0.0)))
	if not sample.has("omega_actual_radps"):
		sample["omega_actual_radps"] = 0.0
	var valid := 0
	var present := 0
	for index in range(SENSOR_CHANNELS):
		var suffix := "%02d" % index
		if sample.has("gray_exist_" + suffix):
			present += int(sample["gray_exist_" + suffix])
			valid += int(sample["gray_valid_" + suffix])
	if present == 0:
		present = maxi(int(sample.get("sensor_count", sensor_count)), 1)
		valid = int(sample.get("valid_count", 0))
	sample["sensor_valid_ratio"] = float(valid) / maxf(float(present), 1.0)
	sample["sensor_count"] = int(sample.get("sensor_count", sensor_count))
	return sample


func _toggle_playback() -> void:
	if live_mode:
		if not live_ready:
			return
		live_running = not live_running
		_update_playback_buttons()
		return
	if trace.is_empty():
		open_dialog.popup_centered_ratio(0.7)
		return
	if playback_time_s >= _duration_s():
		playback_time_s = 0.0
		current_index = 0
		is_playing = true
		_update_playback()
	else:
		is_playing = not is_playing
	_update_playback_buttons()


func _reset_playback() -> void:
	if live_mode:
		live_running = false
		live_reset_requested = true
		_update_playback_buttons()
		return
	if trace.is_empty():
		return
	is_playing = false
	playback_time_s = 0.0
	current_index = 0
	timeline.set_cursor(0.0)
	_update_playback()
	_update_playback_buttons()


func _on_seek_requested(time_s: float) -> void:
	if trace.is_empty():
		return
	is_playing = false
	live_running = false
	if live_mode:
		playback_time_s = clampf(time_s, 0.0, maxf(_last_trace_time(), 0.0))
	else:
		playback_time_s = clampf(time_s, 0.0, _duration_s())
	current_index = 0
	_update_playback()
	_update_playback_buttons()


func _nudge(seconds: float) -> void:
	if trace.is_empty():
		return
	_on_seek_requested(playback_time_s + seconds)


func _set_playback_speed(factor: float) -> void:
	playback_speed = factor
	for index in range(speed_buttons.size()):
		var value: float = [0.5, 1.0, 2.0, 4.0][index]
		speed_buttons[index].set_pressed_no_signal(is_equal_approx(value, factor))


func _update_playback_buttons() -> void:
	if not is_instance_valid(play_button):
		return
	if live_mode:
		play_button.text = "PAUSE" if live_running else ("RESUME" if live_ready else "CONNECT")
	else:
		play_button.text = "PAUSE" if is_playing else "PLAY"
	reset_button.disabled = trace.is_empty()
	step_back_button.disabled = trace.is_empty()
	step_forward_button.disabled = trace.is_empty()
	save_button.disabled = trace.is_empty()
	chart_button.disabled = trace.is_empty()
	live_button.text = "STOP" if live_mode else "LIVE"
	if live_mode:
		live_button.add_theme_stylebox_override("normal", Ui.flat(Color(Ui.ACCENT, 0.13),
			Color(Ui.ACCENT, 0.55), 6, 1, 10.0, 5.0))
		live_button.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		live_button.remove_theme_stylebox_override("normal")
		live_button.remove_theme_color_override("font_color")


func _duration_s() -> float:
	if live_mode:
		return live_duration_s
	if trace.is_empty():
		return 0.0
	return float(trace[-1].t_us) * 1.0e-6


func _last_trace_time() -> float:
	if trace.is_empty():
		return 0.0
	return float(trace[-1].t_us) * 1.0e-6


func _set_dashboard_trace(rows: Array[Dictionary], cursor_s: float, duration_s: float,
		new_trace: bool = false) -> void:
	dashboard.set_trace(rows, cursor_s, duration_s, new_trace)
	plot_dashboard.set_trace(rows, cursor_s, duration_s, new_trace)
	timeline.set_data(rows, cursor_s, duration_s, live_mode, new_trace)
	dashboard.track_half_width_mm = track_line_width_m * 500.0
	plot_dashboard.track_half_width_mm = track_line_width_m * 500.0
	_update_zoom_label()


func _sync_views(source, target) -> void:
	if not is_instance_valid(source) or not is_instance_valid(target):
		return
	target.apply_view_state(source.get_view_state())
	_update_zoom_label()


func _update_zoom_label() -> void:
	if is_instance_valid(zoom_label) and is_instance_valid(dashboard):
		var label: String = dashboard.get_zoom_label()
		zoom_label.text = "" if label == "FIT" else label
		zoom_label.visible = label != "FIT"
	if is_instance_valid(plot_zoom_label) and is_instance_valid(plot_dashboard):
		var plot_label: String = plot_dashboard.get_zoom_label()
		plot_zoom_label.text = "" if plot_label == "FIT" else plot_label
		plot_zoom_label.visible = plot_label != "FIT"


func _fit_plot_views() -> void:
	dashboard.fit_trace()
	plot_dashboard.fit_trace()
	_update_zoom_label()


func _toggle_dock() -> void:
	dock_expanded = not dock_expanded
	dock_panel.custom_minimum_size.y = 520.0 if dock_expanded else 338.0
	rail_panel.visible = not dock_expanded
	expand_button.text = "COMPACT" if dock_expanded else "EXPAND"


func _open_plot_window() -> void:
	if not is_instance_valid(plot_window):
		return
	plot_window.popup_centered(Vector2i(1280, 820))
	plot_window.grab_focus()


func _build_plot_window() -> void:
	plot_window = Window.new()
	plot_window.title = "NN_CAR  /  TELEMETRY"
	plot_window.size = Vector2i(1280, 820)
	plot_window.min_size = Vector2i(980, 620)
	plot_window.visible = false
	plot_window.close_requested.connect(func() -> void: plot_window.hide())
	add_child(plot_window)

	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.theme = Ui.theme()
	plot_window.add_child(root)

	var margin := MarginContainer.new()
	margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", 12)
	margin.add_theme_constant_override("margin_right", 12)
	margin.add_theme_constant_override("margin_top", 12)
	margin.add_theme_constant_override("margin_bottom", 12)
	root.add_child(margin)

	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 8)
	margin.add_child(column)

	var toolbar := PanelContainer.new()
	toolbar.add_theme_stylebox_override("panel", Ui.surface_panel(Ui.SURFACE, Ui.LINE, 8))
	column.add_child(toolbar)
	var toolbar_margin := MarginContainer.new()
	toolbar_margin.add_theme_constant_override("margin_left", 12)
	toolbar_margin.add_theme_constant_override("margin_right", 12)
	toolbar_margin.add_theme_constant_override("margin_top", 8)
	toolbar_margin.add_theme_constant_override("margin_bottom", 8)
	toolbar.add_child(toolbar_margin)
	var bar := HBoxContainer.new()
	bar.add_theme_constant_override("separation", 8)
	toolbar_margin.add_child(bar)
	var title_box := VBoxContainer.new()
	title_box.add_theme_constant_override("separation", 0)
	title_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	bar.add_child(title_box)
	title_box.add_child(Ui.strong_label("TELEMETRY", 13, Ui.TEXT))
	title_box.add_child(Ui.label("synchronized to run cursor", 9, Ui.MUTED))
	var zoom_out_button := Ui.button("−", "ghost", 26.0)
	zoom_out_button.custom_minimum_size.x = 30
	zoom_out_button.tooltip_text = "Zoom out in time"
	zoom_out_button.pressed.connect(func() -> void: plot_dashboard.zoom_out())
	bar.add_child(zoom_out_button)
	var zoom_in_button := Ui.button("+", "ghost", 26.0)
	zoom_in_button.custom_minimum_size.x = 30
	zoom_in_button.tooltip_text = "Zoom in in time"
	zoom_in_button.pressed.connect(func() -> void: plot_dashboard.zoom_in())
	bar.add_child(zoom_in_button)
	var fit_button := Ui.button("FIT", "ghost", 26.0)
	fit_button.custom_minimum_size.x = 40
	fit_button.pressed.connect(_fit_plot_views)
	bar.add_child(fit_button)
	plot_zoom_label = Ui.mono_label("FIT", 9, Ui.MUTED)
	plot_zoom_label.visible = false
	bar.add_child(plot_zoom_label)
	bar.add_child(Ui.label("wheel zoom · drag seek · middle-drag pan", 9, Ui.MUTED))

	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", Ui.surface_panel(Ui.SURFACE, Ui.LINE, 8))
	panel.size_flags_vertical = Control.SIZE_EXPAND_FILL
	column.add_child(panel)
	var plot_margin := MarginContainer.new()
	plot_margin.add_theme_constant_override("margin_left", 6)
	plot_margin.add_theme_constant_override("margin_right", 6)
	plot_margin.add_theme_constant_override("margin_top", 6)
	plot_margin.add_theme_constant_override("margin_bottom", 6)
	panel.add_child(plot_margin)
	plot_dashboard = TelemetryDashboard.new()
	plot_dashboard.set_compact(false)
	plot_dashboard.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	plot_dashboard.size_flags_vertical = Control.SIZE_EXPAND_FILL
	plot_dashboard.view_changed.connect(func() -> void: _sync_views(plot_dashboard, dashboard))
	plot_dashboard.seek_requested.connect(_on_seek_requested)
	plot_margin.add_child(plot_dashboard)
	plot_dashboard.track_half_width_mm = track_line_width_m * 500.0


# ============================================================== dialogs / overlays

func _build_dialogs() -> void:
	open_dialog = FileDialog.new()
	open_dialog.title = "OPEN RUN LOG"
	open_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	open_dialog.access = FileDialog.ACCESS_FILESYSTEM
	open_dialog.filters = PackedStringArray(["*.csv ; NN_Car simulation log"])
	open_dialog.use_native_dialog = true
	open_dialog.file_selected.connect(_load_trace)
	root_ui.add_child(open_dialog)

	save_dialog = FileDialog.new()
	save_dialog.title = "SAVE RUN"
	save_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	save_dialog.access = FileDialog.ACCESS_FILESYSTEM
	save_dialog.filters = PackedStringArray(["*.csv ; NN_Car simulation log"])
	save_dialog.use_native_dialog = true
	save_dialog.file_selected.connect(_save_trace)
	root_ui.add_child(save_dialog)


func _build_help_overlay() -> void:
	help_overlay = Control.new()
	help_overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	help_overlay.mouse_filter = Control.MOUSE_FILTER_STOP
	help_overlay.visible = false
	root_ui.add_child(help_overlay)

	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.78)
	dim.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	dim.mouse_filter = Control.MOUSE_FILTER_STOP
	dim.gui_input.connect(func(event: InputEvent) -> void:
		if event is InputEventMouseButton and event.pressed:
			_set_help_visible(false))
	help_overlay.add_child(dim)

	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	help_overlay.add_child(center)

	var card := PanelContainer.new()
	card.custom_minimum_size = Vector2(620, 0)
	card.add_theme_stylebox_override("panel", Ui.surface_panel(Ui.SURFACE, Ui.LINE_STRONG, 10))
	card.mouse_filter = Control.MOUSE_FILTER_STOP
	center.add_child(card)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 24)
	margin.add_theme_constant_override("margin_right", 24)
	margin.add_theme_constant_override("margin_top", 22)
	margin.add_theme_constant_override("margin_bottom", 20)
	card.add_child(margin)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 12)
	margin.add_child(column)

	var head := HBoxContainer.new()
	column.add_child(head)
	var head_box := VBoxContainer.new()
	head_box.add_theme_constant_override("separation", 0)
	head_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(head_box)
	head_box.add_child(Ui.mono_label("NN·CAR  /  CONTROLS", 9, Ui.ACCENT))
	head_box.add_child(Ui.strong_label("Keyboard & pointer", 18, Ui.TEXT))
	var close := Ui.button("CLOSE", "ghost", 28.0)
	close.custom_minimum_size.x = 62
	close.pressed.connect(func() -> void: _set_help_visible(false))
	head.add_child(close)

	var columns := HBoxContainer.new()
	columns.add_theme_constant_override("separation", 24)
	column.add_child(columns)
	var left := VBoxContainer.new()
	left.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	left.add_theme_constant_override("separation", 6)
	columns.add_child(left)
	var right := VBoxContainer.new()
	right.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	right.add_theme_constant_override("separation", 6)
	columns.add_child(right)

	left.add_child(Ui.section_header("PLAYBACK", ""))
	for row in [
		["SPACE", "Play / pause"],
		["R", "Reset cursor"],
		["← / →", "Step 0.1 s"],
		["SHIFT + ← / →", "Step 1 s"],
		["0.5x - 4x", "Playback speed"]
	]:
		left.add_child(_shortcut_row(row[0], row[1]))
	left.add_child(Ui.divider())
	left.add_child(Ui.section_header("RUN", ""))
	for row in [["O", "Open CSV log"], ["L", "Connect / stop live"], ["S", "Save run"], ["C", "Detached charts"], ["D", "Expand telemetry"]]:
		left.add_child(_shortcut_row(row[0], row[1]))

	right.add_child(Ui.section_header("VIEW", ""))
	for row in [
		["F", "Follow the car"],
		["V", "Fit the whole track"],
		["T", "Top-down track"],
		["0", "Reset current view"],
		["RMB drag", "Orbit camera"],
		["MMB drag", "Pan camera"],
		["WHEEL", "Zoom camera / time"],
		["F11", "Fullscreen"]
	]:
		right.add_child(_shortcut_row(row[0], row[1]))
	right.add_child(Ui.divider())
	right.add_child(Ui.section_header("DATA", ""))
	right.add_child(Ui.label("Charts share one time cursor. Drag inside a chart to seek; hover for a readout.", 9, Ui.MUTED))
	right.add_child(Ui.label("Fault spans are tinted on the timeline and chart background.", 9, Ui.MUTED))


func _shortcut_row(key: String, description: String) -> HBoxContainer:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 9)
	var key_label := Ui.mono_label(key, 9, Ui.TEXT_DIM)
	key_label.custom_minimum_size.x = 92
	row.add_child(key_label)
	var description_label := Ui.label(description, 10, Ui.MUTED)
	description_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(description_label)
	return row


func _toggle_help() -> void:
	_set_help_visible(not help_overlay.visible)


func _set_help_visible(value: bool) -> void:
	if is_instance_valid(help_overlay):
		help_overlay.visible = value
	if value:
		help_overlay.move_to_front()


func _refresh_empty_state() -> void:
	if not is_instance_valid(empty_card):
		return
	var should_show := trace.is_empty()
	empty_card.visible = should_show
	if not should_show:
		return
	if live_mode:
		empty_title.text = "Connecting to the C service"
		empty_body.text = "Waiting for the fixed-step simulator on 127.0.0.1:%d. A reset response starts the run." % live_port
		empty_hint.text = "fixed step  %.0f ms" % (live_control_dt_s * 1000.0)
		empty_connect_button.text = "DISCONNECT"
		empty_open_button.disabled = true
	else:
		empty_title.text = "Load a run or connect live"
		empty_body.text = "Replay a C-generated CSV log, or connect to the fixed-step service on 127.0.0.1:%d." % live_port
		empty_hint.text = "bash simulator/run_demo.sh"
		empty_connect_button.text = "CONNECT LIVE"
		empty_open_button.disabled = false
	empty_connect_button.disabled = false


func _notify(message: String, kind: String = "info") -> void:
	if not is_instance_valid(toast_box):
		return
	var color := Ui.ACCENT
	if kind == "error":
		color = Ui.DANGER
	elif kind == "ok":
		color = Ui.MINT
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.SURFACE_3, 0.97),
		Color(color, 0.55), 6, 1, 0.0, 0.0))
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 9)
	panel.add_child(row)
	var bar := ColorRect.new()
	bar.color = color
	bar.custom_minimum_size = Vector2(2, 0)
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_child(bar)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 8)
	margin.add_theme_constant_override("margin_right", 10)
	margin.add_theme_constant_override("margin_top", 8)
	margin.add_theme_constant_override("margin_bottom", 8)
	margin.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(margin)
	var text := Ui.label(message, 10, Ui.TEXT_DIM)
	text.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	text.custom_minimum_size.x = 300
	margin.add_child(text)
	toast_box.add_child(panel)
	if toast_box.get_child_count() > 4:
		toast_box.get_child(0).queue_free()
	panel.modulate.a = 0.0
	var tween := create_tween()
	tween.tween_property(panel, "modulate:a", 1.0, 0.16)
	tween.tween_interval(3.4)
	tween.tween_property(panel, "modulate:a", 0.0, 0.35)
	tween.finished.connect(func() -> void:
		if is_instance_valid(panel):
			panel.queue_free())


# ============================================================== input

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if open_dialog.visible or save_dialog.visible:
		return
	if help_overlay.visible and event.keycode != KEY_ESCAPE:
		return
	if event.keycode == KEY_ESCAPE and help_overlay.visible:
		_set_help_visible(false)
		get_viewport().set_input_as_handled()
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
		KEY_L:
			_on_live_button()
			get_viewport().set_input_as_handled()
		KEY_S:
			if not trace.is_empty():
				save_dialog.popup_centered_ratio(0.7)
			get_viewport().set_input_as_handled()
		KEY_C:
			_open_plot_window()
			get_viewport().set_input_as_handled()
		KEY_D:
			_toggle_dock()
			get_viewport().set_input_as_handled()
		KEY_F:
			_set_camera_mode("follow", true)
			get_viewport().set_input_as_handled()
		KEY_V:
			_frame_trace()
			get_viewport().set_input_as_handled()
		KEY_T:
			_set_camera_mode("top", true)
			get_viewport().set_input_as_handled()
		KEY_0, KEY_KP_0:
			_reset_camera()
			get_viewport().set_input_as_handled()
		KEY_LEFT:
			_nudge(-1.0 if event.shift_pressed else -0.1)
			get_viewport().set_input_as_handled()
		KEY_RIGHT:
			_nudge(1.0 if event.shift_pressed else 0.1)
			get_viewport().set_input_as_handled()
		KEY_F11:
			_toggle_fullscreen()
			get_viewport().set_input_as_handled()
		KEY_F1:
			_toggle_help()
			get_viewport().set_input_as_handled()
		KEY_SLASH:
			if event.shift_pressed:
				_toggle_help()
				get_viewport().set_input_as_handled()


func _unhandled_input(event: InputEvent) -> void:
	if help_overlay.visible or open_dialog.visible or save_dialog.visible:
		return
	if event is InputEventMouseButton or event is InputEventMouseMotion:
		var inside := viewport_holder.get_global_rect().has_point(get_viewport().get_mouse_position())
		if event is InputEventMouseMotion and (orbit_dragging or pan_dragging):
			inside = true
		if inside:
			_on_cam_mouse(event)


func _toggle_fullscreen() -> void:
	if DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_FULLSCREEN:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	else:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_FULLSCREEN)


# ============================================================== data loading

func _load_trace(path: String) -> void:
	_disconnect_live(true)
	live_mode = false
	live_running = false
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		_notify("Could not open CSV (error %d)." % FileAccess.get_open_error(), "error")
		return

	var header := file.get_csv_line()
	var column_index: Dictionary = {}
	for index in range(header.size()):
		column_index[header[index]] = index
	for column in REQUIRED_COLUMNS:
		if not column_index.has(column):
			_notify("Unsupported CSV: missing '%s'." % column, "error")
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
	var optional_present: Array[String] = []
	for column in OPTIONAL_COLUMNS:
		if column_index.has(column):
			optional_present.append(column)

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
			sample[column] = value.to_int() if column in INTEGER_COLUMNS else value.to_float()
		if not valid_row:
			continue
		for column in optional_present:
			var value := values[int(column_index[column])]
			if value.is_empty():
				continue
			sample[column] = value.to_int() if column in INTEGER_COLUMNS else value.to_float()
		if has_sensor_channels:
			for index in range(SENSOR_CHANNELS):
				var suffix := "%02d" % index
				sample["gray_value_" + suffix] = values[int(column_index["gray_value_" + suffix])].to_float()
				sample["gray_exist_" + suffix] = values[int(column_index["gray_exist_" + suffix])].to_int()
				sample["gray_valid_" + suffix] = values[int(column_index["gray_valid_" + suffix])].to_int()
		if has_rl_fields:
			for column in RL_COLUMNS:
				var value := values[int(column_index[column])]
				sample[column] = value.to_int() if column in INTEGER_COLUMNS else value.to_float()
		loaded.append(sample)
	file.close()

	if loaded.size() < 2:
		_notify("CSV contains fewer than two valid samples.", "error")
		return

	run_file_name = path.get_file()
	trace = loaded
	playback_time_s = 0.0
	current_index = 0
	is_playing = false
	playback_speed = 1.0
	_set_playback_speed(1.0)
	track_start_x = float(trace[0].x) - 1.0
	track_end_x = float(trace[-1].x) + 2.0
	_load_manifest(path)
	_prepare_plot_fields()
	_create_track()
	_set_camera_mode("follow", true)
	_update_car(_derive_sample(trace[0]))
	_update_sample_ui(_derive_sample(trace[0]))
	_set_dashboard_trace(trace, 0.0, _duration_s(), true)
	time_label.text = "0.000"
	time_total_label.text = "/ %0.3f s" % _duration_s()
	_refresh_run_identity()
	_refresh_empty_state()
	_update_status("replay", "%d Hz" % int(round(1.0 / maxf(line_dt_s, 0.0001))))
	_update_playback_buttons()
	_set_help_visible(false)
	_notify("%d samples loaded from %s" % [trace.size(), run_file_name], "ok")


func _load_manifest(csv_path: String) -> void:
	track_amplitude_m = 0.0
	track_wavelength_m = 1.0
	track_line_width_m = 0.02
	line_dt_s = 0.01
	motor_dt_s = 0.002
	profile_name = "mg513x_12v_4s_default"
	profile_version = "1.0.0"
	profile_verified = false
	sensor_count = 8
	sensor_positions.clear()
	for value in DEFAULT_SENSOR_POSITIONS:
		sensor_positions.append(float(value))
	sensor_to_axle_m = 0.09
	has_manifest = false
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
	var manifest: Dictionary = parsed
	has_manifest = true
	var scenario: Variant = manifest.get("scenario", {})
	if scenario is Dictionary:
		run_scenario.merge(scenario, true)
		track_amplitude_m = float(scenario.get("amp_m", 0.0))
		track_wavelength_m = maxf(float(scenario.get("wavelength_m", 1.0)), 0.001)
	var simulation: Variant = manifest.get("simulation", {})
	if simulation is Dictionary:
		track_line_width_m = maxf(float(simulation.get("line_width_m", 0.02)), 0.001)
	var vehicle: Variant = manifest.get("vehicle", {})
	if vehicle is Dictionary:
		sensor_count = clampi(int(vehicle.get("sensor_count", sensor_count)), 1, 16)
		sensor_to_axle_m = float(vehicle.get("sensor_to_axle_m", sensor_to_axle_m))
		var positions: Variant = vehicle.get("sensor_position_m", [])
		if positions is Array and not (positions as Array).is_empty():
			sensor_positions.clear()
			for value in positions:
				sensor_positions.append(float(value))
	line_dt_s = float(manifest.get("line_dt_s", line_dt_s))
	motor_dt_s = float(manifest.get("tick_dt_s", motor_dt_s))
	profile_name = str(manifest.get("profile", profile_name))
	profile_version = str(manifest.get("profile_version", profile_version))
	profile_verified = bool(manifest.get("profile_verified", false))


func _prepare_plot_fields() -> void:
	var previous_theta := 0.0
	var previous_time_us := 0.0
	for index in range(trace.size()):
		var sample: Dictionary = trace[index]
		var time_us := float(sample.get("t_us", 0))
		var x := float(sample.get("x", 0.0))
		var theta := float(sample.get("theta", 0.0))
		sample["lateral_error_mm"] = (float(sample.get("y", 0.0)) - _track_y(x)) * 1000.0
		sample["heading_error_deg"] = rad_to_deg(wrapf(theta - _track_heading(x), -PI, PI))
		var actual_speed := 0.5 * (float(sample.get("wheel_v_l", 0.0)) +
			float(sample.get("wheel_v_r", 0.0)))
		sample["speed_actual_mps"] = actual_speed
		var actual_omega := 0.0
		if index > 0:
			var elapsed := (time_us - previous_time_us) * 1.0e-6
			if elapsed > 0.0:
				actual_omega = wrapf(theta - previous_theta, -PI, PI) / elapsed
		sample["omega_actual_radps"] = actual_omega
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
			sensor_present = maxi(sensor_count, 1)
		sample["sensor_valid_ratio"] = float(sensor_valid) / maxf(float(sensor_present), 1.0)
		sample["sensor_count"] = sensor_count
		previous_theta = theta
		previous_time_us = time_us


func _save_trace(path: String) -> void:
	if trace.is_empty():
		_notify("No samples to save.", "error")
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
		_notify("Could not save CSV (error %d)." % FileAccess.get_open_error(), "error")
		return
	file.store_line(",".join(columns))
	for sample in trace:
		var values := PackedStringArray()
		for column in columns:
			var value: Variant = sample.get(column, 0)
			if column in INTEGER_COLUMNS or column.begins_with("gray_exist_") or \
					column.begins_with("gray_valid_"):
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
		"profile_verified": profile_verified,
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
		},
		"vehicle": {
			"sensor_count": sensor_count,
			"sensor_to_axle_m": sensor_to_axle_m,
			"sensor_position_m": sensor_positions
		},
		"simulation": {"line_width_m": track_line_width_m}
	}
	var manifest_file := FileAccess.open(csv_path.get_basename() + ".manifest.json", FileAccess.WRITE)
	if manifest_file != null:
		manifest_file.store_string(JSON.stringify(manifest, "\t"))
		manifest_file.close()
	_notify("Saved %d samples to %s" % [trace.size(), csv_path.get_file()], "ok")


# ============================================================== live link

func _connect_live(port: int = DEFAULT_LIVE_PORT) -> void:
	if live_peer != null:
		return
	live_port = port if port > 0 else DEFAULT_LIVE_PORT
	live_peer = StreamPeerTCP.new()
	var error := live_peer.connect_to_host("127.0.0.1", live_port)
	if error != OK:
		live_peer = null
		_notify("Live connect failed on 127.0.0.1:%d." % live_port, "error")
		_update_status("error", "connect failed")
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
	run_file_name = ""
	_set_dashboard_trace(trace, 0.0, live_duration_s, true)
	time_label.text = "0.000"
	time_total_label.text = "/ %0.3f s" % live_duration_s
	_refresh_run_identity()
	_refresh_empty_state()
	_update_playback_buttons()
	_update_status("connecting", "%d Hz" % int(round(1.0 / maxf(live_control_dt_s, 0.0001))))
	_notify("Connecting to the C fixed-step service on 127.0.0.1:%d." % live_port)


func _disconnect_live(send_close: bool) -> void:
	var was_live := live_mode
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
	if was_live:
		is_playing = false
		if trace.is_empty():
			_refresh_run_identity()
			_refresh_empty_state()
			_update_status("offline", "idle")
		else:
			playback_time_s = minf(playback_time_s, _last_trace_time())
			current_index = 0
			_set_dashboard_trace(trace, playback_time_s, _duration_s(), true)
			time_total_label.text = "/ %0.3f s" % _duration_s()
			_update_playback()
			_refresh_run_identity()
			_update_status("replay", "%d Hz" % int(round(1.0 / maxf(line_dt_s, 0.0001))))
		_update_playback_buttons()


func _on_live_button() -> void:
	if live_mode:
		_notify("Live link closed · run kept for replay", "info")
		_disconnect_live(true)
	else:
		_connect_live(live_port)


func _process_live(delta: float) -> void:
	if live_peer == null:
		return
	live_peer.poll()
	var status := live_peer.get_status()
	if status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE:
		_notify("C simulation disconnected.", "error")
		_update_status("error", "link lost")
		_disconnect_live(false)
		return
	if status != StreamPeerTCP.STATUS_CONNECTED:
		return

	var available := live_peer.get_available_bytes()
	if available > 0:
		var packet := live_peer.get_data(available)
		if packet[0] != OK:
			_notify("Live socket read failed.", "error")
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

	if not live_running and not trace.is_empty():
		_set_cursor(playback_time_s)
		timeline.set_data(trace, playback_time_s, live_duration_s, true)


func _send_live_command(command: String, kind: String) -> bool:
	if live_peer == null or live_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return false
	var error := live_peer.put_data((command + "\n").to_utf8_buffer())
	if error != OK:
		_notify("Live socket write failed (%d)." % error, "error")
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
			track_end_x = maxf(8.0, float(scenario.get("v_desired_mps", 0.8)) *
				live_duration_s * 1.2 + 3.0)
			_create_track()
		live_profile_verified = bool(message.get("profile_verified", false))
		profile_verified = live_profile_verified
		line_dt_s = float(message.get("line_dt_s", 0.01))
		motor_dt_s = float(message.get("motor_dt_s", 0.002))
		live_control_dt_s = maxf(line_dt_s, 0.001)
		sensor_count = clampi(int(message.get("sensor_count", 8)), 1, 16)
		has_manifest = true
		live_ready = true
		live_reset_requested = false
		_send_live_command("RESET", "reset")
		_refresh_run_identity()
		_refresh_empty_state()
		_update_status("connecting", "handshake ok")
		_notify("C live service ready · fixed step %0.1f ms" % (live_control_dt_s * 1000.0), "ok")
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
		_create_track()
		_update_car({"x": 0.0, "y": 0.0, "theta": 0.0})
		_update_sample_ui({})
		_set_dashboard_trace(trace, 0.0, live_duration_s, true)
		time_label.text = "0.000"
		time_total_label.text = "/ %0.3f s" % live_duration_s
		_refresh_run_identity()
		_refresh_empty_state()
		_update_status("live", "%d Hz" % int(round(1.0 / maxf(live_control_dt_s, 0.0001))))
		_update_playback_buttons()
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
		if trace.size() == 1 or trace.size() % 50 == 0:
			_refresh_run_identity()
		_update_car(sample)
		_update_sample_ui(sample)
		_set_dashboard_trace(trace, playback_time_s, live_duration_s)
		time_label.text = "%0.3f" % playback_time_s
		time_total_label.text = "/ %0.3f s" % live_duration_s
		if _now_ms() - live_last_state_ms > 250:
			live_last_state_ms = _now_ms()
			_add_trajectory()
		if bool(sample.get("done", false)):
			live_running = false
			_update_playback_buttons()
			_notify("Live run complete · %d samples" % trace.size(), "ok")
	elif event == "done":
		live_running = false
		live_outstanding_steps = 0
		_update_playback_buttons()
	elif event == "error":
		live_request_pending = false
		live_pending_kind = ""
		live_running = false
		_notify("C simulator: %s" % str(message.get("message", "protocol error")), "error")
		_update_playback_buttons()


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


func _now_ms() -> int:
	return Time.get_ticks_msec()


# ============================================================== 3D helpers

func _update_car(sample: Dictionary) -> void:
	if not is_instance_valid(car):
		return
	var sim_x := float(sample.get("x", 0.0))
	var sim_y := float(sample.get("y", 0.0))
	car.position = _world_point(sim_x, sim_y)
	car.rotation.y = float(sample.get("theta", 0.0))
	for index in range(mini(sensor_nodes.size(), sensor_materials.size())):
		var suffix := "%02d" % index
		var color := Color("171c18")
		var energy := 0.0
		if sample.has("gray_value_" + suffix):
			var exists := int(sample["gray_exist_" + suffix]) != 0
			var valid := int(sample["gray_valid_" + suffix]) != 0
			if not exists:
				color = Color("141814")
			elif not valid:
				color = Ui.ACCENT
				energy = 1.8
			else:
				var response := clampf((float(sample["gray_value_" + suffix]) + 1.0) * 0.5, 0.0, 1.0)
				color = Color("1d241f").lerp(Ui.MINT, response)
				energy = 0.5 + response * 1.8
		elif int(sample.get("line_found", 0)) != 0:
			color = Ui.MINT
			energy = 1.2
		var material := sensor_materials[index]
		material.albedo_color = color
		material.emission = color
		material.emission_energy_multiplier = energy


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


func _vrule(height: float) -> ColorRect:
	var rule := ColorRect.new()
	rule.color = Color(Ui.LINE, 0.9)
	rule.custom_minimum_size = Vector2(1, height)
	rule.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	rule.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return rule


func _material(tint: Color, metal: float, rough: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = tint
	material.metallic = metal
	material.roughness = rough
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	return material
