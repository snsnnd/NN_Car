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
const LIVE_CHART_REFRESH_MS := 50
const LIVE_TRAJECTORY_REFRESH_MS := 1000
const IMMEDIATE_FAULT_MASK := 0x9C1
const MAX_LIVE_COMMAND_BYTES := 4094
const MAX_MAP_JSON_BYTES := 16 * 1024 * 1024

const FAULT_NAMES := {
	0: "急停", 1: "控制指令超时", 2: "车轮反馈超时", 3: "电机指令超时",
	4: "速度无效", 5: "超速", 6: "严重超速", 7: "欠压",
	8: "过压", 9: "电压过低", 10: "温度警告", 11: "温度过高",
	12: "神经网络故障", 13: "编码器故障", 14: "通信故障", 15: "循线丢失", 16: "曲率过大"
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
var active_map: Dictionary = {}
var selected_map_path := ""
var selected_map_json := ""
var map_bounds := Rect2()
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
var run_title := "暂无运行"
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
var live_deferred_command := ""
var live_deferred_kind := ""
var live_deferred_scenario: Dictionary = {}
var live_deferred_map_path := ""
var live_deferred_map_json := ""
var live_deferred_payload := PackedByteArray()
var live_pending_scenario: Dictionary = {}
var live_pending_map_path := ""
var live_pending_map_json := ""
var live_pending_payload := PackedByteArray()
var live_outstanding_steps := 0
var live_mode := false
var live_ready := false
var live_running := false
var live_accumulator := 0.0
var live_control_dt_s := 0.01
var live_duration_s := 60.0
var live_profile_verified := false
var live_scenario: Dictionary = {}
var live_controller := "unknown"
var live_pending_controller := ""
var live_last_state_ms := 0
var live_last_chart_ms := 0

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
var run_title_label: Label
var run_sub_label: Label
var status_pill: PanelContainer
var status_dot: Panel
var status_text: Label
var controller_chip: PanelContainer
var controller_chip_label: Label
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
var plot_dashboard
var timeline
var time_label: Label
var time_total_label: Label
var play_button: Button
var reset_button: Button
var step_back_button: Button
var step_forward_button: Button
var live_button: Button
var controller_option: OptionButton
var save_button: Button
var chart_button: Button
var map_button: Button
var parameter_button: Button
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
var map_dialog: FileDialog
var save_dialog: FileDialog
var parameter_window: Window
var parameter_fields: Dictionary = {}
var last_faults := -1
var last_latched := -1
var last_mode := -1
var last_stop := -1


func _ready() -> void:
	get_tree().root.gui_embed_subwindows = false
	get_window().min_size = Vector2i(1240, 800)
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
	_build_parameter_window()
	_set_camera_mode("follow", true)
	_update_car({"x": 0.0, "y": 0.0, "theta": 0.0})
	_refresh_run_identity()
	_update_sample_ui({})
	_refresh_empty_state()
	_update_status("offline", "待机")
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

	column.add_child(_build_dock())

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
	var sub := Ui.label("循迹小车仿真台", 9, Ui.FAINT)
	sub.mouse_filter = Control.MOUSE_FILTER_IGNORE
	brand_text.add_child(sub)

	bar.add_child(_vrule(22))

	var identity := VBoxContainer.new()
	identity.add_theme_constant_override("separation", 0)
	identity.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	identity.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(identity)
	run_title_label = Ui.strong_label("暂无运行", 13, Ui.TEXT)
	run_title_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	identity.add_child(run_title_label)
	run_sub_label = Ui.label("连接实时仿真或打开 CSV 记录", 9, Ui.MUTED)
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
	status_text = Ui.mono_label("未连接", 10, Ui.MUTED)
	status_text.mouse_filter = Control.MOUSE_FILTER_IGNORE
	status_row.add_child(status_text)

	controller_chip = PanelContainer.new()
	controller_chip.add_theme_stylebox_override("panel", _status_style(Ui.MUTED))
	controller_chip.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bar.add_child(controller_chip)
	controller_chip_label = Ui.mono_label("寻迹 · 未连接", 10, Ui.MUTED)
	controller_chip_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	controller_chip.add_child(controller_chip_label)

	profile_chip = PanelContainer.new()
	profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.ACCENT, 0.08),
		Color(Ui.ACCENT, 0.45), 4, 1, 8.0, 3.0))
	profile_chip.tooltip_text = "车辆与传感器参数尚未经过硬件标定"
	profile_chip.mouse_filter = Control.MOUSE_FILTER_STOP
	bar.add_child(profile_chip)
	var profile_row := HBoxContainer.new()
	profile_row.add_theme_constant_override("separation", 6)
	profile_row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	profile_chip.add_child(profile_row)
	profile_row.add_child(Ui.status_dot(Ui.ACCENT, 5.0))
	profile_chip_label = Ui.label("模型未验证", 9, Ui.ACCENT)
	profile_chip_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	profile_row.add_child(profile_chip_label)

	var help := Ui.button("?", "quiet", 26.0)
	help.custom_minimum_size.x = 28.0
	help.tooltip_text = "查看操作说明与快捷键（? 或 F1）"
	help.pressed.connect(_toggle_help)
	bar.add_child(help)

	wrapper.add_child(Ui.divider(Color(Ui.LINE, 0.8)))
	return wrapper


func _build_rail() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.custom_minimum_size.x = 328
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

	content.add_child(Ui.section_header("运行", "来源"))
	rail_run_title = Ui.strong_label("暂无运行", 15, Ui.TEXT)
	content.add_child(rail_run_title)
	rail_run_sub = Ui.label("等待数据", 9, Ui.MUTED)
	content.add_child(rail_run_sub)

	var meta := GridContainer.new()
	meta.columns = 2
	meta.add_theme_constant_override("h_separation", 12)
	meta.add_theme_constant_override("v_separation", 7)
	meta.mouse_filter = Control.MOUSE_FILTER_IGNORE
	content.add_child(meta)
	for key in ["AMP", "LAMBDA", "SEED", "RAIL", "MAP"]:
		meta.add_child(_meta_cell(key))

	content.add_child(Ui.divider())
	content.add_child(Ui.section_header("关键指标", "当前采样"))

	var ey_block := VBoxContainer.new()
	ey_block.add_theme_constant_override("separation", 1)
	content.add_child(ey_block)
	ey_block.add_child(Ui.label("横向偏差", 8, Ui.MUTED))
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
	ey_state_label = Ui.label("无数据", 8, Ui.FAINT)
	ey_state_chip.add_child(ey_state_label)

	var twin := HBoxContainer.new()
	twin.add_theme_constant_override("separation", 14)
	content.add_child(twin)
	var heading_block := VBoxContainer.new()
	heading_block.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	heading_block.add_theme_constant_override("separation", 0)
	twin.add_child(heading_block)
	heading_block.add_child(Ui.label("航向偏差", 8, Ui.MUTED))
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
	speed_block.add_child(Ui.label("前进速度", 8, Ui.MUTED))
	var speed_row := HBoxContainer.new()
	speed_row.add_theme_constant_override("separation", 4)
	speed_block.add_child(speed_row)
	speed_value = Ui.mono_label("—", 16, Ui.MINT, true)
	speed_row.add_child(speed_value)
	speed_row.add_child(_unit_label("m/s"))
	speed_sub = Ui.label("目标 —", 8, Ui.FAINT)
	speed_block.add_child(speed_sub)

	content.add_child(Ui.divider())
	content.add_child(Ui.section_header("传感器阵列", "%02d 路" % sensor_count))
	sensor_strip = SensorStrip.new()
	sensor_strip.set_channel_count(sensor_count)
	content.add_child(sensor_strip)

	rail_column.add_child(Ui.divider())
	var safety_box := VBoxContainer.new()
	safety_box.add_theme_constant_override("separation", 6)
	safety_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	rail_column.add_child(safety_box)
	safety_box.add_child(Ui.section_header("安全状态", "故障监控"))

	var safety_row := HBoxContainer.new()
	safety_row.add_theme_constant_override("separation", 8)
	safety_box.add_child(safety_row)
	safety_status = Ui.strong_label("正常", 16, Ui.MINT)
	safety_status.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	safety_row.add_child(safety_status)
	var safety_spacer := Control.new()
	safety_spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	safety_row.add_child(safety_spacer)
	safety_mode_chip = PanelContainer.new()
	safety_mode_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.MINT, 0.07),
		Color(Ui.MINT, 0.45), 4, 1, 7.0, 2.0))
	safety_row.add_child(safety_mode_chip)
	safety_mode_label = Ui.label("循线", 8, Ui.MINT)
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
		var labels := {"RAIL": "电压", "DERATE": "降额", "RAMP": "缓升", "STOP": "停车"}
		cell.add_child(Ui.label(labels[key], 8, Ui.FAINT))
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
	var labels := {"AMP": "振幅", "LAMBDA": "波长", "SEED": "随机种子", "RAIL": "电压", "MAP": "地图"}
	cell.add_child(Ui.label(labels.get(key, key), 8, Ui.FAINT))
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
	var cam_hint := Ui.label("右键旋转 · 滚轮缩放 · 中键平移", 9, Ui.MUTED)
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
	var overline := Ui.mono_label("尚无运行记录", 9, Ui.ACCENT)
	empty_box.add_child(overline)
	empty_title = Ui.strong_label("载入记录或连接实时仿真", 18, Ui.TEXT)
	empty_box.add_child(empty_title)
	empty_body = Ui.label("回放 C 仿真生成的 CSV，或连接固定步长服务（127.0.0.1:37621）。", 11, Ui.MUTED)
	empty_body.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	empty_body.custom_minimum_size.x = 348
	empty_box.add_child(empty_body)
	var empty_actions := HBoxContainer.new()
	empty_actions.add_theme_constant_override("separation", 8)
	empty_box.add_child(empty_actions)
	empty_connect_button = Ui.button("连接实时", "primary", 32.0)
	empty_connect_button.custom_minimum_size.x = 94
	empty_connect_button.pressed.connect(_on_live_button)
	empty_actions.add_child(empty_connect_button)
	empty_open_button = Ui.button("打开 CSV", "ghost", 32.0)
	empty_open_button.custom_minimum_size.x = 82
	empty_open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	empty_actions.add_child(empty_open_button)
	empty_hint = Ui.mono_label("bash simulator/run_demo.sh", 8, Ui.FAINT)
	empty_box.add_child(empty_hint)
	return panel


func _build_dock() -> PanelContainer:
	var panel := PanelContainer.new()
	panel.custom_minimum_size.y = 126.0
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
	transport.custom_minimum_size.y = 34
	column.add_child(transport)

	play_button = Ui.button("播放", "primary", 34.0)
	play_button.custom_minimum_size.x = 78
	play_button.pressed.connect(_toggle_playback)
	transport.add_child(play_button)

	reset_button = Ui.button("重置", "ghost", 34.0)
	reset_button.custom_minimum_size.x = 58
	reset_button.tooltip_text = "重置到起点（R）"
	reset_button.pressed.connect(_reset_playback)
	transport.add_child(reset_button)

	step_back_button = Ui.button("-1s", "ghost", 34.0)
	step_back_button.custom_minimum_size.x = 40
	step_back_button.tooltip_text = "后退 1 秒（Shift+←）"
	step_back_button.pressed.connect(func() -> void: _nudge(-1.0))
	transport.add_child(step_back_button)

	step_forward_button = Ui.button("+1s", "ghost", 34.0)
	step_forward_button.custom_minimum_size.x = 40
	step_forward_button.tooltip_text = "前进 1 秒（Shift+→）"
	step_forward_button.pressed.connect(func() -> void: _nudge(1.0))
	transport.add_child(step_forward_button)

	var time_box := HBoxContainer.new()
	time_box.add_theme_constant_override("separation", 5)
	time_box.custom_minimum_size.x = 176
	time_box.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	transport.add_child(time_box)
	time_label = Ui.mono_label("0.000", 14, Ui.TEXT, true)
	time_box.add_child(time_label)
	time_total_label = Ui.mono_label("/ 0.000 s", 11, Ui.MUTED)
	time_box.add_child(time_total_label)

	var speed_group := ButtonGroup.new()
	speed_group.allow_unpress = false
	var speed_box := HBoxContainer.new()
	speed_box.add_theme_constant_override("separation", 2)
	transport.add_child(speed_box)
	for index in range(4):
		var factor: float = [0.5, 1.0, 2.0, 4.0][index]
		var button := Ui.toggle("%sx" % str(factor), 28.0)
		button.custom_minimum_size.x = 40
		button.button_group = speed_group
		button.button_pressed = is_equal_approx(factor, 1.0)
		button.pressed.connect(func() -> void: _set_playback_speed(factor))
		speed_box.add_child(button)
		speed_buttons.append(button)

	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	transport.add_child(spacer)

	var open_button := Ui.button("打开", "ghost", 34.0)
	open_button.custom_minimum_size.x = 60
	open_button.tooltip_text = "打开 C 仿真 CSV 记录（O）"
	open_button.pressed.connect(func() -> void: open_dialog.popup_centered_ratio(0.7))
	transport.add_child(open_button)

	map_button = Ui.button("地图", "ghost", 34.0)
	map_button.custom_minimum_size.x = 62
	map_button.tooltip_text = "打开并切换 JSON 地图（M）"
	map_button.pressed.connect(func() -> void: map_dialog.popup_centered_ratio(0.7))
	transport.add_child(map_button)

	parameter_button = Ui.button("参数", "ghost", 34.0)
	parameter_button.custom_minimum_size.x = 62
	parameter_button.tooltip_text = "在线调整实时仿真参数（P）"
	parameter_button.pressed.connect(_open_parameter_window)
	transport.add_child(parameter_button)

	live_button = Ui.button("实时", "ghost", 34.0)
	live_button.custom_minimum_size.x = 64
	live_button.tooltip_text = "连接 C 固定步长仿真服务（L）"
	live_button.pressed.connect(_on_live_button)
	transport.add_child(live_button)

	controller_option = OptionButton.new()
	controller_option.custom_minimum_size = Vector2(138, 34)
	controller_option.add_theme_font_size_override("font_size", Ui.scaled_font_size(10))
	controller_option.add_item("PID", 0)
	controller_option.set_item_metadata(0, "pid")
	controller_option.add_item("FP32 MLP", 1)
	controller_option.set_item_metadata(1, "mlp")
	controller_option.add_item("int8 MLP", 2)
	controller_option.set_item_metadata(2, "int8")
	controller_option.select(0)
	controller_option.disabled = true
	controller_option.tooltip_text = "切换 C 端寻迹控制器；切换后会重置运行"
	controller_option.item_selected.connect(_on_live_controller_selected)
	transport.add_child(controller_option)

	save_button = Ui.button("保存", "ghost", 34.0)
	save_button.custom_minimum_size.x = 58
	save_button.tooltip_text = "将本次运行保存为 CSV 和清单（S）"
	save_button.pressed.connect(func() -> void: save_dialog.popup_centered_ratio(0.7))
	transport.add_child(save_button)

	chart_button = Ui.button("图表", "ghost", 34.0)
	chart_button.custom_minimum_size.x = 68
	chart_button.tooltip_text = "在独立窗口中查看遥测曲线（C）"
	chart_button.pressed.connect(_open_plot_window)
	transport.add_child(chart_button)

	timeline = TimelineBar.new()
	timeline.custom_minimum_size.y = 30.0
	timeline.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	timeline.seek_requested.connect(_on_seek_requested)
	column.add_child(timeline)
	return panel


# ============================================================== 3D viewport

func _setup_3d_viewport() -> void:
	viewport = SubViewport.new()
	viewport.own_world_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	viewport.handle_input_locally = false
	viewport.msaa_3d = Viewport.MSAA_DISABLED
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
	if active_map.is_empty():
		_add_ribbon("TrackShoulder", 0.078, Color("20251f"), 0.0005)
		_add_ribbon("TrackLine", maxf(track_line_width_m * 0.5, 0.008), Color("b8b3a4"), 0.002)
		_add_track_ticks()
	else:
		_add_map_ribbon("TrackShoulder", 0.078, Color("20251f"), 0.0005)
		_add_map_ribbon("TrackLine", maxf(track_line_width_m * 0.5, 0.008),
			Color("b8b3a4"), 0.002)
	_add_trajectory()


func _add_map_ribbon(node_name: String, half_width: float, tint: Color, lift: float) -> void:
	var vertices := PackedVector3Array()
	var indices := PackedInt32Array()
	var segments: Array = active_map.get("segments", [])
	for segment in segments:
		if not segment is Array or segment.size() != 4:
			continue
		var start := _world_point(float(segment[0]), float(segment[1]), lift)
		var finish := _world_point(float(segment[2]), float(segment[3]), lift)
		var tangent := (finish - start).normalized()
		if tangent.length_squared() <= 0.000001:
			continue
		var normal := Vector3(-tangent.z, 0.0, tangent.x)
		var base := vertices.size()
		vertices.append(start + normal * half_width)
		vertices.append(start - normal * half_width)
		vertices.append(finish + normal * half_width)
		vertices.append(finish - normal * half_width)
		indices.append_array(PackedInt32Array([base, base + 2, base + 1,
			base + 1, base + 2, base + 3]))
	if vertices.is_empty():
		return
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
		var error_mm := float(row["lateral_error_mm"]) if row.has("lateral_error_mm") else \
			_track_reference(x, y).x * 1000.0
		error_mm = absf(error_mm)
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
	var lateral_m: float
	var heading: float
	if sample.has("lateral_error_mm") and sample.has("heading_error_deg"):
		lateral_m = float(sample["lateral_error_mm"]) * 0.001
		heading = deg_to_rad(float(sample["heading_error_deg"]))
	else:
		var reference := _track_reference(x, y)
		lateral_m = reference.x
		heading = reference.y
	var left_normal := Vector2(-sin(heading), cos(heading))
	var center := Vector2(x, y) - left_normal * lateral_m
	var start := _world_point(x, y, 0.016)
	var end := _world_point(center.x, center.y, 0.016)
	tether_immediate.clear_surfaces()
	tether_immediate.surface_begin(Mesh.PRIMITIVE_LINES, tether_material)
	tether_immediate.surface_add_vertex(start)
	tether_immediate.surface_add_vertex(end)
	var half_width_axis := Vector3(-cos(heading), 0.0, sin(heading)) * 0.012
	tether_immediate.surface_add_vertex(end + half_width_axis)
	tether_immediate.surface_add_vertex(end - half_width_axis)
	tether_immediate.surface_end()
	var error_mm := lateral_m * 1000.0
	tether_label.text = "偏差 %+0.1f 毫米" % error_mm
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
		var width := maxf(track_end_x - track_start_x, 1.0)
		var height := maxf(track_amplitude_m * 2.0, 0.5)
		if not active_map.is_empty():
			width = maxf(map_bounds.size.x, 0.1)
			height = maxf(map_bounds.size.y, 0.1)
		var aspect := maxf(float(viewport.size.x) / maxf(float(viewport.size.y), 1.0), 0.6)
		camera.size = maxf(maxf(height, width / aspect) * 1.1, 0.9) * top_zoom
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
	var width := maxf(track_end_x - track_start_x, 1.0)
	var height := maxf(track_amplitude_m * 2.0, 0.2)
	if not active_map.is_empty():
		width = maxf(map_bounds.size.x, 0.1)
		height = maxf(map_bounds.size.y, 0.1)
	var aspect := maxf(float(viewport.size.x) / maxf(float(viewport.size.y), 1.0), 0.6)
	var fov := deg_to_rad(camera.fov)
	var half_horizontal := atan(tan(fov * 0.5) * aspect)
	var horizontal_distance := (width * 0.5 * 1.18) / maxf(tan(half_horizontal), 0.01)
	var vertical_distance := (height * 0.5 * 1.18) / maxf(tan(fov * 0.5), 0.01)
	var distance := maxf(horizontal_distance, vertical_distance)
	return clampf(distance * 0.95 * track_zoom, 1.5, 90.0)


func _track_center() -> Vector3:
	if not active_map.is_empty():
		return _world_point(map_bounds.position.x + map_bounds.size.x * 0.5,
			map_bounds.position.y + map_bounds.size.y * 0.5)
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
		return "实时 / %s" % str(active_map.get("name", "会话")).to_upper()
	if trace.is_empty():
		return "暂无运行"
	if not has_manifest:
		return "轨道 / 无清单"
	if not active_map.is_empty():
		return "地图 / %s" % str(active_map.get("name", "赛道")).to_upper()
	if track_amplitude_m > 0.0001:
		return "S 弯赛道"
	return "直线赛道"


func _refresh_run_identity() -> void:
	run_title = _scenario_title()
	run_sample_count = trace.size()
	var source := "实时 C 仿真" if live_mode else (run_file_name if not run_file_name.is_empty() else "无来源")
	run_title_label.text = run_title
	var rate := int(round(1.0 / maxf(line_dt_s, 0.0001)))
	if live_mode:
		run_sub_label.text = "固定步长 · %d Hz · %.0f 秒窗口" % [rate, live_duration_s]
	elif trace.is_empty():
		run_sub_label.text = "连接实时仿真或打开 CSV 记录"
	else:
		run_sub_label.text = "%s · %d 个采样 · %d Hz" % [source, run_sample_count, rate]
	rail_run_title.text = run_file_name if not run_file_name.is_empty() else (
		"实时会话" if live_mode else "暂无运行")
	if live_mode:
		rail_run_sub.text = "C 固定步长 · %d 个采样" % run_sample_count if run_sample_count > 0 else "等待首个采样"
	elif not run_file_name.is_empty():
		rail_run_sub.text = "%s · %d 个采样" % [run_title, run_sample_count]
	else:
		rail_run_sub.text = "等待数据"

	var scenario := live_scenario if live_mode else run_scenario
	var amp := float(scenario.get("amp_m", track_amplitude_m))
	var wavelength := float(scenario.get("wavelength_m", track_wavelength_m))
	var seed := int(scenario.get("seed", 0))
	var rail := float(scenario.get("rail_v", 0.0))
	if has_manifest or live_mode:
		meta_labels["AMP"].text = "地图" if not active_map.is_empty() else (
			"直线" if absf(amp) < 0.0005 else "%0.0f 毫米" % (amp * 1000.0))
		meta_labels["LAMBDA"].text = "折线" if not active_map.is_empty() else "%0.2f 米" % wavelength
		meta_labels["SEED"].text = str(seed)
		meta_labels["RAIL"].text = "%0.1f V" % rail
		meta_labels["MAP"].text = str(active_map.get("name", "解析轨道"))
	else:
		meta_labels["AMP"].text = "—"
		meta_labels["LAMBDA"].text = "—"
		meta_labels["SEED"].text = "—"
		meta_labels["RAIL"].text = "—"
		meta_labels["MAP"].text = "—"

	if profile_verified:
		profile_chip_label.text = "模型已验证"
		profile_chip_label.add_theme_color_override("font_color", Ui.MINT)
		profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.MINT, 0.08),
			Color(Ui.MINT, 0.45), 4, 1, 8.0, 3.0))
		profile_chip.tooltip_text = "车辆与传感器参数已完成硬件标定"
	else:
		profile_chip_label.text = "模型未验证"
		profile_chip_label.add_theme_color_override("font_color", Ui.ACCENT)
		profile_chip.add_theme_stylebox_override("panel", Ui.flat(Color(Ui.ACCENT, 0.08),
			Color(Ui.ACCENT, 0.45), 4, 1, 8.0, 3.0))
		profile_chip.tooltip_text = "车辆与传感器参数尚未经过硬件标定"

	if live_mode:
		rail_run_title.tooltip_text = "实时连接 · 地图 %s · C 固定步长 %0.1f 毫秒 · 硬件标定：%s" % [
			str(active_map.get("name", "解析轨道")), live_control_dt_s * 1000.0,
			("是" if profile_verified else "否")]
	elif has_manifest:
		rail_run_title.tooltip_text = "已读取清单 · 线宽 %0.0f 毫米 · 随机种子 %d · 配置 %s v%s" % [
			track_line_width_m * 1000.0, int(run_scenario.get("seed", 0)), profile_name,
			profile_version]
	else:
		rail_run_title.tooltip_text = "未找到清单，按直线赛道处理"

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
	var label := "未连接"
	match kind:
		"replay":
			color = Ui.MUTED
			label = "回放"
		"live":
			color = Ui.ACCENT
			label = "实时"
		"connecting":
			color = Ui.ACCENT
			label = "连接中"
		"error":
			color = Ui.DANGER
			label = "连接错误"
		_:
			color = Ui.FAINT
			label = "未连接"
	status_text.text = label if detail.is_empty() else "%s  %s" % [label, detail]
	status_text.add_theme_color_override("font_color", Ui.TEXT_DIM if kind != "offline" else Ui.MUTED)
	status_dot.add_theme_stylebox_override("panel", Ui.flat(color, color, 4, 0))
	status_pill.add_theme_stylebox_override("panel", _status_style(color))
	_update_controller_controls()


func _controller_name(controller: String) -> String:
	match controller:
		"pid":
			return "PID"
		"mlp":
			return "FP32 MLP"
		"int8":
			return "int8 MLP"
		_:
			return "未连接"


func _controller_index(controller: String) -> int:
	match controller:
		"mlp":
			return 1
		"int8":
			return 2
		_:
			return 0


func _update_controller_controls() -> void:
	if not is_instance_valid(controller_chip_label):
		return
	var color := Ui.MUTED
	if not live_mode or live_controller == "unknown":
		controller_chip_label.text = "寻迹 · 未连接"
		controller_chip.tooltip_text = "连接实时 C 仿真后显示实际生效的寻迹控制器"
	else:
		color = Ui.ACCENT
		controller_chip_label.text = "寻迹 · %s" % _controller_name(live_controller)
		controller_chip.tooltip_text = "C 仿真确认的当前寻迹控制器：%s" % _controller_name(live_controller)
		if not live_pending_controller.is_empty():
			controller_chip_label.text = "切换 · %s→%s" % [
				_controller_name(live_controller), _controller_name(live_pending_controller)]
			controller_chip.tooltip_text = "等待 C 仿真确认切换到 %s；确认前仍以 %s 为准" % [
				_controller_name(live_pending_controller), _controller_name(live_controller)]
	controller_chip_label.add_theme_color_override("font_color", color)
	controller_chip.add_theme_stylebox_override("panel", _status_style(color))
	if is_instance_valid(controller_option):
		var selected_controller := live_pending_controller if not live_pending_controller.is_empty() \
			else live_controller
		controller_option.select(_controller_index(selected_controller))
		controller_option.disabled = not live_mode or not live_ready or \
			live_request_pending or not live_pending_controller.is_empty()


func _on_live_controller_selected(index: int) -> void:
	if not is_instance_valid(controller_option):
		return
	var requested := str(controller_option.get_item_metadata(index))
	if not live_mode or not live_ready or requested == live_controller:
		_update_controller_controls()
		return
	if not _queue_live_command("CTRL " + requested, "controller"):
		_update_controller_controls()
		return
	live_pending_controller = requested
	_update_playback_buttons()
	_update_controller_controls()
	_notify("正在等待当前步完成，并由 C 仿真切换到 %s。" % _controller_name(requested), "info")


func _status_style(color: Color) -> StyleBoxFlat:
	return Ui.flat(Color(color, 0.05), Color(color, 0.38), 4, 1, 8.0, 3.0)


# ============================================================== sample UI

func _update_sample_ui(sample: Dictionary) -> void:
	if trace.is_empty() or sample.is_empty():
		ey_value.text = "—"
		ey_value.add_theme_color_override("font_color", Ui.TEXT)
		_set_state_chip("无数据", Ui.FAINT)
		heading_value.text = "—"
		speed_value.text = "—"
		speed_sub.text = "目标 —"
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

	var lateral := float(sample["lateral_error_mm"]) if sample.has("lateral_error_mm") else \
		_track_reference(float(sample.x), float(sample.y)).x * 1000.0
	var line_found := int(sample.get("line_found", 0)) != 0
	ey_value.text = "%+0.1f" % lateral
	var state_color := Ui.FAINT
	var state_text := "无数据"
	if line_found:
		if absf(lateral) <= track_line_width_m * 500.0 + 1.0:
			state_text = "在线上"
			state_color = Ui.MINT
			ey_value.add_theme_color_override("font_color", Ui.TEXT)
		else:
			state_text = "偏离轨道"
			state_color = Ui.ACCENT
			ey_value.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		state_text = "循线丢失"
		state_color = Ui.DANGER
		ey_value.add_theme_color_override("font_color", Ui.DANGER)
	_set_state_chip(state_text, state_color)

	var heading: float
	if sample.has("heading_error_deg"):
		heading = float(sample["heading_error_deg"])
	else:
		heading = rad_to_deg(wrapf(float(sample.theta) -
			_track_reference(float(sample.x), float(sample.y)).y, -PI, PI))
	heading_value.text = "%+0.2f" % heading
	var target_speed := float(sample.get("v_target", 0.0))
	var actual_speed := float(sample.get("speed_actual_mps",
		0.5 * (float(sample.get("wheel_v_l", 0.0)) + float(sample.get("wheel_v_r", 0.0)))))
	speed_value.text = "%0.3f" % actual_speed
	speed_sub.text = "目标 %0.3f 米/秒" % target_speed

	sensor_strip.set_sample(sample)

	var faults := int(sample.get("safety_faults", 0))
	var latched := int(sample.get("safety_latched", 0))
	var stop := int(sample.get("safety_stop", 0))
	var mode := int(sample.get("mode", 0))
	var derate := float(sample.get("safety_derate", 1.0))
	var ramp := float(sample.get("safety_ramp", 1.0))
	if stop != 0:
		safety_status.text = "已停车"
		safety_status.add_theme_color_override("font_color", Ui.DANGER)
	elif faults != 0 and (faults & IMMEDIATE_FAULT_MASK) != 0:
		safety_status.text = "请求停车"
		safety_status.add_theme_color_override("font_color", Ui.DANGER)
	elif faults != 0:
		safety_status.text = "输出降额" if (derate < 0.999 or ramp < 0.999) else "故障"
		safety_status.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		safety_status.text = "正常"
		safety_status.add_theme_color_override("font_color", Ui.MINT)

	safety_mode_label.text = "恢复" if mode == 1 else "循线"
	var mode_color := Ui.ACCENT if mode == 1 else Ui.MINT
	safety_mode_label.add_theme_color_override("font_color", mode_color)
	safety_mode_chip.add_theme_stylebox_override("panel", Ui.flat(Color(mode_color, 0.07),
		Color(mode_color, 0.45), 4, 1, 7.0, 2.0))

	var rail_value := "—"
	if sample.has("v_rail"):
		rail_value = "%0.1f V" % float(sample.v_rail)
		if int(sample.get("v_rail_valid", 1)) == 0:
			rail_value = "无效"
	safety_values["RAIL"].text = rail_value
	safety_values["DERATE"].text = "%0.0f%%" % (100.0 * float(sample.get("safety_derate", 1.0)))
	safety_values["RAMP"].text = "%0.0f%%" % (100.0 * float(sample.get("safety_ramp", 1.0)))
	safety_values["STOP"].text = "是" if stop != 0 else "否"
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
		fault_flow.add_child(Ui.chip(str(FAULT_NAMES.get(bit, "故障 %d" % bit)), color))
	if latched != 0:
		any = true
		fault_flow.add_child(Ui.chip("已锁存", Ui.DANGER, true))
	if stop != 0:
		any = true
		fault_flow.add_child(Ui.chip("输出已停止", Ui.DANGER, true))
	if not any:
		fault_flow.add_child(Ui.label("当前无故障", 9, Ui.FAINT))


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
	if plot_window.visible:
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
	if not sample.has("lateral_error_mm") or not sample.has("heading_error_deg"):
		var reference := _track_reference(x, y)
		if not sample.has("lateral_error_mm"):
			sample["lateral_error_mm"] = reference.x * 1000.0
		if not sample.has("heading_error_deg"):
			sample["heading_error_deg"] = rad_to_deg(wrapf(theta - reference.y, -PI, PI))
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
		_queue_live_command("RESET", "reset")
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
		play_button.text = "暂停" if live_running else ("继续" if live_ready else "连接")
	else:
		play_button.text = "暂停" if is_playing else "播放"
	reset_button.disabled = trace.is_empty()
	step_back_button.disabled = trace.is_empty()
	step_forward_button.disabled = trace.is_empty()
	save_button.disabled = trace.is_empty()
	chart_button.disabled = trace.is_empty()
	live_button.text = "断开" if live_mode else "实时"
	if live_mode:
		live_button.add_theme_stylebox_override("normal", Ui.flat(Color(Ui.ACCENT, 0.13),
			Color(Ui.ACCENT, 0.55), 6, 1, 10.0, 5.0))
		live_button.add_theme_color_override("font_color", Ui.ACCENT)
	else:
		live_button.remove_theme_stylebox_override("normal")
		live_button.remove_theme_color_override("font_color")
	_update_controller_controls()


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


func _set_chart_trace(rows: Array[Dictionary], cursor_s: float, duration_s: float,
		new_trace: bool = false) -> void:
	var now := _now_ms()
	if live_mode and not new_trace and now - live_last_chart_ms < LIVE_CHART_REFRESH_MS:
		return
	live_last_chart_ms = now
	timeline.set_data(rows, cursor_s, duration_s, live_mode, new_trace)
	plot_dashboard.track_half_width_mm = track_line_width_m * 500.0
	if new_trace or plot_window.visible:
		plot_dashboard.set_trace(rows, cursor_s, duration_s, new_trace)
	_update_zoom_label()


func _update_zoom_label() -> void:
	if plot_window.visible and is_instance_valid(plot_zoom_label) and is_instance_valid(plot_dashboard):
		var plot_label: String = plot_dashboard.get_zoom_label()
		plot_zoom_label.text = "" if plot_label == "FIT" else plot_label
		plot_zoom_label.visible = plot_label != "FIT"


func _fit_plot_views() -> void:
	plot_dashboard.fit_trace()
	_update_zoom_label()


func _open_plot_window() -> void:
	if not is_instance_valid(plot_window):
		return
	live_last_chart_ms = 0
	plot_dashboard.set_trace(trace, playback_time_s, _duration_s())
	plot_window.popup_centered(Vector2i(1280, 820))
	plot_window.grab_focus()


func _build_plot_window() -> void:
	plot_window = Window.new()
	plot_window.title = "NN_CAR / 遥测曲线"
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
	title_box.add_child(Ui.strong_label("遥测曲线", 13, Ui.TEXT))
	title_box.add_child(Ui.label("与运行时间游标同步", 9, Ui.MUTED))
	var zoom_out_button := Ui.button("−", "ghost", 26.0)
	zoom_out_button.custom_minimum_size.x = 30
	zoom_out_button.tooltip_text = "缩小时间范围"
	zoom_out_button.pressed.connect(func() -> void: plot_dashboard.zoom_out())
	bar.add_child(zoom_out_button)
	var zoom_in_button := Ui.button("+", "ghost", 26.0)
	zoom_in_button.custom_minimum_size.x = 30
	zoom_in_button.tooltip_text = "放大时间范围"
	zoom_in_button.pressed.connect(func() -> void: plot_dashboard.zoom_in())
	bar.add_child(zoom_in_button)
	var fit_button := Ui.button("适配", "ghost", 26.0)
	fit_button.custom_minimum_size.x = 40
	fit_button.pressed.connect(_fit_plot_views)
	bar.add_child(fit_button)
	plot_zoom_label = Ui.mono_label("适配", 9, Ui.MUTED)
	plot_zoom_label.visible = false
	bar.add_child(plot_zoom_label)
	bar.add_child(Ui.label("滚轮缩放 · 拖动定位 · 中键平移", 9, Ui.MUTED))

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
	plot_dashboard.view_changed.connect(_update_zoom_label)
	plot_dashboard.seek_requested.connect(_on_seek_requested)
	plot_margin.add_child(plot_dashboard)
	plot_dashboard.track_half_width_mm = track_line_width_m * 500.0


# ============================================================== dialogs / overlays

func _build_dialogs() -> void:
	open_dialog = FileDialog.new()
	open_dialog.title = "打开运行记录"
	open_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	open_dialog.access = FileDialog.ACCESS_FILESYSTEM
	open_dialog.filters = PackedStringArray(["*.csv ; NN_Car 仿真记录"])
	open_dialog.use_native_dialog = true
	open_dialog.file_selected.connect(_load_trace)
	root_ui.add_child(open_dialog)

	map_dialog = FileDialog.new()
	map_dialog.title = "打开赛道地图"
	map_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	map_dialog.access = FileDialog.ACCESS_FILESYSTEM
	map_dialog.filters = PackedStringArray(["*.json ; NN_Car JSON 地图"])
	map_dialog.use_native_dialog = true
	map_dialog.file_selected.connect(_load_map_file)
	root_ui.add_child(map_dialog)

	save_dialog = FileDialog.new()
	save_dialog.title = "保存运行记录"
	save_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	save_dialog.access = FileDialog.ACCESS_FILESYSTEM
	save_dialog.filters = PackedStringArray(["*.csv ; NN_Car 仿真记录"])
	save_dialog.use_native_dialog = true
	save_dialog.file_selected.connect(_save_trace)
	root_ui.add_child(save_dialog)


func _build_parameter_window() -> void:
	parameter_window = Window.new()
	parameter_window.title = "实时仿真参数"
	parameter_window.size = Vector2i(520, 610)
	parameter_window.min_size = Vector2i(480, 560)
	parameter_window.visible = false
	parameter_window.exclusive = true
	parameter_window.theme = Ui.theme()
	parameter_window.close_requested.connect(func() -> void: parameter_window.hide())
	root_ui.add_child(parameter_window)

	var margin := MarginContainer.new()
	margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", 22)
	margin.add_theme_constant_override("margin_right", 22)
	margin.add_theme_constant_override("margin_top", 18)
	margin.add_theme_constant_override("margin_bottom", 18)
	parameter_window.add_child(margin)

	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 10)
	margin.add_child(column)
	column.add_child(Ui.strong_label("场景参数", 18, Ui.TEXT))
	var description := Ui.label("应用后会暂停当前步进、更新 C 仿真并从起点重新运行。", 11, Ui.MUTED)
	description.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	column.add_child(description)
	column.add_child(Ui.divider())

	var specs: Array[Dictionary] = [
		{"key": "v_desired_mps", "label": "目标速度（米/秒）", "default": 0.8},
		{"key": "offset_m", "label": "起始横向偏移（米）", "default": 0.0},
		{"key": "heading_deg", "label": "起始航向偏差（度）", "default": 0.0},
		{"key": "amp_m", "label": "轨迹振幅（米）", "default": 0.08},
		{"key": "wavelength_m", "label": "轨迹波长（米）", "default": 3.0},
		{"key": "duration_s", "label": "仿真时长（秒）", "default": 60.0},
		{"key": "seed", "label": "随机种子", "default": 23},
		{"key": "rail_v", "label": "供电电压（伏）", "default": 12.0}
	]
	for spec in specs:
		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 12)
		row.custom_minimum_size.y = 38
		column.add_child(row)
		var label := Ui.label(str(spec["label"]), 12, Ui.TEXT_DIM)
		label.custom_minimum_size.x = 220
		label.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		row.add_child(label)
		var field := LineEdit.new()
		field.custom_minimum_size.x = 180
		field.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		field.alignment = HORIZONTAL_ALIGNMENT_RIGHT
		field.add_theme_font_override("font", Ui.MONO)
		field.add_theme_font_size_override("font_size", Ui.scaled_font_size(12))
		field.text = str(spec["default"])
		row.add_child(field)
		parameter_fields[str(spec["key"])] = field

	var spacer := Control.new()
	spacer.size_flags_vertical = Control.SIZE_EXPAND_FILL
	column.add_child(spacer)
	column.add_child(Ui.divider())
	var actions := HBoxContainer.new()
	actions.alignment = BoxContainer.ALIGNMENT_END
	actions.add_theme_constant_override("separation", 10)
	column.add_child(actions)
	var cancel := Ui.button("取消", "ghost", 38.0)
	cancel.custom_minimum_size.x = 92
	cancel.pressed.connect(func() -> void: parameter_window.hide())
	actions.add_child(cancel)
	var apply := Ui.button("应用并重置", "primary", 38.0)
	apply.custom_minimum_size.x = 142
	apply.pressed.connect(_apply_live_parameters)
	actions.add_child(apply)


func _open_parameter_window() -> void:
	if not live_mode or not live_ready:
		_notify("请先连接实时仿真，再在线调整参数。", "info")
		return
	for key in parameter_fields:
		var fallback: Variant = _default_live_scenario().get(key, 0.0)
		parameter_fields[key].text = str(live_scenario.get(key, fallback))
	parameter_window.popup_centered()


func _default_live_scenario() -> Dictionary:
	return {
		"duration_s": 60.0,
		"v_desired_mps": 0.8,
		"offset_m": 0.0,
		"heading_deg": 0.0,
		"amp_m": 0.08,
		"wavelength_m": 3.0,
		"rail_v": 12.0,
		"seed": 23
	}


func _apply_live_parameters() -> void:
	if not live_mode or not live_ready:
		_notify("实时仿真当前不可用，参数没有应用。", "error")
		parameter_window.hide()
		return
	var values: Dictionary = {}
	for key in parameter_fields:
		var field: LineEdit = parameter_fields[key]
		var raw: String = field.text.strip_edges()
		if key == "seed":
			if not raw.is_valid_int():
				_notify("随机种子必须是 0 到 4294967295 的整数。", "error")
				return
			var seed_value: int = raw.to_int()
			if seed_value < 0 or seed_value > 4294967295:
				_notify("随机种子必须是 0 到 4294967295 的整数。", "error")
				return
			values[key] = seed_value
		else:
			if not raw.is_valid_float():
				_notify("请检查参数输入；数值字段都需要填写有效数字。", "error")
				return
			var number: float = raw.to_float()
			if not is_finite(number):
				_notify("参数必须是有限数值。", "error")
				return
			values[key] = number
	if float(values["v_desired_mps"]) < 0.0 or \
		float(values["wavelength_m"]) <= 0.0 or \
		float(values["duration_s"]) <= 0.0 or float(values["rail_v"]) <= 0.0:
		_notify("目标速度不能为负，波长、时长和供电电压必须大于 0。", "error")
		return
	var command_values := PackedStringArray([
		str(values["seed"]), str(values["v_desired_mps"]), str(values["offset_m"]),
		str(values["heading_deg"]), str(values["amp_m"]), str(values["wavelength_m"]),
		str(values["duration_s"]), str(values["rail_v"])
	])
	if _queue_live_command("RESET " + " ".join(command_values), "parameters", values):
		parameter_window.hide()
		_notify("正在等待当前步完成后应用参数并重新开始。", "info")


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
	head_box.add_child(Ui.mono_label("NN·CAR / 操作说明", 9, Ui.ACCENT))
	head_box.add_child(Ui.strong_label("键盘与鼠标操作", 18, Ui.TEXT))
	var close := Ui.button("关闭", "ghost", 28.0)
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

	left.add_child(Ui.section_header("播放控制", ""))
	for row in [
		["空格", "播放 / 暂停"],
		["R", "游标归零"],
		["← / →", "步进 0.1 秒"],
		["Shift + ← / →", "步进 1 秒"],
		["0.5x–4x", "播放速度"]
	]:
		left.add_child(_shortcut_row(row[0], row[1]))
	left.add_child(Ui.divider())
	left.add_child(Ui.section_header("运行记录", ""))
	for row in [
		["O", "打开 CSV 记录"], ["M", "打开 / 切换 JSON 地图"],
		["P", "在线调整实时仿真参数"], ["L", "连接 / 断开实时仿真"],
		["S", "保存运行记录"], ["C", "独立图表窗口"]
	]:
		left.add_child(_shortcut_row(row[0], row[1]))

	right.add_child(Ui.section_header("视图操作", ""))
	for row in [
		["F", "跟随小车"],
		["V", "适配整条赛道"],
		["T", "赛道俯视图"],
		["0", "重置当前视角"],
		["右键拖动", "旋转镜头"],
		["中键拖动", "平移镜头"],
		["滚轮", "缩放镜头；图表打开时缩放时间"],
		["F11", "切换全屏"]
	]:
		right.add_child(_shortcut_row(row[0], row[1]))
	right.add_child(Ui.divider())
	right.add_child(Ui.section_header("数据说明", ""))
	right.add_child(Ui.label("图表共用一个时间游标。拖动曲线可定位，悬停可查看采样值。", 9, Ui.MUTED))
	right.add_child(Ui.label("故障区间会在时间轴和图表背景中标色。", 9, Ui.MUTED))


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
		empty_title.text = "正在连接 C 仿真服务"
		empty_body.text = "等待 127.0.0.1:%d 上的固定步长仿真器。收到重置响应后开始运行。" % live_port
		empty_hint.text = "固定步长  %.0f 毫秒" % (live_control_dt_s * 1000.0)
		empty_connect_button.text = "断开连接"
		empty_open_button.disabled = true
	else:
		empty_title.text = "载入记录或连接实时仿真"
		empty_body.text = "回放 C 仿真生成的 CSV，或连接固定步长服务（127.0.0.1:%d）。" % live_port
		empty_hint.text = "bash simulator/run_demo.sh"
		empty_connect_button.text = "连接实时"
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
	if open_dialog.visible or map_dialog.visible or save_dialog.visible or parameter_window.visible:
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
		KEY_M:
			map_dialog.popup_centered_ratio(0.7)
			get_viewport().set_input_as_handled()
		KEY_P:
			_open_parameter_window()
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
	if help_overlay.visible or open_dialog.visible or map_dialog.visible or save_dialog.visible or \
		parameter_window.visible:
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
		_notify("无法打开 CSV（错误码 %d）。" % FileAccess.get_open_error(), "error")
		return

	var header := file.get_csv_line()
	var column_index: Dictionary = {}
	for index in range(header.size()):
		column_index[header[index]] = index
	for column in REQUIRED_COLUMNS:
		if not column_index.has(column):
			_notify("CSV 格式不支持：缺少字段“%s”。" % column, "error")
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
		_notify("CSV 中有效采样不足两条。", "error")
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
	_set_chart_trace(trace, 0.0, _duration_s(), true)
	time_label.text = "0.000"
	time_total_label.text = "/ %0.3f s" % _duration_s()
	_refresh_run_identity()
	_refresh_empty_state()
	_update_status("replay", "%d Hz" % int(round(1.0 / maxf(line_dt_s, 0.0001))))
	_update_playback_buttons()
	_set_help_visible(false)
	_notify("已从 %s 载入 %d 条采样。" % [run_file_name, trace.size()], "ok")


func _load_manifest(csv_path: String) -> void:
	active_map = {}
	map_bounds = Rect2()
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
	_set_active_map(manifest.get("map", {}))
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
		var y := float(sample.get("y", 0.0))
		var theta := float(sample.get("theta", 0.0))
		var reference := _track_reference(x, y)
		sample["lateral_error_mm"] = reference.x * 1000.0
		sample["heading_error_deg"] = rad_to_deg(wrapf(theta - reference.y, -PI, PI))
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
		_notify("没有可保存的采样。", "error")
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
		_notify("无法保存 CSV（错误码 %d）。" % FileAccess.get_open_error(), "error")
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
	manifest["map"] = active_map.duplicate(true) if not active_map.is_empty() else null
	var manifest_file := FileAccess.open(csv_path.get_basename() + ".manifest.json", FileAccess.WRITE)
	if manifest_file != null:
		manifest_file.store_string(JSON.stringify(manifest, "\t"))
		manifest_file.close()
	_notify("已将 %d 条采样保存到 %s。" % [trace.size(), csv_path.get_file()], "ok")


func _load_map_file(path: String) -> void:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		_notify("无法读取地图文件（错误码 %d）。" % FileAccess.get_open_error(), "error")
		return
	if file.get_length() <= 0 or file.get_length() > MAX_MAP_JSON_BYTES:
		file.close()
		_notify("地图 JSON 不能为空且不能超过 16 MiB。", "error")
		return
	var parsed: Variant = JSON.parse_string(file.get_as_text())
	file.close()
	var map_data := _map_document_to_viewer(parsed)
	if map_data.is_empty():
		_notify("地图 JSON 无效或不符合 NN_Car 地图格式。", "error")
		return
	var map_json := JSON.stringify(parsed)
	var map_payload := map_json.to_utf8_buffer()
	if map_payload.size() > MAX_MAP_JSON_BYTES:
		_notify("地图 JSON 超过 16 MiB，无法加载。", "error")
		return
	if live_mode:
		var command := "MAPDATA %d" % map_payload.size()
		if _queue_live_command(command, "map", {}, path, map_payload, map_json):
			_notify("地图已校验，等待 C 仿真完成当前步后同步切换。", "info")
		return

	selected_map_path = path
	selected_map_json = map_json
	_set_active_map(map_data)
	_create_track()
	_set_camera_mode(cam_mode, true)
	if not trace.is_empty():
		_update_playback()
	else:
		_update_car({
			"x": float(active_map.get("start_x_m", 0.0)),
			"y": float(active_map.get("start_y_m", 0.0)),
			"theta": float(active_map.get("start_theta_rad", 0.0))
		})
	_refresh_run_identity()
	_refresh_empty_state()
	_notify("已打开地图：%s。" % str(active_map.get("name", "地图")), "ok")


func _map_document_to_viewer(document: Variant) -> Dictionary:
	if not document is Dictionary:
		return {}
	var map_name := str(document.get("name", ""))
	var version := str(document.get("map_version", ""))
	var name_pattern := RegEx.new()
	var version_pattern := RegEx.new()
	if name_pattern.compile("^[A-Za-z_][A-Za-z0-9_-]*$") != OK or \
		version_pattern.compile("^[0-9]+\\.[0-9]+\\.[0-9]+$") != OK or \
		name_pattern.search(map_name) == null or version_pattern.search(version) == null:
		return {}
	var units: Variant = document.get("units_per_meter", null)
	if not _is_json_number(units) or not is_finite(float(units)) or float(units) <= 0.0:
		return {}
	var line_width: Variant = document.get("line_width_m", 0.02)
	if not _is_json_number(line_width) or not is_finite(float(line_width)) or \
		float(line_width) <= 0.0 or float(line_width) > 1.0:
		return {}
	var raw_nodes: Variant = document.get("nodes", null)
	var raw_edges: Variant = document.get("edges", null)
	if not raw_nodes is Array or not raw_edges is Array or raw_nodes.size() < 2 or \
		raw_nodes.size() > 8192 or raw_edges.is_empty() or raw_edges.size() > 4096:
		return {}
	var points: Dictionary = {}
	for node in raw_nodes:
		if not node is Array or node.size() != 3 or not _is_json_integer(node[0]) or \
				not _is_json_number(node[1]) or not _is_json_number(node[2]):
			return {}
		var node_id := int(node[0])
		var point := Vector2(float(node[1]) / float(units), float(node[2]) / float(units))
		if points.has(node_id) or not is_finite(point.x) or not is_finite(point.y):
			return {}
		points[node_id] = point

	var segments: Array = []
	var adjacency: Dictionary = {}
	for edge in raw_edges:
		if not edge is Array or edge.size() != 2 or not _is_json_integer(edge[0]) or \
			not _is_json_integer(edge[1]):
			return {}
		var from_id := int(edge[0])
		var to_id := int(edge[1])
		if not points.has(from_id) or not points.has(to_id):
			return {}
		var from_point: Vector2 = points[from_id]
		var to_point: Vector2 = points[to_id]
		if from_point.distance_squared_to(to_point) <= 0.000000000001:
			return {}
		segments.append([from_point.x, from_point.y, to_point.x, to_point.y])
		adjacency[Vector2i(from_id, to_id)] = true
		adjacency[Vector2i(to_id, from_id)] = true

	var start: Variant = document.get("start", null)
	if not start is Dictionary or not _is_json_integer(start.get("node", null)):
		return {}
	var start_id := int(start["node"])
	if not points.has(start_id):
		return {}
	var start_point: Vector2 = points[start_id]
	var start_heading := 0.0
	if start.has("toward"):
		if not _is_json_integer(start["toward"]):
			return {}
		var toward_id := int(start["toward"])
		if not points.has(toward_id) or not adjacency.has(Vector2i(start_id, toward_id)):
			return {}
		var toward_point: Vector2 = points[toward_id]
		start_heading = atan2(toward_point.y - start_point.y, toward_point.x - start_point.x)
	elif start.has("heading_deg"):
		if not _is_json_number(start["heading_deg"]) or not is_finite(float(start["heading_deg"])):
			return {}
		start_heading = deg_to_rad(float(start["heading_deg"]))

	return {
		"name": map_name,
		"version": version,
		"line_width_m": float(line_width),
		"start_x_m": start_point.x,
		"start_y_m": start_point.y,
		"start_theta_rad": start_heading,
		"segments": segments
	}


func _is_json_number(value: Variant) -> bool:
	return typeof(value) == TYPE_INT or typeof(value) == TYPE_FLOAT


func _is_json_integer(value: Variant) -> bool:
	if not _is_json_number(value) or not is_finite(float(value)):
		return false
	var number := float(value)
	return number >= 0.0 and number <= 2147483647.0 and floor(number) == number


# ============================================================== live link

func _connect_live(port: int = DEFAULT_LIVE_PORT) -> void:
	if live_peer != null:
		return
	live_port = port if port > 0 else DEFAULT_LIVE_PORT
	live_peer = StreamPeerTCP.new()
	var error := live_peer.connect_to_host("127.0.0.1", live_port)
	if error != OK:
		live_peer = null
		_notify("无法连接 127.0.0.1:%d。" % live_port, "error")
		_update_status("error", "连接失败")
		return
	live_mode = true
	live_ready = false
	live_running = false
	live_controller = "unknown"
	live_pending_controller = ""
	live_rx_buffer = ""
	live_request_pending = false
	live_pending_kind = ""
	live_deferred_command = ""
	live_deferred_kind = ""
	live_deferred_scenario.clear()
	live_deferred_map_path = ""
	live_deferred_map_json = ""
	live_deferred_payload.clear()
	live_pending_scenario.clear()
	live_pending_map_path = ""
	live_pending_map_json = ""
	live_pending_payload.clear()
	live_outstanding_steps = 0
	live_accumulator = 0.0
	active_map = {}
	map_bounds = Rect2()
	track_line_width_m = 0.02
	trace.clear()
	playback_time_s = 0.0
	run_file_name = ""
	_set_chart_trace(trace, 0.0, live_duration_s, true)
	time_label.text = "0.000"
	time_total_label.text = "/ %0.3f s" % live_duration_s
	_refresh_run_identity()
	_refresh_empty_state()
	_update_playback_buttons()
	_update_status("connecting", "%d Hz" % int(round(1.0 / maxf(live_control_dt_s, 0.0001))))
	_notify("正在连接 127.0.0.1:%d 上的 C 固定步长仿真服务。" % live_port)


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
	live_controller = "unknown"
	live_pending_controller = ""
	live_request_pending = false
	live_pending_kind = ""
	live_deferred_command = ""
	live_deferred_kind = ""
	live_deferred_scenario.clear()
	live_deferred_map_path = ""
	live_deferred_map_json = ""
	live_deferred_payload.clear()
	live_pending_scenario.clear()
	live_pending_map_path = ""
	live_pending_map_json = ""
	live_pending_payload.clear()
	live_outstanding_steps = 0
	if was_live:
		_update_controller_controls()
		is_playing = false
		if trace.is_empty():
			_refresh_run_identity()
			_refresh_empty_state()
			_update_status("offline", "待机")
		else:
			playback_time_s = minf(playback_time_s, _last_trace_time())
			current_index = 0
			_set_chart_trace(trace, playback_time_s, _duration_s(), true)
			time_total_label.text = "/ %0.3f s" % _duration_s()
			_update_playback()
			_refresh_run_identity()
			_update_status("replay", "%d Hz" % int(round(1.0 / maxf(line_dt_s, 0.0001))))
		_update_playback_buttons()


func _on_live_button() -> void:
	if live_mode:
		_notify("实时连接已关闭，运行记录仍可回放。", "info")
		_disconnect_live(true)
	else:
		_connect_live(live_port)


func _process_live(delta: float) -> void:
	if live_peer == null:
		return
	live_peer.poll()
	var status := live_peer.get_status()
	if status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE:
		_notify("C 仿真服务已断开。", "error")
		_update_status("error", "连接中断")
		_disconnect_live(false)
		return
	if status != StreamPeerTCP.STATUS_CONNECTED:
		return

	var available := live_peer.get_available_bytes()
	if available > 0:
		var packet := live_peer.get_data(available)
		if packet[0] != OK:
			_notify("实时连接读取失败。", "error")
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

	if live_outstanding_steps == 0 and not live_request_pending and \
			not live_deferred_command.is_empty():
		var command := live_deferred_command
		var kind := live_deferred_kind
		live_deferred_command = ""
		live_deferred_kind = ""
		live_pending_scenario = live_deferred_scenario.duplicate(true)
		live_deferred_scenario.clear()
		live_pending_map_path = live_deferred_map_path
		live_deferred_map_path = ""
		live_pending_map_json = live_deferred_map_json
		live_deferred_map_json = ""
		live_pending_payload = live_deferred_payload
		live_deferred_payload = PackedByteArray()
		if not _send_live_command(command, kind, live_pending_payload):
			live_pending_scenario.clear()
			live_pending_map_path = ""
			live_pending_map_json = ""
			live_pending_payload.clear()

	if not live_running and not trace.is_empty():
		_set_cursor(playback_time_s)


func _queue_live_command(command: String, kind: String,
		scenario: Dictionary = {}, map_path: String = "",
		payload: PackedByteArray = PackedByteArray(), map_json: String = "") -> bool:
	if not live_mode or live_peer == null or live_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		_notify("实时仿真连接不可用，命令未发送。", "error")
		return false
	if live_request_pending or not live_deferred_command.is_empty():
		_notify("请等待当前实时操作完成后再试。", "info")
		return false
	if command.to_utf8_buffer().size() > MAX_LIVE_COMMAND_BYTES or \
		payload.size() > MAX_MAP_JSON_BYTES:
		_notify("实时命令过长，操作已取消。", "error")
		return false
	live_deferred_command = command
	live_deferred_kind = kind
	live_deferred_scenario = scenario.duplicate(true)
	live_deferred_map_path = map_path
	live_deferred_map_json = map_json
	live_deferred_payload = payload
	live_running = false
	live_accumulator = 0.0
	_update_playback_buttons()
	return true


func _send_live_command(command: String, kind: String,
		payload: PackedByteArray = PackedByteArray()) -> bool:
	if live_peer == null or live_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return false
	var packet := (command + "\n").to_utf8_buffer()
	if not payload.is_empty():
		packet.append_array(payload)
		packet.append(10)
	var error := live_peer.put_data(packet)
	if error != OK:
		_notify("实时连接写入失败（错误码 %d）。" % error, "error")
		return false
	if kind != "step":
		live_request_pending = true
		live_pending_kind = kind
	return true


func _handle_live_message(message: Dictionary) -> void:
	var event := str(message.get("event", ""))
	if event == "ready":
		var map_change_ready := live_pending_kind == "map"
		var sync_selected_map := not map_change_ready and not selected_map_path.is_empty()
		live_request_pending = false
		live_pending_kind = ""
		if map_change_ready:
			trace.clear()
			playback_time_s = 0.0
			current_index = 0
			live_running = false
		active_map = {}
		map_bounds = Rect2()
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
		_set_active_map(message.get("map", {}))
		_create_track()
		_set_camera_mode(cam_mode, true)
		if map_change_ready:
			_set_chart_trace(trace, 0.0, live_duration_s, true)
			time_label.text = "0.000"
			time_total_label.text = "/ %0.3f s" % live_duration_s
		if map_change_ready:
			if not live_pending_map_path.is_empty():
				selected_map_path = live_pending_map_path
			if not live_pending_map_json.is_empty():
				selected_map_json = live_pending_map_json
			live_pending_map_path = ""
			live_pending_map_json = ""
			live_pending_payload.clear()
		live_profile_verified = bool(message.get("profile_verified", false))
		profile_verified = live_profile_verified
		var reported_controller := str(message.get("controller", "pid"))
		live_controller = reported_controller if reported_controller in ["pid", "mlp", "int8"] \
			else "unknown"
		live_pending_controller = ""
		line_dt_s = float(message.get("line_dt_s", 0.01))
		motor_dt_s = float(message.get("motor_dt_s", 0.002))
		live_control_dt_s = maxf(line_dt_s, 0.001)
		sensor_count = clampi(int(message.get("sensor_count", 8)), 1, 16)
		has_manifest = true
		live_ready = true
		if sync_selected_map:
			var map_payload := selected_map_json.to_utf8_buffer()
			var map_command := "MAPDATA %d" % map_payload.size()
			if map_payload.size() <= MAX_MAP_JSON_BYTES:
				if _queue_live_command(map_command, "map", {}, selected_map_path,
						map_payload, selected_map_json):
					_notify("正在将已打开的地图同步到 C 仿真。", "info")
			else:
				_notify("所选地图 JSON 过大，保留 C 服务当前地图。", "error")
				_send_live_command("RESET", "reset")
		else:
			_send_live_command("RESET", "map_reset" if map_change_ready else "reset")
		_refresh_run_identity()
		_refresh_empty_state()
		_update_status("connecting", "握手成功")
		_update_controller_controls()
		if not sync_selected_map:
			if map_change_ready:
				_notify("C 仿真已同步新地图，正在重置运行。", "ok")
			else:
				_notify("C 实时服务已就绪 · 固定步长 %0.1f 毫秒" % (live_control_dt_s * 1000.0), "ok")
	elif event == "reset":
		var completed_kind := live_pending_kind
		live_request_pending = false
		live_pending_kind = ""
		var reported_controller := str(message.get("controller", live_controller))
		if reported_controller in ["pid", "mlp", "int8"]:
			live_controller = reported_controller
		var requested_controller := live_pending_controller
		if completed_kind == "controller":
			live_pending_controller = ""
			if live_controller != requested_controller:
				_notify("C 仿真确认的控制器是 %s，与请求不一致。" %
					_controller_name(live_controller), "error")
		if completed_kind == "parameters" and not live_pending_scenario.is_empty():
			live_scenario = live_pending_scenario.duplicate(true)
			run_scenario = live_scenario.duplicate(true)
			track_amplitude_m = float(live_scenario.get("amp_m", track_amplitude_m))
			track_wavelength_m = maxf(float(live_scenario.get("wavelength_m", track_wavelength_m)), 0.001)
			live_duration_s = float(live_scenario.get("duration_s", live_duration_s))
			track_start_x = -1.0
			track_end_x = maxf(8.0, float(live_scenario.get("v_desired_mps", 0.8)) *
				live_duration_s * 1.2 + 3.0)
		live_pending_scenario.clear()
		live_pending_map_path = ""
		live_pending_map_json = ""
		live_pending_payload.clear()
		live_ready = true
		live_outstanding_steps = 0
		live_accumulator = 0.0
		trace.clear()
		playback_time_s = 0.0
		current_index = 0
		live_running = true
		_create_track()
		_update_car({
			"x": float(active_map.get("start_x_m", 0.0)),
			"y": float(active_map.get("start_y_m", 0.0)),
			"theta": float(active_map.get("start_theta_rad", 0.0))
		})
		_update_sample_ui({})
		_set_chart_trace(trace, 0.0, live_duration_s, true)
		time_label.text = "0.000"
		time_total_label.text = "/ %0.3f s" % live_duration_s
		_refresh_run_identity()
		_refresh_empty_state()
		_update_status("live", "%d Hz" % int(round(1.0 / maxf(live_control_dt_s, 0.0001))))
		_update_playback_buttons()
		_update_controller_controls()
		if completed_kind == "parameters":
			_notify("参数已同步到 C 仿真，运行已从起点重启。", "ok")
		elif completed_kind == "map_reset":
			_notify("新地图已生效，实时运行已重置。", "ok")
		elif completed_kind == "controller" and live_controller == requested_controller:
			_notify("C 仿真已确认切换到 %s，运行已从起点重启。" %
				_controller_name(live_controller), "ok")
	elif event == "state":
		var reported_controller := str(message.get("controller", live_controller))
		if reported_controller in ["pid", "mlp", "int8"] and reported_controller != live_controller:
			live_controller = reported_controller
			live_pending_controller = ""
			_update_controller_controls()
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
		_set_chart_trace(trace, playback_time_s, live_duration_s)
		time_label.text = "%0.3f" % playback_time_s
		time_total_label.text = "/ %0.3f s" % live_duration_s
		if _now_ms() - live_last_state_ms > LIVE_TRAJECTORY_REFRESH_MS:
			live_last_state_ms = _now_ms()
			_add_trajectory()
		if bool(sample.get("done", false)):
			live_running = false
			_add_trajectory()
			live_last_chart_ms = 0
			_set_chart_trace(trace, playback_time_s, live_duration_s)
			_update_playback_buttons()
			_notify("实时仿真完成 · 共 %d 条采样" % trace.size(), "ok")
	elif event == "done":
		live_running = false
		live_outstanding_steps = 0
		_add_trajectory()
		live_last_chart_ms = 0
		_set_chart_trace(trace, playback_time_s, live_duration_s)
		_update_playback_buttons()
	elif event == "error":
		live_request_pending = false
		live_pending_kind = ""
		live_deferred_command = ""
		live_deferred_kind = ""
		live_deferred_map_path = ""
		live_deferred_map_json = ""
		live_deferred_payload.clear()
		live_pending_controller = ""
		live_pending_scenario.clear()
		live_pending_map_path = ""
		live_pending_map_json = ""
		live_pending_payload.clear()
		live_running = false
		_notify("C 仿真器错误：%s" % str(message.get("message", "协议错误")), "error")
		_update_playback_buttons()
		_update_controller_controls()


func _live_state_to_row(message: Dictionary) -> Dictionary:
	var sample := message.duplicate()
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
	sample.erase("gray_value")
	sample.erase("gray_exist")
	sample.erase("gray_valid")
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


func _track_reference(x: float, y: float) -> Vector2:
	if not active_map.is_empty():
		var best_distance_squared := INF
		var best_lateral := 0.0
		var best_heading := 0.0
		for segment in active_map.get("segments", []):
			var x0 := float(segment[0])
			var y0 := float(segment[1])
			var dx := float(segment[2]) - x0
			var dy := float(segment[3]) - y0
			var length_squared := dx * dx + dy * dy
			if length_squared <= 0.000000000001:
				continue
			var ratio := clampf(((x - x0) * dx + (y - y0) * dy) / length_squared, 0.0, 1.0)
			var error_x := x - (x0 + ratio * dx)
			var error_y := y - (y0 + ratio * dy)
			var distance_squared := error_x * error_x + error_y * error_y
			if distance_squared < best_distance_squared:
				best_distance_squared = distance_squared
				best_lateral = (dx * error_y - dy * error_x) / sqrt(length_squared)
				best_heading = atan2(dy, dx)
		return Vector2(best_lateral, best_heading)
	if is_zero_approx(track_amplitude_m):
		return Vector2(y, 0.0)
	var slope := track_amplitude_m * (TAU / track_wavelength_m) * cos(TAU * x / track_wavelength_m)
	return Vector2(y - _track_y(x), atan(slope))


func _set_active_map(map_data: Variant) -> void:
	active_map = {}
	map_bounds = Rect2()
	if not map_data is Dictionary:
		return
	var segments: Variant = map_data.get("segments", [])
	if not segments is Array or segments.is_empty():
		return
	var min_x := INF
	var min_y := INF
	var max_x := -INF
	var max_y := -INF
	for segment in segments:
		if not segment is Array or segment.size() != 4:
			return
		var x0 := float(segment[0])
		var y0 := float(segment[1])
		var x1 := float(segment[2])
		var y1 := float(segment[3])
		if not is_finite(x0) or not is_finite(y0) or not is_finite(x1) or not is_finite(y1):
			return
		min_x = minf(min_x, minf(x0, x1))
		min_y = minf(min_y, minf(y0, y1))
		max_x = maxf(max_x, maxf(x0, x1))
		max_y = maxf(max_y, maxf(y0, y1))
	if max_x - min_x <= 0.000001 and max_y - min_y <= 0.000001:
		return
	active_map = map_data.duplicate(true)
	map_bounds = Rect2(min_x, min_y, max_x - min_x, max_y - min_y)
	track_line_width_m = maxf(float(active_map.get("line_width_m", track_line_width_m)), 0.001)
	track_start_x = min_x - 0.5
	track_end_x = max_x + 0.5


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
