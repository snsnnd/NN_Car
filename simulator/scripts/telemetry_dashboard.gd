extends Control

## Synchronized multi-channel strip chart.
## Six bands, one shared time axis, stable scales, hover readout and click-to-seek.

signal view_changed
signal seek_requested(time_s: float)

const Ui := preload("res://scripts/ui_theme.gd")

const IMMEDIATE_FAULT_MASK := 0x9C1  # ESTOP | HARD_OVERSPEED | UNDERVOLT | OVERVOLT | TEMP_STOP

const PLOTS := [
	{
		"name": "CROSS-TRACK", "unit": "mm", "mode": "symmetric", "min_span": 12.0,
		"decimals": 1, "signed": true, "band": true,
		"series": [{"key": "lateral_error_mm", "color": Ui.ACCENT, "width": 1.8, "fill": true}]
	},
	{
		"name": "HEADING", "unit": "deg", "mode": "symmetric", "min_span": 2.0,
		"decimals": 2, "signed": true, "band": false,
		"series": [{"key": "heading_error_deg", "color": Ui.STEEL, "width": 1.6}]
	},
	{
		"name": "SPEED", "unit": "m/s", "mode": "positive", "min_span": 0.3,
		"decimals": 3, "signed": false, "band": false,
		"series": [
			{"key": "v_target", "color": Ui.MUTED, "width": 1.0, "dash": true, "short": "T"},
			{"key": "speed_actual_mps", "color": Ui.MINT, "width": 1.8, "short": "A"}
		]
	},
	{
		"name": "YAW RATE", "unit": "rad/s", "mode": "symmetric", "min_span": 0.4,
		"decimals": 2, "signed": true, "band": false,
		"series": [
			{"key": "omega_target", "color": Ui.MUTED, "width": 1.0, "dash": true, "short": "T"},
			{"key": "omega_actual_radps", "color": Ui.STEEL, "width": 1.8, "short": "P"}
		]
	},
	{
		"name": "MOTOR OUTPUT", "unit": "PWM", "mode": "fixed", "min": -1.0, "max": 1.0,
		"decimals": 2, "signed": true, "band": false,
		"series": [
			{"key": "pwm_l", "color": Ui.MINT, "width": 1.5, "short": "L"},
			{"key": "pwm_r", "color": Ui.ROSE, "width": 1.5, "short": "R"}
		]
	},
	{
		"name": "LINE / SENSORS", "unit": "0-1", "mode": "ratio", "min": 0.0, "max": 1.0,
		"decimals": 2, "signed": false, "band": false,
		"series": [
			{"key": "line_conf", "color": Ui.OLIVE, "width": 1.5, "short": "C"},
			{"key": "sensor_valid_ratio", "color": Ui.STEEL, "width": 1.3, "short": "V"}
		]
	}
]

var samples: Array[Dictionary] = []
var cursor_time_s := 0.0
var duration_s := 1.0
var view_start_s := 0.0
var view_span_s := 0.0
var user_zoomed := false
var follow_latest := true
var compact := true
var track_half_width_mm := 10.0

var _pan_dragging := false
var _scrub_dragging := false
var _hover_time_s := -1.0
var _hover_x := -1.0
var _fault_spans: Array[Dictionary] = []
var _recovery_spans: Array[Dictionary] = []
var _scale_min: Array[float] = []
var _scale_max: Array[float] = []
var _scale_ready := false
var _scale_ms := 0
var _event_ms := 0
var _last_event_count := -1
var _last_sample_count := 0
var _bg_style: StyleBoxFlat
var _tag_style: StyleBoxFlat


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	focus_mode = Control.FOCUS_NONE
	mouse_default_cursor_shape = Control.CURSOR_CROSS
	mouse_exited.connect(_on_mouse_exited)
	_bg_style = Ui.flat(Color("0f1310"), Color("1e251f"), 6, 1)
	_tag_style = Ui.flat(Ui.SURFACE_4, Ui.LINE_STRONG, 4, 1, 7.0, 4.0)
	_scale_min.resize(PLOTS.size())
	_scale_max.resize(PLOTS.size())


func set_compact(value: bool) -> void:
	if compact == value:
		return
	compact = value
	_scale_ready = false
	queue_redraw()


func set_trace(rows: Array[Dictionary], cursor_s: float, run_duration_s: float,
		new_trace: bool = false) -> void:
	samples = rows
	cursor_time_s = cursor_s
	duration_s = maxf(run_duration_s, 0.001)
	if new_trace:
		view_start_s = 0.0
		view_span_s = 0.0
		user_zoomed = false
		follow_latest = true
		_hover_time_s = -1.0
		_scale_ready = false
		_last_event_count = -1
		_event_ms = 0
	if samples.is_empty():
		view_start_s = 0.0
		view_span_s = 0.0
		user_zoomed = false
		follow_latest = true
		_scale_ready = false
		_fault_spans.clear()
		_recovery_spans.clear()
	else:
		_update_scales(new_trace)
		_update_events()
	if new_trace or samples.size() != _last_sample_count:
		_last_sample_count = samples.size()
	queue_redraw()


func set_cursor(cursor_s: float) -> void:
	cursor_time_s = cursor_s
	queue_redraw()


func fit_trace() -> void:
	view_span_s = 0.0
	user_zoomed = false
	follow_latest = true
	queue_redraw()
	view_changed.emit()


func zoom_in() -> void:
	_zoom_at(_plot_rect().get_center().x, 1.6)


func zoom_out() -> void:
	_zoom_at(_plot_rect().get_center().x, 1.0 / 1.6)


func get_zoom_label() -> String:
	if samples.size() < 2 or not user_zoomed:
		return "FIT"
	var total := maxf(_last_sample_time() - _first_sample_time(), 0.001)
	return "x%0.1f" % (total / maxf(view_span_s, 0.001))


func get_view_state() -> Dictionary:
	return {
		"view_start_s": view_start_s,
		"view_span_s": view_span_s,
		"user_zoomed": user_zoomed,
		"follow_latest": follow_latest
	}


func apply_view_state(state: Dictionary) -> void:
	view_start_s = float(state.get("view_start_s", 0.0))
	view_span_s = float(state.get("view_span_s", 0.0))
	user_zoomed = bool(state.get("user_zoomed", false))
	follow_latest = bool(state.get("follow_latest", true))
	queue_redraw()


func _gui_input(event: InputEvent) -> void:
	var plot := _plot_rect()
	if event is InputEventMouseMotion:
		if _scrub_dragging and plot.has_point(event.position):
			cursor_time_s = _time_at_x(event.position.x)
			seek_requested.emit(cursor_time_s)
			queue_redraw()
		elif _pan_dragging:
			_pan_time(event.relative.x)
		elif plot.has_point(event.position):
			_hover_x = clampf(event.position.x, plot.position.x, plot.end.x)
			_hover_time_s = _time_at_x(_hover_x)
			queue_redraw()
		elif _hover_time_s >= 0.0:
			_hover_time_s = -1.0
			queue_redraw()
	elif event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if plot.has_point(event.position):
				_scrub_dragging = event.pressed
				if event.pressed:
					cursor_time_s = _time_at_x(event.position.x)
					seek_requested.emit(cursor_time_s)
					queue_redraw()
				accept_event()
		elif event.button_index == MOUSE_BUTTON_MIDDLE:
			_pan_dragging = event.pressed
			accept_event()
		elif event.pressed and plot.has_point(event.position):
			if event.button_index == MOUSE_BUTTON_WHEEL_UP:
				_zoom_at(event.position.x, 1.6)
				accept_event()
			elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
				_zoom_at(event.position.x, 1.0 / 1.6)
				accept_event()


func _on_mouse_exited() -> void:
	if _hover_time_s >= 0.0:
		_hover_time_s = -1.0
		queue_redraw()


func _zoom_at(anchor_x: float, factor: float) -> void:
	if samples.size() < 2:
		return
	var plot := _plot_rect()
	var ratio := clampf((anchor_x - plot.position.x) / maxf(plot.size.x, 1.0), 0.0, 1.0)
	var current := _view_range()
	var total := maxf(_last_sample_time() - _first_sample_time(), 0.001)
	var old_span := maxf(current.y - current.x, 0.001)
	var new_span := clampf(old_span / factor, maxf(total / 5000.0, 0.002), total)
	var anchor_time := lerpf(current.x, current.y, ratio)
	view_span_s = new_span
	view_start_s = anchor_time - ratio * new_span
	user_zoomed = new_span < total * 0.995
	follow_latest = user_zoomed and ratio >= 0.85 and \
		absf(current.y - _last_sample_time()) <= old_span * 0.12
	if not user_zoomed:
		follow_latest = true
	queue_redraw()
	view_changed.emit()


func _pan_time(delta_x: float) -> void:
	if samples.size() < 2:
		return
	var plot := _plot_rect()
	var current := _view_range()
	var span := maxf(current.y - current.x, 0.001)
	view_span_s = span
	view_start_s = current.x - delta_x / maxf(plot.size.x, 1.0) * span
	user_zoomed = span < (_last_sample_time() - _first_sample_time()) * 0.995
	follow_latest = false
	queue_redraw()
	view_changed.emit()


func _time_at_x(x: float) -> float:
	var plot := _plot_rect()
	var visible := _view_range()
	var ratio := clampf((x - plot.position.x) / maxf(plot.size.x, 1.0), 0.0, 1.0)
	return lerpf(visible.x, visible.y, ratio)


func _plot_rect() -> Rect2:
	var label_w := 154.0 if compact else 196.0
	var axis_h := 26.0 if compact else 30.0
	return Rect2(Vector2(label_w, 4.0), Vector2(maxf(size.x - label_w - 10.0, 20.0),
		maxf(size.y - axis_h - 6.0, 20.0)))


func _strip_rect(index: int) -> Rect2:
	var plot := _plot_rect()
	var strip_h := plot.size.y / float(PLOTS.size())
	return Rect2(plot.position.x, plot.position.y + strip_h * float(index), plot.size.x, strip_h)


func _first_sample_time() -> float:
	return float(samples[0].get("t_us", 0)) * 1.0e-6 if not samples.is_empty() else 0.0


func _last_sample_time() -> float:
	return float(samples[-1].get("t_us", 0)) * 1.0e-6 if not samples.is_empty() else 0.0


func _view_range() -> Vector2:
	var first_time := _first_sample_time()
	var last_time := _last_sample_time()
	var total := maxf(last_time - first_time, 0.001)
	if not user_zoomed or view_span_s <= 0.0:
		return Vector2(first_time, first_time + total)
	var span := minf(view_span_s, total)
	var start := last_time - span if follow_latest else clampf(view_start_s, first_time, last_time - span)
	return Vector2(start, start + span)


func _lower_bound_time(time_us: float) -> int:
	var low := 0
	var high := samples.size()
	while low < high:
		var middle := int((low + high) / 2)
		if float(samples[middle].get("t_us", 0)) < time_us:
			low = middle + 1
		else:
			high = middle
	return low


func _sample_index_at(time_s: float) -> int:
	if samples.is_empty():
		return -1
	var index := _lower_bound_time(time_s * 1.0e6)
	index = clampi(index, 0, samples.size() - 1)
	if index > 0 and absf(float(samples[index].get("t_us", 0)) * 1.0e-6 - time_s) > \
			absf(float(samples[index - 1].get("t_us", 0)) * 1.0e-6 - time_s):
		index -= 1
	return index


func _update_scales(force: bool) -> void:
	var now := Time.get_ticks_msec()
	if not force and _scale_ready and now - _scale_ms < 240:
		return
	var stride := maxi(1, int(ceil(float(samples.size()) / 6000.0)))
	for plot_index in range(PLOTS.size()):
		var spec: Dictionary = PLOTS[plot_index]
		var mode: String = spec["mode"]
		if mode == "fixed" or mode == "ratio":
			_scale_min[plot_index] = float(spec.get("min", 0.0))
			_scale_max[plot_index] = float(spec.get("max", 1.0))
			continue
		var minimum := INF
		var maximum := -INF
		for series in spec["series"]:
			var key: String = series["key"]
			for sample_index in range(0, samples.size(), stride):
				if samples[sample_index].has(key):
					var value := float(samples[sample_index][key])
					minimum = minf(minimum, value)
					maximum = maxf(maximum, value)
		if not is_finite(minimum) or not is_finite(maximum):
			minimum = 0.0
			maximum = 0.0
		minimum = minf(minimum, 0.0)
		var low: float
		var high: float
		if mode == "symmetric":
			var bound := maxf(maxf(absf(minimum), absf(maximum)), float(spec["min_span"]) * 0.5)
			bound *= 1.14
			low = -bound
			high = bound
		else:
			high = maxf(maximum, float(spec["min_span"])) * 1.14
			low = minf(minimum, 0.0)
			if high - low < float(spec["min_span"]):
				high = low + float(spec["min_span"])
		if force or not _scale_ready:
			_scale_min[plot_index] = low
			_scale_max[plot_index] = high
		else:
			_scale_min[plot_index] = minf(_scale_min[plot_index], low)
			_scale_max[plot_index] = maxf(_scale_max[plot_index], high)
	_scale_ready = true
	_scale_ms = now


func _update_events() -> void:
	var now := Time.get_ticks_msec()
	if now - _event_ms < 400 and samples.size() >= _last_event_count and _last_event_count >= 0:
		return
	_last_event_count = samples.size()
	_event_ms = now
	_fault_spans.clear()
	_recovery_spans.clear()
	if samples.size() < 2:
		return
	var fault_start := -1.0
	var fault_severity := 0
	var recovery_start := -1.0
	for index in range(samples.size()):
		var sample: Dictionary = samples[index]
		var time_s := float(sample.get("t_us", 0)) * 1.0e-6
		var faults := int(sample.get("safety_faults", 0))
		var severity := 0
		if faults != 0:
			severity = 2 if (faults & IMMEDIATE_FAULT_MASK) != 0 else 1
		if severity != fault_severity:
			if fault_severity != 0 and fault_start >= 0.0:
				_fault_spans.append({"start": fault_start, "end": time_s, "severity": fault_severity})
			fault_severity = severity
			fault_start = time_s if severity != 0 else -1.0
		var recovery := int(sample.get("mode", 0)) == 1
		if recovery and recovery_start < 0.0:
			recovery_start = time_s
		elif not recovery and recovery_start >= 0.0:
			_recovery_spans.append({"start": recovery_start, "end": time_s})
			recovery_start = -1.0
	if fault_severity != 0 and fault_start >= 0.0:
		_fault_spans.append({"start": fault_start, "end": _last_sample_time(), "severity": fault_severity})
	if recovery_start >= 0.0:
		_recovery_spans.append({"start": recovery_start, "end": _last_sample_time()})


func _draw() -> void:
	draw_style_box(_bg_style, Rect2(Vector2.ZERO, size))
	var label_w := 154.0 if compact else 196.0
	if samples.size() < 2:
		_draw_empty()
		return
	_update_scales(false)
	_update_events()

	var plot := _plot_rect()
	var visible := _view_range()
	var start_us := visible.x * 1.0e6
	var span_us := maxf((visible.y - visible.x) * 1.0e6, 1.0)
	var first_visible := _lower_bound_time(start_us)
	var end_visible := mini(_lower_bound_time(visible.y * 1.0e6) + 2, samples.size())
	var visible_count := maxi(end_visible - first_visible, 1)
	var stride := maxi(1, int(ceil(float(visible_count) / maxf(plot.size.x * 1.4, 100.0))))

	_draw_event_background(plot, visible)

	for index in range(PLOTS.size()):
		_draw_strip(index, _strip_rect(index), visible, first_visible, end_visible, stride)

	_draw_axis(plot, visible)
	draw_line(Vector2(plot.position.x - 7.0, plot.position.y),
		Vector2(plot.position.x - 7.0, plot.end.y), Color(Ui.LINE, 0.8), 1.0)
	_draw_cursor(plot, visible)


func _draw_empty() -> void:
	var font: Font = Ui.SANS
	for index in range(PLOTS.size()):
		var strip := _strip_rect(index)
		draw_line(Vector2(10.0, strip.position.y), Vector2(size.x - 10.0, strip.position.y),
			Color(Ui.LINE, 0.65), 1.0)
		draw_string(font, Vector2(18.0, strip.position.y + strip.size.y * 0.55),
			str(PLOTS[index]["name"]), HORIZONTAL_ALIGNMENT_LEFT, -1.0, 9, Ui.FAINT)
	var plot := _plot_rect()
	var center := plot.get_center()
	draw_string(Ui.SANS_MEDIUM, Vector2(plot.position.x + 24.0, center.y - 2.0),
		"NO SAMPLES YET", HORIZONTAL_ALIGNMENT_LEFT, -1.0, 12, Ui.MUTED)
	draw_string(Ui.MONO, Vector2(plot.position.x + 24.0, center.y + 16.0),
		"connect live or open a CSV log", HORIZONTAL_ALIGNMENT_LEFT, -1.0, 9, Ui.FAINT)


func _draw_event_background(plot: Rect2, visible: Vector2) -> void:
	for span in _fault_spans:
		var x0 := _time_to_x(float(span["start"]), visible, plot)
		var x1 := _time_to_x(float(span["end"]), visible, plot)
		if x1 < plot.position.x or x0 > plot.end.x:
			continue
		var color := Color(Ui.DANGER, 0.10) if int(span["severity"]) == 2 else Color(Ui.ACCENT, 0.06)
		draw_rect(Rect2(Vector2(maxf(x0, plot.position.x), plot.position.y),
			Vector2(minf(x1, plot.end.x) - maxf(x0, plot.position.x), plot.size.y)), color, true)
	for span in _recovery_spans:
		var x0 := _time_to_x(float(span["start"]), visible, plot)
		var x1 := _time_to_x(float(span["end"]), visible, plot)
		draw_rect(Rect2(Vector2(maxf(x0, plot.position.x), plot.position.y),
			Vector2(maxf(minf(x1, plot.end.x) - maxf(x0, plot.position.x), 0.0), 2.0)),
			Color(Ui.ACCENT, 0.7), true)


func _draw_strip(index: int, strip: Rect2, visible: Vector2, first_visible: int,
		end_visible: int, stride: int) -> void:
	var spec: Dictionary = PLOTS[index]
	var label_size := 9 if compact else 10
	var value_size := 11 if compact else 14
	var plot := Rect2(strip.position, strip.size)
	var low := _scale_min[index]
	var high := _scale_max[index]
	var value_span := maxf(high - low, 1.0e-6)

	if index > 0:
		draw_line(Vector2(10.0, strip.position.y), Vector2(size.x - 10.0, strip.position.y),
			Color(Ui.LINE, 0.65), 1.0)

	var zero_y := plot.position.y + (high - clampf(0.0, low, high)) / value_span * plot.size.y

	if bool(spec["band"]) and track_half_width_mm > 0.0:
		var band_top := _value_y(track_half_width_mm, plot, low, high)
		var band_bottom := _value_y(-track_half_width_mm, plot, low, high)
		draw_rect(Rect2(Vector2(plot.position.x, band_top), Vector2(plot.size.x,
			maxf(band_bottom - band_top, 1.0))), Color(Ui.MINT, 0.055), true)
		draw_line(Vector2(plot.position.x, band_top), Vector2(plot.end.x, band_top),
			Color(Ui.MINT, 0.22), 1.0, true)
		draw_line(Vector2(plot.position.x, band_bottom), Vector2(plot.end.x, band_bottom),
			Color(Ui.MINT, 0.22), 1.0, true)

	draw_line(Vector2(plot.position.x, zero_y), Vector2(plot.end.x, zero_y),
		Color(Ui.LINE_STRONG, 0.8), 1.0, true)
	if not compact:
		for ratio in [0.25, 0.75]:
			var grid_y := plot.position.y + plot.size.y * float(ratio)
			draw_line(Vector2(plot.position.x, grid_y), Vector2(plot.end.x, grid_y),
				Color(Ui.LINE, 0.55), 1.0)

	for series in spec["series"]:
		var key: String = series["key"]
		var points := PackedVector2Array()
		for sample_index in range(first_visible, end_visible, stride):
			var sample: Dictionary = samples[sample_index]
			if not sample.has(key):
				continue
			var x := _time_to_x(float(sample.get("t_us", 0)) * 1.0e-6, visible, plot)
			var y := _value_y(float(sample[key]), plot, low, high)
			points.append(Vector2(x, y))
		if points.size() < 2:
			continue
		if bool(series.get("fill", false)):
			_fill_series(points, zero_y, series["color"])
		if bool(series.get("dash", false)):
			_draw_dashed(points, series["color"], float(series["width"]))
		else:
			draw_polyline(points, series["color"], float(series["width"]), true)

	var name_text := "%s  %s" % [str(spec["name"]), str(spec["unit"])]
	draw_string(Ui.SANS_MEDIUM, Vector2(18.0, strip.position.y + 12.0), name_text,
		HORIZONTAL_ALIGNMENT_LEFT, -1.0, label_size, Ui.MUTED)
	draw_rect(Rect2(Vector2(10.0, strip.position.y + 6.0), Vector2(2.0, 8.0)),
		series_color(spec), true)

	var readout_time := _hover_time_s if _hover_time_s >= 0.0 else cursor_time_s
	var readout_index := _sample_index_at(readout_time)
	var value_x := 18.0
	var value_y := strip.position.y + 24.0
	if not compact:
		value_y = strip.position.y + 38.0
	var series_list: Array = spec["series"]
	for series_index in range(series_list.size()):
		var series: Dictionary = series_list[series_index]
		var key: String = series["key"]
		var text := "--"
		if readout_index >= 0 and samples[readout_index].has(key):
			text = _format_value(float(samples[readout_index][key]), spec)
		var short: String = str(series.get("short", ""))
		if not short.is_empty():
			text = "%s %s" % [short, text]
		var width := Ui.MONO_MEDIUM.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1.0,
			value_size).x
		draw_string(Ui.MONO_MEDIUM, Vector2(value_x, value_y), text,
			HORIZONTAL_ALIGNMENT_LEFT, -1.0, value_size, series["color"])
		value_x += width + 7.0
		if series_index < series_list.size() - 1:
			draw_string(Ui.MONO, Vector2(value_x, value_y), "/", HORIZONTAL_ALIGNMENT_LEFT,
				-1.0, value_size - 2, Ui.FAINT)
			value_x += 9.0

	if not compact:
		var top_text := ("%+.2f" % high) if bool(spec["signed"]) else ("%.2f" % high)
		var bottom_text := ("%+.2f" % low) if bool(spec["signed"]) else ("%.2f" % low)
		draw_string(Ui.MONO, Vector2(plot.end.x - 54.0, plot.position.y + 10.0), top_text,
			HORIZONTAL_ALIGNMENT_RIGHT, 50.0, 9, Ui.FAINT)
		draw_string(Ui.MONO, Vector2(plot.end.x - 54.0, plot.end.y - 3.0), bottom_text,
			HORIZONTAL_ALIGNMENT_RIGHT, 50.0, 9, Ui.FAINT)


func _draw_axis(plot: Rect2, visible: Vector2) -> void:
	var axis_y := plot.end.y + (9.0 if compact else 13.0)
	draw_line(Vector2(plot.position.x, axis_y), Vector2(plot.end.x, axis_y), Ui.LINE, 1.0)
	var span := maxf(visible.y - visible.x, 0.001)
	var step := _nice_step(span / 6.0)
	var tick := ceilf(visible.x / step) * step
	var size := 9 if compact else 10
	while tick <= visible.y + 0.0001:
		var x := _time_to_x(tick, visible, plot)
		draw_line(Vector2(x, axis_y), Vector2(x, axis_y + 3.0), Ui.LINE_STRONG, 1.0)
		draw_string(Ui.MONO, Vector2(x - 18.0, axis_y + 15.0), "%0.1fs" % tick,
			HORIZONTAL_ALIGNMENT_CENTER, 36.0, size, Ui.FAINT)
		tick += step


func _draw_cursor(plot: Rect2, visible: Vector2) -> void:
	if _hover_time_s >= 0.0:
		var hx := _time_to_x(_hover_time_s, visible, plot)
		if hx >= plot.position.x and hx <= plot.end.x:
			_draw_vertical_dashes(hx, plot, Color(Ui.MUTED, 0.55))
	var cx := _time_to_x(cursor_time_s, visible, plot)
	if cx >= plot.position.x and cx <= plot.end.x:
		draw_line(Vector2(cx, plot.position.y), Vector2(cx, plot.end.y),
			Color(Ui.ACCENT, 0.75), 1.0, true)
		var knob := Rect2(Vector2(cx - 3.0, plot.end.y - 2.0), Vector2(6.0, 6.0))
		draw_rect(knob, Ui.ACCENT, true)


func _draw_vertical_dashes(x: float, plot: Rect2, color: Color) -> void:
	var y := plot.position.y
	while y < plot.end.y:
		draw_line(Vector2(x, y), Vector2(x, minf(y + 5.0, plot.end.y)), color, 1.0, true)
		y += 9.0


func _fill_series(points: PackedVector2Array, zero_y: float, color: Color) -> void:
	if points.size() < 3:
		return
	var above := false
	var below := false
	var extent := 0.0
	for point in points:
		var delta := point.y - zero_y
		extent = maxf(extent, absf(delta))
		if delta > 0.8:
			above = true
		elif delta < -0.8:
			below = true
	if extent < 0.8 or (above and below):
		return
	var polygon := PackedVector2Array()
	polygon.append(Vector2(points[0].x, zero_y))
	for point in points:
		polygon.append(point)
	polygon.append(Vector2(points[-1].x, zero_y))
	draw_colored_polygon(polygon, Color(color, 0.10))


func _draw_dashed(points: PackedVector2Array, color: Color, width: float) -> void:
	var dash := 5.0 if compact else 7.0
	var gap := 4.0
	var drawing := true
	var remaining := dash
	for index in range(1, points.size()):
		var from := points[index - 1]
		var to := points[index]
		var segment := to - from
		var length := segment.length()
		if length <= 0.01:
			continue
		var direction := segment / length
		var travelled := 0.0
		while travelled < length:
			var step := minf(remaining, length - travelled)
			if drawing:
				draw_line(from + direction * travelled, from + direction * (travelled + step),
					color, width, true)
			travelled += step
			remaining -= step
			if remaining <= 0.0001:
				drawing = not drawing
				remaining = dash if drawing else gap


func _value_y(value: float, plot: Rect2, low: float, high: float) -> float:
	var ratio := clampf((value - low) / maxf(high - low, 1.0e-6), 0.0, 1.0)
	return plot.end.y - ratio * plot.size.y


func _time_to_x(time_s: float, visible: Vector2, plot: Rect2) -> float:
	return plot.position.x + (time_s - visible.x) / maxf(visible.y - visible.x, 0.001) * plot.size.x


func _format_value(value: float, spec: Dictionary) -> String:
	var decimals := int(spec["decimals"])
	var signed := bool(spec["signed"])
	if signed:
		return ("%+." + str(decimals) + "f") % value
	return ("%." + str(decimals) + "f") % value


func series_color(spec: Dictionary) -> Color:
	return spec["series"][0]["color"]


func _nice_step(raw: float) -> float:
	if raw <= 0.0:
		return 1.0
	var magnitude := pow(10.0, floor(log(raw) / log(10.0)))
	var normalized := raw / magnitude
	var nice := 1.0
	if normalized > 5.0:
		nice = 10.0
	elif normalized > 2.0:
		nice = 5.0
	elif normalized > 1.0:
		nice = 2.0
	return nice * magnitude
