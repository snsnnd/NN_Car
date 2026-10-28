extends Control

signal view_changed

const PLOTS := [
	{"title": "CROSS-TRACK ERROR", "unit": "mm", "min_span": 2.0, "zero": true,
		"series": [{"key": "lateral_error_mm", "name": "e_y", "color": Color("54d6bc")}]},
	{"title": "HEADING ERROR", "unit": "deg", "min_span": 2.0, "zero": true,
		"series": [{"key": "heading_error_deg", "name": "e_θ", "color": Color("8ecbff")}]},
	{"title": "FORWARD SPEED", "unit": "m/s", "min_span": 0.12, "zero": false,
		"series": [
			{"key": "v_target", "name": "target", "color": Color("f3b45e")},
			{"key": "speed_actual_mps", "name": "actual", "color": Color("54d6bc")}]},
	{"title": "YAW RATE", "unit": "rad/s", "min_span": 0.12, "zero": true,
		"series": [
			{"key": "omega_target", "name": "target", "color": Color("f3b45e")},
			{"key": "omega_actual_radps", "name": "pose Δθ/Δt", "color": Color("8ecbff")}]},
	{"title": "MOTOR OUTPUT", "unit": "PWM", "min_span": 0.2, "zero": true,
		"series": [
			{"key": "pwm_l", "name": "left", "color": Color("54d6bc")},
			{"key": "pwm_r", "name": "right", "color": Color("ed8d91")}]},
	{"title": "LINE / SENSOR HEALTH", "unit": "ratio", "min_span": 0.2, "zero": false,
		"series": [
			{"key": "line_conf", "name": "confidence", "color": Color("54d6bc")},
			{"key": "sensor_valid_ratio", "name": "valid", "color": Color("f3b45e")}]}
]

var samples: Array[Dictionary] = []
var cursor_time_s := 0.0
var duration_s := 1.0
var view_start_s := 0.0
var view_span_s := 0.0
var user_zoomed := false
var follow_latest := true
var pan_dragging := false


func set_trace(rows: Array[Dictionary], cursor_s: float, run_duration_s: float) -> void:
	samples = rows
	cursor_time_s = cursor_s
	duration_s = maxf(run_duration_s, 0.001)
	if samples.is_empty():
		view_start_s = 0.0
		view_span_s = 0.0
		user_zoomed = false
		follow_latest = true
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
	_zoom_at(1.0, 1.5)


func zoom_out() -> void:
	_zoom_at(1.0, 1.0 / 1.5)


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


func get_zoom_label() -> String:
	if samples.size() < 2 or not user_zoomed:
		return "FIT"
	var total_span := maxf(_last_sample_time() - _first_sample_time(), 0.001)
	return "×%0.1f" % (total_span / maxf(view_span_s, 0.001))


func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
			_zoom_at(_time_ratio_at_x(event.position.x), 1.5)
			accept_event()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
			_zoom_at(_time_ratio_at_x(event.position.x), 1.0 / 1.5)
			accept_event()
		elif event.button_index == MOUSE_BUTTON_MIDDLE:
			pan_dragging = event.pressed
			accept_event()
	elif event is InputEventMouseMotion and pan_dragging:
		_pan_time(event.relative.x)
		accept_event()


func _zoom_at(anchor_ratio: float, factor: float) -> void:
	if samples.size() < 2:
		return
	var current := _view_range()
	var total_span := maxf(_last_sample_time() - _first_sample_time(), 0.001)
	var old_span := maxf(current.y - current.x, 0.001)
	var new_span := clampf(old_span / factor, maxf(total_span / 5000.0, 0.001), total_span)
	var anchor_time := lerpf(current.x, current.y, clampf(anchor_ratio, 0.0, 1.0))
	view_span_s = new_span
	view_start_s = anchor_time - anchor_ratio * new_span
	user_zoomed = new_span < total_span * 0.995
	follow_latest = user_zoomed and anchor_ratio >= 0.85 and \
		absf(current.y - _last_sample_time()) <= old_span * 0.1
	if not user_zoomed:
		follow_latest = true
	queue_redraw()
	view_changed.emit()


func _pan_time(delta_x: float) -> void:
	if samples.size() < 2:
		return
	var graph_width := maxf((size.x - 36.0) / 3.0 - 52.0, 1.0)
	var current := _view_range()
	view_span_s = maxf(current.y - current.x, 0.001)
	view_start_s = current.x - delta_x / graph_width * view_span_s
	user_zoomed = view_span_s < (_last_sample_time() - _first_sample_time()) * 0.995
	follow_latest = false
	queue_redraw()
	view_changed.emit()


func _time_ratio_at_x(x: float) -> float:
	var margin := 10.0
	var gap := 8.0
	var col_width := (size.x - margin * 2.0 - gap * 2.0) / 3.0
	var column := clampi(int((x - margin) / maxf(col_width + gap, 1.0)), 0, 2)
	var graph_left := margin + column * (col_width + gap) + 42.0
	var graph_width := maxf(col_width - 52.0, 1.0)
	return clampf((x - graph_left) / graph_width, 0.0, 1.0)


func _first_sample_time() -> float:
	return float(samples[0].get("t_us", 0)) * 1.0e-6 if not samples.is_empty() else 0.0


func _last_sample_time() -> float:
	return float(samples[-1].get("t_us", 0)) * 1.0e-6 if not samples.is_empty() else 0.0


func _view_range() -> Vector2:
	var first_time := _first_sample_time()
	var last_time := _last_sample_time()
	var total_span := maxf(last_time - first_time, 0.001)
	if not user_zoomed or view_span_s <= 0.0:
		return Vector2(first_time, first_time + total_span)
	var span := minf(view_span_s, total_span)
	var start := last_time - span if follow_latest else clampf(view_start_s, first_time, last_time - span)
	return Vector2(start, start + span)


func _lower_bound_time(time_us: float) -> int:
	var low := 0
	var high := samples.size()
	while low < high:
		var middle: int = int((low + high) / 2)
		if float(samples[middle].get("t_us", 0)) < time_us:
			low = middle + 1
		else:
			high = middle
	return low


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.035, 0.052, 0.065, 0.96), true)
	if samples.size() < 2:
		var font := ThemeDB.fallback_font
		draw_string(font, Vector2(18.0, 26.0), "TELEMETRY  /  waiting for C samples",
			HORIZONTAL_ALIGNMENT_LEFT, -1.0, 14, Color("91a6ae"))
		return

	var margin := 10.0
	var gap := 8.0
	var header_height := 20.0
	var col_width := (size.x - margin * 2.0 - gap * 2.0) / 3.0
	var row_height := (size.y - margin * 2.0 - gap) / 2.0
	for index in range(PLOTS.size()):
		var column := index % 3
		var row := index / 3
		var rect := Rect2(
			Vector2(margin + column * (col_width + gap), margin + row * (row_height + gap)),
			Vector2(col_width, row_height))
		_draw_plot(rect, PLOTS[index], header_height)


func _draw_plot(rect: Rect2, spec: Dictionary, header_height: float) -> void:
	draw_rect(rect, Color("0a151c"), true)
	draw_rect(rect, Color("263943"), false, 1.0)
	var font := ThemeDB.fallback_font
	draw_string(font, rect.position + Vector2(10.0, 15.0), spec["title"],
		HORIZONTAL_ALIGNMENT_LEFT, -1.0, 11, Color("d2e0e4"))

	var legend_x := rect.end.x - 10.0
	var series: Array = spec["series"]
	for series_index in range(series.size() - 1, -1, -1):
		var item: Dictionary = series[series_index]
		var label_width := font.get_string_size(item["name"], HORIZONTAL_ALIGNMENT_LEFT, -1, 10).x
		legend_x -= label_width + 25.0
		draw_line(Vector2(legend_x, rect.position.y + 11.0),
			Vector2(legend_x + 12.0, rect.position.y + 11.0), item["color"], 2.0, true)
		draw_string(font, Vector2(legend_x + 16.0, rect.position.y + 14.0), item["name"],
			HORIZONTAL_ALIGNMENT_LEFT, -1.0, 10, item["color"])

	var graph := Rect2(rect.position + Vector2(42.0, header_height + 4.0),
		Vector2(rect.size.x - 52.0, rect.size.y - header_height - 28.0))
	if graph.size.x < 20.0 or graph.size.y < 20.0:
		return

	var visible_time := _view_range()
	var start_us := visible_time.x * 1.0e6
	var end_us := visible_time.y * 1.0e6
	var span_us := maxf(end_us - start_us, 1.0)
	var first_visible := _lower_bound_time(start_us)
	var end_visible := mini(_lower_bound_time(end_us) + 1, samples.size())
	var visible_count := maxi(end_visible - first_visible, 1)
	var minimum := INF
	var maximum := -INF
	var stride := maxi(1, int(ceil(float(visible_count) / 2500.0)))
	for item in series:
		var key: String = item["key"]
		for sample_index in range(first_visible, end_visible, stride):
			if samples[sample_index].has(key):
				var value := float(samples[sample_index][key])
				minimum = minf(minimum, value)
				maximum = maxf(maximum, value)
	if not is_finite(minimum) or not is_finite(maximum):
		return

	if bool(spec["zero"]):
		var bound := maxf(maxf(absf(minimum), absf(maximum)), float(spec["min_span"]) * 0.5)
		minimum = -bound
		maximum = bound
	else:
		var center := 0.5 * (minimum + maximum)
		var span := maxf(maximum - minimum, float(spec["min_span"]))
		minimum = center - span * 0.58
		maximum = center + span * 0.58
		if spec["unit"] == "ratio":
			minimum = maxf(0.0, minimum)
			maximum = minf(1.0, maxf(maximum, 1.0))
	var value_span := maxf(maximum - minimum, 1.0e-6)

	for grid_index in range(4):
		var ratio := float(grid_index) / 3.0
		var y := graph.position.y + graph.size.y * ratio
		draw_line(Vector2(graph.position.x, y), Vector2(graph.end.x, y),
			Color("1d3038"), 1.0)
		var label_value := maximum - value_span * ratio
		draw_string(font, Vector2(rect.position.x + 4.0, y + 3.0),
			"%0.2f" % label_value, HORIZONTAL_ALIGNMENT_LEFT, 36.0, 9, Color("7f959d"))

	for series_item in series:
		var points := PackedVector2Array()
		var key: String = series_item["key"]
		for sample_index in range(first_visible, end_visible, stride):
			var sample: Dictionary = samples[sample_index]
			if not sample.has(key):
				continue
			var x_ratio := clampf((float(sample.get("t_us", 0)) - start_us) / span_us, 0.0, 1.0)
			var y_ratio := clampf((maximum - float(sample[key])) / value_span, 0.0, 1.0)
			points.append(Vector2(graph.position.x + x_ratio * graph.size.x,
				graph.position.y + y_ratio * graph.size.y))
		if points.size() >= 2:
			draw_polyline(points, series_item["color"], 1.6, true)

	var cursor_ratio := clampf((cursor_time_s * 1.0e6 - start_us) / span_us, 0.0, 1.0)
	var cursor_x := graph.position.x + cursor_ratio * graph.size.x
	draw_line(Vector2(cursor_x, graph.position.y), Vector2(cursor_x, graph.end.y),
		Color(0.92, 0.96, 0.94, 0.6), 1.0)
	draw_string(font, Vector2(graph.position.x, rect.end.y - 4.0), "%0.2fs" % visible_time.x,
		HORIZONTAL_ALIGNMENT_LEFT, -1.0, 9, Color("7f959d"))
	draw_string(font, Vector2(graph.end.x - 44.0, rect.end.y - 4.0), "%0.2fs" % visible_time.y,
		HORIZONTAL_ALIGNMENT_RIGHT, 44.0, 9, Color("7f959d"))
