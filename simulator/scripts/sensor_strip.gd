extends Control

## Magnified gray-sensor array. One bar per installed channel, with existence and
## sample-validity state, line centroid marker and hover detail.

const Ui := preload("res://scripts/ui_theme.gd")

var sample: Dictionary = {}
var channels := 8
var positions_m: Array[float] = [
	0.0525, 0.0375, 0.0225, 0.0075, -0.0075, -0.0225, -0.0375, -0.0525
]
var has_data := false
var hover_channel := -1

var _bar_bg: StyleBoxFlat


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	focus_mode = Control.FOCUS_NONE
	custom_minimum_size.y = 72.0
	_bar_bg = Ui.flat(Color(Ui.SURFACE_2, 0.7), Color(Ui.LINE, 0.9), 3, 1)
	mouse_exited.connect(func() -> void:
		hover_channel = -1
		queue_redraw())


func set_channel_count(count: int) -> void:
	channels = clampi(count, 1, 16)
	queue_redraw()


func set_positions(positions: Array) -> void:
	positions_m.clear()
	for value in positions:
		positions_m.append(float(value))
	queue_redraw()


func set_sample(row: Dictionary) -> void:
	sample = row
	has_data = not row.is_empty() and row.has("gray_value_00")
	var reported := int(row.get("sensor_count", 0))
	if reported > 0:
		channels = clampi(reported, 1, 16)
	queue_redraw()


func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		var channel := _channel_at(event.position.x)
		if channel != hover_channel:
			hover_channel = channel
			queue_redraw()


func _channel_at(x: float) -> int:
	var area := _bar_area()
	if channels <= 0 or not area.has_point(Vector2(x, area.position.y)):
		return -1
	var width := area.size.x / float(channels)
	return clampi(int((x - area.position.x) / maxf(width, 1.0)), 0, channels - 1)


func _bar_area() -> Rect2:
	return Rect2(Vector2(0.0, 13.0), Vector2(size.x, 34.0))


func _draw() -> void:
	var font := Ui.SANS
	var area := _bar_area()
	var gap := 3.0
	var bar_w := maxf((area.size.x - gap * float(channels - 1)) / float(channels), 3.0)
	var baseline_y := area.end.y - 3.0

	for index in range(channels):
		var x := area.position.x + float(index) * (bar_w + gap)
		var bar := Rect2(Vector2(x, area.position.y), Vector2(bar_w, area.size.y - 3.0))
		draw_style_box(_bar_bg, bar)

		var exist := 1
		var valid := 0
		var value := 0.0
		var has_value := false
		if has_data:
			var suffix := "%02d" % index
			exist = int(sample.get("gray_exist_" + suffix, 1))
			valid = int(sample.get("gray_valid_" + suffix, 0))
			has_value = sample.has("gray_value_" + suffix)
			if has_value:
				value = float(sample["gray_value_" + suffix])
		var response := clampf((value + 1.0) * 0.5, 0.0, 1.0)
		var color := Color("2a332b").lerp(Ui.MINT, response)
		if exist == 0:
			color = Color("1c211c")
		elif valid == 0:
			color = Ui.ACCENT

		if exist != 0 and has_value:
			var fill_y := baseline_y - response * (bar.size.y - 4.0)
			draw_rect(Rect2(Vector2(bar.position.x + 1.0, fill_y),
				Vector2(maxf(bar.size.x - 2.0, 1.0), maxf(baseline_y - fill_y, 1.0))), color, true)
		if hover_channel == index:
			draw_rect(bar, Color(Ui.TEXT, 0.55), false, 1.0)

		var index_color := Ui.FAINT
		if exist == 0:
			index_color = Ui.LINE_STRONG
		elif valid == 0:
			index_color = Ui.ACCENT
		draw_string(Ui.MONO, Vector2(x, area.end.y + 9.0), "%02d" % index,
			HORIZONTAL_ALIGNMENT_CENTER, bar_w, 10, index_color)

	draw_line(Vector2(area.position.x, baseline_y), Vector2(area.end.x, baseline_y),
		Ui.LINE, 1.0)
	draw_line(Vector2(area.position.x, area.position.y + area.size.y * 0.5),
		Vector2(area.end.x, area.position.y + area.size.y * 0.5), Color(Ui.FAINT, 0.35), 1.0)

	if has_data and float(sample.get("line_found", 0)) != 0.0:
		var centroid := float(sample.get("line_centroid_m", 0.0))
		var marker_x := _centroid_x(centroid, area)
		draw_line(Vector2(marker_x, area.position.y - 1.0), Vector2(marker_x, area.end.y),
			Color(Ui.ACCENT, 0.45), 1.0)
		var triangle := PackedVector2Array([
			Vector2(marker_x - 3.0, area.position.y - 5.0),
			Vector2(marker_x + 3.0, area.position.y - 5.0),
			Vector2(marker_x, area.position.y - 1.0)
		])
		draw_colored_polygon(triangle, Ui.ACCENT)

	draw_string(font, Vector2(0.0, area.position.y - 4.0), "左", HORIZONTAL_ALIGNMENT_LEFT,
		-1.0, 10, Ui.FAINT)
	draw_string(font, Vector2(0.0, area.position.y - 4.0), "右", HORIZONTAL_ALIGNMENT_RIGHT,
		size.x, 10, Ui.FAINT)
	var caption := ""
	if not has_data:
		caption = "未记录传感器数据"
	elif hover_channel >= 0:
		var suffix := "%02d" % hover_channel
		var exist := int(sample.get("gray_exist_" + suffix, 1))
		var valid := int(sample.get("gray_valid_" + suffix, 0))
		var value := float(sample.get("gray_value_" + suffix, 0.0))
		var state := "缺失" if exist == 0 else ("无效" if valid == 0 else "有效")
		caption = "通道 %02d  %+0.2f  %s" % [hover_channel, value, state]
	else:
		var valid_count := int(sample.get("valid_count", 0))
		var present := 0
		var valid_sum := 0
		for index in range(channels):
			var suffix := "%02d" % index
			present += int(sample.get("gray_exist_" + suffix, 1))
			valid_sum += int(sample.get("gray_valid_" + suffix, 0))
		if present == 0:
			valid_sum = valid_count
			present = channels
		caption = "%02d/%02d 有效 · 置信度 %0.2f" % [valid_sum, present,
			float(sample.get("line_conf", 0.0))]
	draw_string(Ui.MONO, Vector2(0.0, size.y - 2.0), caption, HORIZONTAL_ALIGNMENT_LEFT,
		size.x, 11, Ui.MUTED)


func _centroid_x(centroid_m: float, area: Rect2) -> float:
	if positions_m.is_empty():
		return area.position.x + area.size.x * 0.5
	var left := positions_m[0]
	var right := positions_m[0]
	for value in positions_m:
		left = maxf(left, value)
		right = minf(right, value)
	var span := maxf(left - right, 0.001)
	var ratio := clampf((left - centroid_m) / span, -0.15, 1.15)
	var base_ratio := clampf(ratio, 0.0, 1.0)
	var x := area.position.x + base_ratio * area.size.x
	if ratio < 0.0 or ratio > 1.0:
		x = clampf(x, area.position.x - 4.0, area.end.x + 4.0)
	return x
