extends Control

## Compact run timeline: buffered span, playback head, fault and recovery markers,
## drag-to-seek and hover readout.

signal seek_requested(time_s: float)

const Ui := preload("res://scripts/ui_theme.gd")
const IMMEDIATE_FAULT_MASK := 0x9C1

var samples: Array[Dictionary] = []
var duration_s := 1.0
var cursor_s := 0.0
var buffered_s := 0.0
var live := false

var _fault_spans: Array[Dictionary] = []
var _recovery_spans: Array[Dictionary] = []
var _dragging := false
var _hover_s := -1.0
var _cursor_tag: StyleBoxFlat
var _bar_style: StyleBoxFlat
var _event_ms := 0
var _last_count := -1


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	focus_mode = Control.FOCUS_NONE
	mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	_cursor_tag = Ui.flat(Ui.SURFACE_4, Ui.LINE_STRONG, 3, 1, 6.0, 3.0)
	_bar_style = Ui.flat(Ui.SURFACE_2, Ui.LINE, 3, 1)
	mouse_exited.connect(func() -> void:
		if not _dragging:
			_hover_s = -1.0
			queue_redraw())


func set_data(rows: Array[Dictionary], cursor_time_s: float, run_duration_s: float,
		is_live: bool, new_trace: bool = false) -> void:
	samples = rows
	cursor_s = cursor_time_s
	duration_s = maxf(run_duration_s, 0.001)
	live = is_live
	buffered_s = float(samples[-1].get("t_us", 0)) * 1.0e-6 if not samples.is_empty() else 0.0
	if new_trace:
		_event_ms = 0
		_last_count = -1
	_update_events()
	queue_redraw()


func set_cursor(cursor_time_s: float) -> void:
	cursor_s = cursor_time_s
	queue_redraw()


func _update_events() -> void:
	var now := Time.get_ticks_msec()
	if now - _event_ms < 350 and samples.size() >= _last_count and _last_count >= 0:
		return
	_last_count = samples.size()
	_event_ms = now
	_fault_spans.clear()
	_recovery_spans.clear()
	if samples.size() < 2:
		return
	var fault_start := -1.0
	var severity := 0
	var recovery_start := -1.0
	for index in range(samples.size()):
		var sample: Dictionary = samples[index]
		var time_s := float(sample.get("t_us", 0)) * 1.0e-6
		var faults := int(sample.get("safety_faults", 0))
		var current := 0
		if faults != 0:
			current = 2 if (faults & IMMEDIATE_FAULT_MASK) != 0 else 1
		if current != severity:
			if severity != 0 and fault_start >= 0.0:
				_fault_spans.append({"start": fault_start, "end": time_s, "severity": severity})
			severity = current
			fault_start = time_s if current != 0 else -1.0
		var recovery := int(sample.get("mode", 0)) == 1
		if recovery and recovery_start < 0.0:
			recovery_start = time_s
		elif not recovery and recovery_start >= 0.0:
			_recovery_spans.append({"start": recovery_start, "end": time_s})
			recovery_start = -1.0
	if severity != 0 and fault_start >= 0.0:
		_fault_spans.append({"start": fault_start, "end": buffered_s, "severity": severity})
	if recovery_start >= 0.0:
		_recovery_spans.append({"start": recovery_start, "end": buffered_s})


func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		if _dragging:
			cursor_s = _time_at(event.position.x)
			seek_requested.emit(cursor_s)
		_hover_s = _time_at(event.position.x)
		queue_redraw()
	elif event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			_dragging = event.pressed
			if event.pressed:
				cursor_s = _time_at(event.position.x)
				seek_requested.emit(cursor_s)
			queue_redraw()
			accept_event()


func _time_at(x: float) -> float:
	var inset := 10.0
	var ratio := clampf((x - inset) / maxf(size.x - inset * 2.0, 1.0), 0.0, 1.0)
	return ratio * duration_s


func _time_to_x(time_s: float) -> float:
	var inset := 10.0
	return inset + clampf(time_s / maxf(duration_s, 0.001), 0.0, 1.0) * (size.x - inset * 2.0)


func _draw() -> void:
	var inset := 10.0
	var bar_y := 15.0
	var bar_h := 7.0
	var bar := Rect2(Vector2(inset, bar_y), Vector2(maxf(size.x - inset * 2.0, 4.0), bar_h))
	draw_style_box(_bar_style, bar)

	var recorded_end := buffered_s if live else duration_s
	var recorded_x := _time_to_x(recorded_end)
	if recorded_x > bar.position.x:
		draw_rect(Rect2(bar.position, Vector2(maxf(recorded_x - bar.position.x, 0.0), bar_h)),
			Color(Ui.LINE_STRONG, 0.55), true)
	if live and duration_s > recorded_end:
		draw_line(Vector2(_time_to_x(recorded_end), bar_y + bar_h * 0.5),
			Vector2(bar.end.x, bar_y + bar_h * 0.5), Color(Ui.ACCENT, 0.28), 1.0)

	var cursor_x := _time_to_x(cursor_s)
	draw_rect(Rect2(Vector2(bar.position.x, bar_y), Vector2(maxf(cursor_x - bar.position.x, 0.0),
		bar_h)), Color(Ui.ACCENT, 0.55), true)
	draw_line(Vector2(cursor_x, 7.0), Vector2(cursor_x, bar_y + bar_h + 5.0),
		Color(Ui.ACCENT, 0.9), 1.0, true)
	draw_rect(Rect2(Vector2(cursor_x - 2.5, bar_y - 4.0), Vector2(5.0, bar_h + 8.0)),
		Ui.ACCENT, true)

	for span in _recovery_spans:
		var x0 := _time_to_x(float(span["start"]))
		var x1 := _time_to_x(float(span["end"]))
		draw_rect(Rect2(Vector2(x0, bar_y + bar_h + 2.0), Vector2(maxf(x1 - x0, 1.0), 2.0)),
			Color(Ui.ACCENT, 0.7), true)
	for span in _fault_spans:
		var x0 := _time_to_x(float(span["start"]))
		var x1 := _time_to_x(float(span["end"]))
		var color := Ui.DANGER if int(span["severity"]) == 2 else Ui.ACCENT
		draw_rect(Rect2(Vector2(x0, bar_y - 4.0), Vector2(maxf(x1 - x0, 1.5), 3.0)), color, true)

	var hover_time := _hover_s if _hover_s >= 0.0 else -1.0
	if hover_time >= 0.0 and not _dragging:
		var hx := _time_to_x(hover_time)
		draw_line(Vector2(hx, 9.0), Vector2(hx, bar_y + bar_h), Color(Ui.MUTED, 0.6), 1.0, true)

	var tag_time := hover_time if hover_time >= 0.0 else cursor_s
	var tag_text := "%0.3f s" % tag_time
	var tag_width := 62.0
	var tag_x := clampf(_time_to_x(tag_time) - tag_width * 0.5, 0.0, size.x - tag_width)
	var tag := Rect2(Vector2(tag_x, 0.0), Vector2(tag_width, 13.0))
	draw_style_box(_cursor_tag, tag)
	draw_string(Ui.MONO, Vector2(tag.position.x, 10.0), tag_text, HORIZONTAL_ALIGNMENT_CENTER,
		tag_width, 8, Ui.TEXT_DIM)


func _notification(what: int) -> void:
	if what == NOTIFICATION_RESIZED:
		queue_redraw()
