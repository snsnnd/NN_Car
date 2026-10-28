extends RefCounted

## Shared visual language for the NN_Car workbench.
## One graphite surface, one amber signal accent, tabular numerals, hairline rules.

const SANS := preload("res://assets/fonts/Barlow-Regular.ttf")
const SANS_MEDIUM := preload("res://assets/fonts/Barlow-Medium.ttf")
const SANS_SEMIBOLD := preload("res://assets/fonts/Barlow-SemiBold.ttf")
const MONO := preload("res://assets/fonts/IBMPlexMono-Regular.ttf")
const MONO_MEDIUM := preload("res://assets/fonts/IBMPlexMono-Medium.ttf")
const MONO_SEMIBOLD := preload("res://assets/fonts/IBMPlexMono-SemiBold.ttf")
const CJK := preload("res://assets/fonts/NotoSansSC.otf")

const BG := Color("0b0e0c")
const SURFACE := Color("121613")
const SURFACE_2 := Color("171c18")
const SURFACE_3 := Color("1d231e")
const SURFACE_4 := Color("242b25")
const LINE := Color("293028")
const LINE_STRONG := Color("3a4439")
const TEXT := Color("e9ece3")
const TEXT_DIM := Color("b6bdb1")
const MUTED := Color("8b958a")
const FAINT := Color("5e685d")
const ACCENT := Color("f2b45e")
const ACCENT_SOFT := Color("f2b45e")
const MINT := Color("7ad6b6")
const STEEL := Color("8fb6cc")
const ROSE := Color("d98f8f")
const OLIVE := Color("c9cf86")
const DANGER := Color("f26a5a")
const CAUTION := ACCENT

const RADIUS := 10
const RADIUS_SM := 6
const PAD := 14
const GAP := 12
const FONT_SCALE := 1.25

static var _theme: Theme


static func theme() -> Theme:
	if _theme != null:
		return _theme
	for font in [SANS, SANS_MEDIUM, SANS_SEMIBOLD, MONO, MONO_MEDIUM, MONO_SEMIBOLD]:
		var fallbacks: Array[Font] = font.fallbacks
		if not CJK in fallbacks:
			fallbacks.append(CJK)
			font.fallbacks = fallbacks

	var built := Theme.new()
	built.default_font = SANS
	built.default_font_size = 16

	built.set_font("font", "Label", SANS)
	built.set_font_size("font_size", "Label", 16)
	built.set_color("font_color", "Label", TEXT)

	built.set_font("font", "Button", SANS_MEDIUM)
	built.set_font_size("font_size", "Button", 14)
	built.set_color("font_color", "Button", TEXT_DIM)
	built.set_color("font_hover_color", "Button", TEXT)
	built.set_color("font_pressed_color", "Button", ACCENT)
	built.set_color("font_focus_color", "Button", TEXT)
	built.set_color("font_disabled_color", "Button", FAINT)
	built.set_stylebox("normal", "Button", flat(SURFACE, LINE, RADIUS_SM, 1, 10.0, 5.0))
	built.set_stylebox("hover", "Button", flat(SURFACE_2, LINE_STRONG, RADIUS_SM, 1, 10.0, 5.0))
	built.set_stylebox("pressed", "Button", flat(SURFACE_3, ACCENT, RADIUS_SM, 1, 10.0, 5.0))
	built.set_stylebox("disabled", "Button", flat(Color(SURFACE, 0.55), Color(LINE, 0.5), RADIUS_SM, 1, 10.0, 5.0))
	built.set_stylebox("focus", "Button", outline(ACCENT, RADIUS_SM, 10.0, 5.0))
	built.set_constant("h_separation", "Button", 6)

	built.set_font("font", "TooltipLabel", SANS)
	built.set_font_size("font_size", "TooltipLabel", 15)
	built.set_color("font_color", "TooltipLabel", TEXT_DIM)
	built.set_stylebox("panel", "TooltipPanel", flat(SURFACE_3, LINE_STRONG, RADIUS_SM, 1, 9.0, 7.0))

	built.set_stylebox("panel", "PanelContainer", flat(SURFACE, LINE, RADIUS, 1, 0.0, 0.0))
	built.set_stylebox("panel", "Panel", flat(SURFACE, LINE, RADIUS, 1, 0.0, 0.0))

	built.set_stylebox("panel", "PopupMenu", flat(SURFACE_2, LINE_STRONG, RADIUS_SM, 1, 6.0, 6.0))
	built.set_stylebox("hover", "PopupMenu", flat(SURFACE_3, ACCENT, RADIUS_SM - 2, 1, 6.0, 4.0))
	built.set_color("font_color", "PopupMenu", TEXT_DIM)
	built.set_color("font_hover_color", "PopupMenu", TEXT)
	built.set_color("font_accelerator_color", "PopupMenu", MUTED)
	built.set_stylebox("separator", "PopupMenu", line_style(LINE, 1))

	built.set_stylebox("panel", "ScrollContainer", StyleBoxEmpty.new())
	built.set_stylebox("scroll", "VScrollBar", flat(Color(LINE, 0.18), Color(LINE, 0.18), 3, 0, 3.0, 3.0))
	built.set_stylebox("grabber", "VScrollBar", flat(MUTED, MUTED, 3, 0, 3.0, 3.0))
	built.set_stylebox("grabber_highlight", "VScrollBar", flat(TEXT_DIM, TEXT_DIM, 3, 0, 2.0, 2.0))
	built.set_stylebox("grabber_pressed", "VScrollBar", flat(ACCENT, ACCENT, 3, 0, 2.0, 2.0))

	built.set_stylebox("normal", "LineEdit", flat(SURFACE_2, LINE, RADIUS_SM, 1, 9.0, 5.0))
	built.set_stylebox("focus", "LineEdit", outline(ACCENT, RADIUS_SM, 9.0, 5.0))
	built.set_font("font", "LineEdit", MONO)
	built.set_font_size("font_size", "LineEdit", 15)
	built.set_color("font_color", "LineEdit", TEXT)
	built.set_color("font_placeholder_color", "LineEdit", FAINT)
	built.set_color("caret_color", "LineEdit", ACCENT)
	built.set_color("selection_color", "LineEdit", Color(ACCENT, 0.25))

	_theme = built
	return _theme


static func flat(
	bg: Color = SURFACE,
	border: Color = LINE,
	radius: int = RADIUS,
	border_width: int = 1,
	pad_x: float = 0.0,
	pad_y: float = 0.0
) -> StyleBoxFlat:
	var style := StyleBoxFlat.new()
	style.bg_color = bg
	style.border_color = border
	style.set_border_width_all(border_width)
	style.set_corner_radius_all(radius)
	style.content_margin_left = pad_x
	style.content_margin_right = pad_x
	style.content_margin_top = pad_y
	style.content_margin_bottom = pad_y
	style.anti_aliasing = true
	return style


static func outline(border: Color = ACCENT, radius: int = RADIUS_SM, pad_x: float = 0.0,
		pad_y: float = 0.0) -> StyleBoxFlat:
	var style := flat(Color(0, 0, 0, 0), border, radius, 1, pad_x, pad_y)
	style.draw_center = false
	return style


static func line_style(color: Color, thickness: int = 1) -> StyleBoxLine:
	var style := StyleBoxLine.new()
	style.color = color
	style.thickness = thickness
	return style


static func surface_panel(bg: Color = SURFACE, border: Color = LINE, radius: int = RADIUS) -> StyleBoxFlat:
	var style := flat(bg, border, radius, 1, 0.0, 0.0)
	style.shadow_color = Color(0, 0, 0, 0.26)
	style.shadow_size = 8
	style.shadow_offset = Vector2(0, 3)
	return style


static func label(text: String, size: int = 13, color: Color = TEXT) -> Label:
	var node := Label.new()
	node.text = text
	node.add_theme_font_override("font", SANS)
	node.add_theme_font_size_override("font_size", scaled_font_size(size))
	node.add_theme_color_override("font_color", color)
	return node


static func strong_label(text: String, size: int = 13, color: Color = TEXT) -> Label:
	var node := label(text, size, color)
	node.add_theme_font_override("font", SANS_SEMIBOLD)
	return node


static func mono_label(text: String, size: int = 12, color: Color = TEXT, medium: bool = false) -> Label:
	var node := Label.new()
	node.text = text
	node.add_theme_font_override("font", MONO_MEDIUM if medium else MONO)
	node.add_theme_font_size_override("font_size", scaled_font_size(size))
	node.add_theme_color_override("font_color", color)
	return node


static func scaled_font_size(size: int) -> int:
	return maxi(roundi(float(size) * FONT_SCALE), 1)


static func section_header(text: String, hint: String = "") -> HBoxContainer:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 7)
	var tick := ColorRect.new()
	tick.color = ACCENT
	tick.custom_minimum_size = Vector2(2, 11)
	tick.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	row.add_child(tick)
	var title := label(text.to_upper(), 10, TEXT_DIM)
	title.add_theme_font_override("font", SANS_SEMIBOLD)
	row.add_child(title)
	if not hint.is_empty():
		var spacer := Control.new()
		spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(spacer)
		row.add_child(label(hint, 9, FAINT))
	return row


static func divider(color: Color = LINE) -> ColorRect:
	var rule := ColorRect.new()
	rule.color = color
	rule.custom_minimum_size.y = 1
	rule.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	return rule


static func button(text: String, kind: String = "ghost", height: float = 30.0) -> Button:
	var node := Button.new()
	node.text = text
	node.custom_minimum_size.y = height
	node.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	node.focus_mode = Control.FOCUS_ALL
	match kind:
		"primary":
			_primary(node)
		"quiet":
			_quiet(node)
		"danger":
			_danger(node)
		_:
			node.add_theme_font_size_override("font_size", scaled_font_size(10))
	return node


static func toggle(text: String, height: float = 28.0) -> Button:
	var node := Button.new()
	node.text = text
	node.toggle_mode = true
	node.custom_minimum_size.y = height
	node.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	node.focus_mode = Control.FOCUS_ALL
	node.add_theme_font_size_override("font_size", scaled_font_size(10))
	return node


static func _primary(node: Button) -> void:
	node.add_theme_stylebox_override("normal", flat(ACCENT, ACCENT, RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_stylebox_override("hover", flat(ACCENT.lightened(0.1), ACCENT.lightened(0.1),
		RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_stylebox_override("pressed", flat(ACCENT.darkened(0.12), ACCENT, RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_stylebox_override("focus", outline(ACCENT.lightened(0.35), RADIUS_SM, 12.0, 5.0))
	node.add_theme_color_override("font_color", Color("19140b"))
	node.add_theme_color_override("font_hover_color", Color("19140b"))
	node.add_theme_color_override("font_pressed_color", Color("19140b"))
	node.add_theme_color_override("font_focus_color", Color("19140b"))
	node.add_theme_font_override("font", SANS_SEMIBOLD)
	node.add_theme_font_size_override("font_size", scaled_font_size(11))


static func _quiet(node: Button) -> void:
	var empty := flat(Color(0, 0, 0, 0), Color(0, 0, 0, 0), RADIUS_SM, 0, 8.0, 4.0)
	node.add_theme_stylebox_override("normal", empty)
	node.add_theme_stylebox_override("hover", flat(SURFACE_2, Color(0, 0, 0, 0), RADIUS_SM, 0, 8.0, 4.0))
	node.add_theme_stylebox_override("pressed", flat(SURFACE_3, Color(0, 0, 0, 0), RADIUS_SM, 0, 8.0, 4.0))
	node.add_theme_color_override("font_color", MUTED)
	node.add_theme_color_override("font_hover_color", TEXT)
	node.add_theme_font_size_override("font_size", scaled_font_size(10))


static func _danger(node: Button) -> void:
	node.add_theme_stylebox_override("normal", flat(Color(DANGER, 0.12), Color(DANGER, 0.65), RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_stylebox_override("hover", flat(Color(DANGER, 0.2), DANGER, RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_stylebox_override("pressed", flat(Color(DANGER, 0.28), DANGER, RADIUS_SM, 1, 12.0, 5.0))
	node.add_theme_color_override("font_color", DANGER)
	node.add_theme_color_override("font_hover_color", TEXT)
	node.add_theme_font_size_override("font_size", scaled_font_size(10))


static func chip(text: String, color: Color = ACCENT, filled: bool = false) -> PanelContainer:
	var pill := PanelContainer.new()
	var style := flat(Color(color, 0.14 if filled else 0.07), Color(color, 0.55), 4, 1, 7.0, 2.0)
	style.anti_aliasing = true
	pill.add_theme_stylebox_override("panel", style)
	var text_node := label(text, 9, color)
	text_node.add_theme_font_override("font", MONO_MEDIUM)
	pill.add_child(text_node)
	return pill


static func status_dot(color: Color, size: float = 7.0) -> Panel:
	var dot := Panel.new()
	dot.custom_minimum_size = Vector2(size, size)
	dot.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	dot.add_theme_stylebox_override("panel", flat(color, color, int(size), 0))
	return dot


static func wrap_panel(inner: Control, style: StyleBoxFlat, pad: float = PAD) -> PanelContainer:
	var panel := PanelContainer.new()
	panel.add_theme_stylebox_override("panel", style)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", int(pad))
	margin.add_theme_constant_override("margin_right", int(pad))
	margin.add_theme_constant_override("margin_top", int(pad))
	margin.add_theme_constant_override("margin_bottom", int(pad))
	margin.add_child(inner)
	panel.add_child(margin)
	return panel
