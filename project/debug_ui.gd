extends CanvasLayer

var _values: Dictionary = {}  # fmt -> rendered string
var _label: RichTextLabel

var _dirty := false
var _refresh_accumulator := 0.0
const REFRESH_INTERVAL := 0.1  # 10Hz

func _ready() -> void:
	layer = 128
	_label = RichTextLabel.new()
	var mono := SystemFont.new()
	mono.font_names = PackedStringArray([
		"JetBrains Mono", "Fira Code", "Cascadia Code",
		"Consolas",         # Windows
		"Menlo", "Monaco",  # macOS
		"DejaVu Sans Mono", # Linux
		"monospace"         # generic fallback
	])
	_label.add_theme_font_override("normal_font", mono)
	_label.add_theme_font_override("bold_font", mono)
	_label.add_theme_font_size_override("normal_font_size", 14)
	_label.add_theme_font_size_override("bold_font_size", 14)
	_label.bbcode_enabled = true
	_label.fit_content = true
	_label.scroll_active = false
	_label.add_theme_color_override("default_color", Color.WHITE)
	_label.add_theme_constant_override("outline_size", 4)
	_label.add_theme_color_override("font_outline_color", Color.BLACK)
	_label.set_anchors_preset(Control.PRESET_TOP_LEFT)
	_label.position = Vector2(8, 8)
	_label.custom_minimum_size = Vector2(400, 0)
	add_child(_label)

# fmt is both the identity key AND the display template.
# values can be omitted (plain text), a single Variant, or an Array.
func set_value(fmt: String, values: Variant = null) -> void:
	_values[fmt] = fmt if values == null else (fmt % values)
	_dirty = true

func remove_value(fmt: String) -> void:
	_values.erase(fmt)
	_dirty = true

func _refresh() -> void:
	var out := ""
	for k in _values:
		out += _values[k] + "\n"
	_label.text = out

func _process(delta: float) -> void:
	_refresh_accumulator += delta
	if _dirty and _refresh_accumulator >= REFRESH_INTERVAL:
		_refresh_accumulator = 0.0
		_dirty = false
		_refresh()
# created by claude
