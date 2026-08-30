extends Node
## Sync-free per-frame logger for A/B testing the rendering thread model on
## standalone XR headsets (Quest 2 / Quest 3).
##
## Add as an autoload, or as a child of your XR scene root, then launch with
## `--render-thread safe` or `--render-thread separate` (see run_bench.sh).
##
## IMPORTANT: this only ever reads main-thread-local values per frame. Anything
## routed through RenderingServer (measured render times, draw call counts,
## Performance.RENDER_* monitors) is a `push_and_ret` blocking round-trip to the
## rendering thread. Sampling those every frame adds a sync point that exists
## ONLY in the "Separate" configuration, which would bias the comparison against
## the very thing we are trying to measure. GPU-side truth comes from OVR
## Metrics Tool instead, which costs the app nothing.

@export var warmup_sec: float = 30.0 ## Discarded. Covers shader compilation and clock ramp-up.
@export var duration_sec: float = 300.0 ## Measured window. Use >= 900 for a thermal soak.
@export var target_refresh_hz: float = 72.0 ## Pin both configs to the same rate.
@export var quit_when_done: bool = true

var _xr: XRInterface
var _t_wall := 0.0
var _t_measured := 0.0
var _warm := false
var _done := false

# Preallocated and written by index: CowData::resize() frees the buffer when it
# shrinks, so reserve-then-clear does not work here and append() would realloc
# inside the measured window. Writes stay allocation- and format-free.
var _ts := PackedFloat32Array()
var _delta := PackedFloat32Array()
var _process := PackedFloat32Array()
var _physics := PackedFloat32Array()
var _cap := 0
var _n := 0


func _ready() -> void:
	_xr = XRServer.find_interface("OpenXR")
	if _xr and _xr.is_initialized():
		if _xr.has_method(&"set_display_refresh_rate"):
			_xr.set_display_refresh_rate(target_refresh_hz)
	Engine.max_fps = 0

	# Headroom for a config that runs faster than the pinned rate; we stop at _cap.
	_cap = int(duration_sec * target_refresh_hz * 1.5) + 64
	_ts.resize(_cap)
	_delta.resize(_cap)
	_process.resize(_cap)
	_physics.resize(_cap)

	print("[bench] label=%s refresh=%.1f warmup=%.0fs duration=%.0fs" % [
		_label(), target_refresh_hz, warmup_sec, duration_sec])


func _process(delta: float) -> void:
	if _done:
		return

	_t_wall += delta
	if not _warm:
		if _t_wall < warmup_sec:
			return
		_warm = true
		print("[bench] warmup complete, recording")
		return

	_t_measured += delta
	if _n < _cap:
		_ts[_n] = _t_measured
		_delta[_n] = delta
		# Main-thread local: set by Main::iteration, no server round-trip.
		_process[_n] = Performance.get_monitor(Performance.TIME_PROCESS)
		_physics[_n] = Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS)
		_n += 1

	if _t_measured >= duration_sec or _n >= _cap:
		_finish()


func _label() -> String:
	var args := OS.get_cmdline_args()
	var i := args.find("--render-thread")
	if i != -1 and i + 1 < args.size():
		return args[i + 1]
	return str(ProjectSettings.get_setting("rendering/driver/threads/thread_model", 1))


func _finish() -> void:
	_done = true
	var path := "user://bench_%s_%.0fhz.csv" % [_label(), target_refresh_hz]
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_error("[bench] could not open %s: %s" % [path, error_string(FileAccess.get_open_error())])
		return

	f.store_line("# label=%s refresh=%.1f frames=%d duration=%.2f" % [
		_label(), target_refresh_hz, _n, _t_measured])
	f.store_line("# device=%s renderer=%s" % [
		OS.get_model_name(), ProjectSettings.get_setting("rendering/renderer/rendering_method", "")])
	f.store_line("t_sec,delta_ms,process_ms,physics_ms")
	for i in _n:
		f.store_line("%.4f,%.4f,%.4f,%.4f" % [
			_ts[i], _delta[i] * 1000.0, _process[i], _physics[i]])
	f.close()

	print("[bench] wrote %s (%d frames)" % [path, _n])
	if quit_when_done:
		get_tree().quit()
