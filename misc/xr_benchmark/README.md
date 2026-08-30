# XR rendering thread A/B benchmark

Tooling to decide whether `rendering/driver/threads/thread_model = Separate` is a
win on Quest 2 / Quest 3, and whether it is stable enough to ship.

## What this is actually measuring

The separate rendering thread moves RenderingServer work (culling, draw call
submission, viewport composition) off the main thread. So:

* **CPU-bound-on-main-thread scenes** (lots of GDScript, physics, animation,
  many nodes) should improve.
* **GPU-bound scenes** should be unchanged. That is not a failure, it is the
  shape of the optimisation.
* Total work does not go down. It is redistributed, so the win shows up in the
  *tail* of the frame time distribution, not the average.

The realistic goal is therefore: **it helps where it should, it never hurts, and
it is stable.** "Faster everywhere" is not an outcome this change can produce.

## Why not just look at FPS

On Quest the compositor pins the app to the display refresh rate. If you hit
72/90/120 you are at 72/90/120 in both configurations and the number tells you
nothing. What separates the two configs is:

* the **p95/p99 frame interval** (how bad the bad frames are), and
* the **missed frame rate** (how often you blow the budget and get reprojected).

`analyze_bench.py` reports exactly those, with a 95% confidence interval on the
miss rate so you can tell a real difference from run-to-run noise.

## The measurement trap

Do **not** sample `RenderingServer` state per frame from GDScript
(`viewport_get_measured_render_time_*`, `Performance.RENDER_*`, draw call counts).
Those are `push_and_ret` blocking round-trips to the rendering thread. That sync
point exists *only* in the Separate configuration, so sampling them per frame
penalises the thing you are trying to measure.

`xr_frame_logger.gd` only reads main-thread-local values. GPU-side truth comes
from OVR Metrics Tool, which costs the app nothing.

## Running it

`GodotActivity` reads a `command_line_params` intent extra, so the thread model
flips per launch from one installed APK -- identical binary, assets and shader
cache across both arms:

```sh
adb shell am start -S -n com.example.game/com.godot.game.GodotApp \
    --esa command_line_params "--render-thread,separate"
```

1. Copy `xr_frame_logger.gd` into your project, add it as an autoload.
2. Export once, install.
3. `./run_bench.sh com.example.game both 300 3` -- 3 interleaved reps per arm.
4. ```sh
   python3 analyze_bench.py bench_results/<stamp>/safe_*.csv \
                         -- bench_results/<stamp>/separate_*.csv
   ```

Everything on each side of `--` is pooled into one arm. The analyser prints the
pooled miss rate with its confidence interval *and* the per-run spread, and only
says IMPROVED / REGRESSED when both separate -- frame misses cluster, so the
pooled interval alone is optimistically narrow.

## Protocol that produces a trustworthy answer

* **Fix the workload.** Head motion is the largest source of variance. Put the
  headset on a stand and drive the `XROrigin3D` along a scripted loop, so both
  arms render the same thing.
* **Pin the refresh rate** to the same value in both arms, then repeat at each
  rate you ship (72 / 90, plus 120 on Quest 3).
* **Discard warm-up.** The first seconds are shader compilation and clock ramp.
  The logger already drops 30 s.
* **Interleave and repeat.** safe, separate, safe, separate, safe, separate --
  at least 3 per arm. Thermal state drifts monotonically, so back-to-back
  A-then-B runs confound thread model with temperature.
* **Soak for thermals.** Quest 2 throttles noticeably after ~10-15 min. Use
  `duration_sec >= 900` for the run that decides shipping. The analyser reports
  early-vs-late p95 drift.
* **Test both headsets.** Quest 3 has more CPU headroom, so it will likely show
  a *smaller* improvement than Quest 2. Quest 2 is the device that decides this.

## Proving the pipelining is real

Frame times tell you *whether* it helped. Tracy tells you *why*. This tree is
already instrumented, including an `xrWaitFrame` zone:

```sh
scons platform=android arch=arm64 target=template_debug \
      profiler=tracy profiler_path=/path/to/tracy
adb reverse tcp:8086 tcp:8086   # then connect the Tracy profiler
```

Look for rendering-thread work overlapping the main thread's `xrWaitFrame`
block. If the two do not overlap, the pipelining is not happening and there is a
hidden sync point to find -- that is the single most decisive check.

## Stability matrix (weigh this above the perf numbers)

The EGL context handoff is the risky part of the change, and these are the paths
that touch it. A perf win is bounded; a resume failure is not.

| Case | Watch for |
| --- | --- |
| Take headset off / put back on (proximity) | black screen, frozen main loop |
| System menu, then resume | same |
| Background the app, then resume | `EGL: context did not become current` |
| Long soak (30 min+) | leaks, drift, throttling behaviour |
| Guardian recenter / boundary redraw | surface recreation path |
| App exit and relaunch | teardown ordering |
| **Safe** thread model, same build | unchanged behaviour -- this is the default |

Run with `debug/settings/stdout/verbose_stdout = true` and watch for `EGL:` lines;
every context ownership change is logged with its thread id. There should be a
handful of them per session, at startup, at OpenXR session create/destroy and at
shutdown -- **not** one per frame. Per-frame `EGL:` lines mean the ownership
fast path in `DisplayServerAndroid::gl_window_make_current()` has been broken and
the numbers from that run are worthless.

## Known limitation

Separate-thread rendering with OpenGL on Android is supported **for OpenXR only**.
A non-XR Android GL app renders into the window surface, which the Java rendering
thread recreates across pause/resume without telling the native side; OpenXR does
not care because it presents through the runtime's own swapchains. Leave non-XR
Android projects on the Safe thread model.
