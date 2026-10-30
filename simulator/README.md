# NN_Car simulator and experiment workbench

The C11 host simulator is the source of truth for the controller, motor plant, encoder
and gray-sensor readings. Scenario plans and the headless experiment runner define and
execute repeatable runs; Godot is the optional live viewer and analysis client, not the
simulation system itself.

## Run

From the repository root in WSL/Linux:

```bash
bash simulator/run_demo.sh
```

The script builds `nncar_live` and `nncar_sim`, starts the C server on loopback, and opens
the viewer connected in live mode. The default run is an S curve (`amp=0.08 m`,
`wavelength=3.0 m`) at `0.8 m/s`. Extra arguments are passed to `nncar_live`, so scenario
options can be overridden. The current WSL setup uses Windows Godot with a Linux C
server; they communicate over `127.0.0.1` TCP. Closing Godot stops the server.

## Headless experiment system

Batch experiments are defined in versioned JSON plans, independent of the viewer. A plan
contains named scenarios and explicit seeds; `simctl` validates the plan, runs the C
simulator headlessly, validates every CSV/manifest, and records per-run metrics and
artifacts. The checked-in regression plan includes straight, S-curve, and sensor-dropout
cases:

```bash
python3 tools/python/simctl.py validate simulator/scenarios/regression_suite.json
python3 tools/python/simctl.py run simulator/scenarios/regression_suite.json
```

Each invocation creates a unique directory under `simulator/runs/experiments/` containing
the source and normalized plan, one subdirectory per case/seed (`run.csv`, manifest,
parameters, stdout/stderr), and a live-updated `summary.json` with error, sensor, speed,
PWM, and safety metrics. If `build/nncar_sim` is missing, `simctl` builds that target.
The run folder can be opened in Godot by loading one of its CSV files.

The plan format is described by
[`tools/schema/experiment_plan.schema.json`](../tools/schema/experiment_plan.schema.json).

## Track-map adapter

Pass a JSON map at runtime; adding a map does not require changing or rebuilding the C
simulator. The shared format uses `[id, x, y]` nodes, directed `[from, to]` line-segment
edges, a start node and an optional next node or heading, `units_per_meter`, and
`line_width_m`. Edge order defines the positive side and reference heading; close a loop
with an edge back to the first node. `simulator/maps/oval.json` is a
closed oval example. The file contract is documented in
[`tools/schema/track_map.schema.json`](../tools/schema/track_map.schema.json):

```bash
build/nncar_sim --map-file simulator/maps/oval.json --duration 24 --v 0.4
bash simulator/run_demo.sh --map-file simulator/maps/oval.json --v 0.4
```

Experiment plans accept the same `map_file` scenario field. The C sensor and error
calculations load the map directly, while CSV manifests preserve its geometry for
Godot replay. To add a track, create a map JSON in the same format and pass its path.
The existing analytic straight/S-curve scenarios remain available when no map is given.

Examples:

```bash
# Live curve run at a slower speed.
bash simulator/run_demo.sh --v 0.6 --amp 0.08 --wavelength 3.0 --seed 23

# Lower-rail scenario; still a model scenario, not measured hardware truth.
bash simulator/run_demo.sh --rail 11.0 --seed 23
```

To produce a different run, build the host target and use its existing scenario options:

```bash
cmake -S . -B build
cmake --build build --target nncar_sim
./build/nncar_sim --out simulator/runs/slalom.csv \
  --manifest simulator/runs/slalom.manifest.json \
  --duration 8 --v 0.6 --offset 0 --heading-deg 0 \
  --amp 0.02 --wavelength 0.6 --rail 12.0
godot --path simulator -- --trace simulator/runs/slalom.csv
```

The matching manifest supplies the track amplitude and wavelength. Without a manifest,
the viewer assumes a straight track. New CSVs also contain all 16 `gray_value_NN`,
`gray_exist_NN`, and `gray_valid_NN` channels, so the sensor pods change color during
playback; older logs remain loadable but cannot show per-sensor state.

## Viewer controls

- **Open CSV** / `O`: load a C simulator log.
- **Play/Pause** / `Space`: control playback.
- **Reset** / `R`: return to the beginning.
- Speed selector: `0.5×`, `1×`, `2×` or `4×`; drag the slider to scrub.
- **Live**: connect/disconnect the C fixed-step service; **Play/Pause** controls live stepping.
- **Save**: write the current live/replay samples to CSV plus a matching manifest.
- **Charts ↗**: open the synchronized waveforms in a separate native window. Use the
  window's `+`/`−` or mouse wheel to zoom the time axis, middle-drag to pan, and **Fit**
  to restore the full run. Waveforms are shown only in this separate window.
- **F11**: toggle fullscreen.
- **Right-drag**: orbit; mouse wheel: zoom; **middle-drag**: pan.
- **F**: toggle follow/free camera; **V**: frame the full track; **0**: reset the follow view.

The viewer draws the reference centerline from the CSV's matching manifest and overlays
the logged vehicle path in teal. For an S-curve example, run `nncar_sim` with
`--amp 0.08 --wavelength 3.0 --v 0.8`; the default `--amp 0` is a straight track.

The architecture, loopback protocol, timing contract and waveform definitions are in
[`docs/06_仿真器架构与遥测设计.md`](../docs/06_仿真器架构与遥测设计.md).

The low-poly vehicle source is `assets_source/nncar_vehicle.blend`; its runtime GLB is
`simulator/assets/nncar_vehicle.glb`. Rebuild both from the current `vehicle_default.json`
profile with Blender:

```bash
blender --background --python simulator/assets/build_vehicle.py
```

The wheel radius, drive-wheel spacing and gray-array positions are read from the vehicle
profile. Chassis length, sensor-package height and passive rear caster are explicitly
visual assumptions because those dimensions are not yet in the hardware profile. The
coordinates follow the project convention: vehicle-forward is `+X`, left is `+Y`, and
positive yaw is counter-clockwise. Blender/GLTF conversion maps the model's forward axis
to Godot's local `-Z`.

For full model assumptions and current fidelity limits, see
[`docs/05_仿真平台与建模.md`](../docs/05_仿真平台与建模.md).

## RL environment scaffold

The RL scaffold keeps the environment and rollout truth in C. The policy replaces
only the line-following output; the existing C mixer, safety, motor controller, encoder
model and plant continue to run. One environment step is one 10 ms control interval
(five 2 ms motor ticks), so the action/observation rate is 100 Hz.

Build the line-following environment process and create a deterministic rollout log:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target nncar_rl_env -j2
uv venv
uv pip install --python .venv/bin/python -r requirements-rl.txt
.venv/bin/python tools/python/rl_env.py --episodes 1 --seed 23 \
  --action-speed 0 --action-curvature 0 \
  --log-dir simulator/runs/rl_preview
```

The process bridge is `tools/python/rl_env.py`; the base wrapper needs only Python's
standard library. Install the optional training interface with uv:
`uv venv && uv pip install --python .venv/bin/python -r requirements-rl.txt`, then use `GymnasiumLineFollowingEnv`. Each
completed episode writes the standard CSV + manifest format, which can be inspected in
the existing Godot scene:

```python
from tools.python.rl_env import GymnasiumLineFollowingEnv

env = GymnasiumLineFollowingEnv()
observation, info = env.reset(seed=23)
observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
env.close()
```

```bash
godot --path simulator -- --trace simulator/runs/rl_preview/episode_000001.csv
```

Environment contract and reward/randomization ranges live in
[`configs/rl_line_following.json`](../configs/rl_line_following.json). This is an
environment scaffold, not a trained agent or a claim that the current vehicle model is
real-world calibrated.

## Current plant fidelity boundary

The workbench has deterministic headless execution and experiment archiving, live C
step control, CSV/manifest validation, and replay/analysis. It is not a live Godot physics
simulation or a C/Godot shared-library bridge. The C model is fixed-step and reproducible,
but still uses an ideal no-slip differential-drive chassis and a synthetic reflectance
sensor. It does not yet model tire forces, chassis mass/yaw inertia, photoelectric optics,
sensor height/latency or battery sag. Do not treat the default parameters as a certified
digital twin: the vehicle profile currently says `verified=false`.
