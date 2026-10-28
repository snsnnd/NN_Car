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
artifacts. The checked-in regression plan includes straight, S-curve, oval-map, and
sensor-dropout cases:

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

## Track maps

Choose a JSON polyline map at simulator launch; the same map geometry drives the C
simulation and the Godot live/replay view. For example:

```bash
bash simulator/run_demo.sh --map-file simulator/maps/oval.json --v 0.4
```

To use another track, pass its JSON file with `--map-file`. The format uses `[id, x, y]`
nodes, directed `[from, to]` edges, `units_per_meter`, `line_width_m`, and a start node
with an optional heading/next node. See
[`tools/schema/track_map.schema.json`](../tools/schema/track_map.schema.json). Experiment
plans also accept `scenario.map_file`.

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
- **Map** / `M`: open a JSON track map. In live mode the viewer sends the JSON contents to
  the C simulator, which switches maps and restarts after in-flight steps complete.
- **Parameters** / `P`: edit live speed, start offset/heading, track amplitude/wavelength,
  duration, seed, and rail voltage. Applying parameters resets the live run from its start.
- **Play/Pause** / `Space`: control playback. `R` resets the cursor.
- **Step**: `←`/`→` move 0.1 s, `Shift`+`←`/`→` move 1 s. Speed selector: `0.5×`–`4×`.
- **Seek**: drag the timeline, or click/drag inside a chart after opening **Charts**.
- **Live** / `L`: connect to or stop the C fixed-step service. **Play/Pause**
  controls live stepping.
- **Save** / `S`: write the current live/replay samples to CSV plus a matching manifest.
- **Charts** / `C`: open the six-band telemetry in a detached window; waveforms appear
  only in that separate window.
- **Camera**: `F` follows the car, `V` fits the whole track, `T` switches to a top-down
  map, `0` resets the current view. Right-drag orbits, the wheel zooms, middle-drag pans.
- **Help**: `?` or `F1` opens the controls panel. `F11` toggles fullscreen.

The viewer UI is in Simplified Chinese. It bundles Noto Sans CJK SC as a fallback so the
Chinese text renders even when the host system has no CJK font installed; see
[`assets/fonts/OFL-NotoSansCJK.txt`](assets/fonts/OFL-NotoSansCJK.txt) for its license.

The left rail shows run identity, scenario/seed, primary tracking errors, the magnified
gray-sensor array, and a pinned safety block. The bottom dock contains playback controls
and the run timeline. The detached Charts window contains synchronized strip charts with
stable scales, hover readouts, click-to-seek, wheel time zoom, and tinted fault/recovery
spans. The profile chip reads **MODEL UNVERIFIED** while the vehicle profile is not
hardware-calibrated.

The viewer draws the reference centerline from the CSV's matching manifest and colors
the logged vehicle path by cross-track error (mint on the line, amber and coral as the
error grows). In follow mode a 3D tether shows the signed error at the car. For an
S-curve example, run `nncar_sim` with `--amp 0.08 --wavelength 3.0 --v 0.8`; the default
`--amp 0` is a straight track.

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
