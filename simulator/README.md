# Godot simulation viewer

This is the simulation workbench for NN_Car. The C11 host simulator remains the source
of truth for the controller, motor plant, encoder and gray-sensor readings; Godot replays
its CSV output rather than reimplementing those dynamics in GDScript.

## Run

From the repository root in WSL/Linux:

```bash
bash simulator/run_demo.sh
```

The script builds `nncar_sim`, writes an 8-second straight-track run under the ignored
`simulator/runs/` directory, then opens the viewer with that run. Extra arguments are
passed to `nncar_sim`, so you can override the default scenario and inject sensor faults.
It uses the `godot` command on `PATH`; Godot 4.7.2 is installed in the current WSL
environment. If that command is unavailable, open `simulator/project.godot` in Godot
4.7.2 and load the generated CSV with **Open CSV**.

Examples:

```bash
# Slalom with deterministic sensor noise, 1 of 8 installed channels disconnected.
bash simulator/run_demo.sh --v 0.6 --amp 0.02 --wavelength 0.6 \
  --gray-noise 0.04 --gray-valid-prob 0.98 --gray-missing-mask 0x04 --seed 23

# Stiction stress case; values are model parameters until measured on the real car.
bash simulator/run_demo.sh --stiction-nm 0.0001 --rail 11.0 --seed 23
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

## Current boundary

This milestone is a deterministic 3D replay/inspection workbench, not a live Godot
physics simulation or C/Godot shared-library bridge. The C model is fixed-step and
reproducible, but still uses an ideal no-slip differential-drive chassis and a synthetic
reflectance sensor. It does not yet model tire forces, chassis mass/yaw inertia,
photoelectric optics, sensor height/latency or battery sag. Do not treat the default
parameters as a certified digital twin: the vehicle profile currently says
`verified=false`.
