#!/usr/bin/env python3
"""Python bridge for the deterministic C line-following RL environment.

The base process wrapper has no training-framework dependency. If Gymnasium is
installed, ``GymnasiumLineFollowingEnv`` exposes the same task with standard
Box/Dict spaces.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "configs" / "rl_line_following.json"
PROFILE_PATH = ROOT / "configs" / "vehicle_default.json"
SENSOR_COUNT = 16

CSV_COLUMNS = [
    "t_us", "mode", "line_conf", "line_centroid_m", "line_found", "valid_count",
    "v_target", "omega_target", "v_max_eff", "lambda", "kappa_target",
    "wheel_v_l_target", "wheel_v_r_target", "wheel_v_l", "wheel_v_r",
    "u_l", "u_r", "pwm_l", "pwm_r", "v_rail", "v_rail_valid",
    "safety_faults", "safety_latched", "safety_derate", "safety_ramp", "safety_stop",
    "x", "y", "theta", "recovery_ms",
]
for _sensor_index in range(SENSOR_COUNT):
    _suffix = f"{_sensor_index:02d}"
    CSV_COLUMNS.extend((f"gray_value_{_suffix}", f"gray_exist_{_suffix}", f"gray_valid_{_suffix}"))
CSV_COLUMNS.extend(("rl_action_speed", "rl_action_curvature", "rl_action_applied", "rl_reward", "rl_terminated", "rl_truncated"))


class NNCarProcessEnv:
    """One persistent C simulator process with reset/step semantics.

    ``reset`` returns ``(observation, info)`` and ``step`` returns the standard
    five-tuple. Actions are ``[speed_scale, curvature_norm]`` in [-1, 1].
    """

    def __init__(
        self,
        binary: str | Path | None = None,
        config_path: str | Path = DEFAULT_CONFIG,
        log_dir: str | Path | None = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        binary_from_env = os.environ.get("NNCAR_RL_ENV_BIN")
        self.binary = Path(binary or binary_from_env or ROOT / "build" / "nncar_rl_env")
        if not self.binary.is_file():
            raise FileNotFoundError(
                f"Missing {self.binary}; build the C environment with "
                "cmake --build build --target nncar_rl_env"
            )
        self.log_dir = Path(log_dir) if log_dir is not None else None
        self.process: subprocess.Popen[str] | None = None
        self._rng = random.Random()
        self._done = True
        self._episode = 0
        self._episode_steps = 0
        self.control_dt_s = 1.0 / float(self.config["episode"]["control_frequency_hz"])
        self._log_file = None
        self._log_writer: csv.DictWriter[str] | None = None
        self.log_path: Path | None = None
        self.manifest_path: Path | None = None
        self.scenario: dict[str, float | int] = {}

    def _start(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        episode = self.config["episode"]
        self.process = subprocess.Popen(
            [
                str(self.binary),
                "--horizon-steps", str(int(episode["horizon_steps"])),
                "--terminate-lateral-m", str(float(episode["terminate_lateral_error_m"])),
                "--terminate-heading-deg", str(float(episode["terminate_heading_error_deg"])),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def _exchange(self, command: str) -> dict[str, Any]:
        self._start()
        assert self.process is not None
        assert self.process.stdin is not None
        assert self.process.stdout is not None
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("C RL environment exited without a response")
        result = json.loads(line)
        if "error" in result:
            raise RuntimeError(f"C RL environment: {result['error']}")
        return result

    @staticmethod
    def _uniform(rng: random.Random, interval: list[float]) -> float:
        return rng.uniform(float(interval[0]), float(interval[1]))

    def _sample_scenario(self, options: dict[str, Any]) -> dict[str, float | int]:
        randomization = self.config["domain_randomization"]
        scenario: dict[str, float | int] = {
            "offset_m": self._uniform(rng=self._rng, interval=randomization["initial_lateral_offset_m"]),
            "heading_error_deg": self._uniform(rng=self._rng, interval=randomization["initial_heading_error_deg"]),
            "amp_m": self._uniform(rng=self._rng, interval=randomization["track_amplitude_m"]),
            "wavelength_m": self._uniform(rng=self._rng, interval=randomization["track_wavelength_m"]),
            "gray_noise_std": self._uniform(rng=self._rng, interval=randomization["gray_noise_std"]),
            "gray_offset": self._uniform(rng=self._rng, interval=randomization["gray_offset"]),
            "gray_gain": self._uniform(rng=self._rng, interval=randomization["gray_gain"]),
            "gray_valid_probability": self._uniform(rng=self._rng, interval=randomization["gray_valid_probability"]),
            "gray_missing_mask": int(self._rng.choice(randomization["gray_missing_mask_choices"])),
            "sim_seed": self._rng.randrange(0, 2**32),
        }
        unknown_options = set(options) - set(scenario)
        if unknown_options:
            raise ValueError(f"unknown reset option(s): {', '.join(sorted(unknown_options))}")
        for name in scenario:
            if name in options:
                scenario[name] = options[name]
        return scenario

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, list[float] | list[int]], dict[str, Any]]:
        if seed is not None:
            self._rng = random.Random(int(seed))
        options = options or {}
        self.scenario = self._sample_scenario(options)
        episode_cfg = self.config["episode"]
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        geometry = profile["geometry"]
        configured_dt = 1.0 / float(episode_cfg["control_frequency_hz"])
        profile_dt = float(profile["timing"]["line_dt_s"])
        if not math.isclose(configured_dt, profile_dt, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError("RL control_frequency_hz must match the C profile line loop")
        motor_inertia = 2.0e-5
        motor_damping = 4.0e-4
        torque_gain = motor_damping * float(profile["speed"]["v_max_nom_mps"]) / float(geometry["wheel_radius_m"])
        line_width_m = 0.02
        sim_seed = int(self.scenario["sim_seed"]) & 0xFFFFFFFF

        command_values = [
            sim_seed,
            self.scenario["offset_m"],
            self.scenario["heading_error_deg"],
            self.scenario["amp_m"],
            self.scenario["wavelength_m"],
            line_width_m,
            self.scenario["gray_noise_std"],
            self.scenario["gray_offset"],
            self.scenario["gray_gain"],
            self.scenario["gray_valid_probability"],
            int(self.scenario["gray_missing_mask"]),
            episode_cfg["desired_speed_mps"],
        ]
        command = "RESET " + " ".join(
            str(value) if isinstance(value, int) else f"{float(value):.9g}"
            for value in command_values
        )
        response = self._exchange(command)
        if response.get("event") != "reset":
            raise RuntimeError("Unexpected event from C RL environment reset")

        self._episode = int(response["episode"])
        self._episode_steps = 0
        self.control_dt_s = float(response["info"]["control_dt_s"])
        self._done = False
        self._open_episode_log(
            profile,
            response,
            sim_seed,
            line_width_m,
            motor_inertia,
            motor_damping,
            torque_gain,
        )
        info = dict(response["info"])
        info["scenario"] = dict(self.scenario)
        return response["observation"], info

    def step(
        self, action: list[float] | tuple[float, float]
    ) -> tuple[dict[str, list[float] | list[int]], float, bool, bool, dict[str, Any]]:
        if self._done:
            raise RuntimeError("Call reset() before stepping or after an episode ends")
        if len(action) != 2:
            raise ValueError("action must contain [speed_scale, curvature_norm]")
        a0, a1 = float(action[0]), float(action[1])
        if not math.isfinite(a0) or not math.isfinite(a1) or abs(a0) > 1.0 or abs(a1) > 1.0:
            raise ValueError("both action values must be finite and in [-1, 1]")
        response = self._exchange(f"STEP {a0:.9g} {a1:.9g}")
        if response.get("event") != "transition":
            raise RuntimeError("Unexpected event from C RL environment step")
        self._episode_steps = int(response["info"]["step"])
        terminated = bool(response["terminated"])
        truncated = bool(response["truncated"])
        self._done = terminated or truncated
        info = dict(response["info"])
        self._write_log_row(response, a0, a1)
        if self._done and self._log_file is not None:
            self._log_file.flush()
        return response["observation"], float(response["reward"]), terminated, truncated, info

    def _open_episode_log(
        self,
        profile: dict[str, Any],
        reset_response: dict[str, Any],
        sim_seed: int,
        line_width_m: float,
        motor_inertia: float,
        motor_damping: float,
        torque_gain: float,
    ) -> None:
        self._close_episode_log()
        if self.log_dir is None:
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stem = f"episode_{self._episode:06d}"
        self.log_path = self.log_dir / f"{stem}.csv"
        self.manifest_path = self.log_dir / f"{stem}.manifest.json"
        self._log_file = self.log_path.open("w", encoding="utf-8", newline="")
        self._log_writer = csv.DictWriter(self._log_file, fieldnames=CSV_COLUMNS)
        self._log_writer.writeheader()

        episode = self.config["episode"]
        dt_s = float(reset_response["info"]["control_dt_s"])
        manifest = {
            "schema_version": "1.1.0",
            "generator": "nncar_rl_env",
            "profile": profile["name"],
            "profile_version": profile["profile_version"],
            "profile_verified": bool(profile.get("verified", False)),
            "seed": sim_seed,
            "git_commit": os.environ.get("GIT_COMMIT", "unknown"),
            "tick_dt_s": float(profile["timing"]["motor_dt_s"]),
            "line_dt_s": float(profile["timing"]["line_dt_s"]),
            "scenario": {
                "duration_s": int(episode["horizon_steps"]) * dt_s,
                "v_desired_mps": float(episode["desired_speed_mps"]),
                "offset_m": float(self.scenario["offset_m"]),
                "heading_deg": float(self.scenario["heading_error_deg"]),
                "amp_m": float(self.scenario["amp_m"]),
                "wavelength_m": float(self.scenario["wavelength_m"]),
                "rail_v": float(profile["rail"]["nominal_v"]),
            },
            "vehicle": {
                "wheel_radius_m": float(profile["geometry"]["wheel_radius_m"]),
                "track_width_m": float(profile["geometry"]["track_width_m"]),
                "sensor_count": int(profile["geometry"]["sensor_count"]),
                "sensor_to_axle_m": float(profile["geometry"]["sensor_to_axle_m"]),
                "sensor_position_m": list(profile["geometry"]["sensor_position_m"][: int(profile["geometry"]["sensor_count"])]),
            },
            "simulation": {
                "track_model": "sine_centerline",
                "line_width_m": line_width_m,
                "motor_model": "first_order_dc_motor",
                "motor_inertia_kgm2": motor_inertia,
                "motor_damping_nms": motor_damping,
                "motor_torque_gain_nm": torque_gain,
                "motor_stiction_nm": 0.0,
                "gray_model": "top_hat_linear_falloff",
                "gray_noise_std": float(self.scenario["gray_noise_std"]),
                "gray_offset": float(self.scenario["gray_offset"]),
                "gray_gain": float(self.scenario["gray_gain"]),
                "gray_valid_prob": float(self.scenario["gray_valid_probability"]),
                "gray_missing_mask": int(self.scenario["gray_missing_mask"]),
            },
            "rl_environment": {
                "environment_id": self.config["environment_id"],
                "action_contract": self.config["action"],
                "reward_formula": self.config["reward"]["formula"],
            },
        }
        self.manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    def _write_log_row(self, response: dict[str, Any], action_speed: float, action_curvature: float) -> None:
        if self._log_writer is None:
            return
        info = response["info"]
        observation = response["observation"]
        row: dict[str, Any] = {
            "t_us": round(self._episode_steps * self.control_dt_s * 1.0e6),
            "mode": info["mode"],
            "line_conf": info["line_confidence"],
            "line_centroid_m": info["line_centroid_m"],
            "line_found": info["line_found"],
            "valid_count": info["valid_count"],
            "v_target": info["v_target"],
            "omega_target": info["omega_target"],
            "v_max_eff": info["v_max_eff"],
            "lambda": info["lambda"],
            "kappa_target": info["kappa_target"],
            "wheel_v_l_target": info["wheel_target"][0],
            "wheel_v_r_target": info["wheel_target"][1],
            "wheel_v_l": observation["wheel_velocity_mps"][0],
            "wheel_v_r": observation["wheel_velocity_mps"][1],
            "u_l": info["u"][0],
            "u_r": info["u"][1],
            "pwm_l": info["pwm"][0],
            "pwm_r": info["pwm"][1],
            "v_rail": info["v_rail"],
            "v_rail_valid": 1,
            "safety_faults": info["safety_faults"],
            "safety_latched": info["safety_latched"],
            "safety_derate": info["safety_derate"],
            "safety_ramp": info["safety_ramp"],
            "safety_stop": info["safety_stop"],
            "x": info["x"],
            "y": info["y"],
            "theta": info["theta"],
            "recovery_ms": 0.0,
            "rl_action_speed": action_speed,
            "rl_action_curvature": action_curvature,
            "rl_action_applied": int(info["policy_action_applied"]),
            "rl_reward": response["reward"],
            "rl_terminated": int(response["terminated"]),
            "rl_truncated": int(response["truncated"]),
        }
        for index in range(SENSOR_COUNT):
            suffix = f"{index:02d}"
            row[f"gray_value_{suffix}"] = observation["gray_value"][index]
            row[f"gray_exist_{suffix}"] = observation["gray_exist"][index]
            row[f"gray_valid_{suffix}"] = observation["gray_valid"][index]
        self._log_writer.writerow(row)

    def _close_episode_log(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
        self._log_file = None
        self._log_writer = None

    def close(self) -> None:
        self._close_episode_log()
        if self.process is None:
            return
        if self.process.poll() is None:
            if self.process.stdin is not None:
                self.process.stdin.write("CLOSE\n")
                self.process.stdin.flush()
                self.process.stdin.close()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=2.0)
        self.process = None

    def __enter__(self) -> "NNCarProcessEnv":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


try:
    import gymnasium as gym
    import numpy as np
    from gymnasium import spaces
except ImportError:
    gym = None
    np = None
    spaces = None


if gym is not None:
    class GymnasiumLineFollowingEnv(gym.Env):
        """Optional Gymnasium adapter around ``NNCarProcessEnv``."""

        metadata = {"render_modes": []}

        def __init__(self, **kwargs: Any) -> None:
            super().__init__()
            self.env = NNCarProcessEnv(**kwargs)
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
            self.observation_space = spaces.Dict({
                "gray_value": spaces.Box(-1.0, 1.0, shape=(SENSOR_COUNT,), dtype=np.float32),
                "gray_exist": spaces.MultiBinary(SENSOR_COUNT),
                "gray_valid": spaces.MultiBinary(SENSOR_COUNT),
                "wheel_velocity_mps": spaces.Box(-3.0, 3.0, shape=(2,), dtype=np.float32),
            })

        @staticmethod
        def _convert_observation(observation: dict[str, list[float] | list[int]]) -> dict[str, Any]:
            return {
                "gray_value": np.asarray(observation["gray_value"], dtype=np.float32),
                "gray_exist": np.asarray(observation["gray_exist"], dtype=np.int8),
                "gray_valid": np.asarray(observation["gray_valid"], dtype=np.int8),
                "wheel_velocity_mps": np.asarray(observation["wheel_velocity_mps"], dtype=np.float32),
            }

        def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
            super().reset(seed=seed)
            observation, info = self.env.reset(seed=seed, options=options)
            return self._convert_observation(observation), info

        def step(self, action: Any):
            observation, reward, terminated, truncated, info = self.env.step(action)
            return self._convert_observation(observation), reward, terminated, truncated, info

        def close(self) -> None:
            self.env.close()
else:
    class GymnasiumLineFollowingEnv:
        def __init__(self, **_kwargs: Any) -> None:
            raise ImportError("Install optional RL dependencies with: pip install -r requirements-rl.txt")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--log-dir", type=Path, default=ROOT / "simulator" / "runs" / "rl")
    parser.add_argument("--action-speed", type=float, default=0.0)
    parser.add_argument("--action-curvature", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes must be positive")

    with NNCarProcessEnv(binary=args.binary, log_dir=args.log_dir) as env:
        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            del observation
            while True:
                _, _, terminated, truncated, info = env.step([args.action_speed, args.action_curvature])
                if terminated or truncated:
                    print(
                        f"episode={episode + 1} steps={info['step']} "
                        f"reason={info['termination_reason']} log={env.log_path}"
                    )
                    break
    return 0


if __name__ == "__main__":
    sys.exit(main())
