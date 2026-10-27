#!/usr/bin/env python3
"""Smoke-test the C RL protocol, deterministic reset, horizon and replay log."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

from rl_env import NNCarProcessEnv
from validate_log import DEFAULT_SCHEMA, validate


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_rl_env.py path/to/nncar_rl_env", file=sys.stderr)
        return 2

    config = json.loads((Path(__file__).resolve().parents[2] / "configs" / "rl_line_following.json").read_text())
    config["episode"]["horizon_steps"] = 3
    config["domain_randomization"] = {
        "initial_lateral_offset_m": [0.0, 0.0],
        "initial_heading_error_deg": [0.0, 0.0],
        "track_amplitude_m": [0.0, 0.0],
        "track_wavelength_m": [1.0, 1.0],
        "gray_noise_std": [0.0, 0.0],
        "gray_offset": [0.0, 0.0],
        "gray_gain": [1.0, 1.0],
        "gray_valid_probability": [1.0, 1.0],
        "gray_missing_mask_choices": [0],
    }

    with tempfile.TemporaryDirectory(prefix="nncar-rl-") as temp:
        root = Path(temp)
        config_path = root / "rl_config.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        log_dir = root / "logs"
        env = NNCarProcessEnv(binary=sys.argv[1], config_path=config_path, log_dir=log_dir)
        try:
            observation, info = env.reset(seed=23)
            initial_observation = observation
            initial_scenario = dict(info["scenario"])
            assert len(observation["gray_value"]) == 16
            assert len(observation["gray_exist"]) == 16
            assert len(observation["gray_valid"]) == 16
            assert len(observation["wheel_velocity_mps"]) == 2
            assert info["scenario"]["sim_seed"] == env.scenario["sim_seed"]

            for expected_step in (1, 2):
                observation, reward, terminated, truncated, info = env.step([0.0, 0.0])
                assert info["step"] == expected_step
                assert not terminated and not truncated
                assert isinstance(reward, float)

            _, _, terminated, truncated, final_info = env.step([0.0, 0.0])
            assert not terminated and truncated
            assert final_info["termination_reason"] == "time_limit"
            assert env.log_path is not None and env.manifest_path is not None
            report = validate(env.log_path, env.manifest_path, DEFAULT_SCHEMA)
            assert report.ok, "\n".join(report.errors)
            assert report.rows == 3
            try:
                env.step([0.0, 0.0])
                raise AssertionError("step after episode completion must fail")
            except RuntimeError:
                pass

            repeat, _ = env.reset(seed=23)
            assert repeat == initial_observation
            assert env.scenario == initial_scenario
            _, _, terminated, truncated, _ = env.step([0.0, 0.0])
            assert not terminated and not truncated
        finally:
            env.close()

    print("rl_environment: reset/step/reproducibility/log schema ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
