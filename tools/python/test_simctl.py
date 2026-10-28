#!/usr/bin/env python3
"""Integration test for reproducible experiment-plan execution."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SIMCTL = ROOT / "tools" / "python" / "simctl.py"


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_simctl.py NNCAR_SIM_BINARY", file=sys.stderr)
        return 2
    simulator = Path(sys.argv[1]).resolve()
    plan = {
        "schema_version": "1.0.0",
        "name": "determinism-check",
        "cases": [{
            "name": "short-straight",
            "seeds": [23],
            "scenario": {"duration_s": 0.1, "v_desired_mps": 0.6, "amp_m": 0.0},
        }],
    }
    with tempfile.TemporaryDirectory(prefix="nncar-simctl-") as temporary:
        temp = Path(temporary)
        plan_path = temp / "plan.json"
        plan_path.write_text(json.dumps(plan), encoding="utf-8")

        validation = subprocess.run(
            [sys.executable, str(SIMCTL), "validate", str(plan_path)],
            cwd=ROOT, capture_output=True, text=True, check=False,
        )
        if validation.returncode != 0:
            raise AssertionError(validation.stderr or validation.stdout)

        outputs = []
        for iteration in range(2):
            output_root = temp / f"runs-{iteration}"
            execution = subprocess.run(
                [sys.executable, str(SIMCTL), "run", str(plan_path),
                 "--out", str(output_root), "--sim-bin", str(simulator)],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            if execution.returncode != 0:
                raise AssertionError(execution.stderr or execution.stdout)
            run_root = next(output_root.iterdir())
            summary = json.loads((run_root / "summary.json").read_text(encoding="utf-8"))
            if summary["status"] != "complete" or len(summary["runs"]) != 1:
                raise AssertionError("experiment summary did not complete one run")
            record = summary["runs"][0]
            if record["metrics"]["samples"] < 2 or record["status"] != "complete":
                raise AssertionError("experiment did not produce valid telemetry metrics")
            run_dir = run_root / record["path"]
            if not (run_dir / "run.manifest.json").is_file() or not (run_dir / "parameters.json").is_file():
                raise AssertionError("run artifacts are incomplete")
            outputs.append(((run_dir / "run.csv").read_bytes(), record["metrics"]))

        if outputs[0] != outputs[1]:
            raise AssertionError("identical plan/seed did not reproduce identical CSV and metrics")

        invalid_plan = temp / "invalid.json"
        invalid_plan.write_text(json.dumps({
            "schema_version": "1.0.0", "name": "bad",
            "cases": [{"name": "bad-case", "seeds": [1],
                       "scenario": {"v_desired_mps": -1}}],
        }), encoding="utf-8")
        invalid = subprocess.run(
            [sys.executable, str(SIMCTL), "validate", str(invalid_plan)],
            cwd=ROOT, capture_output=True, text=True, check=False,
        )
        if invalid.returncode == 0:
            raise AssertionError("invalid experiment plan was accepted")

        regression_output = temp / "regression-suite"
        regression = subprocess.run(
            [sys.executable, str(SIMCTL), "run",
             str(ROOT / "simulator" / "scenarios" / "regression_suite.json"),
             "--out", str(regression_output), "--sim-bin", str(simulator)],
            cwd=ROOT, capture_output=True, text=True, check=False,
        )
        if regression.returncode != 0:
            raise AssertionError(regression.stderr or regression.stdout)
        regression_root = next(regression_output.iterdir())
        regression_summary = json.loads(
            (regression_root / "summary.json").read_text(encoding="utf-8")
        )
        if regression_summary["status"] != "complete" or len(regression_summary["runs"]) != 4:
            raise AssertionError("checked-in regression suite did not complete all cases")
        oval_run = next(run for run in regression_summary["runs"] if run["case"] == "oval-map")
        if oval_run["status"] != "complete" or "rms_lateral_error_m" not in oval_run["metrics"]:
            raise AssertionError("runtime map track did not produce valid metrics")

    print("simctl: validation, artifacts, metrics, deterministic rerun, and suite ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
