#!/usr/bin/env python3
"""Run reproducible, headless NN_Car simulation experiment plans."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean
from typing import Any

from validate_log import validate


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / "simulator" / "runs" / "experiments"
DEFAULT_SIM = ROOT / "build" / "nncar_sim"
PLAN_VERSION = "1.0.0"

DEFAULT_SCENARIO: dict[str, Any] = {
    "duration_s": 8.0,
    "v_desired_mps": 0.9,
    "offset_m": 0.02,
    "heading_deg": 3.0,
    "amp_m": 0.0,
    "wavelength_m": 1.0,
    "rail_v": 12.0,
    "line_width_m": 0.02,
    "motor_inertia_kgm2": 2.0e-5,
    "motor_damping_nms": 4.0e-4,
    "motor_torque_gain_nm": None,
    "motor_stiction_nm": 0.0,
    "gray_noise_std": 0.0,
    "gray_offset": 0.0,
    "gray_gain": 1.0,
    "gray_valid_prob": 1.0,
    "gray_missing_mask": 0,
    "map_file": None,
}

CLI_ARGUMENTS = {
    "duration_s": "--duration",
    "v_desired_mps": "--v",
    "offset_m": "--offset",
    "heading_deg": "--heading-deg",
    "amp_m": "--amp",
    "wavelength_m": "--wavelength",
    "rail_v": "--rail",
    "line_width_m": "--line-width",
    "motor_inertia_kgm2": "--motor-inertia",
    "motor_damping_nms": "--motor-damping",
    "motor_torque_gain_nm": "--motor-torque-gain",
    "motor_stiction_nm": "--stiction-nm",
    "gray_noise_std": "--gray-noise",
    "gray_offset": "--gray-offset",
    "gray_gain": "--gray-gain",
    "gray_valid_prob": "--gray-valid-prob",
}


class PlanError(ValueError):
    pass


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _validate_scenario(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PlanError(f"{where} must be an object")
    unknown = sorted(set(value) - set(DEFAULT_SCENARIO))
    if unknown:
        raise PlanError(f"{where} has unknown parameter(s): {', '.join(unknown)}")

    scenario = DEFAULT_SCENARIO | value
    for key, item in scenario.items():
        if key == "map_file":
            if item is not None and (not isinstance(item, str) or not item.strip()):
                raise PlanError(f"{where}.map_file must be a non-empty path string or null")
            if isinstance(item, str):
                map_path = Path(item)
                if not map_path.is_absolute():
                    map_path = ROOT / map_path
                if not map_path.is_file():
                    raise PlanError(f"{where}.map_file does not exist: {item}")
            continue
        if key == "motor_torque_gain_nm" and item is None:
            continue
        expected_integer = key == "gray_missing_mask"
        if expected_integer:
            if not isinstance(item, int) or isinstance(item, bool):
                raise PlanError(f"{where}.{key} must be an integer")
        elif not _is_number(item):
            raise PlanError(f"{where}.{key} must be a finite number")

    if scenario["duration_s"] <= 0:
        raise PlanError(f"{where}.duration_s must be greater than zero")
    if scenario["v_desired_mps"] < 0:
        raise PlanError(f"{where}.v_desired_mps cannot be negative")
    for key in ("wavelength_m", "rail_v", "line_width_m", "motor_inertia_kgm2", "motor_damping_nms"):
        if scenario[key] <= 0:
            raise PlanError(f"{where}.{key} must be greater than zero")
    for key in ("motor_torque_gain_nm", "motor_stiction_nm", "gray_noise_std"):
        item = scenario[key]
        if item is not None and item < 0:
            raise PlanError(f"{where}.{key} cannot be negative")
    if not -1.0 <= scenario["gray_offset"] <= 1.0:
        raise PlanError(f"{where}.gray_offset must be within [-1, 1]")
    if not 0.0 <= scenario["gray_gain"] <= 2.0:
        raise PlanError(f"{where}.gray_gain must be within [0, 2]")
    if not 0.0 <= scenario["gray_valid_prob"] <= 1.0:
        raise PlanError(f"{where}.gray_valid_prob must be within [0, 1]")
    if not 0 <= scenario["gray_missing_mask"] <= 0xFFFF:
        raise PlanError(f"{where}.gray_missing_mask must be within [0, 65535]")
    return scenario


def load_plan(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        source = path.read_bytes()
        plan = json.loads(source)
    except OSError as exc:
        raise PlanError(f"cannot read plan {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise PlanError(f"invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}") from exc

    if not isinstance(plan, dict):
        raise PlanError("plan root must be an object")
    if set(plan) - {"schema_version", "name", "description", "cases"}:
        raise PlanError("plan contains unsupported top-level fields")
    if plan.get("schema_version") != PLAN_VERSION:
        raise PlanError(f"schema_version must be {PLAN_VERSION}")
    name = plan.get("name")
    if not isinstance(name, str) or not name.strip():
        raise PlanError("name must be a non-empty string")
    if "description" in plan and not isinstance(plan["description"], str):
        raise PlanError("description must be a string")
    cases = plan.get("cases")
    if not isinstance(cases, list) or not cases:
        raise PlanError("cases must be a non-empty array")

    normalized_cases = []
    slugs: set[str] = set()
    for index, case in enumerate(cases):
        where = f"cases[{index}]"
        if not isinstance(case, dict) or set(case) - {"name", "description", "scenario", "seeds"}:
            raise PlanError(f"{where} must contain only name, description, scenario, and seeds")
        case_name = case.get("name")
        if not isinstance(case_name, str) or not case_name.strip():
            raise PlanError(f"{where}.name must be a non-empty string")
        slug = re.sub(r"[^a-z0-9]+", "-", case_name.lower()).strip("-")
        if not slug or slug in slugs:
            raise PlanError(f"{where}.name must produce a unique filesystem-safe name")
        slugs.add(slug)
        seeds = case.get("seeds")
        if not isinstance(seeds, list) or not seeds:
            raise PlanError(f"{where}.seeds must be a non-empty array")
        if any(not isinstance(seed, int) or isinstance(seed, bool) or not 0 <= seed <= 0xFFFFFFFF for seed in seeds):
            raise PlanError(f"{where}.seeds must contain unsigned 32-bit integers")
        if len(set(seeds)) != len(seeds):
            raise PlanError(f"{where}.seeds cannot contain duplicates")
        scenario = _validate_scenario(case.get("scenario", {}), f"{where}.scenario")
        description = case.get("description", "")
        if not isinstance(description, str):
            raise PlanError(f"{where}.description must be a string")
        normalized_cases.append({
            "name": case_name.strip(), "slug": slug, "description": description,
            "scenario": scenario, "seeds": seeds,
        })

    return {"schema_version": PLAN_VERSION, "name": name.strip(),
            "description": plan.get("description", ""), "cases": normalized_cases}, source


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _git_metadata() -> tuple[str | None, bool | None]:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
        dirty = bool(subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=ROOT, text=True,
            stderr=subprocess.DEVNULL,
        ).strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None


def _create_run_root(output_root: Path, plan_name: str) -> Path:
    slug = re.sub(r"[^a-z0-9]+", "-", plan_name.lower()).strip("-") or "experiment"
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base = output_root / f"{slug}-{timestamp}"
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = Path(f"{base}-{suffix:02d}")
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def _format_arg(value: int | float) -> str:
    if isinstance(value, int):
        return str(value)
    return format(value, ".9g")


def _command_for_run(sim_binary: Path, csv_path: Path, manifest_path: Path,
                     scenario: dict[str, Any], seed: int) -> list[str]:
    command = [str(sim_binary), "--out", str(csv_path), "--manifest", str(manifest_path)]
    for key, cli_name in CLI_ARGUMENTS.items():
        value = scenario[key]
        if value is None:
            continue
        command.extend((cli_name, _format_arg(value)))
    map_file = scenario["map_file"]
    if map_file is not None:
        command.extend(("--map-file", map_file))
    command.extend(("--seed", str(seed)))
    return command


def _nearest_map_track(x: float, y: float,
                       segments: list[list[float]]) -> tuple[float, float]:
    best_d2 = math.inf
    best_lateral = 0.0
    best_heading = 0.0
    for x0, y0, x1, y1 in segments:
        dx, dy = x1 - x0, y1 - y0
        length2 = dx * dx + dy * dy
        if length2 <= 1.0e-12:
            continue
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / length2))
        ex, ey = x - (x0 + t * dx), y - (y0 + t * dy)
        distance2 = ex * ex + ey * ey
        if distance2 < best_d2:
            best_d2 = distance2
            best_lateral = (dx * ey - dy * ex) / math.sqrt(length2)
            best_heading = math.atan2(dy, dx)
    if not math.isfinite(best_d2):
        raise ValueError("map manifest has no valid segments")
    return best_lateral, best_heading


def _compute_metrics(csv_path: Path, scenario: dict[str, Any]) -> dict[str, Any]:
    lateral_sq = 0.0
    heading_sq = 0.0
    max_lateral = 0.0
    max_heading = 0.0
    speed_sum = 0.0
    max_pwm = 0.0
    line_lost = 0
    safety_fault_samples = 0
    rows = 0
    first_time: int | None = None
    last_time: int | None = None
    track_scale = 2.0 * math.pi / scenario["wavelength_m"]
    manifest_path = csv_path.with_suffix(".manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    map_data = manifest.get("map")
    map_segments = map_data.get("segments") if isinstance(map_data, dict) else None

    with csv_path.open("r", encoding="utf-8", newline="") as file:
        for row in csv.DictReader(file):
            t_us = int(row["t_us"])
            x, y, theta = float(row["x"]), float(row["y"]), float(row["theta"])
            if map_segments is not None:
                lateral, reference_heading = _nearest_map_track(x, y, map_segments)
            else:
                lateral = y - scenario["amp_m"] * math.sin(track_scale * x)
                slope = scenario["amp_m"] * track_scale * math.cos(track_scale * x)
                reference_heading = math.atan(slope)
            heading = math.atan2(math.sin(theta - reference_heading),
                                 math.cos(theta - reference_heading))
            max_lateral = max(max_lateral, abs(lateral))
            max_heading = max(max_heading, abs(heading))
            lateral_sq += lateral * lateral
            heading_sq += heading * heading
            speed_sum += 0.5 * (float(row["wheel_v_l"]) + float(row["wheel_v_r"]))
            max_pwm = max(max_pwm, abs(float(row["pwm_l"])), abs(float(row["pwm_r"])))
            line_lost += int(row["line_found"]) == 0
            safety_fault_samples += int(row["safety_faults"]) != 0
            first_time = t_us if first_time is None else first_time
            last_time = t_us
            rows += 1

    if rows == 0:
        raise ValueError("simulator produced an empty CSV")
    return {
        "samples": rows,
        "duration_s": 0.0 if first_time is None or last_time is None else (last_time - first_time) * 1.0e-6,
        "max_abs_lateral_error_m": max_lateral,
        "rms_lateral_error_m": math.sqrt(lateral_sq / rows),
        "max_abs_heading_error_deg": math.degrees(max_heading),
        "rms_heading_error_deg": math.degrees(math.sqrt(heading_sq / rows)),
        "mean_forward_speed_mps": speed_sum / rows,
        "max_abs_pwm": max_pwm,
        "line_lost_ratio": line_lost / rows,
        "safety_fault_samples": safety_fault_samples,
    }


def _ensure_sim_binary(path: Path, use_default: bool) -> Path:
    if path.is_file():
        return path
    if not use_default:
        raise FileNotFoundError(f"simulator executable not found: {path}")
    subprocess.run(["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"),
                    "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(ROOT / "build"), "--target", "nncar_sim", "-j2"], check=True)
    if not path.is_file():
        raise FileNotFoundError(f"build completed but simulator executable is missing: {path}")
    return path


def run_plan(plan_path: Path, output_root: Path, simulator_path: Path,
             use_default_simulator: bool) -> tuple[Path, bool]:
    plan, source = load_plan(plan_path)
    simulator_path = _ensure_sim_binary(simulator_path, use_default_simulator)
    simulator_path = simulator_path.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    run_root = _create_run_root(output_root, plan["name"])
    plan_snapshot = {
        "schema_version": plan["schema_version"], "name": plan["name"],
        "description": plan["description"],
        "cases": [{key: case[key] for key in ("name", "description", "scenario", "seeds")}
                  for case in plan["cases"]],
    }
    _write_json(run_root / "plan.json", plan_snapshot)
    (run_root / "plan.source.json").write_bytes(source)
    commit, dirty = _git_metadata()
    summary: dict[str, Any] = {
        "schema_version": "1.0.0", "plan_name": plan["name"],
        "plan_sha256": hashlib.sha256(source).hexdigest(),
        "started_at": datetime.now(timezone.utc).isoformat(),
        "git_commit": commit, "git_dirty": dirty,
        "simulator": str(simulator_path), "status": "running",
        "runs": [], "cases": [],
    }
    summary_path = run_root / "summary.json"
    _write_json(summary_path, summary)
    all_ok = True

    for case in plan["cases"]:
        case_results = []
        for seed in case["seeds"]:
            run_dir = run_root / "runs" / case["slug"] / f"seed-{seed:010d}"
            run_dir.mkdir(parents=True)
            csv_path = run_dir / "run.csv"
            manifest_path = run_dir / "run.manifest.json"
            command = _command_for_run(simulator_path, csv_path, manifest_path,
                                      case["scenario"], seed)
            record: dict[str, Any] = {
                "case": case["name"], "seed": seed,
                "path": run_dir.relative_to(run_root).as_posix(),
                "scenario": case["scenario"], "command": command,
                "status": "running",
                "started_at": datetime.now(timezone.utc).isoformat(),
            }
            summary["runs"].append(record)
            case_results.append(record)
            _write_json(summary_path, summary)
            print(f"[{case['name']} seed={seed}] running", flush=True)
            environment = os.environ.copy()
            if commit:
                environment["GIT_COMMIT"] = commit
            try:
                with (run_dir / "stdout.log").open("w", encoding="utf-8") as stdout_file, \
                     (run_dir / "stderr.log").open("w", encoding="utf-8") as stderr_file:
                    result = subprocess.run(command, cwd=ROOT, env=environment,
                                            stdout=stdout_file, stderr=stderr_file, check=False)
                record["exit_code"] = result.returncode
                if result.returncode != 0:
                    raise RuntimeError(f"nncar_sim exited with status {result.returncode}")
                report = validate(csv_path, manifest_path)
                if not report.ok:
                    raise RuntimeError("run log validation failed: " + "; ".join(report.errors))
                metrics = _compute_metrics(csv_path, case["scenario"])
                record["metrics"] = metrics
                record["validation_warnings"] = report.warnings
                record["status"] = "complete"
                _write_json(run_dir / "parameters.json", {
                    "case": case["name"], "seed": seed, "scenario": case["scenario"]
                })
                print(f"[{case['name']} seed={seed}] ok · {metrics['samples']} samples", flush=True)
            except (OSError, ValueError, RuntimeError) as exc:
                all_ok = False
                record["status"] = "failed"
                record["error"] = str(exc)
                print(f"[{case['name']} seed={seed}] failed: {exc}", file=sys.stderr, flush=True)
            record["finished_at"] = datetime.now(timezone.utc).isoformat()
            _write_json(summary_path, summary)

        successful = [record for record in case_results if record["status"] == "complete"]
        metric_keys = sorted({key for record in successful for key in record.get("metrics", {})
                              if key != "samples"})
        case_summary: dict[str, Any] = {
            "name": case["name"], "runs": len(case_results),
            "successful_runs": len(successful), "metrics_mean": {},
        }
        for key in metric_keys:
            values = [float(record["metrics"][key]) for record in successful]
            case_summary["metrics_mean"][key] = fmean(values)
        summary["cases"].append(case_summary)
        _write_json(summary_path, summary)

    summary["status"] = "complete" if all_ok else "failed"
    summary["finished_at"] = datetime.now(timezone.utc).isoformat()
    _write_json(summary_path, summary)
    print(f"Experiment {summary['status']}: {run_root}")
    return run_root, all_ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="NN_Car headless simulation experiment runner")
    commands = parser.add_subparsers(dest="command", required=True)
    validate_parser = commands.add_parser("validate", help="validate an experiment plan")
    validate_parser.add_argument("plan", type=Path)
    run_parser = commands.add_parser("run", help="execute every case/seed in a plan")
    run_parser.add_argument("plan", type=Path)
    run_parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    run_parser.add_argument("--sim-bin", type=Path, default=DEFAULT_SIM)
    args = parser.parse_args(argv)

    try:
        if args.command == "validate":
            plan, _ = load_plan(args.plan)
            run_count = sum(len(case["seeds"]) for case in plan["cases"])
            print(f"valid plan: {plan['name']} · {len(plan['cases'])} cases · {run_count} runs")
            return 0
        _, success = run_plan(args.plan.resolve(), args.out.resolve(), args.sim_bin,
                              args.sim_bin == DEFAULT_SIM)
        return 0 if success else 1
    except (PlanError, FileNotFoundError, subprocess.CalledProcessError) as exc:
        print(f"simctl: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
