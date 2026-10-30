#!/usr/bin/env python3
"""Self-tests for the Python tooling (no third-party dependencies)."""

from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gen_line_nn  # noqa: E402
import gen_line_r4_nn  # noqa: E402
import gen_motor_nn  # noqa: E402
import gen_profile  # noqa: E402
import golden  # noqa: E402
import identify_motor  # noqa: E402
import motor_teacher  # noqa: E402
import train_motor_nn  # noqa: E402
import train_line_r4  # noqa: E402
import validate_log  # noqa: E402

ROOT = HERE.parents[1]
CONFIG = ROOT / "configs" / "vehicle_default.json"
WEIGHTS = ROOT / "configs" / "motor_nn_weights.json"
LINE_WEIGHTS = ROOT / "configs" / "line_nn_weights.json"
SCHEMA = ROOT / "tools" / "schema" / "run_log.schema.json"

FAILURES: list[str] = []

GOOD_ROW = {
    "t_us": 0,
    "mode": 0,
    "line_conf": 1.0,
    "line_centroid_m": 0.0,
    "line_found": 1,
    "valid_count": 8,
    "v_target": 0.9,
    "omega_target": 0.0,
    "v_max_eff": 1.5,
    "lambda": 1.0,
    "kappa_target": 0.0,
    "wheel_v_l_target": 0.9,
    "wheel_v_r_target": 0.9,
    "wheel_v_l": 0.89,
    "wheel_v_r": 0.89,
    "u_l": 0.55,
    "u_r": 0.55,
    "pwm_l": 0.58,
    "pwm_r": 0.58,
    "v_rail": 12.0,
    "v_rail_valid": 1,
    "safety_faults": 0,
    "safety_latched": 0,
    "safety_derate": 1.0,
    "safety_ramp": 1.0,
    "safety_stop": 0,
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,
    "recovery_ms": 0.0,
}


def check(condition: bool, message: str) -> None:
    if not condition:
        FAILURES.append(message)


def write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def test_profile_generation(tmp: Path) -> None:
    doc = gen_profile.load_profile(CONFIG)
    text = gen_profile.render(doc, str(CONFIG))

    check(".track_width_m = 0.15f" in text, "profile: track width not rendered")
    check(".sensor_count = 8" in text, "profile: sensor count not rendered")
    check(".sensor_position_m = {" in text, "profile: sensor array missing")
    check("nncar_profile_name" in text, "profile: name getter missing")

    broken = json.loads(CONFIG.read_text(encoding="utf-8"))
    del broken["motor"]["kp"]
    broken_path = tmp / "broken.json"
    broken_path.write_text(json.dumps(broken), encoding="utf-8")
    try:
        gen_profile.load_profile(broken_path)
        check(False, "profile: missing key was not rejected")
    except KeyError:
        pass

    extra = json.loads(CONFIG.read_text(encoding="utf-8"))
    extra["motor"]["unexpected"] = 1
    extra_path = tmp / "extra.json"
    extra_path.write_text(json.dumps(extra), encoding="utf-8")
    try:
        gen_profile.load_profile(extra_path)
        check(False, "profile: unknown key was not rejected")
    except KeyError:
        pass


def test_validation(tmp: Path) -> None:
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    columns = schema["x-csv"]["required_columns"]

    rows = []
    for i in range(10):
        row = dict(GOOD_ROW)
        row["t_us"] = i * 2000
        rows.append(row)

    good = tmp / "good.csv"
    write_csv(good, rows, columns)
    report = validate_log.validate(good, schema_path=SCHEMA)
    check(report.ok, f"validator: valid file rejected: {report.errors}")

    bad_t = [dict(r) for r in rows]
    bad_t[5]["t_us"] = 500
    path_t = tmp / "bad_t.csv"
    write_csv(path_t, bad_t, columns)
    report = validate_log.validate(path_t, schema_path=SCHEMA)
    check(not report.ok, "validator: non-monotonic t_us accepted")

    bad_pwm = [dict(r) for r in rows]
    bad_pwm[3]["pwm_l"] = 2.0
    path_pwm = tmp / "bad_pwm.csv"
    write_csv(path_pwm, bad_pwm, columns)
    report = validate_log.validate(path_pwm, schema_path=SCHEMA)
    check(not report.ok, "validator: out-of-range pwm accepted")

    bad_flag = [dict(r) for r in rows]
    bad_flag[2]["line_found"] = 7
    path_flag = tmp / "bad_flag.csv"
    write_csv(path_flag, bad_flag, columns)
    report = validate_log.validate(path_flag, schema_path=SCHEMA)
    check(not report.ok, "validator: invalid flag value accepted")

    manifest = {
        "schema_version": "1.0.0",
        "generator": "selftest",
        "profile": "mg513x_12v_4s_default",
        "profile_version": "1.0.0",
        "profile_verified": False,
        "seed": 0,
        "git_commit": "unknown",
        "tick_dt_s": 0.002,
        "line_dt_s": 0.01,
        "scenario": {
            "duration_s": 1.0,
            "v_desired_mps": 0.9,
            "rail_v": 12.0,
        },
    }
    manifest_path = tmp / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    report = validate_log.validate(good, manifest_path, SCHEMA)
    check(report.ok, f"validator: valid manifest rejected: {report.errors}")

    broken_manifest = dict(manifest)
    del broken_manifest["seed"]
    broken_path = tmp / "manifest_bad.json"
    broken_path.write_text(json.dumps(broken_manifest), encoding="utf-8")
    report = validate_log.validate(good, broken_path, SCHEMA)
    check(not report.ok, "validator: manifest missing key accepted")


def test_golden(tmp: Path) -> None:
    a = tmp / "a.csv"
    b = tmp / "b.csv"
    header = "t_us,v\n0,1.000000\n2000,2.000000\n"
    a.write_text(header, encoding="utf-8")
    b.write_text(header, encoding="utf-8")

    problems = golden.compare(a, b)
    check(not problems, f"golden: identical files reported {problems}")

    c = tmp / "c.csv"
    c.write_text("t_us,v\n0,1.000000\n2000,2.5\n", encoding="utf-8")
    problems = golden.compare(a, c)
    check(bool(problems), "golden: differing value not reported")

    d = tmp / "d.csv"
    d.write_text("t_us,v\n0,1.000001\n2000,2.000001\n", encoding="utf-8")
    problems = golden.compare(a, d)
    check(not problems, f"golden: tolerance too strict {problems}")


def test_identification(tmp: Path) -> None:
    """Identify a synthetic first-order plant and require tight recovery."""
    dt = 0.002
    gain = 1.5
    tau = 0.05
    deadband = 0.07
    rail_nom = 12.0

    plan: list[tuple[float, float]] = []
    plan += [(0.01 * i, 12.0) for i in range(1, 13)]
    plan += [(level, 12.0) for level in (0.2, 0.35, 0.5, 0.7, 0.9)]
    plan += [(0.0, 12.0)]
    plan += [(-0.01 * i, 12.0) for i in range(1, 13)]
    plan += [(level, 12.0) for level in (-0.35, -0.7)]
    plan += [(0.5, rail) for rail in (11.0, 10.5, 12.0)]
    durations = ([0.25] * 12 + [0.40] * 5 + [0.50]
                 + [0.25] * 12 + [0.40] * 2 + [0.40] * 3)

    rows: list[dict] = []
    t_us = 0
    speed = 0.0
    decay = math.exp(-dt / tau)

    for sid, ((pwm, rail), duration) in enumerate(zip(plan, durations), start=1):
        drive = 0.0
        if abs(pwm) > deadband:
            drive = math.copysign((abs(pwm) - deadband) / (1.0 - deadband), pwm)
        settled = gain * drive * rail / rail_nom
        for _ in range(int(round(duration / dt))):
            speed = settled + (speed - settled) * decay
            rows.append({
                "t_us": t_us, "segment_id": sid, "phase": 1,
                "pwm": pwm, "v_rail": rail,
                "v_left": speed, "v_right": speed,
                "w_left": speed, "w_right": speed, "load_nm": 0.0,
            })
            t_us += int(dt * 1e6)

    path = tmp / "bench_selftest.csv"
    write_csv(path, rows, list(identify_motor.REQUIRED_COLUMNS))

    segments, count = identify_motor.load_segments(path)
    result = identify_motor.identify_segments(
        segments, identify_motor.estimate_dt(segments), count, "selftest")

    check(abs(result["deadband"] - deadband) <= 0.005,
          f"identify: deadband {result['deadband']:.4f} != {deadband}")
    for channel in identify_motor.CHANNELS:
        identified = result[channel]
        gain_err = abs(identified["static_gain_mps"] - gain) / gain
        tau_err = abs(identified["tau_s"] - tau) / tau
        check(gain_err < 0.01,
              f"identify: {channel} gain error {gain_err * 100:.2f}%")
        check(tau_err < 0.02,
              f"identify: {channel} tau error {tau_err * 100:.2f}%")

    check(not result["checks"]["warnings"],
          f"identify: unexpected warnings {result['checks']['warnings']}")
    rail_error = result["rail"]["max_rel_error"]
    check(rail_error is not None and rail_error < 0.02,
          f"identify: rail model error {rail_error}")


def test_motor_nn_generation(tmp: Path) -> None:
    doc, weights = gen_motor_nn.load_weights(WEIGHTS)
    text = gen_motor_nn.render(doc, weights, str(WEIGHTS))

    params = sum(int(np) for np in (
        weights["w1"].size, weights["b1"].size, weights["w2"].size,
        weights["b2"].size, weights["w3"].size, weights["b3"].size,
        weights["w4"].size, weights["b4"].size,
    ))
    # 8 inputs x 32 + 32 + 32 x 16 + 16 + 16 x 8 + 8 + 8 x 1 + 1
    check(params == 961, f"motor_nn: parameter count {params} != 961")
    check("nncar_motor_nn_trained" in text, "motor_nn: accessor missing")

    # The JSON matrix is [fan_in, fan_out]; the C struct is [fan_out][fan_in].
    first_row = ", ".join(gen_motor_nn._cfloat(v)
                          for v in weights["w1"].T[0])
    check(f"{{ {first_row} }},", "motor_nn: w1 row was not transposed")

    for name, mutate in (
        ("input_dim", lambda d: d.update(input_dim=9)),
        ("hidden", lambda d: d.update(hidden=[16, 16, 8])),
        ("trained", lambda d: d.update(trained=False)),
    ):
        broken = json.loads(WEIGHTS.read_text(encoding="utf-8"))
        mutate(broken)
        path = tmp / f"broken_{name}.json"
        path.write_text(json.dumps(broken), encoding="utf-8")
        try:
            gen_motor_nn.load_weights(path)
            check(False, f"motor_nn: {name} was not rejected")
        except (KeyError, ValueError):
            pass

    profile = json.loads(CONFIG.read_text(encoding="utf-8"))
    profile["motor"]["alpha"] = 0.5
    path = tmp / "alpha_mismatch.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    try:
        gen_motor_nn.check_alpha(doc, path)
        check(False, "motor_nn: alpha mismatch was not rejected")
    except (KeyError, ValueError):
        pass


def test_motor_training_splits() -> None:
    episode_ids = np.arange(240)
    train_eps, val_eps, test_eps = train_motor_nn.split_episodes(episode_ids, seed=23)
    train_set, val_set, test_set = map(set, (train_eps, val_eps, test_eps))
    check((len(train_set), len(val_set), len(test_set)) == (192, 24, 24),
          "motor train: expected deterministic 80/10/10 episode split")
    check(not (train_set & val_set or train_set & test_set or val_set & test_set),
          "motor train: episode splits overlap")
    check(train_set | val_set | test_set == set(episode_ids),
          "motor train: episode split omitted IDs")
    again = train_motor_nn.split_episodes(episode_ids, seed=23)
    check(all(np.array_equal(a, b) for a, b in zip((train_eps, val_eps, test_eps), again)),
          "motor train: episode split is not reproducible")

    availability = motor_teacher.motor_available_mask(np.array([1, 1, 0, 1, 1]))
    check(np.array_equal(availability, np.array([True, True, False, False, False])),
          "motor teacher: invalid rail must latch the C Safety stop")


def test_line_nn_generation(tmp: Path) -> None:
    doc, weights = gen_line_nn.load_weights(LINE_WEIGHTS)
    text = gen_line_nn.render(doc, weights, str(LINE_WEIGHTS))

    params = sum(int(w.size) for w in weights.values())
    check(params == 7058, f"line_nn: parameter count {params} != 7058")
    check("nncar_line_nn_trained" in text, "line_nn: accessor missing")

    first_row = ", ".join(gen_line_nn._cfloat(v)
                          for v in weights["w1"].T[0])
    check(f"{{ {first_row} }},", "line_nn: w1 row was not transposed")

    for name, mutate in (
        ("input_dim", lambda d: d.update(input_dim=66)),
        ("hidden", lambda d: d.update(hidden=[32, 32, 16])),
        ("output", lambda d: d.update(output="tanh")),
        ("trained", lambda d: d.update(trained=False)),
    ):
        broken = json.loads(LINE_WEIGHTS.read_text(encoding="utf-8"))
        mutate(broken)
        path = tmp / f"broken_line_{name}.json"
        path.write_text(json.dumps(broken), encoding="utf-8")
        try:
            gen_line_nn.load_weights(path)
            check(False, f"line_nn: {name} was not rejected")
        except (KeyError, ValueError):
            pass


def test_line_r4_training_and_generation(tmp: Path) -> None:
    rng = np.random.default_rng(17)
    episodes, rows_per_run, runs_per_episode = 4, 5, 2
    rows = episodes * rows_per_run * runs_per_episode
    body = rng.uniform(-0.8, 0.8, (rows, train_line_r4.ROW_DIM)).astype(np.float32)
    body[:, 7:9] = np.tanh(body[:, 7:9])
    body[:, 9:12] = np.clip(body[:, 9:12], -1.0, 1.0)
    dump = tmp / "line_r4_teacher.bin"
    header = np.asarray([train_line_r4.MAGIC, episodes, rows_per_run,
                         runs_per_episode], dtype=np.int32).view(np.float32)
    np.concatenate((header, body.ravel())).tofile(dump)

    x, y, priv, pd_baseline, ids, count = train_line_r4.load_dump(dump)
    check(count == episodes and x.shape == (rows, 7) and y.shape == (rows, 2),
          "line_r4: binary teacher dimensions")
    check(priv.shape == (rows, 2) and np.unique(ids).size == episodes,
          "line_r4: privileged targets or episode IDs")
    train_ids, val_ids = train_line_r4.split_episodes(ids, seed=17)
    train_mask = np.isin(ids, train_ids)
    val_mask = np.isin(ids, val_ids)
    net = train_line_r4.init_net(np.random.default_rng(17))
    net = train_line_r4.train(
        net, x[train_mask], y[train_mask], priv[train_mask], pd_baseline[train_mask],
        x[val_mask], y[val_mask], priv[val_mask], pd_baseline[val_mask],
        2, 17, 0.2, 0.02, 0.5, 1.0e-3)
    control, privileged = train_line_r4.forward(net, x[val_mask])
    check(np.isfinite(control).all() and np.isfinite(privileged).all(),
          "line_r4: training produced non-finite predictions")

    weights_path = tmp / "line_r4_weights.json"
    metrics = {"val_control_mse": float(np.mean((control - y[val_mask]) ** 2))}
    train_line_r4.export_json(weights_path, net, 17, 2, int(train_mask.sum()),
                              metrics, 0.2, 0.02, 0.5)
    doc, weights = gen_line_r4_nn.load_weights(weights_path)
    params = sum(int(w.size) for w in weights.values())
    check(params == 282, f"line_r4: deployment parameters {params} != 282")
    source = gen_line_r4_nn.render(doc, weights, str(weights_path))
    check("nncar_line_r4_nn_trained" in source, "line_r4: generated accessor missing")
    first_row = ", ".join(gen_line_r4_nn.cfloat(v) for v in weights["w1"].T[0])
    check("{ " + first_row + " }," in source, "line_r4: w1 was not transposed")


def main() -> int:
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        test_profile_generation(tmp)
        test_motor_nn_generation(tmp)
        test_motor_training_splits()
        test_line_nn_generation(tmp)
        test_line_r4_training_and_generation(tmp)
        test_validation(tmp)
        test_golden(tmp)
        test_identification(tmp)

    if FAILURES:
        for failure in FAILURES:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1

    print("python tools: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
