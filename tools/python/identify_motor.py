#!/usr/bin/env python3
"""Identify per-wheel first-order DC motor parameters from a bench CSV.

Model (the bench log is exported without deadband compensation, so the
deadband stays observable):

    drive   = sign(pwm) * max(0, (|pwm| - deadband) / (1 - deadband))
    u_eff   = drive * v_rail / v_rail_nom
    v_ss    = static_gain * u_eff
    v(t)    = v_ss + (v_0 - v_ss) * exp(-t / tau)

Identification steps:

1. Every excitation segment is fitted with ``y[k] = A + B * lambda^k`` over the
   sample index (exact for a first-order plant sampled at a constant dt).
   ``A`` is the settled speed, ``lambda = exp(-dt / tau)``.
2. The deadband and the static gain come from the slope/intercept of
   ``v_ss`` versus ``|pwm|``:  ``v_ss = a * |pwm| + b`` with
   ``deadband = -b / a`` and ``static_gain = a + b``.  This needs no prior
   knowledge of the deadband.  A deadband staircase at 0.01 PWM steps provides
   an independent quantized estimate for cross-checking.
3. ``tau`` is the median over well-excited segments.  Segments close to the
   deadband are excluded because stiction is not part of the linear model.
4. Phase-4 segments (same PWM, different rail voltage) validate
   ``v_ss proportional to v_rail``.

Usage:
    python3 tools/python/identify_motor.py bench.csv --out motor_identified.json
    python3 tools/python/identify_motor.py bench.csv --truth truth.json

Exit code 0 = ok, 1 = identification/truth-check failure, 2 = usage error.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

REQUIRED_COLUMNS = (
    "t_us", "segment_id", "phase", "pwm", "v_rail",
    "v_left", "v_right", "w_left", "w_right", "load_nm",
)
CHANNELS = ("left", "right")
VALUE_COLUMN = {"left": "v_left", "right": "v_right"}

GAIN_MIN_DUTY = 0.25        # only well-excited segments feed gain/tau
RAIL_MATCH_V = 0.05         # |v_rail - nominal| tolerance for nominal segments
LAMBDA_MIN = 0.5
LAMBDA_MAX = 0.9999
STAIRCASE_MAX_PWM = 0.12
DEFAULT_TOL = {"gain": 0.03, "tau": 0.10, "deadband": 0.01}


class Fit:
    __slots__ = ("asymptote", "tau_s", "lam", "tau_ok")

    def __init__(self, asymptote: float, tau_s: float | None,
                 lam: float, tau_ok: bool) -> None:
        self.asymptote = asymptote
        self.tau_s = tau_s
        self.lam = lam
        self.tau_ok = tau_ok


class Segment:
    def __init__(self, sid: int, phase: int, pwm: float, rail: float) -> None:
        self.sid = sid
        self.phase = phase
        self.pwm = pwm
        self.rail = rail
        self.t_us: list[int] = []
        self.values: dict[str, list[float]] = {ch: [] for ch in CHANNELS}
        self.fit: dict[str, Fit] = {}


def load_segments(path: Path) -> tuple[list[Segment], int]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        header = reader.fieldnames or []
        missing = [column for column in REQUIRED_COLUMNS if column not in header]
        if missing:
            raise ValueError(f"{path}: missing required columns {missing}")

        segments: list[Segment] = []
        current: Segment | None = None
        last_id = -1
        rows = 0

        for line_no, row in enumerate(reader, start=2):
            try:
                sid = int(float(row["segment_id"]))
                phase = int(float(row["phase"]))
                pwm = float(row["pwm"])
                rail = float(row["v_rail"])
                t_us = int(float(row["t_us"]))
                values = {ch: float(row[VALUE_COLUMN[ch]]) for ch in CHANNELS}
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_no}: unparsable row ({exc})") from exc

            for ch in CHANNELS:
                if not math.isfinite(values[ch]):
                    raise ValueError(f"{path}:{line_no}: non-finite {VALUE_COLUMN[ch]}")

            if sid < last_id:
                raise ValueError(f"{path}:{line_no}: segment_id moved backwards")
            if current is None or sid != current.sid:
                current = Segment(sid, phase, pwm, rail)
                segments.append(current)
                last_id = sid
            elif abs(pwm - current.pwm) > 1e-6 or abs(rail - current.rail) > 1e-6:
                raise ValueError(f"{path}:{line_no}: segment {sid} changed command")

            current.t_us.append(t_us)
            for ch in CHANNELS:
                current.values[ch].append(values[ch])
            rows += 1

    if not segments or rows == 0:
        raise ValueError(f"{path}: no data rows")
    return segments, rows


def estimate_dt(segments: list[Segment]) -> float:
    steps: list[int] = []
    for seg in segments:
        for i in range(1, min(len(seg.t_us), 33)):
            steps.append(seg.t_us[i] - seg.t_us[i - 1])
    if not steps:
        raise ValueError("cannot estimate the tick interval")
    dt = statistics.median(steps) * 1e-6
    if dt <= 0.0:
        raise ValueError("tick interval is not positive")
    return dt


def _fit_given_lambda(values: list[float], lam: float) -> tuple[float, float]:
    """Least-squares A, B for y[k] = A + B*lam^k; returns (A, sse)."""
    n = len(values)
    mean_y = sum(values) / n
    w = [lam ** k for k in range(n)]
    mean_w = sum(w) / n
    cov = sum((w[k] - mean_w) * (values[k] - mean_y) for k in range(n))
    var = sum((w[k] - mean_w) ** 2 for k in range(n))
    if var <= 1e-15:
        return mean_y, float("inf")
    slope = cov / var
    asymptote = mean_y - slope * mean_w
    sse = sum((values[k] - mean_y) ** 2 for k in range(n)) - cov * cov / var
    return asymptote, max(0.0, sse)


def _search_lambda(values: list[float], lo: float, hi: float, steps: int,
                   start: tuple[float, float, float] | None = None
                   ) -> tuple[float, float, float]:
    best = start
    for i in range(steps + 1):
        lam = lo + (hi - lo) * i / steps
        asymptote, sse = _fit_given_lambda(values, lam)
        if best is None or sse < best[0]:
            best = (sse, lam, asymptote)
    assert best is not None
    return best


def _ar1_lambda(values: list[float]) -> float | None:
    """Slope of the lag-one regression y[k+1] = c + lambda*y[k].

    Used only as an initializer: it is not dominated by per-sample noise the
    way successive-difference ratios are, and for a first-order plant the
    slope is the discrete pole.
    """
    xs = values[:-1]
    ys = values[1:]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx <= 1e-15:
        return None
    sxy = sum((xs[i] - mean_x) * (ys[i] - mean_y) for i in range(len(xs)))
    return sxy / sxx


def fit_first_order(values: list[float], dt_s: float) -> Fit:
    n = len(values)
    if n < 8:
        return Fit(statistics.fmean(values), None, 1.0, False)

    span = max(values) - min(values)
    if span < 1e-12:
        # Constant segment: no transient, only the settled value is useful.
        return Fit(statistics.fmean(values), None, 1.0, False)

    lam = _ar1_lambda(values)
    if lam is None or not 0.02 < lam < 1.0:
        tail = max(1, n // 10)
        return Fit(statistics.fmean(values[-tail:]), None, 1.0, False)

    lam = min(max(lam, LAMBDA_MIN), LAMBDA_MAX)

    # Refine with the exact maximum-likelihood fit for fixed lambda.  The tau
    # sensitivity d(tau)/tau = -d(lambda)/(lambda*ln lambda) is large when
    # lambda is close to 1, so one grid is far too coarse.
    lo = max(LAMBDA_MIN, lam * 0.97)
    hi = min(LAMBDA_MAX, lam * 1.03)
    sse, lam_best, asymptote = _search_lambda(values, lo, hi, 30)
    for _ in range(2):
        step = (hi - lo) / 30.0
        lo = max(LAMBDA_MIN, lam_best - step)
        hi = min(LAMBDA_MAX, lam_best + step)
        sse, lam_best, asymptote = _search_lambda(
            values, lo, hi, 30, start=(sse, lam_best, asymptote))

    if not 0.0 < lam_best < 1.0:
        return Fit(asymptote, None, 1.0, False)

    # Do not report a time constant when the transient is buried in noise:
    # a flat segment always has a "best" lambda, but it means nothing.
    residual_rms = math.sqrt(max(sse, 0.0) / n)
    if span <= 8.0 * residual_rms:
        return Fit(asymptote, None, lam_best, False)

    tau = -dt_s / math.log(lam_best)
    return Fit(asymptote, tau, lam_best, True)


def _linear_fit(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    n = len(points)
    if n < 3:
        return None
    mean_x = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mean_x) ** 2 for p in points)
    sxy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points)
    if sxx <= 1e-12:
        return None
    slope = sxy / sxx
    return slope, mean_y - slope * mean_x


def _rail_nominal(segments: list[Segment]) -> float:
    rails = [round(seg.rail, 3) for seg in segments]
    return max(set(rails), key=rails.count)


def _branch_regression(segments: list[Segment], channel: str, sign: float,
                       rail_nom: float, v_eps: float) -> dict | None:
    """Fit |v_ss| = a*|pwm| + b on one polarity; db = -b/a, gain = a+b."""
    points: list[tuple[float, float]] = []
    for seg in segments:
        if seg.pwm * sign <= 0.0:
            continue
        if abs(seg.rail - rail_nom) > RAIL_MATCH_V:
            continue
        settled = abs(seg.fit[channel].asymptote)
        if settled <= v_eps:
            continue
        points.append((abs(seg.pwm), settled))

    fit = _linear_fit(points)
    if fit is None:
        return None
    slope, intercept = fit
    if slope <= 0.0:
        return None
    deadband = -intercept / slope
    gain = slope + intercept
    if not 0.0 <= deadband < 0.5 or gain <= 0.0:
        return None
    return {"deadband": deadband, "gain": gain, "samples": len(points)}


def _quantized_deadband(segments: list[Segment], channel: str, sign: float,
                        rail_nom: float, v_eps: float) -> float | None:
    levels = [seg for seg in segments
              if abs(seg.rail - rail_nom) <= RAIL_MATCH_V
              and 0.0 < seg.pwm * sign <= STAIRCASE_MAX_PWM]
    last_still: float | None = None
    for seg in sorted(levels, key=lambda s: abs(s.pwm)):
        level = abs(seg.pwm)
        if abs(seg.fit[channel].asymptote) <= v_eps:
            last_still = level
            continue
        if last_still is None:
            return None
        return 0.5 * (last_still + level)
    return None


def _wheel_parameters(segments: list[Segment], channel: str, rail_nom: float,
                      v_eps: float) -> dict:
    positive = _branch_regression(segments, channel, +1.0, rail_nom, v_eps)
    negative = _branch_regression(segments, channel, -1.0, rail_nom, v_eps)

    deadbands = [b["deadband"] for b in (positive, negative) if b]
    regression_gains = [b["gain"] for b in (positive, negative) if b]
    if not deadbands or not regression_gains:
        raise ValueError(f"{channel}: deadband/gain regression failed")

    deadband = statistics.fmean(deadbands)
    quantized = [q for q in (
        _quantized_deadband(segments, channel, +1.0, rail_nom, v_eps),
        _quantized_deadband(segments, channel, -1.0, rail_nom, v_eps),
    ) if q is not None]

    # Gain and tau from well-excited segments only, using the identified
    # deadband.  Near-deadband points carry stiction, not linear gain.
    gains = []
    taus = []
    for seg in segments:
        if seg.pwm == 0.0:
            continue
        drive = (abs(seg.pwm) - deadband) / (1.0 - deadband)
        if drive <= 0.0:
            continue
        u_eff = drive * seg.rail / rail_nom
        if u_eff < GAIN_MIN_DUTY:
            continue
        gains.append(abs(seg.fit[channel].asymptote) / u_eff)
        fit = seg.fit[channel]
        if fit.tau_ok and fit.tau_s is not None:
            taus.append(fit.tau_s)

    if not gains:
        raise ValueError(f"{channel}: no segment was excited enough for gain")
    if not taus:
        raise ValueError(f"{channel}: no segment produced a usable tau")

    return {
        "static_gain_mps": statistics.median(gains),
        "tau_s": statistics.median(taus),
        "deadband": deadband,
        "deadband_quantized": statistics.fmean(quantized) if quantized else None,
        "deadband_positive": positive["deadband"] if positive else None,
        "deadband_negative": negative["deadband"] if negative else None,
        "gain_samples": len(gains),
        "tau_samples": len(taus),
        "tau_min_s": min(taus),
        "tau_max_s": max(taus),
        "gain_regression_positive": positive["gain"] if positive else None,
        "gain_regression_negative": negative["gain"] if negative else None,
    }


def _coast_tau(segments: list[Segment], channel: str) -> float | None:
    taus = [seg.fit[channel].tau_s for seg in segments
            if seg.pwm == 0.0 and seg.fit[channel].tau_ok
            and seg.fit[channel].tau_s is not None]
    return statistics.median(taus) if taus else None


def _rail_section(segments: list[Segment], rail_nom: float, left: dict,
                  right: dict) -> dict:
    levels: dict[float, dict] = {}

    for seg in segments:
        if abs(seg.rail - rail_nom) <= RAIL_MATCH_V or seg.pwm == 0.0:
            continue
        entry = levels.setdefault(round(seg.rail, 3), {"wheel": {}})
        for channel, params in (("left", left), ("right", right)):
            deadband = params["deadband"]
            drive = max(0.0, (abs(seg.pwm) - deadband) / (1.0 - deadband))
            u_eff = drive * seg.rail / rail_nom
            measured = abs(seg.fit[channel].asymptote)
            predicted = params["static_gain_mps"] * u_eff
            plain = params["static_gain_mps"] * drive
            entry["wheel"][channel] = {
                "measured_mps": round(measured, 6),
                "predicted_mps": round(predicted, 6),
                "error_rel": _rel_error(measured, predicted),
                "error_rel_if_rail_ignored": _rel_error(measured, plain),
            }

    errors = [w["error_rel"] for level in levels.values()
              for w in level["wheel"].values() if w["error_rel"] is not None]
    ignored = [w["error_rel_if_rail_ignored"] for level in levels.values()
               for w in level["wheel"].values()
               if w["error_rel_if_rail_ignored"] is not None]

    return {
        "levels": [{"rail_v": rail, **levels[rail]}
                   for rail in sorted(levels, reverse=True)],
        "max_rel_error": max(errors) if errors else None,
        "max_rel_error_if_rail_ignored": max(ignored) if ignored else None,
    }


def _rel_error(measured: float, predicted: float) -> float | None:
    scale = max(abs(measured), abs(predicted))
    if scale < 1e-9:
        return None
    return round(abs(measured - predicted) / scale, 6)


def identify_segments(segments: list[Segment], dt_s: float, rows: int,
                      label: str = "bench", source: str = "") -> dict:
    rail_nom = _rail_nominal(segments)

    for seg in segments:
        seg.fit = {ch: fit_first_order(seg.values[ch], dt_s) for ch in CHANNELS}

    settled_all = [abs(seg.fit[ch].asymptote)
                   for seg in segments for ch in CHANNELS]
    v_scale = max(settled_all)
    v_eps = max(2e-3, 2e-3 * v_scale)

    left = _wheel_parameters(segments, "left", rail_nom, v_eps)
    right = _wheel_parameters(segments, "right", rail_nom, v_eps)

    warnings: list[str] = []
    for name, params in (("left", left), ("right", right)):
        if (params["deadband_quantized"] is not None
                and abs(params["deadband"] - params["deadband_quantized"]) > 0.02):
            warnings.append(
                f"{name}: deadband regression {params['deadband']:.3f} vs "
                f"staircase {params['deadband_quantized']:.3f}")
        pos = params["deadband_positive"]
        neg = params["deadband_negative"]
        if pos is not None and neg is not None and abs(pos - neg) > 0.02:
            warnings.append(
                f"{name}: asymmetric deadband (+{pos:.3f} / -{neg:.3f})")
        coast = _coast_tau(segments, name)
        if coast is not None and params["tau_min_s"] > 0:
            ratio = abs(coast - params["tau_s"]) / max(params["tau_s"], 1e-9)
            if ratio > 0.25:
                warnings.append(
                    f"{name}: coast-down tau {coast:.4f} vs step tau "
                    f"{params['tau_s']:.4f}")

    rail = _rail_section(segments, rail_nom, left, right)
    if rail["max_rel_error"] is not None and rail["max_rel_error"] > 0.05:
        warnings.append(
            f"rail: v_ss is not proportional to v_rail "
            f"(max relative error {rail['max_rel_error']:.3f})")

    def _clean(params: dict) -> dict:
        return {key: (round(value, 6) if isinstance(value, float) else value)
                for key, value in params.items()}

    return {
        "schema_version": "1.0.0",
        "model": "first_order_dc_motor_with_deadband",
        "label": label,
        "source_csv": source,
        "rows": rows,
        "segments": len(segments),
        "tick_dt_s": round(dt_s, 9),
        "rail_nominal_v": round(rail_nom, 6),
        "velocity_scale_mps": round(v_scale, 6),
        "deadband": round(statistics.fmean([left["deadband"], right["deadband"]]), 6),
        "left": _clean(left),
        "right": _clean(right),
        "rail": rail,
        "checks": {"warnings": warnings},
        "verified": False,
    }


def _relative_error(actual: float, expected: float) -> float:
    scale = max(abs(actual), abs(expected), 1e-9)
    return abs(actual - expected) / scale


def compare_truth(result: dict, truth: dict,
                  tolerance: dict) -> tuple[bool, list[str]]:
    problems: list[str] = []
    lines: list[str] = []

    for channel in CHANNELS:
        identified = result[channel]
        expected = truth[channel]

        gain_err = _relative_error(identified["static_gain_mps"],
                                   expected["static_gain_mps"])
        tau_err = _relative_error(identified["tau_s"], expected["tau_s"])
        deadband_err = abs(identified["deadband"] - truth["deadband"])

        lines.append(
            f"{channel}: gain {identified['static_gain_mps']:.4f} vs "
            f"{expected['static_gain_mps']:.4f} ({gain_err * 100:.2f}%), "
            f"tau {identified['tau_s'] * 1e3:.2f}ms vs "
            f"{expected['tau_s'] * 1e3:.2f}ms ({tau_err * 100:.2f}%), "
            f"deadband {identified['deadband']:.4f} vs "
            f"{truth['deadband']:.4f} ({deadband_err:.4f})")

        if gain_err > tolerance["gain"]:
            problems.append(f"{channel}: gain error {gain_err * 100:.2f}% "
                            f"> {tolerance['gain'] * 100:.0f}%")
        if tau_err > tolerance["tau"]:
            problems.append(f"{channel}: tau error {tau_err * 100:.2f}% "
                            f"> {tolerance['tau'] * 100:.0f}%")
        if deadband_err > tolerance["deadband"]:
            problems.append(f"{channel}: deadband error {deadband_err:.4f} "
                            f"> {tolerance['deadband']:.3f}")

    return not problems, lines + problems


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Identify per-wheel motor parameters from a bench log")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--truth", type=Path, default=None)
    parser.add_argument("--label", default="bench")
    parser.add_argument("--rel-gain", type=float, default=DEFAULT_TOL["gain"])
    parser.add_argument("--rel-tau", type=float, default=DEFAULT_TOL["tau"])
    parser.add_argument("--abs-deadband", type=float, default=DEFAULT_TOL["deadband"])
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv[1:])

    try:
        segments, rows = load_segments(args.csv)
        dt_s = estimate_dt(segments)
        result = identify_segments(segments, dt_s, rows, args.label, args.csv.name)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    status = 0

    if args.truth is not None:
        try:
            truth = json.loads(args.truth.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"error: cannot read truth {args.truth}: {exc}", file=sys.stderr)
            return 1

        tolerance = {"gain": args.rel_gain, "tau": args.rel_tau,
                     "deadband": args.abs_deadband}
        ok, lines = compare_truth(result, truth, tolerance)
        result["truth_check"] = {
            "passed": ok,
            "gain_tolerance_rel": args.rel_gain,
            "tau_tolerance_rel": args.rel_tau,
            "deadband_tolerance_abs": args.abs_deadband,
            "detail": lines,
        }
        if not args.quiet:
            for line in lines:
                print(line)
        if not ok:
            print("truth check FAILED", file=sys.stderr)
            status = 1

    for warning in result["checks"]["warnings"]:
        print(f"warning: {warning}", file=sys.stderr)

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        if not args.quiet:
            print(f"wrote {args.out}")
    elif not args.quiet:
        print(json.dumps(result, indent=2))

    return status


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
