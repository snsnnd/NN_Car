#!/usr/bin/env python3
"""Generate the C instance-profile source from a vehicle JSON profile.

Usage:
    python3 tools/python/gen_profile.py <profile.json> <out.c>

The JSON is the single source of truth for the deployment (instance) layer.
Unknown, missing or mistyped keys are hard errors so the build fails early.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

MAX_SENSORS = 16

# (json path, C field, kind)  kind: f=float, i=int, fa=float array
FIELDS = [
    ("geometry.track_width_m", "track_width_m", "f"),
    ("geometry.wheel_radius_m", "wheel_radius_m", "f"),
    ("geometry.sensor_count", "sensor_count", "i"),
    ("geometry.sensor_position_m", "sensor_position_m", "fa"),
    ("geometry.sensor_to_axle_m", "sensor_to_axle_m", "f"),
    ("geometry.position_ref_m", "position_ref_m", "f"),
    ("geometry.axle_ref_m", "axle_ref_m", "f"),

    ("speed.v_max_nom_mps", "v_max_nom_mps", "f"),
    ("speed.speed_ref_mps", "speed_ref_mps", "f"),
    ("speed.v_min_kappa_mps", "v_min_kappa_mps", "f"),

    ("line.kappa_ref", "kappa_ref", "f"),
    ("line.a_lat_max_mps2", "a_lat_max_mps2", "f"),
    ("line.curve_slow_factor", "curve_slow_factor", "f"),
    ("line.omega_ref_radps", "omega_ref_radps", "f"),
    ("line.kappa_gain", "line_kappa_gain", "f"),

    ("confidence.enter_low", "conf_enter_low", "f"),
    ("confidence.exit_high", "conf_exit_high", "f"),
    ("confidence.nn_min", "conf_nn_min", "f"),
    ("confidence.valid_min", "conf_valid_min", "i"),
    ("confidence.enter_hold_s", "conf_enter_hold_s", "f"),
    ("confidence.exit_hold_s", "conf_exit_hold_s", "f"),

    ("pid.kp", "pid_kp", "f"),
    ("pid.kd", "pid_kd", "f"),
    ("pid.d_filter_hz", "pid_d_filter_hz", "f"),

    ("recovery.search_speed_mps", "recovery_speed_mps", "f"),
    ("recovery.kappa_scale", "recovery_kappa_scale", "f"),
    ("recovery.timeout_s", "recovery_timeout_s", "f"),
    ("recovery.ramp_s", "recovery_ramp_s", "f"),

    ("motor.kff", "motor_kff", "f"),
    ("motor.kp", "motor_kp", "f"),
    ("motor.ki", "motor_ki", "f"),
    ("motor.tt_s", "motor_tt_s", "f"),
    ("motor.i_limit", "motor_i_limit", "f"),
    ("motor.deadband", "motor_deadband", "f"),
    ("motor.alpha", "motor_alpha", "f"),

    ("rail.nominal_v", "rail_nominal_v", "f"),
    ("rail.min_v", "rail_min_v", "f"),
    ("rail.max_v", "rail_max_v", "f"),
    ("rail.derate_v", "rail_derate_v", "f"),
    ("rail.stop_v", "rail_stop_v", "f"),
    ("rail.ovp_v", "rail_ovp_v", "f"),
    ("rail.scale_min", "rail_scale_min", "f"),
    ("rail.scale_max", "rail_scale_max", "f"),

    ("encoder.ppr", "encoder_ppr", "i"),
    ("encoder.gear_ratio", "encoder_gear_ratio", "f"),
    ("encoder.quadrature", "encoder_quadrature", "i"),
    ("encoder.timeout_s", "encoder_timeout_s", "f"),

    ("safety.overspeed_factor", "overspeed_factor", "f"),
    ("safety.hard_overspeed_factor", "hard_overspeed_factor", "f"),
    ("safety.hard_overspeed_hold_s", "hard_overspeed_hold_s", "f"),
    ("safety.ramp_down_s", "ramp_down_s", "f"),
    ("safety.cmd_ttl_s", "cmd_ttl_s", "f"),
    ("safety.twist_ttl_s", "twist_ttl_s", "f"),
    ("safety.rail_ttl_s", "rail_ttl_s", "f"),
    ("safety.temp_warn_c", "temp_warn_c", "f"),
    ("safety.temp_stop_c", "temp_stop_c", "f"),
    ("safety.encoder_min_pwm", "encoder_min_pwm", "f"),
    ("safety.encoder_no_edge_s", "encoder_no_edge_s", "f"),
    ("safety.line_lost_timeout_s", "line_lost_timeout_s", "f"),
    ("safety.curvature_fault_s", "curvature_fault_s", "f"),
    ("safety.derate_rail_low", "derate_rail_low", "f"),
    ("safety.derate_temp_warn", "derate_temp_warn", "f"),

    ("timing.motor_dt_s", "motor_dt_s", "f"),
    ("timing.line_dt_s", "line_dt_s", "f"),
]


def _walk(doc: dict, dotted: str):
    node = doc
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"missing key: {dotted}")
        node = node[part]
    return node


def load_profile(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        doc = json.load(fh)

    for key in ("profile_version", "name", "verified"):
        if key not in doc:
            raise KeyError(f"missing top-level key: {key}")

    def walk_all(node: dict, prefix: str) -> set[str]:
        found: set[str] = set()
        for key, value in node.items():
            path = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                found |= walk_all(value, path)
            else:
                found.add(path)
        return found

    known = {p for p, _, _ in FIELDS} | {"profile_version", "name", "verified"}
    present = walk_all(doc, "")
    unknown = present - known
    if unknown:
        raise KeyError(f"unknown profile keys: {sorted(unknown)}")

    for path, _, _ in FIELDS:
        _walk(doc, path)

    return doc


def _cfloat(value) -> str:
    text = f"{float(value):.6g}"
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return text + "f"


def _fmt(value, kind: str, path: str) -> str:
    if kind == "f":
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"{path} must be a number")
        return _cfloat(value)
    if kind == "i":
        if not isinstance(value, int) or isinstance(value, bool):
            raise TypeError(f"{path} must be an integer")
        return str(int(value))
    raise ValueError(kind)


def render(doc: dict, source: str) -> str:
    lines = [
        "/* Auto-generated by tools/python/gen_profile.py.",
        f" * Source: {source}",
        " * Do not edit by hand. */",
        '#include "nncar/profile.h"',
        "",
        "static const nncar_profile_t g_profile = {",
    ]

    for path, field, kind in FIELDS:
        value = _walk(doc, path)
        if kind == "fa":
            if not isinstance(value, list):
                raise TypeError(f"{path} must be an array")
            if len(value) > MAX_SENSORS:
                raise ValueError(f"{path} has more than {MAX_SENSORS} entries")
            items = [_cfloat(v) for v in value]
            items += ["0.0f"] * (MAX_SENSORS - len(items))
            lines.append(f"    .{field} = {{ {', '.join(items)} }},")
        else:
            lines.append(f"    .{field} = {_fmt(value, kind, path)},")

    count = int(_walk(doc, "geometry.sensor_count"))
    if not 0 < count <= MAX_SENSORS:
        raise ValueError("geometry.sensor_count out of range")

    name = str(doc["name"]).replace('"', "'")
    version = str(doc["profile_version"]).replace('"', "'")
    verified = "true" if doc["verified"] else "false"

    lines += [
        "};",
        "",
        "const nncar_profile_t *nncar_profile_default(void)",
        "{",
        "    return &g_profile;",
        "}",
        "",
        "const char *nncar_profile_name(void)",
        "{",
        f'    return "{name}";',
        "}",
        "",
        "const char *nncar_profile_version(void)",
        "{",
        f'    return "{version}";',
        "}",
        "",
        "bool nncar_profile_is_verified(void)",
        "{",
        f"    return {verified};",
        "}",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    source = Path(argv[1])
    target = Path(argv[2])

    try:
        doc = load_profile(source)
        text = render(doc, str(source))
    except (KeyError, TypeError, ValueError) as exc:
        print(f"gen_profile: {exc}", file=sys.stderr)
        return 1

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
