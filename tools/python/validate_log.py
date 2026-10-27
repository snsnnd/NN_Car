#!/usr/bin/env python3
"""Validate a NN_Car CSV run log (and optional manifest) against the schema.

Usage:
    python3 tools/python/validate_log.py run.csv [--manifest manifest.json]
    python3 tools/python/validate_log.py run.csv --quiet

Exit code 0 = valid, 1 = schema/semantic errors, 2 = usage error.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCHEMA = ROOT / "tools" / "schema" / "run_log.schema.json"


class Report:
    def __init__(self, max_errors: int = 20) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.max_errors = max_errors
        self.rows = 0

    def error(self, message: str) -> None:
        if len(self.errors) < self.max_errors:
            self.errors.append(message)
        elif len(self.errors) == self.max_errors:
            self.errors.append("... too many errors, truncated")

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    @property
    def ok(self) -> bool:
        return not self.errors


def load_schema(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _check_manifest(manifest: dict, schema: dict, report: Report) -> None:
    for key in schema.get("required", []):
        if key not in manifest:
            report.error(f"manifest: missing required key '{key}'")

    props = schema.get("properties", {})
    for key, spec in props.items():
        if key not in manifest:
            continue
        value = manifest[key]
        kind = spec.get("type")
        if kind == "string" and not isinstance(value, str):
            report.error(f"manifest: '{key}' must be a string")
        elif kind == "integer" and (not isinstance(value, int) or isinstance(value, bool)):
            report.error(f"manifest: '{key}' must be an integer")
        elif kind == "number" and (not isinstance(value, (int, float)) or isinstance(value, bool)):
            report.error(f"manifest: '{key}' must be a number")
        elif kind == "boolean" and not isinstance(value, bool):
            report.error(f"manifest: '{key}' must be a boolean")
        elif kind == "object" and not isinstance(value, dict):
            report.error(f"manifest: '{key}' must be an object")

    scenario = manifest.get("scenario")
    scenario_spec = props.get("scenario", {})
    if isinstance(scenario, dict):
        for key in scenario_spec.get("required", []):
            if key not in scenario:
                report.error(f"manifest: scenario missing '{key}'")
        for key, spec in scenario_spec.get("properties", {}).items():
            if key in scenario and not isinstance(scenario[key], (int, float)):
                report.error(f"manifest: scenario.{key} must be numeric")

    if manifest.get("profile_verified") is False:
        report.warn("manifest: profile is marked unverified (placeholder parameters)")

    for section_name in ("vehicle", "simulation"):
        section_spec = props.get(section_name, {})
        section = manifest.get(section_name)
        if not isinstance(section, dict):
            if section_name in manifest:
                report.error(f"manifest: '{section_name}' must be an object")
            continue
        for key in section_spec.get("required", []):
            if key not in section:
                report.error(f"manifest: {section_name} missing '{key}'")
        for key, spec in section_spec.get("properties", {}).items():
            if key not in section:
                continue
            value = section[key]
            kind = spec.get("type")
            if kind == "number" and (not isinstance(value, (int, float)) or isinstance(value, bool)):
                report.error(f"manifest: {section_name}.{key} must be numeric")
                continue
            if kind == "integer" and (not isinstance(value, int) or isinstance(value, bool)):
                report.error(f"manifest: {section_name}.{key} must be an integer")
                continue
            if kind == "string" and not isinstance(value, str):
                report.error(f"manifest: {section_name}.{key} must be a string")
                continue
            if kind == "array":
                if not isinstance(value, list):
                    report.error(f"manifest: {section_name}.{key} must be an array")
                    continue
                if len(value) < spec.get("minItems", 0) or len(value) > spec.get("maxItems", float("inf")):
                    report.error(f"manifest: {section_name}.{key} has invalid length")
                item_kind = spec.get("items", {}).get("type")
                if item_kind == "number" and any(
                    not isinstance(item, (int, float)) or isinstance(item, bool)
                    for item in value
                ):
                    report.error(f"manifest: {section_name}.{key} must contain only numbers")
                continue
            if kind in ("number", "integer"):
                if "minimum" in spec and value < spec["minimum"]:
                    report.error(f"manifest: {section_name}.{key} is below minimum")
                if "maximum" in spec and value > spec["maximum"]:
                    report.error(f"manifest: {section_name}.{key} is above maximum")
                if "exclusiveMinimum" in spec and value <= spec["exclusiveMinimum"]:
                    report.error(f"manifest: {section_name}.{key} must exceed minimum")

    vehicle = manifest.get("vehicle")
    if isinstance(vehicle, dict):
        positions = vehicle.get("sensor_position_m")
        count = vehicle.get("sensor_count")
        if isinstance(positions, list) and isinstance(count, int) and len(positions) != count:
            report.error("manifest: vehicle.sensor_position_m length must equal sensor_count")


def validate(csv_path: Path,
             manifest_path: Path | None = None,
             schema_path: Path = DEFAULT_SCHEMA,
             max_errors: int = 20) -> Report:
    report = Report(max_errors=max_errors)
    schema = load_schema(schema_path)
    csv_spec = schema.get("x-csv", {})

    if manifest_path is not None:
        try:
            with manifest_path.open("r", encoding="utf-8") as fh:
                manifest = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            report.error(f"manifest: cannot read {manifest_path}: {exc}")
            return report
        _check_manifest(manifest, schema, report)

    required = csv_spec.get("required_columns", [])
    integer_cols = set(csv_spec.get("integer_columns", []))
    flags = csv_spec.get("flags", {})
    ranges = csv_spec.get("ranges", {})

    try:
        fh = csv_path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        report.error(f"csv: cannot open {csv_path}: {exc}")
        return report

    with fh:
        reader = csv.DictReader(fh)
        header = reader.fieldnames or []
        missing = [c for c in required if c not in header]
        for column in missing:
            report.error(f"csv: missing required column '{column}'")
        if missing:
            return report

        checked_columns = list(required)
        sensor_spec = csv_spec.get("optional_sensor_channels")
        sensor_columns: list[str] = []
        if isinstance(sensor_spec, dict):
            sensor_count = int(sensor_spec.get("count", 0))
            value_prefix = sensor_spec.get("value_prefix", "")
            exist_prefix = sensor_spec.get("exist_prefix", "")
            valid_prefix = sensor_spec.get("valid_prefix", "")
            for sensor_index in range(sensor_count):
                suffix = f"{sensor_index:02d}"
                channel = [value_prefix + suffix, exist_prefix + suffix, valid_prefix + suffix]
                channel_present = [column in header for column in channel]
                if any(channel_present) and not all(channel_present):
                    report.error(f"csv: sensor channel {sensor_index} is incomplete")
                if all(channel_present):
                    sensor_columns.extend(channel)
                    integer_cols.update(channel[1:])
                    flags[channel[1]] = [0, 1]
                    flags[channel[2]] = [0, 1]
                    ranges[channel[0]] = sensor_spec.get("value_range", [-1.0, 1.0])
            if sensor_columns:
                checked_columns.extend(sensor_columns)

        prev_t = None
        prev_dt = None
        constant_dt = True

        for index, row in enumerate(reader):
            report.rows += 1
            where = f"csv row {index + 2}"

            for column in checked_columns:
                raw = row.get(column, "")
                try:
                    value = float(raw)
                except (TypeError, ValueError):
                    report.error(f"{where}: '{column}' is not numeric ('{raw}')")
                    continue

                if not math.isfinite(value):
                    report.error(f"{where}: '{column}' is not finite ({raw})")
                    continue

                if column in integer_cols and value != int(value):
                    report.error(f"{where}: '{column}' must be an integer ({raw})")

                if column in flags:
                    lo, hi = flags[column]
                    if not (lo <= value <= hi):
                        report.error(f"{where}: '{column}'={value} outside flag set [{lo},{hi}]")

                if column in ranges:
                    lo, hi = ranges[column]
                    if not (lo <= value <= hi):
                        report.error(
                            f"{where}: '{column}'={value} outside [{lo},{hi}]")

                if column == "t_us":
                    t = int(value)
                    if prev_t is not None:
                        dt = t - prev_t
                        if dt <= 0:
                            report.error(f"{where}: t_us is not strictly increasing")
                        elif prev_dt is not None and dt != prev_dt:
                            constant_dt = False
                        prev_dt = dt
                    prev_t = t

            if sensor_columns:
                for sensor_index in range(sensor_count):
                    value = row.get(f"gray_value_{sensor_index:02d}", "")
                    exist = row.get(f"gray_exist_{sensor_index:02d}", "")
                    valid = row.get(f"gray_valid_{sensor_index:02d}", "")
                    if exist == "0" and valid != "0":
                        report.error(f"{where}: absent gray sensor {sensor_index} cannot be valid")

        if report.rows == 0:
            report.error("csv: no data rows")
        elif not constant_dt:
            report.warn("csv: t_us tick interval is not constant")

    return report


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Validate a NN_Car run log")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv[1:])

    report = validate(args.csv, args.manifest, args.schema)

    if not args.quiet or not report.ok:
        for message in report.errors:
            print(f"error: {message}", file=sys.stderr)
        for message in report.warnings:
            print(f"warning: {message}", file=sys.stderr)
        status = "ok" if report.ok else "FAILED"
        print(f"{args.csv}: {report.rows} rows, {status}, "
              f"{len(report.warnings)} warning(s)")

    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
