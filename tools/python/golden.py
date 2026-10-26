#!/usr/bin/env python3
"""Golden-log comparison for NN_Car CSV runs.

Values are compared cell by cell with a relative/absolute tolerance instead of
byte equality, so recompiles and library differences do not break CI.

Usage:
    python3 tools/python/golden.py --update run.csv golden/run_golden.csv
    python3 tools/python/golden.py --check  run.csv golden/run_golden.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


def _read(path: Path) -> tuple[list[str], list[list[str]]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty file")
        rows = [row for row in reader if row]
    return header, rows


def compare(actual: Path, golden: Path,
            rel_tol: float = 1e-4, abs_tol: float = 1e-6,
            max_reports: int = 15) -> list[str]:
    problems: list[str] = []

    header_a, rows_a = _read(actual)
    header_g, rows_g = _read(golden)

    if header_a != header_g:
        return [f"header mismatch: {actual} vs {golden}"]

    if len(rows_a) != len(rows_g):
        problems.append(f"row count {len(rows_a)} != golden {len(rows_g)}")
        if len(problems) >= max_reports:
            return problems

    numeric = [i for i, name in enumerate(header_g)
               if name not in ("mode", "line_found", "v_rail_valid",
                               "safety_latched", "safety_stop")]

    for index, (row_a, row_g) in enumerate(zip(rows_a, rows_g)):
        for i in numeric:
            try:
                a = float(row_a[i])
                g = float(row_g[i])
            except (ValueError, IndexError):
                problems.append(f"row {index + 2}: non-numeric column '{header_g[i]}'")
                break

            if not math.isfinite(a) or not math.isfinite(g):
                problems.append(f"row {index + 2}: non-finite '{header_g[i]}'")
                break

            diff = abs(a - g)
            if diff > abs_tol and diff > rel_tol * max(abs(a), abs(g)):
                problems.append(
                    f"row {index + 2}: '{header_g[i]}' {a:.9g} != {g:.9g} "
                    f"(diff {diff:.3g})")
                break

            if i == 0 and index == 0 and row_a[i] != row_g[i]:
                problems.append("t_us mismatch")

        if len(problems) >= max_reports:
            problems.append("... more differences truncated")
            break

    return problems


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare a run log to a golden log")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--update", nargs=2, metavar=("ACTUAL", "GOLDEN"))
    group.add_argument("--check", nargs=2, metavar=("ACTUAL", "GOLDEN"))
    parser.add_argument("--rel", type=float, default=1e-4)
    parser.add_argument("--abs", type=float, default=1e-6)
    args = parser.parse_args(argv[1:])

    if args.update:
        actual = Path(args.update[0])
        golden = Path(args.update[1])
        golden.parent.mkdir(parents=True, exist_ok=True)
        golden.write_bytes(actual.read_bytes())
        print(f"golden updated: {golden}")
        return 0

    actual = Path(args.check[0])
    golden = Path(args.check[1])
    if not golden.exists():
        print(f"error: golden file missing: {golden}", file=sys.stderr)
        return 1

    problems = compare(actual, golden, args.rel, args.abs)
    if problems:
        for problem in problems:
            print(f"error: {problem}", file=sys.stderr)
        print(f"golden check FAILED: {actual}", file=sys.stderr)
        return 1

    print(f"golden check ok: {actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
