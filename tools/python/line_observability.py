#!/usr/bin/env python3
"""How much of the LineFollower teacher label is observable?

Fits ridge regressions on the dumped teacher rows to answer "would more history
help?" before building a recurrent model:

  * current centroid                       (what a single frame gives you)
  * current centroid + previous + d/dt     (the 2-feature ablation)
  * last N centroids                       (what a recurrent state could use)

R2 is measured on held-out episodes, so this isolates *information* from
training capacity. A linear fit is an upper bound on what a linear temporal
filter can decode, and a floor for what a nonlinear recurrent model could.

Usage:
    python3 tools/python/line_observability.py --data build/line_teacher.bin

Exit code 0 = ok, 2 = usage/data error.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

INPUT_DIM = 68
OUTPUTS = 2
ROW = INPUT_DIM + OUTPUTS
MAGIC = 0x4C4E4E31
ROOT = Path(__file__).resolve().parents[2]


def load_dump(path: Path) -> tuple[np.ndarray, int, int, int]:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 4:
        raise ValueError("dump is truncated")
    header = raw[:4].view(np.int32)
    if int(header[0]) != MAGIC:
        raise ValueError(f"bad magic 0x{int(header[0]):08x} (expected 0x{MAGIC:08x})")
    episodes = int(header[1])
    rows_per_run = int(header[2])
    runs = int(header[3])

    body = raw[4:].reshape(-1, ROW)
    expected = episodes * runs * rows_per_run
    if body.shape[0] != expected:
        raise ValueError(f"{body.shape[0]} rows, expected {expected}")
    return body, episodes, rows_per_run, runs


def centroids(body: np.ndarray, profile: dict) -> np.ndarray:
    """Recover the centroid that produced each row from the gray pattern."""
    geom = profile["geometry"]
    pos = np.zeros(16)
    pos[:geom["sensor_count"]] = np.asarray(geom["sensor_position_m"])

    value = body[:, 0:16].astype(np.float64)
    exist = body[:, 16:32].astype(np.float64)
    valid = body[:, 32:48].astype(np.float64)
    q = np.where((exist > 0) & (valid > 0), (value + 1.0) * 0.5, 0.0)

    denom = q.sum(axis=1)
    num = (q * pos).sum(axis=1)
    return np.where(denom > 1e-6, num / np.maximum(denom, 1e-6), 0.0)


def ridge_fit(x: np.ndarray, y: np.ndarray, lam: float):
    mu = x.mean(axis=0)
    sd = x.std(axis=0)
    sd[sd < 1e-9] = 1.0
    z = np.hstack([(x - mu) / sd, np.ones((x.shape[0], 1))])
    w = np.linalg.solve(z.T @ z + lam * np.eye(z.shape[1]), z.T @ y)
    return lambda q: np.hstack([(q - mu) / sd, np.ones((q.shape[0], 1))]) @ w


def taps(c: np.ndarray, n: int) -> np.ndarray:
    out = []
    for k in range(n):
        out.append(np.concatenate([np.full(k, c[0]), c[:len(c) - k]]))
    return np.stack(out, axis=1)


def features(frame: np.ndarray, c: np.ndarray, kind: str, dt: float,
             alpha: float) -> np.ndarray:
    if kind == "frame":
        return frame
    if kind == "c":
        return c[:, None]
    if kind == "c2":
        d = np.zeros_like(c)
        d[1:] = (c[1:] - c[:-1]) / dt
        filt = np.zeros_like(c)
        for i in range(len(c)):
            filt[i] = alpha * d[i] + (1.0 - alpha) * (filt[i - 1] if i else 0.0)
        return np.stack([c, np.concatenate([[0.0], c[:-1]]), filt], axis=1)
    if kind.startswith("t"):
        return taps(c, int(kind[1:]))
    if kind.startswith("frame+"):
        return np.hstack([frame, taps(c, int(kind[6:]))])
    raise ValueError(kind)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="build/line_teacher.bin")
    parser.add_argument("--profile", default="configs/vehicle_default.json")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        body, episodes, rows, runs = load_dump(Path(args.data))
        with open(args.profile, "r", encoding="utf-8") as fh:
            profile = json.load(fh)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"line_observability: {exc}", file=sys.stderr)
        return 2

    frame = body[:, :INPUT_DIM].astype(np.float64)
    label = body[:, INPUT_DIM + 1].astype(np.float64)
    cent = centroids(body, profile)
    dt = float(profile["timing"]["line_dt_s"])
    alpha = 1.0 - np.exp(-2.0 * np.pi * float(profile["pid"]["d_filter_hz"]) * dt)

    rng = np.random.default_rng(args.seed)
    order = rng.permutation(episodes)
    held = set(int(e) for e in order[int(round(episodes * 0.8)):])

    def split(kind: str):
        xs, ys, xv, yv = [], [], [], []
        for r in range(episodes * runs):
            c = cent[r * rows:(r + 1) * rows]
            y = label[r * rows:(r + 1) * rows]
            f = frame[r * rows:(r + 1) * rows]
            feat = features(f, c, kind, dt, alpha)
            if (r // runs) in held:
                xv.append(feat)
                yv.append(y)
            else:
                xs.append(feat)
                ys.append(y)
        return (np.vstack(xs), np.concatenate(ys),
                np.vstack(xv), np.concatenate(yv))

    def r2(kind: str, lam: float) -> float:
        x, y, xv, yv = split(kind)
        model = ridge_fit(x, y, lam)
        pred = model(xv)
        ss = float(np.sum((pred - yv) ** 2))
        tot = float(np.sum((yv - yv.mean()) ** 2))
        return 1.0 - ss / tot

    if not args.quiet:
        print(f"rows={body.shape[0]} episodes={episodes} runs/ep={runs} "
              f"rows/run={rows} held-out={len(held)}")
        print(f"{'feature set':<38}{'R2(kappa_norm)':>15}")

    rows_out = [
        ("68-D frame (what the net sees)", "frame", 1e-2),
        ("current centroid", "c", 1e-3),
        ("current + previous + filtered d/dt", "c2", 1e-3),
    ] + [(f"last {n} centroids ({n * dt:.2f} s)", f"t{n}", 1e-2)
         for n in (5, 10, 20, 40, 60)] + [
        ("68-D frame + last 10 centroids", "frame+10", 1e-2),
        ("68-D frame + last 60 centroids", "frame+60", 1e-2),
    ]

    for name, kind, lam in rows_out:
        value = r2(kind, lam)
        if not args.quiet:
            print(f"{name:<38}{value:>15.4f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
