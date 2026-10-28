#!/usr/bin/env python3
"""Train a temporal LineFollower ablation with an auxiliary heading target.

Input: the standard 68-D line frame plus the previous ten normalized
centroids. Outputs during training are speed
scale, normalized curvature, and normalized path-relative heading error. The
export keeps only the speed and curvature heads for the existing controller.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

BASE_INPUT_DIM = 68
HIDDEN = (64, 32, 16)
ROW_WIDTH = 72
MAGIC = 0x4C4E4E33


def load_dump(path: Path):
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 4:
        raise ValueError(f"{path}: truncated header")
    header = raw[:4].view(np.int32)
    if int(header[0]) != MAGIC:
        raise ValueError(f"{path}: temporal training requires LNN3 data")
    episodes, rows_per_run, runs_per_episode = map(int, header[1:4])
    metadata_end = 4 + episodes
    expected = episodes * rows_per_run * runs_per_episode
    if raw.size < metadata_end:
        raise ValueError(f"{path}: truncated vehicle table")
    vehicle_ids = raw[4:metadata_end].view(np.int32).copy()
    payload = raw[metadata_end:]
    if payload.size != expected * ROW_WIDTH:
        raise ValueError(f"{path}: expected {expected} rows of {ROW_WIDTH} floats; "
                         "regenerate with the temporal-label build")
    body = payload.reshape(expected, ROW_WIDTH)
    return body, vehicle_ids, episodes, rows_per_run, runs_per_episode


def temporal_features(frame: np.ndarray, rows_per_run: int, runs_per_episode: int,
                      episodes: int, history_frames: int) -> np.ndarray:
    input_dim = BASE_INPUT_DIM + history_frames
    result = np.empty((frame.shape[0], input_dim), dtype=np.float32)
    result[:, :68] = frame
    sequence_count = episodes * runs_per_episode

    for seq in range(sequence_count):
        start = seq * rows_per_run
        stop = start + rows_per_run
        x = frame[start:stop]
        value = x[:, :16]
        usable = (x[:, 16:32] > 0.0) & (x[:, 32:48] > 0.0)
        q = np.where(usable, np.clip((value + 1.0) * 0.5, 0.0, 1.0), 0.0)
        denom = q.sum(axis=1)
        centroid = (q * x[:, 48:64]).sum(axis=1) / np.maximum(denom, 1.0e-6)

        row = np.arange(len(centroid))
        for tap in range(history_frames):
            history_idx = np.maximum(row - tap - 1, 0)
            result[start:stop, 68 + tap] = np.clip(centroid[history_idx], -1.0, 1.0)

    return result


def init_net(rng: np.random.Generator, input_dim: int):
    dims = (input_dim,) + HIDDEN + (3,)
    net = {}
    for i, (fan_in, fan_out) in enumerate(zip(dims[:-1], dims[1:]), start=1):
        net[f"w{i}"] = (rng.standard_normal((fan_in, fan_out)) *
                        math.sqrt(2.0 / fan_in)).astype(np.float32)
        net[f"b{i}"] = np.zeros(fan_out, dtype=np.float32)
    return net


def forward(net, x):
    z1 = x @ net["w1"] + net["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ net["w2"] + net["b2"]
    h2 = np.maximum(z2, 0.0)
    z3 = h2 @ net["w3"] + net["b3"]
    h3 = np.maximum(z3, 0.0)
    z4 = h3 @ net["w4"] + net["b4"]
    out = np.empty_like(z4)
    out[:, 0] = 1.0 / (1.0 + np.exp(-np.clip(z4[:, 0], -40.0, 40.0)))
    out[:, 1:] = np.tanh(z4[:, 1:])
    return out, (x, z1, h1, z2, h2, z3, h3, out)


def gradients(net, x, y, output_weights):
    out, cache = forward(net, x)
    x, z1, h1, z2, h2, z3, h3, out = cache
    diff = out - y
    dz4 = 2.0 * diff * output_weights / x.shape[0]
    dz4[:, 0] *= out[:, 0] * (1.0 - out[:, 0])
    dz4[:, 1:] *= 1.0 - out[:, 1:] ** 2
    grads = {"w4": h3.T @ dz4, "b4": dz4.sum(axis=0)}
    dz3 = (dz4 @ net["w4"].T) * (z3 > 0.0)
    grads["w3"] = h2.T @ dz3
    grads["b3"] = dz3.sum(axis=0)
    dz2 = (dz3 @ net["w3"].T) * (z2 > 0.0)
    grads["w2"] = h1.T @ dz2
    grads["b2"] = dz2.sum(axis=0)
    dz1 = (dz2 @ net["w2"].T) * (z1 > 0.0)
    grads["w1"] = x.T @ dz1
    grads["b1"] = dz1.sum(axis=0)
    return float(np.mean(np.sum(diff * diff * output_weights, axis=1))), grads


def train(x, y, xv, yv, epochs: int, seed: int, heading_weight: float):
    rng = np.random.default_rng(seed)
    net = init_net(rng, x.shape[1])
    first = {k: np.zeros_like(v) for k, v in net.items()}
    second = {k: np.zeros_like(v) for k, v in net.items()}
    best = {k: v.copy() for k, v in net.items()}
    best_loss = float("inf")
    best_epoch = 0
    step = 0
    output_weights = np.asarray([1.0, 1.0, heading_weight], dtype=np.float32)

    for epoch in range(1, epochs + 1):
        order = rng.permutation(len(x))
        for start in range(0, len(order), 2048):
            idx = order[start:start + 2048]
            _, grad = gradients(net, x[idx], y[idx], output_weights)
            step += 1
            for key in net:
                first[key] = 0.9 * first[key] + 0.1 * grad[key]
                second[key] = 0.999 * second[key] + 0.001 * grad[key] ** 2
                m = first[key] / (1.0 - 0.9 ** step)
                v = second[key] / (1.0 - 0.999 ** step)
                net[key] = (net[key] - 1.0e-3 * m / (np.sqrt(v) + 1.0e-8)).astype(
                    np.float32)

        pred, _ = forward(net, xv)
        val = float(np.mean(np.sum((pred - yv) ** 2 * output_weights, axis=1)))
        if val < best_loss:
            best_loss = val
            best_epoch = epoch
            best = {k: v.copy() for k, v in net.items()}

    return best, best_epoch, best_loss


def r2(pred: np.ndarray, truth: np.ndarray) -> np.ndarray:
    residual = np.sum((pred - truth) ** 2, axis=0)
    total = np.sum((truth - truth.mean(axis=0)) ** 2, axis=0)
    return 1.0 - residual / np.maximum(total, 1.0e-12)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", action="append", required=True)
    parser.add_argument("--holdout-vehicle", type=int, default=4)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--heading-scale-rad", type=float, default=0.15,
                        help="radians represented by heading target +/-1")
    parser.add_argument("--heading-weight", type=float, default=1.0,
                        help="auxiliary heading loss weight; 0 disables the head")
    parser.add_argument("--history-frames", type=int, choices=(0, 10), default=10,
                        help="number of previous centroid taps (0 is single-frame)")
    parser.add_argument("--out-json", default="build/line_nn_temporal.json")
    args = parser.parse_args(argv)
    if args.heading_scale_rad <= 0.0 or args.heading_weight < 0.0:
        parser.error("heading scale must be positive and heading weight non-negative")

    try:
        dumps = [load_dump(Path(path)) for path in args.data]
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"train_line_temporal: {exc}", file=sys.stderr)
        return 2

    all_x, all_y, all_vehicle = [], [], []
    for body, vehicle_ids, episodes, rows_per_run, runs_per_episode in dumps:
        frame = body[:, :68]
        x = temporal_features(frame, rows_per_run, runs_per_episode, episodes,
                              args.history_frames)
        heading_norm = np.clip(body[:, 71] * (0.5 / args.heading_scale_rad), -1.0, 1.0)
        y = np.column_stack((body[:, 68], body[:, 69], heading_norm)).astype(np.float32)
        all_x.append(x)
        all_y.append(y)
        all_vehicle.append(np.repeat(vehicle_ids, rows_per_run * runs_per_episode))

    x = np.concatenate(all_x)
    y = np.concatenate(all_y)
    vehicle = np.concatenate(all_vehicle)
    val_mask = vehicle == args.holdout_vehicle
    if not np.any(val_mask) or np.all(val_mask):
        print("train_line_temporal: holdout vehicle must leave train and validation rows",
              file=sys.stderr)
        return 2
    x_train, y_train = x[~val_mask], y[~val_mask]
    x_val, y_val = x[val_mask], y[val_mask]
    print(f"temporal train={len(x_train)} val={len(x_val)} rows; "
          f"held-out vehicle={args.holdout_vehicle}")

    net, best_epoch, best_loss = train(x_train, y_train, x_val, y_val,
                                       args.epochs, args.seed, args.heading_weight)
    pred, _ = forward(net, x_val)
    scores = r2(pred, y_val)
    train_pred, _ = forward(net, x_train)
    train_scores = r2(train_pred, y_train)
    print(f"best epoch={best_epoch}/{args.epochs} weighted_val_loss={best_loss:.5f}; "
          f"R2 speed={scores[0]:.3f} curvature={scores[1]:.3f} "
          f"heading={scores[2]:.3f}; train R2 heading={train_scores[2]:.3f}")

    # Export only the two control heads. The auxiliary heading head shapes the
    # shared representation during training but is not used by the controller.
    policy = {k: value.copy() for k, value in net.items()}
    policy["w4"] = policy["w4"][:, :2]
    policy["b4"] = policy["b4"][:2]
    doc = {
        "schema_version": "1.0.0",
        "model": "line_mlp",
        "input_dim": x.shape[1],
        "hidden": list(HIDDEN),
        "activation": "relu",
        "output": "sigmoid_tanh",
        "trained": True,
        "seed": args.seed,
        "epochs": best_epoch,
        "samples": int(len(x_train)),
        "temporal_features": [f"centroid_t_minus_{i}"
                              for i in range(1, args.history_frames + 1)],
        "auxiliary_target": f"path_relative_heading_error / {args.heading_scale_rad} rad",
        "heading_loss_weight": args.heading_weight,
        "metrics": {
            "val_mse_policy_and_heading": best_loss,
            "val_r2_speed": float(scores[0]),
            "val_r2_curvature": float(scores[1]),
            "val_r2_heading": float(scores[2]),
        },
        "weights": {k: policy[k].astype(np.float32).ravel().tolist()
                    for k in ("w1", "b1", "w2", "b2", "w3", "b3", "w4", "b4")},
        "shape": {
            "w1": [x.shape[1], 64], "b1": [64],
            "w2": [64, 32], "b2": [32],
            "w3": [32, 16], "b3": [16],
            "w4": [16, 2], "b4": [2],
        },
    }
    out = Path(args.out_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
