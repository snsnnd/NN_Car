#!/usr/bin/env python3
"""Train the LineFollower MLP from oracle teacher data (numpy only).

The network shape is frozen by line.h: 68 -> 64 -> 32 -> 16 -> 2 (7,058
parameters, ReLU hidden layers, sigmoid for s_v, tanh for kappa_norm).

Teacher data comes from ``nncar_line_ab --dump-teacher``. Each binary row is
68 float32 inputs plus ``s_v`` and ``kappa_norm``. Rows are produced by two
closed loops per episode: the ground-truth oracle and the PID baseline. The
PID rows are off-policy states relabelled by the oracle, so the student sees
the kinds of states it will actually visit once it is in the loop.

The teacher is a Stanley-style path tracker on the true pose (curvature
feedforward + lateral and heading feedback). It therefore knows the heading,
which the 68-D single-frame observation does NOT contain; the student can only
learn the conditional mean. That gap is the point of the A/B test: the C-side
``nncar_line_ab --check`` measures whether the student still beats the PID.

Training is deterministic: fixed seed, episode-level split, seeded shuffling.

Usage:
    python3 tools/python/train_line_nn.py --data build/line_teacher.bin
    python3 tools/python/train_line_nn.py --data build/line_teacher.bin --epochs 60

Exit code 0 = trained, 2 = usage/data error.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

INPUT_DIM = 68
HIDDEN = (64, 32, 16)
OUTPUTS = 2
LABELS = 3   # s_v, kappa_teacher, kappa_pid (the residual baseline)
DUMP_ROW = INPUT_DIM + LABELS
MAGIC = 0x4C4E4E31


def load_dump(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int, int]:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 4:
        raise ValueError("dump is truncated")
    header = raw[:4].view(np.int32)
    if int(header[0]) != MAGIC:
        raise ValueError(f"bad magic 0x{int(header[0]):08x}")
    episodes = int(header[1])
    rows_per_run = int(header[2])
    runs_per_ep = int(header[3])

    body = raw[4:].reshape(-1, DUMP_ROW)
    expected = episodes * runs_per_ep * rows_per_run
    if body.shape[0] != expected:
        raise ValueError(f"{body.shape[0]} rows, expected {expected}")

    return (body[:, :INPUT_DIM], body[:, INPUT_DIM:INPUT_DIM + OUTPUTS],
            body[:, INPUT_DIM + OUTPUTS], episodes, rows_per_run, runs_per_ep)


def init_net(rng: np.random.Generator) -> dict[str, np.ndarray]:
    dims = (INPUT_DIM,) + HIDDEN + (OUTPUTS,)
    net: dict[str, np.ndarray] = {}
    for i, (fan_in, fan_out) in enumerate(zip(dims[:-1], dims[1:])):
        scale = math.sqrt(2.0 / fan_in)
        net[f"w{i + 1}"] = (rng.standard_normal((fan_in, fan_out)) * scale).astype(np.float32)
        net[f"b{i + 1}"] = np.zeros(fan_out, dtype=np.float32)
    return net


def forward(net: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    h1 = np.maximum(x @ net["w1"] + net["b1"], 0.0)
    h2 = np.maximum(h1 @ net["w2"] + net["b2"], 0.0)
    h3 = np.maximum(h2 @ net["w3"] + net["b3"], 0.0)
    z4 = h3 @ net["w4"] + net["b4"]
    return np.stack((1.0 / (1.0 + np.exp(-z4[:, 0])), np.tanh(z4[:, 1])), axis=1)


def batch_gradients(net: dict[str, np.ndarray], x: np.ndarray,
                    y: np.ndarray) -> tuple[float, dict[str, np.ndarray]]:
    z1 = x @ net["w1"] + net["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ net["w2"] + net["b2"]
    h2 = np.maximum(z2, 0.0)
    z3 = h2 @ net["w3"] + net["b3"]
    h3 = np.maximum(z3, 0.0)
    z4 = h3 @ net["w4"] + net["b4"]

    out = np.stack((1.0 / (1.0 + np.exp(-z4[:, 0])), np.tanh(z4[:, 1])), axis=1)
    diff = out - y
    loss = float(np.mean(diff * diff))

    n = x.shape[0]
    dz4 = np.empty_like(z4)
    # dL/dz through sigmoid (output 0) and tanh (output 1).
    dz4[:, 0] = 2.0 * diff[:, 0] / n * out[:, 0] * (1.0 - out[:, 0])
    dz4[:, 1] = 2.0 * diff[:, 1] / n * (1.0 - out[:, 1] * out[:, 1])

    g = {}
    g["w4"] = h3.T @ dz4
    g["b4"] = dz4.sum(axis=0)
    dh3 = dz4 @ net["w4"].T
    dz3 = dh3 * (z3 > 0.0)
    g["w3"] = h2.T @ dz3
    g["b3"] = dz3.sum(axis=0)
    dh2 = dz3 @ net["w3"].T
    dz2 = dh2 * (z2 > 0.0)
    g["w2"] = h1.T @ dz2
    g["b2"] = dz2.sum(axis=0)
    dh1 = dz2 @ net["w2"].T
    dz1 = dh1 * (z1 > 0.0)
    g["w1"] = x.T @ dz1
    g["b1"] = dz1.sum(axis=0)

    return loss, g


def train(net: dict[str, np.ndarray], x: np.ndarray, y: np.ndarray,
          x_val: np.ndarray, y_val: np.ndarray, epochs: int, seed: int,
          lr: float = 1.0e-3, batch: int = 2048) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed + 1)
    params = list(net)
    m = {k: np.zeros_like(net[k]) for k in params}
    v = {k: np.zeros_like(net[k]) for k in params}
    beta1, beta2, eps = 0.9, 0.999, 1.0e-8
    step = 0
    best = {k: net[k].copy() for k in params}
    best_loss = float("inf")

    n = x.shape[0]
    for _ in range(epochs):
        order = rng.permutation(n)
        for start in range(0, n, batch):
            idx = order[start:start + batch]
            _, g = batch_gradients(net, x[idx], y[idx])

            step += 1
            for k in params:
                m[k] = beta1 * m[k] + (1.0 - beta1) * g[k]
                v[k] = beta2 * v[k] + (1.0 - beta2) * g[k] * g[k]
                m_hat = m[k] / (1.0 - beta1 ** step)
                v_hat = v[k] / (1.0 - beta2 ** step)
                net[k] = (net[k] - lr * m_hat / (np.sqrt(v_hat) + eps)).astype(np.float32)

        val = float(np.mean((forward(net, x_val) - y_val) ** 2))
        if val < best_loss:
            best_loss = val
            best = {k: net[k].copy() for k in params}

    return best


def export_json(path: Path, net: dict[str, np.ndarray], meta: dict) -> None:
    doc = {
        "schema_version": "1.0.0",
        "model": "line_mlp_residual" if meta.get("residual") else "line_mlp",
        "input_dim": INPUT_DIM,
        "hidden": list(HIDDEN),
        "activation": "relu",
        "output": "sigmoid_tanh",
        "trained": True,
        "seed": meta["seed"],
        "epochs": meta["epochs"],
        "samples": meta["samples"],
        "metrics": meta["metrics"],
        "weights": {k: net[k].astype(np.float32).ravel().tolist()
                    for k in ("w1", "b1", "w2", "b2", "w3", "b3", "w4", "b4")},
        "shape": {
            "w1": [INPUT_DIM, HIDDEN[0]], "b1": [HIDDEN[0]],
            "w2": [HIDDEN[0], HIDDEN[1]], "b2": [HIDDEN[1]],
            "w3": [HIDDEN[1], HIDDEN[2]], "b3": [HIDDEN[2]],
            "w4": [HIDDEN[2], OUTPUTS], "b4": [OUTPUTS],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="build/line_teacher.bin")
    parser.add_argument("--out-json", default="configs/line_nn_weights.json")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--residual", action="store_true",
                        help="learn kappa_teacher - kappa_pid instead of kappa")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        (x_all, y_all, kappa_pid_all, episodes, rows_per_run,
         runs_per_ep) = load_dump(Path(args.data))
    except (OSError, ValueError) as exc:
        print(f"train_line_nn: {exc}", file=sys.stderr)
        return 2

    # Episode-level split: never split inside an episode block.
    block = runs_per_ep * rows_per_run
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(episodes)
    split = int(round(episodes * 0.8))
    train_eps = set(int(e) for e in order[:split])

    train_mask = np.zeros(episodes * block, dtype=bool)
    for ep in train_eps:
        train_mask[ep * block:(ep + 1) * block] = True

    if args.residual:
        # The student only sees the state, never the PID command, so the label
        # is the correction the PID is missing at that same state. Clipped away
        # from the tanh tail like the motor teacher.
        y_all = y_all.copy()
        y_all[:, 1] = np.clip(y_all[:, 1] - kappa_pid_all, -0.96, 0.96)

    x = x_all[train_mask]
    y = y_all[train_mask]
    x_val = x_all[~train_mask]
    y_val = y_all[~train_mask]

    net = init_net(np.random.default_rng(args.seed))
    net = train(net, x, y, x_val, y_val, args.epochs, args.seed)

    pred = forward(net, x_val)
    mse = float(np.mean((pred - y_val) ** 2))
    var = np.var(y_val, axis=0)
    r2 = 1.0 - np.mean((pred - y_val) ** 2, axis=0) / np.maximum(var, 1e-9)

    metrics = {
        "val_mse": mse,
        "val_r2_sv": float(r2[0]),
        "val_r2_kappa": float(r2[1]),
        "epochs": args.epochs,
    }

    if not args.quiet:
        print(f"train_line_nn: train={x.shape[0]} val={x_val.shape[0]} samples, "
              f"epochs={args.epochs}")
        print(f"  val MSE={mse:.5f}  R2(s_v)={r2[0]:.4f}  R2(kappa)={r2[1]:.4f}")
        print(f"  label std: s_v={math.sqrt(float(var[0])):.4f} "
              f"kappa={math.sqrt(float(var[1])):.4f}")

    export_json(Path(args.out_json), net,
                {"seed": args.seed, "epochs": args.epochs,
                 "samples": int(x.shape[0]), "residual": args.residual,
                 "metrics": metrics})

    if not args.quiet:
        print(f"  wrote {args.out_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
