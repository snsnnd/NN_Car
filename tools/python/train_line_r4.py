#!/usr/bin/env python3
"""Train the experimental SmartLine R4 model from ``nncar_line_ab`` data.

The binary file contains a four-int32 header (magic, episodes, rows/run,
runs/episode), followed by float32 rows:
  x[7], steering_residual, speed_logit, true_lateral_error, true_curvature,
  normalized_PD_baseline.

The privileged head is used only to shape the shared representation and is not
exported. Deployment weights are the 7 -> 16 -> 8 -> 2 control network.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

MAGIC = 0x4C4E5231
INPUT_DIM = 7
HIDDEN = (16, 8)
CONTROL_DIM = 2
PRIVILEGED_DIM = 2
ROW_DIM = INPUT_DIM + CONTROL_DIM + PRIVILEGED_DIM
ROW_DIM += 1  # normalized PD baseline for the steering-smoothness regularizer


def load_dump(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                  np.ndarray, int]:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 4:
        raise ValueError("dump is truncated")
    header = raw[:4].view(np.int32)
    if int(header[0]) != MAGIC:
        raise ValueError(f"bad R4 magic 0x{int(header[0]):08x}")
    episodes, rows_per_run, runs_per_ep = map(int, header[1:])
    if min(episodes, rows_per_run, runs_per_ep) <= 0:
        raise ValueError("dump dimensions must be positive")
    body = raw[4:]
    if body.size % ROW_DIM:
        raise ValueError("dump body is not a whole number of R4 rows")
    body = body.reshape(-1, ROW_DIM)
    expected = episodes * rows_per_run * runs_per_ep
    if body.shape[0] != expected:
        raise ValueError(f"{body.shape[0]} rows, expected {expected}")
    if not np.isfinite(body).all():
        raise ValueError("dump contains non-finite values")
    block = rows_per_run * runs_per_ep
    episode_ids = np.repeat(np.arange(episodes), block)
    return (body[:, :INPUT_DIM], body[:, INPUT_DIM:INPUT_DIM + 2],
            body[:, INPUT_DIM + 2:INPUT_DIM + 4], body[:, INPUT_DIM + 4],
            episode_ids, episodes)


def init_net(rng: np.random.Generator) -> dict[str, np.ndarray]:
    net: dict[str, np.ndarray] = {}
    dims = (INPUT_DIM,) + HIDDEN
    for i, (fan_in, fan_out) in enumerate(zip(dims[:-1], dims[1:]), 1):
        scale = math.sqrt(1.0 / fan_in)
        net[f"w{i}"] = (rng.standard_normal((fan_in, fan_out)) * scale).astype(np.float32)
        net[f"b{i}"] = np.zeros(fan_out, dtype=np.float32)
    for prefix, fan_out in (("c", CONTROL_DIM), ("p", PRIVILEGED_DIM)):
        net[f"w{prefix}"] = (rng.standard_normal((HIDDEN[-1], fan_out)) /
                              math.sqrt(HIDDEN[-1])).astype(np.float32)
        net[f"b{prefix}"] = np.zeros(fan_out, dtype=np.float32)
    return net


def forward(net: dict[str, np.ndarray], x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h1 = np.tanh(x @ net["w1"] + net["b1"])
    h2 = np.tanh(h1 @ net["w2"] + net["b2"])
    control = np.tanh(h2 @ net["wc"] + net["bc"])
    privileged = h2 @ net["wp"] + net["bp"]
    return control, privileged


def gradients(net: dict[str, np.ndarray], x: np.ndarray, y: np.ndarray,
              privileged: np.ndarray, pd_baseline: np.ndarray,
              privileged_weight: float, smooth_weight: float,
              alpha_ratio: float
              ) -> tuple[float, dict[str, np.ndarray]]:
    z1 = x @ net["w1"] + net["b1"]
    h1 = np.tanh(z1)
    z2 = h1 @ net["w2"] + net["b2"]
    h2 = np.tanh(z2)
    zc = h2 @ net["wc"] + net["bc"]
    control = np.tanh(zc)
    pred_priv = h2 @ net["wp"] + net["bp"]

    n = x.shape[0]
    smooth_error = pd_baseline + alpha_ratio * control[:, 0] - x[:, 6]
    dc = 2.0 * (control - y) / (n * CONTROL_DIM)
    dc[:, 0] += (2.0 * smooth_weight * alpha_ratio * smooth_error / n)
    dc *= 1.0 - control * control
    dp = (2.0 * privileged_weight * (pred_priv - privileged) /
          (n * PRIVILEGED_DIM))
    loss = float(np.mean((control - y) ** 2) +
                 privileged_weight * np.mean((pred_priv - privileged) ** 2) +
                 smooth_weight * np.mean(smooth_error ** 2))

    g: dict[str, np.ndarray] = {}
    g["wc"] = h2.T @ dc
    g["bc"] = dc.sum(axis=0)
    g["wp"] = h2.T @ dp
    g["bp"] = dp.sum(axis=0)
    dh2 = dc @ net["wc"].T + dp @ net["wp"].T
    dz2 = dh2 * (1.0 - h2 * h2)
    g["w2"] = h1.T @ dz2
    g["b2"] = dz2.sum(axis=0)
    dh1 = dz2 @ net["w2"].T
    dz1 = dh1 * (1.0 - h1 * h1)
    g["w1"] = x.T @ dz1
    g["b1"] = dz1.sum(axis=0)
    return loss, g


def split_episodes(episode_ids: np.ndarray, seed: int) -> tuple[np.ndarray, np.ndarray]:
    ids = np.unique(episode_ids)
    if ids.size < 2:
        raise ValueError("at least two episodes are required for train/validation split")
    order = np.random.default_rng(seed).permutation(ids)
    n_train = min(max(int(round(ids.size * 0.8)), 1), ids.size - 1)
    return order[:n_train], order[n_train:]


def train(net: dict[str, np.ndarray], x: np.ndarray, y: np.ndarray,
          privileged: np.ndarray, pd_baseline: np.ndarray,
          x_val: np.ndarray, y_val: np.ndarray, priv_val: np.ndarray,
          pd_val: np.ndarray, epochs: int, seed: int,
          privileged_weight: float, smooth_weight: float,
          alpha_ratio: float, lr: float) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed + 1)
    keys = list(net)
    m = {k: np.zeros_like(net[k]) for k in keys}
    v = {k: np.zeros_like(net[k]) for k in keys}
    best = {k: net[k].copy() for k in keys}
    best_loss = float("inf")
    step = 0
    batch = min(1024, x.shape[0])

    for _ in range(epochs):
        order = rng.permutation(x.shape[0])
        for start in range(0, x.shape[0], batch):
            idx = order[start:start + batch]
            _, g = gradients(net, x[idx], y[idx], privileged[idx], pd_baseline[idx],
                             privileged_weight, smooth_weight, alpha_ratio)
            step += 1
            for k in keys:
                m[k] = 0.9 * m[k] + 0.1 * g[k]
                v[k] = 0.999 * v[k] + 0.001 * g[k] * g[k]
                mh = m[k] / (1.0 - 0.9 ** step)
                vh = v[k] / (1.0 - 0.999 ** step)
                net[k] = (net[k] - lr * mh / (np.sqrt(vh) + 1.0e-8)).astype(np.float32)

        pc, pp = forward(net, x_val)
        smooth_error = pd_val + alpha_ratio * pc[:, 0] - x_val[:, 6]
        val_loss = float(np.mean((pc - y_val) ** 2) +
                         privileged_weight * np.mean((pp - priv_val) ** 2) +
                         smooth_weight * np.mean(smooth_error ** 2))
        if val_loss < best_loss:
            best_loss = val_loss
            best = {k: net[k].copy() for k in keys}
    return best


def export_json(path: Path, net: dict[str, np.ndarray], seed: int, epochs: int,
                samples: int, metrics: dict, privileged_weight: float,
                smooth_weight: float, alpha_ratio: float) -> None:
    weights = {key: net[key].astype(np.float32).ravel().tolist()
               for key in ("w1", "b1", "w2", "b2", "wc", "bc")}
    shape = {
        "w1": [INPUT_DIM, HIDDEN[0]], "b1": [HIDDEN[0]],
        "w2": [HIDDEN[0], HIDDEN[1]], "b2": [HIDDEN[1]],
        "wc": [HIDDEN[1], CONTROL_DIM], "bc": [CONTROL_DIM],
    }
    doc = {
        "schema_version": "4.0.0",
        "model": "line_r4_mlp",
        "input_dim": INPUT_DIM,
        "hidden": list(HIDDEN),
        "activation": "tanh",
        "output": "tanh_tanh",
        "trained": True,
        "training_only_head": "privileged_error_curvature",
        "privileged_weight": privileged_weight,
        "smooth_weight": smooth_weight,
        "alpha_ratio": alpha_ratio,
        "seed": seed,
        "epochs": epochs,
        "samples": samples,
        "metrics": metrics,
        "weights": weights,
        "shape": shape,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="build/line_r4_teacher.bin")
    parser.add_argument("--out-json", default="configs/line_r4_weights.json")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--privileged-weight", type=float, default=0.2)
    parser.add_argument("--smooth-weight", type=float, default=0.02)
    parser.add_argument("--profile", default=str(Path(__file__).resolve().parents[2] /
                                                   "configs/vehicle_default.json"))
    parser.add_argument("--alpha-ratio", type=float, default=None,
                        help="override profile line.kappa_alpha (normally leave unset)")
    parser.add_argument("--lr", type=float, default=1.0e-3)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    if (args.epochs <= 0 or args.privileged_weight < 0.0 or
            args.smooth_weight < 0.0 or args.lr <= 0.0):
        parser.error("epochs/lr must be positive; loss weights nonnegative")

    try:
        profile_doc = json.loads(Path(args.profile).read_text(encoding="utf-8"))
        alpha_ratio = (float(args.alpha_ratio) if args.alpha_ratio is not None
                       else float(profile_doc["line"]["kappa_alpha"]))
        if not math.isfinite(alpha_ratio) or alpha_ratio <= 0.0:
            raise ValueError("profile line.kappa_alpha must be finite and positive")
        x, y, priv, pd_baseline, episodes, n_episodes = load_dump(Path(args.data))
        train_eps, val_eps = split_episodes(episodes, args.seed)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"train_line_r4: {exc}", file=sys.stderr)
        return 2

    train_mask = np.isin(episodes, train_eps)
    val_mask = np.isin(episodes, val_eps)
    net = init_net(np.random.default_rng(args.seed))
    net = train(net, x[train_mask], y[train_mask], priv[train_mask], pd_baseline[train_mask],
                x[val_mask], y[val_mask], priv[val_mask], pd_baseline[val_mask],
                args.epochs, args.seed, args.privileged_weight, args.smooth_weight,
                alpha_ratio, args.lr)
    pred, priv_pred = forward(net, x[val_mask])
    metrics = {
        "val_control_mse": float(np.mean((pred - y[val_mask]) ** 2)),
        "val_privileged_mse": float(np.mean((priv_pred - priv[val_mask]) ** 2)),
        "val_steering_mse": float(np.mean((pred[:, 0] - y[val_mask, 0]) ** 2)),
        "val_speed_mse": float(np.mean((pred[:, 1] - y[val_mask, 1]) ** 2)),
        "val_smoothness_mse": float(np.mean((pd_baseline[val_mask] +
                                               alpha_ratio * pred[:, 0] -
                                               x[val_mask, 6]) ** 2)),
        "validation_episodes": int(val_eps.size),
    }
    export_json(Path(args.out_json), net, args.seed, args.epochs,
                int(train_mask.sum()), metrics, args.privileged_weight,
                args.smooth_weight, alpha_ratio)
    if not args.quiet:
        print(f"train_line_r4: train={train_mask.sum()} val={val_mask.sum()} "
              f"episodes={n_episodes}, control MSE={metrics['val_control_mse']:.5f}, "
              f"privileged MSE={metrics['val_privileged_mse']:.5f}")
        print(f"  wrote {args.out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
