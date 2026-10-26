#!/usr/bin/env python3
"""Train the residual motor MLP from teacher data (numpy only).

The network shape is frozen by motor.h: 7 -> 32 -> 16 -> 8 -> 1 (929
parameters, ReLU hidden layers, tanh output).  The student output is the
tanh argument t, and the residual is ``u_nn = alpha * t``.

Teacher labels come from tools/python/motor_teacher.py, which runs a
short-horizon MPC over the bounded residual of the identified plant.  The
teacher is not PI, and a static model inverse was measured to be worse than
PI+FF, so it was rejected.

Optional DAgger rounds re-collect states with the current student in the loop
and relabel them with the teacher, which removes the compounding-error gap
between the PI+FF state distribution and the student's own.  Two rounds is the
accepted recipe: it lifts the nominal improvement from 10.2% to 12.2%.

Training is deterministic: fixed seeds, fixed split, seeded shuffling.

Acceptance gate (declared before looking at the result):

    the residual must cut closed-loop RMS speed error by >= 10% versus PI+FF

on both the held-out randomised episodes and the nominal two-step scenario.

Usage:
    python3 tools/python/train_motor_nn.py --data build/motor_teacher.npz
    python3 tools/python/train_motor_nn.py --data build/motor_teacher.npz --dagger 2

Exit code 0 = trained and gate passed, 1 = gate failed, 2 = usage error.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

import motor_teacher as mt

HIDDEN = (32, 16, 8)
GATE = 0.10


def init_net(rng: np.random.Generator) -> dict[str, np.ndarray]:
    dims = (mt.INPUT_DIM,) + HIDDEN + (1,)
    net: dict[str, np.ndarray] = {}
    for i, (fan_in, fan_out) in enumerate(zip(dims[:-1], dims[1:])):
        scale = math.sqrt(2.0 / fan_in)
        net[f"w{i + 1}"] = (rng.standard_normal((fan_in, fan_out)) * scale).astype(np.float32)
        net[f"b{i + 1}"] = np.zeros(fan_out, dtype=np.float32)
    return net


def batch_gradients(net: dict[str, np.ndarray], x: np.ndarray,
                    t: np.ndarray) -> tuple[float, dict[str, np.ndarray]]:
    z1 = x @ net["w1"] + net["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ net["w2"] + net["b2"]
    h2 = np.maximum(z2, 0.0)
    z3 = h2 @ net["w3"] + net["b3"]
    h3 = np.maximum(z3, 0.0)
    out = np.tanh((h3 @ net["w4"] + net["b4"]).ravel())

    diff = out - t
    loss = float(np.mean(diff * diff))

    n = x.shape[0]
    do = (2.0 * diff / n * (1.0 - out * out))[:, None]

    g = {}
    g["w4"] = h3.T @ do
    g["b4"] = do.sum(axis=0)
    dh3 = do @ net["w4"].T
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


def train(net: dict[str, np.ndarray], x: np.ndarray, t: np.ndarray,
          x_val: np.ndarray, t_val: np.ndarray, epochs: int, seed: int,
          lr: float = 2.0e-3, batch: int = 2048) -> dict[str, np.ndarray]:
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
            _, g = batch_gradients(net, x[idx], t[idx])

            step += 1
            for k in params:
                m[k] = beta1 * m[k] + (1.0 - beta1) * g[k]
                v[k] = beta2 * v[k] + (1.0 - beta2) * g[k] * g[k]
                m_hat = m[k] / (1.0 - beta1 ** step)
                v_hat = v[k] / (1.0 - beta2 ** step)
                net[k] = (net[k] - lr * m_hat / (np.sqrt(v_hat) + eps)).astype(np.float32)

        val = float(np.mean((mt.forward(net, x_val).ravel() - t_val) ** 2))
        if val < best_loss:
            best_loss = val
            best = {k: net[k].copy() for k in params}

    return best


def rollout(profile: dict, plant_params: np.ndarray, targets: np.ndarray,
            rail: np.ndarray, valid: np.ndarray, net: dict | None) -> float:
    ctrl = mt.Controller(profile, "pi_ff")
    plant = mt.Plant(float(plant_params[0]), float(plant_params[1]),
                     float(plant_params[2]), ctrl.v_max_nom, ctrl.rail_nom)
    plant.v = float(plant_params[3])

    residual_fn = None
    if net is not None:
        def residual_fn(x: np.ndarray) -> float:
            return float(mt.forward(net, x)[0])

    total = 0.0
    count = 0
    for i in range(targets.shape[0]):
        v = plant.v
        rail_v = float(rail[i])
        v_max_eff = ctrl.v_max_eff(rail_v)
        rail_ratio = (rail_v / ctrl.rail_nom) if valid[i] else 1.0

        _, _, pwm = ctrl.step(float(targets[i]), v, v_max_eff,
                              residual_fn=residual_fn,
                              rail_ratio=rail_ratio, rail_valid=bool(valid[i]))

        plant.step(pwm, rail_v)

        if i >= 25:
            err = float(targets[i]) - plant.v
            total += err * err
            count += 1

    return math.sqrt(total / count) if count else 0.0


def evaluate(profile: dict, data: dict, held_out: list[int],
             net: dict) -> dict:
    params = data["params"]
    targets = data["targets"]
    rail = data["rail"]
    valid = data["valid"]

    rms_ff = []
    rms_nn = []
    for ep in held_out:
        rms_ff.append(rollout(profile, params[ep], targets[ep], rail[ep], valid[ep], None))
        rms_nn.append(rollout(profile, params[ep], targets[ep], rail[ep], valid[ep], net))

    rms_ff_a = float(np.sqrt(np.mean(np.square(rms_ff))))
    rms_nn_a = float(np.sqrt(np.mean(np.square(rms_nn))))

    return {"rms_ff": rms_ff_a, "rms_nn": rms_nn_a,
            "improvement": 1.0 - rms_nn_a / rms_ff_a}


def nominal(profile: dict, net: dict) -> dict:
    """Mirror the two-step scenario used by nncar_baseline."""
    steps = int(3.0 / mt.DT_S)
    targets = np.zeros(steps, dtype=np.float32)
    targets[int(0.2 / mt.DT_S):int(1.6 / mt.DT_S)] = 0.8
    targets[int(1.6 / mt.DT_S):] = 0.4
    rail = np.full(steps, 12.0, dtype=np.float32)
    valid = np.ones(steps, dtype=np.int8)
    params = np.array([1.5, 0.05, 0.07, 0.0], dtype=np.float32)

    out = {"pi_ff": rollout(profile, params, targets, rail, valid, None),
           "residual": rollout(profile, params, targets, rail, valid, net)}
    out["improvement"] = 1.0 - out["residual"] / out["pi_ff"]
    return out


def rollout_direct(profile: dict, plant_params: np.ndarray, targets: np.ndarray,
                   rail: np.ndarray, valid: np.ndarray, net: dict) -> float:
    """Closed loop where the network *is* the controller: no PI, no integrator.

    The teacher command is ``u_pi + alpha * t`` while the state the network sees
    holds ``u_pi`` at zero, so any steady-state error the PI would have removed
    has to be reproduced from (target, actual, previous pwm, rail) alone.
    """
    ctrl = mt.Controller(profile, "pi_ff")
    plant = mt.Plant(float(plant_params[0]), float(plant_params[1]),
                     float(plant_params[2]), ctrl.v_max_nom, ctrl.rail_nom)
    plant.v = float(plant_params[3])

    total = 0.0
    count = 0
    e_acc = 0.0
    for i in range(targets.shape[0]):
        v = plant.v
        rail_v = float(rail[i])
        rail_ratio = (rail_v / ctrl.rail_nom) if valid[i] else 1.0
        prev_pwm = ctrl.applied_pwm

        err = float(targets[i]) - v
        e_acc = float(np.clip(e_acc + err * mt.DT_S, -ctrl.i_limit, ctrl.i_limit))

        x = mt.features(ctrl.v_max_nom, ctrl.scale_min, ctrl.scale_max,
                        float(targets[i]), v, prev_pwm, 0.0, rail_ratio,
                        bool(valid[i]), float(np.clip(e_acc / ctrl.i_limit, -1.0, 1.0)))
        u = float(mt.forward(net, x)[0])
        pwm = ctrl.compensate(float(np.clip(u, -1.0, 1.0)))
        ctrl.applied_pwm = pwm
        plant.step(pwm, rail_v)

        if i >= 25:
            err = float(targets[i]) - plant.v
            total += err * err
            count += 1

    return math.sqrt(total / count) if count else 0.0


def evaluate_direct(profile: dict, data: dict, held_out: list[int],
                    net: dict) -> dict:
    params = data["params"]
    targets = data["targets"]
    rail = data["rail"]
    valid = data["valid"]

    rms_ff = []
    rms_nn = []
    for ep in held_out:
        rms_ff.append(rollout(profile, params[ep], targets[ep], rail[ep], valid[ep], None))
        rms_nn.append(rollout_direct(profile, params[ep], targets[ep], rail[ep],
                                     valid[ep], net))

    rms_ff_a = float(np.sqrt(np.mean(np.square(rms_ff))))
    rms_nn_a = float(np.sqrt(np.mean(np.square(rms_nn))))

    return {"rms_ff": rms_ff_a, "rms_nn": rms_nn_a,
            "improvement": 1.0 - rms_nn_a / rms_ff_a}


def nominal_direct(profile: dict, net: dict) -> dict:
    steps = int(3.0 / mt.DT_S)
    targets = np.zeros(steps, dtype=np.float32)
    targets[int(0.2 / mt.DT_S):int(1.6 / mt.DT_S)] = 0.8
    targets[int(1.6 / mt.DT_S):] = 0.4
    rail = np.full(steps, 12.0, dtype=np.float32)
    valid = np.ones(steps, dtype=np.int8)
    params = np.array([1.5, 0.05, 0.07, 0.0], dtype=np.float32)

    out = {"pi_ff": rollout(profile, params, targets, rail, valid, None),
           "direct": rollout_direct(profile, params, targets, rail, valid, net)}
    out["improvement"] = 1.0 - out["direct"] / out["pi_ff"]
    return out


def export_json(path: Path, net: dict, profile: dict, meta: dict) -> None:
    direct = bool(meta.get("direct", False))
    doc = {
        "schema_version": "1.0.0",
        "model": "direct_mlp" if direct else "residual_mlp",
        "input_dim": mt.INPUT_DIM,
        "hidden": list(HIDDEN),
        "activation": "relu",
        "output": "tanh",
        "trained": True,
        "seed": meta["seed"],
        "epochs": meta["epochs"],
        "dagger": meta["dagger"],
        "samples": meta["samples"],
        "metrics": meta["metrics"],
        "weights": {k: net[k].astype(np.float32).ravel().tolist()
                    for k in ("w1", "b1", "w2", "b2", "w3", "b3", "w4", "b4")},
        "shape": {
            "w1": [mt.INPUT_DIM, HIDDEN[0]], "b1": [HIDDEN[0]],
            "w2": [HIDDEN[0], HIDDEN[1]], "b2": [HIDDEN[1]],
            "w3": [HIDDEN[1], HIDDEN[2]], "b3": [HIDDEN[2]],
            "w4": [HIDDEN[2], 1], "b4": [1],
        },
    }
    if not direct:
        doc["alpha"] = float(profile["motor"]["alpha"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="build/motor_teacher.npz")
    parser.add_argument("--identified", default="configs/motor_identified.json")
    parser.add_argument("--profile", default="configs/vehicle_default.json")
    parser.add_argument("--out-json", default="configs/motor_nn_weights.json")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--dagger", type=int, default=2)
    parser.add_argument("--dagger-episodes", type=int, default=120)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--direct", action="store_true",
                        help="train a standalone policy (no PI anchor) instead")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        with open(args.profile, "r", encoding="utf-8") as fh:
            profile = json.load(fh)
        with open(args.identified, "r", encoding="utf-8") as fh:
            identified = json.load(fh)
        loaded = np.load(args.data)
        data = {k: loaded[k] for k in loaded.files}
    except (OSError, KeyError, json.JSONDecodeError) as exc:
        print(f"train_motor_nn: {exc}", file=sys.stderr)
        return 2

    episodes = int(data["episode"].max()) + 1
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(episodes)
    split = int(round(episodes * 0.8))
    held_out = [int(e) for e in order[split:]]

    mask = np.isin(data["episode"], [int(e) for e in order[:split]])
    val_mask = np.isin(data["episode"], held_out)

    if args.direct:
        # A standalone policy never sees the PI state, so x[3] is pinned to 0
        # and the label is the teacher's absolute command, not its residual.
        alpha = float(profile["motor"]["alpha"])
        xs = data["x"].astype(np.float32).copy()
        labels = np.clip(xs[:, 3] + alpha * data["t"], -1.0, 1.0).astype(np.float32)
        xs[:, 3] = 0.0
        x = xs[mask]
        t = labels[mask]
        x_val = xs[val_mask]
        t_val = labels[val_mask]
    else:
        x = data["x"][mask].astype(np.float32)
        t = data["t"][mask].astype(np.float32)
        x_val = data["x"][val_mask].astype(np.float32)
        t_val = data["t"][val_mask].astype(np.float32)

    net = init_net(np.random.default_rng(args.seed))
    net = train(net, x, t, x_val, t_val, args.epochs, args.seed)

    dagger_rounds = 0 if args.direct else args.dagger

    for it in range(dagger_rounds):
        def residual_fn(sample: np.ndarray, w=net) -> float:
            return float(mt.forward(w, sample)[0])

        extra = mt.generate(profile, identified, args.dagger_episodes,
                            seed=1000 + it, duration_s=2.0,
                            residual_fn=residual_fn)
        x = np.concatenate([x, extra["x"]])
        t = np.concatenate([t, extra["t"]])
        net = init_net(np.random.default_rng(args.seed + it + 1))
        net = train(net, x, t, x_val, t_val, args.epochs, args.seed + it + 1)

    val_mse = float(np.mean((mt.forward(net, x_val).ravel() - t_val) ** 2))
    zero_mse = float(np.mean(t_val ** 2))
    r2 = 1.0 - val_mse / zero_mse

    if args.direct:
        closed = evaluate_direct(profile, data, held_out, net)
        nom = nominal_direct(profile, net)
    else:
        closed = evaluate(profile, data, held_out, net)
        nom = nominal(profile, net)

    metrics = {
        "val_mse": val_mse,
        "val_r2": r2,
        "heldout_rms_ff": closed["rms_ff"],
        "heldout_rms_nn": closed["rms_nn"],
        "heldout_improvement": closed["improvement"],
        "nominal_rms_ff": nom["pi_ff"],
        "nominal_rms_nn": nom["direct"] if args.direct else nom["residual"],
        "nominal_improvement": nom["improvement"],
    }

    if not args.quiet:
        label = "direct" if args.direct else "residual"
        print(f"train_motor_nn: train={x.shape[0]} val={x_val.shape[0]} "
              f"samples, epochs={args.epochs}, dagger={dagger_rounds}")
        print(f"  teacher fit: val MSE={val_mse:.5f} (zero={zero_mse:.5f}) R2={r2:.4f}")
        print(f"  held-out closed loop: PI+FF={closed['rms_ff']:.5f} "
              f"{label}={closed['rms_nn']:.5f} "
              f"({closed['improvement'] * 100.0:+.1f}%)")
        print(f"  nominal closed loop: PI+FF={nom['pi_ff']:.5f} "
              f"{label}={metrics['nominal_rms_nn']:.5f} "
              f"({nom['improvement'] * 100.0:+.1f}%)")

    export_json(Path(args.out_json), net, profile,
                {"seed": args.seed, "epochs": args.epochs,
                 "dagger": dagger_rounds, "samples": int(x.shape[0]),
                 "direct": args.direct, "metrics": metrics})

    passed = closed["improvement"] >= GATE and nom["improvement"] >= GATE

    if not args.quiet:
        verdict = "PASS" if passed else "FAIL"
        print(f"  gate (>= {GATE * 100.0:.0f}% both): {verdict}")
        print(f"  wrote {args.out_json}")

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
