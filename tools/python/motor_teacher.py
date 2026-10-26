#!/usr/bin/env python3
"""Generate residual-NN teacher data for the motor loop.

The teacher is NOT the PI controller and NOT a one-step static model inverse.
A static inverse asks the student to reproduce an *absolute* command, so the
label ``u_star - u_pi`` cancels the PI feedback; measured as an oracle with the
true plant it was 12.7% worse than PI+FF on held-out episodes and 37.9% worse
on the nominal two-step (docs/04_设计决策冻结.md 6.3).

Instead the teacher is a short-horizon MPC over the bounded residual:

    at every visited state, pick delta in [-1, 1] minimising
        sum_{j=1..H} (v[k+j] - target)^2
    with u_cmd = clamp(u_pi + alpha * delta) applied once, then the baseline
    PI+FF continues.  The plant is the identified one (teacher is an oracle
    over the identified uncertainty band), target and rail are held over the
    horizon so nothing is clairvoyant.

The label handed to the student is ``delta`` itself; the student output is the
tanh argument t, and the residual is ``u_nn = alpha * t``.  A zero network
still reproduces PI+FF exactly.

States are collected in closed loop (PI+FF baseline, or the current student for
DAgger rounds), while the plant instance is randomised inside the identified
uncertainty band.  A small fraction of episodes hides the rail sensor to teach
the graceful path of the ``rail_exist`` / ``rail_valid`` inputs.

Usage:
    python3 tools/python/motor_teacher.py --out build/motor_teacher.npz
    python3 tools/python/motor_teacher.py --episodes 240 --seed 0

Exit code 0 = ok, 1 = configuration error, 2 = usage error.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

DT_S = 0.002
INPUT_DIM = 8

# MPC teacher: horizon in ticks, candidate residuals, and the label range.
# The range stays below 1 so the tanh output never sits on its flat tail.
MPC_HORIZON = 10
MPC_LEVELS = 17
LABEL_MAX = 0.96


def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def features(v_max_nom: float, scale_min: float, scale_max: float,
             target: float, actual: float, prev_pwm: float, u_pi: float,
             rail_ratio: float, rail_valid: bool,
             e_acc_norm: float = 0.0) -> np.ndarray:
    """Frozen 8-D input contract, identical to nncar_motor_nn_input().

    ``e_acc_norm`` is the clipped error accumulator divided by ``i_limit``; it
    is the only temporal feature and is maintained in every mode.
    """
    x = np.empty(INPUT_DIM, dtype=np.float32)
    x[0] = clampf(target / v_max_nom, -2.0, 2.0)
    x[1] = clampf(actual / v_max_nom, -2.0, 2.0)
    x[2] = clampf(prev_pwm, -1.0, 1.0)
    x[3] = clampf(u_pi, -1.0, 1.0)
    x[4] = clampf(rail_ratio, scale_min, scale_max)
    x[5] = 1.0
    x[6] = 1.0 if rail_valid else 0.0
    x[7] = clampf(e_acc_norm, -1.0, 1.0)
    return x


def load_weights(path: str) -> dict[str, np.ndarray]:
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    shape = doc["shape"]
    return {k: np.asarray(v, dtype=np.float32).reshape(shape[k])
            for k, v in doc["weights"].items()}


def forward(net: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    """Shared MLP evaluation; x is (7,) or (batch, 7)."""
    h1 = np.maximum(x @ net["w1"] + net["b1"], 0.0)
    h2 = np.maximum(h1 @ net["w2"] + net["b2"], 0.0)
    h3 = np.maximum(h2 @ net["w3"] + net["b3"], 0.0)
    return np.tanh(h3 @ net["w4"] + net["b4"])


class Controller:
    """Bit-faithful copy of nncar_motor_step() for the PI and PI+FF modes."""

    def __init__(self, profile: dict, mode: str = "pi_ff") -> None:
        motor = profile["motor"]
        rail = profile["rail"]
        self.mode = mode
        self.kff = float(motor["kff"])
        self.kp = float(motor["kp"])
        self.ki = float(motor["ki"])
        self.tt = max(float(motor["tt_s"]), 1.0e-4)
        self.i_limit = float(motor["i_limit"])
        self.deadband = float(motor["deadband"])
        self.alpha = float(motor["alpha"])
        self.v_max_nom = float(profile["speed"]["v_max_nom_mps"])
        self.rail_nom = float(rail["nominal_v"])
        self.scale_min = float(rail["scale_min"])
        self.scale_max = float(rail["scale_max"])
        self.integral = 0.0
        self.e_acc = 0.0
        self.applied_pwm = 0.0
        self.u = 0.0
        # Exposed after each step so the teacher can rebuild the same state.
        self.integ0 = 0.0
        self.u_unsat = 0.0

    def v_max_eff(self, rail_v: float) -> float:
        scale = rail_v / self.rail_nom
        return self.v_max_nom * clampf(scale, self.scale_min, self.scale_max)

    def compensate(self, u: float) -> float:
        mag = abs(u)
        if mag < 1.0e-6:
            return 0.0
        compensated = self.deadband + (1.0 - self.deadband) * min(mag, 1.0)
        return compensated if u > 0.0 else -compensated

    def reset(self) -> None:
        self.integral = 0.0
        self.e_acc = 0.0
        self.applied_pwm = 0.0
        self.u = 0.0
        self.integ0 = 0.0
        self.u_unsat = 0.0

    def e_acc_norm(self) -> float:
        return clampf(self.e_acc / max(self.i_limit, 1.0e-3), -1.0, 1.0)

    def step(self, target: float, actual: float, v_max_eff: float,
             residual_fn=None, rail_ratio: float = 1.0,
             rail_valid: bool = True) -> tuple[float, float, float]:
        prev_pwm = self.applied_pwm
        feedforward = self.mode != "pi"

        err = target - actual
        self.e_acc = clampf(self.e_acc + err * DT_S, -self.i_limit, self.i_limit)
        u_ff = 0.0
        if feedforward and v_max_eff > 1.0e-3:
            u_ff = self.kff * target / v_max_eff

        integral = self.integral + err * DT_S
        u_unsat = u_ff + self.kp * err + self.ki * integral
        u_pi = clampf(u_unsat, -1.0, 1.0)

        u_nn = 0.0
        if residual_fn is not None:
            x = features(self.v_max_nom, self.scale_min, self.scale_max,
                         target, actual, prev_pwm, u_pi, rail_ratio, rail_valid,
                         self.e_acc_norm())
            u_nn = self.alpha * float(residual_fn(x))

        u_cmd = clampf(u_pi + u_nn, -1.0, 1.0)

        corrected = integral + (u_cmd - u_unsat) * DT_S / self.tt
        self.integral = clampf(corrected, -self.i_limit, self.i_limit)

        pwm = self.compensate(u_cmd) if feedforward else u_cmd
        self.u = u_cmd
        self.applied_pwm = pwm
        self.integ0 = integral
        self.u_unsat = u_unsat
        return u_pi, u_cmd, pwm


class Plant:
    """First-order DC motor with deadband, as used by nncar_sim_step()."""

    def __init__(self, static_gain: float, tau_s: float, deadband: float,
                 v_max_nom: float, rail_nom: float) -> None:
        self.static_gain = static_gain
        self.tau_s = tau_s
        self.deadband = deadband
        self.v_max_nom = v_max_nom
        self.rail_nom = rail_nom
        self.v = 0.0

    def step(self, pwm: float, rail_v: float) -> float:
        mag = abs(pwm)
        drive = 0.0
        if mag > self.deadband:
            drive = (mag - self.deadband) / (1.0 - self.deadband)
            drive = drive if pwm > 0.0 else -drive
        u_eff = drive * rail_v / self.rail_nom
        v_ss = self.static_gain * u_eff
        self.v += (v_ss - self.v) * DT_S / self.tau_s
        return self.v


def _compensate_vec(ctrl: Controller, u: np.ndarray) -> np.ndarray:
    mag = np.abs(u)
    comp = ctrl.deadband + (1.0 - ctrl.deadband) * np.minimum(mag, 1.0)
    return np.where(mag < 1.0e-6, 0.0, np.sign(u) * comp)


def _plant_step_vec(plant: Plant, v: np.ndarray, pwm: np.ndarray,
                    rail: np.ndarray) -> np.ndarray:
    mag = np.abs(pwm)
    drive = np.where(mag > plant.deadband,
                     (mag - plant.deadband) / (1.0 - plant.deadband), 0.0)
    drive = np.sign(pwm) * drive
    v_ss = plant.static_gain * drive * rail / plant.rail_nom
    return v + (v_ss - v) * DT_S / plant.tau_s


def mpc_labels(ctrl: Controller, plant: Plant, rec: dict[str, np.ndarray]) -> np.ndarray:
    """Vectorised H-step MPC over the bounded residual for one episode.

    ``rec`` holds the states visited by the closed loop:
    ``v, integ0, u_pi, u_unsat, target, rail`` (all shape (T,)), where
    ``integ0`` is the provisional integral and ``u_unsat`` the unsaturated
    PI command of that tick.  Returns the label in [-LABEL_MAX, LABEL_MAX].
    """
    v = rec["v"][:, None]
    integ = rec["integ0"][:, None]
    u_pi = rec["u_pi"][:, None]
    u_unsat = rec["u_unsat"][:, None]
    target = rec["target"]
    rail = rec["rail"]

    levels = np.linspace(-LABEL_MAX, LABEL_MAX, MPC_LEVELS, dtype=np.float32)
    u_cmd = np.clip(u_pi + ctrl.alpha * levels[None, :], -1.0, 1.0)
    integ = np.clip(integ + (u_cmd - u_unsat) * DT_S / ctrl.tt,
                    -ctrl.i_limit, ctrl.i_limit)
    v_next = _plant_step_vec(plant, v, _compensate_vec(ctrl, u_cmd), rail[:, None])
    cost = (v_next - target[:, None]) ** 2

    vme = np.maximum(ctrl.v_max_nom * np.clip(rail / ctrl.rail_nom,
                                              ctrl.scale_min, ctrl.scale_max),
                     1.0e-3)
    tgt = target[:, None]
    for _ in range(MPC_HORIZON - 1):
        err = tgt - v_next
        u_ff = ctrl.kff * tgt / vme[:, None]
        integ = integ + err * DT_S
        u_unsat_h = u_ff + ctrl.kp * err + ctrl.ki * integ
        u_pi_h = np.clip(u_unsat_h, -1.0, 1.0)
        integ = np.clip(integ + (u_pi_h - u_unsat_h) * DT_S / ctrl.tt,
                        -ctrl.i_limit, ctrl.i_limit)
        v_next = _plant_step_vec(plant, v_next, _compensate_vec(ctrl, u_pi_h),
                                 rail[:, None])
        cost += (v_next - tgt) ** 2

    # Exact ties happen whenever the command saturates for every candidate.
    # Break them towards the smallest residual so the label stays "do nothing".
    cmin = cost.min(axis=1, keepdims=True)
    tol = 1.0e-12 + 1.0e-9 * np.maximum(1.0, np.abs(cmin))
    tied = cost <= cmin + tol
    tiny = np.where(tied, levels[None, :] ** 2, np.inf)
    return levels[np.argmin(tiny, axis=1)]


def make_targets(rng: np.random.Generator, steps: int, v_limit: float,
                 kind: str) -> np.ndarray:
    t = np.arange(steps) * DT_S
    out = np.zeros(steps)

    if kind == "sine":
        freq = rng.uniform(0.4, 1.8)
        amp = rng.uniform(0.2, 0.9) * v_limit
        return np.clip(amp * np.sin(2.0 * math.pi * freq * t + rng.uniform(0, 6.28)),
                       -v_limit, v_limit)

    knots_t = [0.0]
    knots_v = [rng.uniform(-0.6, 0.6) * v_limit]
    dwell = rng.uniform(0.10, 0.45)
    while knots_t[-1] < t[-1]:
        knots_t.append(knots_t[-1] + dwell)
        knots_v.append(rng.uniform(-1.0, 1.0) * v_limit)
        dwell = rng.uniform(0.10, 0.45)

    knots_t = np.asarray(knots_t)
    knots_v = np.asarray(knots_v)

    for i in range(steps):
        idx = int(np.searchsorted(knots_t, t[i], side="right") - 1)
        idx = max(0, min(idx, len(knots_v) - 1))
        out[i] = knots_v[idx]

    if kind == "ramp":
        slew = rng.uniform(1.5, 6.0)
        for i in range(1, steps):
            delta = out[i] - out[i - 1]
            limit = slew * DT_S
            out[i] = out[i - 1] + clampf(delta, -limit, limit)

    return out


def rail_schedule(rng: np.random.Generator, steps: int) -> tuple[np.ndarray, np.ndarray]:
    rail = np.full(steps, rng.uniform(11.4, 12.3))
    valid = np.ones(steps, dtype=np.int8)

    for _ in range(int(rng.integers(0, 3))):
        start = int(rng.integers(0, steps))
        rail[start:] = rng.uniform(10.5, 12.3)

    if rng.random() < 0.12:
        start = int(rng.integers(steps // 3, steps))
        valid[start:] = 0

    return rail, valid


def generate(profile: dict, identified: dict, episodes: int, seed: int,
             duration_s: float, residual_fn=None) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    steps = int(duration_s / DT_S)

    left = identified["left"]
    base_gain = float(left["static_gain_mps"])
    base_tau = float(left["tau_s"])
    base_db = float(left["deadband"])

    xs: list[np.ndarray] = []
    ts: list[np.ndarray] = []
    eps: list[np.ndarray] = []
    params = np.zeros((episodes, 4), dtype=np.float32)
    all_targets = np.zeros((episodes, steps), dtype=np.float32)
    all_rail = np.zeros((episodes, steps), dtype=np.float32)
    all_valid = np.zeros((episodes, steps), dtype=np.int8)

    for ep in range(episodes):
        controller = Controller(profile, "pi_ff")

        gain = base_gain * rng.uniform(0.80, 1.15)
        tau = base_tau * rng.uniform(0.70, 1.40)
        db = clampf(base_db + rng.uniform(-0.03, 0.03), 0.02, 0.12)
        v0 = float(rng.uniform(-0.4, 0.4))
        plant = Plant(gain, tau, db, controller.v_max_nom, controller.rail_nom)
        plant.v = v0
        params[ep] = (gain, tau, db, v0)

        kind = "sine" if rng.random() < 0.2 else ("ramp" if rng.random() < 0.45 else "steps")
        targets = make_targets(rng, steps, controller.v_max_nom, kind)
        rail, valid = rail_schedule(rng, steps)
        all_targets[ep] = targets.astype(np.float32)
        all_rail[ep] = rail.astype(np.float32)
        all_valid[ep] = valid

        # Pass 1: closed-loop rollout, recording the state the teacher needs.
        rec = {
            "v": np.zeros(steps), "integ0": np.zeros(steps),
            "u_pi": np.zeros(steps), "u_unsat": np.zeros(steps),
            "e_acc": np.zeros(steps),
            "target": targets.astype(np.float64),
            "rail": rail.astype(np.float64),
            "prev_pwm": np.zeros(steps),
        }
        for i in range(steps):
            v = plant.v
            prev_pwm = controller.applied_pwm
            rail_v = float(rail[i])
            v_max_eff = controller.v_max_eff(rail_v)
            rail_ratio = (rail_v / controller.rail_nom) if valid[i] else 1.0

            u_pi, _, pwm = controller.step(
                targets[i], v, v_max_eff, residual_fn=residual_fn,
                rail_ratio=rail_ratio, rail_valid=bool(valid[i]))

            rec["v"][i] = v
            rec["prev_pwm"][i] = prev_pwm
            rec["integ0"][i] = controller.integ0
            rec["u_pi"][i] = u_pi
            rec["u_unsat"][i] = controller.u_unsat
            rec["e_acc"][i] = controller.e_acc_norm()

            plant.step(pwm, rail_v)

        # Pass 2: teacher labels for every visited state (vectorised).
        t_ep = mpc_labels(controller, plant, rec).astype(np.float32)

        x_ep = np.zeros((steps, INPUT_DIM), dtype=np.float32)
        for i in range(steps):
            rail_ratio = (rail[i] / controller.rail_nom) if valid[i] else 1.0
            x_ep[i] = features(controller.v_max_nom, controller.scale_min,
                               controller.scale_max, targets[i], rec["v"][i],
                               rec["prev_pwm"][i], rec["u_pi"][i], rail_ratio,
                               bool(valid[i]), rec["e_acc"][i])

        xs.append(x_ep)
        ts.append(t_ep)
        eps.append(np.full(steps, ep, dtype=np.int32))

    return {
        "x": np.concatenate(xs),
        "t": np.concatenate(ts),
        "episode": np.concatenate(eps),
        "params": params,
        "targets": all_targets,
        "rail": all_rail,
        "valid": all_valid,
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="configs/vehicle_default.json")
    parser.add_argument("--identified", default="configs/motor_identified.json")
    parser.add_argument("--out", default="build/motor_teacher.npz")
    parser.add_argument("--episodes", type=int, default=240)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        with open(args.profile, "r", encoding="utf-8") as fh:
            profile = json.load(fh)
        with open(args.identified, "r", encoding="utf-8") as fh:
            identified = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"motor_teacher: {exc}", file=sys.stderr)
        return 1

    data = generate(profile, identified, args.episodes, args.seed, args.duration)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **data)

    if not args.quiet:
        x = data["x"]
        t = data["t"]
        print(f"motor_teacher: episodes={args.episodes} samples={x.shape[0]} "
              f"input_dim={x.shape[1]}")
        print(f"  label mean={t.mean():.4f} std={t.std():.4f} "
              f"saturated={np.mean(np.abs(t) >= LABEL_MAX - 1e-3) * 100.0:.1f}%")
        print(f"  wrote {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
