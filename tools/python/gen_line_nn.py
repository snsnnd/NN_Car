#!/usr/bin/env python3
"""Generate the C source for the trained LineFollower MLP.

Usage:
    python3 tools/python/gen_line_nn.py <line_nn_weights.json> <out.c>

The JSON is written by tools/python/train_line_nn.py. Its matrices are stored
row-major as [fan_in, fan_out] (numpy order), while nncar_line_nn_t stores
[fan_out][fan_in], so every matrix is transposed here.

Shapes and activations are checked against the frozen constants in line.h; a
mismatch is a hard error so the build fails instead of shipping a network that
does not fit the firmware.

Exit code 0 = ok, 1 = invalid weights, 2 = usage error.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

INPUT_DIM = 68
HIDDEN = (64, 32, 16)
OUTPUTS = 2

MATRICES = (
    ("w1", (INPUT_DIM, HIDDEN[0]), HIDDEN[0], INPUT_DIM),
    ("w2", (HIDDEN[0], HIDDEN[1]), HIDDEN[1], HIDDEN[0]),
    ("w3", (HIDDEN[1], HIDDEN[2]), HIDDEN[2], HIDDEN[1]),
    ("w4", (HIDDEN[2], OUTPUTS), OUTPUTS, HIDDEN[2]),
)
VECTORS = (
    ("b1", HIDDEN[0]),
    ("b2", HIDDEN[1]),
    ("b3", HIDDEN[2]),
    ("b4", OUTPUTS),
)


def _cfloat(value) -> str:
    # 9 significant digits round-trip float32 exactly.
    text = f"{float(value):.9g}"
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return text + "f"


def load_weights(path: Path) -> tuple[dict, dict]:
    with path.open("r", encoding="utf-8") as fh:
        doc = json.load(fh)

    if doc.get("model") not in ("line_mlp", "line_mlp_residual"):
        raise ValueError(f"unexpected model: {doc.get('model')!r}")
    if int(doc["input_dim"]) != INPUT_DIM:
        raise ValueError(f"input_dim {doc['input_dim']} != {INPUT_DIM}")
    if tuple(int(h) for h in doc["hidden"]) != HIDDEN:
        raise ValueError(f"hidden {doc['hidden']} != {list(HIDDEN)}")
    if doc.get("activation") != "relu" or doc.get("output") != "sigmoid_tanh":
        raise ValueError("activation/output do not match line.h")
    if not doc.get("trained"):
        raise ValueError("network is not trained")

    shape = doc["shape"]
    weights: dict[str, np.ndarray] = {}
    for key, want, _, _ in MATRICES:
        if tuple(int(s) for s in shape[key]) != want:
            raise ValueError(f"shape[{key}] {shape[key]} != {list(want)}")
        flat = np.asarray(doc["weights"][key], dtype=np.float64)
        if flat.size != want[0] * want[1]:
            raise ValueError(f"weights[{key}] has {flat.size} values, "
                             f"expected {want[0] * want[1]}")
        weights[key] = flat.reshape(want)
    for key, size in VECTORS:
        if tuple(int(s) for s in shape[key]) != (size,):
            raise ValueError(f"shape[{key}] {shape[key]} != [{size}]")
        flat = np.asarray(doc["weights"][key], dtype=np.float64)
        if flat.size != size:
            raise ValueError(f"weights[{key}] has {flat.size} values, expected {size}")
        weights[key] = flat

    return doc, weights


def _cint(value) -> str:
    return str(int(value))


def render_int8(doc: dict, weights: dict, source: str) -> list[str]:
    """Emit the int8 weights, int32 bias and per-layer scales."""
    quant = doc.get("quant")
    if quant is None:
        return []

    acts = quant["acts"]
    w_scales = quant["w"]
    dims = [INPUT_DIM, HIDDEN[0], HIDDEN[1], HIDDEN[2], OUTPUTS]

    lines = ["", "/* int8 deployment of the same network. */"]
    for i in range(1, 5):
        w = weights[f"w{i}"]
        scale = float(w_scales[i - 1])
        q = np.clip(np.round(w / scale), -127, 127).astype(np.int32)
        lines.append(f"static const int8_t g_line_i8_w{i}[{dims[i]}][{dims[i-1]}] = {{")
        for r in range(dims[i]):
            values = ", ".join(_cint(v) for v in q[:, r])
            lines.append(f"    {{ {values} }},")
        lines.append("};")

        in_scale = float(acts[i - 1])
        b = weights[f"b{i}"]
        qb = np.round(b / (in_scale * scale)).astype(np.int64)
        qb = np.clip(qb, -(2**31), 2**31 - 1)
        lines.append(f"static const int32_t g_line_i8_b{i}[{dims[i]}] = {{")
        lines.append("    " + ", ".join(_cint(v) for v in qb))
        lines.append("};")

    lines.append("")
    lines.append("static const nncar_nn_q_layer_t g_line_i8_layers[4] = {")
    acts_desc = ("NNCAR_NN_ACT_RELU", "NNCAR_NN_ACT_RELU", "NNCAR_NN_ACT_RELU",
                 "NNCAR_NN_ACT_NONE")
    for i in range(1, 5):
        mult = float(acts[i - 1]) * float(w_scales[i - 1]) / float(acts[i])
        # The last layer only dequantizes: its "output scale" is the scale of
        # the int32 accumulator, not of an activation.
        out_scale = (float(acts[i - 1]) * float(w_scales[i - 1]) if i == 4
                     else float(acts[i]))
        lines.append(
            f"    {{{dims[i-1]}, {dims[i]}, {acts_desc[i-1]}, &g_line_i8_w{i}[0][0], "
            f"g_line_i8_b{i}, {_cfloat(mult)}, {_cfloat(out_scale)}}},")
    lines.append("};")
    lines.append("")
    lines.append("static const nncar_nn_q_model_t g_line_i8_model = {")
    lines.append("    g_line_i8_layers, 4, " + _cfloat(acts[0]))
    lines.append("};")
    lines.append("")
    lines.append("const nncar_nn_q_model_t *nncar_line_nn_int8_trained(void)")
    lines.append("{")
    lines.append("    return &g_line_i8_model;")
    lines.append("}")
    lines.append("")
    return lines


def render(doc: dict, weights: dict, source: str) -> str:
    residual = doc.get("model") == "line_mlp_residual"
    symbol = ("nncar_line_nn_residual_trained" if residual
              else "nncar_line_nn_trained")
    static_name = "g_line_nn_residual" if residual else "g_line_nn"
    metrics = doc.get("metrics", {})
    lines = [
        "/* Auto-generated by tools/python/gen_line_nn.py.",
        f" * Source: {source}",
        f" * metrics: {json.dumps(metrics, sort_keys=True)}",
        " * Do not edit by hand. */",
        '#include "nncar/line.h"',
        "",
        f"static const nncar_line_nn_t {static_name} = {{",
    ]

    for key, _, rows, cols in MATRICES:
        matrix = weights[key].T  # C stores [fan_out][fan_in]
        if matrix.shape != (rows, cols):
            raise ValueError(f"{key} transposed to {matrix.shape}")
        lines.append(f"    .{key} = {{")
        for r in range(rows):
            values = ", ".join(_cfloat(v) for v in matrix[r])
            lines.append(f"        {{ {values} }},")
        lines.append("    },")

    for key, size in VECTORS:
        values = ", ".join(_cfloat(v) for v in weights[key])
        lines.append(f"    .{key} = {{ {values} }},")

    lines += [
        "};",
        "",
        f"const nncar_line_nn_t *{symbol}(void)",
        "{",
        f"    return &{static_name};",
        "}",
        "",
    ]
    lines += render_int8(doc, weights, source)
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    source = Path(argv[0])
    target = Path(argv[1])

    try:
        doc, weights = load_weights(source)
        text = render(doc, weights, str(source))
    except (OSError, KeyError, TypeError, ValueError) as exc:
        print(f"gen_line_nn: {exc}", file=sys.stderr)
        return 1

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
