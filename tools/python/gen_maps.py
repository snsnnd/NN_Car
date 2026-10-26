#!/usr/bin/env python3
"""Generate the C track-map table from configs/maps/*.json.

Usage:
    python3 tools/python/gen_maps.py <out.c> <map.json> [<map.json> ...]

A map JSON stores nodes in native map units (the ERTH field uses centimetres,
so units_per_meter = 100) and undirected edges between node ids. The generator
turns every edge into one line segment in metres; the host plant treats the
union of those segments as the grey line. Unknown, missing or mistyped keys are
hard errors so the build fails early.
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

TOP_KEYS = {
    "map_version", "name", "description", "source",
    "units_per_meter", "line_width_m", "start", "nodes", "edges",
}
START_KEYS = {"node", "toward", "heading_deg"}
IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _reject_unknown(doc: dict, allowed: set[str], where: str) -> None:
    extra = sorted(set(doc) - allowed)
    if extra:
        raise KeyError(f"{where}: unknown key(s): {', '.join(extra)}")


def _as_float(value, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where}: expected a number")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{where}: expected a finite number")
    return out


def _as_int(value, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{where}: expected an integer")
    return value


def load_map(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        doc = json.load(fh)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: top level must be an object")
    _reject_unknown(doc, TOP_KEYS, str(path))

    for key in ("map_version", "name", "source", "units_per_meter", "start",
                "nodes", "edges"):
        if key not in doc:
            raise KeyError(f"{path}: missing key: {key}")

    version = doc["map_version"]
    if not isinstance(version, str) or not re.fullmatch(r"\d+\.\d+\.\d+", version):
        raise ValueError(f"{path}: map_version must look like 1.0.0")

    name = doc["name"]
    if not isinstance(name, str) or not IDENT_RE.match(name):
        raise ValueError(f"{path}: name must be a C identifier")

    units = _as_float(doc["units_per_meter"], f"{path}: units_per_meter")
    if units <= 0.0:
        raise ValueError(f"{path}: units_per_meter must be positive")
    scale = 1.0 / units

    line_width = 0.02
    if "line_width_m" in doc:
        line_width = _as_float(doc["line_width_m"], f"{path}: line_width_m")
        if line_width <= 0.0:
            raise ValueError(f"{path}: line_width_m must be positive")

    nodes: dict[int, tuple[float, float]] = {}
    raw_nodes = doc["nodes"]
    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise ValueError(f"{path}: nodes must be a non-empty list")
    for i, raw in enumerate(raw_nodes):
        if not isinstance(raw, list) or len(raw) != 3:
            raise ValueError(f"{path}: nodes[{i}] must be [id, x, y]")
        node_id = _as_int(raw[0], f"{path}: nodes[{i}].id")
        x = _as_float(raw[1], f"{path}: nodes[{i}].x") * scale
        y = _as_float(raw[2], f"{path}: nodes[{i}].y") * scale
        if node_id in nodes:
            raise ValueError(f"{path}: duplicate node id {node_id}")
        nodes[node_id] = (x, y)

    edges: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    raw_edges = doc["edges"]
    if not isinstance(raw_edges, list) or not raw_edges:
        raise ValueError(f"{path}: edges must be a non-empty list")
    for i, raw in enumerate(raw_edges):
        if not isinstance(raw, list) or len(raw) != 2:
            raise ValueError(f"{path}: edges[{i}] must be [node_a, node_b]")
        a = _as_int(raw[0], f"{path}: edges[{i}].a")
        b = _as_int(raw[1], f"{path}: edges[{i}].b")
        if a == b:
            raise ValueError(f"{path}: edges[{i}] is a self loop on {a}")
        if a not in nodes or b not in nodes:
            raise ValueError(f"{path}: edges[{i}] references a missing node")
        pair = (a, b) if a < b else (b, a)
        if pair in seen:
            raise ValueError(f"{path}: duplicate edge {pair}")
        seen.add(pair)
        edges.append((a, b))

    start = doc["start"]
    if not isinstance(start, dict):
        raise ValueError(f"{path}: start must be an object")
    _reject_unknown(start, START_KEYS, f"{path}: start")
    if "node" not in start:
        raise KeyError(f"{path}: start missing node")
    start_id = _as_int(start["node"], f"{path}: start.node")
    if start_id not in nodes:
        raise ValueError(f"{path}: start.node {start_id} not in nodes")

    sx, sy = nodes[start_id]
    if "toward" in start:
        toward = _as_int(start["toward"], f"{path}: start.toward")
        if (start_id, toward) not in seen and (toward, start_id) not in seen:
            raise ValueError(f"{path}: start.toward {toward} is not adjacent")
        tx, ty = nodes[toward]
        theta = math.atan2(ty - sy, tx - sx)
    elif "heading_deg" in start:
        theta = math.radians(_as_float(start["heading_deg"], f"{path}: start.heading_deg"))
    else:
        theta = 0.0

    xs = [p[0] for p in nodes.values()]
    ys = [p[1] for p in nodes.values()]

    return {
        "path": str(path),
        "name": name,
        "version": version,
        "source": str(doc["source"]),
        "line_width_m": line_width,
        "nodes": nodes,
        "edges": edges,
        "bounds": (min(xs), max(xs), min(ys), max(ys)),
        "start": (sx, sy, theta),
    }


def _cfloat(value: float) -> str:
    return f"{value:.5f}f"


def _cstr(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def render(maps: list[dict], source_paths: list[str]) -> str:
    lines = [
        "/* Generated by tools/python/gen_maps.py; do not edit.",
        " * Sources: " + ", ".join(source_paths),
        " */",
        '#include "nncar/map.h"',
        "",
    ]

    for map_doc in maps:
        ident = map_doc["name"]
        lines.append(f"static const nncar_map_seg_t {ident}_segments[] = {{")
        for a, b in map_doc["edges"]:
            ax, ay = map_doc["nodes"][a]
            bx, by = map_doc["nodes"][b]
            lines.append(
                f"    {{{_cfloat(ax)}, {_cfloat(ay)}, {_cfloat(bx)}, {_cfloat(by)}}},"
                f" /* {a}-{b} */"
            )
        lines.append("};")
        lines.append("")

    for map_doc in maps:
        ident = map_doc["name"]
        x_min, x_max, y_min, y_max = map_doc["bounds"]
        sx, sy, theta = map_doc["start"]
        lines.extend([
            f"static const nncar_map_t nncar_map_{ident} = {{",
            f'    .name = "{_cstr(map_doc["name"])}",',
            f'    .version = "{_cstr(map_doc["version"])}",',
            f'    .source = "{_cstr(map_doc["source"])}",',
            f"    .segments = {ident}_segments,",
            f"    .segment_count = {len(map_doc['edges'])},",
            f"    .x_min = {_cfloat(x_min)},",
            f"    .x_max = {_cfloat(x_max)},",
            f"    .y_min = {_cfloat(y_min)},",
            f"    .y_max = {_cfloat(y_max)},",
            f"    .line_width_m = {_cfloat(map_doc['line_width_m'])},",
            f"    .start_x = {_cfloat(sx)},",
            f"    .start_y = {_cfloat(sy)},",
            f"    .start_theta = {_cfloat(theta)},",
            "};",
            "",
        ])

    lines.append("const nncar_map_t *const nncar_map_table[] = {")
    for map_doc in maps:
        lines.append(f"    &nncar_map_{map_doc['name']},")
    lines.append("};")
    lines.append("const int nncar_map_table_count = "
                 "(int)(sizeof(nncar_map_table) / sizeof(nncar_map_table[0]));")
    lines.append("")
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2

    out_path = Path(argv[1])
    map_paths = [Path(p) for p in argv[2:]]
    maps = [load_map(path) for path in map_paths]
    maps.sort(key=lambda item: item["name"])

    names = [item["name"] for item in maps]
    if len(names) != len(set(names)):
        print("duplicate map names", file=sys.stderr)
        return 1

    out_path.write_text(render(maps, [str(p) for p in map_paths]), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
