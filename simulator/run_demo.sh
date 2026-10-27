#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
RUN_DIR="$ROOT_DIR/simulator/runs"

mkdir -p "$RUN_DIR"
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target nncar_sim -j2
"$BUILD_DIR/nncar_sim" \
	--out "$RUN_DIR/straight.csv" \
	--manifest "$RUN_DIR/straight.manifest.json" \
	--duration 8 --v 0.9 --offset 0.02 --heading-deg 3 "$@"

if command -v godot >/dev/null 2>&1; then
	godot --path "$ROOT_DIR/simulator" -- --trace "$RUN_DIR/straight.csv"
else
	printf 'Simulation log created: %s\nOpen simulator/project.godot in Godot 4.7.2 and load that CSV.\n' \
		"$RUN_DIR/straight.csv"
fi
