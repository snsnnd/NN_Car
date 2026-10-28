#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
RUN_DIR="$ROOT_DIR/simulator/runs"
PORT="${NNCAR_SIM_PORT:-37621}"

mkdir -p "$RUN_DIR"
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target nncar_live nncar_sim -j2

"$BUILD_DIR/nncar_live" --port "$PORT" --duration 60 "$@" \
	>"$RUN_DIR/live-server.log" 2>&1 &
LIVE_PID=$!
trap 'kill "$LIVE_PID" 2>/dev/null || true; wait "$LIVE_PID" 2>/dev/null || true' EXIT

for _attempt in $(seq 1 100); do
	if grep -q 'listening on 127.0.0.1' "$RUN_DIR/live-server.log"; then
		break
	fi
	if ! kill -0 "$LIVE_PID" 2>/dev/null; then
		cat "$RUN_DIR/live-server.log" >&2
		exit 1
	fi
	sleep 0.05
done

if command -v godot >/dev/null 2>&1; then
	godot --path "$ROOT_DIR/simulator" -- --live-port "$PORT"
else
	printf 'C live simulator is running on 127.0.0.1:%s. Open simulator/project.godot and press Live.\n' \
		"$PORT"
	wait "$LIVE_PID"
fi
