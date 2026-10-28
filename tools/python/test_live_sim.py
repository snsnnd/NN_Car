#!/usr/bin/env python3
"""Smoke-test the loopback C simulator request/response protocol."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
from pathlib import Path


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_live_sim.py NNCAR_LIVE_BINARY", file=sys.stderr)
        return 2

    port = free_port()
    map_file = Path(__file__).resolve().parents[2] / "simulator" / "maps" / "oval.json"
    process = subprocess.Popen(
        [sys.argv[1], "--port", str(port), "--duration", "1", "--seed", "23",
         "--map-file", str(map_file)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    connection: socket.socket | None = None
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                error = process.stderr.read() if process.stderr else ""
                raise RuntimeError(f"live simulator exited early: {error}")
            try:
                connection = socket.create_connection(("127.0.0.1", port), timeout=0.2)
                break
            except OSError:
                time.sleep(0.01)
        if connection is None:
            raise RuntimeError("live simulator did not accept a loopback connection")

        stream = connection.makefile("rwb", buffering=0)

        def exchange(command: str) -> dict:
            stream.write((command + "\n").encode("ascii"))
            response = stream.readline()
            if not response:
                raise RuntimeError("live simulator closed without a response")
            result = json.loads(response)
            if result.get("event") == "error":
                raise RuntimeError(result.get("message", "live simulator error"))
            return result

        def exchange_map(document: dict) -> dict:
            payload = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            stream.write(f"MAPDATA {len(payload)}\n".encode("ascii") + payload + b"\n")
            response = stream.readline()
            if not response:
                raise RuntimeError("live simulator closed without a MAPDATA response")
            return json.loads(response)

        ready = json.loads(stream.readline())
        assert ready["event"] == "ready" and ready["protocol"] == 1
        assert ready["scenario"]["amp_m"] == 0.08
        assert ready["map"]["name"] == "oval" and len(ready["map"]["segments"]) == 64
        reset = exchange("RESET 23 0.8 0 0 0.08 3.0 1.0")
        assert reset["event"] == "reset"
        first = exchange("STEP")
        assert first["event"] == "state"
        assert first["t_us"] == 8000
        assert len(first["gray_value"]) == 16
        assert len(first["gray_exist"]) == 16
        assert len(first["gray_valid"]) == 16
        assert abs(first["lateral_error_m"]) < 0.01
        assert first["done"] is False

        exchange("RESET")
        repeated = exchange("STEP")
        assert first == repeated, "same seed/reset must reproduce the first state"

        last = repeated
        while not last["done"]:
            last = exchange("STEP")
        assert last["t_us"] > first["t_us"]

        parameter_reset = exchange("RESET 23 0.5 0.01 2 0.04 2.0 1.0 10.5")
        assert parameter_reset["event"] == "reset"
        adjusted = exchange("STEP")
        assert adjusted["event"] == "state" and adjusted["t_us"] == 8000

        replacement_map = {
            "map_version": "1.0.0",
            "name": "protocol_test",
            "description": "测试地图",
            "units_per_meter": 1,
            "start": {"node": 1, "toward": 2},
            "nodes": [[1, 0, 0], [2, 1, 0]],
            "edges": [[1, 2]],
        }
        changed = exchange_map(replacement_map)
        assert changed["event"] == "ready"
        assert changed["map"]["name"] == "protocol_test"
        assert len(changed["map"]["segments"]) == 1
        assert changed["scenario"]["rail_v"] == 10.5
        assert changed["scenario"]["v_desired_mps"] == 0.5
        restarted = exchange("RESET")
        assert restarted["event"] == "reset"
        assert exchange("STEP")["t_us"] == 8000

        map_error = exchange_map({})
        assert map_error["event"] == "error"
        assert exchange("HELLO")["map"]["name"] == "protocol_test"

        assert exchange("CLOSE")["event"] == "closed"
        connection.close()
        connection = None
        connection = socket.create_connection(("127.0.0.1", port), timeout=1.0)
        stream = connection.makefile("rwb", buffering=0)
        reconnected = json.loads(stream.readline())
        assert reconnected["event"] == "ready"
        assert reconnected["map"]["name"] == "protocol_test"
        stream.write(b"CLOSE\n")
        assert json.loads(stream.readline())["event"] == "closed"
        print("live simulator protocol: ok")
        return 0
    except Exception as exc:  # report a concise CTest failure
        print(f"live simulator protocol: {exc}", file=sys.stderr)
        return 1
    finally:
        if connection is not None:
            connection.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
