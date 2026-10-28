#!/usr/bin/env python3
"""Smoke-test the loopback C simulator request/response protocol."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import time


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_live_sim.py NNCAR_LIVE_BINARY", file=sys.stderr)
        return 2

    port = free_port()
    process = subprocess.Popen(
        [sys.argv[1], "--port", str(port), "--duration", "1", "--seed", "23"],
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

        ready = json.loads(stream.readline())
        assert ready["event"] == "ready" and ready["protocol"] == 1
        assert ready["scenario"]["amp_m"] == 0.08
        reset = exchange("RESET 23 0.8 0 0 0.08 3.0 1.0")
        assert reset["event"] == "reset"
        first = exchange("STEP")
        assert first["event"] == "state"
        assert first["t_us"] == 8000
        assert len(first["gray_value"]) == 16
        assert len(first["gray_exist"]) == 16
        assert len(first["gray_valid"]) == 16
        assert first["done"] is False

        exchange("RESET")
        repeated = exchange("STEP")
        assert first == repeated, "same seed/reset must reproduce the first state"

        last = repeated
        while not last["done"]:
            last = exchange("STEP")
        assert last["t_us"] > first["t_us"]
        assert exchange("CLOSE")["event"] == "closed"
        connection.close()
        connection = None
        connection = socket.create_connection(("127.0.0.1", port), timeout=1.0)
        stream = connection.makefile("rwb", buffering=0)
        assert json.loads(stream.readline())["event"] == "ready"
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
