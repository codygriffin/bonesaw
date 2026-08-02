#!/usr/bin/env python3
"""Dependency-free HTTP/WebSocket smoke test for the hosted Upkie editor."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import ssl
import statistics
import struct
import time
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlparse


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    return ordered[min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))]


def http_probe(base_url: str, connect_address: str | None = None) -> None:
    parsed = urlparse(base_url)
    secure = parsed.scheme == "https"
    port = parsed.port or (443 if secure else 80)
    stream = socket.create_connection((connect_address or parsed.hostname, port), timeout=10)
    if secure:
        stream = ssl.create_default_context().wrap_socket(stream, server_hostname=parsed.hostname)
    path = parsed.path or "/"
    host = parsed.hostname if parsed.port is None else f"{parsed.hostname}:{parsed.port}"
    stream.sendall(
        (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            "Connection: close\r\n"
            "Accept-Encoding: identity\r\n"
            "User-Agent: bonesaw-live-smoke/1\r\n\r\n"
        ).encode("ascii")
    )
    response = bytearray()
    while True:
        chunk = stream.recv(65536)
        if not chunk:
            break
        response.extend(chunk)
    stream.close()
    header, separator, body = bytes(response).partition(b"\r\n\r\n")
    assert separator and header.startswith(b"HTTP/1.1 200"), header.decode(
        "latin1", errors="replace"
    )
    assert b"Bonesaw" in body, "HTTP response is not the Bonesaw editor"


@dataclass
class RawWebSocket:
    stream: socket.socket

    @classmethod
    def connect(
        cls,
        base_url: str,
        connect_address: str | None = None,
        path: str = "/ws",
    ) -> "RawWebSocket":
        parsed = urlparse(base_url)
        secure = parsed.scheme == "https"
        port = parsed.port or (443 if secure else 80)
        stream = socket.create_connection((connect_address or parsed.hostname, port), timeout=10)
        if secure:
            stream = ssl.create_default_context().wrap_socket(stream, server_hostname=parsed.hostname)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host = parsed.hostname if parsed.port is None else f"{parsed.hostname}:{parsed.port}"
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "User-Agent: bonesaw-live-smoke/1\r\n\r\n"
        )
        stream.sendall(request.encode("ascii"))
        response = cls._read_until(stream, b"\r\n\r\n")
        header, remainder = response.split(b"\r\n\r\n", 1)
        assert header.startswith(b"HTTP/1.1 101"), header.decode("latin1", errors="replace")
        accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        ).decode("ascii")
        response_headers = {}
        for line in header.decode("latin1").split("\r\n")[1:]:
            name, separator, value = line.partition(":")
            if separator:
                response_headers[name.strip().lower()] = value.strip()
        assert response_headers.get("sec-websocket-accept") == accept
        instance = cls(stream)
        instance._buffer = bytearray(remainder)
        return instance

    @staticmethod
    def _read_until(stream: socket.socket, marker: bytes) -> bytes:
        data = bytearray()
        while marker not in data:
            chunk = stream.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed during WebSocket handshake")
            data.extend(chunk)
        return bytes(data)

    def _read_exact(self, length: int) -> bytes:
        while len(self._buffer) < length:
            chunk = self.stream.recv(max(4096, length - len(self._buffer)))
            if not chunk:
                raise ConnectionError("WebSocket closed")
            self._buffer.extend(chunk)
        result = bytes(self._buffer[:length])
        del self._buffer[:length]
        return result

    def receive_json(self) -> dict[str, Any]:
        while True:
            first, second = self._read_exact(2)
            opcode = first & 0x0F
            length = second & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._read_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._read_exact(8))[0]
            mask = self._read_exact(4) if second & 0x80 else None
            payload = self._read_exact(length)
            if mask:
                payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
            if opcode == 0x9:
                self._send_frame(payload, 0xA)
                continue
            if opcode == 0x8:
                raise ConnectionError("WebSocket closed by server")
            assert opcode == 0x1, f"unexpected WebSocket opcode {opcode}"
            return json.loads(payload)

    def _send_frame(self, payload: bytes, opcode: int) -> None:
        mask = os.urandom(4)
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(0x80 | length)
        elif length <= 0xFFFF:
            header.extend([0x80 | 126])
            header.extend(struct.pack("!H", length))
        else:
            header.extend([0x80 | 127])
            header.extend(struct.pack("!Q", length))
        masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        self.stream.sendall(bytes(header) + mask + masked)

    def send_json(self, value: dict[str, Any]) -> None:
        self._send_frame(json.dumps(value, separators=(",", ":")).encode(), 0x1)

    def close(self) -> None:
        try:
            self._send_frame(b"", 0x8)
        finally:
            self.stream.close()


def receive_kind(websocket: RawWebSocket, kind: str, attempts: int = 100) -> dict[str, Any]:
    for _ in range(attempts):
        message = websocket.receive_json()
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "server error"))
        if message.get("type") == kind:
            return message
    raise AssertionError(f"did not receive {kind!r} within {attempts} messages")


def frame_translation(message: dict[str, Any], frame_names: list[str], name: str) -> list[float]:
    frame_id = frame_names.index(name)
    frame = next(item for item in message["frames"] if item["id"] == frame_id)
    return frame["translation"]


def run(
    base_url: str, exercise_drag: bool, connect_address: str | None = None
) -> dict[str, Any]:
    http_probe(base_url, connect_address)
    websocket = RawWebSocket.connect(base_url, connect_address)
    try:
        hello = receive_kind(websocket, "hello")
        assert hello["dof"] == 6, hello["dof"]
        assert hello["bodies"] == 41, hello["bodies"]
        assert "upkie" in hello["model"].lower(), hello["model"]
        assert hello["rooted_frames"] == ["control_world", "odom", "map"]
        assert hello["base_execution"] in {"guided_preview", "raw_dynamic"}
        assert hello["squat_execution"] == hello["base_execution"]
        result: dict[str, Any] = {
            "url": base_url,
            "model": hello["model"],
            "dof": hello["dof"],
            "bodies": hello["bodies"],
            "http": "ok",
            "websocket": "ok",
        }
        if not exercise_drag:
            return result

        handle = next(item for item in hello["interaction_handles"] if item["kind"] == "base")
        initial = receive_kind(websocket, "state")
        arrivals: list[float] = []
        previous_arrival = time.perf_counter()
        for _ in range(4):
            initial = receive_kind(websocket, "state")
            arrived = time.perf_counter()
            arrivals.append((arrived - previous_arrival) * 1000)
            previous_arrival = arrived
        initial_reset_epoch = initial["reset_epoch"]
        initial_position = frame_translation(initial, hello["frame_names"], handle["frame"])
        target = [
            initial_position[0],
            initial_position[1],
            initial_position[2] - 0.03,
        ]
        sent_at = time.perf_counter()
        websocket.send_json({"type": "drag", "frame": handle["frame"], "target": target})
        displacement = 0.0
        acknowledged = None
        for _ in range(80):
            state = receive_kind(websocket, "state")
            arrived = time.perf_counter()
            arrivals.append((arrived - previous_arrival) * 1000)
            previous_arrival = arrived
            if state.get("active_frame") == handle["frame"] and acknowledged is None:
                acknowledged = (arrived - sent_at) * 1000
            position = frame_translation(state, hello["frame_names"], handle["frame"])
            displacement = max(
                displacement,
                math.sqrt(sum((value - initial_position[index]) ** 2 for index, value in enumerate(position))),
            )
            if acknowledged is not None and displacement >= 0.005:
                break
        assert acknowledged is not None, "drag was never acknowledged by active_frame"
        assert displacement >= 0.005, f"drag acknowledged but visible frame moved only {displacement:.6f} m"

        # Ordinary in-range torso intent stays exact. Ground/IK clipping is a
        # separate geometric boundary and must not alter this reachable target.
        target_states = [receive_kind(websocket, "state") for _ in range(12)]
        assert all(state.get("active_frame") == handle["frame"] for state in target_states)
        assert not any(
            state["metrics"]["interaction_target_clamped"] for state in target_states
        ), [
            state["metrics"]["interaction_target_clamp_error_m"]
            for state in target_states
        ]
        assert all(
            later["tick"] > earlier["tick"]
            for earlier, later in zip(target_states, target_states[1:])
        ), "stream stopped or reset while the Cartesian base target was active"
        final_base_position = frame_translation(
            target_states[-1], hello["frame_names"], handle["frame"]
        )
        base_target_residual = math.sqrt(
            sum(
                (value - target[index]) ** 2
                for index, value in enumerate(final_base_position)
            )
        )
        assert base_target_residual <= 0.008, (
            f"Cartesian base target residual remained {base_target_residual:.6f} m"
        )

        ground_target = [initial_position[0], initial_position[1], initial_position[2] - 0.50]
        websocket.send_json(
            {"type": "drag", "frame": handle["frame"], "target": ground_target}
        )
        ground_limited = None
        for _ in range(40):
            state = receive_kind(websocket, "state")
            if state["metrics"]["interaction_target_clamped"]:
                ground_limited = state
                break
        assert ground_limited is not None, "below-ground base target was not clipped"
        assert ground_limited["metrics"]["interaction_target_clamp_error_m"] > 0.0

        websocket.send_json({"type": "release"})
        release = None
        for _ in range(20):
            state = receive_kind(websocket, "state")
            if state.get("active_frame") is None:
                release = state
                break
        assert release is not None, "release was never reflected by active_frame"

        websocket.send_json({"type": "reset"})
        reset_state = None
        for _ in range(20):
            candidate = receive_kind(websocket, "state")
            if candidate["reset_epoch"] > initial_reset_epoch:
                reset_state = candidate
                break
        assert reset_state is not None, "reset epoch was never acknowledged"
        joint_handle = next(item for item in hello["interaction_handles"] if item["kind"] == "joint")
        joint_initial = frame_translation(
            reset_state, hello["frame_names"], joint_handle["frame"]
        )
        joint_target = [joint_initial[0] + 0.015, joint_initial[1], joint_initial[2] + 0.015]
        websocket.send_json(
            {"type": "drag", "frame": joint_handle["frame"], "target": joint_target}
        )
        joint_displacement = 0.0
        joint_acknowledged = False
        for _ in range(40):
            state = receive_kind(websocket, "state")
            joint_acknowledged |= state.get("active_frame") == joint_handle["frame"]
            position = frame_translation(state, hello["frame_names"], joint_handle["frame"])
            joint_displacement = max(
                joint_displacement,
                math.sqrt(
                    sum(
                        (value - joint_initial[index]) ** 2
                        for index, value in enumerate(position)
                    )
                ),
            )
            if joint_acknowledged and joint_displacement >= 0.001:
                break
        assert joint_acknowledged, "joint target was not acknowledged"
        assert joint_displacement >= 0.001, "joint target did not produce visible motion"
        websocket.send_json({"type": "release"})
        result.update(
            {
                "drag_frame": handle["frame"],
                "drag_ack_ms": acknowledged,
                "visible_displacement_m": displacement,
                "base_target_residual_m": base_target_residual,
                "base_target_contract": "exact_in_range_ground_clamped_out_of_range",
                "ground_clamp_error_m": ground_limited["metrics"][
                    "interaction_target_clamp_error_m"
                ],
                "preview_wbc_admitted": target_states[-1]["metrics"][
                    "guided_preview_wbc_admitted"
                ],
                "joint_drag_frame": joint_handle["frame"],
                "joint_visible_displacement_m": joint_displacement,
                "release": "ok",
                "snapshot_interval_ms": {
                    "p50": statistics.median(arrivals),
                    "p95": percentile(arrivals, 0.95),
                    "p99": percentile(arrivals, 0.99),
                    "max": max(arrivals),
                },
                "controller_solve_us": release["metrics"]["solve_us"],
            }
        )
        return result
    finally:
        websocket.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--exercise-drag", action="store_true")
    parser.add_argument("--connect-address")
    args = parser.parse_args()
    print(
        json.dumps(
            run(args.url.rstrip("/"), args.exercise_drag, args.connect_address),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
