#!/usr/bin/env python3
"""Dependency-free HTTP/WebSocket smoke test for the hosted Upkie editor."""

from __future__ import annotations

import argparse
import base64
import hashlib
import http.client
import json
import math
import os
import socket
import ssl
import statistics
import struct
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any
from urllib.parse import urljoin, urlparse


HTTP_ASSET_ATTEMPTS = 3
HTTP_ASSET_TIMEOUT_S = 12.0


def chunked_message_length(body: bytes | bytearray) -> int | None:
    """Return the framed chunked-body length once its zero chunk has arrived."""
    cursor = 0
    while True:
        line_end = body.find(b"\r\n", cursor)
        if line_end < 0:
            return None
        size = int(body[cursor:line_end].split(b";", 1)[0], 16)
        payload_end = line_end + 2 + size
        if len(body) < payload_end + 2:
            return None
        assert body[payload_end : payload_end + 2] == b"\r\n"
        cursor = payload_end + 2
        if size == 0:
            return cursor


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    return ordered[min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))]


def _http_get_bytes_once(
    base_url: str, path: str, connect_address: str | None = None
) -> bytes:
    parsed = urlparse(base_url)
    secure = parsed.scheme == "https"
    port = parsed.port or (443 if secure else 80)
    stream = socket.create_connection(
        (connect_address or parsed.hostname, port), timeout=HTTP_ASSET_TIMEOUT_S
    )
    if secure:
        stream = ssl.create_default_context().wrap_socket(stream, server_hostname=parsed.hostname)
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
    header_end: int | None = None
    content_length: int | None = None
    chunked = False
    try:
        while True:
            chunk = stream.recv(65536)
            if not chunk:
                break
            response.extend(chunk)
            if header_end is None:
                marker = response.find(b"\r\n\r\n")
                if marker >= 0:
                    header_end = marker + 4
                    preliminary_headers: dict[str, str] = {}
                    for line in response[:marker].decode("latin1").split("\r\n")[1:]:
                        name, separator, value = line.partition(":")
                        if separator:
                            preliminary_headers[name.strip().lower()] = value.strip()
                    if "content-length" in preliminary_headers:
                        content_length = int(preliminary_headers["content-length"])
                    chunked = (
                        preliminary_headers.get("transfer-encoding", "").lower()
                        == "chunked"
                    )
            if header_end is not None and content_length is not None:
                if len(response) >= header_end + content_length:
                    del response[header_end + content_length :]
                    break
            if header_end is not None and chunked:
                framed_length = chunked_message_length(response[header_end:])
                if framed_length is not None:
                    del response[header_end + framed_length :]
                    break
    except TimeoutError as error:
        stream.close()
        raise TimeoutError(
            f"HTTP read for {path!r} exceeded {HTTP_ASSET_TIMEOUT_S:.0f} seconds"
        ) from error
    stream.close()
    header, separator, body = bytes(response).partition(b"\r\n\r\n")
    assert separator and header.startswith(b"HTTP/1.1 200"), header.decode(
        "latin1", errors="replace"
    )
    headers: dict[str, str] = {}
    for line in header.decode("latin1").split("\r\n")[1:]:
        name, separator, value = line.partition(":")
        if separator:
            headers[name.strip().lower()] = value.strip()
    if "content-length" in headers:
        expected_length = int(headers["content-length"])
        if len(body) != expected_length:
            raise ConnectionError(
                f"HTTP body for {path!r} ended at {len(body)} of {expected_length} bytes"
            )
    if headers.get("transfer-encoding", "").lower() == "chunked":
        decoded = bytearray()
        cursor = 0
        while True:
            line_end = body.index(b"\r\n", cursor)
            size = int(body[cursor:line_end].split(b";", 1)[0], 16)
            cursor = line_end + 2
            if size == 0:
                break
            decoded.extend(body[cursor : cursor + size])
            cursor += size + 2
        body = bytes(decoded)
    return body


def http_get_bytes(
    base_url: str, path: str, connect_address: str | None = None
) -> bytes:
    """Read one asset with bounded retries for transient quick-tunnel stalls."""
    for attempt in range(1, HTTP_ASSET_ATTEMPTS + 1):
        try:
            if connect_address is not None:
                return _http_get_bytes_once(base_url, path, connect_address)
            url = urljoin(f"{base_url.rstrip('/')}/", path.lstrip("/"))
            with urllib.request.urlopen(url, timeout=HTTP_ASSET_TIMEOUT_S) as response:
                assert response.status == 200, f"HTTP {response.status} for {path!r}"
                return response.read()
        except (
            TimeoutError,
            ConnectionError,
            http.client.IncompleteRead,
            ssl.SSLError,
            urllib.error.URLError,
        ):
            if attempt == HTTP_ASSET_ATTEMPTS:
                raise
    raise AssertionError("unreachable HTTP retry state")


def http_probe(base_url: str, connect_address: str | None = None) -> None:
    parsed = urlparse(base_url)
    body = http_get_bytes(base_url, parsed.path or "/", connect_address)
    assert b"Bonesaw" in body, "HTTP response is not the Bonesaw editor"
    assert b"actuator-budget" in body, "hosted editor is missing actuator budget UI"
    assert b"/motion-rig-r16.js?v=235" in body, "hosted editor served stale script"
    script = http_get_bytes(base_url, "/motion-rig-r16.js?v=235", connect_address)
    assert b"updatePlantAuthorityStack" in script, "hosted editor lacks measured plant authority stack"


def quaternion_rotate(rotation: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = rotation
    tx = 2.0 * (y * vector[2] - z * vector[1])
    ty = 2.0 * (z * vector[0] - x * vector[2])
    tz = 2.0 * (x * vector[1] - y * vector[0])
    return [
        vector[0] + w * tx + (y * tz - z * ty),
        vector[1] + w * ty + (z * tx - x * tz),
        vector[2] + w * tz + (x * ty - y * tx),
    ]


def add(left: list[float], right: list[float]) -> list[float]:
    return [a + b for a, b in zip(left, right)]


def binary_stl_vertices(payload: bytes) -> list[list[float]]:
    if len(payload) < 84:
        raise AssertionError("truncated visual STL")
    triangles = struct.unpack_from("<I", payload, 80)[0]
    assert 84 + 50 * triangles <= len(payload), "invalid visual STL triangle count"
    vertices: set[tuple[float, float, float]] = set()
    for triangle in range(triangles):
        offset = 84 + triangle * 50 + 12
        for corner in range(3):
            vertices.add(struct.unpack_from("<fff", payload, offset + 12 * corner))
    return [list(vertex) for vertex in vertices]


def minimum_visual_ground_clearance(
    base_url: str,
    hello: dict[str, Any],
    state: dict[str, Any],
    connect_address: str | None,
    mesh_cache: dict[str, list[list[float]]],
) -> float:
    frames = {frame["id"]: frame for frame in state["frames"]}
    minimum = math.inf
    mesh_prefix = "package://upkie_description/meshes/"
    for visual in hello.get("visual_geometry", []):
        shape = visual["shape"]
        frame = frames[shape["body"]]
        body_rotation = frame["rotation_xyzw"]
        center = add(
            frame["translation"],
            quaternion_rotate(body_rotation, shape["translation"]),
        )
        shape_axis = quaternion_rotate(shape["rotation_xyzw"], [0.0, 0.0, 1.0])
        world_axis = quaternion_rotate(body_rotation, shape_axis)
        kind = shape["kind"]
        if kind == "sphere":
            candidate = center[2] - shape["radius"]
        elif kind == "capsule":
            candidate = (
                center[2]
                - shape["half_length"] * abs(world_axis[2])
                - shape["radius"]
            )
        elif kind == "cylinder":
            radial_z = math.sqrt(max(0.0, 1.0 - world_axis[2] ** 2))
            candidate = (
                center[2]
                - shape["half_length"] * abs(world_axis[2])
                - shape["radius"] * radial_z
            )
        elif kind == "box":
            candidate = center[2]
            for axis, extent in enumerate(shape["half_extents"]):
                local_axis = [0.0, 0.0, 0.0]
                local_axis[axis] = 1.0
                shape_direction = quaternion_rotate(shape["rotation_xyzw"], local_axis)
                world_direction = quaternion_rotate(body_rotation, shape_direction)
                candidate -= abs(world_direction[2]) * extent
        elif kind == "mesh":
            filename = shape["filename"]
            assert filename.startswith(mesh_prefix), filename
            path = "/model-assets/upkie/meshes/" + filename[len(mesh_prefix) :]
            if path not in mesh_cache:
                mesh_cache[path] = binary_stl_vertices(
                    http_get_bytes(base_url, path, connect_address)
                )
            candidate = math.inf
            for vertex in mesh_cache[path]:
                scaled = [
                    vertex[axis] * shape["scale"][axis] for axis in range(3)
                ]
                in_body = add(
                    shape["translation"],
                    quaternion_rotate(shape["rotation_xyzw"], scaled),
                )
                candidate = min(
                    candidate,
                    add(
                        frame["translation"],
                        quaternion_rotate(body_rotation, in_body),
                    )[2],
                )
        else:
            raise AssertionError(f"unsupported visual geometry kind {kind!r}")
        minimum = min(minimum, candidate)
    assert math.isfinite(minimum), "visual geometry did not produce a ground clearance"
    return minimum


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

        mesh_cache: dict[str, list[list[float]]] = {}
        mesh_prefix = "package://upkie_description/meshes/"
        for visual in hello.get("visual_geometry", []):
            shape = visual["shape"]
            if shape["kind"] != "mesh":
                continue
            filename = shape["filename"]
            assert filename.startswith(mesh_prefix), filename
            path = "/model-assets/upkie/meshes/" + filename[len(mesh_prefix) :]
            if path not in mesh_cache:
                mesh_cache[path] = binary_stl_vertices(
                    http_get_bytes(base_url, path, connect_address)
                )
        # Mesh downloads can take several seconds through a public tunnel and
        # therefore queue old 50 Hz states on this dependency-free socket.
        # Start the actual interaction trial on a fresh stream after all ten
        # unique assets are cached locally.
        websocket.close()
        websocket = RawWebSocket.connect(base_url, connect_address)
        trial_hello = receive_kind(websocket, "hello")
        assert trial_hello["model"] == hello["model"]
        hello = trial_hello
        handles = hello["interaction_handles"]
        assert handles and all(item["kind"] == "frame" for item in handles), handles
        handle = next(item for item in handles if item["frame"] == "torso")
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

        # An ordinary in-range advertised-frame draft stays exact. Ground/IK
        # clipping is a separate geometric boundary and must not alter this
        # reachable target.
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
        ), "stream stopped or reset while the Cartesian frame target was active"
        target_collision_ground_clearance = min(
            state["metrics"]["minimum_collision_ground_clearance_m"]
            for state in target_states
        )
        assert target_collision_ground_clearance >= -1.1e-6, (
            "reachable frame target penetrated the z=0 collision plane"
        )
        target_visual_state = target_states[-1]
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
            f"Cartesian frame target residual remained {base_target_residual:.6f} m"
        )

        ground_target = [initial_position[0], initial_position[1], initial_position[2] - 1.00]
        websocket.send_json(
            {"type": "drag", "frame": handle["frame"], "target": ground_target}
        )
        ground_limited = None
        # The guided root target has an explicit 10 mm/tick slew. A one-metre
        # impossible request therefore needs more than one hundred 20 ms frames
        # before it reaches the geometric clamp.
        for _ in range(160):
            state = receive_kind(websocket, "state")
            if state["metrics"]["interaction_target_clamped"]:
                ground_limited = state
                break
        assert ground_limited is not None, "below-ground frame target was not clipped"
        assert ground_limited["metrics"]["interaction_target_clamp_error_m"] > 0.0
        assert (
            ground_limited["metrics"]["minimum_collision_ground_clearance_m"]
            >= -1.1e-6
        ), "clipped frame target still penetrated the z=0 collision plane"
        ground_limited_collision_clearance = ground_limited["metrics"][
            "minimum_collision_ground_clearance_m"
        ]
        ground_limited_visual_state = ground_limited

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
        joint_handle = next(
            item
            for item in hello["interaction_handles"]
            if item["frame"] == "left_knee_qdd100_rotor"
        )
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
        target_visual_clearance = minimum_visual_ground_clearance(
            base_url,
            hello,
            target_visual_state,
            connect_address,
            mesh_cache,
        )
        assert target_visual_clearance >= -1.0e-3, (
            f"reachable frame target visual mesh penetrated z=0 by "
            f"{-1000.0 * target_visual_clearance:.3f} mm"
        )
        ground_limited_visual_clearance = minimum_visual_ground_clearance(
            base_url,
            hello,
            ground_limited_visual_state,
            connect_address,
            mesh_cache,
        )
        assert ground_limited_visual_clearance >= -1.0e-3, (
            f"clipped frame target visual mesh penetrated z=0 by "
            f"{-1000.0 * ground_limited_visual_clearance:.3f} mm"
        )
        result.update(
            {
                "drag_frame": handle["frame"],
                "drag_ack_ms": acknowledged,
                "visible_displacement_m": displacement,
                "frame_target_residual_m": base_target_residual,
                "frame_target_contract": "exact_in_range_ground_clamped_out_of_range",
                "ground_clamp_error_m": ground_limited["metrics"][
                    "interaction_target_clamp_error_m"
                ],
                "target_visual_ground_clearance_m": target_visual_clearance,
                "ground_limited_visual_clearance_m": ground_limited_visual_clearance,
                "target_collision_ground_clearance_m": target_collision_ground_clearance,
                "ground_limited_collision_clearance_m": ground_limited_collision_clearance,
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
