#!/usr/bin/env python3
"""Probe whether equal forces at distinct points produce distinct live moments."""

from __future__ import annotations

import argparse
import json
import math
from typing import Any

from live_editor_smoke import RawWebSocket, http_probe, receive_kind
from live_plant_gateway_smoke import body_position, receive_plant


def rotation_vector(quaternion_wxyz: list[float]) -> list[float]:
    length = math.sqrt(sum(value * value for value in quaternion_wxyz))
    quaternion = [value / length for value in quaternion_wxyz]
    if quaternion[0] < 0.0:
        quaternion = [-value for value in quaternion]
    vector_norm = math.sqrt(sum(value * value for value in quaternion[1:]))
    if vector_norm < 1.0e-12:
        return [2.0 * value for value in quaternion[1:]]
    angle = 2.0 * math.atan2(vector_norm, quaternion[0])
    return [value * angle / vector_norm for value in quaternion[1:]]


def receive_correlated(
    websocket: RawWebSocket,
    request_id: int,
    *,
    active: bool,
    attempts: int = 80,
) -> dict[str, Any]:
    for _ in range(attempts):
        state = receive_plant(websocket, "plant_state")
        if state.get("command_id") != request_id:
            continue
        if bool(state["push"]["active"]) != active:
            continue
        return state
    raise AssertionError(f"request {request_id} was not correlated")


def run_condition(
    base_url: str,
    connect_address: str | None,
    *,
    offset_z_m: float,
    force_n: float = 2.0,
    push_frames: int = 3,
) -> dict[str, Any]:
    websocket = RawWebSocket.connect(base_url, connect_address, "/plant-ws")
    try:
        hello = receive_plant(websocket, "plant_hello")
        initial = receive_plant(websocket, "plant_state")
        initial_pitch = rotation_vector(initial["root_quaternion_wxyz"])[1]
        initial_pitch_rate = initial["root_twist_world"][1]
        point = body_position(initial, "base")
        point[2] += offset_z_m
        states = []
        request_id = 100 + round((offset_z_m + 1.0) * 1000)
        for _ in range(push_frames):
            websocket.send_json(
                {
                    "type": "plant_push",
                    "body": "base",
                    "force_world": [force_n, 0.0, 0.0],
                    "application_point_world": point,
                    "request_id": request_id,
                }
            )
            states.append(
                receive_correlated(websocket, request_id, active=True)
            )
        websocket.send_json({"type": "plant_release", "request_id": request_id + 1})
        released = receive_correlated(websocket, request_id + 1, active=False)
        signed_pitch = [
            rotation_vector(state["root_quaternion_wxyz"])[1] - initial_pitch
            for state in states
        ]
        signed_pitch_rate = [
            state["root_twist_world"][1] - initial_pitch_rate for state in states
        ]
        return {
            "offset_z_m": offset_z_m,
            "force_n": force_n,
            "point_world": point,
            "maximum_application_offset_m": hello[
                "maximum_application_offset_m"
            ],
            "moment_y_nm": [state["push"]["moment_world_nm"][1] for state in states],
            "moment_magnitude_nm": [
                state["push"]["maximum_moment_nm"] for state in states
            ],
            "application_offset_m": [
                state["push"]["application_offset_m"] for state in states
            ],
            "signed_pitch_rad": signed_pitch,
            "signed_pitch_rate_rad_s": signed_pitch_rate,
            "final_pitch_rad": signed_pitch[-1],
            "final_pitch_rate_rad_s": signed_pitch_rate[-1],
            "controller_step_us": [
                state["metrics"]["controller_step_us"] for state in states
            ],
            "worker_step_us": [
                state["metrics"]["worker_step_us"] for state in states
            ],
            "released_tick": released["tick"],
            "fallen": any(state["metrics"]["fallen"] for state in states),
        }
    finally:
        websocket.close()


def run_rejection(base_url: str, connect_address: str | None) -> dict[str, Any]:
    websocket = RawWebSocket.connect(base_url, connect_address, "/plant-ws")
    try:
        receive_plant(websocket, "plant_hello")
        initial = receive_plant(websocket, "plant_state")
        point = body_position(initial, "base")
        point[2] += 0.751
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [2.0, 0.0, 0.0],
                "application_point_world": point,
                "request_id": 9001,
            }
        )
        error = receive_plant(websocket, "plant_error")
        survived = receive_plant(websocket, "plant_state")
        assert "offset limit" in error["message"], error
        assert survived["tick"] > initial["tick"], survived
        assert not survived["push"]["active"], survived["push"]
        return {
            "point_world": point,
            "message": error["message"],
            "stream_survived": True,
            "active_push_cleared": True,
        }
    finally:
        websocket.close()


def run(base_url: str, connect_address: str | None = None) -> dict[str, Any]:
    http_probe(base_url, connect_address)
    control = RawWebSocket.connect(base_url, connect_address, "/ws")
    try:
        hello = receive_kind(control, "hello")
    finally:
        control.close()
    assert hello["plant_gateway"]["maximum_application_offset_m"] == 0.75
    conditions = [
        run_condition(base_url, connect_address, offset_z_m=offset)
        for offset in (-0.2, 0.0, 0.2)
    ]
    low, center, high = conditions
    assert low["moment_y_nm"][-1] < -0.35
    assert abs(center["moment_y_nm"][-1]) < 0.01
    assert high["moment_y_nm"][-1] > 0.35
    assert 0.75 < high["moment_y_nm"][-1] - low["moment_y_nm"][-1] < 0.85
    assert low["final_pitch_rad"] < center["final_pitch_rad"] < high["final_pitch_rad"]
    assert (
        low["final_pitch_rate_rad_s"]
        < center["final_pitch_rate_rad_s"]
        < high["final_pitch_rate_rad_s"]
    )
    assert not any(item["fallen"] for item in conditions)
    return {
        "url": base_url,
        "conditions": conditions,
        "excessive_offset_rejection": run_rejection(base_url, connect_address),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--connect-address")
    args = parser.parse_args()
    print(
        json.dumps(
            run(args.url.rstrip("/"), args.connect_address),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
