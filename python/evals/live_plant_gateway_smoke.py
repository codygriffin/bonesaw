#!/usr/bin/env python3
"""End-to-end smoke and timing probe for the live physical plant gateway."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import time
from typing import Any

from live_editor_smoke import RawWebSocket, http_probe, percentile, receive_kind


EVALUATION_PROVENANCE = {
    "source": "evaluation_harness",
    "load_class": "declared_continuous_wrench",
    "force_frame": "world",
    "application_point_frame": "world",
}


def receive_plant(
    websocket: RawWebSocket,
    kind: str,
    *,
    attempts: int = 120,
    states: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    for _ in range(attempts):
        message = websocket.receive_json()
        if message.get("type") == "plant_state" and states is not None:
            states.append(message)
        if message.get("type") == kind:
            return message
        if message.get("type") == "plant_unavailable":
            raise RuntimeError(message.get("reason", "plant unavailable"))
    raise AssertionError(f"did not receive {kind!r} within {attempts} messages")


def receive_correlated_state(
    websocket: RawWebSocket,
    request_id: int,
    *,
    paused: bool | None = None,
    attempts: int = 120,
) -> dict[str, Any]:
    """Wait for a plant heartbeat carrying one lifecycle command id."""
    for _ in range(attempts):
        state = receive_plant(websocket, "plant_state")
        if state.get("command_id") != request_id:
            continue
        if paused is not None and bool(state.get("paused")) != paused:
            continue
        return state
    raise AssertionError(f"did not receive correlated plant state {request_id}")


def body_position(state: dict[str, Any], name: str) -> list[float]:
    return next(frame["translation"] for frame in state["frames"] if frame["name"] == name)


def distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def run(base_url: str, connect_address: str | None = None) -> dict[str, Any]:
    http_probe(base_url, connect_address)

    control = RawWebSocket.connect(base_url, connect_address, "/ws")
    try:
        control_hello = receive_kind(control, "hello")
    finally:
        control.close()
    gateway = control_hello["plant_gateway"]
    assert gateway["available"], gateway
    assert gateway["websocket_path"] == "/plant-ws", gateway
    assert gateway["maximum_force_n"] == 8.0, gateway
    assert gateway["maximum_application_offset_m"] == 0.75, gateway
    assert gateway["command_ttl_ms"] == 140, gateway
    assert gateway["worker_timeout_ms"] >= 100, gateway
    assert gateway["external_load_protocol"] == 2, gateway
    assert gateway["accepted_external_load_sources"] == [
        "interactive_operator",
        "evaluation_harness",
    ], gateway
    assert (
        gateway["executable_external_load_class"]
        == "declared_continuous_wrench"
    ), gateway

    websocket_started = time.perf_counter()
    websocket = RawWebSocket.connect(base_url, connect_address, gateway["websocket_path"])
    try:
        hello = receive_plant(websocket, "plant_hello")
        worker_startup_ms = (time.perf_counter() - websocket_started) * 1.0e3
        assert hello["protocol"] == 2, hello
        assert hello["physics_hz"] == 250, hello
        assert hello["control_hz"] == 50, hello
        assert hello["stream_hz"] == 50, hello
        assert hello["physics_substeps_per_control"] == 5, hello
        assert hello["actuator_names"] == [
            "left_hip_motor",
            "left_knee_motor",
            "left_wheel_motor",
            "right_hip_motor",
            "right_knee_motor",
            "right_wheel_motor",
        ], hello["actuator_names"]
        assert hello["actuator_effort_limits_nm"] == [
            16.0,
            16.0,
            1.7,
            16.0,
            16.0,
            1.7,
        ], hello["actuator_effort_limits_nm"]
        assert hello["actuator_resource_models"] == [False] * 6
        assert (
            hello["actuator_resource_contract"]["thermal_reliability"]
            == "unmodeled; no calibrated electrical/thermal state"
        )
        assert hello["contact_observation"] == {
            "sample_hz": 250,
            "consumed_hz": 50,
            "window_size": 5,
            "wbc_source": "latest_completed_250hz_substep",
            "prestart_samples": 0,
        }, hello["contact_observation"]
        assert hello["simulator"]["backend"] == "MuJoCo", hello
        assert hello["maximum_force_n"] == gateway["maximum_force_n"], hello
        assert (
            hello["maximum_application_offset_m"]
            == gateway["maximum_application_offset_m"]
        ), hello
        assert "base" in hello["body_names"], hello["body_names"]
        assert (
            hello["external_load_contract"]["executable_class"]
            == "declared_continuous_wrench"
        ), hello
        feedforward = hello["external_load_contract"][
            "wbc_external_wrench_feedforward"
        ]
        assert feedforward["axis_order"] == "root_moment_xyz_then_root_force_xyz", hello
        # The hosted production worker must not silently enable the exploratory
        # R318 vector; evaluation-only profiles opt into it explicitly.
        assert feedforward["axis_scales"] is None, hello

        initial = receive_plant(websocket, "plant_state")
        assert initial["simulator"]["physics_dt_s"] == 0.004, initial["simulator"]
        assert initial["simulator"]["control_dt_s"] == 0.020, initial["simulator"]
        assert initial["simulator"]["physics_substeps"] == 5, initial["simulator"]
        assert initial["simulator"]["ground_plane_z_m"] == 0.0, initial["simulator"]
        assert initial["simulator"]["ground_plane_point_world"] == [0.0, 0.0, 0.0]
        assert initial["simulator"]["ground_plane_normal_world"] == [0.0, 0.0, 1.0]
        assert len(initial["simulator"]["solver_forward_inverse"]) == 2
        assert initial["simulator"]["constraint_count"] == len(
            initial["constraint_force"]
        )
        assert initial["simulator"]["warning_count"] == 0, initial["simulator"]
        assert initial["simulator"]["contact_window_valid"]
        assert len(initial["simulator"]["contact_window_masks"]) == 5
        assert (
            initial["simulator"]["contact_window_frame_end"]
            - initial["simulator"]["contact_window_frame_start"]
            == 4
        )
        assert initial["wbc_observation"]["source"] == (
            "latest_completed_250hz_substep"
        )
        # The schema always carries the independently typed contingency
        # witnesses, while the hosted production profile remains disabled.
        assert initial["metrics"]["wbc_support_contingency_enabled"] is False
        assert initial["metrics"]["wbc_support_contingency_requested"] is False
        assert initial["metrics"]["wbc_support_contingency_admitted"] is False
        assert initial["metrics"]["wbc_support_contingency_selected"] is False
        assert math.isfinite(
            initial["metrics"]["wbc_support_contingency_candidate_power_w"]
        )
        assert math.isfinite(initial["simulator"]["kinetic_energy_j"])
        assert math.isfinite(initial["simulator"]["potential_energy_j"])
        assert initial["metrics"]["ground_contact_count"] >= 1, initial["metrics"]
        assert initial["contacts"], "MuJoCo contacts were not streamed"
        observed_contact = initial["metrics"]["wbc_observed_contact_active"]
        debounced_contact = initial["metrics"]["wbc_debounced_contact_active"]
        hard_contact = initial["metrics"]["wbc_hard_contact_active"]
        executable_contact = initial["metrics"]["wbc_hard_contact_executable"]
        assert initial["metrics"]["wbc_observed_contact_available"]
        assert initial["metrics"]["wbc_raw_status_code"] in (0, 1, 2, 3)
        assert initial["metrics"]["wbc_allocation_calls"] == 0
        assert initial["metrics"]["wbc_allocated_bytes"] == 0
        for key in (
            "wbc_maximum_constraint_violation",
            "wbc_dynamics_residual",
            "wbc_contact_residual",
        ):
            assert math.isfinite(initial["metrics"][key]), key
        assert initial["wbc_observed_contact_active"] == observed_contact
        assert initial["wbc_debounced_contact_active"] == debounced_contact
        assert initial["wbc_hard_contact_active"] == hard_contact
        assert initial["wbc_hard_contact_executable"] == executable_contact
        assert (
            len(observed_contact)
            == len(debounced_contact)
            == len(hard_contact)
            == len(executable_contact)
            == 2
        )
        assert all(
            hard <= observed
            for hard, observed in zip(hard_contact, observed_contact)
        )
        assert initial["metrics"]["wbc_support_active_count"] <= sum(observed_contact)
        assert all(
            executable <= hard
            for executable, hard in zip(executable_contact, hard_contact)
        )
        if not initial["metrics"]["wbc_admitted"]:
            assert executable_contact == [0, 0]
        causal_successor = receive_plant(websocket, "plant_state")
        assert causal_successor["wbc_observation"]["physics_frame_index"] == (
            initial["simulator"]["contact_window_frame_end"]
        )
        assert causal_successor["wbc_observation"]["contact_active"] == (
            initial["simulator"]["contact_window_masks"][-1]
        )
        assert causal_successor["simulator"]["contact_window_frame_start"] == (
            initial["simulator"]["contact_window_frame_end"] + 1
        )
        assert len(initial["actuator_effort_nm"]) == 6
        assert len(initial["actuator_effort_limit_nm"]) == 6
        assert len(initial["actuator_effort_utilization"]) == 6
        assert len(initial["actuator_velocity_rad_s"]) == 6
        assert len(initial["actuator_mechanical_power_w"]) == 6
        assert all(
            math.isfinite(value) and 0.0 <= value <= 1.0 + 1.0e-12
            for value in initial["actuator_effort_utilization"]
        )
        assert len(initial["generalized_acceleration"]) == 12
        assert len(initial["constraint_generalized_force"]) == 12
        assert len(initial["actuator_generalized_force"]) == 12
        assert len(initial["passive_generalized_force"]) == 12
        assert len(initial["bias_generalized_force"]) == 12
        assert len(initial["center_of_mass_world"]) == 3
        assert all(math.isfinite(value) for value in initial["center_of_mass_world"])
        assert initial["metrics"]["total_ground_normal_force_n"] > 0.0
        assert not initial["external_load"]["active"], initial["external_load"]
        assert not initial["measured_impact_impulse"]["available"]
        assert not initial["unobserved_model_reserve"]["available"]
        for key in (
            "maximum_abs_joint_speed_rad_s",
            "maximum_abs_actuator_effort_nm",
            "maximum_actuator_effort_utilization",
            "maximum_abs_actuator_mechanical_power_w",
            "maximum_abs_generalized_acceleration",
            "maximum_abs_constraint_force",
        ):
            assert math.isfinite(initial["metrics"][key]), key
        initial_epoch = initial["reset_epoch"]
        initial_base = body_position(initial, "base")
        initial_root = initial["root_position"]

        # Lifecycle commands are explicit and measured-state based. Pause must
        # freeze MuJoCo time/q/v while continuing heartbeats; resume must start
        # from that state; reset while paused must clear the lease and preserve
        # paused ownership. These are intentionally exercised over the real
        # WebSocket gateway, not just the in-process worker.
        before_pause = receive_plant(websocket, "plant_state")
        pause_tick = before_pause["tick"]
        websocket.send_json({"type": "plant_pause", "request_id": 240})
        paused = receive_correlated_state(websocket, 240, paused=True)
        # The command may arrive between two 50 Hz ticks, so the first
        # acknowledged paused frame is the freeze point. Subsequent heartbeats
        # must retain it exactly.
        pause_time = paused["simulator"]["time_s"]
        heartbeat = receive_plant(websocket, "plant_state")
        assert heartbeat["tick"] > pause_tick
        assert heartbeat["simulator"]["time_s"] == pause_time
        assert heartbeat["metrics"]["wbc_status"] == "paused"
        assert not heartbeat["metrics"]["wbc_observed_contact_available"]
        assert heartbeat["wbc_debounced_contact_active"] == [0, 0]
        assert heartbeat["wbc_hard_contact_active"] == [0, 0]
        assert heartbeat["metrics"]["wbc_support_active_count"] == 0
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [1.0, 0.0, 0.0],
                "application_point_world": body_position(paused, "base"),
                "provenance": EVALUATION_PROVENANCE,
                "request_id": 241,
            }
        )
        paused_push_error = receive_plant(websocket, "plant_error")
        assert "paused" in paused_push_error["message"]
        websocket.send_json({"type": "plant_resume", "request_id": 242})
        resumed = receive_correlated_state(websocket, 242, paused=False)
        assert resumed["simulator"]["time_s"] > pause_time

        websocket.send_json({"type": "plant_pause", "request_id": 243})
        receive_correlated_state(websocket, 243, paused=True)
        websocket.send_json({"type": "plant_reset", "request_id": 244})
        reset_paused = receive_correlated_state(websocket, 244, paused=True)
        assert reset_paused["reset_epoch"] > initial_epoch
        assert reset_paused["simulator"]["time_s"] == 0.0
        assert not reset_paused["metrics"]["wbc_observed_contact_available"]
        assert reset_paused["wbc_debounced_contact_active"] == [0, 0]
        assert reset_paused["wbc_hard_contact_active"] == [0, 0]
        assert reset_paused["metrics"]["wbc_support_active_count"] == 0
        initial_epoch = reset_paused["reset_epoch"]
        websocket.send_json({"type": "plant_resume", "request_id": 245})
        receive_correlated_state(websocket, 245, paused=False)

        # Evidence-only load classes may be reported but cannot be executed
        # through the declared external-wrench command path.
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [1.0, 0.0, 0.0],
                "application_point_world": initial_base,
                "provenance": {
                    **EVALUATION_PROVENANCE,
                    "load_class": "measured_impact_impulse",
                },
                "request_id": 99,
            }
        )
        wrong_class = receive_plant(websocket, "plant_error")
        assert "evidence-only" in wrong_class["message"], wrong_class
        after_wrong_class = receive_plant(websocket, "plant_state")
        assert not after_wrong_class["external_load"]["active"]

        # A rejected overload must not poison the session or stop the stream.
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [8.01, 0.0, 0.0],
                "application_point_world": initial_base,
                "provenance": EVALUATION_PROVENANCE,
                "request_id": 100,
            }
        )
        overload = receive_plant(websocket, "plant_error")
        assert "force limit" in overload["message"], overload
        after_overload = receive_plant(websocket, "plant_state")
        assert after_overload["tick"] > initial["tick"]

        # A bounded wrench is correlated and then expires without a release.
        ttl_request = 101
        sent_at = time.perf_counter()
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [2.0, 0.0, 0.0],
                "application_point_world": body_position(after_overload, "base"),
                "provenance": EVALUATION_PROVENANCE,
                "request_id": ttl_request,
            }
        )
        active = None
        expired = None
        intervals_ms: list[float] = []
        previous_arrival = time.perf_counter()
        for _ in range(30):
            state = receive_plant(websocket, "plant_state")
            arrived = time.perf_counter()
            intervals_ms.append((arrived - previous_arrival) * 1.0e3)
            previous_arrival = arrived
            if (
                state["external_load"]["active"]
                and state["external_load"]["request_id"] == ttl_request
            ):
                active = active or (state, (arrived - sent_at) * 1.0e3)
            if state["command_expired"]:
                expired = (state, (arrived - sent_at) * 1.0e3)
                break
        assert active is not None, "bounded plant_push was never acknowledged"
        assert (
            active[0]["external_load"]["provenance"]
            == EVALUATION_PROVENANCE
        ), active[0]["external_load"]
        assert expired is not None, "plant_push did not expire at its fail-safe TTL"
        assert not expired[0]["external_load"]["active"], expired[0]["external_load"]
        assert gateway["command_ttl_ms"] <= expired[1] <= gateway["command_ttl_ms"] + 120

        # Refresh one request ID while the pointer is held, release it, then
        # observe the model settle without a policy layer or browser-side physics.
        recovery_request = 200
        baseline = receive_plant(websocket, "plant_state")
        baseline_root = baseline["root_position"]
        push_states: list[dict[str, Any]] = []
        push_started = time.perf_counter()
        recovery_command_acknowledged = False
        for _ in range(10):
            websocket.send_json(
                {
                    "type": "plant_push",
                    "body": "base",
                    "force_world": [5.0, 0.0, 0.0],
                    "application_point_world": body_position(baseline, "base"),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": recovery_request,
                }
            )
            # A public tunnel can already have snapshots in flight when the
            # command arrives. Correlate explicitly instead of assuming the
            # very next frame reflects the most recent client message.
            for _ in range(5):
                state = receive_plant(websocket, "plant_state")
                push_states.append(state)
                if (
                    state["command_id"] == recovery_request
                    and state["external_load"]["active"]
                ):
                    recovery_command_acknowledged = True
                    break
        assert recovery_command_acknowledged, "refreshed push was never correlated"
        websocket.send_json({"type": "plant_release", "request_id": 201})

        released = None
        settled = None
        fallen_observed = False
        automatic_fall_reset_observed = False
        recovery_states: list[dict[str, Any]] = []
        consecutive_settled = 0
        # A local worker often enters the settled envelope within 150 frames;
        # the same 50 Hz plant behind a quick Cloudflare tunnel can receive a
        # shorter push burst and then recover more slowly without ever falling.
        # Keep the assertion bounded, but allow six seconds for that valid
        # transport-dependent settling tail.
        for _ in range(300):
            state = receive_plant(websocket, "plant_state")
            recovery_states.append(state)
            fallen_observed = fallen_observed or bool(state["metrics"]["fallen"])
            automatic_fall_reset_observed = automatic_fall_reset_observed or (
                state.get("automatic_reset_reason") == "fall"
            )
            if not state["external_load"]["active"] and state["command_id"] == 201:
                released = released or state
            metrics = state["metrics"]
            is_settled = (
                not metrics["fallen"]
                and abs(metrics["root_tilt_rad"]) < 0.08
                and abs(metrics["station_error_m"]) < 0.05
            )
            consecutive_settled = consecutive_settled + 1 if is_settled else 0
            if released is not None and consecutive_settled >= 8:
                settled = state
                break
        assert released is not None, "plant_release was never reflected"
        assert settled is not None, "plant failed to reach the recovery envelope"
        # A 5 N burst is intentionally bounded rather than a guaranteed fall:
        # local and tunneled transports can deliver different numbers of
        # controller frames while the lease is refreshed.  Accept either the
        # stronger fall/reset witness or a finite no-fall recovery.  The
        # policy-free R310–R313 matrices remain the authoritative fall tests.
        if fallen_observed:
            assert automatic_fall_reset_observed, "fall did not expose its automatic reset"
            assert settled["reset_epoch"] > initial_epoch, "automatic reset did not advance epoch"
        else:
            assert settled["reset_epoch"] >= initial_epoch

        all_motion_states = push_states + recovery_states
        maximum_root_displacement = max(
            distance(state["root_position"], baseline_root) for state in all_motion_states
        )
        maximum_tilt = max(abs(state["metrics"]["root_tilt_rad"]) for state in all_motion_states)
        maximum_capture_pressure = max(
            state["metrics"]["capture_pressure"] for state in all_motion_states
        )
        assert maximum_root_displacement > 1.0e-4, maximum_root_displacement
        assert any(state["external_load"]["active"] for state in push_states)

        # Explicit reset is correlated and advances the worker epoch.
        websocket.send_json({"type": "plant_reset", "request_id": 300})
        reset = None
        for _ in range(20):
            state = receive_plant(websocket, "plant_state")
            if state["command_id"] == 300 and state["reset_epoch"] > initial_epoch:
                reset = state
                break
        assert reset is not None, "plant_reset did not advance reset_epoch"

        result = {
            "url": base_url,
            "gateway": gateway,
            "worker": {
                "model": hello["model"],
                "physics_hz": hello["physics_hz"],
                "control_hz": hello["control_hz"],
                "stream_hz": hello["stream_hz"],
                "startup_ms": worker_startup_ms,
            },
            "stream_interval_ms": {
                "p50": statistics.median(intervals_ms),
                "p95": percentile(intervals_ms, 0.95),
                "max": max(intervals_ms),
            },
            "command": {
                "ack_ms": active[1],
                "expiry_ms": expired[1],
                "overload_rejected_without_disconnect": True,
                "evidence_only_class_rejected_without_disconnect": True,
                "provenance_echoed_exactly": active[0]["external_load"][
                    "provenance"
                ]
                == EVALUATION_PROVENANCE,
                "release_acknowledged": True,
                "reset_acknowledged": True,
                "pause_resume_reset_lifecycle": True,
                "paused_wrench_rejected": True,
            },
            "external_load_accounting": {
                "protocol": hello["protocol"],
                "executable_class": hello["external_load_contract"][
                    "executable_class"
                ],
                "source": active[0]["external_load"]["provenance"]["source"],
                "measured_impact_impulse_available": active[0][
                    "measured_impact_impulse"
                ]["available"],
                "unobserved_model_reserve_available": active[0][
                    "unobserved_model_reserve"
                ]["available"],
            },
            "response": {
                "push_duration_ms": (time.perf_counter() - push_started) * 1.0e3,
                "maximum_root_displacement_m": maximum_root_displacement,
                "maximum_root_tilt_rad": maximum_tilt,
                "maximum_capture_pressure": maximum_capture_pressure,
                "settled_station_error_m": settled["metrics"]["station_error_m"],
                "settled_root_tilt_rad": settled["metrics"]["root_tilt_rad"],
                "fall_resets": settled["metrics"]["fall_resets"],
                "fall_observed": fallen_observed,
                "automatic_fall_reset_observed": automatic_fall_reset_observed,
                "numeric_resets": settled["metrics"]["numeric_resets"],
            },
            "initial": {
                "root_position": initial_root,
                "base_position": initial_base,
            },
        }
    finally:
        websocket.close()

    # A second connection gets a fresh isolated worker after the first child
    # is dropped; this is the deploy-facing reconnect gate.
    reconnect_started = time.perf_counter()
    reconnect = RawWebSocket.connect(base_url, connect_address, gateway["websocket_path"])
    try:
        reconnect_hello = receive_plant(reconnect, "plant_hello")
        reconnect_state = receive_plant(reconnect, "plant_state")
        assert reconnect_state["tick"] >= 1
        assert reconnect_hello["model"] == result["worker"]["model"]
        result["reconnect"] = {
            "status": "ok",
            "fresh_worker_tick": reconnect_state["tick"],
            "startup_ms": (time.perf_counter() - reconnect_started) * 1.0e3,
        }
    finally:
        reconnect.close()
    return result


def run_retained(base_url: str, connect_address: str | None = None) -> dict[str, Any]:
    """Reproduce the five-trial r131 admission probe consumed by the report."""
    http_probe(base_url, connect_address)
    control = RawWebSocket.connect(base_url, connect_address, "/ws")
    try:
        control_hello = receive_kind(control, "hello")
    finally:
        control.close()
    gateway = control_hello["plant_gateway"]
    assert gateway["available"]

    websocket = RawWebSocket.connect(
        base_url, connect_address, gateway["websocket_path"]
    )
    states: list[dict[str, Any]] = []
    intervals_ms: list[float] = []
    previous_arrival: float | None = None

    def next_message() -> dict[str, Any]:
        nonlocal previous_arrival
        message = websocket.receive_json()
        if message.get("type") == "plant_unavailable":
            raise RuntimeError(message.get("reason", "plant unavailable"))
        if message.get("type") == "plant_state":
            arrived = time.perf_counter()
            if previous_arrival is not None:
                intervals_ms.append((arrived - previous_arrival) * 1.0e3)
            previous_arrival = arrived
            states.append(message)
        return message

    def next_kind(kind: str, attempts: int = 160) -> dict[str, Any]:
        for _ in range(attempts):
            message = next_message()
            if message.get("type") == kind:
                return message
        raise AssertionError(f"did not receive {kind!r}")

    def next_correlated(
        request_id: int, *, active: bool | None = None, attempts: int = 80
    ) -> dict[str, Any]:
        for _ in range(attempts):
            state = next_kind("plant_state")
            if state.get("command_id") != request_id:
                continue
            if active is not None and bool(state["external_load"]["active"]) != active:
                continue
            return state
        raise AssertionError(f"request {request_id} was not correlated")

    try:
        hello = next_kind("plant_hello")
        initial = next_kind("plant_state")
        initial_epoch = initial["reset_epoch"]
        initial_root = initial["root_position"]
        initial_tilt = initial["metrics"]["root_tilt_rad"]
        application_point = body_position(initial, "base")

        push_started = time.perf_counter()
        for _ in range(5):
            websocket.send_json(
                {
                    "type": "plant_push",
                    "body": "base",
                    "force_world": [4.0, 0.0, 0.0],
                    "application_point_world": application_point,
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 101,
                }
            )
            next_correlated(101, active=True)
        push_window_ack_ms = (time.perf_counter() - push_started) * 1.0e3

        release_started = time.perf_counter()
        websocket.send_json({"type": "plant_release", "request_id": 102})
        next_correlated(102, active=False)
        release_ack_ms = (time.perf_counter() - release_started) * 1.0e3
        recovery_states = [next_kind("plant_state") for _ in range(105)]

        expiry_started = time.perf_counter()
        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [1.0, 0.0, 0.0],
                "application_point_world": body_position(recovery_states[-1], "base"),
                "provenance": EVALUATION_PROVENANCE,
                "request_id": 103,
            }
        )
        next_correlated(103, active=True)
        expired = None
        for _ in range(30):
            candidate = next_kind("plant_state")
            if candidate["command_expired"]:
                expired = candidate
                break
        assert expired is not None
        expiry_observed_ms = (time.perf_counter() - expiry_started) * 1.0e3

        websocket.send_json(
            {
                "type": "plant_push",
                "body": "base",
                "force_world": [8.01, 0.0, 0.0],
                "application_point_world": application_point,
                "provenance": EVALUATION_PROVENANCE,
                "request_id": 104,
            }
        )
        invalid = next_kind("plant_error")
        assert "force limit" in invalid["message"]
        survived = next_kind("plant_state")

        reset_started = time.perf_counter()
        websocket.send_json({"type": "plant_reset", "request_id": 105})
        reset = next_correlated(105, active=False)
        while reset["reset_epoch"] <= initial_epoch:
            reset = next_correlated(105, active=False)
        reset_ack_ms = (time.perf_counter() - reset_started) * 1.0e3

        response_states = states[: states.index(recovery_states[-1]) + 1]
        controller_us = [state["metrics"]["controller_step_us"] for state in states]
        worker_us = [state["metrics"]["worker_step_us"] for state in states]

        def distribution(values: list[float]) -> dict[str, float]:
            return {
                "p50": statistics.median(values),
                "p99": percentile(values, 0.99),
                "maximum": max(values),
            }

        return {
            "url": base_url,
            "http": "ok",
            "primary_websocket": "ok",
            "plant_websocket": "ok",
            "plant_boundary": hello["boundary"],
            "stream_hz": hello["stream_hz"],
            "control_hz": hello["control_hz"],
            "physics_hz": hello["physics_hz"],
            "maximum_force_n": hello["maximum_force_n"],
            "command_ttl_ms": gateway["command_ttl_ms"],
            "push_ack": 101,
            "push_window_ack_ms": push_window_ack_ms,
            "maximum_root_x_delta_m": max(
                abs(state["root_position"][0] - initial_root[0])
                for state in response_states
            ),
            "maximum_tilt_delta_deg": math.degrees(
                max(
                    abs(state["metrics"]["root_tilt_rad"] - initial_tilt)
                    for state in response_states
                )
            ),
            "maximum_capture_pressure": max(
                state["metrics"]["capture_pressure"] for state in response_states
            ),
            "release_ack": 102,
            "release_ack_ms": release_ack_ms,
            "expiry": "ok",
            "expiry_observed_ms": expiry_observed_ms,
            "invalid_force_rejected_stream_survived": survived["tick"] > initial["tick"],
            "reset_ack": 105,
            "reset_ack_ms": reset_ack_ms,
            "reset_root_error_m": distance(reset["root_position"], initial_root),
            "numeric_resets": max(state["metrics"]["numeric_resets"] for state in states),
            "controller_step_us": distribution(controller_us),
            "worker_step_us": distribution(worker_us),
            "stream_interval_ms": distribution(intervals_ms),
            "final_recovery": {
                "tilt_deg": math.degrees(
                    abs(recovery_states[-1]["metrics"]["root_tilt_rad"])
                ),
                "station_error_m": recovery_states[-1]["metrics"]["station_error_m"],
                "station_authority": recovery_states[-1]["metrics"]["station_authority"],
            },
        }
    finally:
        websocket.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--connect-address")
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    result = run(args.url.rstrip("/"), args.connect_address)
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
