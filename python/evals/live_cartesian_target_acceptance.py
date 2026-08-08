#!/usr/bin/env python3
"""Deterministic live-plant acceptance probe for Cartesian handle commands.

Run this with the MuJoCo evaluation environment after rebuilding the PyO3
extension, for example::

    PYTHONPATH=python/evals /tmp/bonesaw-mujoco/bin/python \
      python/evals/live_cartesian_target_acceptance.py --pretty

The probe drives ``LiveUpkiePlant`` directly.  This keeps transport and wall
clock scheduling out of the measurement while exercising the same MuJoCo,
balance adapter, Rust WBC, and command-state path used by the WebSocket worker.
It prints a machine-readable report and exits nonzero when any fixed acceptance
criterion fails.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from typing import Any, Callable

import numpy as np

from upkie_live_plant_worker import (
    COMMAND_HOLD_TOLERANCE_M,
    COMMAND_X_DAMPING_FULL_ERROR_M,
    COMMAND_X_DAMPING_NEUTRAL_ERROR_M,
    LiveUpkiePlant,
)


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_MODEL = REPO_ROOT / "models/upkie/upkie.urdf"
SETTLE_TICKS = 100
TRAJECTORY_TICKS = 150  # 3.0 s at the production 50 Hz control rate.
HOLD_TICKS = 50
PUSH_TICKS = 25
PRE_PUSH_TARGET_TICKS = 40
SUSTAINED_RECOVERY_MAX_TICKS = 520
SUSTAINED_HOLD_DWELL_TICKS = 50
MEDIAN_WINDOW = 5
NONWHEEL_COORDINATES = np.asarray([0, 1, 3, 4], dtype=np.int64)


def frame_position(state: dict[str, Any], name: str) -> np.ndarray:
    for frame in state["frames"]:
        if frame["name"] == name:
            return np.asarray(frame["translation"], dtype=np.float64)
    raise KeyError(f"plant state omitted frame {name!r}")


def median_frame(states: list[dict[str, Any]], name: str) -> np.ndarray:
    samples = np.asarray(
        [frame_position(state, name) for state in states[-MEDIAN_WINDOW:]],
        dtype=np.float64,
    )
    return np.median(samples, axis=0)


def median_vector(states: list[dict[str, Any]], key: str) -> np.ndarray:
    samples = np.asarray(
        [state[key] for state in states[-MEDIAN_WINDOW:]], dtype=np.float64
    )
    return np.median(samples, axis=0)


def root_tilt_rad(state: dict[str, Any]) -> float:
    quaternion = np.asarray(state["root_quaternion_wxyz"], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        return math.inf
    w, x, y, z = quaternion / norm
    # Angle between the root and world vertical axes, independent of yaw.
    root_vertical_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(float(np.clip(root_vertical_z, -1.0, 1.0)))


def criterion(
    actual: Any,
    expected: Any,
    comparison: str,
    predicate: Callable[[Any, Any], bool],
) -> dict[str, Any]:
    return {
        "passed": bool(predicate(actual, expected)),
        "actual": actual,
        "comparison": comparison,
        "expected": expected,
    }


def safety_report(states: list[dict[str, Any]]) -> dict[str, Any]:
    status_codes = [
        int(state["metrics"].get("wbc_raw_status_code", -1)) for state in states
    ]
    admitted = [bool(state["metrics"].get("wbc_admitted", False)) for state in states]
    hard_contacts = [state.get("wbc_hard_contact_active", []) for state in states]
    hard_executable = [
        state.get("wbc_hard_contact_executable", []) for state in states
    ]
    normals = np.asarray(
        [state.get("wbc_observed_wheel_normal_force_n", [math.nan, math.nan]) for state in states],
        dtype=np.float64,
    )
    root_heights = np.asarray(
        [state["root_position"][2] for state in states], dtype=np.float64
    )
    tilts = np.asarray([root_tilt_rad(state) for state in states], dtype=np.float64)
    utilizations = np.asarray(
        [state["actuator_effort_utilization"] for state in states],
        dtype=np.float64,
    )
    maximum_residual = max(
        max(
            abs(float(state["metrics"].get(key, math.inf)))
            for key in (
                "wbc_maximum_constraint_violation",
                "wbc_dynamics_residual",
                "wbc_contact_residual",
            )
        )
        for state in states
    )
    maximum_allocations = max(
        int(state["metrics"].get("wbc_allocation_calls", -1)) for state in states
    )
    maximum_allocated_bytes = max(
        int(state["metrics"].get("wbc_allocated_bytes", -1)) for state in states
    )
    warning_count = max(
        int(state["simulator"].get("warning_count", -1)) for state in states
    )
    reset_epochs = [int(state["reset_epoch"]) for state in states]
    numeric_resets = sum(bool(state.get("numeric_reset", False)) for state in states)
    automatic_resets = sum(
        state.get("automatic_reset_reason") is not None for state in states
    )
    pending_resets = sum(
        state.get("automatic_reset_pending") is not None for state in states
    )
    controller_step_us = np.asarray(
        [state["metrics"].get("controller_step_us", math.inf) for state in states],
        dtype=np.float64,
    )
    worker_step_us = np.asarray(
        [state["metrics"].get("worker_step_us", math.inf) for state in states],
        dtype=np.float64,
    )
    rolling_zero_mean_residual = max(
        abs(float(state["metrics"].get("command_rolling_zero_mean_residual", math.inf)))
        for state in states
    )
    rolling_common_residual = max(
        abs(float(state["metrics"].get("command_rolling_common_residual", math.inf)))
        for state in states
    )
    rolling_protected_residual = max(
        abs(float(state["metrics"].get("command_rolling_protected_residual", math.inf)))
        for state in states
    )
    maximum_differential_acceleration = max(
        abs(float(state["metrics"].get("command_rolling_maximum_differential_acceleration_m_s2", math.inf)))
        for state in states
    )
    maximum_protected_acceleration = max(
        abs(float(state["metrics"].get("command_rolling_maximum_protected_acceleration_rad_s2", math.inf)))
        for state in states
    )

    checks = {
        "status_is_admitted_0_or_1": criterion(
            sorted(set(status_codes)), [0, 1], "is a nonempty subset of", lambda a, b: bool(a) and set(a) <= set(b)
        ),
        "every_tick_admitted": criterion(all(admitted), True, "==", lambda a, b: a == b),
        "bilateral_hard_contact": criterion(
            all(mask == [1, 1] for mask in hard_contacts), True, "==", lambda a, b: a == b
        ),
        "bilateral_hard_contact_executable": criterion(
            all(mask == [1, 1] for mask in hard_executable), True, "==", lambda a, b: a == b
        ),
        "normal_forces_finite": criterion(
            bool(np.all(np.isfinite(normals))), True, "==", lambda a, b: a == b
        ),
        "minimum_wheel_normal_force_n": criterion(
            float(np.min(normals)), 5.0, ">", lambda a, b: a > b
        ),
        "minimum_total_normal_force_n": criterion(
            float(np.min(np.sum(normals, axis=1))), 20.0, ">", lambda a, b: a > b
        ),
        "minimum_root_height_m": criterion(
            float(np.min(root_heights)), 0.45, ">", lambda a, b: a > b
        ),
        "maximum_root_tilt_rad": criterion(
            float(np.max(tilts)), 0.2, "<", lambda a, b: a < b
        ),
        "maximum_actuator_utilization": criterion(
            float(np.max(utilizations)), 0.8, "<=", lambda a, b: a <= b
        ),
        "maximum_wbc_residual": criterion(
            float(maximum_residual), 1.0e-6, "<=", lambda a, b: a <= b
        ),
        "maximum_timed_allocation_calls": criterion(
            maximum_allocations, 0, "==", lambda a, b: a == b
        ),
        "maximum_timed_allocated_bytes": criterion(
            maximum_allocated_bytes, 0, "==", lambda a, b: a == b
        ),
        "maximum_mujoco_warning_count": criterion(
            warning_count, 0, "==", lambda a, b: a == b
        ),
        "controller_step_p99_us": criterion(
            float(np.percentile(controller_step_us, 99)),
            5_000.0,
            "<=",
            lambda a, b: a <= b,
        ),
        "direct_worker_step_p99_us": criterion(
            float(np.percentile(worker_step_us, 99)),
            20_000.0,
            "<=",
            lambda a, b: a <= b,
        ),
        "rolling_zero_mean_residual": criterion(
            rolling_zero_mean_residual, 1.0e-12, "<=", lambda a, b: a <= b
        ),
        "rolling_common_row_residual": criterion(
            rolling_common_residual, 1.0e-10, "<=", lambda a, b: a <= b
        ),
        # Exact hard equality on the two protected rolling accelerations.
        "rolling_hard_selector_residual_rad_s2": criterion(
            rolling_protected_residual, 1.0e-6, "<=", lambda a, b: a <= b
        ),
        "maximum_differential_acceleration_m_s2": criterion(
            maximum_differential_acceleration, 0.25, "<=", lambda a, b: a <= b + 1.0e-12
        ),
        "maximum_protected_wheel_acceleration_rad_s2": criterion(
            maximum_protected_acceleration, 200.0, "<=", lambda a, b: a <= b + 1.0e-9
        ),
        "numeric_resets": criterion(numeric_resets, 0, "==", lambda a, b: a == b),
        "automatic_resets": criterion(
            automatic_resets, 0, "==", lambda a, b: a == b
        ),
        "pending_automatic_resets": criterion(
            pending_resets, 0, "==", lambda a, b: a == b
        ),
        "reset_epoch_constant": criterion(
            len(set(reset_epochs)), 1, "==", lambda a, b: a == b
        ),
    }
    return {
        "passed": all(check["passed"] for check in checks.values()),
        "ticks": len(states),
        "checks": checks,
    }


def settle(worker: LiveUpkiePlant) -> tuple[dict[str, Any], dict[str, Any]]:
    states = [worker.step({"type": "step"}) for _ in range(SETTLE_TICKS)]
    tail = states[-5:]
    ready = all(
        bool(state["metrics"].get("wbc_admitted", False))
        and state.get("wbc_hard_contact_active") == [1, 1]
        for state in tail
    )
    return states[-1], {
        "passed": ready,
        "ticks": SETTLE_TICKS,
        "last_five_admitted_bilateral": ready,
        "final_root_position_m": states[-1]["root_position"],
        "final_wheel_normal_force_n": states[-1][
            "wbc_observed_wheel_normal_force_n"
        ],
    }


def command(
    worker: LiveUpkiePlant,
    frame: str,
    target: np.ndarray,
    request_id: int,
    ticks: int,
    *,
    duration_ms: int = 3000,
    external_load: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    initial: dict[str, Any] = {
        "type": "step",
        "target_command": {
            "active": True,
            "frame": frame,
            "handle_id": f"frame:{frame}",
            "target": target.tolist(),
            "duration_ms": duration_ms,
            "request_id": request_id,
        },
    }
    if external_load is not None:
        initial["external_load"] = external_load
    states = [worker.step(initial)]
    states.extend(worker.step({"type": "step"}) for _ in range(ticks - 1))
    return states


def torso_scenario(model: pathlib.Path) -> dict[str, Any]:
    worker = LiveUpkiePlant(model)
    settled, settle_report = settle(worker)
    start = frame_position(settled, "torso")
    down_target = start + np.asarray([0.0, 0.0, -0.030], dtype=np.float64)
    down = command(worker, "torso", down_target, 1, TRAJECTORY_TICKS)
    down_endpoint = median_frame(down, "torso")
    down_hold = [worker.step({"type": "step"}) for _ in range(HOLD_TICKS)]
    down_hold_endpoint = median_frame(down_hold, "torso")
    return_states = command(worker, "torso", start, 2, TRAJECTORY_TICKS)
    return_endpoint = median_frame(return_states, "torso")

    checks = {
        "down_signed_travel_m": criterion(
            float(start[2] - down_endpoint[2]), 0.020, ">=", lambda a, b: a >= b
        ),
        "down_endpoint_error_m": criterion(
            float(np.linalg.norm(down_endpoint - down_target)), 0.020, "<=", lambda a, b: a <= b
        ),
        "down_hold_drift_m": criterion(
            float(np.linalg.norm(down_hold_endpoint - down_endpoint)), 0.010, "<=", lambda a, b: a <= b
        ),
        "down_hold_phase": criterion(
            down_hold[-1]["target_command"].get("phase"), "holding", "==", lambda a, b: a == b
        ),
        "down_target_identity_persists": criterion(
            all(state["target_command"].get("request_id") == 1 for state in down + down_hold),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "down_normalized_handle_identity_persists": criterion(
            all(
                state["target_command"].get("handle_id") == "frame:torso"
                for state in down + down_hold
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "return_signed_travel_m": criterion(
            float(return_endpoint[2] - down_hold_endpoint[2]), 0.020, ">=", lambda a, b: a >= b
        ),
        "return_endpoint_error_m": criterion(
            float(np.linalg.norm(return_endpoint - start)), 0.015, "<=", lambda a, b: a <= b
        ),
        "return_target_identity_persists": criterion(
            all(state["target_command"].get("request_id") == 2 for state in return_states),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "return_normalized_handle_identity_persists": criterion(
            all(
                state["target_command"].get("handle_id") == "frame:torso"
                for state in return_states
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
    }
    scored = down + down_hold + return_states
    safety = safety_report(scored)
    return {
        "passed": settle_report["passed"]
        and safety["passed"]
        and all(check["passed"] for check in checks.values()),
        "settle": settle_report,
        "checks": checks,
        "safety": safety,
        "start_position_world_m": start.tolist(),
        "down_target_world_m": down_target.tolist(),
        "down_endpoint_world_m": down_endpoint.tolist(),
        "down_hold_endpoint_world_m": down_hold_endpoint.tolist(),
        "return_endpoint_world_m": return_endpoint.tolist(),
        "down_peak_signed_travel_m": float(
            max(start[2] - frame_position(state, "torso")[2] for state in down)
        ),
    }


def knee_scenario(model: pathlib.Path, side: str, request_id: int) -> dict[str, Any]:
    frame = f"{side}_knee_qdd100_rotor"
    worker = LiveUpkiePlant(model)
    settled, settle_report = settle(worker)
    start = frame_position(settled, frame)
    start_root = np.asarray(settled["root_position"], dtype=np.float64)
    start_joints = np.asarray(settled["joint_positions"], dtype=np.float64)
    target = start + np.asarray([0.015, 0.0, 0.0], dtype=np.float64)
    motion = command(worker, frame, target, request_id, TRAJECTORY_TICKS)
    endpoint = median_frame(motion, frame)
    endpoint_root = median_vector(motion, "root_position")
    endpoint_joints = median_vector(motion, "joint_positions")
    hold = [worker.step({"type": "step"}) for _ in range(HOLD_TICKS)]
    hold_endpoint = median_frame(hold, frame)
    root_participation = float(np.linalg.norm(endpoint_root - start_root))
    joint_participation = float(
        np.max(np.abs(endpoint_joints[NONWHEEL_COORDINATES] - start_joints[NONWHEEL_COORDINATES]))
    )
    checks = {
        "signed_travel_m": criterion(
            float(endpoint[0] - start[0]), 0.008, ">=", lambda a, b: a >= b
        ),
        "endpoint_error_m": criterion(
            float(np.linalg.norm(endpoint - target)), 0.012, "<=", lambda a, b: a <= b
        ),
        "hold_drift_m": criterion(
            float(np.linalg.norm(hold_endpoint - endpoint)), 0.010, "<=", lambda a, b: a <= b
        ),
        "hold_phase": criterion(
            hold[-1]["target_command"].get("phase"), "holding", "==", lambda a, b: a == b
        ),
        "target_identity_persists": criterion(
            all(
                state["target_command"].get("request_id") == request_id
                for state in motion + hold
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "normalized_handle_identity_persists": criterion(
            all(
                state["target_command"].get("handle_id") == f"frame:{frame}"
                for state in motion + hold
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "whole_body_participation": criterion(
            bool(root_participation > 0.001 or joint_participation > 0.010),
            True,
            "==",
            lambda a, b: a == b,
        ),
    }
    safety = safety_report(motion + hold)
    return {
        "passed": settle_report["passed"]
        and safety["passed"]
        and all(check["passed"] for check in checks.values()),
        "frame": frame,
        "settle": settle_report,
        "checks": checks,
        "safety": safety,
        "start_position_world_m": start.tolist(),
        "target_position_world_m": target.tolist(),
        "endpoint_position_world_m": endpoint.tolist(),
        "hold_endpoint_world_m": hold_endpoint.tolist(),
        "root_participation_m": root_participation,
        "maximum_nonwheel_joint_participation_rad": joint_participation,
        "peak_signed_travel_m": float(
            max(frame_position(state, frame)[0] - start[0] for state in motion)
        ),
    }


def ankle_scenario(model: pathlib.Path, side: str, request_id: int) -> dict[str, Any]:
    frame = f"{side}_ankle_mj5208_rotor"
    worker = LiveUpkiePlant(model)
    settled, settle_report = settle(worker)
    start = frame_position(settled, frame)
    target = start + np.asarray([0.010, 0.0, 0.0], dtype=np.float64)
    motion = command(worker, frame, target, request_id, TRAJECTORY_TICKS)
    endpoint = median_frame(motion, frame)
    hold = [worker.step({"type": "step"}) for _ in range(HOLD_TICKS)]
    hold_endpoint = median_frame(hold, frame)
    expected_handle = f"frame:{frame}"
    checks = {
        "signed_travel_m": criterion(
            float(endpoint[0] - start[0]), 0.005, ">=", lambda a, b: a >= b
        ),
        "endpoint_error_m": criterion(
            float(np.linalg.norm(endpoint - target)), 0.012, "<=", lambda a, b: a <= b
        ),
        "hold_drift_m": criterion(
            float(np.linalg.norm(hold_endpoint - endpoint)), 0.010, "<=", lambda a, b: a <= b
        ),
        "hold_phase": criterion(
            hold[-1]["target_command"].get("phase"), "holding", "==", lambda a, b: a == b
        ),
        "target_identity_persists": criterion(
            all(
                state["target_command"].get("request_id") == request_id
                for state in motion + hold
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "normalized_handle_identity_persists": criterion(
            all(
                state["target_command"].get("handle_id") == expected_handle
                for state in motion + hold
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
    }
    safety = safety_report(motion + hold)
    return {
        "passed": settle_report["passed"]
        and safety["passed"]
        and all(check["passed"] for check in checks.values()),
        "frame": frame,
        "settle": settle_report,
        "checks": checks,
        "safety": safety,
        "start_position_world_m": start.tolist(),
        "target_position_world_m": target.tolist(),
        "endpoint_position_world_m": endpoint.tolist(),
        "hold_endpoint_world_m": hold_endpoint.tolist(),
        "peak_signed_travel_m": float(
            max(frame_position(state, frame)[0] - start[0] for state in motion)
        ),
    }


def admission_and_composition_scenario(model: pathlib.Path) -> dict[str, Any]:
    worker = LiveUpkiePlant(model)
    settled, settle_report = settle(worker)
    torso = frame_position(settled, "torso")

    unclamped_request = torso + np.asarray([0.500, 0.0, 0.0])
    clamped_state = command(worker, "torso", unclamped_request, 20, 1)[0]
    target_telemetry = clamped_state["target_command"]
    admitted = np.asarray(
        target_telemetry["admitted_position_world"], dtype=np.float64
    )
    admitted_start = np.asarray(
        target_telemetry["start_position_world"], dtype=np.float64
    )

    for _ in range(10):
        prior = worker.step({"type": "step"})
    replacement_frame = "left_knee_qdd100_rotor"
    replacement_start = frame_position(prior, replacement_frame)
    replacement_target = replacement_start + np.asarray([0.015, 0.0, 0.0])
    body_id = worker.body_by_name["base"]
    application_point = np.asarray(worker.data.xipos[body_id], dtype=np.float64)
    external_load = {
        "active": True,
        "body": "base",
        "force_world": [1.0, 0.0, 0.0],
        "application_point_world": application_point.tolist(),
        "provenance": {
            "source": "evaluation_harness",
            "load_class": "declared_continuous_wrench",
            "force_frame": "world",
            "application_point_frame": "world",
        },
        "request_id": 22,
    }
    qpos_before = np.asarray(worker.data.qpos, dtype=np.float64).copy()
    # Admission itself must only author the WBC plan.  Check that boundary
    # separately from the one physical tick needed to prove push composition.
    worker._accept_target_command(
        {
            "active": True,
            "frame": replacement_frame,
            "handle_id": f"frame:{replacement_frame}",
            "target": replacement_target.tolist(),
            "duration_ms": 3000,
            "request_id": 21,
        }
    )
    qpos_after_admission = np.asarray(worker.data.qpos, dtype=np.float64).copy()
    target_command = {
        "active": True,
        "frame": replacement_frame,
        "handle_id": f"frame:{replacement_frame}",
        "target": replacement_target.tolist(),
        "duration_ms": 3000,
        "request_id": 21,
    }
    replacement = worker.step(
        {
            "type": "step",
            "target_command": target_command,
            "external_load": external_load,
        }
    )
    replacement_telemetry = replacement["target_command"]
    replacement_plan_start = np.asarray(
        replacement_telemetry["start_position_world"], dtype=np.float64
    )
    checks = {
        "unreachable_request_is_clamped": criterion(
            bool(target_telemetry.get("clamped", False)), True, "==", lambda a, b: a == b
        ),
        "uniform_clamp_displacement_m": criterion(
            float(np.linalg.norm(admitted - admitted_start)), 0.050, "approximately ==", lambda a, b: abs(a - b) <= 1.0e-12
        ),
        "replacement_frame_selected": criterion(
            replacement_telemetry.get("frame"), replacement_frame, "==", lambda a, b: a == b
        ),
        "replacement_starts_at_measured_frame": criterion(
            float(np.linalg.norm(replacement_plan_start - replacement_start)), 1.0e-12, "<=", lambda a, b: a <= b
        ),
        "replacement_does_not_write_qpos": criterion(
            float(np.linalg.norm(qpos_after_admission - qpos_before)), 0.0, "==", lambda a, b: a == b
        ),
        "push_remains_active_with_target": criterion(
            bool(replacement["external_load"].get("active", False)), True, "==", lambda a, b: a == b
        ),
        "push_and_target_request_ids_independent": criterion(
            (
                replacement_telemetry.get("request_id"),
                replacement["external_load"].get("request_id"),
            ),
            (21, 22),
            "==",
            lambda a, b: a == b,
        ),
    }
    safety = safety_report([clamped_state, replacement])
    return {
        "passed": settle_report["passed"]
        and safety["passed"]
        and all(check["passed"] for check in checks.values()),
        "settle": settle_report,
        "checks": checks,
        "safety": safety,
    }


def sustained_push_release_scenario(model: pathlib.Path) -> dict[str, Any]:
    worker = LiveUpkiePlant(model)
    settled, settle_report = settle(worker)
    frame = "left_knee_qdd100_rotor"
    start = frame_position(settled, frame)
    target = start + np.asarray([0.015, 0.0, 0.0])
    target_command = {
        "active": True,
        "frame": frame,
        "handle_id": f"frame:{frame}",
        "target": target.tolist(),
        "duration_ms": 3000,
        "request_id": 31,
    }
    pre_push_states = [
        worker.step({"type": "step", "target_command": target_command})
        for _ in range(PRE_PUSH_TARGET_TICKS)
    ]
    frozen_progress = float(pre_push_states[-1]["target_command"]["progress"])
    body_id = worker.body_by_name["base"]
    external_load = {
        "active": True,
        "body": "base",
        "force_world": [1.0, 0.0, 0.0],
        "application_point_world": np.asarray(
            worker.data.xipos[body_id], dtype=np.float64
        ).tolist(),
        "provenance": {
            "source": "evaluation_harness",
            "load_class": "declared_continuous_wrench",
            "force_frame": "world",
            "application_point_frame": "world",
        },
        "request_id": 32,
    }
    push_states = [
        worker.step(
            {
                "type": "step",
                "target_command": target_command,
                "external_load": external_load,
            }
        )
        for _ in range(PUSH_TICKS)
    ]
    position_at_release = frame_position(push_states[-1], frame)
    # Direct equivalent of the worker request emitted by a correlated
    # WebSocket plant_release.
    release_state = worker.step(
        {"type": "step", "external_load": {"active": False, "request_id": 33}}
    )
    post_release = [release_state]
    post_release.extend(
        worker.step({"type": "step"})
        for _ in range(SUSTAINED_RECOVERY_MAX_TICKS - 1)
    )
    first_resumed_index = next(
        (
            index
            for index, state in enumerate(post_release)
            if state["target_command"].get("command_descriptors_active", False)
        ),
        None,
    )
    if first_resumed_index is None or first_resumed_index == 0:
        raise RuntimeError("PUSH recovery never produced a guarded resumed command")
    suppressed_recovery = post_release[:first_resumed_index]
    resumed_state = post_release[first_resumed_index]
    first_holding_tick = next(
        (
            index + 1
            for index, state in enumerate(post_release)
            if state["target_command"].get("phase") == "holding"
        ),
        None,
    )
    first_holding_index = (
        first_holding_tick - 1 if first_holding_tick is not None else None
    )
    first_hold_dwell = (
        post_release[
            first_holding_index : first_holding_index + SUSTAINED_HOLD_DWELL_TICKS
        ]
        if first_holding_index is not None
        else []
    )
    measured_before_resume = np.asarray(
        suppressed_recovery[-1]["target_command"]["measured_position_world"],
        dtype=np.float64,
    )
    endpoint = median_frame(post_release, frame)
    all_states = pre_push_states + push_states + post_release
    release_to_goal = target - position_at_release
    release_to_goal_distance = float(np.linalg.norm(release_to_goal))
    target_direction = (
        release_to_goal / release_to_goal_distance
        if release_to_goal_distance > 1.0e-12
        else np.zeros(3, dtype=np.float64)
    )
    checks = {
        "push_and_target_overlap_ticks": criterion(
            len(push_states), PUSH_TICKS, ">=", lambda a, b: a >= b
        ),
        "push_active_every_overlap_tick": criterion(
            all(state["external_load"].get("active", False) for state in push_states),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "same_tick_wrench_feedforward_every_push_tick": criterion(
            all(
                state["external_load"].get("wbc_feedforward_active", False)
                and np.allclose(
                    state["external_load"].get(
                        "wbc_observed_force_world_n", [math.nan] * 3
                    ),
                    [1.0, 0.0, 0.0],
                    atol=1.0e-12,
                )
                for state in push_states
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "target_progress_frozen_every_push_tick": criterion(
            all(
                abs(
                    float(state["target_command"].get("progress", math.nan))
                    - frozen_progress
                )
                <= 1.0e-12
                for state in push_states
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "target_progress_frozen_through_suppressed_recovery": criterion(
            all(
                abs(
                    float(state["target_command"].get("progress", math.nan))
                    - frozen_progress
                )
                <= 1.0e-12
                for state in suppressed_recovery
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "full_target_bundle_suppressed_during_push_and_recovery": criterion(
            all(
                state["target_command"].get("execution_bundle_suppressed", False)
                and not state["target_command"].get(
                    "command_descriptors_active", True
                )
                for state in push_states + suppressed_recovery
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "station_neutral_every_suppressed_tick_m": criterion(
            max(
                abs(
                    float(
                        state["target_command"].get(
                            "station_neutral_error_at_solve_m", math.inf
                        )
                    )
                )
                for state in push_states + suppressed_recovery
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "station_retarget_respects_live_error_limit_m": criterion(
            max(
                abs(
                    float(
                        state["target_command"].get(
                            "station_admitted_error_at_solve_m", math.inf
                        )
                    )
                )
                for state in all_states
            ),
            0.03,
            "<=",
            lambda a, b: a <= b + 1.0e-12,
        ),
        "station_retarget_cap_engages_after_delayed_push": criterion(
            any(
                state["target_command"].get(
                    "station_error_clamped_at_solve", False
                )
                for state in post_release[first_resumed_index:]
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "cartesian_x_position_and_acceleration_leave_target_direction_to_station": criterion(
            max(
                max(
                    abs(
                        float(
                            state["target_command"].get(
                                "cartesian_x_position_neutral_residual_at_solve_m",
                                math.inf,
                            )
                        )
                    ),
                    abs(
                        float(
                            state["target_command"].get(
                                "cartesian_desired_ax_at_solve_m_s2", math.inf
                            )
                        )
                    ),
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "cartesian_x_velocity_damping_beta_matches_smoothstep": criterion(
            max(
                abs(
                    float(
                        state["target_command"][
                            "cartesian_x_velocity_damping_beta_at_solve"
                        ]
                    )
                    - (
                        1.0
                        if float(state["target_command"]["progress"])
                        < 1.0 - 1.0e-12
                        else (
                            lambda z: z * z * (3.0 - 2.0 * z)
                        )(
                            float(
                                np.clip(
                                    (
                                        abs(
                                            float(
                                                state["target_command"][
                                                    "station_admitted_error_at_solve_m"
                                                ]
                                            )
                                        )
                                        - COMMAND_X_DAMPING_FULL_ERROR_M
                                    )
                                    / (
                                        COMMAND_X_DAMPING_NEUTRAL_ERROR_M
                                        - COMMAND_X_DAMPING_FULL_ERROR_M
                                    ),
                                    0.0,
                                    1.0,
                                )
                            )
                        )
                    )
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "cartesian_x_velocity_damping_has_both_endpoints": criterion(
            sorted({
                float(
                    state["target_command"][
                        "cartesian_x_velocity_damping_beta_at_solve"
                    ]
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
                and float(
                    state["target_command"][
                        "cartesian_x_velocity_damping_beta_at_solve"
                    ]
                )
                in (0.0, 1.0)
            }),
            [0.0, 1.0],
            "==",
            lambda a, b: a == b,
        ),
        "cartesian_x_desired_velocity_matches_damping_beta": criterion(
            max(
                abs(
                    float(
                        state["target_command"][
                            "cartesian_desired_vx_at_solve_m_s"
                        ]
                    )
                    - float(
                        state["target_command"][
                            "cartesian_x_velocity_damping_beta_at_solve"
                        ]
                    )
                    * float(
                        state["target_command"][
                            "cartesian_measured_vx_at_solve_m_s"
                        ]
                    )
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "cartesian_x_velocity_residual_is_passive_damping": criterion(
            max(
                float(
                    state["target_command"][
                        "cartesian_vx_damping_residual_at_solve_m_s"
                    ]
                )
                * float(
                    state["target_command"][
                        "cartesian_measured_vx_at_solve_m_s"
                    ]
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "station_requested_error_owns_sampled_frame_x_m": criterion(
            max(
                abs(
                    float(
                        state["target_command"][
                            "station_requested_error_at_solve_m"
                        ]
                    )
                    - (
                        float(
                            state["target_command"]["sampled_position_world"][0]
                        )
                        - float(
                            state["target_command"][
                                "cartesian_measured_x_at_solve_m"
                            ]
                        )
                    )
                )
                for state in all_states
                if state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "absolute_frame_goal_persists_through_push_and_recovery": criterion(
            all(
                state["target_command"].get("admitted_position_world") is not None
                and np.allclose(
                    state["target_command"]["admitted_position_world"],
                    target,
                    atol=1.0e-12,
                )
                for state in all_states
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "same_target_id_through_push_and_release": criterion(
            all(state["target_command"].get("request_id") == 31 for state in all_states),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "normalized_handle_id_through_push_and_release": criterion(
            all(
                state["target_command"].get("handle_id") == f"frame:{frame}"
                for state in all_states
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "plant_release_clears_only_push": criterion(
            bool(release_state["external_load"].get("active", True)),
            False,
            "==",
            lambda a, b: a == b,
        ),
        "release_solve_has_no_stale_wrench": criterion(
            bool(release_state["external_load"].get("wbc_feedforward_active", True))
            or not np.allclose(
                release_state["external_load"].get(
                    "wbc_observed_force_world_n", [math.nan] * 3
                ),
                [0.0, 0.0, 0.0],
                atol=1.0e-12,
            ),
            False,
            "==",
            lambda a, b: a == b,
        ),
        "reopens_following_exactly_five_safe_recovery_ticks": criterion(
            [
                state["target_command"].get("recovery_safe_ticks")
                for state in suppressed_recovery[-5:]
            ]
            == [1, 2, 3, 4, 5]
            and bool(
                resumed_state["target_command"].get(
                    "command_descriptors_active", False
                )
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "resume_sample_continuous_from_measured_state_m": criterion(
            float(
                np.linalg.norm(
                    np.asarray(
                        resumed_state["target_command"]["sampled_position_world"],
                        dtype=np.float64,
                    )
                    - measured_before_resume
                )
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "resume_progress_has_no_completion_jump": criterion(
            abs(
                float(resumed_state["target_command"]["progress"])
                - frozen_progress
            ),
            1.0e-12,
            "<=",
            lambda a, b: a <= b,
        ),
        "resume_realization_is_measured_neutral": criterion(
            bool(
                resumed_state["target_command"].get(
                    "measured_neutral_realization", False
                )
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "target_directed_progress_after_plant_release_m": criterion(
            float(np.dot(endpoint - position_at_release, target_direction)),
            0.003,
            ">=",
            lambda a, b: a >= b,
        ),
        "target_holds_after_plant_release": criterion(
            post_release[-1]["target_command"].get("phase"),
            "holding",
            "==",
            lambda a, b: a == b,
        ),
        "first_holding_tick_within_sustained_recovery_horizon": criterion(
            first_holding_tick
            if first_holding_tick is not None
            else SUSTAINED_RECOVERY_MAX_TICKS + 1,
            SUSTAINED_RECOVERY_MAX_TICKS,
            "<=",
            lambda a, b: a <= b,
        ),
        "first_hold_qualification_persists_for_50_ticks": criterion(
            len(first_hold_dwell) == SUSTAINED_HOLD_DWELL_TICKS
            and all(
                state["target_command"].get("phase") == "holding"
                and float(
                    np.linalg.norm(frame_position(state, frame) - target)
                )
                <= COMMAND_HOLD_TOLERANCE_M
                for state in first_hold_dwell
            ),
            True,
            "==",
            lambda a, b: a == b,
        ),
        "hold_error_m": criterion(
            float(np.linalg.norm(endpoint - target)),
            COMMAND_HOLD_TOLERANCE_M,
            "<=",
            lambda a, b: a <= b,
        ),
    }
    safety = safety_report(all_states)
    return {
        "passed": settle_report["passed"]
        and safety["passed"]
        and all(check["passed"] for check in checks.values()),
        "settle": settle_report,
        "checks": checks,
        "safety": safety,
        "pre_push_frozen_progress": frozen_progress,
        "suppressed_recovery_ticks": len(suppressed_recovery),
        "first_holding_tick": first_holding_tick,
        "first_holding_time_s": (
            first_holding_tick * 0.020
            if first_holding_tick is not None
            else None
        ),
        "sustained_hold_dwell_ticks": len(first_hold_dwell),
        "position_at_release_world_m": position_at_release.tolist(),
        "hold_endpoint_world_m": endpoint.tolist(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=pathlib.Path, default=DEFAULT_MODEL)
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--json-out", type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = args.model.resolve()
    scenarios: dict[str, Any] = {}
    errors: dict[str, str] = {}
    for name, run in (
        ("torso_down_hold_return", lambda: torso_scenario(model)),
        ("left_knee_positive_x", lambda: knee_scenario(model, "left", 10)),
        ("right_knee_positive_x", lambda: knee_scenario(model, "right", 11)),
        ("left_ankle_positive_x", lambda: ankle_scenario(model, "left", 12)),
        ("right_ankle_positive_x", lambda: ankle_scenario(model, "right", 13)),
        ("sustained_push_release", lambda: sustained_push_release_scenario(model)),
        ("clamp_replacement_push_composition", lambda: admission_and_composition_scenario(model)),
    ):
        try:
            scenarios[name] = run()
        except Exception as error:  # Keep later fresh-plant scenarios observable.
            errors[name] = f"{type(error).__name__}: {error}"
            scenarios[name] = {"passed": False, "error": errors[name]}

    report = {
        "revision": "live-cartesian-target-acceptance-v2",
        "backend": "direct LiveUpkiePlant (same MuJoCo/Rust WBC path as worker)",
        "transport_scope": (
            "The direct deterministic harness measures Rust controller and local "
            "worker-step p99, but no WebSocket/network transport latency. Its "
            "explicit inactive external_load is the worker-side effect of "
            "plant_release; gateway correlation remains a separate protocol test."
        ),
        "model": str(model),
        "fixed_profile": {
            "settle_ticks": SETTLE_TICKS,
            "trajectory_ticks": TRAJECTORY_TICKS,
            "trajectory_duration_s": 3.0,
            "hold_ticks": HOLD_TICKS,
            "pre_push_target_ticks": PRE_PUSH_TARGET_TICKS,
            "simultaneous_push_target_ticks": PUSH_TICKS,
            "sustained_recovery_max_ticks": SUSTAINED_RECOVERY_MAX_TICKS,
            "sustained_hold_dwell_ticks": SUSTAINED_HOLD_DWELL_TICKS,
            "control_period_s": 0.020,
            "controller_step_p99_limit_us": 5_000.0,
            "direct_worker_step_p99_limit_us": 20_000.0,
        },
        "passed": not errors and all(
            scenario.get("passed", False) for scenario in scenarios.values()
        ),
        "scenarios": scenarios,
        "errors": errors,
    }
    output = json.dumps(report, indent=2 if args.pretty else None, sort_keys=True)
    print(output)
    if args.json_out is not None:
        args.json_out.write_text(output + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
