#!/usr/bin/env python3
"""R300 live 250/50 dynamic measured-contact consequence fixture."""

from __future__ import annotations

import argparse
import html
import json
import math
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution
from upkie_live_plant_worker import LiveUpkiePlant


REVISION = "upkie-live-dynamic-contact-transition-r300"
CONTROL_DT = 0.020
MAX_TICKS = 80
PUSH_START_TICK = 25
PUSH_TICKS = 10
FORCE_WORLD_N = (0.0, 8.0, 0.0)
BEHAVIOR_GATE_NAMES = (
    "controller_p99_under_5ms",
    "hard_residuals_under_1e8",
    "candidate_recovers_supported_upright",
    "candidate_recovers_without_boundary",
)
PROVENANCE = {
    "source": "evaluation_harness",
    "load_class": "declared_continuous_wrench",
    "force_frame": "world",
    "application_point_frame": "world",
}


def _summary(index: int, state: dict[str, Any]) -> dict[str, Any]:
    metrics = state["metrics"]
    simulator = state["simulator"]
    return {
        "index": index,
        "tick": int(state["tick"]),
        "time_s": float(simulator["time_s"]),
        "reset_epoch": int(state["reset_epoch"]),
        "numeric_reset": bool(state["numeric_reset"]),
        "automatic_reset_pending": state["automatic_reset_pending"],
        "root_position": [float(value) for value in state["root_position"]],
        "root_quaternion_wxyz": [
            float(value) for value in state["root_quaternion_wxyz"]
        ],
        "root_twist_world": [float(value) for value in state["root_twist_world"]],
        "center_of_mass_world": [
            float(value) for value in state["center_of_mass_world"]
        ],
        "joint_positions": [float(value) for value in state["joint_positions"]],
        "joint_velocities": [float(value) for value in state["joint_velocities"]],
        "actuator_effort_nm": [
            float(value) for value in state["actuator_effort_nm"]
        ],
        "generalized_acceleration": [
            float(value) for value in state["generalized_acceleration"]
        ],
        "observed": [int(value) for value in state["wbc_observed_contact_active"]],
        "observed_wheel_normal_force_n": [
            float(value) for value in state["wbc_observed_wheel_normal_force_n"]
        ],
        "physics_wheel_normal_force_n": [
            float(value) for value in state["physics_wheel_normal_force_n"]
        ],
        "wbc_predicted_normal_force_n": [
            float(value) for value in state["wbc_predicted_normal_force_n"]
        ],
        "wbc_observation_frame_index": int(
            state["wbc_observation"]["physics_frame_index"]
        ),
        "physics_contact_active": [
            int(value) for value in state["physics_contact_active"]
        ],
        "contact_window_valid": bool(simulator["contact_window_valid"]),
        "contact_window_frame_start": int(
            simulator["contact_window_frame_start"]
        ),
        "contact_window_frame_end": int(simulator["contact_window_frame_end"]),
        "contact_window_masks": [
            [int(value) for value in row]
            for row in simulator["contact_window_masks"]
        ],
        "contact_window_loss_masks": [
            [int(value) for value in row]
            for row in simulator["contact_window_loss_masks"]
        ],
        "contact_window_gain_masks": [
            [int(value) for value in row]
            for row in simulator["contact_window_gain_masks"]
        ],
        "contact_window_loss_mask": [
            int(value) for value in simulator["contact_window_loss_mask"]
        ],
        "contact_window_gain_mask": [
            int(value) for value in simulator["contact_window_gain_mask"]
        ],
        "wheel_normal_force_window_n": [
            [float(value) for value in row]
            for row in simulator["wheel_normal_force_window_n"]
        ],
        "debounced": [int(value) for value in state["wbc_debounced_contact_active"]],
        "hard": [int(value) for value in state["wbc_hard_contact_active"]],
        "hard_executable": [
            int(value) for value in state["wbc_hard_contact_executable"]
        ],
        "support_count": int(metrics["wbc_support_active_count"]),
        "wbc_status": str(metrics["wbc_status"]),
        "wbc_raw_status": str(metrics["wbc_raw_status"]),
        "wbc_raw_status_code": int(metrics["wbc_raw_status_code"]),
        "wbc_admitted": bool(metrics["wbc_admitted"]),
        "wbc_allocation_calls": int(metrics["wbc_allocation_calls"]),
        "wbc_allocated_bytes": int(metrics["wbc_allocated_bytes"]),
        "wbc_maximum_constraint_violation": float(
            metrics["wbc_maximum_constraint_violation"]
        ),
        "wbc_dynamics_residual": float(metrics["wbc_dynamics_residual"]),
        "wbc_contact_residual": float(metrics["wbc_contact_residual"]),
        "support_contingency_enabled": bool(
            metrics.get("wbc_support_contingency_enabled", False)
        ),
        "support_contingency_requested": bool(
            metrics.get("wbc_support_contingency_requested", False)
        ),
        "support_contingency_admitted": bool(
            metrics.get("wbc_support_contingency_admitted", False)
        ),
        "support_contingency_selected": bool(
            metrics.get("wbc_support_contingency_selected", False)
        ),
        "support_contingency_armed": bool(
            metrics.get("wbc_support_contingency_armed", False)
        ),
        "support_contingency_mode": int(
            metrics.get("wbc_support_contingency_mode", 0)
        ),
        "support_contingency_support_mask": int(
            metrics.get("wbc_support_contingency_support_mask", 3)
        ),
        "support_contingency_status": str(
            metrics.get("wbc_support_contingency_status", "unavailable")
        ),
        "support_contingency_status_code": int(
            metrics.get("wbc_support_contingency_status_code", -1)
        ),
        "support_contingency_maximum_constraint_violation": float(
            metrics.get(
                "wbc_support_contingency_maximum_constraint_violation", 0.0
            )
        ),
        "support_contingency_author_step_us": float(
            metrics.get("wbc_support_contingency_author_step_us", 0.0)
        ),
        "support_contingency_step_us": float(
            metrics.get("wbc_support_contingency_step_us", 0.0)
        ),
        "support_contingency_candidate_power_w": float(
            metrics.get("wbc_support_contingency_candidate_power_w", 0.0)
        ),
        "support_contingency_incremental_power_w": float(
            metrics.get("wbc_support_contingency_incremental_power_w", 0.0)
        ),
        "support_contingency_forecast_guard_passed": bool(
            metrics.get("wbc_support_contingency_forecast_guard_passed", False)
        ),
        "support_load_guard_enabled": bool(
            metrics.get("wbc_support_load_guard_enabled", False)
        ),
        "support_load_guard_active": bool(
            metrics.get("wbc_support_load_guard_active", False)
        ),
        "support_load_guard_authority": float(
            metrics.get("wbc_support_load_guard_authority", 0.0)
        ),
        "controller_step_us": float(metrics["controller_step_us"]),
        "worker_step_us": float(metrics["worker_step_us"]),
        "root_tilt_rad": float(metrics["root_tilt_rad"]),
        "root_height_m": float(metrics["root_height_m"]),
        "capture_error_m": float(metrics["capture_error_m"]),
        "capture_pressure": float(metrics["capture_pressure"]),
        "station_error_m": float(metrics["station_error_m"]),
        "station_authority": float(metrics["station_authority"]),
        "torque_utilization": float(metrics["torque_utilization"]),
        "fall_safe_risk": float(metrics["fall_safe_risk"]),
        "fallen": bool(metrics["fallen"]),
        "ground_contact_count": int(metrics["ground_contact_count"]),
        "total_ground_normal_force_n": float(metrics["total_ground_normal_force_n"]),
        "maximum_penetration_m": float(metrics["maximum_penetration_m"]),
        "maximum_abs_constraint_force": float(
            metrics["maximum_abs_constraint_force"]
        ),
        "maximum_abs_actuator_effort_nm": float(
            metrics["maximum_abs_actuator_effort_nm"]
        ),
        "kinetic_energy_j": float(simulator["kinetic_energy_j"]),
        "potential_energy_j": float(simulator["potential_energy_j"]),
        "warning_count": int(simulator["warning_count"]),
        "external_load_active": bool(state["external_load"]["active"]),
        "external_force_world": [
            float(value) for value in state["external_load"]["force_world"]
        ],
        "external_moment_world_nm": [
            float(value) for value in state["external_load"]["moment_world_nm"]
        ],
    }


def run_case(
    model_path: pathlib.Path,
    *,
    disturbed: bool,
    maximum_ticks: int = MAX_TICKS,
    controller_options: dict[str, Any] | None = None,
    controller_balance_mode: str = "capture",
    worker_options: dict[str, Any] | None = None,
) -> dict[str, Any]:
    worker = LiveUpkiePlant(
        model_path,
        controller_options=controller_options,
        controller_balance_mode=controller_balance_mode,
        **({} if worker_options is None else worker_options),
    )
    hello = worker.hello()
    base_body = worker.body_by_name["base"]
    initial_root = np.asarray(worker.data.qpos[:3], np.float64).copy()
    initial_com = np.asarray(worker.data.subtree_com[0], np.float64).copy()
    states: list[dict[str, Any]] = []
    for tick in range(maximum_ticks):
        active = disturbed and PUSH_START_TICK <= tick < PUSH_START_TICK + PUSH_TICKS
        request: dict[str, Any] = {"type": "step", "command_id": tick}
        if active:
            request["external_load"] = {
                "active": True,
                "body": "base",
                "force_world": list(FORCE_WORLD_N),
                "application_point_world": worker.data.xipos[base_body].tolist(),
                "provenance": PROVENANCE,
                "request_id": tick,
            }
        state = worker.step(request)
        if state.get("type") != "plant_state":
            raise RuntimeError(f"live worker rejected tick {tick}: {state}")
        states.append(_summary(tick, state))
        # Stop on the first reported boundary. Never execute the next request,
        # because that would consume the pending automatic reset.
        if state["numeric_reset"] or state["automatic_reset_pending"] is not None:
            break
    return {
        "disturbed": disturbed,
        "hello": {
            "physics_hz": int(hello["physics_hz"]),
            "control_hz": int(hello["control_hz"]),
            "physics_substeps_per_control": int(
                hello["physics_substeps_per_control"]
            ),
        },
        "initial_root_position": initial_root.tolist(),
        "initial_center_of_mass_world": initial_com.tolist(),
        "states": states,
    }


def _semantic(case: dict[str, Any]) -> dict[str, Any]:
    copy = json.loads(json.dumps(case))
    for state in copy["states"]:
        for timing_key in (
            "controller_step_us",
            "worker_step_us",
            "support_contingency_author_step_us",
            "support_contingency_step_us",
        ):
            state.pop(timing_key, None)
    return copy


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def supported_upright_recovery_tick(
    states: list[dict[str, Any]],
    *,
    start_tick: int = PUSH_START_TICK + PUSH_TICKS,
    dwell_ticks: int = 10,
) -> int | None:
    """First sustained wheel-supported, near-nominal recovery boundary."""
    consecutive = 0
    for state in states:
        recovered = (
            state["index"] >= start_tick
            and state["observed"] == [1, 1]
            and state["root_height_m"] >= 0.48
            and state["root_tilt_rad"] <= 0.20
            and state["automatic_reset_pending"] is None
        )
        consecutive = consecutive + 1 if recovered else 0
        if consecutive >= dwell_ticks:
            return state["index"] - dwell_ticks + 1
    return None


def maximum_body_ground_stall_ticks(states: list[dict[str, Any]]) -> int:
    """Longest low-body interval without either measured wheel contact."""
    longest = 0
    consecutive = 0
    for state in states:
        stalled = state["root_height_m"] < 0.40 and state["observed"] == [0, 0]
        consecutive = consecutive + 1 if stalled else 0
        longest = max(longest, consecutive)
    return longest


def summarize(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    root = np.asarray([state["root_position"] for state in states], np.float64)
    com = np.asarray([state["center_of_mass_world"] for state in states], np.float64)
    initial_root = np.asarray(case["initial_root_position"], np.float64)
    initial_com = np.asarray(case["initial_center_of_mass_world"], np.float64)
    observed = np.asarray([state["observed"] for state in states], np.uint8)
    hard = np.asarray([state["hard"] for state in states], np.uint8)
    controller_steps = np.asarray(
        [state["controller_step_us"] for state in states], np.float64
    )
    constraint_violations = np.asarray(
        [state["wbc_maximum_constraint_violation"] for state in states],
        np.float64,
    )
    controller_jitter = (
        np.abs(np.diff(controller_steps))
        if len(controller_steps) > 1
        else controller_steps
    )
    worker_steps = np.asarray(
        [state["worker_step_us"] for state in states], np.float64
    )
    worker_jitter = (
        np.abs(np.diff(worker_steps)) if len(worker_steps) > 1 else worker_steps
    )
    transitions = [
        {
            "tick": states[index]["index"],
            "observed": states[index]["observed"],
            "hard": states[index]["hard"],
        }
        for index in range(len(states))
        if index == 0
        or states[index]["observed"] != states[index - 1]["observed"]
        or states[index]["hard"] != states[index - 1]["hard"]
    ]
    return {
        "ticks": len(states),
        "duration_s": float(states[-1]["time_s"]),
        "terminal_pending": states[-1]["automatic_reset_pending"],
        "terminal_tick": states[-1]["index"]
        if states[-1]["automatic_reset_pending"] is not None
        else None,
        "numeric_reset": any(state["numeric_reset"] for state in states),
        "reset_epochs": sorted({state["reset_epoch"] for state in states}),
        "observed_patterns": sorted({"".join(str(value) for value in row) for row in observed}),
        "transitions": transitions,
        "first_non_double_tick": next(
            (state["index"] for state in states if state["observed"] != [1, 1]),
            None,
        ),
        "first_flight_tick": next(
            (state["index"] for state in states if state["observed"] == [0, 0]),
            None,
        ),
        "root_position_rms_m": _rms(root - initial_root),
        "com_position_rms_m": _rms(com - initial_com),
        "root_lateral_rms_m": _rms(root[:, 1] - initial_root[1]),
        "maximum_root_lateral_displacement_m": float(
            np.max(np.abs(root[:, 1] - initial_root[1]))
        ),
        "maximum_root_tilt_rad": float(
            max(state["root_tilt_rad"] for state in states)
        ),
        "minimum_root_height_m": float(
            min(state["root_height_m"] for state in states)
        ),
        "maximum_capture_pressure": float(
            max(state["capture_pressure"] for state in states)
        ),
        "minimum_station_authority": float(
            min(state["station_authority"] for state in states)
        ),
        "maximum_torque_utilization": float(
            max(state["torque_utilization"] for state in states)
        ),
        "maximum_constraint_violation": float(
            max(state["wbc_maximum_constraint_violation"] for state in states)
        ),
        "maximum_dynamics_residual": float(
            max(state["wbc_dynamics_residual"] for state in states)
        ),
        "maximum_contact_residual": float(
            max(state["wbc_contact_residual"] for state in states)
        ),
        "maximum_controller_step_tick": int(np.argmax(controller_steps)),
        "maximum_constraint_violation_tick": int(
            np.argmax(constraint_violations)
        ),
        "wbc_status_counts": {
            status: sum(state["wbc_status"] == status for state in states)
            for status in sorted({state["wbc_status"] for state in states})
        },
        "nonadmitted_ticks": [
            state["index"] for state in states if not state["wbc_admitted"]
        ],
        "max_iterations_ticks": [
            state["index"]
            for state in states
            if state["wbc_status"] == "MaxIterations"
        ],
        "controller_step_us": distribution(controller_steps),
        "controller_adjacent_jitter_us": distribution(controller_jitter),
        "controller_over_5ms_ticks": int(np.sum(controller_steps > 5_000.0)),
        "worker_step_us": distribution(worker_steps),
        "worker_adjacent_jitter_us": distribution(worker_jitter),
        "worker_over_20ms_ticks": int(np.sum(worker_steps > 20_000.0)),
        "ground_normal_force_n": distribution(
            np.asarray([state["total_ground_normal_force_n"] for state in states])
        ),
        "sampled_ground_normal_impulse_ns": float(
            sum(state["total_ground_normal_force_n"] for state in states)
            * CONTROL_DT
        ),
        "allocation_calls": int(sum(state["wbc_allocation_calls"] for state in states)),
        "allocated_bytes": int(sum(state["wbc_allocated_bytes"] for state in states)),
        "hard_subset_raw": bool(np.all(hard <= observed)),
        "warning_count": int(sum(state["warning_count"] for state in states)),
        "supported_upright_recovery_tick": supported_upright_recovery_tick(states),
        "maximum_body_ground_stall_ticks": maximum_body_ground_stall_ticks(states),
    }


def compare(control: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    count = min(len(control["states"]), len(candidate["states"]))
    control_states = control["states"][:count]
    candidate_states = candidate["states"][:count]
    control_root = np.asarray(
        [state["root_position"] for state in control_states], np.float64
    )
    candidate_root = np.asarray(
        [state["root_position"] for state in candidate_states], np.float64
    )
    control_com = np.asarray(
        [state["center_of_mass_world"] for state in control_states], np.float64
    )
    candidate_com = np.asarray(
        [state["center_of_mass_world"] for state in candidate_states], np.float64
    )
    control_effort = np.asarray(
        [state["actuator_effort_nm"] for state in control_states], np.float64
    )
    candidate_effort = np.asarray(
        [state["actuator_effort_nm"] for state in candidate_states], np.float64
    )
    return {
        "paired_ticks": count,
        "root_trace_delta_rms_m": _rms(candidate_root - control_root),
        "com_trace_delta_rms_m": _rms(candidate_com - control_com),
        "actuator_effort_delta_rms_nm": _rms(candidate_effort - control_effort),
        "contact_mask_mismatch_ticks": sum(
            left["observed"] != right["observed"]
            for left, right in zip(control_states, candidate_states, strict=True)
        ),
    }


def evaluate(
    control: dict[str, Any],
    candidate: dict[str, Any],
    replay: dict[str, Any],
) -> dict[str, bool]:
    control_summary = summarize(control)
    candidate_summary = summarize(candidate)
    states = candidate["states"]
    finite_fields = (
        "wbc_maximum_constraint_violation",
        "wbc_dynamics_residual",
        "wbc_contact_residual",
        "controller_step_us",
        "worker_step_us",
        "total_ground_normal_force_n",
        "root_tilt_rad",
    )
    return {
        "live_rate_split_is_250_50": candidate["hello"]
        == {"physics_hz": 250, "control_hz": 50, "physics_substeps_per_control": 5},
        "control_stays_double_support": control_summary["observed_patterns"] == ["11"],
        "control_has_no_boundary": control_summary["terminal_pending"] is None
        and not control_summary["numeric_reset"],
        "candidate_measures_all_contact_modes": candidate_summary["observed_patterns"]
        == ["00", "01", "10", "11"],
        "candidate_reaches_single_then_flight": candidate_summary[
            "first_non_double_tick"
        ]
        is not None
        and candidate_summary["first_flight_tick"] is not None
        and candidate_summary["first_non_double_tick"]
        < candidate_summary["first_flight_tick"],
        "candidate_stops_before_automatic_reset": candidate_summary[
            "terminal_pending"
        ]
        == "fall"
        and candidate_summary["reset_epochs"] == [0],
        "no_numeric_reset_or_warning": not candidate_summary["numeric_reset"]
        and candidate_summary["warning_count"] == 0,
        "hard_rows_always_subset_measured_contact": candidate_summary[
            "hard_subset_raw"
        ],
        "wbc_boundary_consumes_prior_window": _causal_window_contract(states),
        "exact_contact_loss_removes_hard_row": all(
            all(h <= r for h, r in zip(state["hard"], state["observed"], strict=True))
            for state in states
        ),
        "external_wrench_is_bounded_and_ten_ticks": sum(
            state["external_load_active"] for state in states
        )
        == PUSH_TICKS
        and all(
            np.linalg.norm(state["external_force_world"]) <= 8.0 + 1.0e-12
            for state in states
        ),
        "all_wbc_outputs_finite": all(
            math.isfinite(float(state[field])) for state in states for field in finite_fields
        ),
        "rust_hot_loop_zero_allocations": candidate_summary["allocation_calls"] == 0
        and candidate_summary["allocated_bytes"] == 0,
        "max_iterations_are_nonadmitted": all(
            state["wbc_status"] != "MaxIterations" or not state["wbc_admitted"]
            for state in states
        ),
        "executable_hard_rows_require_admission": all(
            all(
                executable <= hard
                and (state["wbc_admitted"] or executable == 0)
                for executable, hard in zip(
                    state["hard_executable"], state["hard"], strict=True
                )
            )
            for state in states
        ),
        "controller_p99_under_5ms": candidate_summary["controller_step_us"]["p99"]
        < 5_000.0,
        "hard_residuals_under_1e8": candidate_summary[
            "maximum_constraint_violation"
        ]
        <= 1.0e-8
        and candidate_summary["maximum_dynamics_residual"] <= 1.0e-8
        and candidate_summary["maximum_contact_residual"] <= 1.0e-8,
        "candidate_recovers_supported_upright": candidate_summary[
            "terminal_pending"
        ]
        is None
        and candidate_summary["supported_upright_recovery_tick"] is not None
        and candidate_summary["maximum_body_ground_stall_ticks"] == 0,
        # Retain the original compact gate name for callers of the R300
        # evaluator; the supported-upright gate above is the stricter witness.
        "candidate_recovers_without_boundary": candidate_summary[
            "terminal_pending"
        ]
        is None,
        "worker_p99_under_20ms": candidate_summary["worker_step_us"]["p99"]
        < 20_000.0,
        "semantic_replay_exact": _semantic(candidate) == _semantic(replay),
        "disturbance_has_visible_consequence": candidate_summary[
            "maximum_root_lateral_displacement_m"
        ]
        > control_summary["maximum_root_lateral_displacement_m"] + 0.05,
    }


def _causal_window_contract(states: list[dict[str, Any]]) -> bool:
    """Check frame-causal WBC input against actual post-step observations."""
    if not states:
        return False
    for index, state in enumerate(states):
        if not state["contact_window_valid"]:
            return False
        masks = state["contact_window_masks"]
        if len(masks) != 5 or any(len(row) != 2 for row in masks):
            return False
        if state["physics_contact_active"] != masks[-1]:
            return False
        if state["contact_window_frame_end"] - state["contact_window_frame_start"] != 4:
            return False
        if state["wbc_observation_frame_index"] == 0:
            if index != 0 or state["observed"] != [1, 1]:
                return False
        else:
            if index == 0:
                return False
            previous = states[index - 1]
            if state["wbc_observation_frame_index"] != previous[
                "contact_window_frame_end"
            ]:
                return False
            if state["observed"] != previous["contact_window_masks"][-1]:
                return False
            if state["contact_window_frame_start"] != previous[
                "contact_window_frame_end"
            ] + 1:
                return False
        for row, loss, gain in zip(
            masks,
            state["contact_window_loss_masks"],
            state["contact_window_gain_masks"],
            strict=True,
        ):
            if any(value not in (0, 1) for value in (*row, *loss, *gain)):
                return False
        expected_loss = [
            int(any(row[column] for row in state["contact_window_loss_masks"]))
            for column in range(2)
        ]
        expected_gain = [
            int(any(row[column] for row in state["contact_window_gain_masks"]))
            for column in range(2)
        ]
        if state["contact_window_loss_mask"] != expected_loss:
            return False
        if state["contact_window_gain_mask"] != expected_gain:
            return False
    return True


def fixture_passed(gates: dict[str, bool]) -> bool:
    return all(
        value for name, value in gates.items() if name not in BEHAVIOR_GATE_NAMES
    )


def behavior_passed(gates: dict[str, bool]) -> bool:
    return fixture_passed(gates) and all(gates[name] for name in BEHAVIOR_GATE_NAMES)


def render_report(
    control: dict[str, Any],
    candidate: dict[str, Any],
    gates: dict[str, bool],
) -> str:
    c0 = summarize(control)
    c1 = summarize(candidate)
    paired = compare(control, candidate)
    lines = [
        "# Upkie live dynamic measured-contact transition · R300",
        "",
        f"> Fixture {'PASS' if fixture_passed(gates) else 'FAIL'} · controller behavior {'PASS' if behavior_passed(gates) else 'REJECTED'} · actual 250 Hz MuJoCo integration / 50 Hz Rust WBC · authority closed.",
        "",
        "The control and candidate use the same live worker, Upkie reference capture composition, persistent Rust WBC, model, and initial state. Each 50 Hz solve consumes the final completed sample of the prior five-frame 250 Hz contact window; the fixture checks the public frame indices and loss/gain edges. The candidate alone receives an 8 N lateral world-frame wrench at the measured base COM for 200 ms. There is no learned policy. The run stops on the first fall boundary before the worker can consume its pending automatic reset.",
        "",
        "| metric | zero-wrench control | 8 N lateral candidate |",
        "|---|---:|---:|",
        f"| duration s | {c0['duration_s']:.3f} | {c1['duration_s']:.3f} |",
        f"| contact patterns | {', '.join(c0['observed_patterns'])} | {', '.join(c1['observed_patterns'])} |",
        f"| first non-double / flight tick | — / — | {c1['first_non_double_tick']} / {c1['first_flight_tick']} |",
        f"| terminal boundary | {c0['terminal_pending'] or '—'} | {c1['terminal_pending'] or '—'} |",
        f"| supported-upright recovery tick (10-tick dwell) | {c0['supported_upright_recovery_tick'] if c0['supported_upright_recovery_tick'] is not None else '—'} | {c1['supported_upright_recovery_tick'] if c1['supported_upright_recovery_tick'] is not None else '—'} |",
        f"| longest low-body / no-wheel-contact stall ticks | {c0['maximum_body_ground_stall_ticks']} | {c1['maximum_body_ground_stall_ticks']} |",
        f"| root position RMS m | {c0['root_position_rms_m']:.5f} | {c1['root_position_rms_m']:.5f} |",
        f"| CoM position RMS m | {c0['com_position_rms_m']:.5f} | {c1['com_position_rms_m']:.5f} |",
        f"| max lateral displacement m | {c0['maximum_root_lateral_displacement_m']:.5f} | {c1['maximum_root_lateral_displacement_m']:.5f} |",
        f"| paired root / CoM trace delta RMS m | — | {paired['root_trace_delta_rms_m']:.5f} / {paired['com_trace_delta_rms_m']:.5f} |",
        f"| paired actuator-effort delta RMS N·m | — | {paired['actuator_effort_delta_rms_nm']:.5f} |",
        f"| max tilt rad | {c0['maximum_root_tilt_rad']:.5f} | {c1['maximum_root_tilt_rad']:.5f} |",
        f"| min height m | {c0['minimum_root_height_m']:.5f} | {c1['minimum_root_height_m']:.5f} |",
        f"| controller p50 / p99 / max µs | {c0['controller_step_us']['p50']:.1f} / {c0['controller_step_us']['p99']:.1f} / {c0['controller_step_us']['maximum']:.1f} | {c1['controller_step_us']['p50']:.1f} / {c1['controller_step_us']['p99']:.1f} / {c1['controller_step_us']['maximum']:.1f} |",
        f"| controller adjacent jitter p50 / p99 / max µs | {c0['controller_adjacent_jitter_us']['p50']:.1f} / {c0['controller_adjacent_jitter_us']['p99']:.1f} / {c0['controller_adjacent_jitter_us']['maximum']:.1f} | {c1['controller_adjacent_jitter_us']['p50']:.1f} / {c1['controller_adjacent_jitter_us']['p99']:.1f} / {c1['controller_adjacent_jitter_us']['maximum']:.1f} |",
        f"| controller calls >5 ms | {c0['controller_over_5ms_ticks']} | {c1['controller_over_5ms_ticks']} |",
        f"| worker p50 / p99 / max µs | {c0['worker_step_us']['p50']:.1f} / {c0['worker_step_us']['p99']:.1f} / {c0['worker_step_us']['maximum']:.1f} | {c1['worker_step_us']['p50']:.1f} / {c1['worker_step_us']['p99']:.1f} / {c1['worker_step_us']['maximum']:.1f} |",
        f"| worker adjacent jitter p50 / p99 / max µs | {c0['worker_adjacent_jitter_us']['p50']:.1f} / {c0['worker_adjacent_jitter_us']['p99']:.1f} / {c0['worker_adjacent_jitter_us']['maximum']:.1f} | {c1['worker_adjacent_jitter_us']['p50']:.1f} / {c1['worker_adjacent_jitter_us']['p99']:.1f} / {c1['worker_adjacent_jitter_us']['maximum']:.1f} |",
        f"| worker calls >20 ms | {c0['worker_over_20ms_ticks']} | {c1['worker_over_20ms_ticks']} |",
        f"| max constraint / dynamics / contact residual | {c0['maximum_constraint_violation']:.3e} / {c0['maximum_dynamics_residual']:.3e} / {c0['maximum_contact_residual']:.3e} | {c1['maximum_constraint_violation']:.3e} / {c1['maximum_dynamics_residual']:.3e} / {c1['maximum_contact_residual']:.3e} |",
        f"| MaxIterations ticks | {c0['max_iterations_ticks'] or '—'} | {c1['max_iterations_ticks'] or '—'} |",
        f"| max controller / constraint tick | {c0['maximum_controller_step_tick']} / {c0['maximum_constraint_violation_tick']} | {c1['maximum_controller_step_tick']} / {c1['maximum_constraint_violation_tick']} |",
        f"| sampled ground impulse N·s | {c0['sampled_ground_normal_impulse_ns']:.3f} | {c1['sampled_ground_normal_impulse_ns']:.3f} |",
        f"| Rust allocation calls / bytes | {c0['allocation_calls']} / {c0['allocated_bytes']} | {c1['allocation_calls']} / {c1['allocated_bytes']} |",
        "",
        "## Measured support transitions",
        "",
        "| tick | observed | hard |",
        "|---:|:---:|:---:|",
    ]
    lines.extend(
        f"| {row['tick']} | `{row['observed'][0]}{row['observed'][1]}` | `{row['hard'][0]}{row['hard'][1]}` |"
        for row in c1["transitions"]
    )
    lines.extend(["", "## Fixture gates", "", "| gate | result |", "|---|:---:|"])
    lines.extend(
        f"| {name} | {'PASS' if value else 'FAIL'} |"
        for name, value in gates.items()
        if name not in BEHAVIOR_GATE_NAMES
    )
    lines.extend(["", "## Controller behavior gates", "", "| gate | result |", "|---|:---:|"])
    lines.extend(
        f"| {name} | {'PASS' if gates[name] else 'FAIL'} |"
        for name in BEHAVIOR_GATE_NAMES
    )
    lines.extend(
        [
            "",
            "This is a discriminating consequence fixture, not a recovery pass. It proves that actual integrated contact loss crosses an explicit prior-window causality boundary and reaches the streamed authority layers without stale hard rows or reset masking. The controller still falls; the next behavior candidate must improve this same frozen trace without weakening any transport, residual, timing, or allocation gate. Rust allocation counters are in-process and exact; process RSS, Python GC, and hardware thermal/power telemetry are deliberately not claimed by this fixture and remain separate benchmark work.",
            "",
        ]
    )
    return "\n".join(lines)


def render_html(report: str, passed: bool) -> str:
    return "\n".join(
        (
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>Bonesaw Upkie live dynamic contact R300</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px}.pass{color:#86efac}.fail{color:#fca5a5}</style>",
            f"<h1 class='{'pass' if passed else 'fail'}'>Upkie live dynamic measured-contact transition · R300</h1>",
            f"<pre>{html.escape(report)}</pre>",
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_LIVE_DYNAMIC_CONTACT_R300.html")
    parser.add_argument(
        "--require-behavior",
        action="store_true",
        help="return failure unless the recovery/residual/deadline behavior gates also pass",
    )
    args = parser.parse_args()
    model = pathlib.Path(args.model)
    candidate = run_case(model, disturbed=True)
    control = run_case(model, disturbed=False, maximum_ticks=len(candidate["states"]))
    replay = run_case(model, disturbed=True)
    gates = evaluate(control, candidate, replay)
    benchmark_passed = fixture_passed(gates)
    controller_behavior_passed = behavior_passed(gates)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "passed": benchmark_passed,
        "fixture_passed": benchmark_passed,
        "controller_behavior_passed": controller_behavior_passed,
        "fixture": {
            "force_world_n": FORCE_WORLD_N,
            "push_start_tick": PUSH_START_TICK,
            "push_ticks": PUSH_TICKS,
            "maximum_ticks": MAX_TICKS,
            "learned_policy_steps": 0,
        },
        "gates": gates,
        "control_summary": summarize(control),
        "candidate_summary": summarize(candidate),
        "paired_comparison": compare(control, candidate),
        "control": control,
        "candidate": candidate,
        "candidate_replay": replay,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-live-dynamic-contact-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    report = render_report(control, candidate, gates)
    (output / "UPKIE_LIVE_DYNAMIC_CONTACT_R300.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report, benchmark_passed))
    print(
        json.dumps(
            {
                "fixture_passed": benchmark_passed,
                "controller_behavior_passed": controller_behavior_passed,
                "gates": gates,
            },
            sort_keys=True,
        )
    )
    return 0 if benchmark_passed and (
        not args.require_behavior or controller_behavior_passed
    ) else 1


if __name__ == "__main__":
    sys.exit(main())
