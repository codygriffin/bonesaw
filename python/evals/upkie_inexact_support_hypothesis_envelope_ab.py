#!/usr/bin/env python3
"""R195 robust terminal chooser over explicit support-mode hypotheses."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_disturbance_envelope import semantic_trace_equal
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_hold_improvement_gate_ab import consequence
from upkie_inexact_terminal_chooser_ab import (
    ROOT_IMPACT_PLANE_M,
    model_limits,
    terminal_contract,
)
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-inexact-support-hypothesis-envelope-ab-r195"
DURATION_S = 6.0
SUPPORT_HYPOTHESES = ("none", "left", "right", "double")
PRESSURE_NAMES = (
    "impact_speed_pressure",
    "tilt_pressure",
    "angular_rate_pressure",
    "joint_position_pressure",
    "joint_velocity_pressure",
    "actuator_effort_pressure",
    "admission_pressure",
)


def profiles() -> dict[str, dict[str, Any]]:
    terminal = {"inexact_hold_ticks": 1, "inexact_terminal_chooser": True}
    envelope = {
        **terminal,
        "inexact_terminal_support_hypothesis_envelope": True,
    }
    return {
        "exact_control": {"profile": {}},
        "exact_envelope_config": {"profile": {}, **envelope},
        "drop5_r191_single_hypothesis": {"profile": DROP5, **terminal},
        "drop10_r191_single_hypothesis": {"profile": DROP10, **terminal},
        "drop5_r195_envelope": {"profile": DROP5, **envelope},
        "drop10_r195_envelope": {"profile": DROP10, **envelope},
        "drop5_matched_r195_envelope": {"profile": DROP5_MATCHED, **envelope},
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_SUPPORT_HYPOTHESIS_ENVELOPE_AB_R195.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def hypothesis_audit(
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, Any]:
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    names = tuple(balance.terminal_impact_diagnostic_names)
    online_hypotheses = np.asarray(
        trace["inexact_observation_terminal_hypothesis_diagnostics"]
    )
    online_envelopes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_envelopes"]
    )
    online_selection = np.asarray(
        trace["inexact_observation_terminal_selector_selection_diagnostics"]
    )
    diagnostics = np.empty((3, 17), np.float64)
    root_acceleration = np.empty((3, 2), np.float64)
    joint_acceleration = np.empty((3, 6), np.float64)
    hypothesis_selection = np.empty(6, np.float64)
    envelopes = np.empty((3, 17), np.float64)
    selection = np.empty(6, np.float64)
    maximum_hypothesis_error = 0.0
    maximum_envelope_error = 0.0
    maximum_selection_error = 0.0
    disagreement_ticks = 0
    scoring_ns = np.empty(len(ticks), np.uint64)
    aggregation_ns = np.empty(len(ticks), np.uint64)
    allocation_calls = np.empty(len(ticks), np.uint64)
    allocated_bytes = np.empty(len(ticks), np.uint64)
    for cursor, tick in enumerate(ticks):
        reaudited = np.empty((3, 4, 17), np.float64)
        scoring_ns[cursor] = 0
        allocation_calls[cursor] = 0
        allocated_bytes[cursor] = 0
        for support_mask in range(4):
            acceleration = np.asarray(
                trace["inexact_observation_terminal_hypothesis_acceleration"]
            )[tick, support_mask]
            root_acceleration[:] = acceleration[:, :2]
            joint_acceleration[:] = acceleration[:, 6:]
            timing = balance.score_terminal_impact_candidates(
                np.asarray(trace["inexact_observation_terminal_selector_state"])[tick],
                np.asarray(trace["q"])[tick],
                np.asarray(trace["v"])[tick],
                lower,
                upper,
                velocity_limit,
                np.asarray(
                    trace["inexact_observation_terminal_hypothesis_available"]
                )[tick, support_mask],
                root_acceleration,
                joint_acceleration,
                np.asarray(
                    trace[
                        "inexact_observation_terminal_hypothesis_effort_utilization"
                    ]
                )[tick],
                0,
                0.0,
                0.01,
                diagnostics,
                hypothesis_selection,
            )
            reaudited[:, support_mask] = diagnostics
            scoring_ns[cursor] += int(timing[0])
            allocation_calls[cursor] += int(timing[1])
            allocated_bytes[cursor] += int(timing[2])
        timing = balance.select_terminal_impact_hypothesis_envelopes(
            reaudited,
            0,
            0.0,
            0.01,
            envelopes,
            selection,
        )
        aggregation_ns[cursor] = int(timing[0])
        allocation_calls[cursor] += int(timing[1])
        allocated_bytes[cursor] += int(timing[2])
        hypothesis_error = float(
            np.max(np.abs(reaudited - online_hypotheses[tick]))
        )
        envelope_error = float(
            np.max(np.abs(envelopes - online_envelopes[tick]))
        )
        selection_error = float(
            np.max(np.abs(selection - online_selection[tick]))
        )
        maximum_hypothesis_error = max(maximum_hypothesis_error, hypothesis_error)
        maximum_envelope_error = max(maximum_envelope_error, envelope_error)
        maximum_selection_error = max(maximum_selection_error, selection_error)
        disagreement_ticks += int(
            hypothesis_error != 0.0
            or envelope_error != 0.0
            or selection_error != 0.0
        )
    available = np.asarray(
        trace["inexact_observation_terminal_hypothesis_available"]
    )[ticks]
    query_allocations = np.asarray(
        trace["inexact_observation_terminal_hypothesis_query_allocation_calls"]
    )[ticks]
    query_bytes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_query_allocated_bytes"]
    )[ticks]
    aggregate_allocations = np.asarray(
        trace["inexact_observation_terminal_hypothesis_aggregate_allocation_calls"]
    )[ticks]
    aggregate_bytes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_aggregate_allocated_bytes"]
    )[ticks]
    return {
        "query_count": int(len(ticks)),
        "support_hypotheses": SUPPORT_HYPOTHESES,
        "baseline_available_for_every_hypothesis": bool(
            len(ticks) and np.all(available[:, :, 0] != 0)
        ),
        "candidate_hypothesis_availability_counts": np.sum(
            available != 0, axis=0
        ).tolist(),
        "online_reaudit_disagreement_ticks": disagreement_ticks,
        "maximum_hypothesis_diagnostic_error": maximum_hypothesis_error,
        "maximum_envelope_diagnostic_error": maximum_envelope_error,
        "maximum_selection_error": maximum_selection_error,
        "zero_allocation": bool(
            np.all(allocation_calls == 0)
            and np.all(allocated_bytes == 0)
            and np.all(query_allocations == 0)
            and np.all(query_bytes == 0)
            and np.all(aggregate_allocations == 0)
            and np.all(aggregate_bytes == 0)
        ),
        "fixed_effort_query_ns": timing_distribution(
            np.asarray(
                trace["inexact_observation_terminal_hypothesis_query_step_ns"]
            )[ticks]
        ),
        "independent_scoring_ns": timing_distribution(scoring_ns),
        "aggregation_ns": timing_distribution(aggregation_ns),
    }


def timing_distribution(values: np.ndarray) -> dict[str, float]:
    """Keep no-query diagnostics defined without inventing evidence."""
    if len(values):
        return distribution(values)
    return {
        "minimum": 0.0,
        "mean": 0.0,
        "stddev": 0.0,
        "mad": 0.0,
        "p50": 0.0,
        "p95": 0.0,
        "p99": 0.0,
        "p99_9": 0.0,
        "maximum": 0.0,
    }


def support_hypothesis_attempt_contract(trace: dict[str, Any]) -> dict[str, Any]:
    """Audit attempts separately from selections that have a valid envelope."""
    unavailable = np.asarray(trace["contact_observation_available"]) == 0
    attempted = (
        np.asarray(
            trace["inexact_observation_terminal_hypothesis_query_step_ns"]
        )
        != 0
    )
    queried = (
        np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    )
    available = np.asarray(
        trace["inexact_observation_terminal_hypothesis_available"]
    )
    baseline_available = np.all(available[:, :, 0] != 0, axis=1)
    expected_queried = attempted & baseline_available
    invalid_attempt = attempted & ~baseline_available
    authority = np.asarray(trace["contact_program_authority_selection"])
    executable = (
        np.asarray(trace["contact_program_authority_executable"]) != 0
    )
    torque = np.asarray(trace["torque"])
    invalid_ticks = np.flatnonzero(invalid_attempt)
    return {
        "unavailable_ticks": int(np.sum(unavailable)),
        "attempted_ticks": int(np.sum(attempted)),
        "queried_ticks": int(np.sum(queried)),
        "invalid_baseline_attempt_ticks": int(len(invalid_ticks)),
        "attempted_every_unavailable_tick": bool(
            np.array_equal(attempted, unavailable)
        ),
        "queried_exactly_when_baseline_envelope_is_valid": bool(
            np.array_equal(queried, expected_queried)
        ),
        "invalid_baseline_fails_closed": bool(
            np.all(authority[invalid_ticks] == 0)
            and np.all(~executable[invalid_ticks])
            and np.array_equal(
                torque[invalid_ticks],
                np.zeros_like(torque[invalid_ticks]),
            )
        ),
        "baseline_support_availability_counts": np.sum(
            available[attempted, :, 0] != 0,
            axis=0,
        ).tolist(),
    }


def first_hypothesis_attempt_prefix_equal(
    left: dict[str, Any], right: dict[str, Any]
) -> bool:
    """Reject any future burst-duration oracle at the first attempted loss."""
    left_attempts = np.flatnonzero(
        np.asarray(
            left["inexact_observation_terminal_hypothesis_query_step_ns"]
        )
        != 0
    )
    right_attempts = np.flatnonzero(
        np.asarray(
            right["inexact_observation_terminal_hypothesis_query_step_ns"]
        )
        != 0
    )
    if (
        not len(left_attempts)
        or not len(right_attempts)
        or left_attempts[0] != right_attempts[0]
    ):
        return False
    tick = int(left_attempts[0])
    fields = (
        "time_s",
        "root_position",
        "root_twist",
        "rotation_vector",
        "q",
        "v",
        "torque",
        "inexact_observation_terminal_hypothesis_acceleration",
        "inexact_observation_terminal_hypothesis_available",
        "inexact_observation_terminal_hypothesis_diagnostics",
        "inexact_observation_terminal_hypothesis_envelopes",
        "inexact_observation_terminal_selector_action",
    )
    return all(
        np.array_equal(
            np.asarray(left[field])[: tick + 1],
            np.asarray(right[field])[: tick + 1],
        )
        for field in fields
    )


def realization_audit(
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, Any]:
    """Compare each selected envelope with the following plant interval."""
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    diagnostic_names = tuple(balance.terminal_impact_diagnostic_names)
    diagnostic_index = {
        name: index for index, name in enumerate(diagnostic_names)
    }
    pressure_indices = np.asarray(
        [diagnostic_index[name] for name in PRESSURE_NAMES], np.int64
    )
    actions = np.asarray(
        trace["inexact_observation_terminal_selector_action"], np.uint8
    )[ticks]
    physical_contact = np.asarray(trace["physical_contact_active"], np.uint8)[ticks]
    support_masks = physical_contact[:, 0] + 2 * physical_contact[:, 1]
    root_twist = np.asarray(trace["root_twist"], np.float64)
    post_root_twist = np.asarray(trace["post_root_twist"], np.float64)
    joint_velocity = np.asarray(trace["v"], np.float64)
    post_joint_velocity = np.asarray(trace["post_v"], np.float64)
    hypothesis_acceleration = np.asarray(
        trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
    )
    hypothesis_available = np.asarray(
        trace["inexact_observation_terminal_hypothesis_available"], np.uint8
    )
    hypothesis_diagnostics = np.asarray(
        trace["inexact_observation_terminal_hypothesis_diagnostics"], np.float64
    )
    envelopes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_envelopes"], np.float64
    )
    effort = np.asarray(
        trace["inexact_observation_terminal_hypothesis_effort_utilization"],
        np.float64,
    )
    state = np.asarray(
        trace["inexact_observation_terminal_selector_state"], np.float64
    )
    q = np.asarray(trace["q"], np.float64)
    diagnostics = np.empty((3, len(diagnostic_names)), np.float64)
    selection = np.empty(6, np.float64)
    candidate_root = np.empty((3, 2), np.float64)
    candidate_joint = np.empty((3, 6), np.float64)
    candidate_available = np.empty(3, np.uint8)
    envelope_excess = np.empty((len(ticks), len(PRESSURE_NAMES)), np.float64)
    physical_hypothesis_error = np.empty_like(envelope_excess)
    score_ns = np.empty(len(ticks), np.uint64)
    allocation_calls = np.empty(len(ticks), np.uint64)
    allocated_bytes = np.empty(len(ticks), np.uint64)
    for cursor, (tick, action, support_mask) in enumerate(
        zip(ticks, actions, support_masks, strict=True)
    ):
        action = int(action)
        support_mask = int(support_mask)
        candidate_root[:] = hypothesis_acceleration[tick, support_mask, :, :2]
        candidate_joint[:] = hypothesis_acceleration[tick, support_mask, :, 6:]
        candidate_available[:] = hypothesis_available[tick, support_mask]
        candidate_root[action] = (
            post_root_twist[tick, :2] - root_twist[tick, :2]
        ) / CONTROL_DT
        candidate_joint[action] = (
            post_joint_velocity[tick] - joint_velocity[tick]
        ) / CONTROL_DT
        timing = balance.score_terminal_impact_candidates(
            state[tick],
            q[tick],
            joint_velocity[tick],
            lower,
            upper,
            velocity_limit,
            candidate_available,
            candidate_root,
            candidate_joint,
            effort[tick],
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )
        score_ns[cursor], allocation_calls[cursor], allocated_bytes[cursor] = timing
        realized_pressure = diagnostics[action, pressure_indices]
        physical_pressure = hypothesis_diagnostics[
            tick, action, support_mask, pressure_indices
        ]
        envelope_pressure = envelopes[tick, action, pressure_indices]
        physical_hypothesis_error[cursor] = realized_pressure - physical_pressure
        envelope_excess[cursor] = np.maximum(
            realized_pressure - envelope_pressure,
            0.0,
        )
    covered = np.all(envelope_excess <= 1.0e-12, axis=1)
    return {
        "sample_count": int(len(ticks)),
        "selected_action_counts": {
            str(action): int(np.sum(actions == action)) for action in range(3)
        },
        "physical_support_mask_counts": {
            str(mask): int(np.sum(support_masks == mask)) for mask in range(4)
        },
        "covered_samples": int(np.sum(covered)),
        "componentwise_coverage": float(np.mean(covered)) if len(covered) else 0.0,
        "maximum_envelope_exceedance": (
            float(np.max(envelope_excess)) if len(envelope_excess) else math.inf
        ),
        "maximum_envelope_exceedance_by_pressure": (
            np.max(envelope_excess, axis=0).tolist()
            if len(envelope_excess)
            else [math.inf] * len(PRESSURE_NAMES)
        ),
        "physical_hypothesis_pressure_error_norm": (
            distribution(np.linalg.norm(physical_hypothesis_error, axis=1))
            if len(physical_hypothesis_error)
            else timing_distribution(np.empty(0))
        ),
        "score_ns": timing_distribution(score_ns),
        "zero_rust_allocation": bool(
            np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
        ),
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in requested)
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")
    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    configured = profiles()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs: dict[str, dict[str, Any]] = {}
        replays: dict[str, dict[str, Any]] = {}
        for name, config in configured.items():
            kwargs = {key: value for key, value in config.items() if key != "profile"}
            runs[name] = execute(
                model, case, args.duration, config["profile"], **kwargs
            )
            replays[name] = execute(
                model, case, args.duration, config["profile"], **kwargs
            )
        rows[case.name] = {}
        for name, run in runs.items():
            row: dict[str, Any] = {
                "metrics": run["metrics"],
                "terminal_contract": terminal_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
            }
            if "r195_envelope" in name:
                row["attempt_contract"] = support_hypothesis_attempt_contract(
                    run["trace"]
                )
                row["hypothesis_audit"] = hypothesis_audit(
                    run["trace"], balance, limits
                )
                row["realization_audit"] = realization_audit(
                    run["trace"], balance, limits
                )
            rows[case.name][name] = row
        rows[case.name]["exact_envelope_config"]["dormant_exact"] = (
            semantic_trace_equal(
                runs["exact_control"]["trace"],
                runs["exact_envelope_config"]["trace"],
            )
        )
        rows[case.name]["drop5_matched_r195_envelope"][
            "first_attempt_prefix_equal"
        ] = first_hypothesis_attempt_prefix_equal(
            runs["drop5_matched_r195_envelope"]["trace"],
            runs["drop10_r195_envelope"]["trace"],
        )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
                if name.startswith("drop") and "matched" not in name
            ),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    envelope_names = (
        "drop5_r195_envelope",
        "drop10_r195_envelope",
        "drop5_matched_r195_envelope",
    )
    envelope_arms = [
        case_rows[name] for case_rows in rows.values() for name in envelope_names
    ]
    gates = {
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_configuration_is_dormant": all(
            case_rows["exact_envelope_config"]["dormant_exact"]
            for case_rows in rows.values()
        ),
        "terminal_hypotheses_attempted_every_unavailable_tick": all(
            arm["attempt_contract"]["attempted_every_unavailable_tick"]
            for arm in envelope_arms
        ),
        "terminal_selection_only_for_valid_baseline_envelope": all(
            arm["attempt_contract"][
                "queried_exactly_when_baseline_envelope_is_valid"
            ]
            for arm in envelope_arms
        ),
        "invalid_baseline_envelope_fails_closed": all(
            arm["attempt_contract"]["invalid_baseline_fails_closed"]
            for arm in envelope_arms
        ),
        "typed_choice_maps_to_authority": all(
            arm["terminal_contract"]["typed_action_maps_to_authority"]
            and arm["terminal_contract"]["selected_action_is_pareto_admissible"]
            for arm in envelope_arms
        ),
        "hypothesis_envelope_reaudit_is_exact": all(
            arm["hypothesis_audit"]["online_reaudit_disagreement_ticks"] == 0
            and arm["hypothesis_audit"]["zero_allocation"]
            for arm in envelope_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_r195_envelope"][
                "first_attempt_prefix_equal"
            ]
            for case_rows in rows.values()
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            and arm["terminal_contract"]["zero_allocation"]
            for arm in arms
        ),
    }
    consequence_names = (
        "drop5_r191_single_hypothesis",
        "drop10_r191_single_hypothesis",
        "drop5_r195_envelope",
        "drop10_r195_envelope",
    )
    consequences = {name: consequence(rows, name) for name in consequence_names}
    consequence_admitted = all(
        consequences[name]["admitted"]
        for name in ("drop5_r195_envelope", "drop10_r195_envelope")
    )
    realization_samples = sum(
        arm["realization_audit"]["sample_count"] for arm in envelope_arms
    )
    realization_covered = sum(
        arm["realization_audit"]["covered_samples"] for arm in envelope_arms
    )
    realization_summary = {
        "sample_count": realization_samples,
        "covered_samples": realization_covered,
        "componentwise_coverage": (
            realization_covered / realization_samples if realization_samples else 0.0
        ),
        "maximum_envelope_exceedance": max(
            arm["realization_audit"]["maximum_envelope_exceedance"]
            for arm in envelope_arms
        ),
        "maximum_physical_hypothesis_pressure_error_norm": max(
            arm["realization_audit"][
                "physical_hypothesis_pressure_error_norm"
            ]["maximum"]
            for arm in envelope_arms
        ),
        "selected_action_counts": {
            str(action): sum(
                arm["realization_audit"]["selected_action_counts"][str(action)]
                for arm in envelope_arms
            )
            for action in range(3)
        },
        "physical_support_mask_counts": {
            str(mask): sum(
                arm["realization_audit"]["physical_support_mask_counts"][str(mask)]
                for arm in envelope_arms
            )
            for mask in range(4)
        },
    }
    bracketing_passed = all(
        arm["realization_audit"]["sample_count"] > 0
        and arm["realization_audit"]["componentwise_coverage"] == 1.0
        and arm["realization_audit"]["zero_rust_allocation"]
        for arm in envelope_arms
    )
    mechanism_passed = all(gates.values())
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "fixed_effort_hypothesis_ns_maximum": max(
            arm["hypothesis_audit"]["fixed_effort_query_ns"]["maximum"]
            for arm in envelope_arms
        ),
        "aggregation_ns_maximum": max(
            arm["hypothesis_audit"]["aggregation_ns"]["maximum"]
            for arm in envelope_arms
        ),
        "realization_score_ns_maximum": max(
            arm["realization_audit"]["score_ns"]["maximum"]
            for arm in envelope_arms
        ),
    }
    timing_passed = (
        timing["loop_overruns"] == 0
        and timing["controller_step_ns_maximum"] <= 5_000_000
    )
    synchronous_admitted = (
        mechanism_passed
        and bracketing_passed
        and consequence_admitted
        and timing_passed
    )
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)
    hypothesis_attempts = sum(
        arm["attempt_contract"]["attempted_ticks"] for arm in envelope_arms
    )
    hypothesis_queries = sum(
        arm["hypothesis_audit"]["query_count"] for arm in envelope_arms
    )
    invalid_baseline_attempts = sum(
        arm["attempt_contract"]["invalid_baseline_attempt_ticks"]
        for arm in envelope_arms
    )
    realization_samples = sum(
        arm["realization_audit"]["sample_count"] for arm in envelope_arms
    )
    realization_covered_samples = sum(
        arm["realization_audit"]["covered_samples"] for arm in envelope_arms
    )
    realization_coverage = (
        realization_covered_samples / realization_samples
        if realization_samples
        else 0.0
    )
    maximum_exceedance_by_pressure = np.max(
        np.asarray(
            [
                arm["realization_audit"][
                    "maximum_envelope_exceedance_by_pressure"
                ]
                for arm in envelope_arms
            ],
            np.float64,
        ),
        axis=0,
    ).tolist()
    query_gaps = [
        {
            "case": case_name,
            "profile": name,
            "unavailable_ticks": arm["terminal_contract"]["unavailable_ticks"],
            "queried_ticks": arm["terminal_contract"]["queried_ticks"],
        }
        for case_name, case_rows in rows.items()
        for name, arm in case_rows.items()
        if name in envelope_names
        and not arm["terminal_contract"]["queries_every_unavailable_tick"]
    ]

    detail: list[list[Any]] = []
    bracketing_detail: list[list[Any]] = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        for duration in ("drop5", "drop10"):
            old = case_rows[f"{duration}_r191_single_hypothesis"]["metrics"]
            new = case_rows[f"{duration}_r195_envelope"]["metrics"]
            detail.append(
                [
                    case_name,
                    duration,
                    outcome(exact),
                    outcome(old),
                    outcome(new),
                    (
                        "—"
                        if not exact["fell"] or not new["fell"]
                        else f"{float(new['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
                    ),
                ]
            )
        for name in envelope_names:
            audit = case_rows[name]["realization_audit"]
            bracketing_detail.append(
                [
                    case_name,
                    name.replace("_r195_envelope", ""),
                    audit["sample_count"],
                    audit["selected_action_counts"],
                    audit["physical_support_mask_counts"],
                    f"{audit['componentwise_coverage'] * 100.0:.1f}%",
                    f"{audit['maximum_envelope_exceedance']:.3f}",
                    f"{audit['physical_hypothesis_pressure_error_norm']['maximum']:.3f}",
                ]
            )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "root_impact_plane_m": ROOT_IMPACT_PLANE_M,
        "support_hypotheses": SUPPORT_HYPOTHESES,
        "mechanism_passed": mechanism_passed,
        "realization_bracketing_passed": bracketing_passed,
        "envelope_consequence_admitted": consequence_admitted,
        "timing_passed": timing_passed,
        "synchronous_profile_admitted": synchronous_admitted,
        "first_run_ticks": first_run_ticks,
        "hypothesis_attempts": hypothesis_attempts,
        "hypothesis_queries": hypothesis_queries,
        "invalid_baseline_attempts": invalid_baseline_attempts,
        "realization_samples": realization_samples,
        "realization_covered_samples": realization_covered_samples,
        "realization_coverage": realization_coverage,
        "maximum_exceedance_by_pressure": dict(
            zip(PRESSURE_NAMES, maximum_exceedance_by_pressure, strict=True)
        ),
        "gates": gates,
        "query_gaps": query_gaps,
        "realization_summary": realization_summary,
        "consequences": consequences,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw support-hypothesis terminal envelope A/B · r195",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · realization bracketing **{'PASS' if bracketing_passed else 'FAIL'}** · plant consequence **{'ADMITTED' if consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_admitted else 'REJECTED'}**.",
            "",
            "## Explicit uncertainty model",
            "",
            "- Missing support evidence no longer collapses the plant to one fictitious contact mode. Rust fixed-effort dynamics realizes withhold, retained effort, and support-free effort under none/left/right/double support; Rust then takes a componentwise harm envelope and applies the unchanged conservative typed chooser.",
            "- Every action × support query is state-local and bounded. MuJoCo remains an offline consequence oracle only. Unavailable alternatives fail closed, ballistic evidence remains common, and no envelope component is hidden inside one scalar score.",
            "- For every executed selection, the audit finite-differences the following 5 ms plant velocity, rescales that realized acceleration through the same Rust scorer, and tests each pressure against the selected action's four-support envelope. The measured physical support hypothesis remains separately visible, so model error cannot be mislabeled as support uncertainty.",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks**; attempted **{hypothesis_attempts}** support envelopes and independently re-audited **{hypothesis_queries}** valid online selections.",
            f"- **{invalid_baseline_attempts}** attempts had at least one inadmissible baseline support witness. All remained typed withhold: authority 0, non-executable, exactly zero actuator effort. They are mechanism evidence, not plant-bracketing samples.",
            "",
            "## Result",
            "",
            f"- Componentwise realization coverage is **{realization_covered}/{realization_samples} ({realization_summary['componentwise_coverage'] * 100.0:.2f}%)**. Maximum envelope exceedance is **{realization_summary['maximum_envelope_exceedance']:.3f} pressure**; maximum measured-physical-hypothesis error norm is **{realization_summary['maximum_physical_hypothesis_pressure_error_norm']:.3f}**.",
            f"- Selected actions withhold/retained/support-free are **{realization_summary['selected_action_counts']['0']}/{realization_summary['selected_action_counts']['1']}/{realization_summary['selected_action_counts']['2']}**. Measured none/left/right/double support counts are **{realization_summary['physical_support_mask_counts']['0']}/{realization_summary['physical_support_mask_counts']['1']}/{realization_summary['physical_support_mask_counts']['2']}/{realization_summary['physical_support_mask_counts']['3']}**.",
            f"- Profiles without a terminal selection on every unavailable tick: **{query_gaps}**. Invalid fixed-support dynamics fails closed to typed withhold; it is not silently removed from the envelope.",
            "- The four support modes expose support uncertainty but do not bound plant realization error. The mechanism remains diagnostic and is not promoted to authority.",
            "",
            "## Plant consequence",
            "",
            *markdown_table(
                ["case", "dropout", "exact", "r191 single", "r195 envelope", "r195 fall Δ s"],
                detail,
            ),
            "",
            "## Realization bracketing",
            "",
            *markdown_table(
                [
                    "case",
                    "profile",
                    "samples",
                    "actions",
                    "physical masks",
                    "coverage",
                    "max exceedance",
                    "physical-hypothesis error max",
                ],
                bracketing_detail,
            ),
            "",
            f"Aggregate full-component coverage: **{realization_covered_samples}/{realization_samples} ({realization_coverage * 100.0:.1f}%)**.",
            "",
            *markdown_table(
                ["pressure", "maximum envelope exceedance"],
                [
                    [name, f"{value:.6f}"]
                    for name, value in zip(
                        PRESSURE_NAMES,
                        maximum_exceedance_by_pressure,
                        strict=True,
                    )
                ],
            ),
            "",
            "## Runtime",
            "",
            f"- Twelve fixed-effort hypotheses / Rust aggregation maxima: **{timing['fixed_effort_hypothesis_ns_maximum'] / 1e3:.3f} / {timing['aggregation_ns_maximum'] / 1e3:.3f} µs**.",
            f"- Independent realized-interval rescoring maximum: **{timing['realization_score_ns_maximum'] / 1e3:.3f} µs** with zero Rust allocation.",
            f"- Full loop: **{timing['loop_overruns']}** 5 ms overruns; loop/controller maxima **{timing['loop_ns_maximum'] / 1e6:.3f} / {timing['controller_step_ns_maximum'] / 1e6:.3f} ms**.",
            "- Fixed-effort, scoring, aggregation, selection, and controller hot paths report zero Rust allocation; Python GC remains zero.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-support-hypothesis-envelope-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_SUPPORT_HYPOTHESIS_ENVELOPE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "realization_bracketing_passed": bracketing_passed,
                "envelope_consequence_admitted": consequence_admitted,
                "synchronous_profile_admitted": synchronous_admitted,
                "consequences": consequences,
                "timing": timing,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
