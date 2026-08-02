#!/usr/bin/env python3
"""R259 zero-physics paired tube freeze from the R258 complete terminal state."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256
from g1_paired_state_tube_freeze_r257 import (
    AGGREGATE_INDEX,
    CANDIDATES,
    COMPONENT_NAMES,
    CONTROL_DT_S,
    GROUPS,
    HEADROOM_INDEX,
    HYPOTHESES,
    actual_deltas,
)
from g1_paired_terminal_score_freeze_r253 import causal_group


REVISION = "g1-terminal-state-tube-freeze-r259"
SOURCE_REVISION = "g1-spent-terminal-state-corpus-r258"
REHEARSAL_REVISION = "g1-paired-terminal-score-plant-holdout-r254"
SOURCE = pathlib.Path(
    "benchmarks/results/g1-spent-terminal-state-corpus-r258/"
    "g1-spent-terminal-state-corpus.npz"
)
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-spent-terminal-state-corpus-r258/"
    "g1-spent-terminal-state-corpus-metrics.json"
)
REHEARSAL = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-plant-holdout-r254/"
    "g1-paired-terminal-score-plant-holdout.npz"
)
MINIMUM_GROUP_LAW_SAMPLES = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source", default=str(SOURCE))
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--rehearsal", default=str(REHEARSAL))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_TERMINAL_STATE_TUBE_FREEZE_R259.html"
    )
    return parser.parse_args()


def predicted_terminal_state(
    root_state: np.ndarray,
    q: np.ndarray,
    initial_velocity: np.ndarray,
    acceleration: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    velocity = initial_velocity[:, None, :] + CONTROL_DT_S * acceleration
    transition_root = np.empty((*velocity.shape[:2], 6), np.float64)
    transition_root[:, :, 0] = root_state[:, None, 0] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, 5] + velocity[:, :, 5]
    )
    transition_root[:, :, 1] = velocity[:, :, 5]
    transition_root[:, :, 2:4] = root_state[:, None, 1:3] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, :2] + velocity[:, :, :2]
    )
    transition_root[:, :, 4:6] = velocity[:, :, :2]
    position = q[:, None, :] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, 6:] + velocity[:, :, 6:]
    )
    return support_free_terminal_coordinates(
        transition_root, position, velocity[:, :, 6:]
    )


def support_free_terminal_coordinates(
    root: np.ndarray, q: np.ndarray, v: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gravity = 9.81
    clearance = np.maximum(root[:, :, 0], 0.0)
    vertical_velocity = root[:, :, 1]
    impact_time = (
        vertical_velocity
        + np.sqrt(vertical_velocity * vertical_velocity + 2.0 * gravity * clearance)
    ) / gravity
    terminal_root = np.empty((*root.shape[:2], 4), np.float64)
    terminal_root[:, :, :2] = root[:, :, 2:4] + root[:, :, 4:6] * impact_time[:, :, None]
    terminal_root[:, :, 2:4] = root[:, :, 4:6]
    terminal_q = q + v * impact_time[:, :, None]
    return terminal_root, terminal_q, v


def fit_box_profiles(
    values: np.ndarray, groups: np.ndarray
) -> tuple[np.ndarray, np.ndarray, int]:
    law_index = np.repeat(np.arange(2, dtype=np.int64), len(groups) // 2)
    shape = (GROUPS, 2, *values.shape[1:])
    lower = np.empty(shape, np.float64)
    upper = np.empty_like(lower)
    fallback = 0
    for group in range(GROUPS):
        for law in range(2):
            members = np.flatnonzero((groups == group) & (law_index == law))
            if len(members) < MINIMUM_GROUP_LAW_SAMPLES:
                members = np.flatnonzero(law_index == law)
                fallback += 1
            lower[group, law] = np.min(values[members], axis=0)
            upper[group, law] = np.max(values[members], axis=0)
    return lower, upper, fallback


def fit_profiles(
    groups: np.ndarray,
    predicted_root: np.ndarray,
    predicted_q: np.ndarray,
    predicted_v: np.ndarray,
    actual_root: np.ndarray,
    actual_q: np.ndarray,
    actual_v: np.ndarray,
) -> tuple[dict[str, np.ndarray], dict[str, int]]:
    baseline_root = actual_root[:, 0] - predicted_root[:, 0]
    baseline_q = actual_q[:, 0] - predicted_q[:, 0]
    baseline_v = actual_v[:, 0] - predicted_v[:, 0]
    paired_root = (
        actual_root - actual_root[:, 0:1]
        - (predicted_root - predicted_root[:, 0:1])
    )
    paired_q = actual_q - actual_q[:, 0:1] - (predicted_q - predicted_q[:, 0:1])
    paired_v = (
        actual_v - actual_v[:, 0:1]
        - (predicted_v - predicted_v[:, 0:1])
    )
    arrays: dict[str, np.ndarray] = {}
    fallbacks: dict[str, int] = {}
    for name, values in (
        ("baseline_root", baseline_root),
        ("baseline_q", baseline_q),
        ("baseline_v", baseline_v),
        ("paired_root", paired_root),
        ("paired_q", paired_q),
        ("paired_v", paired_v),
    ):
        arrays[name + "_lower"], arrays[name + "_upper"], fallbacks[name] = (
            fit_box_profiles(values, groups)
        )
    for name in ("paired_root", "paired_q", "paired_v"):
        arrays[name + "_lower"][:, :, 0] = 0.0
        arrays[name + "_upper"][:, :, 0] = 0.0
    return arrays, fallbacks


def evaluate(
    session: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    root_state: np.ndarray,
    q: np.ndarray,
    initial_velocity: np.ndarray,
    acceleration: np.ndarray,
    effort: np.ndarray,
    fixed_status: np.ndarray,
    actual_diagnostics: np.ndarray,
    profiles: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    rows, candidates, _ = acceleration.shape
    groups = causal_group(root_state, initial_velocity)
    predicted_root, predicted_q, predicted_v = predicted_terminal_state(
        root_state, q, initial_velocity, acceleration
    )
    laws = np.asarray([0, 0, 1, 1], np.int64)
    hypothesis = np.empty((candidates, HYPOTHESES, 14), np.float64)
    envelope = np.empty((candidates, 14), np.float64)
    selection = np.empty(6, np.float64)
    hypothesis_repeat = np.empty_like(hypothesis)
    envelope_repeat = np.empty_like(envelope)
    selection_repeat = np.empty_like(selection)
    envelopes = np.empty((rows, candidates, 14), np.float64)
    selections = np.empty((rows, 6), np.float64)
    timing_ns = np.empty(rows, np.uint64)
    calls = np.empty(rows, np.uint64)
    bytes_ = np.empty(rows, np.uint64)
    repeat = np.ones(rows, np.uint8)
    for row in range(rows):
        group = int(groups[row])
        base_root_lower = np.empty((HYPOTHESES, 4), np.float64)
        base_root_upper = np.empty_like(base_root_lower)
        base_q_lower = np.empty((HYPOTHESES, q.shape[1]), np.float64)
        base_q_upper = np.empty_like(base_q_lower)
        base_v_lower = np.empty_like(base_q_lower)
        base_v_upper = np.empty_like(base_q_lower)
        delta_root_lower = np.zeros((candidates, HYPOTHESES, 4), np.float64)
        delta_root_upper = np.zeros_like(delta_root_lower)
        delta_q_lower = np.zeros((candidates, HYPOTHESES, q.shape[1]), np.float64)
        delta_q_upper = np.zeros_like(delta_q_lower)
        delta_v_lower = np.zeros_like(delta_q_lower)
        delta_v_upper = np.zeros_like(delta_q_lower)
        for h, law in enumerate(laws):
            base_root_lower[h] = predicted_root[row, 0] + profiles["baseline_root_lower"][group, law]
            base_root_upper[h] = predicted_root[row, 0] + profiles["baseline_root_upper"][group, law]
            base_q_lower[h] = predicted_q[row, 0] + profiles["baseline_q_lower"][group, law]
            base_q_upper[h] = predicted_q[row, 0] + profiles["baseline_q_upper"][group, law]
            base_v_lower[h] = predicted_v[row, 0] + profiles["baseline_v_lower"][group, law]
            base_v_upper[h] = predicted_v[row, 0] + profiles["baseline_v_upper"][group, law]
            for candidate in range(1, candidates):
                delta_root_lower[candidate, h] = (
                    predicted_root[row, candidate] - predicted_root[row, 0]
                    + profiles["paired_root_lower"][group, law, candidate]
                )
                delta_root_upper[candidate, h] = (
                    predicted_root[row, candidate] - predicted_root[row, 0]
                    + profiles["paired_root_upper"][group, law, candidate]
                )
                delta_q_lower[candidate, h] = (
                    predicted_q[row, candidate] - predicted_q[row, 0]
                    + profiles["paired_q_lower"][group, law, candidate]
                )
                delta_q_upper[candidate, h] = (
                    predicted_q[row, candidate] - predicted_q[row, 0]
                    + profiles["paired_q_upper"][group, law, candidate]
                )
                delta_v_lower[candidate, h] = (
                    predicted_v[row, candidate] - predicted_v[row, 0]
                    + profiles["paired_v_lower"][group, law, candidate]
                )
                delta_v_upper[candidate, h] = (
                    predicted_v[row, candidate] - predicted_v[row, 0]
                    + profiles["paired_v_upper"][group, law, candidate]
                )
        available = np.repeat((fixed_status[row, :, None] <= 1), HYPOTHESES, axis=1).astype(np.uint8)
        baseline_effort = np.full(HYPOTHESES, effort[row, 0], np.float64)
        candidate_effort = np.repeat(effort[row, :, None], HYPOTHESES, axis=1)
        args = (
            base_root_lower,
            base_root_upper,
            delta_root_lower,
            delta_root_upper,
            base_q_lower,
            base_q_upper,
            delta_q_lower,
            delta_q_upper,
            base_v_lower,
            base_v_upper,
            delta_v_lower,
            delta_v_upper,
            limits[0],
            limits[1],
            limits[2],
            available,
            baseline_effort,
            candidate_effort,
            0,
            0.0,
            0.01,
        )
        timing = session.bound_terminal_impact_paired_state_tubes(
            *args, hypothesis, envelope, selection
        )
        session.bound_terminal_impact_paired_state_tubes(
            *args, hypothesis_repeat, envelope_repeat, selection_repeat
        )
        timing_ns[row], calls[row], bytes_[row] = timing
        repeat[row] = int(
            np.array_equal(hypothesis, hypothesis_repeat)
            and np.array_equal(envelope, envelope_repeat)
            and np.array_equal(selection, selection_repeat)
        )
        envelopes[row] = envelope
        selections[row] = selection
    components, aggregate = actual_deltas(actual_diagnostics)
    component_covered = (
        (components >= envelopes[:, :, :6] - 1.0e-12)
        & (components <= envelopes[:, :, 6:12] + 1.0e-12)
    )
    aggregate_covered = (
        (aggregate >= envelopes[:, :, 12] - 1.0e-12)
        & (aggregate <= envelopes[:, :, 13] + 1.0e-12)
    )
    miss = np.maximum(
        np.maximum(envelopes[:, :, :6] - components, 0.0),
        np.maximum(components - envelopes[:, :, 6:12], 0.0),
    )
    selected = selections[:, 0].astype(np.int64)
    selected_components = components[np.arange(rows), selected]
    selected_aggregate = aggregate[np.arange(rows), selected]
    nonzero = selected != 0
    metrics = {
        "rows": rows,
        "selected_counts": {str(i): int(np.count_nonzero(selected == i)) for i in range(candidates)},
        "nonzero_actions": int(np.count_nonzero(nonzero)),
        "component_coverage_fraction": float(np.mean(component_covered)),
        "component_coverage": {
            name: float(np.mean(component_covered[:, :, index]))
            for index, name in enumerate(COMPONENT_NAMES)
        },
        "maximum_component_miss": {
            name: float(np.max(miss[:, :, index]))
            for index, name in enumerate(COMPONENT_NAMES)
        },
        "all_component_boxes_covered": bool(np.all(component_covered)),
        "aggregate_coverage_fraction": float(np.mean(aggregate_covered)),
        "all_aggregate_boxes_covered": bool(np.all(aggregate_covered)),
        "selected_component_regression_rows": int(
            np.count_nonzero(np.max(selected_components, axis=1) > 1.0e-12)
        ),
        "selected_aggregate_regression_rows": int(np.count_nonzero(selected_aggregate > 1.0e-12)),
        "strict_nonregressing_improving_nonzero_actions": int(
            np.count_nonzero(
                nonzero
                & (np.max(selected_components, axis=1) <= 1.0e-12)
                & (selected_aggregate < -1.0e-12)
            )
        ),
        "minimum_nonzero_component_upper": {
            name: float(np.min(envelopes[:, 1:, 6 + index]))
            for index, name in enumerate(COMPONENT_NAMES)
        },
        "minimum_nonzero_worst_component_upper": float(
            np.min(np.max(envelopes[:, 1:, 6:12], axis=2))
        ),
        "minimum_nonzero_aggregate_upper": float(
            np.min(envelopes[:, 1:, 13])
        ),
        "timing_ns": distribution(timing_ns),
        "zero_rust_allocation": bool(np.all(calls == 0) and np.all(bytes_ == 0)),
        "semantic_repeat": bool(np.all(repeat == 1)),
    }
    return metrics, {"envelopes": envelopes, "selections": selections}


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_path = pathlib.Path(args.source).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    rehearsal_path = pathlib.Path(args.rehearsal).resolve()
    if not all(path.is_file() for path in (model, source_path, source_metrics_path, rehearsal_path)):
        raise SystemExit("R259 requires the pinned model and immutable R258/R254 arrays")
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics.get("revision") != SOURCE_REVISION or not source_metrics.get("extraction_passed"):
        raise ValueError("R259 requires the passed R258 terminal-state extraction")
    hashes = {
        "source": sha256(source_path),
        "source_metrics": sha256(source_metrics_path),
        "rehearsal": sha256(rehearsal_path),
    }
    with np.load(source_path, allow_pickle=False) as source:
        root_state = np.asarray(source["root_state"], np.float64)
        q = np.asarray(source["joint_position"], np.float64)
        initial_velocity = np.asarray(source["initial_velocity"], np.float64)
        acceleration = np.asarray(source["candidate_acceleration"], np.float64)
        effort = np.asarray(source["candidate_effort_utilization"], np.float64)
        fixed_status = np.asarray(source["fixed_status"], np.uint8)
        actual_root = np.asarray(source["actual_root_terminal_state"], np.float64)
        actual_q = np.asarray(source["actual_joint_terminal_position"], np.float64)
        actual_v = np.asarray(source["actual_velocity"], np.float64)
        actual_diagnostics = np.asarray(source["actual_diagnostics"], np.float64)
    predicted_root, predicted_q, predicted_v = predicted_terminal_state(
        root_state, q, initial_velocity, acceleration
    )
    actual_terminal_root, actual_terminal_q, actual_terminal_v = (
        support_free_terminal_coordinates(
            actual_root,
            actual_q,
            actual_v[:, :, 6:],
        )
    )
    groups = causal_group(root_state, initial_velocity)
    profiles, fallbacks = fit_profiles(
        groups,
        predicted_root,
        predicted_q,
        predicted_v,
        actual_terminal_root,
        actual_terminal_q,
        actual_terminal_v,
    )
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    limits = joint_limits(model, list(session.joint_names()))
    source_result, source_arrays = evaluate(
        session, limits, root_state, q, initial_velocity, acceleration, effort,
        fixed_status, actual_diagnostics, profiles
    )
    with np.load(rehearsal_path, allow_pickle=False) as rehearsal:
        rehearsal_result, rehearsal_arrays = evaluate(
            session,
            limits,
            np.asarray(rehearsal["root_state"], np.float64),
            np.asarray(rehearsal["joint_position"], np.float64),
            np.asarray(rehearsal["initial_velocity"], np.float64),
            np.asarray(rehearsal["candidate_acceleration"], np.float64),
            np.asarray(rehearsal["candidate_effort_utilization"], np.float64),
            np.asarray(rehearsal["fixed_status"], np.uint8),
            np.asarray(rehearsal["actual_diagnostics"], np.float64),
            profiles,
        )
    profile_frozen = bool(
        source_result["all_component_boxes_covered"]
        and source_result["all_aggregate_boxes_covered"]
        and source_result["nonzero_actions"] > 0
        and source_result["selected_component_regression_rows"] == 0
        and source_result["selected_aggregate_regression_rows"] == 0
    )
    rehearsal_transferred = bool(
        profile_frozen
        and rehearsal_result["all_component_boxes_covered"]
        and rehearsal_result["all_aggregate_boxes_covered"]
        and rehearsal_result["selected_component_regression_rows"] == 0
        and rehearsal_result["selected_aggregate_regression_rows"] == 0
    )
    immutable = hashes == {
        "source": sha256(source_path),
        "source_metrics": sha256(source_metrics_path),
        "rehearsal": sha256(rehearsal_path),
    }
    mechanism_passed = bool(
        immutable
        and source_result["zero_rust_allocation"]
        and rehearsal_result["zero_rust_allocation"]
        and source_result["semantic_repeat"]
        and rehearsal_result["semantic_repeat"]
    )
    result = {
        "revision": REVISION,
        "source_revision": SOURCE_REVISION,
        "rehearsal_revision": REHEARSAL_REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "source_hashes": hashes,
        "source_immutable": immutable,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "fallback_cells": fallbacks,
        "source": source_result,
        "spent_rehearsal": rehearsal_result,
        "mechanism_passed": mechanism_passed,
        "profile_frozen": profile_frozen,
        "spent_rehearsal_transferred": rehearsal_transferred,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw complete terminal-state tube freeze · r259",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · profile **{'FROZEN' if profile_frozen else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "R259 fits complete terminal attitude, angular-rate, joint-position, and joint-velocity residual tubes from immutable R258 state arrays. Candidate and baseline remain paired through the R256 Rust boundary. This evaluator executes zero policy steps, physics steps, or plant actions; R258's spent extraction is a separate provenance layer.",
            "",
            f"Source component/aggregate coverage is {source_result['component_coverage_fraction'] * 100.0:.3f}%/{source_result['aggregate_coverage_fraction'] * 100.0:.3f}%. The selector chooses {source_result['nonzero_actions']} nonzero rows; selected component/aggregate regressions are {source_result['selected_component_regression_rows']}/{source_result['selected_aggregate_regression_rows']}, and p99 is {source_result['timing_ns']['p99'] / 1e3:.2f} µs.",
            "",
            f"Coverage is complete but not useful: the smallest nonzero candidate joint-position upper is {source_result['minimum_nonzero_component_upper']['joint_position']:.6g}, the smallest worst-component upper is {source_result['minimum_nonzero_worst_component_upper']:.6g}, and the smallest aggregate upper is {source_result['minimum_nonzero_aggregate_upper']:.6g}. The coarse group/law componentwise state boxes discard too much cross-coordinate and state-local structure.",
            "",
            f"The unchanged profile is rehearsed on already-spent R254 diagnostics: component/aggregate coverage is {rehearsal_result['component_coverage_fraction'] * 100.0:.3f}%/{rehearsal_result['aggregate_coverage_fraction'] * 100.0:.3f}%, with {rehearsal_result['nonzero_actions']} nonzero selections and {rehearsal_result['selected_component_regression_rows']}/{rehearsal_result['selected_aggregate_regression_rows']} selected component/aggregate regressions.",
            "",
            "Only complete source coverage plus a useful strictly nonregressing action can freeze the profile. The spent rehearsal may reject but cannot serve as a fresh holdout or admit authority.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-terminal-state-tube-freeze-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-terminal-state-tube-freeze.npz",
        **profiles,
        source_envelopes=source_arrays["envelopes"],
        source_selections=source_arrays["selections"],
        rehearsal_envelopes=rehearsal_arrays["envelopes"],
        rehearsal_selections=rehearsal_arrays["selections"],
    )
    (output / "G1_TERMINAL_STATE_TUBE_FREEZE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw complete state tube · r259"))
    print(json.dumps({
        "mechanism_passed": mechanism_passed,
        "profile_frozen": profile_frozen,
        "spent_rehearsal_transferred": rehearsal_transferred,
        "authority_admitted": False,
    }, indent=2, sort_keys=True))
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
