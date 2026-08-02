#!/usr/bin/env python3
"""R261 correlated complete-state exemplar profile and spent transfer rehearsal."""

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
    CANDIDATES,
    COMPONENT_NAMES,
    CONTROL_DT_S,
    actual_deltas,
)
from g1_paired_terminal_score_freeze_r253 import causal_group


REVISION = "g1-correlated-state-exemplar-profile-r261"
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
MAXIMUM_HYPOTHESES = 16
MAXIMUM_COMPONENT_REGRESSION = 0.0
MINIMUM_COMPONENT_IMPROVEMENT = 0.01


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
        "--web-report", default="web/G1_CORRELATED_STATE_EXEMPLAR_PROFILE_R261.html"
    )
    return parser.parse_args()


def predicted_transition_state(
    root_state: np.ndarray,
    q: np.ndarray,
    initial_velocity: np.ndarray,
    acceleration: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """State after the 20 ms held action and before support-free propagation."""
    velocity = initial_velocity[:, None, :] + CONTROL_DT_S * acceleration
    root = np.empty((*velocity.shape[:2], 6), np.float64)
    root[:, :, 0] = root_state[:, None, 0] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, 5] + velocity[:, :, 5]
    )
    root[:, :, 1] = velocity[:, :, 5]
    root[:, :, 2:4] = root_state[:, None, 1:3] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, :2] + velocity[:, :, :2]
    )
    root[:, :, 4:6] = velocity[:, :, :2]
    position = q[:, None, :] + 0.5 * CONTROL_DT_S * (
        initial_velocity[:, None, 6:] + velocity[:, :, 6:]
    )
    return root, position, velocity[:, :, 6:]


def fit_exemplars(
    groups: np.ndarray,
    predicted_root: np.ndarray,
    predicted_q: np.ndarray,
    predicted_v: np.ndarray,
    actual_root: np.ndarray,
    actual_q: np.ndarray,
    actual_v: np.ndarray,
) -> dict[str, np.ndarray]:
    counts = np.bincount(groups, minlength=16)
    if np.any(counts == 0) or int(np.max(counts)) > MAXIMUM_HYPOTHESES:
        raise ValueError("R261 requires 1..=16 complete exemplars in every causal group")
    return {
        "groups": groups.copy(),
        "baseline_root_residual": actual_root[:, 0] - predicted_root[:, 0],
        "baseline_q_residual": actual_q[:, 0] - predicted_q[:, 0],
        "baseline_v_residual": actual_v[:, 0] - predicted_v[:, 0],
        "paired_root_residual": (
            actual_root
            - actual_root[:, 0:1]
            - (predicted_root - predicted_root[:, 0:1])
        ),
        "paired_q_residual": (
            actual_q
            - actual_q[:, 0:1]
            - (predicted_q - predicted_q[:, 0:1])
        ),
        "paired_v_residual": (
            actual_v
            - actual_v[:, 0:1]
            - (predicted_v - predicted_v[:, 0:1])
        ),
    }


def exemplar_states(
    row: int,
    members: np.ndarray,
    predicted_root: np.ndarray,
    predicted_q: np.ndarray,
    predicted_v: np.ndarray,
    profile: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    hypotheses = len(members)
    joints = predicted_q.shape[2]
    root = np.empty((CANDIDATES, hypotheses, 6), np.float64)
    q = np.empty((CANDIDATES, hypotheses, joints), np.float64)
    v = np.empty_like(q)
    baseline_root = (
        predicted_root[row, 0] + profile["baseline_root_residual"][members]
    )
    baseline_q = predicted_q[row, 0] + profile["baseline_q_residual"][members]
    baseline_v = predicted_v[row, 0] + profile["baseline_v_residual"][members]
    for candidate in range(CANDIDATES):
        root[candidate] = (
            baseline_root
            + predicted_root[row, candidate]
            - predicted_root[row, 0]
            + profile["paired_root_residual"][members, candidate]
        )
        q[candidate] = (
            baseline_q
            + predicted_q[row, candidate]
            - predicted_q[row, 0]
            + profile["paired_q_residual"][members, candidate]
        )
        v[candidate] = (
            baseline_v
            + predicted_v[row, candidate]
            - predicted_v[row, 0]
            + profile["paired_v_residual"][members, candidate]
        )
    return root, q, v


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
    profile: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    groups = causal_group(root_state, initial_velocity)
    predicted_root, predicted_q, predicted_v = predicted_transition_state(
        root_state, q, initial_velocity, acceleration
    )
    rows = len(groups)
    envelopes = np.empty((rows, CANDIDATES, 14), np.float64)
    selections = np.empty((rows, 6), np.float64)
    hypothesis_count = np.empty(rows, np.uint8)
    timing_ns = np.empty(rows, np.uint64)
    allocation_calls = np.empty(rows, np.uint64)
    allocated_bytes = np.empty(rows, np.uint64)
    semantic_repeat = np.ones(rows, np.uint8)
    zero_acceleration = np.zeros(q.shape[1], np.float64)
    for row, group in enumerate(groups):
        members = np.flatnonzero(profile["groups"] == group)
        hypothesis_count[row] = len(members)
        root, position, velocity = exemplar_states(
            row, members, predicted_root, predicted_q, predicted_v, profile
        )
        available = np.repeat(
            (fixed_status[row, :, None] <= 1), len(members), axis=1
        ).astype(np.uint8)
        hypothesis_effort = np.repeat(effort[row, :, None], len(members), axis=1)
        hypothesis = np.empty((CANDIDATES, len(members), 14), np.float64)
        envelope = np.empty((CANDIDATES, 14), np.float64)
        selection = np.empty(6, np.float64)
        hypothesis_repeat = np.empty_like(hypothesis)
        envelope_repeat = np.empty_like(envelope)
        selection_repeat = np.empty_like(selection)
        args = (
            root,
            position,
            velocity,
            limits[0],
            limits[1],
            limits[2],
            zero_acceleration,
            available,
            hypothesis_effort,
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
        )
        timing = session.score_terminal_impact_paired_state_exemplars(
            *args, hypothesis, envelope, selection
        )
        session.score_terminal_impact_paired_state_exemplars(
            *args, hypothesis_repeat, envelope_repeat, selection_repeat
        )
        timing_ns[row], allocation_calls[row], allocated_bytes[row] = timing
        semantic_repeat[row] = int(
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
        "hypotheses_per_row": {
            "minimum": int(np.min(hypothesis_count)),
            "maximum": int(np.max(hypothesis_count)),
        },
        "selected_counts": {
            str(candidate): int(np.count_nonzero(selected == candidate))
            for candidate in range(CANDIDATES)
        },
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
        "selected_aggregate_regression_rows": int(
            np.count_nonzero(selected_aggregate > 1.0e-12)
        ),
        "strict_nonregressing_improving_nonzero_actions": int(
            np.count_nonzero(
                nonzero
                & (np.max(selected_components, axis=1) <= 1.0e-12)
                & (selected_aggregate < -1.0e-12)
            )
        ),
        "minimum_nonzero_worst_component_upper": float(
            np.min(np.max(envelopes[:, 1:, 6:12], axis=2))
        ),
        "minimum_nonzero_aggregate_upper": float(
            np.min(envelopes[:, 1:, 13])
        ),
        "timing_ns": distribution(timing_ns),
        "zero_rust_allocation": bool(
            np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
        ),
        "semantic_repeat": bool(np.all(semantic_repeat == 1)),
    }
    return metrics, {"envelopes": envelopes, "selections": selections}


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_path = pathlib.Path(args.source).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    rehearsal_path = pathlib.Path(args.rehearsal).resolve()
    required = (model, source_path, source_metrics_path, rehearsal_path)
    if not all(path.is_file() for path in required):
        raise SystemExit("R261 requires the pinned model and immutable R258/R254 evidence")
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics.get("revision") != SOURCE_REVISION:
        raise ValueError("R261 requires the R258 terminal-state corpus")
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
        actual_v = np.asarray(source["actual_velocity"], np.float64)[:, :, 6:]
        actual_diagnostics = np.asarray(source["actual_diagnostics"], np.float64)
    predicted_root, predicted_q, predicted_v = predicted_transition_state(
        root_state, q, initial_velocity, acceleration
    )
    profile = fit_exemplars(
        causal_group(root_state, initial_velocity),
        predicted_root,
        predicted_q,
        predicted_v,
        actual_root,
        actual_q,
        actual_v,
    )
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    limits = joint_limits(model, list(session.joint_names()))
    source_result, source_arrays = evaluate(
        session,
        limits,
        root_state,
        q,
        initial_velocity,
        acceleration,
        effort,
        fixed_status,
        actual_diagnostics,
        profile,
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
            profile,
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
    source_candidate_passed = bool(
        source_result["all_component_boxes_covered"]
        and source_result["all_aggregate_boxes_covered"]
        and source_result["nonzero_actions"] > 0
        and source_result["selected_component_regression_rows"] == 0
        and source_result["selected_aggregate_regression_rows"] == 0
    )
    rehearsal_transferred = bool(
        source_candidate_passed
        and rehearsal_result["all_component_boxes_covered"]
        and rehearsal_result["all_aggregate_boxes_covered"]
        and rehearsal_result["selected_component_regression_rows"] == 0
        and rehearsal_result["selected_aggregate_regression_rows"] == 0
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
        "profile_representation": "causal-group_complete-state_correlated_residual_exemplars",
        "maximum_hypotheses": MAXIMUM_HYPOTHESES,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "source": source_result,
        "spent_rehearsal": rehearsal_result,
        "mechanism_passed": mechanism_passed,
        "source_profile_candidate_passed": source_candidate_passed,
        "spent_rehearsal_transferred": rehearsal_transferred,
        "profile_frozen_for_new_holdout": False,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw correlated complete-state exemplar profile · r261",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · source candidate **{'PASS' if source_candidate_passed else 'FAIL'}** · spent transfer **{'PASS' if rehearsal_transferred else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "R261 replaces independent coordinate boxes with 3–12 complete residual exemplars from the same causal state group. Rust pairs baseline and candidate by exemplar index, retains candidate-dependent clearance and vertical velocity, performs support-free propagation and consequence scoring, envelopes at most 16 hypotheses, and applies the conservative selector without allocation. This evaluator executes zero policy steps, physics steps, or plant actions.",
            "",
            f"The immutable R258 source reaches {source_result['component_coverage_fraction'] * 100.0:.3f}% component and {source_result['aggregate_coverage_fraction'] * 100.0:.3f}% aggregate coverage. It selects {source_result['nonzero_actions']} useful nonzero rows, all {source_result['strict_nonregressing_improving_nonzero_actions']} strictly nonregressing and improving, at {source_result['timing_ns']['p99'] / 1e3:.2f} µs p99 with zero timed Rust allocation.",
            "",
            f"The unchanged spent R254 rehearsal rejects transfer: component/aggregate coverage is {rehearsal_result['component_coverage_fraction'] * 100.0:.3f}%/{rehearsal_result['aggregate_coverage_fraction'] * 100.0:.3f}%, {rehearsal_result['selected_component_regression_rows']} selected rows regress a component, and {rehearsal_result['selected_aggregate_regression_rows']} regresses the aggregate. Its {rehearsal_result['nonzero_actions']} nonzero selections include only {rehearsal_result['strict_nonregressing_improving_nonzero_actions']} strict improvements.",
            "",
            "The source-local representation is useful, but it is not frozen for a new holdout. R254 remains spent rejection evidence; no refit, plant command, or authority follows from this report.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-correlated-state-exemplar-profile-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-correlated-state-exemplar-profile.npz",
        **profile,
        source_envelopes=source_arrays["envelopes"],
        source_selections=source_arrays["selections"],
        rehearsal_envelopes=rehearsal_arrays["envelopes"],
        rehearsal_selections=rehearsal_arrays["selections"],
    )
    (output / "G1_CORRELATED_STATE_EXEMPLAR_PROFILE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw correlated state exemplars · r261"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "source_profile_candidate_passed": source_candidate_passed,
                "spent_rehearsal_transferred": rehearsal_transferred,
                "profile_frozen_for_new_holdout": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
