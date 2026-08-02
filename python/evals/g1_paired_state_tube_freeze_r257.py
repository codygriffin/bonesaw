#!/usr/bin/env python3
"""R257 policy-/physics-free paired state-tube freeze and spent rehearsal."""

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
from g1_paired_terminal_score_freeze_r253 import causal_group


REVISION = "g1-paired-state-tube-freeze-r257"
SOURCE_REVISION = "g1-actuator-bandwidth-action-freeze-r250"
REHEARSAL_REVISION = "g1-paired-terminal-score-plant-holdout-r254"
SOURCE = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze.npz"
)
REHEARSAL = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-plant-holdout-r254/"
    "g1-paired-terminal-score-plant-holdout.npz"
)
PREFIX = "bandwidth_25hz_slew_1000_nm_s_zero_wbc_and_neutral_recovery_"
CONTROL_DT_S = 0.020
CANDIDATES = 3
HYPOTHESES = 4
GROUPS = 16
COMPONENT_NAMES = (
    "tilt",
    "angular_rate",
    "joint_position",
    "joint_velocity",
    "actuator_effort",
    "raw_headroom_loss",
)
COMPONENT_INDICES = np.asarray([9, 10, 11, 12, 13], np.int64)
HEADROOM_INDEX = 6
AGGREGATE_INDEX = 16
MINIMUM_GROUP_LAW_SAMPLES = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source", default=str(SOURCE))
    parser.add_argument("--rehearsal", default=str(REHEARSAL))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_PAIRED_STATE_TUBE_FREEZE_R257.html"
    )
    return parser.parse_args()


def actual_deltas(diagnostics: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    components = np.empty((*diagnostics.shape[:2], 6), np.float64)
    components[:, :, :5] = (
        diagnostics[:, :, COMPONENT_INDICES]
        - diagnostics[:, 0:1, COMPONENT_INDICES]
    )
    components[:, :, 5] = (
        diagnostics[:, 0:1, HEADROOM_INDEX]
        - diagnostics[:, :, HEADROOM_INDEX]
    )
    aggregate = (
        diagnostics[:, :, AGGREGATE_INDEX]
        - diagnostics[:, 0:1, AGGREGATE_INDEX]
    )
    return components, aggregate


def fit_profiles(
    groups: np.ndarray,
    predicted_velocity: np.ndarray,
    actual_velocity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, int]]:
    generalized = predicted_velocity.shape[2]
    law_index = np.repeat(np.arange(2, dtype=np.int64), len(groups) // 2)
    baseline_residual = actual_velocity[:, 0] - predicted_velocity[:, 0]
    paired_residual = (
        actual_velocity - actual_velocity[:, 0:1]
        - (predicted_velocity - predicted_velocity[:, 0:1])
    )
    baseline_lower = np.empty((GROUPS, 2, generalized), np.float64)
    baseline_upper = np.empty_like(baseline_lower)
    paired_lower = np.empty((GROUPS, 2, CANDIDATES, generalized), np.float64)
    paired_upper = np.empty_like(paired_lower)
    fallback_cells = 0
    minimum_samples = len(groups)
    maximum_samples = 0
    for group in range(GROUPS):
        for law in range(2):
            members = np.flatnonzero((groups == group) & (law_index == law))
            if len(members) < MINIMUM_GROUP_LAW_SAMPLES:
                members = np.flatnonzero(law_index == law)
                fallback_cells += 1
            minimum_samples = min(minimum_samples, len(members))
            maximum_samples = max(maximum_samples, len(members))
            baseline_lower[group, law] = np.min(baseline_residual[members], axis=0)
            baseline_upper[group, law] = np.max(baseline_residual[members], axis=0)
            paired_lower[group, law] = np.min(paired_residual[members], axis=0)
            paired_upper[group, law] = np.max(paired_residual[members], axis=0)
    # Baseline candidate is a typed exact zero delta, not an empirical box.
    paired_lower[:, :, 0] = 0.0
    paired_upper[:, :, 0] = 0.0
    return baseline_lower, baseline_upper, paired_lower, paired_upper, {
        "fallback_cells": fallback_cells,
        "minimum_cell_samples": minimum_samples,
        "maximum_cell_samples": maximum_samples,
    }


def evaluate_rows(
    session: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    root_state: np.ndarray,
    joint_position: np.ndarray,
    initial_velocity: np.ndarray,
    candidate_acceleration: np.ndarray,
    candidate_effort: np.ndarray,
    fixed_status: np.ndarray,
    actual_diagnostics: np.ndarray,
    baseline_residual_lower: np.ndarray,
    baseline_residual_upper: np.ndarray,
    paired_residual_lower: np.ndarray,
    paired_residual_upper: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    rows, candidates, generalized = candidate_acceleration.shape
    joints = generalized - 6
    groups = causal_group(root_state, initial_velocity)
    predicted_velocity = (
        initial_velocity[:, None, :]
        + CONTROL_DT_S * candidate_acceleration
    )
    hypothesis_output = np.empty((candidates, HYPOTHESES, 14), np.float64)
    envelope_output = np.empty((candidates, 14), np.float64)
    selection_output = np.empty(6, np.float64)
    hypothesis_repeat = np.empty_like(hypothesis_output)
    envelope_repeat = np.empty_like(envelope_output)
    selection_repeat = np.empty_like(selection_output)
    envelopes = np.empty((rows, candidates, 14), np.float64)
    selections = np.empty((rows, 6), np.float64)
    timing_ns = np.empty(rows, np.uint64)
    allocation_calls = np.empty(rows, np.uint64)
    allocated_bytes = np.empty(rows, np.uint64)
    semantic_repeat = np.ones(rows, np.uint8)
    hypothesis_law = np.asarray([0, 0, 1, 1], np.int64)
    for row in range(rows):
        group = int(groups[row])
        baseline_root_lower = np.empty((HYPOTHESES, 4), np.float64)
        baseline_root_upper = np.empty_like(baseline_root_lower)
        baseline_q_lower = np.empty((HYPOTHESES, joints), np.float64)
        baseline_q_upper = np.empty_like(baseline_q_lower)
        baseline_v_lower = np.empty_like(baseline_q_lower)
        baseline_v_upper = np.empty_like(baseline_q_lower)
        delta_root_lower = np.zeros((candidates, HYPOTHESES, 4), np.float64)
        delta_root_upper = np.zeros_like(delta_root_lower)
        delta_q_lower = np.zeros((candidates, HYPOTHESES, joints), np.float64)
        delta_q_upper = np.zeros_like(delta_q_lower)
        delta_v_lower = np.zeros_like(delta_q_lower)
        delta_v_upper = np.zeros_like(delta_q_lower)
        baseline_effort = np.full(HYPOTHESES, candidate_effort[row, 0], np.float64)
        effort = np.repeat(candidate_effort[row, :, None], HYPOTHESES, axis=1)
        available = np.repeat((fixed_status[row, :, None] <= 1), HYPOTHESES, axis=1).astype(np.uint8)
        predicted_baseline = predicted_velocity[row, 0]
        predicted_q = joint_position[row] + 0.5 * CONTROL_DT_S * (
            initial_velocity[row, 6:] + predicted_baseline[6:]
        )
        predicted_tilt = root_state[row, 1:3] + 0.5 * CONTROL_DT_S * (
            initial_velocity[row, :2] + predicted_baseline[:2]
        )
        for hypothesis, law in enumerate(hypothesis_law):
            base_lower = baseline_residual_lower[group, law]
            base_upper = baseline_residual_upper[group, law]
            baseline_displacement_lower = np.minimum.reduce(
                (np.zeros(generalized), CONTROL_DT_S * base_lower, CONTROL_DT_S * base_upper)
            )
            baseline_displacement_upper = np.maximum.reduce(
                (np.zeros(generalized), CONTROL_DT_S * base_lower, CONTROL_DT_S * base_upper)
            )
            baseline_root_lower[hypothesis] = [
                predicted_tilt[0] + baseline_displacement_lower[0],
                predicted_tilt[1] + baseline_displacement_lower[1],
                predicted_baseline[0] + base_lower[0],
                predicted_baseline[1] + base_lower[1],
            ]
            baseline_root_upper[hypothesis] = [
                predicted_tilt[0] + baseline_displacement_upper[0],
                predicted_tilt[1] + baseline_displacement_upper[1],
                predicted_baseline[0] + base_upper[0],
                predicted_baseline[1] + base_upper[1],
            ]
            baseline_q_lower[hypothesis] = (
                predicted_q + baseline_displacement_lower[6:]
            )
            baseline_q_upper[hypothesis] = (
                predicted_q + baseline_displacement_upper[6:]
            )
            baseline_v_lower[hypothesis] = predicted_baseline[6:] + base_lower[6:]
            baseline_v_upper[hypothesis] = predicted_baseline[6:] + base_upper[6:]
            for candidate in range(1, candidates):
                predicted_delta = (
                    predicted_velocity[row, candidate] - predicted_baseline
                )
                paired_lower = paired_residual_lower[group, law, candidate]
                paired_upper = paired_residual_upper[group, law, candidate]
                terminal_delta_lower = predicted_delta + paired_lower
                terminal_delta_upper = predicted_delta + paired_upper
                paired_displacement_lower = np.minimum.reduce(
                    (
                        np.zeros(generalized),
                        CONTROL_DT_S * paired_lower,
                        CONTROL_DT_S * paired_upper,
                    )
                )
                paired_displacement_upper = np.maximum.reduce(
                    (
                        np.zeros(generalized),
                        CONTROL_DT_S * paired_lower,
                        CONTROL_DT_S * paired_upper,
                    )
                )
                predicted_displacement_delta = 0.5 * CONTROL_DT_S * predicted_delta
                delta_root_lower[candidate, hypothesis] = [
                    predicted_displacement_delta[0] + paired_displacement_lower[0],
                    predicted_displacement_delta[1] + paired_displacement_lower[1],
                    terminal_delta_lower[0],
                    terminal_delta_lower[1],
                ]
                delta_root_upper[candidate, hypothesis] = [
                    predicted_displacement_delta[0] + paired_displacement_upper[0],
                    predicted_displacement_delta[1] + paired_displacement_upper[1],
                    terminal_delta_upper[0],
                    terminal_delta_upper[1],
                ]
                delta_q_lower[candidate, hypothesis] = (
                    predicted_displacement_delta[6:] + paired_displacement_lower[6:]
                )
                delta_q_upper[candidate, hypothesis] = (
                    predicted_displacement_delta[6:] + paired_displacement_upper[6:]
                )
                delta_v_lower[candidate, hypothesis] = terminal_delta_lower[6:]
                delta_v_upper[candidate, hypothesis] = terminal_delta_upper[6:]
        timing = session.bound_terminal_impact_paired_state_tubes(
            baseline_root_lower,
            baseline_root_upper,
            delta_root_lower,
            delta_root_upper,
            baseline_q_lower,
            baseline_q_upper,
            delta_q_lower,
            delta_q_upper,
            baseline_v_lower,
            baseline_v_upper,
            delta_v_lower,
            delta_v_upper,
            limits[0],
            limits[1],
            limits[2],
            available,
            baseline_effort,
            effort,
            0,
            0.0,
            0.01,
            hypothesis_output,
            envelope_output,
            selection_output,
        )
        session.bound_terminal_impact_paired_state_tubes(
            baseline_root_lower,
            baseline_root_upper,
            delta_root_lower,
            delta_root_upper,
            baseline_q_lower,
            baseline_q_upper,
            delta_q_lower,
            delta_q_upper,
            baseline_v_lower,
            baseline_v_upper,
            delta_v_lower,
            delta_v_upper,
            limits[0],
            limits[1],
            limits[2],
            available,
            baseline_effort,
            effort,
            0,
            0.0,
            0.01,
            hypothesis_repeat,
            envelope_repeat,
            selection_repeat,
        )
        timing_ns[row], allocation_calls[row], allocated_bytes[row] = timing
        semantic_repeat[row] = int(
            np.array_equal(hypothesis_output, hypothesis_repeat)
            and np.array_equal(envelope_output, envelope_repeat)
            and np.array_equal(selection_output, selection_repeat)
        )
        envelopes[row] = envelope_output
        selections[row] = selection_output

    component_delta, aggregate_delta = actual_deltas(actual_diagnostics)
    component_covered = (
        (component_delta >= envelopes[:, :, :6] - 1.0e-12)
        & (component_delta <= envelopes[:, :, 6:12] + 1.0e-12)
    )
    aggregate_covered = (
        (aggregate_delta >= envelopes[:, :, 12] - 1.0e-12)
        & (aggregate_delta <= envelopes[:, :, 13] + 1.0e-12)
    )
    selected = selections[:, 0].astype(np.int64)
    selected_components = component_delta[np.arange(rows), selected]
    selected_aggregate = aggregate_delta[np.arange(rows), selected]
    nonzero = selected != 0
    component_miss = np.maximum(
        np.maximum(envelopes[:, :, :6] - component_delta, 0.0),
        np.maximum(component_delta - envelopes[:, :, 6:12], 0.0),
    )
    strict_nonregressing = np.max(selected_components, axis=1) <= 1.0e-12
    improving = selected_aggregate < -1.0e-12
    metrics = {
        "rows": rows,
        "selected_counts": {
            str(candidate): int(np.count_nonzero(selected == candidate))
            for candidate in range(candidates)
        },
        "nonzero_actions": int(np.count_nonzero(nonzero)),
        "strict_nonregressing_improving_nonzero_actions": int(
            np.count_nonzero(nonzero & strict_nonregressing & improving)
        ),
        "component_coverage_fraction": float(np.mean(component_covered)),
        "component_coverage": {
            name: float(np.mean(component_covered[:, :, index]))
            for index, name in enumerate(COMPONENT_NAMES)
        },
        "maximum_component_miss": {
            name: float(np.max(component_miss[:, :, index]))
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
        "maximum_selected_component_regression": float(
            np.max(selected_components)
        ),
        "timing_ns": distribution(timing_ns),
        "zero_rust_allocation": bool(
            np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
        ),
        "semantic_repeat": bool(np.all(semantic_repeat == 1)),
    }
    return metrics, {
        "groups": groups,
        "envelopes": envelopes,
        "selections": selections,
        "actual_component_delta": component_delta,
        "actual_aggregate_delta": aggregate_delta,
        "timing_ns": timing_ns,
    }


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_path = pathlib.Path(args.source).resolve()
    rehearsal_path = pathlib.Path(args.rehearsal).resolve()
    if not all(path.is_file() for path in (model, source_path, rehearsal_path)):
        raise SystemExit("R257 requires the pinned model and immutable R250/R254 arrays")
    source_hashes = {"source": sha256(source_path), "rehearsal": sha256(rehearsal_path)}
    with np.load(source_path, allow_pickle=False) as replay:
        root_state = np.asarray(replay[PREFIX + "root_state"], np.float64)
        joint_position = np.asarray(replay[PREFIX + "joint_position"], np.float64)
        initial_velocity = np.asarray(replay[PREFIX + "initial_velocity"], np.float64)
        candidate_acceleration = np.asarray(
            replay[PREFIX + "candidate_acceleration"], np.float64
        )
        candidate_effort = np.asarray(
            replay[PREFIX + "candidate_effort_utilization"], np.float64
        )
        actual_velocity = np.asarray(replay[PREFIX + "actual_velocity"], np.float64)
        actual_diagnostics = np.asarray(
            replay[PREFIX + "actual_diagnostics"], np.float64
        )
        fixed_status = np.asarray(replay[PREFIX + "fixed_status"], np.uint8)
    groups = causal_group(root_state, initial_velocity)
    predicted_velocity = initial_velocity[:, None, :] + CONTROL_DT_S * candidate_acceleration
    baseline_lower, baseline_upper, paired_lower, paired_upper, fit_metrics = fit_profiles(
        groups, predicted_velocity, actual_velocity
    )
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    limits = joint_limits(model, list(session.joint_names()))
    source_metrics, source_arrays = evaluate_rows(
        session,
        limits,
        root_state,
        joint_position,
        initial_velocity,
        candidate_acceleration,
        candidate_effort,
        fixed_status,
        actual_diagnostics,
        baseline_lower,
        baseline_upper,
        paired_lower,
        paired_upper,
    )
    with np.load(rehearsal_path, allow_pickle=False) as replay:
        rehearsal_metrics, rehearsal_arrays = evaluate_rows(
            session,
            limits,
            np.asarray(replay["root_state"], np.float64),
            np.asarray(replay["joint_position"], np.float64),
            np.asarray(replay["initial_velocity"], np.float64),
            np.asarray(replay["candidate_acceleration"], np.float64),
            np.asarray(replay["candidate_effort_utilization"], np.float64),
            np.asarray(replay["fixed_status"], np.uint8),
            np.asarray(replay["actual_diagnostics"], np.float64),
            baseline_lower,
            baseline_upper,
            paired_lower,
            paired_upper,
        )
    source_profile_useful = bool(
        source_metrics["all_component_boxes_covered"]
        and source_metrics["all_aggregate_boxes_covered"]
        and source_metrics["nonzero_actions"] > 0
        and source_metrics["selected_component_regression_rows"] == 0
        and source_metrics["selected_aggregate_regression_rows"] == 0
    )
    profile_frozen = source_profile_useful
    rehearsal_transferred = bool(
        profile_frozen
        and rehearsal_metrics["all_component_boxes_covered"]
        and rehearsal_metrics["all_aggregate_boxes_covered"]
        and rehearsal_metrics["selected_component_regression_rows"] == 0
        and rehearsal_metrics["selected_aggregate_regression_rows"] == 0
    )
    source_immutable = source_hashes == {
        "source": sha256(source_path),
        "rehearsal": sha256(rehearsal_path),
    }
    mechanism_passed = bool(
        source_immutable
        and source_metrics["zero_rust_allocation"]
        and rehearsal_metrics["zero_rust_allocation"]
        and source_metrics["semantic_repeat"]
        and rehearsal_metrics["semantic_repeat"]
        and source_metrics["timing_ns"]["p99"] < 5_000_000.0
        and rehearsal_metrics["timing_ns"]["p99"] < 5_000_000.0
    )
    result = {
        "revision": REVISION,
        "source_revision": SOURCE_REVISION,
        "rehearsal_revision": REHEARSAL_REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "source_hashes": source_hashes,
        "source_immutable": source_immutable,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "fit": fit_metrics,
        "source": source_metrics,
        "spent_rehearsal": rehearsal_metrics,
        "mechanism_passed": mechanism_passed,
        "source_profile_useful": source_profile_useful,
        "profile_frozen": profile_frozen,
        "spent_rehearsal_transferred": rehearsal_transferred,
        "authority_admitted": False,
    }
    source_dominant_miss = max(
        source_metrics["maximum_component_miss"],
        key=source_metrics["maximum_component_miss"].get,
    )
    rehearsal_dominant_miss = max(
        rehearsal_metrics["maximum_component_miss"],
        key=rehearsal_metrics["maximum_component_miss"].get,
    )
    outcome = "FROZEN" if profile_frozen else "REJECTED"
    report = "\n".join(
        [
            "# Bonesaw paired state-tube freeze · r257",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · source profile **{outcome}** · authority **NOT ADMITTED**.",
            "",
            "R257 reopens only immutable R250 arrays to fit baseline and candidate-minus-baseline generalized-velocity residual boxes inside fixed causal closing-speed/tilt groups and two declared source contact-law hypotheses. Rust receives a shared baseline terminal-state box plus paired candidate deltas; no independent consequence uppers are subtracted. The evaluator executes zero policy steps, physics steps, or plant actions.",
            "",
            f"On 96 source rows, component/aggregate coverage is {source_metrics['component_coverage_fraction'] * 100.0:.3f}%/{source_metrics['aggregate_coverage_fraction'] * 100.0:.3f}%; {source_metrics['nonzero_actions']} nonzero actions are selected, with {source_metrics['selected_component_regression_rows']} component-regression rows and {source_metrics['selected_aggregate_regression_rows']} aggregate-regression rows. Paired-query p99 is {source_metrics['timing_ns']['p99'] / 1e3:.2f} µs.",
            "",
            f"Source misses are localized: angular-rate, joint-velocity, and effort coverage are {source_metrics['component_coverage']['angular_rate'] * 100.0:.1f}%/{source_metrics['component_coverage']['joint_velocity'] * 100.0:.1f}%/{source_metrics['component_coverage']['actuator_effort'] * 100.0:.1f}%, while tilt/joint-position/raw-headroom coverage are {source_metrics['component_coverage']['tilt'] * 100.0:.1f}%/{source_metrics['component_coverage']['joint_position'] * 100.0:.1f}%/{source_metrics['component_coverage']['raw_headroom_loss'] * 100.0:.1f}%. The largest miss is {source_dominant_miss} at {source_metrics['maximum_component_miss'][source_dominant_miss]:.6g}, showing that endpoint velocity evidence cannot identify within-step terminal position drift.",
            "",
            f"The unchanged profile is rehearsed on already-spent R254 arrays—not a fresh holdout. Component/aggregate coverage is {rehearsal_metrics['component_coverage_fraction'] * 100.0:.3f}%/{rehearsal_metrics['aggregate_coverage_fraction'] * 100.0:.3f}%; {rehearsal_metrics['nonzero_actions']} nonzero actions are selected, with {rehearsal_metrics['selected_component_regression_rows']} component-regression rows and {rehearsal_metrics['selected_aggregate_regression_rows']} aggregate-regression rows. Its largest miss is {rehearsal_dominant_miss} at {rehearsal_metrics['maximum_component_miss'][rehearsal_dominant_miss]:.6g}.",
            "",
            "A profile is frozen only if every source component and aggregate delta is covered, at least one nonzero action is useful, and no selected source row regresses. The spent rehearsal can reject a frozen design but cannot admit authority or replace a genuinely new no-refit law/offset holdout.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-paired-state-tube-freeze-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-paired-state-tube-freeze.npz",
        baseline_residual_lower=baseline_lower,
        baseline_residual_upper=baseline_upper,
        paired_residual_lower=paired_lower,
        paired_residual_upper=paired_upper,
        source_envelopes=source_arrays["envelopes"],
        source_selections=source_arrays["selections"],
        source_actual_component_delta=source_arrays["actual_component_delta"],
        rehearsal_envelopes=rehearsal_arrays["envelopes"],
        rehearsal_selections=rehearsal_arrays["selections"],
        rehearsal_actual_component_delta=rehearsal_arrays["actual_component_delta"],
    )
    (output / "G1_PAIRED_STATE_TUBE_FREEZE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw paired state tube freeze · r257"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_frozen": profile_frozen,
                "spent_rehearsal_transferred": rehearsal_transferred,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
