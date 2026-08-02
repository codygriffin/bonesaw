#!/usr/bin/env python3
"""R253 spent-state paired terminal-score profile freeze.

R251 showed that candidate and zero-effort plant error is correlated. R253
turns that observation into the fixed Rust paired-delta selector introduced
after R252. Python fits residual boxes on spent R250 labels under one declared
causal grouping; it cannot relax Rust's component/aggregate nonregression or
guaranteed-improvement rules. Any frozen profile must face new laws and offsets
without refitting.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256
from g1_paired_terminal_delta_audit_r251 import FAMILY_NAMES, PREFIX
from g1_terminal_box_wbc_action_audit import make_wbc_session


REVISION = "g1-paired-terminal-score-freeze-r253"
SOURCE_REVISION = "g1-actuator-bandwidth-action-freeze-r250"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze-metrics.json"
)
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze.npz"
)
CONTROL_DT_S = 0.020
CLOSING_SPEED_BINS_M_S = (0.35, 0.50, 0.65)
MINIMUM_GROUP_SAMPLES = 3
MAXIMUM_COMPONENT_REGRESSION = 0.0
MINIMUM_COMPONENT_IMPROVEMENT = 0.01
# Candidate-dependent consequence deltas. Impact speed is candidate-invariant;
# admission is represented separately by availability.
COMPONENT_INDICES = (9, 10, 11, 12, 13)
COMPONENT_NAMES = (
    "tilt_pressure",
    "angular_rate_pressure",
    "joint_position_pressure",
    "joint_velocity_pressure",
    "actuator_effort_pressure",
    "joint_headroom_loss",
)
HEADROOM_INDEX = 6
COMPONENT_DIAGNOSTIC_INDICES = (*COMPONENT_INDICES, HEADROOM_INDEX)
AGGREGATE_INDEX = 16
HYPOTHESIS_INDICES = (0, 16, 48, 64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_PAIRED_TERMINAL_SCORE_FREEZE_R253.html"
    )
    parser.add_argument("--hypothesis-model", default="models/upkie/upkie.urdf")
    return parser.parse_args()


def causal_group(root_state: np.ndarray, initial_velocity: np.ndarray) -> np.ndarray:
    closing_speed = -initial_velocity[:, 5]
    speed_bin = np.digitize(closing_speed, CLOSING_SPEED_BINS_M_S, right=False)
    tilt_quadrant = (
        (root_state[:, 1] >= 0.0).astype(np.int64) * 2
        + (root_state[:, 2] >= 0.0).astype(np.int64)
    )
    return speed_bin.astype(np.int64) * 4 + tilt_quadrant


def predicted_terminal_diagnostics(
    selector: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    root_state: np.ndarray,
    joint_position: np.ndarray,
    initial_velocity: np.ndarray,
    candidate_acceleration: np.ndarray,
    effort_utilization: np.ndarray,
) -> np.ndarray:
    samples, candidates, generalized = candidate_acceleration.shape
    joints = generalized - 6
    diagnostics = np.empty((samples, candidates, 17), np.float64)
    zero_root_acceleration = np.zeros((1, 2), np.float64)
    zero_joint_acceleration = np.zeros((1, joints), np.float64)
    available = np.ones(1, np.uint8)
    for sample in range(samples):
        terminal_velocity = (
            initial_velocity[sample, None, :]
            + CONTROL_DT_S * candidate_acceleration[sample]
        )
        for candidate in range(candidates):
            velocity = terminal_velocity[candidate]
            state = np.asarray(
                [[
                    root_state[sample, 0]
                    + 0.5
                    * CONTROL_DT_S
                    * (initial_velocity[sample, 5] + velocity[5]),
                    velocity[5],
                    root_state[sample, 1]
                    + 0.5
                    * CONTROL_DT_S
                    * (initial_velocity[sample, 0] + velocity[0]),
                    root_state[sample, 2]
                    + 0.5
                    * CONTROL_DT_S
                    * (initial_velocity[sample, 1] + velocity[1]),
                    velocity[0],
                    velocity[1],
                ]],
                np.float64,
            )
            q = joint_position[sample] + 0.5 * CONTROL_DT_S * (
                initial_velocity[sample, 6:] + velocity[6:]
            )
            output = diagnostics[sample, candidate : candidate + 1]
            selector.score_terminal_impact_state_batch(
                state,
                np.ascontiguousarray(q),
                np.ascontiguousarray(velocity[None, 6:]),
                limits[0],
                limits[1],
                limits[2],
                available,
                zero_root_acceleration,
                zero_joint_acceleration,
                np.asarray([effort_utilization[sample, candidate]], np.float64),
                output,
            )
    return diagnostics


def fit_group_bounds(
    residual: np.ndarray, groups: np.ndarray
) -> tuple[np.ndarray, np.ndarray, dict[str, int]]:
    lower = np.empty_like(residual)
    upper = np.empty_like(residual)
    fallback_rows = 0
    group_sizes = []
    for group in np.unique(groups):
        members = np.flatnonzero(groups == group)
        group_sizes.append(len(members))
        use = members
        if len(use) < MINIMUM_GROUP_SAMPLES:
            use = np.arange(len(groups))
            fallback_rows += len(members)
        lower[members] = np.min(residual[use], axis=0)
        upper[members] = np.max(residual[use], axis=0)
    return lower, upper, {
        "group_count": len(group_sizes),
        "minimum_group_size": min(group_sizes),
        "maximum_group_size": max(group_sizes),
        "fallback_rows": fallback_rows,
    }


def evaluate_family(
    selector: Any,
    hypothesis_selector: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    replay: Any,
    family: str,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = PREFIX + family + "_"
    root_state = np.asarray(replay[prefix + "root_state"], np.float64)
    joint_position = np.asarray(replay[prefix + "joint_position"], np.float64)
    initial_velocity = np.asarray(replay[prefix + "initial_velocity"], np.float64)
    candidate_acceleration = np.asarray(
        replay[prefix + "candidate_acceleration"], np.float64
    )
    effort_utilization = np.asarray(
        replay[prefix + "candidate_effort_utilization"], np.float64
    )
    actual = np.asarray(replay[prefix + "actual_diagnostics"], np.float64)
    fixed_status = np.asarray(replay[prefix + "fixed_status"], np.uint8)
    predicted = predicted_terminal_diagnostics(
        selector,
        limits,
        root_state,
        joint_position,
        initial_velocity,
        candidate_acceleration,
        effort_utilization,
    )
    component_indices = np.asarray(COMPONENT_INDICES)
    predicted_component_delta = np.empty(
        (len(root_state), candidate_acceleration.shape[1], 6), np.float64
    )
    actual_component_delta = np.empty_like(predicted_component_delta)
    predicted_component_delta[:, :, :5] = (
        predicted[:, :, component_indices]
        - predicted[:, 0:1, component_indices]
    )
    actual_component_delta[:, :, :5] = (
        actual[:, :, component_indices] - actual[:, 0:1, component_indices]
    )
    # Larger joint headroom is safer, so candidate-minus-baseline harm is the
    # baseline fraction minus the candidate fraction.
    predicted_component_delta[:, :, 5] = (
        predicted[:, 0:1, HEADROOM_INDEX] - predicted[:, :, HEADROOM_INDEX]
    )
    actual_component_delta[:, :, 5] = (
        actual[:, 0:1, HEADROOM_INDEX] - actual[:, :, HEADROOM_INDEX]
    )
    predicted_aggregate_delta = (
        predicted[:, :, AGGREGATE_INDEX] - predicted[:, 0:1, AGGREGATE_INDEX]
    )
    actual_aggregate_delta = (
        actual[:, :, AGGREGATE_INDEX] - actual[:, 0:1, AGGREGATE_INDEX]
    )
    component_residual = actual_component_delta - predicted_component_delta
    aggregate_residual = actual_aggregate_delta - predicted_aggregate_delta
    groups = causal_group(root_state, initial_velocity)
    component_residual_lower, component_residual_upper, group_metrics = (
        fit_group_bounds(component_residual, groups)
    )
    aggregate_residual_lower, aggregate_residual_upper, _ = fit_group_bounds(
        aggregate_residual[:, :, None], groups
    )
    component_lower = predicted_component_delta + component_residual_lower
    component_upper = predicted_component_delta + component_residual_upper
    aggregate_lower = (
        predicted_aggregate_delta + aggregate_residual_lower[:, :, 0]
    )
    aggregate_upper = (
        predicted_aggregate_delta + aggregate_residual_upper[:, :, 0]
    )
    samples = len(groups)
    selected_index = np.empty(samples, np.uint8)
    selection = np.empty((samples, 6), np.float64)
    selection_ns = np.empty(samples, np.uint64)
    selection_calls = np.empty(samples, np.uint64)
    selection_bytes = np.empty(samples, np.uint64)
    diagnostics = np.empty((3, 14), np.float64)
    second_selection = np.empty(6, np.float64)
    available = (fixed_status <= 1).astype(np.uint8)
    semantic_repeat = np.ones(samples, np.uint8)
    for sample in range(samples):
        timing = selector.select_terminal_impact_component_delta_box_candidates(
            np.ascontiguousarray(component_lower[sample]),
            np.ascontiguousarray(component_upper[sample]),
            np.ascontiguousarray(aggregate_lower[sample]),
            np.ascontiguousarray(aggregate_upper[sample]),
            np.ascontiguousarray(available[sample]),
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
            diagnostics,
            selection[sample],
        )
        selector.select_terminal_impact_component_delta_box_candidates(
            np.ascontiguousarray(component_lower[sample]),
            np.ascontiguousarray(component_upper[sample]),
            np.ascontiguousarray(aggregate_lower[sample]),
            np.ascontiguousarray(aggregate_upper[sample]),
            np.ascontiguousarray(available[sample]),
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
            diagnostics,
            second_selection,
        )
        selected_index[sample] = int(selection[sample, 0])
        selection_ns[sample] = timing[0]
        selection_calls[sample] = timing[1]
        selection_bytes[sample] = timing[2]
        semantic_repeat[sample] = int(
            np.array_equal(selection[sample], second_selection)
        )

    # Exercise the Rust-owned paired hypothesis envelope on fixed spent rows.
    # These completed diagnostics are mechanism evidence only; the result is
    # never used to apply a plant command or admit authority.
    paired_hypotheses = np.ascontiguousarray(
        actual[np.asarray(HYPOTHESIS_INDICES, np.int64)].transpose(1, 0, 2)
    )
    hypothesis_lower = np.full((3, 6), 7.0, np.float64)
    hypothesis_upper = np.full((3, 6), 7.0, np.float64)
    hypothesis_aggregate_lower = np.full(3, 7.0, np.float64)
    hypothesis_aggregate_upper = np.full(3, 7.0, np.float64)
    hypothesis_selection = np.full(6, 7.0, np.float64)
    hypothesis_timing = hypothesis_selector.select_terminal_impact_delta_hypothesis_envelopes(
        paired_hypotheses,
        0,
        MAXIMUM_COMPONENT_REGRESSION,
        MINIMUM_COMPONENT_IMPROVEMENT,
        hypothesis_lower,
        hypothesis_upper,
        hypothesis_aggregate_lower,
        hypothesis_aggregate_upper,
        hypothesis_selection,
    )

    selected_components = actual_component_delta[
        np.arange(samples), selected_index
    ]
    selected_aggregate = actual_aggregate_delta[np.arange(samples), selected_index]
    nonzero = selected_index != 0
    actual_regression = np.max(selected_components, axis=1)
    selected_box_covered = np.all(
        (actual_component_delta >= component_lower - 1.0e-12)
        & (actual_component_delta <= component_upper + 1.0e-12)
    ) and np.all(
        (actual_aggregate_delta >= aggregate_lower - 1.0e-12)
        & (actual_aggregate_delta <= aggregate_upper + 1.0e-12)
    )
    strict_passed = bool(
        selected_box_covered
        and np.count_nonzero(nonzero) > 0
        and np.all(actual_regression <= 1.0e-12)
        and np.all(selected_aggregate <= 1.0e-12)
        and np.count_nonzero(selected_aggregate < -1.0e-12) > 0
        and np.all(selection_calls == 0)
        and np.all(selection_bytes == 0)
        and np.all(semantic_repeat == 1)
    )
    metrics = {
        "family": family,
        **group_metrics,
        "samples": samples,
        "selected_counts": {
            "zero_effort": int(np.count_nonzero(selected_index == 0)),
            "bandwidth_zero_wbc": int(np.count_nonzero(selected_index == 1)),
            "bandwidth_third_law": int(np.count_nonzero(selected_index == 2)),
        },
        "nonzero_actions": int(np.count_nonzero(nonzero)),
        "actual_component_nonregression_samples": int(
            np.count_nonzero(actual_regression <= 1.0e-12)
        ),
        "actual_aggregate_improved_actions": int(
            np.count_nonzero(nonzero & (selected_aggregate < -1.0e-12))
        ),
        "maximum_actual_component_regression": float(np.max(actual_regression)),
        "selected_aggregate_delta": distribution(selected_aggregate),
        "source_boxes_cover_all_actual_deltas": bool(selected_box_covered),
        "selector_timing_ns": distribution(selection_ns),
        "zero_rust_allocation": bool(
            np.all(selection_calls == 0) and np.all(selection_bytes == 0)
        ),
        "semantic_repeat_samples": int(np.count_nonzero(semantic_repeat)),
        "rust_hypothesis_sample_indices": HYPOTHESIS_INDICES,
        "rust_hypothesis_selection": hypothesis_selection.tolist(),
        "rust_hypothesis_delta_upper": hypothesis_upper.tolist(),
        "rust_hypothesis_aggregate_upper": hypothesis_aggregate_upper.tolist(),
        "rust_hypothesis_timing_ns": int(hypothesis_timing[0]),
        "rust_hypothesis_allocation_free": bool(hypothesis_timing[1:] == (0, 0)),
        "strict_profile_passed": strict_passed,
    }
    arrays = {
        "groups": groups,
        "predicted_diagnostics": predicted,
        "predicted_component_delta": predicted_component_delta,
        "actual_component_delta": actual_component_delta,
        "component_residual": component_residual,
        "component_residual_lower": component_residual_lower,
        "component_residual_upper": component_residual_upper,
        "component_lower": component_lower,
        "component_upper": component_upper,
        "predicted_aggregate_delta": predicted_aggregate_delta,
        "actual_aggregate_delta": actual_aggregate_delta,
        "aggregate_residual": aggregate_residual,
        "aggregate_residual_lower": aggregate_residual_lower[:, :, 0],
        "aggregate_residual_upper": aggregate_residual_upper[:, :, 0],
        "aggregate_lower": aggregate_lower,
        "aggregate_upper": aggregate_upper,
        "selected_index": selected_index,
        "selection": selection,
        "selection_ns": selection_ns,
        "semantic_repeat": semantic_repeat,
        "rust_hypothesis_selection": hypothesis_selection,
        "rust_hypothesis_delta_lower": hypothesis_lower,
        "rust_hypothesis_delta_upper": hypothesis_upper,
        "rust_hypothesis_aggregate_lower": hypothesis_aggregate_lower,
        "rust_hypothesis_aggregate_upper": hypothesis_aggregate_upper,
    }
    return metrics, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    hypothesis_model_path = pathlib.Path(args.hypothesis_model).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    source_replay_path = pathlib.Path(args.source_replay).resolve()
    if not all(
        path.is_file()
        for path in (model_path, hypothesis_model_path, source_metrics_path, source_replay_path)
    ):
        raise SystemExit("R253 requires the pinned model and immutable R250 evidence")
    source = json.loads(source_metrics_path.read_text())
    if source.get("revision") != SOURCE_REVISION:
        raise ValueError("R253 source revision mismatch")
    source_hashes = {
        "metrics": sha256(source_metrics_path),
        "replay": sha256(source_replay_path),
    }
    wbc = make_wbc_session(bonesaw, model_path)
    limits = joint_limits(model_path, list(wbc.joint_names))
    selector = bonesaw.ContactTransitionModelSession(
        str(model_path), [FOOT_FRAMES[0]] * 2 + [FOOT_FRAMES[1]] * 2
    )
    hypothesis_selector = bonesaw.UpkieBalanceSession(
        str(hypothesis_model_path)
    )
    family_metrics = []
    stored: dict[str, np.ndarray] = {}
    with np.load(source_replay_path) as replay:
        for family in FAMILY_NAMES:
            metrics, arrays = evaluate_family(
                selector, hypothesis_selector, limits, replay, family
            )
            family_metrics.append(metrics)
            stored.update({f"{family}_{key}": value for key, value in arrays.items()})
    passing = [row for row in family_metrics if row["strict_profile_passed"]]
    selected = (
        max(
            passing,
            key=lambda row: (
                row["nonzero_actions"],
                row["actual_aggregate_improved_actions"],
                row["family"] == "zero_wbc_and_neutral_recovery",
            ),
        )
        if passing
        else None
    )
    source_immutable = source_hashes == {
        "metrics": sha256(source_metrics_path),
        "replay": sha256(source_replay_path),
    }
    mechanism_passed = bool(
        source_immutable
        and all(
            row["source_boxes_cover_all_actual_deltas"]
            and row["zero_rust_allocation"]
            and row["semantic_repeat_samples"] == row["samples"]
            and row["rust_hypothesis_allocation_free"]
            and row["rust_hypothesis_selection"][0] == 0.0
            for row in family_metrics
        )
    )
    frozen = mechanism_passed and selected is not None
    frozen_profile = None
    if frozen:
        frozen_profile = {
            "family": selected["family"],
            "realization_profile": "bandwidth_25hz_slew_1000_nm_s",
            "grouping": "closing_speed_bin_x_tilt_quadrant",
            "closing_speed_bins_m_s": CLOSING_SPEED_BINS_M_S,
            "minimum_group_samples": MINIMUM_GROUP_SAMPLES,
            "maximum_component_and_aggregate_regression": MAXIMUM_COMPONENT_REGRESSION,
            "minimum_guaranteed_component_improvement": MINIMUM_COMPONENT_IMPROVEMENT,
        }
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_hashes": source_hashes,
        "source_immutable": source_immutable,
        "design_audit_not_holdout": True,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "hypothesis_model": str(hypothesis_model_path),
        "hypothesis_model_sha256": sha256(hypothesis_model_path),
        "component_names": COMPONENT_NAMES,
        "component_indices": COMPONENT_DIAGNOSTIC_INDICES,
        "headroom_loss_definition": "baseline minimum terminal joint headroom minus candidate minimum terminal joint headroom",
        "grouping": "closing_speed_bin_x_tilt_quadrant",
        "closing_speed_bins_m_s": CLOSING_SPEED_BINS_M_S,
        "minimum_group_samples": MINIMUM_GROUP_SAMPLES,
        "maximum_component_and_aggregate_regression": MAXIMUM_COMPONENT_REGRESSION,
        "minimum_guaranteed_component_improvement": MINIMUM_COMPONENT_IMPROVEMENT,
        "physics_steps": 0,
        "policy_steps": 0,
        "mechanism_passed": mechanism_passed,
        "action_profile_frozen_for_fresh_holdout": frozen,
        "frozen_profile": frozen_profile,
        "authority_admitted": False,
        "families": family_metrics,
    }
    rows = [
        [
            row["family"].replace("zero_wbc_and_", ""),
            str(row["group_count"]),
            f"{row['minimum_group_size']} / {row['fallback_rows']}",
            " / ".join(str(value) for value in row["selected_counts"].values()),
            str(row["actual_aggregate_improved_actions"]),
            f"{row['maximum_actual_component_regression']:.3g}",
            f"{row['selector_timing_ns']['p99'] / 1e3:.2f}",
            "FREEZE" if row["strict_profile_passed"] else "REJECT",
        ]
        for row in family_metrics
    ]
    report = "\n".join(
        [
            "# Bonesaw paired terminal-score freeze · r253",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · action profile **{'FROZEN' if frozen else 'NOT FOUND'}** · authority **NOT ADMITTED**.",
            "",
            "R253 predicts the post-20 ms state with fixed-effort acceleration, scores terminal consequence from that state, then fits candidate-vs-zero residual boxes for six paired consequence deltas: tilt, angular rate, joint position, joint velocity, actuator effort, and raw joint-headroom loss. Ballistic impact speed is candidate-invariant and admission is represented separately by availability. Fixed closing-speed and root-tilt bins are the only grouping features. Rust owns both the paired [3,4,17] hypothesis envelope and the exact-three-candidate delta-box selector, including zero-regression, aggregate nonregression, guaranteed-improvement, validation, atomic output, and allocation guards. Completed R250 labels fit the spent tubes; no fresh plant or authority is exercised.",
            "",
            *markdown_table(
                [
                    "family",
                    "groups",
                    "min group / fallback rows",
                    "selected zero / zero-WBC / third",
                    "actual aggregate improvements",
                    "max actual component regression",
                    "selector p99 µs",
                    "decision",
                ],
                rows,
            ),
            "",
            "A frozen row must select at least one nonzero action, cover every spent paired delta, keep all six actual consequence deltas and aggregate score nonregressing, improve aggregate score at least once, repeat bitwise, and allocate zero Rust bytes. The paired-hypothesis mechanism additionally uses four fixed completed rows and must remain baseline-safe and allocation-free. The frozen profile may now face new contact laws and offsets exactly once without refitting; this report cannot admit authority.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-paired-terminal-score-freeze-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-paired-terminal-score-freeze.npz", **stored)
    (output / "G1_PAIRED_TERMINAL_SCORE_FREEZE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw paired score freeze · r253"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "action_profile_frozen_for_fresh_holdout": frozen,
                "frozen_profile": frozen_profile,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
