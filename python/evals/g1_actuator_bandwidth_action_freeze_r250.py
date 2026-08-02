#!/usr/bin/env python3
"""R250 spent-state actuator-bandwidth action-profile rejection.

R248 is now spent evidence.  This evaluator aligns candidate zero with actual
zero plant effort, passes nonzero WBC efforts through Rust's persistent
bandwidth/slew realization, resolves each average effort through the exact
fixed-effort WBC, and then applies the conservative R224/R247 selector.  It
uses the spent R248 plant only to select one declared R62 synthetic profile,
candidate family, and minimum-improvement guard for a later fresh holdout.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_compliant_terminal_consequence_audit import ROOT_IMPACT_PLANE_M, joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_terminal_box_wbc_action_audit import (
    MAXIMUM_COMPONENT_REGRESSION,
    allocate_wbc_outputs,
    desired_candidates,
    make_wbc_session,
    run_wbc,
)
from g1_terminal_box_wbc_plant_ab import (
    CONTROL_DT,
    FRESH_PLANT_LAWS,
    HEADROOM_INDEX,
    NONREGRESSION_TOLERANCE,
    PHYSICS_DT,
    PRESSURE_INDICES,
    SAMPLE_OFFSETS,
    SOURCE_REVISION as R248_SOURCE_REVISION,
    SUBSTEPS,
    build_plant,
    copy_state,
    plant_layout,
    prepare_initial_state,
    score_terminal_state,
    warning_count,
)


REVISION = "g1-actuator-bandwidth-action-freeze-r250"
SOURCE_REVISION = "g1-terminal-box-wbc-plant-ab-r248"
SOURCE_METRICS_R248 = pathlib.Path(
    "benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/"
    "g1-terminal-box-wbc-plant-ab-metrics.json"
)
PROFILE_CASES = (("bandwidth_25hz_slew_1000_nm_s", 25.0, 1_000.0),)
FAMILY_CASES = (
    ("zero_wbc_and_velocity_damping", 1),
    ("zero_wbc_and_neutral_recovery", 2),
)
IMPROVEMENT_THRESHOLDS = (1.0e-6, 0.05, 0.10, 0.25, 0.50, 1.0, 2.0, 4.0, 8.0, 12.0)
MINIMUM_USEFUL_ACTIONS = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS_R248))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_ACTUATOR_BANDWIDTH_ACTION_FREEZE_R250.html"
    )
    return parser.parse_args()


def rollout_trace(
    model: mujoco.MjModel,
    initial: mujoco.MjData,
    joint_qvel: list[int],
    effort_trace: np.ndarray,
) -> tuple[mujoco.MjData, int]:
    data = copy_state(model, initial)
    warnings_before = warning_count(data)
    for effort in effort_trace:
        data.qfrc_applied.fill(0.0)
        data.qfrc_applied[joint_qvel] = effort
        mujoco.mj_step(model, data)
    mujoco.mj_energyVel(model, data)
    return data, warning_count(data) - warnings_before


def allocate_realization_outputs(steps: int, actuators: int) -> dict[str, np.ndarray]:
    shape = (steps, actuators)
    return {
        "limited": np.empty(shape, np.float64),
        "realized": np.empty(shape, np.float64),
        "error": np.empty(shape, np.float64),
        "clipped": np.empty(shape, np.uint8),
        "slew": np.empty(shape, np.uint8),
        "step_ns": np.empty(steps, np.uint64),
        "allocation_calls": np.empty(steps, np.uint64),
        "allocated_bytes": np.empty(steps, np.uint64),
    }


def realize_constant_request(
    session: Any,
    requested: np.ndarray,
    available: np.ndarray,
    output: dict[str, np.ndarray],
) -> None:
    session.reset(np.zeros(requested.shape[1], np.float64))
    session.run_trace(
        requested,
        available,
        PHYSICS_DT,
        output["limited"],
        output["realized"],
        output["error"],
        output["clipped"],
        output["slew"],
        output["step_ns"],
        output["allocation_calls"],
        output["allocated_bytes"],
    )


def evaluate_case(
    bonesaw: Any,
    model_path: pathlib.Path,
    profile: tuple[str, float, float],
    family: tuple[str, int],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    profile_name, bandwidth_hz, rate_nm_s = profile
    family_name, third_desired_index = family
    wbc = make_wbc_session(bonesaw, model_path)
    joint_names = list(wbc.joint_names)
    q_nominal = standing_posture(joint_names)
    limits = joint_limits(model_path, joint_names)
    effort_limits = np.asarray(wbc.actuator_effort_limits, np.float64)
    frames_per_foot = 2
    selector = bonesaw.ContactTransitionModelSession(
        str(model_path),
        [FOOT_FRAMES[0]] * frames_per_foot + [FOOT_FRAMES[1]] * frames_per_foot,
    )
    frame_ids = np.asarray(
        [list(wbc.frame_names).index(frame) for frame in FOOT_FRAMES], np.int64
    )
    profiles = np.tile(
        np.asarray([bandwidth_hz, rate_nm_s], np.float64), (wbc.dof, 1)
    )
    realization = bonesaw.ActuatorRealizationSession(
        profiles, np.zeros(wbc.dof, np.float64)
    )
    requested = np.empty((SUBSTEPS, wbc.dof), np.float64)
    available = np.broadcast_to(effort_limits, requested.shape).copy()
    realization_output = allocate_realization_outputs(SUBSTEPS, wbc.dof)
    samples = len(FRESH_PLANT_LAWS) * 48
    generalized = wbc.dof + 6
    candidates = 3
    root_state = np.empty((samples, 3), np.float64)
    joint_position = np.empty((samples, wbc.dof), np.float64)
    initial_velocity = np.empty((samples, generalized), np.float64)
    candidate_acceleration = np.empty((samples, candidates, generalized), np.float64)
    candidate_effort_utilization = np.empty((samples, candidates), np.float64)
    candidate_effort_trace = np.empty(
        (samples, candidates, SUBSTEPS, wbc.dof), np.float64
    )
    candidate_average_effort = np.empty((samples, candidates, wbc.dof), np.float64)
    actual_velocity = np.empty((samples, candidates, generalized), np.float64)
    actual_diagnostics = np.empty((samples, candidates, 17), np.float64)
    actual_energy = np.empty((samples, candidates), np.float64)
    selected_index = np.empty(samples, np.uint8)
    selection = np.empty((samples, 6), np.float64)
    component_regression = np.empty(samples, np.float64)
    harm_delta = np.empty(samples, np.float64)
    aggregate_delta = np.empty(samples, np.float64)
    energy_delta = np.empty(samples, np.float64)
    selected_effort_trace = np.empty((samples, SUBSTEPS, wbc.dof), np.float64)
    selected_average_effort = np.empty((samples, wbc.dof), np.float64)
    selected_fixed_acceleration = np.empty((samples, wbc.dof + 6), np.float64)
    raw_status = np.empty((samples, 2), np.uint8)
    fixed_status = np.empty((samples, 3), np.uint8)
    realization_step_ns = np.empty((samples, 2, SUBSTEPS), np.uint64)
    realization_slew_count = np.empty((samples, 2), np.uint16)
    plant_warning = np.empty((samples, candidates), np.uint16)
    all_wbc_allocation_free = True
    all_realization_allocation_free = True
    row = 0

    for law, offset in zip(FRESH_PLANT_LAWS, SAMPLE_OFFSETS, strict=True):
        model, initial = build_plant(model_path, law)
        root_qpos, joint_qpos, joint_qvel, _, foot_geoms, _ = plant_layout(
            model, joint_names
        )
        root_qvel = int(
            model.jnt_dofadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
            ]
        )
        for sample in range(48):
            root, quaternion, q, velocity, contacts, _, roll, pitch = (
                prepare_initial_state(
                    model,
                    initial,
                    root_qpos,
                    root_qvel,
                    joint_qpos,
                    joint_qvel,
                    foot_geoms,
                    q_nominal,
                    offset + sample,
                )
            )
            desired_root, desired_angular, desired_joint = desired_candidates(
                q[None, :], velocity[None, 6:], q_nominal, velocity[None, :3]
            )
            root_state[row] = [root[2] - ROOT_IMPACT_PLANE_M, roll, pitch]
            joint_position[row] = q
            initial_velocity[row] = velocity
            raw_outputs: list[dict[str, np.ndarray]] = []
            for slot, desired_index in enumerate((0, third_desired_index)):
                output = allocate_wbc_outputs(
                    1, wbc.dof, 8, wbc.task_diagnostic_capacity
                )
                run_wbc(
                    wbc,
                    root[None, :],
                    velocity[None, 3:6],
                    desired_root[desired_index],
                    q[None, :],
                    velocity[None, 6:],
                    desired_joint[desired_index],
                    frame_ids,
                    contacts[None, :],
                    output,
                    quaternion[None, :],
                    velocity[None, :3],
                    desired_angular[desired_index],
                )
                raw_outputs.append(output)
                raw_status[row, slot] = output["status"][0]
                all_wbc_allocation_free &= bool(
                    output["allocation_calls"][0] == 0
                    and output["allocated_bytes"][0] == 0
                )

            traces = [np.zeros((SUBSTEPS, wbc.dof), np.float64)]
            for slot, output in enumerate(raw_outputs):
                requested[:] = output["actuator_torque"][0]
                realize_constant_request(
                    realization, requested, available, realization_output
                )
                traces.append(realization_output["realized"].copy())
                realization_step_ns[row, slot] = realization_output["step_ns"]
                realization_slew_count[row, slot] = np.count_nonzero(
                    realization_output["slew"]
                )
                all_realization_allocation_free &= bool(
                    np.all(realization_output["allocation_calls"] == 0)
                    and np.all(realization_output["allocated_bytes"] == 0)
                )
            average_effort = np.stack(
                [np.mean(trace, axis=0) for trace in traces]
            )
            reference_acceleration = np.stack(
                [
                    raw_outputs[0]["generalized_acceleration"][0],
                    raw_outputs[0]["generalized_acceleration"][0],
                    raw_outputs[1]["generalized_acceleration"][0],
                ]
            )
            fixed_outputs: list[dict[str, np.ndarray]] = []
            for candidate in range(3):
                output = allocate_wbc_outputs(
                    1, wbc.dof, 8, wbc.task_diagnostic_capacity
                )
                run_wbc(
                    wbc,
                    root[None, :],
                    velocity[None, 3:6],
                    desired_root[0],
                    q[None, :],
                    velocity[None, 6:],
                    desired_joint[0],
                    frame_ids,
                    contacts[None, :],
                    output,
                    quaternion[None, :],
                    velocity[None, :3],
                    desired_angular[0],
                    average_effort[candidate][None, :],
                    reference_acceleration[candidate][None, :],
                )
                fixed_outputs.append(output)
                fixed_status[row, candidate] = output["status"][0]
                all_wbc_allocation_free &= bool(
                    output["allocation_calls"][0] == 0
                    and output["allocated_bytes"][0] == 0
                )
            acceleration = np.stack(
                [output["generalized_acceleration"][0] for output in fixed_outputs]
            )
            effort_utilization = np.max(
                np.abs(average_effort) / effort_limits[None, :], axis=1
            )
            candidate_acceleration[row] = acceleration
            candidate_effort_utilization[row] = effort_utilization
            candidate_effort_trace[row] = traces
            candidate_average_effort[row] = average_effort
            for candidate, trace in enumerate(traces):
                terminal, warnings = rollout_trace(
                    model, initial, joint_qvel, trace
                )
                plant_warning[row, candidate] = warnings
                diagnostics, terminal_velocity = score_terminal_state(
                    selector,
                    terminal,
                    root_qpos,
                    root_qvel,
                    joint_qpos,
                    joint_qvel,
                    limits,
                    float(effort_utilization[candidate]),
                )
                actual_diagnostics[row, candidate] = diagnostics
                actual_velocity[row, candidate] = terminal_velocity
                actual_energy[row, candidate] = terminal.energy[1]
            row += 1

    # R248 is spent design evidence.  Fit one deterministic componentwise
    # residual tube per candidate over all 96 states, then use only those
    # global tubes for the state-local second pass.  The fresh holdout may
    # consume the frozen tubes but may not refit them.
    predicted_center = (
        initial_velocity[:, None, :] + CONTROL_DT * candidate_acceleration
    )
    plant_residual = actual_velocity - predicted_center
    residual_lower = np.min(plant_residual, axis=0)
    residual_upper = np.max(plant_residual, axis=0)
    residual_span = residual_upper - residual_lower
    diagnostics = np.empty((candidates, 17), np.float64)
    available = (fixed_status <= 1).astype(np.uint8)
    for sample in range(samples):
        lower = predicted_center[sample] + residual_lower
        upper = predicted_center[sample] + residual_upper
        selector.select_terminal_impact_velocity_box_candidates(
            root_state[sample],
            np.ascontiguousarray(lower[:, [5, 0, 1]]),
            np.ascontiguousarray(upper[:, [5, 0, 1]]),
            joint_position[sample],
            np.ascontiguousarray(lower[:, 6:]),
            np.ascontiguousarray(upper[:, 6:]),
            limits[0],
            limits[1],
            limits[2],
            available[sample],
            np.ascontiguousarray(candidate_acceleration[sample, :, :2]),
            np.ascontiguousarray(candidate_acceleration[sample, :, 6:]),
            candidate_effort_utilization[sample],
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            IMPROVEMENT_THRESHOLDS[0],
            diagnostics,
            selection[sample],
        )
        chosen = int(selection[sample, 0])
        selected_index[sample] = chosen
        selected_effort_trace[sample] = candidate_effort_trace[sample, chosen]
        selected_average_effort[sample] = candidate_average_effort[sample, chosen]
        selected_fixed_acceleration[sample] = candidate_acceleration[sample, chosen]
        baseline_diagnostics = actual_diagnostics[sample, 0]
        candidate_diagnostics = actual_diagnostics[sample, chosen]
        pressure_delta = (
            candidate_diagnostics[list(PRESSURE_INDICES)]
            - baseline_diagnostics[list(PRESSURE_INDICES)]
        )
        headroom_delta = (
            candidate_diagnostics[HEADROOM_INDEX]
            - baseline_diagnostics[HEADROOM_INDEX]
        )
        component_regression[sample] = max(
            float(np.max(pressure_delta)), float(-headroom_delta)
        )
        harm_delta[sample] = candidate_diagnostics[15] - baseline_diagnostics[15]
        aggregate_delta[sample] = (
            candidate_diagnostics[16] - baseline_diagnostics[16]
        )
        energy_delta[sample] = actual_energy[sample, chosen] - actual_energy[sample, 0]

    threshold_curve = []
    for threshold in IMPROVEMENT_THRESHOLDS:
        active = (selected_index != 0) & (selection[:, 5] >= threshold)
        effective_regression = np.where(active, component_regression, 0.0)
        effective_harm_delta = np.where(active, harm_delta, 0.0)
        effective_aggregate_delta = np.where(active, aggregate_delta, 0.0)
        safe = effective_regression <= NONREGRESSION_TOLERANCE
        improved = active & (effective_aggregate_delta < -NONREGRESSION_TOLERANCE)
        threshold_curve.append(
            {
                "minimum_component_improvement": threshold,
                "nonzero_actions": int(np.count_nonzero(active)),
                "strict_nonregression_samples": int(np.count_nonzero(safe)),
                "regression_samples": int(np.count_nonzero(~safe)),
                "actually_improved_actions": int(np.count_nonzero(improved)),
                "maximum_component_regression": float(
                    np.max(effective_regression)
                ),
                "aggregate_score_delta": distribution(effective_aggregate_delta),
                "harm_delta": distribution(effective_harm_delta),
            }
        )
    passing_thresholds = [
        row
        for row in threshold_curve
        if row["regression_samples"] == 0
        and row["nonzero_actions"] >= MINIMUM_USEFUL_ACTIONS
        and row["actually_improved_actions"] >= MINIMUM_USEFUL_ACTIONS
    ]
    best_threshold = (
        max(
            passing_thresholds,
            key=lambda row: (
                row["nonzero_actions"],
                row["actually_improved_actions"],
                -row["minimum_component_improvement"],
            ),
        )
        if passing_thresholds
        else None
    )
    metrics = {
        "profile": profile_name,
        "bandwidth_hz": bandwidth_hz,
        "maximum_effort_rate_nm_per_s": rate_nm_s,
        "family": family_name,
        "third_desired_index": third_desired_index,
        "samples": samples,
        "unthresholded_selected_counts": {
            "zero_effort": int(np.count_nonzero(selected_index == 0)),
            "bandwidth_zero_wbc": int(np.count_nonzero(selected_index == 1)),
            "bandwidth_third_law": int(np.count_nonzero(selected_index == 2)),
        },
        "unthresholded_component_improvement": distribution(selection[:, 5]),
        "unthresholded_component_regression": distribution(component_regression),
        "all_raw_wbc_admitted": bool(np.all(raw_status <= 1)),
        "all_fixed_effort_wbc_admitted": bool(np.all(fixed_status <= 1)),
        "zero_wbc_rust_allocation": all_wbc_allocation_free,
        "zero_realization_rust_allocation": all_realization_allocation_free,
        "realization_step_timing_ns": distribution(
            realization_step_ns.reshape(-1)
        ),
        "slew_limited_coordinate_steps": int(np.sum(realization_slew_count)),
        "mujoco_warning_count": int(np.sum(plant_warning)),
        "plant_residual_tube": {
            "fit_samples": samples,
            "fit_candidates": candidates,
            "maximum_span": float(np.max(residual_span)),
            "root_angular_maximum_span_rad_s": float(
                np.max(residual_span[:, :3])
            ),
            "root_linear_maximum_span_m_s": float(
                np.max(residual_span[:, 3:6])
            ),
            "joint_maximum_span_rad_s": float(
                np.max(residual_span[:, 6:])
            ),
        },
        "threshold_curve": threshold_curve,
        "passing_threshold": best_threshold,
        "profile_family_passed": best_threshold is not None,
    }
    arrays = {
        "selected_index": selected_index,
        "selection": selection,
        "component_regression": component_regression,
        "harm_delta": harm_delta,
        "aggregate_delta": aggregate_delta,
        "energy_delta": energy_delta,
        "selected_effort_trace": selected_effort_trace,
        "selected_average_effort": selected_average_effort,
        "selected_fixed_acceleration": selected_fixed_acceleration,
        "raw_status": raw_status,
        "fixed_status": fixed_status,
        "realization_step_ns": realization_step_ns,
        "realization_slew_count": realization_slew_count,
        "plant_warning": plant_warning,
        "root_state": root_state,
        "joint_position": joint_position,
        "initial_velocity": initial_velocity,
        "candidate_acceleration": candidate_acceleration,
        "candidate_effort_utilization": candidate_effort_utilization,
        "candidate_effort_trace": candidate_effort_trace,
        "candidate_average_effort": candidate_average_effort,
        "actual_velocity": actual_velocity,
        "actual_diagnostics": actual_diagnostics,
        "actual_energy": actual_energy,
        "plant_residual": plant_residual,
        "residual_lower": residual_lower,
        "residual_upper": residual_upper,
    }
    return metrics, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    if not model_path.is_file() or not source_metrics_path.is_file():
        raise SystemExit("R250 requires the pinned G1 model and immutable R248 metrics")
    source = json.loads(source_metrics_path.read_text())
    if source["revision"] != SOURCE_REVISION or source["strict_plant_nonregression_passed"]:
        raise ValueError("R250 requires the rejected, now-spent R248 plant evidence")
    source_hash_before = sha256(source_metrics_path)
    cases: list[dict[str, Any]] = []
    stored: dict[str, np.ndarray] = {}
    for profile in PROFILE_CASES:
        for family in FAMILY_CASES:
            metrics, arrays = evaluate_case(bonesaw, model_path, profile, family)
            cases.append(metrics)
            prefix = f"{profile[0]}_{family[0]}"
            stored.update({f"{prefix}_{name}": value for name, value in arrays.items()})
    passing = [case for case in cases if case["profile_family_passed"]]
    selected = (
        max(
            passing,
            key=lambda case: (
                case["passing_threshold"]["nonzero_actions"],
                case["passing_threshold"]["actually_improved_actions"],
                -case["bandwidth_hz"],
            ),
        )
        if passing
        else None
    )
    source_immutable = source_hash_before == sha256(source_metrics_path)
    mechanism_passed = source_immutable and all(
        case["all_raw_wbc_admitted"]
        and case["all_fixed_effort_wbc_admitted"]
        and case["zero_wbc_rust_allocation"]
        and case["zero_realization_rust_allocation"]
        and case["mujoco_warning_count"] == 0
        for case in cases
    )
    profile_frozen = mechanism_passed and selected is not None
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_metrics_sha256": source_hash_before,
        "source_immutable": source_immutable,
        "design_audit_not_holdout": True,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "profile_cases": PROFILE_CASES,
        "family_cases": FAMILY_CASES,
        "improvement_thresholds": IMPROVEMENT_THRESHOLDS,
        "minimum_useful_actions": MINIMUM_USEFUL_ACTIONS,
        "physics_steps": len(cases) * len(FRESH_PLANT_LAWS) * 48 * 3 * SUBSTEPS,
        "policy_steps": 0,
        "mechanism_passed": mechanism_passed,
        "action_profile_frozen_for_fresh_holdout": profile_frozen,
        "selected_profile": selected,
        "authority_admitted": False,
        "cases": cases,
    }
    rows = []
    for case in cases:
        threshold = case["passing_threshold"]
        rows.append(
            [
                case["profile"],
                case["family"],
                " / ".join(
                    str(case["unthresholded_selected_counts"][name])
                    for name in (
                        "zero_effort",
                        "bandwidth_zero_wbc",
                        "bandwidth_third_law",
                    )
                ),
                "NONE" if threshold is None else f"{threshold['minimum_component_improvement']:.3g}",
                "0 / 0"
                if threshold is None
                else f"{threshold['nonzero_actions']} / {threshold['actually_improved_actions']}",
                "PASS" if threshold is not None else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw actuator-bandwidth action freeze · r250",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · useful safe profile **{'FROZEN' if profile_frozen else 'NOT FOUND'}** · authority **NOT ADMITTED**.",
            "",
            "The rejected R248 states are deliberately spent design evidence. Candidate zero is now exact fixed-zero-effort WBC realization. Two nonzero candidates pass the frozen WBC effort through Rust's persistent bandwidth/slew model at each plant substep, then exact fixed-effort WBC predicts the average realized acceleration. All three candidates are rolled out on the spent states to fit one global componentwise plant-residual tube per candidate. The state-local Rust selector sees only those frozen tubes; a minimum-improvement guard may only fall back to zero and cannot switch to an outcome-fitted action.",
            "",
            *markdown_table(
                [
                    "profile",
                    "family",
                    "raw selected · zero / zero-WBC / third",
                    "safe threshold",
                    "nonzero / actually improved",
                    "decision",
                ],
                rows,
            ),
            "",
            "A profile passes only with zero component regressions on all 96 spent plant states, at least one nonzero action, at least one actual aggregate-score improvement, zero MuJoCo warnings, admitted raw and fixed-effort WBC, and zero Rust allocation in WBC and realization hot paths. Any frozen profile must next face new laws and offsets without retuning; this report cannot admit authority.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-actuator-bandwidth-action-freeze-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-actuator-bandwidth-action-freeze.npz", **stored)
    (output / "G1_ACTUATOR_BANDWIDTH_ACTION_FREEZE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw actuator-bandwidth freeze · r250"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "action_profile_frozen_for_fresh_holdout": profile_frozen,
                "selected_profile": None
                if selected is None
                else {
                    "profile": selected["profile"],
                    "family": selected["family"],
                    "threshold": selected["passing_threshold"][
                        "minimum_component_improvement"
                    ],
                },
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
