#!/usr/bin/env python3
"""R254 one-shot no-refit plant holdout for the frozen R253 profile.

The laws, offsets, 250/50 cadence, frozen family, residual-group lookup, and
strict gates are declared below before this revision generates any labels.
R253's residual boxes are consumed byte-for-byte; this evaluator contains no
fit, widening, or threshold-search path.
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
from g1_actuator_bandwidth_action_freeze_r250 import (
    allocate_realization_outputs,
    realize_constant_request,
    rollout_trace,
)
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    ContactLaw,
    sha256,
    standing_posture,
)
from g1_paired_terminal_score_freeze_r253 import (
    causal_group,
    predicted_terminal_diagnostics,
)
from g1_terminal_box_wbc_action_audit import (
    QUERY_DEADLINE_NS,
    allocate_wbc_outputs,
    desired_candidates,
    make_wbc_session,
    run_wbc,
)
from g1_terminal_box_wbc_plant_ab import (
    PHYSICS_DT,
    SUBSTEPS,
    build_plant,
    plant_layout,
    prepare_initial_state,
    score_terminal_state,
)


REVISION = "g1-paired-terminal-score-plant-holdout-r254"
SOURCE_REVISION = "g1-paired-terminal-score-freeze-r253"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-freeze-r253/"
    "g1-paired-terminal-score-freeze-metrics.json"
)
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-freeze-r253/"
    "g1-paired-terminal-score-freeze.npz"
)
FROZEN_FAMILY = "zero_wbc_and_neutral_recovery"
FROZEN_THIRD_DESIRED_INDEX = 2
FROZEN_BANDWIDTH_HZ = 25.0
FROZEN_MAXIMUM_EFFORT_RATE_NM_S = 1_000.0
FRESH_PLANT_LAWS = (
    ContactLaw(
        "compliant_elliptic_implicitfast_paired_r254",
        friction=0.73,
        solref_time_s=0.0041,
        solimp_min=0.972,
        solimp_max=0.996,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "stiff_pyramidal_euler_paired_r254",
        friction=1.05,
        solref_time_s=0.0017,
        solimp_min=0.993,
        solimp_max=0.9993,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_EULER),
    ),
)
SAMPLE_OFFSETS = (310_000, 320_000)
SAMPLES_PER_LAW = 48
CONTROL_DT_S = 0.020
NONREGRESSION_TOLERANCE = 1.0e-12
CANDIDATES = 3
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
AGGREGATE_INDEX = 16
MAXIMUM_COMPONENT_REGRESSION = 0.0
MINIMUM_COMPONENT_IMPROVEMENT = 0.01
COMPONENTS = len(COMPONENT_NAMES)
GROUPS = 16


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_PAIRED_TERMINAL_SCORE_PLANT_HOLDOUT_R254.html",
    )
    return parser.parse_args()


def frozen_group_bounds(
    replay: Any,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    prefix = FROZEN_FAMILY + "_"
    source_groups = np.asarray(replay[prefix + "groups"], np.int64)
    source_component_lower = np.asarray(
        replay[prefix + "component_residual_lower"], np.float64
    )
    source_component_upper = np.asarray(
        replay[prefix + "component_residual_upper"], np.float64
    )
    source_aggregate_lower = np.asarray(
        replay[prefix + "aggregate_residual_lower"], np.float64
    )
    source_aggregate_upper = np.asarray(
        replay[prefix + "aggregate_residual_upper"], np.float64
    )
    component_lower = np.full((GROUPS, CANDIDATES, COMPONENTS), np.nan, np.float64)
    component_upper = np.full_like(component_lower, np.nan)
    aggregate_lower = np.full((GROUPS, CANDIDATES), np.nan, np.float64)
    aggregate_upper = np.full_like(aggregate_lower, np.nan)
    supported = np.zeros(GROUPS, np.uint8)
    for group in np.unique(source_groups):
        if group < 0 or group >= GROUPS:
            raise ValueError("R253 group id is outside the frozen 16-cell layout")
        members = np.flatnonzero(source_groups == group)
        first = int(members[0])
        for source in (
            source_component_lower,
            source_component_upper,
            source_aggregate_lower,
            source_aggregate_upper,
        ):
            if not np.all(source[members] == source[first]):
                raise ValueError("R253 replay does not contain one immutable box per group")
        component_lower[group] = source_component_lower[first]
        component_upper[group] = source_component_upper[first]
        aggregate_lower[group] = source_aggregate_lower[first]
        aggregate_upper[group] = source_aggregate_upper[first]
        supported[group] = 1
    return (
        component_lower,
        component_upper,
        aggregate_lower,
        aggregate_upper,
        supported,
    )


def component_deltas(diagnostics: np.ndarray) -> np.ndarray:
    indices = np.asarray(COMPONENT_INDICES, np.int64)
    delta = np.empty((*diagnostics.shape[:2], COMPONENTS), np.float64)
    delta[:, :, : len(indices)] = (
        diagnostics[:, :, indices] - diagnostics[:, 0:1, indices]
    )
    delta[:, :, len(indices)] = (
        diagnostics[:, 0:1, HEADROOM_INDEX]
        - diagnostics[:, :, HEADROOM_INDEX]
    )
    return delta


def main() -> int:
    import bonesaw

    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    source_replay_path = pathlib.Path(args.source_replay).resolve()
    if not all(
        path.is_file() for path in (model_path, source_metrics_path, source_replay_path)
    ):
        raise SystemExit("R254 requires the pinned model and immutable R253 evidence")
    source = json.loads(source_metrics_path.read_text())
    if (
        source.get("revision") != SOURCE_REVISION
        or not source.get("action_profile_frozen_for_fresh_holdout")
        or source.get("frozen_profile", {}).get("family") != FROZEN_FAMILY
    ):
        raise ValueError("R254 requires the frozen R253 neutral-recovery profile")
    source_hashes = {
        "metrics": sha256(source_metrics_path),
        "replay": sha256(source_replay_path),
    }
    with np.load(source_replay_path) as frozen_replay:
        (
            group_component_residual_lower,
            group_component_residual_upper,
            group_aggregate_residual_lower,
            group_aggregate_residual_upper,
            group_supported,
        ) = frozen_group_bounds(frozen_replay)

    wbc = make_wbc_session(bonesaw, model_path)
    joint_names = list(wbc.joint_names)
    q_nominal = standing_posture(joint_names)
    limits = joint_limits(model_path, joint_names)
    effort_limits = np.asarray(wbc.actuator_effort_limits, np.float64)
    frame_ids = np.asarray(
        [list(wbc.frame_names).index(frame) for frame in FOOT_FRAMES], np.int64
    )
    selector = bonesaw.ContactTransitionModelSession(
        str(model_path), [FOOT_FRAMES[0]] * 2 + [FOOT_FRAMES[1]] * 2
    )
    profiles = np.tile(
        np.asarray(
            [FROZEN_BANDWIDTH_HZ, FROZEN_MAXIMUM_EFFORT_RATE_NM_S], np.float64
        ),
        (wbc.dof, 1),
    )
    realization = bonesaw.ActuatorRealizationSession(
        profiles, np.zeros(wbc.dof, np.float64)
    )
    requested = np.empty((SUBSTEPS, wbc.dof), np.float64)
    available_effort = np.broadcast_to(effort_limits, requested.shape).copy()
    realization_output = allocate_realization_outputs(SUBSTEPS, wbc.dof)

    samples = len(FRESH_PLANT_LAWS) * SAMPLES_PER_LAW
    generalized = wbc.dof + 6
    root_state = np.empty((samples, 3), np.float64)
    joint_position = np.empty((samples, wbc.dof), np.float64)
    initial_velocity = np.empty((samples, generalized), np.float64)
    candidate_acceleration = np.empty((samples, CANDIDATES, generalized), np.float64)
    candidate_effort_utilization = np.empty((samples, CANDIDATES), np.float64)
    candidate_effort_trace = np.empty(
        (samples, CANDIDATES, SUBSTEPS, wbc.dof), np.float64
    )
    actual_diagnostics = np.empty((samples, CANDIDATES, 17), np.float64)
    actual_energy = np.empty((samples, CANDIDATES), np.float64)
    raw_status = np.empty((samples, 2), np.uint8)
    fixed_status = np.empty((samples, CANDIDATES), np.uint8)
    wbc_ns = np.empty((samples, 5), np.uint64)
    realization_ns = np.empty((samples, 2, SUBSTEPS), np.uint64)
    realization_slew = np.empty((samples, 2), np.uint16)
    plant_warning = np.empty((samples, CANDIDATES), np.uint16)
    law_index = np.empty(samples, np.uint8)
    all_wbc_allocation_free = True
    all_realization_allocation_free = True
    row = 0

    for law_slot, (law, offset) in enumerate(
        zip(FRESH_PLANT_LAWS, SAMPLE_OFFSETS, strict=True)
    ):
        model, initial = build_plant(model_path, law)
        root_qpos, joint_qpos, joint_qvel, _, foot_geoms, _ = plant_layout(
            model, joint_names
        )
        root_qvel = int(
            model.jnt_dofadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
            ]
        )
        for sample in range(SAMPLES_PER_LAW):
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
            law_index[row] = law_slot
            root_state[row] = [root[2] - 0.45, roll, pitch]
            joint_position[row] = q
            initial_velocity[row] = velocity
            desired_root, desired_angular, desired_joint = desired_candidates(
                q[None, :], velocity[None, 6:], q_nominal, velocity[None, :3]
            )
            raw_outputs: list[dict[str, np.ndarray]] = []
            for slot, desired_index in enumerate((0, FROZEN_THIRD_DESIRED_INDEX)):
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
                wbc_ns[row, slot] = output["step_ns"][0]
                all_wbc_allocation_free &= bool(
                    output["allocation_calls"][0] == 0
                    and output["allocated_bytes"][0] == 0
                )

            traces = [np.zeros((SUBSTEPS, wbc.dof), np.float64)]
            for slot, output in enumerate(raw_outputs):
                requested[:] = output["actuator_torque"][0]
                realize_constant_request(
                    realization, requested, available_effort, realization_output
                )
                traces.append(realization_output["realized"].copy())
                realization_ns[row, slot] = realization_output["step_ns"]
                realization_slew[row, slot] = np.count_nonzero(
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
            for candidate in range(CANDIDATES):
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
                wbc_ns[row, 2 + candidate] = output["step_ns"][0]
                all_wbc_allocation_free &= bool(
                    output["allocation_calls"][0] == 0
                    and output["allocated_bytes"][0] == 0
                )
            candidate_acceleration[row] = np.stack(
                [output["generalized_acceleration"][0] for output in fixed_outputs]
            )
            candidate_effort_utilization[row] = np.max(
                np.abs(average_effort) / effort_limits[None, :], axis=1
            )
            candidate_effort_trace[row] = traces
            for candidate, trace in enumerate(traces):
                terminal, warnings = rollout_trace(model, initial, joint_qvel, trace)
                plant_warning[row, candidate] = warnings
                diagnostics, _ = score_terminal_state(
                    selector,
                    terminal,
                    root_qpos,
                    root_qvel,
                    joint_qpos,
                    joint_qvel,
                    limits,
                    float(candidate_effort_utilization[row, candidate]),
                )
                actual_diagnostics[row, candidate] = diagnostics
                actual_energy[row, candidate] = terminal.energy[1]
            row += 1

    predicted_diagnostics = predicted_terminal_diagnostics(
        selector,
        limits,
        root_state,
        joint_position,
        initial_velocity,
        candidate_acceleration,
        candidate_effort_utilization,
    )
    predicted_component_delta = component_deltas(predicted_diagnostics)
    actual_component_delta = component_deltas(actual_diagnostics)
    predicted_aggregate_delta = (
        predicted_diagnostics[:, :, AGGREGATE_INDEX]
        - predicted_diagnostics[:, 0:1, AGGREGATE_INDEX]
    )
    actual_aggregate_delta = (
        actual_diagnostics[:, :, AGGREGATE_INDEX]
        - actual_diagnostics[:, 0:1, AGGREGATE_INDEX]
    )
    groups = causal_group(root_state, initial_velocity)
    fresh_group_supported = group_supported[groups] != 0
    component_lower = np.zeros_like(predicted_component_delta)
    component_upper = np.zeros_like(predicted_component_delta)
    aggregate_lower = np.zeros_like(predicted_aggregate_delta)
    aggregate_upper = np.zeros_like(predicted_aggregate_delta)
    for sample, group in enumerate(groups):
        if fresh_group_supported[sample]:
            component_lower[sample] = (
                predicted_component_delta[sample]
                + group_component_residual_lower[group]
            )
            component_upper[sample] = (
                predicted_component_delta[sample]
                + group_component_residual_upper[group]
            )
            aggregate_lower[sample] = (
                predicted_aggregate_delta[sample]
                + group_aggregate_residual_lower[group]
            )
            aggregate_upper[sample] = (
                predicted_aggregate_delta[sample]
                + group_aggregate_residual_upper[group]
            )

    selected_index = np.empty(samples, np.uint8)
    selection = np.empty((samples, 6), np.float64)
    second_selection = np.empty(6, np.float64)
    selector_ns = np.empty(samples, np.uint64)
    selector_calls = np.empty(samples, np.uint64)
    selector_bytes = np.empty(samples, np.uint64)
    semantic_repeat = np.empty(samples, np.uint8)
    selector_diagnostics = np.empty((CANDIDATES, 2 * COMPONENTS + 2), np.float64)
    for sample in range(samples):
        candidate_available = (fixed_status[sample] <= 1).astype(np.uint8)
        if not fresh_group_supported[sample]:
            candidate_available[1:] = 0
        timing = selector.select_terminal_impact_component_delta_box_candidates(
            np.ascontiguousarray(component_lower[sample]),
            np.ascontiguousarray(component_upper[sample]),
            np.ascontiguousarray(aggregate_lower[sample]),
            np.ascontiguousarray(aggregate_upper[sample]),
            candidate_available,
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
            selector_diagnostics,
            selection[sample],
        )
        selector.select_terminal_impact_component_delta_box_candidates(
            np.ascontiguousarray(component_lower[sample]),
            np.ascontiguousarray(component_upper[sample]),
            np.ascontiguousarray(aggregate_lower[sample]),
            np.ascontiguousarray(aggregate_upper[sample]),
            candidate_available,
            0,
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
            selector_diagnostics,
            second_selection,
        )
        selected_index[sample] = int(selection[sample, 0])
        selector_ns[sample] = timing[0]
        selector_calls[sample] = timing[1]
        selector_bytes[sample] = timing[2]
        semantic_repeat[sample] = int(
            np.array_equal(selection[sample], second_selection)
        )

    all_candidate_boxes_covered = np.all(
        (actual_component_delta >= component_lower - NONREGRESSION_TOLERANCE)
        & (actual_component_delta <= component_upper + NONREGRESSION_TOLERANCE)
    ) and np.all(
        (actual_aggregate_delta >= aggregate_lower - NONREGRESSION_TOLERANCE)
        & (actual_aggregate_delta <= aggregate_upper + NONREGRESSION_TOLERANCE)
    )
    component_coverage_miss = np.maximum(
        np.maximum(component_lower - actual_component_delta, 0.0),
        np.maximum(actual_component_delta - component_upper, 0.0),
    )
    aggregate_coverage_miss = np.maximum(
        np.maximum(aggregate_lower - actual_aggregate_delta, 0.0),
        np.maximum(actual_aggregate_delta - aggregate_upper, 0.0),
    )
    selected_component_delta = actual_component_delta[
        np.arange(samples), selected_index
    ]
    selected_aggregate_delta = actual_aggregate_delta[
        np.arange(samples), selected_index
    ]
    selected_energy_delta = (
        actual_energy[np.arange(samples), selected_index] - actual_energy[:, 0]
    )
    selected_maximum_component_regression = np.max(
        selected_component_delta, axis=1
    )
    nonzero = selected_index != 0
    source_immutable = source_hashes == {
        "metrics": sha256(source_metrics_path),
        "replay": sha256(source_replay_path),
    }
    deadline_passed = bool(
        np.percentile(wbc_ns, 99) <= QUERY_DEADLINE_NS
        and np.percentile(selector_ns, 99) <= QUERY_DEADLINE_NS
    )
    mechanism_passed = bool(
        source_immutable
        and np.all(fresh_group_supported)
        and np.all(raw_status <= 1)
        and np.all(fixed_status <= 1)
        and all_wbc_allocation_free
        and all_realization_allocation_free
        and np.all(selector_calls == 0)
        and np.all(selector_bytes == 0)
        and np.all(semantic_repeat == 1)
        and np.sum(plant_warning) == 0
        and deadline_passed
    )
    profile_transferred = bool(
        mechanism_passed
        and all_candidate_boxes_covered
        and np.count_nonzero(nonzero) > 0
        and np.all(
            selected_maximum_component_regression <= NONREGRESSION_TOLERANCE
        )
        and np.all(selected_aggregate_delta <= NONREGRESSION_TOLERANCE)
        and np.all(
            selected_aggregate_delta[nonzero] < -NONREGRESSION_TOLERANCE
        )
    )

    law_metrics = []
    for slot, law in enumerate(FRESH_PLANT_LAWS):
        mask = law_index == slot
        law_nonzero = mask & nonzero
        law_metrics.append(
            {
                "law": law.name,
                "sample_offset": SAMPLE_OFFSETS[slot],
                "samples": int(np.count_nonzero(mask)),
                "selected_counts": {
                    "zero_effort": int(np.count_nonzero(mask & (selected_index == 0))),
                    "bandwidth_zero_wbc": int(
                        np.count_nonzero(mask & (selected_index == 1))
                    ),
                    "bandwidth_neutral_recovery": int(
                        np.count_nonzero(mask & (selected_index == 2))
                    ),
                },
                "nonzero_actions": int(np.count_nonzero(law_nonzero)),
                "improved_nonzero_actions": int(
                    np.count_nonzero(
                        law_nonzero
                        & (selected_aggregate_delta < -NONREGRESSION_TOLERANCE)
                    )
                ),
                "strict_nonregressing_improved_nonzero_actions": int(
                    np.count_nonzero(
                        law_nonzero
                        & (
                            selected_maximum_component_regression
                            <= NONREGRESSION_TOLERANCE
                        )
                        & (selected_aggregate_delta < -NONREGRESSION_TOLERANCE)
                    )
                ),
                "selected_component_regression_samples": int(
                    np.count_nonzero(
                        mask
                        & (
                            selected_maximum_component_regression
                            > NONREGRESSION_TOLERANCE
                        )
                    )
                ),
                "selected_aggregate_regression_samples": int(
                    np.count_nonzero(
                        mask
                        & (selected_aggregate_delta > NONREGRESSION_TOLERANCE)
                    )
                ),
                "maximum_selected_component_regression": float(
                    np.max(selected_maximum_component_regression[mask])
                ),
                "selected_aggregate_delta": distribution(
                    selected_aggregate_delta[mask]
                ),
                "selected_energy_delta_j": distribution(selected_energy_delta[mask]),
                "all_candidate_boxes_covered": bool(
                    np.all(
                        (actual_component_delta[mask]
                         >= component_lower[mask] - NONREGRESSION_TOLERANCE)
                        & (actual_component_delta[mask]
                           <= component_upper[mask] + NONREGRESSION_TOLERANCE)
                    )
                    and np.all(
                        (actual_aggregate_delta[mask]
                         >= aggregate_lower[mask] - NONREGRESSION_TOLERANCE)
                        & (actual_aggregate_delta[mask]
                           <= aggregate_upper[mask] + NONREGRESSION_TOLERANCE)
                    )
                ),
                "component_box_misses": {
                    name: {
                        "candidate_values": int(
                            np.count_nonzero(
                                component_coverage_miss[mask, :, component]
                                > NONREGRESSION_TOLERANCE
                            )
                        ),
                        "rows": int(
                            np.count_nonzero(
                                np.any(
                                    component_coverage_miss[mask, :, component]
                                    > NONREGRESSION_TOLERANCE,
                                    axis=1,
                                )
                            )
                        ),
                        "maximum": float(
                            np.max(component_coverage_miss[mask, :, component])
                        ),
                    }
                    for component, name in enumerate(COMPONENT_NAMES)
                },
                "aggregate_box_misses": {
                    "candidate_values": int(
                        np.count_nonzero(
                            aggregate_coverage_miss[mask]
                            > NONREGRESSION_TOLERANCE
                        )
                    ),
                    "rows": int(
                        np.count_nonzero(
                            np.any(
                                aggregate_coverage_miss[mask]
                                > NONREGRESSION_TOLERANCE,
                                axis=1,
                            )
                        )
                    ),
                    "maximum": float(np.max(aggregate_coverage_miss[mask])),
                },
                "mujoco_warning_count": int(np.sum(plant_warning[mask])),
            }
        )

    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_hashes": source_hashes,
        "source_immutable": source_immutable,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "fresh_laws_and_state_offsets": True,
        "no_refit": True,
        "frozen_family": FROZEN_FAMILY,
        "component_names": COMPONENT_NAMES,
        "samples": samples,
        "physics_steps": samples * CANDIDATES * SUBSTEPS,
        "policy_steps": 0,
        "fresh_group_coverage_samples": int(np.count_nonzero(fresh_group_supported)),
        "all_raw_wbc_admitted": bool(np.all(raw_status <= 1)),
        "all_fixed_effort_wbc_admitted": bool(np.all(fixed_status <= 1)),
        "zero_wbc_rust_allocation": all_wbc_allocation_free,
        "zero_realization_rust_allocation": all_realization_allocation_free,
        "zero_selector_rust_allocation": bool(
            np.all(selector_calls == 0) and np.all(selector_bytes == 0)
        ),
        "semantic_repeat_samples": int(np.count_nonzero(semantic_repeat)),
        "wbc_timing_ns": distribution(wbc_ns.reshape(-1)),
        "realization_timing_ns": distribution(realization_ns.reshape(-1)),
        "selector_timing_ns": distribution(selector_ns),
        "deadline_passed": deadline_passed,
        "slew_limited_coordinate_steps": int(np.sum(realization_slew)),
        "mujoco_warning_count": int(np.sum(plant_warning)),
        "all_candidate_boxes_covered": bool(all_candidate_boxes_covered),
        "selected_counts": {
            "zero_effort": int(np.count_nonzero(selected_index == 0)),
            "bandwidth_zero_wbc": int(np.count_nonzero(selected_index == 1)),
            "bandwidth_neutral_recovery": int(np.count_nonzero(selected_index == 2)),
        },
        "nonzero_actions": int(np.count_nonzero(nonzero)),
        "improved_nonzero_actions": int(
            np.count_nonzero(
                nonzero & (selected_aggregate_delta < -NONREGRESSION_TOLERANCE)
            )
        ),
        "strict_nonregressing_improved_nonzero_actions": int(
            np.count_nonzero(
                nonzero
                & (
                    selected_maximum_component_regression
                    <= NONREGRESSION_TOLERANCE
                )
                & (selected_aggregate_delta < -NONREGRESSION_TOLERANCE)
            )
        ),
        "selected_component_regression_samples": int(
            np.count_nonzero(
                selected_maximum_component_regression > NONREGRESSION_TOLERANCE
            )
        ),
        "selected_aggregate_regression_samples": int(
            np.count_nonzero(selected_aggregate_delta > NONREGRESSION_TOLERANCE)
        ),
        "maximum_selected_component_regression": float(
            np.max(selected_maximum_component_regression)
        ),
        "selected_aggregate_delta": distribution(selected_aggregate_delta),
        "selected_energy_delta_j": distribution(selected_energy_delta),
        "mechanism_passed": mechanism_passed,
        "profile_transferred": profile_transferred,
        "authority_admitted": False,
        "laws": law_metrics,
    }
    rows = [
        [
            law["law"].replace("_paired_r254", ""),
            str(law["sample_offset"]),
            " / ".join(str(value) for value in law["selected_counts"].values()),
            (
                f"{law['strict_nonregressing_improved_nonzero_actions']} / "
                f"{law['nonzero_actions']}"
            ),
            f"{law['maximum_selected_component_regression']:.3g}",
            "yes" if law["all_candidate_boxes_covered"] else "NO",
            str(law["mujoco_warning_count"]),
        ]
        for law in law_metrics
    ]
    report = "\n".join(
        [
            "# Bonesaw paired terminal-score plant holdout · r254",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'TRANSFERRED' if profile_transferred else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "R254 is the one-shot no-refit plant gate declared by R253. It uses new elliptic/implicitfast and pyramidal/Euler laws at untouched offsets 310,000 and 320,000. Each of 96 states produces exact zero, realized zero-WBC, and realized neutral-recovery candidates; all three execute five 4 ms MuJoCo steps. The fixed R253 closing-speed/tilt group boxes, zero-regression threshold, and 0.01 guaranteed-improvement threshold are consumed without fitting or widening. No policy runs.",
            "",
            *markdown_table(
                [
                    "fresh law",
                    "offset",
                    "selected zero / zero-WBC / neutral",
                    "strict safe + improved / nonzero",
                    "max selected regression",
                    "all boxes covered",
                    "warnings",
                ],
                rows,
            ),
            "",
            f"The run executes {result['physics_steps']:,} fresh MuJoCo steps. Frozen-box coverage is {'complete' if all_candidate_boxes_covered else 'incomplete'}; {result['nonzero_actions']} rows select nonzero effort, {result['improved_nonzero_actions']} improve aggregate consequence, but only {result['strict_nonregressing_improved_nonzero_actions']} is simultaneously component-nonregressing and aggregate-improving. Joint-position pressure and its headroom transform are the largest transfer misses on both laws; aggregate boxes miss too. WBC/selector p99 are {result['wbc_timing_ns']['p99'] / 1e3:.2f}/{result['selector_timing_ns']['p99'] / 1e3:.2f} µs with zero measured Rust allocation. The profile is retained as rejected evidence; no widening, refit, command, or authority follows.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-paired-terminal-score-plant-holdout-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-paired-terminal-score-plant-holdout.npz",
        root_state=root_state,
        joint_position=joint_position,
        initial_velocity=initial_velocity,
        groups=groups,
        fresh_group_supported=fresh_group_supported,
        candidate_acceleration=candidate_acceleration,
        candidate_effort_utilization=candidate_effort_utilization,
        candidate_effort_trace=candidate_effort_trace,
        predicted_diagnostics=predicted_diagnostics,
        actual_diagnostics=actual_diagnostics,
        predicted_component_delta=predicted_component_delta,
        actual_component_delta=actual_component_delta,
        component_lower=component_lower,
        component_upper=component_upper,
        predicted_aggregate_delta=predicted_aggregate_delta,
        actual_aggregate_delta=actual_aggregate_delta,
        aggregate_lower=aggregate_lower,
        aggregate_upper=aggregate_upper,
        selected_index=selected_index,
        selection=selection,
        selected_component_delta=selected_component_delta,
        selected_aggregate_delta=selected_aggregate_delta,
        selected_energy_delta=selected_energy_delta,
        wbc_ns=wbc_ns,
        realization_ns=realization_ns,
        selector_ns=selector_ns,
        semantic_repeat=semantic_repeat,
        raw_status=raw_status,
        fixed_status=fixed_status,
        plant_warning=plant_warning,
        law_index=law_index,
    )
    (output / "G1_PAIRED_TERMINAL_SCORE_PLANT_HOLDOUT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw paired plant holdout · r254"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_transferred": profile_transferred,
                "nonzero_actions": result["nonzero_actions"],
                "all_candidate_boxes_covered": bool(all_candidate_boxes_covered),
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
