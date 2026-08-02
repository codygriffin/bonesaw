#!/usr/bin/env python3
"""R248 fresh reset-every-sample plant A/B for the frozen R247 WBC family.

The action profile, uncertainty widths, contact-activation rule, plant laws,
state offsets, metrics, and gates below are constants before any R248 outcome
is generated.  Each fresh state is forked into a zero-generalized-effort
baseline and one held WBC torque selected through the R247/R224 boundary.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_compliant_terminal_consequence_audit import (
    ROOT_IMPACT_PLANE_M,
    joint_limits,
)
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    PHYSICS_DT,
    SPHERE_RADIUS_M,
    SUBSTEPS,
    ContactLaw,
    build_plant,
    plant_layout,
    point_velocities_world,
    quaternion_from_rpy,
    sha256,
    standing_posture,
    to_bonesaw_tangent,
)
from g1_terminal_box_wbc_action_audit import (
    CANDIDATE_COUNT,
    CANDIDATE_NAMES,
    MAXIMUM_COMPONENT_REGRESSION,
    MINIMUM_COMPONENT_IMPROVEMENT,
    QUERY_DEADLINE_NS,
    SOURCE_METRICS,
    SOURCE_REPLAY,
    allocate_wbc_outputs,
    desired_candidates,
    frozen_widths,
    make_wbc_session,
    run_wbc,
)


REVISION = "g1-terminal-box-wbc-plant-ab-r248"
SOURCE_REVISION = "g1-terminal-box-wbc-action-audit-r247"
FRESH_PLANT_LAWS = (
    ContactLaw(
        "compliant_pyramidal_implicitfast_plant_r248",
        friction=0.81,
        solref_time_s=0.0033,
        solimp_min=0.978,
        solimp_max=0.9975,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "stiff_pyramidal_rk4_plant_r248",
        friction=0.93,
        solref_time_s=0.0022,
        solimp_min=0.990,
        solimp_max=0.9990,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (250_000, 260_000)
WIDTH_SOURCE_LAWS = (
    "medium_pyramidal_implicitfast_surface_r246",
    "rigid_pyramidal_rk4_surface_r246",
)
SAMPLES_PER_LAW = 48
PRESSURE_INDICES = tuple(range(8, 17))
HEADROOM_INDEX = 6
NONREGRESSION_TOLERANCE = 1.0e-12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=SAMPLES_PER_LAW)
    parser.add_argument("--source-action-metrics", default=f"benchmarks/results/{SOURCE_REVISION}/g1-terminal-box-wbc-action-audit-metrics.json")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_TERMINAL_BOX_WBC_PLANT_AB_R248.html"
    )
    return parser.parse_args()


def copy_state(model: mujoco.MjModel, source: mujoco.MjData) -> mujoco.MjData:
    target = mujoco.MjData(model)
    target.qpos[:] = source.qpos
    target.qvel[:] = source.qvel
    target.act[:] = source.act
    target.time = source.time
    mujoco.mj_forward(model, target)
    return target


def warning_count(data: mujoco.MjData) -> int:
    return int(sum(int(item.number) for item in data.warning))


def quaternion_roll_pitch(quaternion: np.ndarray) -> tuple[float, float]:
    w, x, y, z = (float(value) for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def maximum_penetration(data: mujoco.MjData) -> float:
    return max((max(-float(data.contact[index].dist), 0.0) for index in range(data.ncon)), default=0.0)


def rollout(
    model: mujoco.MjModel,
    initial: mujoco.MjData,
    joint_qvel: list[int],
    torque: np.ndarray,
) -> tuple[mujoco.MjData, float, float, int]:
    data = copy_state(model, initial)
    constraint_impulse = np.zeros(model.nv, np.float64)
    penetration = maximum_penetration(data)
    warnings_before = warning_count(data)
    for _ in range(SUBSTEPS):
        data.qfrc_applied.fill(0.0)
        data.qfrc_applied[joint_qvel] = torque
        mujoco.mj_step(model, data)
        constraint_impulse += np.asarray(data.qfrc_constraint) * PHYSICS_DT
        penetration = max(penetration, maximum_penetration(data))
    mujoco.mj_energyVel(model, data)
    return (
        data,
        float(np.linalg.norm(constraint_impulse)),
        penetration,
        warning_count(data) - warnings_before,
    )


def score_terminal_state(
    selector: Any,
    data: mujoco.MjData,
    root_qpos: int,
    root_qvel: int,
    joint_qpos: list[int],
    joint_qvel: list[int],
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    effort_utilization: float,
) -> tuple[np.ndarray, np.ndarray]:
    velocity = to_bonesaw_tangent(data.qvel, root_qvel, joint_qvel)
    quaternion = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7], np.float64)
    roll, pitch = quaternion_roll_pitch(quaternion)
    state = np.asarray(
        [[
            float(data.qpos[root_qpos + 2]) - ROOT_IMPACT_PLANE_M,
            velocity[5],
            roll,
            pitch,
            velocity[0],
            velocity[1],
        ]],
        np.float64,
    )
    diagnostics = np.empty((1, 17), np.float64)
    lower, upper, velocity_limit = limits
    selector.score_terminal_impact_state_batch(
        state,
        np.ascontiguousarray(data.qpos[joint_qpos]),
        np.ascontiguousarray(velocity[None, 6:]),
        lower,
        upper,
        velocity_limit,
        np.ones(1, np.uint8),
        np.zeros((1, 2), np.float64),
        np.zeros((1, len(joint_qvel)), np.float64),
        np.asarray([effort_utilization], np.float64),
        diagnostics,
    )
    return diagnostics[0], velocity


def prepare_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    root_qpos: int,
    root_qvel: int,
    joint_qpos: list[int],
    joint_qvel: list[int],
    foot_geoms: list[int],
    q_nominal: np.ndarray,
    state_index: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
    mujoco.mj_resetData(model, data)
    phase = 0.37 * state_index + np.arange(len(joint_qpos), dtype=np.float64) * 0.23
    q = q_nominal + 0.025 * np.sin(phase)
    roll = 0.025 * math.sin(0.31 * state_index)
    pitch = 0.035 * math.cos(0.27 * state_index)
    yaw = 0.02 * math.sin(0.19 * state_index)
    quaternion = quaternion_from_rpy(roll, pitch, yaw)
    data.qpos[root_qpos : root_qpos + 7] = [0.0, 0.0, 0.80, *quaternion]
    data.qpos[joint_qpos] = q
    mujoco.mj_forward(model, data)
    closing_speed = 0.25 + 0.55 * (0.5 + 0.5 * math.sin(0.43 * state_index))
    impact_fraction = 0.12 + 0.70 * ((state_index % 7) / 6.0)
    clearance = closing_speed * CONTROL_DT * impact_fraction
    lowest_surface = min(
        float(data.geom_xpos[geom, 2] - model.geom_size[geom, 0]) for geom in foot_geoms
    )
    data.qpos[root_qpos + 2] += clearance - lowest_surface
    data.qvel.fill(0.0)
    data.qvel[root_qvel : root_qvel + 3] = [
        0.12 * math.sin(0.17 * state_index),
        0.08 * math.cos(0.29 * state_index),
        -closing_speed,
    ]
    data.qvel[root_qvel + 3 : root_qvel + 6] = [
        0.15 * math.sin(0.21 * state_index),
        0.12 * math.cos(0.33 * state_index),
        0.08 * math.sin(0.41 * state_index),
    ]
    data.qvel[joint_qvel] = 0.08 * np.sin(phase + 0.5)
    mujoco.mj_forward(model, data)
    velocity = to_bonesaw_tangent(data.qvel, root_qvel, joint_qvel)
    points = np.asarray(data.geom_xpos[foot_geoms], np.float64).copy()
    points[:, 2] -= SPHERE_RADIUS_M
    point_velocity = point_velocities_world(model, data, foot_geoms)
    predicted_active = points[:, 2] <= np.maximum(-point_velocity[:, 2], 0.0) * CONTROL_DT + 1.0e-12
    contacts = np.any(predicted_active.reshape(2, 4), axis=1).astype(np.uint8)
    return (
        np.asarray(data.qpos[root_qpos : root_qpos + 3], np.float64).copy(),
        quaternion,
        q,
        velocity,
        contacts,
        predicted_active.astype(np.uint8),
        roll,
        pitch,
    )


def main() -> int:
    import bonesaw

    args = parse_args()
    if args.samples_per_law != SAMPLES_PER_LAW:
        raise ValueError(f"R248 is frozen at exactly {SAMPLES_PER_LAW} samples per law")
    model_path = pathlib.Path(args.model).resolve()
    source_action_metrics_path = pathlib.Path(args.source_action_metrics).resolve()
    r246_metrics_path = pathlib.Path(SOURCE_METRICS).resolve()
    r246_replay_path = pathlib.Path(SOURCE_REPLAY).resolve()
    if not all(path.is_file() for path in (model_path, source_action_metrics_path, r246_metrics_path, r246_replay_path)):
        raise SystemExit("R248 requires the pinned model and immutable R246/R247 evidence")
    source_action_metrics = json.loads(source_action_metrics_path.read_text())
    if source_action_metrics["revision"] != SOURCE_REVISION or not source_action_metrics["action_profile_frozen_for_fresh_plant_ab"]:
        raise ValueError("R248 requires the frozen R247 action family")
    source_hashes_before = {
        "r247_metrics": sha256(source_action_metrics_path),
        "r246_metrics": sha256(r246_metrics_path),
        "r246_replay": sha256(r246_replay_path),
    }
    widths = frozen_widths(json.loads(r246_metrics_path.read_text()))
    selector = bonesaw.ContactTransitionModelSession(
        str(model_path), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    wbc = make_wbc_session(bonesaw, model_path)
    joint_names = list(wbc.joint_names)
    q_nominal = standing_posture(joint_names)
    limits = joint_limits(model_path, joint_names)
    effort_limits = np.asarray(wbc.actuator_effort_limits, np.float64)
    frame_ids = np.asarray([list(wbc.frame_names).index(frame) for frame in FOOT_FRAMES], np.int64)
    diagnostic_names = tuple(selector.terminal_impact_state_diagnostic_names)
    stored: dict[str, np.ndarray] = {}
    results: list[dict[str, Any]] = []

    for law, offset, width_source in zip(FRESH_PLANT_LAWS, SAMPLE_OFFSETS, WIDTH_SOURCE_LAWS, strict=True):
        model, initial = build_plant(model_path, law)
        root_qpos, joint_qpos, joint_qvel, _, foot_geoms, _ = plant_layout(model, joint_names)
        root_qvel = int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")])
        samples = args.samples_per_law
        selected_index = np.empty(samples, np.uint8)
        predicted_active = np.empty((samples, 8), np.uint8)
        predicted_foot_active = np.empty((samples, 2), np.uint8)
        selected_torque = np.empty((samples, len(joint_names)), np.float64)
        wbc_status = np.empty((samples, CANDIDATE_COUNT), np.uint8)
        wbc_step_ns = np.empty((samples, CANDIDATE_COUNT), np.uint64)
        selection = np.empty((samples, 6), np.float64)
        selection_ns = np.empty(samples, np.uint64)
        baseline_diagnostics = np.empty((samples, 17), np.float64)
        candidate_diagnostics = np.empty((samples, 17), np.float64)
        baseline_velocity = np.empty((samples, len(joint_names) + 6), np.float64)
        candidate_velocity = np.empty_like(baseline_velocity)
        baseline_kinetic_energy = np.empty(samples, np.float64)
        candidate_kinetic_energy = np.empty(samples, np.float64)
        baseline_constraint_impulse = np.empty(samples, np.float64)
        candidate_constraint_impulse = np.empty(samples, np.float64)
        baseline_penetration = np.empty(samples, np.float64)
        candidate_penetration = np.empty(samples, np.float64)
        baseline_warning = np.empty(samples, np.uint16)
        candidate_warning = np.empty(samples, np.uint16)
        zero_wbc_allocation = True
        zero_selector_allocation = True
        width = widths[width_source]

        for sample in range(samples):
            state_index = offset + sample
            root, quaternion, q, velocity, contacts, points_active, roll, pitch = prepare_initial_state(
                model,
                initial,
                root_qpos,
                root_qvel,
                joint_qpos,
                joint_qvel,
                foot_geoms,
                q_nominal,
                state_index,
            )
            predicted_active[sample] = points_active
            predicted_foot_active[sample] = contacts
            desired_root, desired_angular, desired_joint = desired_candidates(
                q[None, :], velocity[None, 6:], q_nominal, velocity[None, :3]
            )
            outputs: list[dict[str, np.ndarray]] = []
            for candidate in range(CANDIDATE_COUNT):
                result = allocate_wbc_outputs(1, wbc.dof, 8, wbc.task_diagnostic_capacity)
                run_wbc(
                    wbc,
                    root[None, :],
                    velocity[None, 3:6],
                    desired_root[candidate],
                    q[None, :],
                    velocity[None, 6:],
                    desired_joint[candidate],
                    frame_ids,
                    contacts[None, :],
                    result,
                    quaternion[None, :],
                    velocity[None, :3],
                    desired_angular[candidate],
                )
                outputs.append(result)
                wbc_status[sample, candidate] = result["status"][0]
                wbc_step_ns[sample, candidate] = result["step_ns"][0]
                zero_wbc_allocation &= bool(
                    result["allocation_calls"][0] == 0 and result["allocated_bytes"][0] == 0
                )
            acceleration = np.stack([result["generalized_acceleration"][0] for result in outputs])
            center = velocity + CONTROL_DT * acceleration
            root_lower = center[:, [5, 0, 1]] - width[[5, 0, 1]]
            root_upper = center[:, [5, 0, 1]] + width[[5, 0, 1]]
            joint_lower = center[:, 6:] - width[6:]
            joint_upper = center[:, 6:] + width[6:]
            available = np.asarray([result["status"][0] <= 1 for result in outputs], np.uint8)
            utilization = np.asarray([result["maximum_torque_utilization"][0] for result in outputs])
            diagnostics = np.empty((CANDIDATE_COUNT, 17), np.float64)
            timing = selector.select_terminal_impact_velocity_box_candidates(
                np.asarray([root[2] - ROOT_IMPACT_PLANE_M, roll, pitch], np.float64),
                root_lower,
                root_upper,
                q,
                joint_lower,
                joint_upper,
                limits[0],
                limits[1],
                limits[2],
                available,
                np.ascontiguousarray(acceleration[:, :2]),
                np.ascontiguousarray(acceleration[:, 6:]),
                utilization,
                0,
                MAXIMUM_COMPONENT_REGRESSION,
                MINIMUM_COMPONENT_IMPROVEMENT,
                diagnostics,
                selection[sample],
            )
            selection_ns[sample] = timing[0]
            zero_selector_allocation &= timing[1:] == (0, 0)
            chosen = int(selection[sample, 0])
            selected_index[sample] = chosen
            selected_torque[sample] = outputs[chosen]["actuator_torque"][0]
            baseline, baseline_impulse, baseline_depth, baseline_warn = rollout(
                model, initial, joint_qvel, np.zeros(len(joint_names), np.float64)
            )
            candidate, candidate_impulse, candidate_depth, candidate_warn = rollout(
                model, initial, joint_qvel, selected_torque[sample]
            )
            effort_utilization = float(np.max(np.abs(selected_torque[sample]) / effort_limits))
            baseline_diagnostics[sample], baseline_velocity[sample] = score_terminal_state(
                selector, baseline, root_qpos, root_qvel, joint_qpos, joint_qvel, limits, 0.0
            )
            candidate_diagnostics[sample], candidate_velocity[sample] = score_terminal_state(
                selector,
                candidate,
                root_qpos,
                root_qvel,
                joint_qpos,
                joint_qvel,
                limits,
                effort_utilization,
            )
            baseline_kinetic_energy[sample] = baseline.energy[1]
            candidate_kinetic_energy[sample] = candidate.energy[1]
            baseline_constraint_impulse[sample] = baseline_impulse
            candidate_constraint_impulse[sample] = candidate_impulse
            baseline_penetration[sample] = baseline_depth
            candidate_penetration[sample] = candidate_depth
            baseline_warning[sample] = baseline_warn
            candidate_warning[sample] = candidate_warn

        pressure_delta = candidate_diagnostics[:, PRESSURE_INDICES] - baseline_diagnostics[:, PRESSURE_INDICES]
        headroom_delta = candidate_diagnostics[:, HEADROOM_INDEX] - baseline_diagnostics[:, HEADROOM_INDEX]
        component_regression = np.maximum(
            np.max(pressure_delta, axis=1), -headroom_delta
        )
        nonregression = component_regression <= NONREGRESSION_TOLERANCE
        result = {
            "law": law.name,
            "samples": samples,
            "selected_counts": {
                name: int(np.count_nonzero(selected_index == candidate))
                for candidate, name in enumerate(CANDIDATE_NAMES)
            },
            "predicted_foot_active_counts": {
                "left_only": int(np.count_nonzero(np.all(predicted_foot_active == [1, 0], axis=1))),
                "right_only": int(np.count_nonzero(np.all(predicted_foot_active == [0, 1], axis=1))),
                "double": int(np.count_nonzero(np.all(predicted_foot_active == [1, 1], axis=1))),
                "flight": int(np.count_nonzero(np.all(predicted_foot_active == [0, 0], axis=1))),
            },
            "all_candidates_wbc_admitted": bool(np.all(wbc_status <= 1)),
            "zero_wbc_rust_allocation": zero_wbc_allocation,
            "zero_selector_rust_allocation": zero_selector_allocation,
            "wbc_step_timing_ns": distribution(wbc_step_ns.reshape(-1)),
            "selector_timing_ns": distribution(selection_ns),
            "deadline_passed": bool(
                np.percentile(wbc_step_ns, 99) <= QUERY_DEADLINE_NS
                and np.percentile(selection_ns, 99) <= QUERY_DEADLINE_NS
            ),
            "plant_nonregression_samples": int(np.count_nonzero(nonregression)),
            "plant_regression_samples": int(np.count_nonzero(~nonregression)),
            "maximum_terminal_component_regression": float(np.max(component_regression)),
            "terminal_component_regression": distribution(component_regression),
            "maximum_harm_pressure_delta": distribution(
                candidate_diagnostics[:, 15] - baseline_diagnostics[:, 15]
            ),
            "aggregate_score_delta": distribution(
                candidate_diagnostics[:, 16] - baseline_diagnostics[:, 16]
            ),
            "kinetic_energy_delta_j": distribution(
                candidate_kinetic_energy - baseline_kinetic_energy
            ),
            "constraint_impulse_norm_delta": distribution(
                candidate_constraint_impulse - baseline_constraint_impulse
            ),
            "maximum_penetration_delta_m": distribution(
                candidate_penetration - baseline_penetration
            ),
            "mujoco_warning_count": int(np.sum(baseline_warning) + np.sum(candidate_warning)),
            "strict_plant_nonregression": bool(
                np.all(nonregression)
                and np.all(baseline_warning == 0)
                and np.all(candidate_warning == 0)
            ),
        }
        results.append(result)
        stored.update(
            {
                f"{law.name}_selected_index": selected_index,
                f"{law.name}_predicted_active": predicted_active,
                f"{law.name}_predicted_foot_active": predicted_foot_active,
                f"{law.name}_selected_torque": selected_torque,
                f"{law.name}_wbc_status": wbc_status,
                f"{law.name}_wbc_step_ns": wbc_step_ns,
                f"{law.name}_selection": selection,
                f"{law.name}_selection_ns": selection_ns,
                f"{law.name}_baseline_terminal_diagnostics": baseline_diagnostics,
                f"{law.name}_candidate_terminal_diagnostics": candidate_diagnostics,
                f"{law.name}_baseline_generalized_velocity": baseline_velocity,
                f"{law.name}_candidate_generalized_velocity": candidate_velocity,
                f"{law.name}_baseline_kinetic_energy": baseline_kinetic_energy,
                f"{law.name}_candidate_kinetic_energy": candidate_kinetic_energy,
                f"{law.name}_baseline_constraint_impulse": baseline_constraint_impulse,
                f"{law.name}_candidate_constraint_impulse": candidate_constraint_impulse,
                f"{law.name}_baseline_penetration": baseline_penetration,
                f"{law.name}_candidate_penetration": candidate_penetration,
                f"{law.name}_baseline_warning": baseline_warning,
                f"{law.name}_candidate_warning": candidate_warning,
                f"{law.name}_terminal_component_regression": component_regression,
                f"{law.name}_terminal_nonregression": nonregression.astype(np.uint8),
            }
        )

    source_hashes_after = {
        "r247_metrics": sha256(source_action_metrics_path),
        "r246_metrics": sha256(r246_metrics_path),
        "r246_replay": sha256(r246_replay_path),
    }
    source_immutable = source_hashes_before == source_hashes_after
    plant_nonregression = source_immutable and all(row["strict_plant_nonregression"] for row in results)
    mechanism_passed = source_immutable and all(
        row["all_candidates_wbc_admitted"]
        and row["zero_wbc_rust_allocation"]
        and row["zero_selector_rust_allocation"]
        and row["deadline_passed"]
        for row in results
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_hashes": source_hashes_before,
        "source_immutable": source_immutable,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "fresh_laws_and_offsets_frozen_before_outcomes": True,
        "fresh_laws": [law.__dict__ for law in FRESH_PLANT_LAWS],
        "sample_offsets": SAMPLE_OFFSETS,
        "samples": sum(row["samples"] for row in results),
        "baseline": "zero generalized joint effort held for 5 ms",
        "candidate": "R247-selected WBC joint effort held for 5 ms",
        "contact_activation": "pre-step sphere surface gap <= closing speed * 5 ms, collapsed per foot",
        "physics_steps_per_branch": SUBSTEPS,
        "physics_steps": len(FRESH_PLANT_LAWS) * SAMPLES_PER_LAW * 2 * SUBSTEPS,
        "policy_steps": 0,
        "plant_actions": len(FRESH_PLANT_LAWS) * SAMPLES_PER_LAW,
        "wbc_state_local_queries": len(FRESH_PLANT_LAWS) * SAMPLES_PER_LAW * CANDIDATE_COUNT,
        "selector_queries": len(FRESH_PLANT_LAWS) * SAMPLES_PER_LAW,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "nonregression_pressure_names": [diagnostic_names[index] for index in PRESSURE_INDICES],
        "nonregression_headroom_name": diagnostic_names[HEADROOM_INDEX],
        "nonregression_tolerance": NONREGRESSION_TOLERANCE,
        "terminal_score_continuation_acceleration": "zero for both completed branches, matching R222",
        "mechanism_passed": mechanism_passed,
        "strict_plant_nonregression_passed": plant_nonregression,
        "action_profile_promoted": plant_nonregression,
        "authority_admitted": False,
        "results": results,
    }
    rows = [
        [
            row["law"],
            " / ".join(str(row["selected_counts"][name]) for name in CANDIDATE_NAMES),
            f"{row['plant_nonregression_samples']}/{row['samples']}",
            f"{row['maximum_terminal_component_regression']:.4g}",
            f"{row['maximum_harm_pressure_delta']['p50']:.4g} / {row['maximum_harm_pressure_delta']['p99']:.4g}",
            f"{row['kinetic_energy_delta_j']['p50']:.4g} / {row['kinetic_energy_delta_j']['p99']:.4g}",
            f"{row['wbc_step_timing_ns']['p99'] / 1e6:.3f}",
            "PASS" if row["strict_plant_nonregression"] else "REJECT",
        ]
        for row in results
    ]
    report = "\n".join(
        [
            "# Bonesaw fresh terminal-box WBC plant A/B · r248",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · strict plant non-regression **{'PASS' if plant_nonregression else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "R247's fixed three-candidate laws are evaluated without retuning on two new pyramidal contact laws and disjoint offsets 250,000/260,000. Every pre-impact state is forked: baseline holds zero generalized joint effort for five MuJoCo substeps; candidate holds the torque selected before either branch advances. No policy is queried.",
            "",
            *markdown_table(
                [
                    "law",
                    "selected · zero / damp / recover",
                    "non-regressed",
                    "max component regression",
                    "harm Δ · p50 / p99",
                    "kinetic J Δ · p50 / p99",
                    "WBC p99 ms",
                    "decision",
                ],
                rows,
            ),
            "",
            "The strict gate is per-sample and componentwise over the terminal proxy's nine pressure/score outputs plus lower-bounded joint headroom. Both completed branches use R222's zero continuation-acceleration contract; instantaneous MuJoCo contact qacc is not extrapolated over the proxy horizon. Penetration, constraint impulse, kinetic energy, warnings, selection, and timing remain separately archived rather than collapsed into that verdict. Passing would promote only this simulated action profile; authority remains closed pending the ordinary-process deadline audit and broader robustness/calibration evidence.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-terminal-box-wbc-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-terminal-box-wbc-plant-ab.npz", **stored)
    (output / "G1_TERMINAL_BOX_WBC_PLANT_AB.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw terminal-box WBC plant A/B · r248"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "strict_plant_nonregression_passed": plant_nonregression,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
