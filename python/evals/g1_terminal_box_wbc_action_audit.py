#!/usr/bin/env python3
"""R247 state-local WBC candidate selection without policy or plant execution.

This is a design audit, not a holdout.  It reuses the spent R246 pre-impact
states, predicted contact mask, and frozen residual widths.  No completed
per-sample contact label is opened.  Three fixed desired-acceleration laws are
admitted independently by the exact Rust WBC, wrapped in the R224 terminal
velocity box, and selected by the allocation-free Rust boundary added in R247.
No selected acceleration or torque is applied to a simulator or robot.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import reconstruct_prestate
from g1_compliant_terminal_consequence_audit import (
    ROOT_IMPACT_PLANE_M,
    joint_limits,
    reconstruct_prevelocity,
)
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_surface_material_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
)


REVISION = "g1-terminal-box-wbc-action-audit-r247"
SOURCE_REVISION = "g1-surface-material-cross-integrator-holdout-r246"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-surface-material-cross-integrator-holdout-r246/"
    "g1-surface-material-cross-integrator-holdout.npz"
)
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-surface-material-cross-integrator-holdout-r246/"
    "g1-surface-material-cross-integrator-holdout-metrics.json"
)
CONTROL_DT_SECONDS = 0.005
CANDIDATE_NAMES = ("zero_acceleration", "velocity_damping", "neutral_recovery")
CANDIDATE_COUNT = len(CANDIDATE_NAMES)
MAXIMUM_COMPONENT_REGRESSION = 0.0
MINIMUM_COMPONENT_IMPROVEMENT = 1.0e-6
QUERY_DEADLINE_NS = 5_000_000.0
ALLOWED_REPLAY_SUFFIXES = ("root_height", "predicted_active")
WBC_CONFIG = {
    "friction_coefficient": 1.0,
    "maximum_acceleration": 200.0,
    "maximum_torque": 2_000.0,
    "maximum_normal_force_multiple": 3.0,
    "root_angular_task_weight": 1.0,
    "root_height_task_weight": 1.0,
    "root_horizontal_task_weight": 1.0,
    "root_horizontal_task_priority": 1,
    "joint_posture_weight": 0.01,
    "joint_posture_priority": 3,
    "center_of_mass_task_weight": 1.0,
    "center_of_mass_task_priority": 1,
    "centroidal_angular_momentum_weight": 0.0,
    "centroidal_angular_momentum_priority": 1,
    "contact_patch_center_x": 0.035,
    "contact_patch_half_length": 0.085,
    "contact_patch_half_width": 0.0275,
    "contact_patch_z": -0.035,
    "minimum_contact_cop_margin_m": 0.005,
}


def allocate_wbc_outputs(
    ticks: int, dof: int, contacts: int, tasks: int
) -> dict[str, np.ndarray]:
    """Allocate the complete fixed-shape oracle boundary without legacy imports."""
    return {
        "generalized_acceleration": np.empty((ticks, dof + 6), np.float64),
        "actuator_torque": np.empty((ticks, dof), np.float64),
        "contact_normal_force": np.empty((ticks, contacts), np.float64),
        "task_rms": np.empty((ticks, tasks), np.float64),
        "task_clipped": np.empty((ticks, tasks), np.uint8),
        "dynamics_residual": np.empty(ticks, np.float64),
        "contact_residual": np.empty(ticks, np.float64),
        "minimum_friction_margin": np.empty(ticks, np.float64),
        "minimum_support_margin": np.empty(ticks, np.float64),
        "minimum_torque_margin": np.empty(ticks, np.float64),
        "maximum_constraint_violation": np.empty(ticks, np.float64),
        "minimum_bound_margin": np.empty(ticks, np.float64),
        "minimum_joint_margin_rad": np.empty(ticks, np.float64),
        "minimum_joint_headroom_fraction": np.empty(ticks, np.float64),
        "limiting_joint": np.empty(ticks, np.uint16),
        "maximum_torque_utilization": np.empty(ticks, np.float64),
        "minimum_torque_headroom": np.empty(ticks, np.float64),
        "limiting_actuator": np.empty(ticks, np.uint16),
        "witness_acceleration_rms": np.empty(ticks, np.float64),
        "step_ns": np.empty(ticks, np.uint64),
        "status": np.empty(ticks, np.uint8),
        "task_pseudoinverse_calls": np.empty(ticks, np.uint16),
        "task_pseudoinverse_calls_by_priority": np.empty((ticks, 5), np.uint16),
        "clipped_steps": np.empty(ticks, np.uint16),
        "clipped_steps_by_priority": np.empty((ticks, 5), np.uint16),
        "task_jacobi_sweeps": np.empty(ticks, np.uint16),
        "task_jacobi_sweeps_by_priority": np.empty((ticks, 5), np.uint16),
        "feasibility_projection_sweeps": np.empty(ticks, np.uint16),
        "feasibility_halfspace_projections": np.empty(ticks, np.uint32),
        "feasibility_polish_iterations": np.empty(ticks, np.uint16),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
    }


def run_wbc(
    session: Any,
    root_positions: np.ndarray,
    root_velocities: np.ndarray,
    root_accelerations: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    joint_accelerations: np.ndarray,
    frame_ids: np.ndarray,
    contacts: np.ndarray,
    outputs: dict[str, np.ndarray],
    root_quaternions_wxyz: np.ndarray,
    root_angular_velocities_world: np.ndarray,
    root_angular_accelerations_world: np.ndarray,
) -> None:
    session.run_oracle_trace(
        root_positions,
        root_velocities,
        root_accelerations,
        q,
        v,
        joint_accelerations,
        frame_ids,
        contacts,
        np.ones(len(frame_ids), np.uint8),
        np.ones(len(frame_ids), np.float64),
        0,
        0,
        outputs["generalized_acceleration"],
        outputs["actuator_torque"],
        outputs["contact_normal_force"],
        outputs["task_rms"],
        outputs["task_clipped"],
        outputs["dynamics_residual"],
        outputs["contact_residual"],
        outputs["minimum_friction_margin"],
        outputs["minimum_support_margin"],
        outputs["minimum_torque_margin"],
        outputs["maximum_constraint_violation"],
        outputs["minimum_bound_margin"],
        outputs["minimum_joint_margin_rad"],
        outputs["minimum_joint_headroom_fraction"],
        outputs["limiting_joint"],
        outputs["maximum_torque_utilization"],
        outputs["minimum_torque_headroom"],
        outputs["limiting_actuator"],
        outputs["witness_acceleration_rms"],
        outputs["step_ns"],
        outputs["status"],
        outputs["task_pseudoinverse_calls"],
        outputs["task_pseudoinverse_calls_by_priority"],
        outputs["clipped_steps"],
        outputs["clipped_steps_by_priority"],
        outputs["task_jacobi_sweeps"],
        outputs["task_jacobi_sweeps_by_priority"],
        outputs["feasibility_projection_sweeps"],
        outputs["feasibility_halfspace_projections"],
        outputs["feasibility_polish_iterations"],
        outputs["allocation_calls"],
        outputs["allocated_bytes"],
        root_quaternions_wxyz=root_quaternions_wxyz,
        root_angular_velocities_world=root_angular_velocities_world,
        root_angular_accelerations_world=root_angular_accelerations_world,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_TERMINAL_BOX_WBC_ACTION_AUDIT_R247.html"
    )
    return parser.parse_args()


def make_wbc_session(bonesaw: Any, model: pathlib.Path) -> Any:
    return bonesaw.FloatingWbcSession(
        str(model), maximum_contacts=8, **WBC_CONFIG
    )


def frozen_widths(source_metrics: dict[str, Any]) -> dict[str, np.ndarray]:
    widths: dict[str, np.ndarray] = {}
    for result in source_metrics["results"]:
        value = result["diagnostic_fitted_width"]
        # R246 reports a full symmetric width (2 * maximum absolute residual).
        # The R224 boundary consumes lower/upper coordinates around a center,
        # so each side receives exactly half of that archived width.
        widths[result["law"]] = 0.5 * np.asarray(
            [
                *([value["root_angular_rad_s"]] * 3),
                *([value["root_linear_m_s"]] * 3),
                *([value["joint_rad_s"]] * 23),
            ],
            np.float64,
        )
    return widths


def load_causal_inputs(source: pathlib.Path) -> dict[str, np.ndarray]:
    inputs: dict[str, np.ndarray] = {}
    with np.load(source) as replay:
        for law in FRESH_CONTACT_LAWS:
            for suffix in ALLOWED_REPLAY_SUFFIXES:
                key = f"{law.name}_{suffix}"
                inputs[key] = np.asarray(replay[key]).copy()
    return inputs


def desired_candidates(
    q: np.ndarray, v: np.ndarray, q_nominal: np.ndarray, omega: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    samples, joints = q.shape
    root_linear = np.zeros((CANDIDATE_COUNT, samples, 3), np.float64)
    root_angular = np.zeros((CANDIDATE_COUNT, samples, 3), np.float64)
    joint = np.zeros((CANDIDATE_COUNT, samples, joints), np.float64)
    root_angular[1:, :, :2] = np.clip(-10.0 * omega[:, :2], -100.0, 100.0)
    joint[1] = np.clip(-10.0 * v, -100.0, 100.0)
    joint[2] = np.clip(-20.0 * (q - q_nominal) - 8.0 * v, -100.0, 100.0)
    return root_linear, root_angular, joint


def evaluate_once(
    bonesaw: Any,
    model: pathlib.Path,
    causal: dict[str, np.ndarray],
    widths: dict[str, np.ndarray],
) -> tuple[dict[str, np.ndarray], list[dict[str, Any]]]:
    wbc = make_wbc_session(bonesaw, model)
    selector = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    joint_names = list(wbc.joint_names)
    if joint_names != list(selector.joint_names()):
        raise RuntimeError("WBC and terminal selector joint layouts differ")
    q_nominal = standing_posture(joint_names)
    lower, upper, velocity_limit = joint_limits(model, joint_names)
    frame_ids = np.asarray(
        [list(wbc.frame_names).index(frame) for frame in FOOT_FRAMES], np.int64
    )
    stored: dict[str, np.ndarray] = {}
    law_metrics: list[dict[str, Any]] = []

    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        prefix = law.name
        root_height = causal[f"{prefix}_root_height"]
        samples = len(root_height)
        roots = np.empty((samples, 3), np.float64)
        quaternions = np.empty((samples, 4), np.float64)
        q = np.empty((samples, len(joint_names)), np.float64)
        generalized_velocity = np.empty((samples, len(joint_names) + 6), np.float64)
        roll_pitch = np.empty((samples, 2), np.float64)
        for sample, height in enumerate(root_height):
            state_index = offset + sample
            roots[sample], quaternions[sample], q[sample] = reconstruct_prestate(
                len(joint_names), q_nominal, state_index, float(height)
            )
            generalized_velocity[sample] = reconstruct_prevelocity(
                len(joint_names), state_index
            )
            roll_pitch[sample] = [
                0.025 * math.sin(0.31 * state_index),
                0.035 * math.cos(0.27 * state_index),
            ]
        omega = generalized_velocity[:, :3]
        root_velocity = generalized_velocity[:, 3:6]
        v = generalized_velocity[:, 6:]
        contacts = np.any(
            causal[f"{prefix}_predicted_active"].reshape(samples, 2, 4) != 0,
            axis=2,
        ).astype(np.uint8)
        desired_root, desired_angular, desired_joint = desired_candidates(
            q, v, q_nominal, omega
        )

        wbc_outputs: list[dict[str, np.ndarray]] = []
        for candidate in range(CANDIDATE_COUNT):
            outputs = allocate_wbc_outputs(
                samples, wbc.dof, 8, wbc.task_diagnostic_capacity
            )
            run_wbc(
                wbc,
                roots,
                root_velocity,
                desired_root[candidate],
                q,
                v,
                desired_joint[candidate],
                frame_ids,
                contacts,
                outputs,
                quaternions,
                omega,
                desired_angular[candidate],
            )
            wbc_outputs.append(outputs)

        diagnostic_count = len(selector.terminal_impact_state_diagnostic_names)
        diagnostics = np.empty((samples, CANDIDATE_COUNT, diagnostic_count), np.float64)
        selections = np.empty((samples, 6), np.float64)
        selector_ns = np.empty(samples, np.uint64)
        selector_allocation_calls = np.empty(samples, np.uint64)
        selector_allocated_bytes = np.empty(samples, np.uint64)
        width = widths[prefix]
        for sample in range(samples):
            realized_acceleration = np.stack(
                [outputs["generalized_acceleration"][sample] for outputs in wbc_outputs]
            )
            center = generalized_velocity[sample] + CONTROL_DT_SECONDS * realized_acceleration
            root_lower = center[:, [5, 0, 1]] - width[[5, 0, 1]]
            root_upper = center[:, [5, 0, 1]] + width[[5, 0, 1]]
            joint_lower = center[:, 6:] - width[6:]
            joint_upper = center[:, 6:] + width[6:]
            available = np.asarray(
                [outputs["status"][sample] <= 1 for outputs in wbc_outputs], np.uint8
            )
            effort = np.asarray(
                [
                    outputs["maximum_torque_utilization"][sample]
                    for outputs in wbc_outputs
                ],
                np.float64,
            )
            result = selector.select_terminal_impact_velocity_box_candidates(
                np.asarray(
                    [
                        float(root_height[sample]) - ROOT_IMPACT_PLANE_M,
                        *roll_pitch[sample],
                    ],
                    np.float64,
                ),
                root_lower,
                root_upper,
                q[sample],
                joint_lower,
                joint_upper,
                lower,
                upper,
                velocity_limit,
                available,
                np.ascontiguousarray(realized_acceleration[:, :2]),
                np.ascontiguousarray(realized_acceleration[:, 6:]),
                effort,
                0,
                MAXIMUM_COMPONENT_REGRESSION,
                MINIMUM_COMPONENT_IMPROVEMENT,
                diagnostics[sample],
                selections[sample],
            )
            selector_ns[sample], selector_allocation_calls[sample], selector_allocated_bytes[sample] = result

        status = np.stack([outputs["status"] for outputs in wbc_outputs], axis=1)
        wbc_step_ns = np.stack([outputs["step_ns"] for outputs in wbc_outputs], axis=1)
        wbc_allocation_calls = np.stack(
            [outputs["allocation_calls"] for outputs in wbc_outputs], axis=1
        )
        wbc_allocated_bytes = np.stack(
            [outputs["allocated_bytes"] for outputs in wbc_outputs], axis=1
        )
        maximum_torque_utilization = np.stack(
            [outputs["maximum_torque_utilization"] for outputs in wbc_outputs], axis=1
        )
        generalized_acceleration = np.stack(
            [outputs["generalized_acceleration"] for outputs in wbc_outputs], axis=1
        )
        actuator_torque = np.stack(
            [outputs["actuator_torque"] for outputs in wbc_outputs], axis=1
        )
        selected = selections[:, 0].astype(np.int64)
        regression = selections[:, 4]
        improvement = selections[:, 5]
        law_metrics.append(
            {
                "law": prefix,
                "samples": samples,
                "candidate_status_counts": {
                    CANDIDATE_NAMES[candidate]: {
                        "solved": int(np.count_nonzero(status[:, candidate] == 0)),
                        "solved_with_slack": int(np.count_nonzero(status[:, candidate] == 1)),
                        "failed": int(np.count_nonzero(status[:, candidate] >= 2)),
                    }
                    for candidate in range(CANDIDATE_COUNT)
                },
                "selected_counts": {
                    name: int(np.count_nonzero(selected == candidate))
                    for candidate, name in enumerate(CANDIDATE_NAMES)
                },
                "strict_improvement_samples": int(
                    np.count_nonzero(improvement >= MINIMUM_COMPONENT_IMPROVEMENT)
                ),
                "maximum_component_regression": float(np.max(regression)),
                "component_improvement": distribution(improvement),
                "wbc_step_timing_ns": distribution(wbc_step_ns.reshape(-1)),
                "selector_timing_ns": distribution(selector_ns),
                "maximum_torque_utilization": {
                    CANDIDATE_NAMES[candidate]: distribution(
                        maximum_torque_utilization[:, candidate]
                    )
                    for candidate in range(CANDIDATE_COUNT)
                },
                "all_candidates_wbc_admitted": bool(np.all(status <= 1)),
                "zero_wbc_rust_allocation": bool(
                    np.all(wbc_allocation_calls == 0) and np.all(wbc_allocated_bytes == 0)
                ),
                "zero_selector_rust_allocation": bool(
                    np.all(selector_allocation_calls == 0)
                    and np.all(selector_allocated_bytes == 0)
                ),
                "deadline_passed": bool(
                    np.percentile(wbc_step_ns, 99) <= QUERY_DEADLINE_NS
                    and np.percentile(selector_ns, 99) <= QUERY_DEADLINE_NS
                ),
            }
        )
        stored.update(
            {
                f"{prefix}_root_position": roots,
                f"{prefix}_root_quaternion_wxyz": quaternions,
                f"{prefix}_joint_position": q,
                f"{prefix}_generalized_velocity": generalized_velocity,
                f"{prefix}_predicted_foot_active": contacts,
                f"{prefix}_desired_root_angular_acceleration": desired_angular,
                f"{prefix}_desired_joint_acceleration": desired_joint,
                f"{prefix}_wbc_generalized_acceleration": generalized_acceleration,
                f"{prefix}_wbc_actuator_torque": actuator_torque,
                f"{prefix}_wbc_status": status,
                f"{prefix}_wbc_maximum_torque_utilization": maximum_torque_utilization,
                f"{prefix}_wbc_step_ns": wbc_step_ns,
                f"{prefix}_wbc_allocation_calls": wbc_allocation_calls,
                f"{prefix}_wbc_allocated_bytes": wbc_allocated_bytes,
                f"{prefix}_selector_diagnostics": diagnostics,
                f"{prefix}_selection": selections,
                f"{prefix}_selector_ns": selector_ns,
                f"{prefix}_selector_allocation_calls": selector_allocation_calls,
                f"{prefix}_selector_allocated_bytes": selector_allocated_bytes,
            }
        )
    return stored, law_metrics


def semantic_repeat(
    first: dict[str, np.ndarray], second: dict[str, np.ndarray]
) -> tuple[bool, int, int]:
    timing_suffixes = ("_wbc_step_ns", "_selector_ns")
    keys = sorted(key for key in first if not key.endswith(timing_suffixes))
    exact = sum(np.array_equal(first[key], second[key]) for key in keys)
    return exact == len(keys), exact, len(keys)


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    if not model.is_file() or not source.is_file() or not source_metrics_path.is_file():
        raise SystemExit("R247 requires the pinned G1 model and immutable R246 evidence")
    source_hashes_before = {
        "replay": sha256(source),
        "metrics": sha256(source_metrics_path),
    }
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics["revision"] != SOURCE_REVISION or not source_metrics["profile_promoted"]:
        raise ValueError("R247 requires the promoted R246 evaluator profile")
    widths = frozen_widths(source_metrics)
    causal = load_causal_inputs(source)
    first, results = evaluate_once(bonesaw, model, causal, widths)
    second, _ = evaluate_once(bonesaw, model, causal, widths)
    repeat, repeated_arrays, semantic_arrays = semantic_repeat(first, second)
    source_hashes_after = {
        "replay": sha256(source),
        "metrics": sha256(source_metrics_path),
    }
    source_immutable = source_hashes_before == source_hashes_after
    mechanism_passed = bool(
        repeat
        and source_immutable
        and all(
            row["all_candidates_wbc_admitted"]
            and row["zero_wbc_rust_allocation"]
            and row["zero_selector_rust_allocation"]
            and row["deadline_passed"]
            and row["maximum_component_regression"] <= MAXIMUM_COMPONENT_REGRESSION
            for row in results
        )
    )
    selected_counts = {
        name: sum(row["selected_counts"][name] for row in results)
        for name in CANDIDATE_NAMES
    }
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_hashes": source_hashes_before,
        "source_immutable": source_immutable,
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": sum(row["samples"] for row in results),
        "candidate_names": CANDIDATE_NAMES,
        "candidate_laws": {
            "zero_acceleration": "zero desired generalized acceleration",
            "velocity_damping": "-10*v joint and -10*omega roll/pitch damping",
            "neutral_recovery": "-20*(q-q_nominal)-8*v joint recovery plus -10*omega roll/pitch damping",
        },
        "control_dt_seconds": CONTROL_DT_SECONDS,
        "maximum_component_regression": MAXIMUM_COMPONENT_REGRESSION,
        "minimum_component_improvement": MINIMUM_COMPONENT_IMPROVEMENT,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "allowed_source_replay_suffixes": ALLOWED_REPLAY_SUFFIXES,
        "completed_per_sample_contact_labels_opened": False,
        "frozen_uncertainty_width_was_fit_on_spent_source_labels": True,
        "design_audit_not_holdout": True,
        "wbc_configuration": WBC_CONFIG,
        "primary_wbc_state_local_queries": sum(row["samples"] for row in results) * CANDIDATE_COUNT,
        "repeat_wbc_state_local_queries": sum(row["samples"] for row in results) * CANDIDATE_COUNT,
        "primary_selector_queries": sum(row["samples"] for row in results),
        "repeat_selector_queries": sum(row["samples"] for row in results),
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "selected_torques_applied": 0,
        "semantic_repeat": repeat,
        "semantic_arrays_bitwise_exact": repeated_arrays,
        "semantic_arrays_compared": semantic_arrays,
        "selected_counts": selected_counts,
        "mechanism_passed": mechanism_passed,
        "action_profile_frozen_for_fresh_plant_ab": mechanism_passed,
        "plant_non_regression_passed": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = [
        [
            row["law"],
            " / ".join(str(row["selected_counts"][name]) for name in CANDIDATE_NAMES),
            str(row["strict_improvement_samples"]),
            f"{row['maximum_component_regression']:.3g}",
            f"{row['component_improvement']['p50']:.3f} / {row['component_improvement']['p99']:.3f}",
            f"{row['wbc_step_timing_ns']['p99'] / 1e6:.3f}",
            f"{row['selector_timing_ns']['p99'] / 1e3:.3f}",
            "PASS" if row["all_candidates_wbc_admitted"] else "FAIL",
        ]
        for row in results
    ]
    report = "\n".join(
        [
            "# Bonesaw terminal-box WBC action audit · r247",
            "",
            f"> State-local mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant non-regression **NOT RUN** · authority **NOT ADMITTED**.",
            "",
            "This design audit evaluates three fixed acceleration laws on all 96 spent R246 pre-impact states. The exact Rust floating WBC admits each law independently, then the R224 componentwise terminal box and R247 Rust selector choose only candidates with zero component regression. No policy is queried, no physics is stepped, and no selected acceleration or torque is applied.",
            "",
            *markdown_table(
                [
                    "law",
                    "selected · zero / damp / recover",
                    "strictly improved",
                    "max regression",
                    "improvement · p50 / p99",
                    "WBC p99 ms",
                    "selector p99 µs",
                    "WBC",
                ],
                rows,
            ),
            "",
            f"Combined selection is **{selected_counts['zero_acceleration']} / {selected_counts['velocity_damping']} / {selected_counts['neutral_recovery']}** over {metrics['samples']} states. The independent full rerun reproduces {repeated_arrays}/{semantic_arrays} non-timing arrays bitwise; WBC and selector hot paths report zero Rust allocation.",
            "",
            "Causality boundary: the evaluator opens only R246 root height and model-predicted contact activation per sample. Its uncertainty widths were already fit on the spent R246 completed labels, so this is deliberately not fresh evidence. The frozen candidate laws must next face a new MuJoCo baseline/candidate plant matrix before command admission can be reconsidered.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-terminal-box-wbc-action-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-terminal-box-wbc-action-audit.npz", **first)
    (output / "G1_TERMINAL_BOX_WBC_ACTION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw terminal-box WBC audit · r247"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "selected_counts": selected_counts,
                "plant_non_regression_passed": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
