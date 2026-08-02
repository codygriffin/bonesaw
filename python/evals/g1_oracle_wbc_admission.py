#!/usr/bin/env python3
"""Policy-free, simulator-free G1 morphology and WBC admission corpus."""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import pathlib
import platform
import resource
import sys
import time
import tracemalloc
from typing import Any

import numpy as np

from reference_comparison import standing_posture


DT = 0.005
STATUS_NAMES = ("solved", "solved_with_slack", "primal_infeasible", "failed")
TASK_NAMES = (
    "root_angular",
    "root_horizontal",
    "root_height",
    "joint_posture",
    "protected_joint_acceleration",
    "center_of_mass",
    "centroidal_angular_momentum_rate",
    "contact_force_style",
    "actuator_torque_style",
    "frame_angular_0",
    "point_0",
    "point_1",
    "point_2",
    "point_3",
)
PRIORITY_NAMES = ("invariant", "viability", "intent", "preference", "style")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf",
    )
    parser.add_argument(
        "--reference-inputs",
        default="benchmarks/results/g1-bonesaw-lipm-r44/reference-inputs.npz",
    )
    parser.add_argument(
        "--output",
        default="benchmarks/results/g1-oracle-wbc-admission-r45",
    )
    parser.add_argument("--left-foot-frame", default="left_ankle_roll_link")
    parser.add_argument("--right-foot-frame", default="right_ankle_roll_link")
    parser.add_argument("--ik-maximum-iterations", type=int, default=96)
    parser.add_argument("--ik-minimum-iterations", type=int, default=0)
    parser.add_argument("--ik-damping", type=float, default=1e-5)
    parser.add_argument("--ik-posture-weight", type=float, default=1e-5)
    parser.add_argument("--ik-center-of-mass-weight", type=float, default=0.25)
    parser.add_argument("--ik-maximum-step", type=float, default=0.04)
    parser.add_argument("--ik-effector-weight", type=float, default=100.0)
    parser.add_argument("--ik-point-tolerance", type=float, default=5e-3)
    parser.add_argument("--ik-center-of-mass-tolerance", type=float, default=3e-2)
    parser.add_argument("--ik-orientation-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--jet-velocity-damping", type=float, default=1e-4)
    parser.add_argument("--jet-acceleration-damping", type=float, default=1e-3)
    parser.add_argument("--jet-effector-weight", type=float, default=100.0)
    parser.add_argument("--jet-center-of-mass-weight", type=float, default=5.0)
    parser.add_argument("--jet-off-chain-regularization", type=float, default=1.0)
    parser.add_argument(
        "--jet-strict-effectors",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "diagnostic hierarchical morphology projection; the default weighted "
            "least-squares jet avoids trading unbounded joint motion for exact endpoints"
        ),
    )
    parser.add_argument("--wbc-root-angular-weight", type=float, default=1.0)
    parser.add_argument("--wbc-root-angular-priority", type=int, default=0)
    parser.add_argument("--wbc-root-horizontal-weight", type=float, default=1.0)
    parser.add_argument("--wbc-root-height-weight", type=float, default=1.0)
    parser.add_argument("--wbc-root-height-priority", type=int, default=0)
    parser.add_argument("--wbc-center-of-mass-weight", type=float, default=1.0)
    parser.add_argument(
        "--wbc-centroidal-angular-momentum-weight",
        type=float,
        default=0.0,
        help="diagnostic zero centroidal angular-momentum-rate task weight",
    )
    parser.add_argument("--wbc-effector-weight", type=float, default=1.0)
    parser.add_argument("--wbc-friction-coefficient", type=float, default=1.0)
    parser.add_argument("--minimum-contact-cop-margin", type=float, default=0.005)
    parser.add_argument("--minimum-contact-transitions", type=int, default=2)
    parser.add_argument("--minimum-alternating-liftoffs", type=int, default=1)
    parser.add_argument("--transition-window-ticks", type=int, default=8)
    parser.add_argument(
        "--loose-bound-control",
        default=(
            "benchmarks/results/g1-oracle-wbc-admission-r45-margin5mm/"
            "oracle-wbc-admission-raw.npz"
        ),
        help="retained loose-2000-Nm oracle trace used only as a labeled A/B control",
    )
    parser.add_argument(
        "--coupled-actuation-pair",
        nargs=2,
        metavar=("ACTUATOR_A", "ACTUATOR_B"),
        help=(
            "replace the two named identity coordinates with the fixed synthetic "
            "differential block [[0.5, 0.5], [-1, 1]]"
        ),
    )
    parser.add_argument(
        "--coupled-pair-effort-limit-nm",
        type=float,
        default=18.0,
        help="symmetric actuator-space effort limit for both synthetic differential outputs",
    )
    return parser.parse_args()


def distribution_us(values_ns: np.ndarray) -> dict[str, float]:
    values_us = np.asarray(values_ns, dtype=np.float64) / 1_000.0
    return {
        "minimum": float(np.min(values_us)),
        "p50": float(np.percentile(values_us, 50)),
        "p95": float(np.percentile(values_us, 95)),
        "p99": float(np.percentile(values_us, 99)),
        "maximum": float(np.max(values_us)),
        "mean": float(np.mean(values_us)),
    }


def distribution(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    finite = values[np.isfinite(values)]
    if not len(finite):
        return {name: float("nan") for name in ("minimum", "p01", "p50", "p95", "p99", "maximum", "mean")}
    return {
        "minimum": float(np.min(finite)),
        "p01": float(np.percentile(finite, 1)),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "maximum": float(np.max(finite)),
        "mean": float(np.mean(finite)),
    }


def threshold_curve(
    values: np.ndarray,
    thresholds: list[float],
    *,
    adverse: str,
    tolerance: float = 0.0,
) -> list[dict[str, float | int]]:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    rows = []
    for threshold in thresholds:
        adverse_mask = (
            finite < threshold - tolerance
            if adverse == "below"
            else finite > threshold + tolerance
        )
        count = int(np.count_nonzero(adverse_mask))
        rows.append(
            {
                "threshold": threshold,
                "adverse_ticks": count,
                "adverse_fraction": count / max(len(finite), 1),
            }
        )
    return rows


def longest_true_run(mask: np.ndarray) -> int:
    longest = 0
    current = 0
    for active in np.asarray(mask, dtype=np.bool_):
        current = current + 1 if active else 0
        longest = max(longest, current)
    return longest


def contact_transition_metrics(
    contacts: np.ndarray,
    target_positions: np.ndarray,
    target_velocities: np.ndarray,
    target_accelerations: np.ndarray,
    point_error: np.ndarray,
    center_of_mass_error: np.ndarray,
    outputs: dict[str, np.ndarray],
    window_ticks: int,
) -> dict[str, Any]:
    active = np.asarray(contacts, dtype=np.bool_)
    liftoff_edges = np.argwhere(active[:-1] & ~active[1:])
    touchdown_edges = np.argwhere(~active[:-1] & active[1:])
    transitions = sorted(
        [(int(row + 1), int(foot), "liftoff") for row, foot in liftoff_edges]
        + [(int(row + 1), int(foot), "touchdown") for row, foot in touchdown_edges]
    )
    selected = np.zeros(len(active), dtype=np.bool_)
    rows: list[dict[str, Any]] = []
    for edge, foot, kind in transitions:
        start = max(0, edge - window_ticks)
        stop = min(len(active), edge + window_ticks + 1)
        selected[start:stop] = True
        statuses = outputs["status"][start:stop]
        rows.append(
            {
                "tick": edge,
                "foot": foot,
                "kind": kind,
                "window": [start, stop - 1],
                "position_edge_delta_m": float(
                    np.linalg.norm(
                        target_positions[edge, foot]
                        - target_positions[edge - 1, foot]
                    )
                ),
                "velocity_edge_delta_mps": float(
                    np.linalg.norm(
                        target_velocities[edge, foot]
                        - target_velocities[edge - 1, foot]
                    )
                ),
                "acceleration_edge_delta_mps2": float(
                    np.linalg.norm(
                        target_accelerations[edge, foot]
                        - target_accelerations[edge - 1, foot]
                    )
                ),
                "maximum_point_error_m": float(np.max(point_error[start:stop])),
                "maximum_center_of_mass_error_m": float(
                    np.max(center_of_mass_error[start:stop])
                ),
                "solved_ticks": int(np.count_nonzero(statuses <= 1)),
                "window_ticks": int(len(statuses)),
                "maximum_dynamics_residual": float(
                    np.max(outputs["dynamics_residual"][start:stop])
                ),
                "maximum_contact_residual": float(
                    np.max(outputs["contact_residual"][start:stop])
                ),
                "minimum_support_margin_m": float(
                    np.min(outputs["minimum_support_margin"][start:stop])
                ),
                "maximum_actuator_utilization": float(
                    np.max(outputs["maximum_torque_utilization"][start:stop])
                ),
                "maximum_latency_us": float(
                    np.max(outputs["step_ns"][start:stop]) / 1_000.0
                ),
                "clipped_ticks": int(
                    np.count_nonzero(outputs["clipped_steps"][start:stop])
                ),
            }
        )
    edge_position = [row["position_edge_delta_m"] for row in rows]
    edge_velocity = [row["velocity_edge_delta_mps"] for row in rows]
    edge_acceleration = [row["acceleration_edge_delta_mps2"] for row in rows]
    selected_count = int(np.count_nonzero(selected))
    liftoff_feet = [int(foot) for _, foot in liftoff_edges]
    alternating_liftoffs = all(
        current != previous
        for previous, current in zip(liftoff_feet, liftoff_feet[1:])
    )
    step_rows: list[dict[str, Any]] = []
    previous_touchdown = -1
    for step, ((liftoff_row, liftoff_foot), (touchdown_row, touchdown_foot)) in enumerate(
        zip(liftoff_edges, touchdown_edges, strict=False)
    ):
        liftoff_tick = int(liftoff_row + 1)
        touchdown_tick = int(touchdown_row + 1)
        start = previous_touchdown + 1
        stop = touchdown_tick + 1
        status = outputs["status"][start:stop]
        step_rows.append(
            {
                "step": step,
                "foot": int(liftoff_foot),
                "matching_touchdown_foot": int(touchdown_foot),
                "start_tick": start,
                "liftoff_tick": liftoff_tick,
                "touchdown_tick": touchdown_tick,
                "ticks": stop - start,
                "solved": int(np.count_nonzero(status == 0)),
                "solved_with_slack": int(np.count_nonzero(status == 1)),
                "infeasible_or_failed": int(np.count_nonzero(status >= 2)),
                "clipped_ticks": int(
                    np.count_nonzero(outputs["clipped_steps"][start:stop])
                ),
                "maximum_dynamics_residual": float(
                    np.max(outputs["dynamics_residual"][start:stop])
                ),
                "maximum_contact_residual": float(
                    np.max(outputs["contact_residual"][start:stop])
                ),
                "minimum_support_margin_m": float(
                    np.min(outputs["minimum_support_margin"][start:stop])
                ),
                "maximum_actuator_utilization": float(
                    np.max(outputs["maximum_torque_utilization"][start:stop])
                ),
                "latency_us": distribution_us(outputs["step_ns"][start:stop]),
                "maximum_task_rms": {
                    TASK_NAMES[slot]: float(np.max(outputs["task_rms"][start:stop, slot]))
                    for slot in (0, 1, 2, 5, 9, 10, 11)
                },
            }
        )
        previous_touchdown = touchdown_tick
    return {
        "count": len(transitions),
        "liftoff_count": len(liftoff_edges),
        "touchdown_count": len(touchdown_edges),
        "liftoff_feet": liftoff_feet,
        "alternating_liftoffs": alternating_liftoffs,
        "flight_ticks": int(np.count_nonzero(~np.any(active, axis=1))),
        "double_support_ticks": int(np.count_nonzero(np.all(active, axis=1))),
        "maximum_position_edge_delta_m": max(edge_position, default=0.0),
        "maximum_velocity_edge_delta_mps": max(edge_velocity, default=0.0),
        "maximum_acceleration_edge_delta_mps2": max(edge_acceleration, default=0.0),
        "window_radius_ticks": window_ticks,
        "union_window_ticks": selected_count,
        "window_status_counts": {
            name: int(np.count_nonzero(outputs["status"][selected] == code))
            for code, name in enumerate(STATUS_NAMES)
        },
        "maximum_dynamics_residual_near_edges": float(
            np.max(outputs["dynamics_residual"][selected]) if selected_count else 0.0
        ),
        "maximum_contact_residual_near_edges": float(
            np.max(outputs["contact_residual"][selected]) if selected_count else 0.0
        ),
        "minimum_support_margin_near_edges_m": float(
            np.min(outputs["minimum_support_margin"][selected])
            if selected_count
            else np.inf
        ),
        "maximum_actuator_utilization_near_edges": float(
            np.max(outputs["maximum_torque_utilization"][selected])
            if selected_count
            else 0.0
        ),
        "maximum_latency_near_edges_us": float(
            np.max(outputs["step_ns"][selected]) / 1_000.0
            if selected_count
            else 0.0
        ),
        "clipped_ticks_near_edges": int(
            np.count_nonzero(outputs["clipped_steps"][selected])
        ),
        "transitions": rows,
        "steps": step_rows,
    }


def index_counts(indices: np.ndarray, names: list[str]) -> list[dict[str, Any]]:
    rows = []
    for index, count in zip(*np.unique(indices, return_counts=True), strict=True):
        if index == np.iinfo(indices.dtype).max:
            continue
        rows.append({"index": int(index), "name": names[int(index)], "ticks": int(count)})
    return sorted(rows, key=lambda row: (-row["ticks"], row["index"]))


def array_fingerprint(arrays: list[np.ndarray]) -> str:
    digest = hashlib.sha256()
    for value in arrays:
        contiguous = np.ascontiguousarray(value)
        digest.update(str(contiguous.dtype).encode())
        digest.update(str(contiguous.shape).encode())
        digest.update(contiguous.view(np.uint8))
    return digest.hexdigest()


def allocate_wbc_outputs(ticks: int, dof: int, contacts: int, tasks: int) -> dict[str, np.ndarray]:
    return {
        "generalized_acceleration": np.empty((ticks, dof + 6), dtype=np.float64),
        "actuator_torque": np.empty((ticks, dof), dtype=np.float64),
        "contact_normal_force": np.empty((ticks, contacts), dtype=np.float64),
        "task_rms": np.empty((ticks, tasks), dtype=np.float64),
        "task_clipped": np.empty((ticks, tasks), dtype=np.uint8),
        "dynamics_residual": np.empty(ticks, dtype=np.float64),
        "contact_residual": np.empty(ticks, dtype=np.float64),
        "minimum_friction_margin": np.empty(ticks, dtype=np.float64),
        "minimum_support_margin": np.empty(ticks, dtype=np.float64),
        "minimum_torque_margin": np.empty(ticks, dtype=np.float64),
        "maximum_constraint_violation": np.empty(ticks, dtype=np.float64),
        "minimum_bound_margin": np.empty(ticks, dtype=np.float64),
        "minimum_joint_margin_rad": np.empty(ticks, dtype=np.float64),
        "minimum_joint_headroom_fraction": np.empty(ticks, dtype=np.float64),
        "limiting_joint": np.empty(ticks, dtype=np.uint16),
        "maximum_torque_utilization": np.empty(ticks, dtype=np.float64),
        "minimum_torque_headroom": np.empty(ticks, dtype=np.float64),
        "limiting_actuator": np.empty(ticks, dtype=np.uint16),
        "witness_acceleration_rms": np.empty(ticks, dtype=np.float64),
        "step_ns": np.empty(ticks, dtype=np.uint64),
        "status": np.empty(ticks, dtype=np.uint8),
        "task_pseudoinverse_calls": np.empty(ticks, dtype=np.uint16),
        "task_pseudoinverse_calls_by_priority": np.empty((ticks, 5), dtype=np.uint16),
        "clipped_steps": np.empty(ticks, dtype=np.uint16),
        "clipped_steps_by_priority": np.empty((ticks, 5), dtype=np.uint16),
        "task_jacobi_sweeps": np.empty(ticks, dtype=np.uint16),
        "task_jacobi_sweeps_by_priority": np.empty((ticks, 5), dtype=np.uint16),
        "feasibility_projection_sweeps": np.empty(ticks, dtype=np.uint16),
        "feasibility_halfspace_projections": np.empty(ticks, dtype=np.uint32),
        "feasibility_polish_iterations": np.empty(ticks, dtype=np.uint16),
        "allocation_calls": np.empty(ticks, dtype=np.uint64),
        "allocated_bytes": np.empty(ticks, dtype=np.uint64),
    }


def run_oracle(
    session: Any,
    root_positions: np.ndarray,
    root_velocities: np.ndarray,
    root_accelerations: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    joint_accelerations: np.ndarray,
    frame_ids: np.ndarray,
    contacts: np.ndarray,
    effector_weight: float,
    root_angular_priority: int,
    root_height_priority: int,
    outputs: dict[str, np.ndarray],
    fixed_actuator_effort: np.ndarray | None = None,
    realization_reference_acceleration: np.ndarray | None = None,
    contact_force_basis_out: np.ndarray | None = None,
    realization_reference_contact_force_basis: np.ndarray | None = None,
    root_quaternions_wxyz: np.ndarray | None = None,
    root_angular_velocities_world: np.ndarray | None = None,
    root_angular_accelerations_world: np.ndarray | None = None,
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
        np.ones(len(frame_ids), dtype=np.uint8),
        np.full(len(frame_ids), effector_weight, dtype=np.float64),
        root_angular_priority,
        root_height_priority,
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
        fixed_actuator_effort,
        realization_reference_acceleration,
        contact_force_basis_out,
        realization_reference_contact_force_basis,
        root_quaternions_wxyz=root_quaternions_wxyz,
        root_angular_velocities_world=root_angular_velocities_world,
        root_angular_accelerations_world=root_angular_accelerations_world,
    )


def main() -> int:
    args = parse_args()
    if (
        args.minimum_contact_transitions < 0
        or args.minimum_alternating_liftoffs < 0
        or args.transition_window_ticks < 0
    ):
        raise ValueError("contact-transition counts and window must be nonnegative")
    if any(
        not np.isfinite(weight) or weight <= 0.0
        for weight in (
            args.ik_effector_weight,
            args.jet_effector_weight,
            args.wbc_root_angular_weight,
            args.wbc_root_horizontal_weight,
            args.wbc_root_height_weight,
            args.wbc_center_of_mass_weight,
            args.wbc_effector_weight,
            args.wbc_friction_coefficient,
        )
    ):
        raise ValueError("oracle task weights must be finite and positive")
    if (
        not np.isfinite(args.wbc_centroidal_angular_momentum_weight)
        or args.wbc_centroidal_angular_momentum_weight < 0.0
    ):
        raise ValueError(
            "centroidal angular-momentum task weight must be finite and nonnegative"
        )
    if not all(
        0 <= value <= 4
        for value in (args.wbc_root_angular_priority, args.wbc_root_height_priority)
    ):
        raise ValueError("root task priorities must be in 0..=4")
    import bonesaw

    model = pathlib.Path(args.model).resolve()
    reference_path = pathlib.Path(args.reference_inputs).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    with np.load(reference_path) as source:
        reference = {key: np.asarray(source[key]).copy() for key in source.files}
    root_positions = reference["root_targets"]
    center_of_mass_positions = reference["center_of_mass_targets"]
    target_positions = reference["target_positions"]
    target_accelerations = reference["target_accelerations"]
    contacts = reference["reference_stance"].astype(np.uint8, copy=False)
    ticks = len(root_positions)
    if ticks < 3 or target_positions.shape != (ticks, 2, 3):
        raise ValueError("the oracle admission corpus expects a two-foot standalone trace")
    immutable_arrays = list(reference.values())
    input_fingerprint_before = array_fingerprint(immutable_arrays)

    witness = bonesaw.KinematicWitnessSession(str(model), target_capacity=2)
    frame_names = list(witness.frame_names)
    joint_names = list(witness.joint_names)
    frame_ids = np.asarray(
        [
            frame_names.index(args.left_foot_frame),
            frame_names.index(args.right_foot_frame),
        ],
        dtype=np.int64,
    )
    initial_posture = standing_posture(joint_names)
    dof = witness.dof
    q = np.empty((ticks, dof), dtype=np.float64)
    v = np.empty_like(q)
    joint_accelerations = np.empty_like(q)
    point_error = np.empty(ticks, dtype=np.float64)
    orientation_error = np.empty(ticks, dtype=np.float64)
    center_of_mass_error = np.empty(ticks, dtype=np.float64)
    ik_iterations = np.empty(ticks, dtype=np.uint16)
    ik_converged = np.empty(ticks, dtype=np.uint8)
    ik_solve_ns = np.empty(ticks, dtype=np.uint64)
    ik_allocation_calls = np.empty(ticks, dtype=np.uint64)
    ik_allocated_bytes = np.empty(ticks, dtype=np.uint64)
    witness.run_trace(
        DT,
        root_positions,
        center_of_mass_positions,
        frame_ids,
        target_positions,
        np.full(2, args.ik_effector_weight, dtype=np.float64),
        initial_posture,
        q,
        v,
        joint_accelerations,
        point_error,
        orientation_error,
        center_of_mass_error,
        ik_iterations,
        ik_converged,
        ik_solve_ns,
        ik_allocation_calls,
        ik_allocated_bytes,
        maximum_iterations=args.ik_maximum_iterations,
        minimum_iterations=args.ik_minimum_iterations,
        damping=args.ik_damping,
        posture_weight=args.ik_posture_weight,
        center_of_mass_weight=args.ik_center_of_mass_weight,
        orientation_weight=1.0,
        maximum_step_rad=args.ik_maximum_step,
        point_tolerance_m=args.ik_point_tolerance,
        center_of_mass_tolerance_m=args.ik_center_of_mass_tolerance,
        orientation_tolerance_rad=np.deg2rad(args.ik_orientation_tolerance_deg),
    )
    root_velocities = reference.get(
        "root_target_velocities", reference["center_of_mass_target_velocities"]
    ).copy()
    root_accelerations = reference.get(
        "root_target_accelerations", reference["center_of_mass_target_accelerations"]
    ).copy()
    point_velocity_residual = np.empty(ticks, dtype=np.float64)
    point_acceleration_residual = np.empty(ticks, dtype=np.float64)
    angular_velocity_residual = np.empty(ticks, dtype=np.float64)
    angular_acceleration_residual = np.empty(ticks, dtype=np.float64)
    center_of_mass_velocity_residual = np.empty(ticks, dtype=np.float64)
    center_of_mass_acceleration_residual = np.empty(ticks, dtype=np.float64)
    jet_solve_ns = np.empty(ticks, dtype=np.uint64)
    jet_allocation_calls = np.empty(ticks, dtype=np.uint64)
    jet_allocated_bytes = np.empty(ticks, dtype=np.uint64)
    witness.run_jet_trace(
        root_positions,
        root_velocities,
        root_accelerations,
        reference["center_of_mass_target_velocities"],
        reference["center_of_mass_target_accelerations"],
        frame_ids,
        reference["target_velocities"],
        target_accelerations,
        np.full(2, args.jet_effector_weight, dtype=np.float64),
        q,
        v,
        joint_accelerations,
        point_velocity_residual,
        point_acceleration_residual,
        angular_velocity_residual,
        angular_acceleration_residual,
        center_of_mass_velocity_residual,
        center_of_mass_acceleration_residual,
        jet_solve_ns,
        jet_allocation_calls,
        jet_allocated_bytes,
        args.jet_velocity_damping,
        args.jet_acceleration_damping,
        args.jet_center_of_mass_weight,
        args.jet_off_chain_regularization,
        args.jet_strict_effectors,
    )

    contact_patch_points_per_foot = 4
    maximum_contacts = 2 * contact_patch_points_per_foot
    wbc = bonesaw.FloatingWbcSession(
        str(model),
        maximum_contacts=maximum_contacts,
        friction_coefficient=args.wbc_friction_coefficient,
        maximum_acceleration=200.0,
        maximum_torque=2_000.0,
        maximum_normal_force_multiple=3.0,
        root_angular_task_weight=args.wbc_root_angular_weight,
        root_height_task_weight=args.wbc_root_height_weight,
        root_horizontal_task_weight=args.wbc_root_horizontal_weight,
        root_horizontal_task_priority=1,
        joint_posture_weight=0.01,
        joint_posture_priority=3,
        center_of_mass_task_weight=args.wbc_center_of_mass_weight,
        center_of_mass_task_priority=1,
        centroidal_angular_momentum_weight=args.wbc_centroidal_angular_momentum_weight,
        centroidal_angular_momentum_priority=1,
        contact_patch_center_x=0.035,
        contact_patch_half_length=0.085,
        contact_patch_half_width=0.0275,
        contact_patch_z=-0.035,
        minimum_contact_cop_margin_m=args.minimum_contact_cop_margin,
    )
    wbc_joint_names = list(wbc.joint_names)
    if wbc_joint_names != joint_names:
        raise RuntimeError("kinematic witness and WBC coordinate layouts differ")
    coupled_actuation: dict[str, Any] | None = None
    coupled_forward = np.eye(dof, dtype=np.float64)
    coupled_reverse = np.eye(dof, dtype=np.float64)
    if args.coupled_actuation_pair is not None:
        if (
            not np.isfinite(args.coupled_pair_effort_limit_nm)
            or args.coupled_pair_effort_limit_nm <= 0.0
        ):
            raise ValueError("coupled pair effort limit must be finite and positive")
        try:
            first_actuator, second_actuator = (
                joint_names.index(name) for name in args.coupled_actuation_pair
            )
        except ValueError as error:
            raise ValueError(
                f"unknown coupled actuator in {args.coupled_actuation_pair}"
            ) from error
        if first_actuator == second_actuator:
            raise ValueError("coupled actuator pair must name two distinct coordinates")
        coupled_forward[first_actuator, first_actuator] = 0.5
        coupled_forward[first_actuator, second_actuator] = 0.5
        coupled_forward[second_actuator, first_actuator] = -1.0
        coupled_forward[second_actuator, second_actuator] = 1.0
        coupled_reverse = np.linalg.inv(coupled_forward)
        configured_limits = np.asarray(wbc.actuator_effort_limits, dtype=np.float64)
        configured_limits[first_actuator] = args.coupled_pair_effort_limit_nm
        configured_limits[second_actuator] = args.coupled_pair_effort_limit_nm
        wbc.configure_coupled_actuation(coupled_forward, configured_limits)
        coupled_actuation = {
            "synthetic_fixture": True,
            "calibrated_robot_transmission": False,
            "actuator_pair": list(args.coupled_actuation_pair),
            "coordinate_indices": [first_actuator, second_actuator],
            "forward_generalized_from_actuator_block": [
                [0.5, 0.5],
                [-1.0, 1.0],
            ],
            "reverse_actuator_from_generalized_block": [
                [1.0, -0.5],
                [1.0, 0.5],
            ],
            "pair_effort_limit_nm": args.coupled_pair_effort_limit_nm,
        }
    actuator_effort_limits = np.asarray(wbc.actuator_effort_limits, dtype=np.float64)
    if actuator_effort_limits.shape != (dof,) or np.any(actuator_effort_limits <= 0.0):
        raise RuntimeError("WBC did not expose positive actuator effort limits")
    first = allocate_wbc_outputs(ticks, dof, maximum_contacts, wbc.task_diagnostic_capacity)
    repeat = allocate_wbc_outputs(ticks, dof, maximum_contacts, wbc.task_diagnostic_capacity)
    gc.collect()
    gc_before = gc.get_count()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    tracemalloc.start()
    wall_before = time.perf_counter_ns()
    cpu_before = time.process_time_ns()
    run_oracle(
        wbc,
        root_positions,
        root_velocities,
        root_accelerations,
        q,
        v,
        joint_accelerations,
        frame_ids,
        contacts,
        args.wbc_effector_weight,
        args.wbc_root_angular_priority,
        args.wbc_root_height_priority,
        first,
    )
    cpu_after = time.process_time_ns()
    wall_after = time.perf_counter_ns()
    _, python_traced_peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    gc_after = gc.get_count()
    run_oracle(
        wbc,
        root_positions,
        root_velocities,
        root_accelerations,
        q,
        v,
        joint_accelerations,
        frame_ids,
        contacts,
        args.wbc_effector_weight,
        args.wbc_root_angular_priority,
        args.wbc_root_height_priority,
        repeat,
    )
    repeat_fields = [
        key
        for key in first
        if key not in {"step_ns"}
    ]
    exact_repeat = all(np.array_equal(first[key], repeat[key]) for key in repeat_fields)
    input_fingerprint_after = array_fingerprint(immutable_arrays)
    status_counts = {
        name: int(np.count_nonzero(first["status"] == code))
        for code, name in enumerate(STATUS_NAMES)
    }
    task_maximum = {
        TASK_NAMES[index]: float(np.max(first["task_rms"][:, index]))
        for index in range(min(len(TASK_NAMES), first["task_rms"].shape[1]))
    }
    viability_slots = (0, 1, 2, 5, 10, 11)
    maximum_viability_task_rms = float(
        np.max(first["task_rms"][:, viability_slots])
    )
    maximum_authored_point_velocity = float(
        np.max(np.linalg.norm(reference["target_velocities"], axis=2))
    )
    maximum_authored_point_acceleration = float(
        np.max(np.linalg.norm(reference["target_accelerations"], axis=2))
    )
    maximum_authored_com_velocity = float(
        np.max(np.linalg.norm(reference["center_of_mass_target_velocities"], axis=1))
    )
    maximum_authored_com_acceleration = float(
        np.max(np.linalg.norm(reference["center_of_mass_target_accelerations"], axis=1))
    )
    per_actuator_utilization = (
        np.abs(first["actuator_torque"]) / actuator_effort_limits[np.newaxis, :]
    )
    if coupled_actuation is not None:
        actuator_velocity = v @ coupled_reverse.T
        reconstructed_generalized_effort = first["actuator_torque"] @ coupled_reverse
        generalized_power = np.sum(reconstructed_generalized_effort * v, axis=1)
        actuator_power = np.sum(first["actuator_torque"] * actuator_velocity, axis=1)
        coupled_power_error_w = float(
            np.max(np.abs(generalized_power - actuator_power))
        )
        coupled_indices = coupled_actuation["coordinate_indices"]
        coupled_pair_utilization = np.max(
            per_actuator_utilization[:, coupled_indices], axis=1
        )
        coupled_actuation.update(
            {
                "maximum_power_identity_error_w": coupled_power_error_w,
                "pair_utilization": distribution(coupled_pair_utilization),
                "ticks_above_99pct": int(
                    np.count_nonzero(coupled_pair_utilization >= 0.99)
                ),
                "longest_above_99pct_run_ticks": longest_true_run(
                    coupled_pair_utilization >= 0.99
                ),
            }
        )
    maximum_hard_residual = np.maximum.reduce(
        (
            first["dynamics_residual"],
            first["contact_residual"],
            first["maximum_constraint_violation"],
        )
    )
    support_reserve = first["minimum_support_margin"] - args.minimum_contact_cop_margin
    transitions = contact_transition_metrics(
        contacts,
        target_positions,
        reference["target_velocities"],
        target_accelerations,
        point_error,
        center_of_mass_error,
        first,
        args.transition_window_ticks,
    )
    task_clipped_counts = {
        TASK_NAMES[slot]: int(np.count_nonzero(first["task_clipped"][:, slot]))
        for slot in range(min(len(TASK_NAMES), first["task_clipped"].shape[1]))
    }
    solver_by_priority = {
        name: {
            "task_pseudoinverse_calls": distribution(
                first["task_pseudoinverse_calls_by_priority"][:, priority]
            ),
            "task_jacobi_sweeps": distribution(
                first["task_jacobi_sweeps_by_priority"][:, priority]
            ),
            "clipped_steps": distribution(
                first["clipped_steps_by_priority"][:, priority]
            ),
            "ticks_with_clipping": int(
                np.count_nonzero(first["clipped_steps_by_priority"][:, priority])
            ),
        }
        for priority, name in enumerate(PRIORITY_NAMES)
    }
    per_actuator = sorted(
        (
            {
                "coordinate": coordinate,
                "name": joint_names[coordinate],
                "effort_limit_nm": float(actuator_effort_limits[coordinate]),
                "maximum_utilization": float(np.max(per_actuator_utilization[:, coordinate])),
                "p99_utilization": float(
                    np.percentile(per_actuator_utilization[:, coordinate], 99)
                ),
                "maximum_absolute_torque_nm": float(
                    np.max(np.abs(first["actuator_torque"][:, coordinate]))
                ),
            }
            for coordinate in range(dof)
        ),
        key=lambda row: (-row["maximum_utilization"], row["coordinate"]),
    )
    loose_control_path = pathlib.Path(args.loose_bound_control)
    loose_control: dict[str, Any] = {
        "path": str(loose_control_path.resolve()),
        "available": loose_control_path.exists(),
    }
    if loose_control_path.exists():
        comparison_fields = (
            "generalized_acceleration",
            "actuator_torque",
            "contact_normal_force",
            "task_rms",
            "dynamics_residual",
            "contact_residual",
            "minimum_friction_margin",
            "minimum_support_margin",
            "status",
        )
        with np.load(loose_control_path) as loose:
            available_fields = [field for field in comparison_fields if field in loose.files]
            compatible_fields = [
                field
                for field in available_fields
                if first[field].shape == loose[field].shape
            ]
            loose_control.update(
                {
                    "compared_fields": available_fields,
                    "shape_compatible_fields": compatible_fields,
                    "shape_mismatch_fields": [
                        field
                        for field in available_fields
                        if field not in compatible_fields
                    ],
                    "exact_fields": {
                        field: bool(
                            field in compatible_fields
                            and np.array_equal(first[field], loose[field])
                        )
                        for field in available_fields
                    },
                    "maximum_absolute_delta": {
                        field: float(
                            np.max(
                                np.abs(
                                    first[field].astype(np.float64)
                                    - loose[field].astype(np.float64)
                                )
                            )
                        )
                        for field in compatible_fields
                    },
                }
            )
            loose_control["all_compared_fields_exact"] = all(
                loose_control["exact_fields"].values()
            )
    task_thresholds = {
        "root_angular": 1.0,
        "root_horizontal": 1.0,
        "root_height": 1.0,
        "center_of_mass": 1.0,
        "frame_angular_0": 1.0,
        "point_0": 0.1,
        "point_1": 0.1,
        "point_2": 0.1,
        "point_3": 0.1,
    }
    task_over_time = {}
    for slot, name in enumerate(TASK_NAMES):
        values = first["task_rms"][:, slot]
        evidence: dict[str, Any] = {
            "physical_units": (
                "rad/s^2"
                if name in {"root_angular", "frame_angular_0"}
                else "m/s^2"
                if name in {"root_horizontal", "root_height", "center_of_mass"}
                or name.startswith("point_")
                else "task-specific acceleration units"
            ),
            "distribution": distribution(values),
            "time_rms": float(np.sqrt(np.mean(np.square(values)))),
            "integrated_residual_seconds": float(np.sum(values) * DT),
            "clipped_ticks": int(np.count_nonzero(first["task_clipped"][:, slot])),
        }
        if name in task_thresholds:
            threshold = task_thresholds[name]
            adverse = values > threshold
            evidence.update(
                {
                    "tracking_threshold": threshold,
                    "ticks_above_threshold": int(np.count_nonzero(adverse)),
                    "fraction_above_threshold": float(np.mean(adverse)),
                    "longest_above_threshold_run_ticks": longest_true_run(adverse),
                    "longest_above_threshold_run_seconds": longest_true_run(adverse)
                    * DT,
                }
            )
        task_over_time[name] = evidence

    authority_evidence = {
        "semantics": {
            "raw_not_aggregate": True,
            "thresholds_are_provisional_eval_contracts": True,
            "task_rms_units_are_task_specific": True,
            "model_effort_limits_intersect_global_cap": True,
            "thermal_or_reliability_model": False,
        },
        "hard_constraints": {
            "maximum_residual": distribution(maximum_hard_residual),
            "threshold_curve": threshold_curve(
                maximum_hard_residual,
                [1e-10, 3e-10, 1e-9, 3e-9, 1e-8],
                adverse="above",
            ),
            "warning_threshold": 1e-9,
            "critical_threshold": 1e-8,
            "longest_warning_run_ticks": longest_true_run(maximum_hard_residual > 1e-9),
        },
        "finite_support": {
            "margin_m": distribution(first["minimum_support_margin"]),
            "reserve_beyond_declared_erosion_m": distribution(support_reserve),
            "threshold_curve": threshold_curve(
                first["minimum_support_margin"],
                [0.0, 0.0025, 0.005, 0.0075, 0.01, 0.015, 0.02],
                adverse="below",
                tolerance=1e-12,
            ),
            "warning_threshold_m": args.minimum_contact_cop_margin + 0.0025,
            "critical_threshold_m": args.minimum_contact_cop_margin,
            "longest_warning_run_ticks": longest_true_run(
                first["minimum_support_margin"]
                < args.minimum_contact_cop_margin + 0.0025
            ),
        },
        "joint_position": {
            "minimum_margin_rad": distribution(first["minimum_joint_margin_rad"]),
            "minimum_margin_deg": distribution(
                np.rad2deg(first["minimum_joint_margin_rad"])
            ),
            "minimum_fraction_of_range": distribution(
                first["minimum_joint_headroom_fraction"]
            ),
            "limiting_coordinate_counts": index_counts(first["limiting_joint"], joint_names),
            "threshold_curve_deg": threshold_curve(
                np.rad2deg(first["minimum_joint_margin_rad"]),
                [0.0, 1.0, 2.0, 5.0, 10.0, 15.0],
                adverse="below",
                tolerance=1e-10,
            ),
            "warning_threshold_deg": 5.0,
            "critical_threshold_deg": 0.0,
            "longest_warning_run_ticks": longest_true_run(
                np.rad2deg(first["minimum_joint_margin_rad"]) < 5.0
            ),
        },
        "actuator_effort": {
            "maximum_utilization": distribution(first["maximum_torque_utilization"]),
            "minimum_headroom_nm": distribution(first["minimum_torque_headroom"]),
            "limiting_coordinate_counts": index_counts(
                first["limiting_actuator"], joint_names
            ),
            "per_actuator": per_actuator,
            "threshold_curve": threshold_curve(
                first["maximum_torque_utilization"],
                [0.25, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
                adverse="above",
                tolerance=1e-12,
            ),
            "warning_threshold": 0.8,
            "critical_threshold": 1.0,
            "longest_warning_run_ticks": longest_true_run(
                first["maximum_torque_utilization"] > 0.8
            ),
        },
        "solver_budget": {
            "latency_us": distribution_us(first["step_ns"]),
            "threshold_curve_us": threshold_curve(
                first["step_ns"].astype(np.float64) / 1_000.0,
                [2_000.0, 3_000.0, 4_000.0, 5_000.0, 10_000.0, 20_000.0],
                adverse="above",
            ),
            "warning_threshold_us": 4_000.0,
            "critical_threshold_us": 5_000.0,
            "longest_warning_run_ticks": longest_true_run(first["step_ns"] > 4_000_000),
            "task_pseudoinverse_calls": distribution(first["task_pseudoinverse_calls"]),
            "task_jacobi_sweeps": distribution(first["task_jacobi_sweeps"]),
            "feasibility_projection_sweeps": distribution(
                first["feasibility_projection_sweeps"]
            ),
            "feasibility_halfspace_projections": distribution(
                first["feasibility_halfspace_projections"]
            ),
            "feasibility_polish_iterations": distribution(
                first["feasibility_polish_iterations"]
            ),
            "by_priority": solver_by_priority,
        },
        "task_layers": {
            "clipped_ticks_by_task": task_clipped_counts,
            "raw_maximum_rms_by_task": task_maximum,
            "over_time_by_task": task_over_time,
            "warning_requires_task_specific_normalizer_or_clipping": True,
        },
        "loose_2000nm_control": loose_control,
    }
    metrics: dict[str, Any] = {
        "schema": 3,
        "evaluation_boundary": {
            "feedback_policy": False,
            "state_integration": False,
            "physics_simulator": False,
            "oracle_state_per_tick": True,
            "rigid_body_model_equations": True,
            "finite_support_cop_constraints": True,
            "morphology_projection": True,
            "wbc_targets_derived_from_projected_generalized_jet": True,
            "reference_inputs_immutable": input_fingerprint_before == input_fingerprint_after,
            "actuator_space_effort_inequalities": coupled_actuation is not None,
        },
        "configuration": {
            "position_ik_damping": args.ik_damping,
            "position_ik_minimum_iterations": args.ik_minimum_iterations,
            "position_ik_center_of_mass_weight": args.ik_center_of_mass_weight,
            "jet_velocity_damping": args.jet_velocity_damping,
            "jet_acceleration_damping": args.jet_acceleration_damping,
            "jet_center_of_mass_weight": args.jet_center_of_mass_weight,
            "jet_off_chain_regularization": args.jet_off_chain_regularization,
            "jet_strict_effectors": args.jet_strict_effectors,
            "wbc_root_angular_weight": args.wbc_root_angular_weight,
            "wbc_root_angular_priority": args.wbc_root_angular_priority,
            "wbc_root_horizontal_weight": args.wbc_root_horizontal_weight,
            "wbc_root_height_weight": args.wbc_root_height_weight,
            "wbc_root_height_priority": args.wbc_root_height_priority,
            "wbc_center_of_mass_weight": args.wbc_center_of_mass_weight,
            "wbc_centroidal_angular_momentum_weight": args.wbc_centroidal_angular_momentum_weight,
            "wbc_effector_weight": args.wbc_effector_weight,
            "wbc_friction_coefficient": args.wbc_friction_coefficient,
            "minimum_contact_cop_margin_m": args.minimum_contact_cop_margin,
            "minimum_contact_transitions": args.minimum_contact_transitions,
            "minimum_alternating_liftoffs": args.minimum_alternating_liftoffs,
            "transition_window_ticks": args.transition_window_ticks,
            "global_torque_cap_nm": 2_000.0,
            "torque_bound_semantics": (
                "exact G^T generalized-effort actuator polytope intersected with a generalized 2000 Nm guard"
                if coupled_actuation is not None
                else "minimum of global cap and URDF effort limit"
            ),
            "coupled_actuation": coupled_actuation,
        },
        "model": str(model),
        "reference_inputs": str(reference_path),
        "ticks": ticks,
        "dt_seconds": DT,
        "kinematic_witness": {
            "converged_ticks": int(np.count_nonzero(ik_converged)),
            "maximum_point_error_m": float(np.max(point_error)),
            "point_error_p99_m": float(np.percentile(point_error, 99)),
            "maximum_orientation_error_rad": float(np.max(orientation_error)),
            "maximum_center_of_mass_error_m": float(np.max(center_of_mass_error)),
            "center_of_mass_error_p99_m": float(np.percentile(center_of_mass_error, 99)),
            "maximum_joint_velocity_rad_s": float(np.max(np.abs(v))),
            "maximum_joint_acceleration_rad_s2": float(np.max(np.abs(joint_accelerations))),
            "jet_residuals": {
                "maximum_point_velocity_mps": float(np.max(point_velocity_residual)),
                "maximum_point_acceleration_mps2": float(np.max(point_acceleration_residual)),
                "maximum_angular_velocity_rad_s": float(np.max(angular_velocity_residual)),
                "maximum_angular_acceleration_rad_s2": float(np.max(angular_acceleration_residual)),
                "maximum_center_of_mass_velocity_mps": float(np.max(center_of_mass_velocity_residual)),
                "maximum_center_of_mass_acceleration_mps2": float(np.max(center_of_mass_acceleration_residual)),
                "point_velocity_fraction_of_authored_maximum": float(np.max(point_velocity_residual)) / max(maximum_authored_point_velocity, 1e-12),
                "point_acceleration_fraction_of_authored_maximum": float(np.max(point_acceleration_residual)) / max(maximum_authored_point_acceleration, 1e-12),
                "center_of_mass_velocity_fraction_of_authored_maximum": float(np.max(center_of_mass_velocity_residual)) / max(maximum_authored_com_velocity, 1e-12),
                "center_of_mass_acceleration_fraction_of_authored_maximum": float(np.max(center_of_mass_acceleration_residual)) / max(maximum_authored_com_acceleration, 1e-12),
            },
            "jet_latency_us": distribution_us(jet_solve_ns),
            "jet_allocation_calls": int(np.sum(jet_allocation_calls, dtype=np.uint64)),
            "jet_allocated_bytes": int(np.sum(jet_allocated_bytes, dtype=np.uint64)),
            "iterations": {
                "p50": float(np.percentile(ik_iterations, 50)),
                "p99": float(np.percentile(ik_iterations, 99)),
                "maximum": int(np.max(ik_iterations)),
            },
            "latency_us": distribution_us(ik_solve_ns),
            "allocation_calls": int(np.sum(ik_allocation_calls, dtype=np.uint64)),
            "allocated_bytes": int(np.sum(ik_allocated_bytes, dtype=np.uint64)),
        },
        "wbc_admission": {
            "status_counts": status_counts,
            "maximum_dynamics_residual": float(np.max(first["dynamics_residual"])),
            "maximum_contact_acceleration_residual": float(np.max(first["contact_residual"])),
            "minimum_friction_margin": float(np.min(first["minimum_friction_margin"])),
            "minimum_support_margin_m": float(np.min(first["minimum_support_margin"])),
            "minimum_torque_margin": float(np.min(first["minimum_torque_margin"])),
            "maximum_actuator_torque": float(np.max(np.abs(first["actuator_torque"]))),
            "maximum_contact_normal_force": float(np.max(first["contact_normal_force"])),
            "maximum_witness_acceleration_rms": float(np.max(first["witness_acceleration_rms"])),
            "maximum_viability_task_rms": maximum_viability_task_rms,
            "task_maximum_rms": task_maximum,
            "latency_us": distribution_us(first["step_ns"]),
            "jitter_abs_delta_us": distribution_us(
                np.abs(np.diff(first["step_ns"].astype(np.int64)))
            ),
            "deadline_misses_20ms": int(np.count_nonzero(first["step_ns"] > 20_000_000)),
            "allocation_calls": int(np.sum(first["allocation_calls"], dtype=np.uint64)),
            "allocated_bytes": int(np.sum(first["allocated_bytes"], dtype=np.uint64)),
            "task_pseudoinverse_calls": {
                "mean": float(np.mean(first["task_pseudoinverse_calls"])),
                "maximum": int(np.max(first["task_pseudoinverse_calls"])),
            },
            "clipped_steps": {
                "ticks": int(np.count_nonzero(first["clipped_steps"])),
                "maximum": int(np.max(first["clipped_steps"])),
            },
            "task_jacobi_sweeps": {
                "mean": float(np.mean(first["task_jacobi_sweeps"])),
                "maximum": int(np.max(first["task_jacobi_sweeps"])),
            },
            "feasibility_halfspace_projections": {
                "mean": float(np.mean(first["feasibility_halfspace_projections"])),
                "maximum": int(np.max(first["feasibility_halfspace_projections"])),
            },
        },
        "contact_transitions": transitions,
        "runtime": {
            "wall_seconds": (wall_after - wall_before) / 1e9,
            "process_cpu_seconds": (cpu_after - cpu_before) / 1e9,
            "maximum_rss_before_bytes": rss_before,
            "maximum_rss_after_bytes": rss_after,
            "maximum_rss_growth_bytes": max(0, rss_after - rss_before),
            "python_tracemalloc_peak_bytes": python_traced_peak,
            "python_gc_count_before": list(gc_before),
            "python_gc_count_after": list(gc_after),
            "python": sys.version.split()[0],
            "platform": platform.platform(),
        },
        "exact_repeat": exact_repeat,
        "input_fingerprint": input_fingerprint_before,
        "authority_evidence": authority_evidence,
    }
    checks = {
        "reference_inputs_remain_bitwise_immutable": input_fingerprint_before == input_fingerprint_after,
        "all_kinematic_witness_ticks_converge": metrics["kinematic_witness"]["converged_ticks"] == ticks,
        "kinematic_point_error_le_1cm": metrics["kinematic_witness"]["maximum_point_error_m"] <= 0.01,
        "kinematic_com_error_le_3cm": metrics["kinematic_witness"]["maximum_center_of_mass_error_m"] <= 0.03,
        "kinematic_joint_velocity_le_8rad_s": metrics["kinematic_witness"]["maximum_joint_velocity_rad_s"] <= 8.0,
        "kinematic_joint_acceleration_le_200rad_s2": metrics["kinematic_witness"]["maximum_joint_acceleration_rad_s2"] <= 200.0,
        "kinematic_hot_loop_has_zero_allocations": metrics["kinematic_witness"]["allocation_calls"] == 0,
        "kinematic_jet_hot_loop_has_zero_allocations": metrics["kinematic_witness"]["jet_allocation_calls"] == 0,
        "kinematic_point_velocity_residual_le_10pct": metrics["kinematic_witness"]["jet_residuals"]["point_velocity_fraction_of_authored_maximum"] <= 0.10,
        "kinematic_point_acceleration_residual_le_25pct": metrics["kinematic_witness"]["jet_residuals"]["point_acceleration_fraction_of_authored_maximum"] <= 0.25,
        "kinematic_com_velocity_residual_le_40pct": metrics["kinematic_witness"]["jet_residuals"]["center_of_mass_velocity_fraction_of_authored_maximum"] <= 0.40,
        "kinematic_com_acceleration_residual_le_40pct": metrics["kinematic_witness"]["jet_residuals"]["center_of_mass_acceleration_fraction_of_authored_maximum"] <= 0.40,
        "kinematic_orientation_error_le_1deg": metrics["kinematic_witness"]["maximum_orientation_error_rad"] <= np.deg2rad(1.0),
        "kinematic_angular_velocity_residual_le_1mrad_s": metrics["kinematic_witness"]["jet_residuals"]["maximum_angular_velocity_rad_s"] <= 1e-3,
        "kinematic_angular_acceleration_residual_le_1e_2rad_s2": metrics["kinematic_witness"]["jet_residuals"]["maximum_angular_acceleration_rad_s2"] <= 1e-2,
        "no_wbc_infeasible_or_failed_ticks": status_counts["primal_infeasible"] == 0 and status_counts["failed"] == 0,
        "wbc_dynamics_residual_le_1e_8": metrics["wbc_admission"]["maximum_dynamics_residual"] <= 1e-8,
        "wbc_contact_residual_le_1e_8": metrics["wbc_admission"]["maximum_contact_acceleration_residual"] <= 1e-8,
        "wbc_friction_margin_nonnegative": metrics["wbc_admission"]["minimum_friction_margin"] >= -1e-8,
        "wbc_cop_respects_declared_eroded_support": metrics["wbc_admission"]["minimum_support_margin_m"] >= args.minimum_contact_cop_margin - 1e-8,
        "wbc_torque_margin_nonnegative": metrics["wbc_admission"]["minimum_torque_margin"] >= -1e-8,
        "authority_joint_positions_within_authored_limits": float(
            np.min(first["minimum_joint_margin_rad"])
        )
        >= -1e-10,
        "authority_actuator_effort_within_authored_limits": float(
            np.max(first["maximum_torque_utilization"])
        )
        <= 1.0 + 1e-10,
        "authority_original_constraint_violation_le_1e_8": float(
            np.max(first["maximum_constraint_violation"])
        )
        <= 1e-8,
        "wbc_root_angular_task_rms_le_1rad_s2": task_maximum["root_angular"] <= 1.0,
        "wbc_root_linear_and_com_task_rms_le_1m_s2": max(task_maximum["root_horizontal"], task_maximum["root_height"], task_maximum["center_of_mass"]) <= 1.0,
        "wbc_effector_linear_task_rms_le_0_1m_s2": max(task_maximum["point_0"], task_maximum["point_1"], task_maximum["point_2"], task_maximum["point_3"]) <= 0.1,
        "wbc_effector_angular_task_rms_le_1rad_s2": task_maximum["frame_angular_0"] <= 1.0,
        "wbc_p99_latency_le_20ms": metrics["wbc_admission"]["latency_us"]["p99"] <= 20_000.0,
        "wbc_hot_loop_has_zero_allocations": metrics["wbc_admission"]["allocation_calls"] == 0,
        "physical_outputs_are_bitwise_repeatable": exact_repeat,
        "reference_contact_transitions_meet_declared_minimum": transitions["count"]
        >= args.minimum_contact_transitions,
        "reference_liftoffs_meet_declared_minimum": transitions["liftoff_count"]
        >= args.minimum_alternating_liftoffs,
        "reference_liftoffs_strictly_alternate": transitions[
            "alternating_liftoffs"
        ],
        "reference_has_no_flight_ticks": transitions["flight_ticks"] == 0,
        "contact_edge_position_delta_le_1mm": transitions[
            "maximum_position_edge_delta_m"
        ]
        <= 0.001,
        "contact_edge_velocity_delta_le_0_02mps": transitions[
            "maximum_velocity_edge_delta_mps"
        ]
        <= 0.02,
        "contact_edge_acceleration_delta_le_0_5mps2": transitions[
            "maximum_acceleration_edge_delta_mps2"
        ]
        <= 0.5,
        "wbc_transition_windows_all_solve": transitions["window_status_counts"][
            "primal_infeasible"
        ]
        == 0
        and transitions["window_status_counts"]["failed"] == 0,
        "wbc_transition_dynamics_residual_le_1e_8": transitions[
            "maximum_dynamics_residual_near_edges"
        ]
        <= 1e-8,
        "wbc_transition_contact_residual_le_1e_8": transitions[
            "maximum_contact_residual_near_edges"
        ]
        <= 1e-8,
        "wbc_transition_cop_respects_eroded_support": transitions[
            "minimum_support_margin_near_edges_m"
        ]
        >= args.minimum_contact_cop_margin - 1e-8,
        "wbc_transition_latency_le_20ms": transitions[
            "maximum_latency_near_edges_us"
        ]
        <= 20_000.0,
    }
    if coupled_actuation is not None:
        checks.update(
            {
                "coupled_actuator_power_duality_le_1e_10w": coupled_actuation[
                    "maximum_power_identity_error_w"
                ]
                <= 1e-10,
                "coupled_actuator_polytope_is_materially_active": coupled_actuation[
                    "ticks_above_99pct"
                ]
                > 0,
            }
        )
    checks = {name: bool(passed) for name, passed in checks.items()}
    tracking_check_names = {
        "wbc_root_angular_task_rms_le_1rad_s2",
        "wbc_root_linear_and_com_task_rms_le_1m_s2",
        "wbc_effector_linear_task_rms_le_0_1m_s2",
        "wbc_effector_angular_task_rms_le_1rad_s2",
    }
    tracking_checks = {
        name: checks[name] for name in tracking_check_names
    }
    hard_admission_checks = {
        name: passed
        for name, passed in checks.items()
        if name not in tracking_check_names
    }
    metrics["hard_admission_passed"] = all(hard_admission_checks.values())
    metrics["tracking_passed"] = all(tracking_checks.values())
    metrics["check_groups"] = {
        "hard_admission": hard_admission_checks,
        "tracking": tracking_checks,
    }
    metrics["checks"] = checks
    metrics["passed"] = all(checks.values())
    (output / "oracle-wbc-admission-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "oracle-wbc-admission-raw.npz",
        root_positions=root_positions,
        root_velocities=root_velocities,
        root_accelerations=root_accelerations,
        center_of_mass_positions=center_of_mass_positions,
        target_positions=target_positions,
        target_velocities=reference["target_velocities"],
        target_accelerations=target_accelerations,
        contacts=contacts,
        q=q,
        v=v,
        joint_accelerations=joint_accelerations,
        point_error=point_error,
        orientation_error=orientation_error,
        center_of_mass_error=center_of_mass_error,
        point_velocity_residual=point_velocity_residual,
        point_acceleration_residual=point_acceleration_residual,
        angular_velocity_residual=angular_velocity_residual,
        angular_acceleration_residual=angular_acceleration_residual,
        center_of_mass_velocity_residual=center_of_mass_velocity_residual,
        center_of_mass_acceleration_residual=center_of_mass_acceleration_residual,
        ik_iterations=ik_iterations,
        ik_converged=ik_converged,
        ik_solve_ns=ik_solve_ns,
        jet_solve_ns=jet_solve_ns,
        generalized_from_actuator=coupled_forward,
        actuator_from_generalized=coupled_reverse,
        actuator_effort_limits=actuator_effort_limits,
        **first,
    )
    hard = authority_evidence["hard_constraints"]
    support = authority_evidence["finite_support"]
    joint = authority_evidence["joint_position"]
    effort = authority_evidence["actuator_effort"]
    solver = authority_evidence["solver_budget"]
    task_layers = authority_evidence["task_layers"]["over_time_by_task"]
    control = authority_evidence["loose_2000nm_control"]
    coupled_report = (
        "This run additionally enables a **synthetic coupled-actuation fixture** on "
        f"`{coupled_actuation['actuator_pair'][0]}` / "
        f"`{coupled_actuation['actuator_pair'][1]}` with the differential block "
        "`[[0.5, 0.5], [-1, 1]]`. These are deliberately not claimed as G1 "
        "transmission parameters. Rust enforces both actuator outputs as exact "
        f"inequality rows at ±{coupled_actuation['pair_effort_limit_nm']:.3f} Nm; "
        f"the pair reaches at least 99% utilization for {coupled_actuation['ticks_above_99pct']} "
        "ticks and generalized/actuator mechanical power agrees within "
        f"{coupled_actuation['maximum_power_identity_error_w']:.2e} W."
        if coupled_actuation is not None
        else "This run uses the program's independent actuator bounds; no synthetic coupled transmission is enabled."
    )
    effort_bound_report = (
        "Rust enforces the authored actuator-space box through exact rows of "
        "`Gᵀ τ_generalized`; the synthetic pair uses ±"
        f"{coupled_actuation['pair_effort_limit_nm']:.3f} Nm and all other rows retain their program limits."
        if coupled_actuation is not None
        else "The physical bound used by Rust is `min(global 2000 Nm cap, URDF effort limit)` for each coordinate."
    )
    control_interpretation = (
        "The coupled fixture is expected to differ from the independent loose-bound control when its actuator-space rows become active; comparison deltas are evidence of that intervention, not a parity regression."
        if coupled_actuation is not None
        else "The URDF effort limits certify the existing solution rather than changing it; this does **not** imply that effort, speed, electrical power, temperature, or reliability are interchangeable."
    )
    verdict = (
        "PASS"
        if metrics["passed"]
        else "HARD ADMISSION PASS · TRACKING RED"
        if metrics["hard_admission_passed"]
        else "HARD ADMISSION RED"
    )
    report = [
        "# G1 oracle-state WBC admission",
        "",
        f"**{verdict} · {sum(checks.values())}/{len(checks)} gates**",
        "",
        "This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable reference root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.",
        "",
        coupled_report,
        "",
        "## Results",
        "",
        "| stage | status | geometry / residual | p50 | p99 | max | allocations |",
        "|---|---:|---:|---:|---:|---:|---:|",
        f"| Rust whole-body IK witness | {metrics['kinematic_witness']['converged_ticks']}/{ticks} converged | foot {metrics['kinematic_witness']['maximum_point_error_m'] * 1000:.2f} mm · CoM {metrics['kinematic_witness']['maximum_center_of_mass_error_m'] * 1000:.2f} mm | {metrics['kinematic_witness']['latency_us']['p50']:.1f} µs | {metrics['kinematic_witness']['latency_us']['p99']:.1f} µs | {metrics['kinematic_witness']['latency_us']['maximum']:.1f} µs | {metrics['kinematic_witness']['allocation_calls']} calls / {metrics['kinematic_witness']['allocated_bytes']} B |",
        f"| Rust analytic jet projection | authored residual: foot v {metrics['kinematic_witness']['jet_residuals']['point_velocity_fraction_of_authored_maximum'] * 100:.2f}% · a {metrics['kinematic_witness']['jet_residuals']['point_acceleration_fraction_of_authored_maximum'] * 100:.2f}% · CoM v {metrics['kinematic_witness']['jet_residuals']['center_of_mass_velocity_fraction_of_authored_maximum'] * 100:.2f}% · a {metrics['kinematic_witness']['jet_residuals']['center_of_mass_acceleration_fraction_of_authored_maximum'] * 100:.2f}% | analytic Jv / Jq̈+J̇v | {metrics['kinematic_witness']['jet_latency_us']['p50']:.1f} µs | {metrics['kinematic_witness']['jet_latency_us']['p99']:.1f} µs | {metrics['kinematic_witness']['jet_latency_us']['maximum']:.1f} µs | {metrics['kinematic_witness']['jet_allocation_calls']} calls / {metrics['kinematic_witness']['jet_allocated_bytes']} B |",
        f"| Stateless floating WBC | {status_counts['solved'] + status_counts['solved_with_slack']}/{ticks} solved | dynamics {metrics['wbc_admission']['maximum_dynamics_residual']:.2e} · contact {metrics['wbc_admission']['maximum_contact_acceleration_residual']:.2e} · CoP margin {metrics['wbc_admission']['minimum_support_margin_m'] * 100:.3f} cm | {metrics['wbc_admission']['latency_us']['p50']:.1f} µs | {metrics['wbc_admission']['latency_us']['p99']:.1f} µs | {metrics['wbc_admission']['latency_us']['maximum']:.1f} µs | {metrics['wbc_admission']['allocation_calls']} calls / {metrics['wbc_admission']['allocated_bytes']} B |",
        "",
        f"The maximum viability-task RMS is `{maximum_viability_task_rms:.6f}` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `{metrics['kinematic_witness']['maximum_joint_velocity_rad_s']:.3f} rad/s` / `{metrics['kinematic_witness']['maximum_joint_acceleration_rad_s2']:.3f} rad/s²`. The full generalized-acceleration witness deviation is `{metrics['wbc_admission']['maximum_witness_acceleration_rms']:.3f}` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `{exact_repeat}`.",
        "",
        "### Tracking residuals over execution time",
        "",
        "Peak residual remains the admission rule, while p99, time-RMS, integrated residual, violating ticks, and longest contiguous run describe whether unavailable authority is isolated or persistent. Angular and linear task units are kept separate.",
        "",
        "| task | units | p99 | max | time RMS | integral | above contract | longest run |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        *[
            f"| {name} | {task_layers[name]['physical_units']} | {task_layers[name]['distribution']['p99']:.4f} | {task_layers[name]['distribution']['maximum']:.4f} | {task_layers[name]['time_rms']:.4f} | {task_layers[name]['integrated_residual_seconds']:.4f} | {task_layers[name].get('ticks_above_threshold', 0)} / {ticks} | {task_layers[name].get('longest_above_threshold_run_ticks', 0)} ticks |"
            for name in (
                "root_angular",
                "root_horizontal",
                "root_height",
                "center_of_mass",
                "frame_angular_0",
                "point_0",
                "point_1",
            )
        ],
        "",
        "## Contact-transition windows",
        "",
        f"The authored schedule contains `{transitions['liftoff_count']}` liftoffs and `{transitions['touchdown_count']}` touchdowns (`{transitions['count']}` edges total). A ±{transitions['window_radius_ticks']}-tick window around every edge covers `{transitions['union_window_ticks']}` unique ticks. Liftoff feet are `{transitions['liftoff_feet']}` and strictly alternate: `{transitions['alternating_liftoffs']}`.",
        "",
        f"Across contact edges, the maximum authored position / velocity / acceleration deltas are `{transitions['maximum_position_edge_delta_m'] * 1000:.3f} mm` / `{transitions['maximum_velocity_edge_delta_mps']:.6f} m/s` / `{transitions['maximum_acceleration_edge_delta_mps2']:.6f} m/s²`. Every edge window solves without infeasible/failed status; maximum dynamics/contact residual is `{transitions['maximum_dynamics_residual_near_edges']:.2e}` / `{transitions['maximum_contact_residual_near_edges']:.2e}`, minimum CoP margin is `{transitions['minimum_support_margin_near_edges_m'] * 1000:.3f} mm`, peak actuator utilization is `{transitions['maximum_actuator_utilization_near_edges'] * 100:.2f}%`, and maximum WBC time is `{transitions['maximum_latency_near_edges_us'] / 1000:.3f} ms`.",
        "",
        "| tick | edge | foot | WBC solved | hard dynamics / contact | CoP margin | actuator use | max time | clipped ticks |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|",
        *[
            f"| {row['tick']} | {row['kind']} | {row['foot']} | {row['solved_ticks']}/{row['window_ticks']} | {row['maximum_dynamics_residual']:.2e} / {row['maximum_contact_residual']:.2e} | {row['minimum_support_margin_m'] * 1000:.2f} mm | {row['maximum_actuator_utilization'] * 100:.2f}% | {row['maximum_latency_us'] / 1000:.3f} ms | {row['clipped_ticks']} |"
            for row in transitions["transitions"]
        ],
        "",
        "### Per-step execution",
        "",
        "Each row begins after the preceding touchdown (or tick zero) and ends at the current touchdown, so transfer and swing work are both charged to the step that consumes them.",
        "",
        "| step | foot | ticks | solved / slack | clipped | root angular / linear / height | CoM / swing point | hard dynamics / contact | CoP | actuator | p50 / p99 / max |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        *[
            f"| {row['step']} | {row['foot']} | {row['ticks']} | {row['solved']} / {row['solved_with_slack']} | {row['clipped_ticks']} | {row['maximum_task_rms']['root_angular']:.3f} / {row['maximum_task_rms']['root_horizontal']:.3f} / {row['maximum_task_rms']['root_height']:.3f} | {row['maximum_task_rms']['center_of_mass']:.3f} / {max(row['maximum_task_rms']['point_0'], row['maximum_task_rms']['point_1']):.3f} | {row['maximum_dynamics_residual']:.2e} / {row['maximum_contact_residual']:.2e} | {row['minimum_support_margin_m'] * 1000:.2f} mm | {row['maximum_actuator_utilization'] * 100:.1f}% | {row['latency_us']['p50'] / 1000:.3f} / {row['latency_us']['p99'] / 1000:.3f} / {row['latency_us']['maximum'] / 1000:.3f} ms |"
            for row in transitions["steps"]
        ],
        "",
        "## Measured authority stack",
        "",
        "This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.",
        "",
        "| authority signal | observed envelope | provisional warning / critical | longest warning run |",
        "|---|---:|---:|---:|",
        f"| Invariant · maximum hard-row residual | p99 {hard['maximum_residual']['p99']:.2e} · max {hard['maximum_residual']['maximum']:.2e} | {hard['warning_threshold']:.0e} / {hard['critical_threshold']:.0e} | {hard['longest_warning_run_ticks']} ticks |",
        f"| Viability · finite-support margin | p01 {support['margin_m']['p01'] * 1000:.2f} mm · min {support['margin_m']['minimum'] * 1000:.2f} mm | {support['warning_threshold_m'] * 1000:.1f} / {support['critical_threshold_m'] * 1000:.1f} mm | {support['longest_warning_run_ticks']} ticks |",
        f"| Physical · nearest joint-position limit | p01 {joint['minimum_margin_deg']['p01']:.2f}° · min {joint['minimum_margin_deg']['minimum']:.2f}° | {joint['warning_threshold_deg']:.1f} / {joint['critical_threshold_deg']:.1f}° | {joint['longest_warning_run_ticks']} ticks |",
        f"| Physical · peak actuator effort | p99 {effort['maximum_utilization']['p99'] * 100:.2f}% · max {effort['maximum_utilization']['maximum'] * 100:.2f}% | {effort['warning_threshold'] * 100:.0f}% / {effort['critical_threshold'] * 100:.0f}% | {effort['longest_warning_run_ticks']} ticks |",
        f"| Compute · Rust WBC wall time | p99 {solver['latency_us']['p99']:.1f} µs · max {solver['latency_us']['maximum']:.1f} µs | {solver['warning_threshold_us']:.0f} / {solver['critical_threshold_us']:.0f} µs | {solver['longest_warning_run_ticks']} tick |",
        "| Persistent thermal / reliability | UNMODELED | no calibrated plant contract | N/A |",
        "",
        "### Capability curves",
        "",
        f"Each row asks how many of the {ticks} independent oracle ticks would be adverse if that absolute threshold were chosen. These sweeps expose sensitivity without tuning a threshold to this corpus.",
        "",
        "| hard residual ceiling | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold']:.0e} | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in hard["threshold_curve"]
        ],
        "",
        "| minimum support margin | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold'] * 1000:.1f} mm | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in support["threshold_curve"]
        ],
        "",
        "| minimum joint headroom | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold']:.1f}° | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in joint["threshold_curve_deg"]
        ],
        "",
        "| maximum actuator utilization | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold'] * 100:.0f}% | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in effort["threshold_curve"]
        ],
        "",
        "| WBC wall-time ceiling | adverse ticks | fraction |",
        "|---:|---:|---:|",
        *[
            f"| {row['threshold'] / 1000:.1f} ms | {row['adverse_ticks']} | {row['adverse_fraction'] * 100:.2f}% |"
            for row in solver["threshold_curve_us"]
        ],
        "",
        "### Actuator and nullspace provenance",
        "",
        effort_bound_report + " The most highly utilized actuators are:",
        "",
        "| actuator | authored limit | maximum effort | p99 use | peak use |",
        "|---|---:|---:|---:|---:|",
        *[
            f"| `{row['name']}` | {row['effort_limit_nm']:.1f} Nm | {row['maximum_absolute_torque_nm']:.2f} Nm | {row['p99_utilization'] * 100:.2f}% | {row['maximum_utilization'] * 100:.2f}% |"
            for row in effort["per_actuator"][:6]
        ],
        "",
        "The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.",
        "",
        "| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |",
        "|---|---:|---:|---:|---:|",
        *[
            f"| {name.title()} | {row['ticks_with_clipping']} | {row['task_pseudoinverse_calls']['mean']:.2f} / {row['task_pseudoinverse_calls']['maximum']:.0f} | {row['task_jacobi_sweeps']['mean']:.2f} / {row['task_jacobi_sweeps']['maximum']:.0f} | {row['clipped_steps']['mean']:.2f} / {row['clipped_steps']['maximum']:.0f} |"
            for name, row in solver["by_priority"].items()
        ],
        "",
        "Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.",
        "",
        "### Tight physical bounds versus loose control",
        "",
        f"The retained loose-bound control is `{control['path']}`. It is available: `{control['available']}`. All {len(control.get('compared_fields', []))} compared physical/status arrays are bit-for-bit identical: `{control.get('all_compared_fields_exact', False)}`. {control_interpretation}",
        "",
        "## Predeclared gates",
        "",
        *[f"- {'PASS' if passed else 'FAIL'} `{name}`" for name, passed in checks.items()],
        "",
        "## Interpretation",
        "",
        "A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red hard-admission gate means the solver cannot preserve dynamics/contact/actuation invariants. A tracking-red result with hard admission green means the reference remains physically feasible only by continuously relaxing declared acceleration tasks; the solved-with-slack count, per-task residuals, and per-step rows quantify exactly where and how much. Neither result can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.",
        "",
        "The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.",
        "",
    ]
    (output / "ORACLE_WBC_ADMISSION.md").write_text("\n".join(report))
    print(json.dumps({"passed": metrics["passed"], "checks": checks, "output": str(output)}, indent=2))
    return 0 if metrics["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
