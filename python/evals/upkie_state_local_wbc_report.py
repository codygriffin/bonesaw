#!/usr/bin/env python3
"""Policy- and physics-free rolling-WBC admission for the Upkie morphology.

Every row is an independent state query.  There is no controller policy,
integrator, contact estimator, simulator, or state carried between rows.
Rust performs the preallocated floating WBC solve; Python owns the corpus,
Pinocchio oracle, statistics, fault probes, and report rendering.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import math
import pathlib
import platform
import resource
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pinocchio as pin

from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_dynamics_mirror_report import pin_configuration, pinocchio_products, tangent_map


STATUS_NAMES = {
    0: "Solved",
    1: "SolvedWithSlack",
    2: "PrimalInfeasible",
    3: "NumericalOrInvalid",
    4: "MaxIterations",
}
SOLVED = (0, 1)
WHEEL_RADIUS_M = 0.05
ROLLING_GAIN = 2.0
ROLLING_MAXIMUM_CORRECTION_MPS2 = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=256)
    parser.add_argument("--timing-repeats", type=int, default=8)
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--output", default="benchmarks/results/upkie-state-local-wbc-r123"
    )
    parser.add_argument("--web-report", default="web/UPKIE_STATE_LOCAL_WBC_R123.html")
    parser.add_argument(
        "--reference",
        default="benchmarks/results/reference-r89/upkie-controller-metrics.json",
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def status_counts(status: np.ndarray) -> dict[str, int]:
    return {
        STATUS_NAMES[int(value)]: int(np.sum(status == value))
        for value in np.unique(status)
    }


def rss_bytes() -> int:
    status = pathlib.Path("/proc/self/status")
    if status.exists():
        for line in status.read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024


def standing_posture(joint_names: list[str]) -> np.ndarray:
    q = np.zeros(len(joint_names), dtype=np.float64)
    for name, value in (
        ("left_hip", 0.4),
        ("left_knee", -0.625),
        ("right_hip", -0.4),
        ("right_knee", 0.625),
    ):
        q[joint_names.index(name)] = value
    return q


def urdf_effort_limits(model_path: pathlib.Path, joint_names: list[str]) -> np.ndarray:
    root = ET.parse(model_path).getroot()
    limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("effort") is not None:
            limits[joint.get("name", "")] = float(limit.get("effort", "nan"))
    return np.asarray([limits[name] for name in joint_names], dtype=np.float64)


def root_pose_matrix(root_positions: np.ndarray) -> np.ndarray:
    roots = np.zeros((len(root_positions), 12), dtype=np.float64)
    roots[:, [0, 4, 8]] = 1.0
    roots[:, 9:12] = root_positions
    return roots


def rolling_descriptors(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Derive bottom-point wheel coefficients independently with Pinocchio."""
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    data = model.createData()
    q = pin.neutral(model)
    joint_ids = [model.getJointId(name) for name in joint_names]
    q_indices = [model.joints[joint].idx_q for joint in joint_ids]
    for value, index in zip(standing_posture(joint_names), q_indices, strict=True):
        q[index] = value
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    transform = tangent_map(np.eye(3), [model.joints[j].idx_v for j in joint_ids], 6 + len(joint_names))
    coordinates = np.asarray(
        [joint_names.index("left_wheel"), joint_names.index("right_wheel")],
        dtype=np.int64,
    )
    coefficients = np.empty(2, dtype=np.float64)
    for slot, (frame_name, coordinate) in enumerate(zip(frame_names, coordinates, strict=True)):
        frame_id = model.getFrameId(frame_name, pin.FrameType.BODY)
        jacobian = np.asarray(
            pin.getFrameJacobian(
                model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            ),
            dtype=np.float64,
        ) @ transform
        axis_world = jacobian[3:6, 6 + coordinate]
        coefficients[slot] = np.cross(
            axis_world, np.asarray([0.0, 0.0, -WHEEL_RADIUS_M])
        )[0]
    if np.any(np.abs(coefficients) < 1e-9):
        raise RuntimeError("Upkie rolling coefficient is degenerate")
    return (
        np.full(2, 3, dtype=np.uint8),
        coordinates,
        coefficients,
    )


def make_corpus(
    samples: int,
    joint_names: list[str],
    rolling_coordinates: np.ndarray,
    rolling_coefficients: np.ndarray,
    *,
    slip: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    if samples < 16:
        raise ValueError("state-local corpus requires at least 16 samples")
    phase = np.linspace(0.0, 4.0 * math.pi, samples, endpoint=False)
    q = np.tile(standing_posture(joint_names), (samples, 1))
    q[:, joint_names.index("left_hip")] += 0.11 * np.sin(phase)
    q[:, joint_names.index("right_hip")] -= 0.11 * np.sin(phase)
    q[:, joint_names.index("left_knee")] += 0.16 * np.cos(0.5 * phase)
    q[:, joint_names.index("right_knee")] -= 0.16 * np.cos(0.5 * phase)
    q[:, rolling_coordinates[0]] = 0.7 * np.sin(0.37 * phase)
    q[:, rolling_coordinates[1]] = -0.7 * np.sin(0.37 * phase)

    root_positions = np.column_stack(
        (
            0.08 * np.sin(0.41 * phase),
            0.025 * np.sin(0.23 * phase),
            0.56 + 0.025 * np.cos(0.5 * phase),
        )
    )
    root_velocities = np.zeros((samples, 3), dtype=np.float64)
    root_velocities[:, 0] = 0.55 * np.sin(0.67 * phase)
    root_accelerations = np.zeros((samples, 3), dtype=np.float64)
    root_accelerations[:, 0] = 1.10 * np.cos(0.53 * phase)
    root_accelerations[:, 2] = 0.18 * np.sin(0.31 * phase)

    if slip is None:
        slip = np.zeros((samples, 2), dtype=np.float64)
    elif slip.shape == (samples,):
        slip = np.repeat(slip[:, None], 2, axis=1)
    if slip.shape != (samples, 2):
        raise ValueError("slip must have shape [samples] or [samples, 2]")
    v = np.zeros_like(q)
    joint_accelerations = np.zeros_like(q)
    for slot, (coordinate, coefficient) in enumerate(
        zip(rolling_coordinates, rolling_coefficients, strict=True)
    ):
        v[:, coordinate] = (slip[:, slot] - root_velocities[:, 0]) / coefficient
        joint_accelerations[:, coordinate] = -root_accelerations[:, 0] / coefficient
    # Independent pose states intentionally do not claim q-dot consistency.
    # The supplied velocity and acceleration witnesses are nevertheless exact
    # rolling-row witnesses before the explicit slip stabilization probe.
    return {
        "root_positions": np.ascontiguousarray(root_positions),
        "root_velocities": np.ascontiguousarray(root_velocities),
        "root_accelerations": np.ascontiguousarray(root_accelerations),
        "q": np.ascontiguousarray(q),
        "v": np.ascontiguousarray(v),
        "joint_accelerations": np.ascontiguousarray(joint_accelerations),
        "contact_active": np.ones((samples, 2), dtype=np.uint8),
        "slip": np.ascontiguousarray(slip),
    }


def allocate_outputs(session: Any, ticks: int) -> dict[str, np.ndarray]:
    dof = session.dof
    maximum_contacts = 2
    task_capacity = session.task_diagnostic_capacity
    return {
        "generalized_acceleration": np.empty((ticks, dof + 6), np.float64),
        "actuator_torque": np.empty((ticks, dof), np.float64),
        "contact_normal_force": np.empty((ticks, maximum_contacts), np.float64),
        "contact_force_basis": np.empty((ticks, maximum_contacts, 3), np.float64),
        "task_rms": np.empty((ticks, task_capacity), np.float64),
        "task_clipped": np.empty((ticks, task_capacity), np.uint8),
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


def new_session(model_path: pathlib.Path, *, friction: float = 0.8, torque: float = 2000.0):
    import bonesaw

    return bonesaw.FloatingWbcSession(
        str(model_path),
        maximum_contacts=2,
        friction_coefficient=friction,
        maximum_acceleration=250.0,
        maximum_torque=torque,
        maximum_normal_force_multiple=3.0,
        joint_limit_braking=False,
        root_angular_task_weight=10.0,
        root_height_task_weight=10.0,
        root_horizontal_task_weight=10.0,
        root_horizontal_task_priority=1,
        joint_posture_weight=1.0,
        joint_posture_priority=2,
        center_of_mass_task_weight=0.0,
    )


def run_case(
    session: Any,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    contact_modes: np.ndarray | None,
    rolling_coordinates: np.ndarray,
    rolling_coefficients: np.ndarray,
    outputs: dict[str, np.ndarray] | None = None,
    contact_mode_trace: np.ndarray | None = None,
    target_weights: np.ndarray | None = None,
    generalized_acceleration_limit_scales: np.ndarray | None = None,
    actuator_effort_limit_scales: np.ndarray | None = None,
    contact_bases_world: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    ticks = len(corpus["q"])
    out = allocate_outputs(session, ticks) if outputs is None else outputs
    session.run_oracle_trace(
        corpus["root_positions"],
        corpus["root_velocities"],
        corpus["root_accelerations"],
        corpus["q"],
        corpus["v"],
        corpus["joint_accelerations"],
        frame_ids,
        corpus["contact_active"],
        np.ones(2, dtype=np.uint8),
        np.ones(2, dtype=np.float64) if target_weights is None else target_weights,
        0,
        0,
        out["generalized_acceleration"],
        out["actuator_torque"],
        out["contact_normal_force"],
        out["task_rms"],
        out["task_clipped"],
        out["dynamics_residual"],
        out["contact_residual"],
        out["minimum_friction_margin"],
        out["minimum_support_margin"],
        out["minimum_torque_margin"],
        out["maximum_constraint_violation"],
        out["minimum_bound_margin"],
        out["minimum_joint_margin_rad"],
        out["minimum_joint_headroom_fraction"],
        out["limiting_joint"],
        out["maximum_torque_utilization"],
        out["minimum_torque_headroom"],
        out["limiting_actuator"],
        out["witness_acceleration_rms"],
        out["step_ns"],
        out["status"],
        out["task_pseudoinverse_calls"],
        out["task_pseudoinverse_calls_by_priority"],
        out["clipped_steps"],
        out["clipped_steps_by_priority"],
        out["task_jacobi_sweeps"],
        out["task_jacobi_sweeps_by_priority"],
        out["feasibility_projection_sweeps"],
        out["feasibility_halfspace_projections"],
        out["feasibility_polish_iterations"],
        out["allocation_calls"],
        out["allocated_bytes"],
        contact_force_basis_out=out["contact_force_basis"],
        contact_modes=None if contact_mode_trace is not None else contact_modes,
        rolling_coordinates=rolling_coordinates,
        rolling_velocity_coefficients=rolling_coefficients,
        rolling_velocity_stabilization_gains=np.full(2, ROLLING_GAIN, np.float64),
        rolling_maximum_stabilization_accelerations=np.full(
            2, ROLLING_MAXIMUM_CORRECTION_MPS2, np.float64
        ),
        contact_mode_trace=contact_mode_trace,
        generalized_acceleration_limit_scales=generalized_acceleration_limit_scales,
        actuator_effort_limit_scales=actuator_effort_limit_scales,
        contact_bases_world=contact_bases_world,
    )
    return out


def pin_contact_products(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
    corpus: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    data = model.createData()
    joint_ids = [model.getJointId(name) for name in joint_names]
    q_indices = [model.joints[joint].idx_q for joint in joint_ids]
    v_indices = [model.joints[joint].idx_v for joint in joint_ids]
    frame_ids = [model.getFrameId(name, pin.FrameType.BODY) for name in frame_names]
    generalized = 6 + len(joint_names)
    roots = root_pose_matrix(corpus["root_positions"])
    velocity = np.column_stack(
        (
            np.zeros((len(corpus["q"]), 3)),
            corpus["root_velocities"],
            corpus["v"],
        )
    )
    jacobians = np.empty((len(velocity), 2, 3, generalized), np.float64)
    biases = np.empty((len(velocity), 2, 3), np.float64)
    for tick in range(len(velocity)):
        configuration = pin_configuration(model, q_indices, corpus["q"][tick], roots[tick])
        transform = tangent_map(np.eye(3), v_indices, generalized)
        pin_velocity = transform @ velocity[tick]
        pin_acceleration = np.zeros(generalized, dtype=np.float64)
        pin.forwardKinematics(model, data, configuration, pin_velocity, pin_acceleration)
        pin.computeJointJacobians(model, data, configuration)
        pin.updateFramePlacements(model, data)
        for slot, frame_id in enumerate(frame_ids):
            frame_jacobian = np.asarray(
                pin.getFrameJacobian(
                    model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                ),
                dtype=np.float64,
            )
            jacobians[tick, slot] = frame_jacobian[:3] @ transform
            biases[tick, slot] = np.asarray(
                pin.getFrameClassicalAcceleration(
                    model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                ).linear,
                dtype=np.float64,
            )
    return jacobians, biases


def independent_oracle(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
    corpus: dict[str, np.ndarray],
    out: dict[str, np.ndarray],
    rolling_coordinates: np.ndarray,
    rolling_coefficients: np.ndarray,
    friction: float,
    effort_limits: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    roots = root_pose_matrix(corpus["root_positions"])
    velocity = np.column_stack(
        (np.zeros((len(corpus["q"]), 3)), corpus["root_velocities"], corpus["v"])
    )
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81]), (len(velocity), 1))
    mass, bias, _ = pinocchio_products(
        model_path, joint_names, corpus["q"], roots, velocity, gravity
    )
    jacobians, contact_bias = pin_contact_products(
        model_path, joint_names, frame_names, corpus
    )
    dynamics = np.empty((len(velocity), len(joint_names) + 6), np.float64)
    contact = np.empty((len(velocity), 2, 3), np.float64)
    rolling_velocity = np.empty((len(velocity), 2), np.float64)
    rolling_correction = np.empty((len(velocity), 2), np.float64)
    generalized_acceleration = out["generalized_acceleration"]
    for tick in range(len(velocity)):
        rhs = np.zeros(len(joint_names) + 6, dtype=np.float64)
        rhs[6:] = out["actuator_torque"][tick]
        for slot, (coordinate, coefficient) in enumerate(
            zip(rolling_coordinates, rolling_coefficients, strict=True)
        ):
            force = out["contact_force_basis"][tick, slot]
            rhs += jacobians[tick, slot].T @ force
            # RollingWheel uses the same augmented tangent Jacobian for
            # virtual work and acceleration: J_center_x + c * qdot_wheel.
            # Omitting c * force_x here would independently reproduce the
            # pre-r127 force-map defect instead of checking it.
            rhs[6 + coordinate] += coefficient * force[0]
            point_velocity = jacobians[tick, slot] @ velocity[tick]
            rolling_velocity[tick, slot] = (
                point_velocity[0] + coefficient * corpus["v"][tick, coordinate]
            )
            rolling_correction[tick, slot] = np.clip(
                -ROLLING_GAIN * rolling_velocity[tick, slot],
                -ROLLING_MAXIMUM_CORRECTION_MPS2,
                ROLLING_MAXIMUM_CORRECTION_MPS2,
            )
            physical = (
                jacobians[tick, slot] @ generalized_acceleration[tick]
                + contact_bias[tick, slot]
            )
            contact[tick, slot] = physical
            contact[tick, slot, 0] += (
                coefficient * generalized_acceleration[tick, 6 + coordinate]
                - rolling_correction[tick, slot]
            )
        dynamics[tick] = mass[tick] @ generalized_acceleration[tick] + bias[tick] - rhs

    forces = out["contact_force_basis"]
    square_cone_margin = friction * forces[:, :, 2] - np.maximum(
        np.abs(forces[:, :, 0]), np.abs(forces[:, :, 1])
    )
    effort_margin = effort_limits[None, :] - np.abs(out["actuator_torque"])
    metrics = {
        "dynamics_linf": float(np.max(np.abs(dynamics))),
        "rolling_contact_linf": float(np.max(np.abs(contact))),
        "minimum_normal_force_n": float(np.min(forces[:, :, 2])),
        "minimum_square_friction_margin_n": float(np.min(square_cone_margin)),
        "minimum_effort_margin_nm": float(np.min(effort_margin)),
        "rolling_velocity_linf_mps": float(np.max(np.abs(rolling_velocity))),
        "reported_dynamics_linf": float(np.max(out["dynamics_residual"])),
        "reported_contact_linf": float(np.max(out["contact_residual"])),
    }
    return metrics, {
        "pinocchio_dynamics_residual": dynamics,
        "pinocchio_contact_residual": contact,
        "pinocchio_rolling_velocity": rolling_velocity,
        "pinocchio_rolling_correction": rolling_correction,
    }


def timing_metrics(step_ns: np.ndarray) -> dict[str, Any]:
    latency_us = step_ns.astype(np.float64) / 1000.0
    jitter_us = np.abs(np.diff(latency_us))
    return {
        "latency_us": distribution(latency_us),
        "absolute_inter_query_jitter_us": distribution(jitter_us),
        "p99_minus_p50_us": float(np.percentile(latency_us, 99) - np.percentile(latency_us, 50)),
        "deadline_misses": {
            f"{limit_us / 1000:g}ms": int(np.sum(latency_us > limit_us))
            for limit_us in (500, 1000, 2000, 5000, 10000)
        },
    }


def task_stack_metrics(out: dict[str, np.ndarray]) -> list[dict[str, Any]]:
    names = (
        "root_angular",
        "root_horizontal",
        "root_height",
        "joint_posture",
        "joint_acceleration_override",
        "center_of_mass",
        "centroidal_angular_momentum",
        "contact_force_regularization",
        "actuator_torque_regularization",
        "swing_angular",
        "point_0",
        "point_1",
        "point_2",
        "point_3",
    )
    active_slots = {0, 1, 2, 3, 7, 8}
    rows = []
    for slot, name in enumerate(names):
        if slot not in active_slots:
            continue
        values = out["task_rms"][:, slot]
        rows.append(
            {
                "slot": slot,
                "task": name,
                "rms_distribution": distribution(values),
                "clipped_queries": int(np.sum(out["task_clipped"][:, slot])),
            }
        )
    return rows


def solver_work_metrics(out: dict[str, np.ndarray]) -> dict[str, Any]:
    step = out["step_ns"].astype(np.float64)
    fields = (
        "task_pseudoinverse_calls",
        "clipped_steps",
        "task_jacobi_sweeps",
        "feasibility_projection_sweeps",
        "feasibility_halfspace_projections",
        "feasibility_polish_iterations",
    )
    result: dict[str, Any] = {}
    for name in fields:
        values = out[name].astype(np.float64)
        correlation = 0.0
        if np.std(values) > 0.0 and np.std(step) > 0.0:
            correlation = float(np.corrcoef(values, step)[0, 1])
        result[name] = {
            "distribution": distribution(values),
            "latency_correlation": correlation,
        }
    return result


def bound_metrics(out: dict[str, np.ndarray]) -> dict[str, Any]:
    maximum_acceleration = 250.0
    qdd = out["generalized_acceleration"]
    return {
        "configured_generalized_acceleration_limit": maximum_acceleration,
        "maximum_absolute_generalized_acceleration": float(np.max(np.abs(qdd))),
        "maximum_acceleration_utilization": float(np.max(np.abs(qdd)) / maximum_acceleration),
        "minimum_solver_bound_margin": float(np.min(out["minimum_bound_margin"])),
        "queries_at_acceleration_bound": int(
            np.sum(np.any(np.abs(qdd) >= maximum_acceleration - 1e-7, axis=1))
        ),
    }


def tracking_metrics(corpus: dict[str, np.ndarray], out: dict[str, np.ndarray]) -> dict[str, Any]:
    desired = np.column_stack(
        (
            np.zeros((len(corpus["q"]), 3)),
            corpus["root_accelerations"],
            corpus["joint_accelerations"],
        )
    )
    error = out["generalized_acceleration"] - desired
    groups = {
        "all_generalized_mps2_or_radps2": error,
        "root_linear_mps2": error[:, 3:6],
        "joint_radps2": error[:, 6:],
        "wheel_radps2": error[:, [8, 11]],
    }
    metrics: dict[str, Any] = {}
    for name, values in groups.items():
        per_tick = np.sqrt(np.mean(values * values, axis=1))
        metrics[name] = {
            "rms": float(np.sqrt(np.mean(values * values))),
            "absolute": distribution(np.abs(values).reshape(-1)),
            "per_tick_rms": distribution(per_tick),
        }
    return metrics


def semantic_equal(left: dict[str, np.ndarray], right: dict[str, np.ndarray]) -> bool:
    ignored = {"step_ns"}
    return all(
        np.array_equal(left[name], right[name], equal_nan=True)
        for name in left
        if name not in ignored
    )


def invariance_metrics(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    modes: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
    baseline: dict[str, np.ndarray],
) -> dict[str, Any]:
    repeat = run_case(
        new_session(model_path), corpus, frame_ids, modes, coordinates, coefficients
    )
    permutation = np.arange(len(corpus["q"]) - 1, -1, -1)
    permuted_corpus = {
        name: value[permutation] if value.ndim > 0 and len(value) == len(permutation) else value
        for name, value in corpus.items()
    }
    permuted = run_case(
        new_session(model_path),
        permuted_corpus,
        frame_ids,
        modes,
        coordinates,
        coefficients,
    )
    inverse = np.argsort(permutation)
    permuted_ordered = {
        name: value[inverse] if value.ndim > 0 and len(value) == len(inverse) else value
        for name, value in permuted.items()
    }
    midpoint = len(corpus["q"]) // 2
    parts: list[dict[str, np.ndarray]] = []
    for start, stop in ((0, midpoint), (midpoint, len(corpus["q"]))):
        part = {
            name: value[start:stop] if value.ndim > 0 and len(value) == len(corpus["q"]) else value
            for name, value in corpus.items()
        }
        parts.append(
            run_case(new_session(model_path), part, frame_ids, modes, coordinates, coefficients)
        )
    chunked = {name: np.concatenate([part[name] for part in parts]) for name in baseline}
    return {
        "repeat_bitwise_exact": semantic_equal(baseline, repeat),
        "reverse_permutation_exact": semantic_equal(baseline, permuted_ordered),
        "two_chunk_exact": semantic_equal(baseline, chunked),
        "repeat_allocation_calls": int(np.sum(repeat["allocation_calls"])),
        "permutation_allocation_calls": int(np.sum(permuted["allocation_calls"])),
        "chunk_allocation_calls": int(sum(np.sum(part["allocation_calls"]) for part in parts)),
    }


def resource_sweep(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    modes: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> dict[str, Any]:
    rows = []
    for friction, torque in (
        (0.8, 2000.0),
        (0.4, 2000.0),
        (0.2, 2000.0),
        (0.05, 2000.0),
        (0.8, 8.0),
        (0.8, 4.0),
        (0.8, 1.5),
        (0.8, 0.5),
    ):
        out = run_case(
            new_session(model_path, friction=friction, torque=torque),
            corpus,
            frame_ids,
            modes,
            coordinates,
            coefficients,
        )
        rows.append(
            {
                "friction": friction,
                "torque_cap_nm": torque,
                "status_counts": status_counts(out["status"]),
                "maximum_witness_acceleration_rms": float(
                    np.max(out["witness_acceleration_rms"])
                ),
                "minimum_friction_margin_n": float(np.min(out["minimum_friction_margin"])),
                "maximum_torque_utilization": float(np.max(out["maximum_torque_utilization"])),
                "maximum_constraint_violation": float(
                    np.max(out["maximum_constraint_violation"])
                ),
                "finite": bool(
                    all(np.all(np.isfinite(value)) for value in (
                        out["generalized_acceleration"], out["actuator_torque"], out["contact_force_basis"]
                    ))
                ),
            }
        )
    return {"rows": rows, "all_finite": all(row["finite"] for row in rows)}


def slip_and_fault_metrics(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
    frame_ids: np.ndarray,
    modes: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    samples = 128
    slip = np.linspace(-0.8, 0.8, samples)
    corpus = make_corpus(samples, joint_names, coordinates, coefficients, slip=slip)
    out = run_case(
        new_session(model_path), corpus, frame_ids, modes, coordinates, coefficients
    )
    effort = urdf_effort_limits(model_path, joint_names)
    oracle, raw = independent_oracle(
        model_path,
        joint_names,
        frame_names,
        corpus,
        out,
        coordinates,
        coefficients,
        0.8,
        effort,
    )
    invalid_zero_coefficient = False
    try:
        run_case(
            new_session(model_path),
            corpus,
            frame_ids,
            modes,
            coordinates,
            np.zeros(2, dtype=np.float64),
        )
    except ValueError:
        invalid_zero_coefficient = True
    metrics = {
        "slip_range_mps": [float(np.min(slip)), float(np.max(slip))],
        "maximum_expected_stabilization_mps2": ROLLING_MAXIMUM_CORRECTION_MPS2,
        "pinocchio_stabilized_contact_linf": oracle["rolling_contact_linf"],
        "status_counts": status_counts(out["status"]),
        "all_outputs_finite": bool(
            np.all(np.isfinite(out["generalized_acceleration"]))
            and np.all(np.isfinite(out["actuator_torque"]))
            and np.all(np.isfinite(out["contact_force_basis"]))
        ),
        "zero_rolling_coefficient_rejected": invalid_zero_coefficient,
        "allocation_calls": int(np.sum(out["allocation_calls"])),
    }
    return metrics, {
        "slip": corpus["slip"],
        "slip_qdd": out["generalized_acceleration"],
        **raw,
    }


def execution_windows(
    corpus: dict[str, np.ndarray], out: dict[str, np.ndarray], raw: dict[str, np.ndarray]
) -> list[dict[str, Any]]:
    windows = []
    boundaries = np.linspace(0, len(corpus["q"]), 9, dtype=int)
    for start, stop in zip(boundaries[:-1], boundaries[1:], strict=True):
        step_us = out["step_ns"][start:stop].astype(np.float64) / 1000.0
        windows.append(
            {
                "start": int(start),
                "stop": int(stop),
                "p50_us": float(np.percentile(step_us, 50)),
                "p99_us": float(np.percentile(step_us, 99)),
                "max_us": float(np.max(step_us)),
                "tracking_rms": float(
                    np.sqrt(np.mean(out["witness_acceleration_rms"][start:stop] ** 2))
                ),
                "dynamics_linf": float(
                    np.max(np.abs(raw["pinocchio_dynamics_residual"][start:stop]))
                ),
                "contact_linf": float(
                    np.max(np.abs(raw["pinocchio_contact_residual"][start:stop]))
                ),
                "torque_utilization": float(
                    np.max(out["maximum_torque_utilization"][start:stop])
                ),
                "slack": int(np.sum(out["status"][start:stop] == 1)),
            }
        )
    return windows


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle = metrics["independent_pinocchio_oracle"]
    timing = metrics["timing"]
    latency = timing["latency_us"]
    jitter = timing["absolute_inter_query_jitter_us"]
    tracking = metrics["tracking"]
    bounds = metrics["bounds"]
    reference = metrics["retained_official_upkie_reference"]
    authority = [
        ["Invariant", "floating dynamics", "Pinocchio M qdd + h − Sᵀτ − Jᵀf L∞"],
        ["Invariant", "two RollingWheel contacts", "longitudinal wheel coupling + lateral/normal acceleration L∞"],
        ["Resource", "unilateral + square Coulomb cone", "normal force and friction margin per wheel"],
        ["Resource", "URDF actuator effort", "per-actuator effort margin and limiting actuator"],
        ["Viability", "root attitude + height", "priority-0 task residual; hard contacts remain above it"],
        ["Intent", "root horizontal", "priority-1 residual"],
        ["Posture", "joint/wheel acceleration witness", "priority-2 residual and witness RMS"],
        ["Recovery", "bounded solve status", "SolvedWithSlack / MaxIterations / infeasible stay typed"],
        ["Budget", "CPU exact f64", "per-query latency, work counters, allocations, deadline misses"],
    ]
    resource_rows = [
        [
            row["friction"],
            row["torque_cap_nm"],
            row["status_counts"],
            f'{row["maximum_witness_acceleration_rms"]:.4f}',
            f'{row["minimum_friction_margin_n"]:.3e}',
            f'{100 * row["maximum_torque_utilization"]:.1f}%',
            f'{row["maximum_constraint_violation"]:.2e}',
        ]
        for row in metrics["resource_sweep"]["rows"]
    ]
    tracking_rows = [
        [
            name,
            f'{row["rms"]:.6f}',
            f'{row["absolute"]["p50"]:.6f}',
            f'{row["absolute"]["p95"]:.6f}',
            f'{row["absolute"]["p99"]:.6f}',
            f'{row["absolute"]["maximum"]:.6f}',
        ]
        for name, row in tracking.items()
    ]
    task_rows = [
        [
            row["slot"],
            row["task"],
            row["clipped_queries"],
            f'{row["rms_distribution"]["mean"]:.5f}',
            f'{row["rms_distribution"]["p95"]:.5f}',
            f'{row["rms_distribution"]["p99"]:.5f}',
            f'{row["rms_distribution"]["maximum"]:.5f}',
        ]
        for row in metrics["task_stack"]
    ]
    work_rows = [
        [
            name,
            f'{row["distribution"]["mean"]:.2f}',
            f'{row["distribution"]["p95"]:.2f}',
            f'{row["distribution"]["p99"]:.2f}',
            f'{row["distribution"]["maximum"]:.2f}',
            f'{row["latency_correlation"]:.4f}',
        ]
        for name, row in metrics["solver_work"].items()
    ]
    window_rows = [
        [
            f'{row["start"]}–{row["stop"] - 1}',
            f'{row["p50_us"]:.1f} / {row["p99_us"]:.1f} / {row["max_us"]:.1f}',
            f'{row["tracking_rms"]:.5f}',
            f'{row["dynamics_linf"]:.2e}',
            f'{row["contact_linf"]:.2e}',
            f'{100 * row["torque_utilization"]:.1f}%',
            row["slack"],
        ]
        for row in metrics["execution_order_windows"]
    ]
    return "\n".join(
        [
            "# Bonesaw Upkie state-local rolling WBC · r123",
            "",
            "## Outcome",
            "",
            f'> **Admission: {"PASS" if metrics["admission"] else "FAIL"}.** '
            "This is the missing composed Upkie WBC test: two nonholonomic wheel contacts, "
            "floating rigid-body dynamics, contact forces, friction, and actuator effort are "
            "solved together in Rust and rebuilt independently in Pinocchio. Every corpus row "
            "is an immutable state query—policy = null, physics rollout = null.",
            "",
            "## Corpus and semantic boundary",
            "",
            *markdown_table(
                ["item", "value"],
                [
                    ["states", metrics["samples"]],
                    ["model", metrics["model"]],
                    ["model SHA-256", metrics["model_sha256"]],
                    ["coordinates", metrics["joint_names"]],
                    ["contact frames", metrics["contact_frames"]],
                    ["rolling coefficients", metrics["rolling_velocity_coefficients"]],
                    ["policy", "none"],
                    ["physics / integration", "none"],
                    ["state carried between rows", "none"],
                    ["decision vector", "[floating qdd, actuator torque, two 3D contact forces]"],
                ],
            ),
            "",
            "The posture, root velocity, and requested acceleration vary analytically across "
            "the corpus, but rows are not interpreted as a trajectory. Wheel rates are authored "
            "to satisfy v_center,x + c·qdot_wheel = 0 exactly. This isolates WBC admission from "
            "state estimation, balance policy, motor policy, and simulation.",
            "",
            "## Example authority stack",
            "",
            *markdown_table(["layer", "authority", "separate witness"], authority),
            "",
            "## Independent Pinocchio hard-equation oracle",
            "",
            *markdown_table(
                ["quantity", "worst observed", "gate"],
                [
                    ["floating dynamics L∞", f'{oracle["dynamics_linf"]:.3e}', "≤ 2e-6"],
                    ["rolling/lateral/normal contact L∞", f'{oracle["rolling_contact_linf"]:.3e}', "≤ 2e-6"],
                    ["minimum normal force", f'{oracle["minimum_normal_force_n"]:.6f} N', "≥ -1e-8"],
                    ["minimum square friction margin", f'{oracle["minimum_square_friction_margin_n"]:.6e} N', "≥ -2e-7"],
                    ["minimum effort margin", f'{oracle["minimum_effort_margin_nm"]:.6e} Nm', "≥ -2e-7"],
                    ["Rust-reported dynamics L∞", f'{oracle["reported_dynamics_linf"]:.3e}', "diagnostic"],
                    ["Rust-reported contact L∞", f'{oracle["reported_contact_linf"]:.3e}', "diagnostic"],
                ],
            ),
            "",
            "Pinocchio rebuilds mass, nonlinear effects, wheel-center Jacobians, and classical "
            "J-dot-v acceleration from the URDF. The gate uses returned Rust qdd, torque, and "
            "force—not Rust's residual fields—as its witness.",
            "",
            "## Tracking error",
            "",
            *markdown_table(["group", "RMS", "abs p50", "abs p95", "abs p99", "abs max"], tracking_rows),
            "",
            "Tracking error is not a feasibility failure. It is the continuous distance between "
            "the requested acceleration witness and the best command remaining after hard "
            "dynamics, rolling contact, friction, effort, and higher authority are honored.",
            "",
            "### Task and nullspace residual stack",
            "",
            *markdown_table(["slot", "task", "clipped", "mean RMS", "p95", "p99", "max"], task_rows),
            "",
            "The task stack stays expanded rather than collapsed into one score. This table "
            "shows the six active fixed slots; inactive override, CoM, momentum, swing, and "
            "point slots are omitted. Root and posture layers show exactly where the requested "
            "acceleration was surrendered.",
            "",
            "### Acceleration-bound authority",
            "",
            *markdown_table(
                ["measurement", "result"],
                [
                    ["configured absolute qdd cap", f'{bounds["configured_generalized_acceleration_limit"]:.1f}'],
                    ["maximum absolute qdd", f'{bounds["maximum_absolute_generalized_acceleration"]:.6f}'],
                    ["maximum utilization", f'{100 * bounds["maximum_acceleration_utilization"]:.2f}%'],
                    ["minimum solver bound margin", f'{bounds["minimum_solver_bound_margin"]:.3e}'],
                    ["queries touching qdd cap", f'{bounds["queries_at_acceleration_bound"]} / {metrics["samples"]}'],
                ],
            ),
            "",
            "A query can satisfy dynamics and contact exactly while landing on the configured "
            "acceleration cap. That is explicitly reported as exhausted numerical/kinematic "
            "authority, not mislabeled as successful pose tracking.",
            "",
            "## CPU latency, jitter, memory, and allocations",
            "",
            *markdown_table(
                ["measurement", "result"],
                [
                    ["latency mean / std / MAD", f'{latency["mean"]:.1f} / {latency["stddev"]:.1f} / {latency["mad"]:.1f} µs'],
                    ["latency p50 / p95 / p99 / p99.9 / max", f'{latency["p50"]:.1f} / {latency["p95"]:.1f} / {latency["p99"]:.1f} / {latency["p99_9"]:.1f} / {latency["maximum"]:.1f} µs'],
                    ["absolute adjacent-query jitter p50 / p95 / p99 / max", f'{jitter["p50"]:.1f} / {jitter["p95"]:.1f} / {jitter["p99"]:.1f} / {jitter["maximum"]:.1f} µs'],
                    ["p99 − p50", f'{timing["p99_minus_p50_us"]:.1f} µs'],
                    ["deadline misses", timing["deadline_misses"]],
                    ["whole-call wall / CPU", f'{metrics["runtime"]["wall_ms"]:.3f} / {metrics["runtime"]["cpu_ms"]:.3f} ms'],
                    ["CPU / wall", f'{metrics["runtime"]["cpu_to_wall"]:.4f}'],
                    ["measured throughput", f'{metrics["runtime"]["queries_per_second"]:.1f} queries/s'],
                    ["current RSS before / after / delta", f'{metrics["runtime"]["rss_before_bytes"] / 1048576:.2f} / {metrics["runtime"]["rss_after_bytes"] / 1048576:.2f} / {metrics["runtime"]["rss_delta_bytes"] / 1048576:.2f} MiB'],
                    ["process peak RSS before / after", f'{metrics["runtime"]["peak_rss_before_bytes"] / 1048576:.2f} / {metrics["runtime"]["peak_rss_after_bytes"] / 1048576:.2f} MiB'],
                    ["hot-loop allocation sentinel", f'{metrics["runtime"]["allocation_calls"]} calls / {metrics["runtime"]["allocated_bytes"]} bytes'],
                    ["GC collections during measured call", metrics["runtime"]["gc_collection_delta"]],
                ],
            ),
            "",
            "## Execution order windows",
            "",
            *markdown_table(["rows", "p50 / p99 / max µs", "tracking RMS", "dyn L∞", "contact L∞", "effort", "slack"], window_rows),
            "",
            "These windows expose drift and outliers over call order; their x-axis is corpus row "
            "order, not simulated time.",
            "",
            "### Solver work attribution",
            "",
            *markdown_table(["counter", "mean", "p95", "p99", "max", "latency corr."], work_rows),
            "",
            "## Continuous authority sweeps",
            "",
            *markdown_table(["friction", "torque cap", "statuses", "tracking max RMS", "friction margin", "effort use", "hard violation"], resource_rows),
            "",
            "Lower friction and actuator caps do not change the meaning of the query. They show "
            "how task residual rises—or a bounded solve becomes typed—while hard physical "
            "witnesses remain separately inspectable.",
            "",
            "## Rolling slip stabilization and fault typing",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["slip_and_faults"].items()]),
            "",
            "## Determinism and isolation from execution layout",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]),
            "",
            "## Retained official Upkie reference comparison",
            "",
            *markdown_table(
                ["item", "official Upkie C++", "Bonesaw Rust official profile"],
                [
                    ["shared controller-law samples", reference["ticks"], reference["ticks"]],
                    ["wheel command mismatches", "reference", reference["command_mismatches"]],
                    ["maximum command error", "reference", f'{reference["maximum_command_error"]:.3e} rad/s'],
                    ["p50 latency", f'{reference["upkie_latency"]["p50_us"]:.3f} µs', f'{reference["bonesaw_latency"]["p50_us"]:.3f} µs'],
                    ["p99 latency", f'{reference["upkie_latency"]["p99_us"]:.3f} µs', f'{reference["bonesaw_latency"]["p99_us"]:.3f} µs'],
                    ["peak RSS", f'{reference["upkie_peak_rss_bytes"] / 1048576:.2f} MiB', f'{reference["bonesaw_peak_rss_bytes"] / 1048576:.2f} MiB'],
                    ["hot-loop allocations", "not instrumented upstream", f'{reference["bonesaw_allocations"]} calls'],
                ],
            ),
            "",
            "The retained comparison pins upstream WheelBalancer.cpp and proves exact command-law "
            "parity over 100,000 sequential samples. It is deliberately not presented as a WBC "
            "speed comparison: Upkie's measured boundary is dictionary I/O plus a scalar balance "
            "law; this report measures the full floating inverse-dynamics hierarchy.",
            "",
            "## Deliberate remaining boundaries",
            "",
            "This admission does not prove closed-loop stability, estimator robustness, motor "
            "tracking, thermal/power reliability, terrain transitions, or contact switching. "
            "Those require separate evidence. It does prove that the CPU implementation composes "
            "the Upkie morphology's rolling constraints with floating dynamics and bounded "
            "resources without hiding tracking loss inside a binary pose claim. CUDA batching "
            "remains deferred until this CPU semantic reference is stable.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    web_report = pathlib.Path(args.web_report)
    reference_path = pathlib.Path(args.reference)
    if args.samples < 16 or args.timing_repeats < 1:
        raise ValueError("samples must be >= 16 and timing repeats positive")

    session = new_session(model_path)
    joint_names = list(session.joint_names)
    frame_names_all = list(session.frame_names)
    contact_frames = ["left_wheel_center", "right_wheel_center"]
    frame_ids = np.asarray([frame_names_all.index(name) for name in contact_frames], np.int64)
    modes, coordinates, coefficients = rolling_descriptors(
        model_path, joint_names, contact_frames
    )
    corpus = make_corpus(args.samples, joint_names, coordinates, coefficients)
    buffers = allocate_outputs(session, args.samples)
    run_case(session, corpus, frame_ids, modes, coordinates, coefficients, buffers)

    gc.collect()
    gc_before = [item["collections"] for item in gc.get_stats()]
    rss_before = rss_bytes()
    peak_rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    wall_before = time.perf_counter_ns()
    cpu_before = time.process_time_ns()
    timing_samples = np.empty(args.timing_repeats * args.samples, dtype=np.uint64)
    measured_allocation_calls = 0
    measured_allocated_bytes = 0
    for repeat in range(args.timing_repeats):
        run_case(session, corpus, frame_ids, modes, coordinates, coefficients, buffers)
        start = repeat * args.samples
        timing_samples[start : start + args.samples] = buffers["step_ns"]
        measured_allocation_calls += int(np.sum(buffers["allocation_calls"]))
        measured_allocated_bytes += int(np.sum(buffers["allocated_bytes"]))
    cpu_after = time.process_time_ns()
    wall_after = time.perf_counter_ns()
    rss_after = rss_bytes()
    peak_rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    gc_after = [item["collections"] for item in gc.get_stats()]
    baseline = {name: value.copy() for name, value in buffers.items()}

    effort_limits = urdf_effort_limits(model_path, joint_names)
    oracle, oracle_raw = independent_oracle(
        model_path,
        joint_names,
        contact_frames,
        corpus,
        baseline,
        coordinates,
        coefficients,
        0.8,
        effort_limits,
    )
    invariance = invariance_metrics(
        model_path, corpus, frame_ids, modes, coordinates, coefficients, baseline
    )
    resources = resource_sweep(
        model_path, corpus, frame_ids, modes, coordinates, coefficients
    )
    slip_faults, slip_raw = slip_and_fault_metrics(
        model_path,
        joint_names,
        contact_frames,
        frame_ids,
        modes,
        coordinates,
        coefficients,
    )
    reference_source = json.loads(reference_path.read_text())
    official = reference_source["comparisons"]["official_aligned"]
    reference = {
        "source": str(reference_path),
        "ticks": int(reference_source["ticks"]),
        "command_mismatches": int(official["canonical_bit_mismatches"]),
        "maximum_command_error": float(official["maximum_absolute_error"]),
        "upkie_latency": reference_source["latency"]["upkie_cpp"],
        "bonesaw_latency": reference_source["latency"]["bonesaw_official"],
        "upkie_peak_rss_bytes": int(reference_source["process"]["upkie_cpp"]["peak_rss_bytes"]),
        "bonesaw_peak_rss_bytes": int(reference_source["process"]["bonesaw_official"]["peak_rss_bytes"]),
        "bonesaw_allocations": int(reference_source["bonesaw_hot_loop_allocations"]["official"]["allocation_calls"]),
        "upkie_commit": reference_source["provenance"]["upkie_commit"],
    }

    timing = timing_metrics(timing_samples)
    wall_seconds = (wall_after - wall_before) / 1e9
    cpu_seconds = (cpu_after - cpu_before) / 1e9
    runtime = {
        "measured_calls": args.timing_repeats,
        "measured_queries": args.timing_repeats * args.samples,
        "wall_ms": wall_seconds * 1000.0,
        "cpu_ms": cpu_seconds * 1000.0,
        "cpu_to_wall": cpu_seconds / wall_seconds,
        "queries_per_second": args.timing_repeats * args.samples / wall_seconds,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
        "peak_rss_before_bytes": peak_rss_before,
        "peak_rss_after_bytes": peak_rss_after,
        "allocation_calls": measured_allocation_calls,
        "allocated_bytes": measured_allocated_bytes,
        "gc_collection_delta": int(sum(after - before for before, after in zip(gc_before, gc_after, strict=True))),
    }
    admission = bool(
        np.all(np.isin(baseline["status"], SOLVED))
        and oracle["dynamics_linf"] <= 2e-6
        and oracle["rolling_contact_linf"] <= 2e-6
        and oracle["minimum_normal_force_n"] >= -1e-8
        and oracle["minimum_square_friction_margin_n"] >= -2e-7
        and oracle["minimum_effort_margin_nm"] >= -2e-7
        and runtime["allocation_calls"] == 0
        and runtime["allocated_bytes"] == 0
        and invariance["repeat_bitwise_exact"]
        and invariance["reverse_permutation_exact"]
        and invariance["two_chunk_exact"]
        and slip_faults["pinocchio_stabilized_contact_linf"] <= 2e-6
        and slip_faults["all_outputs_finite"]
        and slip_faults["zero_rolling_coefficient_rejected"]
    )
    metrics = {
        "schema_version": 1,
        "revision": "upkie-state-local-wbc-r123",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": args.samples,
        "policy": None,
        "physics_rollout": None,
        "joint_names": joint_names,
        "contact_frames": contact_frames,
        "rolling_coordinates": coordinates.tolist(),
        "rolling_velocity_coefficients": coefficients.tolist(),
        "status_counts": status_counts(baseline["status"]),
        "independent_pinocchio_oracle": oracle,
        "tracking": tracking_metrics(corpus, baseline),
        "task_stack": task_stack_metrics(baseline),
        "bounds": bound_metrics(baseline),
        "timing": timing,
        "solver_work": solver_work_metrics(baseline),
        "runtime": runtime,
        "invariance": invariance,
        "resource_sweep": resources,
        "slip_and_faults": slip_faults,
        "execution_order_windows": execution_windows(corpus, baseline, oracle_raw),
        "retained_official_upkie_reference": reference,
        "admission": admission,
    }
    raw = {
        **{f"input_{name}": value for name, value in corpus.items()},
        **{f"output_{name}": value for name, value in baseline.items()},
        "timing_step_ns_all_repeats": timing_samples,
        **oracle_raw,
        **slip_raw,
    }
    np.savez_compressed(output / "upkie-state-local-wbc-raw.npz", **raw)
    (output / "upkie-state-local-wbc-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "UPKIE_STATE_LOCAL_WBC_AUDIT.md").write_text(markdown)
    web_report.write_text(
        render_report_html(markdown, title="Bonesaw Upkie state-local rolling WBC · r123")
    )
    print(json.dumps(metrics, indent=2))
    return 0 if admission else 1


if __name__ == "__main__":
    raise SystemExit(main())
