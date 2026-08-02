#!/usr/bin/env python3
"""Policy- and physics-free audit of swept world-SDF command admission."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import DynamicAdvanceSession
from cpu_reference_report import distribution, markdown_table, render_report_html


HORIZON_S = 0.020
SAMPLE_PERIOD_S = 0.001
PROBE_ROOT_ANGULAR_REACH_M = 1.8
MISSING = np.iinfo(np.uint64).max
PRIMARY_WORLD_COLLISION = 1 << 11
CONTINGENCY_WORLD_COLLISION = 1 << 12
PRIMARY_WORLD_CONTINUOUS = 1 << 13
CONTINGENCY_WORLD_CONTINUOUS = 1 << 14
PRIMARY_WORLD_UNKNOWN = 1 << 15
CONTINGENCY_WORLD_UNKNOWN = 1 << 16


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument("--cases", type=int, default=321)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument("--wall-x", type=float, default=0.20)
    parser.add_argument(
        "--root-error-growth",
        type=float,
        nargs=6,
        default=[0.0004, 0.020, 0.50, 0.0003, 0.015, 0.40],
        metavar=("P0", "V", "A", "R0", "W", "ALPHA"),
        help="translation radius/velocity/acceleration then rotation radius/velocity/acceleration bounds",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/world-sdf-root-uncertainty-r112"
    )
    parser.add_argument("--web-report", default="web/WORLD_SDF_ROOT_UNCERTAINTY_R112.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def make_plane_field(
    wall_x: float,
    *,
    maximum_x: float = 1.5,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    origin = np.array([-0.5, -1.0, -1.0], dtype=np.float64)
    spacing = np.array([0.1, 0.1, 0.1], dtype=np.float64)
    x_count = int(round((maximum_x - origin[0]) / spacing[0])) + 1
    dimensions_xyz = np.array([x_count, 21, 21], dtype=np.int64)
    world_x = origin[0] + spacing[0] * np.arange(dimensions_xyz[0])
    values = np.empty(
        (dimensions_xyz[2], dimensions_xyz[1], dimensions_xyz[0]),
        dtype=np.float64,
    )
    values[:] = (world_x - wall_x)[None, None, :]
    return values, origin, spacing


def make_session(
    args: argparse.Namespace,
    values: np.ndarray,
    origin: np.ndarray,
    spacing: np.ndarray,
    *,
    outside_policy: int = 0,
    certify: bool = True,
    depth: int = 0,
    error_growth: np.ndarray | None = None,
) -> DynamicAdvanceSession:
    if error_growth is None:
        error_growth = np.asarray(args.root_error_growth, dtype=np.float64)
    return DynamicAdvanceSession(
        args.model,
        [],
        maximum_generalized_acceleration=100.0,
        maximum_generalized_effort=1000.0,
        world_sdf_values_zyx=values,
        world_grid_origin_world=origin,
        world_grid_spacing=spacing,
        world_probe_body_names=["slider"],
        world_outside_policy=outside_policy,
        world_collision_clearance=args.clearance,
        certify_world_collision_between_samples=certify,
        world_collision_max_subdivision_depth=depth,
        root_prediction_error_growth=error_growth,
    )


@dataclass
class Trace:
    qdd: np.ndarray
    splice_position: np.ndarray
    splice_velocity: np.ndarray
    splice_acceleration: np.ndarray
    metrics: np.ndarray
    ids: np.ndarray
    work: np.ndarray
    root_prediction: np.ndarray
    timing: np.ndarray


def trace_buffers(session: DynamicAdvanceSession, ticks: int = 1) -> Trace:
    return Trace(
        qdd=np.zeros((ticks, session.generalized_dof), dtype=np.float64),
        splice_position=np.zeros((ticks, session.actuator_count), dtype=np.float64),
        splice_velocity=np.zeros((ticks, session.actuator_count), dtype=np.float64),
        splice_acceleration=np.zeros((ticks, session.actuator_count), dtype=np.float64),
        metrics=np.zeros((ticks, 9), dtype=np.float64),
        ids=np.zeros((ticks, 29), dtype=np.uint64),
        work=np.zeros((ticks, 8), dtype=np.uint64),
        root_prediction=np.zeros((ticks, 48), dtype=np.float64),
        timing=np.zeros((ticks, 3), dtype=np.uint64),
    )


def run_case(
    session: DynamicAdvanceSession,
    q: float,
    v: float,
    desired_joint_acceleration: float,
    output: Trace,
    *,
    start_time_ns: int = 0,
    root_twist_value: np.ndarray | None = None,
) -> None:
    session.reset_command_state(
        np.array([q], dtype=np.float64), np.array([v], dtype=np.float64)
    )
    observed_q = np.array([[q]], dtype=np.float64)
    observed_v = np.array([[v]], dtype=np.float64)
    root_twist = np.zeros((1, 6), dtype=np.float64)
    if root_twist_value is not None:
        root_twist[0] = root_twist_value
    desired = np.zeros((1, session.generalized_dof), dtype=np.float64)
    desired[0, 6] = desired_joint_acceleration
    session.run_world_trace(
        start_time_ns,
        observed_q,
        observed_v,
        root_twist,
        desired,
        output.qdd,
        output.splice_position,
        output.splice_velocity,
        output.splice_acceleration,
        output.metrics,
        output.ids,
        output.work,
        output.root_prediction,
        output.timing,
    )


def quintic_coefficients(
    q0: float,
    v0: float,
    a0: float,
    q1: float,
    v1: float,
    a1: float,
) -> np.ndarray:
    duration = HORIZON_S
    c0 = q0
    c1 = v0 * duration
    c2 = 0.5 * a0 * duration * duration
    p = q1 - (c0 + c1 + c2)
    velocity_residual = v1 * duration - (c1 + 2.0 * c2)
    acceleration_residual = a1 * duration * duration - 2.0 * c2
    return np.array(
        [
            c0,
            c1,
            c2,
            10.0 * p - 4.0 * velocity_residual + 0.5 * acceleration_residual,
            -15.0 * p + 7.0 * velocity_residual - acceleration_residual,
            6.0 * p - 3.0 * velocity_residual + 0.5 * acceleration_residual,
        ],
        dtype=np.float64,
    )


def segment_coefficients(output: Trace, *, primary: bool) -> np.ndarray:
    q0 = float(output.splice_position[0, 0])
    v0 = float(output.splice_velocity[0, 0])
    a0 = float(output.splice_acceleration[0, 0])
    if primary:
        acceleration = float(output.qdd[0, 6])
        q1 = q0 + HORIZON_S * v0 + 0.5 * HORIZON_S**2 * acceleration
        v1 = v0 + HORIZON_S * acceleration
        a1 = acceleration
    else:
        q1 = q0 + 0.5 * HORIZON_S * v0
        v1 = 0.0
        a1 = 0.0
    return quintic_coefficients(q0, v0, a0, q1, v1, a1)


def position(coefficients: np.ndarray, u: np.ndarray) -> np.ndarray:
    return np.polynomial.polynomial.polyval(u, coefficients)


def maximum_abs_velocity(coefficients: np.ndarray) -> float:
    velocity_u = np.arange(1, 6, dtype=np.float64) * coefficients[1:]
    acceleration_u = np.arange(1, 5, dtype=np.float64) * velocity_u[1:]
    roots = np.roots(acceleration_u[::-1]) if np.any(acceleration_u) else np.array([])
    candidates = [0.0, 1.0]
    candidates.extend(
        float(root.real)
        for root in roots
        if abs(root.imag) < 1e-10 and 0.0 <= root.real <= 1.0
    )
    normalized = np.polynomial.polynomial.polyval(candidates, velocity_u)
    return float(np.max(np.abs(normalized)) / HORIZON_S)


def distance_from_q(q: np.ndarray | float, wall_x: float) -> np.ndarray | float:
    # slider center = 1 + q; authored sphere radius = 0.2 m.
    return np.asarray(q) + 0.8 - wall_x


def prediction_error_erosion(
    growth: np.ndarray, u: np.ndarray | float
) -> np.ndarray | float:
    seconds = np.asarray(u) * HORIZON_S
    translation = growth[0] + growth[1] * seconds + 0.5 * growth[2] * seconds**2
    rotation = growth[3] + growth[4] * seconds + 0.5 * growth[5] * seconds**2
    return translation + PROBE_ROOT_ANGULAR_REACH_M * rotation


def maximum_prediction_error_erosion_rate(growth: np.ndarray) -> float:
    return float(
        growth[1]
        + growth[2] * HORIZON_S
        + PROBE_ROOT_ANGULAR_REACH_M * (growth[4] + growth[5] * HORIZON_S)
    )


def first_violation_time(
    distances: np.ndarray, clearance: float, *, start_time_ns: int = 0
) -> int:
    indices = np.flatnonzero(distances < clearance)
    return int(MISSING if indices.size == 0 else start_time_ns + indices[0] * 1_000_000)


def semantic_digest(output: Trace) -> str:
    digest = hashlib.sha256()
    for name in (
        "qdd",
        "splice_position",
        "splice_velocity",
        "splice_acceleration",
        "metrics",
        "ids",
        "work",
        "root_prediction",
    ):
        digest.update(name.encode())
        digest.update(getattr(output, name).tobytes())
    return digest.hexdigest()


def corpus_audit(
    args: argparse.Namespace,
    session: DynamicAdvanceSession,
) -> dict[str, Any]:
    output = trace_buffers(session)
    phase = np.linspace(0.0, 8.0 * np.pi, args.cases, endpoint=False)
    coordinates = np.linspace(-0.575, -0.30, args.cases)
    velocities = 0.75 * np.cos(0.73 * phase)
    root_twists = np.zeros((args.cases, 6), dtype=np.float64)
    root_twists[:, 3] = 0.35 * np.sin(0.91 * phase)
    desired = 80.0 * np.sin(1.11 * phase)
    coordinates[:6] = [-0.56, -0.55, -0.575, -0.56, -0.57, -0.54]
    velocities[:6] = [-0.5, -0.9, -0.8, 0.5, -0.3, 0.0]
    desired[:6] = [-100.0, -60.0, -100.0, 100.0, -80.0, -100.0]
    grid_u = np.linspace(0.0, 1.0, 21)
    distance_errors: list[float] = []
    continuity_errors: list[float] = []
    rate_errors: list[float] = []
    root_prediction_errors: list[float] = []
    prediction_erosion_errors: list[float] = []
    root_changed_sampled_outcome_count = 0
    maximum_root_distance_effect = 0.0
    violation_time_errors: list[int] = []
    conservative_gaps: list[float] = []
    sampled_outcomes = {"primary": 0, "contingency": 0}
    continuous_outcomes = {"primary": 0, "contingency": 0}
    certified_outcomes = {"primary": 0, "contingency": 0}
    selections = {"primary": 0, "contingency": 0, "rejected": 0}
    status_codes: set[int] = set()
    solve_codes: set[int] = set()
    flags_seen = 0
    max_allocations = 0
    max_allocated_bytes = 0

    for q, v, acceleration, root_twist in zip(
        coordinates, velocities, desired, root_twists, strict=True
    ):
        run_case(
            session,
            float(q),
            float(v),
            float(acceleration),
            output,
            root_twist_value=root_twist,
        )
        flags = int(output.ids[0, 14])
        flags_seen |= flags
        selection = int(output.ids[0, 16])
        selections[("primary", "contingency", "rejected")[selection]] += 1
        status_codes.add(int(output.ids[0, 15]))
        solve_codes.add(int(output.ids[0, 19]))
        max_allocations = max(max_allocations, int(output.timing[0, 1]))
        max_allocated_bytes = max(max_allocated_bytes, int(output.timing[0, 2]))

        for label, primary, metric_offset, flag_collision, flag_continuous in (
            ("primary", True, 0, PRIMARY_WORLD_COLLISION, PRIMARY_WORLD_CONTINUOUS),
            (
                "contingency",
                False,
                1,
                CONTINGENCY_WORLD_COLLISION,
                CONTINGENCY_WORLD_CONTINUOUS,
            ),
        ):
            coefficients = segment_coefficients(output, primary=primary)
            root_coefficients = np.stack(
                [
                    (
                        quintic_coefficients(
                            0.0,
                            float(root_twist[axis]),
                            float(output.qdd[0, axis]),
                            float(root_twist[axis]) * HORIZON_S
                            + 0.5 * float(output.qdd[0, axis]) * HORIZON_S**2,
                            float(root_twist[axis])
                            + float(output.qdd[0, axis]) * HORIZON_S,
                            float(output.qdd[0, axis]),
                        )
                        if primary
                        else quintic_coefficients(
                            0.0,
                            float(root_twist[axis]),
                            0.0,
                            0.5 * float(root_twist[axis]) * HORIZON_S,
                            0.0,
                            0.0,
                        )
                    )
                    for axis in range(6)
                ]
            )
            root_offset = 0 if primary else 24
            expected_root_end = np.concatenate(
                (
                    np.array([position(root_coefficients[axis], 1.0) for axis in range(3, 6)]),
                    np.array([position(root_coefficients[axis], 1.0) for axis in range(3)]),
                    np.array(
                        [
                            np.polynomial.polynomial.polyval(
                                1.0,
                                np.arange(1, 6, dtype=np.float64)
                                * root_coefficients[axis, 1:],
                            )
                            / HORIZON_S
                            for axis in range(6)
                        ]
                    ),
                    np.array(
                        [
                            maximum_abs_velocity(root_coefficients[axis])
                            for axis in range(6)
                        ]
                    ),
                )
            )
            root_prediction_errors.append(
                float(
                    np.max(
                        np.abs(
                            output.root_prediction[0, root_offset : root_offset + 18]
                            - expected_root_end
                        )
                    )
                )
            )
            expected_error_growth = np.asarray(args.root_error_growth, dtype=np.float64)
            root_prediction_errors.append(
                float(
                    np.max(
                        np.abs(
                            output.root_prediction[
                                0, root_offset + 18 : root_offset + 24
                            ]
                            - expected_error_growth
                        )
                    )
                )
            )
            nominal_grid_distance = np.asarray(
                distance_from_q(position(coefficients, grid_u), args.wall_x)
            ) + np.asarray(position(root_coefficients[3], grid_u))
            erosion_grid = np.asarray(
                prediction_error_erosion(expected_error_growth, grid_u)
            )
            grid_distance = nominal_grid_distance - erosion_grid
            fixed_root_grid_distance = np.asarray(
                distance_from_q(position(coefficients, grid_u), args.wall_x)
            )
            maximum_root_distance_effect = max(
                maximum_root_distance_effect,
                float(np.max(np.abs(grid_distance - fixed_root_grid_distance))),
            )
            expected_minimum = float(np.min(grid_distance))
            observed_minimum = float(output.metrics[0, metric_offset])
            distance_errors.append(abs(observed_minimum - expected_minimum))
            observed_erosion = float(output.metrics[0, 7 if primary else 8])
            prediction_erosion_errors.append(
                abs(observed_erosion - float(np.max(erosion_grid)))
            )
            expected_violation_time = first_violation_time(
                grid_distance, args.clearance
            )
            observed_violation_time = int(output.ids[0, 5 if primary else 7])
            if expected_violation_time == int(MISSING):
                violation_time_errors.append(
                    int(observed_violation_time != int(MISSING))
                )
            else:
                violation_time_errors.append(
                    abs(observed_violation_time - expected_violation_time)
                )
            sampled_collision = expected_violation_time != int(MISSING)
            fixed_root_collision = bool(np.any(fixed_root_grid_distance < args.clearance))
            root_changed_sampled_outcome_count += int(
                sampled_collision != fixed_root_collision
            )
            sampled_outcomes[label] += int(sampled_collision)
            if sampled_collision:
                if flags & flag_collision == 0:
                    raise RuntimeError(f"{label} sampled collision flag missing")
                if not np.isneginf(output.metrics[0, metric_offset + 2]):
                    raise RuntimeError(f"{label} collided segment gained certificate")
                continue

            speed_bound = maximum_abs_velocity(coefficients) + sum(
                maximum_abs_velocity(root_coefficients[axis]) for axis in range(6)
            )
            speed_bound += maximum_prediction_error_erosion_rate(expected_error_growth)
            expected_lower = expected_minimum - speed_bound * (0.5 * SAMPLE_PERIOD_S)
            observed_lower = float(output.metrics[0, metric_offset + 2])
            observed_rate = float(output.metrics[0, metric_offset + 4])
            continuity_errors.append(abs(observed_lower - expected_lower))
            rate_errors.append(abs(observed_rate - speed_bound))
            dense_u = np.linspace(0.0, 1.0, 20_001)
            dense_minimum = float(
                np.min(
                    distance_from_q(position(coefficients, dense_u), args.wall_x)
                    + position(root_coefficients[3], dense_u)
                    - prediction_error_erosion(expected_error_growth, dense_u)
                )
            )
            conservative_gaps.append(dense_minimum - observed_lower)
            continuous_failure = expected_lower < args.clearance
            continuous_outcomes[label] += int(continuous_failure)
            certified_outcomes[label] += int(not continuous_failure)
            if bool(flags & flag_continuous) != continuous_failure:
                raise RuntimeError(f"{label} continuous-clearance flag mismatch")

    return {
        "case_count": args.cases,
        "maximum_sampled_distance_oracle_error_m": float(max(distance_errors)),
        "maximum_violation_time_oracle_error_ns": int(max(violation_time_errors)),
        "maximum_continuity_lower_bound_oracle_error_m": float(
            max(continuity_errors, default=0.0)
        ),
        "maximum_rate_bound_oracle_error_mps": float(max(rate_errors, default=0.0)),
        "maximum_root_prediction_oracle_error": float(
            max(root_prediction_errors, default=0.0)
        ),
        "maximum_prediction_erosion_oracle_error_m": float(
            max(prediction_erosion_errors, default=0.0)
        ),
        "root_changed_sampled_outcome_count": root_changed_sampled_outcome_count,
        "maximum_root_distance_effect_m": maximum_root_distance_effect,
        "minimum_dense_truth_minus_certificate_m": float(min(conservative_gaps)),
        "sampled_collision_counts": sampled_outcomes,
        "continuous_only_failure_counts": continuous_outcomes,
        "certified_counts": certified_outcomes,
        "selections": selections,
        "status_codes": sorted(status_codes),
        "solve_status_codes": sorted(solve_codes),
        "flags_union": flags_seen,
        "maximum_allocation_calls": max_allocations,
        "maximum_allocated_bytes": max_allocated_bytes,
    }


def find_adaptive_case(
    args: argparse.Namespace,
    grid_session: DynamicAdvanceSession,
    adaptive_session: DynamicAdvanceSession,
) -> dict[str, Any]:
    grid_output = trace_buffers(grid_session)
    adaptive_output = trace_buffers(adaptive_session)
    candidates = [
        (q, v, acceleration)
        for q in np.linspace(-0.555, -0.48, 16)
        for v in np.linspace(-0.9, 0.9, 13)
        for acceleration in (-100.0, -60.0, 0.0, 60.0, 100.0)
    ]
    selected: tuple[float, float, float] | None = None
    for q, v, acceleration in candidates:
        run_case(grid_session, float(q), float(v), acceleration, grid_output)
        run_case(adaptive_session, float(q), float(v), acceleration, adaptive_output)
        grid_clear = int(grid_output.ids[0, 4]) == int(MISSING)
        adaptive_clear = int(adaptive_output.ids[0, 4]) == int(MISSING)
        grid_lower = float(grid_output.metrics[0, 2])
        adaptive_lower = float(adaptive_output.metrics[0, 2])
        if (
            grid_clear
            and adaptive_clear
            and grid_lower < args.clearance
            and adaptive_lower >= args.clearance
            and int(adaptive_output.work[0, 1]) > 0
        ):
            selected = (float(q), float(v), acceleration)
            break
    if selected is None:
        raise RuntimeError("could not find deterministic adaptive-certification fixture")

    q, v, acceleration = selected
    run_case(grid_session, q, v, acceleration, grid_output)
    run_case(adaptive_session, q, v, acceleration, adaptive_output)
    coefficients = segment_coefficients(adaptive_output, primary=True)
    root_x_acceleration = float(adaptive_output.qdd[0, 3])
    root_x_coefficients = quintic_coefficients(
        0.0,
        0.0,
        root_x_acceleration,
        0.5 * root_x_acceleration * HORIZON_S**2,
        root_x_acceleration * HORIZON_S,
        root_x_acceleration,
    )
    dense_u = np.linspace(0.0, 1.0, 100_001)
    dense_minimum = float(
        np.min(
            distance_from_q(position(coefficients, dense_u), args.wall_x)
            + position(root_x_coefficients, dense_u)
            - prediction_error_erosion(
                np.asarray(args.root_error_growth, dtype=np.float64), dense_u
            )
        )
    )
    adaptive_lower = float(adaptive_output.metrics[0, 2])
    return {
        "q": q,
        "v": v,
        "desired_joint_acceleration_mps2": acceleration,
        "grid_only_lower_bound_m": float(grid_output.metrics[0, 2]),
        "adaptive_lower_bound_m": adaptive_lower,
        "dense_minimum_m": dense_minimum,
        "dense_truth_minus_adaptive_certificate_m": dense_minimum - adaptive_lower,
        "primary_leaf_intervals": int(adaptive_output.work[0, 0]),
        "primary_midpoint_samples": int(adaptive_output.work[0, 1]),
        "primary_unresolved_intervals": int(adaptive_output.work[0, 2]),
        "primary_maximum_depth": int(adaptive_output.work[0, 3]),
        "grid_selection": int(grid_output.ids[0, 16]),
        "adaptive_selection": int(adaptive_output.ids[0, 16]),
        "adaptive_allocation_calls": int(adaptive_output.timing[0, 1]),
        "adaptive_allocated_bytes": int(adaptive_output.timing[0, 2]),
    }


def root_motion_admission_audit(
    args: argparse.Namespace, session: DynamicAdvanceSession
) -> dict[str, Any]:
    output = trace_buffers(session)
    for q in np.linspace(-0.579, -0.54, 40):
        for root_velocity_x in np.linspace(-0.9, -0.2, 29):
            root_twist = np.zeros(6, dtype=np.float64)
            root_twist[3] = root_velocity_x
            run_case(
                session,
                float(q),
                0.0,
                0.0,
                output,
                root_twist_value=root_twist,
            )
            fixed_root_minimum = float(
                np.min(
                    distance_from_q(
                        position(
                            segment_coefficients(output, primary=True),
                            np.linspace(0.0, 1.0, 21),
                        ),
                        args.wall_x,
                    )
                )
            )
            if (
                fixed_root_minimum >= args.clearance
                and int(output.ids[0, 4]) != int(MISSING)
                and int(output.ids[0, 16]) == 1
                and bool(output.ids[0, 18])
            ):
                return {
                    "q": float(q),
                    "root_velocity_x_mps": float(root_velocity_x),
                    "fixed_root_primary_minimum_m": fixed_root_minimum,
                    "predicted_root_primary_minimum_m": float(output.metrics[0, 0]),
                    "predicted_root_contingency_minimum_m": float(output.metrics[0, 1]),
                    "primary_first_violation_time_ns": int(output.ids[0, 5]),
                    "selection": int(output.ids[0, 16]),
                    "primary_valid": bool(output.ids[0, 17]),
                    "contingency_valid": bool(output.ids[0, 18]),
                    "maximum_primary_root_twist_mps": float(
                        output.root_prediction[0, 15]
                    ),
                }
    raise RuntimeError("could not find a root-only world-admission transfer fixture")


def prediction_uncertainty_admission_audit(
    args: argparse.Namespace,
    nominal_session: DynamicAdvanceSession,
    bounded_session: DynamicAdvanceSession,
) -> dict[str, Any]:
    nominal = trace_buffers(nominal_session)
    bounded = trace_buffers(bounded_session)
    for q in np.linspace(-0.579, -0.55, 117):
        run_case(nominal_session, float(q), 0.0, 0.0, nominal)
        run_case(bounded_session, float(q), 0.0, 0.0, bounded)
        if (
            float(nominal.metrics[0, 0]) >= args.clearance
            and int(nominal.ids[0, 16]) == 0
            and float(bounded.metrics[0, 0]) < args.clearance
            and int(bounded.ids[0, 4]) != int(MISSING)
        ):
            growth = np.asarray(args.root_error_growth, dtype=np.float64)
            return {
                "q": float(q),
                "nominal_primary_minimum_m": float(nominal.metrics[0, 0]),
                "bounded_primary_minimum_m": float(bounded.metrics[0, 0]),
                "bounded_contingency_minimum_m": float(bounded.metrics[0, 1]),
                "maximum_clearance_erosion_m": float(bounded.metrics[0, 7]),
                "expected_maximum_clearance_erosion_m": float(
                    prediction_error_erosion(growth, 1.0)
                ),
                "first_violation_time_ns": int(bounded.ids[0, 5]),
                "nominal_selection": int(nominal.ids[0, 16]),
                "bounded_selection": int(bounded.ids[0, 16]),
                "bounded_primary_valid": bool(bounded.ids[0, 17]),
                "bounded_contingency_valid": bool(bounded.ids[0, 18]),
                "error_growth": growth.tolist(),
            }
    raise RuntimeError("could not find prediction-error-only admission fixture")


def boundary_policy_audit(args: argparse.Namespace) -> dict[str, Any]:
    values, origin, spacing = make_plane_field(-1.0, maximum_x=1.2)
    reject = make_session(
        args, values, origin, spacing, outside_policy=0, certify=True, depth=3
    )
    occupied = make_session(
        args, values, origin, spacing, outside_policy=1, certify=True, depth=3
    )
    reject_output = trace_buffers(reject)
    occupied_output = trace_buffers(occupied)
    q = 0.19
    run_case(reject, q, 0.0, 100.0, reject_output)
    run_case(occupied, q, 0.0, 100.0, occupied_output)
    return {
        "reject": {
            "primary_unknown_probe": int(reject_output.ids[0, 8]),
            "primary_unknown_time_ns": int(reject_output.ids[0, 9]),
            "contingency_unknown_probe": int(reject_output.ids[0, 10]),
            "selection": int(reject_output.ids[0, 16]),
            "primary_valid": bool(reject_output.ids[0, 17]),
            "contingency_valid": bool(reject_output.ids[0, 18]),
            "flags": int(reject_output.ids[0, 14]),
        },
        "occupied": {
            "primary_source": int(occupied_output.ids[0, 20]),
            "primary_violation_probe": int(occupied_output.ids[0, 4]),
            "primary_violation_time_ns": int(occupied_output.ids[0, 5]),
            "selection": int(occupied_output.ids[0, 16]),
            "primary_valid": bool(occupied_output.ids[0, 17]),
            "contingency_valid": bool(occupied_output.ids[0, 18]),
            "flags": int(occupied_output.ids[0, 14]),
        },
    }


def benchmark(
    args: argparse.Namespace,
    session: DynamicAdvanceSession,
) -> dict[str, Any]:
    output = trace_buffers(session)
    q, v, acceleration = -0.42, 0.18, 25.0
    for _ in range(50):
        run_case(session, q, v, acceleration, output)
    elapsed_us = np.zeros(args.repeats, dtype=np.float64)
    allocation_calls = np.zeros(args.repeats, dtype=np.uint64)
    allocated_bytes = np.zeros(args.repeats, dtype=np.uint64)
    first_digest: str | None = None
    exact_replay = True
    for repeat in range(args.repeats):
        run_case(session, q, v, acceleration, output)
        elapsed_us[repeat] = output.timing[0, 0] * 1e-3
        allocation_calls[repeat] = output.timing[0, 1]
        allocated_bytes[repeat] = output.timing[0, 2]
        digest = semantic_digest(output)
        if first_digest is None:
            first_digest = digest
        else:
            exact_replay &= digest == first_digest
    return {
        "timing_us": distribution(elapsed_us),
        "maximum_allocation_calls": int(np.max(allocation_calls)),
        "maximum_allocated_bytes": int(np.max(allocated_bytes)),
        "semantic_replay_exact": exact_replay,
        "semantic_digest": first_digest,
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    values, origin, spacing = make_plane_field(args.wall_x)
    grid_session = make_session(
        args, values, origin, spacing, certify=True, depth=0
    )
    adaptive_session = make_session(
        args, values, origin, spacing, certify=True, depth=3
    )
    nominal_session = make_session(
        args,
        values,
        origin,
        spacing,
        certify=True,
        depth=3,
        error_growth=np.zeros(6, dtype=np.float64),
    )
    if grid_session.dof != 1 or grid_session.actuator_count != 1:
        raise RuntimeError("world command fixture no longer has one slider coordinate")
    corpus = corpus_audit(args, grid_session)
    root_motion = root_motion_admission_audit(args, adaptive_session)
    uncertainty = prediction_uncertainty_admission_audit(
        args, nominal_session, adaptive_session
    )
    adaptive = find_adaptive_case(args, grid_session, adaptive_session)
    boundary = boundary_policy_audit(args)
    benchmark_result = benchmark(args, adaptive_session)
    timing = benchmark_result["timing_us"]
    reject = boundary["reject"]
    occupied = boundary["occupied"]
    passed = bool(
        corpus["maximum_sampled_distance_oracle_error_m"] < 1e-12
        and corpus["maximum_violation_time_oracle_error_ns"] == 0
        and corpus["maximum_continuity_lower_bound_oracle_error_m"] < 1e-12
        and corpus["maximum_rate_bound_oracle_error_mps"] < 1e-10
        and corpus["maximum_root_prediction_oracle_error"] < 1e-10
        and corpus["maximum_prediction_erosion_oracle_error_m"] < 1e-12
        and root_motion["fixed_root_primary_minimum_m"] >= args.clearance
        and root_motion["predicted_root_primary_minimum_m"] < args.clearance
        and root_motion["selection"] == 1
        and not root_motion["primary_valid"]
        and root_motion["contingency_valid"]
        and uncertainty["nominal_primary_minimum_m"] >= args.clearance
        and uncertainty["bounded_primary_minimum_m"] < args.clearance
        and uncertainty["nominal_selection"] == 0
        and not uncertainty["bounded_primary_valid"]
        and abs(
            uncertainty["maximum_clearance_erosion_m"]
            - uncertainty["expected_maximum_clearance_erosion_m"]
        )
        < 1e-12
        and corpus["minimum_dense_truth_minus_certificate_m"] >= -1e-12
        and corpus["maximum_allocation_calls"] == 0
        and corpus["maximum_allocated_bytes"] == 0
        and adaptive["grid_only_lower_bound_m"] < args.clearance
        and adaptive["adaptive_lower_bound_m"] >= args.clearance
        and adaptive["dense_truth_minus_adaptive_certificate_m"] >= -1e-12
        and adaptive["primary_midpoint_samples"] > 0
        and adaptive["primary_unresolved_intervals"] == 0
        and adaptive["primary_maximum_depth"] <= 3
        and adaptive["adaptive_allocation_calls"] == 0
        and adaptive["adaptive_allocated_bytes"] == 0
        and reject["primary_unknown_probe"] != int(MISSING)
        and reject["contingency_unknown_probe"] == int(MISSING)
        and reject["selection"] == 1
        and reject["flags"] & PRIMARY_WORLD_UNKNOWN
        and not reject["primary_valid"]
        and reject["contingency_valid"]
        and occupied["primary_source"] == 1
        and occupied["primary_violation_probe"] != int(MISSING)
        and occupied["selection"] == 2
        and occupied["flags"] & PRIMARY_WORLD_COLLISION
        and occupied["flags"] & CONTINGENCY_WORLD_COLLISION
        and not occupied["primary_valid"]
        and not occupied["contingency_valid"]
        and benchmark_result["semantic_replay_exact"]
        and benchmark_result["maximum_allocation_calls"] == 0
        and benchmark_result["maximum_allocated_bytes"] == 0
        and timing["p99"] < 5_000.0
        and timing["maximum"] < 50_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free, integrator-free WBC command transactions",
        "model": args.model,
        "grid_dimensions_xyz": [values.shape[2], values.shape[1], values.shape[0]],
        "grid_spacing_m": spacing.tolist(),
        "wall_x_m": args.wall_x,
        "clearance_m": args.clearance,
        "root_prediction_error_growth": list(args.root_error_growth),
        "corpus": corpus,
        "root_motion_admission": root_motion,
        "prediction_uncertainty_admission": uncertainty,
        "adaptive_refinement": adaptive,
        "boundary_policy": boundary,
        "benchmark": benchmark_result,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    corpus = audit["corpus"]
    root_motion = audit["root_motion_admission"]
    uncertainty = audit["prediction_uncertainty_admission"]
    adaptive = audit["adaptive_refinement"]
    boundary = audit["boundary_policy"]
    benchmark_result = audit["benchmark"]
    timing = benchmark_result["timing_us"]
    timing_table = "\n".join(
        markdown_table(
            ["transaction", "p50 / p99 / max µs", "alloc calls / bytes", "replay"],
            [[
                "WBC + primary/brake + two swept-world certificates",
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{benchmark_result['maximum_allocation_calls']} / {benchmark_result['maximum_allocated_bytes']}",
                benchmark_result["semantic_replay_exact"],
            ]],
        )
    )
    outcome_table = "\n".join(
        markdown_table(
            ["plan", "sampled collision", "continuous-only failure", "certified"],
            [
                [
                    label,
                    corpus["sampled_collision_counts"][label],
                    corpus["continuous_only_failure_counts"][label],
                    corpus["certified_counts"][label],
                ]
                for label in ("primary", "contingency")
            ],
        )
    )
    return f"""# Bonesaw robust floating-root swept world-SDF authority · r112

## Outcome

**{audit['status'].upper()}.** The CPU command controller now certifies the WBC primary trajectory and independently generated brake against an immutable world SDF after robustifying both by a deterministic floating-root forecast-error envelope. The audit supplies states and desired accelerations directly; there is no policy, plant integration, contact response, physics engine, covariance, or probability claim.

The independent NumPy oracle reconstructed every actuator quintic, both floating-root prediction witnesses, and their translation/attitude error erosion for {corpus['case_count']} transactions. Maximum sampled-distance, first-violation-time, continuity-lower-bound, distance-rate-bound, root-prediction, and erosion errors were `{corpus['maximum_sampled_distance_oracle_error_m']:.3e}` m, `{corpus['maximum_violation_time_oracle_error_ns']}` ns, `{corpus['maximum_continuity_lower_bound_oracle_error_m']:.3e}` m, `{corpus['maximum_rate_bound_oracle_error_mps']:.3e}` m/s, `{corpus['maximum_root_prediction_oracle_error']:.3e}`, and `{corpus['maximum_prediction_erosion_oracle_error_m']:.3e}` m. The least dense-truth-minus-certificate gap was `{corpus['minimum_dense_truth_minus_certificate_m']:.3e}` m; a negative value would invalidate the certificate.

The discriminating root-motion fixture starts at joint position `{root_motion['q']:.4f}` rad with root x velocity `{root_motion['root_velocity_x_mps']:.3f}` m/s. Freezing the root would report `{root_motion['fixed_root_primary_minimum_m'] * 1000:.3f}` mm clearance, but the explicit primary root sweep reaches `{root_motion['predicted_root_primary_minimum_m'] * 1000:.3f}` mm and first violates at `{root_motion['primary_first_violation_time_ns']}` ns. The independently swept brake remains at `{root_motion['predicted_root_contingency_minimum_m'] * 1000:.3f}` mm, so authority transfers to Contingency.

The prediction-error-only fixture holds the nominal root still. With zero declared error it has `{uncertainty['nominal_primary_minimum_m'] * 1000:.3f}` mm and selects Primary; the configured envelope erodes at most `{uncertainty['maximum_clearance_erosion_m'] * 1000:.3f}` mm, reducing robust clearance to `{uncertainty['bounded_primary_minimum_m'] * 1000:.3f}` mm and withholding the command at `{uncertainty['first_violation_time_ns']}` ns. This is continuous loss of authority from explicit forecast quality, not a delayed timeout.

{outcome_table}

Selections were `{corpus['selections']}`. Sampled geometry, between-sample continuity, and unknown-space faults have separate bits and witnesses; they are not collapsed into one health score.

## Bounded adaptive refinement

The deterministic fixture `q={adaptive['q']:.4f}`, `v={adaptive['v']:.4f}` m/s, requested acceleration `{adaptive['desired_joint_acceleration_mps2']:.1f}` m/s² demonstrates useful refinement. The global 1 ms certificate was `{adaptive['grid_only_lower_bound_m']:.6f}` m and withheld the primary. Depth-bounded midpoint refinement raised the proven lower bound to `{adaptive['adaptive_lower_bound_m']:.6f}` m against dense truth `{adaptive['dense_minimum_m']:.6f}` m, using {adaptive['primary_leaf_intervals']} leaves, {adaptive['primary_midpoint_samples']} added probe samples, depth {adaptive['primary_maximum_depth']}, and no unresolved leaves.

This is continuous-time command admission with a fixed work budget. A certificate that remains below the {audit['clearance_m']:.3f} m threshold is withheld; the controller does not wait extra servo steps hoping the pose becomes feasible.

## Unknown-space authority

With Reject semantics, the primary left the known volume at `{boundary['reject']['primary_unknown_time_ns']}` ns, set the dedicated unknown bit, and transferred authority to the known-clear brake (`selection={boundary['reject']['selection']}`). With OccupiedBoundary semantics, the field boundary itself is occupied: the 0.2 m sphere already overlaps it, so both primary and brake are measured collisions (`source={boundary['occupied']['primary_source']}`) and the selector rejects (`selection={boundary['occupied']['selection']}`). Unknown and occupied space therefore remain distinguishable while both fail closed.

## Timing and allocation

{timing_table}

Timing covers Rust WBC solve, actuator mapping, exact primary/brake construction, 21-knot scans for both plans, bounded continuous certification, and selection. Python reset/statistics/report work is outside the timed region.

## Example authority stack

1. **Intent/task rows** — desired generalized and end-effector accelerations; may be relaxed by their authored priority.
2. **Local world viability (HOCBF)** — state-local acceleration authority near the SDF wall; separately reports margin and residual.
3. **Floating-root forecast** — primary/brake local-SE(3) paths in smooth `control_world`; prediction evidence, never base actuation.
4. **Root forecast error** — deterministic translation/attitude radii and growth rates become probe-specific clearance erosion; separate from the nominal path.
5. **Primary sampled world geometry** — exact 1 ms robust command-knot evidence with probe, body, source, and first-violation time.
6. **Primary continuous world clearance** — motion plus error-growth Lipschitz/rate certificate and bounded refinement work.
7. **Contingency sampled + continuous world clearance** — independently witnesses the brake; never inherits primary validity.
8. **Joint, actuator, support, effort, thermal, and solver-budget authority** — orthogonal typed limits remain visible alongside world geometry.
9. **Command selector** — primary, contingency, or reject. Only a valid plan is emitted; later plant tracking and reliability remain separate evidence.

## Contract and remaining boundary

The field epoch and probe set are immutable for the controller instance. Sphere covers conservatively represent authored robot collision geometry; this is not arbitrary mesh CCD. The rate proof composes joint motion, an explicit local-SE(3) floating-root prediction in smooth `control_world`, and deterministic radial prediction-error growth. Root linear travel, conservative authored angular reach, and pose-error erosion participate at every knot and adaptive midpoint. The bounds are externally declared worst-case evidence, not learned covariance, an executable root command, or a plant rollout. Moving fields, external-obstacle epochs, estimator calibration, actuator/thermal response, and mesh validation remain explicit later milestones.
"""


def main() -> None:
    args = parse_args()
    if (
        args.repeats <= 1
        or args.cases < 32
        or not np.isfinite(args.clearance)
        or args.clearance < 0.0
        or not np.isfinite(args.wall_x)
        or len(args.root_error_growth) != 6
        or not np.all(np.isfinite(args.root_error_growth))
        or np.any(np.asarray(args.root_error_growth) < 0.0)
    ):
        raise SystemExit("invalid repeat count, corpus size, clearance, or wall")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    model = pathlib.Path(args.model)
    audit = evaluate(args)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/world_collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "world-sdf-root-uncertainty-r112",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "world-sdf-command-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "WORLD_SDF_COMMAND_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(
            report, title="Bonesaw robust floating-root swept world-SDF authority · r112"
        )
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
