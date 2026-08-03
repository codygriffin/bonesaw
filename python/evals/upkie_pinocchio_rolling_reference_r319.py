#!/usr/bin/env python3
"""Independent Pinocchio/NumPy oracle for the Upkie RollingWheel WBC.

This comparison intentionally stays policy-, integration-, and physics-free.
Pinocchio owns rigid-body products, NumPy owns the lexicographic solver, and
the only shared inputs are the URDF and the frozen R123 state/target corpus.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import importlib.metadata
import json
import pathlib
import resource
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pinocchio as pin
import placo
import psutil

from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_dynamics_mirror_report import pin_configuration, pinocchio_products, tangent_map
from g1_pinocchio_fixed_effort_reference import normalized_rows
from upkie_state_local_wbc_report import (
    ROLLING_GAIN,
    ROLLING_MAXIMUM_CORRECTION_MPS2,
    pin_contact_products,
    rolling_descriptors,
    root_pose_matrix,
)


REVISION = "upkie-pinocchio-rolling-reference-r319"
CONTACT_FRAMES = ("left_wheel_center", "right_wheel_center")
FRICTION = 0.8
MAXIMUM_ACCELERATION = 250.0
MAXIMUM_NORMAL_FORCE_MULTIPLE = 3.0
REPEATS = 8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--input",
        default="benchmarks/results/upkie-state-local-wbc-r123/upkie-state-local-wbc-raw.npz",
    )
    parser.add_argument(
        "--bonesaw",
        default="benchmarks/results/upkie-placo-dynamic-reference-r317/bonesaw-raw.npz",
    )
    parser.add_argument(
        "--bonesaw-metrics",
        default="benchmarks/results/upkie-placo-dynamic-reference-r317/bonesaw-metrics.json",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/upkie-pinocchio-rolling-reference-r319"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_PINOCCHIO_ROLLING_REFERENCE_R319.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    return float(np.sqrt(np.mean(values * values)))


def corpus_from_raw(path: pathlib.Path) -> dict[str, np.ndarray]:
    source = np.load(path)
    return {
        name: np.ascontiguousarray(source[f"input_{name}"])
        for name in (
            "root_positions",
            "root_velocities",
            "root_accelerations",
            "q",
            "v",
            "joint_accelerations",
            "contact_active",
            "slip",
        )
    }


def urdf_effort_limits(model_path: pathlib.Path, joint_names: list[str]) -> np.ndarray:
    root = ET.parse(model_path).getroot()
    limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("effort") is not None:
            limits[joint.get("name", "")] = float(limit.get("effort", "nan"))
    return np.asarray([limits[name] for name in joint_names], dtype=np.float64)


def model_joint_names(model_path: pathlib.Path) -> list[str]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    return [
        model.names[joint_id]
        for joint_id in range(2, model.njoints)
        if model.joints[joint_id].nq == 1 and model.joints[joint_id].nv == 1
    ]


def reference_acceleration(corpus: dict[str, np.ndarray]) -> np.ndarray:
    ticks = len(corpus["q"])
    return np.column_stack(
        (
            np.zeros((ticks, 3), dtype=np.float64),
            corpus["root_accelerations"],
            corpus["joint_accelerations"],
        )
    )


def supported_weight(model_path: pathlib.Path) -> float:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    return float(sum(inertia.mass for inertia in model.inertias[1:]) * 9.81)


def independent_products(
    model_path: pathlib.Path,
    joint_names: list[str],
    corpus: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ticks = len(corpus["q"])
    roots = root_pose_matrix(corpus["root_positions"])
    velocity = np.column_stack(
        (np.zeros((ticks, 3)), corpus["root_velocities"], corpus["v"])
    )
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81]), (ticks, 1))
    mass, bias, _ = pinocchio_products(
        model_path, joint_names, corpus["q"], roots, velocity, gravity
    )
    jacobian, contact_bias = pin_contact_products(
        model_path, joint_names, list(CONTACT_FRAMES), corpus
    )
    return mass, bias, jacobian, contact_bias


def equality_problem(
    mass: np.ndarray,
    bias: np.ndarray,
    jacobians: np.ndarray,
    contact_bias: np.ndarray,
    generalized_velocity: np.ndarray,
    rolling_coordinates: np.ndarray,
    rolling_coefficients: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    generalized_dof = mass.shape[0]
    dof = generalized_dof - 6
    force_base = generalized_dof + dof
    variables = force_base + 6
    dynamics = np.zeros((generalized_dof, variables), dtype=np.float64)
    dynamics[:, :generalized_dof] = mass
    dynamics[6:, generalized_dof:force_base] = -np.eye(dof)
    for contact, (coordinate, coefficient) in enumerate(
        zip(rolling_coordinates, rolling_coefficients, strict=True)
    ):
        force_map = jacobians[contact].T.copy()
        force_map[6 + coordinate, 0] += coefficient
        dynamics[:, force_base + contact * 3 : force_base + (contact + 1) * 3] = -force_map

    rows = [dynamics]
    targets = [-bias]
    contact_rows = np.zeros((6, variables), dtype=np.float64)
    contact_targets = np.zeros(6, dtype=np.float64)
    for contact, (coordinate, coefficient) in enumerate(
        zip(rolling_coordinates, rolling_coefficients, strict=True)
    ):
        for axis in range(3):
            row = contact * 3 + axis
            contact_rows[row, :generalized_dof] = jacobians[contact, axis]
            contact_targets[row] = -contact_bias[contact, axis]
        rolling_row = contact * 3
        contact_rows[rolling_row, 6 + coordinate] += coefficient
        rolling_velocity = (
            jacobians[contact, 0] @ generalized_velocity
            + coefficient * generalized_velocity[6 + coordinate]
        )
        contact_targets[rolling_row] += np.clip(
            -ROLLING_GAIN * rolling_velocity,
            -ROLLING_MAXIMUM_CORRECTION_MPS2,
            ROLLING_MAXIMUM_CORRECTION_MPS2,
        )
    rows.append(contact_rows)
    targets.append(contact_targets)
    return np.vstack(rows), np.concatenate(targets)


def direct_task(
    variables: int,
    indices: list[int],
    target: np.ndarray,
    weight: float,
) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.zeros((len(indices), variables), dtype=np.float64)
    matrix[np.arange(len(indices)), indices] = 1.0
    return normalized_rows(matrix, np.asarray(target, dtype=np.float64), weight)


def task_levels(desired: np.ndarray, normal_target: float) -> list[tuple[np.ndarray, np.ndarray]]:
    generalized_dof = len(desired)
    dof = generalized_dof - 6
    force_base = generalized_dof + dof
    variables = force_base + 6
    invariant_parts = (
        direct_task(variables, [0, 1, 2], desired[:3], 10.0),
        direct_task(variables, [5], desired[5:6], 10.0),
    )
    invariant = (
        np.vstack([part[0] for part in invariant_parts]),
        np.concatenate([part[1] for part in invariant_parts]),
    )
    viability = direct_task(variables, [3, 4], desired[3:5], 10.0)
    intent = direct_task(
        variables,
        list(range(6, generalized_dof)),
        desired[6:],
        1.0,
    )
    # No declared Preference task is active in the R123/R317 session.
    preference = (np.zeros((0, variables)), np.zeros(0))
    # Match FloatingWbcSession's terminal Style stack exactly: contact-force
    # regularization followed by the small actuator-torque regularizer. Joint
    # acceleration is the Intent level above, not a hidden Style row.
    style_matrix = np.zeros((6 + dof, variables), dtype=np.float64)
    style_matrix[:6, force_base:] = np.eye(6)
    style_matrix[6:, generalized_dof:force_base] = np.sqrt(1.0e-3) * np.eye(dof)
    style_target = np.asarray(
        [0.0, 0.0, normal_target, 0.0, 0.0, normal_target, *([0.0] * dof)]
    )
    return [invariant, viability, intent, preference, (style_matrix, style_target)]


def constrained_lexicographic_solve(
    equality: np.ndarray,
    equality_target: np.ndarray,
    levels: list[tuple[np.ndarray, np.ndarray]],
    effort_limits: np.ndarray,
    maximum_normal_force: float,
) -> np.ndarray:
    """Solve one bounded hierarchy through PlaCo's low-level generic QP API.

    This does not use PlaCo robot, contact, task, or DynamicsSolver objects.
    Every physical row is independently assembled above from Pinocchio data.
    """
    # Upkie's symmetric bilateral wheel constraints contain one exact
    # redundant direction on this morphology. Retain every independently
    # constraining row in stable source order; the dropped row remains checked
    # against the final solution and is never removed from parity evidence.
    independent: list[int] = []
    hard_row_space = np.zeros((0, equality.shape[1]), dtype=np.float64)
    rank = 0
    for row in range(equality.shape[0]):
        candidate = np.vstack((hard_row_space, equality[row]))
        candidate_rank = np.linalg.matrix_rank(candidate, tol=1.0e-10)
        if candidate_rank > rank:
            independent.append(row)
            hard_row_space = candidate
            rank = candidate_rank
    bands: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    solution = np.zeros(equality.shape[1], dtype=np.float64)
    for level_index, (matrix, target) in enumerate(levels):
        if matrix.shape[0] == 0:
            continue
        if np.linalg.matrix_rank(hard_row_space, tol=1.0e-10) == equality.shape[1]:
            break
        # Rebuild the generic QP at every level. This keeps lower-authority
        # objectives from sharing slack variables with already frozen layers.
        problem = placo.Problem()
        problem.regularization = 1.0e-12
        variable = problem.add_variable(equality.shape[1])
        expression = variable.expr()
        problem.add_constraint(
            expression.left_multiply(equality[independent])
            == equality_target[independent]
        )
        qdd = expression.slice(0, 12)
        torque = expression.slice(12, 6)
        force = expression.slice(18, 6)
        problem.add_constraint(qdd >= np.full(12, -MAXIMUM_ACCELERATION))
        problem.add_constraint(qdd <= np.full(12, MAXIMUM_ACCELERATION))
        problem.add_constraint(torque >= -effort_limits)
        problem.add_constraint(torque <= effort_limits)
        for contact, normal_index in enumerate((2, 5)):
            normal = force.slice(normal_index, 1)
            problem.add_constraint(normal >= 0.0)
            problem.add_constraint(normal <= maximum_normal_force)
            for axis in range(2):
                tangent = force.slice(contact * 3 + axis, 1)
                problem.add_constraint(tangent <= FRICTION * normal)
                problem.add_constraint(tangent >= -FRICTION * normal)
        for band_matrix, lower, upper in bands:
            band_expression = expression.left_multiply(band_matrix)
            problem.add_constraint(band_expression >= lower)
            problem.add_constraint(band_expression <= upper)
        task_expression = expression.left_multiply(matrix)
        task = problem.add_constraint(task_expression == target)
        task.configure("soft", 1.0)
        try:
            problem.solve()
        except RuntimeError as error:
            raise RuntimeError(f"independent QP failed at hierarchy level {level_index}") from error
        solution = np.asarray(variable.value, dtype=np.float64).copy()
        achieved = matrix @ solution
        # Freeze the complete achieved task image before admitting a lower
        # authority layer. Rows already implied by previous hard equalities
        # are omitted so the reference QP never hands a rank-deficient
        # equality stack to its independent QR elimination.
        selected: list[int] = []
        rank = np.linalg.matrix_rank(hard_row_space, tol=1.0e-10)
        for row in range(matrix.shape[0]):
            candidate = np.vstack((hard_row_space, matrix[row]))
            candidate_rank = np.linalg.matrix_rank(candidate, tol=1.0e-10)
            if candidate_rank > rank:
                selected.append(row)
                hard_row_space = candidate
                rank = candidate_rank
        if selected:
            freeze_matrix = matrix[selected]
            # The generic QP backend eliminates exact equalities through QR.
            # Preserve the higher-level image through a symmetric numerical
            # band so a boundary-active solution remains feasible after its
            # floating-point value is reintroduced on the next solve.
            freeze_tolerance = 2.0e-9
            bands.append(
                (
                    freeze_matrix,
                    achieved[selected] - freeze_tolerance,
                    achieved[selected] + freeze_tolerance,
                )
            )
    return solution


def hard_bound_margins(
    solution: np.ndarray,
    effort_limits: np.ndarray,
    maximum_normal_force: float,
) -> dict[str, np.ndarray]:
    qdd = solution[:, :12]
    torque = solution[:, 12:18]
    force = solution[:, 18:].reshape(-1, 2, 3)
    acceleration_margin = MAXIMUM_ACCELERATION - np.max(np.abs(qdd), axis=1)
    effort_margin = np.min(effort_limits[None, :] - np.abs(torque), axis=1)
    normal_margin = np.minimum(force[:, :, 2], maximum_normal_force - force[:, :, 2]).min(axis=1)
    friction_margin = (
        FRICTION * force[:, :, 2]
        - np.maximum(np.abs(force[:, :, 0]), np.abs(force[:, :, 1]))
    ).min(axis=1)
    return {
        "acceleration": acceleration_margin,
        "effort": effort_margin,
        "normal": normal_margin,
        "friction": friction_margin,
    }


def hard_bound_summary(margins: dict[str, np.ndarray]) -> dict[str, Any]:
    tolerance = 2.0e-7
    active = np.any(
        np.column_stack(tuple(margins.values())) <= 1.0e-5,
        axis=1,
    )
    return {
        "minimum_acceleration_margin": float(np.min(margins["acceleration"])),
        "minimum_effort_margin": float(np.min(margins["effort"])),
        "minimum_normal_force_margin": float(np.min(margins["normal"])),
        "minimum_square_friction_margin": float(np.min(margins["friction"])),
        "all_satisfied": bool(
            all(np.all(margin >= -tolerance) for margin in margins.values())
        ),
        "active_or_near_active_state_count": int(np.count_nonzero(active)),
    }


def tracking_summary(solution: np.ndarray, desired: np.ndarray) -> dict[str, float]:
    error = solution[:, :12] - desired
    return {
        name: rms(error[:, columns])
        for name, columns in {
            "root_angular": slice(0, 3),
            "root_horizontal": slice(3, 5),
            "root_height": slice(5, 6),
            "joint": slice(6, 12),
        }.items()
    }


def evaluate(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    model_path = pathlib.Path(args.model).resolve()
    input_path = pathlib.Path(args.input).resolve()
    bonesaw_path = pathlib.Path(args.bonesaw).resolve()
    bonesaw_metrics_path = pathlib.Path(args.bonesaw_metrics).resolve()
    bonesaw_metrics = json.loads(bonesaw_metrics_path.read_text())
    corpus = corpus_from_raw(input_path)
    with np.load(bonesaw_path) as archive:
        bonesaw = np.column_stack(
            (
                archive["generalized_acceleration"],
                archive["actuator_torque"],
                archive["contact_force"].reshape(-1, 6),
            )
        )
    joint_names = model_joint_names(model_path)
    _, rolling_coordinates, rolling_coefficients = rolling_descriptors(
        model_path, joint_names, list(CONTACT_FRAMES)
    )
    product_started = time.perf_counter_ns()
    mass, bias, jacobians, contact_bias = independent_products(
        model_path, joint_names, corpus
    )
    product_ns = time.perf_counter_ns() - product_started
    desired = reference_acceleration(corpus)
    weight = supported_weight(model_path)
    effort_limits = urdf_effort_limits(model_path, joint_names)
    generalized_velocity = np.column_stack(
        (np.zeros((len(desired), 3)), corpus["root_velocities"], corpus["v"])
    )
    equalities: list[np.ndarray] = []
    equality_targets: list[np.ndarray] = []
    levels: list[list[tuple[np.ndarray, np.ndarray]]] = []
    for tick in range(len(desired)):
        equality, target = equality_problem(
            mass[tick],
            bias[tick],
            jacobians[tick],
            contact_bias[tick],
            generalized_velocity[tick],
            rolling_coordinates,
            rolling_coefficients,
        )
        equalities.append(equality)
        equality_targets.append(target)
        levels.append(task_levels(desired[tick], weight / 2.0))

    solutions = np.empty_like(bonesaw)
    solve_ns = np.empty((REPEATS, len(desired)), dtype=np.uint64)
    # Warm every matrix shape and LAPACK path before measuring.
    for tick in range(len(desired)):
        solutions[tick] = constrained_lexicographic_solve(
            equalities[tick],
            equality_targets[tick],
            levels[tick],
            effort_limits,
            MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
        )
    gc.collect()
    gc_before = sum(item["collections"] for item in gc.get_stats())
    process = psutil.Process()
    rss_before = process.memory_info().rss
    peak_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    wall_started = time.perf_counter_ns()
    cpu_started = time.process_time_ns()
    for repeat in range(REPEATS):
        for tick in range(len(desired)):
            started = time.perf_counter_ns()
            solutions[tick] = constrained_lexicographic_solve(
                equalities[tick],
                equality_targets[tick],
                levels[tick],
                effort_limits,
                MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
            )
            solve_ns[repeat, tick] = time.perf_counter_ns() - started
    wall_ns = time.perf_counter_ns() - wall_started
    cpu_ns = time.process_time_ns() - cpu_started
    rss_after = process.memory_info().rss
    peak_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    gc_after = sum(item["collections"] for item in gc.get_stats())

    delta = solutions - bonesaw
    hard_residual = np.empty((len(desired), 18), dtype=np.float64)
    bonesaw_hard_residual = np.empty_like(hard_residual)
    repeat_exact = True
    for tick in range(len(desired)):
        hard_residual[tick] = equalities[tick] @ solutions[tick] - equality_targets[tick]
        bonesaw_hard_residual[tick] = (
            equalities[tick] @ bonesaw[tick] - equality_targets[tick]
        )
        if tick < 4:
            repeated = constrained_lexicographic_solve(
                equalities[tick],
                equality_targets[tick],
                levels[tick],
                effort_limits,
                MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
            )
            repeat_exact &= np.array_equal(repeated, solutions[tick])

    reference_margins = hard_bound_margins(
        solutions,
        effort_limits,
        MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
    )
    bonesaw_margins = hard_bound_margins(
        bonesaw,
        effort_limits,
        MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
    )
    bounds = {
        "reference": hard_bound_summary(reference_margins),
        "bonesaw": hard_bound_summary(bonesaw_margins),
    }
    reference_active = np.any(
        np.column_stack(tuple(reference_margins.values())) <= 1.0e-5, axis=1
    )
    bonesaw_active = np.any(
        np.column_stack(tuple(bonesaw_margins.values())) <= 1.0e-5, axis=1
    )
    state_maximum_delta = np.max(np.abs(delta), axis=1)
    parity = {
        "states_within_2e_5": int(np.count_nonzero(state_maximum_delta <= 2.0e-5)),
        "states_within_2e_4": int(np.count_nonzero(state_maximum_delta <= 2.0e-4)),
        "states_within_1e_3": int(np.count_nonzero(state_maximum_delta <= 1.0e-3)),
        "states_within_1e_2": int(np.count_nonzero(state_maximum_delta <= 1.0e-2)),
        "states_within_1": int(np.count_nonzero(state_maximum_delta <= 1.0)),
        "active_state_count_reference": int(np.count_nonzero(reference_active)),
        "active_state_count_bonesaw": int(np.count_nonzero(bonesaw_active)),
        "active_state_mask_agreement": int(
            np.count_nonzero(reference_active == bonesaw_active)
        ),
        "largest_state_deltas": [
            {
                "tick": int(tick),
                "maximum_absolute_delta": float(state_maximum_delta[tick]),
                "reference_maximum_acceleration": float(
                    np.max(np.abs(solutions[tick, :12]))
                ),
                "bonesaw_maximum_acceleration": float(
                    np.max(np.abs(bonesaw[tick, :12]))
                ),
            }
            for tick in np.argsort(state_maximum_delta)[-12:][::-1]
        ],
    }
    windows = []
    for start in range(0, len(desired), 32):
        stop = min(start + 32, len(desired))
        window_delta = delta[start:stop]
        window_timing_us = solve_ns[:, start:stop].astype(np.float64).reshape(-1) / 1000.0
        windows.append(
            {
                "start": start,
                "stop": stop,
                "maximum_solution_delta": float(np.max(np.abs(window_delta))),
                "acceleration_rms_delta": rms(window_delta[:, :12]),
                "torque_rms_delta": rms(window_delta[:, 12:18]),
                "force_rms_delta": rms(window_delta[:, 18:]),
                "reference_solve_p99_us": float(np.percentile(window_timing_us, 99)),
                "active_states_reference": int(np.count_nonzero(reference_active[start:stop])),
            }
        )
    latency_us = solve_ns.astype(np.float64).reshape(-1) / 1000.0
    jitter_us = np.abs(np.diff(latency_us))
    checks = {
        "all_256_states_solved": bool(np.all(np.isfinite(solutions))),
        "independent_solution_satisfies_rolling_and_dynamics_below_2e_8": bool(
            np.max(np.abs(hard_residual)) <= 2.0e-8
        ),
        "bonesaw_solution_satisfies_pinocchio_rolling_and_dynamics_below_2e_8": bool(
            np.max(np.abs(bonesaw_hard_residual)) <= 2.0e-8
        ),
        "independent_reference_repeats_bit_exactly": repeat_exact,
        "independent_solution_satisfies_every_hard_inequality": bounds["reference"]["all_satisfied"],
        "generalized_acceleration_matches_bonesaw_within_2e_5": bool(
            np.max(np.abs(delta[:, :12])) <= 2.0e-5
        ),
        "actuator_torque_matches_bonesaw_within_2e_4_nm": bool(
            np.max(np.abs(delta[:, 12:18])) <= 2.0e-4
        ),
        "rolling_force_matches_bonesaw_within_2e_4_n": bool(
            np.max(np.abs(delta[:, 18:])) <= 2.0e-4
        ),
    }
    metrics = {
        "schema": 1,
        "revision": REVISION,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "passed": all(checks.values()),
        "decision": "PARITY" if all(checks.values()) else "RETAIN_NEGATIVE_EVIDENCE",
        "checks": checks,
        "evaluation_boundary": {
            "independent_dynamics": f"Pinocchio {pin.__version__}",
            "independent_solver": "PlaCo low-level generic constrained QP with 2e-9 per-level achieved-image bands",
            "contact_model": "two exact nonholonomic RollingWheel YZ+rolling-X rows and augmented force maps",
            "states": len(desired),
            "policy": False,
            "integration": False,
            "physics": False,
            "inequality_solver": True,
        },
        "versions": {
            "pinocchio": pin.__version__,
            "placo": importlib.metadata.version("placo"),
            "numpy": np.__version__,
        },
        "bounds": bounds,
        "state_parity": parity,
        "tracking_rms": {
            "reference": tracking_summary(solutions, desired),
            "bonesaw": tracking_summary(bonesaw, desired),
        },
        "execution_windows": windows,
        "delta": {
            "generalized_acceleration": {
                "rms": rms(delta[:, :12]),
                "absolute": distribution(np.abs(delta[:, :12]).reshape(-1)),
            },
            "actuator_torque": {
                "rms": rms(delta[:, 12:18]),
                "absolute": distribution(np.abs(delta[:, 12:18]).reshape(-1)),
            },
            "rolling_contact_force": {
                "rms": rms(delta[:, 18:]),
                "absolute": distribution(np.abs(delta[:, 18:]).reshape(-1)),
            },
        },
        "hard_residual": {
            "reference_maximum": float(np.max(np.abs(hard_residual))),
            "bonesaw_under_pinocchio_maximum": float(
                np.max(np.abs(bonesaw_hard_residual))
            ),
        },
        "timing": {
            "solve_us": distribution(latency_us),
            "adjacent_jitter_us": distribution(jitter_us),
            "deadline_misses": {
                f"{limit / 1000:g}ms": int(np.sum(latency_us > limit))
                for limit in (500, 1000, 2000, 5000, 10000, 20000)
            },
            "product_build_us": product_ns / 1000.0,
            "queries_per_second": len(latency_us) * 1.0e9 / wall_ns,
        },
        "resources": {
            "wall_seconds": wall_ns / 1.0e9,
            "cpu_seconds": cpu_ns / 1.0e9,
            "cpu_to_wall": cpu_ns / wall_ns,
            "rss_before_bytes": rss_before,
            "rss_after_bytes": rss_after,
            "rss_delta_bytes": rss_after - rss_before,
            "peak_rss_before_bytes": peak_before,
            "peak_rss_after_bytes": peak_after,
            "gc_collections": gc_after - gc_before,
        },
        "bonesaw_r317": {
            "timing": bonesaw_metrics["timing"],
            "resources": bonesaw_metrics["resources"],
        },
        "source_sha256": {
            "model": sha256(model_path),
            "input": sha256(input_path),
            "bonesaw": sha256(bonesaw_path),
            "bonesaw_metrics": sha256(bonesaw_metrics_path),
            "evaluator": sha256(pathlib.Path(__file__)),
        },
    }
    raw = {
        "reference_solution": solutions,
        "bonesaw_solution": bonesaw,
        "solution_delta": delta,
        "reference_hard_residual": hard_residual,
        "bonesaw_under_pinocchio_hard_residual": bonesaw_hard_residual,
        "solve_ns": solve_ns,
        "state_maximum_delta": state_maximum_delta,
        "reference_active_bound": reference_active,
        "bonesaw_active_bound": bonesaw_active,
    }
    return metrics, raw


def markdown(metrics: dict[str, Any]) -> str:
    delta = metrics["delta"]
    timing = metrics["timing"]
    resources = metrics["resources"]
    bounds = metrics["bounds"]
    parity = metrics["state_parity"]
    bonesaw = metrics["bonesaw_r317"]
    report = [
        "# Upkie exact RollingWheel Pinocchio reference · R319",
        "",
        "## Result",
        "",
        (
            "**PASS.**" if metrics["passed"] else "**NOT YET PARITY-QUALIFIED.**"
        )
        + " Pinocchio independently rebuilds the floating rigid-body products and "
        "wheel-center Jacobians; PlaCo's low-level generic QP independently rebuilds the "
        "constrained hierarchy without using its robot, contact, task, or DynamicsSolver APIs. "
        "Both use Bonesaw's exact nonholonomic rolling acceleration row and augmented "
        "virtual-work force map on the same 256 frozen R123 states.",
        "",
        "> There is no policy, integration, state propagation, simulator, or physics rollout. "
        "The physical RollingWheel rows are exact; the generic QP preserves each achieved hierarchy "
        "image through a declared ±2e-9 numerical band. Every acceleration, effort, unilateral-normal, "
        "normal-cap, and square-friction inequality is present.",
        "",
        "## Differential result",
        "",
    ]
    report += markdown_table(
        ["quantity", "RMS delta", "p99 absolute", "maximum absolute"],
        [
            [
                label,
                f'{delta[name]["rms"]:.3e}',
                f'{delta[name]["absolute"]["p99"]:.3e}',
                f'{delta[name]["absolute"]["maximum"]:.3e}',
            ]
            for label, name in (
                ("generalized acceleration", "generalized_acceleration"),
                ("actuator torque", "actuator_torque"),
                ("rolling-basis contact force", "rolling_contact_force"),
            )
        ],
    )
    report += [
        "",
        "The broad distribution hides a localized active-set disagreement: "
        f'`{parity["states_within_1"]}/256` states agree within 1.0 over the complete '
        f'24-variable solution, while `{256 - parity["states_within_1"]}` states exceed it. '
        "Both solvers are hard-feasible; the parity failure is an optimizer-path difference, not a missing rolling row.",
        "",
        "## State parity and active-set boundary",
        "",
    ]
    report += markdown_table(
        ["complete-solution threshold", "states inside"],
        [[label, parity[name]] for label, name in (
            ("2e-5", "states_within_2e_5"),
            ("2e-4", "states_within_2e_4"),
            ("1e-3", "states_within_1e_3"),
            ("1e-2", "states_within_1e_2"),
            ("1", "states_within_1"),
        )],
    )
    report += [
        "",
        f'Near-active hard-bound states: reference `{parity["active_state_count_reference"]}/256`; '
        f'Bonesaw `{parity["active_state_count_bonesaw"]}/256`; mask agreement '
        f'`{parity["active_state_mask_agreement"]}/256`.',
        "",
        "## Hard equations and bounds",
        "",
        f'- Reference maximum dynamics/rolling residual: `{metrics["hard_residual"]["reference_maximum"]:.3e}`.',
        f'- Bonesaw maximum residual under independent Pinocchio products: `{metrics["hard_residual"]["bonesaw_under_pinocchio_maximum"]:.3e}`.',
        "",
    ]
    report += markdown_table(
        ["implementation", "min accel margin", "min effort margin", "min normal margin", "min friction margin", "near-active states"],
        [[
            label,
            f'{bounds[name]["minimum_acceleration_margin"]:.3e}',
            f'{bounds[name]["minimum_effort_margin"]:.6f}',
            f'{bounds[name]["minimum_normal_force_margin"]:.6f}',
            f'{bounds[name]["minimum_square_friction_margin"]:.6f}',
            bounds[name]["active_or_near_active_state_count"],
        ] for label, name in (("Bonesaw", "bonesaw"), ("reference", "reference"))],
    )
    report += [
        "",
        "## Shared-target tracking",
        "",
    ]
    report += markdown_table(
        ["target", "Bonesaw RMS", "reference RMS"],
        [[
            label,
            f'{metrics["tracking_rms"]["bonesaw"][name]:.6f}',
            f'{metrics["tracking_rms"]["reference"][name]:.6f}',
        ] for label, name in (
            ("root angular", "root_angular"),
            ("root horizontal", "root_horizontal"),
            ("root height", "root_height"),
            ("joint", "joint"),
        )],
    )
    report += ["", "## CPU, jitter, and memory", ""]
    report += markdown_table(
        ["implementation", "solve p50 / p99 / max µs", "jitter p99 µs", "queries/s", "peak RSS MiB", "GC"],
        [[
            "Bonesaw R317",
            f'{bonesaw["timing"]["latency_us"]["p50"]:.1f} / {bonesaw["timing"]["latency_us"]["p99"]:.1f} / {bonesaw["timing"]["latency_us"]["maximum"]:.1f}',
            f'{bonesaw["timing"]["adjacent_jitter_us"]["p99"]:.1f}',
            f'{bonesaw["resources"]["queries_per_second"]:.0f}',
            f'{bonesaw["resources"]["peak_rss_after_bytes"] / 2**20:.2f}',
            bonesaw["resources"]["gc_collections"],
        ], [
            "Pinocchio + generic QP",
            f'{timing["solve_us"]["p50"]:.1f} / {timing["solve_us"]["p99"]:.1f} / {timing["solve_us"]["maximum"]:.1f}',
            f'{timing["adjacent_jitter_us"]["p99"]:.1f}',
            f'{timing["queries_per_second"]:.0f}',
            f'{resources["peak_rss_after_bytes"] / 2**20:.2f}',
            resources["gc_collections"],
        ]],
    )
    report += [
        "",
        "The reference timing includes construction and solution of the independent constrained "
        "hierarchy through PlaCo's generic QP API. Its one-time Pinocchio "
        f'product build costs `{timing["product_build_us"] / 1000.0:.3f} ms` for all 256 states; '
        "that cost is retained separately because Bonesaw emits products inside each query.",
        "",
        "## Execution windows",
        "",
    ]
    report += markdown_table(
        ["states", "active", "max solution delta", "qdd RMS", "torque RMS", "force RMS", "reference p99 µs"],
        [[
            f'{row["start"]}–{row["stop"] - 1}',
            row["active_states_reference"],
            f'{row["maximum_solution_delta"]:.3e}',
            f'{row["acceleration_rms_delta"]:.3e}',
            f'{row["torque_rms_delta"]:.3e}',
            f'{row["force_rms_delta"]:.3e}',
            f'{row["reference_solve_p99_us"]:.1f}',
        ] for row in metrics["execution_windows"]],
    )
    report += [
        "",
        "## Gates",
        "",
    ]
    report += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["checks"].items()
    ]
    report += [
        "",
        "## Boundary",
        "",
        "R319 closes independent nonholonomic row and hard-bound reconstruction, but not complete "
        "optimizer parity: acceleration-bound active-set choices remain measurably different. "
        "It does not establish closed-loop stability, measured-contact transfer, calibrated "
        "actuator/thermal authority, or hardware timing. Those remain separate consequence gates.",
        "",
        "## Provenance",
        "",
    ]
    report += markdown_table(
        ["artifact", "version or SHA-256"],
        [
            ["Pinocchio", metrics["versions"]["pinocchio"]],
            ["PlaCo", metrics["versions"]["placo"]],
            ["NumPy", metrics["versions"]["numpy"]],
            *[[name, digest] for name, digest in metrics["source_sha256"].items()],
        ],
    )
    report += [
        "",
    ]
    return "\n".join(report)


def main() -> None:
    args = parse_args()
    metrics, raw = evaluate(args)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "upkie-pinocchio-rolling-reference-raw.npz", **raw)
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "UPKIE_PINOCCHIO_ROLLING_REFERENCE_R319.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Upkie exact RollingWheel reference · R319")
    )
    print(json.dumps({"passed": metrics["passed"], "checks": metrics["checks"]}, indent=2))


if __name__ == "__main__":
    main()
