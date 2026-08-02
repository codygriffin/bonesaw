#!/usr/bin/env python3
"""Long policy-/physics-free Upkie contact-mode transition replay.

The schedule is caller-authored evaluation data, not a contact estimator or
controller. Every row is solved independently. Rust owns fixed-shape mode
selection, row emission, and WBC; Python owns the corpus, Pinocchio oracle,
fault probes, replay invariance, statistics, and report.
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
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_dynamics_mirror_report import pinocchio_products
from cuda_point_query_mirror_report import pin_point_products
from upkie_state_local_wbc_report import (
    ROLLING_GAIN,
    ROLLING_MAXIMUM_CORRECTION_MPS2,
    SOLVED,
    allocate_outputs,
    new_session,
    pin_contact_products,
    rolling_descriptors,
    root_pose_matrix,
    rss_bytes,
    run_case,
    semantic_equal,
    sha256,
    standing_posture,
    status_counts,
    urdf_effort_limits,
)


MODE_NAMES = {0: "LockedPoint", 1: "NormalPoint", 2: "RollingPoint", 3: "RollingWheel"}
SCENARIOS = (
    "double_rolling",
    "left_rolling",
    "right_rolling",
    "double_normal",
    "double_free_rolling",
    "double_locked",
    "airborne",
    "left_roll_right_normal",
    "left_normal_right_roll",
    "double_rolling_slip",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=20_000)
    parser.add_argument("--seed", type=lambda value: int(value, 0), default=0xB0E5A7)
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--output", default="benchmarks/results/upkie-contact-transition-r124"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_TRANSITION_R124.html"
    )
    return parser.parse_args()


def schedule(samples: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    scenario = np.empty(samples, np.uint8)
    cursor = 0
    cycle = 0
    while cursor < samples:
        scenario_id = cycle % len(SCENARIOS)
        length = int(rng.integers(31, 98))
        stop = min(samples, cursor + length)
        scenario[cursor:stop] = scenario_id
        cursor = stop
        cycle += 1
    active = np.ones((samples, 2), np.uint8)
    modes = np.full((samples, 2), 3, np.uint8)
    for tick, item in enumerate(scenario):
        if item == 1:
            active[tick] = (1, 0)
        elif item == 2:
            active[tick] = (0, 1)
        elif item == 3:
            modes[tick] = (1, 1)
        elif item == 4:
            modes[tick] = (2, 2)
        elif item == 5:
            modes[tick] = (0, 0)
        elif item == 6:
            active[tick] = (0, 0)
        elif item == 7:
            modes[tick] = (3, 1)
        elif item == 8:
            modes[tick] = (1, 3)
    return scenario, active, modes


def make_transition_corpus(
    model_path: pathlib.Path,
    samples: int,
    seed: int,
    joint_names: list[str],
    frame_names: list[str],
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    scenario, active, modes = schedule(samples, rng)
    phase = np.arange(samples, dtype=np.float64)
    q = np.tile(standing_posture(joint_names), (samples, 1))
    q[:, joint_names.index("left_hip")] += (
        0.10 * np.sin(phase * 0.011) + rng.uniform(-0.025, 0.025, samples)
    )
    q[:, joint_names.index("right_hip")] -= (
        0.10 * np.sin(phase * 0.011) + rng.uniform(-0.025, 0.025, samples)
    )
    q[:, joint_names.index("left_knee")] += (
        0.15 * np.cos(phase * 0.007) + rng.uniform(-0.035, 0.035, samples)
    )
    q[:, joint_names.index("right_knee")] -= (
        0.15 * np.cos(phase * 0.007) + rng.uniform(-0.035, 0.035, samples)
    )
    q[:, coordinates[0]] = rng.uniform(-math.pi, math.pi, samples)
    q[:, coordinates[1]] = rng.uniform(-math.pi, math.pi, samples)

    root_positions = np.column_stack(
        (
            0.10 * np.sin(phase * 0.0031),
            0.03 * np.sin(phase * 0.0047),
            0.56 + 0.025 * np.cos(phase * 0.0037),
        )
    )
    root_velocities = np.zeros((samples, 3), np.float64)
    root_velocities[:, 0] = 0.65 * np.sin(phase * 0.009) + rng.uniform(-0.05, 0.05, samples)
    root_velocities[:, 1] = 0.08 * np.sin(phase * 0.005)
    root_accelerations = np.zeros((samples, 3), np.float64)
    root_accelerations[:, 0] = 1.25 * np.cos(phase * 0.008)
    root_accelerations[:, 1] = 0.18 * np.sin(phase * 0.006)
    root_accelerations[:, 2] = 0.20 * np.sin(phase * 0.004)

    v = np.zeros_like(q)
    joint_accelerations = np.zeros_like(q)
    for name in ("left_hip", "left_knee", "right_hip", "right_knee"):
        coordinate = joint_names.index(name)
        joint_accelerations[:, coordinate] = rng.uniform(-1.0, 1.0, samples)

    roots = root_pose_matrix(root_positions)
    _, jacobians = pin_point_products(
        model_path,
        joint_names,
        frame_names,
        np.zeros((2, 3), np.float64),
        q,
        roots,
    )
    slip = np.zeros((samples, 2), np.float64)
    slip_rows = scenario == 9
    slip[slip_rows, 0] = 0.65 * np.sin(phase[slip_rows] * 0.17)
    slip[slip_rows, 1] = -0.65 * np.cos(phase[slip_rows] * 0.13)

    for tick in range(samples):
        rolling_targets = [
            target
            for target in range(2)
            if active[tick, target] and modes[tick, target] == 3
        ]
        if not rolling_targets:
            continue
        matrix = np.empty((len(rolling_targets), len(rolling_targets)), np.float64)
        rhs = np.empty(len(rolling_targets), np.float64)
        generalized_velocity = np.zeros(6 + len(joint_names), np.float64)
        generalized_velocity[3:6] = root_velocities[tick]
        for row, target in enumerate(rolling_targets):
            jacobian = jacobians[tick, target, 0]
            rhs[row] = slip[tick, target] - jacobian @ generalized_velocity
            for column, wheel_target in enumerate(rolling_targets):
                coordinate = coordinates[wheel_target]
                matrix[row, column] = jacobian[6 + coordinate]
                if wheel_target == target:
                    matrix[row, column] += coefficients[target]
        solved = np.linalg.solve(matrix, rhs)
        for target, value in zip(rolling_targets, solved, strict=True):
            v[tick, coordinates[target]] = value

    provisional = {
        "root_positions": root_positions,
        "root_velocities": root_velocities,
        "root_accelerations": root_accelerations,
        "q": q,
        "v": v,
        "joint_accelerations": joint_accelerations,
        "contact_active": active,
    }
    _, contact_bias = pin_contact_products(model_path, joint_names, frame_names, provisional)
    for tick in range(samples):
        rolling_targets = [
            target
            for target in range(2)
            if active[tick, target] and modes[tick, target] == 3
        ]
        if not rolling_targets:
            continue
        matrix = np.empty((len(rolling_targets), len(rolling_targets)), np.float64)
        rhs = np.empty(len(rolling_targets), np.float64)
        desired = np.zeros(6 + len(joint_names), np.float64)
        desired[3:6] = root_accelerations[tick]
        desired[6:] = joint_accelerations[tick]
        correction = np.clip(
            -ROLLING_GAIN * slip[tick],
            -ROLLING_MAXIMUM_CORRECTION_MPS2,
            ROLLING_MAXIMUM_CORRECTION_MPS2,
        )
        for row, target in enumerate(rolling_targets):
            jacobian = jacobians[tick, target, 0]
            rhs[row] = correction[target] - contact_bias[tick, target, 0] - jacobian @ desired
            for column, wheel_target in enumerate(rolling_targets):
                coordinate = coordinates[wheel_target]
                matrix[row, column] = jacobian[6 + coordinate]
                if wheel_target == target:
                    matrix[row, column] += coefficients[target]
        solved = np.linalg.solve(matrix, rhs)
        for target, value in zip(rolling_targets, solved, strict=True):
            joint_accelerations[tick, coordinates[target]] = value

    return {
        "root_positions": np.ascontiguousarray(root_positions),
        "root_velocities": np.ascontiguousarray(root_velocities),
        "root_accelerations": np.ascontiguousarray(root_accelerations),
        "q": np.ascontiguousarray(q),
        "v": np.ascontiguousarray(v),
        "joint_accelerations": np.ascontiguousarray(joint_accelerations),
        "contact_active": np.ascontiguousarray(active),
        "contact_mode_trace": np.ascontiguousarray(modes),
        "scenario": scenario,
        "rolling_slip": slip,
    }


def run_transition_case(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
    outputs: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    return run_case(
        new_session(model_path),
        corpus,
        frame_ids,
        None,
        coordinates,
        coefficients,
        outputs,
        contact_mode_trace=corpus["contact_mode_trace"],
        target_weights=np.zeros(2, np.float64),
    )


def independent_mode_oracle(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
    corpus: dict[str, np.ndarray],
    out: dict[str, np.ndarray],
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    ticks = len(corpus["q"])
    generalized = 6 + len(joint_names)
    roots = root_pose_matrix(corpus["root_positions"])
    velocity = np.column_stack(
        (np.zeros((ticks, 3)), corpus["root_velocities"], corpus["v"])
    )
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81]), (ticks, 1))
    mass, bias, _ = pinocchio_products(
        model_path, joint_names, corpus["q"], roots, velocity, gravity
    )
    jacobians, contact_bias = pin_contact_products(
        model_path, joint_names, frame_names, corpus
    )
    dynamics = np.empty((ticks, generalized), np.float64)
    contact_linf = np.zeros(ticks, np.float64)
    rolling_velocity = np.full((ticks, 2), np.nan, np.float64)
    active_force_count = np.sum(corpus["contact_active"], axis=1).astype(np.uint8)
    minimum_normal = math.inf
    minimum_friction = math.inf
    inactive_force_linf = 0.0
    for tick in range(ticks):
        rhs = np.zeros(generalized, np.float64)
        rhs[6:] = out["actuator_torque"][tick]
        packed = 0
        for target in range(2):
            if not corpus["contact_active"][tick, target]:
                continue
            force = out["contact_force_basis"][tick, packed]
            rhs += jacobians[tick, target].T @ force
            minimum_normal = min(minimum_normal, float(force[2]))
            minimum_friction = min(
                minimum_friction,
                float(0.8 * force[2] - max(abs(force[0]), abs(force[1]))),
            )
            physical = (
                jacobians[tick, target] @ out["generalized_acceleration"][tick]
                + contact_bias[tick, target]
            )
            mode = int(corpus["contact_mode_trace"][tick, target])
            residuals: list[float]
            if mode == 0:
                residuals = physical.tolist()
            elif mode == 1:
                residuals = [float(physical[2])]
            elif mode == 2:
                residuals = [float(physical[1]), float(physical[2])]
            else:
                coordinate = coordinates[target]
                point_velocity = jacobians[tick, target] @ velocity[tick]
                rolling_velocity[tick, target] = (
                    point_velocity[0] + coefficients[target] * corpus["v"][tick, coordinate]
                )
                correction = float(
                    np.clip(
                        -ROLLING_GAIN * rolling_velocity[tick, target],
                        -ROLLING_MAXIMUM_CORRECTION_MPS2,
                        ROLLING_MAXIMUM_CORRECTION_MPS2,
                    )
                )
                rolling = (
                    physical[0]
                    + coefficients[target]
                    * out["generalized_acceleration"][tick, 6 + coordinate]
                    - correction
                )
                residuals = [float(rolling), float(physical[1]), float(physical[2])]
            contact_linf[tick] = max(contact_linf[tick], max(abs(value) for value in residuals))
            packed += 1
        if packed < 2:
            inactive_force_linf = max(
                inactive_force_linf,
                float(np.max(np.abs(out["contact_force_basis"][tick, packed:]))),
            )
        dynamics[tick] = mass[tick] @ out["generalized_acceleration"][tick] + bias[tick] - rhs
    effort_limits = urdf_effort_limits(model_path, joint_names)
    effort_margin = effort_limits[None, :] - np.abs(out["actuator_torque"])
    metrics = {
        "dynamics_linf": float(np.max(np.abs(dynamics))),
        "contact_linf": float(np.max(contact_linf)),
        "minimum_normal_force_n": minimum_normal,
        "minimum_friction_margin_n": minimum_friction,
        "minimum_effort_margin_nm": float(np.min(effort_margin)),
        "inactive_force_linf": inactive_force_linf,
        "rolling_velocity_witness_linf_mps": float(
            np.nanmax(np.abs(rolling_velocity - corpus["rolling_slip"]))
        ),
        "reported_dynamics_linf": float(np.max(out["dynamics_residual"])),
        "reported_contact_linf": float(np.max(out["contact_residual"])),
    }
    return metrics, {
        "pinocchio_dynamics_residual": dynamics,
        "pinocchio_contact_linf": contact_linf,
        "pinocchio_rolling_velocity": rolling_velocity,
        "active_force_count": active_force_count,
    }


def fingerprint(corpus: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for name in sorted(corpus):
        value = corpus[name]
        digest.update(name.encode())
        digest.update(str(value.dtype).encode())
        digest.update(np.asarray(value.shape, np.int64).tobytes())
        digest.update(value.tobytes())
    return digest.hexdigest()


def slice_corpus(corpus: dict[str, np.ndarray], selection: slice | np.ndarray) -> dict[str, np.ndarray]:
    ticks = len(corpus["q"])
    return {
        name: value[selection] if value.ndim > 0 and len(value) == ticks else value
        for name, value in corpus.items()
    }


def invariance(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
    baseline: dict[str, np.ndarray],
) -> dict[str, Any]:
    repeat = run_transition_case(model_path, corpus, frame_ids, coordinates, coefficients)
    reverse_index = np.arange(len(corpus["q"]) - 1, -1, -1)
    reversed_out = run_transition_case(
        model_path,
        slice_corpus(corpus, reverse_index),
        frame_ids,
        coordinates,
        coefficients,
    )
    inverse = np.argsort(reverse_index)
    reordered = {name: value[inverse] for name, value in reversed_out.items()}
    boundaries = np.linspace(0, len(corpus["q"]), 5, dtype=int)
    parts = []
    for start, stop in zip(boundaries[:-1], boundaries[1:], strict=True):
        parts.append(
            run_transition_case(
                model_path,
                slice_corpus(corpus, slice(start, stop)),
                frame_ids,
                coordinates,
                coefficients,
            )
        )
    chunked = {name: np.concatenate([part[name] for part in parts]) for name in baseline}
    calls = int(
        np.sum(repeat["allocation_calls"])
        + np.sum(reversed_out["allocation_calls"])
        + sum(np.sum(part["allocation_calls"]) for part in parts)
    )
    return {
        "repeat_bitwise_exact": semantic_equal(baseline, repeat),
        "reverse_order_exact": semantic_equal(baseline, reordered),
        "four_chunk_exact": semantic_equal(baseline, chunked),
        "allocation_calls": calls,
    }


def atomic_faults(
    model_path: pathlib.Path,
    corpus: dict[str, np.ndarray],
    frame_ids: np.ndarray,
    coordinates: np.ndarray,
    coefficients: np.ndarray,
) -> dict[str, Any]:
    sample = slice_corpus(corpus, slice(0, 128))

    def probe(name: str, bad: dict[str, np.ndarray], bad_coefficients=coefficients) -> tuple[str, bool, bool]:
        session = new_session(model_path)
        outputs = allocate_outputs(session, len(bad["q"]))
        for value in outputs.values():
            value.fill(77)
        rejected = False
        try:
            run_case(
                session,
                bad,
                frame_ids,
                None,
                coordinates,
                bad_coefficients,
                outputs,
                contact_mode_trace=bad["contact_mode_trace"],
                target_weights=np.zeros(2, np.float64),
            )
        except ValueError:
            rejected = True
        atomic = all(np.all(value == 77) for value in outputs.values())
        return name, rejected, atomic

    cases = []
    bad = {name: value.copy() for name, value in sample.items()}
    bad["q"][64, 0] = np.nan
    cases.append(probe("nonfinite_state", bad))
    bad = {name: value.copy() for name, value in sample.items()}
    bad["contact_mode_trace"][64, 0] = 9
    cases.append(probe("invalid_contact_mode", bad))
    bad = {name: value.copy() for name, value in sample.items()}
    bad["contact_active"][64, 0] = 2
    cases.append(probe("invalid_contact_activity", bad))
    bad = {name: value.copy() for name, value in sample.items()}
    cases.append(probe("zero_rolling_coefficient", bad, np.zeros(2, np.float64)))
    rows = [
        {"fault": name, "typed_value_error": rejected, "outputs_unchanged": atomic}
        for name, rejected, atomic in cases
    ]
    return {"rows": rows, "all_typed_and_atomic": all(row["typed_value_error"] and row["outputs_unchanged"] for row in rows)}


def scenario_metrics(
    corpus: dict[str, np.ndarray],
    out: dict[str, np.ndarray],
    oracle_raw: dict[str, np.ndarray],
) -> list[dict[str, Any]]:
    desired = np.column_stack(
        (
            np.zeros((len(corpus["q"]), 3)),
            corpus["root_accelerations"],
            corpus["joint_accelerations"],
        )
    )
    error = out["generalized_acceleration"] - desired
    rows = []
    for scenario_id, name in enumerate(SCENARIOS):
        mask = corpus["scenario"] == scenario_id
        latency = out["step_ns"][mask].astype(np.float64) / 1000.0
        rows.append(
            {
                "scenario": name,
                "queries": int(np.sum(mask)),
                "status_counts": status_counts(out["status"][mask]),
                "tracking_rms": float(np.sqrt(np.mean(error[mask] ** 2))),
                "dynamics_linf": float(np.max(np.abs(oracle_raw["pinocchio_dynamics_residual"][mask]))),
                "contact_linf": float(np.max(oracle_raw["pinocchio_contact_linf"][mask])),
                "latency_p50_us": float(np.percentile(latency, 50)),
                "latency_p99_us": float(np.percentile(latency, 99)),
                "latency_max_us": float(np.max(latency)),
                "qdd_bound_queries": int(
                    np.sum(np.any(np.abs(out["generalized_acceleration"][mask]) >= 250.0 - 1e-7, axis=1))
                ),
            }
        )
    return rows


def edge_metrics(corpus: dict[str, np.ndarray], out: dict[str, np.ndarray], raw: dict[str, np.ndarray]) -> dict[str, Any]:
    edges = np.flatnonzero(
        np.any(np.diff(corpus["contact_active"], axis=0) != 0, axis=1)
        | np.any(np.diff(corpus["contact_mode_trace"], axis=0) != 0, axis=1)
    ) + 1
    selected = np.zeros(len(corpus["q"]), bool)
    for edge in edges:
        selected[max(0, edge - 2) : min(len(selected), edge + 3)] = True
    return {
        "transition_count": int(len(edges)),
        "queries_in_plus_minus_two_window": int(np.sum(selected)),
        "status_counts": status_counts(out["status"][selected]),
        "dynamics_linf": float(np.max(np.abs(raw["pinocchio_dynamics_residual"][selected]))),
        "contact_linf": float(np.max(raw["pinocchio_contact_linf"][selected])),
        "maximum_constraint_violation": float(np.max(out["maximum_constraint_violation"][selected])),
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle = metrics["independent_pinocchio_oracle"]
    latency = metrics["timing"]["latency_us"]
    jitter = metrics["timing"]["jitter_us"]
    scenario_rows = [
        [
            row["scenario"], row["queries"], row["status_counts"], f'{row["tracking_rms"]:.3f}',
            f'{row["dynamics_linf"]:.2e}', f'{row["contact_linf"]:.2e}',
            f'{row["latency_p50_us"]:.1f} / {row["latency_p99_us"]:.1f} / {row["latency_max_us"]:.1f}',
            row["qdd_bound_queries"],
        ]
        for row in metrics["scenarios"]
    ]
    return "\n".join(
        [
            "# Bonesaw Upkie contact-transition replay · r124",
            "",
            "## Outcome",
            "",
            f'> **Admission: {"PASS" if metrics["admission"] else "FAIL"}.** '
            f'{metrics["samples"]:,} immutable Upkie states exercise every point-contact mode, '
            "single/double/no support, mixed touchdown rows, and bounded rolling slip. There is "
            "no policy, contact estimator, state integration, simulator, or physics rollout.",
            "",
            "## Boundary and corpus",
            "",
            *markdown_table(
                ["item", "value"],
                [
                    ["states", f'{metrics["samples"]:,}'],
                    ["seed", metrics["seed"]],
                    ["model SHA-256", metrics["model_sha256"]],
                    ["mode codes", metrics["mode_names"]],
                    ["contact transitions", metrics["edges"]["transition_count"]],
                    ["queries touching ±250 qdd cap", f'{metrics["qdd_bound_queries"]:,} / {metrics["samples"]:,} ({100 * metrics["qdd_bound_fraction"]:.2f}%)'],
                    ["input fingerprint preserved", metrics["input_immutable"]],
                    ["policy / estimator / integration / physics", "none / none / none / none"],
                ],
            ),
            "",
            "`contact_mode_trace[tick,target]` selects LockedPoint, NormalPoint, RollingPoint, or "
            "RollingWheel before the preallocated Rust tick. Contact activity remains a separate "
            "mask. Descriptors are validated for the entire call before any output is written.",
            "",
            "## Independent Pinocchio hard equations",
            "",
            *markdown_table(
                ["witness", "worst observed", "gate"],
                [
                    ["floating dynamics L∞", f'{oracle["dynamics_linf"]:.3e}', "≤ 2e-6"],
                    ["mode-specific contact L∞", f'{oracle["contact_linf"]:.3e}', "≤ 2e-6"],
                    ["rolling velocity witness L∞", f'{oracle["rolling_velocity_witness_linf_mps"]:.3e} m/s', "≤ 2e-10"],
                    ["minimum normal force", f'{oracle["minimum_normal_force_n"]:.6f} N', "≥ -1e-8"],
                    ["minimum friction margin", f'{oracle["minimum_friction_margin_n"]:.3e} N', "≥ -2e-7"],
                    ["minimum effort margin", f'{oracle["minimum_effort_margin_nm"]:.3e} Nm', "≥ -2e-7"],
                    ["inactive packed-force tail", f'{oracle["inactive_force_linf"]:.3e}', "exact zero"],
                ],
            ),
            "",
            "For each active target the oracle selects only the rows declared by its mode: XYZ "
            "for locked, Z for normal, YZ for free rolling, and wheel-coupled X plus YZ for "
            "RollingWheel. Contact forces are independently packed in active-target order before "
            "reconstructing M·qdd+h−Sᵀτ−Jᵀf.",
            "",
            "## Behavior by contact regime",
            "",
            *markdown_table(
                ["regime", "queries", "statuses", "tracking RMS", "dyn L∞", "contact L∞", "p50 / p99 / max µs", "qdd cap"],
                scenario_rows,
            ),
            "",
            "Tracking remains a continuous lower-layer witness. Exact hard equations do not "
            "claim that the requested posture or acceleration is realizable.",
            "",
            "## Transition-edge windows",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["edges"].items()]),
            "",
            "Each ±2-row edge window is checked independently; there is intentionally no contact "
            "state carried across the edge.",
            "",
            "## Long-run CPU, memory, jitter, and work",
            "",
            *markdown_table(
                ["measurement", "result"],
                [
                    ["latency mean / std / MAD", f'{latency["mean"]:.1f} / {latency["stddev"]:.1f} / {latency["mad"]:.1f} µs'],
                    ["latency p50 / p95 / p99 / p99.9 / max", f'{latency["p50"]:.1f} / {latency["p95"]:.1f} / {latency["p99"]:.1f} / {latency["p99_9"]:.1f} / {latency["maximum"]:.1f} µs'],
                    ["adjacent jitter p50 / p95 / p99 / max", f'{jitter["p50"]:.1f} / {jitter["p95"]:.1f} / {jitter["p99"]:.1f} / {jitter["maximum"]:.1f} µs'],
                    ["deadline misses >0.5 / 1 / 2 / 5 ms", " / ".join(str(metrics["timing"]["deadline_misses"][key]) for key in ("0.5ms", "1ms", "2ms", "5ms"))],
                    ["whole call wall / CPU", f'{metrics["runtime"]["wall_seconds"]:.3f} / {metrics["runtime"]["cpu_seconds"]:.3f} s'],
                    ["throughput", f'{metrics["runtime"]["queries_per_second"]:.1f} queries/s'],
                    ["RSS before / after / delta", f'{metrics["runtime"]["rss_before_bytes"] / 1048576:.2f} / {metrics["runtime"]["rss_after_bytes"] / 1048576:.2f} / {metrics["runtime"]["rss_delta_bytes"] / 1048576:.2f} MiB'],
                    ["hot-loop allocation sentinel", f'{metrics["runtime"]["allocation_calls"]} calls / {metrics["runtime"]["allocated_bytes"]} bytes'],
                    ["GC collections", metrics["runtime"]["gc_collections"]],
                ],
            ),
            "",
            "## Replay invariance",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]),
            "",
            "## Invalid-input fault probes",
            "",
            *markdown_table(
                ["fault", "typed ValueError", "all outputs unchanged"],
                [[row["fault"], row["typed_value_error"], row["outputs_unchanged"]] for row in metrics["faults"]["rows"]],
            ),
            "",
            "Nonfinite state, invalid mode/activity, and degenerate rolling coefficients are "
            "rejected atomically before the first measured tick. Runtime infeasibility remains a "
            "different typed solver status; schema faults are not mislabeled as physics.",
            "",
            "## Remaining boundary",
            "",
            "This report admits row composition and bounded state-local execution across contact "
            "regimes. It does not choose those regimes, estimate touchdown, model impact impulses, "
            "or prove hybrid closed-loop stability. The next CPU tranche is a long adversarial "
            "resource/observation replay over this same mode boundary, followed by a separately "
            "versioned contact estimator or external simulator. CUDA batching remains downstream "
            "of this exact CPU reference.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.samples < 1_000:
        raise ValueError("long transition replay requires at least 1,000 states")
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    session = new_session(model_path)
    joint_names = list(session.joint_names)
    all_frames = list(session.frame_names)
    contact_frames = ["left_wheel_center", "right_wheel_center"]
    frame_ids = np.asarray([all_frames.index(name) for name in contact_frames], np.int64)
    _, coordinates, coefficients = rolling_descriptors(
        model_path, joint_names, contact_frames
    )
    corpus = make_transition_corpus(
        model_path,
        args.samples,
        args.seed,
        joint_names,
        contact_frames,
        coordinates,
        coefficients,
    )
    input_before = fingerprint(corpus)
    buffers = allocate_outputs(session, args.samples)
    run_case(
        session,
        corpus,
        frame_ids,
        None,
        coordinates,
        coefficients,
        buffers,
        contact_mode_trace=corpus["contact_mode_trace"],
        target_weights=np.zeros(2, np.float64),
    )
    gc.collect()
    gc_before = [entry["collections"] for entry in gc.get_stats()]
    rss_before = rss_bytes()
    wall_before = time.perf_counter_ns()
    cpu_before = time.process_time_ns()
    run_case(
        session,
        corpus,
        frame_ids,
        None,
        coordinates,
        coefficients,
        buffers,
        contact_mode_trace=corpus["contact_mode_trace"],
        target_weights=np.zeros(2, np.float64),
    )
    cpu_after = time.process_time_ns()
    wall_after = time.perf_counter_ns()
    rss_after = rss_bytes()
    gc_after = [entry["collections"] for entry in gc.get_stats()]
    baseline = {name: value.copy() for name, value in buffers.items()}
    input_after = fingerprint(corpus)
    oracle, oracle_raw = independent_mode_oracle(
        model_path,
        joint_names,
        contact_frames,
        corpus,
        baseline,
        coordinates,
        coefficients,
    )
    replay = invariance(
        model_path, corpus, frame_ids, coordinates, coefficients, baseline
    )
    faults = atomic_faults(model_path, corpus, frame_ids, coordinates, coefficients)
    step_us = baseline["step_ns"].astype(np.float64) / 1000.0
    jitter_us = np.abs(np.diff(step_us))
    timing = {
        "latency_us": distribution(step_us),
        "jitter_us": distribution(jitter_us),
        "deadline_misses": {
            f"{limit / 1000:g}ms": int(np.sum(step_us > limit))
            for limit in (500, 1000, 2000, 5000)
        },
    }
    wall_seconds = (wall_after - wall_before) / 1e9
    cpu_seconds = (cpu_after - cpu_before) / 1e9
    runtime = {
        "wall_seconds": wall_seconds,
        "cpu_seconds": cpu_seconds,
        "cpu_to_wall": cpu_seconds / wall_seconds,
        "queries_per_second": args.samples / wall_seconds,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "allocation_calls": int(np.sum(baseline["allocation_calls"])),
        "allocated_bytes": int(np.sum(baseline["allocated_bytes"])),
        "gc_collections": int(
            sum(after - before for before, after in zip(gc_before, gc_after, strict=True))
        ),
    }
    edges = edge_metrics(corpus, baseline, oracle_raw)
    admission = bool(
        np.all(np.isin(baseline["status"], SOLVED))
        and oracle["dynamics_linf"] <= 2e-6
        and oracle["contact_linf"] <= 2e-6
        and oracle["rolling_velocity_witness_linf_mps"] <= 2e-10
        and oracle["minimum_normal_force_n"] >= -1e-8
        and oracle["minimum_friction_margin_n"] >= -2e-7
        and oracle["minimum_effort_margin_nm"] >= -2e-7
        and oracle["inactive_force_linf"] == 0.0
        and runtime["allocation_calls"] == 0
        and runtime["allocated_bytes"] == 0
        and replay["repeat_bitwise_exact"]
        and replay["reverse_order_exact"]
        and replay["four_chunk_exact"]
        and faults["all_typed_and_atomic"]
        and input_before == input_after
    )
    metrics = {
        "schema_version": 1,
        "revision": "upkie-contact-transition-r124",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": args.samples,
        "seed": hex(args.seed),
        "policy": None,
        "contact_estimator": None,
        "physics_rollout": None,
        "mode_names": MODE_NAMES,
        "contact_frames": contact_frames,
        "rolling_coordinates": coordinates.tolist(),
        "rolling_velocity_coefficients": coefficients.tolist(),
        "input_fingerprint": input_before,
        "input_immutable": input_before == input_after,
        "status_counts": status_counts(baseline["status"]),
        "qdd_bound_queries": int(
            np.sum(
                np.any(
                    np.abs(baseline["generalized_acceleration"]) >= 250.0 - 1e-7,
                    axis=1,
                )
            )
        ),
        "qdd_bound_fraction": float(
            np.mean(
                np.any(
                    np.abs(baseline["generalized_acceleration"]) >= 250.0 - 1e-7,
                    axis=1,
                )
            )
        ),
        "independent_pinocchio_oracle": oracle,
        "scenarios": scenario_metrics(corpus, baseline, oracle_raw),
        "edges": edges,
        "timing": timing,
        "runtime": runtime,
        "invariance": replay,
        "faults": faults,
        "admission": admission,
    }
    raw = {
        **{f"input_{name}": value for name, value in corpus.items()},
        **{f"output_{name}": value for name, value in baseline.items()},
        **oracle_raw,
    }
    np.savez_compressed(output / "upkie-contact-transition-raw.npz", **raw)
    (output / "upkie-contact-transition-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "UPKIE_CONTACT_TRANSITION_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw Upkie contact-transition replay · r124")
    )
    print(json.dumps(metrics, indent=2))
    return 0 if admission else 1


if __name__ == "__main__":
    raise SystemExit(main())
