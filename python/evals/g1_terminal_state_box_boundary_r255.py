#!/usr/bin/env python3
"""R255 policy-/physics-free complete terminal-state box mechanism audit."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture


REVISION = "g1-terminal-state-box-boundary-r255"
ROWS = 64
POINTS_PER_BOX = 32
REPEAT_CALLS = 500


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_TERMINAL_STATE_BOX_BOUNDARY_R255.html"
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit("R255 requires the pinned G1 model")
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    joint_names = list(session.joint_names())
    joints = int(session.joint_dof())
    q_nominal = standing_posture(joint_names)
    position_lower, position_upper, velocity_limit = joint_limits(model, joint_names)

    row = np.arange(ROWS, dtype=np.float64)
    coordinate = np.arange(joints, dtype=np.float64)
    phase = 0.31 * row[:, None] + 0.19 * coordinate[None, :]
    root_center = np.column_stack(
        (
            0.16 + 0.04 * np.sin(0.17 * row),
            -0.55 + 0.30 * np.cos(0.23 * row),
            0.10 * np.sin(0.13 * row),
            0.12 * np.cos(0.11 * row),
            0.70 * np.sin(0.29 * row),
            0.65 * np.cos(0.27 * row),
        )
    )
    root_radius = np.column_stack(
        (
            0.008 + 0.004 * (row % 3.0),
            np.full(ROWS, 0.08),
            np.full(ROWS, 0.015),
            np.full(ROWS, 0.015),
            np.full(ROWS, 0.12),
            np.full(ROWS, 0.12),
        )
    )
    root_lower = np.ascontiguousarray(root_center - root_radius)
    root_upper = np.ascontiguousarray(root_center + root_radius)
    q_center = q_nominal[None, :] + 0.025 * np.sin(phase)
    q_radius = 0.006 + 0.003 * ((row[:, None] + coordinate[None, :]) % 3.0)
    q_lower = np.ascontiguousarray(q_center - q_radius)
    q_upper = np.ascontiguousarray(q_center + q_radius)
    v_center = 0.12 * np.sin(phase + 0.4)
    v_radius = 0.05 + 0.01 * ((row[:, None] + coordinate[None, :]) % 2.0)
    v_lower = np.ascontiguousarray(v_center - v_radius)
    v_upper = np.ascontiguousarray(v_center + v_radius)
    available = np.ones(ROWS, np.uint8)
    root_acceleration = np.zeros((ROWS, 2), np.float64)
    joint_acceleration = np.zeros((ROWS, joints), np.float64)
    effort = 0.15 + 0.25 * (0.5 + 0.5 * np.sin(0.21 * row))
    diagnostics = np.empty((ROWS, 17), np.float64)

    timing_ns = np.empty(REPEAT_CALLS, np.uint64)
    allocation_calls = np.empty(REPEAT_CALLS, np.uint64)
    allocated_bytes = np.empty(REPEAT_CALLS, np.uint64)
    reference = None
    semantic_repeat = True
    for repeat in range(REPEAT_CALLS):
        timing = session.score_terminal_impact_state_box_batch(
            root_lower,
            root_upper,
            q_lower,
            q_upper,
            v_lower,
            v_upper,
            position_lower,
            position_upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        timing_ns[repeat], allocation_calls[repeat], allocated_bytes[repeat] = timing
        if reference is None:
            reference = diagnostics.copy()
        else:
            semantic_repeat &= np.array_equal(reference, diagnostics)

    names = tuple(session.terminal_impact_state_diagnostic_names)
    upper_names = (
        "time_to_impact_s",
        "vertical_specific_impact_energy_j_kg",
        "terminal_tilt_rad",
        "terminal_angular_rate_rad_s",
        "maximum_terminal_joint_velocity_utilization",
        "impact_speed_pressure",
        "tilt_pressure",
        "angular_rate_pressure",
        "joint_position_pressure",
        "joint_velocity_pressure",
        "actuator_effort_pressure",
        "maximum_terminal_harm_pressure",
        "aggregate_score",
    )
    upper_indices = np.asarray([names.index(name) for name in upper_names], np.int64)
    headroom_index = names.index("minimum_terminal_joint_headroom_fraction")
    point_diagnostics = np.empty((ROWS, POINTS_PER_BOX, 17), np.float64)
    point_out = np.empty((1, 17), np.float64)
    point_available = np.ones(1, np.uint8)
    point_root_acceleration = np.zeros((1, 2), np.float64)
    point_joint_acceleration = np.zeros((1, joints), np.float64)
    for box in range(ROWS):
        for point in range(POINTS_PER_BOX):
            bits = np.asarray(
                [((point * 37 + box * 17 + axis * 13) >> (axis % 5)) & 1 for axis in range(6)],
                np.float64,
            )
            root = root_lower[box] + bits * (root_upper[box] - root_lower[box])
            q_bits = ((point + box + np.arange(joints) * 3) % 2).astype(np.float64)
            v_bits = ((point * 5 + box + np.arange(joints) * 7) % 2).astype(np.float64)
            q = q_lower[box] + q_bits * (q_upper[box] - q_lower[box])
            v = v_lower[box] + v_bits * (v_upper[box] - v_lower[box])
            session.score_terminal_impact_state_batch(
                root[None, :],
                np.ascontiguousarray(q),
                np.ascontiguousarray(v[None, :]),
                position_lower,
                position_upper,
                velocity_limit,
                point_available,
                point_root_acceleration,
                point_joint_acceleration,
                np.asarray([effort[box]], np.float64),
                point_out,
            )
            point_diagnostics[box, point] = point_out[0]

    upper_slack = diagnostics[:, None, upper_indices] - point_diagnostics[:, :, upper_indices]
    headroom_slack = (
        point_diagnostics[:, :, headroom_index]
        - diagnostics[:, None, headroom_index]
    )
    point_containment = bool(
        np.all(upper_slack >= -1.0e-12) and np.all(headroom_slack >= -1.0e-12)
    )
    mechanism_passed = bool(
        semantic_repeat
        and np.all(allocation_calls == 0)
        and np.all(allocated_bytes == 0)
        and point_containment
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "rows": ROWS,
        "points_per_box": POINTS_PER_BOX,
        "independent_point_queries": ROWS * POINTS_PER_BOX,
        "repeat_calls": REPEAT_CALLS,
        "physics_steps": 0,
        "policy_steps": 0,
        "selector_queries": 0,
        "plant_actions": 0,
        "semantic_repeat": semantic_repeat,
        "zero_rust_allocation": bool(
            np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
        ),
        "point_containment": point_containment,
        "minimum_upper_slack": float(np.min(upper_slack)),
        "minimum_headroom_slack": float(np.min(headroom_slack)),
        "batch_timing_ns": distribution(timing_ns),
        "mechanism_passed": mechanism_passed,
        "profile_frozen": False,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw complete terminal-state box boundary · r255",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · profile **NOT FROZEN** · authority **NOT ADMITTED**.",
            "",
            "R255 adds a Rust-owned complete state interval before support-free terminal propagation. Clearance, vertical speed, roll/pitch, angular rate, joint position, and joint velocity are all bounded; pressure fields are upper bounds and joint headroom is a lower bound. The query performs no policy step, physics step, selector call, plant action, probability assignment, or command admission.",
            "",
            f"A G1-shaped 64-row batch repeats {REPEAT_CALLS} times with p50/p99 {result['batch_timing_ns']['p50'] / 1e3:.2f}/{result['batch_timing_ns']['p99'] / 1e3:.2f} µs and zero measured Rust allocation. {ROWS * POINTS_PER_BOX:,} independently scored box points are contained; minimum upper/headroom slack is {result['minimum_upper_slack']:.3g}/{result['minimum_headroom_slack']:.3g}.",
            "",
            "This boundary closes only the interval-composition mechanism. Independent candidate boxes cannot prove a paired candidate-minus-baseline improvement because their errors may be correlated. A causal shared-hypothesis or paired state tube must be frozen on spent contact-law/estimator evidence before another new-law holdout.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-terminal-state-box-boundary-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-terminal-state-box-boundary.npz",
        root_lower=root_lower,
        root_upper=root_upper,
        joint_position_lower_state=q_lower,
        joint_position_upper_state=q_upper,
        joint_velocity_lower=v_lower,
        joint_velocity_upper=v_upper,
        diagnostics=diagnostics,
        point_diagnostics=point_diagnostics,
        timing_ns=timing_ns,
        allocation_calls=allocation_calls,
        allocated_bytes=allocated_bytes,
    )
    (output / "G1_TERMINAL_STATE_BOX_BOUNDARY.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw terminal state box · r255"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_frozen": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
