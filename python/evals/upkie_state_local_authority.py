#!/usr/bin/env python3
"""Measure Upkie's state-local task authority without a policy or plant.

Python owns the immutable probe matrix and scoring.  A persistent Rust
FloatingWbcSession owns model products, contact rows, rigid-body dynamics,
hierarchical task solve, effort limits, residuals, timing, and allocations.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "upkie-state-local-authority-r135"
JOINT_ORDER = (
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
)
CONTACT_FRAMES = ("left_wheel_center", "right_wheel_center")
ROLLING_COORDINATES = np.asarray([2, 5], dtype=np.int64)
ROLLING_COEFFICIENTS = np.asarray([-0.05, 0.05], dtype=np.float64)
CONTACT_MODES = {
    "locked_point": 0,
    "normal_point": 1,
    "rolling_point": 2,
    "rolling_wheel": 3,
}
AXES = ("roll", "pitch", "yaw", "forward", "lateral", "vertical")
DEMANDS = (1.0, 10.0, 50.0, 250.0)
STATUS_NAMES = {
    0: "Solved",
    1: "SolvedWithSlack",
    2: "PrimalInfeasible",
    3: "NumericalOrInvalid",
    4: "MaxIterations",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_STATE_LOCAL_AUTHORITY_R135.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def standing_posture() -> np.ndarray:
    return np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)


def allocate_outputs(session: Any, ticks: int) -> dict[str, np.ndarray]:
    dof = session.dof
    tasks = session.task_diagnostic_capacity
    levels = 5
    return {
        "generalized_acceleration": np.empty((ticks, dof + 6), np.float64),
        "actuator_torque": np.empty((ticks, dof), np.float64),
        "contact_normal_force": np.empty((ticks, 2), np.float64),
        "contact_force_basis": np.empty((ticks, 2, 3), np.float64),
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
        "task_pseudoinverse_calls_by_priority": np.empty(
            (ticks, levels), np.uint16
        ),
        "clipped_steps": np.empty(ticks, np.uint16),
        "clipped_steps_by_priority": np.empty((ticks, levels), np.uint16),
        "task_jacobi_sweeps": np.empty(ticks, np.uint16),
        "task_jacobi_sweeps_by_priority": np.empty(
            (ticks, levels), np.uint16
        ),
        "feasibility_projection_sweeps": np.empty(ticks, np.uint16),
        "feasibility_halfspace_projections": np.empty(ticks, np.uint32),
        "feasibility_polish_iterations": np.empty(ticks, np.uint16),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
    }


def build_probe_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for mode_name, mode_code in CONTACT_MODES.items():
        rows.append(
            {
                "mode": mode_name,
                "mode_code": mode_code,
                "axis": "baseline",
                "coordinate": -1,
                "demand": 0.0,
            }
        )
        for axis, coordinate in zip(AXES, range(6), strict=True):
            for magnitude in DEMANDS:
                for sign in (-1.0, 1.0):
                    rows.append(
                        {
                            "mode": mode_name,
                            "mode_code": mode_code,
                            "axis": axis,
                            "coordinate": coordinate,
                            "demand": sign * magnitude,
                        }
                    )
    return rows


def run_probe(model_path: pathlib.Path) -> tuple[list[dict[str, Any]], dict[str, np.ndarray]]:
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model_path))
    root = np.asarray([0.0, 0.0, 0.539], np.float64)
    q = standing_posture()
    balanced_root = np.empty(3, np.float64)
    balanced_q = np.empty(6, np.float64)
    error = balance.balanced_standing(root, q, balanced_root, balanced_q)
    if abs(error) > 1.0e-6:
        raise RuntimeError(f"balanced standing error {error:.3e} m exceeds gate")

    session = bonesaw.FloatingWbcSession(
        str(model_path),
        maximum_contacts=2,
        friction_coefficient=0.8,
        maximum_acceleration=250.0,
        maximum_torque=2000.0,
        maximum_normal_force_multiple=3.0,
        root_angular_task_weight=10.0,
        root_height_task_weight=10.0,
        root_horizontal_task_weight=10.0,
        root_horizontal_task_priority=0,
        joint_posture_weight=0.0,
        center_of_mass_task_weight=0.0,
    )
    if tuple(session.joint_names) != JOINT_ORDER:
        raise RuntimeError("Upkie coordinate order changed")
    frames = list(session.frame_names)
    frame_ids = np.asarray(
        [frames.index(name) for name in CONTACT_FRAMES], np.int64
    )

    rows = build_probe_rows()
    ticks = len(rows)
    root_acceleration = np.zeros((ticks, 3), np.float64)
    root_angular_acceleration = np.zeros((ticks, 3), np.float64)
    mode_trace = np.empty((ticks, 2), np.uint8)
    for tick, row in enumerate(rows):
        mode_trace[tick] = row["mode_code"]
        coordinate = row["coordinate"]
        if 0 <= coordinate < 3:
            root_angular_acceleration[tick, coordinate] = row["demand"]
        elif coordinate >= 3:
            root_acceleration[tick, coordinate - 3] = row["demand"]

    out = allocate_outputs(session, ticks)
    tiled_root = np.repeat(balanced_root[None, :], ticks, axis=0)
    tiled_q = np.repeat(balanced_q[None, :], ticks, axis=0)
    zeros_root = np.zeros((ticks, 3), np.float64)
    zeros_joint = np.zeros((ticks, 6), np.float64)
    identity_quaternion = np.zeros((ticks, 4), np.float64)
    identity_quaternion[:, 0] = 1.0
    contact_bases = np.repeat(
        np.eye(3, dtype=np.float64)[None, None, :, :], ticks * 2, axis=0
    ).reshape(ticks, 2, 3, 3)

    session.run_oracle_trace(
        tiled_root,
        zeros_root,
        root_acceleration,
        tiled_q,
        zeros_joint,
        zeros_joint,
        frame_ids,
        np.ones((ticks, 2), np.uint8),
        np.zeros(2, np.uint8),
        np.ones(2, np.float64),
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
        contact_mode_trace=mode_trace,
        rolling_coordinates=ROLLING_COORDINATES,
        rolling_velocity_coefficients=ROLLING_COEFFICIENTS,
        rolling_velocity_stabilization_gains=np.full(2, 2.0, np.float64),
        rolling_maximum_stabilization_accelerations=np.full(2, 3.0, np.float64),
        root_quaternions_wxyz=identity_quaternion,
        root_angular_velocities_world=zeros_root,
        root_angular_accelerations_world=root_angular_acceleration,
        contact_bases_world=contact_bases,
    )
    return rows, out


def summarize(rows: list[dict[str, Any]], out: dict[str, np.ndarray]) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    for tick, row in enumerate(rows):
        coordinate = row["coordinate"]
        achieved_vector = out["generalized_acceleration"][tick, :6]
        requested = row["demand"]
        achieved = 0.0 if coordinate < 0 else float(achieved_vector[coordinate])
        orthogonal = achieved_vector.copy()
        if coordinate >= 0:
            orthogonal[coordinate] = 0.0
        samples.append(
            {
                **row,
                "status": STATUS_NAMES[int(out["status"][tick])],
                "achieved": achieved,
                "signed_fraction": (
                    None if requested == 0.0 else achieved / requested
                ),
                "orthogonal_acceleration_l2": float(np.linalg.norm(orthogonal)),
                "task_rms": [float(value) for value in out["task_rms"][tick, :4]],
                "task_clipped": [int(value) for value in out["task_clipped"][tick, :4]],
                "torque_utilization": float(out["maximum_torque_utilization"][tick]),
                "minimum_friction_margin": float(out["minimum_friction_margin"][tick]),
                "maximum_constraint_violation": float(
                    out["maximum_constraint_violation"][tick]
                ),
                "dynamics_residual": float(out["dynamics_residual"][tick]),
                "contact_residual": float(out["contact_residual"][tick]),
                "step_ns": int(out["step_ns"][tick]),
                "allocation_calls": int(out["allocation_calls"][tick]),
                "allocated_bytes": int(out["allocated_bytes"][tick]),
            }
        )

    envelope: list[dict[str, Any]] = []
    for mode in CONTACT_MODES:
        for axis in AXES:
            selected = [
                sample
                for sample in samples
                if sample["mode"] == mode
                and sample["axis"] == axis
                and abs(sample["demand"]) == DEMANDS[-1]
            ]
            small_signal = [
                sample
                for sample in samples
                if sample["mode"] == mode
                and sample["axis"] == axis
                and abs(sample["demand"]) == DEMANDS[0]
            ]
            all_demands = [
                sample
                for sample in samples
                if sample["mode"] == mode and sample["axis"] == axis
            ]
            assert len(selected) == 2
            assert len(small_signal) == 2
            envelope.append(
                {
                    "mode": mode,
                    "axis": axis,
                    "negative_achieved": selected[0]["achieved"],
                    "positive_achieved": selected[1]["achieved"],
                    "minimum_signed_fraction": min(
                        float(sample["signed_fraction"]) for sample in selected
                    ),
                    "minimum_small_signal_fraction": min(
                        float(sample["signed_fraction"]) for sample in small_signal
                    ),
                    "maximum_absolute_achieved": max(
                        abs(sample["achieved"]) for sample in all_demands
                    ),
                    "maximum_torque_utilization": max(
                        sample["torque_utilization"] for sample in selected
                    ),
                    "maximum_task_rms": max(
                        max(sample["task_rms"]) for sample in selected
                    ),
                    "statuses": sorted({sample["status"] for sample in selected}),
                }
            )
    status_counts = {
        name: int(np.sum(out["status"] == code))
        for code, name in STATUS_NAMES.items()
        if np.any(out["status"] == code)
    }
    return {
        "samples": samples,
        "envelope_250": envelope,
        "status_counts": status_counts,
        "latency_ns": distribution(out["step_ns"]),
        "maximum_hard_violation": float(
            np.max(out["maximum_constraint_violation"])
        ),
        "maximum_dynamics_residual": float(np.max(out["dynamics_residual"])),
        "maximum_contact_residual": float(np.max(out["contact_residual"])),
        "allocation_calls": int(np.sum(out["allocation_calls"])),
        "allocated_bytes": int(np.sum(out["allocated_bytes"])),
    }


def make_report(metrics: dict[str, Any]) -> str:
    envelope = metrics["envelope_250"]
    rolling = [row for row in envelope if row["mode"] == "rolling_wheel"]
    lines = [
        "# Bonesaw Upkie state-local task authority · r135",
        "",
        "> No policy, state rollout, simulator, or physics engine participates. Python authors 196 immutable requests; one persistent Rust session emits model products, rolling/contact rows, rigid-body dynamics, strict hierarchy, bounded effort, and diagnostics.",
        "",
        "## Outcome",
        "",
        "The probe makes the lateral limitation explicit instead of blaming a late solver status. At the nominal two-wheel state, a ±250 rad/s² roll request is clipped to a small feasible response while pitch and yaw retain materially different authority. The value is a local acceleration witness, not a recovery guarantee or a learned-policy score.",
        "",
        "Angular rows use rad/s²; translation rows use m/s². `±1 local gain` is the worse signed response fraction over the two request signs. `maximum achieved` is the largest directional response anywhere on the 1/10/50/250 request curve.",
        "",
        *markdown_table(
            ["rolling-wheel axis", "±1 local gain", "maximum achieved", "−250 achieved", "+250 achieved", "250 minimum fraction", "max effort use", "status"],
            [
                [
                    row["axis"],
                    f'{row["minimum_small_signal_fraction"]:.4f}',
                    f'{row["maximum_absolute_achieved"]:.4f}',
                    f'{row["negative_achieved"]:.4f}',
                    f'{row["positive_achieved"]:.4f}',
                    f'{row["minimum_signed_fraction"]:.4f}',
                    f'{100.0 * row["maximum_torque_utilization"]:.2f}%',
                    ", ".join(row["statuses"]),
                ]
                for row in rolling
            ],
        ),
        "",
        "## Contact-mode comparison at the saturation request",
        "",
        *markdown_table(
            ["mode", "axis", "±1 local gain", "maximum achieved", "−250 achieved", "+250 achieved", "250 minimum fraction", "max task RMS"],
            [
                [
                    row["mode"],
                    row["axis"],
                    f'{row["minimum_small_signal_fraction"]:.4f}',
                    f'{row["maximum_absolute_achieved"]:.4f}',
                    f'{row["negative_achieved"]:.4f}',
                    f'{row["positive_achieved"]:.4f}',
                    f'{row["minimum_signed_fraction"]:.4f}',
                    f'{row["maximum_task_rms"]:.4f}',
                ]
                for row in envelope
            ],
        ),
        "",
        "## Admission and execution evidence",
        "",
        *markdown_table(
            ["witness", "value"],
            [
                ["statuses", json.dumps(metrics["status_counts"], sort_keys=True)],
                ["maximum hard violation", f'{metrics["maximum_hard_violation"]:.3e}'],
                ["maximum dynamics residual", f'{metrics["maximum_dynamics_residual"]:.3e}'],
                ["maximum contact residual", f'{metrics["maximum_contact_residual"]:.3e}'],
                ["Rust allocations in timed calls", f'{metrics["allocation_calls"]} calls / {metrics["allocated_bytes"]} bytes'],
                ["Rust solve p50 / p99", f'{metrics["latency_ns"]["p50"] / 1e3:.1f} / {metrics["latency_ns"]["p99"] / 1e3:.1f} µs'],
            ],
        ),
        "",
        "## Interpretation contract",
        "",
        "- `SolvedWithSlack` means hard dynamics/contact/bounds are admitted while at least one soft priority cannot realize its request.",
        "- Signed fraction is achieved acceleration along the requested axis divided by the request; it is deliberately threshold-free.",
        "- Orthogonal leakage, per-layer RMS/clipping, friction margin, effort utilization, and hard residual remain separate in the JSON rather than being collapsed into one health score.",
        "- The r133 MuJoCo envelope remains the plant-consequence gate. This report explains a local authority boundary; it does not promote the experimental planar steering law or claim lateral recovery.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rows, raw = run_probe(model_path)
    summary = summarize(rows, raw)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "probe_count": len(rows),
        "contact_modes": CONTACT_MODES,
        "demands": list(DEMANDS),
        **summary,
    }
    report = make_report(metrics)
    (output / "upkie-state-local-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_STATE_LOCAL_AUTHORITY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({
        "output": str(output),
        "web_report": str(web_report),
        "probe_count": len(rows),
        "status_counts": metrics["status_counts"],
        "allocation_calls": metrics["allocation_calls"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
