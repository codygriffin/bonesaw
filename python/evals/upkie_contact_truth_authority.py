#!/usr/bin/env python3
"""Replay frozen pre-fall states with declared versus measured wheel contacts."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_mujoco_plant_report import run_case
from upkie_state_local_authority import (
    CONTACT_FRAMES,
    ROLLING_COEFFICIENTS,
    ROLLING_COORDINATES,
    STATUS_NAMES,
    allocate_outputs,
)


REVISION = "upkie-contact-truth-authority-r138"
CASES = (
    ("left_1n", (0.0, 1.0, 0.0)),
    ("left_2n", (0.0, 2.0, 0.0)),
    ("right_2n", (0.0, -2.0, 0.0)),
    ("forward_6n_overload", (6.0, 0.0, 0.0)),
)
OFFSETS_S = (0.50, 0.25, 0.10, 0.05)
VARIANTS = ("roll_rate_arrest", "lateral_rate_arrest", "coupled_rate_arrest")
CONTACT_SOURCES = ("declared_double", "measured")
ARREST_HORIZON_S = 0.25


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_TRUTH_AUTHORITY_R138.html"
    )
    return parser.parse_args()


def collect_snapshots(model: pathlib.Path) -> list[dict[str, Any]]:
    snapshots: list[dict[str, Any]] = []
    for case, force in CASES:
        trace = run_case(
            model,
            duration=6.0,
            push_start=1.0,
            push_duration=0.1,
            push_force=0.0,
            push_force_world=force,
            contact_model="soft",
            balance_mode="capture",
            capture_velocity_fraction=0.2,
            terminate_on_fall=True,
        )
        fall_time = float(np.asarray(trace["time_s"])[-1] + 0.005)
        for offset in OFFSETS_S:
            target = fall_time - offset
            tick = int(np.argmin(np.abs(np.asarray(trace["time_s"]) - target)))
            snapshots.append(
                {
                    "case": case,
                    "fall_time_s": fall_time,
                    "offset_s": offset,
                    "time_s": float(np.asarray(trace["time_s"])[tick]),
                    "root_position": np.asarray(trace["root_position"])[tick].copy(),
                    "root_quaternion": np.asarray(trace["root_quaternion"])[tick].copy(),
                    "root_twist": np.asarray(trace["root_twist"])[tick].copy(),
                    "q": np.asarray(trace["q"])[tick].copy(),
                    "v": np.asarray(trace["v"])[tick].copy(),
                    "measured_contact": np.asarray(
                        trace["measured_wheel_contact_active"]
                    )[tick].copy(),
                }
            )
    return snapshots


def build_rows(snapshots: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for snapshot_index, snapshot in enumerate(snapshots):
        roll = float(
            np.clip(-snapshot["root_twist"][0] / ARREST_HORIZON_S, -250.0, 250.0)
        )
        lateral = float(
            np.clip(-snapshot["root_twist"][4] / ARREST_HORIZON_S, -250.0, 250.0)
        )
        for contact_source in CONTACT_SOURCES:
            contact = (
                np.ones(2, np.uint8)
                if contact_source == "declared_double"
                else snapshot["measured_contact"].copy()
            )
            for variant in VARIANTS:
                request = np.zeros(6, np.float64)
                if variant != "lateral_rate_arrest":
                    request[0] = roll
                if variant != "roll_rate_arrest":
                    request[4] = lateral
                rows.append(
                    {
                        "snapshot_index": snapshot_index,
                        "contact_source": contact_source,
                        "contact": contact,
                        "variant": variant,
                        "request": request,
                    }
                )
    return rows


def query(
    session: Any,
    frame_ids: np.ndarray,
    snapshots: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, np.ndarray]:
    ticks = len(rows)
    root_position = np.empty((ticks, 3), np.float64)
    root_velocity = np.empty((ticks, 3), np.float64)
    root_acceleration = np.zeros((ticks, 3), np.float64)
    root_quaternion = np.empty((ticks, 4), np.float64)
    root_angular_velocity = np.empty((ticks, 3), np.float64)
    root_angular_acceleration = np.zeros((ticks, 3), np.float64)
    q = np.empty((ticks, 6), np.float64)
    v = np.empty((ticks, 6), np.float64)
    joint_acceleration = np.zeros((ticks, 6), np.float64)
    contact_active = np.empty((ticks, 2), np.uint8)
    mode_trace = np.full((ticks, 2), 3, np.uint8)
    for tick, row in enumerate(rows):
        snapshot = snapshots[row["snapshot_index"]]
        root_position[tick] = snapshot["root_position"]
        root_quaternion[tick] = snapshot["root_quaternion"]
        root_angular_velocity[tick] = snapshot["root_twist"][:3]
        root_velocity[tick] = snapshot["root_twist"][3:]
        q[tick] = snapshot["q"]
        v[tick] = snapshot["v"]
        contact_active[tick] = row["contact"]
        root_angular_acceleration[tick] = row["request"][:3]
        root_acceleration[tick] = row["request"][3:]
    out = allocate_outputs(session, ticks)
    bases = np.repeat(
        np.eye(3, dtype=np.float64)[None, None, :, :], ticks * 2, axis=0
    ).reshape(ticks, 2, 3, 3)
    session.run_oracle_trace(
        root_position,
        root_velocity,
        root_acceleration,
        q,
        v,
        joint_acceleration,
        frame_ids,
        contact_active,
        np.zeros(2, np.uint8),
        np.zeros(2, np.float64),
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
        root_quaternions_wxyz=root_quaternion,
        root_angular_velocities_world=root_angular_velocity,
        root_angular_accelerations_world=root_angular_acceleration,
        contact_bases_world=bases,
    )
    return out


def semantic_equal(left: dict[str, np.ndarray], right: dict[str, np.ndarray]) -> bool:
    fields = (
        "generalized_acceleration",
        "actuator_torque",
        "contact_force_basis",
        "task_rms",
        "task_clipped",
        "status",
        "maximum_constraint_violation",
    )
    return all(np.array_equal(left[field], right[field]) for field in fields)


def main() -> None:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    snapshots = collect_snapshots(model)
    rows = build_rows(snapshots)
    session = bonesaw.FloatingWbcSession(
        str(model),
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
    frames = list(session.frame_names)
    frame_ids = np.asarray([frames.index(name) for name in CONTACT_FRAMES], np.int64)
    out = query(session, frame_ids, snapshots, rows)
    replay = query(session, frame_ids, snapshots, rows)
    samples: list[dict[str, Any]] = []
    for tick, row in enumerate(rows):
        snapshot = snapshots[row["snapshot_index"]]
        request = row["request"]
        achieved = out["generalized_acceleration"][tick, :6]
        active = np.flatnonzero(request)
        fraction = (
            1.0
            if len(active) == 0
            else float(np.dot(achieved[active], request[active]) / np.dot(request[active], request[active]))
        )
        samples.append(
            {
                "case": snapshot["case"],
                "offset_s": snapshot["offset_s"],
                "time_s": snapshot["time_s"],
                "measured_contact": [int(value) for value in snapshot["measured_contact"]],
                "contact_source": row["contact_source"],
                "variant": row["variant"],
                "requested_roll_rad_s2": float(request[0]),
                "requested_lateral_m_s2": float(request[4]),
                "achieved_roll_rad_s2": float(achieved[0]),
                "achieved_lateral_m_s2": float(achieved[4]),
                "directional_fraction": fraction,
                "status": STATUS_NAMES[int(out["status"][tick])],
                "hard_violation": float(out["maximum_constraint_violation"][tick]),
                "torque_utilization": float(out["maximum_torque_utilization"][tick]),
            }
        )
    coupled = [sample for sample in samples if sample["variant"] == "coupled_rate_arrest"]
    paired_rows = []
    maximum_delta = 0.0
    for snapshot in snapshots:
        selected = [
            sample
            for sample in coupled
            if sample["case"] == snapshot["case"] and sample["offset_s"] == snapshot["offset_s"]
        ]
        declared = next(sample for sample in selected if sample["contact_source"] == "declared_double")
        measured = next(sample for sample in selected if sample["contact_source"] == "measured")
        if declared["status"] in ("Solved", "SolvedWithSlack"):
            maximum_delta = max(maximum_delta, abs(declared["directional_fraction"] - measured["directional_fraction"]))
        paired_rows.append(
            [
                snapshot["case"],
                f"{snapshot['offset_s']:.2f}",
                "".join(str(int(value)) for value in snapshot["measured_contact"]),
                (
                    f"{declared['directional_fraction']:.3f}"
                    if declared["status"] in ("Solved", "SolvedWithSlack")
                    else "NON-EXEC"
                ),
                f"{measured['directional_fraction']:.3f}",
                declared["status"],
                measured["status"],
            ]
        )
    patterns = {tuple(int(value) for value in snapshot["measured_contact"]) for snapshot in snapshots}
    measured_ticks = [
        tick for tick, row in enumerate(rows) if row["contact_source"] == "measured"
    ]
    declared_ticks = [
        tick for tick, row in enumerate(rows) if row["contact_source"] == "declared_double"
    ]
    gates = {
        "matrix_complete": len(snapshots) == len(CASES) * len(OFFSETS_S) and len(rows) == len(snapshots) * len(CONTACT_SOURCES) * len(VARIANTS),
        "double_single_zero_contact_discriminate": {(1, 1), (1, 0), (0, 0)}.issubset(patterns),
        "declared_contact_mismatch_observed": any(tuple(snapshot["measured_contact"]) != (1, 1) for snapshot in snapshots),
        "authority_result_changes": maximum_delta > 1.0e-3,
        "measured_contact_hard_rows_feasible": float(
            np.max(out["maximum_constraint_violation"][measured_ticks])
        ) < 1.0e-8,
        "stale_double_contact_hard_failure_observed": any(
            int(out["status"][tick]) == 4
            and float(out["maximum_constraint_violation"][tick]) > 1.0e-3
            for tick in declared_ticks
        ),
        "exact_replay": semantic_equal(out, replay),
        "zero_rust_allocation": bool(
            int(np.sum(out["allocation_calls"])) == 0
            and int(np.sum(out["allocated_bytes"])) == 0
        ),
        "finite": bool(np.isfinite(out["generalized_acceleration"]).all()),
    }
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "snapshot_count": len(snapshots),
        "probe_count": len(rows),
        "contact_patterns": sorted("".join(str(value) for value in pattern) for pattern in patterns),
        "maximum_declared_measured_directional_fraction_delta": maximum_delta,
        "samples": samples,
    }
    report = "\n".join(
        [
            "# Bonesaw Upkie measured-contact authority audit · r138",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. Frozen-state queries contain no policy, integration, or physics step; the source snapshots come from the retained MuJoCo failure boundary.",
            "",
            "## Outcome",
            "",
            "The baseline plant controller declares both RollingWheel contacts hard-active on every tick, but the actual plant reaches double-, single-, and zero-wheel support before the four frozen falls. Replaying the same root/joint states and the same 250 ms roll/lateral rate-arrest request through persistent Rust changes the local authority result when measured contact activity replaces the stale declaration. `Projection gain` is the signed achieved/request projection: negative means response away from the request and values above one mean over-response, not extra health. This identifies contact observation/admission as a missing controller input; it does not authorize a transition controller or claim recovery.",
            "",
            *markdown_table(
                ["case", "before fall s", "measured L/R", "declared projection gain", "measured projection gain", "declared status", "measured status"],
                paired_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]),
            "",
            "## Contract",
            "",
            "- `11`, `10`, and `00` are exact MuJoCo wheel-subtree-to-world contact observations, not inferred from total contact count. The frozen rows do not contain the mirrored `01` single-contact state, and the audit does not manufacture it.",
            "- Declared and measured queries share identical frozen q/v/root state, request, contact basis, model, effort bounds, and solver hierarchy; only the active contact mask changes.",
            "- The 250 ms rate-arrest vector is a diagnostic request and is never executed. The audit adds no policy or rollout.",
            "- A future live transition needs debounced/causal contact observations and an independently admitted one-/zero-contact action; simulator truth is not a hardware estimator.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-truth-authority-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (output / "UPKIE_CONTACT_TRUTH_AUTHORITY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
