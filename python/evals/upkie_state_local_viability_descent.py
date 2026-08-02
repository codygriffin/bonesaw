#!/usr/bin/env python3
"""Policy- and plant-step-free local viability descent at retained fall boundaries."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_truth_authority import collect_snapshots, query
from upkie_mujoco_plant_report import quaternion_rotation_vector
from upkie_state_local_authority import CONTACT_FRAMES, STATUS_NAMES


REVISION = "upkie-state-local-viability-descent-r146"
HORIZON_S = 0.25
ROLL_BOUND_RAD = math.radians(45.0)
LATERAL_CAPTURE_BOUND_M = 0.10
ROLL_REQUESTS = (-250.0, -100.0, -40.0, 0.0, 40.0, 100.0, 250.0)
LATERAL_REQUESTS = (-250.0, -100.0, -40.0, 0.0, 40.0, 100.0, 250.0)
YAW_REQUESTS = (-80.0, -20.0, 0.0, 20.0, 80.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_STATE_LOCAL_VIABILITY_DESCENT_R146.html"
    )
    return parser.parse_args()


def build_rows(snapshots: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for snapshot_index, snapshot in enumerate(snapshots):
        contact = np.asarray(snapshot["measured_contact"], np.uint8)
        for roll in ROLL_REQUESTS:
            for lateral in LATERAL_REQUESTS:
                for yaw in YAW_REQUESTS:
                    request = np.zeros(6, np.float64)
                    request[0] = roll
                    request[2] = yaw
                    request[4] = lateral
                    rows.append(
                        {
                            "snapshot_index": snapshot_index,
                            "contact_source": "measured",
                            "contact": contact,
                            "variant": "viability_grid",
                            "request": request,
                        }
                    )
    return rows


def pressure(
    snapshot: dict[str, Any], achieved: np.ndarray, horizon_s: float
) -> dict[str, float]:
    rotation = quaternion_rotation_vector(np.asarray(snapshot["root_quaternion"]))
    roll = float(rotation[0])
    roll_rate = float(snapshot["root_twist"][0])
    lateral = float(snapshot["root_position"][1])
    lateral_rate = float(snapshot["root_twist"][4])
    height = max(float(snapshot["root_position"][2]), 0.05)
    omega = math.sqrt(9.81 / height)
    roll_acceleration = float(achieved[0])
    lateral_acceleration = float(achieved[4])
    predicted_roll = (
        roll
        + horizon_s * roll_rate
        + 0.5 * horizon_s * horizon_s * roll_acceleration
    )
    predicted_roll_rate = roll_rate + horizon_s * roll_acceleration
    predicted_lateral = (
        lateral
        + horizon_s * lateral_rate
        + 0.5 * horizon_s * horizon_s * lateral_acceleration
    )
    predicted_lateral_rate = lateral_rate + horizon_s * lateral_acceleration
    roll_capture = predicted_roll + predicted_roll_rate / omega
    lateral_capture = predicted_lateral + predicted_lateral_rate / omega
    roll_pressure = abs(roll_capture) / ROLL_BOUND_RAD
    lateral_pressure = abs(lateral_capture) / LATERAL_CAPTURE_BOUND_M
    return {
        "omega_rad_s": omega,
        "roll_capture_rad": roll_capture,
        "lateral_capture_m": lateral_capture,
        "roll_pressure": roll_pressure,
        "lateral_pressure": lateral_pressure,
        "maximum_pressure": max(roll_pressure, lateral_pressure),
    }


def required_acceleration(snapshot: dict[str, Any]) -> tuple[float, float]:
    rotation = quaternion_rotation_vector(np.asarray(snapshot["root_quaternion"]))
    height = max(float(snapshot["root_position"][2]), 0.05)
    omega = math.sqrt(9.81 / height)
    denominator = 0.5 * HORIZON_S * HORIZON_S + HORIZON_S / omega
    roll = float(rotation[0])
    roll_rate = float(snapshot["root_twist"][0])
    lateral = float(snapshot["root_position"][1])
    lateral_rate = float(snapshot["root_twist"][4])
    roll_required = -(
        roll + roll_rate * (HORIZON_S + 1.0 / omega)
    ) / denominator
    lateral_required = -(
        lateral + lateral_rate * (HORIZON_S + 1.0 / omega)
    ) / denominator
    return roll_required, lateral_required


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
    output = query(session, frame_ids, snapshots, rows)
    replay = query(session, frame_ids, snapshots, rows)
    row_count = len(ROLL_REQUESTS) * len(LATERAL_REQUESTS) * len(YAW_REQUESTS)
    samples: list[dict[str, Any]] = []
    for snapshot_index, snapshot in enumerate(snapshots):
        start = snapshot_index * row_count
        stop = start + row_count
        zero_index = next(
            tick
            for tick in range(start, stop)
            if np.count_nonzero(rows[tick]["request"]) == 0
        )
        executable = [
            tick
            for tick in range(start, stop)
            if int(output["status"][tick]) in (0, 1)
            and float(output["maximum_constraint_violation"][tick]) <= 1.0e-8
        ]
        scored = [
            (pressure(snapshot, output["generalized_acceleration"][tick, :6], HORIZON_S)["maximum_pressure"], tick)
            for tick in executable
        ]
        best_pressure, best_tick = min(scored)
        current = pressure(snapshot, np.zeros(6, np.float64), 0.0)
        zero = pressure(
            snapshot, output["generalized_acceleration"][zero_index, :6], HORIZON_S
        )
        best = pressure(
            snapshot, output["generalized_acceleration"][best_tick, :6], HORIZON_S
        )
        required_roll, required_lateral = required_acceleration(snapshot)
        samples.append(
            {
                "case": snapshot["case"],
                "offset_s": float(snapshot["offset_s"]),
                "measured_contact": [int(value) for value in snapshot["measured_contact"]],
                "current_pressure": current["maximum_pressure"],
                "zero_request_pressure": zero["maximum_pressure"],
                "best_pressure": best_pressure,
                "best_over_current": best_pressure / max(current["maximum_pressure"], 1.0e-12),
                "best_over_zero_request": best_pressure / max(zero["maximum_pressure"], 1.0e-12),
                "current_roll_capture_rad": current["roll_capture_rad"],
                "current_lateral_capture_m": current["lateral_capture_m"],
                "best_roll_capture_rad": best["roll_capture_rad"],
                "best_lateral_capture_m": best["lateral_capture_m"],
                "required_roll_acceleration_rad_s2": required_roll,
                "required_lateral_acceleration_m_s2": required_lateral,
                "best_request": rows[best_tick]["request"].tolist(),
                "best_achieved": output["generalized_acceleration"][best_tick, :6].tolist(),
                "best_status": STATUS_NAMES[int(output["status"][best_tick])],
                "best_torque_utilization": float(output["maximum_torque_utilization"][best_tick]),
                "best_hard_violation": float(output["maximum_constraint_violation"][best_tick]),
                "executable_candidates": len(executable),
            }
        )

    double = [sample for sample in samples if sample["measured_contact"] == [1, 1]]
    single = [sample for sample in samples if sum(sample["measured_contact"]) == 1]
    zero = [sample for sample in samples if sample["measured_contact"] == [0, 0]]
    replay_fields = (
        "generalized_acceleration",
        "actuator_torque",
        "contact_force_basis",
        "status",
        "maximum_constraint_violation",
        "allocation_calls",
        "allocated_bytes",
    )
    gates = {
        "matrix_complete": len(snapshots) == 16 and len(rows) == 16 * row_count,
        "double_single_zero_support_covered": bool(double and single and zero),
        "every_snapshot_has_executable_candidate": all(
            sample["executable_candidates"] > 0 for sample in samples
        ),
        "exact_replay": all(
            np.array_equal(output[field], replay[field]) for field in replay_fields
        ),
        "zero_rust_allocation": int(np.sum(output["allocation_calls"])) == 0
        and int(np.sum(output["allocated_bytes"])) == 0,
        "finite": all(
            math.isfinite(value)
            for sample in samples
            for key, value in sample.items()
            if isinstance(value, float)
        ),
    }
    passed = all(gates.values())
    table_rows = [
        [
            sample["case"],
            f"{sample['offset_s']:.2f}",
            "".join(str(value) for value in sample["measured_contact"]),
            f"{sample['current_pressure']:.3f}",
            f"{sample['zero_request_pressure']:.3f}",
            f"{sample['best_pressure']:.3f}",
            f"{sample['best_over_zero_request']:.3f}",
            f"{sample['best_request'][0]:+.0f}/{sample['best_request'][4]:+.0f}/{sample['best_request'][2]:+.0f}",
            sample["best_status"],
        ]
        for sample in samples
    ]
    by_support = {}
    for name, group in (("double", double), ("single", single), ("zero", zero)):
        ratios = np.asarray([sample["best_over_zero_request"] for sample in group])
        by_support[name] = {
            "count": len(group),
            "median_best_over_zero_request": float(np.median(ratios)),
            "maximum_best_over_zero_request": float(np.max(ratios)),
            "descent_rows": int(np.sum(ratios < 1.0)),
            "half_pressure_rows": int(np.sum(ratios <= 0.5)),
        }
    descent_rows = sum(sample["best_over_zero_request"] < 1.0 for sample in samples)
    inside_rows = sum(sample["best_pressure"] <= 1.0 for sample in samples)
    edge_rows = sum(
        abs(sample["best_request"][0]) >= max(abs(value) for value in ROLL_REQUESTS)
        or abs(sample["best_request"][4])
        >= max(abs(value) for value in LATERAL_REQUESTS)
        or abs(sample["best_request"][2]) >= max(abs(value) for value in YAW_REQUESTS)
        for sample in samples
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "contract": {
            "horizon_s": HORIZON_S,
            "roll_bound_rad": ROLL_BOUND_RAD,
            "lateral_capture_bound_m": LATERAL_CAPTURE_BOUND_M,
            "candidate_count_per_snapshot": row_count,
            "roll_requests_rad_s2": ROLL_REQUESTS,
            "lateral_requests_m_s2": LATERAL_REQUESTS,
            "yaw_requests_rad_s2": YAW_REQUESTS,
            "policy_steps": 0,
            "plant_steps": 0,
        },
        "by_support": by_support,
        "classification": {
            "descent_rows": descent_rows,
            "inside_declared_boundary_after_best_forecast": inside_rows,
            "request_lattice_edge_rows": edge_rows,
        },
        "samples": samples,
    }
    report = "\n".join(
        [
            "# Bonesaw frozen-state local viability descent · r146",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. The source states come from retained MuJoCo boundaries, but this audit performs no policy step, external plant step, contact integration, or clock read. It makes {row_count} fixed WBC queries per state and applies one declared constant-acceleration forecast.",
            "",
            "## Outcome",
            "",
            "For each of 16 frozen states, the score is the larger of normalized roll-capture and lateral-capture magnitude after 250 ms. `zero` is the WBC response to a zero root request; `best` is the minimum over the fixed roll/lateral/yaw request lattice under the exact measured support mask. A ratio below one proves local forecast descent exists at that state. It does not prove that a causal controller can select, sustain, or realize the action through changing contact.",
            "",
            f"Descent exists in {descent_rows}/16 rows, but only {inside_rows}/16 best forecasts finish inside the declared unit boundary and {edge_rows}/16 select at least one request-lattice edge. The evidence therefore rejects a static-gain interpretation: the next candidate needs bounded finite-horizon response estimation, regularization/slew, an exact verification solve, and the external plant gate.",
            "",
            *markdown_table(
                ["case", "before fall s", "L/R", "now", "zero", "best", "best/zero", "request roll/lat/yaw", "status"],
                table_rows,
            ),
            "",
            "## Support summary",
            "",
            *markdown_table(
                ["support", "rows", "descent rows", "≤0.5 rows", "median best/zero", "worst best/zero"],
                [
                    [
                        name,
                        value["count"],
                        value["descent_rows"],
                        value["half_pressure_rows"],
                        f"{value['median_best_over_zero_request']:.3f}",
                        f"{value['maximum_best_over_zero_request']:.3f}",
                    ]
                    for name, value in by_support.items()
                ],
            ),
            "",
            "## Integrity gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Boundary",
            "",
            "- The score is a declared local forecast, not a viability certificate, fall probability, or aggregate authority verdict.",
            "- Root-origin lateral acceleration approximates CoM capture evolution over 250 ms; a promoted controller must replace this with an exact predicted state/contact program and then pass the external plant matrix.",
            "- Zero- and single-support descent may come from internal angular-momentum exchange and cannot create ground impulse. Contact return and command realization remain separate admissions.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-state-local-viability-descent-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_STATE_LOCAL_VIABILITY_DESCENT_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "by_support": by_support}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
