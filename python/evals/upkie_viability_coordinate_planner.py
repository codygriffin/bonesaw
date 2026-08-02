#!/usr/bin/env python3
"""Bounded state-local coordinate planner over exact Rust WBC queries."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_truth_authority import collect_snapshots, query
from upkie_state_local_authority import CONTACT_FRAMES, STATUS_NAMES
from upkie_state_local_viability_descent import (
    HORIZON_S,
    LATERAL_REQUESTS,
    REVISION as ORACLE_REVISION,
    ROLL_REQUESTS,
    YAW_REQUESTS,
    pressure,
)


REVISION = "upkie-viability-coordinate-planner-r147"
ACTIVATION_PRESSURE = 0.10
PASSES = 2
AXIS_ORDER = (2, 0, 1)  # yaw, roll, lateral
VALUES = {0: ROLL_REQUESTS, 1: LATERAL_REQUESTS, 2: YAW_REQUESTS}
GENERALIZED_INDEX = {0: 0, 1: 4, 2: 2}
LIMITS = np.asarray(
    [max(abs(value) for value in ROLL_REQUESTS),
     max(abs(value) for value in LATERAL_REQUESTS),
     max(abs(value) for value in YAW_REQUESTS)],
    np.float64,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--oracle",
        default="benchmarks/results/upkie-state-local-viability-descent-r146/upkie-state-local-viability-descent-metrics.json",
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_VIABILITY_COORDINATE_PLANNER_R147.html"
    )
    return parser.parse_args()


def request_row(
    snapshot_index: int, snapshot: dict[str, Any], request: np.ndarray
) -> dict[str, Any]:
    generalized = np.zeros(6, np.float64)
    generalized[[0, 4, 2]] = request
    return {
        "snapshot_index": snapshot_index,
        "contact_source": "measured",
        "contact": snapshot["measured_contact"],
        "variant": "coordinate_planner",
        "request": generalized,
    }


def make_session(bonesaw: Any, model: pathlib.Path) -> tuple[Any, np.ndarray]:
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
    names = list(session.frame_names)
    frame_ids = np.asarray([names.index(name) for name in CONTACT_FRAMES], np.int64)
    return session, frame_ids


def run_planner(
    session: Any,
    frame_ids: np.ndarray,
    snapshots: list[dict[str, Any]],
) -> dict[str, Any]:
    started = time.perf_counter_ns()
    zero_rows = [
        request_row(index, snapshot, np.zeros(3, np.float64))
        for index, snapshot in enumerate(snapshots)
    ]
    zero_output = query(session, frame_ids, snapshots, zero_rows)
    zero_pressure = np.asarray(
        [
            pressure(snapshot, zero_output["generalized_acceleration"][index, :6], HORIZON_S)[
                "maximum_pressure"
            ]
            for index, snapshot in enumerate(snapshots)
        ]
    )
    active = np.flatnonzero(zero_pressure >= ACTIVATION_PRESSURE)
    requests = np.zeros((len(snapshots), 3), np.float64)
    rust_step_ns = np.asarray(zero_output["step_ns"], np.uint64).copy()
    query_count = np.ones(len(snapshots), np.uint16)
    allocation_calls = int(np.sum(zero_output["allocation_calls"]))
    allocated_bytes = int(np.sum(zero_output["allocated_bytes"]))

    for _ in range(PASSES):
        for axis in AXIS_ORDER:
            values = VALUES[axis]
            rows = []
            for snapshot_index in active:
                for value in values:
                    candidate = requests[snapshot_index].copy()
                    candidate[axis] = value
                    rows.append(
                        request_row(
                            int(snapshot_index), snapshots[snapshot_index], candidate
                        )
                    )
            output = query(session, frame_ids, snapshots, rows)
            allocation_calls += int(np.sum(output["allocation_calls"]))
            allocated_bytes += int(np.sum(output["allocated_bytes"]))
            for active_index, snapshot_index in enumerate(active):
                start = active_index * len(values)
                stop = start + len(values)
                candidates = [
                    pressure(
                        snapshots[snapshot_index],
                        output["generalized_acceleration"][tick, :6],
                        HORIZON_S,
                    )["maximum_pressure"]
                    for tick in range(start, stop)
                ]
                requests[snapshot_index, axis] = values[int(np.argmin(candidates))]
                rust_step_ns[snapshot_index] += int(
                    np.sum(output["step_ns"][start:stop])
                )
                query_count[snapshot_index] += len(values)

    verify_rows = [
        request_row(int(index), snapshots[index], requests[index]) for index in active
    ]
    verify_output = query(session, frame_ids, snapshots, verify_rows)
    allocation_calls += int(np.sum(verify_output["allocation_calls"]))
    allocated_bytes += int(np.sum(verify_output["allocated_bytes"]))
    generalized = np.asarray(zero_output["generalized_acceleration"]).copy()
    status = np.asarray(zero_output["status"]).copy()
    hard_violation = np.asarray(zero_output["maximum_constraint_violation"]).copy()
    torque_utilization = np.asarray(zero_output["maximum_torque_utilization"]).copy()
    for active_index, snapshot_index in enumerate(active):
        generalized[snapshot_index] = verify_output["generalized_acceleration"][active_index]
        status[snapshot_index] = verify_output["status"][active_index]
        hard_violation[snapshot_index] = verify_output["maximum_constraint_violation"][active_index]
        torque_utilization[snapshot_index] = verify_output["maximum_torque_utilization"][active_index]
        rust_step_ns[snapshot_index] += int(verify_output["step_ns"][active_index])
        query_count[snapshot_index] += 1
    candidate_pressure = np.asarray(
        [
            pressure(snapshot, generalized[index, :6], HORIZON_S)["maximum_pressure"]
            for index, snapshot in enumerate(snapshots)
        ]
    )
    return {
        "requests": requests,
        "zero_pressure": zero_pressure,
        "candidate_pressure": candidate_pressure,
        "generalized_acceleration": generalized,
        "status": status,
        "hard_violation": hard_violation,
        "torque_utilization": torque_utilization,
        "active": active,
        "rust_step_ns": rust_step_ns,
        "query_count": query_count,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
        "wall_ns": time.perf_counter_ns() - started,
    }


def distribution(values: np.ndarray) -> dict[str, float]:
    return {
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
    }


def main() -> None:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    oracle_path = pathlib.Path(args.oracle)
    oracle = json.loads(oracle_path.read_text())
    if oracle.get("revision") != ORACLE_REVISION:
        raise SystemExit(f"expected oracle revision {ORACLE_REVISION!r}")
    snapshots = collect_snapshots(model)
    session, frame_ids = make_session(bonesaw, model)
    candidate = run_planner(session, frame_ids, snapshots)
    replay = run_planner(session, frame_ids, snapshots)
    oracle_pressure = np.asarray(
        [sample["best_pressure"] for sample in oracle["samples"]], np.float64
    )
    samples = []
    for index, snapshot in enumerate(snapshots):
        request = candidate["requests"][index]
        samples.append(
            {
                "case": snapshot["case"],
                "offset_s": float(snapshot["offset_s"]),
                "measured_contact": [int(value) for value in snapshot["measured_contact"]],
                "active": bool(index in candidate["active"]),
                "query_count": int(candidate["query_count"][index]),
                "zero_pressure": float(candidate["zero_pressure"][index]),
                "candidate_pressure": float(candidate["candidate_pressure"][index]),
                "oracle_pressure": float(oracle_pressure[index]),
                "candidate_over_zero": float(
                    candidate["candidate_pressure"][index]
                    / max(candidate["zero_pressure"][index], 1.0e-12)
                ),
                "candidate_over_oracle": float(
                    candidate["candidate_pressure"][index]
                    / max(oracle_pressure[index], 1.0e-12)
                ),
                "request_roll_lateral_yaw": request.tolist(),
                "request_at_edge": bool(np.any(np.isclose(np.abs(request), LIMITS))),
                "achieved": candidate["generalized_acceleration"][index, :6].tolist(),
                "status": STATUS_NAMES[int(candidate["status"][index])],
                "hard_violation": float(candidate["hard_violation"][index]),
                "torque_utilization": float(candidate["torque_utilization"][index]),
                "rust_query_time_us": float(candidate["rust_step_ns"][index]) / 1.0e3,
            }
        )
    active_samples = [sample for sample in samples if sample["active"]]
    inactive_samples = [sample for sample in samples if not sample["active"]]
    exact_replay = all(
        np.array_equal(candidate[field], replay[field])
        for field in (
            "requests",
            "zero_pressure",
            "candidate_pressure",
            "generalized_acceleration",
            "status",
            "hard_violation",
            "torque_utilization",
            "active",
            "query_count",
        )
    )
    gates = {
        "matrix_complete": len(samples) == 16,
        "inactive_rows_emit_zero_request": all(
            sample["request_roll_lateral_yaw"] == [0.0, 0.0, 0.0]
            for sample in inactive_samples
        ),
        "every_active_row_has_strict_descent": all(
            sample["candidate_pressure"] < sample["zero_pressure"]
            for sample in active_samples
        ),
        "bounded_query_count": all(
            sample["query_count"] == 40 for sample in active_samples
        )
        and all(sample["query_count"] == 1 for sample in inactive_samples),
        "verified_candidates_hard_feasible": all(
            sample["status"] in ("Solved", "SolvedWithSlack")
            and sample["hard_violation"] <= 1.0e-8
            for sample in samples
        ),
        "exact_replay": exact_replay,
        "zero_rust_allocation": candidate["allocation_calls"] == 0
        and candidate["allocated_bytes"] == 0,
        "finite": all(
            math.isfinite(value)
            for sample in samples
            for value in (
                sample["zero_pressure"],
                sample["candidate_pressure"],
                sample["oracle_pressure"],
                sample["rust_query_time_us"],
            )
        ),
    }
    passed = all(gates.values())
    rust_timing = distribution(np.asarray(candidate["rust_step_ns"], np.float64) / 1.0e3)
    table_rows = [
        [
            sample["case"],
            f"{sample['offset_s']:.2f}",
            "".join(str(value) for value in sample["measured_contact"]),
            "YES" if sample["active"] else "NO",
            sample["query_count"],
            f"{sample['zero_pressure']:.3f}",
            f"{sample['candidate_pressure']:.3f}",
            f"{sample['oracle_pressure']:.3f}",
            f"{sample['candidate_over_oracle']:.3f}",
            f"{sample['request_roll_lateral_yaw'][0]:+.0f}/{sample['request_roll_lateral_yaw'][1]:+.0f}/{sample['request_roll_lateral_yaw'][2]:+.0f}",
            f"{sample['rust_query_time_us']:.1f}",
        ]
        for sample in samples
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "oracle_revision": ORACLE_REVISION,
        "passed": passed,
        "gates": gates,
        "contract": {
            "activation_pressure": ACTIVATION_PRESSURE,
            "passes": PASSES,
            "axis_order": ["yaw", "roll", "lateral"],
            "active_query_count": 40,
            "inactive_query_count": 1,
            "horizon_s": HORIZON_S,
            "plant_steps": 0,
        },
        "summary": {
            "active_rows": len(active_samples),
            "inactive_rows": len(inactive_samples),
            "inside_unit_boundary_rows": sum(
                sample["candidate_pressure"] <= 1.0 for sample in samples
            ),
            "request_edge_rows": sum(sample["request_at_edge"] for sample in samples),
            "oracle_exact_rows": sum(
                math.isclose(
                    sample["candidate_pressure"], sample["oracle_pressure"], abs_tol=1.0e-12
                )
                for sample in samples
            ),
            "median_candidate_over_oracle": float(
                np.median([sample["candidate_over_oracle"] for sample in active_samples])
            ),
            "rust_query_time_us": rust_timing,
            "offline_corpus_wall_ms": candidate["wall_ns"] / 1.0e6,
        },
        "samples": samples,
    }
    report = "\n".join(
        [
            "# Bonesaw bounded viability coordinate planner · r147",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. This is a frozen-state action selector with no external plant step or contact integration. It is not live or physically promoted.",
            "",
            "## Outcome",
            "",
            "A 0.10 pressure gate keeps near-equilibrium rows at zero request. Active rows run two deterministic yaw→roll→lateral coordinate passes over the r145 lattice and one exact verification solve: 40 fixed Rust WBC queries per planner event. Every active row strictly improves the 250 ms local forecast, every inactive row stays zero, every verified solve is hard-feasible, exact replay passes, and the Rust hot path allocates nothing.",
            "",
            f"The compact search finishes inside the unit boundary in {metrics['summary']['inside_unit_boundary_rows']}/16 rows, exactly matches the 245-query oracle in {metrics['summary']['oracle_exact_rows']}/16, and has active-row median candidate/oracle pressure {metrics['summary']['median_candidate_over_oracle']:.3f}. It still selects a lattice edge in {metrics['summary']['request_edge_rows']}/16 rows. This admits the bounded search mechanism, not its command smoothness or plant consequence.",
            "",
            *markdown_table(
                ["case", "before fall s", "L/R", "active", "queries", "zero", "candidate", "oracle", "cand/oracle", "request r/l/y", "Rust µs"],
                table_rows,
            ),
            "",
            "## Integrity gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Timing",
            "",
            *markdown_table(
                ["scope", "p50 µs", "p95 µs", "p99 µs", "max µs"],
                [["summed Rust queries per frozen state", f"{rust_timing['p50']:.1f}", f"{rust_timing['p95']:.1f}", f"{rust_timing['p99']:.1f}", f"{rust_timing['maximum']:.1f}"]],
            ),
            "",
            "## Boundary",
            "",
            "- The discrete request can jump between lattice edges. A live candidate needs Rust-owned activation hysteresis, request slew, a fixed update cadence, and a fresh exact verification solve each control tick.",
            "- Timing sums the retained per-query Rust clocks on this host; Python batching and a target-hardware tail certificate remain separate.",
            "- The next gate must execute a smoothed version through the external plant matrix. Failure there leaves r137 live behavior unchanged.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-viability-coordinate-planner-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_VIABILITY_COORDINATE_PLANNER_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "summary": metrics["summary"]}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
