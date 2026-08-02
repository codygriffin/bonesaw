#!/usr/bin/env python3
"""Policy-/physics-free robot-observation batch and reconstruction audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import RobotObservationHistorySession
from cpu_reference_report import distribution, markdown_table, render_report_html


REPORT_FIELDS = (
    "batch_sorted",
    "policy_valid",
    "inserted",
    "replaced_sequence",
    "replaced_source",
    "ignored_duplicate",
    "ignored_source",
    "rejected_epoch",
    "rejected_future",
    "rejected_stale",
    "rejected_uncertain",
    "rejected_invalid_policy",
    "rejected_invalid_state",
    "rejected_out_of_order",
    "rejected_unsorted",
)
PROVENANCE = {0: "exact", 1: "interpolated", 2: "predicted", 3: "held"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument(
        "--output", default="benchmarks/results/robot-observation-history-r116"
    )
    parser.add_argument(
        "--web-report", default="web/ROBOT_OBSERVATION_HISTORY_R116.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def session(model: pathlib.Path, capacity: int = 4) -> RobotObservationHistorySession:
    return RobotObservationHistorySession(str(model), capacity=capacity)


def batch(
    dof: int,
    mapped: list[int],
    sources: list[int],
    sequences: list[int],
    positions: list[float],
    *,
    epochs: list[int] | None = None,
    uncertainties: list[int] | None = None,
    velocities: list[float] | None = None,
) -> dict[str, np.ndarray]:
    count = len(mapped)
    roots = np.zeros((count, 7), dtype=np.float64)
    roots[:, 6] = 1.0
    q = np.zeros((count, dof), dtype=np.float64)
    v = np.zeros((count, dof), dtype=np.float64)
    q[:, 0] = positions
    if velocities is not None:
        v[:, 0] = velocities
    return {
        "epochs": np.asarray(epochs or [116] * count, dtype=np.uint64),
        "source_times": np.asarray(mapped, dtype=np.int64) + 7_000_000_000,
        "mapped": np.asarray(mapped, dtype=np.int64),
        "sequences": np.asarray(sequences, dtype=np.uint64),
        "sources": np.asarray(sources, dtype=np.uint64),
        "uncertainties": np.asarray(
            uncertainties or [200] * count, dtype=np.int64
        ),
        "roots": roots,
        "root_twists": np.zeros((count, 6), dtype=np.float64),
        "q": q,
        "v": v,
    }


def ingest(
    target: RobotObservationHistorySession,
    tick_time_ns: int,
    values: dict[str, np.ndarray],
) -> tuple[dict[str, int], tuple[int, int, int]]:
    report = np.zeros(len(REPORT_FIELDS), dtype=np.uint64)
    timing = target.ingest_batch(
        tick_time_ns,
        values["epochs"],
        values["source_times"],
        values["mapped"],
        values["sequences"],
        values["sources"],
        values["uncertainties"],
        values["roots"],
        values["root_twists"],
        values["q"],
        values["v"],
        report,
    )
    return dict(zip(REPORT_FIELDS, map(int, report), strict=True)), timing


def reconstruct(
    target: RobotObservationHistorySession, time_ns: int
) -> tuple[
    int,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    tuple[int, int, int],
]:
    root = np.zeros(7, dtype=np.float64)
    root_twist = np.zeros(6, dtype=np.float64)
    q = np.zeros(target.dof, dtype=np.float64)
    v = np.zeros(target.dof, dtype=np.float64)
    evidence = np.zeros(15, dtype=np.int64)
    status, elapsed, allocations, allocated_bytes = target.reconstruct_into(
        time_ns, root, root_twist, q, v, evidence
    )
    return (
        int(status),
        root,
        root_twist,
        q,
        v,
        evidence,
        (elapsed, allocations, allocated_bytes),
    )


def evaluate(model: pathlib.Path, repeats: int) -> dict[str, Any]:
    canonical = session(model, 2)
    canonical_batch = batch(
        canonical.dof,
        [10, 10, 20, 20],
        [9, 9, 3, 9],
        [1, 2, 1, 1],
        [0.1, 0.2, 0.3, 0.9],
    )
    canonical_report, _ = ingest(canonical, 20, canonical_batch)
    canonical_result = reconstruct(canonical, 20)

    chunked = session(model, 2)
    ingest(
        chunked,
        20,
        batch(chunked.dof, [10, 10], [9, 9], [1, 2], [0.1, 0.2]),
    )
    ingest(chunked, 20, batch(chunked.dof, [20], [9], [1], [0.9]))
    replacement_report, _ = ingest(
        chunked, 20, batch(chunked.dof, [20], [3], [1], [0.3])
    )
    chunked_result = reconstruct(chunked, 20)
    chunking_invariant = bool(
        canonical_result[0] == chunked_result[0] == 0
        and np.array_equal(canonical_result[3], chunked_result[3])
        and np.array_equal(canonical_result[5], chunked_result[5])
        and canonical_result[3][0] == 0.3
        and canonical_result[5][4] == 3
        and replacement_report["replaced_source"] == 1
    )

    unsorted = session(model)
    unsorted_report, _ = ingest(
        unsorted, 20, batch(unsorted.dof, [20, 10], [1, 1], [1, 1], [0.2, 0.1])
    )
    unsorted_atomic = bool(
        unsorted.len == 0
        and unsorted_report["batch_sorted"] == 0
        and unsorted_report["rejected_unsorted"] == 2
    )

    fault_specs = {
        "epoch": (20, batch(canonical.dof, [20], [1], [1], [0.0], epochs=[115]), "rejected_epoch"),
        "future": (20, batch(canonical.dof, [21], [1], [1], [0.0]), "rejected_future"),
        "stale": (50_000_001, batch(canonical.dof, [10_000_000], [1], [1], [0.0]), "rejected_stale"),
        "uncertain": (20, batch(canonical.dof, [20], [1], [1], [0.0], uncertainties=[2_000_001]), "rejected_uncertain"),
        "invalid_state": (20, batch(canonical.dof, [20], [1], [1], [float("nan")]), "rejected_invalid_state"),
    }
    fault_rows: list[dict[str, Any]] = []
    fault_gates = True
    for name, (tick, values, expected_field) in fault_specs.items():
        target = session(model)
        report, _ = ingest(target, tick, values)
        passed = report[expected_field] == 1 and target.len == 0
        fault_gates = fault_gates and passed
        fault_rows.append(
            {"name": name, "expected": expected_field, "report": report, "passed": passed}
        )

    reconstruction = session(model)
    reconstruction_batch = batch(
        reconstruction.dof,
        [0, 20_000_000],
        [2, 2],
        [1, 2],
        [0.0, 0.02],
        velocities=[1.0, 1.0],
    )
    reconstruction_batch["roots"][1, 0] = 0.02
    reconstruction_batch["root_twists"][:, 3] = 1.0
    ingest(
        reconstruction,
        20_000_000,
        reconstruction_batch,
    )
    reconstruction_rows = []
    for name, query, expected_status, expected_provenance, expected_q in (
        ("exact", 20_000_000, 0, 0, 0.02),
        ("interpolated", 10_000_000, 0, 1, 0.01),
        ("predicted", 25_000_000, 0, 2, 0.025),
    ):
        status, root, root_twist, q, _, evidence, timing = reconstruct(
            reconstruction, query
        )
        passed = bool(
            status == expected_status
            and evidence[0] == expected_provenance
            and abs(q[0] - expected_q) < 1e-12
            and abs(root[0] - expected_q) < 1e-12
            and abs(root_twist[3] - 1.0) < 1e-12
            and evidence[12] == 1
        )
        reconstruction_rows.append(
            {
                "name": name,
                "status": status,
                "provenance": PROVENANCE.get(int(evidence[0]), "error"),
                "q0": float(q[0]),
                "hard_eligible": bool(evidence[12]),
                "timing_ns": timing[0],
                "passed": passed,
            }
        )
    reconstruction.set_query_policy(100_000_000, 40_000_000, 40_000_000, 2_000_000, False, True)
    status, root, root_twist, q, _, evidence, timing = reconstruct(
        reconstruction, 25_000_000
    )
    held_passed = bool(status == 0 and evidence[0] == 3 and evidence[12] == 0 and q[0] == 0.02)
    reconstruction_rows.append(
        {"name": "held", "status": status, "provenance": PROVENANCE.get(int(evidence[0])), "q0": float(q[0]), "hard_eligible": bool(evidence[12]), "timing_ns": timing[0], "passed": held_passed}
    )
    reconstruction.set_query_policy(100_000_000, 40_000_000, 40_000_000, 2_000_000, True, False)
    too_old_status = reconstruct(reconstruction, 60_000_001)[0]
    reconstruction.set_query_policy(10_000_000, 40_000_000, 40_000_000, 2_000_000, True, False)
    gap_status = reconstruct(reconstruction, 10_000_000)[0]
    failure_gates = too_old_status == 5 and gap_status == 3

    wrapped = session(model, 2)
    ingest(
        wrapped,
        20,
        batch(wrapped.dof, [0, 10, 20], [1, 1, 1], [1, 2, 3], [0.0, 0.1, 0.2]),
    )
    wraparound_passed = bool(wrapped.len == 2 and reconstruct(wrapped, 0)[0] == 2)

    ingest_timings = np.empty(repeats, dtype=np.float64)
    query_timings = np.empty(repeats, dtype=np.float64)
    maximum_allocation_calls = 0
    maximum_allocated_bytes = 0
    retained_digest: bytes | None = None
    semantic_replay_exact = True
    replay = session(model, 2)
    for repeat in range(repeats):
        replay.clear()
        report, ingest_timing = ingest(replay, 20, canonical_batch)
        status, root, root_twist, q, v, evidence, query_timing = reconstruct(replay, 25)
        digest = hashlib.sha256(
            np.asarray(list(report.values()), dtype=np.uint64).tobytes()
            + bytes([status])
            + root.tobytes()
            + root_twist.tobytes()
            + q.tobytes()
            + v.tobytes()
            + evidence.tobytes()
        ).digest()
        if retained_digest is None:
            retained_digest = digest
        else:
            semantic_replay_exact = semantic_replay_exact and digest == retained_digest
        ingest_timings[repeat] = ingest_timing[0] / 1000.0
        query_timings[repeat] = query_timing[0] / 1000.0
        maximum_allocation_calls = max(
            maximum_allocation_calls, ingest_timing[1], query_timing[1]
        )
        maximum_allocated_bytes = max(
            maximum_allocated_bytes, ingest_timing[2], query_timing[2]
        )

    passed = bool(
        chunking_invariant
        and unsorted_atomic
        and fault_gates
        and all(row["passed"] for row in reconstruction_rows)
        and failure_gates
        and wraparound_passed
        and semantic_replay_exact
        and maximum_allocation_calls == 0
        and maximum_allocated_bytes == 0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free canonical observation history and reconstruction",
        "repeats": repeats,
        "capacity": 2,
        "preallocated_generalized_scalars": 2 * 2 * canonical.dof,
        "canonical_report": canonical_report,
        "chunking_invariant": chunking_invariant,
        "unsorted_batch_atomic": unsorted_atomic,
        "fault_cases": fault_rows,
        "reconstruction_cases": reconstruction_rows,
        "too_old_status": too_old_status,
        "interpolation_gap_status": gap_status,
        "wraparound_passed": wraparound_passed,
        "semantic_replay_exact": semantic_replay_exact,
        "maximum_allocation_calls": maximum_allocation_calls,
        "maximum_allocated_bytes": maximum_allocated_bytes,
        "ingest_timing_us": distribution(ingest_timings),
        "query_timing_us": distribution(query_timings),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    reconstruction_rows = [
        [
            row["name"],
            row["status"],
            row["provenance"],
            f"{row['q0']:.6f}",
            row["hard_eligible"],
            row["passed"],
        ]
        for row in audit["reconstruction_cases"]
    ]
    reconstruction_table = "\n".join(
        markdown_table(
            ["query", "status", "provenance", "q[0]", "hard eligible", "gate"],
            reconstruction_rows,
        )
    )
    fault_table = "\n".join(
        markdown_table(
            ["fault", "typed disposition", "gate"],
            [[row["name"], row["expected"], row["passed"]] for row in audit["fault_cases"]],
        )
    )
    ingest_timing = audit["ingest_timing_us"]
    query_timing = audit["query_timing_us"]
    return f"""# Bonesaw canonical observation history · r116

## Outcome

**{audit['status'].upper()}.** R116 adds a fixed-capacity Rust authority boundary between asynchronous robot observations and WBC state queries. Equal mapped timestamps resolve to the lowest stable source ID; within that source, the highest sequence wins. The same evidence is produced whether observations arrive as one sorted batch or as separately scheduled chunks: **{audit['chunking_invariant']}**. Unsorted batches reject atomically: **{audit['unsorted_batch_atomic']}**. Ring wraparound is bounded and deterministic: **{audit['wraparound_passed']}**.

{reconstruction_table}

Held state remains queryable but cannot authorize hard constraints. Interpolated and bounded constant-velocity predicted states retain source interval, source identity, sequence, age headroom, and synchronization headroom. Too-old and excessive interpolation-gap queries returned typed status `{audit['too_old_status']}` and `{audit['interpolation_gap_status']}`.

## Ingest fault matrix

{fault_table}

## Execution and memory evidence

Across {audit['repeats']} exact semantic replays, four-record ingest ran in `{ingest_timing['p50']:.3f}/{ingest_timing['p99']:.3f}/{ingest_timing['maximum']:.3f}` µs p50/p99/max; reconstruction ran in `{query_timing['p50']:.3f}/{query_timing['p99']:.3f}/{query_timing['maximum']:.3f}` µs. Semantic replay was exact: **{audit['semantic_replay_exact']}**. Timed allocation calls/bytes were `{audit['maximum_allocation_calls']}/{audit['maximum_allocated_bytes']}`. Capacity is fixed at construction and this corpus preallocated `{audit['preallocated_generalized_scalars']}` generalized q/v scalars across its two slots; no online push grows or shifts storage.

## Scope boundary

This audit has no policy, estimator, physics engine, or plant rollout. Python authors observations and statistics; Rust owns validation, canonical ingest, ring retention, manifold interpolation, bounded prediction, provenance, and timing. The next integration step is to make this reconstructed state—not an ad hoc latest sample—the only state accepted by the live WBC transaction.
"""


def main() -> None:
    args = parse_args()
    if args.repeats <= 1:
        raise SystemExit("--repeats must exceed one")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(model, args.repeats)
    sources = [
        model,
        pathlib.Path("crates/bonesaw-core/src/history.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "robot-observation-history-r116",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "robot-observation-history-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "ROBOT_OBSERVATION_HISTORY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw canonical observation history · r116")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
