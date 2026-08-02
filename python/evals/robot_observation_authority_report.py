#!/usr/bin/env python3
"""Policy-/physics-free robot-observation timing authority differential."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import (
    DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE,
    DYNAMIC_ADMISSION_ROBOT_OBSERVATION_STALE,
    DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN,
    DynamicAdvanceSession,
)
from cpu_reference_report import distribution, markdown_table, render_report_html


MAXIMUM_AGE_NS = 10_000_000
MAXIMUM_SYNC_UNCERTAINTY_NS = 2_000_000
SELECTION_NAMES = {0: "primary", 1: "contingency", 2: "rejected"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument(
        "--output", default="benchmarks/results/robot-observation-authority-r115"
    )
    parser.add_argument(
        "--web-report", default="web/ROBOT_OBSERVATION_AUTHORITY_R115.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def make_session(model: pathlib.Path, **overrides: int) -> DynamicAdvanceSession:
    return DynamicAdvanceSession(
        str(model),
        [],
        maximum_generalized_acceleration=100.0,
        maximum_generalized_effort=1000.0,
        maximum_robot_observation_age_ns=overrides.get(
            "maximum_age_ns", MAXIMUM_AGE_NS
        ),
        maximum_robot_observation_sync_uncertainty_ns=overrides.get(
            "maximum_sync_uncertainty_ns", MAXIMUM_SYNC_UNCERTAINTY_NS
        ),
    )


def semantic_bytes(value: Any) -> bytes:
    if isinstance(value, tuple):
        return b"".join(semantic_bytes(item) for item in value)
    if isinstance(value, bool):
        return bytes([int(value)])
    if isinstance(value, int):
        return int(value).to_bytes(8, "little", signed=value < 0)
    raise TypeError(type(value))


def evaluate(model: pathlib.Path, repeats: int) -> dict[str, Any]:
    cases = [
        ("exact", 0, 0, 0, "primary"),
        (
            "limits_inclusive",
            MAXIMUM_AGE_NS,
            MAXIMUM_SYNC_UNCERTAINTY_NS,
            0,
            "primary",
        ),
        (
            "stale_by_one_ns",
            MAXIMUM_AGE_NS + 1,
            0,
            DYNAMIC_ADMISSION_ROBOT_OBSERVATION_STALE,
            "rejected",
        ),
        (
            "future_by_one_ns",
            -1,
            0,
            DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE,
            "rejected",
        ),
        (
            "sync_uncertain_by_one_ns",
            0,
            MAXIMUM_SYNC_UNCERTAINTY_NS + 1,
            DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN,
            "rejected",
        ),
        (
            "negative_sync_uncertainty",
            0,
            -1,
            DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN,
            "rejected",
        ),
        (
            "future_and_sync_uncertain",
            -1,
            MAXIMUM_SYNC_UNCERTAINTY_NS + 1,
            DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE
            | DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN,
            "rejected",
        ),
    ]
    rows: list[dict[str, Any]] = []
    all_timing_us: list[float] = []
    maximum_allocation_calls = 0
    maximum_allocated_bytes = 0
    replay_exact = True
    tick_time_ns = 20_000_000
    for name, age_ns, uncertainty_ns, expected_flags, expected_selection in cases:
        session = make_session(model)
        q = np.zeros(session.dof, dtype=np.float64)
        v = np.zeros(session.dof, dtype=np.float64)
        root_twist = np.zeros(6, dtype=np.float64)
        desired = np.zeros(session.generalized_dof, dtype=np.float64)
        retained_digest: str | None = None
        retained: tuple[Any, ...] | None = None
        case_timing_us: list[float] = []
        mapped_time_ns = tick_time_ns - age_ns
        source_time_ns = 7_000_000_000 + mapped_time_ns
        for _ in range(repeats):
            session.reset_command_state(q, v)
            result = session.advance_observation(
                tick_time_ns,
                source_time_ns,
                mapped_time_ns,
                41,
                0xB015,
                uncertainty_ns,
                q,
                v,
                root_twist,
                desired,
            )
            decision, evidence, timing = result
            digest = hashlib.sha256(
                semantic_bytes(decision) + semantic_bytes(evidence)
            ).hexdigest()
            if retained_digest is None:
                retained_digest = digest
                retained = result
            else:
                replay_exact = replay_exact and digest == retained_digest
            elapsed_us = timing[0] / 1000.0
            case_timing_us.append(elapsed_us)
            all_timing_us.append(elapsed_us)
            maximum_allocation_calls = max(maximum_allocation_calls, timing[1])
            maximum_allocated_bytes = max(maximum_allocated_bytes, timing[2])
        assert retained is not None
        decision, evidence, _ = retained
        status, selection, flags, primary_valid, contingency_valid = decision
        (
            retained_source_time_ns,
            retained_mapped_time_ns,
            source_sequence,
            source_id,
            retained_uncertainty_ns,
            measured_age_ns,
            age_headroom_ns,
            sync_headroom_ns,
            causal,
            age_valid,
            synchronization_valid,
        ) = evidence
        rows.append(
            {
                "name": name,
                "source_time_ns": retained_source_time_ns,
                "mapped_time_ns": retained_mapped_time_ns,
                "source_sequence": source_sequence,
                "source_id": source_id,
                "synchronization_uncertainty_ns": retained_uncertainty_ns,
                "age_ns": measured_age_ns,
                "age_headroom_ns": age_headroom_ns,
                "synchronization_headroom_ns": sync_headroom_ns,
                "causal": causal,
                "age_valid": age_valid,
                "synchronization_valid": synchronization_valid,
                "status_code": status,
                "selection": SELECTION_NAMES[selection],
                "expected_selection": expected_selection,
                "flags": flags,
                "expected_observation_flags": expected_flags,
                "primary_valid": primary_valid,
                "contingency_valid": contingency_valid,
                "timing_us": distribution(np.asarray(case_timing_us)),
            }
        )

    invalid_configuration_rejected = True
    for overrides in (
        {"maximum_age_ns": -1},
        {"maximum_sync_uncertainty_ns": -1},
    ):
        try:
            make_session(model, **overrides)
        except ValueError:
            pass
        else:
            invalid_configuration_rejected = False

    observation_mask = (
        DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE
        | DYNAMIC_ADMISSION_ROBOT_OBSERVATION_STALE
        | DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN
    )
    passed = bool(
        invalid_configuration_rejected
        and replay_exact
        and maximum_allocation_calls == 0
        and maximum_allocated_bytes == 0
        and all(row["selection"] == row["expected_selection"] for row in rows)
        and all(
            row["flags"] & observation_mask == row["expected_observation_flags"]
            for row in rows
        )
        and all(
            row["primary_valid"] and row["contingency_valid"]
            for row in rows[:2]
        )
        and all(
            not row["primary_valid"] and not row["contingency_valid"]
            for row in rows[2:]
        )
        and rows[1]["age_headroom_ns"] == 0
        and rows[1]["synchronization_headroom_ns"] == 0
        and rows[2]["age_headroom_ns"] == -1
        and rows[4]["synchronization_headroom_ns"] == -1
        and rows[-1]["flags"] & observation_mask
        == DYNAMIC_ADMISSION_ROBOT_OBSERVATION_FUTURE
        | DYNAMIC_ADMISSION_ROBOT_OBSERVATION_UNCERTAIN
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free robot-observation timing authority",
        "repeats_per_case": repeats,
        "limits": {
            "maximum_age_ns": MAXIMUM_AGE_NS,
            "maximum_synchronization_uncertainty_ns": MAXIMUM_SYNC_UNCERTAINTY_NS,
        },
        "cases": rows,
        "invalid_configuration_rejected": invalid_configuration_rejected,
        "semantic_replay_exact": replay_exact,
        "maximum_allocation_calls": maximum_allocation_calls,
        "maximum_allocated_bytes": maximum_allocated_bytes,
        "timing_us": distribution(np.asarray(all_timing_us)),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = [
        [
            case["name"],
            case["age_ns"],
            case["age_headroom_ns"],
            case["synchronization_uncertainty_ns"],
            case["synchronization_headroom_ns"],
            f"{case['causal']}/{case['age_valid']}/{case['synchronization_valid']}",
            case["selection"],
            f"0x{case['flags']:x}",
        ]
        for case in audit["cases"]
    ]
    table = "\n".join(
        markdown_table(
            [
                "case",
                "age ns",
                "age headroom ns",
                "sync uncertainty ns",
                "sync headroom ns",
                "causal/age/sync",
                "selection",
                "flags",
            ],
            rows,
        )
    )
    timing = audit["timing_us"]
    return f"""# Bonesaw robot-observation authority · r115

## Outcome

**{audit['status'].upper()}.** R115 makes robot-state timing an explicit authority boundary. Every transaction retains the producer timestamp, mapped monotonic control timestamp, stable source identity, sequence, and synchronization uncertainty. Rust reads no clock: it derives signed age only from the caller's tick and mapped timestamp. Future, stale, and synchronization-uncertain causes remain independently visible, and any invalid observation withholds both Primary and brake. This is observation admission only—there is no policy, estimator, physics engine, state integration, or plant rollout.

{table}

Limits are inclusive: age `{MAXIMUM_AGE_NS}` ns and synchronization uncertainty `{MAXIMUM_SYNC_UNCERTAINTY_NS}` ns remain admitted with exactly zero headroom. A one-nanosecond breach is rejected with `-1` ns headroom. The combined case retains both failure bits rather than collapsing them into one health score.

## Execution evidence

Across {len(audit['cases']) * audit['repeats_per_case']} retained transactions, timing was `{timing['p50']:.3f}/{timing['p99']:.3f}/{timing['maximum']:.3f}` µs p50/p99/max. Semantic replay was exact: **{audit['semantic_replay_exact']}**. Timed allocation calls/bytes were `{audit['maximum_allocation_calls']}/{audit['maximum_allocated_bytes']}`. Negative configured limits are rejected: **{audit['invalid_configuration_rejected']}**.

## Authority boundary

The shell remains responsible for mapping ROS, sensor, and wall clocks into the monotonic control domain and for reconstructing state at the tick. R115 proves that the command transaction no longer silently treats unstamped, future, stale, or insufficiently synchronized state as authoritative. It does not yet merge sorted observation batches into `RobotHistory`, resolve duplicate source/timestamp samples in the full controller transaction, or attach a conservative configuration-error bound to reconstructed state.
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
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "robot-observation-authority-r115",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "robot-observation-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "ROBOT_OBSERVATION_AUTHORITY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw robot-observation authority · r115")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
