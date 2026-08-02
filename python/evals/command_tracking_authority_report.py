#!/usr/bin/env python3
"""Policy-/physics-free observed-versus-commanded authority differential."""

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
    DYNAMIC_ADMISSION_COMMAND_TRACKING_CONTINGENCY,
    DYNAMIC_ADMISSION_COMMAND_TRACKING_REJECTED,
    DynamicAdvanceSession,
)
from cpu_reference_report import distribution, markdown_table, render_report_html
from dynamic_admission_fault_report import execute, semantic_bytes


POSITION_CONTINGENCY = 0.01
POSITION_REJECT = 0.03
VELOCITY_CONTINGENCY = 0.2
VELOCITY_REJECT = 0.5
ACTION_NAMES = {0: "nominal", 1: "contingency", 2: "rejected"}
SELECTION_NAMES = {0: "primary", 1: "contingency", 2: "rejected"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=2000)
    parser.add_argument(
        "--output", default="benchmarks/results/command-tracking-authority-r114"
    )
    parser.add_argument(
        "--web-report", default="web/COMMAND_TRACKING_AUTHORITY_R114.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def make_session(model: pathlib.Path) -> DynamicAdvanceSession:
    return DynamicAdvanceSession(
        str(model),
        [],
        maximum_generalized_acceleration=100.0,
        maximum_generalized_effort=1000.0,
        command_tracking_position_contingency=POSITION_CONTINGENCY,
        command_tracking_position_reject=POSITION_REJECT,
        command_tracking_velocity_contingency=VELOCITY_CONTINGENCY,
        command_tracking_velocity_reject=VELOCITY_REJECT,
    )


def run_case(
    session: DynamicAdvanceSession,
    position_error: float,
    velocity_error: float,
) -> tuple[dict[str, np.ndarray], tuple[Any, ...]]:
    q = np.zeros(session.dof, dtype=np.float64)
    v = np.zeros(session.dof, dtype=np.float64)
    desired = np.zeros(session.generalized_dof, dtype=np.float64)
    session.reset_command_state(q, v)
    execute(session, 0, q, v, desired)
    observed_q = q.copy()
    observed_v = v.copy()
    observed_q[0] = position_error
    observed_v[0] = velocity_error
    out = execute(session, 20_000_000, observed_q, observed_v, desired)
    return out, session.command_tracking_evidence()


def evaluate(model: pathlib.Path, repeats: int) -> dict[str, Any]:
    cases = [
        ("nominal", 0.005, 0.10, 0, 0),
        ("position_contingency", 0.020, 0.10, 1, 1),
        ("position_rejected", 0.040, 0.10, 2, 2),
        ("velocity_contingency", 0.005, 0.30, 1, 1),
        ("velocity_rejected", 0.005, 0.60, 2, 2),
    ]
    rows: list[dict[str, Any]] = []
    all_timing_us: list[float] = []
    maximum_allocation_calls = 0
    maximum_allocated_bytes = 0
    replay_exact = True
    session = make_session(model)
    for name, position_error, velocity_error, expected_action, expected_selection in cases:
        retained_digest: str | None = None
        retained_out: dict[str, np.ndarray] | None = None
        retained_evidence: tuple[Any, ...] | None = None
        case_timing_us: list[float] = []
        for _ in range(repeats):
            out, evidence = run_case(session, position_error, velocity_error)
            digest = hashlib.sha256(semantic_bytes(out) + np.asarray(evidence).tobytes()).hexdigest()
            if retained_digest is None:
                retained_digest = digest
                retained_out = out
                retained_evidence = evidence
            else:
                replay_exact = replay_exact and digest == retained_digest
            elapsed_us = float(out["step_ns"][0]) / 1000.0
            case_timing_us.append(elapsed_us)
            all_timing_us.append(elapsed_us)
            maximum_allocation_calls = max(
                maximum_allocation_calls, int(out["allocation_calls"][0])
            )
            maximum_allocated_bytes = max(
                maximum_allocated_bytes, int(out["allocated_bytes"][0])
            )
        assert retained_out is not None and retained_evidence is not None
        (
            measured_position,
            measured_velocity,
            position_actuator,
            velocity_actuator,
            position_contingency_headroom,
            position_reject_headroom,
            velocity_contingency_headroom,
            velocity_reject_headroom,
            action,
        ) = retained_evidence
        flags = int(retained_out["admission_flags"][0])
        selection = int(retained_out["selection"][0])
        rows.append(
            {
                "name": name,
                "input_position_error": position_error,
                "input_velocity_error": velocity_error,
                "measured_position_error": measured_position,
                "measured_velocity_error": measured_velocity,
                "limiting_position_actuator": position_actuator,
                "limiting_velocity_actuator": velocity_actuator,
                "position_contingency_headroom": position_contingency_headroom,
                "position_reject_headroom": position_reject_headroom,
                "velocity_contingency_headroom": velocity_contingency_headroom,
                "velocity_reject_headroom": velocity_reject_headroom,
                "action": ACTION_NAMES[action],
                "expected_action": ACTION_NAMES[expected_action],
                "selection": SELECTION_NAMES[selection],
                "expected_selection": SELECTION_NAMES[expected_selection],
                "flags": flags,
                "primary_valid": bool(retained_out["primary_valid"][0]),
                "contingency_valid": bool(retained_out["contingency_valid"][0]),
                "maximum_admitted_effort_abs": float(
                    np.max(np.abs(retained_out["admitted_effort"]))
                ),
                "timing_us": distribution(np.asarray(case_timing_us)),
            }
        )

    invalid_config_rejected = False
    try:
        DynamicAdvanceSession(
            str(model),
            [],
            command_tracking_position_contingency=0.03,
            command_tracking_position_reject=0.01,
        )
    except ValueError:
        invalid_config_rejected = True

    nominal, *non_nominal = rows
    passed = bool(
        invalid_config_rejected
        and replay_exact
        and maximum_allocation_calls == 0
        and maximum_allocated_bytes == 0
        and all(row["action"] == row["expected_action"] for row in rows)
        and all(row["selection"] == row["expected_selection"] for row in rows)
        and nominal["flags"]
        & (
            DYNAMIC_ADMISSION_COMMAND_TRACKING_CONTINGENCY
            | DYNAMIC_ADMISSION_COMMAND_TRACKING_REJECTED
        )
        == 0
        and nominal["primary_valid"]
        and all(
            row["maximum_admitted_effort_abs"] == 0.0 for row in non_nominal
        )
        and all(
            row["flags"] & DYNAMIC_ADMISSION_COMMAND_TRACKING_CONTINGENCY
            for row in rows
            if row["action"] == "contingency"
        )
        and all(
            row["flags"] & DYNAMIC_ADMISSION_COMMAND_TRACKING_REJECTED
            for row in rows
            if row["action"] == "rejected"
        )
        and all(
            row["contingency_valid"] for row in rows if row["action"] == "contingency"
        )
        and all(
            not row["primary_valid"] for row in rows if row["action"] != "nominal"
        )
        and all(
            not row["contingency_valid"] for row in rows if row["action"] == "rejected"
        )
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free observed-versus-commanded command authority",
        "repeats_per_case": repeats,
        "thresholds": {
            "position_contingency": POSITION_CONTINGENCY,
            "position_reject": POSITION_REJECT,
            "velocity_contingency": VELOCITY_CONTINGENCY,
            "velocity_reject": VELOCITY_REJECT,
        },
        "cases": rows,
        "invalid_config_rejected": invalid_config_rejected,
        "semantic_replay_exact": replay_exact,
        "maximum_allocation_calls": maximum_allocation_calls,
        "maximum_allocated_bytes": maximum_allocated_bytes,
        "timing_us": distribution(np.asarray(all_timing_us)),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = []
    for case in audit["cases"]:
        rows.append(
            [
                case["name"],
                f"{case['measured_position_error']:.3f} / {case['measured_velocity_error']:.3f}",
                case["action"],
                case["selection"],
                f"0x{case['flags']:x}",
                f"{case['position_contingency_headroom']:.3f} / {case['position_reject_headroom']:.3f}",
                f"{case['velocity_contingency_headroom']:.3f} / {case['velocity_reject_headroom']:.3f}",
            ]
        )
    timing = audit["timing_us"]
    table = "\n".join(
        markdown_table(
            [
                "case",
                "position / velocity error",
                "tracking action",
                "selection",
                "flags",
                "position contingency / reject headroom",
                "velocity contingency / reject headroom",
            ],
            rows,
        )
    )
    return f"""# Bonesaw observed-versus-commanded authority · r114

## Outcome

**{audit['status'].upper()}.** R114 turns the existing command-divergence diagnostic into a two-stage authority envelope. The observed joint state is mapped through the exact actuation transform, then compared with the commanded splice position and velocity. Crossing a contingency threshold withholds feed-forward effort and selects the independently validated brake. Crossing a reject threshold withholds both plans. This evaluates a controller contract only; it introduces no policy, physics engine, state integration, or claim that the plant follows the command.

{table}

The configured position thresholds are `{POSITION_CONTINGENCY:.3f}/{POSITION_REJECT:.3f}` actuator units and velocity thresholds are `{VELOCITY_CONTINGENCY:.3f}/{VELOCITY_REJECT:.3f}` actuator units/s. Headroom remains signed and continuous on both sides of each decision. Limiting actuator indices are retained independently for position and velocity.

## Execution evidence

Across {len(audit['cases']) * audit['repeats_per_case']} retained transactions, timing was `{timing['p50']:.3f}/{timing['p99']:.3f}/{timing['maximum']:.3f}` µs p50/p99/max. Semantic replay was exact: **{audit['semantic_replay_exact']}**. Timed allocation calls/bytes were `{audit['maximum_allocation_calls']}/{audit['maximum_allocated_bytes']}`. An inverted contingency/reject envelope is rejected at construction: **{audit['invalid_config_rejected']}**.

## Authority boundary

This is direct evidence that observation and command have diverged, not an explanation of why. Torque saturation, bandwidth, delay, contact loss, calibration error, thermal derating, and mechanical failure remain separately typed causes. A warning selects braking but does not certify that the physical body will realize that brake; a hard breach rejects because command-space geometry is no longer an adequate witness for the observed mechanism state.
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
        "revision": "command-tracking-authority-r114",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "command-tracking-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "COMMAND_TRACKING_AUTHORITY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw observed-versus-commanded authority · r114")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
