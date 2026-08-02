#!/usr/bin/env python3
"""Exercise typed dynamic trajectory admission without a policy or plant."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any, Callable

import numpy as np

from bonesaw import (
    DYNAMIC_ADMISSION_PREVIOUS_PLAN_EXPIRED,
    DYNAMIC_ADMISSION_PRIMARY_ACTUATOR_LIMIT,
    DYNAMIC_ADMISSION_PRIMARY_JOINT_POSITION_LIMIT,
    DynamicAdvanceSession,
)
from cpu_reference_report import distribution, markdown_table, render_report_html


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--output", default="benchmarks/results/dynamic-admission-r100")
    parser.add_argument("--web-report", default="web/DYNAMIC_ADMISSION_R100.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table(headers, rows))


def output_arrays(session: DynamicAdvanceSession) -> dict[str, np.ndarray]:
    ticks = 1
    dof = session.dof
    generalized = session.generalized_dof
    actuators = session.actuator_count
    samples = session.samples_per_tick
    return {
        "sample_position": np.empty((ticks, samples, actuators), np.float64),
        "sample_velocity": np.empty((ticks, samples, actuators), np.float64),
        "sample_acceleration": np.empty((ticks, samples, actuators), np.float64),
        "solved_acceleration": np.empty((ticks, generalized), np.float64),
        "splice_position": np.empty((ticks, actuators), np.float64),
        "splice_velocity": np.empty((ticks, actuators), np.float64),
        "splice_acceleration": np.empty((ticks, actuators), np.float64),
        "admitted_effort": np.empty((ticks, actuators), np.float64),
        "selected_extrema": np.empty((ticks, 3), np.float64),
        "selected_position_headroom": np.empty(ticks, np.float64),
        "admission_flags": np.empty(ticks, np.uint32),
        "primary_collision_distance": np.empty(ticks, np.float64),
        "contingency_collision_distance": np.empty(ticks, np.float64),
        "primary_continuous_clearance": np.empty(ticks, np.float64),
        "contingency_continuous_clearance": np.empty(ticks, np.float64),
        "primary_relative_speed_bound": np.empty(ticks, np.float64),
        "contingency_relative_speed_bound": np.empty(ticks, np.float64),
        "primary_collision_pair": np.empty(ticks, np.int64),
        "primary_collision_time_ns": np.empty(ticks, np.int64),
        "status": np.empty(ticks, np.uint8),
        "selection": np.empty(ticks, np.uint8),
        "solve_status": np.empty(ticks, np.uint8),
        "primary_valid": np.empty(ticks, np.uint8),
        "contingency_valid": np.empty(ticks, np.uint8),
        "previous_expired": np.empty(ticks, np.uint8),
        "dynamics_residual": np.empty(ticks, np.float64),
        "contact_residual": np.empty(ticks, np.float64),
        "command_divergence": np.empty(ticks, np.float64),
        "step_ns": np.empty(ticks, np.uint64),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
        "_observed_q": np.empty((ticks, dof), np.float64),
        "_observed_v": np.empty((ticks, dof), np.float64),
        "_root_twist": np.zeros((ticks, 6), np.float64),
        "_desired": np.empty((ticks, generalized), np.float64),
    }


def execute(
    session: DynamicAdvanceSession,
    start_time_ns: int,
    q: np.ndarray,
    v: np.ndarray,
    desired: np.ndarray,
) -> dict[str, np.ndarray]:
    out = output_arrays(session)
    out["_observed_q"][0] = q
    out["_observed_v"][0] = v
    out["_desired"][0] = desired
    session.run_trace(
        start_time_ns,
        out["_observed_q"],
        out["_observed_v"],
        out["_root_twist"],
        out["_desired"],
        out["sample_position"],
        out["sample_velocity"],
        out["sample_acceleration"],
        out["solved_acceleration"],
        out["splice_position"],
        out["splice_velocity"],
        out["splice_acceleration"],
        out["admitted_effort"],
        out["selected_extrema"],
        out["selected_position_headroom"],
        out["admission_flags"],
        out["primary_collision_distance"],
        out["contingency_collision_distance"],
        out["primary_continuous_clearance"],
        out["contingency_continuous_clearance"],
        out["primary_relative_speed_bound"],
        out["contingency_relative_speed_bound"],
        out["primary_collision_pair"],
        out["primary_collision_time_ns"],
        out["status"],
        out["selection"],
        out["solve_status"],
        out["primary_valid"],
        out["contingency_valid"],
        out["previous_expired"],
        out["dynamics_residual"],
        out["contact_residual"],
        out["command_divergence"],
        out["step_ns"],
        out["allocation_calls"],
        out["allocated_bytes"],
    )
    return out


def semantic_bytes(out: dict[str, np.ndarray]) -> bytes:
    excluded = {
        "step_ns",
        "allocation_calls",
        "allocated_bytes",
        "_observed_q",
        "_observed_v",
        "_root_twist",
        "_desired",
    }
    return b"".join(out[name].tobytes() for name in sorted(out.keys() - excluded))


def nominal_case(session: DynamicAdvanceSession) -> dict[str, np.ndarray]:
    q = np.zeros(session.dof, np.float64)
    v = np.zeros(session.dof, np.float64)
    desired = np.zeros(session.generalized_dof, np.float64)
    session.reset_command_state(q, v)
    return execute(session, 0, q, v, desired)


def position_limit_case(session: DynamicAdvanceSession) -> dict[str, np.ndarray]:
    q = np.zeros(session.dof, np.float64)
    v = np.zeros(session.dof, np.float64)
    desired = np.zeros(session.generalized_dof, np.float64)
    q[0] = 1.2595
    v[0] = 0.05
    desired[6] = 100.0
    session.reset_command_state(q, v)
    return execute(session, 0, q, v, desired)


def expired_plan_case(session: DynamicAdvanceSession) -> dict[str, np.ndarray]:
    q = np.zeros(session.dof, np.float64)
    v = np.zeros(session.dof, np.float64)
    desired = np.zeros(session.generalized_dof, np.float64)
    session.reset_command_state(q, v)
    execute(session, 0, q, v, desired)
    return execute(session, 50_000_001, q, v, desired)


def run_case(
    model: pathlib.Path,
    repeats: int,
    runner: Callable[[DynamicAdvanceSession], dict[str, np.ndarray]],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = DynamicAdvanceSession(
        str(model), ["left_wheel_center", "right_wheel_center"]
    )
    retained: dict[str, np.ndarray] | None = None
    retained_bytes: bytes | None = None
    replay_exact = True
    steps_us: list[float] = []
    maximum_allocation_calls = 0
    maximum_allocated_bytes = 0
    for _ in range(repeats):
        out = runner(session)
        current = semantic_bytes(out)
        if retained_bytes is None:
            retained = out
            retained_bytes = current
        else:
            replay_exact = replay_exact and current == retained_bytes
        steps_us.append(float(out["step_ns"][0]) / 1e3)
        maximum_allocation_calls = max(
            maximum_allocation_calls, int(out["allocation_calls"][0])
        )
        maximum_allocated_bytes = max(
            maximum_allocated_bytes, int(out["allocated_bytes"][0])
        )
    assert retained is not None
    timing = distribution(np.asarray(steps_us, np.float64))
    return (
        {
            "repeats": repeats,
            "selection": int(retained["selection"][0]),
            "status": int(retained["status"][0]),
            "solve_status": int(retained["solve_status"][0]),
            "admission_flags": int(retained["admission_flags"][0]),
            "primary_valid": bool(retained["primary_valid"][0]),
            "contingency_valid": bool(retained["contingency_valid"][0]),
            "previous_expired": bool(retained["previous_expired"][0]),
            "selected_minimum_position_headroom": float(
                retained["selected_position_headroom"][0]
            ),
            "maximum_admitted_effort_abs": float(
                np.max(np.abs(retained["admitted_effort"]))
            ),
            "maximum_hard_residual": float(
                max(retained["dynamics_residual"][0], retained["contact_residual"][0])
            ),
            "semantic_replay_exact": replay_exact,
            "maximum_allocation_calls": maximum_allocation_calls,
            "maximum_allocated_bytes": maximum_allocated_bytes,
            "step_us": timing,
        },
        retained,
    )


def evaluate(model: pathlib.Path, repeats: int) -> dict[str, Any]:
    cases: dict[str, dict[str, Any]] = {}
    retained: dict[str, dict[str, np.ndarray]] = {}
    for name, runner in (
        ("nominal", nominal_case),
        ("primary_position_limit", position_limit_case),
        ("expired_plan", expired_plan_case),
    ):
        cases[name], retained[name] = run_case(model, repeats, runner)
    limit = cases["primary_position_limit"]
    expired = cases["expired_plan"]
    nominal = cases["nominal"]
    passed = bool(
        nominal["selection"] == 0
        and nominal["status"] <= 1
        and nominal["admission_flags"] == 0
        and nominal["primary_valid"]
        and limit["selection"] == 1
        and limit["status"] == 2
        and not limit["primary_valid"]
        and limit["contingency_valid"]
        and limit["admission_flags"]
        & DYNAMIC_ADMISSION_PRIMARY_JOINT_POSITION_LIMIT
        and not (
            limit["admission_flags"] & DYNAMIC_ADMISSION_PRIMARY_ACTUATOR_LIMIT
        )
        and limit["selected_minimum_position_headroom"] >= 0.0
        and limit["maximum_admitted_effort_abs"] == 0.0
        and expired["selection"] == 1
        and expired["status"] == 2
        and expired["previous_expired"]
        and expired["admission_flags"] & DYNAMIC_ADMISSION_PREVIOUS_PLAN_EXPIRED
        and expired["maximum_admitted_effort_abs"] == 0.0
        and all(case["semantic_replay_exact"] for case in cases.values())
        and all(case["maximum_allocation_calls"] == 0 for case in cases.values())
        and all(case["maximum_allocated_bytes"] == 0 for case in cases.values())
        and all(case["maximum_hard_residual"] < 1e-7 for case in cases.values())
        and all(case["step_us"]["maximum"] < 20_000.0 for case in cases.values())
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free analytic dynamic command admission faults",
        "model": str(model),
        "cases": cases,
        "flag_contract": {
            "previous_plan_expired": DYNAMIC_ADMISSION_PREVIOUS_PLAN_EXPIRED,
            "primary_actuator_limit": DYNAMIC_ADMISSION_PRIMARY_ACTUATOR_LIMIT,
            "primary_joint_position_limit": DYNAMIC_ADMISSION_PRIMARY_JOINT_POSITION_LIMIT,
        },
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    cases = audit["cases"]
    rows = []
    for name, case in cases.items():
        timing = case["step_us"]
        rows.append(
            [
                name,
                case["selection"],
                case["status"],
                f"0x{case['admission_flags']:02x}",
                f"{case['primary_valid']} / {case['contingency_valid']}",
                case["selected_minimum_position_headroom"],
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{case['maximum_allocation_calls']} / {case['maximum_allocated_bytes']}",
            ]
        )
    return f"""# Bonesaw analytic dynamic admission faults · r100

## Outcome

**{audit['status'].upper()}.** R100 adds analytic position extrema to every quintic and validates the actuator polynomial after exact transmission mapping in generalized joint coordinates. Endpoint-safe but interior-unsafe motion is rejected. Admission emits composable fixed flags, so solver, expiry, actuator derivative, and joint-position causes do not collapse into one health score.

This corpus remains policy- and physics-free. It evaluates command safety and fallback selection; it does not claim the robot realized the command.

## Retained cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`; status `2` is contingency.

{table(['case', 'selection', 'status', 'flags', 'primary / contingency valid', 'selected position headroom', 'p50 / p99 / max µs', 'alloc calls / bytes'], rows)}

The near-limit case starts Upkie's left hip at `1.2595 rad` with `+0.05 rad/s` and asks for `+100 rad/s²`. Its strict dynamics/contact solve remains finite, but the primary quintic crosses the `1.26 rad` joint limit at an analytic interior extremum. Flag `{DYNAMIC_ADMISSION_PRIMARY_JOINT_POSITION_LIMIT:#x}` activates, primary is withheld, the braking contingency retains nonnegative position headroom, and admitted feed-forward effort is exactly zero.

The expiry case deliberately waits beyond the previous 20 ms plan. Flag `{DYNAMIC_ADMISSION_PREVIOUS_PLAN_EXPIRED:#x}` activates and independently valid braking is selected. All three cases replay semantic output bytes exactly across {cases['nominal']['repeats']} resets, allocate zero bytes inside the Rust transaction, satisfy hard dynamics/contact residuals below `1e-7`, and remain inside the 20 ms budget.

## Boundary and remaining work

This closes analytic joint-position admission for fixed-horizon segments, including coupled square transmissions. It does not yet provide continuous collision sweeps, effort/impedance samples, observation history/scene ingest, root-pose arrays in this eval adapter, calibrated plant realization, or a dynamically re-solved contingency. A command state already outside the viable set may still require rejection; this report does not disguise that as safe braking.
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
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-core/src/trajectory.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "dynamic-admission-r100",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "dynamic-admission-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "DYNAMIC_ADMISSION_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw analytic dynamic admission faults · r100")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
