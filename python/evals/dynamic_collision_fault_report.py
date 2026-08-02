#!/usr/bin/env python3
"""Retain dynamic segment self-collision admission evidence without a plant."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import DYNAMIC_ADMISSION_PRIMARY_COLLISION, DynamicAdvanceSession
from cpu_reference_report import distribution, markdown_table, render_report_html
from dynamic_admission_fault_report import execute, semantic_bytes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument("--output", default="benchmarks/results/dynamic-collision-r101")
    parser.add_argument("--web-report", default="web/DYNAMIC_COLLISION_R101.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table(headers, rows))


def run_case(
    model: pathlib.Path,
    repeats: int,
    clearance: float | None,
    q_value: float,
    velocity: float,
    acceleration: float,
    certify_between_samples: bool = False,
    collision_max_subdivision_depth: int = 0,
) -> dict[str, Any]:
    kwargs = (
        {}
        if clearance is None
        else {
            "self_collision_clearance": clearance,
            "certify_collision_between_samples": certify_between_samples,
            "collision_max_subdivision_depth": collision_max_subdivision_depth,
        }
    )
    session = DynamicAdvanceSession(str(model), [], **kwargs)
    q = np.asarray([q_value], np.float64)
    v = np.asarray([velocity], np.float64)
    desired = np.zeros(session.generalized_dof, np.float64)
    desired[6] = acceleration
    reference_bytes: bytes | None = None
    retained: dict[str, np.ndarray] | None = None
    retained_continuity: tuple[int, ...] | None = None
    replay_exact = True
    steps_us: list[float] = []
    max_calls = 0
    max_bytes = 0
    for _ in range(repeats):
        session.reset_command_state(q, v)
        out = execute(session, 0, q, v, desired)
        current = semantic_bytes(out)
        if reference_bytes is None:
            reference_bytes = current
            retained = out
            retained_continuity = tuple(session.continuity_diagnostics())
        else:
            replay_exact = replay_exact and current == reference_bytes
        steps_us.append(float(out["step_ns"][0]) / 1e3)
        max_calls = max(max_calls, int(out["allocation_calls"][0]))
        max_bytes = max(max_bytes, int(out["allocated_bytes"][0]))
    assert retained is not None
    assert retained_continuity is not None
    return {
        "repeats": repeats,
        "clearance": clearance,
        "selection": int(retained["selection"][0]),
        "status": int(retained["status"][0]),
        "solve_status": int(retained["solve_status"][0]),
        "admission_flags": int(retained["admission_flags"][0]),
        "primary_valid": bool(retained["primary_valid"][0]),
        "contingency_valid": bool(retained["contingency_valid"][0]),
        "primary_minimum_signed_distance": float(
            retained["primary_collision_distance"][0]
        ),
        "contingency_minimum_signed_distance": float(
            retained["contingency_collision_distance"][0]
        ),
        "primary_continuous_clearance_lower_bound": float(
            retained["primary_continuous_clearance"][0]
        ),
        "contingency_continuous_clearance_lower_bound": float(
            retained["contingency_continuous_clearance"][0]
        ),
        "primary_maximum_relative_speed_bound": float(
            retained["primary_relative_speed_bound"][0]
        ),
        "contingency_maximum_relative_speed_bound": float(
            retained["contingency_relative_speed_bound"][0]
        ),
        "primary_first_violation_pair": int(retained["primary_collision_pair"][0]),
        "primary_first_violation_time_ns": int(
            retained["primary_collision_time_ns"][0]
        ),
        "primary_continuity_leaf_intervals": retained_continuity[0],
        "primary_refinement_pair_samples": retained_continuity[1],
        "primary_unresolved_intervals": retained_continuity[2],
        "primary_maximum_subdivision_depth": retained_continuity[3],
        "contingency_continuity_leaf_intervals": retained_continuity[4],
        "contingency_refinement_pair_samples": retained_continuity[5],
        "contingency_unresolved_intervals": retained_continuity[6],
        "contingency_maximum_subdivision_depth": retained_continuity[7],
        "maximum_admitted_effort_abs": float(
            np.max(np.abs(retained["admitted_effort"]))
        ),
        "maximum_hard_residual": float(
            max(retained["dynamics_residual"][0], retained["contact_residual"][0])
        ),
        "semantic_replay_exact": replay_exact,
        "maximum_allocation_calls": max_calls,
        "maximum_allocated_bytes": max_bytes,
        "step_us": distribution(np.asarray(steps_us, np.float64)),
    }


def evaluate(model: pathlib.Path, repeats: int, clearance: float) -> dict[str, Any]:
    cases = {
        "safe_enabled": run_case(model, repeats, clearance, -0.40, 0.0, 0.0),
        "swept_primary_collision": run_case(
            model, repeats, clearance, -0.56, -0.5, -100.0
        ),
        "same_motion_collision_disabled": run_case(
            model, repeats, None, -0.56, -0.5, -100.0
        ),
    }
    safe = cases["safe_enabled"]
    collision = cases["swept_primary_collision"]
    disabled = cases["same_motion_collision_disabled"]
    passed = bool(
        safe["selection"] == 0
        and safe["primary_valid"]
        and safe["primary_minimum_signed_distance"] >= clearance
        and collision["selection"] == 1
        and collision["status"] == 2
        and not collision["primary_valid"]
        and collision["contingency_valid"]
        and collision["admission_flags"] & DYNAMIC_ADMISSION_PRIMARY_COLLISION
        and collision["primary_minimum_signed_distance"] < clearance
        and collision["contingency_minimum_signed_distance"] >= clearance
        and collision["primary_first_violation_pair"] == 0
        and collision["primary_first_violation_time_ns"] >= 0
        and collision["maximum_admitted_effort_abs"] == 0.0
        and disabled["selection"] == 0
        and not (
            disabled["admission_flags"] & DYNAMIC_ADMISSION_PRIMARY_COLLISION
        )
        and np.isinf(disabled["primary_minimum_signed_distance"])
        and all(case["semantic_replay_exact"] for case in cases.values())
        and all(case["maximum_allocation_calls"] == 0 for case in cases.values())
        and all(case["maximum_allocated_bytes"] == 0 for case in cases.values())
        and all(case["maximum_hard_residual"] < 1e-7 for case in cases.values())
        and all(case["step_us"]["maximum"] < 20_000.0 for case in cases.values())
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free deterministic dynamic self-collision admission",
        "model": str(model),
        "clearance": clearance,
        "cases": cases,
        "primary_collision_flag": DYNAMIC_ADMISSION_PRIMARY_COLLISION,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = []
    for name, case in audit["cases"].items():
        timing = case["step_us"]
        rows.append(
            [
                name,
                "DISABLED" if case["clearance"] is None else case["clearance"],
                case["selection"],
                f"0x{case['admission_flags']:03x}",
                f"{case['primary_valid']} / {case['contingency_valid']}",
                case["primary_minimum_signed_distance"],
                case["contingency_minimum_signed_distance"],
                f"{case['primary_first_violation_pair']} / {case['primary_first_violation_time_ns']}",
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{case['maximum_allocation_calls']} / {case['maximum_allocated_bytes']}",
            ]
        )
    collision = audit["cases"]["swept_primary_collision"]
    return f"""# Bonesaw dynamic swept-collision admission · r101

## Outcome

**{audit['status'].upper()}.** R101 applies the compiled conservative self-collision model to the actual commanded joint polynomial on a deterministic 1 ms grid, including both endpoints. Primary and braking contingency are swept independently; their clearance evidence and typed flags remain separate from solver, joint-limit, effort, and plant-response layers.

This is command-geometry validation, not physics. The observation supplies the root pose, while the joint command segment is swept without integrating root acceleration or inferring contact response.

## Retained cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

{table(['case', 'clearance m', 'selection', 'flags', 'primary / contingency valid', 'primary min m', 'contingency min m', 'first pair / time ns', 'p50 / p99 / max µs', 'alloc calls / bytes'], rows)}

The collision fixture has two 0.2 m spheres separated by one prismatic coordinate. In the fault case the primary command crosses the {audit['clearance']} m clearance at pair `{collision['primary_first_violation_pair']}` and time `{collision['primary_first_violation_time_ns']} ns`; flag `{DYNAMIC_ADMISSION_PRIMARY_COLLISION:#x}` activates, primary is withheld, and a zero-effort braking contingency remains clear. The same motion with collision policy explicitly disabled selects primary and reports no fabricated distance evidence (`Infinity`). Unknown or disabled geometry therefore does not silently masquerade as a measured clear path.

All three cases replay semantic bytes exactly across {collision['repeats']} resets, satisfy hard dynamics residuals below `1e-7`, remain inside 20 ms, and allocate zero bytes inside the Rust transaction.

## Boundary and remaining work

The current CPU sweep uses deterministic dense sampling over conservative spheres. It is not continuous collision detection and cannot prove safety between 1 ms samples without a distance-rate bound. Meshes are skipped by the compiled high-rate proxy, world SDF/obstacles are not yet ingested, and the floating root command is not synthesized. These remain explicit gaps rather than free-space assumptions.
"""


def main() -> None:
    args = parse_args()
    if args.repeats <= 1 or not np.isfinite(args.clearance):
        raise SystemExit("invalid repeats or clearance")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(model, args.repeats, args.clearance)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "dynamic-collision-r101",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "dynamic-collision-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "DYNAMIC_COLLISION_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw dynamic swept-collision admission · r101")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
