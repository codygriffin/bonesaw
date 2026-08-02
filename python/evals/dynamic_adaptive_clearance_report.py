#!/usr/bin/env python3
"""Audit bounded pair/interval adaptive continuous-clearance refinement."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from dynamic_collision_fault_report import run_case


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument(
        "--output", default="benchmarks/results/dynamic-adaptive-clearance-r106"
    )
    parser.add_argument(
        "--web-report", default="web/DYNAMIC_ADAPTIVE_CLEARANCE_R106.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table(headers, rows))


def evaluate(model: pathlib.Path, repeats: int, clearance: float) -> dict[str, Any]:
    cases = {
        f"depth_{depth}": run_case(
            model,
            repeats,
            clearance,
            -0.5503,
            -0.5,
            -100.0,
            True,
            depth,
        )
        for depth in range(4)
    }
    ordered = [cases[f"depth_{depth}"] for depth in range(4)]
    lower_bounds = [case["primary_continuous_clearance_lower_bound"] for case in ordered]
    sampled = [case["primary_minimum_signed_distance"] for case in ordered]
    passed = bool(
        [case["selection"] for case in ordered] == [1, 1, 1, 0]
        and all(case["primary_first_violation_pair"] == -1 for case in ordered)
        and all(value >= clearance for value in sampled)
        and max(sampled) - min(sampled) < 1e-15
        and all(a <= b for a, b in zip(lower_bounds, lower_bounds[1:]))
        and lower_bounds[0] < clearance
        and lower_bounds[1] < clearance
        and lower_bounds[2] < clearance
        and lower_bounds[3] >= clearance
        and [case["primary_refinement_pair_samples"] for case in ordered]
        == [0, 1, 2, 3]
        and [case["primary_unresolved_intervals"] for case in ordered]
        == [0, 1, 1, 0]
        and all(case["contingency_valid"] for case in ordered)
        and all(case["semantic_replay_exact"] for case in ordered)
        and all(case["maximum_allocation_calls"] == 0 for case in ordered)
        and all(case["maximum_allocated_bytes"] == 0 for case in ordered)
        and all(case["maximum_hard_residual"] < 1e-7 for case in ordered)
        and all(case["step_us"]["maximum"] < 20_000.0 for case in ordered)
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free bounded adaptive command-clearance certificate",
        "model": str(model),
        "clearance": clearance,
        "base_sample_period_ns": 1_000_000,
        "cases": cases,
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = []
    for name, case in audit["cases"].items():
        timing = case["step_us"]
        rows.append(
            [
                name,
                case["selection"],
                case["primary_minimum_signed_distance"] * 1e3,
                case["primary_continuous_clearance_lower_bound"] * 1e3,
                case["primary_refinement_pair_samples"],
                case["primary_continuity_leaf_intervals"],
                case["primary_unresolved_intervals"],
                case["primary_maximum_subdivision_depth"],
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{case['maximum_allocation_calls']} / {case['maximum_allocated_bytes']}",
            ]
        )
    final = audit["cases"]["depth_3"]
    return f"""# Bonesaw adaptive pair/interval clearance · r106

## Outcome

**{audit['status'].upper()}.** R106 retains the exact 1 ms command grid, then recursively samples only pair/interval leaves whose local Lipschitz reserve is still below the required clearance. Maximum depth is an explicit construction-time bound. Joint-velocity extrema are analytic on each subinterval; midpoint FK and primitive distance use caller-owned Rust scratch.

The four cases below are the same observed state, WBC result, command polynomial, and 20 mm requirement. Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

{table(['case', 'selection', 'sampled min mm', 'continuous lower mm', 'midpoint pair queries', 'leaf intervals', 'unresolved', 'depth reached', 'p50 / p99 / max µs', 'alloc calls / bytes'], rows)}

The base 21-point trace remains bit-identical at **{final['primary_minimum_signed_distance'] * 1e3:.3f} mm** with no sampled violation. Depths one and two improve the bound but keep one leaf explicitly unresolved. Depth three performs exactly three midpoint pair queries and proves **{final['primary_continuous_clearance_lower_bound'] * 1e3:.3f} mm**, so Primary is admitted without weakening the 20 mm requirement.

All depths replay semantic bytes exactly across {final['repeats']} command-state resets, allocate zero calls/bytes inside the timed Rust transaction, and satisfy hard dynamics/contact residuals below `1e-7`. A separate Rust regression uses safe base endpoints with a penetrating midpoint and proves refinement records a real sampled collision pair/time rather than allowing a continuous certificate to conceal it.

## Authority boundary

Refinement work, unresolved leaves, sampled violations, and final selection are independent witnesses. A deeper certificate may reduce false contingency selection, but it is not a policy, plant rollout, collision response, or guarantee about future command-state evolution. The R105 live path already demonstrates why that distinction matters.

## Remaining work

The bound still uses conservative primitive-point reach coefficients rather than closest-feature velocity, and box-box clearance remains a separating-axis lower bound. Pair-specific stopping criteria, shared primitive avoidance Jacobians, world collision, mesh CCD, and calibrated plant response remain outside this claim.
"""


def main() -> None:
    args = parse_args()
    if args.repeats <= 1 or not np.isfinite(args.clearance) or args.clearance < 0.0:
        raise SystemExit("invalid repeats or clearance")
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(model, args.repeats, args.clearance)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/trajectory.rs"),
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "dynamic-adaptive-clearance-r106",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "dynamic-adaptive-clearance-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "DYNAMIC_ADAPTIVE_CLEARANCE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw adaptive pair/interval clearance · r106")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
