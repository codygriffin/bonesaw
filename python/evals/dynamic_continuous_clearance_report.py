#!/usr/bin/env python3
"""Audit the conservative between-sample collision-clearance certificate."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import DYNAMIC_ADMISSION_PRIMARY_CONTINUOUS_CLEARANCE
from cpu_reference_report import markdown_table, render_report_html
from dynamic_collision_fault_report import run_case


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/collision_sweep_toy.urdf")
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument(
        "--output", default="benchmarks/results/dynamic-continuous-clearance-r102"
    )
    parser.add_argument(
        "--web-report", default="web/DYNAMIC_CONTINUOUS_CLEARANCE_R102.html"
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
    # At q=-0.5503 the command remains above 20 mm at all 21 grid samples,
    # while its conservative center-speed guard exceeds that sampled margin.
    cases = {
        "safe_certified": run_case(
            model, repeats, clearance, -0.40, 0.0, 0.0, True
        ),
        "near_grid_only": run_case(
            model, repeats, clearance, -0.5503, -0.5, -100.0, False
        ),
        "near_rate_certified": run_case(
            model, repeats, clearance, -0.5503, -0.5, -100.0, True
        ),
    }
    safe = cases["safe_certified"]
    grid = cases["near_grid_only"]
    certified = cases["near_rate_certified"]
    passed = bool(
        safe["selection"] == 0
        and safe["primary_continuous_clearance_lower_bound"] >= clearance
        and grid["selection"] == 0
        and grid["primary_minimum_signed_distance"] >= clearance
        and grid["primary_continuous_clearance_lower_bound"] < clearance
        and grid["primary_first_violation_pair"] == -1
        and certified["selection"] == 1
        and not certified["primary_valid"]
        and certified["contingency_valid"]
        and certified["admission_flags"]
        & DYNAMIC_ADMISSION_PRIMARY_CONTINUOUS_CLEARANCE
        and certified["primary_minimum_signed_distance"] >= clearance
        and certified["primary_continuous_clearance_lower_bound"] < clearance
        and certified["contingency_continuous_clearance_lower_bound"] >= clearance
        and certified["primary_first_violation_pair"] == -1
        and certified["maximum_admitted_effort_abs"] == 0.0
        and all(case["semantic_replay_exact"] for case in cases.values())
        and all(case["maximum_allocation_calls"] == 0 for case in cases.values())
        and all(case["maximum_allocated_bytes"] == 0 for case in cases.values())
        and all(case["maximum_hard_residual"] < 1e-7 for case in cases.values())
        and all(case["step_us"]["maximum"] < 20_000.0 for case in cases.values())
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free continuous clearance certificate",
        "model": str(model),
        "clearance": clearance,
        "sample_period_ns": 1_000_000,
        "certificate": "sampled_minimum - maximum_relative_center_speed * sample_period / 2",
        "primary_continuous_clearance_flag": DYNAMIC_ADMISSION_PRIMARY_CONTINUOUS_CLEARANCE,
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
                f"0x{case['admission_flags']:03x}",
                case["primary_minimum_signed_distance"],
                case["primary_continuous_clearance_lower_bound"],
                case["primary_maximum_relative_speed_bound"],
                case["contingency_continuous_clearance_lower_bound"],
                f"{case['primary_first_violation_pair']} / {case['primary_first_violation_time_ns']}",
                f"{timing['p50']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}",
                f"{case['maximum_allocation_calls']} / {case['maximum_allocated_bytes']}",
            ]
        )
    certified = audit["cases"]["near_rate_certified"]
    sampled_mm = certified["primary_minimum_signed_distance"] * 1e3
    lower_mm = certified["primary_continuous_clearance_lower_bound"] * 1e3
    return f"""# Bonesaw conservative continuous-clearance admission · r102

## Outcome

**{audit['status'].upper()}.** R102 keeps the deterministic 1 ms collision grid as direct evidence, then optionally requires a conservative continuous certificate. For each compiled sphere, Rust precomputes joint-coordinate center-speed coefficients. Analytic maximum joint velocity over the quintic bounds every pair's relative center speed, and admission subtracts half an interval of possible travel from the sampled minimum.

This is a conservative Lipschitz certificate over the represented sphere proxies. It is policy-free and physics-free, and it does not claim exact mesh continuous collision detection.

## Retained adversarial cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

{table(['case', 'selection', 'flags', 'sampled primary min m', 'continuous lower m', 'relative speed m/s', 'braking lower m', 'raw violation pair / ns', 'p50 / p99 / max µs', 'alloc calls / bytes'], rows)}

The near case is clear at every grid point: its sampled primary minimum is **{sampled_mm:.3f} mm** against a 20 mm requirement, and no raw violation pair or time exists. Its continuous lower bound is only **{lower_mm:.3f} mm**, so flag `{DYNAMIC_ADMISSION_PRIMARY_CONTINUOUS_CLEARANCE:#x}` activates only when continuous certification is requested. The exact same command is selected under explicit grid-only policy and withheld under conservative-rate policy; the independently certified braking segment is selected with zero admitted feed-forward effort.

All cases replay semantic bytes exactly across {certified['repeats']} resets, allocate zero bytes inside the Rust transaction, satisfy hard dynamics/contact residuals below `1e-7`, and remain far inside the 20 ms horizon.

## Authority interpretation

The example authority stack now has two collision rows: **sampled geometry** records the measured proxy minimum and exact first failing sample, while **continuous clearance** records the lower-bound certificate and speed budget. A red continuous row with a green sampled row means “no collision was observed, but clearance was not proved”; it is not reported as an observed collision.

## Boundary and remaining work

The certificate is intentionally conservative and fixed-root over each 20 ms command segment. Unsupported authored geometry is still rejected by default; world SDF/obstacles, synthesized floating-root trajectories, tighter pair-specific interval bounds, and exact mesh CCD remain explicit future work.
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
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "dynamic-continuous-clearance-r102",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "dynamic-continuous-clearance-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "DYNAMIC_CONTINUOUS_CLEARANCE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(
            report, title="Bonesaw conservative continuous-clearance admission · r102"
        )
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
