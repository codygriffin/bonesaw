#!/usr/bin/env python3
"""Qualify the default-off Rust support-transfer motion-headroom witness."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import subprocess
from typing import Any

from cpu_reference_report import markdown_table, render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "joint-motion-headroom-r288"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "JOINT_MOTION_HEADROOM_R288.md"
WEB_REPORT = ROOT / "web/JOINT_MOTION_HEADROOM_R288.html"


def run_audit(binary: pathlib.Path, model: pathlib.Path, cpu: int | None) -> dict[str, Any]:
    command = [str(binary), str(model)]
    if cpu is not None:
        command = ["taskset", "-c", str(cpu), *command]
    completed = subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise ValueError("joint-motion-headroom audit produced no JSON result")
    return json.loads(lines[-1])


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def qualify(audits: list[dict[str, Any]], cpu: int | None) -> dict[str, Any]:
    if len(audits) < 5:
        raise ValueError("R288 requires at least five independent process repeats")
    first = audits[0]
    for repeat, audit in enumerate(audits):
        if audit.get("schema") != "bonesaw.joint-motion-headroom-r288.v1":
            raise ValueError(f"repeat {repeat} has an unexpected schema")
        for key in ("model", "repetitions_per_case", "maximum_acceleration_mps2", "reaction_time_seconds"):
            if audit[key] != first[key]:
                raise ValueError(f"repeat {repeat} changed fixed execution shape: {key}")
        cases = audit.get("cases", [])
        if [case["name"] for case in cases] != ["centered", "moving", "near_limit"]:
            raise ValueError(f"repeat {repeat} has unexpected cases")
        if any(
            case["measured_allocation_calls"]
            or case["measured_allocated_bytes"]
            or case["measured_deallocation_calls"]
            or not case["repeated_result_bitwise_equal"]
            for case in cases
        ):
            raise ValueError(f"repeat {repeat} violated the allocation/repeatability contract")
        centered, moving, near_limit = cases
        if not (centered["fraction_of_range"] > moving["fraction_of_range"] > near_limit["fraction_of_range"]):
            raise ValueError(f"repeat {repeat} lost conservative headroom ordering")
        if near_limit["fraction_of_range"] >= 0.0:
            raise ValueError(f"repeat {repeat} failed to expose an unrecoverable near-limit witness")

    samples = [
        float(case["nanoseconds_per_call"])
        for audit in audits
        for case in audit["cases"]
    ]
    return {
        "schema": "bonesaw.joint-motion-headroom-r288.v1",
        "revision": REVISION,
        "execution": {
            "model": first["model"],
            "cpu_affinity": cpu,
            "process_repeats": len(audits),
            "repetitions_per_case": first["repetitions_per_case"],
            "maximum_acceleration_mps2": first["maximum_acceleration_mps2"],
            "reaction_time_seconds": first["reaction_time_seconds"],
        },
        "performance_ns_per_call": {
            "samples": samples,
            "minimum": min(samples),
            "median": statistics.median(samples),
            "mean": statistics.fmean(samples),
            "p95": percentile(samples, 0.95),
            "p99": percentile(samples, 0.99),
            "maximum": max(samples),
            "population_standard_deviation": statistics.pstdev(samples),
        },
        "allocation_contract": {
            "all_cases_zero_allocations": all(
                case["measured_allocation_calls"] == 0
                for audit in audits
                for case in audit["cases"]
            ),
            "all_cases_zero_bytes": all(
                case["measured_allocated_bytes"] == 0
                for audit in audits
                for case in audit["cases"]
            ),
            "all_cases_zero_deallocations": all(
                case["measured_deallocation_calls"] == 0
                for audit in audits
                for case in audit["cases"]
            ),
        },
        "behavior_contract": {
            "ordered_centered_moving_near_limit": True,
            "negative_near_limit_witness": True,
            "default_off": True,
            "policy_steps": 0,
            "physics_steps": 0,
            "authority_admitted": False,
        },
        "audits": audits,
    }


def build_report(result: dict[str, Any]) -> str:
    rows = []
    for case in result["audits"][0]["cases"]:
        rows.append(
            [
                case["name"],
                f"{case['position_margin_rad']:.3f}",
                f"{case['stopping_margin_rad']:.3f}",
                f"{case['velocity_fraction']:.3f}",
                f"{case['fraction_of_range']:.3f}",
                f"{case['nanoseconds_per_call']:.1f}",
                "0",
                "exact",
            ]
        )
    performance = result["performance_ns_per_call"]
    contract = result["allocation_contract"]
    execution = result["execution"]
    lines = [
        "# Joint motion headroom witness · R288",
        "",
        "> Mechanism **PASS** · allocation contract **PASS** · authority **NOT ADMITTED**.",
        "",
        "R288 adds a default-off support-transfer witness in Rust. It combines one controller reaction interval, ideal stopping distance at the declared acceleration envelope, and authored joint velocity utilization. The result is evidence for the existing support-tube scale; it never clamps a command or turns a soft request into authority.",
        "",
        "## Frozen CPU audit",
        "",
        *markdown_table(
            ["case", "position margin rad", "stopping margin rad", "velocity fraction", "conservative fraction", "ns/call", "alloc bytes", "repeatability"],
            rows,
        ),
        "",
        f"The Upkie model is evaluated with {execution['repetitions_per_case']:,} warmed calls per case, acceleration authority {execution['maximum_acceleration_mps2']:.1f} m/s², and {execution['reaction_time_seconds']:.3f} s reaction time. Centered → moving → near-limit headroom is strictly ordered; the near-limit case remains negative instead of being silently saturated. Across {execution['process_repeats']} CPU-{execution['cpu_affinity']} process repeats, median cost is {performance['median']:.1f} ns/call and p99 is {performance['p99']:.1f} ns/call.",
        "",
        f"The measured loop reports zero allocations ({contract['all_cases_zero_allocations']}), zero allocated bytes ({contract['all_cases_zero_bytes']}), and zero deallocations ({contract['all_cases_zero_deallocations']}); repeated outputs are bitwise exact. Python performs only orchestration/reporting, with no policy or physics steps.",
        "",
        "## Integration boundary",
        "",
        "The feature remains opt-in through `support_trajectory_tube_motion_headroom`. Existing dormant and R280/R287 profiles do not execute it and therefore retain their established semantics. When enabled with a positive headroom floor, the support-transfer envelope uses the minimum of the legacy position margin and this stopping/velocity witness. A negative margin is evidence of an unrecoverable observation, not permission to invent authority.",
        "",
        "This closes the missing typed joint-headroom input to the support-feasible tube. It does not claim that the G1 walking trace now meets its release, tracking, timing, contact, thermal, or plant gates; those remain separate qualification work.",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=pathlib.Path, default=ROOT / "target/release/bonesaw-joint-motion-headroom-audit")
    parser.add_argument("--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--cpu", type=int, default=4)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    if args.repeats < 5:
        raise SystemExit("R288 requires at least five repeats")
    audits = [run_audit(args.binary, args.model, args.cpu) for _ in range(args.repeats)]
    result = qualify(audits, args.cpu)
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(result, indent=2) + "\n")
        report = build_report(result)
        REPORT.write_text(report)
        WEB_REPORT.write_text(render_report_html(report, title="Joint motion headroom · R288"))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
