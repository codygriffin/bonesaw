#!/usr/bin/env python3
"""Qualify allocation-stable historical frame queries (R287)."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import subprocess
from typing import Any

from cpu_reference_report import render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "frame-query-allocation-r287"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "FRAME_QUERY_ALLOCATION_R287.md"
WEB_REPORT = ROOT / "web/FRAME_QUERY_ALLOCATION_R287.html"
R286_RESULT = (
    ROOT
    / "benchmarks/results/frame-query-batch-r286"
    / "frame-query-batch-r286-metrics.json"
)


def run_audit(
    binary: pathlib.Path, model: pathlib.Path, cpu: int | None
) -> dict[str, Any]:
    command = [str(binary), str(model)]
    if cpu is not None:
        command = ["taskset", "-c", str(cpu), *command]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise ValueError("frame-query audit produced no JSON result")
    return json.loads(lines[-1])


def percentile(samples: list[float], quantile: float) -> float:
    ordered = sorted(samples)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def qualify(audits: list[dict[str, Any]], cpu: int | None) -> dict[str, Any]:
    if len(audits) < 5:
        raise ValueError("R287 requires at least five independent process repeats")
    first = audits[0]
    stable_fields = ("model", "query_count", "warmup_batches", "measured_batches")
    for repeat, audit in enumerate(audits):
        if audit.get("schema") != "bonesaw.frame-query-allocation-r287.v1":
            raise ValueError(f"repeat {repeat} has unexpected schema: {audit.get('schema')}")
        if any(audit[field] != first[field] for field in stable_fields):
            raise ValueError(f"repeat {repeat} changed the fixed execution shape")
        if audit["query_count"] < 32 or audit["measured_batches"] < 8:
            raise ValueError("R287 requires a bounded, nontrivial batch replay")
        if not audit["repeated_results_bitwise_equal"]:
            raise ValueError(f"repeat {repeat} was not bitwise stable")
        if audit["measured_allocation_calls"] != 0:
            raise ValueError(f"repeat {repeat} allocated in the measured loop")
        if audit["measured_allocated_bytes"] != 0:
            raise ValueError(f"repeat {repeat} allocated bytes in the measured loop")
        if audit["measured_deallocation_calls"] != 0:
            raise ValueError(f"repeat {repeat} deallocated in the measured loop")
        for owner in ("output", "workspace"):
            if audit[f"{owner}_external_capacity_before"] != audit[
                f"{owner}_external_capacity_after"
            ]:
                raise ValueError(f"repeat {repeat} grew {owner} capacity")
        if audit["workspace_external_sample_capacity_before"] != audit[
            "workspace_external_sample_capacity_after"
        ]:
            raise ValueError(f"repeat {repeat} grew external sample capacity")

    samples = [float(audit["nanoseconds_per_query"]) for audit in audits]
    mean = statistics.fmean(samples)
    standard_deviation = statistics.pstdev(samples)
    previous = json.loads(R286_RESULT.read_text()) if R286_RESULT.exists() else None
    previous_ns = (
        float(previous["performance"]["nanoseconds_per_query"])
        if previous is not None
        else None
    )
    median = statistics.median(samples)
    return {
        "schema": "bonesaw.frame-query-allocation-r287.v1",
        "revision": REVISION,
        "execution": {
            "model": first["model"],
            "cpu_affinity": cpu,
            "process_repeats": len(audits),
            "query_count": first["query_count"],
            "warmup_batches_per_repeat": first["warmup_batches"],
            "measured_batches_per_repeat": first["measured_batches"],
            "measured_queries_per_repeat": first["query_count"]
            * first["measured_batches"],
        },
        "performance_ns_per_query": {
            "samples": samples,
            "minimum": min(samples),
            "median": median,
            "mean": mean,
            "p95": percentile(samples, 0.95),
            "p99": percentile(samples, 0.99),
            "maximum": max(samples),
            "peak_to_peak": max(samples) - min(samples),
            "population_standard_deviation": standard_deviation,
            "coefficient_of_variation": standard_deviation / mean,
        },
        "allocation_contract": {
            "repeats_with_zero_allocation_calls": len(audits),
            "repeats_with_zero_allocated_bytes": len(audits),
            "repeats_with_zero_deallocation_calls": len(audits),
            "total_measured_allocation_calls": sum(
                audit["measured_allocation_calls"] for audit in audits
            ),
            "total_measured_allocated_bytes": sum(
                audit["measured_allocated_bytes"] for audit in audits
            ),
            "total_measured_deallocation_calls": sum(
                audit["measured_deallocation_calls"] for audit in audits
            ),
            "output_external_capacity": [
                first["output_external_capacity_before"],
                first["output_external_capacity_after"],
            ],
            "workspace_external_capacity": [
                first["workspace_external_capacity_before"],
                first["workspace_external_capacity_after"],
            ],
            "workspace_external_sample_capacity": [
                first["workspace_external_sample_capacity_before"],
                first["workspace_external_sample_capacity_after"],
            ],
            "all_results_bitwise_equal_within_repeat": all(
                audit["repeated_results_bitwise_equal"] for audit in audits
            ),
            "claim": first["allocation_claim"],
        },
        "comparison_to_r286_recorded_sample": {
            "r286_ns_per_query": previous_ns,
            "r287_median_ns_per_query": median,
            "median_delta_percent": (
                100.0 * (median - previous_ns) / previous_ns
                if previous_ns is not None
                else None
            ),
            "caveat": "R286 retained one recorded timing sample and did not instrument legacy reconstruction allocations; this comparison is contextual, not a paired statistical claim.",
        },
        "raw_repeats": audits,
        "verdict": {
            "strict_historical_query_allocation_stable": True,
            "legacy_allocating_api_retained_for_compatibility": True,
            "wbc_authority_admitted": False,
            "physics_authority_admitted": False,
        },
    }


def build_report(result: dict[str, Any]) -> str:
    execution = result["execution"]
    performance = result["performance_ns_per_query"]
    allocation = result["allocation_contract"]
    comparison = result["comparison_to_r286_recorded_sample"]
    delta = comparison["median_delta_percent"]
    delta_text = "unavailable" if delta is None else f"{delta:+.2f}%"
    return f"""# Allocation-stable historical frame queries · R287

> Strict caller-owned path **PASS** · timed-loop allocations **0** · authority **UNCHANGED**.

R287 adds `RobotHistory::reconstruct_into`,
`HistoricalFrameQueryWorkspace`, and strict scalar/batch atlas query entry
points. Robot state, external provenance, atlas input, model cache, snapshot,
and retained outputs are all caller-owned. Undersized output storage is a
typed error instead of an implicit growth operation. The older allocating API
remains available for compatibility and delegates to the same semantics.

## Qualification

| field | value |
|---|---:|
| model | `{execution['model']}` |
| CPU affinity | {execution['cpu_affinity'] if execution['cpu_affinity'] is not None else 'unbound'} |
| independent process repeats | {execution['process_repeats']} |
| queries per batch | {execution['query_count']} |
| warmup batches per repeat | {execution['warmup_batches_per_repeat']} |
| measured batches per repeat | {execution['measured_batches_per_repeat']} |
| measured queries per repeat | {execution['measured_queries_per_repeat']} |
| latency minimum | {performance['minimum'] / 1e3:.3f} µs/query |
| latency median | {performance['median'] / 1e3:.3f} µs/query |
| latency mean | {performance['mean'] / 1e3:.3f} µs/query |
| latency p95 | {performance['p95'] / 1e3:.3f} µs/query |
| latency p99 | {performance['p99'] / 1e3:.3f} µs/query |
| latency maximum | {performance['maximum'] / 1e3:.3f} µs/query |
| peak-to-peak process jitter | {performance['peak_to_peak'] / 1e3:.3f} µs/query |
| coefficient of variation | {100.0 * performance['coefficient_of_variation']:.2f}% |
| measured allocation calls | {allocation['total_measured_allocation_calls']} |
| measured allocated bytes | {allocation['total_measured_allocated_bytes']} |
| measured deallocation calls | {allocation['total_measured_deallocation_calls']} |
| output provenance capacity | {allocation['output_external_capacity'][0]} → {allocation['output_external_capacity'][1]} |
| workspace provenance capacity | {allocation['workspace_external_capacity'][0]} → {allocation['workspace_external_capacity'][1]} |
| workspace external-sample capacity | {allocation['workspace_external_sample_capacity'][0]} → {allocation['workspace_external_sample_capacity'][1]} |
| bitwise repeatability within every process | `{allocation['all_results_bitwise_equal_within_repeat']}` |

The counting allocator surrounds only the warmed measured batch loop. Every
process repeat independently observed zero allocation, zero allocated bytes,
and zero deallocation. Layout/capacity mistakes are rejected before
reconstruction; the retained output is committed only after all robot and
external reconstruction plus atlas evaluation succeeds.

## R286 context

The prior recorded R286 sample was
{comparison['r286_ns_per_query'] / 1e3:.3f} µs/query; R287's multi-process
median is {comparison['r287_median_ns_per_query'] / 1e3:.3f} µs/query
({delta_text}). {comparison['caveat']}

No policy step, physics step, actuator command, plant observation, or WBC
authority admission is part of this result.
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--binary",
        type=pathlib.Path,
        default=ROOT / "target/release/frame_query_batch_audit",
    )
    parser.add_argument(
        "--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf"
    )
    parser.add_argument("--repeats", type=int, default=21)
    parser.add_argument(
        "--cpu",
        type=int,
        default=4,
        help="pin each measured process to one CPU; pass -1 to leave unbound",
    )
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    if args.repeats < 5:
        parser.error("--repeats must be at least 5")
    cpu = None if args.cpu < 0 else args.cpu
    result = qualify(
        [run_audit(args.binary, args.model, cpu) for _ in range(args.repeats)], cpu
    )
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        report = build_report(result)
        REPORT.write_text(report)
        WEB_REPORT.write_text(
            render_report_html(report, title="Allocation-stable frame queries · R287")
        )
    print(f"validated {RESULT}")


if __name__ == "__main__":
    main()
