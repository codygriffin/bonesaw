#!/usr/bin/env python3
"""Qualify the exact seven-iteration feasibility-polish profile (R283)."""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_cumulative_solve_diagnostics_r282 import (
    R281_NON_TIMING_ARRAY_COUNT,
    R281_NON_TIMING_DIGEST,
    non_timing_digest,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "g1-feasibility-polish-budget-r283"
BASE = ROOT / "benchmarks/results/floating-g1-r282-cumulative-solve-diagnostics"
REPEATS = tuple(
    ROOT / f"benchmarks/results/floating-g1-r283-polish-budget7-repeat{repeat}"
    for repeat in range(5)
)
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "G1_FEASIBILITY_POLISH_BUDGET_R283.md"
WEB_REPORT = ROOT / "web/G1_FEASIBILITY_POLISH_BUDGET_R283.html"
TAIL_TICKS = (1694, 2296)


def _load_run(path: pathlib.Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    metrics = json.loads((path / "floating-walk-metrics.json").read_text())
    raw = dict(np.load(path / "floating-walk-raw.npz", allow_pickle=False))
    return metrics, raw


def build_result() -> dict[str, Any]:
    baseline_document, baseline = _load_run(BASE)
    baseline_tail_ms = [float(baseline["step_ns"][tick] / 1e6) for tick in TAIL_TICKS]
    rows = []
    candidate_tail_samples_ms: list[float] = []
    for repeat, path in enumerate(REPEATS):
        document, raw = _load_run(path)
        count, digest = non_timing_digest(raw)
        latency = document["metrics"]["latency_us"]
        tail_ms = [float(raw["step_ns"][tick] / 1e6) for tick in TAIL_TICKS]
        candidate_tail_samples_ms.extend(tail_ms)
        rows.append(
            {
                "repeat": repeat,
                "non_timing_arrays": count,
                "non_timing_sha256": digest,
                "semantic_exact": count == R281_NON_TIMING_ARRAY_COUNT
                and digest == R281_NON_TIMING_DIGEST,
                "p50_us": float(latency["p50"]),
                "p95_us": float(latency["p95"]),
                "p99_us": float(latency["p99"]),
                "maximum_us": float(latency["max"]),
                "over_5ms": int(np.count_nonzero(raw["step_ns"] > 5_000_000)),
                "tail_ms": tail_ms,
                "tail_polish_iterations": [
                    int(raw["cumulative_feasibility_polish_iterations"][tick])
                    for tick in TAIL_TICKS
                ],
                "tail_polish_pseudoinverse_calls": [
                    int(
                        raw[
                            "cumulative_feasibility_polish_pseudoinverse_calls"
                        ][tick]
                    )
                    for tick in TAIL_TICKS
                ],
                "rss_delta_mib": float(
                    document["metrics"]["runtime"]["rss_delta_bytes"] / 2**20
                ),
                "python_gc_collections": int(
                    document["metrics"]["runtime"]["python_gc_delta"][
                        "collections"
                    ]
                ),
            }
        )
    baseline_pinv = [
        int(baseline["cumulative_feasibility_polish_pseudoinverse_calls"][tick])
        for tick in TAIL_TICKS
    ]
    candidate_pinv = rows[0]["tail_polish_pseudoinverse_calls"]
    baseline_tail_mean = float(np.mean(baseline_tail_ms))
    candidate_tail_mean = float(np.mean(candidate_tail_samples_ms))
    exact = all(row["semantic_exact"] for row in rows)
    tail_bounded = all(max(row["tail_ms"]) < 10.0 for row in rows)
    p99_passed = all(row["p99_us"] <= 5_000.0 for row in rows)
    return {
        "schema": "bonesaw.g1-feasibility-polish-budget-r283.v1",
        "revision": REVISION,
        "execution": {
            "policy_steps": 0,
            "physics_steps": 0,
            "ticks_per_repeat": int(baseline_document["metrics"]["ticks"]),
            "dt_seconds": float(baseline_document["metrics"]["dt_seconds"]),
            "timing_cpu": 4,
            "maximum_feasibility_iterations": 7,
            "maximum_feasibility_projection_sweeps": 8,
        },
        "semantic_contract": {
            "non_timing_arrays": R281_NON_TIMING_ARRAY_COUNT,
            "non_timing_sha256": R281_NON_TIMING_DIGEST,
            "all_repeats_exact": exact,
        },
        "baseline": {
            "maximum_feasibility_iterations": 64,
            "tail_ms": baseline_tail_ms,
            "tail_mean_ms": baseline_tail_mean,
            "tail_polish_pseudoinverse_calls": baseline_pinv,
        },
        "repeats": rows,
        "summary": {
            "candidate_tail_mean_ms": candidate_tail_mean,
            "candidate_tail_maximum_ms": float(max(candidate_tail_samples_ms)),
            "tail_latency_reduction_percent": float(
                100.0 * (1.0 - candidate_tail_mean / baseline_tail_mean)
            ),
            "tail_polish_pseudoinverse_calls": candidate_pinv,
            "polish_pseudoinverse_reduction_percent": float(
                100.0 * (1.0 - np.mean(candidate_pinv) / np.mean(baseline_pinv))
            ),
            "p99_mean_us": float(np.mean([row["p99_us"] for row in rows])),
            "p99_stddev_us": float(np.std([row["p99_us"] for row in rows])),
            "deadline_misses": int(sum(row["over_5ms"] for row in rows)),
        },
        "verdict": {
            "profile_retained": exact and tail_bounded,
            "tail_gate_passed": tail_bounded,
            "p99_gate_passed": p99_passed,
            "default_changed": False,
            "authority_admitted": False,
        },
    }


def validate_result(result: dict[str, Any]) -> None:
    if result.get("revision") != REVISION:
        raise ValueError("unexpected R283 revision")
    execution = result["execution"]
    if execution["policy_steps"] or execution["physics_steps"]:
        raise ValueError("R283 must remain policy/physics free")
    if execution["maximum_feasibility_iterations"] != 7:
        raise ValueError("R283 must qualify the seven-iteration profile")
    if not result["semantic_contract"]["all_repeats_exact"]:
        raise ValueError("R283 changed the established non-timing trace")
    if len(result["repeats"]) != 5:
        raise ValueError("R283 requires five pinned repeats")
    verdict = result["verdict"]
    if not verdict["profile_retained"] or not verdict["tail_gate_passed"]:
        raise ValueError("R283 tail profile gate failed")
    if verdict["p99_gate_passed"]:
        raise ValueError("R283 must not conceal the remaining p99 miss")
    if verdict["default_changed"] or verdict["authority_admitted"]:
        raise ValueError("R283 cannot change the default or admit authority")


def build_report(result: dict[str, Any]) -> str:
    rows = [
        [
            row["repeat"],
            "exact" if row["semantic_exact"] else "DIFF",
            f"{row['p50_us']:.1f}",
            f"{row['p95_us']:.1f}",
            f"{row['p99_us']:.1f}",
            f"{row['maximum_us']:.1f}",
            row["over_5ms"],
            "/".join(f"{value:.2f}" for value in row["tail_ms"]),
            f"{row['rss_delta_mib']:.3f}",
            row["python_gc_collections"],
        ]
        for row in result["repeats"]
    ]
    summary = result["summary"]
    baseline = result["baseline"]
    return (
        "\n".join(
            [
                "# G1 bounded feasibility-polish profile · R283",
                "",
                "> Exact profile **RETAINED** · deterministic tail **PASS** · p99 **REJECTED** · default/authority **UNCHANGED**.",
                "",
                "R283 applies the user-suggested bounded-compute tradeoff to the policy- and physics-free R280/R281 WBC replay. It reduces the speculative active-set accelerator from 64 to seven iterations while retaining the existing eight-sweep exact Dykstra boundary. Failed acceleration remains fail-closed and is never integrated.",
                "",
                "## Five CPU-4-pinned repeats",
                "",
                *markdown_table(
                    ["repeat", "89 arrays", "p50 µs", "p95 µs", "p99 µs", "max µs", ">5 ms", "tails 1694/2296 ms", "RSS Δ MiB", "GC"],
                    rows,
                ),
                "",
                f"All five runs reproduce the established 89-array digest `{result['semantic_contract']['non_timing_sha256']}`. The two deterministic tails fall from {baseline['tail_ms'][0]:.1f}/{baseline['tail_ms'][1]:.1f} ms to at most {summary['candidate_tail_maximum_ms']:.2f} ms ({summary['tail_latency_reduction_percent']:.2f}% mean reduction). Dense polish pseudoinverses fall from {baseline['tail_polish_pseudoinverse_calls'][0]} per tail to {summary['tail_polish_pseudoinverse_calls'][0]} ({summary['polish_pseudoinverse_reduction_percent']:.2f}% reduction).",
                "",
                "## Decision",
                "",
                f"The profile is retained as an explicit real-time configuration, not promoted to the universal default: lower budgets changed behavior in the qualification sweep, so seven is corpus-qualified rather than a global theorem. Ordinary work still misses the 5,000 µs gate in every repeat (mean p99 {summary['p99_mean_us']:.1f} µs, σ {summary['p99_stddev_us']:.1f} µs; {summary['deadline_misses']} total misses). Preference/Style task inversions remain the next exact optimization target.",
                "",
                "No support, contact, actuator, thermal, or walking authority is admitted by this result.",
            ]
        )
        + "\n"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    result = build_result()
    validate_result(result)
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        report = build_report(result)
        REPORT.write_text(report)
        WEB_REPORT.write_text(
            render_report_html(report, title="G1 bounded feasibility polish · R283")
        )
    print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
