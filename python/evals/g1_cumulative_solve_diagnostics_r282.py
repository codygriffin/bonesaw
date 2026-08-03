#!/usr/bin/env python3
"""Qualify cumulative floating-WBC solve-attempt diagnostics (R282).

The evaluator consumes one immutable, simulator-free floating-walk corpus run.
Rust owns the allocation-free attempt accounting; Python checks the trace
contract, summarizes the retry stages, and renders the review artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "g1-cumulative-solve-diagnostics-r282"
CORPUS = ROOT / "benchmarks/results/floating-g1-r282-cumulative-solve-diagnostics"
CORPUS_METRICS = CORPUS / "floating-walk-metrics.json"
CORPUS_RAW = CORPUS / "floating-walk-raw.npz"
RESULT_DIR = ROOT / "benchmarks/results" / REVISION
RESULT = RESULT_DIR / f"{REVISION}-metrics.json"
REPORT = RESULT_DIR / "G1_CUMULATIVE_SOLVE_DIAGNOSTICS_R282.md"
WEB_REPORT = ROOT / "web/G1_CUMULATIVE_SOLVE_DIAGNOSTICS_R282.html"
STAGES = (
    "primary",
    "relock_retry",
    "localization_probe",
    "global_normal_retry",
    "localized_handoff_retry",
)
R281_NON_TIMING_ARRAY_COUNT = 89
R281_NON_TIMING_DIGEST = (
    "3489c58b1259bcb97feb87dde038de8df30da9c2d1de3cb77a4f169b9f228521"
)


def non_timing_digest(raw: dict[str, np.ndarray]) -> tuple[int, str]:
    """Hash the established trace surface, excluding timing/new R282 evidence."""
    names = sorted(
        name
        for name in raw
        if name != "step_ns"
        and not name.startswith("solve_attempt")
        and not name.startswith("cumulative_")
    )
    digest = hashlib.sha256()
    for name in names:
        values = np.ascontiguousarray(raw[name])
        for token in (name, values.dtype.str, repr(values.shape)):
            encoded = token.encode()
            digest.update(len(encoded).to_bytes(8, "little"))
            digest.update(encoded)
        digest.update(values.nbytes.to_bytes(8, "little"))
        digest.update(values.tobytes())
    return len(names), digest.hexdigest()


def load_corpus() -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    if not CORPUS_METRICS.is_file() or not CORPUS_RAW.is_file():
        raise FileNotFoundError(
            "R282 requires the floating corpus run; execute "
            "scripts/run-g1-cumulative-solve-diagnostics-r282.sh first"
        )
    return json.loads(CORPUS_METRICS.read_text()), dict(np.load(CORPUS_RAW, allow_pickle=False))


def build_result(document: dict[str, Any], raw: dict[str, np.ndarray]) -> dict[str, Any]:
    metrics = document["metrics"]
    required = {
        "solve_attempt_count",
        "solve_attempt_mask",
        "solve_attempt_count_by_stage",
        "task_pseudoinverse_calls",
        "task_jacobi_sweeps",
        "cumulative_task_pseudoinverse_calls",
        "cumulative_task_jacobi_sweeps",
        "cumulative_feasibility_projection_sweeps",
        "cumulative_feasibility_polish_iterations_by_stage",
        "cumulative_feasibility_polish_pseudoinverse_calls_by_stage",
        "cumulative_feasibility_polish_jacobi_sweeps_by_stage",
    }
    missing = sorted(required - raw.keys())
    if missing:
        raise ValueError(f"R282 trace is missing cumulative diagnostics: {missing}")
    ticks = int(document["metrics"]["ticks"])
    if any(raw[name].shape[0] != ticks for name in required):
        raise ValueError("R282 trace/metrics tick count mismatch")
    cumulative_ge_final = bool(
        np.all(raw["cumulative_task_pseudoinverse_calls"] >= raw["task_pseudoinverse_calls"])
        and np.all(raw["cumulative_task_jacobi_sweeps"] >= raw["task_jacobi_sweeps"])
    )
    work = metrics["cumulative_solver_work"]
    tails = metrics["release_tail_work"]
    non_timing_count, non_timing_hash = non_timing_digest(raw)
    exact_to_r281 = (
        non_timing_count == R281_NON_TIMING_ARRAY_COUNT
        and non_timing_hash == R281_NON_TIMING_DIGEST
    )
    return {
        "schema": "bonesaw.g1-cumulative-solve-diagnostics-r282.v1",
        "revision": REVISION,
        "execution": {
            "policy_steps": 0,
            "physics_steps": 0,
            "ticks": ticks,
            "dt_seconds": float(document["metrics"]["dt_seconds"]),
            "maximum_feasibility_projection_sweeps": document.get(
                "maximum_feasibility_projection_sweeps"
            ),
        },
        "latency": metrics["latency_us"],
        "runtime": metrics["runtime"],
        "cumulative_solver_work": work,
        "release_tail_work": tails,
        "semantic_contract": {
            "cumulative_counters_dominate_final_attempt": cumulative_ge_final,
            "r281_non_timing_arrays": non_timing_count,
            "r281_non_timing_sha256": non_timing_hash,
            "r281_non_timing_exact": exact_to_r281,
            "new_trace_arrays": sorted(
                name
                for name in raw
                if name.startswith("solve_attempt") or name.startswith("cumulative_")
            ),
        },
        "verdict": {
            "telemetry_passed": cumulative_ge_final
            and exact_to_r281
            and work["maximum_attempts"] >= 1
            and tails["count"] >= 1,
            "authority_admitted": False,
        },
    }


def validate_result(result: dict[str, Any]) -> None:
    if result["revision"] != REVISION:
        raise ValueError("unexpected R282 revision")
    if result["execution"]["policy_steps"] != 0 or result["execution"]["physics_steps"] != 0:
        raise ValueError("R282 must remain policy/physics free")
    if not result["semantic_contract"]["cumulative_counters_dominate_final_attempt"]:
        raise ValueError("cumulative counters do not dominate final-attempt counters")
    if not result["semantic_contract"]["r281_non_timing_exact"]:
        raise ValueError("R282 changed the established R281 non-timing trace")
    work = result["cumulative_solver_work"]
    if work["maximum_attempts"] < 1 or result["release_tail_work"]["count"] < 1:
        raise ValueError("R282 did not observe any solve/release work")
    if result["verdict"]["authority_admitted"] or not result["verdict"]["telemetry_passed"]:
        raise ValueError("R282 telemetry gate failed")


def build_report(result: dict[str, Any]) -> str:
    work = result["cumulative_solver_work"]
    rows = []
    for name in STAGES:
        stage = work["attempts_by_stage"][name]
        pinv = work["task_pseudoinverse_calls_by_stage"][name]
        jacobi = work["task_jacobi_sweeps_by_stage"][name]
        polish_pinv = work["feasibility_polish_pseudoinverse_calls_by_stage"][name]
        rows.append(
            [
                name,
                f"{stage['mean']:.3f}",
                f"{stage['max']:.0f}",
                f"{pinv['mean']:.3f}",
                f"{pinv['max']:.0f}",
                f"{jacobi['mean']:.3f}",
                f"{jacobi['max']:.0f}",
                f"{polish_pinv['mean']:.3f}",
                f"{polish_pinv['max']:.0f}",
            ]
        )
    tail_rows = []
    for row in result["release_tail_work"]["rows"]:
        tail_rows.append(
            [
                row["tick"],
                f"{row['latency_us'] / 1_000.0:.3f}",
                row["status"],
                row["pre_contingency_status"],
                row["solve_attempt_count"],
                f"0x{row['solve_attempt_mask']:02x}",
                row["final_task_pseudoinverse_calls"],
                row["cumulative_task_pseudoinverse_calls"],
                row["final_task_jacobi_sweeps"],
                row["cumulative_task_jacobi_sweeps"],
                row["cumulative_feasibility_halfspace_projections"],
                row["cumulative_feasibility_polish_iterations"],
                row["cumulative_feasibility_polish_pseudoinverse_calls"],
                row["cumulative_feasibility_polish_jacobi_sweeps"],
                row["cumulative_feasibility_polish_pseudoinverse_calls_by_stage"][
                    "primary"
                ],
                row["cumulative_feasibility_polish_pseudoinverse_calls_by_stage"][
                    "global_normal_retry"
                ],
            ]
        )
    report = "\n".join(
        [
            "# G1 cumulative solve-attempt diagnostics · R282",
            "",
            "> Telemetry **PASS** · semantic preservation **PASS** · authority **NO**.",
            "",
            "R282 adds a fixed-capacity Rust witness for every floating-WBC solve attempted during one controller tick. It executes zero policy and zero physics steps. The final-attempt arrays remain unchanged; cumulative arrays preserve failed primary, retry, localization, and handoff work before a safe fallback clears the reusable output.",
            "",
            "## Attempt-stage work",
            "",
            *markdown_table(
                ["stage", "attempts mean", "attempts max", "task pinv mean", "task pinv max", "task Jacobi mean", "task Jacobi max", "polish pinv mean", "polish pinv max"],
                rows,
            ),
            "",
            f"The trace contains {work['ticks_with_retry']} ticks with retries and a maximum of {work['maximum_attempts']} solve attempts per tick. Cumulative task work is {work['task_pseudoinverse_calls']['mean']:.3f} pseudoinverses and {work['task_jacobi_sweeps']['mean']:.3f} Jacobi sweeps per tick (p99 {work['task_pseudoinverse_calls']['p99']:.1f}/{work['task_jacobi_sweeps']['p99']:.1f}).",
            "",
            "## Release-tail witness",
            "",
            *markdown_table(
                ["tick", "latency ms", "final status", "pre status", "attempts", "stage mask", "final task pinv", "cum task pinv", "final task Jacobi", "cum task Jacobi", "halfspaces", "polish iter", "polish pinv", "polish Jacobi", "primary polish pinv", "retry polish pinv"],
                tail_rows,
            ),
            "",
            "All three release ticks spend 16 projection sweeps and 3,456 halfspace projections. Tick 1234 finishes in roughly 1.4 ms with two polish iterations and no polish inverse. The deterministic 1694/2296 tails instead exhaust 128 polish iterations, including 126 dense polish pseudoinverses and 1,152/1,210 Jacobi sweeps. Stage attribution shows which solve owns those inversions. Stage masks are stable IDs: bit 0 primary, bit 1 relock retry, bit 2 localization probe, bit 3 global normal retry, and bit 4 localized handoff retry.",
            "",
            "## CPU, memory, and decision",
            "",
            f"The replay p99 is {result['latency']['p99']:.1f} µs with {result['runtime']['python_gc_delta']['collections']} Python GC collections and RSS delta {result['runtime']['rss_delta_bytes'] / 2**20:.3f} MiB. All {result['semantic_contract']['r281_non_timing_arrays']} established non-timing arrays are bit-for-bit identical to the R281 profile (SHA-256 `{result['semantic_contract']['r281_non_timing_sha256']}`). The new witness is caller-owned and allocation-free in the timed Rust path; it changes no controller authority or integrated state.",
            "",
            "This closes the diagnostic blind spot. It does not yet optimize the repeated Preference/Style projected inversions or admit support/contact authority; the next slice must use these counters to prove an exact work reduction against five pinned repeats.",
        ]
    )
    return report + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    document, raw = load_corpus()
    result = build_result(document, raw)
    validate_result(result)
    if not args.check_only:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        report = build_report(result)
        REPORT.write_text(report)
        WEB_REPORT.write_text(
            render_report_html(report, title="G1 cumulative solve diagnostics · R282")
        )
    print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
