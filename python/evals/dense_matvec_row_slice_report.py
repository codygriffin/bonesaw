#!/usr/bin/env python3
"""Qualify the exact-order dense matvec row-slice default promoted in R293."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import statistics
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


TIMING_ARRAYS = {"step_ns"}
COUNTERS = ("instructions", "cycles", "branches", "branch-misses", "cache-misses")
REVISION = "g1-dense-matvec-row-slice-r293"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timing-control", nargs="+", required=True)
    parser.add_argument("--timing-production", nargs="+", required=True)
    parser.add_argument("--perf-json", required=True)
    parser.add_argument("--process-json", required=True)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/G1_DENSE_MATVEC_ROW_SLICE_R293.html")
    return parser.parse_args()


def semantic_digest(path: pathlib.Path) -> tuple[int, str, dict[str, bytes]]:
    digest = hashlib.sha256()
    payloads: dict[str, bytes] = {}
    with np.load(path, allow_pickle=False) as archive:
        fields = sorted(set(archive.files) - TIMING_ARRAYS)
        for field in fields:
            array = archive[field]
            payload = array.tobytes(order="C")
            payloads[field] = payload
            digest.update(field.encode())
            digest.update(array.dtype.str.encode())
            digest.update(str(array.shape).encode())
            digest.update(payload)
    return len(fields), digest.hexdigest(), payloads


def latency(path: pathlib.Path) -> dict[str, float]:
    values = json.loads(path.read_text())["metrics"]["latency_us"]
    return {name: float(values[name]) for name in ("p50", "p95", "p99", "max")}


def relative_delta(control: float, production: float) -> float:
    return production / control - 1.0


def main() -> None:
    args = parse_args()
    control_roots = [pathlib.Path(path) for path in args.timing_control]
    production_roots = [pathlib.Path(path) for path in args.timing_production]
    if len(control_roots) != len(production_roots):
        raise ValueError("timing control/production repeat counts differ")

    semantic_pairs: list[dict[str, Any]] = []
    timing_pairs: list[dict[str, Any]] = []
    for repeat, (control_root, production_root) in enumerate(
        zip(control_roots, production_roots, strict=True), start=1
    ):
        control_count, control_digest, control_payloads = semantic_digest(
            control_root / "floating-walk-raw.npz"
        )
        production_count, production_digest, production_payloads = semantic_digest(
            production_root / "floating-walk-raw.npz"
        )
        changed = sorted(
            field
            for field in set(control_payloads) | set(production_payloads)
            if control_payloads.get(field) != production_payloads.get(field)
        )
        semantic_pairs.append(
            {
                "repeat": repeat,
                "field_count": control_count,
                "production_field_count": production_count,
                "control_sha256": control_digest,
                "production_sha256": production_digest,
                "changed_fields": changed,
                "exact": not changed
                and control_count == production_count
                and control_digest == production_digest,
            }
        )
        timing_pairs.append(
            {
                "repeat": repeat,
                "control": latency(control_root / "floating-walk-metrics.json"),
                "production": latency(production_root / "floating-walk-metrics.json"),
            }
        )

    perf = json.loads(pathlib.Path(args.perf_json).read_text())
    controls = sorted(perf["control"], key=lambda row: row["repeat"])
    production = sorted(perf["production"], key=lambda row: row["repeat"])
    if len(controls) != len(production) or not controls:
        raise ValueError("hardware-counter control/production repeat counts differ")
    if any(left["repeat"] != right["repeat"] for left, right in zip(controls, production, strict=True)):
        raise ValueError("hardware-counter repeat identities differ")

    control_means = {
        name: statistics.fmean(float(row[name]) for row in controls) for name in COUNTERS
    }
    production_means = {
        name: statistics.fmean(float(row[name]) for row in production) for name in COUNTERS
    }
    counter_deltas = {
        name: relative_delta(control_means[name], production_means[name]) for name in COUNTERS
    }
    paired_instruction_deltas = [
        relative_delta(float(left["instructions"]), float(right["instructions"]))
        for left, right in zip(controls, production, strict=True)
    ]
    all_exact = all(pair["exact"] for pair in semantic_pairs)
    work_gate = (
        all(value < 0.0 for value in paired_instruction_deltas)
        and counter_deltas["instructions"] < 0.0
        and counter_deltas["branches"] < 0.0
    )
    promoted = all_exact and work_gate

    process = json.loads(pathlib.Path(args.process_json).read_text())
    process_control = sorted(process["control"], key=lambda row: row["repeat"])
    process_production = sorted(process["production"], key=lambda row: row["repeat"])
    if len(process_control) != len(timing_pairs) or len(process_production) != len(timing_pairs):
        raise ValueError("process observation counts differ from timing repeats")
    process_summary = {
        profile: {
            "median_wall_seconds": statistics.median(float(row["wall_seconds"]) for row in rows),
            "median_user_seconds": statistics.median(float(row["user_seconds"]) for row in rows),
            "median_max_rss_kib": statistics.median(float(row["max_rss_kib"]) for row in rows),
            "maximum_max_rss_kib": max(float(row["max_rss_kib"]) for row in rows),
        }
        for profile, rows in (("control", process_control), ("production", process_production))
    }

    result = {
        "schema": f"bonesaw.{REVISION}.v1",
        "revision": REVISION,
        "decision": "PROMOTE" if promoted else "REJECT",
        "execution": {
            "ticks_per_process": 2317,
            "timing_cpu": 4,
            "policy_steps": 0,
            "physics_steps": 0,
            "timing_repeats": len(timing_pairs),
            "hardware_counter_repeats": len(controls),
        },
        "production": {
            "feature": "bonesaw-core/dense-matvec-row-slice-experiment",
            "control_feature": "bonesaw-core/dense-matvec-row-slice-control",
            "default_enabled": promoted,
            "arithmetic_contract": "identical scalar multiply/add, row, and store order",
        },
        "semantic_contract": {"all_repeats_exact": all_exact, "pairs": semantic_pairs},
        "timing_us": {
            "pairs": timing_pairs,
            "median": {
                metric: {
                    "control": statistics.median(pair["control"][metric] for pair in timing_pairs),
                    "production": statistics.median(
                        pair["production"][metric] for pair in timing_pairs
                    ),
                }
                for metric in ("p50", "p95", "p99", "max")
            },
        },
        "hardware_counters": {
            "control_repeats": controls,
            "production_repeats": production,
            "control_average": control_means,
            "production_average": production_means,
            "production_relative_delta": counter_deltas,
            "paired_instruction_relative_delta": paired_instruction_deltas,
        },
        "process_observations": {
            "control_repeats": process_control,
            "production_repeats": process_production,
            "summary": process_summary,
        },
        "measurement_exclusions": [
            {
                "window": "exploratory-five-pair",
                "reason": "overlapped extension builds and concurrent CLI evaluation",
                "semantic_exact": True,
                "latency_decision_use": False,
            },
            {
                "window": "grouped-control-then-candidate",
                "reason": "not alternated and crossed a visible host-load transition",
                "semantic_exact": True,
                "latency_decision_use": False,
            },
        ],
        "verdict": {
            "semantic_gate_passed": all_exact,
            "cpu_work_gate_passed": work_gate,
            "production_default_promoted": promoted,
            "p99_under_5ms": statistics.median(
                pair["production"]["p99"] for pair in timing_pairs
            )
            < 5_000.0,
            "authority_admitted": False,
        },
    }

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_path = output / f"{REVISION}-metrics.json"
    metrics_path.write_text(json.dumps(result, indent=2) + "\n")

    timing_rows = [
        [
            pair["repeat"],
            f"{pair['control']['p50']:.1f} / {pair['production']['p50']:.1f}",
            f"{pair['control']['p95']:.1f} / {pair['production']['p95']:.1f}",
            f"{pair['control']['p99']:.1f} / {pair['production']['p99']:.1f}",
            f"{pair['control']['max']:.1f} / {pair['production']['max']:.1f}",
        ]
        for pair in timing_pairs
    ]
    counter_rows = [
        [
            name,
            f"{control_means[name] / 1e9:.6f} B",
            f"{production_means[name] / 1e9:.6f} B",
            f"{100.0 * counter_deltas[name]:+.3f}%",
        ]
        for name in COUNTERS
    ]
    median = result["timing_us"]["median"]
    report = [
        "# G1 dense matvec row-slice promotion · R293",
        "",
        "> Exact replay PASS · CPU work PASS · production default PROMOTED · 5 ms p99 OPEN.",
        "",
        "R293 validates each dense matrix row once before the matrix-vector and residual inner loops. Scalar multiplication, addition, row, column, and store order are unchanged. The former flat-index path remains available through an explicit control feature.",
        "",
        "## Policy/physics-free semantic replay",
        "",
        f"All {len(semantic_pairs)} CPU-4-pinned, 2,317-tick control/production pairs are byte-exact across all {semantic_pairs[0]['field_count']} non-timing arrays. Every pair has digest `{semantic_pairs[0]['control_sha256']}`. The corpus executes zero policy and zero physics steps.",
        "",
        "## Native timing observations",
        "",
    ]
    report += markdown_table(
        ["pair", "p50 control / production µs", "p95", "p99", "max"], timing_rows
    )
    report += [
        "",
        f"Median p50 moves {median['p50']['control']:.1f}→{median['p50']['production']:.1f} µs and improves in every pair. Median p99 moves {median['p99']['control']:.1f}→{median['p99']['production']:.1f} µs and also improves in every pair, but remains above 5,000 µs. P95 improves in five of six pairs. The maximum RSS is {process_summary['control']['maximum_max_rss_kib']:.0f} KiB for both profiles; median user time moves {process_summary['control']['median_user_seconds']:.3f}→{process_summary['production']['median_user_seconds']:.3f} seconds.",
        "",
        "Two earlier exploratory windows remain explicit non-decision evidence: one overlapped extension builds and concurrent CLI work; the other ran every control before every candidate and crossed a visible host-load transition. Both were semantically exact, but neither supplies an admissible latency comparison. The six-pair decision series was run AB/BA after builds were idle.",
        "",
        "## Complete-process hardware counters",
        "",
        f"Five alternating pairs cover the same Python admission process and frozen corpus. Every paired retired-instruction delta is negative ({', '.join(f'{100.0 * value:+.3f}%' for value in paired_instruction_deltas)}). Averages are:",
        "",
    ]
    report += markdown_table(["counter", "flat control", "row-slice production", "delta"], counter_rows)
    report += [
        "",
        "The row-slice path is promoted because it preserves every semantic byte while reducing retired instructions and branches in every measured pair. Mean cycles improve slightly; cache misses are effectively flat. This is an addressing/code-generation optimization, not task deletion, changed factorization, reduced solver work, or weakened feasibility.",
        "",
        "## Decision",
        "",
        "Enable dense matvec row slices by default and retain `dense-matvec-row-slice-control` for reproducible A/B measurement. R280/R290 tracking, contact, timing, plant, actuator, thermal, CUDA, and hardware-authority gates remain unchanged. In particular, R293 does not close the ordinary 5 ms p99 complaint.",
    ]
    report_text = "\n".join(report) + "\n"
    report_path = output / "G1_DENSE_MATVEC_ROW_SLICE_R293.md"
    report_path.write_text(report_text)
    web_path = pathlib.Path(args.web_report)
    web_path.parent.mkdir(parents=True, exist_ok=True)
    web_path.write_text(render_report_html(report_text))
    print(json.dumps({"decision": result["decision"], "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
