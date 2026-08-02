#!/usr/bin/env python3
"""Evaluate the rejected fused initial Jacobi energy scan."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact
from jacobi_discarded_pair_ab import relative_delta


METRICS = (
    "instructions_per_tick",
    "cycles_per_tick",
    "task_clock_per_tick",
    "p50_tick_us",
    "p99_tick_us",
    "max_tick_us",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", default="benchmarks/results/g1-jacobi-energy-scan-r61/control")
    parser.add_argument("--experiment", default="benchmarks/results/g1-jacobi-energy-scan-r61/experiment")
    parser.add_argument("--r54-baseline", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--r54-default", default="benchmarks/results/g1-jacobi-energy-scan-r61/r54-default")
    parser.add_argument("--output", default="benchmarks/results/g1-jacobi-energy-scan-r61")
    parser.add_argument("--web-report", default="web/JACOBI_ENERGY_SCAN_AB_R61.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    control_path = pathlib.Path(args.control) / "native-wbc-counter-metrics.json"
    experiment_path = pathlib.Path(args.experiment) / "native-wbc-counter-metrics.json"
    control = json.loads(control_path.read_text())
    experiment = json.loads(experiment_path.read_text())
    if control["semantic_report"] != experiment["semantic_report"]:
        raise ValueError("native semantic reports differ")

    baseline_raw = pathlib.Path(args.r54_baseline) / "oracle-wbc-admission-raw.npz"
    default_raw = pathlib.Path(args.r54_default) / "oracle-wbc-admission-raw.npz"
    with np.load(baseline_raw) as baseline, np.load(default_raw) as current:
        fields = sorted(set(baseline.files) - TIMING_ARRAYS)
        differences = [
            field for field in fields
            if field not in current or not arrays_byte_exact(baseline[field], current[field])
        ]
    if differences:
        raise ValueError(f"production r54 semantics changed: {differences}")

    rows = []
    for variant, source in (("production control", control), ("fused experiment", experiment)):
        row = {"variant": variant}
        for metric in METRICS:
            row[metric] = source["summary"][metric]["median"]
            row[metric + "_relative_span"] = source["summary"][metric]["relative_span"]
        rows.append(row)
    deltas = {metric: relative_delta(rows[0][metric], rows[1][metric]) for metric in METRICS}
    if deltas["instructions_per_tick"] < 0.0:
        raise ValueError("r61 policy expected the fusion experiment to remain rejected")

    hashes = {
        str(control_path): sha256(control_path),
        str(experiment_path): sha256(experiment_path),
        str(baseline_raw): sha256(baseline_raw),
        str(default_raw): sha256(default_raw),
    }
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": "REJECT",
        "reason": "the fused two-accumulator scan inhibits the efficient separate reduction and increases stable retired instructions",
        "production_default_uses_separate_scans": True,
        "native_semantic_reports_exact": True,
        "r54_non_timing_arrays_exact": True,
        "r54_semantic_field_count": len(fields),
        "variants": rows,
        "experiment_relative_deltas": deltas,
        "artifact_sha256": hashes,
    }
    (output / "jacobi-energy-scan-ab-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "jacobi-energy-scan-ab.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    added = rows[1]["instructions_per_tick"] - rows[0]["instructions_per_tick"]
    report = [
        "# Bonesaw fused Jacobi energy-scan A/B · r61",
        "",
        "## Decision · REJECT",
        "",
        f"Fusing the initial Frobenius reduction into the per-column energy scan preserves both results bit-for-bit, but adds {added:,.0f} retired instructions per native tick ({100 * deltas['instructions_per_tick']:+.3f}%). Production keeps the original two scans; the fusion remains disabled.",
        "",
        "> Removing a source-level matrix pass is not automatically less machine work. The original independent Frobenius reduction is compiler-friendly; the fused loop carries two dependent accumulators and loses enough reduction efficiency to outweigh one fewer scalar square.",
        "",
        "## Native five-run A/B",
        "",
    ]
    report += markdown_table(
        ["variant", "instructions/tick M", "cycles/tick M", "task-clock/tick ms", "p50 / p99 / max µs"],
        [[
            row["variant"], f"{row['instructions_per_tick'] / 1e6:.6f}",
            f"{row['cycles_per_tick'] / 1e6:.3f}", f"{row['task_clock_per_tick'] / 1e6:.3f}",
            f"{row['p50_tick_us']:.3f} / {row['p99_tick_us']:.3f} / {row['max_tick_us']:.3f}",
        ] for row in rows],
    )
    report += [
        "",
        "## Preservation and scope",
        "",
        "- A direct Rust witness confirms bit-exact Frobenius squared norm and every initial column energy between separate and fused scans.",
        "- Native control and experiment semantic reports are exactly equal, including status, residuals, margins, tracking, extrema, repeatability, and allocations.",
        f"- The unchanged production default passes r54 43/43 and keeps all {len(fields)} non-timing arrays byte-exact.",
        "- The experimental fusion is available only through `bonesaw-core/jacobi-energy-scan-experiment` and is disabled by default.",
        "- Stable instruction work decides this A/B; sequential-block latency and cycle medians remain observational host signals.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(["artifact", "SHA-256"], hashes.items())
    report_text = "\n".join(report) + "\n"
    (output / "JACOBI_ENERGY_SCAN_AB.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
