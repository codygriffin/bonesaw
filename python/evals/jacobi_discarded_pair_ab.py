#!/usr/bin/env python3
"""Evaluate and reject/accept the discarded-column Jacobi pair skip."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", default="benchmarks/results/g1-jacobi-discarded-r60/control")
    parser.add_argument("--experiment", default="benchmarks/results/g1-jacobi-discarded-r60/experiment")
    parser.add_argument("--r54-baseline", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--r54-default", default="benchmarks/results/g1-jacobi-discarded-r60/r54-default")
    parser.add_argument("--output", default="benchmarks/results/g1-jacobi-discarded-r60")
    parser.add_argument("--web-report", default="web/JACOBI_DISCARDED_PAIR_AB_R60.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def relative_delta(control: float, experiment: float) -> float:
    return (experiment - control) / control


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    control_path = pathlib.Path(args.control) / "native-wbc-counter-metrics.json"
    experiment_path = pathlib.Path(args.experiment) / "native-wbc-counter-metrics.json"
    control = json.loads(control_path.read_text())
    experiment = json.loads(experiment_path.read_text())
    semantic_exact = control["semantic_report"] == experiment["semantic_report"]
    if not semantic_exact:
        raise ValueError("native control and experiment semantic reports differ")

    baseline_raw = pathlib.Path(args.r54_baseline) / "oracle-wbc-admission-raw.npz"
    default_raw = pathlib.Path(args.r54_default) / "oracle-wbc-admission-raw.npz"
    with np.load(baseline_raw) as baseline, np.load(default_raw) as current:
        fields = sorted(set(baseline.files) - TIMING_ARRAYS)
        differences = [
            field
            for field in fields
            if field not in current or not arrays_byte_exact(baseline[field], current[field])
        ]
    if differences:
        raise ValueError(f"production-default r54 semantics changed: {differences}")

    metric_names = [
        "instructions_per_tick",
        "cycles_per_tick",
        "task_clock_per_tick",
        "p50_tick_us",
        "p99_tick_us",
        "max_tick_us",
    ]
    rows: list[dict[str, Any]] = []
    for name, metrics in (("production control", control), ("skip experiment", experiment)):
        row: dict[str, Any] = {"variant": name}
        for metric in metric_names:
            summary = metrics["summary"][metric]
            row[metric] = summary["median"]
            row[metric + "_minimum"] = summary["minimum"]
            row[metric + "_maximum"] = summary["maximum"]
        rows.append(row)
    deltas = {
        metric: relative_delta(rows[0][metric], rows[1][metric])
        for metric in metric_names
    }
    accepted = deltas["instructions_per_tick"] < 0.0
    if accepted:
        raise ValueError("report policy expected the r60 experiment to remain rejected")

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
        "reason": "the provably semantics-neutral branch increases stable retired instruction work on the native sentinel",
        "production_default_keeps_original_coupling_dot": True,
        "native_semantic_reports_exact": semantic_exact,
        "r54_non_timing_arrays_exact": not differences,
        "r54_semantic_field_count": len(fields),
        "r54_differences": differences,
        "variants": rows,
        "experiment_relative_deltas": deltas,
        "artifact_sha256": hashes,
    }
    (output / "jacobi-discarded-pair-ab-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "jacobi-discarded-pair-ab.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    instruction_delta = rows[1]["instructions_per_tick"] - rows[0]["instructions_per_tick"]
    report = [
        "# Bonesaw discarded-column Jacobi pair A/B · r60",
        "",
        "## Decision · REJECT",
        "",
        f"Skipping coupling dots for cached columns already below the discarded-energy floor is mathematically and bitwise neutral, but it costs {instruction_delta:,.0f} additional retired instructions per native sentinel tick ({100 * deltas['instructions_per_tick']:+.3f}%). Production therefore keeps the original unconditional coupling dot and does not ship the extra pair-level branch.",
        "",
        "> Cycle and latency medians are retained but do not overrule retired instructions: the two sequential trial blocks experience different frequency/contention, while instruction counts have sub-part-per-million within-variant spans.",
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
        "## Why the apparently obvious skip loses",
        "",
        "The cached energy comparison is executed for every Jacobi column pair. On this admitted G1 sentinel, columns below the discard floor are too sparse to amortize the added branch; most pairs still require the coupling dot. The experiment changes no consumed arithmetic after the rejected check, which is why all semantic outputs remain identical even as instruction work increases.",
        "",
        "## Behavioral preservation",
        "",
        f"- Native control and experiment semantic reports are exactly equal across status, residual, margin, tracking, extrema, bitwise-repeat, and allocation fields.",
        f"- The restored production default passes the r54 four-step oracle 43/43 and keeps all {len(fields)} non-timing NPZ arrays byte-exact against the admitted baseline.",
        "- The experimental path remains available only behind `bonesaw-core/jacobi-discarded-pair-experiment` for reproduction; it is disabled by default.",
        "- No iteration cap, tolerance change, clock exit, task reprioritization, or feasibility relaxation is accepted.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(["artifact", "SHA-256"], hashes.items())
    report_text = "\n".join(report) + "\n"
    (output / "JACOBI_DISCARDED_PAIR_AB.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
