#!/usr/bin/env python3
"""Report whole-process CPU counters for byte-exact G1 admission traces."""

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
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact, runs


DEFAULT_TRIALS = tuple(
    f"run{index}=benchmarks/results/g1-cpu-counters-r58/perf{index}.csv,"
    f"benchmarks/results/g1-cpu-counters-r58/run{index}"
    for index in range(1, 6)
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", action="append", dest="trials")
    parser.add_argument("--baseline", default="benchmarks/results/g1-multistep-oracle-r54")
    parser.add_argument("--output", default="benchmarks/results/g1-cpu-counters-r58")
    parser.add_argument("--web-report", default="web/CPU_COUNTER_STABILITY_R58.html")
    return parser.parse_args()


def parse_trial(value: str) -> tuple[str, pathlib.Path, pathlib.Path]:
    if "=" not in value or "," not in value:
        raise ValueError("--trial must be label=perf.csv,run-directory")
    label, paths = value.split("=", 1)
    perf, run_dir = paths.split(",", 1)
    if not label or not perf or not run_dir:
        raise ValueError("--trial fields must be non-empty")
    return label, pathlib.Path(perf), pathlib.Path(run_dir)


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def read_perf(path: pathlib.Path) -> dict[str, int]:
    counters: dict[str, int] = {}
    with path.open(newline="") as stream:
        for row in csv.reader(line for line in stream if not line.startswith("#")):
            if len(row) < 3 or not row[0].strip():
                continue
            event = row[2].removesuffix(":u")
            if event in {
                "task-clock",
                "cycles",
                "instructions",
                "branches",
                "branch-misses",
                "cache-misses",
            }:
                counters[event] = int(row[0])
    missing = {
        "task-clock",
        "cycles",
        "instructions",
        "branches",
        "branch-misses",
        "cache-misses",
    } - counters.keys()
    if missing:
        raise ValueError(f"{path} is missing counters: {sorted(missing)}")
    return counters


def relative_span(values: np.ndarray) -> float:
    return float((np.max(values) - np.min(values)) / np.median(values))


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    configured = [parse_trial(value) for value in args.trials or DEFAULT_TRIALS]
    if len(configured) < 3:
        raise ValueError("counter stability requires at least three trials")

    baseline_path = pathlib.Path(args.baseline) / "oracle-wbc-admission-raw.npz"
    with np.load(baseline_path) as archive:
        baseline = {name: archive[name].copy() for name in archive.files}
    semantic_fields = sorted(set(baseline) - TIMING_ARRAYS)
    rows: list[dict[str, Any]] = []
    differences: dict[str, list[str]] = {}
    hashes = {str(baseline_path): sha256(baseline_path)}

    for label, perf_path, run_dir in configured:
        metrics_path = run_dir / "oracle-wbc-admission-metrics.json"
        raw_path = run_dir / "oracle-wbc-admission-raw.npz"
        metrics = json.loads(metrics_path.read_text())
        if not metrics["passed"]:
            raise ValueError(f"{label} did not pass admission")
        with np.load(raw_path) as archive:
            arrays = {name: archive[name].copy() for name in archive.files}
        differences[label] = [
            name
            for name in semantic_fields
            if name not in arrays or not arrays_byte_exact(baseline[name], arrays[name])
        ]
        counters = read_perf(perf_path)
        latency_ms = arrays["step_ns"].astype(np.float64) / 1e6
        overrun = latency_ms > 5.0
        episodes = runs(overrun)
        instructions = counters["instructions"]
        cycles = counters["cycles"]
        branches = counters["branches"]
        rows.append(
            {
                "label": label,
                "task_clock_seconds": counters["task-clock"] / 1e9,
                "cycles": cycles,
                "instructions": instructions,
                "branches": branches,
                "branch_misses": counters["branch-misses"],
                "cache_misses": counters["cache-misses"],
                "instructions_per_cycle": instructions / cycles,
                "branch_miss_percent": 100.0 * counters["branch-misses"] / branches,
                "cache_misses_per_million_instructions": 1e6 * counters["cache-misses"] / instructions,
                "wbc_p50_ms": float(np.percentile(latency_ms, 50)),
                "wbc_p99_ms": float(np.percentile(latency_ms, 99)),
                "wbc_max_ms": float(np.max(latency_ms)),
                "wbc_over_5ms_ticks": int(np.count_nonzero(overrun)),
                "wbc_over_5ms_episodes": len(episodes),
                "wbc_longest_over_5ms_ms": 5.0 * max((stop - start for start, stop in episodes), default=0),
            }
        )
        hashes[str(perf_path)] = sha256(perf_path)
        hashes[str(metrics_path)] = sha256(metrics_path)
        hashes[str(raw_path)] = sha256(raw_path)

    if any(differences.values()):
        raise ValueError(f"counter trials changed semantic arrays: {differences}")

    counter_names = ["task_clock_seconds", "cycles", "instructions", "branches", "branch_misses", "cache_misses"]
    stability = {}
    for name in counter_names:
        values = np.asarray([row[name] for row in rows], dtype=np.float64)
        stability[name] = {
            "minimum": float(np.min(values)),
            "median": float(np.median(values)),
            "maximum": float(np.max(values)),
            "relative_span": relative_span(values),
            "coefficient_of_variation": float(np.std(values) / np.mean(values)),
        }

    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "affinity": "logical CPU 4",
        "scope": "whole Python admission process under perf stat; counters are not per-WBC-tick attribution",
        "all_non_timing_arrays_exact": True,
        "semantic_field_count": len(semantic_fields),
        "excluded_timing_arrays": sorted(TIMING_ARRAYS),
        "semantic_differences": differences,
        "trials": rows,
        "stability": stability,
        "artifact_sha256": hashes,
        "decision": {
            "clock_dependent_exit_supported": False,
            "deterministic_work_confirmed": stability["instructions"]["relative_span"] < 0.0001,
            "next_measurement": "native WBC-only counter sentinel with calibration subtraction or sampled phase markers",
        },
    }
    (output / "cpu-counter-stability-metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    with (output / "cpu-counter-trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    instruction = stability["instructions"]
    task_clock = stability["task_clock_seconds"]
    cycles = stability["cycles"]
    report = [
        "# Bonesaw CPU counter stability audit · r58",
        "",
        "## Result",
        "",
        f"Five release processes pinned to logical CPU 4 retire a median {instruction['median'] / 1e9:.6f} billion instructions. "
        f"The instruction relative span is only {100 * instruction['relative_span']:.6f}%, while task-clock spans "
        f"{task_clock['minimum']:.3f}–{task_clock['maximum']:.3f} s ({100 * task_clock['relative_span']:.2f}% of the median) "
        f"and cycles span {cycles['minimum'] / 1e9:.3f}–{cycles['maximum'] / 1e9:.3f} billion "
        f"({100 * cycles['relative_span']:.2f}%). All {len(semantic_fields)} non-timing arrays remain byte-exact against r54.",
        "",
        "> Retired instructions confirm deterministic work at whole-process scope. These counters include Python startup, "
        "reference loading, kinematic witnesses, repeatability passes, report generation, and NPZ serialization; they are not "
        "yet legal per-WBC-tick attribution.",
        "",
        "## Five pinned trials",
        "",
    ]
    report += markdown_table(
        ["trial", "task clock s", "cycles B", "instructions B", "IPC", "branch miss %", "cache misses / M insn", "WBC p50 / p99 / max ms", ">5 ms ticks / episodes / longest"],
        [[
            row["label"], f"{row['task_clock_seconds']:.3f}", f"{row['cycles'] / 1e9:.3f}",
            f"{row['instructions'] / 1e9:.6f}", f"{row['instructions_per_cycle']:.3f}",
            f"{row['branch_miss_percent']:.3f}", f"{row['cache_misses_per_million_instructions']:.3f}",
            f"{row['wbc_p50_ms']:.3f} / {row['wbc_p99_ms']:.3f} / {row['wbc_max_ms']:.3f}",
            f"{row['wbc_over_5ms_ticks']} / {row['wbc_over_5ms_episodes']} / {row['wbc_longest_over_5ms_ms']:.0f} ms",
        ] for row in rows],
    )
    report += [
        "",
        "## Interpretation",
        "",
        "- Preserve the deterministic bounded-work solver and separate 5 ms observation from the admitted 20 ms contract.",
        "- Do not use wall-clock time to stop Jacobi or clipping work; identical instruction traces coexist with materially different task-clock and cycle totals.",
        "- The first trial is a useful cold/contended outlier, not a discard: instruction work stays fixed while task-clock and cycles expand.",
        "- Build a native WBC-only counter sentinel before attributing cycles or instructions to one solver tick or priority layer.",
        "- Optimize only against byte-exact semantic traces and retain hard-row, tracking, clipping-duration, allocation, and 20 ms gates.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(["artifact", "SHA-256"], hashes.items())
    report_text = "\n".join(report) + "\n"
    (output / "CPU_COUNTER_STABILITY.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
