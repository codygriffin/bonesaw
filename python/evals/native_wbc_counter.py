#!/usr/bin/env python3
"""Measure marginal native G1 WBC counters with setup-process subtraction."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import subprocess
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_counter_stability import read_perf
from cpu_reference_report import markdown_table, render_report_html


COUNTERS = (
    "task-clock",
    "cycles",
    "instructions",
    "branches",
    "branch-misses",
    "cache-misses",
)
TIMING_FIELDS = {"mean_tick_us", "p50_tick_us", "p99_tick_us", "max_tick_us"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="target/release/bonesaw-eval")
    parser.add_argument("--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
    parser.add_argument("--ticks", type=int, default=2_000)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--cpu", type=int, default=4)
    parser.add_argument("--output", default="benchmarks/results/g1-native-wbc-counter-r59")
    parser.add_argument("--web-report", default="web/NATIVE_WBC_COUNTER_R59.html")
    parser.add_argument("--render-only", action="store_true")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def semantic_projection(report: dict[str, Any]) -> dict[str, Any]:
    wbc = report["floating_dynamic_wbc"]
    return {
        "model": report["model"],
        "program_fingerprint": report["program_fingerprint"],
        "scope": report["scope"],
        "floating_dynamic_wbc": {
            key: value for key, value in wbc.items() if key not in TIMING_FIELDS
        },
    }


def run_perf(
    *,
    binary: pathlib.Path,
    model: pathlib.Path,
    ticks: int,
    cpu: int,
    perf_path: pathlib.Path,
    json_path: pathlib.Path,
) -> tuple[dict[str, int], dict[str, Any]]:
    command = [
        "perf",
        "stat",
        "-x,",
        "-o",
        str(perf_path),
        "-e",
        ",".join(COUNTERS),
        "taskset",
        "-c",
        str(cpu),
        str(binary),
        "--model",
        str(model),
        "--ticks",
        str(ticks),
        "--floating-wbc-only",
        "--json",
    ]
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    report = json.loads(completed.stdout)
    json_path.write_text(json.dumps(report, indent=2) + "\n")
    return read_perf(perf_path), report


def summarize(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    minimum = float(np.min(array))
    median = float(np.median(array))
    maximum = float(np.max(array))
    mean = float(np.mean(array))
    span = maximum - minimum
    return {
        "minimum": minimum,
        "median": median,
        "maximum": maximum,
        "p95": float(np.percentile(array, 95)),
        "relative_span": 0.0 if median == 0.0 and span == 0.0 else span / median,
        "coefficient_of_variation": 0.0 if mean == 0.0 else float(np.std(array) / mean),
    }


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    web_report = pathlib.Path(args.web_report)
    if args.render_only:
        report_text = (output / "NATIVE_WBC_COUNTER.md").read_text()
        web_report.parent.mkdir(parents=True, exist_ok=True)
        web_report.write_text(render_report_html(report_text))
        return
    if args.ticks < 2:
        raise ValueError("--ticks must be at least 2 for setup subtraction")
    if args.repeats < 3:
        raise ValueError("--repeats must be at least 3")
    binary = pathlib.Path(args.binary).resolve()
    model = pathlib.Path(args.model).resolve()
    if not binary.is_file() or not model.is_file():
        raise FileNotFoundError("release binary and model must exist")
    output.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    semantic_reports: list[dict[str, Any]] = []
    hashes: dict[str, str] = {
        str(binary): sha256(binary),
        str(model): sha256(model),
    }
    for repeat in range(1, args.repeats + 1):
        # Alternate the one-tick setup process before/after the long process so
        # monotonic thermal or desktop-load drift does not always bias the same
        # side of the subtraction.
        short_perf = output / f"short-{repeat}-perf.csv"
        short_json = output / f"short-{repeat}.json"
        long_perf = output / f"long-{repeat}-perf.csv"
        long_json = output / f"long-{repeat}.json"
        if repeat % 2:
            short_counters, _ = run_perf(
                binary=binary, model=model, ticks=1, cpu=args.cpu,
                perf_path=short_perf, json_path=short_json,
            )
            long_counters, long_report = run_perf(
                binary=binary, model=model, ticks=args.ticks, cpu=args.cpu,
                perf_path=long_perf, json_path=long_json,
            )
        else:
            long_counters, long_report = run_perf(
                binary=binary, model=model, ticks=args.ticks, cpu=args.cpu,
                perf_path=long_perf, json_path=long_json,
            )
            short_counters, _ = run_perf(
                binary=binary, model=model, ticks=1, cpu=args.cpu,
                perf_path=short_perf, json_path=short_json,
            )
        semantic_reports.append(semantic_projection(long_report))
        denominator = args.ticks - 1
        marginal = {
            name.replace("-", "_") + "_per_tick":
            (long_counters[name] - short_counters[name]) / denominator
            for name in COUNTERS
        }
        wbc = long_report["floating_dynamic_wbc"]
        rows.append({
            "repeat": repeat,
            **marginal,
            "instructions_per_cycle": marginal["instructions_per_tick"] / marginal["cycles_per_tick"],
            "p50_tick_us": wbc["p50_tick_us"],
            "p99_tick_us": wbc["p99_tick_us"],
            "max_tick_us": wbc["max_tick_us"],
            "allocations_per_tick": wbc["allocations_per_tick"],
            "allocated_bytes_per_tick": wbc["allocated_bytes_per_tick"],
        })
        for path in (short_perf, short_json, long_perf, long_json):
            hashes[str(path)] = sha256(path)

    first_semantic = semantic_reports[0]
    semantic_exact = all(report == first_semantic for report in semantic_reports[1:])
    if not semantic_exact:
        raise ValueError("native long-run semantic reports differ")
    wbc_semantic = first_semantic["floating_dynamic_wbc"]
    if wbc_semantic["infeasible_ticks"] != 0 or not wbc_semantic["bitwise_repeat"]:
        raise ValueError("native sentinel did not preserve feasibility/repeatability")
    if wbc_semantic["allocations_per_tick"] != 0 or wbc_semantic["allocated_bytes_per_tick"] != 0:
        raise ValueError("native sentinel allocated inside the measured solve")

    summary = {
        key: summarize([float(row[key]) for row in rows])
        for key in rows[0]
        if key != "repeat"
    }
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "binary": str(binary),
        "affinity": f"logical CPU {args.cpu}",
        "ticks_per_long_process": args.ticks,
        "setup_ticks_per_short_process": 1,
        "repeats": args.repeats,
        "counter_scope": "long-process minus adjacent one-tick process, divided by additional ticks; native marginal upper bound including per-loop timing and semantic aggregation around each Rust WBC solve",
        "not_claimed": "hardware-counter sampling of an individual solve or equivalence to the richer r54 G1 task/support corpus",
        "semantic_reports_exact": semantic_exact,
        "semantic_report": first_semantic,
        "trials": rows,
        "summary": summary,
        "artifact_sha256": hashes,
    }
    metrics_path = output / "native-wbc-counter-metrics.json"
    metrics_path.write_text(json.dumps(result, indent=2) + "\n")
    with (output / "native-wbc-counter-trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    instructions = summary["instructions_per_tick"]
    cycles = summary["cycles_per_tick"]
    task_clock = summary["task_clock_per_tick"]
    p99 = summary["p99_tick_us"]
    report = [
        "# Bonesaw native WBC counter sentinel · r59",
        "",
        "## Result",
        "",
        f"Five native release trials on the official 23-DOF G1 model each execute {args.ticks:,} fixed-shape floating WBC solves. "
        f"After subtracting an adjacent one-tick process, the marginal median is {instructions['median'] / 1e6:.3f} million instructions, "
        f"{cycles['median'] / 1e6:.3f} million cycles, and {task_clock['median'] / 1e6:.3f} ms task-clock per additional tick. "
        f"Native measured latency is {p99['median']:.3f} µs p99 median across trials. Every long trial has the same non-timing semantic report, "
        "zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.",
        "",
        "> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.",
        "",
        "## Repeated native trials",
        "",
    ]
    report += markdown_table(
        ["trial", "instructions / tick M", "cycles / tick M", "task-clock / tick ms", "IPC", "p50 / p99 / max µs", "alloc calls / bytes"],
        [[
            row["repeat"], f"{row['instructions_per_tick'] / 1e6:.3f}",
            f"{row['cycles_per_tick'] / 1e6:.3f}", f"{row['task_clock_per_tick'] / 1e6:.3f}",
            f"{row['instructions_per_cycle']:.3f}",
            f"{row['p50_tick_us']:.3f} / {row['p99_tick_us']:.3f} / {row['max_tick_us']:.3f}",
            f"{row['allocations_per_tick']:.0f} / {row['allocated_bytes_per_tick']:.0f}",
        ] for row in rows],
    )
    report += [
        "",
        "## Scope and decision",
        "",
        f"- Model fingerprint: `{first_semantic['program_fingerprint']}`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.",
        "- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.",
        "- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.",
        "- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.",
        "- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.",
        "",
        "## Artifact integrity",
        "",
    ]
    report += markdown_table(["artifact", "SHA-256"], hashes.items())
    report_text = "\n".join(report) + "\n"
    (output / "NATIVE_WBC_COUNTER.md").write_text(report_text)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))


if __name__ == "__main__":
    main()
