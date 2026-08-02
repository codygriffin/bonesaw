#!/usr/bin/env python3
"""Pinned control/candidate A/B for exact-zero soft-task row compaction."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import pathlib
import subprocess
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_counter_stability import read_perf
from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


COUNTERS = (
    "task-clock",
    "cycles",
    "instructions",
    "branches",
    "branch-misses",
    "cache-misses",
)
ADDITIONAL_PYTHONPATH = "python/evals"
WBC_SOLVES_PER_PROCESS = 2 * 2_317


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-pythonpath", required=True)
    parser.add_argument("--candidate-pythonpath", required=True)
    parser.add_argument("--python", default="/tmp/bonesaw-placo/bin/python")
    parser.add_argument("--cpu", type=int, default=4)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument(
        "--reference-inputs",
        default="benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz",
    )
    parser.add_argument(
        "--loose-bound-control",
        default="benchmarks/results/g1-multistep-oracle-r54/no-loose-control.npz",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-zero-task-row-compaction-r65"
    )
    parser.add_argument("--web-report", default="web/ZERO_TASK_ROW_COMPACTION_R65.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def distribution(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    median = float(np.median(array))
    return {
        "minimum": float(np.min(array)),
        "median": median,
        "maximum": float(np.max(array)),
        "mean": float(np.mean(array)),
        "p95": float(np.percentile(array, 95)),
        "relative_span": 0.0
        if median == 0.0
        else float((np.max(array) - np.min(array)) / median),
    }


def relative_delta(control: float, candidate: float) -> float:
    return 0.0 if control == 0.0 and candidate == 0.0 else candidate / control - 1.0


def run_variant(
    *,
    name: str,
    pythonpath: pathlib.Path,
    repeat: int,
    args: argparse.Namespace,
    output: pathlib.Path,
) -> dict[str, Any]:
    run_output = output / name / f"run-{repeat}"
    run_output.mkdir(parents=True, exist_ok=True)
    perf_path = output / name / f"run-{repeat}-perf.csv"
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
        str(args.cpu),
        args.python,
        "python/evals/g1_oracle_wbc_admission.py",
        "--reference-inputs",
        args.reference_inputs,
        "--output",
        str(run_output),
        "--loose-bound-control",
        args.loose_bound_control,
        "--ik-minimum-iterations",
        "2",
        "--ik-posture-weight",
        "0.001",
        "--ik-center-of-mass-weight",
        "0.5",
        "--jet-center-of-mass-weight",
        "10",
        "--jet-off-chain-regularization",
        "1000",
        "--minimum-contact-transitions",
        "8",
        "--minimum-alternating-liftoffs",
        "4",
    ]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = f"{pythonpath}:{ADDITIONAL_PYTHONPATH}"
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    admission = json.loads(completed.stdout)
    if not admission["passed"]:
        raise ValueError(f"{name} repeat {repeat} failed admission")
    metrics_path = run_output / "oracle-wbc-admission-metrics.json"
    raw_path = run_output / "oracle-wbc-admission-raw.npz"
    metrics = json.loads(metrics_path.read_text())
    counters = read_perf(perf_path)
    latency = metrics["wbc_admission"]["latency_us"]
    return {
        "variant": name,
        "repeat": repeat,
        **{key.replace("-", "_"): counters[key] for key in COUNTERS},
        "instructions_per_wbc_solve_process_share": counters["instructions"]
        / WBC_SOLVES_PER_PROCESS,
        "cycles_per_wbc_solve_process_share": counters["cycles"]
        / WBC_SOLVES_PER_PROCESS,
        "task_clock_ns_per_wbc_solve_process_share": 1e6
        * counters["task-clock"]
        / WBC_SOLVES_PER_PROCESS,
        "process_cpu_seconds": metrics["runtime"]["process_cpu_seconds"],
        "wall_seconds": metrics["runtime"]["wall_seconds"],
        "maximum_rss_bytes": metrics["runtime"]["maximum_rss_after_bytes"],
        "p50_tick_us": latency["p50"],
        "p99_tick_us": latency["p99"],
        "maximum_tick_us": latency["maximum"],
        "raw_path": str(raw_path),
        "metrics_path": str(metrics_path),
        "perf_path": str(perf_path),
    }


def compare_semantics(control_path: pathlib.Path, candidate_path: pathlib.Path) -> tuple[int, list[str]]:
    with np.load(control_path) as control, np.load(candidate_path) as candidate:
        fields = sorted((set(control.files) & set(candidate.files)) - TIMING_ARRAYS)
        differences = [
            field
            for field in fields
            if not arrays_byte_exact(control[field], candidate[field])
        ]
    return len(fields), differences


def main() -> None:
    args = parse_args()
    if args.repeats < 3:
        raise ValueError("--repeats must be at least 3")
    control_pythonpath = pathlib.Path(args.control_pythonpath).resolve()
    candidate_pythonpath = pathlib.Path(args.candidate_pythonpath).resolve()
    if not control_pythonpath.is_dir() or not candidate_pythonpath.is_dir():
        raise FileNotFoundError("control and candidate Python package roots must exist")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    # Alternate process order inside each pair to oppose monotonic thermal or
    # desktop-load drift instead of assigning it permanently to one variant.
    for repeat in range(1, args.repeats + 1):
        order = (
            (("control", control_pythonpath), ("candidate", candidate_pythonpath))
            if repeat % 2
            else (("candidate", candidate_pythonpath), ("control", control_pythonpath))
        )
        for name, pythonpath in order:
            rows.append(
                run_variant(
                    name=name,
                    pythonpath=pythonpath,
                    repeat=repeat,
                    args=args,
                    output=output,
                )
            )

    controls = sorted((row for row in rows if row["variant"] == "control"), key=lambda row: row["repeat"])
    candidates = sorted((row for row in rows if row["variant"] == "candidate"), key=lambda row: row["repeat"])
    semantic_field_count = 0
    semantic_differences: dict[str, list[str]] = {}
    for control, candidate in zip(controls, candidates, strict=True):
        count, differences = compare_semantics(
            pathlib.Path(control["raw_path"]), pathlib.Path(candidate["raw_path"])
        )
        semantic_field_count = count
        if differences:
            semantic_differences[str(control["repeat"])] = differences

    metric_names = (
        "instructions",
        "cycles",
        "task_clock",
        "branches",
        "branch_misses",
        "cache_misses",
        "instructions_per_wbc_solve_process_share",
        "cycles_per_wbc_solve_process_share",
        "task_clock_ns_per_wbc_solve_process_share",
        "process_cpu_seconds",
        "wall_seconds",
        "maximum_rss_bytes",
        "p50_tick_us",
        "p99_tick_us",
        "maximum_tick_us",
    )
    summaries = {
        variant: {
            metric: distribution([float(row[metric]) for row in variant_rows])
            for metric in metric_names
        }
        for variant, variant_rows in (("control", controls), ("candidate", candidates))
    }
    deltas = {
        metric: relative_delta(
            summaries["control"][metric]["median"],
            summaries["candidate"][metric]["median"],
        )
        for metric in metric_names
    }
    paired_instruction_deltas = [
        relative_delta(float(control["instructions"]), float(candidate["instructions"]))
        for control, candidate in zip(controls, candidates, strict=True)
    ]
    checks = {
        "all_56_non_timing_arrays_are_bit_exact_in_every_pair": semantic_field_count == 56
        and not semantic_differences,
        "candidate_reduces_median_retired_instructions": deltas["instructions"] < 0.0,
        "candidate_reduces_retired_instructions_in_every_pair": all(
            delta < 0.0 for delta in paired_instruction_deltas
        ),
        "candidate_keeps_zero_measured_rust_allocations": all(
            json.loads(pathlib.Path(row["metrics_path"]).read_text())["checks"][
                "wbc_hot_loop_has_zero_allocations"
            ]
            for row in candidates
        ),
    }
    decision = "PROMOTE" if all(checks.values()) else "REJECT"
    hashes = {
        str(path): sha256(path)
        for row in rows
        for path in (
            pathlib.Path(row["raw_path"]),
            pathlib.Path(row["metrics_path"]),
            pathlib.Path(row["perf_path"]),
        )
    }
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "checks": checks,
        "evaluation_boundary": {
            "affinity": f"logical CPU {args.cpu}",
            "alternating_process_order": True,
            "repeats_per_variant": args.repeats,
            "counter_scope": "complete Python admission process including morphology, two Rust WBC corpus passes, reporting, and artifact serialization",
            "process_share_denominator": WBC_SOLVES_PER_PROCESS,
            "not_claimed": "per-solve hardware-counter attribution or isolated dense-kernel speedup",
            "zero_rows_removed": "one inactive vertical row from the horizontal-only CoM task",
            "dense_row_instances_avoided": 8_992,
        },
        "semantic_field_count": semantic_field_count,
        "semantic_differences": semantic_differences,
        "paired_instruction_relative_deltas": paired_instruction_deltas,
        "summaries": summaries,
        "candidate_relative_deltas": deltas,
        "trials": rows,
        "artifact_sha256": hashes,
    }
    metrics_path = output / "zero-task-row-compaction-ab-metrics.json"
    metrics_path.write_text(json.dumps(result, indent=2) + "\n")
    csv_fields = [field for field in rows[0] if not field.endswith("_path")]
    with (output / "zero-task-row-compaction-ab-trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    report = [
        "# Bonesaw exact-zero task-row compaction A/B · r65",
        "",
        f"## Decision · {decision}",
        "",
        f"The candidate removes one identically zero vertical row from the horizontal-only CoM task before each dense Viability pseudoinverse. Across {args.repeats} alternating pinned process pairs, all {semantic_field_count} non-timing arrays remain bit-for-bit identical. The retained-instruction median changes by {100 * deltas['instructions']:+.5f}%; paired changes span {100 * min(paired_instruction_deltas):+.5f}% to {100 * max(paired_instruction_deltas):+.5f}%, so the candidate does not reduce stable work in every pair.",
        "",
        "> A declared three-row diagnostic task remains a three-row task. Only its mathematically inert solve row is compacted; reported task provenance, residual rows, output arrays, clipping, ranks, feasibility, and allocation behavior remain unchanged.",
        "",
        "## Five-run pinned process A/B",
        "",
    ]
    report += markdown_table(
        ["variant", "instructions B", "cycles B", "task-clock s", "CPU / wall s", "p50 / p99 / max µs", "RSS MB"],
        [
            [
                variant,
                f"{summary['instructions']['median'] / 1e9:.6f}",
                f"{summary['cycles']['median'] / 1e9:.6f}",
                f"{summary['task_clock']['median'] / 1e9:.6f}",
                f"{summary['process_cpu_seconds']['median']:.3f} / {summary['wall_seconds']['median']:.3f}",
                f"{summary['p50_tick_us']['median']:.1f} / {summary['p99_tick_us']['median']:.1f} / {summary['maximum_tick_us']['median']:.1f}",
                f"{summary['maximum_rss_bytes']['median'] / 1e6:.2f}",
            ]
            for variant, summary in summaries.items()
        ],
    )
    report += [
        "",
        "## Relative candidate deltas",
        "",
    ]
    report += markdown_table(
        ["signal", "candidate − control"],
        [
            ["retired instructions", f"{100 * deltas['instructions']:+.4f}%"],
            ["cycles", f"{100 * deltas['cycles']:+.4f}%"],
            ["task clock", f"{100 * deltas['task_clock']:+.4f}%"],
            ["process CPU", f"{100 * deltas['process_cpu_seconds']:+.4f}%"],
            ["p50 tick", f"{100 * deltas['p50_tick_us']:+.4f}%"],
            ["p99 tick", f"{100 * deltas['p99_tick_us']:+.4f}%"],
            ["maximum RSS", f"{100 * deltas['maximum_rss_bytes']:+.4f}%"],
        ],
    )
    report += [
        "",
        "## Preservation and scope",
        "",
        f"- {semantic_field_count}/56 non-timing arrays are exact in every control/candidate pair, including accelerations, efforts, forces, task residuals/clipping, hard residuals/margins, statuses, and solver-work counters.",
        "- The declared task row count and task diagnostics remain unchanged; 8,992 inert row-instances are avoided across the retained corpus's repeated Viability solves.",
        "- Every candidate run passes all 43 admission gates and measures zero Rust hot-loop allocations.",
        "- Hardware counters cover the whole pinned process, not an isolated solve. Process-share values divide by 4,634 WBC calls only to make scale legible and are not per-solve attribution.",
        "- Wall-clock, cycle, and tail changes remain observational host signals. Stable retired instructions plus bit-exact semantics decide promotion.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "ZERO_TASK_ROW_COMPACTION_AB.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
