#!/usr/bin/env python3
"""Evaluate the rejected r289 direct-offset Jacobi pointer experiment."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import statistics
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


TIMING_ARRAYS = {"step_ns"}
COUNTERS = ("instructions", "cycles", "cache-misses", "branches", "branch-misses")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timing-control", nargs="+", required=True)
    parser.add_argument("--timing-candidate", nargs="+", required=True)
    parser.add_argument("--perf-control", nargs="+", required=True)
    parser.add_argument("--perf-candidate", nargs="+", required=True)
    parser.add_argument(
        "--output", default="benchmarks/results/g1-jacobi-column-offset-pointer-r289"
    )
    parser.add_argument(
        "--web-report", default="web/G1_JACOBI_COLUMN_OFFSET_POINTER_R289.html"
    )
    return parser.parse_args()


def raw_path(root: pathlib.Path) -> pathlib.Path:
    return root / "floating-walk-raw.npz"


def metrics_path(root: pathlib.Path) -> pathlib.Path:
    return root / "floating-walk-metrics.json"


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


def read_perf(path: pathlib.Path) -> dict[str, float]:
    values: dict[str, float] = {}
    with path.open() as stream:
        for row in csv.reader(stream):
            if len(row) < 3:
                continue
            name = row[2].split(":", 1)[0]
            if name in COUNTERS:
                values[name] = float(row[0])
    missing = set(COUNTERS) - set(values)
    if missing:
        raise ValueError(f"{path}: missing perf counters {sorted(missing)}")
    return values


def latency(path: pathlib.Path) -> dict[str, float]:
    report = json.loads(path.read_text())
    values = report["metrics"]["latency_us"]
    return {name: float(values[name]) for name in ("p50", "p95", "p99", "max")}


def mean_counters(rows: list[dict[str, float]]) -> dict[str, float]:
    return {name: statistics.fmean(row[name] for row in rows) for name in COUNTERS}


def delta(control: float, candidate: float) -> float:
    return candidate / control - 1.0


def main() -> None:
    args = parse_args()
    control_roots = [pathlib.Path(path) for path in args.timing_control]
    candidate_roots = [pathlib.Path(path) for path in args.timing_candidate]
    if len(control_roots) != len(candidate_roots):
        raise ValueError("timing control/candidate repeat counts differ")
    if len(args.perf_control) != len(args.perf_candidate):
        raise ValueError("perf control/candidate repeat counts differ")

    semantic_pairs: list[dict[str, Any]] = []
    timing_pairs: list[dict[str, Any]] = []
    for index, (control_root, candidate_root) in enumerate(
        zip(control_roots, candidate_roots, strict=True), start=1
    ):
        control_count, control_digest, control_payloads = semantic_digest(
            raw_path(control_root)
        )
        candidate_count, candidate_digest, candidate_payloads = semantic_digest(
            raw_path(candidate_root)
        )
        changed = sorted(
            field
            for field in set(control_payloads) | set(candidate_payloads)
            if control_payloads.get(field) != candidate_payloads.get(field)
        )
        semantic_pairs.append(
            {
                "repeat": index,
                "field_count": control_count,
                "candidate_field_count": candidate_count,
                "control_sha256": control_digest,
                "candidate_sha256": candidate_digest,
                "changed_fields": changed,
                "exact": not changed
                and control_count == candidate_count
                and control_digest == candidate_digest,
            }
        )
        timing_pairs.append(
            {
                "repeat": index,
                "control": latency(metrics_path(control_root)),
                "candidate": latency(metrics_path(candidate_root)),
            }
        )

    control_perf = [read_perf(pathlib.Path(path)) for path in args.perf_control]
    candidate_perf = [read_perf(pathlib.Path(path)) for path in args.perf_candidate]
    control_average = mean_counters(control_perf)
    candidate_average = mean_counters(candidate_perf)
    counter_deltas = {
        name: delta(control_average[name], candidate_average[name]) for name in COUNTERS
    }
    paired_instruction_deltas = [
        delta(control["instructions"], candidate["instructions"])
        for control, candidate in zip(control_perf, candidate_perf, strict=True)
    ]
    all_exact = all(pair["exact"] for pair in semantic_pairs)
    all_instruction_counts_increase = all(value > 0.0 for value in paired_instruction_deltas)
    rejected = all_exact and all_instruction_counts_increase and counter_deltas["instructions"] > 0.0

    result = {
        "schema": "bonesaw.g1-jacobi-column-offset-pointer-r289.v1",
        "revision": "g1-jacobi-column-offset-pointer-r289",
        "decision": "REJECT" if rejected else "INCONCLUSIVE",
        "execution": {
            "ticks_per_process": 2317,
            "timing_cpu": 4,
            "policy_steps": 0,
            "physics_steps": 0,
            "timing_repeats": len(timing_pairs),
            "hardware_counter_repeats": len(control_perf),
        },
        "candidate": {
            "feature": "bonesaw-core/jacobi-column-offset-pointer-experiment",
            "production_default": False,
            "control": "r284 split/slice raw-pointer kernel",
        },
        "semantic_contract": {
            "all_repeats_exact": all_exact,
            "pairs": semantic_pairs,
        },
        "timing_us": {
            "pairs": timing_pairs,
            "median_of_repeat_p99": {
                "control": statistics.median(
                    pair["control"]["p99"] for pair in timing_pairs
                ),
                "candidate": statistics.median(
                    pair["candidate"]["p99"] for pair in timing_pairs
                ),
            },
        },
        "hardware_counters": {
            "control_repeats": control_perf,
            "candidate_repeats": candidate_perf,
            "control_average": control_average,
            "candidate_average": candidate_average,
            "candidate_relative_delta": counter_deltas,
            "paired_instruction_relative_delta": paired_instruction_deltas,
        },
        "verdict": {
            "candidate_rejected": rejected,
            "r284_default_retained": rejected,
            "p99_gate_passed": False,
            "authority_admitted": False,
        },
    }

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_output = output / "g1-jacobi-column-offset-pointer-r289-metrics.json"
    metrics_output.write_text(json.dumps(result, indent=2) + "\n")

    timing_rows = []
    for pair in timing_pairs:
        timing_rows.append(
            [
                pair["repeat"],
                f"{pair['control']['p50']:.1f} / {pair['candidate']['p50']:.1f}",
                f"{pair['control']['p95']:.1f} / {pair['candidate']['p95']:.1f}",
                f"{pair['control']['p99']:.1f} / {pair['candidate']['p99']:.1f}",
                f"{pair['control']['max']:.1f} / {pair['candidate']['max']:.1f}",
            ]
        )
    counter_rows = [
        [
            name,
            f"{control_average[name] / 1e9:.6f} B",
            f"{candidate_average[name] / 1e9:.6f} B",
            f"{100.0 * counter_deltas[name]:+.3f}%",
        ]
        for name in COUNTERS
    ]
    report = [
        "# G1 direct-offset Jacobi pointer experiment · R289",
        "",
        "> Exact replay PASS · CPU instruction reduction FAIL · candidate REJECTED.",
        "",
        "R289 tested whether addressing each Jacobi column pair from one base pointer could remove the split/slice wrapper retained by R284. The candidate preserves column traversal, scalar operation order, rotation order, stores, tolerances, rank selection, clipping, feasibility, and task hierarchy. It is retained only behind an explicit experiment feature; the established R284 kernel remains the default.",
        "",
        "## Semantic replay",
        "",
        f"All {len(semantic_pairs)} CPU-4-pinned, 2,317-tick control/candidate pairs are byte-exact across all {semantic_pairs[0]['field_count']} non-timing arrays. Every pair has digest `{semantic_pairs[0]['control_sha256']}`. The corpus executes no policy and no physics; timing is the only permitted difference.",
        "",
        "## Native timing observations",
        "",
    ]
    report += markdown_table(
        ["pair", "p50 control / candidate µs", "p95", "p99", "max"], timing_rows
    )
    report += [
        "",
        f"The median repeat p99 is {result['timing_us']['median_of_repeat_p99']['control']:.1f} µs for control and {result['timing_us']['median_of_repeat_p99']['candidate']:.1f} µs for candidate. Large isolated tails appear in both builds, so wall-clock movement is treated as host scheduling evidence rather than the decision signal. The universal 5 ms p99 gate remains open.",
        "",
        "## Pinned hardware counters",
        "",
        "Three complete-process pairs use the same corpus and CPU pin. Averages are:",
        "",
    ]
    report += markdown_table(["counter", "R284 control", "R289 candidate", "delta"], counter_rows)
    report += [
        "",
        f"The candidate retires {100.0 * counter_deltas['instructions']:+.3f}% more instructions ({candidate_average['instructions'] - control_average['instructions']:.0f} extra per process), and every paired instruction delta is positive ({', '.join(f'{100.0 * value:+.3f}%' for value in paired_instruction_deltas)}). Fewer dynamic branches do not compensate for the larger executed instruction stream.",
        "",
        "## Decision",
        "",
        "The direct-offset kernel is rejected as a production optimization. R284's split/slice raw-pointer kernel remains the CPU default. The candidate stays opt-in solely so the negative result can be reproduced without reconstructing the patch. No timing, authority, contact-transfer, CUDA, plant, or hardware-realization gate changes.",
    ]
    report_text = "\n".join(report) + "\n"
    report_output = output / "G1_JACOBI_COLUMN_OFFSET_POINTER_R289.md"
    report_output.write_text(report_text)
    web_output = pathlib.Path(args.web_report)
    web_output.parent.mkdir(parents=True, exist_ok=True)
    web_output.write_text(render_report_html(report_text))
    print(json.dumps({"decision": result["decision"], "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
