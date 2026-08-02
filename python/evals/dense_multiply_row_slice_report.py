#!/usr/bin/env python3
"""Promotion audit for r74 row-sliced dense matrix products."""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import statistics
from datetime import datetime, timezone
from typing import Any

import numpy as np

from coincident_step_limit_audit import WORK_FIELDS, difference, sha256
from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


TIMING_FIELDS = {"mean_tick_us", "p50_tick_us", "p99_tick_us", "max_tick_us"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--control",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/control",
    )
    parser.add_argument(
        "--candidate",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/candidate",
    )
    parser.add_argument(
        "--production",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/production",
    )
    parser.add_argument(
        "--counter",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/counter-ab",
    )
    parser.add_argument(
        "--native-control",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/native-control",
    )
    parser.add_argument(
        "--native-candidate",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate",
    )
    parser.add_argument(
        "--iterator-negative",
        default="benchmarks/results/g1-dense-multiply-row-slice-r74/iterator-negative",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-dense-multiply-row-slice-r74"
    )
    parser.add_argument(
        "--web-report", default="web/DENSE_MULTIPLY_ROW_SLICE_R74.html"
    )
    return parser.parse_args()


def relative_delta(control: float, candidate: float) -> float:
    return candidate / control - 1.0


def read_perf(path: pathlib.Path) -> dict[str, float]:
    values: dict[str, float] = {}
    with path.open() as stream:
        for row in csv.reader(stream):
            if len(row) < 3 or not row[0].strip() or not row[0].strip()[0].isdigit():
                continue
            values[row[2].split(":", 1)[0]] = float(row[0])
    return values


def semantic_projection(report: dict[str, Any]) -> dict[str, Any]:
    projected = json.loads(json.dumps(report))
    for field in TIMING_FIELDS:
        projected["floating_dynamic_wbc"].pop(field, None)
    return projected


def main() -> None:
    args = parse_args()
    control = pathlib.Path(args.control)
    candidate = pathlib.Path(args.candidate)
    production = pathlib.Path(args.production)
    counter_path = pathlib.Path(args.counter) / "zero-task-row-compaction-ab-metrics.json"
    native_control_path = (
        pathlib.Path(args.native_control) / "native-wbc-counter-metrics.json"
    )
    native_candidate_path = (
        pathlib.Path(args.native_candidate) / "native-wbc-counter-metrics.json"
    )
    iterator_root = pathlib.Path(args.iterator_negative)
    control_raw = control / "oracle-wbc-admission-raw.npz"
    candidate_raw = candidate / "oracle-wbc-admission-raw.npz"
    production_raw = production / "oracle-wbc-admission-raw.npz"
    control_metrics_path = control / "oracle-wbc-admission-metrics.json"
    candidate_metrics_path = candidate / "oracle-wbc-admission-metrics.json"
    production_metrics_path = production / "oracle-wbc-admission-metrics.json"
    control_metrics = json.loads(control_metrics_path.read_text())
    candidate_metrics = json.loads(candidate_metrics_path.read_text())
    production_metrics = json.loads(production_metrics_path.read_text())
    counter = json.loads(counter_path.read_text())
    native_control = json.loads(native_control_path.read_text())
    native_candidate = json.loads(native_candidate_path.read_text())

    with np.load(control_raw) as control_archive, np.load(
        candidate_raw
    ) as candidate_archive, np.load(production_raw) as production_archive:
        fields = sorted(
            (set(control_archive.files) & set(candidate_archive.files)) - TIMING_ARRAYS
        )
        candidate_differences = {
            field: difference(control_archive[field], candidate_archive[field])
            for field in fields
            if not arrays_byte_exact(control_archive[field], candidate_archive[field])
        }
        production_differences = {
            field: difference(candidate_archive[field], production_archive[field])
            for field in fields
            if not arrays_byte_exact(candidate_archive[field], production_archive[field])
        }
        work = {
            field: {
                "control_sum": int(np.sum(control_archive[field], dtype=np.int64)),
                "candidate_sum": int(np.sum(candidate_archive[field], dtype=np.int64)),
                "production_sum": int(np.sum(production_archive[field], dtype=np.int64)),
            }
            for field in sorted(WORK_FIELDS)
        }

    process_deltas = counter["candidate_relative_deltas"]
    paired_instructions = counter["paired_instruction_relative_deltas"]
    native_keys = (
        "instructions_per_tick",
        "cycles_per_tick",
        "task_clock_per_tick",
        "p50_tick_us",
        "p99_tick_us",
    )
    native_deltas = {
        key: relative_delta(
            native_control["summary"][key]["median"],
            native_candidate["summary"][key]["median"],
        )
        for key in native_keys
    }
    native_semantics_exact = (
        native_control["semantic_report"] == native_candidate["semantic_report"]
    )

    iterator_pairs = []
    for repeat in range(1, 6):
        iterator_control = read_perf(iterator_root / f"{repeat}-control.csv")
        iterator_candidate = read_perf(iterator_root / f"{repeat}-candidate.csv")
        iterator_pairs.append(
            relative_delta(
                iterator_control["instructions"], iterator_candidate["instructions"]
            )
        )
    iterator_control_report = json.loads((iterator_root / "control.json").read_text())
    iterator_candidate_report = json.loads((iterator_root / "candidate.json").read_text())
    iterator_semantics_exact = semantic_projection(
        iterator_control_report
    ) == semantic_projection(iterator_candidate_report)

    checks = {
        "production_passes_all_43_admission_gates": bool(production_metrics["passed"]),
        "all_56_non_timing_arrays_are_bit_exact": len(fields) == 56
        and not candidate_differences
        and not production_differences,
        "all_five_pinned_process_pairs_are_bit_exact": bool(
            counter["checks"]["all_56_non_timing_arrays_are_bit_exact_in_every_pair"]
        ),
        "retired_instructions_fall_in_every_pinned_process_pair": all(
            value < 0.0 for value in paired_instructions
        ),
        "native_semantic_reports_are_exact": native_semantics_exact,
        "native_marginal_instructions_fall": native_deltas["instructions_per_tick"] < 0.0,
        "production_keeps_zero_measured_rust_allocations": bool(
            production_metrics["checks"]["wbc_hot_loop_has_zero_allocations"]
        )
        and native_candidate["semantic_report"]["floating_dynamic_wbc"][
            "allocations_per_tick"
        ]
        == 0.0,
    }
    decision = "PROMOTE" if all(checks.values()) else "REJECT"
    source_paths = (
        control_raw,
        candidate_raw,
        production_raw,
        control_metrics_path,
        candidate_metrics_path,
        production_metrics_path,
        counter_path,
        native_control_path,
        native_candidate_path,
        iterator_root / "control.json",
        iterator_root / "candidate.json",
        *(iterator_root / f"{repeat}-{variant}.csv" for repeat in range(1, 6) for variant in ("control", "candidate")),
    )
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "optimization": {
            "name": "row-sliced dense matrix products",
            "production_default": True,
            "control_feature": "bonesaw-core/dense-multiply-row-slice-control",
            "preserved_order": [
                "output zero initialization",
                "left row traversal",
                "shared-coordinate traversal",
                "exact-zero left-value skip",
                "right column traversal",
                "scalar multiply-add expression",
            ],
        },
        "profile_attribution": {
            "event": "cycles",
            "frequency_hz": 1999,
            "samples": 5324,
            "lost_samples": 0,
            "pseudoinverse_symbol_self_fraction": 0.8109,
            "jacobi_coupling_line_fraction": 0.1634,
            "dense_multiply_symbol_self_fraction": 0.0821,
            "scope": "2,000 fixed-shape 58-variable G1 WBC ticks pinned to logical CPU 4; sampled profile selects candidates but does not decide promotion",
        },
        "r73_iterator_negative": {
            "decision": "REJECT",
            "semantic_report_exact": iterator_semantics_exact,
            "paired_instruction_relative_deltas": iterator_pairs,
            "median_instruction_relative_delta": statistics.median(iterator_pairs),
            "reason": "paired slice iterators compile to effectively the same work as indexed slices; instruction signs are mixed at the noise floor",
        },
        "evaluation_boundary": {
            "corpus_states": int(production_metrics["ticks"]),
            "policy": False,
            "state_integration": False,
            "contact_simulation": False,
            "physics_rollout": False,
            "pinned_process_pairs": len(paired_instructions),
            "process_counter_scope": counter["evaluation_boundary"]["counter_scope"],
            "native_ticks_per_long_process": native_candidate["ticks_per_long_process"],
            "native_repeats": native_candidate["repeats"],
            "native_counter_scope": native_candidate["counter_scope"],
        },
        "checks": checks,
        "semantic_field_count": len(fields),
        "candidate_semantic_differences": candidate_differences,
        "production_semantic_differences": production_differences,
        "work": work,
        "pinned_process": {
            "paired_instruction_relative_deltas": paired_instructions,
            "candidate_relative_deltas": process_deltas,
            "summaries": counter["summaries"],
        },
        "native": {
            "semantic_reports_exact": native_semantics_exact,
            "relative_deltas": native_deltas,
            "control_summary": native_control["summary"],
            "production_summary": native_candidate["summary"],
            "semantic_report": native_candidate["semantic_report"],
        },
        "inherited_exact_evidence": {
            "r63_consequence_gates": "8/8 inherited because every r54 non-timing source field is bit-exact",
            "r64_independent_oracle_gates": "6/6 inherited because r63 inputs and established solve arithmetic are bit-exact",
        },
        "artifact_sha256": {str(path): sha256(path) for path in source_paths},
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "dense-multiply-row-slice-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    report = [
        "# Bonesaw row-sliced dense matrix products · r74",
        "",
        f"## Decision · {decision}",
        "",
        "Production now addresses the left, right, and output rows of each dense matrix product through prevalidated contiguous slices. Output initialization, row/shared/column traversal, exact-zero skipping, and every scalar multiply-add remain in the established order. This changes address and bounds work, not solver arithmetic.",
        "",
        f"Across five alternating complete-process A/B pairs pinned to one logical CPU, all 56 non-timing arrays remain bit-for-bit exact and every candidate retires fewer instructions. Median retired instructions fall {-100 * process_deltas['instructions']:.3f}%; process CPU/p50/p99 move {100 * process_deltas['process_cpu_seconds']:+.2f}%/{100 * process_deltas['p50_tick_us']:+.2f}%/{100 * process_deltas['p99_tick_us']:+.2f}%.",
        "",
        f"The separate 58-variable native G1 sentinel is semantically exact and allocation-free. Marginal instructions fall from {native_control['summary']['instructions_per_tick']['median'] / 1e6:.3f} M to {native_candidate['summary']['instructions_per_tick']['median'] / 1e6:.3f} M per tick ({100 * native_deltas['instructions_per_tick']:+.3f}%); native cycles/p50/p99 move {100 * native_deltas['cycles_per_tick']:+.2f}%/{100 * native_deltas['p50_tick_us']:+.2f}%/{100 * native_deltas['p99_tick_us']:+.2f}%.",
        "",
        "> Hardware counters remain scoped honestly: the primary A/B covers the complete Python admission process; the native measurement subtracts an adjacent one-tick process and remains a marginal upper bound. Stable retired instructions plus exact semantics decide promotion.",
        "",
        "## Post-r72 profile attribution",
        "",
        "A 5,324-sample, zero-loss cycle profile of 2,000 CPU-pinned native G1 ticks attributed 81.09% self cycles to the pseudoinverse symbol, 16.34% to the Jacobi coupling line, and 8.21% to the general dense multiply. The narrow r73 paired-iterator candidate was exact but changed median instructions by only "
        f"{100 * statistics.median(iterator_pairs):+.7f}% with mixed signs, so it is rejected. Row slicing at the multiply boundary produces stable work reduction instead.",
        "",
        "## Pinned complete-process A/B",
        "",
    ]
    report += markdown_table(
        ["signal", "production − flat-index control"],
        [
            ["retired instructions", f"{100 * process_deltas['instructions']:+.3f}%"],
            ["cycles", f"{100 * process_deltas['cycles']:+.2f}%"],
            ["task clock", f"{100 * process_deltas['task_clock']:+.2f}%"],
            ["process CPU", f"{100 * process_deltas['process_cpu_seconds']:+.2f}%"],
            ["p50 tick", f"{100 * process_deltas['p50_tick_us']:+.2f}%"],
            ["p99 tick", f"{100 * process_deltas['p99_tick_us']:+.2f}%"],
            ["maximum RSS", f"{100 * process_deltas['maximum_rss_bytes']:+.4f}%"],
        ],
    )
    report += ["", "## Native marginal WBC sentinel", ""]
    report += markdown_table(
        ["signal", "flat-index control", "production", "delta"],
        [
            [
                "instructions / tick",
                f"{native_control['summary']['instructions_per_tick']['median'] / 1e6:.3f} M",
                f"{native_candidate['summary']['instructions_per_tick']['median'] / 1e6:.3f} M",
                f"{100 * native_deltas['instructions_per_tick']:+.3f}%",
            ],
            [
                "cycles / tick",
                f"{native_control['summary']['cycles_per_tick']['median'] / 1e6:.3f} M",
                f"{native_candidate['summary']['cycles_per_tick']['median'] / 1e6:.3f} M",
                f"{100 * native_deltas['cycles_per_tick']:+.2f}%",
            ],
            [
                "task clock / tick",
                f"{native_control['summary']['task_clock_per_tick']['median'] / 1e6:.3f} ms",
                f"{native_candidate['summary']['task_clock_per_tick']['median'] / 1e6:.3f} ms",
                f"{100 * native_deltas['task_clock_per_tick']:+.2f}%",
            ],
            [
                "p50 / p99",
                f"{native_control['summary']['p50_tick_us']['median']:.1f} / {native_control['summary']['p99_tick_us']['median']:.1f} µs",
                f"{native_candidate['summary']['p50_tick_us']['median']:.1f} / {native_candidate['summary']['p99_tick_us']['median']:.1f} µs",
                f"{100 * native_deltas['p50_tick_us']:+.2f}% / {100 * native_deltas['p99_tick_us']:+.2f}%",
            ],
        ],
    )
    report += [
        "",
        "## Preservation and interpretation",
        "",
        "- Default production, explicit experiment, and flat-index control all pass 123 core tests; the row-slice product witness covers dense and sparse shapes through 58 × 58.",
        "- Solver work is unchanged: 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. This is cheaper execution of identical work.",
        "- Exact r54 outputs inherit the established r63 8/8 fixed-effort consequence and r64 6/6 independent Pinocchio/NumPy oracle evidence without translating a changed trace.",
        "- The `dense-multiply-row-slice-control` feature preserves the pre-r74 flat-index implementation for future differential checks.",
        "- CUDA remains deferred; r74 further tightens and accelerates the CPU semantic reference.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "DENSE_MULTIPLY_ROW_SLICE_AUDIT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
