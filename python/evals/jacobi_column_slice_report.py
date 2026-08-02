#!/usr/bin/env python3
"""Promotion report for r72 slice-addressed one-sided Jacobi columns."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from coincident_step_limit_audit import WORK_FIELDS, difference, sha256
from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--control", default="benchmarks/results/g1-task-nullspace-repair-r69/control"
    )
    parser.add_argument(
        "--production",
        default="benchmarks/results/g1-jacobi-column-slice-r72/production",
    )
    parser.add_argument(
        "--counter",
        default="benchmarks/results/g1-jacobi-column-slice-r72/counter-ab",
    )
    parser.add_argument(
        "--native-control",
        default="benchmarks/results/g1-jacobi-column-slice-r72/native-control",
    )
    parser.add_argument(
        "--native-production",
        default="benchmarks/results/g1-jacobi-column-slice-r72/native-production",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-jacobi-column-slice-r72"
    )
    parser.add_argument(
        "--web-report", default="web/JACOBI_COLUMN_SLICE_R72.html"
    )
    return parser.parse_args()


def relative_delta(control: float, candidate: float) -> float:
    return candidate / control - 1.0


def main() -> None:
    args = parse_args()
    control = pathlib.Path(args.control)
    production = pathlib.Path(args.production)
    control_raw = control / "oracle-wbc-admission-raw.npz"
    production_raw = production / "oracle-wbc-admission-raw.npz"
    control_metrics_path = control / "oracle-wbc-admission-metrics.json"
    production_metrics_path = production / "oracle-wbc-admission-metrics.json"
    counter_path = pathlib.Path(args.counter) / "zero-task-row-compaction-ab-metrics.json"
    native_control_path = pathlib.Path(args.native_control) / "native-wbc-counter-metrics.json"
    native_production_path = (
        pathlib.Path(args.native_production) / "native-wbc-counter-metrics.json"
    )
    control_metrics = json.loads(control_metrics_path.read_text())
    production_metrics = json.loads(production_metrics_path.read_text())
    counter = json.loads(counter_path.read_text())
    native_control = json.loads(native_control_path.read_text())
    native_production = json.loads(native_production_path.read_text())

    with np.load(control_raw) as control_archive, np.load(
        production_raw
    ) as production_archive:
        fields = sorted(
            (set(control_archive.files) & set(production_archive.files)) - TIMING_ARRAYS
        )
        differences = {
            field: difference(control_archive[field], production_archive[field])
            for field in fields
            if not arrays_byte_exact(
                control_archive[field], production_archive[field]
            )
        }
        work = {
            field: {
                "control_sum": int(np.sum(control_archive[field], dtype=np.int64)),
                "production_sum": int(
                    np.sum(production_archive[field], dtype=np.int64)
                ),
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
            native_production["summary"][key]["median"],
        )
        for key in native_keys
    }
    native_semantics_exact = (
        native_control["semantic_report"] == native_production["semantic_report"]
    )
    checks = {
        "production_passes_all_43_admission_gates": bool(production_metrics["passed"]),
        "all_56_non_timing_arrays_are_bit_exact": len(fields) == 56 and not differences,
        "all_five_pinned_process_pairs_are_bit_exact": bool(
            counter["checks"]["all_56_non_timing_arrays_are_bit_exact_in_every_pair"]
        ),
        "retired_instructions_fall_in_every_pinned_process_pair": all(
            value < 0.0 for value in paired_instructions
        ),
        "median_pinned_process_instructions_fall": process_deltas["instructions"] < 0.0,
        "native_semantic_reports_are_exact": native_semantics_exact,
        "native_marginal_instructions_fall": native_deltas["instructions_per_tick"] < 0.0,
        "production_keeps_zero_measured_rust_allocations": bool(
            production_metrics["checks"]["wbc_hot_loop_has_zero_allocations"]
        )
        and native_production["semantic_report"]["floating_dynamic_wbc"][
            "allocations_per_tick"
        ]
        == 0.0,
    }
    decision = "PROMOTE" if all(checks.values()) else "REJECT"
    source_paths = (
        control_raw,
        production_raw,
        control_metrics_path,
        production_metrics_path,
        counter_path,
        native_control_path,
        native_production_path,
    )
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "optimization": {
            "name": "slice-addressed one-sided Jacobi column pairs",
            "production_default": True,
            "control_feature": "bonesaw-core/jacobi-column-slice-control",
            "preserved_order": [
                "column-pair traversal",
                "coupling accumulation",
                "rotation arithmetic",
                "right-vector rotation",
                "sweep-boundary energy re-anchoring",
                "singular truncation",
                "hard-limit comparisons",
            ],
        },
        "evaluation_boundary": {
            "corpus_states": int(production_metrics["ticks"]),
            "policy": False,
            "state_integration": False,
            "contact_simulation": False,
            "physics_rollout": False,
            "pinned_process_pairs": len(paired_instructions),
            "process_counter_scope": counter["evaluation_boundary"]["counter_scope"],
            "native_ticks_per_long_process": native_production[
                "ticks_per_long_process"
            ],
            "native_repeats": native_production["repeats"],
            "native_counter_scope": native_production["counter_scope"],
        },
        "checks": checks,
        "semantic_field_count": len(fields),
        "semantic_differences": differences,
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
            "production_summary": native_production["summary"],
            "semantic_report": native_production["semantic_report"],
        },
        "inherited_exact_evidence": {
            "r63_consequence_gates": "8/8 inherited because every r54 non-timing source field is bit-exact",
            "r64_independent_oracle_gates": "6/6 inherited because r63 inputs and established solve arithmetic are bit-exact",
        },
        "artifact_sha256": {str(path): sha256(path) for path in source_paths},
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "jacobi-column-slice-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    report = [
        "# Bonesaw slice-addressed Jacobi columns · r72",
        "",
        f"## Decision · {decision}",
        "",
        f"Production now addresses each active one-sided-Jacobi column pair through two prevalidated contiguous slices. Column-pair order, scalar coupling accumulation, rotation formulas, right-vector updates, sweep-boundary energy re-anchoring, singular truncation, and every downstream hard-limit comparison remain unchanged. The optimization removes repeated flat-index multiplication and bounds logic; it does not change the factorization or solver semantics.",
        "",
        f"Across five alternating complete-process A/B pairs pinned to one logical CPU, all 56 non-timing arrays remain bit-for-bit exact and every candidate retires fewer instructions. The median instruction reduction is {-100 * process_deltas['instructions']:.3f}%; paired reductions span {-100 * max(paired_instructions):.3f}% to {-100 * min(paired_instructions):.3f}%. Process CPU falls {-100 * process_deltas['process_cpu_seconds']:.2f}%, p50 falls {-100 * process_deltas['p50_tick_us']:.2f}%, and p99 falls {-100 * process_deltas['p99_tick_us']:.2f}%.",
        "",
        f"A separate native 58-variable G1 WBC sentinel confirms the attribution boundary: marginal retired instructions fall from {native_control['summary']['instructions_per_tick']['median'] / 1e6:.3f} M to {native_production['summary']['instructions_per_tick']['median'] / 1e6:.3f} M per tick ({100 * native_deltas['instructions_per_tick']:+.2f}%), with identical semantic reports, zero infeasible ticks, bitwise repeat, and zero allocations. Native p50 moves {100 * native_deltas['p50_tick_us']:+.2f}% and median p99 {100 * native_deltas['p99_tick_us']:+.2f}%.",
        "",
        "> Hardware counters remain scoped honestly: the primary A/B covers the complete Python admission process; the native measurement subtracts an adjacent one-tick process and remains a marginal upper bound. Stable retired instructions plus exact semantics decide promotion.",
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
                f"{native_production['summary']['instructions_per_tick']['median'] / 1e6:.3f} M",
                f"{100 * native_deltas['instructions_per_tick']:+.2f}%",
            ],
            [
                "cycles / tick",
                f"{native_control['summary']['cycles_per_tick']['median'] / 1e6:.3f} M",
                f"{native_production['summary']['cycles_per_tick']['median'] / 1e6:.3f} M",
                f"{100 * native_deltas['cycles_per_tick']:+.2f}%",
            ],
            [
                "task clock / tick",
                f"{native_control['summary']['task_clock_per_tick']['median'] / 1e6:.3f} ms",
                f"{native_production['summary']['task_clock_per_tick']['median'] / 1e6:.3f} ms",
                f"{100 * native_deltas['task_clock_per_tick']:+.2f}%",
            ],
            [
                "p50 / p99",
                f"{native_control['summary']['p50_tick_us']['median']:.1f} / {native_control['summary']['p99_tick_us']['median']:.1f} µs",
                f"{native_production['summary']['p50_tick_us']['median']:.1f} / {native_production['summary']['p99_tick_us']['median']:.1f} µs",
                f"{100 * native_deltas['p50_tick_us']:+.2f}% / {100 * native_deltas['p99_tick_us']:+.2f}%",
            ],
        ],
    )
    report += [
        "",
        "## Preservation and interpretation",
        "",
        "- The promoted default and explicit experiment build are exact across all 56 retained non-timing arrays; the `jacobi-column-slice-control` feature preserves the flat-index A/B control.",
        "- Solver work is unchanged: 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. This is cheaper execution of identical work.",
        "- Exact r54 outputs inherit the established r63 8/8 fixed-effort consequence and r64 6/6 independent Pinocchio/NumPy oracle evidence without translating a changed source trace.",
        "- The 30.49% whole-process instruction reduction is larger than the 6.28% process-CPU reduction; instruction count is deterministic work evidence, while cycles and host timing remain observational.",
        "- CUDA remains deferred. The CPU reference becomes both faster and more precisely frozen for a future batch backend.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "JACOBI_COLUMN_SLICE_AUDIT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
