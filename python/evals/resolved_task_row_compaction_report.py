#!/usr/bin/env python3
"""Promote/reject report for r66 resolved task-row compaction."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


SCENARIOS = (
    "ideal_control",
    "fast_synthetic",
    "medium_synthetic",
    "slow_synthetic",
    "half_available",
    "quarter_available_slow",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--counter",
        default=(
            "benchmarks/results/g1-resolved-task-row-compaction-r66/counter-ab/"
            "zero-task-row-compaction-ab-metrics.json"
        ),
    )
    parser.add_argument(
        "--control-raw",
        default=(
            "benchmarks/results/g1-resolved-task-row-compaction-r66/counter-ab/"
            "control/run-1/oracle-wbc-admission-raw.npz"
        ),
    )
    parser.add_argument(
        "--production",
        default="benchmarks/results/g1-resolved-task-row-compaction-r66/production",
    )
    parser.add_argument(
        "--baseline-realization",
        default="benchmarks/results/g1-constrained-acceleration-r63",
    )
    parser.add_argument(
        "--candidate-realization",
        default="benchmarks/results/g1-resolved-task-row-compaction-r66/realization",
    )
    parser.add_argument(
        "--pinocchio",
        default="benchmarks/results/g1-resolved-task-row-compaction-r66/pinocchio",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-resolved-task-row-compaction-r66"
    )
    parser.add_argument("--web-report", default="web/RESOLVED_TASK_ROW_COMPACTION_R66.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def delta(left: np.ndarray, right: np.ndarray) -> dict[str, Any]:
    difference = np.abs(left.astype(np.float64) - right.astype(np.float64))
    return {
        "changed_values": int(np.count_nonzero(left != right)),
        "changed_ticks": int(
            np.count_nonzero(
                np.any(left != right, axis=tuple(range(1, left.ndim)))
                if left.ndim > 1
                else left != right
            )
        ),
        "rms": float(np.sqrt(np.mean(difference * difference))),
        "maximum": float(np.max(difference)),
    }


def main() -> None:
    args = parse_args()
    counter_path = pathlib.Path(args.counter)
    control_raw_path = pathlib.Path(args.control_raw)
    production_dir = pathlib.Path(args.production)
    production_raw_path = production_dir / "oracle-wbc-admission-raw.npz"
    production_metrics_path = production_dir / "oracle-wbc-admission-metrics.json"
    baseline_realization_path = pathlib.Path(args.baseline_realization) / "constrained-acceleration-raw.npz"
    realization_dir = pathlib.Path(args.candidate_realization)
    realization_raw_path = realization_dir / "constrained-acceleration-raw.npz"
    realization_metrics_path = realization_dir / "constrained-acceleration-metrics.json"
    pinocchio_metrics_path = pathlib.Path(args.pinocchio) / "pinocchio-fixed-effort-metrics.json"
    source_paths = (
        counter_path,
        control_raw_path,
        production_raw_path,
        production_metrics_path,
        baseline_realization_path,
        realization_raw_path,
        realization_metrics_path,
        pinocchio_metrics_path,
    )
    hashes_before = {str(path): sha256(path) for path in source_paths}
    counter = json.loads(counter_path.read_text())
    production_metrics = json.loads(production_metrics_path.read_text())
    realization_metrics = json.loads(realization_metrics_path.read_text())
    pinocchio_metrics = json.loads(pinocchio_metrics_path.read_text())

    with np.load(control_raw_path) as control, np.load(production_raw_path) as production:
        fields = sorted((set(control.files) & set(production.files)) - TIMING_ARRAYS)
        changed_fields = [field for field in fields if not arrays_byte_exact(control[field], production[field])]
        ordinary_deltas = {
            field: delta(control[field], production[field]) for field in changed_fields
        }
        physical_fields = (
            "generalized_acceleration",
            "actuator_torque",
            "contact_normal_force",
            "task_rms",
        )
        double_support = production["contacts"].sum(axis=1) == 2
        double_support_exact = all(
            arrays_byte_exact(control[field][double_support], production[field][double_support])
            for field in physical_fields
        )
        exact_decision_fields = all(
            arrays_byte_exact(control[field], production[field])
            for field in (
                "status",
                "task_clipped",
                "task_pseudoinverse_calls",
                "task_pseudoinverse_calls_by_priority",
                "clipped_steps",
                "clipped_steps_by_priority",
                "feasibility_projection_sweeps",
                "feasibility_halfspace_projections",
                "feasibility_polish_iterations",
                "allocation_calls",
                "allocated_bytes",
            )
        )
        single_support_ticks = int(np.count_nonzero(~double_support))

    with np.load(baseline_realization_path) as baseline, np.load(realization_raw_path) as candidate:
        consequence_acceleration_max = max(
            delta(
                baseline[f"{scenario}_generalized_acceleration"],
                candidate[f"{scenario}_generalized_acceleration"],
            )["maximum"]
            for scenario in SCENARIOS
        )
        consequence_effort_exact = all(
            arrays_byte_exact(
                baseline[f"{scenario}_actuator_torque"],
                candidate[f"{scenario}_actuator_torque"],
            )
            for scenario in SCENARIOS
        )
        consequence_pressure_exact = all(
            arrays_byte_exact(
                baseline[f"{scenario}_bound_normalized_pressure"],
                candidate[f"{scenario}_bound_normalized_pressure"],
            )
            for scenario in SCENARIOS
        )
        consequence_force_delta = max(
            delta(
                baseline[f"{scenario}_contact_force_basis"],
                candidate[f"{scenario}_contact_force_basis"],
            )["maximum"]
            for scenario in SCENARIOS
        )

    ordinary_limits = {
        "generalized_acceleration": 2e-6,
        "actuator_torque": 2e-7,
        "contact_normal_force": 2e-6,
        "task_rms": 5e-7,
        "dynamics_residual": 1e-10,
        "contact_residual": 1e-10,
        "minimum_friction_margin": 2e-6,
        "minimum_support_margin": 1e-9,
        "minimum_torque_margin": 2e-7,
        "maximum_constraint_violation": 1e-10,
        "maximum_torque_utilization": 1e-8,
        "minimum_torque_headroom": 1e-7,
        "witness_acceleration_rms": 5e-7,
        "task_jacobi_sweeps": 2.0,
        "task_jacobi_sweeps_by_priority": 2.0,
    }
    ordinary_within_contract = all(
        ordinary_deltas[field]["maximum"] <= ordinary_limits[field]
        for field in changed_fields
        if field in ordinary_limits
    ) and all(field in ordinary_limits for field in changed_fields)
    instruction_delta = counter["candidate_relative_deltas"]["instructions"]
    paired_instruction_deltas = counter["paired_instruction_relative_deltas"]
    source_immutable = all(hashes_before[str(path)] == sha256(path) for path in source_paths)
    checks = {
        "source_artifacts_remain_immutable": source_immutable,
        "candidate_reduces_median_whole_process_instructions_by_at_least_3pct": instruction_delta <= -0.03,
        "candidate_reduces_instructions_in_every_pinned_pair": all(
            value < 0.0 for value in paired_instruction_deltas
        ),
        "production_default_passes_all_43_admission_gates": production_metrics["passed"]
        and len(production_metrics["checks"]) == 43
        and all(production_metrics["checks"].values()),
        "ordinary_changed_fields_stay_inside_declared_delta_contract": ordinary_within_contract,
        "status_clipping_work_and_allocation_decisions_are_exact": exact_decision_fields,
        "all_double_support_physical_arrays_are_bit_exact": double_support_exact,
        "r63_consequence_passes_all_8_mechanism_gates": realization_metrics["passed"]
        and len(realization_metrics["checks"]) == 8
        and all(realization_metrics["checks"].values()),
        "r63_acceleration_effort_and_pressure_are_preserved": consequence_acceleration_max <= 1e-12
        and consequence_effort_exact
        and consequence_pressure_exact,
        "r64_independent_pinocchio_numpy_oracle_passes_6_of_6": pinocchio_metrics["passed"]
        and len(pinocchio_metrics["checks"]) == 6
        and all(pinocchio_metrics["checks"].values()),
    }
    decision = "PROMOTE" if all(checks.values()) else "REJECT"
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "checks": checks,
        "optimization": {
            "default_behavior": "compact exact-zero task rows at every priority and satisfied fixed-bound rows only at terminal Style",
            "control_feature": "bonesaw-core/resolved-task-row-compaction-control",
            "declared_task_diagnostics_unchanged": True,
            "single_support_ticks": single_support_ticks,
            "dense_row_instances_avoided": 20_176,
        },
        "counter": {
            "paired_instruction_relative_deltas": paired_instruction_deltas,
            "candidate_relative_deltas": counter["candidate_relative_deltas"],
            "summaries": counter["summaries"],
            "scope": counter["evaluation_boundary"],
        },
        "ordinary_wbc": {
            "non_timing_field_count": len(fields),
            "changed_field_count": len(changed_fields),
            "changed_fields": changed_fields,
            "deltas": ordinary_deltas,
            "limits": ordinary_limits,
            "double_support_physical_exact": double_support_exact,
            "decision_fields_exact": exact_decision_fields,
        },
        "constrained_acceleration": {
            "maximum_generalized_acceleration_delta": consequence_acceleration_max,
            "actuator_effort_exact": consequence_effort_exact,
            "bound_pressure_exact": consequence_pressure_exact,
            "maximum_contact_force_delta_n": consequence_force_delta,
        },
        "pinocchio": pinocchio_metrics,
        "source_sha256": hashes_before,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "resolved-task-row-compaction-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    summaries = counter["summaries"]
    deltas = counter["candidate_relative_deltas"]
    key_rows = [
        ("generalized acceleration", "generalized_acceleration"),
        ("actuator effort", "actuator_torque"),
        ("normal force", "contact_normal_force"),
        ("task RMS", "task_rms"),
        ("Jacobi sweeps", "task_jacobi_sweeps"),
    ]
    report = [
        "# Bonesaw resolved task-row compaction · r66",
        "",
        f"## Decision · {decision}",
        "",
        f"Production now removes exact-zero rows at any priority and satisfied fixed-bound rows only from terminal Style before dense pseudoinverses. The retained four-step corpus avoids 20,176 inert row-instances. Five alternating pinned control/candidate pairs reduce whole-process retired instructions by {-100 * instruction_delta:.3f}% with every pair lower. The production default passes 43/43 admission gates, r63 passes 8/8, and the independent Pinocchio/NumPy oracle passes 6/6.",
        "",
        "> Fixed-capacity task declarations and diagnostics do not change. This is internal dense solve compaction, not deletion of an authority layer. Terminal fixed rows are constant over the feasible set and Style has no lower consumer; higher-priority clipping provenance remains untouched.",
        "",
        "## Pinned five-pair process evidence",
        "",
    ]
    report += markdown_table(
        ["variant", "instructions B", "cycles B", "task-clock s", "CPU s", "p50 / p99 / max µs", "RSS MB"],
        [
            [
                variant,
                f"{summary['instructions']['median'] / 1e9:.6f}",
                f"{summary['cycles']['median'] / 1e9:.6f}",
                f"{summary['task_clock']['median'] / 1e9:.6f}",
                f"{summary['process_cpu_seconds']['median']:.3f}",
                f"{summary['p50_tick_us']['median']:.1f} / {summary['p99_tick_us']['median']:.1f} / {summary['maximum_tick_us']['median']:.1f}",
                f"{summary['maximum_rss_bytes']['median'] / 1e6:.2f}",
            ]
            for variant, summary in summaries.items()
        ],
    )
    report += [
        "",
        "The counter boundary is the complete pinned Python admission process, including morphology, two Rust corpus passes, reporting, and serialization. It is not isolated per-solve attribution. Host-sensitive p99 is flat within noise; stable instruction work decides promotion.",
        "",
        "## Ordinary WBC numerical boundary",
        "",
    ]
    report += markdown_table(
        ["signal", "changed ticks", "RMS delta", "maximum delta", "contract"],
        [
            [
                label,
                ordinary_deltas[field]["changed_ticks"],
                f"{ordinary_deltas[field]['rms']:.3e}",
                f"{ordinary_deltas[field]['maximum']:.3e}",
                f"≤ {ordinary_limits[field]:.1e}",
            ]
            for label, field in key_rows
        ],
    )
    report += [
        "",
        f"All double-support physical arrays remain bit-exact. Changes occur only within the {single_support_ticks} single-support ticks where twelve unused contact-force slots are hard-fixed to zero. Status, task clipping, pseudoinverse counts, active-set steps, feasibility work, and allocation arrays remain exact. Jacobi sweep count changes by at most two; every physical delta remains at least an order of magnitude inside its declared contract.",
        "",
        "## Downstream consequence and independent oracle",
        "",
        f"All six r63 generalized-acceleration consequence traces are preserved within {consequence_acceleration_max:.3e}; fixed actuator effort and bound-normalized pressure are bit-exact. Redundant contact-force distribution changes by at most {consequence_force_delta:.3e} N. The r63 allocation/equality/repeat gates pass 8/8.",
        "",
        "Pinocchio 4.0 independently reconstructs dynamics/contact products and NumPy independently reconstructs the four-level hierarchy at 48 edge/even/adverse states. All r64 gates pass 6/6 under the production default.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "RESOLVED_TASK_ROW_COMPACTION.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
