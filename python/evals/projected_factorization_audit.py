#!/usr/bin/env python3
"""Joint audit for r70/r71 projected task-solve factorization experiments."""

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
        "--row-gram", default="benchmarks/results/g1-row-gram-r70/candidate"
    )
    parser.add_argument(
        "--row-gram-realization",
        default="benchmarks/results/g1-row-gram-r70/realization",
    )
    parser.add_argument(
        "--row-gram-pinocchio",
        default="benchmarks/results/g1-row-gram-r70/pinocchio",
    )
    parser.add_argument(
        "--rank-one",
        default="benchmarks/results/g1-rank-one-pseudoinverse-r71/candidate",
    )
    parser.add_argument(
        "--rank-one-counter",
        default="benchmarks/results/g1-rank-one-pseudoinverse-r71/counter-ab",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-projected-factorization-r71"
    )
    parser.add_argument(
        "--web-report", default="web/PROJECTED_FACTORIZATION_R71.html"
    )
    return parser.parse_args()


def compare_archives(
    control_path: pathlib.Path, candidate_path: pathlib.Path
) -> tuple[list[str], dict[str, dict[str, float | int]], dict[str, dict[str, int]]]:
    with np.load(control_path) as control, np.load(candidate_path) as candidate:
        fields = sorted((set(control.files) & set(candidate.files)) - TIMING_ARRAYS)
        differences = {
            field: difference(control[field], candidate[field])
            for field in fields
            if not arrays_byte_exact(control[field], candidate[field])
        }
        work = {
            field: {
                "control_sum": int(np.sum(control[field], dtype=np.int64)),
                "candidate_sum": int(np.sum(candidate[field], dtype=np.int64)),
            }
            for field in sorted(WORK_FIELDS)
        }
    return fields, differences, work


def main() -> None:
    args = parse_args()
    control = pathlib.Path(args.control)
    row_gram = pathlib.Path(args.row_gram)
    rank_one = pathlib.Path(args.rank_one)
    control_raw = control / "oracle-wbc-admission-raw.npz"
    row_gram_raw = row_gram / "oracle-wbc-admission-raw.npz"
    rank_one_raw = rank_one / "oracle-wbc-admission-raw.npz"
    control_metrics_path = control / "oracle-wbc-admission-metrics.json"
    row_gram_metrics_path = row_gram / "oracle-wbc-admission-metrics.json"
    rank_one_metrics_path = rank_one / "oracle-wbc-admission-metrics.json"
    realization_path = (
        pathlib.Path(args.row_gram_realization) / "constrained-acceleration-metrics.json"
    )
    pinocchio_path = (
        pathlib.Path(args.row_gram_pinocchio) / "pinocchio-fixed-effort-metrics.json"
    )
    counter_path = (
        pathlib.Path(args.rank_one_counter) / "zero-task-row-compaction-ab-metrics.json"
    )

    control_metrics = json.loads(control_metrics_path.read_text())
    row_gram_metrics = json.loads(row_gram_metrics_path.read_text())
    rank_one_metrics = json.loads(rank_one_metrics_path.read_text())
    realization = json.loads(realization_path.read_text())
    pinocchio = json.loads(pinocchio_path.read_text())
    counter = json.loads(counter_path.read_text())
    fields, row_gram_differences, row_gram_work = compare_archives(
        control_raw, row_gram_raw
    )
    rank_one_fields, rank_one_differences, rank_one_work = compare_archives(
        control_raw, rank_one_raw
    )

    failed_replay_fields = {
        name: values
        for name, values in realization["reference_replay_deltas"].items()
        if not values["within_contract"]
    }
    row_gram_checks = {
        "control_passes_all_43_admission_gates": bool(control_metrics["passed"]),
        "candidate_passes_all_43_admission_gates": bool(row_gram_metrics["passed"]),
        "candidate_keeps_zero_measured_rust_allocations": bool(
            row_gram_metrics["checks"]["wbc_hot_loop_has_zero_allocations"]
        ),
        "r63_replay_contract_passes": bool(
            realization["reference_replay_within_delta_contract"]
        ),
        "r64_independent_pinocchio_numpy_oracle_passes": bool(pinocchio["passed"]),
        "all_56_non_timing_arrays_are_bit_exact": len(fields) == 56
        and not row_gram_differences,
    }
    rank_one_checks = {
        "candidate_passes_all_43_admission_gates": bool(rank_one_metrics["passed"]),
        "all_56_non_timing_arrays_are_bit_exact": len(rank_one_fields) == 56
        and not rank_one_differences,
        "all_five_pinned_pairs_are_bit_exact": bool(
            counter["checks"]["all_56_non_timing_arrays_are_bit_exact_in_every_pair"]
        ),
        "candidate_reduces_median_retired_instructions": bool(
            counter["checks"]["candidate_reduces_median_retired_instructions"]
        ),
        "candidate_reduces_instructions_in_every_pair": bool(
            counter["checks"]["candidate_reduces_retired_instructions_in_every_pair"]
        ),
        "candidate_keeps_zero_measured_rust_allocations": bool(
            counter["checks"]["candidate_keeps_zero_measured_rust_allocations"]
        ),
    }
    decisions = {"row_gram_r70": "REJECT", "rank_one_r71": "REJECT"}
    source_paths = (
        control_raw,
        row_gram_raw,
        rank_one_raw,
        control_metrics_path,
        row_gram_metrics_path,
        rank_one_metrics_path,
        realization_path,
        pinocchio_path,
        counter_path,
    )
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decisions": decisions,
        "evaluation_boundary": {
            "states": int(control_metrics["ticks"]),
            "policy": False,
            "state_integration": False,
            "contact_simulation": False,
            "physics_rollout": False,
            "pinned_counter_pairs": len(counter["paired_instruction_relative_deltas"]),
            "counter_scope": counter["evaluation_boundary"]["counter_scope"],
        },
        "row_gram_r70": {
            "checks": row_gram_checks,
            "changed_non_timing_fields": row_gram_differences,
            "failed_r63_replay_fields": failed_replay_fields,
            "work": row_gram_work,
            "pinocchio_checks": pinocchio["checks"],
        },
        "rank_one_r71": {
            "checks": rank_one_checks,
            "changed_non_timing_fields": rank_one_differences,
            "work": rank_one_work,
            "paired_instruction_relative_deltas": counter[
                "paired_instruction_relative_deltas"
            ],
            "candidate_relative_deltas": counter["candidate_relative_deltas"],
            "counter_summaries": counter["summaries"],
        },
        "artifact_sha256": {str(path): sha256(path) for path in source_paths},
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "projected-factorization-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    row_sweeps = row_gram_work["task_jacobi_sweeps"]
    sweep_delta = row_sweeps["candidate_sum"] - row_sweeps["control_sum"]
    sweep_relative = sweep_delta / row_sweeps["control_sum"]
    inverse_delta = (
        row_gram_work["task_pseudoinverse_calls"]["candidate_sum"]
        - row_gram_work["task_pseudoinverse_calls"]["control_sum"]
    )
    instruction_delta = counter["candidate_relative_deltas"]["instructions"]
    paired = counter["paired_instruction_relative_deltas"]
    report = [
        "# Bonesaw projected-solve factorization audit · r70–r71",
        "",
        "## Decision · REJECT BOTH",
        "",
        f"R70 replaces well-conditioned, full-row-rank wide task SVDs with a guarded row-Gram Cholesky solve. It passes all 43 admission gates, removes {-sweep_delta:,} reported Jacobi sweeps ({-100 * sweep_relative:.2f}%) and {abs(inverse_delta):,} downstream inverse calls, and remains allocation-free. It nevertheless changes {len(row_gram_differences)}/56 non-timing fields, changes the monotonic clipping path, and fails the established r63 replay contract. The candidate is not production.",
        "",
        "The independent r64 Pinocchio/NumPy oracle still passes 6/6 on the r70 fixed-effort queries. That is useful negative evidence: the row-Gram solution is a valid state-local optimizer on the sampled equality problems, but a different floating-point factorization is not equivalent to Bonesaw's deterministic bounded active-set trajectory.",
        "",
        f"R71 specializes only a one-row task projection and reproduces the established kernel bit-for-bit. All 56 non-timing arrays are exact in the full corpus and in every one of five pinned A/B pairs. The retained-instruction median changes by {100 * instruction_delta:+.6f}%; paired changes span {100 * min(paired):+.6f}% to {100 * max(paired):+.6f}%. Since the candidate does not reduce instructions in every pair, it is a workload/noise-floor rejection rather than a semantic failure.",
        "",
        "> Neither result weakens a gate. Production remains r66 feasible-set row compaction. GPU batching remains deferred.",
        "",
        "## Candidate matrix",
        "",
    ]
    report += markdown_table(
        ["candidate", "semantic result", "work result", "reference result", "decision"],
        [
            [
                "r70 guarded row Gram",
                f"{len(row_gram_differences)}/56 fields changed; {len(failed_replay_fields)} r63 replay fields fail",
                f"{sweep_relative * 100:+.2f}% sweeps; {inverse_delta:+d} inverse calls",
                "43/43 admission; independent r64 6/6",
                "REJECT",
            ],
            [
                "r71 exact rank one",
                "56/56 exact in corpus and five pinned pairs",
                f"{100 * instruction_delta:+.6f}% median instructions; mixed pair signs",
                "43/43 admission; zero allocation",
                "REJECT",
            ],
        ],
    )
    report += [
        "",
        "## R70 semantic deltas",
        "",
    ]
    report += markdown_table(
        ["field", "changed samples", "maximum absolute delta", "r63 replay contract"],
        [
            [
                name.replace("_", " "),
                values["changed_samples"],
                f"{values['maximum_absolute_delta']:.6g}",
                "FAIL" if name in failed_replay_fields else "not replay-gating",
            ]
            for name, values in row_gram_differences.items()
        ],
    )
    report += [
        "",
        "## R71 pinned counter evidence",
        "",
    ]
    report += markdown_table(
        ["signal", "candidate − control median"],
        [
            ["retired instructions", f"{100 * instruction_delta:+.6f}%"],
            [
                "process CPU",
                f"{100 * counter['candidate_relative_deltas']['process_cpu_seconds']:+.3f}%",
            ],
            ["p50 tick", f"{100 * counter['candidate_relative_deltas']['p50_tick_us']:+.3f}%"],
            ["p99 tick", f"{100 * counter['candidate_relative_deltas']['p99_tick_us']:+.3f}%"],
        ],
    )
    report += [
        "",
        "## Interpretation",
        "",
        "- Full-row-rank algebra is not enough to preserve this solver's bounded semantics. Floating-point changes before a hard-limit comparison can select a different monotonic active set and produce a different lower-priority optimum.",
        "- Independent reference agreement and replay equivalence answer different questions. R70 demonstrates why both are required.",
        "- An arithmetic-exact specialization is promotable only when the retained workload contains enough matching shapes to reduce stable instructions. R71 does not.",
        "- The next CPU candidate should preserve the established rotation and comparison order while reducing memory traffic inside multi-row Jacobi sweeps, or introduce a separately versioned solver semantics rather than silently replacing the current one.",
        "",
        "## Gates",
        "",
        "### R70",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in row_gram_checks.items()]
    report += ["", "### R71", ""]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in rank_one_checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "PROJECTED_FACTORIZATION_AUDIT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decisions": decisions, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
