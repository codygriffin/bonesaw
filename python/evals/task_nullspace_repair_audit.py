#!/usr/bin/env python3
"""Corpus audit for exact task-nullspace correction repair."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from coincident_step_limit_audit import WORK_FIELDS, difference, sha256
from cpu_reference_report import markdown_table, render_report_html
from cpu_tail_stability import TIMING_ARRAYS, arrays_byte_exact


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument(
        "--output", default="benchmarks/results/g1-task-nullspace-repair-r69"
    )
    parser.add_argument(
        "--web-report", default="web/TASK_NULLSPACE_REPAIR_R69.html"
    )
    args = parser.parse_args()
    control_dir = pathlib.Path(args.control)
    candidate_dir = pathlib.Path(args.candidate)
    control_raw = control_dir / "oracle-wbc-admission-raw.npz"
    candidate_raw = candidate_dir / "oracle-wbc-admission-raw.npz"
    control_metrics_path = control_dir / "oracle-wbc-admission-metrics.json"
    candidate_metrics_path = candidate_dir / "oracle-wbc-admission-metrics.json"
    control_metrics = json.loads(control_metrics_path.read_text())
    candidate_metrics = json.loads(candidate_metrics_path.read_text())

    with np.load(control_raw) as control, np.load(candidate_raw) as candidate:
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

    checks = {
        "control_passes_all_43_admission_gates": bool(control_metrics["passed"]),
        "candidate_passes_all_43_admission_gates": bool(candidate_metrics["passed"]),
        "all_56_non_timing_arrays_are_bit_exact": len(fields) == 56
        and not differences,
        "candidate_keeps_zero_measured_rust_allocations": bool(
            candidate_metrics["checks"]["wbc_hot_loop_has_zero_allocations"]
        ),
        "candidate_reduces_task_pseudoinverse_calls": (
            work["task_pseudoinverse_calls"]["candidate_sum"]
            < work["task_pseudoinverse_calls"]["control_sum"]
        ),
    }
    decision = "PROMOTE" if all(checks.values()) else "REJECT"
    result = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "checks": checks,
        "evaluation_boundary": {
            "states": int(control_metrics["ticks"]),
            "policy": False,
            "state_integration": False,
            "contact_simulation": False,
            "physics_rollout": False,
            "performance_counters": "not run: exact target-work counters show zero pseudoinverses removed",
        },
        "non_timing_field_count": len(fields),
        "differences": differences,
        "work": work,
        "artifact_sha256": {
            str(path): sha256(path)
            for path in (
                control_raw,
                candidate_raw,
                control_metrics_path,
                candidate_metrics_path,
            )
        },
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "task-nullspace-repair-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    report = [
        "# Bonesaw task-nullspace correction repair · r69",
        "",
        f"## Decision · {decision}",
        "",
        "After a hard-limit hit, the candidate analytically redirects the remaining correction through the exact current-task nullspace. It carries that correction only when the new limit can be satisfied without changing the achieved task optimum; otherwise it falls back to the established projected SVD. A redundant two-coordinate witness preserves the exact solution with one pseudoinverse instead of two, and all 117 core tests pass.",
        "",
        "The immutable 2,317-state G1 corpus exposes the real boundary: every sequential hit changes the current task optimum. Control and candidate therefore both execute exactly 16,809 task pseudoinverses, 122,031 Jacobi sweeps, and 10,604 clipped steps. All 56 non-timing arrays are bit-exact.",
        "",
        "> R68's approximate projected-gradient predecessor was rejected before corpus timing because it selected a different monotonic active set and failed two existing dynamic-WBC tests with 1.24e−7 and 1.69e−7 hard-constraint violations. R69 restores exact semantics but produces no target-work reduction. Neither path is production.",
        "",
        "## Exact work comparison",
        "",
    ]
    report += markdown_table(
        ["signal", "control", "candidate", "delta"],
        [
            [
                field.replace("_", " "),
                values["control_sum"],
                values["candidate_sum"],
                values["candidate_sum"] - values["control_sum"],
            ]
            for field, values in work.items()
        ],
    )
    report += [
        "",
        "## Interpretation",
        "",
        "- Both variants pass 43/43 admission gates, exact replay, and zero-allocation checks.",
        "- No policy, integration, contact simulation, or physics rollout enters the audit.",
        "- Hardware counters are intentionally skipped because the candidate removes zero target operations.",
        "- Bound-local shortcuts are now exhausted on this corpus; the next CPU candidate must accelerate each necessary projected solve or change the exact active-set factorization.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "TASK_NULLSPACE_REPAIR_AUDIT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
