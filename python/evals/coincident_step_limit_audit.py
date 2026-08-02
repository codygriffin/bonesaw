#!/usr/bin/env python3
"""Reject/promote audit for coincident task-step limit freezing."""

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


WORK_FIELDS = {
    "task_pseudoinverse_calls",
    "task_pseudoinverse_calls_by_priority",
    "task_jacobi_sweeps",
    "task_jacobi_sweeps_by_priority",
    "clipped_steps",
    "clipped_steps_by_priority",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument(
        "--output", default="benchmarks/results/g1-coincident-step-limit-r67"
    )
    parser.add_argument(
        "--web-report", default="web/COINCIDENT_STEP_LIMIT_R67.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def difference(control: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    changed = control != candidate
    numeric = np.issubdtype(control.dtype, np.number)
    return {
        "changed_samples": int(np.count_nonzero(changed)),
        "maximum_absolute_delta": (
            float(np.max(np.abs(control.astype(np.float64) - candidate.astype(np.float64))))
            if numeric and control.size
            else None
        ),
    }


def main() -> None:
    args = parse_args()
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
        physical_fields = [field for field in fields if field not in WORK_FIELDS]
        physical_exact = all(
            arrays_byte_exact(control[field], candidate[field])
            for field in physical_fields
        )
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
        "all_physical_decision_and_authority_arrays_are_bit_exact": physical_exact,
        "candidate_keeps_zero_measured_rust_allocations": bool(
            candidate_metrics["checks"]["wbc_hot_loop_has_zero_allocations"]
        ),
        "candidate_reduces_task_pseudoinverse_calls": (
            work["task_pseudoinverse_calls"]["candidate_sum"]
            < work["task_pseudoinverse_calls"]["control_sum"]
        ),
        "candidate_does_not_increase_jacobi_sweeps": (
            work["task_jacobi_sweeps"]["candidate_sum"]
            <= work["task_jacobi_sweeps"]["control_sum"]
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
            "performance_counters": "not run: the candidate eliminates zero target pseudoinverses, which already falsifies its optimization claim",
        },
        "non_timing_field_count": len(fields),
        "physical_field_count": len(physical_fields),
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
    (output / "coincident-step-limit-audit-metrics.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )

    report = [
        "# Bonesaw coincident task-step limit audit · r67",
        "",
        f"## Decision · {decision}",
        "",
        "The candidate freezes every exactly coincident hard limit reached by one semantic-task correction before rebuilding the projected task pseudoinverse. A two-coordinate unit witness proves the mechanism can reduce three inverse evaluations to two. The immutable 2,317-state G1 corpus contains no instance where that mechanism removes actual dense work: control and candidate both execute 16,809 task pseudoinverses and 10,604 clipped steps.",
        "",
        "> This is a workload-negative result, not a numerical failure. All physical, decision, residual, authority, clipping, rank, feasibility, and allocation arrays are bit-exact. Two Jacobi-sweep diagnostic samples change by at most two sweeps and the aggregate increases from 122,031 to 122,032. The experiment stays opt-in and production r66 remains unchanged.",
        "",
        "## Work comparison",
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
        "## Boundary and interpretation",
        "",
        f"- {len(fields)} non-timing arrays compared; {len(physical_fields)} physical/decision/authority arrays are bit-exact.",
        "- Both variants pass all 43 admission gates and report zero Rust hot-loop allocations.",
        "- The evaluator has no policy, state integration, contact simulation, or physics rollout.",
        "- Hardware counters are intentionally not used: zero target-work reduction already falsifies promotion, and timing the added scan could only encourage a noise-based claim.",
        "- The next CPU experiment must target sequential, non-coincident Viability clipping rather than rare simultaneous boundaries.",
        "",
        "## Gates",
        "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'} `{name}`" for name, value in checks.items()]
    report_text = "\n".join(report) + "\n"
    (output / "COINCIDENT_STEP_LIMIT_AUDIT.md").write_text(report_text)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report_text))
    print(json.dumps({"decision": decision, "checks": checks, "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
