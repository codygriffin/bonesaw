#!/usr/bin/env python3
"""Retain the R97 fixed-level CpuMirrorF32 solve audit and review page."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
import subprocess
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table as markdown_table_rows, render_report_html


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table_rows(headers, rows))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="target/release/bonesaw-cpu-mirror-solve-audit")
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cpu-mirror-solve-r97")
    parser.add_argument("--web-report", default="web/CPU_MIRROR_SOLVE_R97.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def run_audit(binary: pathlib.Path, model: pathlib.Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(binary), str(model)], check=False, capture_output=True, text=True
    )
    try:
        audit = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"mirror solve audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    audit["process_exit_code"] = completed.returncode
    audit["process_stderr"] = completed.stderr.strip()
    return audit


def status_label(value: str) -> str:
    return "".join((" " + c if c.isupper() else c) for c in value).strip()


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    conformance = audit["conformance"]
    budget = audit["budget_exhaustion_case"]
    timing = audit["timing_us_per_seven_agent_batch"]
    mirror_timing = timing["cpu_mirror_f32"]
    exact_timing = timing["cpu_exact_f64"]
    memory = audit["resident_bytes"]
    outcome = "PASS" if audit["status"] == "pass" else "FAIL"
    authority = [
        [row["layer"], row["example"], row["witness"]]
        for row in audit["authority_stack"]
    ]
    agents = [
        [
            row["agent"],
            status_label(row["mirror_status"]),
            status_label(row["exact_status"]),
            f'{row["hard_initial"]:.6g} → {row["hard_best"]:.6g} → {row["hard_final"]:.6g}',
            "/".join(f"{value:.4g}" for value in row["level_rms"]),
            row["task_sweeps"],
        ]
        for row in audit["agents"]
    ]
    return f"""# Bonesaw fixed-level CPU mirror solve · r97

## Outcome

**{outcome}.** R97 freezes the allocation-free `CpuMirrorF32` hierarchy that the future CUDA solve must reproduce: stable row order, 64 hard-projection sweeps, 32 row-action sweeps per active soft level, two hard/prior-level restoration sweeps per task-row update, f32 FMA arithmetic, and no early exit. Its semantics are explicitly **FixedLevelApproximate**. The deployed CPU reference remains **StrictLexicographic f64**.

This is a solver-algorithm admission, not CUDA execution, actuator authority, a plant rollout, or proof that a requested pose is physically realizable. CUDA solve and CUDA Graph remain **NOT IMPLEMENTED**.

## Example authority stack shown in the architecture review

{table(['layer', 'concrete r97 example', 'independent witness'], authority)}

The stack is intentionally not collapsed into a health score. Invariant failure blocks admission; Viability is protected from lower Intent; bounds are a separate resource surface; fixed compute budget is separate from mathematical infeasibility; backend agreement is separate from physical authority; and an unadmitted candidate is separate from an executable command.

## Continuous budget exhaustion, not a timeout cliff

{table(['signal', 'result'], [
    ['mirror status', status_label(budget['mirror_status'])],
    ['strict reference status', status_label(budget['exact_status'])],
    ['hard residual, initial → best → final', f"{budget['initial_hard_violation']:.6g} → {budget['best_hard_violation']:.6g} → {budget['final_hard_violation']:.6g}"],
    ['best candidate retained and finite', budget['candidate_finite']],
    ['admitted command exactly zero', budget['admitted_command_zero']],
    ['infeasibility claim', 'NONE — fixed work exhaustion does not prove primal infeasibility'],
])}

The contradictory two-contact agent always consumes the fixed 64-sweep hard budget. Diagnostics retain how far the solve got. The candidate is queryable for observability and later warm-start research, but it cannot cross the command boundary.

## Conformance gates

{table(['gate', 'result'], [
    ['500-call complete-output byte replay (D1)', conformance['d1_500_call_complete_output_bytes_exact']],
    ['compatible mirror vs strict command maximum absolute delta (D3)', conformance['d3_compatible_command_max_abs']],
    ['D3 / hard / preservation tolerance', conformance['d3_tolerance']],
    ['maximum admitted hard violation', conformance['maximum_admitted_hard_violation']],
    ['maximum priority-preservation drift', conformance['maximum_priority_preservation_drift']],
    ['typed MaxIterations / InvalidProblem / InvalidInput', conformance['typed_budget_invalid_problem_invalid_input']],
    ['invalid-agent neighbor isolation unit gate', conformance['neighbor_isolation_unit_gate']],
    ['inactive padded lanes exactly zero', conformance['padding_inactive_and_zero']],
    ['hot execute allocation unit gate', conformance['allocation_free_hot_path_unit_gate']],
])}

## Per-agent semantic corpus

Level RMS order is Invariant / Viability / Intent / Preference / Style. Cartesian RMS includes all three task axes, including a deliberate z-axis conflict between the Viability point target and the invariant normal contact.

{table(['agent', 'CpuMirrorF32', 'CpuExactF64', 'hard initial → best → final', 'level RMS', 'task sweeps'], agents)}

The corpus covers compatible conflicting soft levels, contradictory hard rows, invalid bounds, invalid emitted data, a hard-row/bound conflict, an empty problem, and an isolated compatible neighbor. The exact solver agrees on executable commands for the compatible agents in this fixture; status names remain backend-specific rather than being coerced into one enum.

## Timing and fixed memory

{table(['backend · seven-agent batch', 'mean µs', 'p50 µs', 'p95 µs', 'p99 µs', 'max µs', 'p99−p50 µs'], [
    ['CpuMirrorF32 fixed-level', f"{mirror_timing['mean']:.3f}", f"{mirror_timing['p50']:.3f}", f"{mirror_timing['p95']:.3f}", f"{mirror_timing['p99']:.3f}", f"{mirror_timing['max']:.3f}", f"{mirror_timing['jitter_p99_minus_p50']:.3f}"],
    ['CpuExactF64 strict', f"{exact_timing['mean']:.3f}", f"{exact_timing['p50']:.3f}", f"{exact_timing['p95']:.3f}", f"{exact_timing['p99']:.3f}", f"{exact_timing['max']:.3f}", f"{exact_timing['jitter_p99_minus_p50']:.3f}"],
])}

{table(['preallocated mirror boundary', 'bytes'], [
    ['solver scratch', memory['mirror_solver_scratch']],
    ['bounds input', memory['mirror_input']],
    ['command + candidate + diagnostics output', memory['mirror_output']],
    ['total', memory['total_fixed_mirror_solve_boundary']],
])}

Timing is retained raw as a 500-sample same-process release-build distribution, per complete seven-agent batch. It is descriptive on this host, not a universal speed claim and not CUDA evidence. The fixed-memory count covers owned solve input, output, and scratch buffers; upstream model-product and row-emission storage is excluded and labeled separately in earlier reports.

## Frozen identity and next gate

{table(['field', 'value'], [
    ['semantics', audit['semantics']],
    ['algorithm + descriptor SHA-256', audit['algorithm_sha256']],
    ['generalized coordinates', audit['layout']['generalized_coordinate_count']],
    ['capacity / stride', f"{audit['layout']['agent_capacity']} / {audit['layout']['agent_stride']}"],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
    ['CUDA solve', audit['cuda_solve']],
    ['CUDA Graph', audit['cuda_graph']],
])}

The next implementation slice may port this exact fingerprinted algorithm to a fixed-buffer CUDA kernel. Device admission still requires independent compiler/runtime presence, D1 repeat, D3 against this mirror, permutation/chunking/padding/isolation/error-injection gates, memory scaling, timing, direct-versus-Graph equivalence, and no-fallback proof. It may not inherit this CPU pass.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    sources = [
        pathlib.Path("crates/bonesaw-cuda/src/cpu_mirror_solve.rs"),
        pathlib.Path("crates/bonesaw-cuda/src/cpu_exact_solve.rs"),
        pathlib.Path("crates/bonesaw-tools/src/bin/cpu_mirror_solve_audit.rs"),
    ]
    metrics = {
        "schema": 1,
        "revision": "cpu-mirror-solve-r97",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [binary, model, *sources]},
        "audit": audit,
    }
    (output / "cpu-mirror-solve-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "CPU_MIRROR_SOLVE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw fixed-level CPU mirror solve · r97")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass" or audit["process_exit_code"] != 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
