#!/usr/bin/env python3
"""Retain the R98 CUDA fixed-level solve boundary without borrowing CPU evidence."""

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
    parser.add_argument("--binary", default="target/release/bonesaw-cuda-solve-audit")
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-solve-r98")
    parser.add_argument("--web-report", default="web/CUDA_SOLVE_R98.html")
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
            f"CUDA solve audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    audit["process_exit_code"] = completed.returncode
    audit["process_stderr"] = completed.stderr.strip()
    return audit


def validate_source(source: pathlib.Path, executor: pathlib.Path) -> dict[str, Any]:
    cuda = source.read_text()
    rust = executor.read_text()
    checks = {
        "fixed_entry": 'extern "C" __global__ void bonesaw_solve_v1' in cuda,
        "one_thread_per_agent": "blockIdx.x * blockDim.x + threadIdx.x" in cuda,
        "fixed_64_32_budget": all(
            token in cuda for token in ("HARD_SWEEPS = 64", "TASK_SWEEPS = 32")
        ),
        "candidate_command_split": all(
            token in cuda
            for token in ("float* command", "float* candidate", "status[agent] = 3")
        ),
        "no_device_allocation": "malloc" not in cuda and "new " not in cuda,
        "no_atomics": "atomic" not in cuda,
        "no_block_barrier": "__syncthreads" not in cuda,
        "one_stream_one_final_sync": all(
            token in rust
            for token in (
                "self.launch_emission(emission_input, dynamics_input)?;",
                "self.launch_solve(solve_input)?;",
                "self.inner.driver.synchronize()?;",
            )
        ),
        "fixed_global_scratch": all(
            token in rust
            for token in (
                "locked_rows: CuDevicePtr",
                "locked_reference_rhs: CuDevicePtr",
                "stage.locked_rows =",
                "stage.candidate =",
            )
        ),
        "explicit_no_fallback_contract": "fallback behind this API" in rust,
    }
    return {"status": "pass" if all(checks.values()) else "fail", "checks": checks}


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    stage = audit["cuda_stage"]
    cpu = audit["conformance"]
    source = metrics["source_validation"]
    compilers = stage["compiler_probes"]
    runtime = stage["runtime_probe"]["status"]["kind"]
    capabilities = stage["capabilities"]
    if audit["status"] == "pass":
        outcome = "DEVICE PASS"
        device = stage["device_checks"]
        device_summary = table(
            ["device gate", "result"],
            [
                ["D1 complete output bytes", device["d1_200_call_complete_output_bytes_exact"]],
                ["D3 CPU mirror", device["d3_cpu_mirror"]["pass"]],
                ["maximum solve abs error", device["d3_cpu_mirror"]["maximum_absolute_error"]],
                ["hot Rust allocations", "0 (unit gate)"],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU MIRROR PASS; COMPILER/DEVICE UNAVAILABLE"
        device_summary = (
            "Generated solve PTX, module loading, device execution, D1/D3, memory, "
            "timing, isolation, and direct-versus-Graph gates are **NOT RUN**. "
            "The CPU mirror is not substituted for device evidence."
        )
    else:
        outcome = "FAIL"
        device_summary = f"Device construction or execution failed: `{stage.get('device_error', audit['process_stderr'])}`"

    compiler_rows = [
        [label, probe["status"].upper(), probe.get("detail") or probe.get("ptx_sha256")]
        for label, probe in (
            ("NVRTC FK/CoM", compilers["fk_com"]),
            ("NVRTC Jacobians", compilers["jacobians"]),
            ("NVRTC Dynamics", compilers["dynamics"]),
            ("NVRTC Point queries", compilers["point_queries"]),
            ("NVRTC Emission", compilers["emission"]),
            ("NVRTC Solve", compilers["solve"]),
        )
    ]
    authority_rows = [
        [row["layer"], row["example"], row["witness"]]
        for row in audit["authority_stack"]
    ]
    memory = audit["resident_bytes"]
    budget = audit["budget_exhaustion_case"]
    return f"""# Bonesaw CUDA fixed-level hierarchical solve · r98

## Outcome

**{outcome}.** R98 adds the real no-fallback CUDA `HierarchicalSolve` source, fixed global scratch, compiler probe, manifest stage, capability, host allocation/launch/copy boundary, and conditional device conformance test. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries → Emission → Solve is ordered in one stream with one terminal synchronization.

This does not promote the full CUDA mirror. Actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent backend authority levels

{table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'fixed entry and scratch; no allocation, atomic, or block barrier'],
    ['CpuMirrorF32 algorithm witness', 'PASS' if cpu['pass'] else 'FAIL', '500-call replay, strict-f64 compatible command, budget typing'],
] + compiler_rows + [
    ['CUDA runtime', runtime.upper(), f"{len(stage['runtime_probe']['devices'])} visible device(s)"],
    ['device solve D1/D3', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from source or CPU checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'later stages remain gated'],
])}

## Frozen solve boundary

{table(['signal', 'value'], [
    ['stage manifest bits', stage['kernel_manifest_bits']],
    ['solve source SHA-256', stage['source_sha256']],
    ['mirror algorithm SHA-256', stage['algorithm_sha256']],
    ['arithmetic semantics', audit['semantics']],
    ['hard / task budget', '64 hard sweeps · 32 sweeps per active level · no early exit'],
    ['batch layout', f"{audit['layout']['generalized_coordinate_count']} generalized coordinates · capacity {audit['layout']['agent_capacity']} · stride {audit['layout']['agent_stride']}"],
    ['fixed CPU solve input/output/scratch', f"{memory['total_fixed_mirror_solve_boundary']} B"],
    ['hidden CPU fallback', stage['hidden_cpu_fallback']],
])}

One thread owns one complete agent and every reduction order. Immutable task priority/weight and contact mode arrays are uploaded once. Bounds are copied into fixed input buffers; commands, retained candidates, typed status, per-level residual/preservation fields, hard progress, margins, and work counters use fixed device outputs. Scratch is global agent-minor storage sized at construction.

## Example authority stack retained for architecture review

{table(['layer', 'concrete example', 'independent witness'], authority_rows)}

No layer is aggregated. Backend agreement cannot certify physical authority, a lower task cannot overwrite a higher achieved row, and a finite candidate is not an executable command.

## Continuous budget case

{table(['signal', 'result'], [
    ['status', budget['mirror_status']],
    ['hard residual initial → best → final', f"{budget['initial_hard_violation']} → {budget['best_hard_violation']} → {budget['final_hard_violation']}"],
    ['finite candidate retained', budget['candidate_finite']],
    ['admitted command zero', budget['admitted_command_zero']],
    ['infeasibility conclusion', 'NONE — MaxIterations is fixed-work exhaustion'],
])}

## CPU algorithm witness

{table(['gate', 'result'], [
    ['500-call complete-output byte replay', cpu['d1_500_call_complete_output_bytes_exact']],
    ['compatible command max absolute delta vs strict f64', cpu['d3_compatible_command_max_abs']],
    ['maximum admitted hard violation', cpu['maximum_admitted_hard_violation']],
    ['maximum priority-preservation drift', cpu['maximum_priority_preservation_drift']],
    ['typed budget / invalid-problem / invalid-input', cpu['typed_budget_invalid_problem_invalid_input']],
    ['neighbor isolation unit gate', cpu['neighbor_isolation_unit_gate']],
    ['padding zero', cpu['padding_inactive_and_zero']],
    ['allocation-free execute unit gate', cpu['allocation_free_hot_path_unit_gate']],
])}

{device_summary}

## Admission boundary

R98 promotes only the implemented CUDA solve source/executor capability. This host reports solve NVRTC `{compilers['solve']['status']}` and runtime `{runtime}`. A CUDA host must retain source/model/descriptor/algorithm/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-input canaries, permutation, padding, chunking, neighbor isolation, memory scaling, timing, error injection, isolated executors, and direct-versus-Graph gates before device admission. Successful CPU solve or CUDA row emission does not satisfy that contract.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    solve_source = pathlib.Path("crates/bonesaw-cuda/src/kernels/solve_v1.cu")
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    sources = [
        pathlib.Path("crates/bonesaw-cuda/src/kernels/fk_com_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/jacobians_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/dynamics_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/point_queries_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/emission_v1.cu"),
        solve_source,
    ]
    metrics = {
        "schema": 1,
        "revision": "cuda-solve-r98",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {
            str(path): sha256(path) for path in [binary, model, *sources, executor]
        },
        "source_validation": validate_source(solve_source, executor),
        "audit": audit,
    }
    (output / "cuda-solve-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "CUDA_SOLVE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw CUDA fixed-level solve · r98"))
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "fail"
        or audit["process_exit_code"] != 0
        or metrics["source_validation"]["status"] == "fail"
        or not audit["conformance"]["pass"]
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
