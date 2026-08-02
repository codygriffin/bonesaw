#!/usr/bin/env python3
"""Render the R95 CUDA point-product boundary without borrowing device evidence."""

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
    parser.add_argument("--binary", default="target/release/bonesaw-cuda-point-queries-audit")
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-point-queries-r95")
    parser.add_argument("--web-report", default="web/CUDA_POINT_QUERIES_R95.html")
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
        result = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"CUDA point-query audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    result["process_exit_code"] = completed.returncode
    result["process_stderr"] = completed.stderr.strip()
    return result


def validate_sources(paths: list[pathlib.Path], executor: pathlib.Path) -> dict[str, Any]:
    sources = [path.read_text() for path in paths]
    combined = "".join(sources)
    rust = executor.read_text()
    entries = [
        "bonesaw_fk_com_v1",
        "bonesaw_jacobians_v1",
        "bonesaw_dynamics_v1",
        "bonesaw_point_queries_v1",
    ]
    checks = {
        "four_fixed_entries": all(
            f'extern "C" __global__ void {entry}' in source
            for entry, source in zip(entries, sources, strict=True)
        ),
        "one_thread_per_agent": combined.count(
            "blockIdx.x * blockDim.x + threadIdx.x"
        ) == 4,
        "no_device_allocation": "malloc" not in combined and "new " not in combined,
        "no_atomics": "atomic" not in combined,
        "no_block_barrier": "__syncthreads" not in combined,
        "one_stream_one_final_sync": all(
            token in rust
            for token in (
                "self.launch_fk(fk_input)?;",
                "self.launch_jacobians()?;",
                "self.launch_dynamics(dynamics_input)?;",
                "self.launch_point_queries()?;",
                "self.inner.driver.synchronize()?;",
            )
        ),
        "fixed_point_buffers": all(
            token in rust
            for token in (
                "query_frame: CuDevicePtr",
                "query_point_in_frame: CuDevicePtr",
                "point_position: CuDevicePtr",
                "point_jacobian: CuDevicePtr",
                "point_bias_acceleration: CuDevicePtr",
            )
        ),
        "explicit_no_fallback_contract": "fallback behind this API" in rust,
    }
    return {"status": "pass" if all(checks.values()) else "fail", "checks": checks}


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    source = metrics["source_validation"]
    cpu = audit["cpu_mirror_checks"]
    compilers = audit["compiler_probes"]
    runtime = audit["runtime_probe"]["status"]["kind"]
    capabilities = audit["capabilities"]
    layout = audit["layout"]
    device = audit.get("device_checks")
    if audit["status"] == "pass":
        outcome = "DEVICE PASS"
        d3 = device["d3_cpu_mirror"]
        device_summary = table(
            ["device gate", "result"],
            [
                ["D1 complete pipeline bytes", device["d1_complete_pipeline_bytes_exact"]],
                ["D3 position/J/Jdot-v", d3["pass"]],
                ["max position abs+rel", d3["max_position_abs_rel"]],
                ["max Jacobian abs+rel", d3["max_jacobian_abs_rel"]],
                ["max Jdot-v abs+rel", d3["max_bias_acceleration_abs_rel"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE"
        device_summary = (
            "Generated PTX and device D1/D3, timing, memory, isolation, and Graph "
            "gates are **NOT RUN**. CPU execution is not substituted for a device pass."
        )
    else:
        outcome = "FAIL"
        device_summary = f"Device execution failed: `{audit.get('device_error', audit['process_stderr'])}`"

    compiler_rows = [
        [label, probe["status"].upper(), probe.get("detail") or probe.get("ptx_sha256")]
        for label, probe in (
            ("NVRTC FK/CoM", compilers["fk_com"]),
            ("NVRTC Jacobians", compilers["jacobians"]),
            ("NVRTC Dynamics", compilers["dynamics"]),
            ("NVRTC Point queries", compilers["point_queries"]),
        )
    ]
    return f"""# Bonesaw CUDA fixed point products · r95

## Outcome

**{outcome}.** R95 extends the fixed-buffer CUDA slice through compiler-resolved point position, floating point Jacobian, and kinematic bias acceleration `Jdot-v`. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries launches in one ordered stream and synchronizes once. Query frame indices and local offsets are immutable construction-time constants.

This remains a **model-product pipeline**, not a CUDA WBC. Task/constraint emission, hierarchical solve, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

{table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'four fixed entries; no atomics, device allocation, or block barrier'],
    ['CPU mirror witness', 'PASS' if cpu['pass'] else 'FAIL', '200-call complete point-product reference only'],
] + compiler_rows + [
    ['CUDA runtime', runtime.upper(), f"{len(audit['runtime_probe']['devices'])} visible device(s)"],
    ['device D1/D3', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from source or CPU checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'later stages remain gated'],
])}

## Pipeline boundary

{table(['signal', 'value'], [
    ['stage manifest bits', audit['kernel_manifest_bits']],
    ['point source SHA-256', audit['source_sha256']['point_queries']],
    ['query sites', layout['point_query_count']],
    ['layout', f"{layout['body_count']} bodies · {layout['generalized_coordinate_count']} generalized coordinates · capacity {layout['agent_capacity']} · stride {layout['agent_stride']}"],
    ['pipeline synchronization points', 1],
    ['provisional device D3 abs+rel tolerance', '2e-4 for position, J, and Jdot-v'],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

## CPU mirror witness

{table(['gate', 'result'], [
    ['200-call point-product complete-byte repeat', cpu['d1_complete_point_products_bytes_exact']],
    ['active agents typed OK', cpu['active_agents_ok']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free hot-path unit gate', cpu['allocation_free_hot_path_unit_gate']],
    ['f64 position/J/Jdot-v oracle gate', cpu['f64_position_jacobian_jdot_v_gate']],
])}

{device_summary}

## Example authority stack retained for architecture review

{table(['layer', 'example authority', 'separate witness'], [
    ['Invariant', 'floating dynamics + locked support', 'hard residual; never inferred from point-query success'],
    ['Viability', 'finite CoM/support and joint stopping', 'reserve, limiting stable ID, and recovery remain distinct'],
    ['Intent', 'hip/torso/handle point tasks', 'world position, J, Jdot-v, and task residual'],
    ['Resources', 'contact force, actuator effort, power, thermal', 'separate continuous headroom; no pose promise'],
    ['Solver budget', 'iteration/time budget', 'bounded-work status is not infeasibility'],
    ['Backend', 'source → CPU → compiler → runtime → device', 'no aggregate health boolean'],
])}

## Admission boundary

R95 promotes only the implemented point-product stage capability. This host reports point-query NVRTC `{compilers['point_queries']['status']}` and runtime `{runtime}`. A CUDA CI host must retain source/model/query/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-neighbor canaries, permutation, padding, chunking, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    sources = [
        pathlib.Path("crates/bonesaw-cuda/src/kernels/fk_com_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/jacobians_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/dynamics_v1.cu"),
        pathlib.Path("crates/bonesaw-cuda/src/kernels/point_queries_v1.cu"),
    ]
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    metrics = {
        "schema": 1,
        "revision": "cuda-point-queries-r95",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [binary, model, *sources, executor]},
        "source_validation": validate_sources(sources, executor),
        "audit": audit,
    }
    (output / "cuda-point-queries-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "CUDA_POINT_QUERIES_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw CUDA fixed point products · r95"))
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "failed"
        or audit["process_exit_code"] != 0
        or metrics["source_validation"]["status"] == "fail"
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
