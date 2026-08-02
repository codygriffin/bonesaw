#!/usr/bin/env python3
"""Render the R94 CUDA dynamics boundary without borrowing absent device evidence."""

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
    parser.add_argument("--binary", default="target/release/bonesaw-cuda-dynamics-audit")
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-dynamics-r94")
    parser.add_argument("--web-report", default="web/CUDA_DYNAMICS_R94.html")
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
            f"CUDA dynamics audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    result["process_exit_code"] = completed.returncode
    result["process_stderr"] = completed.stderr.strip()
    return result


def validate_sources(paths: list[pathlib.Path], executor: pathlib.Path) -> dict[str, Any]:
    sources = [path.read_text() for path in paths]
    combined = "".join(sources)
    rust = executor.read_text()
    checks = {
        "fk_entry": 'extern "C" __global__ void bonesaw_fk_com_v1' in sources[0],
        "jacobian_entry": 'extern "C" __global__ void bonesaw_jacobians_v1' in sources[1],
        "dynamics_entry": 'extern "C" __global__ void bonesaw_dynamics_v1' in sources[2],
        "one_thread_per_agent": combined.count("blockIdx.x * blockDim.x + threadIdx.x") == 3,
        "no_device_allocation": "malloc" not in combined and "new " not in combined,
        "no_atomics": "atomic" not in combined,
        "no_block_barrier": "__syncthreads" not in combined,
        "one_stream_one_final_sync": (
            "self.launch_fk(fk_input)?;" in rust
            and "self.launch_jacobians()?;" in rust
            and "self.launch_dynamics(dynamics_input)?;" in rust
            and "self.inner.driver.synchronize()?;" in rust
        ),
        "fixed_dynamics_buffers": all(
            token in rust
            for token in (
                "mass_matrix: CuDevicePtr",
                "bias_force: CuDevicePtr",
                "centroidal_map: CuDevicePtr",
                "angular_velocity: CuDevicePtr",
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
                ["D1 complete pipeline bytes", device["d1_repeat_bytes_exact"]],
                ["D3 M/h/Ag", d3["pass"]],
                ["max M abs/rel", d3["max_mass_matrix_abs_rel"]],
                ["max h abs/rel", d3["max_bias_force_abs_rel"]],
                ["max Ag abs/rel", d3["max_centroidal_map_abs_rel"]],
                ["invalid typing", device["invalid_agent_typed"]],
                ["neighbor isolation", device["neighbor_isolation_exact"]],
                ["padding zero", device["padding_inactive_and_zero"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE"
        device_summary = (
            "Generated PTX and device D1/D3, timing, isolation, memory, and Graph "
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
        )
    ]
    return f"""# Bonesaw CUDA floating dynamics · r94

## Outcome

**{outcome}.** R94 extends the fixed-buffer device slice through floating `M(q)`, `h(q,v,g)`, and `Ag(q)`. StateInput → FK/CoM → Jacobians → Dynamics launches in one ordered stream and synchronizes once. One CUDA thread owns one complete agent and all body reductions remain in deterministic body/coordinate order.

This is still a **model-product pipeline**, not the full WBC. Point products, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

{table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'three fixed entries; no atomics, device allocation, or block barrier'],
    ['CPU mirror witness', 'PASS' if cpu['pass'] else 'FAIL', '200-call complete-product reference only'],
] + compiler_rows + [
    ['CUDA runtime', runtime.upper(), f"{len(audit['runtime_probe']['devices'])} visible device(s)"],
    ['device D1/D3', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from source or CPU checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'later stages remain gated'],
])}

## Pipeline boundary

{table(['signal', 'value'], [
    ['stage manifest bits', audit['kernel_manifest_bits']],
    ['FK/CoM source SHA-256', audit['source_sha256']['fk_com']],
    ['Jacobian source SHA-256', audit['source_sha256']['jacobians']],
    ['Dynamics source SHA-256', audit['source_sha256']['dynamics']],
    ['packed model SHA-256', audit['model_pack_sha256']],
    ['layout', f"{layout['joint_count']} joints · {layout['body_count']} bodies · {layout['generalized_coordinate_count']} generalized coordinates · capacity {layout['agent_capacity']} · stride {layout['agent_stride']}"],
    ['pipeline synchronization points', 1],
    ['provisional device D3 abs+rel tolerance', '5e-4 for M, h, and Ag'],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

The immutable model pack now includes each body's row-major inertia about its center of mass. The device recursion consumes world-expressed root/joint velocity and per-agent gravity, then assembles the symmetric mass matrix, bias covector, and centroidal map from the admitted frame Jacobians with explicit FMA.

## CPU mirror witness

{table(['gate', 'result'], [
    ['200-call FK+J+M/h/Ag complete-byte repeat', cpu['d1_repeat_bytes_exact']],
    ['active agents typed OK', cpu['active_agents_ok']],
    ['invalid velocity typed invalid', cpu['invalid_dynamics_input_typed']],
    ['invalid neighbor isolation exact', cpu['neighbor_isolation_exact']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free hot-path unit gate', cpu['allocation_free_hot_path_unit_gate']],
    ['Pinocchio + physical-identity f64 gate', cpu['f64_pinocchio_and_physical_identity_gate']],
])}

{device_summary}

## Admission boundary

R94 promotes only the implemented floating-dynamics stage capability. This host reports NVRTC FK/CoM `{compilers['fk_com']['status']}`, Jacobians `{compilers['jacobians']['status']}`, Dynamics `{compilers['dynamics']['status']}`, and runtime `{runtime}`. A CUDA CI host must retain source/model/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, permutation, padding, chunking, canaries, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission.
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
    ]
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    metrics = {
        "schema": 1,
        "revision": "cuda-dynamics-r94",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {
            str(path): sha256(path) for path in [binary, model, *sources, executor]
        },
        "source_validation": validate_sources(sources, executor),
        "audit": audit,
    }
    (output / "cuda-dynamics-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "CUDA_DYNAMICS_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw CUDA floating dynamics · r94"))
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "failed"
        or audit["process_exit_code"] != 0
        or metrics["source_validation"]["status"] == "fail"
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
