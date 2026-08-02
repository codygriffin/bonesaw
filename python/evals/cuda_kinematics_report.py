#!/usr/bin/env python3
"""Report the CUDA kinematics pipeline without borrowing absent device evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
import subprocess
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import (
    markdown_table as markdown_table_rows,
    render_report_html,
)


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table_rows(headers, rows))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--binary", default="target/release/bonesaw-cuda-kinematics-audit"
    )
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-kinematics-r93")
    parser.add_argument("--web-report", default="web/CUDA_KINEMATICS_R93.html")
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
            f"CUDA kinematics audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    result["process_exit_code"] = completed.returncode
    result["process_stderr"] = completed.stderr.strip()
    return result


def validate_sources(
    fk_source: pathlib.Path, jacobian_source: pathlib.Path, executor: pathlib.Path
) -> dict[str, Any]:
    fk = fk_source.read_text()
    jacobian = jacobian_source.read_text()
    rust = executor.read_text()
    combined = fk + jacobian
    checks = {
        "fk_entry": 'extern "C" __global__ void bonesaw_fk_com_v1' in fk,
        "jacobian_entry": (
            'extern "C" __global__ void bonesaw_jacobians_v1' in jacobian
        ),
        "one_thread_per_agent": combined.count(
            "blockIdx.x * blockDim.x + threadIdx.x"
        ) == 2,
        "no_device_allocation": "malloc" not in combined and "new " not in combined,
        "no_atomics": "atomic" not in combined,
        "no_block_barrier": "__syncthreads" not in combined,
        "single_stream_launch_chain": (
            "launch_fk(input)?" in rust
            and "launch(stage.function" in rust
            and "self.inner.driver.synchronize()?" in rust
        ),
        "fixed_output_buffers": (
            "frame_jacobian: CuDevicePtr" in rust
            and "center_of_mass_jacobian: CuDevicePtr" in rust
        ),
        "explicit_no_fallback_contract": (
            "there is no CPU" in rust and "fallback behind this API" in rust
        ),
    }
    return {"status": "pass" if all(checks.values()) else "fail", "checks": checks}


def report_markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    runtime_kind = audit["runtime_probe"]["status"]["kind"]
    compilers = audit["compiler_probes"]
    fk_compiler = compilers["fk_com"]
    jacobian_compiler = compilers["jacobians"]
    cpu = audit["cpu_mirror_checks"]
    source = metrics["source_validation"]
    capabilities = audit["capabilities"]
    layout = audit["layout"]
    device = audit.get("device_checks")
    if audit["status"] == "pass":
        outcome = "DEVICE PASS"
        d3 = device["d3_cpu_mirror"]
        device_summary = markdown_table(
            ["device gate", "result"],
            [
                ["D1 complete pipeline bytes", device["d1_repeat_bytes_exact"]],
                ["D3 CPU mirror", d3["pass"]],
                ["max frame-Jacobian abs/rel", d3["max_frame_jacobian_abs_rel"]],
                ["max CoM-Jacobian abs/rel", d3["max_com_jacobian_abs_rel"]],
                ["malformed-agent typing", device["invalid_agent_typed"]],
                ["neighbor isolation", device["neighbor_isolation_exact"]],
                ["padding inactive and zero", device["padding_inactive_and_zero"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE"
        device_summary = (
            "NVRTC compilation and device D1/D3, launch, timing, isolation, memory, "
            "and Graph gates are **NOT RUN**. CPU execution is not substituted for a "
            "device pass."
        )
    else:
        outcome = "FAIL"
        device_summary = f"Device execution failed: `{audit.get('device_error', audit['process_stderr'])}`"

    return f"""# Bonesaw CUDA kinematics pipeline · r93

## Outcome

**{outcome}.** R93 extends the generic device slice through floating-root frame-origin Jacobians and the center-of-mass Jacobian. The fixed-buffer executor now launches StateInput → FK/CoM → Jacobians in one stream, synchronizes once, and copies fixed outputs. One CUDA thread owns one complete agent; no stage performs a cross-agent reduction.

This remains a **kinematics-only** device implementation. Dynamics, point products, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

{markdown_table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'two fixed entries, fixed ownership, no atomics/allocation/barrier'],
    ['CPU mirror witness', 'PASS' if cpu['pass'] else 'FAIL', 'semantic and deterministic reference only'],
    ['NVRTC FK/CoM', fk_compiler['status'].upper(), fk_compiler.get('detail') or fk_compiler.get('ptx_sha256')],
    ['NVRTC Jacobians', jacobian_compiler['status'].upper(), jacobian_compiler.get('detail') or jacobian_compiler.get('ptx_sha256')],
    ['CUDA runtime', runtime_kind.upper(), f"{len(audit['runtime_probe']['devices'])} visible device(s)"],
    ['device D1/D3', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from CPU/source checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'later stages remain gated'],
])}

The authority rows remain independent. Implemented code, a CPU reference, a compiler, and a conforming device are different facts.

## Pipeline boundary

{markdown_table(['signal', 'value'], [
    ['stage manifest bits', audit['kernel_manifest_bits']],
    ['FK/CoM source SHA-256', audit['source_sha256']['fk_com']],
    ['Jacobian source SHA-256', audit['source_sha256']['jacobians']],
    ['packed model SHA-256', audit['model_pack_sha256']],
    ['compiler target', '.'.join(str(value) for value in fk_compiler['target_compute_capability'])],
    ['layout', f"{layout['joint_count']} joints · {layout['body_count']} bodies · {layout['generalized_coordinate_count']} generalized coordinates · capacity {layout['agent_capacity']} · stride {layout['agent_stride']}"],
    ['pipeline synchronization points', 1],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

The model pack now includes a constant parent-joint index for every body. Each agent thread walks its ancestor chain in deterministic order, constructs root and joint columns in `[angular; linear]` convention, and accumulates body-mass CoM columns with explicit FMA.

## CPU mirror witness

{markdown_table(['gate', 'result'], [
    ['200-call FK+Jacobian complete-byte repeat', cpu['d1_repeat_bytes_exact']],
    ['active agents typed OK', cpu['active_agents_ok']],
    ['malformed agent typed invalid', cpu['invalid_agent_typed']],
    ['malformed neighbor isolation exact', cpu['neighbor_isolation_exact']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free hot-path unit gate', cpu['allocation_free_hot_path_unit_gate']],
    ['Pinocchio + finite-difference f64 gate', cpu['f64_pinocchio_and_finite_difference_gate']],
])}

{device_summary}

## Admission boundary

R93 promotes only the implemented Jacobian-stage capability. This host reports FK/CoM NVRTC `{fk_compiler['status']}`, Jacobian NVRTC `{jacobian_compiler['status']}`, and runtime `{runtime_kind}`. A CUDA CI host must retain the model/source/PTX/toolkit/driver/GPU fingerprint and pass D1 repeat, D3 abs+rel tolerance, permutation, padding, chunking, cross-agent canaries, memory scaling, direct-launch timing, error injection, isolated-executor equivalence, and direct-versus-Graph equivalence before device admission.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    fk_source = pathlib.Path("crates/bonesaw-cuda/src/kernels/fk_com_v1.cu")
    jacobian_source = pathlib.Path(
        "crates/bonesaw-cuda/src/kernels/jacobians_v1.cu"
    )
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    metrics = {
        "schema": 1,
        "revision": "cuda-kinematics-r93",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {
            str(binary): sha256(binary),
            str(model): sha256(model),
            str(fk_source): sha256(fk_source),
            str(jacobian_source): sha256(jacobian_source),
            str(executor): sha256(executor),
        },
        "source_validation": validate_sources(
            fk_source, jacobian_source, executor
        ),
        "audit": audit,
    }
    (output / "cuda-kinematics-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "CUDA_KINEMATICS_AUDIT.md").write_text(markdown)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(markdown, title="Bonesaw CUDA kinematics pipeline · r93")
    )
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "failed"
        or audit["process_exit_code"] != 0
        or metrics["source_validation"]["status"] == "fail"
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
