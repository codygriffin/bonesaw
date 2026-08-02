#!/usr/bin/env python3
"""Report the generic CUDA FK/CoM boundary without borrowing absent device evidence."""

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
        "--binary", default="target/release/bonesaw-cuda-fk-com-audit"
    )
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-fk-com-r92")
    parser.add_argument("--web-report", default="web/CUDA_FK_COM_R92.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def run_audit(binary: pathlib.Path, model: pathlib.Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(binary), str(model)],
        check=False,
        capture_output=True,
        text=True,
    )
    try:
        result = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"CUDA FK/CoM audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    result["process_exit_code"] = completed.returncode
    result["process_stderr"] = completed.stderr.strip()
    return result


def validate_source(source: pathlib.Path, executor: pathlib.Path) -> dict[str, Any]:
    cuda = source.read_text()
    rust = executor.read_text()
    checks = {
        "fixed_entry": 'extern "C" __global__ void bonesaw_fk_com_v1' in cuda,
        "one_thread_per_agent": (
            "blockIdx.x * blockDim.x + threadIdx.x" in cuda
            and "if (agent >= agent_stride) return" in cuda
        ),
        "no_device_allocation": "malloc" not in cuda and "new " not in cuda,
        "no_atomics": "atomic" not in cuda,
        "no_block_barrier": "__syncthreads" not in cuda,
        "state_then_fk_launches": (
            "state_function" in rust and "fk_function" in rust
        ),
        "fixed_status_staging": "status_staging: Vec<u8>" in rust,
        "explicit_no_fallback_contract": (
            "there is no CPU" in rust and "fallback behind this API" in rust
        ),
    }
    return {"status": "pass" if all(checks.values()) else "fail", "checks": checks}


def report_markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    runtime = audit["runtime_probe"]
    runtime_kind = runtime["status"]["kind"]
    compiler = audit["compiler_probe"]
    compiler_kind = compiler["status"]
    source = metrics["source_validation"]
    device = audit.get("device_checks")
    if audit["status"] == "pass":
        outcome = "DEVICE PASS"
        d2 = device["d2_cpu_mirror"]
        device_summary = markdown_table(
            ["device gate", "result"],
            [
                ["D1 repeated complete output bytes", device["d1_repeat_bytes_exact"]],
                ["D2 CPU-mirror tolerance", d2["pass"]],
                ["D2 max body-pose error", d2["max_abs_body_pose"]],
                ["D2 max CoM error", d2["max_abs_center_of_mass"]],
                ["malformed-agent typing", device["invalid_agent_typed"]],
                ["neighbor isolation", device["neighbor_isolation_exact"]],
                ["padding inactive and zero", device["padding_inactive_and_zero"]],
                ["repeats", device["repeats"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE"
        device_summary = (
            "NVRTC compilation and device D1/D2, launch, timing, isolation, and memory "
            "gates are **NOT RUN**. The report does not substitute CPU execution or call "
            "the FK/CoM stage device-certified."
        )
    else:
        outcome = "FAIL"
        device_summary = (
            f"Device construction/execution failed: `"
            f"{audit.get('device_error', audit['process_stderr'])}`"
        )

    capabilities = audit["capabilities"]
    layout = audit["layout"]
    cpu = audit["cpu_mirror_checks"]
    return f"""# Bonesaw CUDA FK + center of mass · r92

## Outcome

**{outcome}.** R92 adds the first generic model-product device boundary after R91 StateInput: a packed immutable tree ABI, NVRTC-loaded CUDA source, a fixed-buffer Driver API executor, ordered StateInput→FK/CoM launches, and one complete agent per CUDA thread. The hot Rust call owns no allocation and never falls back to the CPU.

This is deliberately a **StateInput + ForwardKinematics + CenterOfMass** implementation claim. Jacobians, dynamics, point products, row emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable on device.

## Independent authority levels

{markdown_table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'entry point, ownership, no atomic/allocation/barrier policy'],
    ['CPU mirror witness', 'PASS' if cpu['pass'] else 'FAIL', 'semantic and deterministic reference only'],
    ['NVRTC compiler', compiler_kind.upper(), compiler.get('detail') or compiler.get('ptx_sha256')],
    ['CUDA runtime', runtime_kind.upper(), f"{len(runtime['devices'])} visible device(s)"],
    ['device D1/D2', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from CPU or source checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'later stages remain gated'],
])}

These rows are intentionally not collapsed into one backend-health boolean. A source implementation can exist while its host compiler or device evidence is unavailable.

## Compiled boundary

{markdown_table(['signal', 'value'], [
    ['stage manifest bits', audit['kernel_manifest_bits']],
    ['CUDA source SHA-256', audit['source_sha256']],
    ['packed model SHA-256', audit['model_pack_sha256']],
    ['NVRTC target', '.'.join(str(value) for value in compiler['target_compute_capability'])],
    ['NVRTC toolkit', compiler.get('toolkit_version')],
    ['generated PTX bytes', compiler.get('ptx_bytes')],
    ['layout', f"{layout['joint_count']} joints · {layout['body_count']} bodies · {layout['coordinate_count']} coordinates · capacity {layout['agent_capacity']} · stride {layout['agent_stride']}"],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

The packed model uses joint AoS constants because all warp lanes read the same joint record, while state and output remain agent-minor SoA. Each thread walks the same topologically compiled tree, writes all body poses, and performs its own mass-weighted CoM reduction. There are no atomics or cross-agent reductions.

## CPU mirror witness

{markdown_table(['gate', 'result'], [
    ['200-call complete-output bitwise repeat', cpu['d1_repeat_bytes_exact']],
    ['active agents typed OK', cpu['active_agents_ok']],
    ['malformed agent typed invalid', cpu['invalid_agent_typed']],
    ['malformed neighbor isolation exact', cpu['neighbor_isolation_exact']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free hot-path Rust unit gate', cpu['allocation_free_hot_path_unit_gate']],
])}

{device_summary}

## Admission boundary

R92 promotes `cuda_mirror_fk_com_f32` only as an implemented stage capability; it does not promote the full CUDA backend. This host reports NVRTC `{compiler_kind}` and runtime `{runtime_kind}`. A CUDA CI host must retain the source/model/toolkit/driver/GPU fingerprint and pass D1 repeated bytes, D2 CPU-mirror tolerance, invalid-neighbor canaries, permutation/padding/chunking, memory scaling, direct-launch timing, error injection, and direct-versus-Graph equivalence before device admission.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    source = pathlib.Path("crates/bonesaw-cuda/src/kernels/fk_com_v1.cu")
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    metrics = {
        "schema": 1,
        "revision": "cuda-fk-com-r92",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {
            str(binary): sha256(binary),
            str(model): sha256(model),
            str(source): sha256(source),
            str(executor): sha256(executor),
        },
        "source_validation": validate_source(source, executor),
        "audit": audit,
    }
    (output / "cuda-fk-com-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "CUDA_FK_COM_AUDIT.md").write_text(markdown)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(markdown, title="Bonesaw CUDA FK + center of mass · r92")
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
