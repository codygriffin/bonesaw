#!/usr/bin/env python3
"""Render the retained R96 CUDA row-emission boundary without borrowing device evidence."""

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
    parser.add_argument("--binary", default="target/release/bonesaw-cuda-emission-audit")
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--output", default="benchmarks/results/cuda-emission-r96")
    parser.add_argument("--web-report", default="web/CUDA_EMISSION_R96.html")
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
            f"CUDA emission audit did not return JSON (exit {completed.returncode}): "
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
        "bonesaw_emission_v1",
    ]
    checks = {
        "five_fixed_entries": all(
            f'extern "C" __global__ void {entry}' in source
            for entry, source in zip(entries, sources, strict=True)
        ),
        "one_thread_per_agent": combined.count(
            "blockIdx.x * blockDim.x + threadIdx.x"
        ) == 5,
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
                "self.launch_emission(emission_input, dynamics_input)?;",
                "self.inner.driver.synchronize()?;",
            )
        ),
        "fixed_emission_constants_and_buffers": all(
            token in rust
            for token in (
                "task_query_slot: CuDevicePtr",
                "task_bandwidth_hz: CuDevicePtr",
                "contact_query_slot: CuDevicePtr",
                "contact_mode: CuDevicePtr",
                "task_jacobian: CuDevicePtr",
                "contact_jacobian: CuDevicePtr",
            )
        ),
        "explicit_no_fallback_contract": "fallback behind this API" in rust,
    }
    return {"status": "pass" if all(checks.values()) else "fail", "checks": checks}


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    source = metrics["source_validation"]
    cpu = audit["cpu_mirror_checks"]
    oracle = cpu["independent_row_oracle"]
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
                ["D1 emitted-row bytes", device["d1_emission_bytes_exact"]],
                ["D3 CPU mirror", d3["pass"]],
                ["max emitted-row abs+rel", d3["maximum_abs_rel"]],
                ["malformed mask + NaN typed", device["malformed_mask_and_nan_typed"]],
                ["neighbor isolation", device["invalid_neighbor_isolation_exact"]],
                ["padding zero", device["padding_inactive_and_zero"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "SOURCE + CPU ROW WITNESS PASS; COMPILER/DEVICE UNAVAILABLE"
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
            ("NVRTC Emission", compilers["emission"]),
        )
    ]
    return f"""# Bonesaw CUDA fixed row emission · r96

## Outcome

**{outcome}.** R96 extends the fixed-buffer CUDA slice through point-attractor and contact-lock row emission. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries → Emission launches in one ordered stream and synchronizes once. Query slots, task bandwidths, and contact modes are immutable construction-time constants; runtime supplies only fixed-shape activation masks and target jets.

This remains a **row-production pipeline**, not a CUDA WBC solve. Hierarchical solve, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent backend authority levels

{table(['authority level', 'result', 'meaning'], [
    ['source / ABI', source['status'].upper(), 'five fixed entries; no atomics, device allocation, or block barrier'],
    ['CPU row witness', 'PASS' if cpu['pass'] else 'FAIL', '200-call emission replay plus independent formulas'],
] + compiler_rows + [
    ['CUDA runtime', runtime.upper(), f"{len(audit['runtime_probe']['devices'])} visible device(s)"],
    ['device D1/D3', 'PASS' if audit['status'] == 'pass' else 'NOT RUN', 'never inherited from source or CPU checks'],
    ['full CudaMirrorF32 / Graph', f"{capabilities['cuda_mirror_f32']} / {capabilities['cuda_graph_executor']}", 'solve and later stages remain gated'],
])}

## Frozen emission boundary

{table(['signal', 'value'], [
    ['stage manifest bits', audit['kernel_manifest_bits']],
    ['emission source SHA-256', audit['source_sha256']['emission']],
    ['fixed topology', f"{layout['point_task_count']} point tasks · {layout['contact_lock_count']} contact locks · {layout['point_query_count']} point sites"],
    ['batch layout', f"{layout['generalized_coordinate_count']} generalized coordinates · capacity {layout['agent_capacity']} · stride {layout['agent_stride']}"],
    ['contact modes covered', 'LockedPoint · NormalPoint · RollingPoint · Disabled'],
    ['pipeline synchronization points', 1],
    ['provisional device D3 abs+rel tolerance', '5e-4 over every emitted floating row field'],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

Point tasks emit stable position error, velocity error, desired acceleration, Jacobian, and bias-corrected right-hand side. Contact locks copy only axes enabled by their compiled kinematic mode. Inactive, invalid, disabled-axis, and padded rows remain exactly zero.

## CPU row witness

{table(['gate', 'result'], [
    ['200-call complete emission-byte replay', cpu['d1_emission_bytes_exact']],
    ['independent formula oracle', oracle['pass']],
    ['maximum formula error', oracle['maximum_absolute_error']],
    ['malformed mask and active NaN typed invalid', cpu['malformed_mask_and_nan_typed']],
    ['invalid-agent neighbor isolation exact', cpu['invalid_neighbor_isolation_exact']],
    ['inactive and mode-filtered rows zero', cpu['inactive_and_mode_filtered_rows_zero']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free hot-path unit gate', cpu['allocation_free_hot_path_unit_gate']],
])}

{device_summary}

## Example authority stack retained for architecture review

{table(['layer', 'example authority', 'separate witness'], [
    ['Invariant', 'floating dynamics + compiled contact rows', 'hard residual; row emission is not solve feasibility'],
    ['Viability', 'finite support and joint stopping', 'reserve and limiting stable ID remain distinct'],
    ['Intent', 'hip/torso/handle point rows', 'position, velocity, desired acceleration, J, and rhs'],
    ['Resources', 'contact force, actuator effort, power, thermal', 'continuous headroom; no pose promise'],
    ['Solver budget', 'iteration/time budget', 'not represented by emission success'],
    ['Backend', 'source → CPU → compiler → runtime → device', 'no aggregate health boolean'],
])}

## Admission boundary

R96 promotes only the implemented row-emission stage capability. This host reports emission NVRTC `{compilers['emission']['status']}` and runtime `{runtime}`. A CUDA CI host must retain source/model/query/task/contact/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-input canaries, permutation, padding, chunking, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission. Successful row production does not certify a compatible hierarchical solve or physical realizability.
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
        pathlib.Path("crates/bonesaw-cuda/src/kernels/emission_v1.cu"),
    ]
    executor = pathlib.Path("crates/bonesaw-cuda/src/cuda_fk_com.rs")
    metrics = {
        "schema": 1,
        "revision": "cuda-emission-r96",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [binary, model, *sources, executor]},
        "source_validation": validate_sources(sources, executor),
        "audit": audit,
    }
    (output / "cuda-emission-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "CUDA_EMISSION_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw CUDA fixed row emission · r96"))
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "failed"
        or audit["process_exit_code"] != 0
        or metrics["source_validation"]["status"] == "fail"
        or not audit["cpu_mirror_checks"]["pass"]
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
