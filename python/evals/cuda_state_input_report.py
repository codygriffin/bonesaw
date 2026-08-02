#!/usr/bin/env python3
"""Audit the first real CUDA mirror stage without hiding device unavailability."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
import shutil
import subprocess
import tempfile
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
        "--binary", default="target/release/bonesaw-cuda-state-input-audit"
    )
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument(
        "--output", default="benchmarks/results/cuda-state-input-r91"
    )
    parser.add_argument(
        "--web-report", default="web/CUDA_STATE_INPUT_R91.html"
    )
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
            f"CUDA audit did not return JSON (exit {completed.returncode}): "
            f"{completed.stderr.strip()}"
        ) from error
    result["process_exit_code"] = completed.returncode
    result["process_stderr"] = completed.stderr.strip()
    return result


def validate_ptx(ptx: pathlib.Path) -> dict[str, Any]:
    ptxas = shutil.which("ptxas")
    if ptxas is None:
        return {"status": "unavailable", "ptxas": None, "stderr": ""}
    with tempfile.TemporaryDirectory(prefix="bonesaw-ptxas-") as directory:
        output = pathlib.Path(directory) / "state_input_v1.cubin"
        completed = subprocess.run(
            [ptxas, "--gpu-name=sm_50", "--output-file", str(output), str(ptx)],
            check=False,
            capture_output=True,
            text=True,
        )
        return {
            "status": "pass" if completed.returncode == 0 else "fail",
            "ptxas": ptxas,
            "exit_code": completed.returncode,
            "stderr": completed.stderr.strip(),
            "cubin_bytes": output.stat().st_size if output.exists() else 0,
        }


def report_markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    runtime = audit["runtime_probe"]
    runtime_kind = runtime["status"]["kind"]
    ptx_validation = metrics["ptx_validation"]
    device = audit.get("device_checks")
    if audit["status"] == "pass":
        outcome = "PASS"
        device_summary = markdown_table(
            ["device gate", "result"],
            [
                ["D1 repeated output bytes", device["d1_repeat_bytes_exact"]],
                ["D2 CPU mirror output bytes", device["d2_cpu_mirror_bytes_exact"]],
                ["malformed-agent typing", device["invalid_agent_typed"]],
                ["neighbor isolation", device["neighbor_isolation_exact"]],
                ["padding inactive and zero", device["padding_inactive_and_zero"]],
                ["repeats", device["repeats"]],
            ],
        )
    elif audit["status"] == "unavailable":
        outcome = "DEVICE UNAVAILABLE"
        device_summary = (
            "Device D1/D2, launch, timing, and isolation gates are **NOT RUN**. "
            "The report does not substitute CPU execution or call this a CUDA pass."
        )
    else:
        outcome = "FAIL"
        device_summary = f"Device execution failed: `{audit.get('device_error', audit['process_stderr'])}`"

    capabilities = audit["capabilities"]
    layout = audit["layout"]
    cpu = audit["cpu_mirror_checks"]
    return f"""# Bonesaw CUDA mirror state input · r91

## Outcome

**{outcome}.** R91 is the first real device implementation boundary after CPU concept admission. `bonesaw-cuda` now embeds a fixed PTX state-input kernel, dynamically loads the CUDA Driver API, owns a context/module/five fixed device buffers, validates finite `f32` state on-device, zeros malformed and padded agents, and exposes a runtime fingerprint. The execution method has no CPU fallback.

This is deliberately a **StateInput-only** claim. FK, Jacobians, dynamics, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unimplemented on device.

## Compiled boundary

{markdown_table(['signal', 'value'], [
    ['runtime probe', runtime_kind],
    ['driver version', runtime.get('driver_version')],
    ['visible devices', len(runtime['devices'])],
    ['kernel manifest bits', audit['kernel_manifest_bits']],
    ['kernel SHA-256', audit['kernel_sha256']],
    ['ptxas syntax validation', ptx_validation['status']],
    ['profile capability', f"state input={capabilities['cuda_mirror_state_input_f32']} · full mirror={capabilities['cuda_mirror_f32']} · Graph={capabilities['cuda_graph_executor']}"],
    ['layout', f"{layout['coordinate_count']} coordinates · {layout['agent_capacity']} active capacity · stride {layout['agent_stride']}"],
    ['hidden CPU fallback', audit['hidden_cpu_fallback']],
])}

The PTX assigns one thread to one complete agent, uses no atomics, performs no cross-agent reduction, and writes deterministic zeroes for inactive or invalid lanes. Driver symbols are resolved only during executor construction, so a CPU-only process can load the crate and receive typed unavailability.

## CPU mirror witness

{markdown_table(['gate', 'result'], [
    ['active agents typed OK', cpu['active_agents_ok']],
    ['padding inactive and zero', cpu['padding_inactive_and_zero']],
    ['allocation-free 100-call Rust unit sentinel', True],
    ['bitwise repeated CPU output', True],
])}

{device_summary}

## Admission boundary

R91 does not promote `CudaMirrorF32` as a backend. It promotes only the compiled StateInput stage and the runtime probe. A CUDA-capable CI host must still JIT the retained kernel and pass D1 repeated bytes, D2 CPU-mirror bytes, malformed-agent isolation, padding, permutation/chunking, direct-versus-Graph, allocation/memory, error-injection, and timing gates. The current host reports `{runtime_kind}`; therefore all device gates remain unavailable rather than emulated.
"""


def main() -> None:
    args = parse_args()
    binary = pathlib.Path(args.binary)
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = run_audit(binary, model)
    ptx = pathlib.Path("crates/bonesaw-cuda/src/kernels/state_input_v1.ptx")
    metrics = {
        "schema": 1,
        "revision": "cuda-state-input-r91",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "artifacts": {
            str(binary): sha256(binary),
            str(model): sha256(model),
            str(ptx): sha256(ptx),
        },
        "ptx_validation": validate_ptx(ptx),
        "audit": audit,
    }
    (output / "cuda-state-input-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    markdown = report_markdown(metrics)
    (output / "CUDA_STATE_INPUT_AUDIT.md").write_text(markdown)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(markdown, title="Bonesaw CUDA mirror state input · r91")
    )
    print(json.dumps(metrics, indent=2))
    if (
        audit["status"] == "failed"
        or audit["process_exit_code"] != 0
        or metrics["ptx_validation"]["status"] == "fail"
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
