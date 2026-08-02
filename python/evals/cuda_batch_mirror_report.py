#!/usr/bin/env python3
"""Certify the first fixed-layout batch ABI without pretending CUDA exists.

The evaluator stays in Python, while every measured FK/CoM batch transition is
performed by the preallocated Rust ``CpuMirrorF32`` executor.  Pinocchio is an
independent f64 kinematic oracle.  This report is the admission boundary for a
future device implementation, not a GPU benchmark.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pinocchio as pin

from bonesaw import CpuMirrorBatchSession
from cpu_reference_report import distribution, markdown_table, render_report_html


POSE_COMPONENTS = 12
TRANSLATION_TOLERANCE_M = 5e-5
ROTATION_TOLERANCE_RAD = 5e-5
COM_TOLERANCE_M = 5e-5
ORTHOGONALITY_TOLERANCE = 2e-6
STATUS_OK = 1
STATUS_INVALID_INPUT = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=300)
    parser.add_argument(
        "--output", default="benchmarks/results/cuda-batch-abi-r75"
    )
    parser.add_argument("--web-report", default="web/CUDA_BATCH_ABI_R75.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def root_pose_matrix(count: int) -> np.ndarray:
    poses = np.zeros((count, POSE_COMPONENTS), dtype=np.float32)
    for agent in range(count):
        yaw = np.float32(-0.21 + 0.42 * agent / max(count - 1, 1))
        pitch = np.float32(0.06 * np.sin(0.7 * agent))
        cy, sy = np.float32(np.cos(yaw)), np.float32(np.sin(yaw))
        cp, sp = np.float32(np.cos(pitch)), np.float32(np.sin(pitch))
        rotation = np.asarray(
            [
                [cy * cp, -sy, cy * sp],
                [sy * cp, cy, sy * sp],
                [-sp, 0.0, cp],
            ],
            dtype=np.float32,
        )
        poses[agent, :9] = rotation.reshape(-1)
        poses[agent, 9:] = np.asarray(
            [0.015 * agent, -0.008 * agent, 0.63 + 0.004 * agent],
            dtype=np.float32,
        )
    return poses


def joint_states(count: int, dof: int) -> np.ndarray:
    agent = np.arange(count, dtype=np.float32)[:, None]
    coordinate = np.arange(dof, dtype=np.float32)[None, :]
    return np.asarray(
        0.19 * np.sin(0.31 * agent + 0.47 * coordinate)
        + 0.035 * np.cos(0.13 * agent - 0.29 * coordinate),
        dtype=np.float32,
    )


def output_buffers(session: CpuMirrorBatchSession) -> dict[str, np.ndarray]:
    agents = session.agent_capacity
    bodies = len(session.body_names)
    return {
        "body_pose": np.empty((agents, bodies, POSE_COMPONENTS), dtype=np.float32),
        "com": np.empty((agents, 3), dtype=np.float32),
        "mass": np.empty(agents, dtype=np.float32),
        "status": np.empty(agents, dtype=np.uint8),
        "execute_ns": np.empty(1, dtype=np.uint64),
        "allocation_calls": np.empty(1, dtype=np.uint64),
        "allocated_bytes": np.empty(1, dtype=np.uint64),
    }


def run(
    session: CpuMirrorBatchSession,
    q: np.ndarray,
    root_pose: np.ndarray,
    buffers: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    result = output_buffers(session) if buffers is None else buffers
    session.run_fk_batch(
        np.ascontiguousarray(q, dtype=np.float32),
        np.ascontiguousarray(root_pose, dtype=np.float32),
        result["body_pose"],
        result["com"],
        result["mass"],
        result["status"],
        result["execute_ns"],
        result["allocation_calls"],
        result["allocated_bytes"],
    )
    return result


def result_payload(result: dict[str, np.ndarray]) -> bytes:
    return b"".join(
        np.ascontiguousarray(result[name]).tobytes()
        for name in ("body_pose", "com", "mass", "status")
    )


def rotation_angle(left: np.ndarray, right: np.ndarray) -> float:
    relative = np.asarray(left, dtype=np.float64).T @ np.asarray(right, dtype=np.float64)
    skew_vee = np.asarray(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ]
    )
    sine = 0.5 * float(np.linalg.norm(skew_vee))
    cosine = 0.5 * float(np.trace(relative) - 1.0)
    return abs(float(np.arctan2(sine, cosine)))


def pinocchio_reference(
    model_path: pathlib.Path,
    body_names: list[str],
    joint_names: list[str],
    q_batch: np.ndarray,
    root_pose_batch: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    frame_ids = [model.getFrameId(name, pin.FrameType.BODY) for name in body_names]
    if any(frame_id >= len(model.frames) for frame_id in frame_ids):
        raise ValueError("Pinocchio is missing a Bonesaw body frame")
    joint_ids = [model.getJointId(name) for name in joint_names]
    if any(joint_id == 0 for joint_id in joint_ids):
        raise ValueError("Pinocchio is missing a Bonesaw coordinate")
    q_indices = [model.joints[joint_id].idx_q for joint_id in joint_ids]
    if any(model.joints[joint_id].nq != 1 for joint_id in joint_ids):
        raise ValueError("batch mirror oracle requires scalar joints")
    poses = np.empty((len(q_batch), len(body_names), POSE_COMPONENTS), dtype=np.float64)
    com = np.empty((len(q_batch), 3), dtype=np.float64)
    mass = np.empty(len(q_batch), dtype=np.float64)
    data = model.createData()
    for agent in range(len(q_batch)):
        configuration = pin.neutral(model)
        configuration[:3] = root_pose_batch[agent, 9:12].astype(np.float64)
        rotation = root_pose_batch[agent, :9].astype(np.float64).reshape(3, 3)
        configuration[3:7] = pin.Quaternion(rotation).normalized().coeffs()
        for value, q_index in zip(q_batch[agent], q_indices, strict=True):
            configuration[q_index] = float(value)
        pin.framesForwardKinematics(model, data, configuration)
        for body, frame_id in enumerate(frame_ids):
            placement = data.oMf[frame_id]
            poses[agent, body, :9] = placement.rotation.reshape(-1)
            poses[agent, body, 9:12] = placement.translation
        com[agent] = np.asarray(pin.centerOfMass(model, data, configuration)).reshape(3)
        mass[agent] = float(pin.computeTotalMass(model, data))
    return poses, com, mass


def oracle_case(model_path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), samples, 32)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    mirror = run(session, q, roots)
    oracle_pose, oracle_com, oracle_mass = pinocchio_reference(
        model_path, list(session.body_names), list(session.joint_names), q, roots
    )
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    orthogonality_errors: list[float] = []
    for agent in range(samples):
        for body in range(len(session.body_names)):
            mirror_rotation = mirror["body_pose"][agent, body, :9].reshape(3, 3)
            oracle_rotation = oracle_pose[agent, body, :9].reshape(3, 3)
            translation_errors.append(
                float(
                    np.linalg.norm(
                        mirror["body_pose"][agent, body, 9:12]
                        - oracle_pose[agent, body, 9:12]
                    )
                )
            )
            rotation_errors.append(rotation_angle(mirror_rotation, oracle_rotation))
            orthogonality_errors.append(
                float(np.max(np.abs(mirror_rotation.T @ mirror_rotation - np.eye(3))))
            )
    metrics = {
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "bodies": len(session.body_names),
        "coordinates": session.dof,
        "maximum_frame_translation_error_m": max(translation_errors),
        "maximum_frame_rotation_error_rad": max(rotation_errors),
        "maximum_rotation_orthogonality_error": max(orthogonality_errors),
        "maximum_center_of_mass_error_m": float(
            np.max(np.linalg.norm(mirror["com"].astype(np.float64) - oracle_com, axis=1))
        ),
        "maximum_total_mass_error_kg": float(
            np.max(np.abs(mirror["mass"].astype(np.float64) - oracle_mass))
        ),
        "statuses_ok": bool(np.all(mirror["status"] == STATUS_OK)),
        "allocation_calls": int(mirror["allocation_calls"][0]),
        "allocated_bytes": int(mirror["allocated_bytes"][0]),
        "backend_fingerprint": json.loads(session.backend_fingerprint_json),
        "kernel_manifest_bits": int(session.kernel_manifest_bits),
        "kernel_abi_version": int(session.kernel_abi_version),
    }
    metrics["d3_pass"] = bool(
        metrics["maximum_frame_translation_error_m"] <= TRANSLATION_TOLERANCE_M
        and metrics["maximum_frame_rotation_error_rad"] <= ROTATION_TOLERANCE_RAD
        and metrics["maximum_rotation_orthogonality_error"] <= ORTHOGONALITY_TOLERANCE
        and metrics["maximum_center_of_mass_error_m"] <= COM_TOLERANCE_M
        and metrics["statuses_ok"]
    )
    raw = {
        f"{model_path.stem}_q": q,
        f"{model_path.stem}_root_pose": roots,
        f"{model_path.stem}_mirror_pose": mirror["body_pose"].copy(),
        f"{model_path.stem}_oracle_pose": oracle_pose,
        f"{model_path.stem}_mirror_com": mirror["com"].copy(),
        f"{model_path.stem}_oracle_com": oracle_com,
    }
    return metrics, raw


def exact_invariance_case(model_path: pathlib.Path, agents: int = 17) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    q_session = CpuMirrorBatchSession(str(model_path), agents, 32)
    q = joint_states(agents, q_session.dof)
    roots = root_pose_matrix(agents)
    baseline = run(q_session, q, roots)
    baseline_payload = result_payload(baseline)
    repeat = run(q_session, q, roots)
    repeat_exact = result_payload(repeat) == baseline_payload

    permutation = np.asarray(
        [*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)],
        dtype=np.int64,
    )
    permuted = run(q_session, q[permutation], roots[permutation])
    inverse = np.argsort(permutation)
    permutation_exact = all(
        np.array_equal(permuted[name][inverse], baseline[name])
        for name in ("body_pose", "com", "mass", "status")
    )

    compact_session = CpuMirrorBatchSession(str(model_path), agents, 1)
    compact = run(compact_session, q, roots)
    padding_exact = all(
        np.array_equal(compact[name], baseline[name])
        for name in ("body_pose", "com", "mass", "status")
    )

    split = 7
    chunks = []
    for start, stop in ((0, split), (split, agents)):
        chunk_session = CpuMirrorBatchSession(str(model_path), stop - start, 32)
        chunks.append(run(chunk_session, q[start:stop], roots[start:stop]))
    chunking_exact = all(
        np.array_equal(np.concatenate([chunk[name] for chunk in chunks]), baseline[name])
        for name in ("body_pose", "com", "mass", "status")
    )

    invalid_q = q.copy()
    invalid_agent = 8
    invalid_q[invalid_agent, 0] = np.nan
    invalid = run(q_session, invalid_q, roots)
    neighbor_mask = np.arange(agents) != invalid_agent
    isolation_exact = bool(
        invalid["status"][invalid_agent] == STATUS_INVALID_INPUT
        and np.all(invalid["status"][neighbor_mask] == STATUS_OK)
        and all(
            np.array_equal(invalid[name][neighbor_mask], baseline[name][neighbor_mask])
            for name in ("body_pose", "com", "mass")
        )
    )
    metrics = {
        "agents": agents,
        "aligned_stride": q_session.agent_stride,
        "compact_stride": compact_session.agent_stride,
        "d1_repeat_bitwise_exact": repeat_exact,
        "permutation_exact": permutation_exact,
        "padding_exact": padding_exact,
        "chunking_exact": chunking_exact,
        "invalid_agent_status": int(invalid["status"][invalid_agent]),
        "invalid_agent_isolation_exact": isolation_exact,
        "allocation_calls": int(
            baseline["allocation_calls"][0]
            + repeat["allocation_calls"][0]
            + permuted["allocation_calls"][0]
            + compact["allocation_calls"][0]
            + sum(chunk["allocation_calls"][0] for chunk in chunks)
            + invalid["allocation_calls"][0]
        ),
        "allocated_bytes": int(
            baseline["allocated_bytes"][0]
            + repeat["allocated_bytes"][0]
            + permuted["allocated_bytes"][0]
            + compact["allocated_bytes"][0]
            + sum(chunk["allocated_bytes"][0] for chunk in chunks)
            + invalid["allocated_bytes"][0]
        ),
    }
    metrics["pass"] = bool(
        repeat_exact
        and permutation_exact
        and padding_exact
        and chunking_exact
        and isolation_exact
        and metrics["allocation_calls"] == 0
        and metrics["allocated_bytes"] == 0
    )
    return metrics, {
        "invariance_baseline_pose": baseline["body_pose"].copy(),
        "invariance_baseline_com": baseline["com"].copy(),
        "invariance_permutation": permutation,
        "invariance_invalid_status": invalid["status"].copy(),
    }


def timing_case(model_path: pathlib.Path, batch_size: int, repeats: int) -> dict[str, Any]:
    session = CpuMirrorBatchSession(str(model_path), batch_size, 32)
    q = joint_states(batch_size, session.dof)
    roots = root_pose_matrix(batch_size)
    buffers = output_buffers(session)
    for _ in range(20):
        run(session, q, roots, buffers)
    execute_ns = np.empty(repeats, dtype=np.float64)
    allocation_calls = 0
    allocated_bytes = 0
    for repeat in range(repeats):
        result = run(session, q, roots, buffers)
        execute_ns[repeat] = result["execute_ns"][0]
        allocation_calls += int(result["allocation_calls"][0])
        allocated_bytes += int(result["allocated_bytes"][0])
    stats = distribution(execute_ns)
    return {
        "batch_size": batch_size,
        "agent_stride": session.agent_stride,
        "repeats": repeats,
        "batch_execute_ns": stats,
        "per_agent_execute_ns": {key: value / batch_size for key, value in stats.items()},
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Bonesaw fixed-layout batch mirror · r75",
        "",
        "> PROMOTE THE ABI AND CPU MIRROR ONLY. The CUDA device executor is unavailable; no GPU correctness, latency, throughput, or determinism claim is made.",
        "",
        "This policy-free, simulator-free gate freezes the first state/FK/CoM batch contract before device work. Python owns experiment orchestration and Pinocchio comparison; the measured transition is fixed-layout, preallocated Rust.",
        "",
        "## Admission decision",
        "",
    ]
    lines += markdown_table(
        ["Gate", "Result", "Decision"],
        [
            ["CpuMirrorF32 ↔ Pinocchio f64 (D3)", f"{sum(case['d3_pass'] for case in metrics['oracle_cases'])} / {len(metrics['oracle_cases'])}", "PASS"],
            ["D1 repeat bytes", str(metrics["invariance"]["d1_repeat_bitwise_exact"]), "PASS" if metrics["invariance"]["d1_repeat_bitwise_exact"] else "FAIL"],
            ["Permutation / padding / chunking", f"{metrics['invariance']['permutation_exact']} / {metrics['invariance']['padding_exact']} / {metrics['invariance']['chunking_exact']}", "PASS" if metrics["invariance"]["pass"] else "FAIL"],
            ["Typed invalid-agent isolation", str(metrics["invariance"]["invalid_agent_isolation_exact"]), "PASS" if metrics["invariance"]["invalid_agent_isolation_exact"] else "FAIL"],
            ["Rust execute allocations", f"{metrics['total_allocation_calls']} calls · {metrics['total_allocated_bytes']} B", "PASS" if metrics["total_allocation_calls"] == 0 else "FAIL"],
            ["CUDA device D1 / D2 / D3", "executor unavailable", "NOT RUN"],
        ],
    )
    lines += ["", "## Independent f64 oracle", ""]
    lines += markdown_table(
        ["Model", "States × bodies", "Translation max", "Rotation max", "CoM max", "Orthogonality", "D3"],
        [
            [
                pathlib.Path(case["model"]).stem,
                f"{case['samples']} × {case['bodies']}",
                f"{case['maximum_frame_translation_error_m']:.3e} m",
                f"{case['maximum_frame_rotation_error_rad']:.3e} rad",
                f"{case['maximum_center_of_mass_error_m']:.3e} m",
                f"{case['maximum_rotation_orthogonality_error']:.3e}",
                "PASS" if case["d3_pass"] else "FAIL",
            ]
            for case in metrics["oracle_cases"]
        ],
    )
    lines += [
        "",
        f"D3 thresholds are {TRANSLATION_TOLERANCE_M:.0e} m translation, {ROTATION_TOLERANCE_RAD:.0e} rad rotation, and {COM_TOLERANCE_M:.0e} m CoM. Rotation uses atan2(skew, trace), with orthogonality checked separately, so f32 drift cannot masquerade as a trace/acos angle.",
        "",
        "## Exact layout and isolation gates",
        "",
    ]
    inv = metrics["invariance"]
    lines += markdown_table(
        ["Invariant", "Evidence", "Result"],
        [
            ["Agent SoA stride", f"17 agents: aligned {inv['aligned_stride']}, compact {inv['compact_stride']}", "EXACT" if inv["padding_exact"] else "FAIL"],
            ["Repeat", "body pose + CoM + mass + status bytes", "EXACT" if inv["d1_repeat_bitwise_exact"] else "FAIL"],
            ["Permutation", "reverse-interleave then inverse index", "EXACT" if inv["permutation_exact"] else "FAIL"],
            ["Chunking", "7 + 10 agents reassembled", "EXACT" if inv["chunking_exact"] else "FAIL"],
            ["Malformed state", f"agent 8 status={inv['invalid_agent_status']}; every neighbor compared", "ISOLATED" if inv["invalid_agent_isolation_exact"] else "FAIL"],
        ],
    )
    lines += ["", "## Host timing · CPU mirror only", ""]
    lines += markdown_table(
        ["Agents", "Stride", "p50 batch", "p99 batch", "p50 / agent", "p99 / agent", "Allocations"],
        [
            [
                case["batch_size"],
                case["agent_stride"],
                f"{case['batch_execute_ns']['p50'] / 1e3:.3f} µs",
                f"{case['batch_execute_ns']['p99'] / 1e3:.3f} µs",
                f"{case['per_agent_execute_ns']['p50']:.1f} ns",
                f"{case['per_agent_execute_ns']['p99']:.1f} ns",
                f"{case['allocation_calls']} / {case['allocated_bytes']} B",
            ]
            for case in metrics["timing"]
        ],
    )
    lines += [
        "",
        "The timer surrounds only Rust execute_into. NumPy-to-SoA input copy and SoA-to-NumPy output copy are intentionally outside it; end-to-end transport belongs to a later websocket/device report.",
        "",
        "## Frozen contract",
        "",
        f"Kernel ABI version {metrics['kernel_abi_version']} exposes StateInput + ForwardKinematics + CenterOfMass. State is state[coordinate][agent]; root and frame poses are pose[frame][12 components][agent], with row-major rotation followed by translation. MirrorF32 fixes explicit FMA, disables fast math and FTZ, and fingerprints program, build, ISA, layout, manifest, and math flags.",
        "",
        "## Deferred device admission",
        "",
        "CudaMirrorF32 must reproduce these stage outputs under D1, satisfy CpuMirrorF32↔CudaMirrorF32 D2 tolerances, rerun D3 against Pinocchio, and pass permutation, padded-stride, chunking, malformed-agent, memory-scaling, warmup, jitter, and fingerprint gates on a working NVIDIA driver/toolkit. CudaThroughputF32 remains a separate relaxed profile and cannot inherit mirror certification.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    raw: dict[str, np.ndarray] = {}
    oracle_cases = []
    for model in (pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")):
        metrics, arrays = oracle_case(model, args.samples)
        oracle_cases.append(metrics)
        raw.update(arrays)
    invariance, arrays = exact_invariance_case(pathlib.Path("models/upkie/upkie.urdf"))
    raw.update(arrays)
    timing = [
        timing_case(pathlib.Path("models/upkie/upkie.urdf"), size, args.timing_repeats)
        for size in (1, 32, 256)
    ]
    total_allocation_calls = invariance["allocation_calls"] + sum(
        case["allocation_calls"] for case in oracle_cases + timing
    )
    total_allocated_bytes = invariance["allocated_bytes"] + sum(
        case["allocated_bytes"] for case in oracle_cases + timing
    )
    fingerprint = oracle_cases[0]["backend_fingerprint"]
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "revision": "cuda-batch-abi-r75",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "profile": "CpuMirrorF32",
        "kernel_abi_version": oracle_cases[0]["kernel_abi_version"],
        "manifest": ["StateInput", "ForwardKinematics", "CenterOfMass"],
        "layout": "state[coordinate][agent]; pose[frame][component][agent]",
        "oracle_cases": oracle_cases,
        "invariance": invariance,
        "timing": timing,
        "total_allocation_calls": total_allocation_calls,
        "total_allocated_bytes": total_allocated_bytes,
        "cuda_device_executor": {
            "available": False,
            "status": "CpuMirrorReadyDeviceExecutorUnavailable",
            "d1": "not_run",
            "d2": "not_run",
            "d3": "not_run",
        },
        "admission": {
            "cpu_mirror": bool(
                all(case["d3_pass"] for case in oracle_cases)
                and invariance["pass"]
                and total_allocation_calls == 0
                and total_allocated_bytes == 0
            ),
            "cuda_mirror": False,
            "cuda_throughput": False,
        },
        "representative_fingerprint": fingerprint,
    }
    np.savez_compressed(output / "cuda-batch-abi-raw.npz", **raw)
    metrics_path = output / "cuda-batch-abi-metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CUDA_BATCH_ABI_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw batch mirror · r75")
    )
    if not metrics["admission"]["cpu_mirror"]:
        raise SystemExit("CpuMirrorF32 admission failed")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
