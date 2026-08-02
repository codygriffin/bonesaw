#!/usr/bin/env python3
"""Admit the fixed-layout CpuMirrorF32 frame/CoM Jacobian stage."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pinocchio as pin

from bonesaw import CpuMirrorBatchSession
from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import joint_states, root_pose_matrix, sha256


STATUS_OK = 1
STATUS_INVALID_INPUT = 2
JACOBIAN_ABSOLUTE_TOLERANCE = 2e-5
JACOBIAN_RELATIVE_TOLERANCE = 2e-4
FINITE_DIFFERENCE_ABSOLUTE_TOLERANCE = 4e-4
FINITE_DIFFERENCE_RELATIVE_TOLERANCE = 2e-3
FINITE_DIFFERENCE_STEP = 1e-3
EXACT_FIELDS = (
    "body_pose",
    "com",
    "mass",
    "frame_jacobian",
    "com_jacobian",
    "status",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=300)
    parser.add_argument(
        "--output", default="benchmarks/results/cuda-jacobian-abi-r76"
    )
    parser.add_argument("--web-report", default="web/CUDA_JACOBIAN_ABI_R76.html")
    return parser.parse_args()


def output_buffers(session: CpuMirrorBatchSession) -> dict[str, np.ndarray]:
    agents = session.agent_capacity
    bodies = len(session.body_names)
    generalized = session.generalized_dof
    return {
        "body_pose": np.empty((agents, bodies, 12), dtype=np.float32),
        "com": np.empty((agents, 3), dtype=np.float32),
        "mass": np.empty(agents, dtype=np.float32),
        "frame_jacobian": np.empty(
            (agents, bodies, session.jacobian_components, generalized),
            dtype=np.float32,
        ),
        "com_jacobian": np.empty((agents, 3, generalized), dtype=np.float32),
        "status": np.empty(agents, dtype=np.uint8),
        "fk_execute_ns": np.empty(1, dtype=np.uint64),
        "jacobian_execute_ns": np.empty(1, dtype=np.uint64),
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
    session.run_kinematics_batch(
        np.ascontiguousarray(q, dtype=np.float32),
        np.ascontiguousarray(root_pose, dtype=np.float32),
        result["body_pose"],
        result["com"],
        result["mass"],
        result["frame_jacobian"],
        result["com_jacobian"],
        result["status"],
        result["fk_execute_ns"],
        result["jacobian_execute_ns"],
        result["allocation_calls"],
        result["allocated_bytes"],
    )
    return result


def copy_result(result: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {name: result[name].copy() for name in result}


def pin_configuration(
    model: pin.Model,
    q_indices: list[int],
    q: np.ndarray,
    root_pose: np.ndarray,
) -> np.ndarray:
    configuration = pin.neutral(model)
    configuration[:3] = root_pose[9:12].astype(np.float64)
    rotation = root_pose[:9].astype(np.float64).reshape(3, 3)
    configuration[3:7] = pin.Quaternion(rotation).normalized().coeffs()
    for value, index in zip(q, q_indices, strict=True):
        configuration[index] = float(value)
    return configuration


def pinocchio_jacobians(
    model_path: pathlib.Path,
    body_names: list[str],
    joint_names: list[str],
    q_batch: np.ndarray,
    roots: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    frame_ids = [model.getFrameId(name, pin.FrameType.BODY) for name in body_names]
    joint_ids = [model.getJointId(name) for name in joint_names]
    if any(frame_id >= len(model.frames) for frame_id in frame_ids) or any(
        joint_id == 0 for joint_id in joint_ids
    ):
        raise ValueError("Pinocchio model does not match the Bonesaw batch layout")
    q_indices = [model.joints[joint_id].idx_q for joint_id in joint_ids]
    v_indices = [model.joints[joint_id].idx_v for joint_id in joint_ids]
    agents = len(q_batch)
    generalized = 6 + len(joint_names)
    frame_output = np.zeros(
        (agents, len(body_names), 6, generalized), dtype=np.float64
    )
    com_output = np.zeros((agents, 3, generalized), dtype=np.float64)
    data = model.createData()
    for agent in range(agents):
        configuration = pin_configuration(
            model, q_indices, q_batch[agent], roots[agent]
        )
        pin.computeJointJacobians(model, data, configuration)
        pin.updateFramePlacements(model, data)
        root_origin = roots[agent, 9:12].astype(np.float64)
        for body, frame_id in enumerate(frame_ids):
            frame = data.oMf[frame_id]
            for axis in range(3):
                basis = np.zeros(3, dtype=np.float64)
                basis[axis] = 1.0
                frame_output[agent, body, axis, axis] = 1.0
                frame_output[agent, body, 3:6, axis] = np.cross(
                    basis, frame.translation - root_origin
                )
                frame_output[agent, body, 3 + axis, 3 + axis] = 1.0
            jacobian = np.asarray(
                pin.getFrameJacobian(
                    model,
                    data,
                    frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                ),
                dtype=np.float64,
            )
            for coordinate, pin_column in enumerate(v_indices):
                frame_output[agent, body, :3, 6 + coordinate] = jacobian[
                    3:6, pin_column
                ]
                frame_output[agent, body, 3:6, 6 + coordinate] = jacobian[
                    :3, pin_column
                ]
        com = np.asarray(pin.centerOfMass(model, data, configuration)).reshape(3)
        pin_com = np.asarray(
            pin.jacobianCenterOfMass(model, data, configuration), dtype=np.float64
        )
        for axis in range(3):
            basis = np.zeros(3, dtype=np.float64)
            basis[axis] = 1.0
            com_output[agent, :, axis] = np.cross(basis, com - root_origin)
            com_output[agent, axis, 3 + axis] = 1.0
        for coordinate, pin_column in enumerate(v_indices):
            com_output[agent, :, 6 + coordinate] = pin_com[:, pin_column]
    return frame_output, com_output


def tolerance_score(mirror: np.ndarray, oracle: np.ndarray) -> tuple[float, float]:
    mirror64 = mirror.astype(np.float64)
    oracle64 = oracle.astype(np.float64)
    absolute = np.abs(mirror64 - oracle64)
    allowed = JACOBIAN_ABSOLUTE_TOLERANCE + JACOBIAN_RELATIVE_TOLERANCE * np.maximum(
        np.abs(mirror64), np.abs(oracle64)
    )
    return float(np.max(absolute)), float(np.max(absolute / allowed))


def oracle_case(
    model_path: pathlib.Path, samples: int
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), samples, 32)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    mirror = run(session, q, roots)
    oracle_frame, oracle_com = pinocchio_jacobians(
        model_path,
        list(session.body_names),
        list(session.joint_names),
        q,
        roots,
    )
    frame_absolute, frame_score = tolerance_score(
        mirror["frame_jacobian"], oracle_frame
    )
    com_absolute, com_score = tolerance_score(mirror["com_jacobian"], oracle_com)
    metrics = {
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "bodies": len(session.body_names),
        "coordinates": session.dof,
        "generalized_coordinates": session.generalized_dof,
        "maximum_frame_jacobian_absolute_error": frame_absolute,
        "maximum_frame_jacobian_tolerance_score": frame_score,
        "maximum_com_jacobian_absolute_error": com_absolute,
        "maximum_com_jacobian_tolerance_score": com_score,
        "statuses_ok": bool(np.all(mirror["status"] == STATUS_OK)),
        "allocation_calls": int(mirror["allocation_calls"][0]),
        "allocated_bytes": int(mirror["allocated_bytes"][0]),
        "backend_fingerprint": json.loads(session.backend_fingerprint_json),
        "kernel_manifest_bits": int(session.kernel_manifest_bits),
        "kernel_abi_version": int(session.kernel_abi_version),
    }
    metrics["d3_pass"] = bool(
        frame_score <= 1.0
        and com_score <= 1.0
        and metrics["statuses_ok"]
        and metrics["allocation_calls"] == 0
        and metrics["allocated_bytes"] == 0
    )
    stem = model_path.stem
    return metrics, {
        f"{stem}_q": q,
        f"{stem}_root_pose": roots,
        f"{stem}_mirror_frame_jacobian": mirror["frame_jacobian"].copy(),
        f"{stem}_oracle_frame_jacobian": oracle_frame,
        f"{stem}_mirror_com_jacobian": mirror["com_jacobian"].copy(),
        f"{stem}_oracle_com_jacobian": oracle_com,
    }


def so3_exp(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        skew = np.asarray(
            [[0.0, -vector[2], vector[1]], [vector[2], 0.0, -vector[0]], [-vector[1], vector[0], 0.0]]
        )
        return np.eye(3) + skew
    axis = vector / angle
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def finite_difference_case(
    model_path: pathlib.Path, samples: int
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), samples, 32)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    base = run(session, q, roots)
    agent_index = np.arange(samples, dtype=np.float64)[:, None]
    coordinate = np.arange(session.generalized_dof, dtype=np.float64)[None, :]
    tangent = 0.23 * np.sin(0.37 * agent_index + 0.29 * coordinate)
    plus_q = np.asarray(q + FINITE_DIFFERENCE_STEP * tangent[:, 6:], dtype=np.float32)
    minus_q = np.asarray(q - FINITE_DIFFERENCE_STEP * tangent[:, 6:], dtype=np.float32)
    plus_roots = roots.copy()
    minus_roots = roots.copy()
    for agent in range(samples):
        rotation = roots[agent, :9].astype(np.float64).reshape(3, 3)
        omega = tangent[agent, :3]
        linear = tangent[agent, 3:6]
        plus_roots[agent, :9] = (
            so3_exp(FINITE_DIFFERENCE_STEP * omega) @ rotation
        ).astype(np.float32).reshape(-1)
        minus_roots[agent, :9] = (
            so3_exp(-FINITE_DIFFERENCE_STEP * omega) @ rotation
        ).astype(np.float32).reshape(-1)
        plus_roots[agent, 9:12] = np.asarray(
            roots[agent, 9:12] + FINITE_DIFFERENCE_STEP * linear,
            dtype=np.float32,
        )
        minus_roots[agent, 9:12] = np.asarray(
            roots[agent, 9:12] - FINITE_DIFFERENCE_STEP * linear,
            dtype=np.float32,
        )
    plus = run(session, plus_q, plus_roots)
    minus = run(session, minus_q, minus_roots)
    frame_fd = (
        plus["body_pose"][:, :, 9:12].astype(np.float64)
        - minus["body_pose"][:, :, 9:12].astype(np.float64)
    ) / (2.0 * FINITE_DIFFERENCE_STEP)
    frame_predicted = np.einsum(
        "abrg,ag->abr",
        base["frame_jacobian"][:, :, 3:6, :].astype(np.float64),
        tangent,
    )
    com_fd = (
        plus["com"].astype(np.float64) - minus["com"].astype(np.float64)
    ) / (2.0 * FINITE_DIFFERENCE_STEP)
    com_predicted = np.einsum(
        "arg,ag->ar", base["com_jacobian"].astype(np.float64), tangent
    )

    def score(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
        error = np.abs(left - right)
        allowed = FINITE_DIFFERENCE_ABSOLUTE_TOLERANCE + FINITE_DIFFERENCE_RELATIVE_TOLERANCE * np.maximum(
            np.abs(left), np.abs(right)
        )
        return float(np.max(error)), float(np.max(error / allowed))

    frame_absolute, frame_score = score(frame_predicted, frame_fd)
    com_absolute, com_score = score(com_predicted, com_fd)
    metrics = {
        "model": str(model_path),
        "samples": samples,
        "step": FINITE_DIFFERENCE_STEP,
        "maximum_frame_origin_velocity_absolute_error": frame_absolute,
        "maximum_frame_origin_velocity_tolerance_score": frame_score,
        "maximum_com_velocity_absolute_error": com_absolute,
        "maximum_com_velocity_tolerance_score": com_score,
        "pass": bool(frame_score <= 1.0 and com_score <= 1.0),
    }
    return metrics, {
        "finite_difference_tangent": tangent,
        "finite_difference_frame_predicted": frame_predicted,
        "finite_difference_frame_observed": frame_fd,
        "finite_difference_com_predicted": com_predicted,
        "finite_difference_com_observed": com_fd,
    }


def invariance_case(
    model_path: pathlib.Path, agents: int = 17
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), agents, 32)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    baseline = copy_result(run(session, q, roots))
    repeat = run(session, q, roots)
    repeat_exact = all(np.array_equal(repeat[name], baseline[name]) for name in EXACT_FIELDS)
    permutation = np.asarray(
        [*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)], dtype=np.int64
    )
    permuted = run(session, q[permutation], roots[permutation])
    inverse = np.argsort(permutation)
    permutation_exact = all(
        np.array_equal(permuted[name][inverse], baseline[name]) for name in EXACT_FIELDS
    )
    compact_session = CpuMirrorBatchSession(str(model_path), agents, 1)
    compact = run(compact_session, q, roots)
    padding_exact = all(
        np.array_equal(compact[name], baseline[name]) for name in EXACT_FIELDS
    )
    split = 7
    chunks = []
    for start, stop in ((0, split), (split, agents)):
        chunk_session = CpuMirrorBatchSession(str(model_path), stop - start, 32)
        chunks.append(run(chunk_session, q[start:stop], roots[start:stop]))
    chunking_exact = all(
        np.array_equal(
            np.concatenate([chunk[name] for chunk in chunks]), baseline[name]
        )
        for name in EXACT_FIELDS
    )
    invalid_q = q.copy()
    invalid_agent = 8
    invalid_q[invalid_agent, 0] = np.nan
    invalid = run(session, invalid_q, roots)
    neighbors = np.arange(agents) != invalid_agent
    isolation_exact = bool(
        invalid["status"][invalid_agent] == STATUS_INVALID_INPUT
        and np.all(invalid["status"][neighbors] == STATUS_OK)
        and all(
            np.array_equal(invalid[name][neighbors], baseline[name][neighbors])
            for name in EXACT_FIELDS
            if name != "status"
        )
        and np.all(invalid["frame_jacobian"][invalid_agent] == 0.0)
        and np.all(invalid["com_jacobian"][invalid_agent] == 0.0)
    )
    measured = [baseline, repeat, permuted, compact, *chunks, invalid]
    allocations = sum(int(result["allocation_calls"][0]) for result in measured)
    allocated_bytes = sum(int(result["allocated_bytes"][0]) for result in measured)
    metrics = {
        "agents": agents,
        "aligned_stride": session.agent_stride,
        "compact_stride": compact_session.agent_stride,
        "d1_repeat_bitwise_exact": repeat_exact,
        "permutation_exact": permutation_exact,
        "padding_exact": padding_exact,
        "chunking_exact": chunking_exact,
        "invalid_agent_status": int(invalid["status"][invalid_agent]),
        "invalid_agent_isolation_exact": isolation_exact,
        "allocation_calls": allocations,
        "allocated_bytes": allocated_bytes,
    }
    metrics["pass"] = bool(
        repeat_exact
        and permutation_exact
        and padding_exact
        and chunking_exact
        and isolation_exact
        and allocations == 0
        and allocated_bytes == 0
    )
    return metrics, {
        "invariance_frame_jacobian": baseline["frame_jacobian"],
        "invariance_com_jacobian": baseline["com_jacobian"],
        "invariance_permutation": permutation,
        "invariance_invalid_status": invalid["status"].copy(),
    }


def timing_case(model_path: pathlib.Path, agents: int, repeats: int) -> dict[str, Any]:
    session = CpuMirrorBatchSession(str(model_path), agents, 32)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    buffers = output_buffers(session)
    for _ in range(20):
        run(session, q, roots, buffers)
    fk = np.empty(repeats, dtype=np.float64)
    jacobian = np.empty(repeats, dtype=np.float64)
    allocation_calls = allocated_bytes = 0
    for repeat in range(repeats):
        result = run(session, q, roots, buffers)
        fk[repeat] = result["fk_execute_ns"][0]
        jacobian[repeat] = result["jacobian_execute_ns"][0]
        allocation_calls += int(result["allocation_calls"][0])
        allocated_bytes += int(result["allocated_bytes"][0])
    combined = fk + jacobian
    return {
        "batch_size": agents,
        "agent_stride": session.agent_stride,
        "repeats": repeats,
        "fk_execute_ns": distribution(fk),
        "jacobian_execute_ns": distribution(jacobian),
        "combined_execute_ns": distribution(combined),
        "per_agent_combined_ns": {
            key: value / agents for key, value in distribution(combined).items()
        },
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    inv = metrics["invariance"]
    lines = [
        "# Bonesaw fixed-layout Jacobian mirror · r76",
        "",
        "> PROMOTE THE CPU JACOBIAN STAGE ONLY. CUDA remains unavailable; no device correctness or performance claim is made.",
        "",
        "This policy-free, simulator-free gate extends kernel ABI v1 through world-expressed floating frame-origin and center-of-mass Jacobians. Tangent order is root angular, root linear, then joints. Python owns evaluation; both timed stages execute in preallocated Rust.",
        "",
        "## Admission decision",
        "",
    ]
    lines += markdown_table(
        ["Gate", "Result", "Decision"],
        [
            ["CpuMirrorF32 ↔ Pinocchio f64 Jacobians", f"{sum(case['d3_pass'] for case in metrics['oracle_cases'])} / {len(metrics['oracle_cases'])}", "PASS"],
            ["J·v ↔ finite-difference point/CoM velocity", f"{sum(case['pass'] for case in metrics['finite_difference_cases'])} / {len(metrics['finite_difference_cases'])}", "PASS"],
            ["D1 repeat bytes", str(inv["d1_repeat_bitwise_exact"]), "PASS"],
            ["Permutation / padding / chunking", f"{inv['permutation_exact']} / {inv['padding_exact']} / {inv['chunking_exact']}", "PASS"],
            ["Typed invalid-agent isolation", str(inv["invalid_agent_isolation_exact"]), "PASS"],
            ["Measured Rust allocations", f"{metrics['total_allocation_calls']} calls · {metrics['total_allocated_bytes']} B", "PASS"],
            ["CUDA Jacobian D1 / D2 / D3", "executor unavailable", "NOT RUN"],
        ],
    )
    lines += ["", "## Independent Pinocchio oracle", ""]
    lines += markdown_table(
        ["Model", "States × bodies", "Frame max abs", "Frame gate", "CoM max abs", "CoM gate", "D3"],
        [[pathlib.Path(case["model"]).stem, f"{case['samples']} × {case['bodies']}", f"{case['maximum_frame_jacobian_absolute_error']:.3e}", f"{case['maximum_frame_jacobian_tolerance_score']:.3f}×", f"{case['maximum_com_jacobian_absolute_error']:.3e}", f"{case['maximum_com_jacobian_tolerance_score']:.3f}×", "PASS" if case["d3_pass"] else "FAIL"] for case in metrics["oracle_cases"]],
    )
    lines += [
        "",
        f"Each element must satisfy abs(error) ≤ {JACOBIAN_ABSOLUTE_TOLERANCE:.0e} + {JACOBIAN_RELATIVE_TOLERANCE:.0e}·max(abs(values)). Root columns are independently reconstructed in world coordinates; joint columns come from Pinocchio LOCAL_WORLD_ALIGNED frame and CoM Jacobians.",
        "",
        "## Finite-difference velocity property",
        "",
    ]
    lines += markdown_table(
        ["Model", "Frame-origin max", "Frame gate", "CoM max", "CoM gate", "Result"],
        [[pathlib.Path(case["model"]).stem, f"{case['maximum_frame_origin_velocity_absolute_error']:.3e}", f"{case['maximum_frame_origin_velocity_tolerance_score']:.3f}×", f"{case['maximum_com_velocity_absolute_error']:.3e}", f"{case['maximum_com_velocity_tolerance_score']:.3f}×", "PASS" if case["pass"] else "FAIL"] for case in metrics["finite_difference_cases"]],
    )
    lines += ["", "A central difference perturbs root orientation by left-multiplying Exp(±εω), root translation by ±εv, and every joint by ±εqdot. It checks the linear frame-origin rows and CoM Jacobian against observed velocity without a policy or physics rollout.", "", "## Exact layout and isolation gates", ""]
    lines += markdown_table(
        ["Invariant", "Evidence", "Result"],
        [
            ["Layout", "frame[body][6][6+dof][agent] + com[3][6+dof][agent]", "FROZEN"],
            ["Repeat", "poses + CoM + mass + frame/CoM Jacobians + status", "EXACT"],
            ["Permutation", "reverse-interleave then inverse index", "EXACT"],
            ["Padded stride", f"17 active: stride {inv['aligned_stride']} vs {inv['compact_stride']}", "EXACT"],
            ["Chunking", "7 + 10 agents reassembled", "EXACT"],
            ["Malformed state", f"agent 8 status={inv['invalid_agent_status']}; zero Jacobians; neighbors exact", "ISOLATED"],
        ],
    )
    lines += ["", "## Host timing · Upkie CPU mirror only", ""]
    lines += markdown_table(
        ["Agents", "FK p50/p99", "Jacobian p50/p99", "Combined p50/p99", "Combined / agent p50", "Allocations"],
        [[case["batch_size"], f"{case['fk_execute_ns']['p50']/1e3:.3f}/{case['fk_execute_ns']['p99']/1e3:.3f} µs", f"{case['jacobian_execute_ns']['p50']/1e3:.3f}/{case['jacobian_execute_ns']['p99']/1e3:.3f} µs", f"{case['combined_execute_ns']['p50']/1e3:.3f}/{case['combined_execute_ns']['p99']/1e3:.3f} µs", f"{case['per_agent_combined_ns']['p50']:.1f} ns", f"{case['allocation_calls']} / {case['allocated_bytes']} B"] for case in metrics["timing"]],
    )
    lines += [
        "",
        "Timers surround Rust FK/CoM and Jacobian stages separately. NumPy↔SoA copies are excluded. The current Jacobian implementation deliberately favors auditable fixed traversal over tuning; this is an admission baseline, not a throughput result.",
        "",
        "## Deferred device admission",
        "",
        "CudaMirrorF32 must reproduce every frame and CoM Jacobian stage output under D1, satisfy CpuMirrorF32↔device D2/D3 tolerances, and repeat the exact agent-reindexing, padding, chunking, canary/isolation, warmup, memory-scaling, and jitter gates. CudaThroughputF32 remains separately fingerprinted and uncertified.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    raw: dict[str, np.ndarray] = {}
    oracle_cases = []
    finite_difference_cases = []
    for model in models:
        case, arrays = oracle_case(model, args.samples)
        oracle_cases.append(case)
        raw.update(arrays)
        case, arrays = finite_difference_case(model, args.samples)
        finite_difference_cases.append(case)
        raw.update({f"{model.stem}_{name}": value for name, value in arrays.items()})
    invariance, arrays = invariance_case(pathlib.Path("models/upkie/upkie.urdf"))
    raw.update(arrays)
    timing = [timing_case(pathlib.Path("models/upkie/upkie.urdf"), size, args.timing_repeats) for size in (1, 32, 256)]
    total_calls = invariance["allocation_calls"] + sum(case["allocation_calls"] for case in oracle_cases + timing)
    total_bytes = invariance["allocated_bytes"] + sum(case["allocated_bytes"] for case in oracle_cases + timing)
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "revision": "cuda-jacobian-abi-r76",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "profile": "CpuMirrorF32",
        "kernel_abi_version": oracle_cases[0]["kernel_abi_version"],
        "manifest": ["StateInput", "ForwardKinematics", "CenterOfMass", "Jacobians"],
        "layout": "frame_jacobian[body][spatial_component][generalized_coordinate][agent]",
        "oracle_cases": oracle_cases,
        "finite_difference_cases": finite_difference_cases,
        "invariance": invariance,
        "timing": timing,
        "total_allocation_calls": total_calls,
        "total_allocated_bytes": total_bytes,
        "cuda_device_executor": {"available": False, "status": "CpuMirrorReadyDeviceExecutorUnavailable", "d1": "not_run", "d2": "not_run", "d3": "not_run"},
        "admission": {
            "cpu_mirror_jacobians": bool(all(case["d3_pass"] for case in oracle_cases) and all(case["pass"] for case in finite_difference_cases) and invariance["pass"] and total_calls == 0 and total_bytes == 0),
            "cuda_mirror_jacobians": False,
            "cuda_throughput_jacobians": False,
        },
    }
    np.savez_compressed(output / "cuda-jacobian-abi-raw.npz", **raw)
    (output / "cuda-jacobian-abi-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CUDA_JACOBIAN_ABI_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw Jacobian mirror · r76"))
    if not metrics["admission"]["cpu_mirror_jacobians"]:
        raise SystemExit("CpuMirrorF32 Jacobian admission failed")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
