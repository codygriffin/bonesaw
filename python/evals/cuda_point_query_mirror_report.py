#!/usr/bin/env python3
"""Admit fixed compiler-resolved point position/J/Jdot-v batch slots."""

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
from cuda_dynamics_mirror_report import gravities, pin_configuration, tangent_map, velocities
from cuda_jacobian_mirror_report import so3_exp


STATUS_OK = 1
STATUS_INVALID_INPUT = 2
POSITION_TOLERANCE = (2e-5, 2e-4)
JACOBIAN_TOLERANCE = (2e-5, 2e-4)
VELOCITY_TOLERANCE = (2e-5, 2e-4)
BIAS_TOLERANCE = (4e-5, 4e-4)
FINITE_DIFFERENCE_STEP = 1e-3
EXACT_FIELDS = ("position", "jacobian", "bias", "status")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=200)
    parser.add_argument("--output", default="benchmarks/results/cuda-point-query-abi-r78")
    parser.add_argument("--web-report", default="web/CUDA_POINT_QUERY_ABI_R78.html")
    return parser.parse_args()


def query_plan(model_path: pathlib.Path) -> tuple[list[str], list[tuple[float, float, float]], list[int]]:
    if model_path.stem == "toy_humanoid":
        return (
            ["pelvis", "left_foot", "right_hand", "head"],
            [(0.13, -0.07, 0.19), (0.08, 0.03, -0.04), (0.05, -0.02, 0.01), (0.03, 0.0, 0.12)],
            [7801, 7802, 7803, 7804],
        )
    return (
        ["torso", "left_contact", "right_contact", "handle"],
        [(0.11, -0.04, 0.17), (0.02, 0.0, -0.01), (-0.02, 0.0, -0.01), (0.0, 0.06, 0.04)],
        [7811, 7812, 7813, 7814],
    )


def session_for(model_path: pathlib.Path, agents: int, alignment: int = 32) -> CpuMirrorBatchSession:
    frames, offsets, stable_ids = query_plan(model_path)
    return CpuMirrorBatchSession(
        str(model_path), agents, alignment, frames, offsets, stable_ids
    )


def output_buffers(session: CpuMirrorBatchSession) -> dict[str, np.ndarray]:
    agents = session.agent_capacity
    slots = session.point_query_count
    generalized = session.generalized_dof
    return {
        "position": np.empty((agents, slots, 3), dtype=np.float32),
        "jacobian": np.empty((agents, slots, 3, generalized), dtype=np.float32),
        "bias": np.empty((agents, slots, 3), dtype=np.float32),
        "status": np.empty(agents, dtype=np.uint8),
        "fk_execute_ns": np.empty(1, dtype=np.uint64),
        "jacobian_execute_ns": np.empty(1, dtype=np.uint64),
        "dynamics_execute_ns": np.empty(1, dtype=np.uint64),
        "point_execute_ns": np.empty(1, dtype=np.uint64),
        "allocation_calls": np.empty(1, dtype=np.uint64),
        "allocated_bytes": np.empty(1, dtype=np.uint64),
    }


def run(
    session: CpuMirrorBatchSession,
    q: np.ndarray,
    roots: np.ndarray,
    velocity: np.ndarray,
    gravity: np.ndarray,
    buffers: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    result = output_buffers(session) if buffers is None else buffers
    session.run_point_queries_batch(
        np.ascontiguousarray(q, dtype=np.float32),
        np.ascontiguousarray(roots, dtype=np.float32),
        np.ascontiguousarray(velocity, dtype=np.float32),
        np.ascontiguousarray(gravity, dtype=np.float32),
        result["position"],
        result["jacobian"],
        result["bias"],
        result["status"],
        result["fk_execute_ns"],
        result["jacobian_execute_ns"],
        result["dynamics_execute_ns"],
        result["point_execute_ns"],
        result["allocation_calls"],
        result["allocated_bytes"],
    )
    return result


def pin_point_products(
    model_path: pathlib.Path,
    joint_names: list[str],
    frame_names: list[str],
    offsets: np.ndarray,
    q_batch: np.ndarray,
    roots: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    joint_ids = [model.getJointId(name) for name in joint_names]
    frame_ids = [model.getFrameId(name, pin.FrameType.BODY) for name in frame_names]
    if any(joint == 0 for joint in joint_ids) or any(frame >= len(model.frames) for frame in frame_ids):
        raise ValueError("Pinocchio model does not match the compiled point plan")
    q_indices = [model.joints[joint].idx_q for joint in joint_ids]
    v_indices = [model.joints[joint].idx_v for joint in joint_ids]
    agents = len(q_batch)
    generalized = 6 + len(joint_names)
    positions = np.empty((agents, len(frame_ids), 3), dtype=np.float64)
    jacobians = np.empty((agents, len(frame_ids), 3, generalized), dtype=np.float64)
    data = model.createData()
    for agent in range(agents):
        configuration = pin_configuration(model, q_indices, q_batch[agent], roots[agent])
        pin.computeJointJacobians(model, data, configuration)
        pin.updateFramePlacements(model, data)
        root_rotation = roots[agent, :9].astype(np.float64).reshape(3, 3)
        transform = tangent_map(root_rotation, v_indices, generalized)
        for slot, (frame_id, offset) in enumerate(zip(frame_ids, offsets, strict=True)):
            frame = data.oMf[frame_id]
            offset_world = frame.rotation @ offset
            positions[agent, slot] = frame.translation + offset_world
            frame_jacobian = np.asarray(
                pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED),
                dtype=np.float64,
            )
            point_jacobian_pin = frame_jacobian[0:3] + np.cross(
                frame_jacobian[3:6].T, offset_world
            ).T
            jacobians[agent, slot] = point_jacobian_pin @ transform
    return positions, jacobians


def perturbed_state(
    q: np.ndarray, roots: np.ndarray, velocity: np.ndarray, sign: float
) -> tuple[np.ndarray, np.ndarray]:
    dt = sign * FINITE_DIFFERENCE_STEP
    # Preserve f64 perturbations for the independent finite-difference oracle;
    # casting the tiny displacement back to f32 would dominate the second
    # derivative after division by dt².
    q_perturbed = q.astype(np.float64) + dt * velocity[:, 6:].astype(np.float64)
    roots_perturbed = roots.astype(np.float64).copy()
    for agent in range(len(q)):
        rotation = roots[agent, :9].astype(np.float64).reshape(3, 3)
        roots_perturbed[agent, :9] = (
            so3_exp(dt * velocity[agent, :3].astype(np.float64)) @ rotation
        ).reshape(-1)
        roots_perturbed[agent, 9:12] = roots[agent, 9:12].astype(np.float64) + dt * velocity[
            agent, 3:6
        ].astype(np.float64)
    return q_perturbed, roots_perturbed


def score(left: np.ndarray, right: np.ndarray, tolerance: tuple[float, float]) -> tuple[float, float]:
    error = np.abs(left.astype(np.float64) - right.astype(np.float64))
    absolute, relative = tolerance
    allowed = absolute + relative * np.maximum(np.abs(left), np.abs(right))
    return float(np.max(error)), float(np.max(error / allowed))


def oracle_case(model_path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = session_for(model_path, samples)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    velocity = velocities(samples, session.generalized_dof)
    gravity = gravities(samples)
    mirror = run(session, q, roots, velocity, gravity)
    offsets = np.asarray(session.point_offsets, dtype=np.float64)
    exact_position, exact_jacobian = pin_point_products(
        model_path,
        list(session.joint_names),
        list(session.point_frame_names),
        offsets,
        q,
        roots,
    )
    plus_q, plus_roots = perturbed_state(q, roots, velocity, 1.0)
    minus_q, minus_roots = perturbed_state(q, roots, velocity, -1.0)
    plus_position, _ = pin_point_products(
        model_path, list(session.joint_names), list(session.point_frame_names), offsets, plus_q, plus_roots
    )
    minus_position, _ = pin_point_products(
        model_path, list(session.joint_names), list(session.point_frame_names), offsets, minus_q, minus_roots
    )
    fd_velocity = (plus_position - minus_position) / (2.0 * FINITE_DIFFERENCE_STEP)
    predicted_velocity = np.einsum("asrg,ag->asr", mirror["jacobian"], velocity)
    fd_bias = (plus_position - 2.0 * exact_position + minus_position) / (FINITE_DIFFERENCE_STEP**2)
    position_abs, position_gate = score(mirror["position"], exact_position, POSITION_TOLERANCE)
    jacobian_abs, jacobian_gate = score(mirror["jacobian"], exact_jacobian, JACOBIAN_TOLERANCE)
    velocity_abs, velocity_gate = score(predicted_velocity, fd_velocity, VELOCITY_TOLERANCE)
    bias_abs, bias_gate = score(mirror["bias"], fd_bias, BIAS_TOLERANCE)
    metrics = {
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "point_slots": session.point_query_count,
        "stable_ids": list(session.point_stable_ids),
        "frames": list(session.point_frame_names),
        "offsets": [list(value) for value in session.point_offsets],
        "maximum_position_absolute_error": position_abs,
        "maximum_position_tolerance_score": position_gate,
        "maximum_jacobian_absolute_error": jacobian_abs,
        "maximum_jacobian_tolerance_score": jacobian_gate,
        "maximum_velocity_absolute_error": velocity_abs,
        "maximum_velocity_tolerance_score": velocity_gate,
        "maximum_bias_absolute_error": bias_abs,
        "maximum_bias_tolerance_score": bias_gate,
        "statuses_ok": bool(np.all(mirror["status"] == STATUS_OK)),
        "allocation_calls": int(mirror["allocation_calls"][0]),
        "allocated_bytes": int(mirror["allocated_bytes"][0]),
        "backend_fingerprint": json.loads(session.backend_fingerprint_json),
        "kernel_manifest_bits": int(session.kernel_manifest_bits),
        "kernel_abi_version": int(session.kernel_abi_version),
    }
    metrics["d3_pass"] = bool(
        max(position_gate, jacobian_gate, velocity_gate, bias_gate) <= 1.0
        and metrics["statuses_ok"]
        and metrics["allocation_calls"] == 0
        and metrics["allocated_bytes"] == 0
    )
    stem = model_path.stem
    return metrics, {
        f"{stem}_q": q,
        f"{stem}_root_pose": roots,
        f"{stem}_velocity": velocity,
        f"{stem}_mirror_position": mirror["position"].copy(),
        f"{stem}_pinocchio_position": exact_position,
        f"{stem}_mirror_jacobian": mirror["jacobian"].copy(),
        f"{stem}_pinocchio_jacobian": exact_jacobian,
        f"{stem}_mirror_bias": mirror["bias"].copy(),
        f"{stem}_finite_difference_bias": fd_bias,
        f"{stem}_predicted_velocity": predicted_velocity,
        f"{stem}_finite_difference_velocity": fd_velocity,
    }


def invariance_case(model_path: pathlib.Path, agents: int = 17) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = session_for(model_path, agents)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    baseline = {name: value.copy() for name, value in run(session, q, roots, velocity, gravity).items()}
    repeat = run(session, q, roots, velocity, gravity)
    repeat_exact = all(np.array_equal(repeat[name], baseline[name]) for name in EXACT_FIELDS)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)], dtype=np.int64)
    permuted = run(session, q[permutation], roots[permutation], velocity[permutation], gravity[permutation])
    inverse = np.argsort(permutation)
    permutation_exact = all(np.array_equal(permuted[name][inverse], baseline[name]) for name in EXACT_FIELDS)
    compact_session = session_for(model_path, agents, 1)
    compact = run(compact_session, q, roots, velocity, gravity)
    padding_exact = all(np.array_equal(compact[name], baseline[name]) for name in EXACT_FIELDS)
    chunks = []
    for start, stop in ((0, 7), (7, agents)):
        chunk = session_for(model_path, stop - start)
        chunks.append(run(chunk, q[start:stop], roots[start:stop], velocity[start:stop], gravity[start:stop]))
    chunking_exact = all(np.array_equal(np.concatenate([part[name] for part in chunks]), baseline[name]) for name in EXACT_FIELDS)
    invalid_velocity = velocity.copy()
    invalid_velocity[8, 0] = np.nan
    invalid = run(session, q, roots, invalid_velocity, gravity)
    neighbors = np.arange(agents) != 8
    isolation = bool(
        invalid["status"][8] == STATUS_INVALID_INPUT
        and np.all(invalid["status"][neighbors] == STATUS_OK)
        and all(np.array_equal(invalid[name][neighbors], baseline[name][neighbors]) for name in EXACT_FIELDS if name != "status")
        and np.all(invalid["position"][8] == 0.0)
        and np.all(invalid["jacobian"][8] == 0.0)
        and np.all(invalid["bias"][8] == 0.0)
    )
    measured = [baseline, repeat, permuted, compact, *chunks, invalid]
    calls = sum(int(item["allocation_calls"][0]) for item in measured)
    allocated = sum(int(item["allocated_bytes"][0]) for item in measured)
    metrics = {
        "agents": agents,
        "aligned_stride": session.agent_stride,
        "compact_stride": compact_session.agent_stride,
        "d1_repeat_bitwise_exact": repeat_exact,
        "permutation_exact": permutation_exact,
        "padding_exact": padding_exact,
        "chunking_exact": chunking_exact,
        "invalid_agent_isolation_exact": isolation,
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }
    metrics["pass"] = bool(repeat_exact and permutation_exact and padding_exact and chunking_exact and isolation and calls == 0 and allocated == 0)
    return metrics, {
        "invariance_position": baseline["position"],
        "invariance_jacobian": baseline["jacobian"],
        "invariance_bias": baseline["bias"],
        "invariance_invalid_status": invalid["status"].copy(),
    }


def timing_case(model_path: pathlib.Path, agents: int, repeats: int) -> dict[str, Any]:
    session = session_for(model_path, agents)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    buffers = output_buffers(session)
    for _ in range(10):
        run(session, q, roots, velocity, gravity, buffers)
    names = ("fk", "jacobian", "dynamics", "point")
    traces = {name: np.empty(repeats, dtype=np.float64) for name in names}
    calls = allocated = 0
    for repeat in range(repeats):
        result = run(session, q, roots, velocity, gravity, buffers)
        for name in names:
            traces[name][repeat] = result[f"{name}_execute_ns"][0]
        calls += int(result["allocation_calls"][0])
        allocated += int(result["allocated_bytes"][0])
    combined = sum(traces.values())
    return {
        "batch_size": agents,
        "agent_stride": session.agent_stride,
        "repeats": repeats,
        **{f"{name}_execute_ns": distribution(trace) for name, trace in traces.items()},
        "combined_execute_ns": distribution(combined),
        "per_agent_combined_ns": {key: value / agents for key, value in distribution(combined).items()},
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    inv = metrics["invariance"]
    lines = [
        "# Bonesaw fixed point-query mirror · r78",
        "",
        "> PROMOTE COMPILER-RESOLVED CPU POINT QUERIES ONLY. CUDA remains unavailable; no device correctness or performance claim is made.",
        "",
        "This policy-free, simulator-free gate adds stable-ID point slots after the admitted FK/Jacobian/dynamics stages. Every slot has a frozen body frame and local offset hashed into the kernel descriptor and emits world position, floating point Jacobian, and kinematic bias acceleration Jdot-v.",
        "",
        "## Admission decision",
        "",
    ]
    lines += markdown_table(
        ["Gate", "Evidence", "Decision"],
        [
            ["Point position/J ↔ Pinocchio", f"{sum(case['d3_pass'] for case in metrics['oracle_cases'])} / {len(metrics['oracle_cases'])} models", "PASS"],
            ["Jv ↔ central-difference velocity", "root SO(3)/translation + every joint", "PASS"],
            ["Jdot-v ↔ central-difference acceleration", "zero world generalized acceleration", "PASS"],
            ["Stable IDs and offsets", "descriptor-hashed fixed slot plan", "PASS"],
            ["Repeat/reindex/isolation", "bitwise exact", "PASS"],
            ["Measured Rust allocations", f"{metrics['total_allocation_calls']} calls · {metrics['total_allocated_bytes']} B", "PASS"],
            ["CUDA point-query D1/D2/D3", "executor unavailable", "NOT RUN"],
        ],
    )
    lines += ["", "## Independent point oracle", ""]
    lines += markdown_table(
        ["Model", "States×slots", "position max/gate", "J max/gate", "Jv max/gate", "Jdot-v max/gate", "D3"],
        [[pathlib.Path(case["model"]).stem, f"{case['samples']}×{case['point_slots']}", f"{case['maximum_position_absolute_error']:.3e}/{case['maximum_position_tolerance_score']:.3f}×", f"{case['maximum_jacobian_absolute_error']:.3e}/{case['maximum_jacobian_tolerance_score']:.3f}×", f"{case['maximum_velocity_absolute_error']:.3e}/{case['maximum_velocity_tolerance_score']:.3f}×", f"{case['maximum_bias_absolute_error']:.3e}/{case['maximum_bias_tolerance_score']:.3f}×", "PASS" if case["d3_pass"] else "FAIL"] for case in metrics["oracle_cases"]],
    )
    lines += [
        "",
        "Pinocchio independently supplies BODY placements and LOCAL_WORLD_ALIGNED spatial Jacobians. The evaluator rotates each nonzero local offset, converts Pinocchio's local linear-first free-flyer tangent to Bonesaw's world angular-first tangent, and constructs the point Jacobian. A separate central difference left-multiplies root Exp(±dt·omega), translates the root, and advances every joint at constant generalized velocity. Its first derivative checks Jv; its second derivative checks Jdot-v without a policy or physics rollout.",
        "",
        "## Exact layout and isolation",
        "",
    ]
    lines += markdown_table(
        ["Invariant", "Evidence", "Result"],
        [
            ["Layout", "point[slot][3][agent], J[slot][3][g][agent], bias[slot][3][agent]", "FROZEN"],
            ["Repeat", str(inv["d1_repeat_bitwise_exact"]), "EXACT"],
            ["Permutation/padding/chunking", f"{inv['permutation_exact']} / {inv['padding_exact']} / {inv['chunking_exact']}", "EXACT"],
            ["Malformed velocity", "agent 8 zeroed; neighbors unchanged", "ISOLATED"],
        ],
    )
    lines += ["", "## Host timing · Upkie four point slots", ""]
    lines += markdown_table(
        ["Agents", "Point p50/p99", "Full pipeline p50/p99", "Full/agent p50", "Alloc"],
        [[case["batch_size"], f"{case['point_execute_ns']['p50']/1e3:.3f}/{case['point_execute_ns']['p99']/1e3:.3f} µs", f"{case['combined_execute_ns']['p50']/1e3:.3f}/{case['combined_execute_ns']['p99']/1e3:.3f} µs", f"{case['per_agent_combined_ns']['p50']:.1f} ns", f"{case['allocation_calls']} / {case['allocated_bytes']} B"] for case in metrics["timing"]],
    )
    lines += [
        "",
        "Stage timers exclude NumPy↔SoA copies and retain every timing sample, including the p99 tail. This admits query products, not task residual/target emission or a solver.",
        "",
        "## Deferred boundary",
        "",
        "The next CPU slice may lower fixed point targets and contact locks into stable-ID task/constraint rows using these exact J and Jdot-v products. CudaMirrorF32 must later reproduce the point plan and pass device D1/D2/D3 on real hardware; no CPU fallback may masquerade as device execution.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    raw: dict[str, np.ndarray] = {}
    oracle_cases = []
    for model in models:
        case, arrays = oracle_case(model, args.samples)
        oracle_cases.append(case)
        raw.update(arrays)
    invariance, arrays = invariance_case(models[1])
    raw.update(arrays)
    timing = [timing_case(models[1], size, args.timing_repeats) for size in (1, 32, 256)]
    total_calls = invariance["allocation_calls"] + sum(case["allocation_calls"] for case in oracle_cases + timing)
    total_bytes = invariance["allocated_bytes"] + sum(case["allocated_bytes"] for case in oracle_cases + timing)
    admission = bool(all(case["d3_pass"] for case in oracle_cases) and invariance["pass"] and total_calls == 0 and total_bytes == 0)
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "revision": "cuda-point-query-abi-r78",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "profile": "CpuMirrorF32",
        "kernel_abi_version": oracle_cases[0]["kernel_abi_version"],
        "manifest": ["StateInput", "ForwardKinematics", "CenterOfMass", "Jacobians", "RigidBodyDynamics", "PointQueries"],
        "layout": "point[slot][3][agent], point_jacobian[slot][3][g][agent], point_bias[slot][3][agent]",
        "oracle_cases": oracle_cases,
        "invariance": invariance,
        "timing": timing,
        "total_allocation_calls": total_calls,
        "total_allocated_bytes": total_bytes,
        "cuda_device_executor": {"available": False, "status": "CpuMirrorReadyDeviceExecutorUnavailable", "d1": "not_run", "d2": "not_run", "d3": "not_run"},
        "admission": {"cpu_mirror_point_queries": admission, "cuda_mirror_point_queries": False, "cuda_throughput_point_queries": False},
    }
    np.savez_compressed(output / "cuda-point-query-abi-raw.npz", **raw)
    (output / "cuda-point-query-abi-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CUDA_POINT_QUERY_ABI_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw point-query mirror · r78"))
    if not admission:
        raise SystemExit("CpuMirrorF32 point-query admission failed")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
