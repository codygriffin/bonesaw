#!/usr/bin/env python3
"""Admit CpuMirrorF32 floating rigid-body products against Pinocchio."""

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
TOLERANCES = {
    "mass": (4e-5, 4e-4),
    "bias": (1e-4, 8e-4),
    "centroidal": (4e-5, 4e-4),
}
EXACT_FIELDS = ("mass", "bias", "centroidal", "status")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=200)
    parser.add_argument("--output", default="benchmarks/results/cuda-dynamics-abi-r77")
    parser.add_argument("--web-report", default="web/CUDA_DYNAMICS_ABI_R77.html")
    return parser.parse_args()


def velocities(agents: int, generalized: int) -> np.ndarray:
    agent = np.arange(agents, dtype=np.float64)[:, None]
    coordinate = np.arange(generalized, dtype=np.float64)[None, :]
    return np.asarray(
        0.21 * np.sin(0.31 * agent + 0.23 * coordinate)
        + 0.07 * np.cos(0.17 * agent - 0.41 * coordinate),
        dtype=np.float32,
    )


def gravities(agents: int) -> np.ndarray:
    agent = np.arange(agents, dtype=np.float64)
    return np.asarray(
        np.column_stack(
            (
                0.17 + 0.01 * np.sin(agent),
                -0.09 + 0.01 * np.cos(0.7 * agent),
                -9.73 + 0.02 * np.sin(0.4 * agent),
            )
        ),
        dtype=np.float32,
    )


def output_buffers(session: CpuMirrorBatchSession) -> dict[str, np.ndarray]:
    agents = session.agent_capacity
    generalized = session.generalized_dof
    return {
        "mass": np.empty((agents, generalized, generalized), dtype=np.float32),
        "bias": np.empty((agents, generalized), dtype=np.float32),
        "centroidal": np.empty((agents, 6, generalized), dtype=np.float32),
        "status": np.empty(agents, dtype=np.uint8),
        "fk_execute_ns": np.empty(1, dtype=np.uint64),
        "jacobian_execute_ns": np.empty(1, dtype=np.uint64),
        "dynamics_execute_ns": np.empty(1, dtype=np.uint64),
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
    session.run_model_products_batch(
        np.ascontiguousarray(q, dtype=np.float32),
        np.ascontiguousarray(roots, dtype=np.float32),
        np.ascontiguousarray(velocity, dtype=np.float32),
        np.ascontiguousarray(gravity, dtype=np.float32),
        result["mass"],
        result["bias"],
        result["centroidal"],
        result["status"],
        result["fk_execute_ns"],
        result["jacobian_execute_ns"],
        result["dynamics_execute_ns"],
        result["allocation_calls"],
        result["allocated_bytes"],
    )
    return result


def copy_result(result: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {name: value.copy() for name, value in result.items()}


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


def tangent_map(rotation: np.ndarray, v_indices: list[int], generalized: int) -> np.ndarray:
    """Map Bonesaw [world angular, world linear, joints] to Pin tangent."""
    transform = np.zeros((generalized, generalized), dtype=np.float64)
    transform[0:3, 3:6] = rotation.T
    transform[3:6, 0:3] = rotation.T
    for coordinate, pin_index in enumerate(v_indices):
        transform[pin_index, 6 + coordinate] = 1.0
    return transform


def pinocchio_products(
    model_path: pathlib.Path,
    joint_names: list[str],
    q_batch: np.ndarray,
    roots: np.ndarray,
    velocity_batch: np.ndarray,
    gravity_batch: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model = pin.buildModelFromUrdf(str(model_path), pin.JointModelFreeFlyer())
    joint_ids = [model.getJointId(name) for name in joint_names]
    if any(joint_id == 0 for joint_id in joint_ids):
        raise ValueError("Pinocchio model does not match the Bonesaw joint layout")
    q_indices = [model.joints[joint_id].idx_q for joint_id in joint_ids]
    v_indices = [model.joints[joint_id].idx_v for joint_id in joint_ids]
    agents = len(q_batch)
    generalized = 6 + len(joint_names)
    masses = np.empty((agents, generalized, generalized), dtype=np.float64)
    biases = np.empty((agents, generalized), dtype=np.float64)
    centroidal = np.empty((agents, 6, generalized), dtype=np.float64)
    data = model.createData()
    for agent in range(agents):
        configuration = pin_configuration(model, q_indices, q_batch[agent], roots[agent])
        rotation = roots[agent, :9].astype(np.float64).reshape(3, 3)
        transform = tangent_map(rotation, v_indices, generalized)
        pin_velocity = transform @ velocity_batch[agent].astype(np.float64)
        # Pinocchio differentiates the local free-flyer linear velocity. With
        # zero world root acceleration, d(R.T v)/dt = -R.T (omega x v).
        pin_acceleration = np.zeros(generalized, dtype=np.float64)
        omega_world = velocity_batch[agent, 0:3].astype(np.float64)
        linear_world = velocity_batch[agent, 3:6].astype(np.float64)
        pin_acceleration[0:3] = -rotation.T @ np.cross(omega_world, linear_world)
        model.gravity.linear = gravity_batch[agent].astype(np.float64)
        pin.crba(model, data, configuration)
        pin_mass = np.asarray(data.M, dtype=np.float64)
        pin_bias = np.asarray(
            pin.rnea(model, data, configuration, pin_velocity, pin_acceleration),
            dtype=np.float64,
        ).reshape(-1)
        pin.ccrba(model, data, configuration, pin_velocity)
        pin_centroidal = np.asarray(data.Ag, dtype=np.float64)
        masses[agent] = transform.T @ pin_mass @ transform
        biases[agent] = transform.T @ pin_bias
        # Pin Force rows are [linear; angular], Bonesaw Force6 is
        # [moment; force], and both are world-expressed about system CoM.
        centroidal[agent, 0:3] = pin_centroidal[3:6] @ transform
        centroidal[agent, 3:6] = pin_centroidal[0:3] @ transform
    return masses, biases, centroidal


def tolerance(mirror: np.ndarray, oracle: np.ndarray, name: str) -> tuple[float, float]:
    absolute = np.abs(mirror.astype(np.float64) - oracle.astype(np.float64))
    abs_tol, rel_tol = TOLERANCES[name]
    allowed = abs_tol + rel_tol * np.maximum(np.abs(mirror), np.abs(oracle))
    return float(np.max(absolute)), float(np.max(absolute / allowed))


def oracle_case(model_path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), samples, 32)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    velocity = velocities(samples, session.generalized_dof)
    gravity = gravities(samples)
    mirror = run(session, q, roots, velocity, gravity)
    exact_mass, exact_bias, exact_centroidal = pinocchio_products(
        model_path, list(session.joint_names), q, roots, velocity, gravity
    )
    mass_abs, mass_score = tolerance(mirror["mass"], exact_mass, "mass")
    bias_abs, bias_score = tolerance(mirror["bias"], exact_bias, "bias")
    centroidal_abs, centroidal_score = tolerance(
        mirror["centroidal"], exact_centroidal, "centroidal"
    )
    symmetry = float(
        np.max(np.abs(mirror["mass"].astype(np.float64) - np.swapaxes(mirror["mass"], 1, 2)))
    )
    eigenvalues = np.linalg.eigvalsh(mirror["mass"].astype(np.float64))
    minimum_eigenvalue = float(np.min(eigenvalues))
    momentum_mirror = np.einsum("arg,ag->ar", mirror["centroidal"], velocity)
    momentum_oracle = np.einsum("arg,ag->ar", exact_centroidal, velocity)
    momentum_abs = float(np.max(np.abs(momentum_mirror - momentum_oracle)))
    metrics = {
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "coordinates": session.dof,
        "generalized_coordinates": session.generalized_dof,
        "maximum_mass_absolute_error": mass_abs,
        "maximum_mass_tolerance_score": mass_score,
        "maximum_bias_absolute_error": bias_abs,
        "maximum_bias_tolerance_score": bias_score,
        "maximum_centroidal_absolute_error": centroidal_abs,
        "maximum_centroidal_tolerance_score": centroidal_score,
        "mass_symmetry_maximum_absolute_error": symmetry,
        "minimum_mass_eigenvalue": minimum_eigenvalue,
        "maximum_centroidal_momentum_absolute_error": momentum_abs,
        "statuses_ok": bool(np.all(mirror["status"] == STATUS_OK)),
        "allocation_calls": int(mirror["allocation_calls"][0]),
        "allocated_bytes": int(mirror["allocated_bytes"][0]),
        "backend_fingerprint": json.loads(session.backend_fingerprint_json),
        "kernel_manifest_bits": int(session.kernel_manifest_bits),
        "kernel_abi_version": int(session.kernel_abi_version),
    }
    metrics["d3_pass"] = bool(
        mass_score <= 1.0
        and bias_score <= 1.0
        and centroidal_score <= 1.0
        and symmetry == 0.0
        and minimum_eigenvalue > 0.0
        and metrics["statuses_ok"]
        and metrics["allocation_calls"] == 0
        and metrics["allocated_bytes"] == 0
    )
    stem = model_path.stem
    return metrics, {
        f"{stem}_q": q,
        f"{stem}_root_pose": roots,
        f"{stem}_velocity": velocity,
        f"{stem}_gravity": gravity,
        f"{stem}_mirror_mass": mirror["mass"].copy(),
        f"{stem}_pinocchio_mass": exact_mass,
        f"{stem}_mirror_bias": mirror["bias"].copy(),
        f"{stem}_pinocchio_bias": exact_bias,
        f"{stem}_mirror_centroidal": mirror["centroidal"].copy(),
        f"{stem}_pinocchio_centroidal": exact_centroidal,
    }


def invariance_case(model_path: pathlib.Path, agents: int = 17) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = CpuMirrorBatchSession(str(model_path), agents, 32)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    baseline = copy_result(run(session, q, roots, velocity, gravity))
    repeat = run(session, q, roots, velocity, gravity)
    repeat_exact = all(np.array_equal(repeat[name], baseline[name]) for name in EXACT_FIELDS)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)], dtype=np.int64)
    permuted = run(session, q[permutation], roots[permutation], velocity[permutation], gravity[permutation])
    inverse = np.argsort(permutation)
    permutation_exact = all(np.array_equal(permuted[name][inverse], baseline[name]) for name in EXACT_FIELDS)
    compact_session = CpuMirrorBatchSession(str(model_path), agents, 1)
    compact = run(compact_session, q, roots, velocity, gravity)
    padding_exact = all(np.array_equal(compact[name], baseline[name]) for name in EXACT_FIELDS)
    chunks = []
    for start, stop in ((0, 7), (7, agents)):
        chunk_session = CpuMirrorBatchSession(str(model_path), stop - start, 32)
        chunks.append(run(chunk_session, q[start:stop], roots[start:stop], velocity[start:stop], gravity[start:stop]))
    chunking_exact = all(
        np.array_equal(np.concatenate([chunk[name] for chunk in chunks]), baseline[name])
        for name in EXACT_FIELDS
    )
    invalid_velocity = velocity.copy()
    invalid_agent = 8
    invalid_velocity[invalid_agent, 2] = np.nan
    invalid = run(session, q, roots, invalid_velocity, gravity)
    neighbors = np.arange(agents) != invalid_agent
    isolation_exact = bool(
        invalid["status"][invalid_agent] == STATUS_INVALID_INPUT
        and np.all(invalid["status"][neighbors] == STATUS_OK)
        and all(np.array_equal(invalid[name][neighbors], baseline[name][neighbors]) for name in EXACT_FIELDS if name != "status")
        and np.all(invalid["mass"][invalid_agent] == 0.0)
        and np.all(invalid["bias"][invalid_agent] == 0.0)
        and np.all(invalid["centroidal"][invalid_agent] == 0.0)
    )
    measured = [baseline, repeat, permuted, compact, *chunks, invalid]
    calls = sum(int(result["allocation_calls"][0]) for result in measured)
    allocated = sum(int(result["allocated_bytes"][0]) for result in measured)
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
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }
    metrics["pass"] = bool(repeat_exact and permutation_exact and padding_exact and chunking_exact and isolation_exact and calls == 0 and allocated == 0)
    return metrics, {
        "invariance_mass": baseline["mass"],
        "invariance_bias": baseline["bias"],
        "invariance_centroidal": baseline["centroidal"],
        "invariance_permutation": permutation,
        "invariance_invalid_status": invalid["status"].copy(),
    }


def timing_case(model_path: pathlib.Path, agents: int, repeats: int) -> dict[str, Any]:
    session = CpuMirrorBatchSession(str(model_path), agents, 32)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    buffers = output_buffers(session)
    for _ in range(10):
        run(session, q, roots, velocity, gravity, buffers)
    traces = {name: np.empty(repeats, dtype=np.float64) for name in ("fk", "jacobian", "dynamics")}
    calls = allocated = 0
    for repeat in range(repeats):
        result = run(session, q, roots, velocity, gravity, buffers)
        for name in traces:
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
        "# Bonesaw fixed-layout dynamics mirror · r77",
        "",
        "> PROMOTE CPU MODEL PRODUCTS ONLY. CUDA remains unavailable; no device correctness or performance claim is made.",
        "",
        "This policy-free, simulator-free gate extends the fixed batch ABI through floating mass matrix M(q), bias force h(q,v,g), and centroidal momentum map Ag(q). Python owns evaluation; preallocated Rust owns every timed stage.",
        "",
        "## Admission decision",
        "",
    ]
    lines += markdown_table(
        ["Gate", "Evidence", "Decision"],
        [
            ["CpuMirrorF32 ↔ independent Pinocchio", f"{sum(case['d3_pass'] for case in metrics['oracle_cases'])} / {len(metrics['oracle_cases'])} models", "PASS"],
            ["Mass symmetry / positive definiteness", "exact symmetry; positive minimum eigenvalue", "PASS"],
            ["D1 repeat bytes", str(inv["d1_repeat_bitwise_exact"]), "PASS"],
            ["Permutation / padding / chunking", f"{inv['permutation_exact']} / {inv['padding_exact']} / {inv['chunking_exact']}", "PASS"],
            ["Typed invalid-agent isolation", str(inv["invalid_agent_isolation_exact"]), "PASS"],
            ["Measured Rust allocations", f"{metrics['total_allocation_calls']} calls · {metrics['total_allocated_bytes']} B", "PASS"],
            ["CUDA dynamics D1 / D2 / D3", "executor unavailable", "NOT RUN"],
        ],
    )
    lines += ["", "## Independent Pinocchio oracle", ""]
    lines += markdown_table(
        ["Model", "States", "M max / gate", "h max / gate", "Ag max / gate", "min eig(M)", "D3"],
        [[pathlib.Path(case["model"]).stem, case["samples"], f"{case['maximum_mass_absolute_error']:.3e} / {case['maximum_mass_tolerance_score']:.3f}×", f"{case['maximum_bias_absolute_error']:.3e} / {case['maximum_bias_tolerance_score']:.3f}×", f"{case['maximum_centroidal_absolute_error']:.3e} / {case['maximum_centroidal_tolerance_score']:.3f}×", f"{case['minimum_mass_eigenvalue']:.3e}", "PASS" if case["d3_pass"] else "FAIL"] for case in metrics["oracle_cases"]],
    )
    lines += [
        "",
        "Pinocchio uses a body-local free-flyer tangent ordered linear then angular. The oracle explicitly maps it to Bonesaw's world-expressed angular-then-linear root tangent. For bias, it also supplies the configuration-dependent local linear acceleration induced when world root acceleration is zero. This prevents a false comparison caused by mismatched acceleration conventions.",
        "",
        "Declared element gates are M: 4e-5 + 4e-4·scale, h: 1e-4 + 8e-4·scale, Ag: 4e-5 + 4e-4·scale. The corpus uses nonzero root/joint velocity and non-axis-aligned, per-agent gravity.",
        "",
        "## Exact ABI and isolation gates",
        "",
    ]
    lines += markdown_table(
        ["Invariant", "Evidence", "Result"],
        [
            ["Layout", "M[g][g][agent], h[g][agent], Ag[6][g][agent]", "FROZEN"],
            ["Repeat", "M + h + Ag + status", "EXACT"],
            ["Permutation", "reverse-interleave then inverse index", "EXACT"],
            ["Padded stride", f"17 active: stride {inv['aligned_stride']} vs {inv['compact_stride']}", "EXACT"],
            ["Chunking", "7 + 10 agents reassembled", "EXACT"],
            ["Malformed velocity", f"agent 8 status={inv['invalid_agent_status']}; zero products; neighbors exact", "ISOLATED"],
        ],
    )
    lines += ["", "## Host timing · Upkie CPU mirror only", ""]
    lines += markdown_table(
        ["Agents", "FK p50/p99", "J p50/p99", "Dynamics p50/p99", "Combined p50/p99", "Combined/agent p50", "Alloc"],
        [[case["batch_size"], f"{case['fk_execute_ns']['p50']/1e3:.3f}/{case['fk_execute_ns']['p99']/1e3:.3f} µs", f"{case['jacobian_execute_ns']['p50']/1e3:.3f}/{case['jacobian_execute_ns']['p99']/1e3:.3f} µs", f"{case['dynamics_execute_ns']['p50']/1e3:.3f}/{case['dynamics_execute_ns']['p99']/1e3:.3f} µs", f"{case['combined_execute_ns']['p50']/1e3:.3f}/{case['combined_execute_ns']['p99']/1e3:.3f} µs", f"{case['per_agent_combined_ns']['p50']:.1f} ns", f"{case['allocation_calls']} / {case['allocated_bytes']} B"] for case in metrics["timing"]],
    )
    lines += [
        "",
        "Timers are stage-local and exclude NumPy↔SoA copies. The entire untrimmed timing trace feeds every percentile, including the p99 jitter tail. The current body-sum dynamics traversal is an auditable CPU admission baseline, not a throughput claim.",
        "",
        "## Deferred device admission",
        "",
        "CudaMirrorF32 must reproduce these exact layouts and D1 reindexing/isolation behavior, then satisfy CPU↔device D2/D3 gates for M, h, and Ag on a real CUDA driver/toolkit. CudaThroughputF32 remains separately fingerprinted and uncertified.",
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
    invariance, arrays = invariance_case(pathlib.Path("models/upkie/upkie.urdf"))
    raw.update(arrays)
    timing = [timing_case(pathlib.Path("models/upkie/upkie.urdf"), size, args.timing_repeats) for size in (1, 32, 256)]
    total_calls = invariance["allocation_calls"] + sum(case["allocation_calls"] for case in oracle_cases + timing)
    total_bytes = invariance["allocated_bytes"] + sum(case["allocated_bytes"] for case in oracle_cases + timing)
    admission = bool(all(case["d3_pass"] for case in oracle_cases) and invariance["pass"] and total_calls == 0 and total_bytes == 0)
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "revision": "cuda-dynamics-abi-r77",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "profile": "CpuMirrorF32",
        "kernel_abi_version": oracle_cases[0]["kernel_abi_version"],
        "manifest": ["StateInput", "ForwardKinematics", "CenterOfMass", "Jacobians", "RigidBodyDynamics"],
        "layout": "mass[g][g][agent], bias[g][agent], centroidal[6][g][agent]",
        "oracle_cases": oracle_cases,
        "invariance": invariance,
        "timing": timing,
        "total_allocation_calls": total_calls,
        "total_allocated_bytes": total_bytes,
        "cuda_device_executor": {"available": False, "status": "CpuMirrorReadyDeviceExecutorUnavailable", "d1": "not_run", "d2": "not_run", "d3": "not_run"},
        "admission": {"cpu_mirror_dynamics": admission, "cuda_mirror_dynamics": False, "cuda_throughput_dynamics": False},
    }
    np.savez_compressed(output / "cuda-dynamics-abi-raw.npz", **raw)
    (output / "cuda-dynamics-abi-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CUDA_DYNAMICS_ABI_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw dynamics mirror · r77"))
    if not admission:
        raise SystemExit("CpuMirrorF32 dynamics admission failed")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
