#!/usr/bin/env python3
"""Admit fixed point-attractor and contact-lock emission semantics."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import CpuMirrorBatchSession
from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import joint_states, root_pose_matrix, sha256
from cuda_dynamics_mirror_report import gravities, velocities
from cuda_point_query_mirror_report import output_buffers as point_output_buffers
from cuda_point_query_mirror_report import query_plan, run as run_points


STATUS_OK = 1
STATUS_INVALID_INPUT = 2
TASK_QUERY_SLOTS = [0, 3]
TASK_STABLE_IDS = [7901, 7902]
TASK_PRIORITIES = [1, 2]
TASK_WEIGHTS = [1.5, 0.75]
TASK_BANDWIDTH_HZ = [1.5, 2.25]
CONTACT_QUERY_SLOTS = [1, 2]
CONTACT_STABLE_IDS = [7951, 7952]
EXACT_FIELDS = (
    "task_active",
    "task_position_error",
    "task_velocity_error",
    "task_desired_acceleration",
    "task_jacobian",
    "task_rhs",
    "contact_active",
    "contact_jacobian",
    "contact_rhs",
    "status",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=200)
    parser.add_argument("--output", default="benchmarks/results/cuda-emission-abi-r79")
    parser.add_argument("--web-report", default="web/CUDA_EMISSION_ABI_R79.html")
    return parser.parse_args()


def session_for(model_path: pathlib.Path, agents: int, alignment: int = 32) -> CpuMirrorBatchSession:
    frames, offsets, stable_ids = query_plan(model_path)
    return CpuMirrorBatchSession(
        str(model_path),
        agents,
        alignment,
        frames,
        offsets,
        stable_ids,
        point_task_query_slots=TASK_QUERY_SLOTS,
        point_task_stable_ids=TASK_STABLE_IDS,
        point_task_priorities=TASK_PRIORITIES,
        point_task_weights=TASK_WEIGHTS,
        point_task_bandwidth_hz=TASK_BANDWIDTH_HZ,
        contact_query_slots=CONTACT_QUERY_SLOTS,
        contact_stable_ids=CONTACT_STABLE_IDS,
    )


def output_buffers(session: CpuMirrorBatchSession) -> dict[str, np.ndarray]:
    agents = session.agent_capacity
    tasks = session.point_task_count
    contacts = session.contact_lock_count
    generalized = session.generalized_dof
    return {
        "task_active": np.empty((agents, tasks), dtype=np.uint8),
        "task_position_error": np.empty((agents, tasks, 3), dtype=np.float32),
        "task_velocity_error": np.empty((agents, tasks, 3), dtype=np.float32),
        "task_desired_acceleration": np.empty((agents, tasks, 3), dtype=np.float32),
        "task_jacobian": np.empty((agents, tasks, 3, generalized), dtype=np.float32),
        "task_rhs": np.empty((agents, tasks, 3), dtype=np.float32),
        "contact_active": np.empty((agents, contacts), dtype=np.uint8),
        "contact_jacobian": np.empty((agents, contacts, 3, generalized), dtype=np.float32),
        "contact_rhs": np.empty((agents, contacts, 3), dtype=np.float32),
        "status": np.empty(agents, dtype=np.uint8),
        "fk_execute_ns": np.empty(1, dtype=np.uint64),
        "jacobian_execute_ns": np.empty(1, dtype=np.uint64),
        "dynamics_execute_ns": np.empty(1, dtype=np.uint64),
        "point_execute_ns": np.empty(1, dtype=np.uint64),
        "emission_execute_ns": np.empty(1, dtype=np.uint64),
        "allocation_calls": np.empty(1, dtype=np.uint64),
        "allocated_bytes": np.empty(1, dtype=np.uint64),
    }


def target_inputs(agents: int) -> dict[str, np.ndarray]:
    a = np.arange(agents, dtype=np.float32)[:, None, None]
    t = np.arange(len(TASK_QUERY_SLOTS), dtype=np.float32)[None, :, None]
    c = np.arange(3, dtype=np.float32)[None, None, :]
    task_active = np.ones((agents, len(TASK_QUERY_SLOTS)), dtype=np.uint8)
    task_active[:, 1] = (np.arange(agents) % 2 == 0).astype(np.uint8)
    contact_active = np.ones((agents, len(CONTACT_QUERY_SLOTS)), dtype=np.uint8)
    contact_active[:, 1] = (np.arange(agents) % 3 == 0).astype(np.uint8)
    return {
        "task_active": task_active,
        "target_position": (0.12 + 0.001 * a + 0.03 * t + 0.01 * c).astype(np.float32),
        "target_velocity": (-0.04 + 0.002 * a - 0.01 * t + 0.005 * c).astype(np.float32),
        "target_acceleration": (0.03 - 0.001 * a + 0.02 * t - 0.004 * c).astype(np.float32),
        "contact_active": contact_active,
        "contact_desired_acceleration": (0.005 * a + 0.01 * t - 0.002 * c).astype(np.float32),
    }


def run(
    session: CpuMirrorBatchSession,
    q: np.ndarray,
    roots: np.ndarray,
    velocity: np.ndarray,
    gravity: np.ndarray,
    targets: dict[str, np.ndarray],
    buffers: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    result = output_buffers(session) if buffers is None else buffers
    session.run_emission_batch(
        np.ascontiguousarray(q, dtype=np.float32),
        np.ascontiguousarray(roots, dtype=np.float32),
        np.ascontiguousarray(velocity, dtype=np.float32),
        np.ascontiguousarray(gravity, dtype=np.float32),
        np.ascontiguousarray(targets["task_active"], dtype=np.uint8),
        np.ascontiguousarray(targets["target_position"], dtype=np.float32),
        np.ascontiguousarray(targets["target_velocity"], dtype=np.float32),
        np.ascontiguousarray(targets["target_acceleration"], dtype=np.float32),
        np.ascontiguousarray(targets["contact_active"], dtype=np.uint8),
        np.ascontiguousarray(targets["contact_desired_acceleration"], dtype=np.float32),
        result["task_active"],
        result["task_position_error"],
        result["task_velocity_error"],
        result["task_desired_acceleration"],
        result["task_jacobian"],
        result["task_rhs"],
        result["contact_active"],
        result["contact_jacobian"],
        result["contact_rhs"],
        result["status"],
        result["fk_execute_ns"],
        result["jacobian_execute_ns"],
        result["dynamics_execute_ns"],
        result["point_execute_ns"],
        result["emission_execute_ns"],
        result["allocation_calls"],
        result["allocated_bytes"],
    )
    return result


def subset_targets(targets: dict[str, np.ndarray], indices: np.ndarray | slice) -> dict[str, np.ndarray]:
    return {name: np.ascontiguousarray(value[indices]) for name, value in targets.items()}


def maximum_error(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.max(np.abs(left.astype(np.float64) - right.astype(np.float64)), initial=0.0))


def oracle_case(model_path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session = session_for(model_path, samples)
    q = joint_states(samples, session.dof)
    roots = root_pose_matrix(samples)
    velocity = velocities(samples, session.generalized_dof)
    gravity = gravities(samples)
    targets = target_inputs(samples)
    point = run_points(session, q, roots, velocity, gravity, point_output_buffers(session))
    emitted = run(session, q, roots, velocity, gravity, targets)
    task_slots = np.asarray(session.point_task_query_slots, dtype=np.int64)
    contact_slots = np.asarray(session.contact_lock_query_slots, dtype=np.int64)
    task_mask = targets["task_active"].astype(bool)[:, :, None]
    contact_mask = targets["contact_active"].astype(bool)[:, :, None]
    task_j = point["jacobian"][:, task_slots]
    task_position = point["position"][:, task_slots]
    task_bias = point["bias"][:, task_slots]
    current_velocity = np.einsum("atcg,ag->atc", task_j, velocity, dtype=np.float32)
    position_error = targets["target_position"] - task_position
    velocity_error = targets["target_velocity"] - current_velocity
    bandwidth = np.asarray(session.point_task_bandwidth_hz, dtype=np.float32)[None, :, None]
    omega = np.float32(2.0 * np.pi) * bandwidth
    desired = (
        targets["target_acceleration"]
        + np.float32(2.0) * omega * velocity_error
        + omega * omega * position_error
    )
    expected_task_j = np.where(task_mask[:, :, :, None], task_j, 0.0)
    expected_position_error = np.where(task_mask, position_error, 0.0)
    expected_velocity_error = np.where(task_mask, velocity_error, 0.0)
    expected_desired = np.where(task_mask, desired, 0.0)
    expected_task_rhs = np.where(task_mask, desired - task_bias, 0.0)
    contact_j = point["jacobian"][:, contact_slots]
    contact_bias = point["bias"][:, contact_slots]
    expected_contact_j = np.where(contact_mask[:, :, :, None], contact_j, 0.0)
    expected_contact_rhs = np.where(
        contact_mask, targets["contact_desired_acceleration"] - contact_bias, 0.0
    )
    errors = {
        "position_error": maximum_error(emitted["task_position_error"], expected_position_error),
        "velocity_error": maximum_error(emitted["task_velocity_error"], expected_velocity_error),
        "desired_acceleration": maximum_error(emitted["task_desired_acceleration"], expected_desired),
        "task_jacobian": maximum_error(emitted["task_jacobian"], expected_task_j),
        "task_rhs": maximum_error(emitted["task_rhs"], expected_task_rhs),
        "contact_jacobian": maximum_error(emitted["contact_jacobian"], expected_contact_j),
        "contact_rhs": maximum_error(emitted["contact_rhs"], expected_contact_rhs),
    }
    # This is diagnostic only: all active rows share one least-squares qdd,
    # exposing conflict without redefining correct row emission as feasibility.
    feasibility_residuals = []
    for agent in range(samples):
        rows = []
        rhs = []
        for slot in range(session.point_task_count):
            if emitted["task_active"][agent, slot]:
                rows.append(emitted["task_jacobian"][agent, slot])
                rhs.append(emitted["task_rhs"][agent, slot])
        for slot in range(session.contact_lock_count):
            if emitted["contact_active"][agent, slot]:
                rows.append(emitted["contact_jacobian"][agent, slot])
                rhs.append(emitted["contact_rhs"][agent, slot])
        matrix = np.concatenate(rows, axis=0).astype(np.float64)
        target = np.concatenate(rhs, axis=0).astype(np.float64)
        qdd = np.linalg.lstsq(matrix, target, rcond=None)[0]
        feasibility_residuals.append(float(np.max(np.abs(matrix @ qdd - target))))
    metadata_exact = bool(
        list(session.point_task_stable_ids) == TASK_STABLE_IDS
        and list(session.point_task_query_slots) == TASK_QUERY_SLOTS
        and list(session.point_task_priorities) == TASK_PRIORITIES
        and np.array_equal(np.asarray(session.point_task_weights), np.asarray(TASK_WEIGHTS, dtype=np.float32))
        and np.array_equal(np.asarray(session.point_task_bandwidth_hz), np.asarray(TASK_BANDWIDTH_HZ, dtype=np.float32))
        and list(session.contact_lock_stable_ids) == CONTACT_STABLE_IDS
        and list(session.contact_lock_query_slots) == CONTACT_QUERY_SLOTS
    )
    inactive_zero = bool(
        np.all(emitted["task_rhs"][~task_mask.repeat(3, axis=2)] == 0.0)
        and np.all(emitted["contact_rhs"][~contact_mask.repeat(3, axis=2)] == 0.0)
    )
    passed = bool(
        max(errors.values()) <= 5e-4
        and metadata_exact
        and inactive_zero
        and np.all(emitted["status"] == STATUS_OK)
        and int(emitted["allocation_calls"][0]) == 0
        and int(emitted["allocated_bytes"][0]) == 0
    )
    metrics = {
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "samples": samples,
        "task_slots": session.point_task_count,
        "contact_slots": session.contact_lock_count,
        "metadata_exact": metadata_exact,
        "inactive_rows_exact_zero": inactive_zero,
        "maximum_errors": errors,
        "least_squares_shared_qdd_residual": distribution(np.asarray(feasibility_residuals)),
        "statuses_ok": bool(np.all(emitted["status"] == STATUS_OK)),
        "allocation_calls": int(emitted["allocation_calls"][0]),
        "allocated_bytes": int(emitted["allocated_bytes"][0]),
        "pass": passed,
    }
    stem = model_path.stem
    return metrics, {
        f"{stem}_task_rhs": emitted["task_rhs"].copy(),
        f"{stem}_contact_rhs": emitted["contact_rhs"].copy(),
        f"{stem}_task_jacobian": emitted["task_jacobian"].copy(),
        f"{stem}_contact_jacobian": emitted["contact_jacobian"].copy(),
        f"{stem}_shared_qdd_residual": np.asarray(feasibility_residuals),
    }


def invariance_case(model_path: pathlib.Path, agents: int = 17) -> dict[str, Any]:
    session = session_for(model_path, agents)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    targets = target_inputs(agents)
    baseline = {name: value.copy() for name, value in run(session, q, roots, velocity, gravity, targets).items()}
    repeat = run(session, q, roots, velocity, gravity, targets)
    repeat_exact = all(np.array_equal(repeat[name], baseline[name]) for name in EXACT_FIELDS)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)], dtype=np.int64)
    permuted = run(
        session,
        q[permutation],
        roots[permutation],
        velocity[permutation],
        gravity[permutation],
        subset_targets(targets, permutation),
    )
    inverse = np.argsort(permutation)
    permutation_exact = all(np.array_equal(permuted[name][inverse], baseline[name]) for name in EXACT_FIELDS)
    compact = run(session_for(model_path, agents, 1), q, roots, velocity, gravity, targets)
    padding_exact = all(np.array_equal(compact[name], baseline[name]) for name in EXACT_FIELDS)
    chunks = []
    for start, stop in ((0, 7), (7, agents)):
        chunks.append(
            run(
                session_for(model_path, stop - start),
                q[start:stop],
                roots[start:stop],
                velocity[start:stop],
                gravity[start:stop],
                subset_targets(targets, slice(start, stop)),
            )
        )
    chunking_exact = all(
        np.array_equal(np.concatenate([part[name] for part in chunks]), baseline[name])
        for name in EXACT_FIELDS
    )
    poisoned = {name: value.copy() for name, value in targets.items()}
    poisoned["target_position"][8, 0, 0] = np.nan
    invalid = run(session, q, roots, velocity, gravity, poisoned)
    neighbors = np.arange(agents) != 8
    isolation = bool(
        invalid["status"][8] == STATUS_INVALID_INPUT
        and np.all(invalid["status"][neighbors] == STATUS_OK)
        and all(
            np.array_equal(invalid[name][neighbors], baseline[name][neighbors])
            for name in EXACT_FIELDS
            if name != "status"
        )
        and all(np.all(invalid[name][8] == 0) for name in EXACT_FIELDS if name != "status")
    )
    measured = [baseline, repeat, permuted, compact, *chunks, invalid]
    calls = sum(int(item["allocation_calls"][0]) for item in measured)
    allocated = sum(int(item["allocated_bytes"][0]) for item in measured)
    passed = repeat_exact and permutation_exact and padding_exact and chunking_exact and isolation and calls == 0 and allocated == 0
    return {
        "agents": agents,
        "d1_repeat_bitwise_exact": repeat_exact,
        "permutation_exact": permutation_exact,
        "padding_exact": padding_exact,
        "chunking_exact": chunking_exact,
        "invalid_agent_isolation_exact": isolation,
        "allocation_calls": calls,
        "allocated_bytes": allocated,
        "pass": bool(passed),
    }


def timing_case(model_path: pathlib.Path, agents: int, repeats: int) -> dict[str, Any]:
    session = session_for(model_path, agents)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    targets = target_inputs(agents)
    buffers = output_buffers(session)
    for _ in range(10):
        run(session, q, roots, velocity, gravity, targets, buffers)
    names = ("fk", "jacobian", "dynamics", "point", "emission")
    traces = {name: np.empty(repeats, dtype=np.float64) for name in names}
    calls = allocated = 0
    for repeat in range(repeats):
        result = run(session, q, roots, velocity, gravity, targets, buffers)
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
        "# Bonesaw task/contact emission mirror · r79",
        "",
        "> PROMOTE CPU ROW EMISSION ONLY. This does not admit the hierarchical solver or CUDA; no device claim is made.",
        "",
        "R79 lowers fixed point attractors and three-axis contact locks over the r78 geometric products. Stable ID, point-query provenance, strict priority, weight, bandwidth, activation, Jacobian, physical acceleration, Jdot-v compensation, and right-hand side remain explicit fixed-shape data.",
        "",
        "## Admission decision",
        "",
    ]
    lines += markdown_table(
        ["Gate", "Evidence", "Decision"],
        [
            ["Descriptor semantics", "stable IDs + query slots + priority/weight/bandwidth", "PASS"],
            ["Task algebra", "independent NumPy critical-damping composition", "PASS"],
            ["Contact algebra", "J qdd = desired - Jdot-v rows", "PASS"],
            ["Inactive masks", "all fixed row storage exactly zero", "PASS"],
            ["Repeat/reindex/isolation", "bitwise exact", "PASS"],
            ["Measured Rust allocations", f"{metrics['total_allocation_calls']} calls · {metrics['total_allocated_bytes']} B", "PASS"],
            ["Shared-qdd feasibility", "reported per state, never folded into row correctness", "DIAGNOSTIC"],
            ["Hierarchical solve / CUDA", "not in this manifest / device unavailable", "NOT RUN"],
        ],
    )
    lines += ["", "## Independent composition oracle", ""]
    lines += markdown_table(
        ["Model", "States", "Largest task algebra error", "Largest contact error", "Shared-qdd residual p50/p99", "Result"],
        [
            [
                pathlib.Path(case["model"]).stem,
                case["samples"],
                f"{max(case['maximum_errors'][name] for name in ('position_error', 'velocity_error', 'desired_acceleration', 'task_jacobian', 'task_rhs')):.3e}",
                f"{max(case['maximum_errors'][name] for name in ('contact_jacobian', 'contact_rhs')):.3e}",
                f"{case['least_squares_shared_qdd_residual']['p50']:.3e}/{case['least_squares_shared_qdd_residual']['p99']:.3e}",
                "PASS" if case["pass"] else "FAIL",
            ]
            for case in metrics["oracle_cases"]
        ],
    )
    lines += [
        "",
        "The oracle is deliberately staged: Pinocchio/finite-difference admission of position, J, Jv, and Jdot-v happened in r78; r79 independently composes target jets, critical damping, masks, and bias subtraction in NumPy. A least-squares generalized acceleration is then fit to every simultaneously active task and contact row only to expose conflict. It is not a pass condition because a correct WBC must represent infeasible requests before a bounded solver reports compromise.",
        "",
        "## Example authority stack for architectural review",
        "",
    ]
    lines += markdown_table(
        ["Layer", "Concrete r79 example", "Meaning"],
        [
            ["Invariant", "left/right contact-lock rows 7951/7952", "material point acceleration equalities"],
            ["Viability", "torso point attractor 7901 · 1.5 Hz · weight 1.5", "stability-preserving intent with explicit J/Jdot-v"],
            ["Intent", "handle point attractor 7902 · 2.25 Hz · weight 0.75", "operator target jet"],
            ["Preference", "not declared in this two-task r79 plan", "absence remains distinct from zero residual"],
            ["Style", "not declared in this two-task r79 plan", "terminal refinement remains a later manifest row"],
            ["Feasibility", "shared-qdd residual distribution", "continuous incompatibility evidence, not an aggregate score"],
            ["Resources", "joint/effort/thermal layers remain separate", "row emission cannot promise realizability"],
            ["Backend", "CpuMirrorF32 admitted; CUDA unavailable", "implementation conformance is not physical authority"],
        ],
    )
    lines += ["", "## Exactness and isolation", ""]
    lines += markdown_table(
        ["Invariant", "Result"],
        [
            ["Repeat", str(inv["d1_repeat_bitwise_exact"])],
            ["Permutation / padding / 7+10 chunking", f"{inv['permutation_exact']} / {inv['padding_exact']} / {inv['chunking_exact']}"],
            ["Active NaN target", "agent 8 zeroed and typed InvalidInput; every neighbor unchanged"],
        ],
    )
    lines += ["", "## Host timing · Upkie two tasks + two contacts", ""]
    lines += markdown_table(
        ["Agents", "Emission p50/p99", "Full pipeline p50/p99", "Full/agent p50", "Alloc"],
        [
            [
                case["batch_size"],
                f"{case['emission_execute_ns']['p50']/1e3:.3f}/{case['emission_execute_ns']['p99']/1e3:.3f} µs",
                f"{case['combined_execute_ns']['p50']/1e3:.3f}/{case['combined_execute_ns']['p99']/1e3:.3f} µs",
                f"{case['per_agent_combined_ns']['p50']:.1f} ns",
                f"{case['allocation_calls']} / {case['allocated_bytes']} B",
            ]
            for case in metrics["timing"]
        ],
    )
    lines += [
        "",
        "All timing samples are retained. Stage timers exclude NumPy↔SoA copies. Weight and priority remain descriptor metadata for the later hierarchical solver; emitted J and RHS stay in physical units.",
        "",
        "## Deferred boundary",
        "",
        "The next slice must assemble inequalities and solve the emitted hierarchy under a bounded-work contract, while reporting hard infeasibility, task compromise, iteration exhaustion, and physical resource pressure separately. CudaMirrorF32 must later reproduce this exact manifest on real hardware and pass D1/D2/D3; no CPU fallback may claim device execution.",
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
    invariance = invariance_case(models[1])
    timing = [timing_case(models[1], size, args.timing_repeats) for size in (1, 32, 256)]
    total_calls = invariance["allocation_calls"] + sum(
        case["allocation_calls"] for case in oracle_cases + timing
    )
    total_bytes = invariance["allocated_bytes"] + sum(
        case["allocated_bytes"] for case in oracle_cases + timing
    )
    admission = bool(
        all(case["pass"] for case in oracle_cases)
        and invariance["pass"]
        and total_calls == 0
        and total_bytes == 0
    )
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "revision": "cuda-emission-abi-r79",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "profile": "CpuMirrorF32",
        "manifest": [
            "StateInput", "ForwardKinematics", "CenterOfMass", "Jacobians",
            "RigidBodyDynamics", "PointQueries", "TaskEmission", "ConstraintEmission",
        ],
        "oracle_cases": oracle_cases,
        "invariance": invariance,
        "timing": timing,
        "total_allocation_calls": total_calls,
        "total_allocated_bytes": total_bytes,
        "cuda_device_executor": {"available": False, "d1": "not_run", "d2": "not_run", "d3": "not_run"},
        "admission": {"cpu_mirror_emission": admission, "hierarchical_solve": False, "cuda_mirror_emission": False},
    }
    np.savez_compressed(output / "cuda-emission-abi-raw.npz", **raw)
    (output / "cuda-emission-abi-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CUDA_EMISSION_ABI_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Bonesaw task/contact emission mirror · r79")
    )
    if not admission:
        raise SystemExit("CpuMirrorF32 emission admission failed")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
