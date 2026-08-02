#!/usr/bin/env python3
"""Admit the first fixed-layout CPU-exact dynamic batch solve.

The Rust decision vector is [qdd, generalized joint effort, point-contact
force].  Python is orchestration and an independent Pinocchio/NumPy oracle;
there is no policy and no physics rollout in this checkpoint.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import joint_states, sha256
from cuda_dynamics_mirror_report import pinocchio_products
from cuda_emission_mirror_report import (
    CONTACT_QUERY_SLOTS,
    CONTACT_STABLE_IDS,
    TASK_STABLE_IDS,
    session_for,
    subset_targets,
)
from cuda_point_query_mirror_report import pin_point_products, run as run_points


INACTIVE = 0
SOLVED = 1
SOLVED_WITH_SLACK = 2
MAX_ITERATIONS = 3
PRIMAL_INFEASIBLE = 4
NUMERICAL_FAILURE = 5
INVALID_PROBLEM = 6
INVALID_INPUT = 7
SOLVED_STATUSES = (SOLVED, SOLVED_WITH_SLACK)
STATUS_NAMES = {
    INACTIVE: "Inactive",
    SOLVED: "Solved",
    SOLVED_WITH_SLACK: "SolvedWithSlack",
    MAX_ITERATIONS: "MaxIterations",
    PRIMAL_INFEASIBLE: "PrimalInfeasible",
    NUMERICAL_FAILURE: "NumericalFailure",
    INVALID_PROBLEM: "InvalidProblem",
    INVALID_INPUT: "InvalidInput",
}
EXACT_FIELDS = (
    "qdd",
    "effort",
    "contact_force_basis",
    "contact_force_world",
    "status",
    "active_contacts",
    "dynamics_residual",
    "contact_residual",
    "minimum_friction_margin",
    "minimum_support_margin",
    "limiting_support_patch",
    "active_support_patches",
    "minimum_actuator_effort_margin",
    "limiting_actuator",
    "minimum_bound_margin",
    "maximum_hard_violation",
    "clipped_steps",
    "task_l2",
    "task_clipped",
)
ENVELOPE_FIELDS = (
    "joint_envelope_lower",
    "joint_envelope_upper",
    "joint_envelope_status",
    "joint_position_headroom",
    "joint_velocity_headroom",
    "limiting_joint_position",
    "limiting_joint_velocity",
    "joint_recovery_count",
    "minimum_joint_envelope_margin",
    "limiting_joint_envelope_coordinate",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=60)
    parser.add_argument("--output", default="benchmarks/results/cpu-exact-dynamic-batch-r82")
    parser.add_argument("--web-report", default="web/CPU_EXACT_DYNAMIC_BATCH_R82.html")
    return parser.parse_args()


def urdf_limits_and_mass(model_path: pathlib.Path, joint_names: list[str]) -> tuple[np.ndarray, float]:
    root = ET.parse(model_path).getroot()
    efforts: dict[str, float] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") and limit is not None and limit.get("effort") is not None:
            efforts[joint.get("name", "")] = float(limit.get("effort", "nan"))
    ordered = np.asarray([efforts[name] for name in joint_names], dtype=np.float64)
    masses = [float(node.get("value", "0")) for node in root.findall("./link/inertial/mass")]
    return ordered, float(sum(masses))


def dynamic_buffers(session) -> dict[str, np.ndarray]:
    a, g, d = session.agent_capacity, session.generalized_dof, session.dof
    t, c = session.point_task_count, session.contact_lock_count
    p = session.support_patch_count
    out = {
        "qdd": np.empty((a, g), np.float64),
        "effort": np.empty((a, d), np.float64),
        "contact_force_basis": np.empty((a, c, 3), np.float64),
        "contact_force_world": np.empty((a, c, 3), np.float64),
        "status": np.empty(a, np.uint8),
        "active_contacts": np.empty(a, np.uint32),
        "dynamics_residual": np.empty(a, np.float64),
        "contact_residual": np.empty(a, np.float64),
        "minimum_friction_margin": np.empty(a, np.float64),
        "minimum_support_margin": np.empty(a, np.float64),
        "limiting_support_patch": np.empty(a, np.uint32),
        "active_support_patches": np.empty(a, np.uint32),
        "minimum_actuator_effort_margin": np.empty(a, np.float64),
        "limiting_actuator": np.empty(a, np.uint32),
        "minimum_bound_margin": np.empty(a, np.float64),
        "maximum_hard_violation": np.empty(a, np.float64),
        "clipped_steps": np.empty(a, np.uint32),
        "task_l2": np.empty((a, t), np.float64),
        "task_clipped": np.empty((a, t), np.uint8),
        "joint_envelope_lower": np.empty((a, g), np.float64),
        "joint_envelope_upper": np.empty((a, g), np.float64),
        "joint_envelope_status": np.empty(a, np.uint8),
        "joint_position_headroom": np.empty(a, np.float64),
        "joint_velocity_headroom": np.empty(a, np.float64),
        "limiting_joint_position": np.empty(a, np.uint32),
        "limiting_joint_velocity": np.empty(a, np.uint32),
        "joint_recovery_count": np.empty(a, np.uint32),
        "minimum_joint_envelope_margin": np.empty(a, np.float64),
        "limiting_joint_envelope_coordinate": np.empty(a, np.uint32),
    }
    for name in ("joint_envelope", "fk", "jacobian", "dynamics", "point", "emission", "solve"):
        out[f"{name}_execute_ns"] = np.empty(1, np.uint64)
    out["allocation_calls"] = np.empty(1, np.uint64)
    out["allocated_bytes"] = np.empty(1, np.uint64)
    return out


def run_dynamic(session, case: dict[str, np.ndarray], buffers=None) -> dict[str, np.ndarray]:
    out = dynamic_buffers(session) if buffers is None else buffers
    session.run_exact_dynamic_batch(
        np.ascontiguousarray(case["q"], np.float32),
        np.ascontiguousarray(case["roots"], np.float32),
        np.ascontiguousarray(case["velocity"], np.float32),
        np.ascontiguousarray(case["gravity"], np.float32),
        np.ascontiguousarray(case["task_active"], np.uint8),
        np.ascontiguousarray(case["target_position"], np.float32),
        np.ascontiguousarray(case["target_velocity"], np.float32),
        np.ascontiguousarray(case["target_acceleration"], np.float32),
        np.ascontiguousarray(case["contact_active"], np.uint8),
        np.ascontiguousarray(case["contact_desired_acceleration"], np.float32),
        np.ascontiguousarray(case["qdd_lower"], np.float64),
        np.ascontiguousarray(case["qdd_upper"], np.float64),
        np.ascontiguousarray(case["effort_lower"], np.float64),
        np.ascontiguousarray(case["effort_upper"], np.float64),
        np.ascontiguousarray(case["friction"], np.float64),
        np.ascontiguousarray(case["minimum_normal_force"], np.float64),
        np.ascontiguousarray(case["maximum_normal_force"], np.float64),
        np.ascontiguousarray(case["nominal_normal_force"], np.float64),
        np.ascontiguousarray(case["joint_envelope_enabled"], np.uint8),
        np.ascontiguousarray(case["joint_envelope_dt_seconds"], np.float64),
        np.ascontiguousarray(case["joint_maximum_acceleration"], np.float64),
        np.ascontiguousarray(case["support_patch_enabled"], np.uint8),
        np.ascontiguousarray(case["support_patch_minimum_margin"], np.float64),
        *[out[name] for name in EXACT_FIELDS],
        *[out[name] for name in ENVELOPE_FIELDS],
        out["joint_envelope_execute_ns"],
        out["fk_execute_ns"], out["jacobian_execute_ns"], out["dynamics_execute_ns"],
        out["point_execute_ns"], out["emission_execute_ns"], out["solve_execute_ns"],
        out["allocation_calls"], out["allocated_bytes"],
    )
    return out


def make_case(model_path: pathlib.Path, agents: int, *, alignment: int = 32) -> tuple[Any, dict[str, np.ndarray], float]:
    session = session_for(model_path, agents, alignment)
    q = (0.18 * joint_states(agents, session.dof)).astype(np.float32)
    roots = np.zeros((agents, 12), np.float32)
    roots[:, [0, 4, 8]] = 1.0
    roots[:, 11] = 0.62
    velocity = np.zeros((agents, session.generalized_dof), np.float32)
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81], np.float32), (agents, 1))
    points = run_points(session, q, roots, velocity, gravity)
    target_position = points["position"][:, [0, 3]].copy()
    target_position[:, 0, 2] += 0.012
    target_position[:, 1, 0] += 0.008
    effort, mass = urdf_limits_and_mass(model_path, list(session.joint_names))
    nominal = max(mass * 9.81 / 2.0, 1.0)
    case = {
        "q": q,
        "roots": roots,
        "velocity": velocity,
        "gravity": gravity,
        "task_active": np.ones((agents, session.point_task_count), np.uint8),
        "target_position": target_position,
        "target_velocity": np.zeros((agents, session.point_task_count, 3), np.float32),
        "target_acceleration": np.zeros((agents, session.point_task_count, 3), np.float32),
        "contact_active": np.ones((agents, session.contact_lock_count), np.uint8),
        "contact_desired_acceleration": np.zeros((agents, session.contact_lock_count, 3), np.float32),
        "qdd_lower": np.full((agents, session.generalized_dof), -120.0, np.float64),
        "qdd_upper": np.full((agents, session.generalized_dof), 120.0, np.float64),
        "effort_lower": np.tile(-effort, (agents, 1)),
        "effort_upper": np.tile(effort, (agents, 1)),
        "friction": np.full((agents, session.contact_lock_count), 0.8, np.float64),
        "minimum_normal_force": np.zeros((agents, session.contact_lock_count), np.float64),
        "maximum_normal_force": np.full((agents, session.contact_lock_count), max(1000.0, nominal * 10.0), np.float64),
        "nominal_normal_force": np.full((agents, session.contact_lock_count), nominal, np.float64),
        "joint_envelope_enabled": np.zeros(agents, np.uint8),
        "joint_envelope_dt_seconds": np.full(agents, 0.02, np.float64),
        "joint_maximum_acceleration": np.full((agents, session.dof), np.inf, np.float64),
        "support_patch_enabled": np.zeros((agents, session.support_patch_count), np.uint8),
        "support_patch_minimum_margin": np.zeros((agents, session.support_patch_count), np.float64),
    }
    return session, case, mass


def copy_case(case: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {name: value.copy() for name, value in case.items()}


def pinocchio_hard_residuals(
    model_path: pathlib.Path, session, case: dict[str, np.ndarray], result: dict[str, np.ndarray]
) -> tuple[float, float]:
    # The ABI deliberately admits only the exact fixed world gravity.  Build
    # that f64 value directly for the oracle rather than widening the f32 ABI
    # spelling (-9.810000419...), which would create mass-scaled false error.
    oracle_gravity = np.tile(np.asarray([0.0, 0.0, -9.81], np.float64), (len(case["q"]), 1))
    mass, bias, _ = pinocchio_products(
        model_path, list(session.joint_names), case["q"], case["roots"],
        case["velocity"], oracle_gravity,
    )
    _, point_jacobian = pin_point_products(
        model_path, list(session.joint_names), list(session.point_frame_names),
        np.asarray(session.point_offsets, np.float64), case["q"], case["roots"],
    )
    dynamics_linf = 0.0
    contact_linf = 0.0
    for agent in range(len(case["q"])):
        if int(result["status"][agent]) not in SOLVED_STATUSES:
            continue
        rhs = np.zeros(session.generalized_dof, np.float64)
        rhs[6:] = result["effort"][agent]
        for contact_slot, query_slot in enumerate(CONTACT_QUERY_SLOTS):
            if case["contact_active"][agent, contact_slot]:
                jacobian = point_jacobian[agent, query_slot]
                rhs += jacobian.T @ result["contact_force_world"][agent, contact_slot]
                contact = jacobian @ result["qdd"][agent] - case["contact_desired_acceleration"][agent, contact_slot]
                contact_linf = max(contact_linf, float(np.max(np.abs(contact))))
        dynamics = mass[agent] @ result["qdd"][agent] + bias[agent] - rhs
        dynamics_linf = max(dynamics_linf, float(np.max(np.abs(dynamics))))
    return dynamics_linf, contact_linf


def oracle_case(model_path: pathlib.Path, samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session, case, mass = make_case(model_path, samples)
    result = run_dynamic(session, case)
    independent_dynamics, independent_contact = pinocchio_hard_residuals(model_path, session, case, result)
    basis = result["contact_force_basis"]
    normal = basis[:, :, 2]
    tangent = np.linalg.norm(basis[:, :, :2], axis=2)
    cone_margin = case["friction"] * normal - tangent
    effort_margin = np.minimum(result["effort"] - case["effort_lower"], case["effort_upper"] - result["effort"])
    solved = bool(np.all(np.isin(result["status"], SOLVED_STATUSES)))
    passed = bool(
        solved
        and independent_dynamics <= 2e-7
        and independent_contact <= 2e-8
        and float(np.min(normal)) >= -1e-9
        and float(np.min(cone_margin)) >= -2e-8
        and float(np.min(effort_margin)) >= -2e-8
        and result["allocation_calls"][0] == 0
        and result["allocated_bytes"][0] == 0
    )
    metrics = {
        "model": str(model_path), "model_sha256": sha256(model_path), "samples": samples,
        "total_mass_kg": mass, "status_counts": status_counts(result["status"]),
        "pinocchio_dynamics_linf": independent_dynamics,
        "pinocchio_contact_acceleration_linf": independent_contact,
        "reported_dynamics_linf": float(np.max(result["dynamics_residual"])),
        "reported_contact_linf": float(np.max(result["contact_residual"])),
        "minimum_recomputed_friction_margin": float(np.min(cone_margin)),
        "minimum_recomputed_effort_margin_nm": float(np.min(effort_margin)),
        "allocation_calls": int(result["allocation_calls"][0]),
        "allocated_bytes": int(result["allocated_bytes"][0]), "pass": passed,
    }
    stem = model_path.stem
    raw = {f"{stem}_qdd": result["qdd"].copy(), f"{stem}_effort": result["effort"].copy(),
           f"{stem}_contact_force_world": result["contact_force_world"].copy()}
    return metrics, raw


def status_counts(status: np.ndarray) -> dict[str, int]:
    return {STATUS_NAMES[int(value)]: int(np.sum(status == value)) for value in np.unique(status)}


def resource_sweep(model_path: pathlib.Path) -> dict[str, Any]:
    session, base, _ = make_case(model_path, 8)
    friction_rows = []
    for mu in (1.0, 0.5, 0.2, 0.05):
        case = copy_case(base); case["friction"].fill(mu)
        case["target_position"][:, 1, 0] += 0.08
        result = run_dynamic(session, case)
        friction_rows.append({"mu": mu, "status_counts": status_counts(result["status"]),
                              "minimum_margin": float(np.min(result["minimum_friction_margin"])),
                              "intent_task_l2": float(np.max(result["task_l2"][:, 1]))})
    effort_rows = []
    for scale in (1.0, 0.6, 0.35, 0.2):
        case = copy_case(base)
        case["effort_lower"] *= scale; case["effort_upper"] *= scale
        case["target_position"][:, 0, 2] += 0.10
        result = run_dynamic(session, case)
        effort_rows.append({"scale": scale, "status_counts": status_counts(result["status"]),
                            "minimum_margin_nm": float(np.min(result["minimum_actuator_effort_margin"])),
                            "viability_task_l2": float(np.max(result["task_l2"][:, 0]))})
    finite = all(np.isfinite(row["intent_task_l2"]) and row["minimum_margin"] >= -2e-8 for row in friction_rows)
    finite &= all(np.isfinite(row["viability_task_l2"]) and row["minimum_margin_nm"] >= -2e-8 for row in effort_rows)
    return {"friction": friction_rows, "effort_derating": effort_rows, "separate_continuous_signals": True,
            "pass": bool(finite)}


def failure_and_isolation(model_path: pathlib.Path) -> dict[str, Any]:
    session, base, _ = make_case(model_path, 17)
    baseline = {name: value.copy() for name, value in run_dynamic(session, base).items()}
    invalid_case = copy_case(base); invalid_case["gravity"][8, 2] = -9.8
    invalid = run_dynamic(session, invalid_case)
    neighbors = np.arange(17) != 8
    isolated = bool(invalid["status"][8] == INVALID_INPUT and np.all(invalid["qdd"][8] == 0.0)
                    and all(np.array_equal(invalid[name][neighbors], baseline[name][neighbors]) for name in EXACT_FIELDS))
    unsupported = copy_case(base)
    unsupported["maximum_normal_force"].fill(0.0)
    unsupported["nominal_normal_force"].fill(0.0)
    failed = run_dynamic(session, unsupported)
    failure_typed = bool(np.all(np.isin(failed["status"], [MAX_ITERATIONS, PRIMAL_INFEASIBLE])))
    zeroed = bool(np.all(failed["qdd"] == 0.0) and np.all(failed["effort"] == 0.0)
                  and np.all(failed["contact_force_world"] == 0.0))
    return {"invalid_agent": 8, "invalid_status": STATUS_NAMES[int(invalid["status"][8])],
            "invalid_status_code": int(invalid["status"][8]),
            "neighbor_outputs_bitwise_exact": isolated,
            "zero_normal_authority_status_counts": status_counts(failed["status"]),
            "failure_typed": failure_typed, "failed_commands_zeroed": zeroed,
            "pass": bool(isolated and failure_typed and zeroed)}


def invariance_case(model_path: pathlib.Path) -> dict[str, Any]:
    agents = 17
    session, base, _ = make_case(model_path, agents)
    first = {name: value.copy() for name, value in run_dynamic(session, base).items()}
    repeat = run_dynamic(session, base)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)])
    perm_case = {name: value[permutation] for name, value in base.items()}
    permuted = run_dynamic(session, perm_case); inverse = np.argsort(permutation)
    compact_session, compact_case, _ = make_case(model_path, agents, alignment=1)
    compact = run_dynamic(compact_session, compact_case)
    fields = EXACT_FIELDS + ENVELOPE_FIELDS
    repeat_exact = all(np.array_equal(first[name], repeat[name]) for name in fields)
    permutation_exact = all(np.array_equal(first[name], permuted[name][inverse]) for name in fields)
    padding_exact = all(np.array_equal(first[name], compact[name]) for name in fields)
    allocations = sum(int(x["allocation_calls"][0]) for x in (first, repeat, permuted, compact))
    return {"repeat_bitwise_exact": repeat_exact, "permutation_exact": permutation_exact,
            "padding_exact": padding_exact, "allocation_calls": allocations,
            "pass": bool(repeat_exact and permutation_exact and padding_exact and allocations == 0)}


def timing_case(model_path: pathlib.Path, agents: int, repeats: int) -> dict[str, Any]:
    session, case, _ = make_case(model_path, agents)
    buffers = dynamic_buffers(session)
    for _ in range(4):
        run_dynamic(session, case, buffers)
    stages = ("joint_envelope", "fk", "jacobian", "dynamics", "point", "emission", "solve")
    traces = {stage: np.empty(repeats, np.float64) for stage in stages}
    calls = allocated = 0
    for index in range(repeats):
        result = run_dynamic(session, case, buffers)
        for stage in stages:
            traces[stage][index] = result[f"{stage}_execute_ns"][0]
        calls += int(result["allocation_calls"][0]); allocated += int(result["allocated_bytes"][0])
    total = sum(traces.values())
    return {"batch_size": agents, "repeats": repeats,
            "solve_execute_ns": distribution(traces["solve"]),
            "full_pipeline_ns": distribution(total),
            "per_agent_full_pipeline_ns": {key: value / agents for key, value in distribution(total).items()},
            "allocation_calls": calls, "allocated_bytes": allocated}


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle_rows = [[pathlib.Path(row["model"]).stem, row["samples"], f'{row["pinocchio_dynamics_linf"]:.3e}',
                    f'{row["pinocchio_contact_acceleration_linf"]:.3e}',
                    f'{row["minimum_recomputed_friction_margin"]:.3e}',
                    f'{row["minimum_recomputed_effort_margin_nm"]:.3e}', row["pass"]]
                   for row in metrics["oracle_cases"]]
    friction_rows = [[row["mu"], row["status_counts"], f'{row["minimum_margin"]:.3e}',
                      f'{row["intent_task_l2"]:.3e}'] for row in metrics["resource_sweep"]["friction"]]
    effort_rows = [[row["scale"], row["status_counts"], f'{row["minimum_margin_nm"]:.3e}',
                    f'{row["viability_task_l2"]:.3e}'] for row in metrics["resource_sweep"]["effort_derating"]]
    timing_rows = [[row["batch_size"], f'{row["solve_execute_ns"]["p50"] / 1e3:.3f}',
                    f'{row["solve_execute_ns"]["p99"] / 1e3:.3f}',
                    f'{row["full_pipeline_ns"]["p50"] / 1e3:.3f}', row["allocation_calls"]]
                   for row in metrics["timing"]]
    authority = [
        ["Invariant", "contact locks 7951 / 7952", "hard 3-axis point acceleration equalities"],
        ["Invariant", "floating dynamics", "M qdd + h = Sᵀ effort + Jᶜᵀ force"],
        ["Viability", "torso point 7901 + r81 stopping box", "protected task residual and joint-limit authority remain separate"],
        ["Intent", "handle point 7902", "yields before Viability and retains its own residual"],
        ["Contact resource", "normal-force and Coulomb-cone margins", "continuous force authority; finite only for declared point contacts"],
        ["Actuator resource", "per-actuator effort margin + limiting actuator", "continuous effort headroom; no thermal inference"],
        ["Recovery", "typed MaxIterations / PrimalInfeasible", "bounded search exhaustion is not relabeled as a proof"],
        ["Solver budget", "clipped steps + hard violation", "numerical pressure stays distinct from physical headroom"],
        ["Backend", "CpuExactF64", "allocation-free CPU semantic reference; CUDA solve unavailable"],
    ]
    return "\n".join([
        "# Bonesaw CPU-exact dynamic batch · r82", "", "## Outcome", "",
        f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** The fixed Rust batch decision vector is now `[qdd, generalized joint effort, point-contact force]`. It enforces floating rigid-body dynamics, locked-point acceleration, actuator effort bounds, unilateral normal force, and Coulomb friction before optimizing emitted point tasks. Python supplies evaluation and an independent Pinocchio/NumPy hard-equation oracle; no policy or physics rollout is used.', "",
        "## Example authority stack", "",
        "This is the concrete architectural review stack. Rows are ordered by authority but their witnesses are never collapsed into one score.", "",
        *markdown_table(["layer", "example authority", "separate witness"], authority), "",
        "## Independent Pinocchio oracle", "",
        *markdown_table(["model", "states", "dynamics L∞", "contact L∞", "friction margin", "effort margin Nm", "pass"], oracle_rows), "",
        "The oracle rebuilds mass, bias, and contact Jacobians from URDF in Pinocchio and recomputes the equations from returned commands. Rust's own residual fields are not used as the admission witness.", "",
        "## Contact authority sweep", "",
        *markdown_table(["friction μ", "statuses", "min cone margin", "Intent residual"], friction_rows), "",
        "## Actuator derating sweep", "",
        *markdown_table(["effort scale", "statuses", "min effort margin Nm", "Viability residual"], effort_rows), "",
        "These sweeps intentionally expose friction margin, actuator margin, and tracking residual separately. A physically bounded solve may sacrifice a task; `SolvedWithSlack` is useful output, not a false pose guarantee.", "",
        "## Failure typing and isolation", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["failure_and_isolation"].items()]), "",
        "## Determinism", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]), "",
        "## Release timing (untrimmed)", "",
        *markdown_table(["agents", "dynamic solve p50 us", "dynamic solve p99 us", "pipeline p50 us", "alloc calls"], timing_rows), "",
        "Timers cover preallocated Rust stages and exclude NumPy marshalling. Every sampled call is retained.", "",
        "## Deliberate scope boundary", "",
        "R82 admits horizontal point contacts only. Finite support patches, center-of-pressure/CoM polygon inequalities, contact switching, power, thermal state, actuator realization, walking rollout, and a CUDA hierarchical solver remain unavailable. Gravity is currently the explicit fixed world vector `[0, 0, -9.81]`; other gravity inputs are typed `InvalidInput`, not silently accepted. The existing r81 joint stopping envelope is architecturally upstream but is not yet composed into this dynamic Python entry point.", "",
    ])


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output); output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    oracle_cases, raw = [], {}
    for model in models:
        result, arrays = oracle_case(model, args.samples)
        oracle_cases.append(result); raw.update(arrays)
    resource = resource_sweep(models[1])
    failure = failure_and_isolation(models[1])
    invariance = invariance_case(models[1])
    timing = [timing_case(models[1], agents, args.timing_repeats) for agents in (1, 8, 32)]
    admission = (all(row["pass"] for row in oracle_cases) and resource["pass"]
                 and failure["pass"] and invariance["pass"]
                 and all(row["allocation_calls"] == 0 and row["allocated_bytes"] == 0 for row in timing))
    metrics = {"schema_version": 1, "revision": "cpu-exact-dynamic-batch-r82",
               "generated_at": datetime.now(timezone.utc).isoformat(),
               "host": {"platform": platform.platform(), "processor": platform.processor()},
               "decision_vector": ["qdd", "generalized_joint_effort", "point_contact_force"],
               "policy": None, "physics_rollout": None, "oracle_cases": oracle_cases,
               "resource_sweep": resource, "failure_and_isolation": failure,
               "invariance": invariance, "timing": timing,
               "support_patch": {"available": False, "claimed": False},
               "cuda_hierarchical_solver": {"available": False, "claimed": False},
               "admission": bool(admission)}
    np.savez_compressed(output / "cpu-exact-dynamic-batch-raw.npz", **raw)
    (output / "cpu-exact-dynamic-batch-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CPU_EXACT_DYNAMIC_BATCH_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw CPU-exact dynamic batch · r82"))
    print(json.dumps(metrics, indent=2))
    if not admission:
        raise SystemExit("CPU-exact dynamic batch admission failed")


if __name__ == "__main__":
    main()
