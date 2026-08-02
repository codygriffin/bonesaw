#!/usr/bin/env python3
"""Admit fixed finite-support declarations and hard CoP inequalities.

Rust owns the fixed topology, hull construction, hard rows, solve, and hot-loop
storage. Python owns fixtures, independent Pinocchio/NumPy oracles, timing, and
the report. No policy and no physics rollout are used.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np
from bonesaw import CpuMirrorBatchSession

from cpu_exact_dynamic_batch_report import (
    EXACT_FIELDS,
    INVALID_INPUT,
    SOLVED_STATUSES,
    STATUS_NAMES,
    copy_case,
    distribution,
    dynamic_buffers,
    markdown_table,
    render_report_html,
    run_dynamic,
    status_counts,
    urdf_limits_and_mass,
)
from cuda_batch_mirror_report import sha256
from cuda_dynamics_mirror_report import pinocchio_products
from cuda_emission_mirror_report import (
    CONTACT_QUERY_SLOTS,
    CONTACT_STABLE_IDS,
    TASK_BANDWIDTH_HZ,
    TASK_PRIORITIES,
    TASK_QUERY_SLOTS,
    TASK_STABLE_IDS,
    TASK_WEIGHTS,
)
from cuda_point_query_mirror_report import pin_point_products, query_plan, run as run_points


MODEL = pathlib.Path("models/toy_humanoid.urdf")
PATCH_STABLE_ID = 8401
PATCH_MARGIN_M = 0.02
UINT32_MAX = np.iinfo(np.uint32).max
POINT_FRAMES = ["pelvis", *(["left_foot"] * 4), "right_foot"]
POINT_OFFSETS = [
    (0.0, 0.0, 0.0),
    (-0.08, -0.04, -0.07),
    (-0.08, 0.04, -0.07),
    (0.12, -0.04, -0.07),
    (0.12, 0.04, -0.07),
    (0.02, 0.0, -0.07),
]
POINT_IDS = [8001, 8101, 8102, 8103, 8104, 8105]
CONTACT_SLOTS = [1, 2, 3, 4, 5]
CONTACT_IDS = [8301, 8302, 8303, 8304, 8305]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=60)
    parser.add_argument("--output", default="benchmarks/results/finite-support-patch-r84")
    parser.add_argument("--web-report", default="web/FINITE_SUPPORT_PATCH_R84.html")
    return parser.parse_args()


def session_for(
    agents: int,
    alignment: int = 32,
    contact_kinematic_modes: list[int] | None = None,
    auto_rigid_support_basis: bool = False,
) -> CpuMirrorBatchSession:
    return CpuMirrorBatchSession(
        str(MODEL),
        agents,
        alignment,
        POINT_FRAMES,
        POINT_OFFSETS,
        POINT_IDS,
        point_task_query_slots=[0],
        point_task_stable_ids=[8201],
        point_task_priorities=[2],
        point_task_weights=[1.0],
        point_task_bandwidth_hz=[1.5],
        contact_query_slots=CONTACT_SLOTS,
        contact_stable_ids=CONTACT_IDS,
        contact_kinematic_modes=contact_kinematic_modes,
        support_patch_first_contact_slots=[0],
        support_patch_contact_counts=[4],
        support_patch_stable_ids=[PATCH_STABLE_ID],
        auto_rigid_support_basis=auto_rigid_support_basis,
    )


def make_case(
    agents: int,
    alignment: int = 32,
    contact_kinematic_modes: list[int] | None = None,
    auto_rigid_support_basis: bool = False,
) -> tuple[CpuMirrorBatchSession, dict[str, np.ndarray]]:
    session = session_for(
        agents,
        alignment,
        contact_kinematic_modes,
        auto_rigid_support_basis,
    )
    q = np.zeros((agents, session.dof), np.float32)
    roots = np.zeros((agents, 12), np.float32)
    roots[:, [0, 4, 8]] = 1.0
    roots[:, 11] = 0.98
    velocity = np.zeros((agents, session.generalized_dof), np.float32)
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81], np.float32), (agents, 1))
    points = run_points(session, q, roots, velocity, gravity)
    target = points["position"][:, [0]].copy()
    target[:, 0, 1] += np.linspace(0.0, 0.1875, agents, dtype=np.float32)
    effort, mass = urdf_limits_and_mass(MODEL, list(session.joint_names))
    nominal = mass * 9.81 / 4.0
    case = {
        "q": q,
        "roots": roots,
        "velocity": velocity,
        "gravity": gravity,
        "task_active": np.ones((agents, 1), np.uint8),
        "target_position": target,
        "target_velocity": np.zeros((agents, 1, 3), np.float32),
        "target_acceleration": np.zeros((agents, 1, 3), np.float32),
        "contact_active": np.tile(np.asarray([1, 1, 1, 1, 0], np.uint8), (agents, 1)),
        "contact_desired_acceleration": np.zeros((agents, 5, 3), np.float32),
        "qdd_lower": np.full((agents, session.generalized_dof), -200.0, np.float64),
        "qdd_upper": np.full((agents, session.generalized_dof), 200.0, np.float64),
        "effort_lower": np.tile(-effort, (agents, 1)),
        "effort_upper": np.tile(effort, (agents, 1)),
        "friction": np.full((agents, 5), 0.8, np.float64),
        "minimum_normal_force": np.zeros((agents, 5), np.float64),
        "maximum_normal_force": np.full((agents, 5), 2_000.0, np.float64),
        "nominal_normal_force": np.column_stack(
            [np.full((agents, 4), nominal, np.float64), np.zeros(agents, np.float64)]
        ),
        "joint_envelope_enabled": np.zeros(agents, np.uint8),
        "joint_envelope_dt_seconds": np.full(agents, 0.02, np.float64),
        "joint_maximum_acceleration": np.full((agents, session.dof), np.inf, np.float64),
        "support_patch_enabled": np.ones((agents, 1), np.uint8),
        "support_patch_minimum_margin": np.full((agents, 1), PATCH_MARGIN_M, np.float64),
    }
    return session, case


def convex_hull(points: np.ndarray) -> np.ndarray:
    ordered = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(ordered) < 3:
        raise ValueError("finite support requires three distinct points")

    def cross(origin, a, b) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-12:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-12:
            upper.pop()
        upper.append(point)
    hull = np.asarray(lower[:-1] + upper[:-1], np.float64)
    if len(hull) < 3:
        raise ValueError("support declaration is collinear")
    return hull


def polygon_margin(hull: np.ndarray, point: np.ndarray) -> float:
    margins = []
    for index, start in enumerate(hull):
        edge = hull[(index + 1) % len(hull)] - start
        offset = point - start
        margins.append(float((edge[0] * offset[1] - edge[1] * offset[0]) / np.linalg.norm(edge)))
    return min(margins)


def independent_oracle(
    session: CpuMirrorBatchSession,
    case: dict[str, np.ndarray],
    result: dict[str, np.ndarray],
) -> dict[str, Any]:
    gravity = np.tile(np.asarray([0.0, 0.0, -9.81], np.float64), (len(case["q"]), 1))
    mass, bias, _ = pinocchio_products(
        MODEL,
        list(session.joint_names),
        case["q"],
        case["roots"],
        case["velocity"],
        gravity,
    )
    point_position, point_jacobian = pin_point_products(
        MODEL,
        list(session.joint_names),
        list(session.point_frame_names),
        np.asarray(session.point_offsets, np.float64),
        case["q"],
        case["roots"],
    )
    dynamics_linf = 0.0
    contact_linf = 0.0
    oracle_margins = np.full(len(case["q"]), np.inf, np.float64)
    oracle_cop = np.full((len(case["q"]), 2), np.nan, np.float64)
    for agent in range(len(case["q"])):
        if int(result["status"][agent]) not in SOLVED_STATUSES:
            continue
        rhs = np.zeros(session.generalized_dof, np.float64)
        rhs[6:] = result["effort"][agent]
        for contact, query in enumerate(CONTACT_SLOTS):
            if not case["contact_active"][agent, contact]:
                continue
            jacobian = point_jacobian[agent, query]
            rhs += jacobian.T @ result["contact_force_world"][agent, contact]
            residual = jacobian @ result["qdd"][agent]
            mode = int(session.contact_kinematic_modes[contact])
            axes = {0: (), 1: (0, 1, 2), 2: (2,), 3: (1, 2)}[mode]
            if axes:
                contact_linf = max(contact_linf, float(np.max(np.abs(residual[list(axes)]))))
        residual = mass[agent] @ result["qdd"][agent] + bias[agent] - rhs
        dynamics_linf = max(dynamics_linf, float(np.max(np.abs(residual))))
        normals = result["contact_force_basis"][agent, :4, 2]
        total = float(np.sum(normals))
        if total > 1e-9:
            patch_points = point_position[agent, 1:5, :2]
            hull = convex_hull(patch_points)
            cop = np.sum(patch_points * normals[:, None], axis=0) / total
            oracle_cop[agent] = cop
            oracle_margins[agent] = polygon_margin(hull, cop)
    loaded = np.isfinite(oracle_margins)
    reported_delta = float(
        np.max(np.abs(oracle_margins[loaded] - result["minimum_support_margin"][loaded]), initial=0.0)
    )
    return {
        "pinocchio_dynamics_linf": dynamics_linf,
        "pinocchio_contact_acceleration_linf": contact_linf,
        "oracle_margin_m": oracle_margins,
        "oracle_cop_xy_m": oracle_cop,
        "reported_margin_max_abs_delta_m": reported_delta,
    }


def topology_case() -> dict[str, Any]:
    frames, offsets, stable_ids = query_plan(pathlib.Path("models/upkie/upkie.urdf"))
    rejected = False
    message = ""
    try:
        CpuMirrorBatchSession(
            "models/upkie/upkie.urdf",
            1,
            32,
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
            support_patch_first_contact_slots=[0],
            support_patch_contact_counts=[2],
            support_patch_stable_ids=[1],
        )
    except ValueError as error:
        rejected = True
        message = str(error)
    return {
        "upkie_declared_contact_points": 2,
        "finite_area_claimed": False,
        "two_point_patch_rejected": rejected,
        "rejection": message,
        "pass": rejected,
    }


def oracle_case(samples: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    session, case = make_case(samples)
    result = run_dynamic(session, case)
    oracle = independent_oracle(session, case, result)
    margins = oracle.pop("oracle_margin_m")
    cop = oracle.pop("oracle_cop_xy_m")
    passed = bool(
        np.all(np.isin(result["status"], SOLVED_STATUSES))
        and np.min(margins) >= PATCH_MARGIN_M - 2e-8
        and oracle["reported_margin_max_abs_delta_m"] <= 2e-8
        and oracle["pinocchio_dynamics_linf"] <= 3e-7
        and oracle["pinocchio_contact_acceleration_linf"] <= 3e-8
        and result["allocation_calls"][0] == 0
        and result["allocated_bytes"][0] == 0
    )
    metrics = {
        "samples": samples,
        "status_counts": status_counts(result["status"]),
        "minimum_oracle_support_margin_m": float(np.min(margins)),
        "maximum_oracle_support_margin_m": float(np.max(margins)),
        **oracle,
        "limiting_patch_ids": sorted({int(value) for value in result["limiting_support_patch"]}),
        "allocation_calls": int(result["allocation_calls"][0]),
        "allocated_bytes": int(result["allocated_bytes"][0]),
        "pass": passed,
    }
    raw = {
        "oracle_cop_xy_m": cop,
        "oracle_support_margin_m": margins,
        "reported_support_margin_m": result["minimum_support_margin"].copy(),
        "contact_normal_force_n": result["contact_force_basis"][:, :, 2].copy(),
        "intent_residual": result["task_l2"][:, 0].copy(),
    }
    return metrics, raw


def edge_conflict_case(contact_kinematic_modes: list[int] | None = None) -> dict[str, Any]:
    session, case = make_case(2, contact_kinematic_modes=contact_kinematic_modes)
    case["target_position"][:, 0, 1] = case["target_position"][0, 0, 1] + 0.1875
    case["support_patch_enabled"][:, 0] = [0, 1]
    result = run_dynamic(session, case)
    oracle = independent_oracle(session, case, result)
    margins = oracle["oracle_margin_m"]
    passed = bool(
        np.all(np.isin(result["status"], SOLVED_STATUSES))
        and margins[0] < PATCH_MARGIN_M - 1e-4
        and abs(margins[1] - PATCH_MARGIN_M) <= 2e-8
        and result["task_l2"][1, 0] > result["task_l2"][0, 0]
        and np.isinf(result["minimum_support_margin"][0])
        and result["limiting_support_patch"][0] == UINT32_MAX
        and result["limiting_support_patch"][1] == PATCH_STABLE_ID
    )
    return {
        "lateral_target_offset_m": 0.1875,
        "without_patch_oracle_margin_m": float(margins[0]),
        "with_patch_oracle_margin_m": float(margins[1]),
        "requested_margin_m": PATCH_MARGIN_M,
        "without_patch_intent_residual": float(result["task_l2"][0, 0]),
        "with_patch_intent_residual": float(result["task_l2"][1, 0]),
        "with_patch_reported_margin_m": float(result["minimum_support_margin"][1]),
        "pass": passed,
    }


def zero_load_case(contact_kinematic_modes: list[int] | None = None) -> dict[str, Any]:
    session, case = make_case(1, contact_kinematic_modes=contact_kinematic_modes)
    case["task_active"].fill(0)
    case["contact_active"].fill(1)
    case["maximum_normal_force"][:, :4] = 0.0
    case["nominal_normal_force"][:, :4] = 0.0
    case["nominal_normal_force"][:, 4] = 40.2 * 9.81
    result = run_dynamic(session, case)
    normals = result["contact_force_basis"][0, :, 2]
    passed = bool(
        int(result["status"][0]) in SOLVED_STATUSES
        and np.all(normals[:4] == 0.0)
        and normals[4] > 0.0
        and np.isinf(result["minimum_support_margin"][0])
        and result["limiting_support_patch"][0] == UINT32_MAX
        and result["active_support_patches"][0] == 1
    )
    return {
        "status": STATUS_NAMES[int(result["status"][0])],
        "patch_total_normal_force_n": float(np.sum(normals[:4])),
        "outside_patch_normal_force_n": float(normals[4]),
        "reported_margin": "Infinity" if np.isinf(result["minimum_support_margin"][0]) else float(result["minimum_support_margin"][0]),
        "limiting_patch": None if result["limiting_support_patch"][0] == UINT32_MAX else int(result["limiting_support_patch"][0]),
        "pass": passed,
    }


def failure_and_isolation(contact_kinematic_modes: list[int] | None = None) -> dict[str, Any]:
    session, base = make_case(17, contact_kinematic_modes=contact_kinematic_modes)
    baseline = {name: value.copy() for name, value in run_dynamic(session, base).items()}
    invalid = copy_case(base)
    invalid["support_patch_minimum_margin"][8, 0] = np.nan
    invalid_result = run_dynamic(session, invalid)
    neighbors = np.arange(17) != 8
    margin_isolated = bool(
        invalid_result["status"][8] == INVALID_INPUT
        and np.all(invalid_result["qdd"][8] == 0.0)
        and all(np.array_equal(invalid_result[name][neighbors], baseline[name][neighbors]) for name in EXACT_FIELDS)
    )
    missing = copy_case(base)
    missing["contact_active"][9, 2] = 0
    missing_result = run_dynamic(session, missing)
    missing_isolated = bool(
        missing_result["status"][9] == INVALID_INPUT
        and np.all(missing_result["qdd"][9] == 0.0)
        and all(np.array_equal(missing_result[name][np.arange(17) != 9], baseline[name][np.arange(17) != 9]) for name in EXACT_FIELDS)
    )
    disabled = copy_case(base)
    disabled["support_patch_enabled"].fill(0)
    disabled["support_patch_minimum_margin"].fill(0.0)
    disabled_clean = {name: value.copy() for name, value in run_dynamic(session, disabled).items()}
    disabled["support_patch_minimum_margin"].fill(np.nan)
    disabled_poison = run_dynamic(session, disabled)
    poison_ignored = all(np.array_equal(disabled_clean[name], disabled_poison[name]) for name in EXACT_FIELDS)
    return {
        "invalid_margin_agent": 8,
        "invalid_margin_isolated": margin_isolated,
        "missing_patch_point_agent": 9,
        "missing_point_isolated": missing_isolated,
        "disabled_poisoned_margin_ignored_bitwise": poison_ignored,
        "pass": bool(margin_isolated and missing_isolated and poison_ignored),
    }


def invariance_case(contact_kinematic_modes: list[int] | None = None) -> dict[str, Any]:
    session, case = make_case(17, contact_kinematic_modes=contact_kinematic_modes)
    first = {name: value.copy() for name, value in run_dynamic(session, case).items()}
    repeat = run_dynamic(session, case)
    permutation = np.asarray([*range(16, -1, -2), *range(15, -1, -2)])
    permuted_case = {name: np.ascontiguousarray(value[permutation]) for name, value in case.items()}
    permuted = run_dynamic(session, permuted_case)
    inverse = np.argsort(permutation)
    compact_session, compact_case = make_case(
        17, alignment=1, contact_kinematic_modes=contact_kinematic_modes
    )
    compact = run_dynamic(compact_session, compact_case)
    exact = EXACT_FIELDS
    repeat_exact = all(np.array_equal(first[name], repeat[name]) for name in exact)
    permutation_exact = all(np.array_equal(first[name], permuted[name][inverse]) for name in exact)
    padding_exact = all(np.array_equal(first[name], compact[name]) for name in exact)
    allocations = sum(int(item["allocation_calls"][0]) for item in (first, repeat, permuted, compact))
    return {
        "repeat_bitwise_exact": repeat_exact,
        "permutation_exact": permutation_exact,
        "padding_exact": padding_exact,
        "allocation_calls": allocations,
        "pass": bool(repeat_exact and permutation_exact and padding_exact and allocations == 0),
    }


def timing_case(
    agents: int,
    repeats: int,
    contact_kinematic_modes: list[int] | None = None,
) -> dict[str, Any]:
    session, case = make_case(agents, contact_kinematic_modes=contact_kinematic_modes)
    buffers = dynamic_buffers(session)
    for _ in range(4):
        run_dynamic(session, case, buffers)
    solve = np.empty(repeats, np.float64)
    pipeline = np.empty(repeats, np.float64)
    calls = allocated = 0
    stages = ("joint_envelope", "fk", "jacobian", "dynamics", "point", "emission", "solve")
    for index in range(repeats):
        result = run_dynamic(session, case, buffers)
        solve[index] = result["solve_execute_ns"][0]
        pipeline[index] = sum(float(result[f"{stage}_execute_ns"][0]) for stage in stages)
        calls += int(result["allocation_calls"][0])
        allocated += int(result["allocated_bytes"][0])
    return {
        "batch_size": agents,
        "repeats": repeats,
        "solve_execute_ns": distribution(solve),
        "full_pipeline_ns": distribution(pipeline),
        "allocation_calls": calls,
        "allocated_bytes": allocated,
    }


def report_markdown(metrics: dict[str, Any]) -> str:
    authority = [
        ["Invariant", "floating rigid-body dynamics", "M qdd + h = Sᵀ effort + Jᶜᵀ force; hard residual"],
        ["Invariant", "five explicitly declared point contacts", "per-point lock residual, unilateral load, friction margin"],
        ["Viability", "finite left-sole patch 8401", "hard CoP polygon; 20 mm requested margin + limiting stable ID"],
        ["Viability", "r83 joint stopping envelope", "per-joint qdd interval and recovery count; independent of support"],
        ["Intent", "pelvis lateral attractor 8201", "residual rises when the CoP edge owns authority"],
        ["Contact resource", "normal and friction headroom", "point-force blocks stay observable; no inferred wrench"],
        ["Actuator resource", "effort headroom + limiting actuator", "continuous authority, separate from geometric support"],
        ["Recovery", "typed invalid/missing declarations", "bad agent zeroed; neighbors bitwise unchanged"],
        ["Solver budget", "hard violation + clipped steps + task slack", "numerical pressure is not physical feasibility"],
        ["Backend", "CpuExactF64", "preallocated Rust semantic reference; CUDA solve still unavailable"],
    ]
    timing = [
        [
            row["batch_size"],
            f'{row["solve_execute_ns"]["p50"] / 1e3:.3f}',
            f'{row["solve_execute_ns"]["p99"] / 1e3:.3f}',
            f'{row["full_pipeline_ns"]["p50"] / 1e3:.3f}',
            row["allocation_calls"],
        ]
        for row in metrics["timing"]
    ]
    oracle = metrics["independent_oracle"]
    conflict = metrics["edge_conflict"]
    return "\n".join(
        [
            "# Bonesaw finite support patch · r84",
            "",
            "## Outcome",
            "",
            f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** Fixed support-patch topology now crosses the Rust/Python batch boundary. Rust constructs the convex hull from declared point-force slots and emits exact inward-eroded CoP inequalities. Python independently rebuilds Pinocchio dynamics/contact equations and the polygon/CoP geometry. This checkpoint uses neither a policy nor a physics rollout.',
            "",
            "## Example authority stack",
            "",
            "This is the concrete stack used for architectural review. Authority ordering and every witness remain separate; no scalar ‘feasibility score’ hides which resource is binding.",
            "",
            *markdown_table(["layer", "example authority", "separate witness"], authority),
            "",
            "## Independent Pinocchio + NumPy oracle",
            "",
            *markdown_table(
                ["states", "dynamics L∞", "contact L∞", "minimum CoP margin m", "reported Δ m", "pass"],
                [[oracle["samples"], f'{oracle["pinocchio_dynamics_linf"]:.3e}', f'{oracle["pinocchio_contact_acceleration_linf"]:.3e}', f'{oracle["minimum_oracle_support_margin_m"]:.6f}', f'{oracle["reported_margin_max_abs_delta_m"]:.3e}', oracle["pass"]]],
            ),
            "",
            "The oracle uses URDF geometry and Pinocchio products, returned point forces, and an independent monotone-chain hull. It does not trust Rust’s reported support or dynamics residuals for admission.",
            "",
            "## Edge conflict",
            "",
            *markdown_table(["condition", "oracle margin m", "Intent residual"], [
                ["support patch disabled", f'{conflict["without_patch_oracle_margin_m"]:.6f}', f'{conflict["without_patch_intent_residual"]:.6f}'],
                ["20 mm support margin enabled", f'{conflict["with_patch_oracle_margin_m"]:.6f}', f'{conflict["with_patch_intent_residual"]:.6f}'],
            ]),
            "",
            "The unconstrained solve places CoP inside the physical sole but below the requested inward margin. Enabling patch 8401 pins CoP to 20 mm and increases only the lower-authority Intent residual.",
            "",
            "## Zero-load semantics and honest Upkie topology",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["zero_load"].items()]),
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["topology"].items()]),
            "",
            "Upkie’s two wheel contacts are a line, not a finite-area sole. R84 rejects a two-point patch and makes no finite support-margin claim for Upkie.",
            "",
            "## Failure isolation",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["failure_and_isolation"].items()]),
            "",
            "## Determinism",
            "",
            *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]),
            "",
            "## Release timing (untrimmed)",
            "",
            *markdown_table(["agents", "solve p50 us", "solve p99 us", "pipeline p50 us", "alloc calls"], timing),
            "",
            "All timing samples are retained. Timers cover preallocated Rust stages and exclude NumPy marshalling.",
            "",
            "## Deliberate scope boundary",
            "",
            "R84 admits fixed, horizontal finite-support declarations over point-force slots. It does not yet switch patches during gait, add a CoM-in-polygon task, model deformable contact, or stream this dynamic authority row into the orbit editor. The four sole points are all locked in this reference fixture; a later contact declaration revision should separate force-point existence from rank-minimal kinematic locking. CUDA remains gated on completion of the CPU concept and evaluations.",
            "",
        ]
    )


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    topology = topology_case()
    oracle, raw = oracle_case(args.samples)
    conflict = edge_conflict_case()
    zero_load = zero_load_case()
    failure = failure_and_isolation()
    invariance = invariance_case()
    timing = [timing_case(agents, args.timing_repeats) for agents in (1, 8, 32)]
    admission = bool(
        topology["pass"]
        and oracle["pass"]
        and conflict["pass"]
        and zero_load["pass"]
        and failure["pass"]
        and invariance["pass"]
        and all(row["allocation_calls"] == 0 and row["allocated_bytes"] == 0 for row in timing)
    )
    metrics = {
        "schema_version": 1,
        "revision": "finite-support-patch-r84",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.processor()},
        "model": str(MODEL),
        "model_sha256": sha256(MODEL),
        "policy": None,
        "physics_rollout": None,
        "support_patch": {"stable_id": PATCH_STABLE_ID, "point_count": 4, "minimum_margin_m": PATCH_MARGIN_M},
        "topology": topology,
        "independent_oracle": oracle,
        "edge_conflict": conflict,
        "zero_load": zero_load,
        "failure_and_isolation": failure,
        "invariance": invariance,
        "timing": timing,
        "admission": admission,
    }
    np.savez_compressed(output / "finite-support-patch-raw.npz", **raw)
    (output / "finite-support-patch-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "FINITE_SUPPORT_PATCH_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw finite support patch · r84"))
    print(json.dumps(metrics, indent=2))
    if not admission:
        raise SystemExit("finite support patch admission failed")


if __name__ == "__main__":
    main()
