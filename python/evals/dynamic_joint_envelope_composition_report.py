#!/usr/bin/env python3
"""Admit joint-stopping envelopes inside the physical CPU-exact batch solve."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_exact_dynamic_batch_report import (
    ENVELOPE_FIELDS,
    EXACT_FIELDS,
    INVALID_INPUT,
    INVALID_PROBLEM,
    SOLVED_STATUSES,
    copy_case,
    dynamic_buffers,
    make_case,
    pinocchio_hard_residuals,
    run_dynamic,
    status_counts,
)
from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import sha256
from cuda_point_query_mirror_report import run as run_points
from joint_viability_envelope_report import interval, urdf_limits


ENVELOPE_DISABLED = 1
ENVELOPE_OK = 2
ENVELOPE_INVALID_INPUT = 3
ALL_RESULT_FIELDS = EXACT_FIELDS + ENVELOPE_FIELDS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=60)
    parser.add_argument("--output", default="benchmarks/results/dynamic-joint-envelope-r83")
    parser.add_argument("--web-report", default="web/DYNAMIC_JOINT_ENVELOPE_R83.html")
    return parser.parse_args()


def enabled_case(model: pathlib.Path, agents: int, *, alignment: int = 32):
    session, case, mass = make_case(model, agents, alignment=alignment)
    case["joint_envelope_enabled"].fill(1)
    case["joint_envelope_dt_seconds"].fill(0.02)
    case["joint_maximum_acceleration"].fill(20.0)
    return session, case, mass


def numpy_envelope(session, model: pathlib.Path, case: dict[str, np.ndarray]):
    lower_position, upper_position, maximum_velocity = urdf_limits(model, list(session.joint_names))
    agents = len(case["q"])
    lower = np.full((agents, session.generalized_dof), -np.inf, np.float64)
    upper = np.full((agents, session.generalized_dof), np.inf, np.float64)
    for agent in range(agents):
        for coordinate in range(session.dof):
            lo, hi = interval(
                float(case["q"][agent, coordinate]),
                float(case["velocity"][agent, 6 + coordinate]),
                lower_position[coordinate], upper_position[coordinate], maximum_velocity[coordinate],
                case["joint_maximum_acceleration"][agent, coordinate],
                case["joint_envelope_dt_seconds"][agent],
            )
            lower[agent, 6 + coordinate] = lo
            upper[agent, 6 + coordinate] = hi
    return lower, upper


def oracle_case(model: pathlib.Path, samples: int):
    session, case, _ = enabled_case(model, samples)
    result = run_dynamic(session, case)
    expected_lower, expected_upper = numpy_envelope(session, model, case)
    finite = np.isfinite(expected_lower) | np.isfinite(expected_upper)
    bound_error = float(max(
        np.max(np.abs(result["joint_envelope_lower"][finite] - expected_lower[finite]), initial=0.0),
        np.max(np.abs(result["joint_envelope_upper"][finite] - expected_upper[finite]), initial=0.0),
    ))
    infinity_exact = bool(
        np.array_equal(np.isneginf(result["joint_envelope_lower"]), np.isneginf(expected_lower))
        and np.array_equal(np.isposinf(result["joint_envelope_upper"]), np.isposinf(expected_upper))
    )
    solved = np.isin(result["status"], SOLVED_STATUSES)
    inside = bool(np.all(result["qdd"][solved] >= expected_lower[solved] - 2e-9)
                  and np.all(result["qdd"][solved] <= expected_upper[solved] + 2e-9))
    recomputed_margin = np.min(np.minimum(
        result["qdd"][:, 6:] - expected_lower[:, 6:],
        expected_upper[:, 6:] - result["qdd"][:, 6:],
    ), axis=1)
    margin_error = float(np.max(np.abs(
        result["minimum_joint_envelope_margin"][solved] - recomputed_margin[solved]
    ), initial=0.0))
    dynamics, contact = pinocchio_hard_residuals(model, session, case, result)
    passed = bool(np.all(solved) and np.all(result["joint_envelope_status"] == ENVELOPE_OK)
                  and bound_error <= 2e-12 and infinity_exact and inside and margin_error <= 2e-9
                  and dynamics <= 2e-7 and contact <= 2e-8
                  and result["allocation_calls"][0] == 0 and result["allocated_bytes"][0] == 0)
    return {
        "model": str(model), "model_sha256": sha256(model), "samples": samples,
        "status_counts": status_counts(result["status"]), "maximum_numpy_bound_error": bound_error,
        "infinity_masks_exact": infinity_exact, "all_solved_commands_inside_envelope": inside,
        "minimum_margin_diagnostic_error": margin_error,
        "pinocchio_dynamics_linf": dynamics, "pinocchio_contact_linf": contact,
        "allocation_calls": int(result["allocation_calls"][0]),
        "allocated_bytes": int(result["allocated_bytes"][0]), "pass": passed,
    }


def choose_limited_coordinate(model: pathlib.Path, session) -> tuple[int, np.ndarray, np.ndarray]:
    lower, upper, _ = urdf_limits(model, list(session.joint_names))
    names = list(session.joint_names)
    coordinate = names.index("right_hip") if "right_hip" in names else int(np.flatnonzero(np.isfinite(upper))[0])
    return coordinate, lower, upper


def outward_handle_target(session, case: dict[str, np.ndarray], distance: float):
    points = run_points(session, case["q"], case["roots"], case["velocity"], case["gravity"])
    case["target_position"][:] = points["position"][:, [0, 3]]
    # The handle is torso-fixed, so its leg-joint Jacobian is intentionally
    # zero. With bilateral wheel locks, moving it in -X requires the right hip
    # to accelerate toward its upper limit; this exercises the contact
    # nullspace coupling that a WBC actually resolves.
    case["target_position"][:, 1, 0] -= np.float32(distance)


def conflict_and_recovery_case(model: pathlib.Path):
    agents = 8
    session, base, _ = enabled_case(model, agents)
    coordinate, _, upper = choose_limited_coordinate(model, session)
    base["q"].fill(0.0)
    base["q"][:, coordinate] = np.float32(upper[coordinate] - 1e-5)
    base["velocity"].fill(0.0)
    base["qdd_lower"].fill(-500.0); base["qdd_upper"].fill(500.0)
    base["effort_lower"] *= 20.0; base["effort_upper"] *= 20.0
    base["maximum_normal_force"].fill(10000.0)
    outward_handle_target(session, base, 0.10)

    disabled = copy_case(base); disabled["joint_envelope_enabled"].fill(0)
    unconstrained = run_dynamic(session, disabled)
    constrained = run_dynamic(session, base)
    expected_lower, expected_upper = numpy_envelope(session, model, base)
    solved = np.isin(constrained["status"], SOLVED_STATUSES)
    inside = bool(np.all(constrained["qdd"][solved, 6 + coordinate]
                         <= expected_upper[solved, 6 + coordinate] + 2e-9))
    consumed = float(np.min(constrained["minimum_joint_envelope_margin"][solved], initial=np.inf))
    task_cost = float(np.max(constrained["task_l2"][solved, 1], initial=0.0))
    unconstrained_qdd = float(np.max(unconstrained["qdd"][:, 6 + coordinate]))
    constrained_qdd = float(np.max(constrained["qdd"][solved, 6 + coordinate], initial=-np.inf))

    recovery = copy_case(base)
    recovery["q"][:, coordinate] = np.float32(upper[coordinate] + 0.10)
    recovery["velocity"][:, 6 + coordinate] = 1.0
    outward_handle_target(session, recovery, 0.10)
    recovered = run_dynamic(session, recovery)
    recovery_solved = np.isin(recovered["status"], SOLVED_STATUSES)
    recovery_exact = bool(np.all(recovery_solved)
                          and np.all(recovered["joint_recovery_count"] == 1)
                          and np.allclose(recovered["qdd"][:, 6 + coordinate], -20.0, atol=2e-8, rtol=0.0))
    passed = bool(np.all(solved) and inside and consumed >= -2e-8
                  and constrained_qdd < unconstrained_qdd - 1e-3 and task_cost > 1e-6
                  and recovery_exact)
    return {
        "coordinate": coordinate, "joint_name": list(session.joint_names)[coordinate],
        "near_limit_envelope_upper_qdd": float(expected_upper[-1, 6 + coordinate]),
        "unconstrained_max_qdd": unconstrained_qdd, "constrained_max_qdd": constrained_qdd,
        "minimum_envelope_margin": consumed, "maximum_intent_residual": task_cost,
        "constrained_status_counts": status_counts(constrained["status"]),
        "recovery_status_counts": status_counts(recovered["status"]),
        "outside_limit_recovery_qdd_exact": recovery_exact, "pass": passed,
    }


def failure_typing_case(model: pathlib.Path):
    session, base, _ = enabled_case(model, 17)
    coordinate, _, upper = choose_limited_coordinate(model, session)
    base["q"].fill(0.0); base["q"][:, coordinate] = np.float32(upper[coordinate] - 1e-5)
    base["velocity"].fill(0.0); base["velocity"][:, 6 + coordinate] = 0.4
    baseline = {name: value.copy() for name, value in run_dynamic(session, base).items()}

    invalid_case = copy_case(base); invalid_case["joint_maximum_acceleration"][8, coordinate] = np.nan
    invalid = run_dynamic(session, invalid_case)
    neighbors = np.arange(17) != 8
    isolated = bool(invalid["joint_envelope_status"][8] == ENVELOPE_INVALID_INPUT
                    and invalid["status"][8] == INVALID_INPUT and np.all(invalid["qdd"][8] == 0.0)
                    and all(np.array_equal(invalid[name][neighbors], baseline[name][neighbors])
                            for name in ALL_RESULT_FIELDS))

    disabled = copy_case(base); disabled["joint_envelope_enabled"].fill(0)
    disabled["joint_envelope_dt_seconds"].fill(np.nan)
    disabled["joint_maximum_acceleration"].fill(np.nan)
    disabled_result = run_dynamic(session, disabled)
    clean_disabled = copy_case(base); clean_disabled["joint_envelope_enabled"].fill(0)
    clean_disabled_result = run_dynamic(session, clean_disabled)
    disabled_exact = bool(np.all(disabled_result["joint_envelope_status"] == ENVELOPE_DISABLED)
                          and all(np.array_equal(disabled_result[name], clean_disabled_result[name])
                                  for name in EXACT_FIELDS))

    contradiction = copy_case(base)
    contradiction["qdd_lower"][:, 6 + coordinate] = 10.0
    contradiction_result = run_dynamic(session, contradiction)
    contradiction_typed = bool(np.all(contradiction_result["status"] == INVALID_PROBLEM)
                               and np.all(contradiction_result["qdd"] == 0.0)
                               and np.all(contradiction_result["effort"] == 0.0)
                               and np.all(contradiction_result["contact_force_world"] == 0.0))
    return {
        "invalid_agent": 8, "invalid_agent_isolated_bitwise": isolated,
        "disabled_poisoned_payload_ignored_exactly": disabled_exact,
        "empty_intersection_status_counts": status_counts(contradiction_result["status"]),
        "empty_intersection_zero_commands": contradiction_typed,
        "pass": bool(isolated and disabled_exact and contradiction_typed),
    }


def approach_sweep_case(model: pathlib.Path):
    agents = 64
    session, case, _ = enabled_case(model, agents)
    coordinate, _, upper = choose_limited_coordinate(model, session)
    case["q"].fill(0.0)
    case["q"][:, coordinate] = np.linspace(upper[coordinate] - 0.5, upper[coordinate] - 1e-5, agents).astype(np.float32)
    case["velocity"].fill(0.0)
    case["effort_lower"] *= 20.0; case["effort_upper"] *= 20.0
    case["maximum_normal_force"].fill(10000.0)
    outward_handle_target(session, case, 0.08)
    result = run_dynamic(session, case)
    upper_qdd = result["joint_envelope_upper"][:, 6 + coordinate]
    solved = np.isin(result["status"], SOLVED_STATUSES)
    monotone = bool(np.all(np.diff(upper_qdd) <= 1e-12))
    inside = bool(np.all(result["qdd"][solved, 6 + coordinate] <= upper_qdd[solved] + 2e-8))
    return {
        "coordinate": coordinate, "samples": agents,
        "farthest_upper_qdd": float(upper_qdd[0]), "nearest_upper_qdd": float(upper_qdd[-1]),
        "upper_bound_monotone_nonincreasing": monotone,
        "solved_commands_inside_envelope": inside, "status_counts": status_counts(result["status"]),
        "minimum_envelope_margin": float(np.min(result["minimum_joint_envelope_margin"][solved], initial=np.inf)),
        "maximum_intent_residual": float(np.max(result["task_l2"][solved, 1], initial=0.0)),
        "pass": bool(monotone and inside and np.all(solved)),
    }


def invariance_case(model: pathlib.Path):
    agents = 17
    session, case, _ = enabled_case(model, agents)
    first = {name: value.copy() for name, value in run_dynamic(session, case).items()}
    repeat = run_dynamic(session, case)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)])
    permuted = run_dynamic(session, {name: value[permutation] for name, value in case.items()})
    inverse = np.argsort(permutation)
    compact_session, compact_case, _ = enabled_case(model, agents, alignment=1)
    compact = run_dynamic(compact_session, compact_case)
    repeat_exact = all(np.array_equal(first[name], repeat[name]) for name in ALL_RESULT_FIELDS)
    permutation_exact = all(np.array_equal(first[name], permuted[name][inverse]) for name in ALL_RESULT_FIELDS)
    padding_exact = all(np.array_equal(first[name], compact[name]) for name in ALL_RESULT_FIELDS)
    allocations = sum(int(result["allocation_calls"][0]) for result in (first, repeat, permuted, compact))
    return {"repeat_bitwise_exact": repeat_exact, "permutation_exact": permutation_exact,
            "padding_exact": padding_exact, "allocation_calls": allocations,
            "pass": bool(repeat_exact and permutation_exact and padding_exact and allocations == 0)}


def timing_case(model: pathlib.Path, agents: int, repeats: int):
    session, case, _ = enabled_case(model, agents)
    out = dynamic_buffers(session)
    for _ in range(4):
        run_dynamic(session, case, out)
    stages = ("joint_envelope", "fk", "jacobian", "dynamics", "point", "emission", "solve")
    traces = {name: np.empty(repeats, np.float64) for name in stages}
    calls = allocated = 0
    for index in range(repeats):
        result = run_dynamic(session, case, out)
        for name in stages:
            traces[name][index] = result[f"{name}_execute_ns"][0]
        calls += int(result["allocation_calls"][0]); allocated += int(result["allocated_bytes"][0])
    total = sum(traces.values())
    return {"batch_size": agents, "repeats": repeats,
            "joint_envelope_execute_ns": distribution(traces["joint_envelope"]),
            "dynamic_solve_execute_ns": distribution(traces["solve"]),
            "full_pipeline_ns": distribution(total), "allocation_calls": calls, "allocated_bytes": allocated}


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle_rows = [[pathlib.Path(row["model"]).stem, row["samples"],
                    f'{row["maximum_numpy_bound_error"]:.3e}', f'{row["pinocchio_dynamics_linf"]:.3e}',
                    f'{row["pinocchio_contact_linf"]:.3e}', row["pass"]]
                   for row in metrics["oracle_cases"]]
    timing_rows = [[row["batch_size"], f'{row["joint_envelope_execute_ns"]["p50"] / 1e3:.3f}',
                    f'{row["dynamic_solve_execute_ns"]["p50"] / 1e3:.3f}',
                    f'{row["dynamic_solve_execute_ns"]["p99"] / 1e3:.3f}',
                    f'{row["full_pipeline_ns"]["p50"] / 1e3:.3f}', row["allocation_calls"]]
                   for row in metrics["timing"]]
    authority = [
        ["Invariant", "contact locks 7951 / 7952", "hard point acceleration equalities"],
        ["Invariant", "floating dynamics", "qdd, effort, and force remain one physical decision"],
        ["Viability", "r83 stopping envelope", "position/velocity/braking limits intersect caller qdd authority in Rust"],
        ["Viability", "torso point 7901", "protected residual after every hard physical row"],
        ["Intent", "handle point 7902", "retains residual as the stopping box consumes authority"],
        ["Contact resource", "normal-force / friction margins", "not merged with joint or actuator margin"],
        ["Actuator resource", "effort margin / limiting actuator", "not inferred from the acceleration box"],
        ["Recovery", "outside-limit maximum inward qdd", "deterministic hard recovery command"],
        ["Problem typing", "InvalidInput / InvalidProblem / MaxIterations", "malformed data, empty boxes, and work exhaustion stay distinct"],
        ["Backend", "CpuExactF64", "zero-allocation semantic path; CUDA remains unavailable"],
    ]
    return "\n".join([
        "# Bonesaw dynamic joint-envelope composition · r83", "", "## Outcome", "",
        f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** The R81 stopping envelope now executes inside the same preallocated Rust call as the R82 physical decision vector `[qdd, generalized joint effort, point-contact force]`. Caller acceleration authority and the derived position/velocity/braking box are intersected before floating dynamics, locked contacts, effort, unilateral force, friction, and lexicographic point tasks are solved. Python remains eval/oracle land; no policy or physics rollout is used.', "",
        "## Example authority stack", "",
        "These witnesses are ordered but never aggregated. In particular, a joint-envelope margin is not an actuator-effort or friction margin.", "",
        *markdown_table(["layer", "concrete authority", "separate witness"], authority), "",
        "## Independent double oracle", "",
        *markdown_table(["model", "states", "NumPy bound delta", "Pin dynamics L∞", "Pin contact L∞", "pass"], oracle_rows), "",
        "NumPy independently reconstructs every stopping interval from URDF limits and the observed f32 state. Pinocchio independently rebuilds M, h, and contact Jacobians and checks the returned physical command. Neither admission witness trusts Rust's own diagnostics.", "",
        "## Near-limit conflict and recovery", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["conflict_and_recovery"].items()]), "",
        "The outward handle request is intentionally retained rather than silently weakened. As the stopping envelope closes, its Intent residual rises while dynamics, contact, effort, force, and the envelope remain hard.", "",
        "## Continuous approach sweep", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["approach_sweep"].items()]), "",
        "## Failure typing and isolation", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["failure_typing"].items()]), "",
        "## Determinism", "",
        *markdown_table(["check", "result"], [[key, value] for key, value in metrics["invariance"].items()]), "",
        "## Release timing (untrimmed)", "",
        *markdown_table(["agents", "envelope p50 us", "solve p50 us", "solve p99 us", "pipeline p50 us", "alloc calls"], timing_rows), "",
        "Timers cover preallocated Rust stages only and exclude NumPy marshalling. All calls are retained.", "",
        "## Scope boundary", "",
        "R83 composes joint viability with point-contact dynamics, effort, and friction. Finite support patches, CoP/CoM polygon inequalities, power, thermal realization, contact switching, and a CUDA hierarchical solver remain unavailable. The fixed world-gravity contract remains `[0, 0, -9.81]`. Body response is not inferred without a later realization or closed-loop stage.", "",
    ])


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output); output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    oracle = [oracle_case(model, args.samples) for model in models]
    conflict = conflict_and_recovery_case(models[1])
    approach = approach_sweep_case(models[1])
    failure = failure_typing_case(models[1])
    invariance = invariance_case(models[1])
    timing = [timing_case(models[1], agents, args.timing_repeats) for agents in (1, 8, 32)]
    admission = (all(row["pass"] for row in oracle) and conflict["pass"] and approach["pass"]
                 and failure["pass"] and invariance["pass"]
                 and all(row["allocation_calls"] == 0 and row["allocated_bytes"] == 0 for row in timing))
    metrics = {"schema_version": 1, "revision": "dynamic-joint-envelope-r83",
               "generated_at": datetime.now(timezone.utc).isoformat(),
               "host": {"platform": platform.platform(), "processor": platform.processor()},
               "policy": None, "physics_rollout": None, "oracle_cases": oracle,
               "conflict_and_recovery": conflict, "approach_sweep": approach,
               "failure_typing": failure, "invariance": invariance, "timing": timing,
               "finite_support": {"available": False, "claimed": False},
               "cuda_hierarchical_solver": {"available": False, "claimed": False},
               "admission": bool(admission)}
    (output / "dynamic-joint-envelope-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "DYNAMIC_JOINT_ENVELOPE_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw dynamic joint envelope · r83"))
    print(json.dumps(metrics, indent=2))
    if not admission:
        raise SystemExit("dynamic joint-envelope composition admission failed")


if __name__ == "__main__":
    main()
