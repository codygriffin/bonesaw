#!/usr/bin/env python3
"""Audit the fixed emission ABI through the strict f64 CPU hierarchy."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import joint_states, root_pose_matrix, sha256
from cuda_dynamics_mirror_report import gravities, velocities
from cuda_emission_mirror_report import (
    CONTACT_STABLE_IDS,
    TASK_PRIORITIES,
    TASK_STABLE_IDS,
    output_buffers as emission_buffers,
    run as run_emission,
    session_for,
    subset_targets,
    target_inputs,
)


SOLVED = 1
SOLVED_WITH_SLACK = 2
MAX_ITERATIONS = 3
INVALID_PROBLEM = 6
LEVEL_NAMES = ("Invariant", "Viability", "Intent", "Preference", "Style")
EXACT_FIELDS = (
    "qdd", "status", "level_rows", "level_l2", "level_rank", "level_clipped",
    "minimum_bound_margin", "maximum_hard_violation", "active_constraint_count",
    "task_pseudoinverse_calls", "task_jacobi_sweeps", "clipped_steps",
    "feasibility_projection_sweeps", "feasibility_halfspace_projections",
    "feasibility_polish_iterations",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--timing-repeats", type=int, default=100)
    parser.add_argument("--output", default="benchmarks/results/cpu-exact-batch-solve-r80")
    parser.add_argument("--web-report", default="web/CPU_EXACT_BATCH_SOLVE_R80.html")
    return parser.parse_args()


def solve_buffers(session) -> dict[str, np.ndarray]:
    a, g = session.agent_capacity, session.generalized_dof
    p = session.priority_level_count
    result = {
        "qdd": np.empty((a, g), np.float64),
        "status": np.empty(a, np.uint8),
        "level_rows": np.empty((a, p), np.uint32),
        "level_l2": np.empty((a, p), np.float64),
        "level_rank": np.empty((a, p), np.uint32),
        "level_clipped": np.empty((a, p), np.uint8),
        "minimum_bound_margin": np.empty(a, np.float64),
        "maximum_hard_violation": np.empty(a, np.float64),
        "active_constraint_count": np.empty(a, np.uint32),
        "task_pseudoinverse_calls": np.empty(a, np.uint32),
        "task_jacobi_sweeps": np.empty(a, np.uint32),
        "clipped_steps": np.empty(a, np.uint32),
        "feasibility_projection_sweeps": np.empty(a, np.uint32),
        "feasibility_halfspace_projections": np.empty(a, np.uint32),
        "feasibility_polish_iterations": np.empty(a, np.uint32),
    }
    for name in ("fk", "jacobian", "dynamics", "point", "emission", "solve"):
        result[f"{name}_execute_ns"] = np.empty(1, np.uint64)
    result["allocation_calls"] = np.empty(1, np.uint64)
    result["allocated_bytes"] = np.empty(1, np.uint64)
    return result


def run_exact(session, q, roots, velocity, gravity, targets, lower, upper, buffers=None):
    out = solve_buffers(session) if buffers is None else buffers
    session.run_exact_solve_batch(
        np.ascontiguousarray(q, np.float32), np.ascontiguousarray(roots, np.float32),
        np.ascontiguousarray(velocity, np.float32), np.ascontiguousarray(gravity, np.float32),
        np.ascontiguousarray(targets["task_active"], np.uint8),
        np.ascontiguousarray(targets["target_position"], np.float32),
        np.ascontiguousarray(targets["target_velocity"], np.float32),
        np.ascontiguousarray(targets["target_acceleration"], np.float32),
        np.ascontiguousarray(targets["contact_active"], np.uint8),
        np.ascontiguousarray(targets["contact_desired_acceleration"], np.float32),
        np.ascontiguousarray(lower, np.float64), np.ascontiguousarray(upper, np.float64),
        *[out[name] for name in EXACT_FIELDS],
        out["fk_execute_ns"], out["jacobian_execute_ns"], out["dynamics_execute_ns"],
        out["point_execute_ns"], out["emission_execute_ns"], out["solve_execute_ns"],
        out["allocation_calls"], out["allocated_bytes"],
    )
    return out


def normalized_rows(matrix: np.ndarray, target: np.ndarray, weight: float = 1.0):
    norms = np.linalg.norm(matrix, axis=1)
    scale = np.where(norms > 1e-12, np.sqrt(weight) / norms, np.sqrt(weight))
    return matrix * scale[:, None], target * scale


def absolute_pinv(matrix: np.ndarray, tolerance: float) -> np.ndarray:
    """Moore-Penrose inverse with the Rust solver's absolute rank threshold."""
    u, singular, vh = np.linalg.svd(matrix, full_matrices=False)
    reciprocal = np.where(singular > tolerance, 1.0 / singular, 0.0)
    return (vh.T * reciprocal) @ u.T


def numpy_hierarchy(emitted: dict[str, np.ndarray], agent: int, priorities: list[int], weights: list[float]):
    g = emitted["task_jacobian"].shape[-1]
    equalities, equality_targets = [], []
    for slot, active in enumerate(emitted["contact_active"][agent]):
        if active:
            a, b = normalized_rows(emitted["contact_jacobian"][agent, slot].astype(np.float64),
                                   emitted["contact_rhs"][agent, slot].astype(np.float64))
            equalities.append(a); equality_targets.append(b)
    if equalities:
        aeq, beq = np.concatenate(equalities), np.concatenate(equality_targets)
        pinv = absolute_pinv(aeq, 1e-12)
        x = pinv @ beq
        nullspace = np.eye(g) - pinv @ aeq
    else:
        x, nullspace = np.zeros(g), np.eye(g)
    level_l2 = np.zeros(5)
    for priority in range(5):
        matrices, targets = [], []
        for slot, active in enumerate(emitted["task_active"][agent]):
            if active and priorities[slot] == priority and weights[slot] > 0:
                a, b = normalized_rows(emitted["task_jacobian"][agent, slot].astype(np.float64),
                                       emitted["task_rhs"][agent, slot].astype(np.float64), weights[slot])
                matrices.append(a); targets.append(b)
        if not matrices:
            continue
        a, b = np.concatenate(matrices), np.concatenate(targets)
        projected = a @ nullspace
        pinv = absolute_pinv(projected, 1e-9)
        x += nullspace @ pinv @ (b - a @ x)
        nullspace -= nullspace @ pinv @ projected
        level_l2[priority] = np.linalg.norm(a @ x - b)
    return x, level_l2


def problem(model: pathlib.Path, agents: int):
    session = session_for(model, agents)
    q = joint_states(agents, session.dof)
    roots = root_pose_matrix(agents)
    velocity = velocities(agents, session.generalized_dof)
    gravity = gravities(agents)
    targets = target_inputs(agents)
    lower = np.full((agents, session.generalized_dof), -np.inf)
    upper = np.full((agents, session.generalized_dof), np.inf)
    return session, q, roots, velocity, gravity, targets, lower, upper


def oracle_case(model: pathlib.Path, samples: int):
    session, q, roots, velocity, gravity, targets, lower, upper = problem(model, samples)
    emitted = run_emission(session, q, roots, velocity, gravity, targets, emission_buffers(session))
    exact = run_exact(session, q, roots, velocity, gravity, targets, lower, upper)
    oracle_qdd, oracle_l2 = [], []
    hard = []
    for agent in range(samples):
        x, l2 = numpy_hierarchy(emitted, agent, TASK_PRIORITIES, [1.5, 0.75])
        oracle_qdd.append(x); oracle_l2.append(l2)
        for slot, active in enumerate(emitted["contact_active"][agent]):
            if active:
                hard.append(np.max(np.abs(emitted["contact_jacobian"][agent, slot] @ exact["qdd"][agent]
                                                  - emitted["contact_rhs"][agent, slot])))
    oracle_qdd, oracle_l2 = np.asarray(oracle_qdd), np.asarray(oracle_l2)
    qdd_error = float(np.max(np.abs(exact["qdd"] - oracle_qdd)))
    l2_error = float(np.max(np.abs(exact["level_l2"] - oracle_l2)))
    hard_error = float(max(hard, default=0.0))
    passed = bool(qdd_error <= 2e-5 and l2_error <= 2e-5 and hard_error <= 2e-8
                  and np.all(np.isin(exact["status"], [SOLVED, SOLVED_WITH_SLACK]))
                  and exact["allocation_calls"][0] == 0 and exact["allocated_bytes"][0] == 0)
    return {
        "model": str(model), "model_sha256": sha256(model), "samples": samples,
        "numpy_qdd_max_abs_error": qdd_error, "numpy_level_l2_max_abs_error": l2_error,
        "hard_contact_linf": hard_error, "status_counts": {str(i): int(np.sum(exact["status"] == i)) for i in np.unique(exact["status"])},
        "allocation_calls": int(exact["allocation_calls"][0]), "allocated_bytes": int(exact["allocated_bytes"][0]),
        "pass": passed,
    }, {f"{model.stem}_qdd": exact["qdd"].copy(), f"{model.stem}_level_l2": exact["level_l2"].copy()}


def bounded_case(model: pathlib.Path):
    session, q, roots, velocity, gravity, targets, lower, upper = problem(model, 8)
    targets["contact_active"].fill(0)
    lower.fill(-0.01); upper.fill(0.01)
    out = run_exact(session, q, roots, velocity, gravity, targets, lower, upper)
    maximum = float(np.max(np.abs(out["qdd"])))
    residual = float(np.max(out["level_l2"][:, 1:3]))
    passed = bool(np.all(out["status"] == SOLVED_WITH_SLACK) and maximum <= 0.0100000001
                  and np.any(out["clipped_steps"] > 0) and residual > 1e-6)
    return {"status_all_solved_with_slack": bool(np.all(out["status"] == SOLVED_WITH_SLACK)),
            "maximum_abs_qdd": maximum, "maximum_soft_level_l2": residual,
            "clipped_steps": int(np.sum(out["clipped_steps"])), "pass": passed}


def invalid_isolation(model: pathlib.Path):
    session, q, roots, velocity, gravity, targets, lower, upper = problem(model, 17)
    baseline = run_exact(session, q, roots, velocity, gravity, targets, lower, upper)
    bad_lower = lower.copy(); bad_upper = upper.copy(); bad_lower[8, 0] = 1.0; bad_upper[8, 0] = 0.0
    invalid = run_exact(session, q, roots, velocity, gravity, targets, bad_lower, bad_upper)
    neighbors = np.arange(17) != 8
    passed = bool(invalid["status"][8] == INVALID_PROBLEM
                  and np.array_equal(invalid["status"][neighbors], baseline["status"][neighbors])
                  and np.array_equal(invalid["qdd"][neighbors], baseline["qdd"][neighbors])
                  and np.all(invalid["qdd"][8] == 0.0))
    return {"invalid_agent": 8, "invalid_status": int(invalid["status"][8]),
            "neighbors_bitwise_exact": bool(np.array_equal(invalid["qdd"][neighbors], baseline["qdd"][neighbors])), "pass": passed}


def invariance_case(model: pathlib.Path):
    agents = 17
    session, q, roots, velocity, gravity, targets, lower, upper = problem(model, agents)
    base = run_exact(session, q, roots, velocity, gravity, targets, lower, upper)
    repeat = run_exact(session, q, roots, velocity, gravity, targets, lower, upper)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)])
    permuted = run_exact(session, q[permutation], roots[permutation], velocity[permutation], gravity[permutation],
                         subset_targets(targets, permutation), lower[permutation], upper[permutation])
    inverse = np.argsort(permutation)
    compact_session = session_for(model, agents, 1)
    compact = run_exact(compact_session, q, roots, velocity, gravity, targets, lower, upper)
    repeat_exact = all(np.array_equal(base[n], repeat[n]) for n in EXACT_FIELDS)
    permutation_exact = all(np.array_equal(base[n], permuted[n][inverse]) for n in EXACT_FIELDS)
    padding_exact = all(np.array_equal(base[n], compact[n]) for n in EXACT_FIELDS)
    calls = sum(int(x["allocation_calls"][0]) for x in (base, repeat, permuted, compact))
    passed = repeat_exact and permutation_exact and padding_exact and calls == 0
    return {"repeat_bitwise_exact": repeat_exact, "permutation_exact": permutation_exact,
            "padding_exact": padding_exact, "allocation_calls": calls, "pass": bool(passed)}


def timing_case(model: pathlib.Path, agents: int, repeats: int):
    session, q, roots, velocity, gravity, targets, lower, upper = problem(model, agents)
    buffers = solve_buffers(session)
    for _ in range(5): run_exact(session, q, roots, velocity, gravity, targets, lower, upper, buffers)
    traces = {name: np.empty(repeats) for name in ("fk", "jacobian", "dynamics", "point", "emission", "solve")}
    calls = allocated = 0
    for index in range(repeats):
        out = run_exact(session, q, roots, velocity, gravity, targets, lower, upper, buffers)
        for name in traces: traces[name][index] = out[f"{name}_execute_ns"][0]
        calls += int(out["allocation_calls"][0]); allocated += int(out["allocated_bytes"][0])
    total = sum(traces.values())
    return {"batch_size": agents, "repeats": repeats,
            "solve_execute_ns": distribution(traces["solve"]), "full_pipeline_ns": distribution(total),
            "per_agent_full_pipeline_ns": {k: v / agents for k, v in distribution(total).items()},
            "allocation_calls": calls, "allocated_bytes": allocated}


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle_rows = [[pathlib.Path(x["model"]).stem, f'{x["numpy_qdd_max_abs_error"]:.3e}',
                    f'{x["numpy_level_l2_max_abs_error"]:.3e}', f'{x["hard_contact_linf"]:.3e}', str(x["pass"])]
                   for x in metrics["oracle_cases"]]
    timing_rows = [[x["batch_size"], f'{x["solve_execute_ns"]["p50"] / 1e3:.3f}',
                    f'{x["solve_execute_ns"]["p99"] / 1e3:.3f}', f'{x["full_pipeline_ns"]["p50"] / 1e3:.3f}',
                    x["allocation_calls"]] for x in metrics["timing"]]
    authority = [
        ["Invariant", "contact-lock blocks 7951 / 7952", "hard 3-axis equalities; violation is reported separately"],
        ["Viability", "torso attractor 7901 · priority 1", "its normalized residual is frozen before lower layers"],
        ["Intent", "handle attractor 7902 · priority 2", "may retain residual when it conflicts with viability"],
        ["Preference", "no row in this example", "undeclared is not displayed as a zero residual"],
        ["Style", "no row in this example", "reserved terminal refinement"],
        ["Feasibility", "bound margin + hard violation + clipping", "continuous compromise evidence, not a binary pose promise"],
        ["Solver budget", "MaxIterations", "bounded search exhausted; explicitly not an infeasibility certificate"],
        ["Resources", "effort / power / thermal remain separate", "this kinematic row bridge cannot promise physical realization"],
        ["Backend", "CpuExactF64", "strict CPU reference path; no CUDA solve claim"],
    ]
    return "\n".join([
        "# Bonesaw strict CPU batch solve · r80", "",
        "## Outcome", "",
        f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** Fixed r79 task/contact rows now feed the established allocation-free `f64` strict hierarchy. This is the CPU semantic implementation requested before GPU batching; `CpuMirrorF32` still stops at row emission and no CUDA hierarchical solver is claimed.', "",
        "The independent oracle is NumPy SVD over the same emitted physical rows, with per-row normalization, hard contact equalities, and a sequential null-space projector. It is independent of the Rust pseudoinverse implementation. Evals use no policy, physics rollout, or learned component.", "",
        "## Independent numerical oracle", "",
        *markdown_table(["model", "max |qdd−NumPy|", "max |level l2−NumPy|", "hard contact L∞", "pass"], oracle_rows), "",
        "## Example authority stack", "",
        "This concrete stack is also published in the live architecture review. Values never collapse into one health score.", "",
        *markdown_table(["layer", "example authority", "interpretation"], authority), "",
        "## Bounded compromise and failure typing", "",
        f'* Narrow ±0.01 acceleration bounds: status is `SolvedWithSlack` for every agent; max |qdd| {metrics["bounded"]["maximum_abs_qdd"]:.6f}; soft residual {metrics["bounded"]["maximum_soft_level_l2"]:.3e}; {metrics["bounded"]["clipped_steps"]} clipped steps.',
        f'* Invalid bounds isolate agent 8 with `InvalidProblem`; neighboring commands remain bitwise exact: {metrics["invalid_isolation"]["neighbors_bitwise_exact"]}.',
        "* Contradictory nonzero hard rows are covered by Rust tests as `MaxIterations`, with continuous maximum violation and projection-work counters. Structural zero-row contradiction remains `PrimalInfeasible`. Commands are zeroed for non-solved statuses.", "",
        "## Determinism and allocation", "",
        *markdown_table(["check", "result"], [[k, v] for k, v in metrics["invariance"].items()]), "",
        "## Release timing (untrimmed)", "",
        *markdown_table(["agents", "solve p50 µs", "solve p99 µs", "pipeline p50 µs", "alloc calls"], timing_rows), "",
        "Timers exclude Python/NumPy marshalling and include only preallocated Rust execution. The full-pipeline timer is the sum of separately sampled FK, Jacobian, dynamics, point-query, emission, and exact-solve stages.", "",
        "## Scope boundary", "",
        "This checkpoint proves a stable CPU-exact consumer for point-attractor and contact-lock rows, strict priority, bound compromise, typed bounded-work failure, fixed diagnostics, determinism, and zero measured hot-path allocation. It does not yet add friction cones, torque/power/thermal inequalities to this batch ABI, physics realization, walking retargeting, or a CUDA solver. Those remain explicit subsequent admission gates.", "",
    ])


def main() -> None:
    args = parse_args(); output = pathlib.Path(args.output); output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    oracle_cases, raw = [], {}
    for model in models:
        metrics, arrays = oracle_case(model, args.samples); oracle_cases.append(metrics); raw.update(arrays)
    bounded = bounded_case(models[1]); invalid = invalid_isolation(models[1]); invariance = invariance_case(models[1])
    timing = [timing_case(models[1], agents, args.timing_repeats) for agents in (1, 32, 256)]
    admission = all(x["pass"] for x in oracle_cases) and bounded["pass"] and invalid["pass"] and invariance["pass"] \
        and all(x["allocation_calls"] == 0 and x["allocated_bytes"] == 0 for x in timing)
    metrics = {"schema_version": 1, "revision": "cpu-exact-batch-solve-r80",
               "generated_at": datetime.now(timezone.utc).isoformat(),
               "host": {"platform": platform.platform(), "processor": platform.processor()},
               "semantics": "StrictLexicographic", "scalar_format": "f64", "oracle_cases": oracle_cases,
               "bounded": bounded, "invalid_isolation": invalid, "invariance": invariance, "timing": timing,
               "cuda_hierarchical_solver": {"available": False, "claimed": False}, "admission": bool(admission)}
    np.savez_compressed(output / "cpu-exact-batch-solve-raw.npz", **raw)
    (output / "cpu-exact-batch-solve-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics)
    (output / "CPU_EXACT_BATCH_SOLVE_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw strict CPU batch solve · r80"))
    print(json.dumps(metrics, indent=2))
    if not admission: raise SystemExit("CpuExactF64 batch solve admission failed")


if __name__ == "__main__": main()
