#!/usr/bin/env python3
"""Audit Rust next-tick joint viability envelopes and strict-solve composition."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_exact_batch_solve_report import problem, run_exact, solve_buffers
from cpu_reference_report import distribution, markdown_table, render_report_html
from cuda_batch_mirror_report import joint_states, sha256
from cuda_emission_mirror_report import session_for


DISABLED, OK, INVALID_INPUT = 1, 2, 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--timing-repeats", type=int, default=200)
    parser.add_argument("--output", default="benchmarks/results/joint-viability-envelope-r81")
    parser.add_argument("--web-report", default="web/JOINT_VIABILITY_ENVELOPE_R81.html")
    return parser.parse_args()


def urdf_limits(path: pathlib.Path, joint_names: list[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    root = ET.parse(path).getroot()
    authored: dict[str, tuple[float, float, float]] = {}
    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        kind = joint.attrib.get("type")
        lower = -np.inf if kind == "continuous" else float(limit.attrib.get("lower", "-inf"))
        upper = np.inf if kind == "continuous" else float(limit.attrib.get("upper", "inf"))
        velocity = float(limit.attrib["velocity"])
        authored[joint.attrib["name"]] = (lower, upper, velocity)
    ordered = np.asarray([authored[name] for name in joint_names], dtype=np.float64)
    return ordered[:, 0], ordered[:, 1], ordered[:, 2]


def buffers(session) -> dict[str, np.ndarray]:
    agents, dof, generalized = session.agent_capacity, session.dof, session.generalized_dof
    return {
        "lower": np.empty((agents, generalized), np.float64),
        "upper": np.empty((agents, generalized), np.float64),
        "status": np.empty(agents, np.uint8),
        "position_headroom": np.empty(agents, np.float64),
        "velocity_headroom": np.empty(agents, np.float64),
        "limiting_position": np.empty(agents, np.uint32),
        "limiting_velocity": np.empty(agents, np.uint32),
        "recovery_count": np.empty(agents, np.uint32),
        "execute_ns": np.empty(1, np.uint64),
        "allocation_calls": np.empty(1, np.uint64),
        "allocated_bytes": np.empty(1, np.uint64),
        "maximum_acceleration": np.empty((agents, dof), np.float64),
    }


def run(session, q, v, enabled, dt, maximum_acceleration, out=None):
    result = buffers(session) if out is None else out
    session.run_joint_envelope_batch(
        np.ascontiguousarray(q, np.float64), np.ascontiguousarray(v, np.float64),
        np.ascontiguousarray(enabled, np.uint8), np.ascontiguousarray(dt, np.float64),
        np.ascontiguousarray(maximum_acceleration, np.float64), result["lower"], result["upper"],
        result["status"], result["position_headroom"], result["velocity_headroom"],
        result["limiting_position"], result["limiting_velocity"], result["recovery_count"],
        result["execute_ns"], result["allocation_calls"], result["allocated_bytes"],
    )
    return result


def interval(position, velocity, lower_position, upper_position, maximum_velocity, maximum_acceleration, dt):
    lower = max(-maximum_acceleration, (-maximum_velocity - velocity) / dt)
    upper = min(maximum_acceleration, (maximum_velocity - velocity) / dt)
    if np.isfinite(lower_position):
        distance = max(position - lower_position, 0.0)
        safe_velocity = -np.sqrt(2.0 * maximum_acceleration * distance)
        lower = max(lower, (safe_velocity - velocity) / dt,
                    2.0 * (lower_position - position - velocity * dt) / dt**2)
    if np.isfinite(upper_position):
        distance = max(upper_position - position, 0.0)
        safe_velocity = np.sqrt(2.0 * maximum_acceleration * distance)
        upper = min(upper, (safe_velocity - velocity) / dt,
                    2.0 * (upper_position - position - velocity * dt) / dt**2)
    lower = float(np.clip(lower, -maximum_acceleration, maximum_acceleration))
    upper = float(np.clip(upper, -maximum_acceleration, maximum_acceleration))
    if lower <= upper:
        return lower, upper
    if velocity < 0.0 and np.isfinite(lower_position):
        return maximum_acceleration, maximum_acceleration
    if velocity > 0.0 and np.isfinite(upper_position):
        return -maximum_acceleration, -maximum_acceleration
    raise ValueError("indeterminate invalid interval")


def oracle_case(path: pathlib.Path, samples: int):
    session = session_for(path, samples)
    q = joint_states(samples, session.dof).astype(np.float64)
    v = (0.35 * np.sin(np.arange(samples)[:, None] * 0.19 + np.arange(session.dof)[None, :] * 0.31)).astype(np.float64)
    enabled = np.ones(samples, np.uint8)
    dt = (0.012 + 0.0002 * np.arange(samples)).astype(np.float64)
    amax = (8.0 + 0.1 * np.arange(samples)[:, None] + 0.05 * np.arange(session.dof)[None, :]).astype(np.float64)
    lower_pos, upper_pos, vmax = urdf_limits(path, list(session.joint_names))
    result = run(session, q, v, enabled, dt, amax)
    expected_lower = np.full_like(result["lower"], -np.inf)
    expected_upper = np.full_like(result["upper"], np.inf)
    for agent in range(samples):
        for coordinate in range(session.dof):
            lo, hi = interval(q[agent, coordinate], v[agent, coordinate], lower_pos[coordinate],
                              upper_pos[coordinate], vmax[coordinate], amax[agent, coordinate], dt[agent])
            expected_lower[agent, 6 + coordinate] = lo
            expected_upper[agent, 6 + coordinate] = hi
    finite = np.isfinite(expected_lower) | np.isfinite(expected_upper)
    error = float(max(np.max(np.abs(result["lower"][finite] - expected_lower[finite]), initial=0.0),
                      np.max(np.abs(result["upper"][finite] - expected_upper[finite]), initial=0.0)))
    infinity_exact = bool(np.array_equal(np.isneginf(result["lower"]), np.isneginf(expected_lower))
                          and np.array_equal(np.isposinf(result["upper"]), np.isposinf(expected_upper)))
    passed = bool(error <= 2e-12 and infinity_exact and np.all(result["status"] == OK)
                  and result["allocation_calls"][0] == 0 and result["allocated_bytes"][0] == 0)
    return {"model": str(path), "model_sha256": sha256(path), "samples": samples,
            "maximum_bound_error": error, "infinity_masks_exact": infinity_exact,
            "minimum_position_headroom": float(np.min(result["position_headroom"])),
            "minimum_velocity_headroom": float(np.min(result["velocity_headroom"])),
            "allocation_calls": int(result["allocation_calls"][0]), "allocated_bytes": int(result["allocated_bytes"][0]),
            "pass": passed}


def recovery_case(path: pathlib.Path):
    session = session_for(path, 17)
    q = np.zeros((17, session.dof), np.float64); v = np.zeros_like(q)
    enabled = np.ones(17, np.uint8); dt = np.full(17, 0.02); amax = np.full_like(q, 20.0)
    lower_pos, upper_pos, _ = urdf_limits(path, list(session.joint_names))
    coordinate = int(np.flatnonzero(np.isfinite(upper_pos))[0])
    q[8, coordinate] = upper_pos[coordinate] + 0.1; v[8, coordinate] = 1.0
    result = run(session, q, v, enabled, dt, amax)
    index = 6 + coordinate
    recovery_exact = bool(result["lower"][8, index] == -20.0 and result["upper"][8, index] == -20.0
                          and result["recovery_count"][8] == 1)
    poisoned = amax.copy(); poisoned[9, coordinate] = np.nan
    invalid = run(session, q, v, enabled, dt, poisoned)
    neighbors = np.arange(17) != 9
    isolation = bool(invalid["status"][9] == INVALID_INPUT and np.all(invalid["status"][neighbors] == OK)
                     and np.array_equal(invalid["lower"][neighbors], result["lower"][neighbors])
                     and np.all(np.isneginf(invalid["lower"][9])) and np.all(np.isposinf(invalid["upper"][9])))
    disabled_enabled = enabled.copy(); disabled_enabled[10] = 0
    disabled_amax = amax.copy(); disabled_amax[10].fill(np.nan)
    disabled = run(session, q, v, disabled_enabled, dt, disabled_amax)
    disabled_ignores = bool(disabled["status"][10] == DISABLED and np.all(np.isneginf(disabled["lower"][10]))
                            and np.all(np.isposinf(disabled["upper"][10])))
    return {"coordinate": coordinate, "recovery_acceleration": float(result["lower"][8, index]),
            "recovery_exact": recovery_exact, "invalid_agent_isolation_exact": isolation,
            "disabled_agent_ignores_payload": disabled_ignores,
            "pass": bool(recovery_exact and isolation and disabled_ignores)}


def composition_case(path: pathlib.Path):
    session, q, roots, generalized_velocity, gravity, targets, lower, upper = problem(path, 16)
    targets["contact_active"].fill(0)
    enabled = np.ones(16, np.uint8); dt = np.full(16, 0.02); amax = np.full((16, session.dof), 0.01)
    envelope = run(session, q.astype(np.float64), generalized_velocity[:, 6:].astype(np.float64), enabled, dt, amax)
    lower = np.maximum(lower, envelope["lower"]); upper = np.minimum(upper, envelope["upper"])
    lower[:, :6] = 0.0; upper[:, :6] = 0.0
    solved = run_exact(session, q, roots, generalized_velocity, gravity, targets, lower, upper)
    inside = bool(np.all(solved["qdd"] >= lower - 1e-12) and np.all(solved["qdd"] <= upper + 1e-12))
    margins = np.minimum(solved["qdd"][:, 6:] - envelope["lower"][:, 6:],
                         envelope["upper"][:, 6:] - solved["qdd"][:, 6:])
    passed = bool(inside and np.all(solved["status"] == 2) and np.any(solved["clipped_steps"] > 0)
                  and solved["allocation_calls"][0] == 0)
    return {"all_commands_inside_envelope": inside, "status_all_solved_with_slack": bool(np.all(solved["status"] == 2)),
            "minimum_solved_envelope_margin": float(np.min(margins)), "clipped_steps": int(np.sum(solved["clipped_steps"])),
            "maximum_soft_residual": float(np.max(solved["level_l2"])), "allocation_calls": int(solved["allocation_calls"][0]),
            "pass": passed}


def approach_sweep_case(path: pathlib.Path):
    agents = 64; session = session_for(path, agents)
    lower_pos, upper_pos, _ = urdf_limits(path, list(session.joint_names))
    coordinate = int(np.flatnonzero(np.isfinite(upper_pos))[0])
    q = np.zeros((agents, session.dof), np.float64); v = np.zeros_like(q)
    q[:, coordinate] = np.linspace(upper_pos[coordinate] - 0.5, upper_pos[coordinate] - 1e-5, agents)
    v[:, coordinate] = 0.4
    result = run(session, q, v, np.ones(agents, np.uint8), np.full(agents, 0.02), np.full_like(q, 20.0))
    upper = result["upper"][:, 6 + coordinate]
    monotone = bool(np.all(np.diff(upper) <= 1e-12))
    inward_before_limit = bool(upper[-1] < 0.0)
    return {"coordinate": coordinate, "sample_count": agents, "farthest_upper_qdd": float(upper[0]),
            "nearest_upper_qdd": float(upper[-1]), "monotone_nonincreasing": monotone,
            "requests_inward_before_limit": inward_before_limit,
            "pass": bool(monotone and inward_before_limit and np.all(result["status"] == OK))}


def invariance_case(path: pathlib.Path):
    agents = 17; session = session_for(path, agents)
    q = joint_states(agents, session.dof).astype(np.float64)
    v = (0.2 * np.cos(np.arange(agents)[:, None] + np.arange(session.dof)[None, :] * 0.1)).astype(np.float64)
    enabled = np.ones(agents, np.uint8); dt = np.full(agents, 0.02); amax = np.full_like(q, 12.0)
    base = run(session, q, v, enabled, dt, amax); repeat = run(session, q, v, enabled, dt, amax)
    permutation = np.asarray([*range(agents - 1, -1, -2), *range(agents - 2, -1, -2)])
    permuted = run(session, q[permutation], v[permutation], enabled[permutation], dt[permutation], amax[permutation])
    inverse = np.argsort(permutation)
    compact = run(session_for(path, agents, 1), q, v, enabled, dt, amax)
    fields = ("lower", "upper", "status", "position_headroom", "velocity_headroom", "limiting_position", "limiting_velocity", "recovery_count")
    repeat_exact = all(np.array_equal(base[name], repeat[name]) for name in fields)
    permutation_exact = all(np.array_equal(base[name], permuted[name][inverse]) for name in fields)
    padding_exact = all(np.array_equal(base[name], compact[name]) for name in fields)
    calls = sum(int(x["allocation_calls"][0]) for x in (base, repeat, permuted, compact))
    return {"repeat_bitwise_exact": repeat_exact, "permutation_exact": permutation_exact,
            "padding_exact": padding_exact, "allocation_calls": calls,
            "pass": bool(repeat_exact and permutation_exact and padding_exact and calls == 0)}


def timing_case(path: pathlib.Path, agents: int, repeats: int):
    session = session_for(path, agents); q = joint_states(agents, session.dof).astype(np.float64)
    v = np.zeros_like(q); enabled = np.ones(agents, np.uint8); dt = np.full(agents, 0.02); amax = np.full_like(q, 12.0)
    out = buffers(session)
    for _ in range(10): run(session, q, v, enabled, dt, amax, out)
    trace = np.empty(repeats); calls = allocated = 0
    for index in range(repeats):
        result = run(session, q, v, enabled, dt, amax, out); trace[index] = result["execute_ns"][0]
        calls += int(result["allocation_calls"][0]); allocated += int(result["allocated_bytes"][0])
    return {"batch_size": agents, "repeats": repeats, "execute_ns": distribution(trace),
            "per_agent_execute_ns": {k: value / agents for k, value in distribution(trace).items()},
            "allocation_calls": calls, "allocated_bytes": allocated}


def report_markdown(metrics: dict[str, Any]) -> str:
    oracle_rows = [[pathlib.Path(x["model"]).stem, x["samples"], f'{x["maximum_bound_error"]:.3e}',
                    x["infinity_masks_exact"], x["pass"]] for x in metrics["oracle_cases"]]
    timing_rows = [[x["batch_size"], f'{x["execute_ns"]["p50"] / 1e3:.3f}',
                    f'{x["execute_ns"]["p99"] / 1e3:.3f}', f'{x["per_agent_execute_ns"]["p50"]:.1f}',
                    x["allocation_calls"]] for x in metrics["timing"]]
    authority = [
        ["Invariant", "contact locks 7951/7952", "remain hard equalities in the downstream strict solve"],
        ["Viability", "joint stopping envelope", "position, velocity, next-tick position, and braking distance intersect as hard qdd bounds"],
        ["Intent", "point target 7902", "retains residual when the joint envelope consumes authority"],
        ["Recovery", "maximum inward qdd", "an already-unsafe observation yields a deterministic recovery box, never an inverted interval"],
        ["Resources", "actuator acceleration only when explicitly authored", "URDF effort is not reinterpreted as acceleration or torque feasibility"],
        ["Dynamic feasibility", "UNAVAILABLE in this batch slice", "effort/friction/support require [qdd, tau, f] and remain the next admission"],
    ]
    return "\n".join([
        "# Bonesaw joint viability envelope · r81", "", "## Outcome", "",
        f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** Rust now derives fixed-shape next-tick joint acceleration envelopes from observed position/velocity, authored position/velocity limits, explicit acceleration authority, and control period. The envelope feeds `CpuExactF64` directly in Rust; Python independently reconstructs the algebra and orchestrates evaluation.', "",
        "This is kinematic safety evidence, not actuator-effort or contact-force feasibility. Those require the dynamic decision vector and are deliberately not inferred from this result.", "",
        "## Independent URDF/NumPy oracle", "", *markdown_table(["model", "states", "max bound error", "∞ masks exact", "pass"], oracle_rows), "",
        "## Example authority stack", "", *markdown_table(["layer", "concrete signal", "meaning"], authority), "",
        "## Recovery and isolation", "",
        f'- Outside-limit coordinate {metrics["recovery"]["coordinate"]} receives exactly {metrics["recovery"]["recovery_acceleration"]:.1f} rad/s² inward acceleration authority.',
        f'- Invalid-agent isolation exact: {metrics["recovery"]["invalid_agent_isolation_exact"]}; disabled payload ignored: {metrics["recovery"]["disabled_agent_ignores_payload"]}.', "",
        "## Continuous approach sweep", "", *markdown_table(["check", "result"], [[k, v] for k, v in metrics["approach_sweep"].items()]), "",
        "## Strict-solve composition", "", *markdown_table(["check", "result"], [[k, v] for k, v in metrics["composition"].items()]), "",
        "## Determinism", "", *markdown_table(["check", "result"], [[k, v] for k, v in metrics["invariance"].items()]), "",
        "## Release timing", "", *markdown_table(["agents", "p50 µs", "p99 µs", "p50 ns/agent", "alloc calls"], timing_rows), "",
        "Timers cover the preallocated Rust envelope executor only and exclude NumPy marshalling. All 200 calls per batch are retained.", "",
        "## Next physical gate", "",
        "The next batch slice must extend the decision vector to `[q̈, τ, contact force]`, enforce `M q̈ + h = Sᵀτ + Jᶜᵀf`, and only then add exact actuator effort, unilateral force, friction, and finite-support inequalities. Keeping that boundary explicit prevents a kinematically reachable request from being mislabeled physically realizable.", "",
    ])


def main() -> None:
    args = parse_args(); output = pathlib.Path(args.output); output.mkdir(parents=True, exist_ok=True)
    models = [pathlib.Path("models/toy_humanoid.urdf"), pathlib.Path("models/upkie/upkie.urdf")]
    oracle_cases = [oracle_case(model, args.samples) for model in models]
    recovery = recovery_case(models[1]); approach_sweep = approach_sweep_case(models[1])
    composition = composition_case(models[1]); invariance = invariance_case(models[1])
    timing = [timing_case(models[1], agents, args.timing_repeats) for agents in (1, 32, 256)]
    admission = all(x["pass"] for x in oracle_cases) and recovery["pass"] and approach_sweep["pass"] and composition["pass"] and invariance["pass"] \
        and all(x["allocation_calls"] == 0 and x["allocated_bytes"] == 0 for x in timing)
    metrics = {"schema_version": 1, "revision": "joint-viability-envelope-r81",
               "generated_at": datetime.now(timezone.utc).isoformat(),
               "host": {"platform": platform.platform(), "processor": platform.processor()},
               "oracle_cases": oracle_cases, "recovery": recovery, "approach_sweep": approach_sweep, "composition": composition,
               "invariance": invariance, "timing": timing,
               "dynamic_effort_force_feasibility": {"available": False, "claimed": False}, "admission": bool(admission)}
    (output / "joint-viability-envelope-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report_markdown(metrics); (output / "JOINT_VIABILITY_ENVELOPE_AUDIT.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(render_report_html(markdown, title="Bonesaw joint viability envelope · r81"))
    print(json.dumps(metrics, indent=2))
    if not admission: raise SystemExit("joint viability envelope admission failed")


if __name__ == "__main__": main()
