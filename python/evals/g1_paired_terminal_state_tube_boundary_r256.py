#!/usr/bin/env python3
"""R256 policy-/physics-free paired terminal-state tube mechanism audit."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture


REVISION = "g1-paired-terminal-state-tube-boundary-r256"
CANDIDATES = 3
HYPOTHESES = 4
POINTS_PER_TUBE = 32
REPEAT_CALLS = 500


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_PAIRED_TERMINAL_STATE_TUBE_BOUNDARY_R256.html",
    )
    return parser.parse_args()


def soft_pressure(value: float, soft_limit: float) -> float:
    return max((value - soft_limit) / (1.0 - soft_limit), 0.0)


def consequence_components(
    root: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    effort: float,
    lower: np.ndarray,
    upper: np.ndarray,
    velocity_limit: np.ndarray,
) -> tuple[np.ndarray, float]:
    finite = np.isfinite(lower) & np.isfinite(upper)
    if np.any(finite):
        span = upper[finite] - lower[finite]
        headroom = np.min(
            np.minimum(q[finite] - lower[finite], upper[finite] - q[finite]) / span
        )
    else:
        headroom = 0.5
    tilt = float(np.linalg.norm(root[:2]) / np.deg2rad(45.0))
    angular_rate = float(np.linalg.norm(root[2:]) / 4.0)
    joint_position = max((0.10 - float(headroom)) / 0.10, 0.0)
    joint_velocity = soft_pressure(float(np.max(np.abs(v) / velocity_limit)), 0.70)
    effort_pressure = soft_pressure(float(effort), 0.65)
    components = np.asarray(
        [tilt, angular_rate, joint_position, joint_velocity, effort_pressure, -headroom],
        np.float64,
    )
    aggregate = (
        tilt
        + 0.5 * angular_rate
        + 0.5 * joint_position
        + 0.25 * joint_velocity
        + 0.15 * effort_pressure
    )
    return components, aggregate


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit("R256 requires the pinned G1 model")
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]])
    joint_names = list(session.joint_names())
    joints = int(session.joint_dof())
    q_nominal = standing_posture(joint_names)
    position_limit_lower, position_limit_upper, velocity_limit = joint_limits(
        model, joint_names
    )

    hypothesis = np.arange(HYPOTHESES, dtype=np.float64)
    coordinate = np.arange(joints, dtype=np.float64)
    root_center = np.column_stack(
        (
            0.10 * np.sin(0.7 * hypothesis),
            0.12 * np.cos(0.5 * hypothesis),
            0.8 * np.sin(0.4 * hypothesis),
            0.7 * np.cos(0.6 * hypothesis),
        )
    )
    root_radius = np.asarray([0.025, 0.025, 0.14, 0.14], np.float64)
    baseline_root_lower = np.ascontiguousarray(root_center - root_radius)
    baseline_root_upper = np.ascontiguousarray(root_center + root_radius)
    root_delta_center = np.zeros((CANDIDATES, HYPOTHESES, 4), np.float64)
    root_delta_center[1, :, :2] = -0.035 * np.sign(root_center[:, :2])
    root_delta_center[1, :, 2:] = -0.16 * np.sign(root_center[:, 2:])
    root_delta_center[2] = 1.7 * root_delta_center[1]
    root_delta_radius = np.zeros_like(root_delta_center)
    root_delta_radius[1] = np.asarray([0.008, 0.008, 0.04, 0.04])
    root_delta_radius[2] = np.asarray([0.014, 0.014, 0.07, 0.07])
    root_delta_lower = np.ascontiguousarray(root_delta_center - root_delta_radius)
    root_delta_upper = np.ascontiguousarray(root_delta_center + root_delta_radius)

    phase = 0.31 * hypothesis[:, None] + 0.17 * coordinate[None, :]
    q_center = q_nominal[None, :] + 0.025 * np.sin(phase)
    q_radius = 0.008 + 0.002 * ((hypothesis[:, None] + coordinate) % 3.0)
    q_lower = np.ascontiguousarray(q_center - q_radius)
    q_upper = np.ascontiguousarray(q_center + q_radius)
    q_delta_center = np.zeros((CANDIDATES, HYPOTHESES, joints), np.float64)
    q_delta_center[1] = -0.010 * np.sin(phase)
    q_delta_center[2] = -0.018 * np.sin(phase)
    q_delta_radius = np.zeros_like(q_delta_center)
    q_delta_radius[1] = 0.003
    q_delta_radius[2] = 0.006
    q_delta_lower = np.ascontiguousarray(q_delta_center - q_delta_radius)
    q_delta_upper = np.ascontiguousarray(q_delta_center + q_delta_radius)

    v_center = 0.24 * np.sin(phase + 0.4)
    v_radius = 0.06 + 0.01 * ((hypothesis[:, None] + coordinate) % 2.0)
    v_lower = np.ascontiguousarray(v_center - v_radius)
    v_upper = np.ascontiguousarray(v_center + v_radius)
    v_delta_center = np.zeros((CANDIDATES, HYPOTHESES, joints), np.float64)
    v_delta_center[1] = -0.10 * np.sign(v_center)
    v_delta_center[2] = -0.16 * np.sign(v_center)
    v_delta_radius = np.zeros_like(v_delta_center)
    v_delta_radius[1] = 0.025
    v_delta_radius[2] = 0.045
    v_delta_lower = np.ascontiguousarray(v_delta_center - v_delta_radius)
    v_delta_upper = np.ascontiguousarray(v_delta_center + v_delta_radius)

    available = np.ones((CANDIDATES, HYPOTHESES), np.uint8)
    baseline_effort = 0.30 + 0.03 * hypothesis
    candidate_effort = np.tile(baseline_effort, (CANDIDATES, 1))
    candidate_effort[1] += 0.08
    candidate_effort[2] += 0.14
    hypothesis_bounds = np.empty((CANDIDATES, HYPOTHESES, 14), np.float64)
    envelope_bounds = np.empty((CANDIDATES, 14), np.float64)
    selection = np.empty(6, np.float64)
    timing_ns = np.empty(REPEAT_CALLS, np.uint64)
    allocation_calls = np.empty(REPEAT_CALLS, np.uint64)
    allocated_bytes = np.empty(REPEAT_CALLS, np.uint64)
    reference = None
    semantic_repeat = True
    for repeat in range(REPEAT_CALLS):
        timing = session.bound_terminal_impact_paired_state_tubes(
            baseline_root_lower,
            baseline_root_upper,
            root_delta_lower,
            root_delta_upper,
            q_lower,
            q_upper,
            q_delta_lower,
            q_delta_upper,
            v_lower,
            v_upper,
            v_delta_lower,
            v_delta_upper,
            position_limit_lower,
            position_limit_upper,
            velocity_limit,
            available,
            baseline_effort,
            candidate_effort,
            0,
            0.0,
            0.01,
            hypothesis_bounds,
            envelope_bounds,
            selection,
        )
        timing_ns[repeat], allocation_calls[repeat], allocated_bytes[repeat] = timing
        current = (hypothesis_bounds.copy(), envelope_bounds.copy(), selection.copy())
        if reference is None:
            reference = current
        else:
            semantic_repeat &= all(
                np.array_equal(left, right) for left, right in zip(reference, current)
            )

    rng = np.random.default_rng(25_600_002)
    lower_slack = []
    upper_slack = []
    aggregate_lower_slack = []
    aggregate_upper_slack = []
    nonbaseline_slack = []
    for candidate in range(CANDIDATES):
        for h in range(HYPOTHESES):
            for _ in range(POINTS_PER_TUBE):
                root = rng.uniform(baseline_root_lower[h], baseline_root_upper[h])
                q = rng.uniform(q_lower[h], q_upper[h])
                v = rng.uniform(v_lower[h], v_upper[h])
                root_delta = rng.uniform(root_delta_lower[candidate, h], root_delta_upper[candidate, h])
                q_delta = rng.uniform(q_delta_lower[candidate, h], q_delta_upper[candidate, h])
                v_delta = rng.uniform(v_delta_lower[candidate, h], v_delta_upper[candidate, h])
                baseline_components, baseline_aggregate = consequence_components(
                    root,
                    q,
                    v,
                    baseline_effort[h],
                    position_limit_lower,
                    position_limit_upper,
                    velocity_limit,
                )
                candidate_components, candidate_aggregate = consequence_components(
                    root + root_delta,
                    q + q_delta,
                    v + v_delta,
                    candidate_effort[candidate, h],
                    position_limit_lower,
                    position_limit_upper,
                    velocity_limit,
                )
                delta = candidate_components - baseline_components
                aggregate_delta = candidate_aggregate - baseline_aggregate
                lower_slack.append(delta - hypothesis_bounds[candidate, h, :6])
                upper_slack.append(hypothesis_bounds[candidate, h, 6:12] - delta)
                aggregate_lower_slack.append(
                    aggregate_delta - hypothesis_bounds[candidate, h, 12]
                )
                aggregate_upper_slack.append(
                    hypothesis_bounds[candidate, h, 13] - aggregate_delta
                )
                if candidate != 0:
                    nonbaseline_slack.extend(
                        (
                            float(np.min(lower_slack[-1])),
                            float(np.min(upper_slack[-1])),
                            float(aggregate_lower_slack[-1]),
                            float(aggregate_upper_slack[-1]),
                        )
                    )
    minimum_slack = min(
        float(np.min(lower_slack)),
        float(np.min(upper_slack)),
        float(np.min(aggregate_lower_slack)),
        float(np.min(aggregate_upper_slack)),
    )
    point_containment = minimum_slack >= -1.0e-12
    zero_allocation = bool(
        np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
    )
    exact_zero_baseline = bool(
        np.array_equal(hypothesis_bounds[0], np.zeros((HYPOTHESES, 14)))
        and np.array_equal(envelope_bounds[0], np.zeros(14))
    )
    mechanism_passed = bool(
        semantic_repeat and zero_allocation and point_containment and exact_zero_baseline
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "candidate_count": CANDIDATES,
        "hypothesis_count": HYPOTHESES,
        "points_per_tube": POINTS_PER_TUBE,
        "independent_point_queries": CANDIDATES * HYPOTHESES * POINTS_PER_TUBE,
        "repeat_calls": REPEAT_CALLS,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "semantic_repeat": semantic_repeat,
        "zero_rust_allocation": zero_allocation,
        "exact_zero_baseline": exact_zero_baseline,
        "point_containment": point_containment,
        "minimum_containment_slack": minimum_slack,
        "minimum_nonbaseline_containment_slack": float(min(nonbaseline_slack)),
        "batch_timing_ns": distribution(timing_ns),
        "selected_index": int(selection[0]),
        "unsafe_upper_subtraction_used": False,
        "mechanism_passed": mechanism_passed,
        "profile_frozen": False,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw paired terminal-state tube boundary · r256",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · profile **NOT FROZEN** · authority **NOT ADMITTED**.",
            "",
            "R256 bounds candidate-minus-baseline consequence directly over a shared terminal-state tube. Each candidate is the same uncertain baseline plus a candidate delta; Rust never subtracts independent score uppers. Tilt, angular rate, joint-position pressure, joint-velocity pressure, actuator effort, raw headroom loss, and aggregate consequence remain separate.",
            "",
            f"A fixed 3-candidate × 4-hypothesis G1-shaped batch repeats {REPEAT_CALLS} times at p50/p99 {result['batch_timing_ns']['p50'] / 1e3:.2f}/{result['batch_timing_ns']['p99'] / 1e3:.2f} µs with zero measured Rust allocation. All {result['independent_point_queries']:,} independently sampled paired points are contained; minimum slack is {minimum_slack:.3g}. The baseline is exactly zero by construction.",
            "",
            "This freezes only the correlation-preserving mechanism. No contact-law/estimator tube has been fitted, no action profile is selected, and authority remains closed. The next evaluation must fit a causal tube on spent state evidence before any new-law holdout.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-paired-terminal-state-tube-boundary-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-paired-terminal-state-tube-boundary.npz",
        baseline_root_lower=baseline_root_lower,
        baseline_root_upper=baseline_root_upper,
        candidate_root_delta_lower=root_delta_lower,
        candidate_root_delta_upper=root_delta_upper,
        hypothesis_bounds=hypothesis_bounds,
        envelope_bounds=envelope_bounds,
        selection=selection,
        timing_ns=timing_ns,
    )
    (output / "G1_PAIRED_TERMINAL_STATE_TUBE_BOUNDARY.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw paired state tube · r256"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_frozen": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
