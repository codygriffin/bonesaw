#!/usr/bin/env python3
"""R224 zero-plant propagation of the frozen R220 velocity tube to terminal harm."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import reconstruct_prestate
from g1_compliant_terminal_consequence_audit import (
    ROOT_IMPACT_PLANE_M,
    reconstruct_prevelocity,
    joint_limits,
)
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_substepped_compliant_contact_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROFILE,
    SAMPLE_OFFSETS,
    residual_half_width,
)


REVISION = "g1-terminal-velocity-box-audit-r224"
SOURCE_REVISION = "g1-substepped-compliant-contact-holdout-r221"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-substepped-compliant-contact-holdout-r221/"
    "g1-substepped-compliant-contact-holdout.npz"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_TERMINAL_VELOCITY_BOX_AUDIT_R224.html"
    )
    return parser.parse_args()


def terminal_relevant_indices(generalized_dof: int) -> np.ndarray:
    if generalized_dof < 6:
        raise ValueError("terminal velocity projection requires a floating root")
    return np.asarray([0, 1, 5, *range(6, generalized_dof)], np.int64)


def score_law(
    session: Any,
    model: pathlib.Path,
    replay: Any,
    law: Any,
    offset: int,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray],
    q_nominal: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    joint_dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    relevant = terminal_relevant_indices(generalized_dof)
    lower, upper, velocity_limit = limits
    names = tuple(session.terminal_impact_state_diagnostic_names)
    index = {name: coordinate for coordinate, name in enumerate(names)}
    samples = len(replay[f"{prefix}_root_height"])
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    point_states = np.empty((2, 6), np.float64)
    point_joint_velocity = np.empty((2, joint_dof), np.float64)
    box_root_state = np.empty((1, 3), np.float64)
    box_root_lower = np.empty((1, 3), np.float64)
    box_root_upper = np.empty((1, 3), np.float64)
    box_joint_lower = np.empty((1, joint_dof), np.float64)
    box_joint_upper = np.empty((1, joint_dof), np.float64)
    available_pair = np.ones(2, np.uint8)
    available_box = np.ones(1, np.uint8)
    root_acceleration_pair = np.zeros((2, 2), np.float64)
    root_acceleration_box = np.zeros((1, 2), np.float64)
    joint_acceleration_pair = np.zeros((2, joint_dof), np.float64)
    joint_acceleration_box = np.zeros((1, joint_dof), np.float64)
    effort_pair = np.zeros(2, np.float64)
    effort_box = np.zeros(1, np.float64)
    point_diagnostics = np.empty((2, len(names)), np.float64)
    box_diagnostics = np.empty((1, len(names)), np.float64)
    all_point_diagnostics = np.empty((samples, 2, len(names)), np.float64)
    all_box_diagnostics = np.empty((samples, len(names)), np.float64)
    pair_timing = np.empty((samples, 2), np.uint64)
    box_timing = np.empty((samples, 2), np.uint64)
    full_inside = np.empty(samples, np.uint8)
    relevant_inside = np.empty(samples, np.uint8)
    bound_violation = np.empty(samples, np.uint8)
    half_width = residual_half_width(generalized_dof)
    zero_allocation = True

    upper_bounded_names = (
        "time_to_impact_s",
        "vertical_specific_impact_energy_j_kg",
        "terminal_tilt_rad",
        "terminal_angular_rate_rad_s",
        "maximum_terminal_joint_velocity_utilization",
        "impact_speed_pressure",
        "tilt_pressure",
        "angular_rate_pressure",
        "joint_position_pressure",
        "joint_velocity_pressure",
        "actuator_effort_pressure",
        "admission_pressure",
        "maximum_terminal_harm_pressure",
        "aggregate_score",
    )
    upper_indices = np.asarray([index[name] for name in upper_bounded_names])
    headroom_index = index["minimum_terminal_joint_headroom_fraction"]

    for sample, root_height in enumerate(replay[f"{prefix}_root_height"]):
        state_index = offset + sample
        root, quaternion, q = reconstruct_prestate(
            joint_dof, q_nominal, state_index, float(root_height)
        )
        points = replay[f"{prefix}_contact_points"][sample]
        session.point_impulse_velocity_response(
            root,
            quaternion,
            q,
            points,
            bases,
            response,
            effective_mass,
            delassus,
        )
        prevelocity = reconstruct_prevelocity(joint_dof, state_index)
        predicted_velocity = prevelocity + np.einsum(
            "dca,ca->d",
            response,
            replay[f"{prefix}_compliant_predicted_impulse"][sample],
            optimize=True,
        )
        oracle_velocity = prevelocity + np.einsum(
            "dca,ca->d",
            response,
            replay[f"{prefix}_contact_impulse"][sample],
            optimize=True,
        )
        velocity_lower = predicted_velocity - half_width
        velocity_upper = predicted_velocity + half_width
        roll = 0.025 * math.sin(0.31 * state_index)
        pitch = 0.035 * math.cos(0.27 * state_index)
        for row, velocity in enumerate((predicted_velocity, oracle_velocity)):
            point_states[row] = [
                float(root_height) - ROOT_IMPACT_PLANE_M,
                velocity[5],
                roll,
                pitch,
                velocity[0],
                velocity[1],
            ]
            point_joint_velocity[row] = velocity[6:]
        box_root_state[0] = [float(root_height) - ROOT_IMPACT_PLANE_M, roll, pitch]
        box_root_lower[0] = [velocity_lower[5], velocity_lower[0], velocity_lower[1]]
        box_root_upper[0] = [velocity_upper[5], velocity_upper[0], velocity_upper[1]]
        box_joint_lower[0] = velocity_lower[6:]
        box_joint_upper[0] = velocity_upper[6:]

        result = session.score_terminal_impact_state_batch(
            point_states,
            q,
            point_joint_velocity,
            lower,
            upper,
            velocity_limit,
            available_pair,
            root_acceleration_pair,
            joint_acceleration_pair,
            effort_pair,
            point_diagnostics,
        )
        pair_timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first_points = point_diagnostics.copy()
        result = session.score_terminal_impact_state_batch(
            point_states,
            q,
            point_joint_velocity,
            lower,
            upper,
            velocity_limit,
            available_pair,
            root_acceleration_pair,
            joint_acceleration_pair,
            effort_pair,
            point_diagnostics,
        )
        pair_timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if not np.array_equal(first_points, point_diagnostics):
            raise RuntimeError("terminal point score is not bitwise deterministic")

        box_arguments = (
            box_root_state,
            box_root_lower,
            box_root_upper,
            q,
            box_joint_lower,
            box_joint_upper,
            lower,
            upper,
            velocity_limit,
            available_box,
            root_acceleration_box,
            joint_acceleration_box,
            effort_box,
            box_diagnostics,
        )
        result = session.score_terminal_impact_velocity_box_batch(*box_arguments)
        box_timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first_box = box_diagnostics.copy()
        result = session.score_terminal_impact_velocity_box_batch(*box_arguments)
        box_timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if not np.array_equal(first_box, box_diagnostics):
            raise RuntimeError("terminal velocity-box score is not bitwise deterministic")

        full_inside[sample] = np.all(
            (oracle_velocity >= velocity_lower - 1.0e-12)
            & (oracle_velocity <= velocity_upper + 1.0e-12)
        )
        relevant_inside[sample] = np.all(
            (oracle_velocity[relevant] >= velocity_lower[relevant] - 1.0e-12)
            & (oracle_velocity[relevant] <= velocity_upper[relevant] + 1.0e-12)
        )
        oracle_diagnostics = point_diagnostics[1]
        bound_violation[sample] = bool(
            np.any(
                oracle_diagnostics[upper_indices]
                > box_diagnostics[0, upper_indices] + 1.0e-12
            )
            or oracle_diagnostics[headroom_index]
            < box_diagnostics[0, headroom_index] - 1.0e-12
        )
        all_point_diagnostics[sample] = point_diagnostics
        all_box_diagnostics[sample] = box_diagnostics[0]

    predicted_harm = all_point_diagnostics[:, 0, index["maximum_terminal_harm_pressure"]]
    oracle_harm = all_point_diagnostics[:, 1, index["maximum_terminal_harm_pressure"]]
    bounded_harm = all_box_diagnostics[:, index["maximum_terminal_harm_pressure"]]
    center_false_safe = (predicted_harm < 1.0) & (oracle_harm >= 1.0)
    bound_false_safe = (bounded_harm < 1.0) & (oracle_harm >= 1.0)
    conservative_reject = (bounded_harm >= 1.0) & (oracle_harm < 1.0)
    contained_violation = (relevant_inside != 0) & (bound_violation != 0)
    contained_slack = (bounded_harm - oracle_harm)[relevant_inside != 0]
    result = {
        "law": prefix,
        "samples": samples,
        "source_sample_coverage": float(
            np.mean(replay[f"{prefix}_compliant_sample_covered"] != 0)
        ),
        "contact_oracle_full_box_coverage": float(np.mean(full_inside)),
        "contact_oracle_terminal_projection_coverage": float(np.mean(relevant_inside)),
        "center_false_safe_samples": int(np.count_nonzero(center_false_safe)),
        "box_false_safe_samples": int(np.count_nonzero(bound_false_safe)),
        "contained_box_false_safe_samples": int(
            np.count_nonzero(bound_false_safe & (relevant_inside != 0))
        ),
        "conservative_reject_samples": int(np.count_nonzero(conservative_reject)),
        "contained_bound_violations": int(np.count_nonzero(contained_violation)),
        "harm_upper_slack": distribution(bounded_harm - oracle_harm),
        "contained_harm_upper_slack": distribution(contained_slack),
        "point_query_timing_ns": distribution(pair_timing.reshape(-1)),
        "box_query_timing_ns": distribution(box_timing.reshape(-1)),
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "profile_promoted": False,
        "authority_admitted": False,
    }
    arrays = {
        f"{prefix}_point_terminal_diagnostics": all_point_diagnostics,
        f"{prefix}_box_terminal_diagnostics": all_box_diagnostics,
        f"{prefix}_contact_oracle_full_box_covered": full_inside,
        f"{prefix}_contact_oracle_terminal_projection_covered": relevant_inside,
        f"{prefix}_terminal_bound_violation": bound_violation,
        f"{prefix}_center_false_safe": center_false_safe.astype(np.uint8),
        f"{prefix}_box_false_safe": bound_false_safe.astype(np.uint8),
        f"{prefix}_conservative_reject": conservative_reject.astype(np.uint8),
        f"{prefix}_point_timing_ns": pair_timing,
        f"{prefix}_box_timing_ns": box_timing,
    }
    return result, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R224 requires the pinned G1 model and immutable R221 replay")
    replay = np.load(source)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    limits = joint_limits(model, list(session.joint_names()))
    q_nominal = standing_posture(list(session.joint_names()))
    results: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        result, law_arrays = score_law(
            session, model, replay, law, offset, limits, q_nominal
        )
        results.append(result)
        arrays.update(law_arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"]
        and result["bitwise_repeat"]
        and result["contained_bound_violations"] == 0
        and result["contained_box_false_safe_samples"] == 0
        for result in results
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": sum(result["samples"] for result in results),
        "root_impact_plane_m": ROOT_IMPACT_PLANE_M,
        "frozen_profile": FROZEN_PROFILE,
        "physics_steps": 0,
        "policy_or_controller_steps": 0,
        "integration_steps": 0,
        "continuous_force_delta_excluded_equally_from_prediction_and_oracle": True,
        "componentwise_outer_bound_not_reachability_or_probability": True,
        "prediction_profile_already_rejected": True,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        rows.append(
            [
                result["law"],
                f"{100.0 * result['source_sample_coverage']:.3f}%",
                f"{100.0 * result['contact_oracle_terminal_projection_coverage']:.3f}%",
                f"{result['center_false_safe_samples']} → {result['box_false_safe_samples']}",
                str(result["contained_box_false_safe_samples"]),
                f"{result['conservative_reject_samples']} / {result['samples']}",
                f"{result['contained_harm_upper_slack']['p50']:.3f} / {result['contained_harm_upper_slack']['p95']:.3f}",
                f"{result['box_query_timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 terminal velocity-box audit · r224",
            "",
            f"> Velocity-box mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen prediction profile **ALREADY REJECTED** · authority **NOT ADMITTED** · physics/policy/controller/integration **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            f"- Rust propagates the entire frozen R220 generalized-velocity box to the declared {ROOT_IMPACT_PLANE_M:.2f} m root-impact plane. Ballistic time is bounded from vertical-velocity endpoints; tilt, angular rate, joint position, and joint velocity use allocation-free interval arithmetic across that time interval. Pressure and aggregate fields are upper bounds; joint headroom is a lower bound.",
            "- The box is componentwise and distribution-free. It assigns no probability, does not assert every Cartesian corner is reachable, omits yaw/horizontal velocity because the declared terminal proxy does not consume them, and remains only a collision-consequence proxy—not injury, recovery, or hardware safety.",
            "- The paired predicted/completed contact states are exactly the R222 zero-plant ablation. Smooth-force evolution is excluded identically. Every contained completed state must be bounded componentwise; all queries repeat bitwise and allocate nothing in the timed Rust path.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "law",
                    "source full coverage",
                    "terminal projection coverage",
                    "center → box false-safe",
                    "contained false-safe",
                    "conservative rejects",
                    "contained harm slack p50/p95",
                    "box p99 µs",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            "Retain the generic velocity-box terminal bound: it removes false-safe decisions for every completed state actually contained by the declared terminal projection, with deterministic allocation-free execution. Do not admit the frozen R220/R221 profile: it missed fresh-law rows before terminal scoring, and the conservative box can reject safe completed states. The next gate is therefore a tighter transferable transition set—not more permissive terminal thresholding.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-terminal-velocity-box-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_TERMINAL_VELOCITY_BOX_AUDIT.md").write_text(report)
    np.savez_compressed(output / "g1-terminal-velocity-box-audit.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "center_to_box_false_safe": {
                    result["law"]: [
                        result["center_false_safe_samples"],
                        result["box_false_safe_samples"],
                    ]
                    for result in results
                },
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
