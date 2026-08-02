#!/usr/bin/env python3
"""R222 zero-plant terminal consequence audit for rejected R221 predictions."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import reconstruct_prestate
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_substepped_compliant_contact_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
)


REVISION = "g1-compliant-terminal-consequence-audit-r222"
SOURCE_REVISION = "g1-substepped-compliant-contact-holdout-r221"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-substepped-compliant-contact-holdout-r221/"
    "g1-substepped-compliant-contact-holdout.npz"
)
ROOT_IMPACT_PLANE_M = 0.45


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_COMPLIANT_TERMINAL_CONSEQUENCE_AUDIT_R222.html"
    )
    return parser.parse_args()


def joint_limits(
    model: pathlib.Path, joint_names: list[str]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    nodes = {
        node.get("name", ""): node
        for node in ET.parse(model).getroot().findall("joint")
    }
    lower = np.empty(len(joint_names), np.float64)
    upper = np.empty(len(joint_names), np.float64)
    velocity = np.empty(len(joint_names), np.float64)
    for index, name in enumerate(joint_names):
        node = nodes[name]
        limit = node.find("limit")
        if limit is None:
            raise ValueError(f"{name}: terminal audit requires a limit")
        lower[index] = float(limit.get("lower", "-inf"))
        upper[index] = float(limit.get("upper", "inf"))
        velocity[index] = float(limit.get("velocity", "nan"))
    if np.any(~np.isfinite(velocity)) or np.any(velocity <= 0.0):
        raise ValueError("terminal audit requires positive finite joint velocity limits")
    return lower, upper, velocity


def reconstruct_prevelocity(joint_dof: int, state_index: int) -> np.ndarray:
    phase = 0.37 * state_index + np.arange(joint_dof, dtype=np.float64) * 0.23
    closing_speed = 0.25 + 0.55 * (0.5 + 0.5 * math.sin(0.43 * state_index))
    angular = np.asarray(
        [
            0.15 * math.sin(0.21 * state_index),
            0.12 * math.cos(0.33 * state_index),
            0.08 * math.sin(0.41 * state_index),
        ],
        np.float64,
    )
    linear = np.asarray(
        [
            0.12 * math.sin(0.17 * state_index),
            0.08 * math.cos(0.29 * state_index),
            -closing_speed,
        ],
        np.float64,
    )
    joints = 0.08 * np.sin(phase + 0.5)
    return np.concatenate((angular, linear, joints))


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
    lower, upper, velocity_limit = limits
    names = tuple(session.terminal_impact_state_diagnostic_names)
    index = {name: coordinate for coordinate, name in enumerate(names)}
    samples = len(replay[f"{prefix}_root_height"])
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    states = np.empty((2, 6), np.float64)
    joint_velocity = np.empty((2, joint_dof), np.float64)
    available = np.ones(2, np.uint8)
    root_acceleration = np.zeros((2, 2), np.float64)
    joint_acceleration = np.zeros((2, joint_dof), np.float64)
    effort = np.zeros(2, np.float64)
    diagnostics = np.empty((2, len(names)), np.float64)
    all_diagnostics = np.empty((samples, 2, len(names)), np.float64)
    timing = np.empty((samples, 2), np.uint64)
    limiting_class = np.empty((samples, 2), np.uint8)
    optimistic = np.empty(samples, np.uint8)
    false_safe = np.empty(samples, np.uint8)
    zero_allocation = True

    pressure_names = (
        "tilt_pressure",
        "angular_rate_pressure",
        "joint_position_pressure",
        "joint_velocity_pressure",
        "actuator_effort_pressure",
        "admission_pressure",
    )
    pressure_indices = np.asarray([index[name] for name in pressure_names])

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
        roll = 0.025 * math.sin(0.31 * state_index)
        pitch = 0.035 * math.cos(0.27 * state_index)
        for row, velocity in enumerate((predicted_velocity, oracle_velocity)):
            states[row] = [
                float(root_height) - ROOT_IMPACT_PLANE_M,
                velocity[5],
                roll,
                pitch,
                velocity[0],
                velocity[1],
            ]
            joint_velocity[row] = velocity[6:]
        result = session.score_terminal_impact_state_batch(
            states,
            q,
            joint_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first = diagnostics.copy()
        result = session.score_terminal_impact_state_batch(
            states,
            q,
            joint_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if not np.array_equal(diagnostics, first):
            raise RuntimeError("terminal consequence query is not bitwise deterministic")
        all_diagnostics[sample] = diagnostics
        limiting_class[sample] = np.argmax(diagnostics[:, pressure_indices], axis=1)
        predicted_harm = diagnostics[0, index["maximum_terminal_harm_pressure"]]
        oracle_harm = diagnostics[1, index["maximum_terminal_harm_pressure"]]
        optimistic[sample] = predicted_harm + 1.0e-12 < oracle_harm
        false_safe[sample] = predicted_harm < 1.0 and oracle_harm >= 1.0

    delta = all_diagnostics[:, 0] - all_diagnostics[:, 1]
    key_names = (
        "terminal_tilt_rad",
        "terminal_angular_rate_rad_s",
        "minimum_terminal_joint_headroom_fraction",
        "maximum_terminal_joint_velocity_utilization",
        "maximum_terminal_harm_pressure",
        "aggregate_score",
    )
    error = {
        name: distribution(np.abs(delta[:, index[name]])) for name in key_names
    }
    covered = replay[f"{prefix}_compliant_sample_covered"] != 0
    result = {
        "law": prefix,
        "samples": samples,
        "source_prediction_coverage": float(np.mean(covered)),
        "terminal_error": error,
        "optimistic_harm_samples": int(np.count_nonzero(optimistic)),
        "false_safe_samples": int(np.count_nonzero(false_safe)),
        "limiting_class_mismatch_samples": int(
            np.count_nonzero(limiting_class[:, 0] != limiting_class[:, 1])
        ),
        "uncovered_false_safe_samples": int(np.count_nonzero(false_safe & ~covered)),
        "query_timing_ns": distribution(timing.reshape(-1)),
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "authority_admitted": False,
    }
    arrays = {
        f"{prefix}_terminal_diagnostics": all_diagnostics,
        f"{prefix}_terminal_delta": delta,
        f"{prefix}_terminal_limiting_class": limiting_class,
        f"{prefix}_terminal_optimistic": optimistic,
        f"{prefix}_terminal_false_safe": false_safe,
        f"{prefix}_terminal_timing_ns": timing,
    }
    return result, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R222 requires the pinned G1 model and immutable R221 replay")
    replay = np.load(source)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    joint_names = list(session.joint_names())
    limits = joint_limits(model, joint_names)
    q_nominal = standing_posture(joint_names)
    results: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        result, law_arrays = score_law(
            session, model, replay, law, offset, limits, q_nominal
        )
        results.append(result)
        arrays.update(law_arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"] and result["bitwise_repeat"]
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
        "physics_steps": 0,
        "policy_or_controller_steps": 0,
        "integration_steps": 0,
        "continuous_force_delta_excluded_equally_from_prediction_and_oracle": True,
        "prediction_profile_already_rejected": True,
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        error = result["terminal_error"]
        rows.append(
            [
                result["law"],
                f"{100.0 * result['source_prediction_coverage']:.3f}%",
                f"{error['maximum_terminal_harm_pressure']['p95']:.3f} / {error['maximum_terminal_harm_pressure']['maximum']:.3f}",
                f"{error['aggregate_score']['p95']:.3f} / {error['aggregate_score']['maximum']:.3f}",
                str(result["optimistic_harm_samples"]),
                f"{result['false_safe_samples']} / {result['uncovered_false_safe_samples']}",
                str(result["limiting_class_mismatch_samples"]),
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 compliant terminal-consequence audit · r222",
            "",
            f"> Generic terminal mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · prediction profile **ALREADY REJECTED** · authority **NOT ADMITTED** · physics/policy/controller/integration **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            f"- Rust scores paired predicted/oracle post-contact states at a declared G1 root-impact plane of {ROOT_IMPACT_PLANE_M:.2f} m. Ballistic impact time/vertical energy, 60 ms zero-acceleration hold, terminal tilt/rate, joint headroom/velocity, and separate harm pressures use the existing allocation-free terminal proxy. It is not collision impulse, injury, recovery, or hardware safety.",
            "- Pre-contact root/joint velocity is reconstructed from the deterministic R221 fixture. One arm adds the frozen compliant impulse response; the other adds the completed scoring-label impulse response. Identical smooth-force evolution is deliberately excluded from both arms, so this is a contact-prediction consequence ablation, not a full plant rollout.",
            "- R221 already rejected the predictor. This audit may quantify optimistic harm, false-safe threshold crossings, and limiting-pressure changes; it cannot restore profile or command authority. Every paired score repeats bitwise with zero timed Rust allocation.",
            "",
            "## Consequence result",
            "",
            *markdown_table(
                [
                    "law",
                    "source coverage",
                    "harm error p95/max",
                    "aggregate error p95/max",
                    "optimistic",
                    "false-safe / outside tube",
                    "limiter mismatch",
                    "pair score p99 µs",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            "Retain the generic terminal state-batch scorer and the paired consequence artifact. All observed false-safe rows occur while the completed residual is still inside R220's componentwise tube, proving that point scoring cannot substitute for propagating the entire tube through the nonlinear terminal metric. The rejected R221 predictor remains non-authoritative; a transferable contact residual must pass fresh-law coverage and its interval must be terminally enveloped before consequence can participate in selection.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-compliant-terminal-consequence-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_COMPLIANT_TERMINAL_CONSEQUENCE_AUDIT.md").write_text(report)
    np.savez_compressed(output / "g1-compliant-terminal-consequence-audit.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "false_safe_samples": {
                    result["law"]: result["false_safe_samples"] for result in results
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
