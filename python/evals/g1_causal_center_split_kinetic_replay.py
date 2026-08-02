#!/usr/bin/env python3
"""R216 zero-plant replay of a causal contact-center plus split kinetic residual.

The immutable R215 corpus supplies completed contact labels for scoring only.
This evaluator reconstructs each pre-impact model state, predicts one resultant
point impulse per foot from pre-step gap/velocity and the declared contact-law
time constant, and asks Rust for all model responses and split kinetic support.
No policy, controller, integration, or physics step is executed here.
"""

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
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    GRAVITY,
    WIDTH_GATES,
    group_maximum,
    quaternion_from_rpy,
    sha256,
    standing_posture,
)
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


REVISION = "g1-causal-center-split-kinetic-replay-r216"
SOURCE_REVISION = "g1-spatial-patch-transition-holdout-r215"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-spatial-patch-transition-holdout-r215/"
    "g1-spatial-patch-transition-replay.npz"
)
RESTITUTION_UPPER = 1.0
ENVELOPE_SLOPE_MAX = 20.0
ENVELOPE_SLOPE_STEPS = 4_001


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_CAUSAL_CENTER_SPLIT_KINETIC_REPLAY_R216.html",
    )
    return parser.parse_args()


def total_urdf_mass(model: pathlib.Path) -> float:
    masses = [
        float(node.get("value", "nan"))
        for node in ET.parse(model).getroot().findall(".//inertial/mass")
    ]
    total = float(sum(masses))
    if not masses or not math.isfinite(total) or total <= 0.0:
        raise ValueError("URDF needs finite positive authored inertial mass")
    return total


def reconstruct_prestate(
    joint_dof: int,
    q_nominal: np.ndarray,
    state_index: int,
    root_height: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phase = 0.37 * state_index + np.arange(joint_dof, dtype=np.float64) * 0.23
    q = q_nominal + 0.025 * np.sin(phase)
    quaternion = quaternion_from_rpy(
        0.025 * math.sin(0.31 * state_index),
        0.035 * math.cos(0.27 * state_index),
        0.02 * math.sin(0.19 * state_index),
    )
    return np.asarray([0.0, 0.0, root_height], np.float64), quaternion, q


def causal_center_prediction(
    points_world: np.ndarray,
    velocities_world: np.ndarray,
    point_effective_mass: np.ndarray,
    friction: float,
    relaxation_time_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Predict two point impulses using only the prospective four-corner state."""

    points = np.asarray(points_world, np.float64)
    velocities = np.asarray(velocities_world, np.float64)
    effective_mass = np.asarray(point_effective_mass, np.float64)
    if (
        points.shape != (8, 3)
        or velocities.shape != (8, 3)
        or effective_mass.shape != (8, 3)
        or not np.all(np.isfinite(points))
        or not np.all(np.isfinite(velocities))
        or not np.all(np.isfinite(effective_mass))
        or np.any(effective_mass <= 0.0)
        or not math.isfinite(friction)
        or friction < 0.0
        or not math.isfinite(relaxation_time_s)
        or relaxation_time_s <= 0.0
    ):
        raise ValueError("invalid causal center witness")

    centers = np.empty((2, 3), np.float64)
    center_velocities = np.empty((2, 3), np.float64)
    normal_impulse = np.zeros(2, np.float64)
    remaining_time = np.zeros((2, 4), np.float64)
    for foot in range(2):
        sl = slice(4 * foot, 4 * foot + 4)
        gap = np.maximum(points[sl, 2], 0.0)
        closing = np.maximum(-velocities[sl, 2], 0.0)
        time_to_impact = np.full(4, np.inf, np.float64)
        moving = closing > 1.0e-12
        time_to_impact[moving] = gap[moving] / closing[moving]
        remaining = np.maximum(CONTROL_DT - time_to_impact, 0.0)
        remaining_time[foot] = remaining
        relaxation = 1.0 - np.exp(-remaining / relaxation_time_s)
        point_normal_impulse = (
            (1.0 + RESTITUTION_UPPER)
            * relaxation
            * effective_mass[sl, 2]
            * closing
        )
        weight_sum = float(np.sum(point_normal_impulse))
        if weight_sum > 1.0e-15:
            weights = point_normal_impulse / weight_sum
            normal_impulse[foot] = weight_sum
        else:
            weights = np.full(4, 0.25, np.float64)
        centers[foot] = weights @ points[sl]
        center_velocities[foot] = weights @ velocities[sl]
    return centers, center_velocities, normal_impulse, remaining_time


def predicted_resultant_impulse(
    center_velocity: np.ndarray,
    center_effective_mass: np.ndarray,
    normal_impulse: np.ndarray,
    friction: float,
) -> np.ndarray:
    impulse = np.zeros((2, 3), np.float64)
    impulse[:, 2] = normal_impulse
    for foot in range(2):
        tangent_cap = friction * normal_impulse[foot]
        for axis in (0, 1):
            passive = -center_effective_mass[foot, axis] * center_velocity[foot, axis]
            impulse[foot, axis] = float(
                np.clip(passive, -tangent_cap, tangent_cap)
            )
    return impulse


def actual_normal_centers(points: np.ndarray, impulse: np.ndarray) -> np.ndarray:
    centers = np.empty((2, 3), np.float64)
    for foot in range(2):
        sl = slice(4 * foot, 4 * foot + 4)
        weights = np.maximum(impulse[sl, 2], 0.0)
        total = float(np.sum(weights))
        centers[foot] = (
            weights @ points[sl] / total
            if total > 1.0e-15
            else np.mean(points[sl], axis=0)
        )
    return centers


def fit_affine_envelope(
    severity: np.ndarray,
    required_fraction: np.ndarray,
) -> dict[str, Any]:
    """Fit `fraction=a+b*severity` as a deterministic conservative envelope."""

    severity = np.asarray(severity, np.float64)
    required = np.asarray(required_fraction, np.float64)
    if (
        severity.ndim != 1
        or required.shape != severity.shape
        or not np.all(np.isfinite(severity))
        or not np.all(np.isfinite(required))
        or np.any(severity < 0.0)
        or np.any(required < 0.0)
    ):
        raise ValueError("envelope inputs must be finite nonnegative vectors")
    best: tuple[tuple[float, float, float, float], float, float, np.ndarray] | None = None
    for slope in np.linspace(0.0, ENVELOPE_SLOPE_MAX, ENVELOPE_SLOPE_STEPS):
        intercept = max(0.0, float(np.max(required - slope * severity)))
        fraction = intercept + slope * severity
        key = (
            float(np.percentile(fraction, 95.0)),
            float(np.max(fraction)),
            intercept,
            float(slope),
        )
        if best is None or key < best[0]:
            best = (key, intercept, float(slope), fraction)
    assert best is not None
    _, intercept, slope, fraction = best
    if np.any(fraction + 1.0e-12 < required):
        raise RuntimeError("fitted envelope is not conservative")
    return {
        "intercept": intercept,
        "slope": slope,
        "fraction": fraction,
        "required_fraction": required,
    }


def grouped_summary(values: np.ndarray) -> dict[str, dict[str, float]]:
    groups = np.asarray([group_maximum(row) for row in values], np.float64)
    return {
        "root_angular_rad_s": distribution(groups[:, 0]),
        "root_linear_m_s": distribution(groups[:, 1]),
        "joint_rad_s": distribution(groups[:, 2]),
    }


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    replay_path = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not replay_path.is_file():
        raise SystemExit("R216 requires the pinned G1 model and immutable R215 replay")
    replay = np.load(replay_path)
    point_session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    center_session = bonesaw.ContactTransitionModelSession(str(model), list(FOOT_FRAMES))
    joint_names = list(point_session.joint_names())
    joint_dof = int(point_session.joint_dof())
    generalized_dof = int(point_session.generalized_dof())
    q_nominal = standing_posture(joint_names)
    total_mass = total_urdf_mass(model)
    weight_tick_impulse = total_mass * GRAVITY * CONTROL_DT
    unit_twice_energy = total_mass * (GRAVITY * CONTROL_DT) ** 2

    point_bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 8, axis=0)
    center_bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    point_response = np.empty((generalized_dof, 8, 3), np.float64)
    point_mass = np.empty((8, 3), np.float64)
    point_delassus = np.empty((24, 24), np.float64)
    center_response = np.empty((generalized_dof, 2, 3), np.float64)
    center_mass = np.empty((2, 3), np.float64)
    center_delassus = np.empty((6, 6), np.float64)
    lower = np.empty(generalized_dof, np.float64)
    upper = np.empty(generalized_dof, np.float64)

    law_rows: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    all_residual: list[np.ndarray] = []
    all_exact_residual: list[np.ndarray] = []
    all_root_unit: list[np.ndarray] = []
    all_joint_unit: list[np.ndarray] = []
    all_severity: list[float] = []
    all_law_index: list[int] = []
    all_point_timing: list[int] = []
    all_center_timing: list[int] = []
    all_split_timing: list[int] = []
    zero_allocation = True

    for law_index, (law, sample_offset) in enumerate(
        zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    ):
        prefix = law.name
        points_series = replay[f"{prefix}_contact_points"]
        velocity_series = replay[f"{prefix}_prospective_velocity"]
        impulse_series = replay[f"{prefix}_contact_impulse"]
        raw_error_series = replay[f"{prefix}_raw_velocity_error"]
        root_height_series = replay[f"{prefix}_root_height"]
        samples = len(root_height_series)
        predicted_centers = np.empty((samples, 2, 3), np.float64)
        predicted_impulses = np.empty((samples, 2, 3), np.float64)
        actual_centers = np.empty((samples, 2, 3), np.float64)
        center_error = np.empty((samples, 2), np.float64)
        residual = np.empty((samples, generalized_dof), np.float64)
        root_unit = np.empty_like(residual)
        joint_unit = np.empty_like(residual)
        severity = np.empty(samples, np.float64)
        false_positive = np.zeros(samples, np.uint8)
        false_negative = np.zeros(samples, np.uint8)

        for sample in range(samples):
            state_index = sample_offset + sample
            root_position, quaternion, q = reconstruct_prestate(
                joint_dof, q_nominal, state_index, float(root_height_series[sample])
            )
            points = points_series[sample]
            velocities = velocity_series[sample]
            actual_impulse = impulse_series[sample]
            timing = point_session.point_impulse_velocity_response(
                root_position,
                quaternion,
                q,
                points,
                point_bases,
                point_response,
                point_mass,
                point_delassus,
            )
            all_point_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            centers, center_velocity, normal_impulse, _ = causal_center_prediction(
                points,
                velocities,
                point_mass,
                law.friction,
                law.solref_time_s,
            )
            timing = center_session.point_impulse_velocity_response(
                root_position,
                quaternion,
                q,
                centers,
                center_bases,
                center_response,
                center_mass,
                center_delassus,
            )
            all_center_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            predicted_impulse = predicted_resultant_impulse(
                center_velocity, center_mass, normal_impulse, law.friction
            )
            actual_delta = np.einsum(
                "dca,ca->d", point_response, actual_impulse, optimize=True
            )
            predicted_delta = np.einsum(
                "dca,ca->d", center_response, predicted_impulse, optimize=True
            )
            residual[sample] = (
                raw_error_series[sample] + actual_delta - predicted_delta
            )
            timing = center_session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                root_position,
                quaternion,
                q,
                unit_twice_energy,
                0.0,
                lower,
                upper,
            )
            all_split_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            root_unit[sample] = upper
            timing = center_session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                root_position,
                quaternion,
                q,
                0.0,
                unit_twice_energy,
                lower,
                upper,
            )
            all_split_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            joint_unit[sample] = upper
            predicted_centers[sample] = centers
            predicted_impulses[sample] = predicted_impulse
            actual_centers[sample] = actual_normal_centers(points, actual_impulse)
            center_error[sample] = np.linalg.norm(
                actual_centers[sample] - centers, axis=1
            )
            predicted_active = np.linalg.norm(predicted_impulse) > 1.0e-12
            actual_active = np.linalg.norm(actual_impulse) > 1.0e-12
            false_positive[sample] = predicted_active and not actual_active
            false_negative[sample] = actual_active and not predicted_active
            severity[sample] = np.linalg.norm(predicted_impulse) / weight_tick_impulse

        arrays.update(
            {
                f"{prefix}_predicted_centers": predicted_centers,
                f"{prefix}_predicted_impulses": predicted_impulses,
                f"{prefix}_actual_normal_centers": actual_centers,
                f"{prefix}_center_error": center_error,
                f"{prefix}_causal_residual": residual,
                f"{prefix}_root_unit_support": root_unit,
                f"{prefix}_joint_unit_support": joint_unit,
                f"{prefix}_severity": severity,
                f"{prefix}_false_positive": false_positive,
                f"{prefix}_false_negative": false_negative,
            }
        )
        all_residual.extend(residual)
        all_exact_residual.extend(raw_error_series)
        all_root_unit.extend(root_unit)
        all_joint_unit.extend(joint_unit)
        all_severity.extend(severity)
        all_law_index.extend([law_index] * samples)
        law_rows.append(
            {
                "law": prefix,
                "samples": samples,
                "center_error_m": distribution(center_error.reshape(-1)),
                "causal_residual": grouped_summary(residual),
                "exact_completed_point_oracle_residual": grouped_summary(raw_error_series),
                "false_positive_samples": int(np.count_nonzero(false_positive)),
                "false_negative_samples": int(np.count_nonzero(false_negative)),
                "predicted_impulse_norm_ns": distribution(
                    np.linalg.norm(predicted_impulses, axis=(1, 2))
                ),
                "actual_impulse_norm_ns": distribution(
                    np.linalg.norm(impulse_series, axis=(1, 2))
                ),
            }
        )

    residual = np.asarray(all_residual)
    exact_residual = np.asarray(all_exact_residual)
    root_unit = np.asarray(all_root_unit)
    joint_unit = np.asarray(all_joint_unit)
    severity = np.asarray(all_severity)
    law_index = np.asarray(all_law_index)
    root_required = np.max(
        np.abs(residual[:, :6]) / np.maximum(root_unit[:, :6], 1.0e-15), axis=1
    )
    joint_required = np.max(
        np.abs(residual[:, 6:]) / np.maximum(joint_unit[:, 6:], 1.0e-15), axis=1
    )
    root_profile = fit_affine_envelope(severity, root_required)
    joint_profile = fit_affine_envelope(severity, joint_required)
    root_fraction = root_profile.pop("fraction")
    joint_fraction = joint_profile.pop("fraction")
    root_profile.pop("required_fraction")
    joint_profile.pop("required_fraction")
    half_width = (
        root_fraction[:, None] * root_unit
        + joint_fraction[:, None] * joint_unit
    )
    covered_components = np.abs(residual) <= half_width + 1.0e-12
    covered = np.all(covered_components, axis=1)
    interval_width = 2.0 * half_width
    width_summary = grouped_summary(interval_width)
    lower_bound_width = grouped_summary(2.0 * np.abs(residual))
    width_gate_passed = all(
        width_summary[name]["p95"] <= limit for name, limit in WIDTH_GATES.items()
    )
    strict_coverage = bool(np.all(covered))
    mechanism_passed = bool(
        zero_allocation
        and np.all(np.isfinite(residual))
        and np.all(root_unit >= 0.0)
        and np.all(joint_unit >= 0.0)
    )
    profile_promoted = bool(mechanism_passed and strict_coverage and width_gate_passed)

    for index, row in enumerate(law_rows):
        mask = law_index == index
        row["envelope_sample_coverage"] = float(np.mean(covered[mask]))
        row["envelope_component_coverage"] = float(
            np.mean(covered_components[mask])
        )
        row["envelope_interval_width"] = grouped_summary(interval_width[mask])
        row["causal_center_lower_bound_width"] = grouped_summary(
            2.0 * np.abs(residual[mask])
        )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(replay_path),
        "source_replay_sha256": sha256(replay_path),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(residual),
        "physics_steps": 0,
        "policy_or_controller_steps": 0,
        "source_label_physics_steps": len(residual) * 5,
        "completed_contact_used_for_scoring_only": True,
        "total_mass_kg": total_mass,
        "unit_twice_energy_j": unit_twice_energy,
        "severity": "norm(predicted resultant impulse) / (m g dt)",
        "center_predictor": {
            "prospective_penetration_horizon_s": CONTROL_DT,
            "restitution_upper": RESTITUTION_UPPER,
            "relaxation": "1-exp(-remaining_time/declared_solref_time)",
            "tangent": "passive effective-mass cancellation clipped by declared friction",
            "post_step_inputs": [],
        },
        "split_profile": {
            "root_fraction": root_profile,
            "articulated_fraction": joint_profile,
            "twice_energy": "fraction(severity)^2 * m * (g dt)^2",
            "fit_provenance": "all R215 labels; construction only; not untouched holdout",
        },
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(covered_components)),
        "strict_coverage_passed": strict_coverage,
        "interval_width": width_summary,
        "causal_center_lower_bound_width": lower_bound_width,
        "exact_completed_point_oracle_residual": grouped_summary(exact_residual),
        "width_gates": WIDTH_GATES,
        "width_gate_passed": width_gate_passed,
        "point_response_timing_ns": distribution(np.asarray(all_point_timing)),
        "center_response_timing_ns": distribution(np.asarray(all_center_timing)),
        "split_bound_timing_ns": distribution(np.asarray(all_split_timing)),
        "zero_rust_allocation": zero_allocation,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": profile_promoted,
        "authority_admitted": False,
        "laws": law_rows,
    }

    table_rows = []
    for row in law_rows:
        causal = row["causal_residual"]
        oracle = row["exact_completed_point_oracle_residual"]
        width = row["envelope_interval_width"]
        lower_bound = row["causal_center_lower_bound_width"]
        table_rows.append(
            [
                row["law"],
                row["samples"],
                f"{row['center_error_m']['p95'] * 1_000.0:.1f}",
                f"{causal['root_angular_rad_s']['p95']:.3f} / {causal['root_linear_m_s']['p95']:.3f} / {causal['joint_rad_s']['p95']:.3f}",
                f"{oracle['joint_rad_s']['p95']:.3f}",
                f"{100.0 * row['envelope_sample_coverage']:.1f}%",
                f"{width['root_angular_rad_s']['p95']:.3f} / {width['root_linear_m_s']['p95']:.3f} / {width['joint_rad_s']['p95']:.3f}",
                f"{lower_bound['joint_rad_s']['p95']:.3f}",
                f"{row['false_positive_samples']} / {row['false_negative_samples']}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 causal-center + split-kinetic replay · r216",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · fitted construction **{'PASS' if profile_promoted else 'REJECTED'}** · authority **NOT ADMITTED** · replay physics/policy/controller steps **0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- One causal resultant point per foot is predicted only from pre-step four-corner gap, velocity, model effective mass, declared friction, and declared contact relaxation time. Prospective remaining time weights the center; no completed state, impulse, contact count, or case identity enters the prediction.",
            "- Rust evaluates both point responses and exact support of separately budgeted root/articulated kinetic impulse ellipsoids. Python fits only an affine severity envelope over the immutable R215 labels. This is construction evidence, not an untouched holdout or command authority.",
            "- The completed R215 point impulse is used only to reconstruct the scoring target. The exact completed-point residual is retained as an optimistic oracle control. Any centered component interval covering the causal predictor has width at least twice its absolute residual, independently of residual-set geometry.",
            "",
            "## Replay result",
            "",
            *markdown_table(
                [
                    "law",
                    "samples",
                    "center p95 mm",
                    "causal residual p95 ω/v/joint",
                    "exact-point joint p95",
                    "fitted coverage",
                    "fitted width p95 ω/v/joint",
                    "unavoidable joint width p95",
                    "false + / −",
                ],
                table_rows,
            ),
            "",
            f"The all-label affine envelopes are root fraction `{root_profile['intercept']:.6f} + {root_profile['slope']:.6f}·severity` and articulated fraction `{joint_profile['intercept']:.6f} + {joint_profile['slope']:.6f}·severity`, squared into separate energy budgets. They cover {int(np.count_nonzero(covered))}/{len(covered)} construction samples. Overall p95 width is {width_summary['root_angular_rad_s']['p95']:.3f}/{width_summary['root_linear_m_s']['p95']:.3f}/{width_summary['joint_rad_s']['p95']:.3f} against {WIDTH_GATES['root_angular_rad_s']:.1f}/{WIDTH_GATES['root_linear_m_s']:.1f}/{WIDTH_GATES['joint_rad_s']:.1f} gates.",
            "",
            "## Decision",
            "",
            "The split kinetic primitive is retained, but this causal center/impulse predictor and its label-fitted state-conditioned profile are rejected. Even before choosing a residual geometry, twice the causal predictor's p95 joint residual exceeds the useful-width gate on the rigid law; the fitted conservative envelope is wider still. The missing mechanism is a coupled contact-law realization/CoP evolution model or a typed estimator uncertainty set—not a larger energy radius. A new law/state sequence is required after that construction is frozen.",
        ]
    ) + "\n"

    arrays.update(
        {
            "all_causal_residual": residual,
            "all_exact_completed_point_residual": exact_residual,
            "all_root_unit_support": root_unit,
            "all_joint_unit_support": joint_unit,
            "all_severity": severity,
            "all_root_fraction": root_fraction,
            "all_joint_fraction": joint_fraction,
            "all_interval_width": interval_width,
            "all_covered": covered.astype(np.uint8),
            "all_component_coverage": covered_components.astype(np.uint8),
            "all_law_index": law_index,
        }
    )
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-causal-center-split-kinetic-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_CAUSAL_CENTER_SPLIT_KINETIC_REPLAY.md").write_text(report)
    np.savez_compressed(output / "g1-causal-center-split-kinetic-replay.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_promoted": profile_promoted,
                "sample_coverage": metrics["sample_coverage"],
                "width_gate_passed": width_gate_passed,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
