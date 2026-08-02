#!/usr/bin/env python3
"""R217 zero-plant coupled contact-law and CoP-evolution construction replay."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import (
    SOURCE_REPLAY,
    SOURCE_REVISION,
    actual_normal_centers,
    fit_affine_envelope,
    grouped_summary,
    reconstruct_prestate,
    total_urdf_mass,
)
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    GRAVITY,
    WIDTH_GATES,
    sha256,
    standing_posture,
)
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


REVISION = "g1-coupled-contact-law-replay-r217"
SWEEPS = 16
# Construction-time mapping selected after R216 localized the missing layer.
# It is intentionally labelled calibration, not a physical/hardware constant.
REGULARIZATION_TIME_SCALE = 6.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_COUPLED_CONTACT_LAW_REPLAY_R217.html"
    )
    return parser.parse_args()


def contact_law_parameters(relaxation_time_s: float) -> tuple[float, float]:
    if not math.isfinite(relaxation_time_s) or relaxation_time_s <= 0.0:
        raise ValueError("relaxation time must be finite and positive")
    restitution = math.exp(-relaxation_time_s / CONTROL_DT)
    regularization = relaxation_time_s / (
        REGULARIZATION_TIME_SCALE * CONTROL_DT
    )
    return restitution, regularization


def prospective_step_velocity(
    point_world: np.ndarray, velocity_world: np.ndarray
) -> np.ndarray:
    points = np.asarray(point_world, np.float64)
    velocity = np.asarray(velocity_world, np.float64)
    if points.shape != (8, 3) or velocity.shape != (8, 3):
        raise ValueError("prospective step expects eight point positions and velocities")
    adjusted = velocity.copy()
    # Moreau-style end-of-step nonpenetration: v_n + gap/dt < 0 is the
    # prospective contact violation. Tangent velocity is left physical.
    adjusted[:, 2] += np.maximum(points[:, 2], 0.0) / CONTROL_DT
    return adjusted


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R217 requires the pinned G1 model and immutable R215 replay")
    replay = np.load(source)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    joint_dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    q_nominal = standing_posture(list(session.joint_names()))
    total_mass = total_urdf_mass(model)
    weight_tick_impulse = total_mass * GRAVITY * CONTROL_DT
    unit_twice_energy = total_mass * (GRAVITY * CONTROL_DT) ** 2
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    lower = np.empty(generalized_dof, np.float64)
    upper = np.empty(generalized_dof, np.float64)
    friction = np.empty(8, np.float64)

    arrays: dict[str, np.ndarray] = {}
    rows: list[dict[str, Any]] = []
    all_residual: list[np.ndarray] = []
    all_root_unit: list[np.ndarray] = []
    all_joint_unit: list[np.ndarray] = []
    all_severity: list[float] = []
    all_law_index: list[int] = []
    response_timing: list[int] = []
    solve_timing: list[int] = []
    split_timing: list[int] = []
    zero_allocation = True

    for law_index, (law, offset) in enumerate(
        zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    ):
        prefix = law.name
        points_series = replay[f"{prefix}_contact_points"]
        velocity_series = replay[f"{prefix}_prospective_velocity"]
        actual_impulse_series = replay[f"{prefix}_contact_impulse"]
        raw_error_series = replay[f"{prefix}_raw_velocity_error"]
        upper_series = replay[f"{prefix}_grouped_acceleration_impulse_upper"]
        height_series = replay[f"{prefix}_root_height"]
        samples = len(height_series)
        residual = np.empty((samples, generalized_dof), np.float64)
        predicted = np.empty((samples, 8, 3), np.float64)
        predicted_center = np.empty((samples, 2, 3), np.float64)
        actual_center = np.empty((samples, 2, 3), np.float64)
        center_error = np.empty((samples, 2), np.float64)
        root_unit = np.empty_like(residual)
        joint_unit = np.empty_like(residual)
        severity = np.empty(samples, np.float64)
        false_positive = np.zeros(samples, np.uint8)
        false_negative = np.zeros(samples, np.uint8)
        restitution, regularization = contact_law_parameters(law.solref_time_s)
        friction.fill(law.friction)

        for sample in range(samples):
            state_index = offset + sample
            root_position, quaternion, q = reconstruct_prestate(
                joint_dof, q_nominal, state_index, float(height_series[sample])
            )
            points = points_series[sample]
            actual_impulse = actual_impulse_series[sample]
            timing = session.point_impulse_velocity_response(
                root_position,
                quaternion,
                q,
                points,
                bases,
                response,
                effective_mass,
                delassus,
            )
            response_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            step_velocity = prospective_step_velocity(
                points, velocity_series[sample]
            )
            timing = session.solve_coupled_contact_impulse(
                step_velocity,
                delassus,
                upper_series[sample],
                friction,
                restitution,
                regularization,
                SWEEPS,
                predicted_impulse,
                velocity_after,
            )
            solve_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            actual_delta = np.einsum(
                "dca,ca->d", response, actual_impulse, optimize=True
            )
            predicted_delta = np.einsum(
                "dca,ca->d", response, predicted_impulse, optimize=True
            )
            residual[sample] = (
                raw_error_series[sample] + actual_delta - predicted_delta
            )
            timing = session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                root_position,
                quaternion,
                q,
                unit_twice_energy,
                0.0,
                lower,
                upper,
            )
            split_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            root_unit[sample] = upper
            timing = session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                root_position,
                quaternion,
                q,
                0.0,
                unit_twice_energy,
                lower,
                upper,
            )
            split_timing.append(int(timing[0]))
            zero_allocation &= timing[1:] == (0, 0)
            joint_unit[sample] = upper
            predicted[sample] = predicted_impulse
            predicted_center[sample] = actual_normal_centers(
                points, predicted_impulse
            )
            actual_center[sample] = actual_normal_centers(points, actual_impulse)
            center_error[sample] = np.linalg.norm(
                predicted_center[sample] - actual_center[sample], axis=1
            )
            predicted_active = np.linalg.norm(predicted_impulse) > 1.0e-12
            actual_active = np.linalg.norm(actual_impulse) > 1.0e-12
            false_positive[sample] = predicted_active and not actual_active
            false_negative[sample] = actual_active and not predicted_active
            severity[sample] = np.linalg.norm(predicted_impulse) / weight_tick_impulse

        arrays.update(
            {
                f"{prefix}_predicted_impulse": predicted,
                f"{prefix}_predicted_normal_center": predicted_center,
                f"{prefix}_actual_normal_center": actual_center,
                f"{prefix}_center_error": center_error,
                f"{prefix}_residual": residual,
                f"{prefix}_root_unit_support": root_unit,
                f"{prefix}_joint_unit_support": joint_unit,
                f"{prefix}_severity": severity,
                f"{prefix}_false_positive": false_positive,
                f"{prefix}_false_negative": false_negative,
            }
        )
        all_residual.extend(residual)
        all_root_unit.extend(root_unit)
        all_joint_unit.extend(joint_unit)
        all_severity.extend(severity)
        all_law_index.extend([law_index] * samples)
        rows.append(
            {
                "law": prefix,
                "samples": samples,
                "restitution": restitution,
                "regularization_ratio": regularization,
                "residual": grouped_summary(residual),
                "lower_bound_width": grouped_summary(2.0 * np.abs(residual)),
                "center_error_m": distribution(center_error.reshape(-1)),
                "predicted_impulse_norm_ns": distribution(
                    np.linalg.norm(predicted, axis=(1, 2))
                ),
                "actual_impulse_norm_ns": distribution(
                    np.linalg.norm(actual_impulse_series, axis=(1, 2))
                ),
                "false_positive_samples": int(np.count_nonzero(false_positive)),
                "false_negative_samples": int(np.count_nonzero(false_negative)),
            }
        )

    residual = np.asarray(all_residual)
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
    half_width = root_fraction[:, None] * root_unit + joint_fraction[:, None] * joint_unit
    component_coverage = np.abs(residual) <= half_width + 1.0e-12
    covered = np.all(component_coverage, axis=1)
    interval_width = 2.0 * half_width
    width_summary = grouped_summary(interval_width)
    lower_bound = grouped_summary(2.0 * np.abs(residual))
    width_gate_passed = all(
        width_summary[name]["p95"] <= limit for name, limit in WIDTH_GATES.items()
    )
    mechanism_passed = bool(
        zero_allocation
        and np.all(np.isfinite(residual))
        and np.all(root_unit >= 0.0)
        and np.all(joint_unit >= 0.0)
    )
    profile_promoted = bool(
        mechanism_passed and np.all(covered) and width_gate_passed
    )
    for index, row in enumerate(rows):
        mask = law_index == index
        row["sample_coverage"] = float(np.mean(covered[mask]))
        row["component_coverage"] = float(np.mean(component_coverage[mask]))
        row["interval_width"] = grouped_summary(interval_width[mask])

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(residual),
        "physics_steps": 0,
        "policy_or_controller_steps": 0,
        "source_label_physics_steps": len(residual) * 5,
        "completed_contact_used_for_scoring_only": True,
        "predictor": {
            "normal_velocity": "v_n + max(gap,0)/dt",
            "restitution": "exp(-declared_relaxation_time/dt)",
            "regularization": "declared_relaxation_time/(6*dt)",
            "sweeps": SWEEPS,
            "time_scale_fit_provenance": "selected with R215/R216 construction labels",
            "post_step_inputs": [],
        },
        "split_profile": {
            "root_fraction": root_profile,
            "articulated_fraction": joint_profile,
            "twice_energy": "fraction(severity)^2 * m * (g dt)^2",
            "fit_provenance": "all R215 labels; construction only",
        },
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(component_coverage)),
        "strict_coverage_passed": bool(np.all(covered)),
        "interval_width": width_summary,
        "predictor_lower_bound_width": lower_bound,
        "width_gates": WIDTH_GATES,
        "width_gate_passed": width_gate_passed,
        "response_timing_ns": distribution(np.asarray(response_timing)),
        "coupled_solve_timing_ns": distribution(np.asarray(solve_timing)),
        "split_bound_timing_ns": distribution(np.asarray(split_timing)),
        "zero_rust_allocation": zero_allocation,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": profile_promoted,
        "authority_admitted": False,
        "laws": rows,
    }

    table = []
    for row in rows:
        residual_summary = row["residual"]
        width = row["interval_width"]
        table.append(
            [
                row["law"],
                f"{row['restitution']:.3f} / {row['regularization_ratio']:.3f}",
                f"{row['center_error_m']['p95'] * 1_000.0:.1f}",
                f"{residual_summary['root_angular_rad_s']['p95']:.3f} / {residual_summary['root_linear_m_s']['p95']:.3f} / {residual_summary['joint_rad_s']['p95']:.3f}",
                f"{row['lower_bound_width']['joint_rad_s']['p95']:.3f}",
                f"{100.0 * row['sample_coverage']:.1f}%",
                f"{width['root_angular_rad_s']['p95']:.3f} / {width['root_linear_m_s']['p95']:.3f} / {width['joint_rad_s']['p95']:.3f}",
                f"{row['false_positive_samples']} / {row['false_negative_samples']}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 coupled contact-law replay · r217",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · construction profile **{'PASS' if profile_promoted else 'REJECTED'}** · authority **NOT ADMITTED** · replay physics/policy/controller steps **0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- Rust solves all eight prospective G1 foot points together through the full Delassus operator. The normal input is `v_n + max(gap,0)/dt`, so only end-of-step penetration pressure enters; tangential velocity stays physical. Projection keeps normal impulse nonnegative and enforces a circular Coulomb section.",
            "- Declared contact relaxation maps continuously to restitution `exp(-τ/dt)` and diagonal regularization `τ/(6dt)`. The divisor 6 was selected after inspecting R215/R216 construction labels; it is calibration, not hardware physics and not an untouched holdout.",
            "- Completed R215 impulses remain scoring-only. The all-label split kinetic envelope is again construction diagnosis. Any centered component interval covering the predictor has width at least twice its absolute residual.",
            "",
            "## Replay result",
            "",
            *markdown_table(
                [
                    "law",
                    "restitution / reg",
                    "CoP p95 mm",
                    "predictor residual p95 ω/v/joint",
                    "unavoidable joint width p95",
                    "fitted coverage",
                    "fitted width p95 ω/v/joint",
                    "false + / −",
                ],
                table,
            ),
            "",
            f"The fitted envelope covers {int(np.count_nonzero(covered))}/{len(covered)} construction samples at overall p95 width {width_summary['root_angular_rad_s']['p95']:.3f}/{width_summary['root_linear_m_s']['p95']:.3f}/{width_summary['joint_rad_s']['p95']:.3f}. The Rust coupled solve is {metrics['coupled_solve_timing_ns']['p99'] / 1_000.0:.3f} µs p99 with zero timed allocation.",
            "",
            "## Decision",
            "",
            "The coupled time-step/CoP mechanism is retained for a fresh-law holdout, but its R215-fitted predictor/profile is not authority. It materially improves compliant-law joint residual over R216, yet the rigid-law lower-bound width remains above the useful gate. Freeze the mapping before new laws; do not tune from that holdout. If the fresh rigid result remains wide, contact estimator uncertainty or a higher-order compliant law is required.",
        ]
    ) + "\n"

    arrays.update(
        {
            "all_residual": residual,
            "all_root_unit_support": root_unit,
            "all_joint_unit_support": joint_unit,
            "all_severity": severity,
            "all_root_fraction": root_fraction,
            "all_joint_fraction": joint_fraction,
            "all_interval_width": interval_width,
            "all_covered": covered.astype(np.uint8),
            "all_component_coverage": component_coverage.astype(np.uint8),
            "all_law_index": law_index,
        }
    )
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-coupled-contact-law-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_COUPLED_CONTACT_LAW_REPLAY.md").write_text(report)
    np.savez_compressed(output / "g1-coupled-contact-law-replay.npz", **arrays)
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
