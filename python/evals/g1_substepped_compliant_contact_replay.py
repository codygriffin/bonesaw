#!/usr/bin/env python3
"""R220 zero-plant substepped compliant-contact construction replay."""

from __future__ import annotations

import argparse
import json
import pathlib
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary, reconstruct_prestate
from g1_contact_law_momentum_holdout import FOOT_FRAMES, WIDTH_GATES, sha256, standing_posture
from g1_coupled_contact_law_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


REVISION = "g1-substepped-compliant-contact-replay-r220"
SOURCE_REVISION = "g1-coupled-contact-law-holdout-r218"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-coupled-contact-law-holdout-r218/"
    "g1-coupled-contact-law-holdout-replay.npz"
)
CONTROL_DT = 0.005


@dataclass(frozen=True)
class CompliantProfile:
    name: str
    substeps: int
    stiffness_scale: float
    damping_scale: float
    impulse_cap_scale: float


PROFILES = (
    CompliantProfile("substeps8", 8, 1.0, 1.0, 16.0),
    CompliantProfile("substeps16", 16, 1.0, 1.0, 16.0),
    CompliantProfile("substeps32", 32, 1.0, 1.0, 16.0),
    CompliantProfile("substeps64", 64, 1.0, 1.0, 16.0),
    CompliantProfile("substeps96", 96, 1.0, 1.0, 16.0),
    CompliantProfile("substeps128", 128, 1.0, 1.0, 16.0),
    CompliantProfile("substeps192", 192, 1.0, 1.0, 16.0),
    CompliantProfile("substeps256", 256, 1.0, 1.0, 16.0),
    CompliantProfile("soft_k05_c075", 128, 0.5, 0.75, 16.0),
    CompliantProfile("stiff_k2_c15", 128, 2.0, 1.5, 16.0),
    CompliantProfile("stiff_k4_c2", 128, 4.0, 2.0, 16.0),
    CompliantProfile("under_damped", 128, 1.0, 0.5, 16.0),
    CompliantProfile("over_damped", 128, 1.0, 2.0, 16.0),
)
FROZEN_PROFILE_NAME = "substeps128"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_SUBSTEPPED_COMPLIANT_CONTACT_REPLAY_R220.html"
    )
    return parser.parse_args()


def compliant_parameters(
    effective_normal_mass: np.ndarray,
    relaxation_time_s: float,
    impedance: float,
    stiffness_scale: float,
    damping_scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    mass = np.asarray(effective_normal_mass, np.float64)
    if (
        mass.ndim != 1
        or np.any(~np.isfinite(mass))
        or np.any(mass <= 0.0)
        or not np.isfinite(relaxation_time_s)
        or relaxation_time_s <= 0.0
        or not np.isfinite(impedance)
        or impedance <= 0.0
        or not np.isfinite(stiffness_scale)
        or stiffness_scale <= 0.0
        or not np.isfinite(damping_scale)
        or damping_scale <= 0.0
    ):
        raise ValueError("compliant parameter mapping requires positive finite inputs")
    stiffness = (
        stiffness_scale * impedance * mass / (relaxation_time_s * relaxation_time_s)
    )
    damping = (
        damping_scale
        * 2.0
        * np.sqrt(impedance)
        * mass
        / relaxation_time_s
    )
    return stiffness, damping


def group_maximum_absolute(residual: np.ndarray) -> dict[str, float]:
    return {
        "root_angular_rad_s": float(np.max(np.abs(residual[:, :3]))),
        "root_linear_m_s": float(np.max(np.abs(residual[:, 3:6]))),
        "joint_rad_s": float(np.max(np.abs(residual[:, 6:]))),
    }


def evaluate_profile(
    session: Any,
    replay: Any,
    profile: CompliantProfile,
    q_nominal: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    generalized_dof = int(session.generalized_dof())
    joint_dof = int(session.joint_dof())
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    gap_after = np.empty(8, np.float64)
    samples = sum(len(replay[f"{law.name}_root_height"]) for law in FRESH_CONTACT_LAWS)
    residual = np.empty((samples, generalized_dof), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    impulse_error_norm = np.empty(samples, np.float64)
    timing = np.empty(samples, np.uint64)
    zero_allocation = True
    cursor = 0

    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        prefix = law.name
        for sample, height in enumerate(replay[f"{prefix}_root_height"]):
            root, quaternion, q = reconstruct_prestate(
                joint_dof, q_nominal, offset + sample, float(height)
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
            stiffness, damping = compliant_parameters(
                effective_mass[:, 2],
                law.solref_time_s,
                law.solimp_max,
                profile.stiffness_scale,
                profile.damping_scale,
            )
            result = session.solve_substepped_compliant_contact_impulse(
                np.ascontiguousarray(points[:, 2]),
                replay[f"{prefix}_prospective_velocity"][sample],
                delassus,
                replay[f"{prefix}_grouped_acceleration_impulse_upper"][sample]
                * profile.impulse_cap_scale,
                np.full(8, law.friction, np.float64),
                stiffness,
                damping,
                CONTROL_DT,
                profile.substeps,
                predicted_impulse,
                velocity_after,
                gap_after,
            )
            timing[cursor] = result[0]
            zero_allocation &= result[1:] == (0, 0)
            actual_impulse = replay[f"{prefix}_contact_impulse"][sample]
            residual[cursor] = replay[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
                "dca,ca->d",
                response,
                actual_impulse - predicted_impulse,
                optimize=True,
            )
            predicted[cursor] = predicted_impulse
            impulse_error_norm[cursor] = np.linalg.norm(
                actual_impulse - predicted_impulse
            )
            cursor += 1

    residual_summary = grouped_summary(residual)
    half_width = group_maximum_absolute(residual)
    fitted_width = {name: 2.0 * value for name, value in half_width.items()}
    width_gate = all(fitted_width[name] <= limit for name, limit in WIDTH_GATES.items())
    metrics = {
        "profile": profile.name,
        "substeps": profile.substeps,
        "stiffness_scale": profile.stiffness_scale,
        "damping_scale": profile.damping_scale,
        "impulse_cap_scale": profile.impulse_cap_scale,
        "predictor_residual": residual_summary,
        "fitted_group_half_width": half_width,
        "fitted_group_width": fitted_width,
        "construction_sample_coverage": 1.0,
        "width_gate_passed": width_gate,
        "construction_gate_passed": bool(width_gate and zero_allocation),
        "impulse_error_norm_ns": distribution(impulse_error_norm),
        "query_timing_ns": distribution(timing),
        "zero_rust_allocation": zero_allocation,
        "profile_promoted": False,
    }
    arrays = {
        f"{profile.name}_residual": residual,
        f"{profile.name}_predicted_impulse": predicted,
        f"{profile.name}_impulse_error_norm": impulse_error_norm,
        f"{profile.name}_timing_ns": timing,
    }
    return metrics, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R220 requires the pinned G1 model and immutable R218 replay")
    replay = np.load(source)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    results: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    for profile in PROFILES:
        result, profile_arrays = evaluate_profile(session, replay, profile, q_nominal)
        results.append(result)
        arrays.update(profile_arrays)
    frozen = next(result for result in results if result["profile"] == FROZEN_PROFILE_NAME)
    mechanism_passed = all(result["zero_rust_allocation"] for result in results)
    construction_passed = bool(frozen["construction_gate_passed"])
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": 96,
        "replay_physics_steps": 0,
        "policy_or_controller_steps": 0,
        "integration_steps": 0,
        "construction_selected_after_source_labels": True,
        "width_gates": WIDTH_GATES,
        "frozen_profile_name": FROZEN_PROFILE_NAME,
        "frozen_profile": frozen,
        "mechanism_passed": mechanism_passed,
        "construction_passed": construction_passed,
        "profile_promoted": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        residual = result["predictor_residual"]
        width = result["fitted_group_width"]
        rows.append(
            [
                result["profile"],
                str(result["substeps"]),
                f"{result['stiffness_scale']:.2f} / {result['damping_scale']:.2f}",
                f"{residual['root_angular_rad_s']['p95']:.3f} / {residual['root_linear_m_s']['p95']:.3f} / {residual['joint_rad_s']['p95']:.3f}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['impulse_error_norm_ns']['mean']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "PASS" if result["construction_gate_passed"] else "reject",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 substepped compliant-contact replay · r220",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen construction **{'PASS' if construction_passed else 'FAIL'}** · profile/authority **NOT PROMOTED** · replay physics/policy/controller/integration **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- Rust carries signed point gap and contact velocity through fixed microsteps. Each microstep predicts penetration, applies explicit Kelvin–Voigt normal impulse, projects tangent impulse to the current circular Coulomb disk, updates all points through the full Delassus operator, and advances gap with post-impulse normal velocity. It is a deterministic model witness, not a continuous-time enclosure.",
            "- The model-owned mapping is `k = s_k z m_eff / τ²`, `c = s_c 2 sqrt(z) m_eff / τ`, using causal state-local effective mass and declared contact relaxation/impedance. A 16× impulse-cap scale and every parameter/sweep row were selected with R218 construction labels; none is hardware calibration or untouched evidence.",
            "- The fitted groupwise residual box uses every R218 label and is eligible only to be frozen for a new-law holdout. Strict construction coverage, 2.0/0.5/10.0 width, deterministic replay, and zero allocation are conjunctive. No construction result is authority.",
            "",
            "## Construction sweep",
            "",
            *markdown_table(
                [
                    "profile",
                    "microsteps",
                    "k/c scale",
                    "predictor residual p95 ω/v/joint",
                    "strict fitted width ω/v/joint",
                    "impulse error mean N·s",
                    "query p99 µs",
                    "gate",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            f"Freeze `{FROZEN_PROFILE_NAME}` unchanged for new contact laws and state offsets. Its all-label group box is {frozen['fitted_group_width']['root_angular_rad_s']:.3f}/{frozen['fitted_group_width']['root_linear_m_s']:.3f}/{frozen['fitted_group_width']['joint_rad_s']:.3f}, inside every construction gate, while the compliant query is {frozen['query_timing_ns']['p99'] / 1_000.0:.3f} µs p99 with zero Rust allocation. Promotion remains false until an untouched holdout preserves strict coverage, width, repeat, and deadline together.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-substepped-compliant-contact-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_SUBSTEPPED_COMPLIANT_CONTACT_REPLAY.md").write_text(report)
    np.savez_compressed(output / "g1-substepped-compliant-contact-replay.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "construction_passed": construction_passed,
                "frozen_profile": FROZEN_PROFILE_NAME,
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed and construction_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
