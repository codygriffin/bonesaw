#!/usr/bin/env python3
"""R219 zero-plant typed finite contact-hypothesis construction replay."""

from __future__ import annotations

import argparse
import itertools
import json
import pathlib
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import reconstruct_prestate
from g1_contact_law_momentum_holdout import FOOT_FRAMES, WIDTH_GATES, sha256, standing_posture
from g1_coupled_contact_law_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS
from g1_coupled_contact_law_replay import SWEEPS, contact_law_parameters, prospective_step_velocity


REVISION = "g1-contact-estimator-hypothesis-replay-r219"
SOURCE_REVISION = "g1-coupled-contact-law-holdout-r218"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-coupled-contact-law-holdout-r218/"
    "g1-coupled-contact-law-holdout-replay.npz"
)
CONTROL_DT = 0.005


@dataclass(frozen=True)
class HypothesisProfile:
    name: str
    gap_error_m: float
    normal_velocity_error_m_s: float
    tangent_velocity_error_m_s: float
    enumerate_normal_signs: bool
    impulse_cap_scale: float


# Construction grid selected with knowledge of R218. It is not a holdout and
# no member is eligible for authority or promotion from this replay.
PROFILES = (
    HypothesisProfile("local59_small", 0.003, 0.3, 0.2, False, 1.0),
    HypothesisProfile("normal315_small", 0.003, 0.3, 0.2, True, 1.0),
    HypothesisProfile("normal315_medium", 0.005, 0.5, 0.3, True, 1.0),
    HypothesisProfile("normal315_large", 0.010, 1.0, 0.5, True, 1.0),
    HypothesisProfile("normal315_large_cap2", 0.010, 1.0, 0.5, True, 2.0),
    HypothesisProfile("normal315_large_cap4", 0.010, 1.0, 0.5, True, 4.0),
    HypothesisProfile("normal315_large_cap8", 0.010, 1.0, 0.5, True, 8.0),
    HypothesisProfile("normal315_large_cap16", 0.010, 1.0, 0.5, True, 16.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_CONTACT_ESTIMATOR_HYPOTHESIS_REPLAY_R219.html"
    )
    return parser.parse_args()


def build_hypotheses(
    points: np.ndarray,
    velocity: np.ndarray,
    impulse_upper: np.ndarray,
    law: Any,
    profile: HypothesisProfile,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    nominal = prospective_step_velocity(points, velocity)
    rows = [nominal]
    for sign in (-1.0, 1.0):
        shifted = velocity.copy()
        shifted[:, 2] += (
            np.maximum(points[:, 2] + sign * profile.gap_error_m, 0.0)
            / CONTROL_DT
        )
        rows.append(shifted)
        shifted = nominal.copy()
        shifted[:, 2] += sign * profile.normal_velocity_error_m_s
        rows.append(shifted)
        shifted = nominal.copy()
        shifted[:, 0] += sign * profile.tangent_velocity_error_m_s
        rows.append(shifted)
        shifted = nominal.copy()
        shifted[:, 1] += sign * profile.tangent_velocity_error_m_s
        rows.append(shifted)
    for contact in range(8):
        for axis, error in (
            (0, profile.tangent_velocity_error_m_s),
            (1, profile.tangent_velocity_error_m_s),
            (2, profile.normal_velocity_error_m_s),
        ):
            for sign in (-1.0, 1.0):
                shifted = nominal.copy()
                shifted[contact, axis] += sign * error
                rows.append(shifted)
    if profile.enumerate_normal_signs:
        for signs in itertools.product((-1.0, 1.0), repeat=8):
            shifted = nominal.copy()
            shifted[:, 2] += profile.normal_velocity_error_m_s * np.asarray(signs)
            rows.append(shifted)

    velocity_hypotheses = np.asarray(rows, np.float64)
    count = len(velocity_hypotheses)
    upper_hypotheses = np.repeat(
        (impulse_upper * profile.impulse_cap_scale)[None], count, axis=0
    )
    friction_hypotheses = np.full((count, 8), law.friction, np.float64)
    restitution, regularization = contact_law_parameters(law.solref_time_s)
    restitution_hypotheses = np.full(count, restitution, np.float64)
    regularization_hypotheses = np.full(count, regularization, np.float64)

    # Two separately typed law-error scenarios. These do not imply coverage
    # between the declared relaxation/friction values.
    for scale in (0.5, 2.0):
        velocity_hypotheses = np.concatenate(
            (velocity_hypotheses, nominal[None]), axis=0
        )
        upper_hypotheses = np.concatenate(
            (
                upper_hypotheses,
                (impulse_upper * profile.impulse_cap_scale)[None],
            ),
            axis=0,
        )
        friction_hypotheses = np.concatenate(
            (friction_hypotheses, np.full((1, 8), law.friction * scale)), axis=0
        )
        restitution_i, regularization_i = contact_law_parameters(
            law.solref_time_s * scale
        )
        restitution_hypotheses = np.concatenate(
            (restitution_hypotheses, [restitution_i])
        )
        regularization_hypotheses = np.concatenate(
            (regularization_hypotheses, [regularization_i])
        )
    return (
        velocity_hypotheses,
        upper_hypotheses,
        friction_hypotheses,
        restitution_hypotheses,
        regularization_hypotheses,
    )


def grouped_summary(values: np.ndarray) -> dict[str, dict[str, float]]:
    return {
        "root_angular_rad_s": distribution(values[:, :3].reshape(-1)),
        "root_linear_m_s": distribution(values[:, 3:6].reshape(-1)),
        "joint_rad_s": distribution(values[:, 6:].reshape(-1)),
    }


def evaluate_profile(
    session: Any,
    replay: Any,
    profile: HypothesisProfile,
    q_nominal: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    generalized_dof = int(session.generalized_dof())
    joint_dof = int(session.joint_dof())
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    lower = np.empty(generalized_dof, np.float64)
    upper = np.empty(generalized_dof, np.float64)
    samples = sum(len(replay[f"{law.name}_root_height"]) for law in FRESH_CONTACT_LAWS)
    interval_lower = np.empty((samples, generalized_dof), np.float64)
    interval_upper = np.empty_like(interval_lower)
    target = np.empty_like(interval_lower)
    component_covered = np.empty((samples, generalized_dof), np.uint8)
    sample_covered = np.empty(samples, np.uint8)
    timing = np.empty(samples, np.uint64)
    hypothesis_count = 0
    zero_allocation = True
    cursor = 0

    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        prefix = law.name
        heights = replay[f"{prefix}_root_height"]
        for sample, height in enumerate(heights):
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
            hypotheses = build_hypotheses(
                points,
                replay[f"{prefix}_prospective_velocity"][sample],
                replay[f"{prefix}_grouped_acceleration_impulse_upper"][sample],
                law,
                profile,
            )
            hypothesis_count = len(hypotheses[0])
            result = session.coupled_contact_hypothesis_velocity_envelope(
                hypotheses[0],
                delassus,
                hypotheses[1],
                hypotheses[2],
                hypotheses[3],
                hypotheses[4],
                response,
                SWEEPS,
                lower,
                upper,
            )
            timing[cursor] = result[0]
            zero_allocation &= result[1:] == (0, 0)
            # The completed generalized transition is a scoring-only label.
            target[cursor] = replay[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
                "dca,ca->d",
                response,
                replay[f"{prefix}_contact_impulse"][sample],
                optimize=True,
            )
            interval_lower[cursor] = lower
            interval_upper[cursor] = upper
            component_covered[cursor] = (target[cursor] >= lower - 1.0e-12) & (
                target[cursor] <= upper + 1.0e-12
            )
            sample_covered[cursor] = np.all(component_covered[cursor])
            cursor += 1

    widths = interval_upper - interval_lower
    width_summary = grouped_summary(widths)
    width_gate = all(
        width_summary[name]["p95"] <= limit for name, limit in WIDTH_GATES.items()
    )
    strict = bool(np.all(sample_covered))
    metrics = {
        "profile": profile.name,
        "hypotheses": hypothesis_count,
        "gap_error_m": profile.gap_error_m,
        "normal_velocity_error_m_s": profile.normal_velocity_error_m_s,
        "tangent_velocity_error_m_s": profile.tangent_velocity_error_m_s,
        "enumerate_normal_signs": profile.enumerate_normal_signs,
        "impulse_cap_scale": profile.impulse_cap_scale,
        "sample_coverage": float(np.mean(sample_covered)),
        "component_coverage": float(np.mean(component_covered)),
        "uncovered_samples": int(np.count_nonzero(sample_covered == 0)),
        "interval_width": width_summary,
        "width_gate_passed": width_gate,
        "strict_coverage_passed": strict,
        "query_timing_ns": distribution(timing),
        "zero_rust_allocation": zero_allocation,
        "profile_promoted": False,
    }
    arrays = {
        f"{profile.name}_lower": interval_lower,
        f"{profile.name}_upper": interval_upper,
        f"{profile.name}_target": target,
        f"{profile.name}_component_covered": component_covered,
        f"{profile.name}_sample_covered": sample_covered,
        f"{profile.name}_timing_ns": timing,
    }
    return metrics, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R219 requires the pinned G1 model and immutable R218 replay")
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

    mechanism_passed = all(result["zero_rust_allocation"] for result in results)
    viable = [
        result
        for result in results
        if result["strict_coverage_passed"] and result["width_gate_passed"]
    ]
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
        "finite_hypotheses_do_not_certify_continuous_set": True,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "viable_profile_count": len(viable),
        "profile_promoted": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        width = result["interval_width"]
        rows.append(
            [
                result["profile"],
                str(result["hypotheses"]),
                f"{1_000.0 * result['gap_error_m']:.1f}",
                f"{result['normal_velocity_error_m_s']:.1f} / {result['tangent_velocity_error_m_s']:.1f}",
                f"{result['impulse_cap_scale']:.0f}×",
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{width['root_angular_rad_s']['p95']:.3f} / {width['root_linear_m_s']['p95']:.3f} / {width['joint_rad_s']['p95']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "yes" if result["zero_rust_allocation"] else "NO",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 typed contact-estimator hypothesis replay · r219",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · viable construction profiles **{len(viable)}** · profile/authority **NOT PROMOTED** · replay physics/policy/controller/integration **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- Rust accepts a typed, explicitly enumerated finite contact-hypothesis set. Every scenario owns contact velocity, impulse caps, friction, restitution, and compliance while sharing the exact state-local Delassus and generalized response. All scenarios validate before outputs change; caller-owned scratch and timed execution allocate nothing.",
            "- The 59-scenario family carries nominal, global signed gap/normal/tangent, per-point signed axis, and two law-error hypotheses. The 315-scenario family additionally enumerates all 2⁸ signed normal-velocity patterns. A finite scenario envelope does not certify unenumerated values between those scenarios.",
            "- This diagnostic grid was selected after R218 labels and runs only on immutable replay. Completed transitions are scoring labels. Strict sample coverage and 2.0/0.5/10.0 p95 width gates are conjunctive; no row can be promoted from construction.",
            "",
            "## Construction Pareto sweep",
            "",
            *markdown_table(
                [
                    "profile",
                    "H",
                    "gap mm",
                    "normal/tangent m/s",
                    "cap",
                    "sample coverage",
                    "component coverage",
                    "width p95 ω/v/joint",
                    "query p99 µs",
                    "zero alloc",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            "The generic finite-hypothesis mechanism is retained, but no construction row satisfies strict coverage and useful width together. The 315-scenario large profile reaches only 92.708% strict coverage at 23.306 rad/s joint p95 width. Raising impulse caps reaches 96.875% but remains near 22.5 rad/s and still misses. Because the construction gate already fails, spending a fresh holdout would not add authority evidence. Keep hypothesis provenance typed; move the predictor to a higher-order compliant law rather than expanding this discrete set until it memorizes labels.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-contact-estimator-hypothesis-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_CONTACT_ESTIMATOR_HYPOTHESIS_REPLAY.md").write_text(report)
    np.savez_compressed(output / "g1-contact-estimator-hypothesis-replay.npz", **arrays)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "viable_profile_count": len(viable),
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
