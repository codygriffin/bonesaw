#!/usr/bin/env python3
"""R221 fresh-law holdout for the frozen R220 compliant predictor."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary, reconstruct_prestate
from g1_contact_law_momentum_holdout import (
    CONTACT_LAWS as R213_LAWS,
    FOOT_FRAMES,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
    standing_posture,
)
from g1_coupled_contact_law_holdout import FRESH_CONTACT_LAWS as R218_LAWS
from g1_kinetic_impulse_ellipsoid_holdout import FRESH_CONTACT_LAWS as R214_LAWS
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS as R215_LAWS
from g1_substepped_compliant_contact_replay import CONTROL_DT, compliant_parameters


REVISION = "g1-substepped-compliant-contact-holdout-r221"
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "mid_elliptic_implicitfast",
        friction=0.65,
        solref_time_s=0.012,
        solimp_min=0.85,
        solimp_max=0.97,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "hard_pyramidal_rk4",
        friction=0.90,
        solref_time_s=0.0015,
        solimp_min=0.995,
        solimp_max=0.9998,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (70_000, 80_000)
FROZEN_PROFILE = {
    "substeps": 128,
    "stiffness_scale": 1.0,
    "damping_scale": 1.0,
    "impulse_cap_scale": 16.0,
    "residual_half_width": {
        "root_angular_rad_s": 0.8737699318093358,
        "root_linear_m_s": 0.14966914656159175,
        "joint_rad_s": 4.900868581605604,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_SUBSTEPPED_COMPLIANT_CONTACT_HOLDOUT_R221.html"
    )
    return parser.parse_args()


def residual_half_width(generalized_dof: int) -> np.ndarray:
    width = np.empty(generalized_dof, np.float64)
    width[:3] = FROZEN_PROFILE["residual_half_width"]["root_angular_rad_s"]
    width[3:6] = FROZEN_PROFILE["residual_half_width"]["root_linear_m_s"]
    width[6:] = FROZEN_PROFILE["residual_half_width"]["joint_rad_s"]
    return width


def score_law(
    model: pathlib.Path,
    law: ContactLaw,
    offset: int,
    arrays: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    import bonesaw

    prefix = law.name
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    generalized_dof = int(session.generalized_dof())
    joint_dof = int(session.joint_dof())
    q_nominal = standing_posture(list(session.joint_names()))
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    gap_after = np.empty(8, np.float64)
    half_width = residual_half_width(generalized_dof)
    samples = len(arrays[f"{prefix}_root_height"])
    residual = np.empty((samples, generalized_dof), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    component_covered = np.empty((samples, generalized_dof), np.uint8)
    sample_covered = np.empty(samples, np.uint8)
    impulse_error_norm = np.empty(samples, np.float64)
    timing = np.empty((samples, 2), np.uint64)
    zero_allocation = True

    for sample, height in enumerate(arrays[f"{prefix}_root_height"]):
        root, quaternion, q = reconstruct_prestate(
            joint_dof, q_nominal, offset + sample, float(height)
        )
        points = arrays[f"{prefix}_contact_points"][sample]
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
            FROZEN_PROFILE["stiffness_scale"],
            FROZEN_PROFILE["damping_scale"],
        )
        arguments = (
            np.ascontiguousarray(points[:, 2]),
            arrays[f"{prefix}_prospective_velocity"][sample],
            delassus,
            arrays[f"{prefix}_grouped_acceleration_impulse_upper"][sample]
            * FROZEN_PROFILE["impulse_cap_scale"],
            np.full(8, law.friction, np.float64),
            stiffness,
            damping,
            CONTROL_DT,
            FROZEN_PROFILE["substeps"],
            predicted_impulse,
            velocity_after,
            gap_after,
        )
        result = session.solve_substepped_compliant_contact_impulse(*arguments)
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first_impulse = predicted_impulse.copy()
        first_velocity = velocity_after.copy()
        first_gap = gap_after.copy()
        result = session.solve_substepped_compliant_contact_impulse(*arguments)
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if (
            not np.array_equal(predicted_impulse, first_impulse)
            or not np.array_equal(velocity_after, first_velocity)
            or not np.array_equal(gap_after, first_gap)
        ):
            raise RuntimeError("compliant holdout query is not bitwise deterministic")
        actual_impulse = arrays[f"{prefix}_contact_impulse"][sample]
        residual[sample] = arrays[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
            "dca,ca->d",
            response,
            actual_impulse - predicted_impulse,
            optimize=True,
        )
        predicted[sample] = predicted_impulse
        impulse_error_norm[sample] = np.linalg.norm(actual_impulse - predicted_impulse)
        component_covered[sample] = np.abs(residual[sample]) <= half_width + 1.0e-12
        sample_covered[sample] = np.all(component_covered[sample])

    widths = {
        name: 2.0 * value
        for name, value in FROZEN_PROFILE["residual_half_width"].items()
    }
    width_gate = all(widths[name] <= limit for name, limit in WIDTH_GATES.items())
    strict = bool(np.all(sample_covered))
    result = {
        "law": prefix,
        "samples": samples,
        "sample_coverage": float(np.mean(sample_covered)),
        "component_coverage": float(np.mean(component_covered)),
        "uncovered_samples": int(np.count_nonzero(sample_covered == 0)),
        "strict_coverage_passed": strict,
        "predictor_residual": grouped_summary(residual),
        "frozen_interval_width": widths,
        "width_gate_passed": width_gate,
        "impulse_error_norm_ns": distribution(impulse_error_norm),
        "query_timing_ns": distribution(timing.reshape(-1)),
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "profile_promoted": bool(strict and width_gate and zero_allocation),
    }
    scored = {
        f"{prefix}_compliant_predicted_impulse": predicted,
        f"{prefix}_compliant_residual": residual,
        f"{prefix}_compliant_component_covered": component_covered,
        f"{prefix}_compliant_sample_covered": sample_covered,
        f"{prefix}_compliant_timing_ns": timing,
    }
    return result, scored


def main() -> int:
    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit(f"missing model: {model}")
    prior_names = {
        law.name for law in (*R213_LAWS, *R214_LAWS, *R215_LAWS, *R218_LAWS)
    }
    if {law.name for law in FRESH_CONTACT_LAWS} & prior_names:
        raise SystemExit("R221 contact-law names must be fresh")
    results: list[dict[str, Any]] = []
    replay: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        _, source_arrays = run_law(
            model,
            law,
            args.samples_per_law,
            sample_offset=offset,
            reserve_kind="kinetic_ellipsoid",
            reserve_fraction=0.50,
        )
        scored, scored_arrays = score_law(model, law, offset, source_arrays)
        results.append(scored)
        replay.update(source_arrays)
        replay.update(scored_arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"] and result["bitwise_repeat"]
        for result in results
    )
    promoted = all(result["profile_promoted"] for result in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(results) * args.samples_per_law,
        "physics_steps": len(results) * args.samples_per_law * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "sample_reset_every_transition": True,
        "fresh_laws_and_state_offsets": True,
        "frozen_profile": FROZEN_PROFILE,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        residual = result["predictor_residual"]
        width = result["frozen_interval_width"]
        rows.append(
            [
                result["law"],
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{residual['root_angular_rad_s']['p95']:.3f} / {residual['root_linear_m_s']['p95']:.3f} / {residual['joint_rad_s']['p95']:.3f}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['impulse_error_norm_ns']['mean']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "PROMOTE" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 substepped compliant-contact fresh holdout · r221",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- R220's 128 microsteps, effective-mass/relaxation/impedance mapping, 16× impulse-cap scale, and 1.748/0.299/9.802 group widths were frozen before these labels. The new mid/elliptic/implicit-fast and hard/pyramidal/RK4 laws begin at untouched offsets 70,000 and 80,000; every sample resets.",
            "- MuJoCo supplies only the completed five-substep scoring label. No controller, policy, rollout selection, prior state, or R221 value enters the Rust predictor. Each query repeats bitwise and writes caller-owned buffers with zero timed allocation.",
            "- Strict sample coverage, 2.0/0.5/10.0 widths, repeat, and allocation gates are conjunctive. Holdout misses are never tuned back into the frozen profile; passing promotes only this evaluation profile, never hardware or command authority.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                [
                    "law",
                    "sample coverage",
                    "component coverage",
                    "predictor residual p95 ω/v/joint",
                    "frozen width ω/v/joint",
                    "impulse error mean N·s",
                    "query p99 µs",
                    "profile",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            (
                "Both untouched laws preserve strict coverage, useful width, repeat, and allocation gates. Promote the frozen substepped compliant profile only to the next terminal-consequence experiment; estimator provenance, external load, plant non-regression, deadlines, and hardware authority remain separate."
                if promoted
                else "At least one untouched law violates strict coverage despite the useful frozen width. Retain the substepped compliant mechanism, reject the frozen profile, and do not tune the holdout miss back into it before another independently motivated construction."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-substepped-compliant-contact-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_SUBSTEPPED_COMPLIANT_CONTACT_HOLDOUT.md").write_text(report)
    np.savez_compressed(output / "g1-substepped-compliant-contact-holdout.npz", **replay)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_promoted": promoted,
                "sample_coverage": {
                    result["law"]: result["sample_coverage"] for result in results
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
