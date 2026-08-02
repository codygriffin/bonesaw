#!/usr/bin/env python3
"""R218 fresh-law holdout for the frozen R217 coupled contact construction."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import (
    actual_normal_centers,
    grouped_summary,
    reconstruct_prestate,
    total_urdf_mass,
)
from g1_contact_law_momentum_holdout import (
    CONTACT_LAWS as R213_LAWS,
    CONTROL_DT,
    FOOT_FRAMES,
    GRAVITY,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
    standing_posture,
)
from g1_coupled_contact_law_replay import (
    SWEEPS,
    contact_law_parameters,
    prospective_step_velocity,
)
from g1_kinetic_impulse_ellipsoid_holdout import FRESH_CONTACT_LAWS as R214_LAWS
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS as R215_LAWS


REVISION = "g1-coupled-contact-law-holdout-r218"
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "soft_pyramidal_euler",
        friction=0.25,
        solref_time_s=0.045,
        solimp_min=0.65,
        solimp_max=0.90,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_EULER),
    ),
    ContactLaw(
        "stiff_elliptic_rk4",
        friction=1.25,
        solref_time_s=0.0009,
        solimp_min=0.9985,
        solimp_max=0.99995,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (50_000, 60_000)

# Frozen from the all-label R217 construction before either R218 law ran.
FROZEN_SPLIT_PROFILE = {
    "root_intercept": 0.2072754931882615,
    "root_slope": 1.5050000000000001,
    "articulated_intercept": 0.6958508917557675,
    "articulated_slope": 2.91,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_COUPLED_CONTACT_LAW_HOLDOUT_R218.html"
    )
    return parser.parse_args()


def score_fresh_law(
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
    joint_dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    q_nominal = standing_posture(list(session.joint_names()))
    total_mass = total_urdf_mass(model)
    weight_tick_impulse = total_mass * GRAVITY * CONTROL_DT
    unit_twice_energy = total_mass * (GRAVITY * CONTROL_DT) ** 2
    points_series = arrays[f"{prefix}_contact_points"]
    velocity_series = arrays[f"{prefix}_prospective_velocity"]
    actual_impulse_series = arrays[f"{prefix}_contact_impulse"]
    raw_error_series = arrays[f"{prefix}_raw_velocity_error"]
    upper_series = arrays[f"{prefix}_grouped_acceleration_impulse_upper"]
    height_series = arrays[f"{prefix}_root_height"]
    samples = len(height_series)
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    friction = np.full(8, law.friction, np.float64)
    lower = np.empty(generalized_dof, np.float64)
    upper = np.empty(generalized_dof, np.float64)
    residual = np.empty((samples, generalized_dof), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    interval_width = np.empty_like(residual)
    component_coverage = np.zeros((samples, generalized_dof), np.uint8)
    covered = np.zeros(samples, np.uint8)
    center_error = np.empty((samples, 2), np.float64)
    severity = np.empty(samples, np.float64)
    solve_timing = np.empty(samples, np.uint64)
    split_timing = np.empty((samples, 2), np.uint64)
    response_timing = np.empty(samples, np.uint64)
    false_positive = np.zeros(samples, np.uint8)
    false_negative = np.zeros(samples, np.uint8)
    zero_allocation = True
    restitution, regularization = contact_law_parameters(law.solref_time_s)

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
        response_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        timing = session.solve_coupled_contact_impulse(
            prospective_step_velocity(points, velocity_series[sample]),
            delassus,
            upper_series[sample],
            friction,
            restitution,
            regularization,
            SWEEPS,
            predicted_impulse,
            velocity_after,
        )
        solve_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        actual_delta = np.einsum(
            "dca,ca->d", response, actual_impulse, optimize=True
        )
        predicted_delta = np.einsum(
            "dca,ca->d", response, predicted_impulse, optimize=True
        )
        residual[sample] = raw_error_series[sample] + actual_delta - predicted_delta
        predicted[sample] = predicted_impulse
        actual_center = actual_normal_centers(points, actual_impulse)
        predicted_center = actual_normal_centers(points, predicted_impulse)
        center_error[sample] = np.linalg.norm(actual_center - predicted_center, axis=1)
        predicted_active = np.linalg.norm(predicted_impulse) > 1.0e-12
        actual_active = np.linalg.norm(actual_impulse) > 1.0e-12
        false_positive[sample] = predicted_active and not actual_active
        false_negative[sample] = actual_active and not predicted_active
        severity[sample] = np.linalg.norm(predicted_impulse) / weight_tick_impulse
        root_fraction = (
            FROZEN_SPLIT_PROFILE["root_intercept"]
            + FROZEN_SPLIT_PROFILE["root_slope"] * severity[sample]
        )
        articulated_fraction = (
            FROZEN_SPLIT_PROFILE["articulated_intercept"]
            + FROZEN_SPLIT_PROFILE["articulated_slope"] * severity[sample]
        )
        timing = session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
            root_position,
            quaternion,
            q,
            (root_fraction * root_fraction) * unit_twice_energy,
            (articulated_fraction * articulated_fraction) * unit_twice_energy,
            lower,
            upper,
        )
        split_timing[sample, 0] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        # Repeat exactly to retain an independent determinism witness.
        first_lower = lower.copy()
        first_upper = upper.copy()
        timing = session.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
            root_position,
            quaternion,
            q,
            (root_fraction * root_fraction) * unit_twice_energy,
            (articulated_fraction * articulated_fraction) * unit_twice_energy,
            lower,
            upper,
        )
        split_timing[sample, 1] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        if not np.array_equal(lower, first_lower) or not np.array_equal(upper, first_upper):
            raise RuntimeError("split kinetic holdout query is not bitwise deterministic")
        interval_width[sample] = upper - lower
        component_coverage[sample] = np.abs(residual[sample]) <= upper + 1.0e-12
        covered[sample] = np.all(component_coverage[sample])

    width = grouped_summary(interval_width)
    lower_bound = grouped_summary(2.0 * np.abs(residual))
    width_gate = all(width[name]["p95"] <= limit for name, limit in WIDTH_GATES.items())
    strict = bool(np.all(covered))
    result = {
        "law": prefix,
        "samples": samples,
        "restitution": restitution,
        "regularization_ratio": regularization,
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(component_coverage)),
        "uncovered_samples": int(np.count_nonzero(covered == 0)),
        "strict_coverage_passed": strict,
        "interval_width": width,
        "predictor_lower_bound_width": lower_bound,
        "width_gate_passed": width_gate,
        "center_error_m": distribution(center_error.reshape(-1)),
        "predicted_impulse_norm_ns": distribution(
            np.linalg.norm(predicted, axis=(1, 2))
        ),
        "actual_impulse_norm_ns": distribution(
            np.linalg.norm(actual_impulse_series, axis=(1, 2))
        ),
        "false_positive_samples": int(np.count_nonzero(false_positive)),
        "false_negative_samples": int(np.count_nonzero(false_negative)),
        "response_timing_ns": distribution(response_timing),
        "coupled_solve_timing_ns": distribution(solve_timing),
        "split_bound_timing_ns": distribution(split_timing.reshape(-1)),
        "zero_rust_allocation": zero_allocation,
        "profile_promoted": bool(strict and width_gate and zero_allocation),
    }
    scored = {
        f"{prefix}_coupled_predicted_impulse": predicted,
        f"{prefix}_coupled_residual": residual,
        f"{prefix}_coupled_interval_width": interval_width,
        f"{prefix}_coupled_component_coverage": component_coverage,
        f"{prefix}_coupled_covered": covered,
        f"{prefix}_coupled_center_error": center_error,
        f"{prefix}_coupled_severity": severity,
        f"{prefix}_coupled_solve_timing_ns": solve_timing,
        f"{prefix}_split_bound_timing_ns": split_timing,
        f"{prefix}_coupled_false_positive": false_positive,
        f"{prefix}_coupled_false_negative": false_negative,
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
        law.name for law in (*R213_LAWS, *R214_LAWS, *R215_LAWS)
    }
    if {law.name for law in FRESH_CONTACT_LAWS} & prior_names:
        raise SystemExit("R218 contact-law names must be fresh")
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
        scored, scored_arrays = score_fresh_law(model, law, offset, source_arrays)
        results.append(scored)
        replay.update(source_arrays)
        replay.update(scored_arrays)
    mechanism_passed = all(result["zero_rust_allocation"] for result in results)
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
        "frozen_split_profile": FROZEN_SPLIT_PROFILE,
        "frozen_coupled_sweeps": SWEEPS,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "results": results,
    }
    table = []
    for result in results:
        width = result["interval_width"]
        lower_bound = result["predictor_lower_bound_width"]
        table.append(
            [
                result["law"],
                f"{result['restitution']:.3f} / {result['regularization_ratio']:.3f}",
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{result['center_error_m']['p95'] * 1_000.0:.1f}",
                f"{lower_bound['joint_rad_s']['p95']:.3f}",
                f"{width['root_angular_rad_s']['p95']:.3f} / {width['root_linear_m_s']['p95']:.3f} / {width['joint_rad_s']['p95']:.3f}",
                f"{result['coupled_solve_timing_ns']['p99'] / 1_000.0:.3f}",
                "yes" if result["zero_rust_allocation"] else "NO",
                "PASS" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 coupled contact-law fresh holdout · r218",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- R217's coupled eight-point time-step law, 16 sweeps, relaxation mapping, and split root/articulated energy coefficients were frozen before these labels. The new soft/pyramidal/Euler and stiff/elliptic/RK4 laws begin at untouched state offsets 50,000 and 60,000; every sample resets.",
            "- MuJoCo supplies only the completed five-substep scoring label. No controller, policy, successful-rollout selection, or prior state enters either predictor. Rust owns full Delassus coupling, Coulomb projection, split kinetic support, bitwise repeat, and zero-allocation witnesses.",
            "- Strict sample coverage and 2.0/0.5/10.0 p95 root-angular/root-linear/joint widths are conjunctive. A predictor-centered interval cannot be narrower than twice its absolute residual.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                [
                    "law",
                    "restitution / reg",
                    "sample coverage",
                    "component coverage",
                    "CoP p95 mm",
                    "unavoidable joint width p95",
                    "frozen width p95 ω/v/joint",
                    "solve p99 µs",
                    "zero alloc",
                    "profile",
                ],
                table,
            ),
            "",
            "## Decision",
            "",
            (
                "Both untouched laws satisfy strict coverage and useful width. This promotes only the frozen coupled contact/residual profile to the next terminal-consequence experiment; estimator provenance, deadlines and hardware remain separate."
                if promoted
                else "The allocation-free mechanism crosses the untouched laws, but the frozen construction does not satisfy strict coverage and useful width together. No holdout miss is tuned back into the profile. Retain the generic coupled solver and split support; reject this calibration and require typed contact-estimator uncertainty or a higher-order compliant law before authority."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-coupled-contact-law-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_COUPLED_CONTACT_LAW_HOLDOUT.md").write_text(report)
    np.savez_compressed(output / "g1-coupled-contact-law-holdout-replay.npz", **replay)
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
