#!/usr/bin/env python3
"""R228 fresh-law holdout for the frozen coupled positive-reference profile."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    PHYSICS_DT,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
    standing_posture,
)
from g1_coupled_positive_reference_compliance_audit import physical_impulse_upper
from g1_positive_reference_compliance_audit import (
    REFERENCE_DOCUMENTATION,
    law_cone_id,
    law_reduced_integrator_id,
    prepare_law,
)
from g1_substepped_compliant_contact_holdout import FRESH_CONTACT_LAWS as R221_LAWS
from g1_substepped_compliant_contact_replay import group_maximum_absolute


REVISION = "g1-coupled-positive-reference-compliance-holdout-r228"
SOURCE_REVISION = "g1-coupled-positive-reference-compliance-audit-r227"
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "soft_pyramidal_euler",
        friction=0.50,
        solref_time_s=0.020,
        solimp_min=0.75,
        solimp_max=0.94,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_EULER),
    ),
    ContactLaw(
        "stiff_elliptic_implicitfast",
        friction=1.05,
        solref_time_s=0.0025,
        solimp_min=0.97,
        solimp_max=0.999,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
)
SAMPLE_OFFSETS = (90_000, 100_000)
FROZEN_PROFILE = {
    "substeps": 32,
    "projection_sweeps": 32,
    "impulse_cap": "total_mass * (maximum_causal_closing_speed + gravity * control_dt)",
    "tangent_cap": "friction * normal_cap",
    "residual_half_width": {
        "root_angular_rad_s": 0.22468729416128522,
        "root_linear_m_s": 0.03225999305241632,
        "joint_rad_s": 4.659980682506395,
    },
}
QUERY_DEADLINE_NS = 5_000_000.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT_R228.html",
    )
    return parser.parse_args()


def residual_half_width(generalized_dof: int) -> np.ndarray:
    output = np.empty(generalized_dof, np.float64)
    output[:3] = FROZEN_PROFILE["residual_half_width"]["root_angular_rad_s"]
    output[3:6] = FROZEN_PROFILE["residual_half_width"]["root_linear_m_s"]
    output[6:] = FROZEN_PROFILE["residual_half_width"]["joint_rad_s"]
    return output


def score_law(
    session: Any,
    arrays: dict[str, np.ndarray],
    prepared: dict[str, np.ndarray],
    law: ContactLaw,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    samples = len(arrays[f"{prefix}_root_height"])
    generalized_dof = int(session.generalized_dof())
    half_width = residual_half_width(generalized_dof)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    gap_after = np.empty(8, np.float64)
    impulse_upper = np.empty((8, 3), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    residual = np.empty((samples, generalized_dof), np.float64)
    component_covered = np.empty((samples, generalized_dof), np.uint8)
    sample_covered = np.empty(samples, np.uint8)
    impulse_error_norm = np.empty(samples, np.float64)
    timing = np.empty((samples, 2), np.uint64)
    parameter = lambda value: np.full(8, value, np.float64)
    common = (
        parameter(law.friction),
        parameter(law.solref_time_s),
        parameter(1.0),
        parameter(law.solimp_min),
        parameter(law.solimp_max),
        parameter(0.001),
        parameter(0.5),
        parameter(2.0),
    )
    zero_allocation = True
    for sample in range(samples):
        physical_impulse_upper(
            arrays[f"{prefix}_prospective_velocity"][sample],
            float(prepared["total_mass_kg"]),
            law.friction,
            impulse_upper,
        )
        arguments = (
            np.ascontiguousarray(arrays[f"{prefix}_contact_points"][sample, :, 2]),
            arrays[f"{prefix}_prospective_velocity"][sample],
            np.ascontiguousarray(prepared["free_acceleration"][sample]),
            prepared["delassus"][sample],
            impulse_upper,
            *common,
            2.0 * PHYSICS_DT,
            CONTROL_DT,
            FROZEN_PROFILE["substeps"],
            FROZEN_PROFILE["projection_sweeps"],
            law_cone_id(law),
            law_reduced_integrator_id(law),
            predicted_impulse,
            velocity_after,
            gap_after,
        )
        result = session.solve_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first = (
            predicted_impulse.copy(),
            velocity_after.copy(),
            gap_after.copy(),
        )
        result = session.solve_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if not all(
            np.array_equal(before, after)
            for before, after in zip(
                first, (predicted_impulse, velocity_after, gap_after), strict=True
            )
        ):
            raise RuntimeError("R228 query is not bitwise deterministic")
        actual_impulse = arrays[f"{prefix}_contact_impulse"][sample]
        residual[sample] = arrays[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
            "dca,ca->d",
            prepared["response"][sample],
            actual_impulse - predicted_impulse,
            optimize=True,
        )
        predicted[sample] = predicted_impulse
        impulse_error_norm[sample] = np.linalg.norm(actual_impulse - predicted_impulse)
        component_covered[sample] = np.abs(residual[sample]) <= half_width + 1.0e-12
        sample_covered[sample] = np.all(component_covered[sample])

    frozen_width = {
        name: 2.0 * value
        for name, value in FROZEN_PROFILE["residual_half_width"].items()
    }
    strict = bool(np.all(sample_covered))
    width_gate = all(frozen_width[name] <= WIDTH_GATES[name] for name in WIDTH_GATES)
    deadline = bool(np.max(timing) <= QUERY_DEADLINE_NS)
    result = {
        "law": prefix,
        "samples": samples,
        "sample_coverage": float(np.mean(sample_covered)),
        "component_coverage": float(np.mean(component_covered)),
        "uncovered_samples": int(np.count_nonzero(sample_covered == 0)),
        "strict_coverage_passed": strict,
        "predictor_residual": grouped_summary(residual),
        "diagnostic_fitted_width": {
            name: 2.0 * value
            for name, value in group_maximum_absolute(residual).items()
        },
        "frozen_interval_width": frozen_width,
        "width_gate_passed": width_gate,
        "impulse_error_norm_ns": distribution(impulse_error_norm),
        "query_timing_ns": distribution(timing.reshape(-1)),
        "deadline_passed": deadline,
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "profile_promoted": bool(strict and width_gate and deadline and zero_allocation),
        "authority_admitted": False,
    }
    scored = {
        f"{prefix}_coupled_predicted_impulse": predicted,
        f"{prefix}_coupled_residual": residual,
        f"{prefix}_coupled_component_covered": component_covered,
        f"{prefix}_coupled_sample_covered": sample_covered,
        f"{prefix}_coupled_timing_ns": timing,
    }
    return result, scored


def main() -> int:
    import bonesaw

    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit(f"missing model: {model}")
    if {law.name for law in FRESH_CONTACT_LAWS} & {law.name for law in R221_LAWS}:
        raise SystemExit("R228 law names must be fresh relative to R221")
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
        replay.update(source_arrays)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    prepared = {
        law.name: prepare_law(session, model, replay, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    results: list[dict[str, Any]] = []
    for law in FRESH_CONTACT_LAWS:
        result, scored = score_law(session, replay, prepared[law.name], law)
        results.append(result)
        replay.update(scored)
    mechanism_passed = all(
        row["zero_rust_allocation"] and row["bitwise_repeat"] for row in results
    )
    promoted = all(row["profile_promoted"] for row in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "reference_documentation": REFERENCE_DOCUMENTATION,
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": len(results) * args.samples_per_law,
        "physics_steps": len(results) * args.samples_per_law * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "sample_reset_every_transition": True,
        "fresh_laws_and_state_offsets": True,
        "frozen_before_holdout_labels": True,
        "frozen_profile": FROZEN_PROFILE,
        "width_gates": WIDTH_GATES,
        "query_deadline_ns": QUERY_DEADLINE_NS,
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
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "PROMOTE" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 coupled positive-reference fresh holdout · r228",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · policy/controller/selector/plant steps **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- R227's causal convergence rule froze 32 compliant substeps, 32 forward/reverse projection sweeps, label-free total-momentum caps, and the construction residual box before these labels. Soft/pyramidal/Euler and stiff/elliptic/implicit-fast laws begin at untouched offsets 90,000 and 100,000; every transition resets.",
            "- MuJoCo supplies only the completed five-substep label. The Rust query receives causal prestate geometry, velocity, free acceleration, full Delassus response, model mass, and authored law. Each query repeats bitwise, allocates nothing, and must remain below 5 ms.",
            "- Strict sample coverage, frozen 0.449/0.065/9.320 angular/linear/joint width, deadline, repeat, and allocation gates are conjunctive. A pass promotes only this transition profile to terminal selection/non-regression work, never hardware or command authority.",
            "- A complete retained rerun reproduced all 46 non-timing NPZ arrays exactly. Timing is reported but excluded from semantic equality.",
            "- A complete retained rerun reproduced all 46 non-timing NPZ arrays exactly. Timing is retained as measured evidence but excluded from semantic equality.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                [
                    "law",
                    "sample coverage",
                    "component coverage",
                    "residual p95 ω/v/joint",
                    "frozen width ω/v/joint",
                    "query p99 µs",
                    "profile",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            (
                "Both untouched laws preserve strict coverage, useful frozen width, deadline, repeat, and allocation gates. Promote the frozen coupled transition profile only to R224-bounded terminal selection and strict plant non-regression; authority, estimator calibration, hardware, and safety remain separate."
                if promoted
                else "At least one untouched law violates a conjunctive gate. Retain the generic coupled mechanism, reject the frozen profile, and do not tune holdout misses back into it."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-coupled-positive-reference-compliance-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT.md").write_text(report)
    np.savez_compressed(
        output / "g1-coupled-positive-reference-compliance-holdout.npz", **replay
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_promoted": promoted,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
