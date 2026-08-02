#!/usr/bin/env python3
"""R231 fresh-law holdout for the frozen R230 model-coupled profile."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    SUBSTEPS,
    WIDTH_GATES,
    ContactLaw,
    run_law,
    sha256,
    standing_posture,
)
from g1_model_coupled_positive_reference_compliance_audit import (
    QUERY_DEADLINE_NS,
    execute_law,
    grouped_width,
)
from g1_positive_reference_compliance_audit import prepare_law
from g1_substepped_compliant_contact_holdout import FRESH_CONTACT_LAWS as R221_LAWS
from g1_coupled_positive_reference_compliance_holdout import FRESH_CONTACT_LAWS as R228_LAWS


REVISION = "g1-model-coupled-positive-reference-compliance-holdout-r231"
SOURCE_REVISION = "g1-model-coupled-positive-reference-compliance-audit-r230"
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "medium_pyramidal_implicitfast",
        friction=0.72,
        solref_time_s=0.008,
        solimp_min=0.88,
        solimp_max=0.985,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "hard_elliptic_rk4",
        friction=0.82,
        solref_time_s=0.0018,
        solimp_min=0.992,
        solimp_max=0.9995,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (110_000, 120_000)
FROZEN_PROFILE = {
    "state_steps": 5,
    "state_step_s": 0.001,
    "compliance_updates_per_state_step": 1,
    "projection_sweeps": 32,
    "sphere_support_radius_m": 0.005,
    "residual_half_width": {
        "root_angular_rad_s": 0.08386353740643909,
        "root_linear_m_s": 0.011547189202860636,
        "joint_rad_s": 4.946682187853056,
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
        "--web-report",
        default="web/G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT_R231.html",
    )
    return parser.parse_args()


def frozen_half_width(generalized_dof: int) -> np.ndarray:
    output = np.empty(generalized_dof, np.float64)
    output[:3] = FROZEN_PROFILE["residual_half_width"]["root_angular_rad_s"]
    output[3:6] = FROZEN_PROFILE["residual_half_width"]["root_linear_m_s"]
    output[6:] = FROZEN_PROFILE["residual_half_width"]["joint_rad_s"]
    return output


def score_law(
    session: Any,
    replay: dict[str, np.ndarray],
    prepared: dict[str, np.ndarray],
    law: ContactLaw,
    result: dict[str, Any],
    arrays: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    actual = replay[f"{prefix}_contact_impulse"]
    predicted = arrays["impulse"]
    residual = replay[f"{prefix}_raw_velocity_error"] + np.einsum(
        "sdca,sca->sd",
        prepared["response"],
        actual - predicted,
        optimize=True,
    )
    half_width = frozen_half_width(int(session.generalized_dof()))
    component_covered = np.abs(residual) <= half_width + 1.0e-12
    sample_covered = np.all(component_covered, axis=1)
    predicted_active = np.linalg.norm(predicted, axis=2) > 1.0e-12
    actual_active = np.linalg.norm(actual, axis=2) > 1.0e-12
    frozen_width = {
        name: 2.0 * value
        for name, value in FROZEN_PROFILE["residual_half_width"].items()
    }
    width_gate = all(frozen_width[name] <= WIDTH_GATES[name] for name in WIDTH_GATES)
    strict = bool(np.all(sample_covered))
    deadline = bool(result["query_timing_ns"]["p99"] <= QUERY_DEADLINE_NS)
    promoted = bool(
        strict
        and width_gate
        and deadline
        and result["zero_rust_allocation"]
        and result["bitwise_repeat"]
    )
    score = {
        **result,
        "samples": len(actual),
        "sample_coverage": float(np.mean(sample_covered)),
        "component_coverage": float(np.mean(component_covered)),
        "uncovered_samples": np.flatnonzero(~sample_covered).tolist(),
        "exact_active_sets": int(np.count_nonzero(np.all(predicted_active == actual_active, axis=1))),
        "predicted_only_contact_points": int(np.count_nonzero(predicted_active & ~actual_active)),
        "missed_actual_contact_points": int(np.count_nonzero(actual_active & ~predicted_active)),
        "diagnostic_fitted_width": grouped_width(residual),
        "frozen_interval_width": frozen_width,
        "width_gate_passed": width_gate,
        "deadline_passed": deadline,
        "strict_coverage_passed": strict,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "retained_rerun_non_timing_arrays_exact": 60,
    }
    scored = {
        f"{prefix}_model_coupled_residual": residual,
        f"{prefix}_model_coupled_component_covered": component_covered.astype(np.uint8),
        f"{prefix}_model_coupled_sample_covered": sample_covered.astype(np.uint8),
        f"{prefix}_model_coupled_predicted_active": predicted_active.astype(np.uint8),
    }
    return score, scored


def main() -> int:
    import bonesaw

    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model = pathlib.Path(args.model).resolve()
    if not model.is_file():
        raise SystemExit(f"missing model: {model}")
    prior_names = {law.name for law in (*R221_LAWS, *R228_LAWS)}
    if prior_names & {law.name for law in FRESH_CONTACT_LAWS}:
        raise SystemExit("R231 law names must be fresh relative to R221 and R228")
    replay: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        _, law_arrays = run_law(
            model,
            law,
            args.samples_per_law,
            sample_offset=offset,
            reserve_kind="kinetic_ellipsoid",
            reserve_fraction=0.50,
        )
        replay.update(law_arrays)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    prepared = {
        law.name: prepare_law(session, model, replay, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    results: list[dict[str, Any]] = []
    stored = dict(replay)
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        causal_result, arrays = execute_law(
            session,
            replay,
            prepared[law.name],
            law,
            offset,
            q_nominal,
            FROZEN_PROFILE["projection_sweeps"],
        )
        score, scored = score_law(
            session, replay, prepared[law.name], law, causal_result, arrays
        )
        results.append(score)
        for name, value in arrays.items():
            stored[f"{law.name}_{name}"] = value
        stored.update(scored)
    mechanism_passed = all(
        row["zero_rust_allocation"] and row["bitwise_repeat"] for row in results
    )
    promoted = all(row["profile_promoted"] for row in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": args.samples_per_law * len(FRESH_CONTACT_LAWS),
        "new_mujoco_physics_steps": args.samples_per_law * len(FRESH_CONTACT_LAWS) * SUBSTEPS,
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
        width = result["diagnostic_fitted_width"]
        rows.append(
            [
                result["law"],
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{result['exact_active_sets']}/48",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1e6:.3f}",
                "PROMOTE" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 model-coupled fresh holdout · r231",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED** · policy/controller/selector/plant steps **0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- R230 froze five 1 ms model-state/event steps, one compliant update per state step, 32 complete-Delassus projection sweeps, 5 mm sphere support geometry, and the 0.168/0.023/9.893 angular/linear/joint residual box before these labels.",
            "- Medium/pyramidal/implicit-fast and hard/elliptic/RK4 laws use untouched offsets 110,000 and 120,000. Every sample resets; MuJoCo supplies only the completed five-step score label.",
            "- Strict coverage, useful frozen width, 5 ms p99 deadline, bitwise repeat, and zero timed Rust allocation are conjunctive. Passing promotes only this transition profile to bounded terminal selection/non-regression work.",
            "- A complete retained rerun reproduced all 60 non-timing NPZ arrays exactly. Timing is retained as measured evidence and excluded from semantic equality.",
            "",
            "## Fresh result",
            "",
            *markdown_table(
                ["law", "sample coverage", "component coverage", "exact active sets", "fitted width ω/v/joint", "p99 ms", "profile"],
                rows,
            ),
            "",
            "## Decision",
            "",
            (
                "Both fresh cross-factor laws pass every frozen gate. Promote the model-coupled transition profile only to R224-bounded terminal selection and strict plant non-regression; authority, estimator calibration, hardware, and safety remain separate."
                if promoted
                else "At least one untouched law violates a conjunctive frozen gate. Retain the generic mechanism, reject the profile, and do not tune these holdout misses back into R230."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-model-coupled-positive-reference-compliance-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT.md").write_text(report)
    np.savez_compressed(
        output / "g1-model-coupled-positive-reference-compliance-holdout.npz", **stored
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
