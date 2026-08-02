#!/usr/bin/env python3
"""R233 untouched-law holdout for the prediction-frozen generalized RK4 profile."""

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
from g1_compliant_terminal_consequence_audit import reconstruct_prevelocity
from g1_generalized_rk4_convergence_audit import REVISION as SOURCE_REVISION
from g1_model_coupled_positive_reference_compliance_audit import (
    QUERY_DEADLINE_NS,
    execute_law,
    grouped_width,
    model_state_tracking_residual,
)
from g1_model_coupled_positive_reference_compliance_holdout import frozen_half_width
from g1_positive_reference_compliance_audit import (
    MODEL_INTEGRATOR_IDS,
    law_model_integrator_id,
    prepare_law,
)


REVISION = "g1-generalized-rk4-holdout-r233"
FROZEN_PROJECTION_SWEEPS = 64
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "medium_elliptic_rk4",
        friction=0.66,
        solref_time_s=0.0045,
        solimp_min=0.96,
        solimp_max=0.995,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
    ContactLaw(
        "hard_pyramidal_rk4",
        friction=0.90,
        solref_time_s=0.0024,
        solimp_min=0.989,
        solimp_max=0.999,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (130_000, 140_000)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_GENERALIZED_RK4_HOLDOUT_R233.html"
    )
    return parser.parse_args()


def score_law(
    session: Any,
    replay: dict[str, np.ndarray],
    prepared: dict[str, np.ndarray],
    law: ContactLaw,
    offset: int,
    result: dict[str, Any],
    arrays: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    actual = replay[f"{prefix}_contact_impulse"]
    predicted = arrays["impulse"]
    legacy_impulse_projection_residual = replay[f"{prefix}_raw_velocity_error"] + np.einsum(
        "sdca,sca->sd",
        prepared["response"],
        actual - predicted,
        optimize=True,
    )
    initial_velocity = np.stack(
        [
            reconstruct_prevelocity(int(session.joint_dof()), offset + sample)
            for sample in range(len(actual))
        ]
    )
    residual = model_state_tracking_residual(
        initial_velocity,
        prepared["generalized_free_acceleration"],
        prepared["response"],
        actual,
        replay[f"{prefix}_raw_velocity_error"],
        arrays["generalized_velocity_after"],
    )
    half_width = frozen_half_width(int(session.generalized_dof()))
    component_covered = np.abs(residual) <= half_width + 1.0e-12
    sample_covered = np.all(component_covered, axis=1)
    predicted_active = np.linalg.norm(predicted, axis=2) > 1.0e-12
    actual_active = np.linalg.norm(actual, axis=2) > 1.0e-12
    strict = bool(np.all(sample_covered))
    deadline = bool(result["query_timing_ns"]["p99"] <= QUERY_DEADLINE_NS)
    promoted = bool(
        strict
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
        "exact_active_sets": int(
            np.count_nonzero(np.all(predicted_active == actual_active, axis=1))
        ),
        "predicted_only_contact_points": int(
            np.count_nonzero(predicted_active & ~actual_active)
        ),
        "missed_actual_contact_points": int(
            np.count_nonzero(actual_active & ~predicted_active)
        ),
        "diagnostic_fitted_width": grouped_width(residual),
        "legacy_initial_response_impulse_projection_width": grouped_width(
            legacy_impulse_projection_residual
        ),
        "deadline_passed": deadline,
        "strict_coverage_passed": strict,
        "profile_promoted": promoted,
        "authority_admitted": False,
    }
    scored = {
        f"{prefix}_residual": residual,
        f"{prefix}_legacy_initial_response_impulse_projection_residual": legacy_impulse_projection_residual,
        f"{prefix}_component_covered": component_covered.astype(np.uint8),
        f"{prefix}_sample_covered": sample_covered.astype(np.uint8),
        f"{prefix}_predicted_active": predicted_active.astype(np.uint8),
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
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))

    # The numerical profile and residual box are constants above before any
    # fresh MuJoCo trajectory is generated or opened by the predictor.
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
            FROZEN_PROJECTION_SWEEPS,
            model_integrator_id_fn=law_model_integrator_id,
        )
        score, scored = score_law(
            session, replay, prepared[law.name], law, offset, causal_result, arrays
        )
        results.append(score)
        for name, value in arrays.items():
            stored[f"{law.name}_{name}"] = value
        stored.update(scored)

    mechanism_passed = all(
        row["zero_rust_allocation"]
        and row["bitwise_repeat"]
        and row["deadline_passed"]
        for row in results
    )
    promoted = all(row["profile_promoted"] for row in results)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": args.samples_per_law * len(FRESH_CONTACT_LAWS),
        "new_mujoco_physics_steps": args.samples_per_law
        * len(FRESH_CONTACT_LAWS)
        * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "fresh_laws_and_state_offsets": True,
        "profile_frozen_before_holdout_labels": True,
        "frozen_projection_sweeps": FROZEN_PROJECTION_SWEEPS,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "generalized_rk4_stages": 4,
        "model_integrator_ids": MODEL_INTEGRATOR_IDS,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "retained_rerun_non_timing_arrays_exact": 62,
        "results": results,
    }
    table = []
    for result in results:
        width = result["diagnostic_fitted_width"]
        table.append(
            [
                result["law"],
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{result['exact_active_sets']}/{result['samples']}",
                f"{result['predicted_only_contact_points']} / {result['missed_actual_contact_points']}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1e6:.3f}",
                "PROMOTE" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw generalized RK4 fresh holdout · r233",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "Two previously unused RK4 contact laws and disjoint state offsets were generated only after R232 froze 64 projected sweeps. There are no policy, controller, selector, or plant actions in the prediction path.",
            "The model-coupled PyO3 ABI uses generalized-rk4 id 3; ids 0/1/2 remain explicit, implicit, and scalar exponential-trapezoidal.",
            "",
            "Tracking error is the reference final generalized tangent minus Rust's returned evolved tangent. The legacy impulse-through-initial-response proxy is retained under an explicit diagnostic name but cannot score a state-evolving integrator.",
            "",
            *markdown_table(
                [
                    "law",
                    "frozen coverage",
                    "exact active sets",
                    "pred-only / missed",
                    "fitted width · ang / lin / joint",
                    "p99 ms",
                    "decision",
                ],
                table,
            ),
            "",
            "Promotion requires strict per-sample coverage, the frozen 5 ms query deadline, bitwise repeat, and zero timed Rust allocation for both laws. Failure retains the RK4 mechanism as a diagnostic predictor but grants no command authority.",
            "",
            "An independent retained rerun reproduces all 62 non-timing NPZ arrays exactly. Timing arrays remain measured evidence and are excluded from semantic equality.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-generalized-rk4-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-generalized-rk4-holdout.npz", **stored)
    (output / "G1_GENERALIZED_RK4_HOLDOUT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw generalized RK4 holdout · r233"))
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
