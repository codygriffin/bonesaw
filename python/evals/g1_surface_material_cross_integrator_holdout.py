#!/usr/bin/env python3
"""R246 untouched crossed holdout for sphere surface-material motion."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import mujoco
import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, SUBSTEPS, ContactLaw, run_law, sha256, standing_posture
from g1_generalized_rk4_holdout import score_law
from g1_model_coupled_positive_reference_compliance_audit import CAUSAL_SOURCE_SUFFIXES, QUERY_DEADLINE_NS, execute_law
from g1_positive_reference_compliance_audit import law_model_constraint_rhs_integrator_id, prepare_law


REVISION = "g1-surface-material-cross-integrator-holdout-r246"
SOURCE_REVISION = "g1-pyramid-edge-cross-profile-audit-r245"
FROZEN_SWEEPS_BY_INTEGRATOR_ID = {0: 64, 4: 32}
MODEL_CONE_ID = 2
FRESH_CONTACT_LAWS = (
    ContactLaw("medium_pyramidal_implicitfast_surface_r246", 0.77, 0.0036, 0.975, 0.997, int(mujoco.mjtCone.mjCONE_PYRAMIDAL), int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)),
    ContactLaw("rigid_pyramidal_rk4_surface_r246", 0.88, 0.0026, 0.986, 0.9986, int(mujoco.mjtCone.mjCONE_PYRAMIDAL), int(mujoco.mjtIntegrator.mjINT_RK4)),
)
SAMPLE_OFFSETS = (230_000, 240_000)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/G1_SURFACE_MATERIAL_CROSS_INTEGRATOR_HOLDOUT_R246.html")
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    if args.samples_per_law <= 0 or not model.is_file():
        raise SystemExit("R246 requires a positive sample count and the pinned G1 model")
    session = bonesaw.ContactTransitionModelSession(str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4)
    q_nominal = standing_posture(list(session.joint_names()))
    replay: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        _, arrays = run_law(model, law, args.samples_per_law, sample_offset=offset, reserve_kind="kinetic_ellipsoid", reserve_fraction=0.50)
        replay.update(arrays)
    causal = {
        f"{law.name}_{suffix}": replay[f"{law.name}_{suffix}"].copy()
        for law in FRESH_CONTACT_LAWS
        for suffix in CAUSAL_SOURCE_SUFFIXES
    }
    prepared = {
        law.name: prepare_law(session, model, causal, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    stored = dict(replay)
    results = []
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        integrator_id = law_model_constraint_rhs_integrator_id(law)
        result, arrays = execute_law(
            session, causal, prepared[law.name], law, offset, q_nominal,
            FROZEN_SWEEPS_BY_INTEGRATOR_ID[integrator_id],
            model_integrator_id_fn=law_model_constraint_rhs_integrator_id,
            model_cone_id_fn=lambda _: MODEL_CONE_ID,
        )
        score, scored = score_law(session, replay, prepared[law.name], law, offset, result, arrays)
        score["model_integrator_id"] = integrator_id
        score["model_cone_id"] = MODEL_CONE_ID
        results.append(score)
        stored.update({f"{law.name}_{name}": value for name, value in arrays.items()})
        stored.update(scored)
    mechanism_passed = all(row["zero_rust_allocation"] and row["bitwise_repeat"] and row["deadline_passed"] for row in results)
    holdout_passed = all(row["profile_promoted"] for row in results)
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
        "fresh_laws_and_state_offsets": True,
        "surface_material_velocity_includes_omega_cross_r": True,
        "constraint_rhs_uses_current_boundary_gap": True,
        "both_rows_use_pyramid_edge_coordinates": True,
        "profile_frozen_before_holdout_labels": True,
        "predictor_received_causal_arrays_only": True,
        "frozen_sweeps_by_integrator_id": FROZEN_SWEEPS_BY_INTEGRATOR_ID,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "mechanism_passed": mechanism_passed,
        "strict_holdout_passed": holdout_passed,
        "profile_promoted": holdout_passed,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        width = result["diagnostic_fitted_width"]
        rows.append([
            result["law"], f"{result['model_integrator_id']} / {result['model_cone_id']}", result["projection_sweeps"],
            f"{100 * result['sample_coverage']:.3f}%", f"{result['exact_active_sets']}/{result['samples']}",
            f"{result['predicted_only_contact_points']} / {result['missed_actual_contact_points']}",
            f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
            f"{result['query_timing_ns']['p99'] / 1e6:.3f}", "PROMOTE" if result["profile_promoted"] else "REJECT",
        ])
    report = "\n".join([
        "# Bonesaw surface-material fresh holdout · r246", "",
        f"> Runtime mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · strict holdout **{'PASS' if holdout_passed else 'FAIL'}** · authority **NOT ADMITTED**.", "",
        "Two new pyramidal laws and disjoint offsets 230,000/240,000 were generated after R245 froze per-integrator work. Gap geometry is centre-minus-radius; friction motion includes ω×r at the instantaneous surface point; aref uses the current collision-boundary gap.", "",
        *markdown_table(["law", "integrator / cone ABI", "sweeps", "coverage", "exact active", "pred-only / missed", "width · ang / lin / joint", "p99 ms", "decision"], rows), "",
        "Prediction received causal arrays only. Strict coverage, five-millisecond p99, bitwise repeat, and zero Rust allocation are conjunctive. Plant selection and authority remain disabled.",
    ]) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-surface-material-cross-integrator-holdout-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(output / "g1-surface-material-cross-integrator-holdout.npz", **stored)
    (output / "G1_SURFACE_MATERIAL_CROSS_INTEGRATOR_HOLDOUT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw surface-material holdout · r246"))
    print(json.dumps({"mechanism_passed": mechanism_passed, "strict_holdout_passed": holdout_passed, "authority_admitted": False}, indent=2, sort_keys=True))
    return 0 if holdout_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
