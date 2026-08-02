#!/usr/bin/env python3
"""R244 untouched cross-integrator holdout for pyramid-edge coordinates."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, SUBSTEPS, ContactLaw, run_law, sha256, standing_posture
from g1_generalized_rk4_holdout import score_law
from g1_model_coupled_positive_reference_compliance_audit import CAUSAL_SOURCE_SUFFIXES, QUERY_DEADLINE_NS, execute_law
from g1_positive_reference_compliance_audit import (
    MODEL_INTEGRATOR_IDS,
    law_model_constraint_rhs_integrator_id,
    law_model_edge_cone_id,
    prepare_law,
)


REVISION = "g1-pyramid-edge-cross-integrator-holdout-r244"
SOURCE_REVISION = "g1-pyramid-edge-convergence-audit-r243"
FROZEN_PROJECTION_SWEEPS = 64
FRESH_CONTACT_LAWS = (
    ContactLaw(
        "balanced_pyramidal_implicitfast_edge_r244",
        friction=0.69,
        solref_time_s=0.0041,
        solimp_min=0.968,
        solimp_max=0.996,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
    ContactLaw(
        "hard_pyramidal_rk4_edge_r244",
        friction=0.93,
        solref_time_s=0.0022,
        solimp_min=0.990,
        solimp_max=0.9992,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (190_000, 200_000)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/G1_PYRAMID_EDGE_CROSS_INTEGRATOR_HOLDOUT_R244.html")
    return parser.parse_args()


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
    causal = {
        f"{law.name}_{suffix}": replay[f"{law.name}_{suffix}"].copy()
        for law in FRESH_CONTACT_LAWS
        for suffix in CAUSAL_SOURCE_SUFFIXES
    }
    prepared = {
        law.name: prepare_law(session, model, causal, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    results: list[dict[str, Any]] = []
    stored = dict(replay)
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        causal_result, arrays = execute_law(
            session,
            causal,
            prepared[law.name],
            law,
            offset,
            q_nominal,
            FROZEN_PROJECTION_SWEEPS,
            model_integrator_id_fn=law_model_constraint_rhs_integrator_id,
            model_cone_id_fn=law_model_edge_cone_id,
        )
        score, scored = score_law(session, replay, prepared[law.name], law, offset, causal_result, arrays)
        score["model_integrator_id"] = law_model_constraint_rhs_integrator_id(law)
        score["model_cone_id"] = law_model_edge_cone_id(law)
        results.append(score)
        for name, value in arrays.items():
            stored[f"{law.name}_{name}"] = value
        stored.update(scored)

    mechanism_passed = all(
        row["zero_rust_allocation"] and row["bitwise_repeat"] and row["deadline_passed"]
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
        "new_mujoco_physics_steps": args.samples_per_law * len(FRESH_CONTACT_LAWS) * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "fresh_laws_and_state_offsets": True,
        "both_rows_use_pyramid_edge_coordinates": True,
        "profile_frozen_before_holdout_labels": True,
        "predictor_received_causal_arrays_only": True,
        "frozen_projection_sweeps": FROZEN_PROJECTION_SWEEPS,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "model_integrator_ids": MODEL_INTEGRATOR_IDS,
        "mechanism_passed": mechanism_passed,
        "profile_promoted": promoted,
        "authority_admitted": False,
        "results": results,
    }
    table = []
    for result in results:
        width = result["diagnostic_fitted_width"]
        table.append(
            [
                result["law"],
                f"{result['model_integrator_id']} / {result['model_cone_id']}",
                f"{100 * result['sample_coverage']:.3f}%",
                f"{result['exact_active_sets']}/{result['samples']}",
                f"{result['predicted_only_contact_points']} / {result['missed_actual_contact_points']}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1e6:.3f}",
                "PROMOTE" if result["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw pyramid-edge fresh holdout · r244",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{'PROMOTED' if promoted else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "Two new pyramidal laws and disjoint offsets 190,000/200,000 were generated after R243 froze 64 sweeps. Both implicitfast and RK4 rows use model-only pyramid-edge cone ABI 2; RK4 separately uses current-stage integrator ABI 4.",
            "",
            *markdown_table(["law", "integrator / cone ABI", "coverage", "exact active", "pred-only / missed", "width · ang / lin / joint", "p99 ms", "decision"], table),
            "",
            "Prediction received causal arrays only; scoring compares returned and reference final generalized tangents. Promotion requires strict coverage, deadline, repeat, and zero allocation for both rows. Authority remains a separate disabled boundary.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-pyramid-edge-cross-integrator-holdout-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(output / "g1-pyramid-edge-cross-integrator-holdout.npz", **stored)
    (output / "G1_PYRAMID_EDGE_CROSS_INTEGRATOR_HOLDOUT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw pyramid-edge holdout · r244"))
    print(json.dumps({"mechanism_passed": mechanism_passed, "profile_promoted": promoted, "authority_admitted": False}, indent=2, sort_keys=True))
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
