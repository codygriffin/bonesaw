#!/usr/bin/env python3
"""R245 causal work freeze for both pyramid-edge integrator profiles."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, WIDTH_GATES, sha256, standing_posture
from g1_constraint_rhs_cross_integrator_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS
from g1_generalized_rk4_holdout import score_law
from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    QUERY_DEADLINE_NS,
    execute_law,
    grouped_width,
)
from g1_positive_reference_compliance_audit import (
    MODEL_INTEGRATOR_IDS,
    law_model_constraint_rhs_integrator_id,
    prepare_law,
)
from g1_pyramid_edge_convergence_audit import PROJECTION_SWEEPS, REFINEMENT_FRACTION_GATE, SOURCE_REPLAY


REVISION = "g1-pyramid-edge-cross-profile-audit-r245"
SOURCE_REVISION = "g1-pyramid-edge-convergence-audit-r243"
FORCED_MODEL_CONE_ID = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/G1_PYRAMID_EDGE_CROSS_PROFILE_AUDIT_R245.html")
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R245 requires the pinned G1 model and immutable R241 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    with np.load(source) as replay:
        causal = {
            f"{law.name}_{suffix}": replay[f"{law.name}_{suffix}"].copy()
            for law in FRESH_CONTACT_LAWS
            for suffix in CAUSAL_SOURCE_SUFFIXES
        }
    prepared = {
        law.name: prepare_law(session, model, causal, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    profiles: list[dict[str, Any]] = []
    selected_arrays: dict[str, dict[str, np.ndarray]] = {}
    selected_results: dict[str, dict[str, Any]] = {}
    stored: dict[str, np.ndarray] = {}
    for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
        runs: dict[int, tuple[dict[str, Any], dict[str, np.ndarray]]] = {}
        for sweeps in PROJECTION_SWEEPS:
            result, arrays = execute_law(
                session,
                causal,
                prepared[law.name],
                law,
                offset,
                q_nominal,
                sweeps,
                model_integrator_id_fn=law_model_constraint_rhs_integrator_id,
                model_cone_id_fn=lambda _: FORCED_MODEL_CONE_ID,
            )
            runs[sweeps] = (result, arrays)
            for name, value in arrays.items():
                stored[f"sweeps{sweeps}_{law.name}_{name}"] = value
        rows = []
        for sweeps in PROJECTION_SWEEPS:
            width = None
            fraction = None
            if sweeps * 2 in runs:
                width = grouped_width(
                    runs[sweeps * 2][1]["generalized_velocity_after"]
                    - runs[sweeps][1]["generalized_velocity_after"]
                )
                fraction = max(width[name] / WIDTH_GATES[name] for name in WIDTH_GATES)
            timing = runs[sweeps][0]["query_timing_ns"]
            eligible = bool(
                fraction is not None
                and fraction <= REFINEMENT_FRACTION_GATE
                and timing["p99"] <= QUERY_DEADLINE_NS
                and runs[sweeps][0]["zero_rust_allocation"]
                and runs[sweeps][0]["bitwise_repeat"]
            )
            rows.append(
                {
                    "projection_sweeps": sweeps,
                    "double_sweep_refinement_width": width,
                    "double_sweep_refinement_fraction_of_gate": fraction,
                    "query_p99_ns": timing["p99"],
                    "zero_rust_allocation": runs[sweeps][0]["zero_rust_allocation"],
                    "bitwise_repeat": runs[sweeps][0]["bitwise_repeat"],
                    "prediction_only_selection_eligible": eligible,
                }
            )
        eligible_rows = [row for row in rows if row["prediction_only_selection_eligible"]]
        selected = min(eligible_rows, key=lambda row: row["projection_sweeps"]) if eligible_rows else None
        if selected is not None:
            selected_arrays[law.name] = runs[selected["projection_sweeps"]][1]
            selected_results[law.name] = runs[selected["projection_sweeps"]][0]
        profiles.append(
            {
                "law": law.name,
                "model_integrator_id": law_model_constraint_rhs_integrator_id(law),
                "model_cone_id": FORCED_MODEL_CONE_ID,
                "profile_rows": rows,
                "selected_projection_sweeps": selected["projection_sweeps"] if selected else None,
            }
        )

    frozen = all(profile["selected_projection_sweeps"] is not None for profile in profiles)
    spent_scores: list[dict[str, Any]] = []
    if frozen:
        with np.load(source) as spent:
            for law, offset, profile in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, profiles, strict=True):
                score, scored = score_law(
                    session,
                    spent,
                    prepared[law.name],
                    law,
                    offset,
                    selected_results[law.name],
                    selected_arrays[law.name],
                )
                score["model_integrator_id"] = profile["model_integrator_id"]
                score["model_cone_id"] = FORCED_MODEL_CONE_ID
                spent_scores.append(score)
                stored.update({f"spent_{name}": value for name, value in scored.items()})

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "surface_material_velocity_includes_omega_cross_r": True,
        "both_profiles_forced_to_pyramid_edge_coordinates": True,
        "completed_labels_used_for_selection": False,
        "completed_spent_labels_accessed_after_selection": bool(spent_scores),
        "new_mujoco_physics_or_integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "projection_sweep_candidates": PROJECTION_SWEEPS,
        "refinement_fraction_gate": REFINEMENT_FRACTION_GATE,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "model_integrator_ids": MODEL_INTEGRATOR_IDS,
        "profiles": profiles,
        "profile_frozen_for_fresh_holdout": frozen,
        "spent_label_diagnostic": spent_scores,
        "authority_admitted": False,
    }
    profile_table = []
    for profile in profiles:
        selected = next(
            (row for row in profile["profile_rows"] if row["projection_sweeps"] == profile["selected_projection_sweeps"]),
            None,
        )
        profile_table.append(
            [
                profile["law"],
                profile["model_integrator_id"],
                profile["selected_projection_sweeps"] or "—",
                f"{100 * selected['double_sweep_refinement_fraction_of_gate']:.3f}%" if selected else "—",
                f"{selected['query_p99_ns'] / 1e6:.3f}" if selected else "—",
            ]
        )
    spent_table = []
    for score in spent_scores:
        width = score["diagnostic_fitted_width"]
        spent_table.append(
            [score["law"], f"{100 * score['sample_coverage']:.3f}%", f"{score['exact_active_sets']}/{score['samples']}", f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}"]
        )
    report = "\n".join(
        [
            "# Bonesaw pyramid-edge cross-profile audit · r245",
            "",
            f"> Per-profile work **{'FROZEN' if frozen else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "R245 corrects the R243 profiling gap by forcing both the non-RK and RK4 rows through model-only pyramid-edge cone ABI 2. Each integrator freezes its own smallest causal refinement profile; completed labels remain inaccessible until afterward.",
            "",
            *markdown_table(["causal law", "integrator ABI", "selected sweeps", "2× refinement", "p99 ms"], profile_table),
            "",
            "The model now evaluates friction velocity at the instantaneous sphere surface material point, retaining centre-minus-radius gap geometry while including ω×r in tangent motion.",
            "",
            "## Spent R241 diagnostic (ineligible for selection)",
            "",
            *markdown_table(["law", "coverage", "exact active", "width · ang / lin / joint"], spent_table),
            "",
            "No new reference physics, policy, controller, selector, or plant step is used. A new untouched crossed holdout is required.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-pyramid-edge-cross-profile-audit-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(output / "g1-pyramid-edge-cross-profile-audit.npz", **stored)
    (output / "G1_PYRAMID_EDGE_CROSS_PROFILE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw pyramid-edge cross profile · r245"))
    print(json.dumps({"profile_frozen_for_fresh_holdout": frozen, "selected_projection_sweeps": {p['model_integrator_id']: p['selected_projection_sweeps'] for p in profiles}, "authority_admitted": False}, indent=2, sort_keys=True))
    return 0 if frozen else 1


if __name__ == "__main__":
    raise SystemExit(main())
