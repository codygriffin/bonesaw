#!/usr/bin/env python3
"""R243 causal convergence freeze for pyramid-edge contact coordinates."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, WIDTH_GATES, sha256, standing_posture
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
    law_model_edge_cone_id,
    prepare_law,
)
from g1_constraint_rhs_cross_integrator_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


REVISION = "g1-pyramid-edge-convergence-audit-r243"
SOURCE_REVISION = "g1-constraint-rhs-cross-integrator-holdout-r241"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-constraint-rhs-cross-integrator-holdout-r241/"
    "g1-constraint-rhs-cross-integrator-holdout.npz"
)
PROJECTION_SWEEPS = (1, 2, 4, 8, 16, 32, 64, 128, 256)
REFINEMENT_FRACTION_GATE = 0.02


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/G1_PYRAMID_EDGE_CONVERGENCE_AUDIT_R243.html")
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R243 requires the pinned G1 model and immutable R241 replay")
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
    results: dict[int, list[dict[str, Any]]] = {}
    arrays: dict[tuple[int, str], dict[str, np.ndarray]] = {}
    stored: dict[str, np.ndarray] = {}
    for sweeps in PROJECTION_SWEEPS:
        results[sweeps] = []
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
            result, law_arrays = execute_law(
                session,
                causal,
                prepared[law.name],
                law,
                offset,
                q_nominal,
                sweeps,
                model_integrator_id_fn=law_model_constraint_rhs_integrator_id,
                model_cone_id_fn=law_model_edge_cone_id,
            )
            results[sweeps].append(result)
            arrays[(sweeps, law.name)] = law_arrays
            for name, value in law_arrays.items():
                stored[f"sweeps{sweeps}_{law.name}_{name}"] = value

    rows: list[dict[str, Any]] = []
    for sweeps in PROJECTION_SWEEPS:
        double = sweeps * 2
        width = None
        fraction = None
        if double in results:
            width = grouped_width(
                np.concatenate(
                    [
                        arrays[(double, law.name)]["generalized_velocity_after"]
                        - arrays[(sweeps, law.name)]["generalized_velocity_after"]
                        for law in FRESH_CONTACT_LAWS
                    ],
                    axis=0,
                )
            )
            fraction = max(width[name] / WIDTH_GATES[name] for name in WIDTH_GATES)
        p99 = max(row["query_timing_ns"]["p99"] for row in results[sweeps])
        zero = all(row["zero_rust_allocation"] for row in results[sweeps])
        repeat = all(row["bitwise_repeat"] for row in results[sweeps])
        eligible = bool(
            fraction is not None
            and fraction <= REFINEMENT_FRACTION_GATE
            and p99 <= QUERY_DEADLINE_NS
            and zero
            and repeat
        )
        rows.append(
            {
                "projection_sweeps": sweeps,
                "double_sweep_refinement_width": width,
                "double_sweep_refinement_fraction_of_gate": fraction,
                "query_p99_ns": p99,
                "zero_rust_allocation": zero,
                "bitwise_repeat": repeat,
                "prediction_only_selection_eligible": eligible,
                "laws": results[sweeps],
            }
        )
    eligible = [row for row in rows if row["prediction_only_selection_eligible"]]
    selected = min(eligible, key=lambda row: row["projection_sweeps"]) if eligible else None
    selected_sweeps = selected["projection_sweeps"] if selected else None

    spent_scores: list[dict[str, Any]] = []
    if selected_sweeps is not None:
        with np.load(source) as spent:
            for index, (law, offset) in enumerate(zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)):
                score, scored = score_law(
                    session,
                    spent,
                    prepared[law.name],
                    law,
                    offset,
                    results[selected_sweeps][index],
                    arrays[(selected_sweeps, law.name)],
                )
                score["model_integrator_id"] = law_model_constraint_rhs_integrator_id(law)
                score["model_cone_id"] = law_model_edge_cone_id(law)
                spent_scores.append(score)
                stored.update({f"spent_{name}": value for name, value in scored.items()})

    frozen = selected_sweeps is not None
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "equation_driven_change": True,
        "pyramid_edge_basis": "N + mu*T1; N - mu*T1; N + mu*T2; N - mu*T2",
        "legacy_cartesian_pyramid_preserved": True,
        "model_integrator_ids": MODEL_INTEGRATOR_IDS,
        "completed_labels_used_for_selection": False,
        "completed_spent_labels_accessed_after_selection": bool(spent_scores),
        "new_mujoco_physics_or_integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "projection_sweep_candidates": PROJECTION_SWEEPS,
        "refinement_fraction_gate": REFINEMENT_FRACTION_GATE,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "profile_rows": rows,
        "selected_projection_sweeps": selected_sweeps,
        "profile_frozen_for_fresh_holdout": frozen,
        "spent_label_diagnostic": spent_scores,
        "authority_admitted": False,
    }
    table = []
    for row in rows:
        width = row["double_sweep_refinement_width"]
        table.append(
            [
                row["projection_sweeps"],
                f"{width['root_angular_rad_s']:.4f} / {width['root_linear_m_s']:.4f} / {width['joint_rad_s']:.4f}" if width else "—",
                f"{100.0 * row['double_sweep_refinement_fraction_of_gate']:.3f}%" if row["double_sweep_refinement_fraction_of_gate"] is not None else "—",
                f"{row['query_p99_ns'] / 1e6:.3f}",
                "FREEZE" if row is selected else "—",
            ]
        )
    spent_table = []
    for score in spent_scores:
        width = score["diagnostic_fitted_width"]
        spent_table.append(
            [score["law"], score["model_cone_id"], f"{100 * score['sample_coverage']:.3f}%", f"{score['exact_active_sets']}/{score['samples']}", f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}"]
        )
    report = "\n".join(
        [
            "# Bonesaw pyramid-edge convergence audit · r243",
            "",
            f"> Prediction-only profile **{'FROZEN' if frozen else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "Pyramidal contacts now have an opt-in model-only cone ABI 2 that solves nonnegative edge variables N±μTᵢ. Historical Cartesian diamond ABI 1 remains unchanged. Selection uses causal prediction refinement only.",
            "",
            *markdown_table(["sweeps", "2× width · ang / lin / joint", "% gate", "p99 ms", "decision"], table),
            "",
            f"The smallest stable profile is **{selected_sweeps} sweeps**." if frozen else "No bounded profile converged under the frozen gate.",
            "",
            "## Spent R241 diagnostic (ineligible for selection)",
            "",
            *markdown_table(["law", "cone ABI", "coverage", "exact active sets", "width · ang / lin / joint"], spent_table),
            "",
            "No new reference physics, policy, controller, selector, or plant step is used. A new untouched holdout is required before promotion.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-pyramid-edge-convergence-audit-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(output / "g1-pyramid-edge-convergence-audit.npz", **stored)
    (output / "G1_PYRAMID_EDGE_CONVERGENCE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw pyramid-edge convergence · r243"))
    print(json.dumps({"profile_frozen_for_fresh_holdout": frozen, "selected_projection_sweeps": selected_sweeps, "authority_admitted": False}, indent=2, sort_keys=True))
    return 0 if frozen else 1


if __name__ == "__main__":
    raise SystemExit(main())
