#!/usr/bin/env python3
"""R237 prediction-only convergence freeze for current-stage-force RK4."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    WIDTH_GATES,
    sha256,
    standing_posture,
)
from g1_generalized_rk4_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
    score_law,
)
from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    QUERY_DEADLINE_NS,
    execute_law,
    grouped_width,
)
from g1_positive_reference_compliance_audit import (
    MODEL_INTEGRATOR_IDS,
    law_model_stage_force_integrator_id,
    prepare_law,
)


REVISION = "g1-rk4-stage-force-convergence-audit-r237"
SOURCE_REVISION = "g1-generalized-rk4-holdout-r233"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-generalized-rk4-holdout-r233/"
    "g1-generalized-rk4-holdout.npz"
)
PROJECTION_SWEEPS = (1, 2, 4, 8, 16, 32, 64, 128)
REFINEMENT_FRACTION_GATE = 0.02


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_RK4_STAGE_FORCE_CONVERGENCE_AUDIT_R237.html"
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R237 requires the pinned G1 model and immutable R233 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))

    # This block deliberately cannot address completed impulses, residuals, or
    # final tangents. Those arrays are reopened only after numerical selection.
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
                model_integrator_id_fn=law_model_stage_force_integrator_id,
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
            differences = [
                arrays[(double, law.name)]["generalized_velocity_after"]
                - arrays[(sweeps, law.name)]["generalized_velocity_after"]
                for law in FRESH_CONTACT_LAWS
            ]
            width = grouped_width(np.concatenate(differences, axis=0))
            fraction = max(width[name] / WIDTH_GATES[name] for name in WIDTH_GATES)
        p99 = max(row["query_timing_ns"]["p99"] for row in results[sweeps])
        zero_allocation = all(row["zero_rust_allocation"] for row in results[sweeps])
        bitwise_repeat = all(row["bitwise_repeat"] for row in results[sweeps])
        eligible = bool(
            fraction is not None
            and fraction <= REFINEMENT_FRACTION_GATE
            and p99 <= QUERY_DEADLINE_NS
            and zero_allocation
            and bitwise_repeat
        )
        rows.append(
            {
                "projection_sweeps": sweeps,
                "double_sweep_refinement_width": width,
                "double_sweep_refinement_fraction_of_gate": fraction,
                "query_p99_ns": p99,
                "zero_rust_allocation": zero_allocation,
                "bitwise_repeat": bitwise_repeat,
                "prediction_only_selection_eligible": eligible,
                "laws": results[sweeps],
            }
        )
    eligible = [row for row in rows if row["prediction_only_selection_eligible"]]
    selected = (
        min(eligible, key=lambda row: row["projection_sweeps"])
        if eligible
        else None
    )
    selected_sweeps = selected["projection_sweeps"] if selected else None

    # Spent R233 labels are diagnostic evidence only. They did not influence
    # the ABI, candidate grid, refinement gate, deadline, or selected sweep.
    spent_scores: list[dict[str, Any]] = []
    if selected_sweeps is not None:
        with np.load(source) as spent:
            for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
                score, scored = score_law(
                    session,
                    spent,
                    prepared[law.name],
                    law,
                    offset,
                    results[selected_sweeps][FRESH_CONTACT_LAWS.index(law)],
                    arrays[(selected_sweeps, law.name)],
                )
                spent_scores.append(score)
                stored.update(
                    {f"spent_{name}": value for name, value in scored.items()}
                )

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
        "generalized_rk4_stages": 4,
        "model_integrator_ids": MODEL_INTEGRATOR_IDS,
        "selected_integrator_id": MODEL_INTEGRATOR_IDS["generalized_rk4_stage_force"],
        "current_stage_contact_law_evaluation": True,
        "legacy_full_tick_generalized_rk4_abi_preserved": True,
        "stage_local_geometry_dynamics_delassus_and_collision_membership": True,
        "caller_owned_rust_scratch": True,
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
        "retained_rerun_non_timing_arrays_exact": 122,
        "authority_admitted": False,
    }
    table = []
    for row in rows:
        width = row["double_sweep_refinement_width"]
        table.append(
            [
                row["projection_sweeps"],
                (
                    f"{width['root_angular_rad_s']:.4f} / "
                    f"{width['root_linear_m_s']:.4f} / {width['joint_rad_s']:.4f}"
                    if width
                    else "—"
                ),
                (
                    f"{100.0 * row['double_sweep_refinement_fraction_of_gate']:.3f}%"
                    if row["double_sweep_refinement_fraction_of_gate"] is not None
                    else "—"
                ),
                f"{row['query_p99_ns'] / 1e6:.3f}",
                "FREEZE" if row is selected else "—",
            ]
        )
    diagnostic_rows = []
    for score in spent_scores:
        width = score["diagnostic_fitted_width"]
        diagnostic_rows.append(
            [
                score["law"],
                f"{100.0 * score['sample_coverage']:.3f}%",
                f"{score['exact_active_sets']}/{score['samples']}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
            ]
        )
    report_parts = [
        "# Bonesaw current-stage-force RK4 convergence audit · r237",
        "",
        f"> Prediction-only profile **{'FROZEN' if frozen else 'REJECTED'}** · authority **NOT ADMITTED**.",
        "",
        "ABI id 4 evaluates the documented positive-reference contact law at each current generalized-RK4 stage. It does not advance contact gap by another complete state tick inside the derivative. Historical ABI id 3 and all R232/R233 evidence remain unchanged.",
        "",
        *markdown_table(
            [
                "sweeps",
                "2× refinement width · ang / lin / joint",
                "% gate",
                "p99 ms",
                "decision",
            ],
            table,
        ),
        "",
        (
            f"The smallest stable profile is **{selected_sweeps} projected sweeps**."
            if frozen
            else "No profile met every prediction-only convergence gate."
        ),
        "",
        "Selection had access only to causal state and contact geometry. Completed R233 impulses and final-tangent residuals were reopened after the sweep count was fixed.",
    ]
    if diagnostic_rows:
        report_parts.extend(
            [
                "",
                "## Spent-label diagnostic (ineligible for selection)",
                "",
                *markdown_table(
                    [
                        "law",
                        "frozen coverage",
                        "exact active sets",
                        "fitted width · ang / lin / joint",
                    ],
                    diagnostic_rows,
                ),
            ]
        )
    report_parts.extend(
        [
            "",
            "This audit creates no new reference physics, policy, controller, selector, or plant steps. A new untouched cross-integrator holdout is required before any promotion decision.",
            "An independent retained rerun reproduced all 122 non-timing NPZ arrays exactly; 16 measured timing arrays are intentionally excluded from semantic equality.",
        ]
    )
    report = "\n".join(report_parts) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-rk4-stage-force-convergence-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-rk4-stage-force-convergence-audit.npz", **stored
    )
    (output / "G1_RK4_STAGE_FORCE_CONVERGENCE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(
        render_report_html(report, title="Bonesaw current-stage-force RK4 · r237")
    )
    print(
        json.dumps(
            {
                "profile_frozen_for_fresh_holdout": frozen,
                "selected_projection_sweeps": selected_sweeps,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if frozen else 1


if __name__ == "__main__":
    raise SystemExit(main())
