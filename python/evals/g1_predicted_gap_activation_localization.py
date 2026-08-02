#!/usr/bin/env python3
"""R242 spent-label diagnostic for the opt-in within-tick activation ABI."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from g1_constraint_rhs_cross_integrator_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256, standing_posture
from g1_generalized_rk4_holdout import score_law
from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    QUERY_DEADLINE_NS,
    execute_law,
)
from g1_positive_reference_compliance_audit import (
    law_model_predicted_gap_activation_integrator_id,
    prepare_law,
)


REVISION = "g1-predicted-gap-activation-localization-r242"
SOURCE_REVISION = "g1-constraint-rhs-cross-integrator-holdout-r241"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-constraint-rhs-cross-integrator-holdout-r241/"
    "g1-constraint-rhs-cross-integrator-holdout.npz"
)
PROJECTION_SWEEPS = 128


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_PREDICTED_GAP_ACTIVATION_LOCALIZATION_R242.html"
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R242 requires the pinned G1 model and immutable R241 replay")
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
    rows = []
    stored: dict[str, np.ndarray] = {}
    with np.load(source) as replay:
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True):
            result, arrays = execute_law(
                session,
                causal,
                prepared[law.name],
                law,
                offset,
                q_nominal,
                PROJECTION_SWEEPS,
                model_integrator_id_fn=law_model_predicted_gap_activation_integrator_id,
            )
            score, scored = score_law(
                session,
                replay,
                prepared[law.name],
                law,
                offset,
                result,
                arrays,
            )
            score["model_integrator_id"] = law_model_predicted_gap_activation_integrator_id(law)
            rows.append(score)
            stored.update({f"{law.name}_{name}": value for name, value in arrays.items()})
            stored.update({f"{law.name}_scored_{name}": value for name, value in scored.items()})

    mechanism_passed = all(
        row["zero_rust_allocation"]
        and row["bitwise_repeat"]
        and row["deadline_passed"]
        for row in rows
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "completed_labels_used_for_selection": False,
        "new_mujoco_physics_or_integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "state_steps": 5,
        "state_step_s": 0.001,
        "projection_sweeps": PROJECTION_SWEEPS,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "activation_criterion": "predicted one-state-step gap < 0 and current normal velocity < 0",
        "mechanism_passed": mechanism_passed,
        "authority_admitted": False,
        "results": rows,
    }
    table = []
    for row in rows:
        width = row["diagnostic_fitted_width"]
        table.append(
            [
                row["law"],
                row["model_integrator_id"],
                f"{100.0 * row['sample_coverage']:.3f}%",
                f"{row['exact_active_sets']}/{row['samples']}",
                f"{row['predicted_only_contact_points']} / {row['missed_actual_contact_points']}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{row['query_timing_ns']['p99'] / 1e6:.3f}",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw predicted-gap activation localization · r242",
            "",
            "> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE**.",
            "",
            "This diagnostic removes the historical non-RK boundary-only activation guard only for opt-in ABI 6. A positive-gap point is admitted when its one-state-step causal prediction crosses the plane while its current normal velocity is closing. Historical ABI 0 and the proven RK4 ABI 4 are unchanged.",
            "",
            *markdown_table(
                [
                    "law",
                    "ABI",
                    "coverage",
                    "exact active sets",
                    "pred-only / missed",
                    "fitted width · ang / lin / joint",
                    "p99 ms",
                ],
                table,
            ),
            "",
            "ABI 6 is rejected for promotion: the implicitfast row predicts extra unloaded points and widens the joint residual tail. No new MuJoCo, policy, controller, selector, or plant steps were used; the R241 labels were opened only after the causal diagnostic completed.",
            "An independent retained rerun is required before any future construction uses this path.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-predicted-gap-activation-localization-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-predicted-gap-activation-localization.npz", **stored
    )
    (output / "G1_PREDICTED_GAP_ACTIVATION_LOCALIZATION.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw predicted-gap activation · r242"))
    print(
        json.dumps(
            {"mechanism_passed": mechanism_passed, "authority_admitted": False},
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
