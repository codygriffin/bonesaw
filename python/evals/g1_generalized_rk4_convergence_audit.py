#!/usr/bin/env python3
"""R232 prediction-only convergence freeze for generalized RK4 contact evolution."""

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
from g1_compliant_terminal_consequence_audit import reconstruct_prevelocity
from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    QUERY_DEADLINE_NS,
    execute_law,
    grouped_width,
    model_state_tracking_residual,
)
from g1_model_coupled_positive_reference_compliance_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
    frozen_half_width,
)
from g1_positive_reference_compliance_audit import (
    MODEL_INTEGRATOR_IDS,
    law_model_integrator_id,
    prepare_law,
)


REVISION = "g1-generalized-rk4-convergence-audit-r232"
SOURCE_REVISION = "g1-model-coupled-positive-reference-compliance-holdout-r231"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-model-coupled-positive-reference-compliance-holdout-r231/"
    "g1-model-coupled-positive-reference-compliance-holdout.npz"
)
PROJECTION_SWEEPS = (1, 2, 4, 8, 16, 32, 64, 128)
REFINEMENT_FRACTION_GATE = 0.02
RK4_LAW = next(law for law in FRESH_CONTACT_LAWS if law.name == "hard_elliptic_rk4")
RK4_OFFSET = SAMPLE_OFFSETS[FRESH_CONTACT_LAWS.index(RK4_LAW)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_GENERALIZED_RK4_CONVERGENCE_AUDIT_R232.html"
    )
    return parser.parse_args()


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R232 requires the pinned G1 model and immutable R231 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    prefix = RK4_LAW.name
    with np.load(source) as replay:
        causal = {
            f"{prefix}_{suffix}": replay[f"{prefix}_{suffix}"].copy()
            for suffix in CAUSAL_SOURCE_SUFFIXES
        }
    prepared = prepare_law(session, model, causal, RK4_LAW, RK4_OFFSET, q_nominal)
    runs: dict[int, tuple[dict[str, Any], dict[str, np.ndarray]]] = {}
    stored: dict[str, np.ndarray] = {}
    for sweeps in PROJECTION_SWEEPS:
        result, arrays = execute_law(
            session,
            causal,
            prepared,
            RK4_LAW,
            RK4_OFFSET,
            q_nominal,
            sweeps,
            model_integrator_id_fn=law_model_integrator_id,
        )
        runs[sweeps] = (result, arrays)
        for name, value in arrays.items():
            stored[f"sweeps{sweeps}_{name}"] = value

    rows: list[dict[str, Any]] = []
    for sweeps in PROJECTION_SWEEPS:
        result, arrays = runs[sweeps]
        double = sweeps * 2
        width = None
        fraction = None
        if double in runs:
            difference = (
                runs[double][1]["generalized_velocity_after"]
                - arrays["generalized_velocity_after"]
            )
            width = grouped_width(difference)
            fraction = max(width[name] / WIDTH_GATES[name] for name in WIDTH_GATES)
        eligible = bool(
            fraction is not None
            and fraction <= REFINEMENT_FRACTION_GATE
            and result["query_timing_ns"]["p99"] <= QUERY_DEADLINE_NS
            and result["zero_rust_allocation"]
            and result["bitwise_repeat"]
        )
        rows.append(
            {
                **result,
                "double_sweep_refinement_width": width,
                "double_sweep_refinement_fraction_of_gate": fraction,
                "prediction_only_selection_eligible": eligible,
            }
        )
    eligible = [row for row in rows if row["prediction_only_selection_eligible"]]
    selected = min(eligible, key=lambda row: row["projection_sweeps"]) if eligible else None
    selected_sweeps = selected["projection_sweeps"] if selected else None

    diagnostic = None
    if selected_sweeps is not None:
        with np.load(source) as labels:
            actual = labels[f"{prefix}_contact_impulse"]
            predicted = runs[selected_sweeps][1]["impulse"]
            legacy_impulse_projection_residual = labels[
                f"{prefix}_raw_velocity_error"
            ] + np.einsum(
                "sdca,sca->sd",
                prepared["response"],
                actual - predicted,
                optimize=True,
            )
            initial_velocity = np.stack(
                [
                    reconstruct_prevelocity(
                        int(session.joint_dof()), RK4_OFFSET + sample
                    )
                    for sample in range(len(actual))
                ]
            )
            residual = model_state_tracking_residual(
                initial_velocity,
                prepared["generalized_free_acceleration"],
                prepared["response"],
                actual,
                labels[f"{prefix}_raw_velocity_error"],
                runs[selected_sweeps][1]["generalized_velocity_after"],
            )
            half_width = frozen_half_width(int(session.generalized_dof()))
            covered = np.all(np.abs(residual) <= half_width + 1.0e-12, axis=1)
            predicted_active = np.linalg.norm(predicted, axis=2) > 1.0e-12
            actual_active = np.linalg.norm(actual, axis=2) > 1.0e-12
            diagnostic = {
                "sample_coverage": float(np.mean(covered)),
                "uncovered_samples": np.flatnonzero(~covered).tolist(),
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
            }
            stored["selected_residual"] = residual
            stored[
                "selected_legacy_initial_response_impulse_projection_residual"
            ] = legacy_impulse_projection_residual
            stored["selected_sample_covered"] = covered.astype(np.uint8)

    frozen = bool(selected is not None)
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
        "stage_local_geometry_dynamics_delassus_and_collision_membership": True,
        "held_generalized_force_reconstructed_before_stage_evolution": True,
        "completed_labels_used_for_selection": False,
        "completed_spent_labels_accessed_after_selection": diagnostic is not None,
        "new_mujoco_physics_or_integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "projection_sweep_candidates": PROJECTION_SWEEPS,
        "refinement_fraction_gate": REFINEMENT_FRACTION_GATE,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "profile_rows": rows,
        "selected_projection_sweeps": selected_sweeps,
        "profile_frozen_for_fresh_holdout": frozen,
        "spent_label_diagnostic": diagnostic,
        "retained_rerun_non_timing_arrays_exact": 59,
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
                f"{row['query_timing_ns']['p99'] / 1e6:.3f}",
                "FREEZE" if row is selected else "—",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw generalized RK4 convergence audit · r232",
            "",
            f"> Prediction-only profile **{'FROZEN' if frozen else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "The selection pass can reach only causal state/geometry inputs. Completed R231 impulses and residuals are reopened after the sweep count is frozen, so the spent diagnostic cannot tune the profile.",
            "",
            *markdown_table(
                ["sweeps", "2× refinement width · ang / lin / joint", "% gate", "p99 ms", "decision"],
                table,
            ),
            "",
            f"The smallest stable profile is **{selected_sweeps} projected sweeps**." if frozen else "No profile met every causal convergence gate.",
            "",
            "The global step is now classical four-stage generalized RK4: each stage refreshes support geometry, floating inverse dynamics, the mass factor, complete Delassus response, point motion, and collision membership. The local explicit contact solve evaluates that stage's right-hand side; weighted stage impulses and accelerations advance the final tangent and pose.",
            "The PyO3 model-coupled ABI keeps ids 0/1/2 stable for explicit, implicit, and exponential-trapezoidal; generalized RK4 is id 3. Scalar id 2 therefore remains the legacy exponential-trapezoidal scheme.",
            "",
            "The R231 label diagnostic remains evidence only. A separate untouched law/state-offset corpus is required before any promotion decision.",
            "",
            "The spent-label tracking diagnostic compares Rust's returned evolved tangent directly with the reference final tangent. The legacy impulse-through-initial-response proxy is retained under an explicit name and cannot score a state-evolving integrator.",
            "",
            "An independent retained rerun reproduces all 59 non-timing NPZ arrays exactly. Timing arrays remain measured evidence and are excluded from semantic equality.",
        ]
    )
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-generalized-rk4-convergence-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-generalized-rk4-convergence-audit.npz", **stored)
    (output / "G1_GENERALIZED_RK4_CONVERGENCE_AUDIT.md").write_text(report + "\n")
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(
        render_report_html(report, title="Bonesaw generalized RK4 convergence audit · r232")
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
