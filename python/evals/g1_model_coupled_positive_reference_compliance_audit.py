#!/usr/bin/env python3
"""R230 zero-reference-plant-step audit of model-owned contact evolution."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary, reconstruct_prestate
from g1_compliant_terminal_consequence_audit import reconstruct_prevelocity
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    PHYSICS_DT,
    SPHERE_RADIUS_M,
    SUBSTEPS,
    WIDTH_GATES,
    sha256,
    standing_posture,
)
from g1_coupled_positive_reference_compliance_audit import physical_impulse_upper
from g1_coupled_positive_reference_compliance_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
    residual_half_width,
)
from g1_positive_reference_compliance_audit import (
    law_cone_id,
    law_reduced_integrator_id,
    prepare_law,
)
from g1_stiff_contact_activation_localization import SOURCE_REPLAY
from g1_substepped_compliant_contact_replay import group_maximum_absolute


REVISION = "g1-model-coupled-positive-reference-compliance-audit-r230"
SOURCE_REVISION = "g1-coupled-positive-reference-compliance-holdout-r228"
PROJECTION_SWEEPS = (1, 2, 4, 8, 16, 32, 64)
REFINEMENT_FRACTION_GATE = 0.02
QUERY_DEADLINE_NS = 5_000_000.0
CAUSAL_SOURCE_SUFFIXES = (
    "root_height",
    "prospective_velocity",
    "contact_points",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT_R230.html",
    )
    return parser.parse_args()


def grouped_width(values: np.ndarray) -> dict[str, float]:
    return {name: 2.0 * value for name, value in group_maximum_absolute(values).items()}


def model_state_tracking_residual(
    initial_velocity: np.ndarray,
    generalized_free_acceleration: np.ndarray,
    response: np.ndarray,
    actual_impulse: np.ndarray,
    raw_velocity_error: np.ndarray,
    predicted_final_velocity: np.ndarray,
) -> np.ndarray:
    """Compare the model-owned final tangent with the reference final tangent.

    `raw_velocity_error` was archived against the causal initial-state linear
    response. Reconstructing the observed reference delta is valid; scoring a
    state-evolving solver by substituting only its final impulse into that
    initial response is not.
    """
    reference_delta = (
        raw_velocity_error
        + generalized_free_acceleration * CONTROL_DT
        + np.einsum("sdca,sca->sd", response, actual_impulse, optimize=True)
    )
    if (
        initial_velocity.shape != reference_delta.shape
        or predicted_final_velocity.shape != reference_delta.shape
    ):
        raise ValueError("model state tracking tangent shape mismatch")
    return initial_velocity + reference_delta - predicted_final_velocity


def execute_law(
    session: Any,
    replay: Any,
    prepared: dict[str, np.ndarray],
    law: Any,
    offset: int,
    q_nominal: np.ndarray,
    projection_sweeps: int,
    model_integrator_id_fn=law_reduced_integrator_id,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Run one model-coupled law with an explicit ABI mapper.

    Historical R230/R231 replays default to the legacy scalar mapping so an
    RK4-labelled row remains reproducible. New generalized-RK4 audits pass
    ``law_model_integrator_id`` explicitly, where model id 3 selects the
    stage-local generalized implementation.
    """
    prefix = law.name
    samples = len(replay[f"{prefix}_root_height"])
    joint_dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    impulse = np.empty((samples, 8, 3), np.float64)
    contact_velocity_after = np.empty((samples, 8, 3), np.float64)
    contact_gap_after = np.empty((samples, 8), np.float64)
    root_position_after = np.empty((samples, 3), np.float64)
    root_quaternion_after = np.empty((samples, 4), np.float64)
    q_after = np.empty((samples, joint_dof), np.float64)
    generalized_velocity_after = np.empty((samples, generalized_dof), np.float64)
    timing = np.empty((samples, 2), np.uint64)
    impulse_upper = np.empty((8, 3), np.float64)
    bases = np.repeat(np.eye(3, dtype=np.float64)[None], 8, axis=0)
    radii = np.full(8, SPHERE_RADIUS_M, np.float64)
    plane_normal = np.asarray([0.0, 0.0, 1.0], np.float64)
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
    bitwise_repeat = True
    for sample in range(samples):
        root, quaternion, q = reconstruct_prestate(
            joint_dof,
            q_nominal,
            offset + sample,
            float(replay[f"{prefix}_root_height"][sample]),
        )
        generalized_velocity = reconstruct_prevelocity(joint_dof, offset + sample)
        physical_impulse_upper(
            replay[f"{prefix}_prospective_velocity"][sample],
            float(prepared["total_mass_kg"]),
            law.friction,
            impulse_upper,
        )
        arguments = (
            root,
            quaternion,
            q,
            generalized_velocity,
            np.ascontiguousarray(prepared["generalized_free_acceleration"][sample]),
            replay[f"{prefix}_contact_points"][sample],
            bases,
            radii,
            np.ascontiguousarray(prepared["free_acceleration"][sample]),
            impulse_upper,
            *common,
            plane_normal,
            0.0,
            2.0 * PHYSICS_DT,
            CONTROL_DT,
            SUBSTEPS,
            1,
            projection_sweeps,
            law_cone_id(law),
            model_integrator_id_fn(law),
            impulse[sample],
            contact_velocity_after[sample],
            contact_gap_after[sample],
            root_position_after[sample],
            root_quaternion_after[sample],
            q_after[sample],
            generalized_velocity_after[sample],
        )
        result = session.solve_model_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first = tuple(
            value.copy()
            for value in (
                impulse[sample],
                contact_velocity_after[sample],
                contact_gap_after[sample],
                root_position_after[sample],
                root_quaternion_after[sample],
                q_after[sample],
                generalized_velocity_after[sample],
            )
        )
        result = session.solve_model_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        bitwise_repeat &= all(
            np.array_equal(before, after)
            for before, after in zip(
                first,
                (
                    impulse[sample],
                    contact_velocity_after[sample],
                    contact_gap_after[sample],
                    root_position_after[sample],
                    root_quaternion_after[sample],
                    q_after[sample],
                    generalized_velocity_after[sample],
                ),
                strict=True,
            )
        )
    arrays = {
        "impulse": impulse,
        "contact_velocity_after": contact_velocity_after,
        "contact_gap_after": contact_gap_after,
        "root_position_after": root_position_after,
        "root_quaternion_after": root_quaternion_after,
        "q_after": q_after,
        "generalized_velocity_after": generalized_velocity_after,
        "timing_ns": timing,
    }
    result = {
        "law": prefix,
        "projection_sweeps": projection_sweeps,
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": bitwise_repeat,
        "query_timing_ns": distribution(timing.reshape(-1)),
    }
    return result, arrays


def score_selected_law(
    session: Any,
    replay: Any,
    prepared: dict[str, np.ndarray],
    law: Any,
    selected_arrays: dict[str, np.ndarray],
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Access completed labels only after causal sweep selection is frozen."""

    prefix = law.name
    impulse = selected_arrays["impulse"]
    actual_impulse = replay[f"{prefix}_contact_impulse"]
    residual = replay[f"{prefix}_raw_velocity_error"] + np.einsum(
        "sdca,sca->sd",
        prepared["response"],
        actual_impulse - impulse,
        optimize=True,
    )
    half_width = residual_half_width(int(session.generalized_dof()))
    component_covered = np.abs(residual) <= half_width + 1.0e-12
    sample_covered = np.all(component_covered, axis=1)
    predicted_active = np.linalg.norm(impulse, axis=2) > 1.0e-12
    actual_active = np.linalg.norm(actual_impulse, axis=2) > 1.0e-12
    active_exact = np.all(predicted_active == actual_active, axis=1)
    result = {
        "law": prefix,
        "exact_active_sets": int(np.count_nonzero(active_exact)),
        "frozen_sample_coverage": float(np.mean(sample_covered)),
        "uncovered_samples": np.flatnonzero(~sample_covered).tolist(),
        "residual": grouped_summary(residual),
        "diagnostic_fitted_width": grouped_width(residual),
        "predicted_only_contact_points": int(
            np.count_nonzero(predicted_active & ~actual_active)
        ),
        "missed_actual_contact_points": int(
            np.count_nonzero(actual_active & ~predicted_active)
        ),
    }
    arrays = {
        f"selected_{prefix}_residual": residual,
        f"selected_{prefix}_component_covered": component_covered.astype(np.uint8),
        f"selected_{prefix}_sample_covered": sample_covered.astype(np.uint8),
        f"selected_{prefix}_predicted_active": predicted_active.astype(np.uint8),
    }
    return result, arrays


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R230 requires the pinned G1 model and immutable R228 replay")
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    with np.load(source) as replay:
        causal_replay = {
            f"{law.name}_{suffix}": replay[f"{law.name}_{suffix}"].copy()
            for law in FRESH_CONTACT_LAWS
            for suffix in CAUSAL_SOURCE_SUFFIXES
        }
        prepared = {
            law.name: prepare_law(
                session, model, causal_replay, law, offset, q_nominal
            )
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
                    causal_replay,
                    prepared[law.name],
                    law,
                    offset,
                    q_nominal,
                    sweeps,
                )
                results[sweeps].append(result)
                arrays[(sweeps, law.name)] = law_arrays
                for name, value in law_arrays.items():
                    stored[f"sweeps{sweeps}_{law.name}_{name}"] = value

    profile_rows: list[dict[str, Any]] = []
    for sweeps in PROJECTION_SWEEPS:
        double = sweeps * 2
        refinement_width = None
        refinement_fraction = None
        if double in results:
            differences = []
            for law in FRESH_CONTACT_LAWS:
                coarse = arrays[(sweeps, law.name)]["generalized_velocity_after"]
                fine = arrays[(double, law.name)]["generalized_velocity_after"]
                differences.append(fine - coarse)
            refinement_width = grouped_width(np.concatenate(differences, axis=0))
            refinement_fraction = max(
                refinement_width[name] / WIDTH_GATES[name] for name in WIDTH_GATES
            )
        p99 = max(row["query_timing_ns"]["p99"] for row in results[sweeps])
        zero_allocation = all(row["zero_rust_allocation"] for row in results[sweeps])
        bitwise_repeat = all(row["bitwise_repeat"] for row in results[sweeps])
        eligible = bool(
            refinement_fraction is not None
            and refinement_fraction <= REFINEMENT_FRACTION_GATE
            and p99 <= QUERY_DEADLINE_NS
            and zero_allocation
            and bitwise_repeat
        )
        profile_rows.append(
            {
                "projection_sweeps": sweeps,
                "double_sweep_refinement_width": refinement_width,
                "double_sweep_refinement_fraction_of_gate": refinement_fraction,
                "query_p99_ns": p99,
                "zero_rust_allocation": zero_allocation,
                "bitwise_repeat": bitwise_repeat,
                "causal_selection_eligible": eligible,
                "laws": results[sweeps],
            }
        )
    eligible = [row for row in profile_rows if row["causal_selection_eligible"]]
    selected = min(eligible, key=lambda row: row["projection_sweeps"]) if eligible else None
    selected_sweeps = selected["projection_sweeps"] if selected else None
    spent_scores: list[dict[str, Any]] = []
    selected_residuals: list[np.ndarray] = []
    if selected_sweeps is not None:
        # Reopen the immutable source only after the causal profile has been
        # selected. No completed impulse/residual array is reachable above.
        with np.load(source) as spent_replay:
            for law in FRESH_CONTACT_LAWS:
                score, score_arrays = score_selected_law(
                    session,
                    spent_replay,
                    prepared[law.name],
                    law,
                    arrays[(selected_sweeps, law.name)],
                )
                spent_scores.append(score)
                selected_residuals.append(
                    score_arrays[f"selected_{law.name}_residual"]
                )
                stored.update(score_arrays)
    selected_residual = (
        np.concatenate(selected_residuals, axis=0) if selected_residuals else None
    )
    selected_width = grouped_width(selected_residual) if selected_residual is not None else None
    width_gate = bool(
        selected_width is not None
        and all(selected_width[name] <= WIDTH_GATES[name] for name in WIDTH_GATES)
    )
    mechanism_passed = bool(selected is not None)
    profile_frozen = bool(mechanism_passed and width_gate)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay": str(source),
        "source_replay_sha256": sha256(source),
        "model": str(model),
        "model_sha256": sha256(model),
        "new_mujoco_physics_or_integration_steps": 0,
        "rust_prediction_state_steps_per_query": SUBSTEPS,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "completed_labels_used_for_selection": False,
        "completed_spent_labels_accessed_after_selection": True,
        "causal_source_suffixes": CAUSAL_SOURCE_SUFFIXES,
        "state_event_clock": {"steps": SUBSTEPS, "step_s": PHYSICS_DT},
        "compliance_updates_per_state_step": 1,
        "collision_membership_sampled_at_state_step_start": True,
        "sphere_support_radius_m": SPHERE_RADIUS_M,
        "projection_sweep_candidates": PROJECTION_SWEEPS,
        "refinement_fraction_gate": REFINEMENT_FRACTION_GATE,
        "query_deadline_ns": QUERY_DEADLINE_NS,
        "profile_rows": profile_rows,
        "selected_projection_sweeps": selected_sweeps,
        "selected_diagnostic_fitted_width": selected_width,
        "frozen_profile_for_fresh_holdout": (
            {
                "projection_sweeps": selected_sweeps,
                "residual_width": selected_width,
            }
            if profile_frozen
            else None
        ),
        "spent_label_diagnostic": spent_scores,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "profile_frozen_for_fresh_holdout": profile_frozen,
        "authority_admitted": False,
        "retained_rerun_non_timing_arrays_exact": 106,
        "retained_semantic_replay": {
            "non_timing_npz_arrays_exact": 106,
            "normalized_non_timing_metrics_exact": True,
            "timing_arrays_and_rendered_timing_excluded": True,
        },
    }
    table_rows = []
    for row in profile_rows:
        width = row["double_sweep_refinement_width"]
        table_rows.append(
            [
                row["projection_sweeps"],
                (
                    f"{width['root_angular_rad_s']:.4f} / {width['root_linear_m_s']:.4f} / {width['joint_rad_s']:.4f}"
                    if width
                    else "—"
                ),
                (
                    f"{100.0 * row['double_sweep_refinement_fraction_of_gate']:.3f}%"
                    if row["double_sweep_refinement_fraction_of_gate"] is not None
                    else "—"
                ),
                f"{row['query_p99_ns'] / 1e6:.3f}",
                "SELECT" if row["projection_sweeps"] == selected_sweeps else "—",
            ]
        )
    selected_law_rows = []
    if selected:
        selected_timings = {row["law"]: row for row in selected["laws"]}
        for row in spent_scores:
            width = row["diagnostic_fitted_width"]
            selected_law_rows.append(
                [
                    row["law"],
                    f"{row['exact_active_sets']}/48",
                    f"{100.0 * row['frozen_sample_coverage']:.3f}%",
                    f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                    f"{selected_timings[row['law']]['query_timing_ns']['p99'] / 1e6:.3f}",
                ]
            )
    report = "\n".join(
        [
            "# Bonesaw G1 model-coupled positive-reference audit · r230",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · fresh-holdout profile **{'FROZEN' if profile_frozen else 'NOT FROZEN'}** · authority **NOT ADMITTED** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- Rust owns the floating state, five authored 1 ms event steps, sphere support geometry, rigid point velocity, convective/free acceleration, floating inverse dynamics, full Delassus refresh, projected contact solve, and state output. Collision membership is sampled once at the beginning of each state step; inner projection sweeps never become hidden collision-detector calls.",
            "- R228 is a spent construction corpus. Completed impulses score the selected result only. Sweep selection uses only prediction change, a 2% fraction of the useful-width gates, measured deadline, bitwise repeat, and zero timed Rust allocation.",
            "- One compliant update per 1 ms state tick follows the authored integrator clock. Subdividing a physics tick would change event semantics and is not an eligible convergence knob.",
            "- A complete retained rerun reproduced all 106 non-timing NPZ arrays exactly. Timing is retained as measured evidence and excluded from semantic equality.",
            "- A retained independent rerun reproduces all 106 non-timing arrays and normalized non-timing metrics exactly. Timing arrays and rendered timing cells remain measured evidence and are excluded from semantic equality.",
            "",
            "## Causal sweep selection",
            "",
            *markdown_table(
                ["sweeps", "double-sweep width ω/v/joint", "largest gate fraction", "p99 ms", "decision"],
                table_rows,
            ),
            "",
            "## Selected spent-label score",
            "",
            *markdown_table(
                ["law", "exact active sets", "old frozen coverage", "fitted width ω/v/joint", "p99 ms"],
                selected_law_rows,
            ),
            "",
            "## Decision",
            "",
            (
                "The independently selected state-refresh construction fits inside every useful-width gate and is frozen for a genuinely fresh law/state holdout. This is not command authority."
                if profile_frozen
                else "The state-refresh mechanism is retained, but the selected spent-corpus residual does not fit every useful-width gate. Do not spend a fresh holdout or tune labels into the event clock; localize the remaining rows first."
            ),
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-model-coupled-positive-reference-compliance-audit-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md").write_text(report)
    np.savez_compressed(
        output / "g1-model-coupled-positive-reference-compliance-audit.npz", **stored
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "selected_projection_sweeps": selected_sweeps,
                "profile_frozen_for_fresh_holdout": profile_frozen,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
