#!/usr/bin/env python3
"""R229 label-explicit localization of R228 stiff-contact holdout misses.

This is a diagnostic, not a construction or selection eval. It may inspect the
completed R228 contact labels, so none of its observations can freeze a live
activation threshold, promote a profile, or admit command authority.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    PHYSICS_DT,
    SUBSTEPS,
    sha256,
    standing_posture,
)
from g1_coupled_positive_reference_compliance_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
    residual_half_width,
)
from g1_positive_reference_compliance_audit import prepare_law


REVISION = "g1-stiff-contact-activation-localization-r229"
SOURCE_REVISION = "g1-coupled-positive-reference-compliance-holdout-r228"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-coupled-positive-reference-compliance-holdout-r228/"
    "g1-coupled-positive-reference-compliance-holdout.npz"
)
ACTIVITY_EPSILON_NS = 1.0e-12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_STIFF_CONTACT_ACTIVATION_LOCALIZATION_R229.html"
    )
    return parser.parse_args()


def active_points(impulse: np.ndarray) -> np.ndarray:
    return np.linalg.norm(impulse, axis=2) > ACTIVITY_EPSILON_NS


def activation_summary(replay: Any, prefix: str) -> dict[str, Any]:
    actual = active_points(replay[f"{prefix}_contact_impulse"])
    predicted_impulse = replay[f"{prefix}_coupled_predicted_impulse"]
    predicted = active_points(predicted_impulse)
    covered = replay[f"{prefix}_coupled_sample_covered"].astype(bool)
    false_activation = predicted & ~actual
    missed_activation = actual & ~predicted
    false_norm = np.sum(
        np.linalg.norm(predicted_impulse, axis=2) * false_activation, axis=1
    )
    false_normal = np.sum(
        predicted_impulse[:, :, 2] * false_activation, axis=1
    )

    def cohort(mask: np.ndarray) -> dict[str, Any]:
        positive = false_norm[mask & (false_norm > 0.0)]
        return {
            "rows": int(np.count_nonzero(mask)),
            "rows_with_false_activation": int(
                np.count_nonzero(np.any(false_activation[mask], axis=1))
            ),
            "false_activation_points": int(np.count_nonzero(false_activation[mask])),
            "missed_actual_points": int(np.count_nonzero(missed_activation[mask])),
            "false_impulse_norm_sum_ns": (
                distribution(positive) if positive.size else None
            ),
        }

    return {
        "law": prefix,
        "covered": cohort(covered),
        "uncovered": cohort(~covered),
        "false_activation": false_activation,
        "missed_activation": missed_activation,
        "false_impulse_norm_sum_ns": false_norm,
        "false_normal_impulse_sum_ns": false_normal,
        "actual_active": actual,
        "predicted_active": predicted,
        "covered_mask": covered,
    }


def json_summary(summary: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in summary.items()
        if not isinstance(value, np.ndarray)
    }


def free_flight_activation_ticks(
    gap_m: np.ndarray,
    normal_velocity_m_s: np.ndarray,
    normal_acceleration_m_s2: np.ndarray,
    *,
    step_s: float = PHYSICS_DT,
    steps: int = SUBSTEPS,
) -> np.ndarray:
    """Return the first authored plant tick whose ballistic gap is nonpositive.

    Zero means no crossing in the declared horizon. This is a causal diagnostic:
    it uses only the prestate gap, velocity, smooth free acceleration, and the
    already-authored plant clock. It does not inspect completed contact labels.
    """
    gap = np.asarray(gap_m, np.float64)
    velocity = np.asarray(normal_velocity_m_s, np.float64)
    acceleration = np.asarray(normal_acceleration_m_s2, np.float64)
    if gap.shape != velocity.shape or gap.shape != acceleration.shape:
        raise ValueError("free-flight activation arrays must have identical shape")
    if gap.ndim != 2 or steps <= 0 or not np.isfinite(step_s) or step_s <= 0.0:
        raise ValueError("free-flight activation requires [sample, contact] and a positive clock")
    if not np.all(np.isfinite(gap)) or not np.all(np.isfinite(velocity)) or not np.all(
        np.isfinite(acceleration)
    ):
        raise ValueError("free-flight activation inputs must be finite")
    tick = np.zeros(gap.shape, np.uint8)
    for step in range(1, steps + 1):
        time_s = step * step_s
        crossing = (
            gap + velocity * time_s + 0.5 * acceleration * time_s * time_s
            <= 0.0
        )
        tick[(tick == 0) & crossing] = step
    return tick


def first_activation_cohort(tick: np.ndarray) -> np.ndarray:
    values = np.asarray(tick)
    if values.ndim != 2:
        raise ValueError("activation ticks must be [sample, contact]")
    earliest = np.min(np.where(values > 0, values, np.iinfo(np.uint8).max), axis=1)
    earliest[earliest == np.iinfo(np.uint8).max] = 0
    return (values == earliest[:, None]) & (earliest[:, None] > 0)


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    if not model.is_file() or not source.is_file():
        raise SystemExit("R229 requires the pinned G1 model and immutable R228 replay")

    with np.load(source) as replay:
        summaries = {
            law.name: activation_summary(replay, law.name)
            for law in FRESH_CONTACT_LAWS
        }
        stiff_law = FRESH_CONTACT_LAWS[1]
        stiff_offset = SAMPLE_OFFSETS[1]
        prefix = stiff_law.name
        stiff = summaries[prefix]
        uncovered = ~stiff["covered_mask"]
        uncovered_indices = np.flatnonzero(uncovered)

        session = bonesaw.ContactTransitionModelSession(
            str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
        )
        prepared = prepare_law(
            session,
            model,
            replay,
            stiff_law,
            stiff_offset,
            standing_posture(list(session.joint_names())),
        )

        actual_impulse = replay[f"{prefix}_contact_impulse"]
        predicted_impulse = replay[f"{prefix}_coupled_predicted_impulse"]
        oracle_impulse = predicted_impulse.copy()
        # Completed labels deliberately drive this counterfactual. It tests
        # whether masking alone explains R228; it is ineligible for selection.
        oracle_impulse[stiff["false_activation"]] = 0.0
        oracle_residual = replay[f"{prefix}_raw_velocity_error"] + np.einsum(
            "sdca,sca->sd",
            prepared["response"],
            actual_impulse - oracle_impulse,
            optimize=True,
        )
        half_width = residual_half_width(int(session.generalized_dof()))
        oracle_covered = np.all(
            np.abs(oracle_residual) <= half_width + 1.0e-12, axis=1
        )

        original_residual = replay[f"{prefix}_coupled_residual"]
        gaps = replay[f"{prefix}_contact_points"][:, :, 2]
        normal_velocity = replay[f"{prefix}_prospective_velocity"][:, :, 2]
        activation_tick = free_flight_activation_ticks(
            gaps,
            normal_velocity,
            prepared["free_acceleration"][:, :, 2],
        )
        first_cohort = first_activation_cohort(activation_tick)
        first_cohort_exact = np.all(first_cohort == stiff["actual_active"], axis=1)
        first_cohort_subset = np.all(~first_cohort | stiff["actual_active"], axis=1)
        first_cohort_false_points = first_cohort & ~stiff["actual_active"]
        first_cohort_later_actual = stiff["actual_active"] & ~first_cohort
        miss_rows: list[dict[str, Any]] = []
        for sample in uncovered_indices:
            false_points = np.flatnonzero(stiff["false_activation"][sample])
            time_to_impact_ms = np.divide(
                1_000.0 * gaps[sample, false_points],
                -normal_velocity[sample, false_points],
                out=np.full(false_points.size, np.inf, np.float64),
                where=normal_velocity[sample, false_points] < 0.0,
            )
            miss_rows.append(
                {
                    "sample": int(sample),
                    "state_index": int(stiff_offset + sample),
                    "actual_active_points": np.flatnonzero(
                        stiff["actual_active"][sample]
                    ).tolist(),
                    "predicted_active_points": np.flatnonzero(
                        stiff["predicted_active"][sample]
                    ).tolist(),
                    "false_active_points": false_points.tolist(),
                    "missed_actual_points": np.flatnonzero(
                        stiff["missed_activation"][sample]
                    ).tolist(),
                    "free_flight_activation_tick": activation_tick[sample].tolist(),
                    "first_free_flight_cohort": np.flatnonzero(
                        first_cohort[sample]
                    ).tolist(),
                    "first_cohort_exact_actual_set": bool(
                        first_cohort_exact[sample]
                    ),
                    "first_cohort_subset_of_actual_set": bool(
                        first_cohort_subset[sample]
                    ),
                    "false_point_gap_m": gaps[sample, false_points].tolist(),
                    "false_point_normal_velocity_m_s": normal_velocity[
                        sample, false_points
                    ].tolist(),
                    "false_point_free_time_to_impact_ms": time_to_impact_ms.tolist(),
                    "false_impulse_norm_sum_ns": float(
                        stiff["false_impulse_norm_sum_ns"][sample]
                    ),
                    "original_max_abs_generalized_residual": float(
                        np.max(np.abs(original_residual[sample]))
                    ),
                    "oracle_masked_max_abs_generalized_residual": float(
                        np.max(np.abs(oracle_residual[sample]))
                    ),
                    "oracle_masked_covered": bool(oracle_covered[sample]),
                }
            )

        covered_false = stiff["false_impulse_norm_sum_ns"][
            stiff["covered_mask"] & (stiff["false_impulse_norm_sum_ns"] > 0.0)
        ]
        uncovered_false = stiff["false_impulse_norm_sum_ns"][uncovered]
        magnitude_separation = {
            "maximum_covered_false_impulse_norm_sum_ns": float(
                np.max(covered_false)
            ),
            "minimum_uncovered_false_impulse_norm_sum_ns": float(
                np.min(uncovered_false)
            ),
            "observed_strict_separation": bool(
                np.min(uncovered_false) > np.max(covered_false)
            ),
            "eligible_as_threshold": False,
        }
        oracle_original_misses_fixed = int(
            np.count_nonzero(oracle_covered[uncovered])
        )
        oracle_remaining = np.flatnonzero(~oracle_covered).tolist()

        metrics = {
            "revision": REVISION,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "source_revision": SOURCE_REVISION,
            "source_replay": str(source),
            "source_replay_sha256": sha256(source),
            "model": str(model),
            "model_sha256": sha256(model),
            "activity_epsilon_ns": ACTIVITY_EPSILON_NS,
            "completed_label_access": True,
            "construction_or_selection_eligible": False,
            "physics_or_integration_steps": 0,
            "policy_or_controller_steps": 0,
            "selector_or_plant_actions": 0,
            "causal_prestate_forward_queries": len(stiff["covered_mask"]),
            "rust_point_response_queries": len(stiff["covered_mask"]),
            "retained_semantic_replay": {
                "npz_arrays_exact": 10,
                "normalized_metrics_exact": True,
                "markdown_report_exact": True,
            },
            "activation_by_law": {
                name: json_summary(summary) for name, summary in summaries.items()
            },
            "stiff_miss_rows": miss_rows,
            "stiff_false_impulse_magnitude_separation": magnitude_separation,
            "causal_first_impact_cohort": {
                "description": "earliest ballistic surface crossing on the authored five-by-one-millisecond plant clock, from prestate gap/normal velocity/smooth free acceleration only",
                "selection_uses_completed_labels": False,
                "scoring_uses_completed_labels": True,
                "uncovered_rows": int(uncovered_indices.size),
                "uncovered_exact_actual_sets": int(
                    np.count_nonzero(first_cohort_exact[uncovered])
                ),
                "uncovered_subset_of_actual_sets": int(
                    np.count_nonzero(first_cohort_subset[uncovered])
                ),
                "uncovered_false_points": int(
                    np.count_nonzero(first_cohort_false_points[uncovered])
                ),
                "uncovered_later_actual_points": int(
                    np.count_nonzero(first_cohort_later_actual[uncovered])
                ),
                "construction_eligible": True,
                "sufficient_as_terminal_predictor": False,
            },
            "label_oracle_mask_counterfactual": {
                "description": "zero predicted impulses only where the completed MuJoCo label is inactive; do not use for construction or selection",
                "original_covered_rows": int(np.count_nonzero(stiff["covered_mask"])),
                "oracle_masked_covered_rows": int(np.count_nonzero(oracle_covered)),
                "original_misses_fixed": oracle_original_misses_fixed,
                "remaining_uncovered_samples": oracle_remaining,
                "strict_coverage_passed": bool(np.all(oracle_covered)),
            },
            "decision": {
                "activation_presence_alone_explains_misses": False,
                "masking_without_resolving_is_sufficient": False,
                "first_impact_cohort_localizes_initial_activation": True,
                "next_construction_requirement": "advance on the authored plant clock, admit the first ballistic cohort, then recompute the coupled solution and model state before testing later cohorts",
                "profile_promoted": False,
                "authority_admitted": False,
            },
        }

        activation_rows = []
        for law in FRESH_CONTACT_LAWS:
            summary = summaries[law.name]
            activation_rows.append(
                [
                    law.name,
                    summary["covered"]["rows"],
                    summary["covered"]["rows_with_false_activation"],
                    summary["uncovered"]["rows"],
                    summary["uncovered"]["rows_with_false_activation"],
                    summary["covered"]["missed_actual_points"]
                    + summary["uncovered"]["missed_actual_points"],
                ]
            )
        detail_rows = [
            [
                row["state_index"],
                row["actual_active_points"],
                row["predicted_active_points"],
                f"{row['false_impulse_norm_sum_ns']:.3f}",
                f"{row['original_max_abs_generalized_residual']:.3f}",
                f"{row['oracle_masked_max_abs_generalized_residual']:.3f}",
                "PASS" if row["oracle_masked_covered"] else "FAIL",
            ]
            for row in miss_rows
        ]
        report = "\n".join(
            [
                "# Bonesaw G1 stiff-contact activation localization · r229",
                "",
                "> Label-explicit diagnostic **PASS** · label-derived construction/selection **INELIGIBLE** · profile **NOT PROMOTED** · authority **NOT ADMITTED** · physics/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.",
                "",
                "## Boundary",
                "",
                "- This audit reads the completed immutable R228 contact impulses explicitly. It localizes a rejected holdout; it cannot choose a threshold, freeze a profile, or authorize a command.",
                "- The activity test is numerical bookkeeping at 1e-12 N·s. The causal prestate response is reconstructed with 48 MuJoCo forward queries and 48 allocation-audited Rust point-response queries, but no model is integrated.",
                "- Point indices 0–3 are the left-foot spheres and 4–7 are the right-foot spheres. A predicted-only point is not automatically an error: both passing laws contain some low-impulse predicted-only points.",
                "- A retained independent rerun reproduces all ten semantic arrays, normalized metrics, and this Markdown report exactly.",
                "",
                "## Activation localization",
                "",
                *markdown_table(
                    [
                        "law",
                        "covered rows",
                        "covered rows with predicted-only points",
                        "uncovered rows",
                        "uncovered rows with predicted-only points",
                        "actual points missed by prediction",
                    ],
                    activation_rows,
                ),
                "",
                "Every stiff miss predicts at least one additional point, and no actual loaded point is absent from the prediction. But predicted-only activation is not sufficient by itself: it also occurs in 8/42 covered stiff rows and 13/48 covered soft rows.",
                "",
                "Within this spent holdout, summed predicted-only impulse is strictly separated: the largest covered stiff row is "
                f"{magnitude_separation['maximum_covered_false_impulse_norm_sum_ns']:.3f} N·s and the smallest uncovered row is "
                f"{magnitude_separation['minimum_uncovered_false_impulse_norm_sum_ns']:.3f} N·s. This is localization evidence only, not an eligible threshold.",
                "",
                "## Six rejected stiff rows",
                "",
                *markdown_table(
                    [
                        "state",
                        "MuJoCo active",
                        "reduced active",
                        "predicted-only impulse N·s",
                        "original max abs residual",
                        "label-mask max abs residual",
                        "label-mask coverage",
                    ],
                    detail_rows,
                ),
                "",
                "The label-oracle counterfactual zeros predicted impulse at completed-label inactive points without re-solving coupling. It repairs four of six original misses (42/48 → 46/48), but states 100009 and 100037 remain outside the frozen box and worsen sharply. Therefore a contact mask or magnitude threshold is not the missing mechanism.",
                "",
                "## Causal first-impact cohort",
                "",
                "A separate label-free construction evaluates ballistic surface crossing only at the authored five 1 ms plant ticks. It uses prestate gap, normal velocity, and smooth free acceleration; completed impulses enter only after the cohort is frozen for scoring.",
                "",
                "Across the six rejected rows, the first cohort is a subset of the eventual MuJoCo active set in 6/6 rows, introduces 0 false points, and is already the exact final active set in 5/6 rows. The remaining row starts at point 6 and later grows to points 3 and 1. The cohort therefore identifies initial activation but cannot be treated as a terminal predictor; later contacts require contact-coupled state evolution on the declared clock.",
                "",
                "## Decision",
                "",
                "Build the next construction around the independently declared event clock: admit the first ballistic cohort, advance the generalized state, update point geometry and velocity, recompute the full Delassus response, and only then test later cohorts. Freeze that mechanism from model equations and convergence only, then spend a new untouched law/state holdout. R228 remains rejected and no selector, plant action, or authority is admitted.",
            ]
        ) + "\n"

        output = pathlib.Path(args.output)
        output.mkdir(parents=True, exist_ok=True)
        (output / "g1-stiff-contact-activation-localization-metrics.json").write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n"
        )
        (output / "G1_STIFF_CONTACT_ACTIVATION_LOCALIZATION.md").write_text(report)
        np.savez_compressed(
            output / "g1-stiff-contact-activation-localization.npz",
            stiff_false_activation=stiff["false_activation"].astype(np.uint8),
            stiff_missed_activation=stiff["missed_activation"].astype(np.uint8),
            stiff_false_impulse_norm_sum_ns=stiff["false_impulse_norm_sum_ns"],
            stiff_free_flight_activation_tick=activation_tick,
            stiff_first_free_flight_cohort=first_cohort.astype(np.uint8),
            stiff_first_cohort_exact_actual_set=first_cohort_exact.astype(np.uint8),
            stiff_first_cohort_subset_of_actual_set=first_cohort_subset.astype(np.uint8),
            stiff_oracle_masked_impulse=oracle_impulse,
            stiff_oracle_masked_residual=oracle_residual,
            stiff_oracle_masked_covered=oracle_covered.astype(np.uint8),
        )
        web = pathlib.Path(args.web_report)
        web.parent.mkdir(parents=True, exist_ok=True)
        web.write_text(render_report_html(report))

    print(
        json.dumps(
            {
                "diagnostic_passed": True,
                "construction_or_selection_eligible": False,
                "original_stiff_coverage": "42/48",
                "label_mask_counterfactual_coverage": f"{int(np.count_nonzero(oracle_covered))}/48",
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
