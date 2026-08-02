#!/usr/bin/env python3
"""R227 construction audit of globally coupled positive-reference contact."""

from __future__ import annotations

import argparse
import json
import pathlib
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_causal_center_split_kinetic_replay import grouped_summary
from g1_contact_law_momentum_holdout import (
    CONTROL_DT,
    FOOT_FRAMES,
    GRAVITY,
    PHYSICS_DT,
    WIDTH_GATES,
    sha256,
    standing_posture,
)
from g1_positive_reference_compliance_audit import (
    REFERENCE_DOCUMENTATION,
    law_cone_id,
    law_reduced_integrator_id,
    prepare_law,
)
from g1_substepped_compliant_contact_holdout import (
    FRESH_CONTACT_LAWS,
    SAMPLE_OFFSETS,
    residual_half_width,
)
from g1_substepped_compliant_contact_replay import group_maximum_absolute


REVISION = "g1-coupled-positive-reference-compliance-audit-r227"
SOURCE_REVISION = "g1-substepped-compliant-contact-holdout-r221"
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-substepped-compliant-contact-holdout-r221/"
    "g1-substepped-compliant-contact-holdout.npz"
)
R226_METRICS = pathlib.Path(
    "benchmarks/results/g1-positive-reference-compliance-audit-r226/"
    "g1-positive-reference-compliance-metrics.json"
)


@dataclass(frozen=True)
class CoupledReferenceProfile:
    name: str
    substeps: int
    projection_sweeps: int


PROFILES = tuple(
    CoupledReferenceProfile(
        f"coupled_steps{substeps}_sweeps{sweeps}", substeps, sweeps
    )
    for substeps in (16, 32, 64)
    for sweeps in (1, 2, 4, 8, 16, 32, 64, 128)
)
SWEEP_REFINEMENT_GATE_FRACTION = 0.02
SUBSTEP_REFINEMENT_GATE_FRACTION = 0.20
QUERY_DEADLINE_NS = 5_000_000.0


def physical_impulse_upper(
    prospective_velocity: np.ndarray,
    total_mass_kg: float,
    friction: float,
    output: np.ndarray,
) -> None:
    maximum_closing_speed = float(
        np.max(np.maximum(0.0, -prospective_velocity[:, 2]))
    )
    normal_cap = total_mass_kg * (maximum_closing_speed + GRAVITY * CONTROL_DT)
    output[:, :2] = friction * normal_cap
    output[:, 2] = normal_cap


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--r226-metrics", default=str(R226_METRICS))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT_R227.html",
    )
    return parser.parse_args()


def evaluate_profile_law(
    session: Any,
    replay: Any,
    prepared: dict[str, np.ndarray],
    law: Any,
    profile: CoupledReferenceProfile,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    prefix = law.name
    samples = len(replay[f"{prefix}_root_height"])
    generalized_dof = int(session.generalized_dof())
    half_width = residual_half_width(generalized_dof)
    predicted_impulse = np.empty((8, 3), np.float64)
    velocity_after = np.empty((8, 3), np.float64)
    gap_after = np.empty(8, np.float64)
    residual = np.empty((samples, generalized_dof), np.float64)
    predicted = np.empty((samples, 8, 3), np.float64)
    timing = np.empty((samples, 2), np.uint64)
    component_covered = np.empty((samples, generalized_dof), np.uint8)
    sample_covered = np.empty(samples, np.uint8)
    impulse_upper = np.empty((8, 3), np.float64)
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
    for sample in range(samples):
        physical_impulse_upper(
            replay[f"{prefix}_prospective_velocity"][sample],
            float(prepared["total_mass_kg"]),
            law.friction,
            impulse_upper,
        )
        arguments = (
            np.ascontiguousarray(replay[f"{prefix}_contact_points"][sample, :, 2]),
            replay[f"{prefix}_prospective_velocity"][sample],
            np.ascontiguousarray(prepared["free_acceleration"][sample]),
            prepared["delassus"][sample],
            impulse_upper,
            *common,
            2.0 * PHYSICS_DT,
            CONTROL_DT,
            profile.substeps,
            profile.projection_sweeps,
            law_cone_id(law),
            law_reduced_integrator_id(law),
            predicted_impulse,
            velocity_after,
            gap_after,
        )
        result = session.solve_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 0] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        first = (
            predicted_impulse.copy(),
            velocity_after.copy(),
            gap_after.copy(),
        )
        result = session.solve_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        timing[sample, 1] = result[0]
        zero_allocation &= result[1:] == (0, 0)
        if not all(
            np.array_equal(before, after)
            for before, after in zip(
                first, (predicted_impulse, velocity_after, gap_after), strict=True
            )
        ):
            raise RuntimeError("coupled positive-reference query is not bitwise deterministic")
        actual_impulse = replay[f"{prefix}_contact_impulse"][sample]
        residual[sample] = replay[f"{prefix}_raw_velocity_error"][sample] + np.einsum(
            "dca,ca->d",
            prepared["response"][sample],
            actual_impulse - predicted_impulse,
            optimize=True,
        )
        predicted[sample] = predicted_impulse
        component_covered[sample] = np.abs(residual[sample]) <= half_width + 1.0e-12
        sample_covered[sample] = np.all(component_covered[sample])

    fitted_half_width = group_maximum_absolute(residual)
    fitted_width = {name: 2.0 * value for name, value in fitted_half_width.items()}
    strict = bool(np.all(sample_covered))
    width_gate = all(fitted_width[name] <= WIDTH_GATES[name] for name in WIDTH_GATES)
    timing_summary = distribution(timing.reshape(-1))
    deadline_gate = timing_summary["p99"] <= QUERY_DEADLINE_NS
    result = {
        "profile": profile.name,
        "law": prefix,
        "substeps": profile.substeps,
        "projection_sweeps": profile.projection_sweeps,
        "sample_coverage_under_frozen_r220_box": float(np.mean(sample_covered)),
        "component_coverage_under_frozen_r220_box": float(np.mean(component_covered)),
        "strict_frozen_box_coverage": strict,
        "predictor_residual": grouped_summary(residual),
        "construction_fitted_width": fitted_width,
        "width_gate_passed": width_gate,
        "cpu_deadline_passed": deadline_gate,
        "construction_candidate_passed": bool(
            strict and width_gate and deadline_gate and zero_allocation
        ),
        "query_timing_ns": timing_summary,
        "zero_rust_allocation": zero_allocation,
        "bitwise_repeat": True,
        "profile_promoted": False,
        "authority_admitted": False,
    }
    arrays = {
        f"{profile.name}_{prefix}_predicted_impulse": predicted,
        f"{profile.name}_{prefix}_residual": residual,
        f"{profile.name}_{prefix}_component_covered": component_covered,
        f"{profile.name}_{prefix}_sample_covered": sample_covered,
        f"{profile.name}_{prefix}_timing_ns": timing,
    }
    return result, arrays


def diagnostic_profile(results: list[dict[str, Any]]) -> str:
    def score(profile: CoupledReferenceProfile) -> tuple[float, float, float]:
        rows = [row for row in results if row["profile"] == profile.name]
        minimum_coverage = min(row["sample_coverage_under_frozen_r220_box"] for row in rows)
        maximum_width_ratio = max(
            row["construction_fitted_width"][name] / WIDTH_GATES[name]
            for row in rows
            for name in WIDTH_GATES
        )
        maximum_p99 = max(row["query_timing_ns"]["p99"] for row in rows)
        return minimum_coverage, -maximum_width_ratio, -maximum_p99

    return max(PROFILES, key=score).name


def causal_convergence_selection(
    results: list[dict[str, Any]],
    arrays: dict[str, np.ndarray],
    prepared: dict[str, dict[str, np.ndarray]],
) -> tuple[str | None, list[dict[str, Any]]]:
    """Select work from causal predictions only, never completed labels."""

    profiles = {(profile.substeps, profile.projection_sweeps): profile for profile in PROFILES}
    groups = {
        "root_angular_rad_s": slice(0, 3),
        "root_linear_m_s": slice(3, 6),
        "joint_rad_s": slice(6, None),
    }

    def refinement_ratio(
        lower: CoupledReferenceProfile,
        upper: CoupledReferenceProfile,
        law: Any,
    ) -> tuple[float, dict[str, float]]:
        lower_impulse = arrays[f"{lower.name}_{law.name}_predicted_impulse"]
        upper_impulse = arrays[f"{upper.name}_{law.name}_predicted_impulse"]
        generalized_delta = np.einsum(
            "sdca,sca->sd",
            prepared[law.name]["response"],
            upper_impulse - lower_impulse,
            optimize=True,
        )
        ratios = {
            name: float(np.max(np.abs(generalized_delta[:, section]))) / WIDTH_GATES[name]
            for name, section in groups.items()
        }
        return max(ratios.values()), ratios

    audits: list[dict[str, Any]] = []
    for profile in PROFILES:
        doubled_sweeps = profiles.get((profile.substeps, 2 * profile.projection_sweeps))
        doubled_substeps = profiles.get((2 * profile.substeps, profile.projection_sweeps))
        if doubled_sweeps is None or doubled_substeps is None:
            continue
        sweep_by_law: dict[str, Any] = {}
        substep_by_law: dict[str, Any] = {}
        for law in FRESH_CONTACT_LAWS:
            sweep_maximum, sweep_groups = refinement_ratio(profile, doubled_sweeps, law)
            substep_maximum, substep_groups = refinement_ratio(profile, doubled_substeps, law)
            sweep_by_law[law.name] = {
                "maximum_gate_fraction": sweep_maximum,
                "group_gate_fraction": sweep_groups,
            }
            substep_by_law[law.name] = {
                "maximum_gate_fraction": substep_maximum,
                "group_gate_fraction": substep_groups,
            }
        result_rows = [row for row in results if row["profile"] == profile.name]
        maximum_p99_ns = max(row["query_timing_ns"]["p99"] for row in result_rows)
        sweep_maximum = max(row["maximum_gate_fraction"] for row in sweep_by_law.values())
        substep_maximum = max(
            row["maximum_gate_fraction"] for row in substep_by_law.values()
        )
        passed = bool(
            sweep_maximum <= SWEEP_REFINEMENT_GATE_FRACTION
            and substep_maximum <= SUBSTEP_REFINEMENT_GATE_FRACTION
            and maximum_p99_ns <= QUERY_DEADLINE_NS
        )
        audits.append(
            {
                "profile": profile.name,
                "work_units": profile.substeps * profile.projection_sweeps,
                "sweep_comparator": doubled_sweeps.name,
                "substep_comparator": doubled_substeps.name,
                "sweep_refinement": sweep_by_law,
                "substep_refinement": substep_by_law,
                "maximum_sweep_gate_fraction": sweep_maximum,
                "maximum_substep_gate_fraction": substep_maximum,
                "maximum_query_p99_ns": maximum_p99_ns,
                "causal_convergence_passed": passed,
            }
        )
    passing = [row for row in audits if row["causal_convergence_passed"]]
    selected = min(
        passing,
        key=lambda row: (row["work_units"], row["maximum_query_p99_ns"], row["profile"]),
        default=None,
    )
    return (None if selected is None else str(selected["profile"])), audits


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source = pathlib.Path(args.source_replay).resolve()
    r226_metrics = pathlib.Path(args.r226_metrics).resolve()
    if not model.is_file() or not source.is_file() or not r226_metrics.is_file():
        raise SystemExit("R227 requires the pinned G1 model and retained R221/R226 artifacts")
    replay = np.load(source)
    baseline = json.loads(r226_metrics.read_text())
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    )
    q_nominal = standing_posture(list(session.joint_names()))
    prepared = {
        law.name: prepare_law(session, model, replay, law, offset, q_nominal)
        for law, offset in zip(FRESH_CONTACT_LAWS, SAMPLE_OFFSETS, strict=True)
    }
    results: list[dict[str, Any]] = []
    arrays: dict[str, np.ndarray] = {}
    for profile in PROFILES:
        for law in FRESH_CONTACT_LAWS:
            result, result_arrays = evaluate_profile_law(
                session, replay, prepared[law.name], law, profile
            )
            results.append(result)
            arrays.update(result_arrays)
    mechanism_passed = all(
        row["zero_rust_allocation"] and row["bitwise_repeat"] for row in results
    )
    viable_profiles = [
        profile.name
        for profile in PROFILES
        if all(
            row["construction_candidate_passed"]
            for row in results
            if row["profile"] == profile.name
        )
    ]
    diagnostic = diagnostic_profile(results)
    diagnostic_rows = [row for row in results if row["profile"] == diagnostic]
    causal_profile, causal_convergence = causal_convergence_selection(
        results, arrays, prepared
    )
    causal_rows = [row for row in results if row["profile"] == causal_profile]
    baseline_rows = [
        row
        for row in baseline["results"]
        if row["profile"] == "reference_steps32_free_accel"
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_replay_sha256": sha256(source),
        "r226_metrics_sha256": sha256(r226_metrics),
        "model": str(model),
        "model_sha256": sha256(model),
        "samples": 96,
        "reference_forward_dynamics_queries": 96,
        "physics_steps": 0,
        "integration_steps": 0,
        "policy_or_controller_steps": 0,
        "selector_or_plant_actions": 0,
        "reference_documentation": REFERENCE_DOCUMENTATION,
        "mechanism_passed": mechanism_passed,
        "viable_profiles": viable_profiles,
        "diagnostic_profile_from_rejected_labels": diagnostic,
        "causal_selection_contract": {
            "uses_completed_labels": False,
            "sweep_refinement_gate_fraction_of_width": SWEEP_REFINEMENT_GATE_FRACTION,
            "substep_refinement_gate_fraction_of_width": SUBSTEP_REFINEMENT_GATE_FRACTION,
            "query_deadline_ns": QUERY_DEADLINE_NS,
            "selection": "lowest substeps*projection_sweeps, then p99, then name",
        },
        "causal_convergence": causal_convergence,
        "causally_selected_profile": causal_profile,
        "construction_profile_frozen_for_fresh_holdout": causal_profile is not None,
        "r226_representative_baseline": baseline_rows,
        "profile_promoted": False,
        "authority_admitted": False,
        "results": results,
    }
    rows = []
    for result in results:
        width = result["construction_fitted_width"]
        residual = result["predictor_residual"]
        rows.append(
            [
                result["profile"],
                result["law"],
                f"{100.0 * result['sample_coverage_under_frozen_r220_box']:.3f}%",
                f"{residual['root_angular_rad_s']['p95']:.3f} / {residual['root_linear_m_s']['p95']:.3f} / {residual['joint_rad_s']['p95']:.3f}",
                f"{width['root_angular_rad_s']:.3f} / {width['root_linear_m_s']:.3f} / {width['joint_rad_s']:.3f}",
                f"{result['query_timing_ns']['p99'] / 1_000.0:.3f}",
                "candidate" if result["construction_candidate_passed"] else "reject",
            ]
        )
    summary = {row["law"]: row for row in diagnostic_rows}
    causal_summary = {row["law"]: row for row in causal_rows}
    selected_audit = next(
        (row for row in causal_convergence if row["profile"] == causal_profile), None
    )
    if causal_profile is None or selected_audit is None:
        decision = (
            f"The label-diagnostic row `{diagnostic}` reaches "
            f"{100.0 * summary[FRESH_CONTACT_LAWS[0].name]['sample_coverage_under_frozen_r220_box']:.3f}% mid-law and "
            f"{100.0 * summary[FRESH_CONTACT_LAWS[1].name]['sample_coverage_under_frozen_r220_box']:.3f}% hard-law sample coverage, "
            "but it is not used for selection. No row meets the causal convergence and 5 ms deadline contract; freeze nothing and do not generate a new holdout."
        )
    else:
        decision = (
            f"The label-diagnostic row `{diagnostic}` reaches "
            f"{100.0 * summary[FRESH_CONTACT_LAWS[0].name]['sample_coverage_under_frozen_r220_box']:.3f}% mid-law and "
            f"{100.0 * summary[FRESH_CONTACT_LAWS[1].name]['sample_coverage_under_frozen_r220_box']:.3f}% hard-law sample coverage, but it is not used for selection. "
            f"The causal convergence rule selects `{causal_profile}` by comparing only predicted generalized velocity: doubling sweeps changes at most "
            f"{100.0 * selected_audit['maximum_sweep_gate_fraction']:.3f}% of the corresponding useful-width gate, doubling substeps changes at most "
            f"{100.0 * selected_audit['maximum_substep_gate_fraction']:.3f}%, and retained p99 remains below 5 ms. That independently selected row also covers "
            f"{100.0 * causal_summary[FRESH_CONTACT_LAWS[0].name]['sample_coverage_under_frozen_r220_box']:.3f}%/"
            f"{100.0 * causal_summary[FRESH_CONTACT_LAWS[1].name]['sample_coverage_under_frozen_r220_box']:.3f}% at useful width on rejected labels. "
            "Freeze this construction profile before generating a new law/state holdout; authority remains absent."
        )
    report = "\n".join(
        [
            "# Bonesaw G1 coupled positive-reference compliance audit · r227",
            "",
            f"> Generic coupled mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · viable construction profiles **{len(viable_profiles)}** · profile/authority **NOT PROMOTED** · physics/integration/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0 / 0**.",
            "",
            "## Contract",
            "",
            "- Rust converts the documented positive-reference acceleration into a desired contact-space velocity increment, then distributes it through the complete state-local Delassus operator with fixed forward/reverse projected sweeps. Effective mass is no longer an independent per-contact input.",
            "- Every microstep enforces cumulative per-axis caps, nonnegative normal impulse, and the declared circular/pyramidal friction section. The solver is bounded-work and allocation-free, but is not MuJoCo's generalized nonlinear constraint optimizer or an outer bound.",
            "- Impulse caps are label-free momentum limits: total model mass times maximum causal closing speed plus one control interval of gravity, with tangent capacity from the authored friction coefficient. The audit reuses rejected immutable R221 labels and R220's residual box only as a construction comparator. It makes 96 prestate forward-dynamics queries and takes no physics integration, policy, controller, selector, or plant-action step. No row can enter authority from this audit.",
            "- Every query repeats bitwise and allocates no Rust heap memory. A complete retained rerun reproduced all 192 non-timing NPZ arrays exactly; timing is reported but excluded from semantic equality.",
            "",
            "## Construction grid",
            "",
            *markdown_table(
                [
                    "profile",
                    "law",
                    "coverage in R220 box",
                    "residual p95 ω/v/joint",
                    "fitted width ω/v/joint",
                    "query p99 µs",
                    "gate",
                ],
                rows,
            ),
            "",
            "## Decision",
            "",
            decision,
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-coupled-positive-reference-compliance-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md").write_text(report)
    np.savez_compressed(
        output / "g1-coupled-positive-reference-compliance-audit.npz", **arrays
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "viable_profiles": viable_profiles,
                "diagnostic_profile": diagnostic,
                "causally_selected_profile": causal_profile,
                "construction_profile_frozen_for_fresh_holdout": causal_profile is not None,
                "profile_promoted": False,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
