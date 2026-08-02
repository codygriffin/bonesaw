#!/usr/bin/env python3
"""Exactly-once no-refit plant holdout for the frozen R264 profile.

The two laws, state offsets, source hashes, profile hash, candidate family, and
gates below are declared before any R265 labels exist.  This module contains no
fit, widening, threshold search, retry, or authority path.  It refuses to
generate labels when its output directory already exists.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
from datetime import datetime, timezone

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_contact_law_momentum_holdout import FOOT_FRAMES, ContactLaw, sha256
from g1_paired_terminal_score_plant_holdout_r254 import COMPONENT_NAMES
import g1_paired_terminal_score_plant_holdout_r254 as label_generator
from g1_residual_prototype_profile_r264 import (
    COMPONENTS,
    MAXIMUM_COMPONENT_REGRESSION,
    MINIMUM_COMPONENT_IMPROVEMENT,
    component_deltas,
    feature_matrix,
)


REVISION = "g1-residual-prototype-plant-holdout-r265"
SOURCE_REVISION = "g1-residual-prototype-profile-r264"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-residual-prototype-profile-r264/"
    "g1-residual-prototype-profile-metrics.json"
)
SOURCE_PROFILE = pathlib.Path(
    "benchmarks/results/g1-residual-prototype-profile-r264/"
    "g1-residual-prototype-profile.npz"
)
EXPECTED_SOURCE_PROFILE_SHA256 = (
    "c97010deab87353edb08ff67fb02e738baf5c9be4035248b0a3ab8a45bd77bff"
)
EXPECTED_MODEL_SHA256 = (
    "9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43"
)
FRESH_PLANT_LAWS = (
    ContactLaw(
        "medium_elliptic_euler_residual_r265",
        friction=0.82,
        solref_time_s=0.0034,
        solimp_min=0.981,
        solimp_max=0.997,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_EULER),
    ),
    ContactLaw(
        "stiff_elliptic_rk4_residual_r265",
        friction=0.98,
        solref_time_s=0.0020,
        solimp_min=0.991,
        solimp_max=0.999,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
)
SAMPLE_OFFSETS = (330_000, 340_000)
SAMPLES_PER_LAW = 48
CANDIDATES = 3
DIAGNOSTICS = 2 * COMPONENTS + 2
NONREGRESSION_TOLERANCE = 1.0e-12


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--source-profile", default=str(SOURCE_PROFILE))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT_R265.html",
    )
    return parser.parse_args(argv)


def allocate_profile_outputs(rows: int) -> dict[str, np.ndarray]:
    return {
        "envelopes": np.empty((rows, CANDIDATES, DIAGNOSTICS), np.float64),
        "selections": np.empty((rows, 6), np.float64),
        "nearest": np.empty(rows, np.int64),
        "distance": np.empty(rows, np.float64),
        "supported": np.empty(rows, np.uint8),
        "timing": np.empty(rows, np.uint64),
        "allocation_calls": np.empty(rows, np.uint64),
        "allocated_bytes": np.empty(rows, np.uint64),
    }


def score_profile(
    session: object,
    profile: dict[str, np.ndarray],
    features: np.ndarray,
    groups: np.ndarray,
    predicted_component: np.ndarray,
    predicted_aggregate: np.ndarray,
    available: np.ndarray,
) -> dict[str, np.ndarray]:
    outputs = allocate_profile_outputs(len(features))
    session.score_terminal_impact_residual_prototype_profile(
        features,
        groups,
        predicted_component,
        predicted_aggregate,
        available,
        profile["feature_inverse_scale"],
        profile["prototype_features"],
        profile["prototype_groups"],
        profile["prototype_component_residuals"],
        profile["prototype_aggregate_residuals"],
        profile["group_component_lower_extension"],
        profile["group_component_upper_extension"],
        profile["group_aggregate_lower_extension"],
        profile["group_aggregate_upper_extension"],
        profile["maximum_distance_squared_by_group"],
        MAXIMUM_COMPONENT_REGRESSION,
        MINIMUM_COMPONENT_IMPROVEMENT,
        outputs["envelopes"],
        outputs["selections"],
        outputs["nearest"],
        outputs["distance"],
        outputs["supported"],
        outputs["timing"],
        outputs["allocation_calls"],
        outputs["allocated_bytes"],
    )
    return outputs


def generate_labels_once(model: pathlib.Path, label_output: pathlib.Path) -> int:
    original_argv = sys.argv
    original_laws = label_generator.FRESH_PLANT_LAWS
    original_offsets = label_generator.SAMPLE_OFFSETS
    original_revision = label_generator.REVISION
    try:
        label_generator.FRESH_PLANT_LAWS = FRESH_PLANT_LAWS
        label_generator.SAMPLE_OFFSETS = SAMPLE_OFFSETS
        label_generator.REVISION = REVISION + "-labels"
        sys.argv = [
            str(pathlib.Path(label_generator.__file__)),
            "--model",
            str(model),
            "--output",
            str(label_output),
            "--web-report",
            str(label_output / "labels.html"),
        ]
        return int(label_generator.main())
    finally:
        sys.argv = original_argv
        label_generator.FRESH_PLANT_LAWS = original_laws
        label_generator.SAMPLE_OFFSETS = original_offsets
        label_generator.REVISION = original_revision


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_metrics = pathlib.Path(args.source_metrics).resolve()
    source_profile = pathlib.Path(args.source_profile).resolve()
    output = pathlib.Path(args.output)
    label_output = output.with_name(output.name + "-labels")
    web = pathlib.Path(args.web_report)
    if output.exists() or label_output.exists():
        raise SystemExit(
            "R265 is exactly-once: refusing to overwrite existing holdout or labels"
        )
    if not model.is_file() or not source_metrics.is_file() or not source_profile.is_file():
        raise SystemExit("R265 requires the pinned model and frozen R264 evidence")
    source = json.loads(source_metrics.read_text())
    source_hash = sha256(source_profile)
    model_hash = sha256(model)
    frozen_source = bool(
        source.get("revision") == SOURCE_REVISION
        and source.get("profile_frozen_for_one_fresh_holdout") is True
        and source.get("authority_admitted") is False
        and source_hash == EXPECTED_SOURCE_PROFILE_SHA256
        and model_hash == EXPECTED_MODEL_SHA256
    )
    if not frozen_source:
        raise SystemExit("R265 frozen profile/model provenance does not match declaration")

    generator_status = generate_labels_once(model, label_output)
    label_metrics_path = label_output / "g1-paired-terminal-score-plant-holdout-metrics.json"
    labels_path = label_output / "g1-paired-terminal-score-plant-holdout.npz"
    label_metrics = json.loads(label_metrics_path.read_text())
    if generator_status != 0 or not label_metrics.get("mechanism_passed"):
        raise RuntimeError("fresh label mechanism failed; R265 remains consumed")

    with np.load(source_profile, allow_pickle=False) as frozen:
        profile = {
            name: np.ascontiguousarray(frozen[name])
            for name in (
                "feature_inverse_scale",
                "prototype_features",
                "prototype_groups",
                "prototype_component_residuals",
                "prototype_aggregate_residuals",
                "group_component_lower_extension",
                "group_component_upper_extension",
                "group_aggregate_lower_extension",
                "group_aggregate_upper_extension",
                "maximum_distance_squared_by_group",
            )
        }
    with np.load(labels_path, allow_pickle=False) as labels:
        root = np.asarray(labels["root_state"], np.float64)
        q = np.asarray(labels["joint_position"], np.float64)
        velocity = np.asarray(labels["initial_velocity"], np.float64)
        groups = np.asarray(labels["groups"], np.uint8)
        predicted = np.asarray(labels["predicted_diagnostics"], np.float64)
        actual = np.asarray(labels["actual_diagnostics"], np.float64)
        fixed_status = np.asarray(labels["fixed_status"], np.uint8)
        law_index = np.asarray(labels["law_index"], np.uint8)
        wbc_ns = np.asarray(labels["wbc_ns"], np.uint64)
        realization_ns = np.asarray(labels["realization_ns"], np.uint64)
        plant_warning = np.asarray(labels["plant_warning"], np.uint16)

    features = feature_matrix(root, q, velocity)
    predicted_component = component_deltas(predicted)
    actual_component = component_deltas(actual)
    predicted_aggregate = predicted[:, :, 16] - predicted[:, 0:1, 16]
    actual_aggregate = actual[:, :, 16] - actual[:, 0:1, 16]
    available = (fixed_status <= 1).astype(np.uint8)
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 2 + [FOOT_FRAMES[1]] * 2
    )
    rehearsal = score_profile(
        session,
        profile,
        features,
        groups,
        predicted_component,
        predicted_aggregate,
        available,
    )
    repeat = score_profile(
        session,
        profile,
        features,
        groups,
        predicted_component,
        predicted_aggregate,
        available,
    )
    semantic_repeat = all(
        np.array_equal(rehearsal[name], repeat[name])
        for name in ("envelopes", "selections", "nearest", "distance", "supported")
    )
    lower = rehearsal["envelopes"][:, :, :COMPONENTS]
    upper = rehearsal["envelopes"][:, :, COMPONENTS : 2 * COMPONENTS]
    aggregate_lower = rehearsal["envelopes"][:, :, 2 * COMPONENTS]
    aggregate_upper = rehearsal["envelopes"][:, :, 2 * COMPONENTS + 1]
    component_covered = (actual_component >= lower - NONREGRESSION_TOLERANCE) & (
        actual_component <= upper + NONREGRESSION_TOLERANCE
    )
    aggregate_covered = (
        actual_aggregate >= aggregate_lower - NONREGRESSION_TOLERANCE
    ) & (actual_aggregate <= aggregate_upper + NONREGRESSION_TOLERANCE)
    supported = rehearsal["supported"] != 0
    supported_covered = bool(
        np.all(component_covered[supported]) and np.all(aggregate_covered[supported])
    )
    selected = rehearsal["selections"][:, 0].astype(np.int64)
    selected_component = actual_component[np.arange(len(selected)), selected]
    selected_aggregate = actual_aggregate[np.arange(len(selected)), selected]
    nonzero = selected != 0
    safe_nonzero = nonzero & (
        np.max(selected_component, axis=1) <= NONREGRESSION_TOLERANCE
    ) & (selected_aggregate < -NONREGRESSION_TOLERANCE)
    zero_allocation = bool(
        np.all(rehearsal["allocation_calls"] == 0)
        and np.all(rehearsal["allocated_bytes"] == 0)
    )
    mechanism_passed = bool(
        frozen_source
        and label_metrics.get("mechanism_passed")
        and semantic_repeat
        and zero_allocation
        and np.all(available[:, 0] == 1)
        and np.sum(plant_warning) == 0
    )
    profile_transferred = bool(
        mechanism_passed
        and supported_covered
        and np.count_nonzero(nonzero) > 0
        and np.all(safe_nonzero[nonzero])
    )

    laws = []
    for slot, law in enumerate(FRESH_PLANT_LAWS):
        members = law_index == slot
        active = members & supported
        laws.append(
            {
                "law": law.name,
                "offset": SAMPLE_OFFSETS[slot],
                "rows": int(np.count_nonzero(members)),
                "distance_supported_rows": int(np.count_nonzero(active)),
                "supported_component_coverage_fraction": float(
                    np.mean(component_covered[active]) if np.any(active) else 1.0
                ),
                "supported_aggregate_coverage_fraction": float(
                    np.mean(aggregate_covered[active]) if np.any(active) else 1.0
                ),
                "selected_counts": {
                    str(candidate): int(np.count_nonzero(members & (selected == candidate)))
                    for candidate in range(CANDIDATES)
                },
                "nonzero_actions": int(np.count_nonzero(members & nonzero)),
                "strict_nonregressing_improving_nonzero_actions": int(
                    np.count_nonzero(members & safe_nonzero)
                ),
                "selected_component_regression_rows": int(
                    np.count_nonzero(
                        members
                        & (np.max(selected_component, axis=1) > NONREGRESSION_TOLERANCE)
                    )
                ),
                "selected_aggregate_regression_rows": int(
                    np.count_nonzero(
                        members & (selected_aggregate > NONREGRESSION_TOLERANCE)
                    )
                ),
            }
        )

    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_profile_sha256": source_hash,
        "expected_source_profile_sha256": EXPECTED_SOURCE_PROFILE_SHA256,
        "model_sha256": model_hash,
        "source_frozen_and_immutable": frozen_source,
        "fresh_laws_and_offsets_declared_before_labels": True,
        "no_refit_or_widening": True,
        "holdout_consumed": True,
        "repeat_holdout_allowed": False,
        "samples": len(features),
        "physics_steps": int(label_metrics["physics_steps"]),
        "policy_steps": 0,
        "plant_actions": 0,
        "distance_supported_rows": int(np.count_nonzero(supported)),
        "distance_rejected_rows": int(np.count_nonzero(~supported)),
        "supported_component_coverage_fraction": float(
            np.mean(component_covered[supported]) if np.any(supported) else 1.0
        ),
        "supported_aggregate_coverage_fraction": float(
            np.mean(aggregate_covered[supported]) if np.any(supported) else 1.0
        ),
        "all_supported_candidate_boxes_covered": supported_covered,
        "selected_counts": {
            str(candidate): int(np.count_nonzero(selected == candidate))
            for candidate in range(CANDIDATES)
        },
        "nonzero_actions": int(np.count_nonzero(nonzero)),
        "strict_nonregressing_improving_nonzero_actions": int(
            np.count_nonzero(safe_nonzero)
        ),
        "selected_component_regression_rows": int(
            np.count_nonzero(
                np.max(selected_component, axis=1) > NONREGRESSION_TOLERANCE
            )
        ),
        "selected_aggregate_regression_rows": int(
            np.count_nonzero(selected_aggregate > NONREGRESSION_TOLERANCE)
        ),
        "semantic_repeat": semantic_repeat,
        "zero_profile_rust_allocation": zero_allocation,
        "profile_timing_ns": distribution(rehearsal["timing"]),
        "wbc_timing_ns": distribution(wbc_ns.reshape(-1)),
        "realization_timing_ns": distribution(realization_ns.reshape(-1)),
        "mujoco_warning_count": int(np.sum(plant_warning)),
        "mechanism_passed": mechanism_passed,
        "profile_transferred": profile_transferred,
        "authority_admitted": False,
        "laws": laws,
    }
    table = [
        [
            law["law"].replace("_residual_r265", ""),
            str(law["offset"]),
            f"{law['distance_supported_rows']}/{law['rows']}",
            " / ".join(str(law["selected_counts"][str(i)]) for i in range(3)),
            (
                f"{law['strict_nonregressing_improving_nonzero_actions']} / "
                f"{law['nonzero_actions']}"
            ),
            f"{law['supported_component_coverage_fraction'] * 100.0:.3f}%",
            f"{law['supported_aggregate_coverage_fraction'] * 100.0:.3f}%",
        ]
        for law in laws
    ]
    verdict = "TRANSFERRED" if profile_transferred else "REJECTED"
    report = "\n".join(
        [
            "# Bonesaw residual-prototype one-shot plant holdout · r265",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen profile **{verdict}** · holdout **CONSUMED** · authority **NOT ADMITTED**.",
            "",
            "R265 is the single no-refit holdout authorized by R264. The frozen 55-coordinate profile, all 192 prototypes, causal-cell calibration, distance gates, candidates, thresholds, source hashes, and model hash are consumed unchanged. New medium elliptic/Euler and stiff elliptic/RK4 laws use untouched offsets 330,000 and 340,000. Each of 96 states executes exact zero, realized zero-WBC, and realized neutral-recovery branches for five 4 ms MuJoCo steps. No policy or plant command runs.",
            "",
            *markdown_table(
                [
                    "fresh law",
                    "offset",
                    "distance support",
                    "selected 0 / 1 / 2",
                    "strict safe+improved / nonzero",
                    "component coverage",
                    "aggregate coverage",
                ],
                table,
            ),
            "",
            f"The run executes {result['physics_steps']:,} fresh MuJoCo steps. {result['distance_supported_rows']}/{result['samples']} rows are inside the frozen distance envelope; unsupported rows fail closed to baseline. Supported component/aggregate coverage is {result['supported_component_coverage_fraction'] * 100.0:.3f}%/{result['supported_aggregate_coverage_fraction'] * 100.0:.3f}%. The selector emits {result['nonzero_actions']} nonzero actions; {result['strict_nonregressing_improving_nonzero_actions']} are actually strictly nonregressing and improving, with {result['selected_component_regression_rows']}/{result['selected_aggregate_regression_rows']} selected component/aggregate regression rows. Profile query p99 is {result['profile_timing_ns']['p99'] / 1e3:.2f} µs with zero Rust allocations and exact semantic repeat.",
            "",
            "This one-shot result is final evidence. The evaluator refuses overwrite, and the holdout may not be rerun, widened, or reinterpreted as a plant command or authority.",
        ]
    ) + "\n"
    output.mkdir(parents=True, exist_ok=False)
    (output / "g1-residual-prototype-plant-holdout-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-residual-prototype-plant-holdout.npz",
        groups=groups,
        profile_supported=rehearsal["supported"],
        nearest_prototype_index=rehearsal["nearest"],
        nearest_distance_squared=rehearsal["distance"],
        envelopes=rehearsal["envelopes"],
        selections=rehearsal["selections"],
        selected_component_delta=selected_component,
        selected_aggregate_delta=selected_aggregate,
        component_covered=component_covered,
        aggregate_covered=aggregate_covered,
        law_index=law_index,
    )
    (output / "G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT.md").write_text(report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw one-shot holdout · r265"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "profile_transferred": profile_transferred,
                "distance_supported_rows": result["distance_supported_rows"],
                "nonzero_actions": result["nonzero_actions"],
                "holdout_consumed": True,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
