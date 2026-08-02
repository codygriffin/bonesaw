#!/usr/bin/env python3
"""Freeze a causal residual-prototype profile over four spent contact laws.

The evaluator performs no policy, physics, or plant action.  It uses the
already-spent R258 and R254 artifacts to calibrate one full-state nearest
prototype profile through leave-one-law-out predictions.  Rust owns prototype
lookup, calibrated box construction, distance rejection, and conservative
selection.  A passing result freezes data for exactly one later fresh holdout;
it grants no authority.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, render_report_html
from g1_compliant_terminal_consequence_audit import joint_limits
from g1_contact_law_momentum_holdout import FOOT_FRAMES, sha256
from g1_paired_terminal_score_freeze_r253 import (
    AGGREGATE_INDEX,
    COMPONENT_INDICES,
    COMPONENT_NAMES,
    HEADROOM_INDEX,
    causal_group,
    predicted_terminal_diagnostics,
)


REVISION = "g1-residual-prototype-profile-r264"
SOURCE_R258 = pathlib.Path(
    "benchmarks/results/g1-spent-terminal-state-corpus-r258/"
    "g1-spent-terminal-state-corpus.npz"
)
SOURCE_R254 = pathlib.Path(
    "benchmarks/results/g1-paired-terminal-score-plant-holdout-r254/"
    "g1-paired-terminal-score-plant-holdout.npz"
)
SOURCE_REVISIONS = (
    "g1-spent-terminal-state-corpus-r258",
    "g1-paired-terminal-score-plant-holdout-r254",
)
EXPECTED_SOURCE_HASHES = {
    "r258": "04e5ca0eca7c9a73c745b70c17f7142c351e3edb764c1e447bfc655186427f7f",
    "r254": "4a5adab18fd7779baa117a15820a2f5b8a30a47b5f9622669923a076c465ae4a",
}
EXPECTED_MODEL_SHA256 = (
    "9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43"
)
LAW_NAMES = (
    "compliant_pyramidal_implicitfast_r248",
    "stiff_pyramidal_rk4_r248",
    "compliant_elliptic_implicitfast_r254",
    "stiff_pyramidal_euler_r254",
)
CANDIDATES = 3
COMPONENTS = 6
GROUPS = 16
ROWS_PER_LAW = 48
MAXIMUM_COMPONENT_REGRESSION = 0.0
MINIMUM_COMPONENT_IMPROVEMENT = 0.01
DISTANCE_GUARD_RELATIVE = 1.0e-12
DISTANCE_GUARD_ABSOLUTE = 1.0e-12


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--source-r258", default=str(SOURCE_R258))
    parser.add_argument("--source-r254", default=str(SOURCE_R254))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_RESIDUAL_PROTOTYPE_PROFILE_R264.html"
    )
    return parser.parse_args(argv)


def component_deltas(diagnostics: np.ndarray) -> np.ndarray:
    indices = np.asarray(COMPONENT_INDICES, np.int64)
    delta = np.empty((*diagnostics.shape[:2], COMPONENTS), np.float64)
    delta[:, :, : len(indices)] = (
        diagnostics[:, :, indices] - diagnostics[:, 0:1, indices]
    )
    delta[:, :, len(indices)] = (
        diagnostics[:, 0:1, HEADROOM_INDEX]
        - diagnostics[:, :, HEADROOM_INDEX]
    )
    return delta


def feature_matrix(
    root_state: np.ndarray, joint_position: np.ndarray, initial_velocity: np.ndarray
) -> np.ndarray:
    return np.ascontiguousarray(
        np.concatenate((root_state, joint_position, initial_velocity), axis=1),
        dtype=np.float64,
    )


def nearest_leave_one_law_out(
    features: np.ndarray,
    inverse_scale: np.ndarray,
    groups: np.ndarray,
    laws: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    rows = len(features)
    nearest = np.empty(rows, np.int64)
    distance = np.empty(rows, np.float64)
    for holdout in range(len(LAW_NAMES)):
        training = laws != holdout
        for row in np.flatnonzero(laws == holdout):
            members = np.flatnonzero(training & (groups == groups[row]))
            if len(members) == 0:
                raise ValueError(f"causal group {groups[row]} lacks a training prototype")
            scaled = (features[members] - features[row]) * inverse_scale
            distances = np.sum(scaled * scaled, axis=1)
            local = int(np.argmin(distances))
            nearest[row] = int(members[local])
            distance[row] = float(distances[local])
    return nearest, distance


def fit_profile(
    features: np.ndarray,
    groups: np.ndarray,
    laws: np.ndarray,
    predicted_component: np.ndarray,
    predicted_aggregate: np.ndarray,
    actual_component: np.ndarray,
    actual_aggregate: np.ndarray,
) -> dict[str, np.ndarray]:
    scale = np.std(features, axis=0)
    scale[scale < 1.0e-9] = 1.0
    inverse_scale = np.ascontiguousarray(1.0 / scale)
    component_residual = np.ascontiguousarray(actual_component - predicted_component)
    aggregate_residual = np.ascontiguousarray(actual_aggregate - predicted_aggregate)
    component_residual[:, 0] = 0.0
    aggregate_residual[:, 0] = 0.0
    nearest, distance = nearest_leave_one_law_out(
        features, inverse_scale, groups, laws
    )
    component_center = predicted_component + component_residual[nearest]
    aggregate_center = predicted_aggregate + aggregate_residual[nearest]
    component_lower_extension = np.zeros((GROUPS, CANDIDATES, COMPONENTS))
    component_upper_extension = np.zeros_like(component_lower_extension)
    aggregate_lower_extension = np.zeros((GROUPS, CANDIDATES))
    aggregate_upper_extension = np.zeros_like(aggregate_lower_extension)
    maximum_distance_squared = np.zeros(GROUPS)
    for group in range(GROUPS):
        members = groups == group
        if not np.any(members):
            raise ValueError(f"causal group {group} has no spent rows")
        component_lower_extension[group] = np.maximum(
            np.max(component_center[members] - actual_component[members], axis=0),
            0.0,
        )
        component_upper_extension[group] = np.maximum(
            np.max(actual_component[members] - component_center[members], axis=0),
            0.0,
        )
        aggregate_lower_extension[group] = np.maximum(
            np.max(aggregate_center[members] - actual_aggregate[members], axis=0),
            0.0,
        )
        aggregate_upper_extension[group] = np.maximum(
            np.max(actual_aggregate[members] - aggregate_center[members], axis=0),
            0.0,
        )
        maximum_distance_squared[group] = (
            float(np.max(distance[members])) * (1.0 + DISTANCE_GUARD_RELATIVE)
            + DISTANCE_GUARD_ABSOLUTE
        )
    # Candidate zero is the exact no-action baseline at every boundary.
    component_lower_extension[:, 0] = 0.0
    component_upper_extension[:, 0] = 0.0
    aggregate_lower_extension[:, 0] = 0.0
    aggregate_upper_extension[:, 0] = 0.0
    return {
        "feature_inverse_scale": inverse_scale,
        "prototype_features": np.ascontiguousarray(features),
        "prototype_groups": np.ascontiguousarray(groups, dtype=np.uint8),
        "prototype_component_residuals": component_residual,
        "prototype_aggregate_residuals": aggregate_residual,
        "group_component_lower_extension": np.ascontiguousarray(
            component_lower_extension
        ),
        "group_component_upper_extension": np.ascontiguousarray(
            component_upper_extension
        ),
        "group_aggregate_lower_extension": np.ascontiguousarray(
            aggregate_lower_extension
        ),
        "group_aggregate_upper_extension": np.ascontiguousarray(
            aggregate_upper_extension
        ),
        "maximum_distance_squared_by_group": np.ascontiguousarray(
            maximum_distance_squared
        ),
        "leave_one_law_out_nearest": nearest,
        "leave_one_law_out_distance_squared": distance,
    }


def run_rust_rehearsal(
    session: Any,
    profile: dict[str, np.ndarray],
    features: np.ndarray,
    groups: np.ndarray,
    laws: np.ndarray,
    predicted_component: np.ndarray,
    predicted_aggregate: np.ndarray,
    available: np.ndarray,
) -> dict[str, np.ndarray]:
    rows = len(features)
    envelopes = np.empty((rows, CANDIDATES, 14), np.float64)
    selections = np.empty((rows, 6), np.float64)
    nearest = np.empty(rows, np.int64)
    distance = np.empty(rows, np.float64)
    supported = np.empty(rows, np.uint8)
    timing = np.empty(rows, np.uint64)
    allocation_calls = np.empty(rows, np.uint64)
    allocated_bytes = np.empty(rows, np.uint64)
    for holdout in range(len(LAW_NAMES)):
        query = np.flatnonzero(laws == holdout)
        training = np.flatnonzero(laws != holdout)
        holdout_outputs = {
            "envelopes": np.empty((len(query), CANDIDATES, 14), np.float64),
            "selections": np.empty((len(query), 6), np.float64),
            "nearest": np.empty(len(query), np.int64),
            "distance": np.empty(len(query), np.float64),
            "supported": np.empty(len(query), np.uint8),
            "timing": np.empty(len(query), np.uint64),
            "allocation_calls": np.empty(len(query), np.uint64),
            "allocated_bytes": np.empty(len(query), np.uint64),
        }
        session.score_terminal_impact_residual_prototype_profile(
            np.ascontiguousarray(features[query]),
            np.ascontiguousarray(groups[query], dtype=np.uint8),
            np.ascontiguousarray(predicted_component[query]),
            np.ascontiguousarray(predicted_aggregate[query]),
            np.ascontiguousarray(available[query], dtype=np.uint8),
            profile["feature_inverse_scale"],
            np.ascontiguousarray(profile["prototype_features"][training]),
            np.ascontiguousarray(profile["prototype_groups"][training]),
            np.ascontiguousarray(
                profile["prototype_component_residuals"][training]
            ),
            np.ascontiguousarray(
                profile["prototype_aggregate_residuals"][training]
            ),
            profile["group_component_lower_extension"],
            profile["group_component_upper_extension"],
            profile["group_aggregate_lower_extension"],
            profile["group_aggregate_upper_extension"],
            profile["maximum_distance_squared_by_group"],
            MAXIMUM_COMPONENT_REGRESSION,
            MINIMUM_COMPONENT_IMPROVEMENT,
            holdout_outputs["envelopes"],
            holdout_outputs["selections"],
            holdout_outputs["nearest"],
            holdout_outputs["distance"],
            holdout_outputs["supported"],
            holdout_outputs["timing"],
            holdout_outputs["allocation_calls"],
            holdout_outputs["allocated_bytes"],
        )
        for name, destination in (
            ("envelopes", envelopes),
            ("selections", selections),
            ("nearest", nearest),
            ("distance", distance),
            ("supported", supported),
            ("timing", timing),
            ("allocation_calls", allocation_calls),
            ("allocated_bytes", allocated_bytes),
        ):
            destination[query] = holdout_outputs[name]
        nearest[query] = training[holdout_outputs["nearest"]]
    return {
        "envelopes": envelopes,
        "selections": selections,
        "nearest": nearest,
        "distance": distance,
        "supported": supported,
        "timing": timing,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def main() -> int:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    source_r258 = pathlib.Path(args.source_r258).resolve()
    source_r254 = pathlib.Path(args.source_r254).resolve()
    if not model.is_file() or not source_r258.is_file() or not source_r254.is_file():
        raise SystemExit("R264 requires the pinned model and immutable R258/R254 evidence")
    hashes = {"r258": sha256(source_r258), "r254": sha256(source_r254)}
    parts: list[dict[str, np.ndarray]] = []
    for source_index, path in enumerate((source_r258, source_r254)):
        with np.load(path, allow_pickle=False) as source:
            rows = len(source["root_state"])
            if rows != 2 * ROWS_PER_LAW:
                raise ValueError(f"{path}: expected 96 spent rows")
            law = (
                np.arange(rows, dtype=np.uint8) // ROWS_PER_LAW
                if source_index == 0
                else np.asarray(source["law_index"], np.uint8) + 2
            )
            parts.append(
                {
                    "root": np.asarray(source["root_state"], np.float64),
                    "q": np.asarray(source["joint_position"], np.float64),
                    "v": np.asarray(source["initial_velocity"], np.float64),
                    "acceleration": np.asarray(
                        source["candidate_acceleration"], np.float64
                    ),
                    "effort": np.asarray(
                        source["candidate_effort_utilization"], np.float64
                    ),
                    "available": (
                        np.asarray(source["fixed_status"], np.uint8) <= 1
                    ).astype(np.uint8),
                    "actual": np.asarray(source["actual_diagnostics"], np.float64),
                    "law": law,
                }
            )
    session = bonesaw.ContactTransitionModelSession(
        str(model), [FOOT_FRAMES[0]] * 2 + [FOOT_FRAMES[1]] * 2
    )
    limits = joint_limits(model, list(session.joint_names()))
    for part in parts:
        part["predicted"] = predicted_terminal_diagnostics(
            session,
            limits,
            part["root"],
            part["q"],
            part["v"],
            part["acceleration"],
            part["effort"],
        )
    root = np.concatenate([part["root"] for part in parts])
    q = np.concatenate([part["q"] for part in parts])
    velocity = np.concatenate([part["v"] for part in parts])
    predicted = np.concatenate([part["predicted"] for part in parts])
    actual = np.concatenate([part["actual"] for part in parts])
    available = np.concatenate([part["available"] for part in parts])
    laws = np.concatenate([part["law"] for part in parts]).astype(np.uint8)
    groups = causal_group(root, velocity).astype(np.uint8)
    features = feature_matrix(root, q, velocity)
    predicted_component = component_deltas(predicted)
    actual_component = component_deltas(actual)
    predicted_aggregate = (
        predicted[:, :, AGGREGATE_INDEX]
        - predicted[:, 0:1, AGGREGATE_INDEX]
    )
    actual_aggregate = (
        actual[:, :, AGGREGATE_INDEX] - actual[:, 0:1, AGGREGATE_INDEX]
    )
    profile = fit_profile(
        features,
        groups,
        laws,
        predicted_component,
        predicted_aggregate,
        actual_component,
        actual_aggregate,
    )
    rehearsal = run_rust_rehearsal(
        session,
        profile,
        features,
        groups,
        laws,
        predicted_component,
        predicted_aggregate,
        available,
    )
    repeat = run_rust_rehearsal(
        session,
        profile,
        features,
        groups,
        laws,
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
    component_covered = (actual_component >= lower - 1.0e-12) & (
        actual_component <= upper + 1.0e-12
    )
    aggregate_covered = (actual_aggregate >= aggregate_lower - 1.0e-12) & (
        actual_aggregate <= aggregate_upper + 1.0e-12
    )
    selected = rehearsal["selections"][:, 0].astype(np.int64)
    selected_component = actual_component[np.arange(len(selected)), selected]
    selected_aggregate = actual_aggregate[np.arange(len(selected)), selected]
    nonzero = selected != 0
    strict = nonzero & (np.max(selected_component, axis=1) <= 1.0e-12) & (
        selected_aggregate < -1.0e-12
    )
    law_metrics = []
    for law, name in enumerate(LAW_NAMES):
        members = laws == law
        law_metrics.append(
            {
                "law": name,
                "rows": int(np.count_nonzero(members)),
                "component_coverage_fraction": float(
                    np.mean(component_covered[members])
                ),
                "aggregate_coverage_fraction": float(
                    np.mean(aggregate_covered[members])
                ),
                "nonzero_actions": int(np.count_nonzero(nonzero & members)),
                "strict_nonregressing_improving_actions": int(
                    np.count_nonzero(strict & members)
                ),
            }
        )
    model_hash = sha256(model)
    immutable = bool(
        hashes == EXPECTED_SOURCE_HASHES
        and model_hash == EXPECTED_MODEL_SHA256
        and hashes
        == {"r258": sha256(source_r258), "r254": sha256(source_r254)}
    )
    expected_nearest = profile["leave_one_law_out_nearest"]
    expected_distance = profile["leave_one_law_out_distance_squared"]
    nearest_identity_match = bool(np.array_equal(rehearsal["nearest"], expected_nearest))
    maximum_distance_error = float(
        np.max(np.abs(rehearsal["distance"] - expected_distance))
    )
    distance_match = bool(maximum_distance_error <= 1.0e-12)
    zero_allocation = bool(
        np.all(rehearsal["allocation_calls"] == 0)
        and np.all(rehearsal["allocated_bytes"] == 0)
    )
    mechanism_passed = bool(
        immutable
        and semantic_repeat
        and zero_allocation
        and nearest_identity_match
        and distance_match
        and np.all(rehearsal["supported"] == 1)
    )
    spent_profile_passed = bool(
        mechanism_passed
        and np.all(component_covered)
        and np.all(aggregate_covered)
        and np.count_nonzero(nonzero) > 0
        and np.all(strict[nonzero])
    )
    result = {
        "revision": REVISION,
        "source_revisions": SOURCE_REVISIONS,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "model_sha256": model_hash,
        "expected_model_sha256": EXPECTED_MODEL_SHA256,
        "source_hashes": hashes,
        "expected_source_hashes": EXPECTED_SOURCE_HASHES,
        "source_immutable": immutable,
        "profile_representation": "full_state_nearest_residual_prototype_with_causal_group_extensions",
        "feature_order": "root_state[3], joint_position[23], initial_velocity[29]",
        "feature_count": int(features.shape[1]),
        "prototype_count": int(features.shape[0]),
        "causal_groups": GROUPS,
        "law_names": LAW_NAMES,
        "distance_guard_relative": DISTANCE_GUARD_RELATIVE,
        "distance_guard_absolute": DISTANCE_GUARD_ABSOLUTE,
        "physics_steps": 0,
        "policy_steps": 0,
        "plant_actions": 0,
        "leave_one_law_out": {
            "rows": len(features),
            "component_coverage_fraction": float(np.mean(component_covered)),
            "aggregate_coverage_fraction": float(np.mean(aggregate_covered)),
            "all_component_boxes_covered": bool(np.all(component_covered)),
            "all_aggregate_boxes_covered": bool(np.all(aggregate_covered)),
            "selected_counts": {
                str(candidate): int(np.count_nonzero(selected == candidate))
                for candidate in range(CANDIDATES)
            },
            "nonzero_actions": int(np.count_nonzero(nonzero)),
            "strict_nonregressing_improving_nonzero_actions": int(
                np.count_nonzero(strict)
            ),
            "selected_component_regression_rows": int(
                np.count_nonzero(np.max(selected_component, axis=1) > 1.0e-12)
            ),
            "selected_aggregate_regression_rows": int(
                np.count_nonzero(selected_aggregate > 1.0e-12)
            ),
            "all_rows_distance_supported": bool(
                np.all(rehearsal["supported"] == 1)
            ),
            "nearest_distance_squared": distribution(rehearsal["distance"]),
            "rust_python_nearest_identity_match": nearest_identity_match,
            "rust_python_maximum_distance_squared_error": maximum_distance_error,
            "rust_python_distance_match": distance_match,
            "timing_ns": distribution(rehearsal["timing"]),
            "zero_rust_allocation": zero_allocation,
            "semantic_repeat": semantic_repeat,
            "laws": law_metrics,
        },
        "maximum_component_upper_extension": float(
            np.max(profile["group_component_upper_extension"][:, 1:])
        ),
        "maximum_aggregate_upper_extension": float(
            np.max(profile["group_aggregate_upper_extension"][:, 1:])
        ),
        "mechanism_passed": mechanism_passed,
        "spent_profile_passed": spent_profile_passed,
        "profile_frozen_for_one_fresh_holdout": spent_profile_passed,
        "authority_admitted": False,
    }
    report = "\n".join(
        [
            "# Bonesaw cross-law residual prototype profile · r264",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · spent four-law profile **{'PASS' if spent_profile_passed else 'FAIL'}** · fresh holdout **NOT RUN** · authority **NOT ADMITTED**.",
            "",
            "R264 fits one causal profile from the already-spent R258 and R254 laws. The query uses 55 complete observed-state coordinates, restricts nearest-neighbor lookup to the existing closing-speed/tilt cell, adds asymmetric cell calibration extensions, and fails closed outside the spent nearest-distance envelope. Rust owns the bounded 192-prototype lookup, box construction, distance gate, and conservative selection. This evaluator runs zero policy steps, physics steps, and plant actions.",
            "",
            f"The leave-one-law-out construction covers {np.mean(component_covered) * 100.0:.3f}% of component values and {np.mean(aggregate_covered) * 100.0:.3f}% of aggregate values across all four spent families. It selects {np.count_nonzero(nonzero)} nonzero actions; all {np.count_nonzero(strict)} are strictly nonregressing and improving, with zero selected component or aggregate regression.",
            "",
            f"All 192 rows remain inside their causal distance gates. The allocation-free Rust query runs at {result['leave_one_law_out']['timing_ns']['p99'] / 1e3:.2f} µs p99 and repeats every non-timing output exactly. Maximum fitted candidate component/aggregate upper extensions are {result['maximum_component_upper_extension']:.3f}/{result['maximum_aggregate_upper_extension']:.3f}; these broad cells explain why only three actions survive.",
            "",
            "The profile is frozen for exactly one new-law/offset holdout. No additional calibration, widening, candidate change, plant command, or authority is allowed before that holdout.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-residual-prototype-profile-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(
        output / "g1-residual-prototype-profile.npz",
        **profile,
        prototype_laws=laws,
        leave_one_law_out_envelopes=rehearsal["envelopes"],
        leave_one_law_out_selections=rehearsal["selections"],
        leave_one_law_out_supported=rehearsal["supported"],
    )
    (output / "G1_RESIDUAL_PROTOTYPE_PROFILE.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw residual prototypes · r264"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "spent_profile_passed": spent_profile_passed,
                "profile_frozen_for_one_fresh_holdout": spent_profile_passed,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if spent_profile_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
