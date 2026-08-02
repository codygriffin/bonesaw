#!/usr/bin/env python3
"""R251 spent-state paired candidate-vs-zero residual design audit.

R250 rejected a global componentwise velocity tube because the baseline and
candidate residuals were treated as independent boxes.  This audit measures
the next *causal* primitive: a paired residual for candidate ``c``

    (v_actual[c] - v_actual[zero])
      - (v_predicted[c] - v_predicted[zero]).

The state partition is frozen from pre-action observations (root tilt, root
height, and measured root angular-rate signs).  Completed plant velocities are
used only to fit and describe spent design tubes; they never enter a selector,
candidate choice, or authority decision.  Consequently this evaluator can
show whether correlation is worth implementing in Rust, but it cannot freeze
an action for a fresh holdout.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-paired-terminal-delta-audit-r251"
SOURCE_REVISION = "g1-actuator-bandwidth-action-freeze-r250"
SOURCE_METRICS = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze-metrics.json"
)
SOURCE_REPLAY = pathlib.Path(
    "benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/"
    "g1-actuator-bandwidth-action-freeze.npz"
)
PREFIX = "bandwidth_25hz_slew_1000_nm_s_"
FAMILY_NAMES = (
    "zero_wbc_and_velocity_damping",
    "zero_wbc_and_neutral_recovery",
)
CANDIDATE_NAMES = ("bandwidth_zero_wbc", "bandwidth_third_law")
CANDIDATE_INDICES = (1, 2)
SAMPLES_PER_LAW = 48
TOTAL_SAMPLES = 2 * SAMPLES_PER_LAW
GENERALIZED_DOF = 29
CONTROL_DT_S = 0.020
# These bins are a declaration of the online feature contract, not fitted
# quantiles.  They are deliberately coarse so a fresh state cannot be mapped
# by a hidden sample index or a completed contact label.
ROOT_HEIGHT_BINS_M = (0.382, 0.388, 0.392)
MIN_GROUP_SAMPLES = 4
HARM_INDICES = tuple(range(8, 17))
HEADROOM_INDEX = 6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-metrics", default=str(SOURCE_METRICS))
    parser.add_argument("--source-replay", default=str(SOURCE_REPLAY))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_PAIRED_TERMINAL_DELTA_AUDIT_R251.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_family_arrays(replay: Any, family: str) -> dict[str, np.ndarray]:
    prefix = PREFIX + family + "_"
    names = (
        "root_state",
        "initial_velocity",
        "candidate_acceleration",
        "actual_velocity",
        "actual_diagnostics",
    )
    result: dict[str, np.ndarray] = {}
    for name in names:
        key = prefix + name
        if key not in replay:
            raise KeyError(f"R250 replay is missing {key}")
        result[name] = np.asarray(replay[key], dtype=np.float64).copy()
    return result


def group_schemes(
    root_state: np.ndarray, initial_velocity: np.ndarray
) -> dict[str, np.ndarray]:
    """Return fixed causal group ids from observations available at the tick."""

    roll = root_state[:, 1] >= 0.0
    pitch = root_state[:, 2] >= 0.0
    roll_rate = initial_velocity[:, 0] >= 0.0
    pitch_rate = initial_velocity[:, 1] >= 0.0
    height = np.digitize(root_state[:, 0], ROOT_HEIGHT_BINS_M, right=False)
    return {
        "global": np.zeros(len(root_state), dtype=np.int64),
        "tilt_quadrant": (roll.astype(np.int64) * 2 + pitch.astype(np.int64)),
        "tilt_rate_quadrant": (
            roll.astype(np.int64) * 8
            + pitch.astype(np.int64) * 4
            + roll_rate.astype(np.int64) * 2
            + pitch_rate.astype(np.int64)
        ),
        "height_tilt_quadrant": (
            height.astype(np.int64) * 4
            + roll.astype(np.int64) * 2
            + pitch.astype(np.int64)
        ),
    }


def group_bounds(
    residual: np.ndarray, groups: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit one frozen componentwise tube per group, with a small-group fallback."""

    if residual.ndim != 2 or residual.shape[1] != GENERALIZED_DOF:
        raise ValueError("residual must have shape [N,29]")
    global_lower = residual.min(axis=0)
    global_upper = residual.max(axis=0)
    lower = np.empty_like(residual)
    upper = np.empty_like(residual)
    members_count = np.empty(len(residual), dtype=np.int64)
    for row, group in enumerate(groups):
        members = np.flatnonzero(groups == group)
        if len(members) < MIN_GROUP_SAMPLES:
            members = np.arange(len(residual))
        lower[row] = residual[members].min(axis=0)
        upper[row] = residual[members].max(axis=0)
        members_count[row] = len(members)
    # The fallback is intentionally observable in the report.  Keeping the
    # global values here also makes accidental empty-group behavior impossible.
    bad_lower = ~np.isfinite(lower)
    bad_upper = ~np.isfinite(upper)
    lower[bad_lower] = np.broadcast_to(global_lower, lower.shape)[bad_lower]
    upper[bad_upper] = np.broadcast_to(global_upper, upper.shape)[bad_upper]
    return lower, upper, members_count


def component_delta(
    actual_diagnostics: np.ndarray, candidate: int
) -> np.ndarray:
    """Offline label-only candidate-vs-zero harm delta for reporting."""

    baseline = actual_diagnostics[:, 0]
    candidate_diagnostics = actual_diagnostics[:, candidate]
    pressure = candidate_diagnostics[:, HARM_INDICES] - baseline[:, HARM_INDICES]
    headroom = baseline[:, HEADROOM_INDEX] - candidate_diagnostics[:, HEADROOM_INDEX]
    return np.concatenate((pressure, headroom[:, None]), axis=1)


def summarize_family(
    family: str, arrays: dict[str, np.ndarray]
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    root_state = arrays["root_state"]
    initial_velocity = arrays["initial_velocity"]
    predicted = initial_velocity[:, None, :] + CONTROL_DT_S * arrays[
        "candidate_acceleration"
    ]
    actual = arrays["actual_velocity"]
    residual = actual - predicted
    paired_residual = np.empty((TOTAL_SAMPLES, len(CANDIDATE_INDICES), GENERALIZED_DOF))
    for slot, candidate in enumerate(CANDIDATE_INDICES):
        paired_residual[:, slot] = (
            (actual[:, candidate] - actual[:, 0])
            - (predicted[:, candidate] - predicted[:, 0])
        )
    schemes = group_schemes(root_state, initial_velocity)
    rows: list[dict[str, Any]] = []
    stored: dict[str, np.ndarray] = {}
    for scheme_name, groups in schemes.items():
        group_count = len(np.unique(groups))
        group_sizes = np.asarray(
            [np.count_nonzero(groups == group) for group in np.unique(groups)],
            dtype=np.int64,
        )
        for slot, candidate in enumerate(CANDIDATE_INDICES):
            lower, upper, members_count = group_bounds(
                paired_residual[:, slot], groups
            )
            span = upper - lower
            independent_span = (
                residual[:, candidate].max(axis=0)
                - residual[:, candidate].min(axis=0)
                + residual[:, 0].max(axis=0)
                - residual[:, 0].min(axis=0)
            )
            reduction = np.where(
                independent_span > 1.0e-12,
                1.0 - np.max(span, axis=0) / independent_span,
                0.0,
            )
            # Every source row is contained by construction.  This is a
            # design-mechanism check, not a fresh-law coverage claim.
            covered = np.all(
                (paired_residual[:, slot] >= lower - 1.0e-12)
                & (paired_residual[:, slot] <= upper + 1.0e-12)
            )
            harm_delta = component_delta(arrays["actual_diagnostics"], candidate)
            rows.append(
                {
                    "family": family,
                    "scheme": scheme_name,
                    "candidate": CANDIDATE_NAMES[slot],
                    "group_count": group_count,
                    "group_size_min": int(group_sizes.min()),
                    "group_size_max": int(group_sizes.max()),
                    "fallback_rows": int(np.count_nonzero(members_count == TOTAL_SAMPLES)),
                    "paired_residual_span": distribution(span.reshape(-1)),
                    "paired_residual_max_span": float(np.max(span)),
                    "paired_residual_p95_span": float(np.percentile(span, 95.0)),
                    "independent_span_max": float(np.max(independent_span)),
                    "max_span_reduction_fraction": float(np.max(reduction)),
                    "median_span_reduction_fraction": float(np.median(reduction)),
                    "source_rows_covered": bool(covered),
                    "label_only_actual_regression_rows": int(
                        np.count_nonzero(np.max(harm_delta, axis=1) > 1.0e-12)
                    ),
                    "label_only_actual_improvement_rows": int(
                        np.count_nonzero(
                            np.max(-harm_delta, axis=1) > 1.0e-12
                        )
                    ),
                    "label_only_aggregate_delta": distribution(
                        arrays["actual_diagnostics"][:, candidate, 16]
                        - arrays["actual_diagnostics"][:, 0, 16]
                    ),
                }
            )
            prefix = f"{family}_{scheme_name}_{CANDIDATE_NAMES[slot]}"
            stored.update(
                {
                    f"{prefix}_group": groups,
                    f"{prefix}_paired_lower": lower,
                    f"{prefix}_paired_upper": upper,
                    f"{prefix}_paired_span": span,
                    f"{prefix}_members_count": members_count,
                }
            )
    summary = {
        "family": family,
        "samples": len(root_state),
        "candidate_names": CANDIDATE_NAMES,
        "paired_residual_max_abs": float(np.max(np.abs(paired_residual))),
        "paired_residual_p95_abs": float(np.percentile(np.abs(paired_residual), 95.0)),
        "schemes": rows,
    }
    stored[f"{family}_predicted_velocity"] = predicted
    stored[f"{family}_residual"] = residual
    stored[f"{family}_paired_residual"] = paired_residual
    return summary, stored


def main() -> int:
    args = parse_args()
    source_metrics_path = pathlib.Path(args.source_metrics).resolve()
    source_replay_path = pathlib.Path(args.source_replay).resolve()
    if not source_metrics_path.is_file() or not source_replay_path.is_file():
        raise SystemExit("R251 requires immutable R250 metrics and replay")
    source_metrics = json.loads(source_metrics_path.read_text())
    if source_metrics.get("revision") != SOURCE_REVISION:
        raise ValueError("R251 source revision mismatch")
    source_metrics_hash = sha256(source_metrics_path)
    source_replay_hash = sha256(source_replay_path)
    with np.load(source_replay_path) as replay:
        summaries: list[dict[str, Any]] = []
        stored: dict[str, np.ndarray] = {}
        for family in FAMILY_NAMES:
            summary, family_stored = summarize_family(
                family, load_family_arrays(replay, family)
            )
            summaries.append(summary)
            stored.update(family_stored)
    source_immutable = (
        source_metrics_hash == sha256(source_metrics_path)
        and source_replay_hash == sha256(source_replay_path)
    )
    finite = all(
        np.all(np.isfinite(value))
        for value in stored.values()
        if np.issubdtype(value.dtype, np.number)
    )
    mechanism_passed = bool(source_immutable and finite)
    # There is intentionally no selector invocation here.  A paired residual
    # is not representable by R250's independent candidate boxes, so this
    # design audit cannot freeze a fresh action profile.
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_revision": SOURCE_REVISION,
        "source_metrics_sha256": source_metrics_hash,
        "source_replay_sha256": source_replay_hash,
        "source_immutable": source_immutable,
        "design_audit_not_holdout": True,
        "feature_contract": {
            "uses_completed_plant_labels_for_features": False,
            "uses_completed_plant_labels_for_spent_fit": True,
            "uses_policy": False,
            "uses_sample_index": False,
            "uses_contact_label": False,
            "root_height_bins_m": ROOT_HEIGHT_BINS_M,
            "minimum_group_samples": MIN_GROUP_SAMPLES,
            "fields": [
                "root_state[height,roll,pitch]",
                "initial_velocity[root_roll_rate,root_pitch_rate]",
            ],
        },
        "paired_residual_definition": "(actual_candidate - actual_zero) - (predicted_candidate - predicted_zero)",
        "control_dt_seconds": CONTROL_DT_S,
        "samples_per_law": SAMPLES_PER_LAW,
        "samples": TOTAL_SAMPLES,
        "physics_steps": 0,
        "policy_steps": 0,
        "rust_selector_queries": 0,
        "mechanism_passed": mechanism_passed,
        "action_profile_frozen_for_fresh_holdout": False,
        "authority_admitted": False,
        "next_gate": "implement a Rust-owned paired terminal-delta bound/selector, then rerun on a new law/offset holdout without refitting",
        "families": summaries,
    }
    rows = []
    for summary in summaries:
        for row in summary["schemes"]:
            rows.append(
                [
                    row["family"].replace("zero_wbc_and_", ""),
                    row["scheme"],
                    row["candidate"],
                    str(row["group_count"]),
                    f"{row['paired_residual_p95_span']:.4g}",
                    f"{row['paired_residual_max_span']:.4g}",
                    f"{row['independent_span_max']:.4g}",
                    f"{row['median_span_reduction_fraction']:.3f}",
                    f"{row['label_only_actual_improvement_rows']}/{row['label_only_actual_regression_rows']}",
                ]
            )
    report = "\n".join(
        [
            "# Bonesaw paired terminal-delta audit · r251",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · action profile **NOT FROZEN** · authority **NOT ADMITTED**.",
            "",
            "R250's global candidate boxes force zero because baseline and candidate residuals are independent. R251 measures a paired residual around candidate-vs-zero velocity change. Group ids are computed only from pre-action height, tilt, and root angular-rate signs using fixed bins; no completed contact, sample index, policy, or selector result is observed by the feature contract.",
            "",
            "The table's aggregate/improvement counts are completed-plant labels for diagnosis only. They do not choose an action. A paired residual cannot be consumed by the existing independent-box Rust selector, so this artifact deliberately freezes no profile and performs zero physics steps. The next implementation gate is a Rust-owned paired bound that proves candidate-vs-zero harm under the shared uncertainty, followed by a new law/offset holdout with no refit.",
            "",
            *markdown_table(
                [
                    "family",
                    "causal grouping",
                    "candidate",
                    "groups",
                    "paired span p95",
                    "paired span max",
                    "independent max",
                    "median reduction",
                    "label improved / regressed",
                ],
                rows,
            ),
            "",
            "All source rows are covered by their fitted spent-state tube by construction; that is a mechanism check, not a transferable coverage claim. This report is design evidence only and leaves authority closed.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-paired-terminal-delta-audit-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    np.savez_compressed(output / "g1-paired-terminal-delta-audit.npz", **stored)
    (output / "G1_PAIRED_TERMINAL_DELTA_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report, title="Bonesaw paired terminal delta · r251"))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "action_profile_frozen_for_fresh_holdout": False,
                "authority_admitted": False,
                "families": len(summaries),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
