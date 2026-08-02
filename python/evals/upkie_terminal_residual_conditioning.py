#!/usr/bin/env python3
"""R196 leave-one-case-out conditioning of terminal realization residuals."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any, Callable

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_support_hypothesis_envelope_ab import PRESSURE_NAMES
from upkie_inexact_terminal_chooser_ab import model_limits
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-terminal-residual-conditioning-r196"
DURATION_S = 6.0
PROFILE_CONFIGS = {
    "drop5": DROP5,
    "drop10": DROP10,
    "drop5_matched": DROP5_MATCHED,
}
METHOD_ORDER = (
    "global_max",
    "global_max_1p25",
    "action_max",
    "causal_knn32_action_max",
    "causal_lipschitz_action",
    "oracle_support_action_max",
)
DEPLOYABLE_METHODS = (
    "global_max",
    "action_max",
    "causal_knn32_action_max",
    "causal_lipschitz_action",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_TERMINAL_RESIDUAL_CONDITIONING_R196.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def causal_feature(
    state: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    velocity_limit: np.ndarray,
    envelope_pressure: np.ndarray,
    hypothesis_pressure: np.ndarray,
    effort_utilization: float,
) -> np.ndarray:
    """Fixed-scale feature using only evidence present at the selection tick."""
    state_scale = np.asarray([0.225, 5.0, 0.7, 0.7, 10.0, 10.0])
    finite_velocity_limit = np.where(
        np.isfinite(velocity_limit) & (velocity_limit > 0.0),
        velocity_limit,
        1.0,
    )
    posture = q[np.asarray([0, 1, 3, 4])] / 2.5
    hypothesis_range = np.max(hypothesis_pressure, axis=0) - np.min(
        hypothesis_pressure, axis=0
    )
    return np.concatenate(
        (
            state / state_scale,
            posture,
            v / finite_velocity_limit,
            envelope_pressure,
            hypothesis_range,
            np.asarray([effort_utilization]),
        )
    )


def collect_samples(
    case_name: str,
    profile_name: str,
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, Any]:
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    names = tuple(balance.terminal_impact_diagnostic_names)
    name_index = {name: index for index, name in enumerate(names)}
    pressure_indices = np.asarray(
        [name_index[name] for name in PRESSURE_NAMES], np.int64
    )
    actions = np.asarray(
        trace["inexact_observation_terminal_selector_action"], np.uint8
    )[ticks]
    physical_contact_all = np.asarray(
        trace["physical_contact_active"], np.uint8
    )
    physical_contact = physical_contact_all[ticks]
    support_masks = physical_contact[:, 0] + 2 * physical_contact[:, 1]
    support_masks_all = physical_contact_all[:, 0] + 2 * physical_contact_all[:, 1]
    previous_support_masks = support_masks_all[np.maximum(ticks - 1, 0)]
    next_support_masks = support_masks_all[
        np.minimum(ticks + 1, len(support_masks_all) - 1)
    ]
    state = np.asarray(
        trace["inexact_observation_terminal_selector_state"], np.float64
    )
    q = np.asarray(trace["q"], np.float64)
    v = np.asarray(trace["v"], np.float64)
    root_twist = np.asarray(trace["root_twist"], np.float64)
    post_root_twist = np.asarray(trace["post_root_twist"], np.float64)
    post_v = np.asarray(trace["post_v"], np.float64)
    hypothesis_acceleration = np.asarray(
        trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
    )
    hypothesis_available = np.asarray(
        trace["inexact_observation_terminal_hypothesis_available"], np.uint8
    )
    hypothesis_diagnostics = np.asarray(
        trace["inexact_observation_terminal_hypothesis_diagnostics"], np.float64
    )
    envelopes = np.asarray(
        trace["inexact_observation_terminal_hypothesis_envelopes"], np.float64
    )
    effort = np.asarray(
        trace["inexact_observation_terminal_hypothesis_effort_utilization"],
        np.float64,
    )
    diagnostics = np.empty((3, len(names)), np.float64)
    selection = np.empty(6, np.float64)
    candidate_root = np.empty((3, 2), np.float64)
    candidate_joint = np.empty((3, 6), np.float64)
    candidate_available = np.empty(3, np.uint8)
    features: list[np.ndarray] = []
    signed_residual = np.empty((len(ticks), len(PRESSURE_NAMES)), np.float64)
    envelope_pressure = np.empty_like(signed_residual)
    predicted_acceleration = np.empty((len(ticks), 8), np.float64)
    realized_acceleration = np.empty_like(predicted_acceleration)
    score_ns = np.empty(len(ticks), np.uint64)
    allocation_calls = np.empty(len(ticks), np.uint64)
    allocated_bytes = np.empty(len(ticks), np.uint64)
    for cursor, (tick, action, support_mask) in enumerate(
        zip(ticks, actions, support_masks, strict=True)
    ):
        action = int(action)
        support_mask = int(support_mask)
        candidate_root[:] = hypothesis_acceleration[tick, support_mask, :, :2]
        candidate_joint[:] = hypothesis_acceleration[tick, support_mask, :, 6:]
        candidate_available[:] = hypothesis_available[tick, support_mask]
        predicted_acceleration[cursor, :2] = candidate_root[action]
        predicted_acceleration[cursor, 2:] = candidate_joint[action]
        realized_acceleration[cursor, :2] = (
            post_root_twist[tick, :2] - root_twist[tick, :2]
        ) / CONTROL_DT
        realized_acceleration[cursor, 2:] = (post_v[tick] - v[tick]) / CONTROL_DT
        candidate_root[action] = realized_acceleration[cursor, :2]
        candidate_joint[action] = realized_acceleration[cursor, 2:]
        timing = balance.score_terminal_impact_candidates(
            state[tick],
            q[tick],
            v[tick],
            lower,
            upper,
            velocity_limit,
            candidate_available,
            candidate_root,
            candidate_joint,
            effort[tick],
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )
        score_ns[cursor], allocation_calls[cursor], allocated_bytes[cursor] = timing
        envelope_pressure[cursor] = envelopes[tick, action, pressure_indices]
        signed_residual[cursor] = (
            diagnostics[action, pressure_indices] - envelope_pressure[cursor]
        )
        features.append(
            causal_feature(
                state[tick],
                q[tick],
                v[tick],
                velocity_limit,
                envelope_pressure[cursor],
                hypothesis_diagnostics[tick, action, :, pressure_indices],
                float(effort[tick, action]),
            )
        )
    return {
        "case": np.full(len(ticks), case_name, object),
        "profile": np.full(len(ticks), profile_name, object),
        "ticks": ticks,
        "actions": actions,
        "support_masks": support_masks,
        "previous_support_masks": previous_support_masks,
        "next_support_masks": next_support_masks,
        "features": np.asarray(features, np.float64),
        "signed_residual": signed_residual,
        "positive_residual": np.maximum(signed_residual, 0.0),
        "envelope_pressure": envelope_pressure,
        "predicted_acceleration": predicted_acceleration,
        "realized_acceleration": realized_acceleration,
        "acceleration_error": realized_acceleration - predicted_acceleration,
        "score_ns": score_ns,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def concatenate_samples(sample_sets: list[dict[str, Any]]) -> dict[str, Any]:
    keys = tuple(sample_sets[0])
    return {
        key: np.concatenate([samples[key] for samples in sample_sets], axis=0)
        for key in keys
    }


def global_max_bound(train: dict[str, Any], test: dict[str, Any]) -> np.ndarray:
    bound = np.max(train["positive_residual"], axis=0)
    return np.broadcast_to(bound, test["positive_residual"].shape).copy()


def grouped_max_bound(
    train: dict[str, Any],
    test: dict[str, Any],
    group: Callable[[dict[str, Any], int], tuple[int, ...]],
) -> np.ndarray:
    fallback = np.max(train["positive_residual"], axis=0)
    bounds = np.empty_like(test["positive_residual"])
    train_groups: dict[tuple[int, ...], list[int]] = {}
    for index in range(len(train["positive_residual"])):
        train_groups.setdefault(group(train, index), []).append(index)
    for index in range(len(bounds)):
        members = train_groups.get(group(test, index))
        bounds[index] = (
            np.max(train["positive_residual"][members], axis=0)
            if members
            else fallback
        )
    return bounds


def action_group(samples: dict[str, Any], index: int) -> tuple[int, ...]:
    return (int(samples["actions"][index]),)


def support_action_group(samples: dict[str, Any], index: int) -> tuple[int, ...]:
    return (
        int(samples["support_masks"][index]),
        int(samples["actions"][index]),
    )


def action_members(train: dict[str, Any], action: int) -> np.ndarray:
    members = np.flatnonzero(train["actions"] == action)
    return members if len(members) else np.arange(len(train["actions"]))


def knn_action_max_bound(
    train: dict[str, Any], test: dict[str, Any], neighbors: int = 32
) -> np.ndarray:
    bounds = np.empty_like(test["positive_residual"])
    dimensions = train["features"].shape[1]
    for index, feature in enumerate(test["features"]):
        members = action_members(train, int(test["actions"][index]))
        distance = np.linalg.norm(train["features"][members] - feature, axis=1) / math.sqrt(
            dimensions
        )
        count = min(neighbors, len(members))
        nearest = members[np.argpartition(distance, count - 1)[:count]]
        bounds[index] = np.max(train["positive_residual"][nearest], axis=0)
    return bounds


def lipschitz_constants(features: np.ndarray, residual: np.ndarray) -> np.ndarray:
    constants = np.zeros(residual.shape[1], np.float64)
    for left in range(len(features)):
        distance = np.linalg.norm(features[left + 1 :] - features[left], axis=1)
        valid = distance > 1.0e-12
        if not np.any(valid):
            continue
        slope = np.abs(residual[left + 1 :][valid] - residual[left]) / distance[
            valid, None
        ]
        constants = np.maximum(constants, np.max(slope, axis=0))
    return constants


def lipschitz_action_bound(train: dict[str, Any], test: dict[str, Any]) -> np.ndarray:
    bounds = np.empty_like(test["positive_residual"])
    cache: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for action in range(3):
        members = action_members(train, action)
        cache[action] = (
            members,
            lipschitz_constants(
                train["features"][members], train["positive_residual"][members]
            ),
        )
    for index, feature in enumerate(test["features"]):
        members, constants = cache[int(test["actions"][index])]
        distance = np.linalg.norm(train["features"][members] - feature, axis=1)
        cones = train["positive_residual"][members] + distance[:, None] * constants
        bounds[index] = np.min(cones, axis=0)
    return bounds


def predict_bounds(
    method: str, train: dict[str, Any], test: dict[str, Any]
) -> np.ndarray:
    if method == "global_max":
        return global_max_bound(train, test)
    if method == "global_max_1p25":
        return 1.25 * global_max_bound(train, test)
    if method == "action_max":
        return grouped_max_bound(train, test, action_group)
    if method == "causal_knn32_action_max":
        return knn_action_max_bound(train, test)
    if method == "causal_lipschitz_action":
        return lipschitz_action_bound(train, test)
    if method == "oracle_support_action_max":
        return grouped_max_bound(train, test, support_action_group)
    raise ValueError(f"unknown method: {method}")


def score_bound(residual: np.ndarray, bound: np.ndarray) -> dict[str, Any]:
    exceedance = np.maximum(residual - bound, 0.0)
    sample_covered = np.all(exceedance <= 1.0e-12, axis=1)
    return {
        "sample_count": int(len(residual)),
        "covered_samples": int(np.sum(sample_covered)),
        "sample_coverage": float(np.mean(sample_covered)),
        "component_coverage": float(np.mean(exceedance <= 1.0e-12)),
        "maximum_exceedance": float(np.max(exceedance)),
        "maximum_exceedance_by_pressure": np.max(exceedance, axis=0).tolist(),
        "bound_norm": distribution(np.linalg.norm(bound, axis=1)),
        "bound_maximum_component": distribution(np.max(bound, axis=1)),
    }


def leave_one_case_out(
    samples: dict[str, Any], case_names: tuple[str, ...]
) -> dict[str, Any]:
    methods: dict[str, dict[str, Any]] = {}
    for method in METHOD_ORDER:
        fold_metrics: dict[str, Any] = {}
        residual_parts: list[np.ndarray] = []
        bound_parts: list[np.ndarray] = []
        for holdout in case_names:
            held_mask = samples["case"] == holdout
            train_mask = ~held_mask
            train = {key: values[train_mask] for key, values in samples.items()}
            held = {key: values[held_mask] for key, values in samples.items()}
            bound = predict_bounds(method, train, held)
            fold = score_bound(held["positive_residual"], bound)
            exceedance = np.maximum(held["positive_residual"] - bound, 0.0)
            sample_index, pressure_index = np.unravel_index(
                int(np.argmax(exceedance)), exceedance.shape
            )
            fold["worst_sample"] = {
                "profile": str(held["profile"][sample_index]),
                "tick": int(held["ticks"][sample_index]),
                "action": int(held["actions"][sample_index]),
                "physical_support_mask": int(held["support_masks"][sample_index]),
                "pressure": PRESSURE_NAMES[pressure_index],
                "positive_residual": float(
                    held["positive_residual"][sample_index, pressure_index]
                ),
                "bound": float(bound[sample_index, pressure_index]),
                "exceedance": float(exceedance[sample_index, pressure_index]),
            }
            if "previous_support_masks" in held:
                fold["worst_sample"].update(
                    {
                        "support_transition": [
                            int(held["previous_support_masks"][sample_index]),
                            int(held["support_masks"][sample_index]),
                            int(held["next_support_masks"][sample_index]),
                        ],
                        "predicted_acceleration": held[
                            "predicted_acceleration"
                        ][sample_index].tolist(),
                        "realized_acceleration": held["realized_acceleration"][
                            sample_index
                        ].tolist(),
                        "acceleration_error_maximum_absolute": float(
                            np.max(np.abs(held["acceleration_error"][sample_index]))
                        ),
                    }
                )
            fold_metrics[holdout] = fold
            residual_parts.append(held["positive_residual"])
            bound_parts.append(bound)
        aggregate = score_bound(
            np.concatenate(residual_parts, axis=0),
            np.concatenate(bound_parts, axis=0),
        )
        aggregate["folds"] = fold_metrics
        aggregate["worst_fold_sample"] = max(
            (
                fold["worst_sample"] | {"case": case_name}
                for case_name, fold in fold_metrics.items()
            ),
            key=lambda sample: sample["exceedance"],
        )
        aggregate["strict_holdout_admitted"] = bool(
            aggregate["sample_coverage"] == 1.0
        )
        methods[method] = aggregate
    return methods


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in requested)
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")
    if len(selected_cases) < 2:
        raise SystemExit("R196 requires at least two named cases for held-out evaluation")
    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    sample_sets: list[dict[str, Any]] = []
    run_metrics: dict[str, Any] = {}
    for case_index, case in enumerate(selected_cases, 1):
        run_metrics[case.name] = {}
        for profile_name, profile in PROFILE_CONFIGS.items():
            run = execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=1,
                inexact_terminal_chooser=True,
                inexact_terminal_support_hypothesis_envelope=True,
            )
            sample_set = collect_samples(
                case.name, profile_name, run["trace"], balance, limits
            )
            sample_sets.append(sample_set)
            run_metrics[case.name][profile_name] = run["metrics"]
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            f"samples={sum(len(samples['ticks']) for samples in sample_sets if samples['case'][0] == case.name)}",
            flush=True,
        )
    samples = concatenate_samples(sample_sets)
    case_names = tuple(case.name for case in selected_cases)
    methods = leave_one_case_out(samples, case_names)
    dominant_outlier = methods["global_max"]["worst_fold_sample"]
    deployable_admitted = any(
        methods[name]["strict_holdout_admitted"] for name in DEPLOYABLE_METHODS
    )
    score_ns = samples["score_ns"]
    zero_allocation = bool(
        np.all(samples["allocation_calls"] == 0)
        and np.all(samples["allocated_bytes"] == 0)
    )
    detail = [
        [
            method,
            methods[method]["sample_count"],
            f"{methods[method]['sample_coverage'] * 100.0:.2f}%",
            f"{methods[method]['component_coverage'] * 100.0:.3f}%",
            f"{methods[method]['maximum_exceedance']:.3f}",
            f"{methods[method]['bound_maximum_component']['p95']:.3f}",
            "YES" if methods[method]["strict_holdout_admitted"] else "NO",
        ]
        for method in METHOD_ORDER
    ]
    fold_detail = [
        [
            method,
            case_name,
            fold["sample_count"],
            f"{fold['sample_coverage'] * 100.0:.1f}%",
            f"{fold['maximum_exceedance']:.3f}",
        ]
        for method in METHOD_ORDER
        for case_name, fold in methods[method]["folds"].items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "case_names": case_names,
        "profiles": tuple(PROFILE_CONFIGS),
        "pressure_names": PRESSURE_NAMES,
        "sample_count": int(len(samples["ticks"])),
        "selected_action_counts": {
            str(action): int(np.sum(samples["actions"] == action))
            for action in range(3)
        },
        "physical_support_mask_counts": {
            str(mask): int(np.sum(samples["support_masks"] == mask))
            for mask in range(4)
        },
        "deployable_holdout_bracketing_admitted": deployable_admitted,
        "zero_rust_allocation": zero_allocation,
        "score_ns": distribution(score_ns),
        "methods": methods,
        "run_metrics": run_metrics,
    }
    report = "\n".join(
        [
            "# Bonesaw terminal realization residual conditioning · r196",
            "",
            f"> Deployable leave-one-named-case-out bracketing **{'ADMITTED' if deployable_admitted else 'REJECTED'}**.",
            "",
            "## Contract",
            "",
            "- The target is the positive componentwise pressure residual between the following measured 5 ms plant interval and r195's selected four-support envelope. The plant remains offline evaluation only.",
            "- Deployable features are current root clearance/vertical velocity/tilt/angular rate, four bounded hip/knee coordinates, six normalized joint velocities, the selected envelope pressures, per-pressure spread across support hypotheses, action class, and effort use. No physical contact mask, named case, future dropout duration, external-force label, or post-step state enters those methods.",
            "- Every result is leave-one-named-case-out. `oracle_support_action_max` uses measured physical support only as a diagnostic ceiling; `global_max_1p25` is a declared sensitivity row, not a fitted authority.",
            f"- Samples: **{len(samples['ticks'])}**; actions 0/1/2 **{metrics['selected_action_counts']['0']}/{metrics['selected_action_counts']['1']}/{metrics['selected_action_counts']['2']}**; physical masks 0/1/2/3 **{metrics['physical_support_mask_counts']['0']}/{metrics['physical_support_mask_counts']['1']}/{metrics['physical_support_mask_counts']['2']}/{metrics['physical_support_mask_counts']['3']}**.",
            "",
            "## Aggregate holdout",
            "",
            *markdown_table(
                [
                    "method",
                    "samples",
                    "sample coverage",
                    "component coverage",
                    "max exceedance",
                    "bound p95 max component",
                    "strict",
                ],
                detail,
            ),
            "",
            "## Worst held-out samples",
            "",
            *markdown_table(
                [
                    "method",
                    "case/profile/tick",
                    "action/support transition",
                    "pressure",
                    "residual",
                    "bound",
                    "exceedance",
                    "qdd error max",
                ],
                [
                    [
                        method,
                        f"{methods[method]['worst_fold_sample']['case']}/{methods[method]['worst_fold_sample']['profile']}/{methods[method]['worst_fold_sample']['tick']}",
                        f"{methods[method]['worst_fold_sample']['action']}/{methods[method]['worst_fold_sample'].get('support_transition', [methods[method]['worst_fold_sample']['physical_support_mask']])}",
                        methods[method]["worst_fold_sample"]["pressure"],
                        f"{methods[method]['worst_fold_sample']['positive_residual']:.3f}",
                        f"{methods[method]['worst_fold_sample']['bound']:.3f}",
                        f"{methods[method]['worst_fold_sample']['exceedance']:.3f}",
                        f"{methods[method]['worst_fold_sample'].get('acceleration_error_maximum_absolute', math.nan):.1f}",
                    ]
                    for method in METHOD_ORDER
                ],
            ),
            "",
            "## Named-case folds",
            "",
            *markdown_table(
                ["method", "held-out case", "samples", "coverage", "max exceedance"],
                fold_detail,
            ),
            "",
            "## Runtime and interpretation",
            "",
            f"- Independent Rust rescoring maximum: **{metrics['score_ns']['maximum'] / 1e3:.3f} µs**; zero Rust allocation: **{zero_allocation}**.",
            f"- The global worst holdout is **{dominant_outlier['case']}/{dominant_outlier['profile']} tick {dominant_outlier['tick']}**, action **{dominant_outlier['action']}**, support transition **{dominant_outlier.get('support_transition')}**, with **{dominant_outlier.get('acceleration_error_maximum_absolute', math.nan):.1f} rad-or-m/s²** maximum acceleration error. This is a contact-impact discontinuity; a smooth fixed-contact acceleration residual is the wrong state representation for that tail.",
            "- This audit can admit a residual representation for another consequence A/B; it cannot promote terminal authority by itself. Any selected model must next be frozen, independently replayed, tested on new disturbances/morphology perturbations, and composed with the 5 ms deadline.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-terminal-residual-conditioning-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_TERMINAL_RESIDUAL_CONDITIONING.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "deployable_holdout_bracketing_admitted": deployable_admitted,
                "sample_count": metrics["sample_count"],
                "methods": {
                    name: {
                        "sample_coverage": methods[name]["sample_coverage"],
                        "maximum_exceedance": methods[name]["maximum_exceedance"],
                    }
                    for name in METHOD_ORDER
                },
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
