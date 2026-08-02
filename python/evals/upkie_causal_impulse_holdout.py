#!/usr/bin/env python3
"""R199 strict holdout audit for causal pre-step contact-impulse witnesses."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_impulse_residual_audit import profiles, source_samples
from upkie_contact_program_robustness_ab import cases, execute
from upkie_inexact_terminal_chooser_ab import model_limits
from upkie_mujoco_plant_report import CONTROL_DT
from upkie_terminal_residual_conditioning import (
    METHOD_ORDER,
    predict_bounds,
)


REVISION = "upkie-causal-impulse-holdout-r199"
DURATION_S = 6.0
TARGET_NAMES = (
    "normal_impulse_ns",
    "tangential_impulse_ns",
    "best_support_delta_velocity_error",
    "qdd_box_delta_velocity_exceedance",
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
        "--web-report", default="web/UPKIE_CAUSAL_IMPULSE_HOLDOUT_R199.html"
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def causal_feature(
    root_twist: np.ndarray,
    q: np.ndarray,
    v: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    velocity_limit: np.ndarray,
    action: int,
    hypothesis_acceleration: np.ndarray,
) -> np.ndarray:
    """Fixed-scale feature containing only evidence available before the step."""
    finite_span = np.where(
        np.isfinite(lower) & np.isfinite(upper) & (upper > lower), upper - lower, 1.0
    )
    midpoint = np.zeros_like(lower)
    finite_interval = np.isfinite(lower) & np.isfinite(upper)
    midpoint[finite_interval] = 0.5 * (
        lower[finite_interval] + upper[finite_interval]
    )
    finite_velocity = np.where(
        np.isfinite(velocity_limit) & (velocity_limit > 0.0), velocity_limit, 1.0
    )
    root_scale = np.asarray([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
    acceleration_scale = np.asarray([100.0, 100.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0])
    selected = np.concatenate(
        (
            hypothesis_acceleration[:, action, :2],
            hypothesis_acceleration[:, action, 6:],
        ),
        axis=1,
    )
    one_hot = np.zeros(3, np.float64)
    one_hot[action] = 1.0
    return np.concatenate(
        (
            root_twist / root_scale,
            (q - midpoint) / finite_span,
            v / finite_velocity,
            np.mean(selected, axis=0) / acceleration_scale,
            (np.max(selected, axis=0) - np.min(selected, axis=0))
            / acceleration_scale,
            one_hot,
        )
    )


def collect_samples(
    case_name: str,
    profile_name: str,
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, np.ndarray]:
    lower, upper, velocity_limit, _effort_limit = limits
    base = source_samples(trace, balance, limits)
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    actions = np.asarray(base["action"], np.uint8)
    root_twist = np.asarray(trace["root_twist"], np.float64)
    q = np.asarray(trace["q"], np.float64)
    v = np.asarray(trace["v"], np.float64)
    hypothesis_acceleration = np.asarray(
        trace["inexact_observation_terminal_hypothesis_acceleration"], np.float64
    )
    features = np.asarray(
        [
            causal_feature(
                root_twist[tick],
                q[tick],
                v[tick],
                lower,
                upper,
                velocity_limit,
                int(action),
                hypothesis_acceleration[tick],
            )
            for tick, action in zip(ticks, actions, strict=True)
        ],
        np.float64,
    )
    targets = np.column_stack(
        (
            base["normal_impulse_ns"],
            base["tangential_impulse_ns"],
            base["best_support_error_norm"] * CONTROL_DT,
            base["qdd_envelope_exceedance"] * CONTROL_DT,
        )
    )
    return {
        "case": np.full(len(ticks), case_name, object),
        "profile": np.full(len(ticks), profile_name, object),
        "ticks": ticks,
        "actions": actions,
        "support_masks": np.asarray(base["support_mask"], np.uint8),
        "features": features,
        "positive_residual": targets,
    }


def concatenate_samples(items: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    return {
        name: np.concatenate([item[name] for item in items], axis=0)
        for name in items[0]
    }


def score_bound(target: np.ndarray, bound: np.ndarray) -> dict[str, Any]:
    exceedance = np.maximum(target - bound, 0.0)
    covered = np.all(exceedance <= 1.0e-12, axis=1)
    return {
        "sample_count": int(len(target)),
        "covered_samples": int(np.sum(covered)),
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(exceedance <= 1.0e-12)),
        "maximum_exceedance_by_target": {
            name: float(value)
            for name, value in zip(TARGET_NAMES, np.max(exceedance, axis=0), strict=True)
        },
        "bound_by_target": {
            name: distribution(bound[:, index])
            for index, name in enumerate(TARGET_NAMES)
        },
    }


def leave_one_case_out(
    samples: dict[str, np.ndarray], case_names: tuple[str, ...]
) -> dict[str, Any]:
    results: dict[str, Any] = {}
    for method in METHOD_ORDER:
        targets: list[np.ndarray] = []
        bounds: list[np.ndarray] = []
        folds: dict[str, Any] = {}
        for holdout in case_names:
            held_mask = samples["case"] == holdout
            train = {name: values[~held_mask] for name, values in samples.items()}
            held = {name: values[held_mask] for name, values in samples.items()}
            bound = predict_bounds(method, train, held)
            fold = score_bound(held["positive_residual"], bound)
            exceedance = np.maximum(held["positive_residual"] - bound, 0.0)
            row, column = np.unravel_index(int(np.argmax(exceedance)), exceedance.shape)
            fold["worst_sample"] = {
                "profile": str(held["profile"][row]),
                "tick": int(held["ticks"][row]),
                "action": int(held["actions"][row]),
                "physical_support_mask": int(held["support_masks"][row]),
                "target": TARGET_NAMES[column],
                "value": float(held["positive_residual"][row, column]),
                "bound": float(bound[row, column]),
                "exceedance": float(exceedance[row, column]),
            }
            folds[holdout] = fold
            targets.append(held["positive_residual"])
            bounds.append(bound)
        aggregate = score_bound(np.concatenate(targets), np.concatenate(bounds))
        aggregate["folds"] = folds
        aggregate["worst_fold_sample"] = max(
            (
                fold["worst_sample"] | {"case": case_name}
                for case_name, fold in folds.items()
            ),
            key=lambda sample: sample["exceedance"],
        )
        aggregate["strict_holdout_admitted"] = aggregate["sample_coverage"] == 1.0
        results[method] = aggregate
    return results


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
        raise SystemExit("R199 requires at least two named cases")
    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    sample_sets: list[dict[str, np.ndarray]] = []
    run_metrics: dict[str, Any] = {}
    for case_index, case in enumerate(selected_cases, 1):
        run_metrics[case.name] = {}
        for profile_name, config in profiles().items():
            kwargs = {name: value for name, value in config.items() if name != "profile"}
            run = execute(model, case, args.duration, config["profile"], **kwargs)
            sample_sets.append(
                collect_samples(case.name, profile_name, run["trace"], balance, limits)
            )
            run_metrics[case.name][profile_name] = run["metrics"]
        print(f"[{case_index:02d}/{len(selected_cases)}] {case.name}", flush=True)
    samples = concatenate_samples(sample_sets)
    case_names = tuple(case.name for case in selected_cases)
    methods = leave_one_case_out(samples, case_names)
    deployable_admitted = any(
        methods[name]["strict_holdout_admitted"] for name in DEPLOYABLE_METHODS
    )
    all_finite = all(
        row["finite"] and not row["numeric_fault"]
        for case_rows in run_metrics.values()
        for row in case_rows.values()
    )
    zero_allocation = all(
        row["allocation_free"] and row["python_gc_collections"] == 0
        for case_rows in run_metrics.values()
        for row in case_rows.values()
    )
    rows = [
        [
            method,
            methods[method]["sample_count"],
            f"{methods[method]['sample_coverage'] * 100.0:.3f}%",
            f"{methods[method]['component_coverage'] * 100.0:.3f}%",
            f"{methods[method]['maximum_exceedance_by_target']['tangential_impulse_ns']:.6f}",
            f"{methods[method]['maximum_exceedance_by_target']['best_support_delta_velocity_error']:.3f}",
            "YES" if methods[method]["strict_holdout_admitted"] else "NO",
        ]
        for method in METHOD_ORDER
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "case_names": case_names,
        "profiles": tuple(profiles()),
        "target_names": TARGET_NAMES,
        "feature_contract": {
            "causal": True,
            "current_root_twist": True,
            "current_joint_position_velocity": True,
            "selected_action": True,
            "four_support_model_acceleration_center_spread": True,
            "physical_support_mask": False,
            "post_step_impulse_or_state": False,
            "future_dropout_duration": False,
            "case_identity": False,
        },
        "sample_count": int(len(samples["ticks"])),
        "all_runs_finite": all_finite,
        "zero_rust_allocation_and_python_gc": zero_allocation,
        "deployable_method_admitted": deployable_admitted,
        "authority_admitted": False,
        "methods": methods,
        "run_metrics": run_metrics,
    }
    best_method = max(
        DEPLOYABLE_METHODS, key=lambda name: methods[name]["sample_coverage"]
    )
    best = methods[best_method]
    worst = best["worst_fold_sample"]
    report = "\n".join(
        [
            "# Bonesaw causal impulse holdout audit · r199",
            "",
            f"> Deployable strict holdout **{'PASS' if deployable_admitted else 'FAIL'}** · authority **NOT ADMITTED**.",
            "",
            "## Boundary",
            "",
            "- Targets are completed-interval wheel impulses plus velocity-jump residuals. They are labels only. Features stop at the selection tick: root twist, joint state, selected action, and the center/spread of the four support-mode acceleration predictions.",
            "- Exact physical support, post-step state/impulse, case identity, and future dropout duration are forbidden from deployable features. The physical-support/action row remains a labelled oracle diagnostic.",
            "- Every bound is trained on six named cases and evaluated on the seventh. A sample passes only if all four targets are componentwise covered.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "method",
                    "n",
                    "sample coverage",
                    "component coverage",
                    "max tangential miss (N·s)",
                    "max Δv-error miss",
                    "strict",
                ],
                rows,
            ),
            "",
            f"The strongest deployable row is **{best_method}** at **{best['sample_coverage'] * 100.0:.3f}%** complete-sample coverage. Its worst miss is **{worst['case']}/{worst['profile']}/tick {worst['tick']}**, target **{worst['target']}**, value/bound/exceedance **{worst['value']:.6f}/{worst['bound']:.6f}/{worst['exceedance']:.6f}**.",
            "",
            "## Decision",
            "",
            "No empirical pre-step bound becomes authority unless it covers every named holdout without hidden future or simulator-only inputs and remains useful rather than unbounded. This audit either rejects the tested current-state feature family or records only a diagnostic success; plant consequence, online Rust ownership, and the independent 5 ms timing gate remain separate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-causal-impulse-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CAUSAL_IMPULSE_HOLDOUT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "sample_count": metrics["sample_count"],
                "deployable_method_admitted": deployable_admitted,
                "best_method": best_method,
                "best_sample_coverage": best["sample_coverage"],
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
