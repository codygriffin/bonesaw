#!/usr/bin/env python3
"""R198 held-out residual conditioning with causal lagged impulse witnesses."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_terminal_chooser_ab import model_limits
from upkie_terminal_residual_conditioning import (
    collect_samples,
    concatenate_samples,
    leave_one_case_out,
)


REVISION = "upkie-lagged-impulse-conditioning-r198"
DURATION_S = 6.0
PROFILE_CONFIGS = {
    "drop5": DROP5,
    "drop10": DROP10,
    "drop5_matched": DROP5_MATCHED,
}
FEATURE_SETS = ("state_only_r196", "lagged_impulse_linear", "lagged_impulse_log")
METHODS = ("causal_knn32_action_max", "causal_lipschitz_action")
IMPULSE_SCALES = np.asarray([0.25, 0.10, 0.50], np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_LAGGED_IMPULSE_CONDITIONING_R198.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def append_lagged_impulse_features(
    samples: dict[str, Any], trace: dict[str, Any]
) -> None:
    """Append only the completed interval immediately before each query tick."""
    ticks = np.asarray(samples["ticks"], np.int64)
    previous_ticks = np.maximum(ticks - 1, 0)
    wheel_impulse = np.asarray(
        trace["physical_wheel_contact_impulse_ns"], np.float64
    )[previous_ticks]
    constraint_impulse = np.asarray(
        trace["physical_constraint_generalized_impulse_ns"], np.float64
    )[previous_ticks]
    lagged = np.column_stack(
        (
            np.sum(wheel_impulse[:, :, 0], axis=1),
            np.sum(wheel_impulse[:, :, 1], axis=1),
            np.linalg.norm(constraint_impulse, axis=1),
        )
    )
    # A first-tick query has no preceding interval in this run. Zero is an
    # explicit unavailable-history sentinel, never the current interval.
    lagged[ticks == 0] = 0.0
    samples["lagged_impulse"] = lagged
    samples["state_only_features"] = samples["features"].copy()


def select_feature_set(samples: dict[str, Any], name: str) -> dict[str, Any]:
    selected = dict(samples)
    base = samples["state_only_features"]
    if name == "state_only_r196":
        selected["features"] = base
    elif name == "lagged_impulse_linear":
        selected["features"] = np.concatenate(
            (base, samples["lagged_impulse"] / IMPULSE_SCALES), axis=1
        )
    elif name == "lagged_impulse_log":
        selected["features"] = np.concatenate(
            (
                base,
                np.log1p(samples["lagged_impulse"] / IMPULSE_SCALES),
            ),
            axis=1,
        )
    else:
        raise ValueError(f"unknown feature set: {name}")
    return selected


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
        raise SystemExit("R198 requires at least two named cases")
    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    sample_sets: list[dict[str, Any]] = []
    run_metrics: dict[str, Any] = {}
    for case_index, case in enumerate(selected_cases, 1):
        run_metrics[case.name] = {}
        case_samples = 0
        for profile_name, profile in PROFILE_CONFIGS.items():
            run = execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=1,
                inexact_terminal_chooser=True,
                inexact_terminal_support_hypothesis_envelope=True,
                record_physical_contact_impulses=True,
            )
            samples = collect_samples(
                case.name, profile_name, run["trace"], balance, limits
            )
            append_lagged_impulse_features(samples, run["trace"])
            sample_sets.append(samples)
            case_samples += len(samples["ticks"])
            run_metrics[case.name][profile_name] = run["metrics"]
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            f"samples={case_samples}",
            flush=True,
        )
    samples = concatenate_samples(sample_sets)
    case_names = tuple(case.name for case in selected_cases)
    results: dict[str, Any] = {}
    for feature_set in FEATURE_SETS:
        conditioned = select_feature_set(samples, feature_set)
        methods = leave_one_case_out(conditioned, case_names)
        results[feature_set] = {method: methods[method] for method in METHODS}
    strict_admitted = any(
        results[feature_set][method]["strict_holdout_admitted"]
        for feature_set in FEATURE_SETS[1:]
        for method in METHODS
    )
    baseline_reproduced = bool(
        abs(
            results["state_only_r196"]["causal_knn32_action_max"][
                "sample_coverage"
            ]
            - 0.8533834586466166
        )
        <= 1.0e-15
        and abs(
            results["state_only_r196"]["causal_lipschitz_action"][
                "sample_coverage"
            ]
            - 0.9135338345864662
        )
        <= 1.0e-15
    )
    lagged = samples["lagged_impulse"]
    gates = {
        "r196_state_only_baseline_reproduced": baseline_reproduced,
        "lagged_history_is_nonnegative_and_finite": bool(
            np.all(np.isfinite(lagged)) and np.all(lagged >= 0.0)
        ),
        "lagged_history_is_nontrivial": bool(np.max(lagged) > 0.0),
        "finite_without_numeric_fault": all(
            metrics["finite"] and not metrics["numeric_fault"]
            for case_metrics in run_metrics.values()
            for metrics in case_metrics.values()
        ),
        "zero_rust_allocation_and_python_gc": all(
            metrics["allocation_free"] and metrics["python_gc_collections"] == 0
            for case_metrics in run_metrics.values()
            for metrics in case_metrics.values()
        ),
    }
    audit_passed = all(gates.values())
    detail = []
    for feature_set in FEATURE_SETS:
        for method in METHODS:
            result = results[feature_set][method]
            worst = result["worst_fold_sample"]
            detail.append(
                [
                    feature_set,
                    method.replace("causal_", ""),
                    f"{result['sample_coverage'] * 100.0:.2f}%",
                    f"{result['component_coverage'] * 100.0:.3f}%",
                    f"{result['maximum_exceedance']:.3f}",
                    f"{result['bound_maximum_component']['p95']:.3f}",
                    f"{worst['case']}/{worst['profile']}/{worst['tick']}",
                    "YES" if result["strict_holdout_admitted"] else "NO",
                ]
            )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "sample_count": int(len(samples["ticks"])),
        "impulse_scales": IMPULSE_SCALES.tolist(),
        "audit_passed": audit_passed,
        "strict_holdout_admitted": strict_admitted,
        "authority_admitted": False,
        "gates": gates,
        "lagged_impulse_distribution": {
            "maximum": np.max(lagged, axis=0).tolist(),
            "nonzero_rows": int(np.sum(np.any(lagged > 0.0, axis=1))),
        },
        "results": results,
        "run_metrics": run_metrics,
    }
    best = max(
        (
            (results[feature_set][method]["sample_coverage"], feature_set, method)
            for feature_set in FEATURE_SETS[1:]
            for method in METHODS
        ),
        key=lambda item: item[0],
    )
    report = "\n".join(
        [
            "# Bonesaw lagged-impulse residual conditioning · r198",
            "",
            f"> Causal sensor-feature audit **{'PASS' if audit_passed else 'FAIL'}** · strict held-out bound **{'ADMITTED' if strict_admitted else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "## Contract",
            "",
            "- R198 appends only the immediately preceding completed 5 ms interval's wheel normal impulse, wheel tangential impulse, and generalized constraint-impulse norm to R196's current-state/model feature. The current interval, next support, case name, disturbance label, and future dropout duration are excluded.",
            "- Exact simulator impulse is a candidate sensor contract, not an existing Bonesaw observation or hardware claim. A real adapter would need calibrated force/torque or contact estimation, timestamp/age/uncertainty, and the same missing-evidence behavior as every other authority input.",
            "- Linear and log-scaled encodings are frozen before the named-case split. kNN32 and Lipschitz bounds train on six cases and test the seventh componentwise.",
            f"- The unmodified state-only feature reproduces R196 kNN/Lipschitz coverage exactly: **{results['state_only_r196']['causal_knn32_action_max']['sample_coverage'] * 100.0:.2f}%/{results['state_only_r196']['causal_lipschitz_action']['sample_coverage'] * 100.0:.2f}%**.",
            "",
            "## Held-out result",
            "",
            *markdown_table(
                [
                    "features",
                    "method",
                    "sample coverage",
                    "component coverage",
                    "max exceedance",
                    "bound p95 max",
                    "worst holdout",
                    "strict",
                ],
                detail,
            ),
            "",
            f"Best lagged row: **{best[1]} / {best[2]} = {best[0] * 100.0:.2f}% sample coverage**. No row gains authority unless every held-out sample and component is covered.",
            "",
            "## Decision",
            "",
            "Lagged impulse is causal in time but does not automatically become an admissible observation or conservative transition tube. If strict holdout remains red, the next representation must expose pre-impact relative velocity, penetration/load and transition-time uncertainty, or solve a bounded momentum jump directly; another nearest-neighbour tuning pass is not authority.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-lagged-impulse-conditioning-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_LAGGED_IMPULSE_CONDITIONING.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "audit_passed": audit_passed,
                "strict_holdout_admitted": strict_admitted,
                "best": best,
            }
        )
    )
    return 0 if audit_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
