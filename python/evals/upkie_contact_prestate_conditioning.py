#!/usr/bin/env python3
"""R200 held-out terminal-residual conditioning with contact prestate.

The added evidence is sampled before the 5 ms plant interval whose terminal
realization residual is scored.  It is exact MuJoCo contact-constraint state
and therefore remains an offline candidate-sensor oracle, not online authority.
"""

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


REVISION = "upkie-contact-prestate-conditioning-r200"
DURATION_S = 6.0
PROFILE_CONFIGS = {
    "drop5": DROP5,
    "drop10": DROP10,
    "drop5_matched": DROP5_MATCHED,
}
FEATURE_SETS = (
    "state_only_r196",
    "contact_prestate_linear",
    "contact_prestate_signed_log",
)
METHODS = ("causal_knn32_action_max", "causal_lipschitz_action")
DISTANCE_SCALE_M = 0.01
VELOCITY_SCALE_M_S = 1.0
EXPECTED_CANONICAL_COVERAGE = {
    "causal_knn32_action_max": 0.8533834586466166,
    "causal_lipschitz_action": 0.9135338345864662,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_PRESTATE_CONDITIONING_R200.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def append_contact_prestate_features(
    samples: dict[str, Any], trace: dict[str, Any]
) -> None:
    """Attach only exact constraint state available at each current query tick."""
    ticks = np.asarray(samples["ticks"], np.int64)
    available = np.asarray(
        trace["physical_wheel_contact_prestate_available"], np.float64
    )[ticks]
    distance = np.asarray(
        trace["physical_wheel_contact_distance_m"], np.float64
    )[ticks]
    velocity = np.asarray(
        trace["physical_wheel_contact_relative_velocity_m_s"], np.float64
    )[ticks]

    # Unavailable contacts have an explicit indicator and zero payload.  This
    # prevents the +inf trace sentinel, or stale backend storage, from becoming
    # a numeric feature.
    distance = np.where(available != 0.0, distance, 0.0)
    velocity = np.where(available[:, :, None] != 0.0, velocity, 0.0)
    contact_prestate = np.concatenate(
        (
            available,
            distance / DISTANCE_SCALE_M,
            (velocity / VELOCITY_SCALE_M_S).reshape(len(ticks), -1),
        ),
        axis=1,
    )
    if not np.all(np.isfinite(contact_prestate)):
        raise ValueError("contact prestate must be finite after unavailable masking")
    samples["state_only_features"] = samples["features"].copy()
    samples["contact_prestate"] = contact_prestate


def select_feature_set(samples: dict[str, Any], name: str) -> dict[str, Any]:
    selected = dict(samples)
    base = samples["state_only_features"]
    contact = samples["contact_prestate"]
    if name == "state_only_r196":
        selected["features"] = base
    elif name == "contact_prestate_linear":
        selected["features"] = np.concatenate((base, contact), axis=1)
    elif name == "contact_prestate_signed_log":
        # Availability indicators remain binary; only signed metric payloads
        # are compressed.
        encoded = contact.copy()
        encoded[:, 2:] = np.sign(encoded[:, 2:]) * np.log1p(
            np.abs(encoded[:, 2:])
        )
        selected["features"] = np.concatenate((base, encoded), axis=1)
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
        raise SystemExit("R200 requires at least two named cases")

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
                record_physical_contact_prestate=True,
            )
            samples = collect_samples(
                case.name, profile_name, run["trace"], balance, limits
            )
            append_contact_prestate_features(samples, run["trace"])
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

    canonical = (
        not args.cases
        and abs(args.duration - DURATION_S) <= 1.0e-12
        and len(selected_cases) == len(cases())
    )
    baseline_reproduced = all(
        abs(results["state_only_r196"][method]["sample_coverage"] - expected)
        <= 1.0e-15
        for method, expected in EXPECTED_CANONICAL_COVERAGE.items()
    ) if canonical else True
    contact = samples["contact_prestate"]
    contact_rows = np.any(contact[:, :2] != 0.0, axis=1)
    strict_admitted = any(
        results[feature_set][method]["strict_holdout_admitted"]
        for feature_set in FEATURE_SETS[1:]
        for method in METHODS
    )
    gates = {
        "canonical_r196_state_only_baseline_reproduced": baseline_reproduced,
        "contact_prestate_is_finite": bool(np.all(np.isfinite(contact))),
        "contact_prestate_has_available_samples": bool(np.any(contact_rows)),
        "contact_prestate_velocity_is_nontrivial": bool(
            np.max(np.abs(contact[:, 4:])) > 0.0
        ),
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

    detail: list[list[Any]] = []
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
    best = max(
        (
            (results[feature_set][method]["sample_coverage"], feature_set, method)
            for feature_set in FEATURE_SETS[1:]
            for method in METHODS
        ),
        key=lambda item: item[0],
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "sample_count": int(len(samples["ticks"])),
        "feature_contract": {
            "distance_scale_m": DISTANCE_SCALE_M,
            "velocity_scale_m_s": VELOCITY_SCALE_M_S,
            "contact_payload_dimensions": 10,
            "source": "exact_mujoco_pre_solve_constraint_state_candidate_sensor",
        },
        "audit_passed": audit_passed,
        "strict_holdout_admitted": strict_admitted,
        "authority_admitted": False,
        "gates": gates,
        "contact_prestate_distribution": {
            "available_rows": int(np.sum(contact_rows)),
            "flight_rows": int(np.sum(~contact_rows)),
            "available_wheel_samples": int(np.sum(contact[:, :2] != 0.0)),
            "maximum_absolute_scaled_distance": float(
                np.max(np.abs(contact[:, 2:4]))
            ),
            "maximum_absolute_scaled_velocity": float(
                np.max(np.abs(contact[:, 4:]))
            ),
        },
        "results": results,
        "run_metrics": run_metrics,
    }
    report = "\n".join(
        [
            "# Bonesaw contact-prestate residual conditioning · r200",
            "",
            f"> Candidate-sensor audit **{'PASS' if audit_passed else 'FAIL'}** · strict held-out bound **{'ADMITTED' if strict_admitted else 'REJECTED'}** · authority **NOT ADMITTED**.",
            "",
            "## Frozen contract",
            "",
            "- Each query receives only the contact state already present before its scored 5 ms interval: per-wheel availability, signed contact distance, and three signed MuJoCo constraint-coordinate relative velocities. Unavailable payloads are explicitly zeroed behind the availability bits.",
            "- The unchanged R196 state/model feature is the baseline. Linear and signed-log encodings use frozen 0.01 m and 1 m/s scales. Current-interval impulse, post-step state, next support, case identity, disturbance identity, and future outage duration are forbidden.",
            "- These values come from exact pre-solve MuJoCo contacts. They are candidate-sensor/oracle evidence only; an online contract would require physically available sensing or estimation plus provenance, timestamp, age, uncertainty and fail-closed missing-evidence semantics.",
            f"- Samples: **{len(samples['ticks'])}**; contact-present query rows **{int(np.sum(contact_rows))}**; flight rows **{int(np.sum(~contact_rows))}**.",
            "",
            "## Leave-one-named-case-out result",
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
            f"Best contact row: **{best[1]} / {best[2]} = {best[0] * 100.0:.2f}% sample coverage**. Admission requires 100% coverage of every pressure component in every held-out named case.",
            "",
            "## Decision",
            "",
            "Contact prestate is useful only if it makes the discontinuous terminal residual conservatively bracketable under a fresh named-case holdout. Even a passing oracle row would not admit authority: the observation must first be realized as a typed, timed sensor contract and repeated on new disturbances and morphology perturbations. A failed row means current contact kinematics do not expose the future within-interval impact transition; the next model should bound transition time and momentum jump directly rather than tune another smoother.",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-contact-prestate-conditioning-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_PRESTATE_CONDITIONING.md").write_text(report)
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
