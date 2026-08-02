#!/usr/bin/env python3
"""Calibrate R194 zero-effort model error against one-step plant realization."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP10
from upkie_inexact_terminal_chooser_ab import model_limits
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-zero-effort-realization-calibration-r194"
DURATION_S = 6.0
ACCELERATION_NAMES = (
    "root_roll",
    "root_pitch",
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
)
PRESSURE_NAMES = (
    "tilt_pressure",
    "angular_rate_pressure",
    "joint_position_pressure",
    "joint_velocity_pressure",
    "actuator_effort_pressure",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_ZERO_EFFORT_REALIZATION_CALIBRATION_R194.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def profile(name: str) -> dict[str, int]:
    return DROP5 if name == "drop5" else DROP10


def realization_samples(
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, Any]:
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    action = np.asarray(trace["inexact_observation_terminal_selector_action"])
    ticks = np.flatnonzero(queried & (action == 0))
    root_twist = np.asarray(trace["root_twist"])
    post_root_twist = np.asarray(trace["post_root_twist"])
    joint_velocity = np.asarray(trace["v"])
    post_joint_velocity = np.asarray(trace["post_v"])
    predicted = np.asarray(
        trace["inexact_observation_terminal_zero_effort_acceleration"]
    )
    realized_root = (post_root_twist[ticks, :2] - root_twist[ticks, :2]) / CONTROL_DT
    realized_joint = (
        post_joint_velocity[ticks] - joint_velocity[ticks]
    ) / CONTROL_DT
    realized = np.column_stack((realized_root, realized_joint))
    predicted_reduced = np.column_stack((predicted[ticks, :2], predicted[ticks, 6:]))
    acceleration_error = realized - predicted_reduced

    diagnostic_names = tuple(balance.terminal_impact_diagnostic_names)
    diagnostic_index = {name: index for index, name in enumerate(diagnostic_names)}
    pressure_indices = np.asarray(
        [diagnostic_index[name] for name in PRESSURE_NAMES], np.int64
    )
    online_diagnostics = np.asarray(
        trace["inexact_observation_terminal_selector_candidate_diagnostics"]
    )
    pressure_error = np.empty((len(ticks), len(PRESSURE_NAMES)), np.float64)
    allocation_calls = np.empty(len(ticks), np.uint64)
    allocated_bytes = np.empty(len(ticks), np.uint64)
    score_ns = np.empty(len(ticks), np.uint64)
    for cursor, tick in enumerate(ticks):
        candidate_root = np.array(
            trace["inexact_observation_terminal_selector_root_acceleration"][tick],
            copy=True,
        )
        candidate_joint = np.array(
            trace["inexact_observation_terminal_selector_joint_acceleration"][tick],
            copy=True,
        )
        candidate_root[0] = realized_root[cursor]
        candidate_joint[0] = realized_joint[cursor]
        diagnostics = np.empty((3, len(diagnostic_names)), np.float64)
        selection = np.empty(6, np.float64)
        timing = balance.score_terminal_impact_candidates(
            np.asarray(trace["inexact_observation_terminal_selector_state"])[tick],
            np.asarray(trace["q"])[tick],
            joint_velocity[tick],
            lower,
            upper,
            velocity_limit,
            np.asarray(
                [
                    1,
                    trace[
                        "inexact_observation_terminal_selector_retained_available"
                    ][tick],
                    trace[
                        "inexact_observation_terminal_selector_support_free_available"
                    ][tick],
                ],
                np.uint8,
            ),
            candidate_root,
            candidate_joint,
            np.asarray(
                trace[
                    "inexact_observation_terminal_selector_effort_utilization"
                ]
            )[tick],
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )
        score_ns[cursor], allocation_calls[cursor], allocated_bytes[cursor] = timing
        pressure_error[cursor] = (
            diagnostics[0, pressure_indices]
            - online_diagnostics[tick, 0, pressure_indices]
        )
    physical_contact = np.asarray(trace["physical_contact_active"])[ticks]
    support_masks = physical_contact[:, 0] + 2 * physical_contact[:, 1]
    return {
        "ticks": ticks,
        "predicted_acceleration": predicted_reduced,
        "realized_acceleration": realized,
        "acceleration_error": acceleration_error,
        "pressure_error": pressure_error,
        "support_masks": support_masks,
        "score_ns": score_ns,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    }


def vector(values: np.ndarray) -> list[float]:
    return [float(value) for value in values]


def summarize_samples(samples: dict[str, Any]) -> dict[str, Any]:
    acceleration_error = samples["acceleration_error"]
    pressure_error = samples["pressure_error"]
    support_masks = samples["support_masks"]
    return {
        "sample_count": int(len(acceleration_error)),
        "physical_support_mask_counts": {
            str(int(mask)): int(np.sum(support_masks == mask))
            for mask in np.unique(support_masks)
        },
        "predicted_acceleration_maximum_absolute": vector(
            np.max(np.abs(samples["predicted_acceleration"]), axis=0)
        ),
        "realized_acceleration_maximum_absolute": vector(
            np.max(np.abs(samples["realized_acceleration"]), axis=0)
        ),
        "acceleration_error_maximum_absolute": vector(
            np.max(np.abs(acceleration_error), axis=0)
        ),
        "acceleration_error_norm": distribution(
            np.linalg.norm(acceleration_error, axis=1)
        ),
        "pressure_error_maximum_absolute": vector(
            np.max(np.abs(pressure_error), axis=0)
        ),
        "pressure_error_positive_maximum": vector(
            np.max(np.maximum(pressure_error, 0.0), axis=0)
        ),
        "pressure_error_norm": distribution(np.linalg.norm(pressure_error, axis=1)),
        "score_ns": distribution(samples["score_ns"]),
        "zero_allocation": bool(
            np.all(samples["allocation_calls"] == 0)
            and np.all(samples["allocated_bytes"] == 0)
        ),
    }


def leave_one_case_out(
    sample_sets: dict[str, dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    for holdout, holdout_samples in sample_sets.items():
        training = [
            samples["pressure_error"]
            for name, samples in sample_sets.items()
            if name != holdout and len(samples["pressure_error"])
        ]
        held = holdout_samples["pressure_error"]
        if not training or not len(held):
            results[holdout] = {
                "training_samples": int(sum(len(values) for values in training)),
                "holdout_samples": int(len(held)),
                "covered_samples": 0,
                "coverage": 0.0,
                "maximum_exceedance": math.inf,
                "calibrated_absolute_pressure_bound": [math.inf] * len(PRESSURE_NAMES),
            }
            continue
        training_values = np.concatenate(training, axis=0)
        bound = np.max(np.abs(training_values), axis=0)
        exceedance = np.maximum(np.abs(held) - bound, 0.0)
        covered = np.all(exceedance <= 1.0e-12, axis=1)
        results[holdout] = {
            "training_samples": int(len(training_values)),
            "holdout_samples": int(len(held)),
            "covered_samples": int(np.sum(covered)),
            "coverage": float(np.mean(covered)),
            "maximum_exceedance": float(np.max(exceedance)),
            "calibrated_absolute_pressure_bound": vector(bound),
            "holdout_maximum_absolute_pressure_error": vector(
                np.max(np.abs(held), axis=0)
            ),
        }
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

    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    balance = bonesaw.UpkieBalanceSession(str(model))
    sample_sets: dict[str, dict[str, Any]] = {}
    summaries: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        per_case: list[dict[str, Any]] = []
        for dropout in ("drop5", "drop10"):
            result = execute(
                model,
                case,
                args.duration,
                profile(dropout),
                inexact_hold_ticks=1,
                inexact_terminal_chooser=True,
                inexact_terminal_zero_effort_baseline=True,
            )
            samples = realization_samples(result["trace"], balance, limits)
            sample_sets[f"{case.name}/{dropout}"] = samples
            per_case.append(samples)
        combined = {
            key: np.concatenate([samples[key] for samples in per_case], axis=0)
            for key in per_case[0]
        }
        sample_sets[case.name] = combined
        summaries[case.name] = summarize_samples(combined)
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            f"withhold samples={summaries[case.name]['sample_count']}, "
            f"pressure error max={max(summaries[case.name]['pressure_error_maximum_absolute']):.3f}",
            flush=True,
        )

    case_only = {case.name: sample_sets[case.name] for case in selected_cases}
    all_samples = {
        key: np.concatenate([samples[key] for samples in case_only.values()], axis=0)
        for key in next(iter(case_only.values()))
    }
    aggregate = summarize_samples(all_samples)
    cross_validation = leave_one_case_out(case_only)
    calibration_admitted = bool(
        all(result["coverage"] == 1.0 for result in cross_validation.values())
        and aggregate["zero_allocation"]
    )
    detail = [
        [
            name,
            summary["sample_count"],
            f"{summary['pressure_error_norm']['p95']:.3f}",
            f"{summary['pressure_error_norm']['maximum']:.3f}",
            f"{summary['acceleration_error_norm']['maximum']:.1f}",
            summary["physical_support_mask_counts"],
            f"{cross_validation[name]['coverage'] * 100.0:.1f}%",
            f"{cross_validation[name]['maximum_exceedance']:.3f}",
        ]
        for name, summary in summaries.items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "acceleration_names": ACCELERATION_NAMES,
        "pressure_names": PRESSURE_NAMES,
        "calibration_admitted": calibration_admitted,
        "aggregate": aggregate,
        "case_summaries": summaries,
        "leave_one_case_out": cross_validation,
    }
    report = "\n".join(
        [
            "# Bonesaw zero-effort realization calibration · r194",
            "",
            f"> Leave-one-case-out empirical bound **{'ADMITTED' if calibration_admitted else 'REJECTED'}**.",
            "",
            "## What is measured",
            "",
            "- Only ticks where the Rust chooser selected typed withhold are audited. The model prediction is the no-contact fixed-zero-effort generalized acceleration; realization is the finite difference of MuJoCo root/joint velocity over the following 5 ms while the same zero torque executes.",
            "- The realized acceleration is rescored from the same current state. Signed terminal-pressure error is `realized − predicted`; positive error means the online model underestimated harm.",
            "- This is calibration evidence, not online physics. The online path remains a fixed Rust model query and terminal scorer.",
            "",
            *markdown_table(
                [
                    "held-out case",
                    "samples",
                    "pressure error p95",
                    "pressure error max",
                    "qdd error max",
                    "physical masks",
                    "LOCO coverage",
                    "max exceedance",
                ],
                detail,
            ),
            "",
            "## Result",
            "",
            f"- Aggregate withhold samples: **{aggregate['sample_count']}**; pressure-error norm p95/max **{aggregate['pressure_error_norm']['p95']:.3f}/{aggregate['pressure_error_norm']['maximum']:.3f}**; acceleration-error norm max **{aggregate['acceleration_error_norm']['maximum']:.1f} rad-or-m/s²**.",
            "- A no-contact zero-effort solve is not a calibrated predictor when physical wheel support may still exist but its observation is unavailable. The next online representation must retain a visible interval or hypothesis set over support modes; choosing one fictitious contact mode is not conservative.",
            f"- Offline rescoring maximum: **{aggregate['score_ns']['maximum'] / 1e3:.3f} µs** with zero Rust allocation: **{aggregate['zero_allocation']}**.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-zero-effort-realization-calibration-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_ZERO_EFFORT_REALIZATION_CALIBRATION.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "calibration_admitted": calibration_admitted,
                "sample_count": aggregate["sample_count"],
                "pressure_error_maximum": aggregate["pressure_error_norm"]["maximum"],
                "acceleration_error_maximum": aggregate["acceleration_error_norm"]["maximum"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
