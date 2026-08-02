#!/usr/bin/env python3
"""r159 forecast-versus-realized calibration and frozen holdout audit.

The experiment observes the reduced-order path emitted from each final, fresh,
exact-WBC command.  It never changes the command or advances the plant during
selection.  A forecast sample is censored as soon as raw support differs from
the support provenance at its origin; support transitions are revocations, not
errors silently folded into a wider dynamics envelope.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, run_case, semantic_trace_equal
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-executed-forecast-realization-contract-r159"
DEFAULT_DURATION_S = 6.0
STATE_NAMES = (
    "roll_rad",
    "roll_rate_rad_s",
    "pitch_rad",
    "pitch_rate_rad_s",
    "lateral_position_m",
    "lateral_velocity_m_s",
    "yaw_rad",
    "yaw_rate_rad_s",
)
STATE_SCALES = np.asarray(
    [math.pi / 4.0, 4.0, math.pi / 4.0, 4.0, 0.10, 1.0, math.pi / 4.0, 4.0],
    np.float64,
)
# Frozen before observing r157.  Each side spans signs, axes, durations, load
# levels, and non-sagittal disturbances; related cases are deliberately split.
CALIBRATION_CASES = frozenset(
    {
        "nominal",
        "forward_2n",
        "forward_6n_overload",
        "backward_2n",
        "left_1n",
        "left_4n",
        "right_2n",
        "diagonal_4n",
        "up_4n",
        "short_8n_50ms",
    }
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_EXECUTED_FORECAST_REALIZATION_CONTRACT_R159.html",
    )
    parser.add_argument(
        "--cases", help="comma-separated subset for smoke/debug; full audit uses 20"
    )
    return parser.parse_args()


def execute(model: pathlib.Path, case: Any, duration_s: float) -> dict[str, Any]:
    return run_case(
        model,
        case,
        duration_s,
        "capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=True,
        execute_reduced_support=True,
        viability_planner_enabled=True,
        viability_planner_strategy="multistep_budgeted",
        viability_confirmation_updates=2,
    )


def raw_support_masks(trace: dict[str, Any]) -> np.ndarray:
    observed = np.asarray(trace["observed_contact_active"], np.uint8)
    return observed[:, 0] | (observed[:, 1] << 1)


def collect_case_samples(trace: dict[str, Any]) -> dict[str, Any]:
    valid = np.asarray(trace["execution_forecast_path_valid"], np.uint8) != 0
    forecast_support = np.asarray(
        trace["execution_forecast_support_mask"], np.uint8
    )
    actual = np.asarray(trace["execution_forecast_reduced_state"], np.float64)
    paths = np.asarray(trace["execution_forecast_path"], np.float64)
    raw_support = raw_support_masks(trace)
    external_force = np.asarray(trace["external_force_world"], np.float64)
    knot_times = paths[valid][0, :, 0] if np.any(valid) else np.arange(1, 9) * 0.03
    offsets = np.rint(knot_times / CONTROL_DT).astype(np.int64)
    errors: list[list[np.ndarray]] = [[] for _ in range(8)]
    quiescent_errors: list[list[np.ndarray]] = [[] for _ in range(8)]
    external_disturbance_samples = np.zeros(8, np.int64)
    transition_censored = np.zeros(8, np.int64)
    support_mismatch_censored = np.zeros(8, np.int64)
    truncated = np.zeros(8, np.int64)
    origins = np.flatnonzero(valid)
    for origin in origins:
        origin_raw_support = int(raw_support[origin])
        for knot, offset in enumerate(offsets):
            endpoint = int(origin + offset)
            if endpoint >= len(actual):
                truncated[knot] += 1
                continue
            if int(forecast_support[origin]) != origin_raw_support:
                support_mismatch_censored[knot] += 1
                continue
            if np.any(raw_support[origin : endpoint + 1] != origin_raw_support):
                transition_censored[knot] += 1
                continue
            error = np.abs(paths[origin, knot, 1:] - actual[endpoint])
            errors[knot].append(error)
            # Force at tick i is applied after the origin state/command and
            # before state i+1. It is therefore an unmodeled future input for
            # every interval [origin, endpoint), including the origin tick.
            if np.any(external_force[origin:endpoint] != 0.0):
                external_disturbance_samples[knot] += 1
            else:
                quiescent_errors[knot].append(error)
    packed = [
        np.vstack(values) if values else np.empty((0, 8), np.float64)
        for values in errors
    ]
    quiescent_packed = [
        np.vstack(values) if values else np.empty((0, 8), np.float64)
        for values in quiescent_errors
    ]
    return {
        "errors": packed,
        "quiescent_errors": quiescent_packed,
        "knot_times_s": knot_times,
        "offset_ticks": offsets,
        "valid_origins": int(len(origins)),
        "stable_samples_by_knot": [int(len(values)) for values in packed],
        "quiescent_samples_by_knot": [
            int(len(values)) for values in quiescent_packed
        ],
        "external_disturbance_samples_by_knot": external_disturbance_samples.tolist(),
        "transition_censored_by_knot": transition_censored.tolist(),
        "support_mismatch_censored_by_knot": support_mismatch_censored.tolist(),
        "truncated_by_knot": truncated.tolist(),
    }


def concatenate_samples(
    cases: dict[str, dict[str, Any]],
    names: set[str],
    field: str = "errors",
) -> list[np.ndarray]:
    output: list[np.ndarray] = []
    for knot in range(8):
        values = [cases[name][field][knot] for name in names if name in cases]
        values = [value for value in values if len(value)]
        output.append(
            np.vstack(values) if values else np.empty((0, len(STATE_NAMES)), np.float64)
        )
    return output


def scalar_distribution(values: np.ndarray) -> dict[str, float]:
    if not len(values):
        return {
            name: 0.0
            for name in (
                "minimum",
                "mean",
                "stddev",
                "mad",
                "p50",
                "p95",
                "p99",
                "p99_9",
                "maximum",
            )
        }
    return distribution(values)


def partition_summary(samples: list[np.ndarray]) -> list[dict[str, Any]]:
    rows = []
    for knot, values in enumerate(samples):
        normalized = values / STATE_SCALES if len(values) else values
        maximum_normalized = (
            np.max(normalized, axis=1) if len(normalized) else np.empty(0)
        )
        rows.append(
            {
                "knot": knot + 1,
                "sample_count": int(len(values)),
                "absolute_error": {
                    name: scalar_distribution(values[:, column])
                    for column, name in enumerate(STATE_NAMES)
                },
                "maximum_normalized_error": scalar_distribution(maximum_normalized),
            }
        )
    return rows


def calibrated_bounds(samples: list[np.ndarray]) -> np.ndarray:
    bounds = np.empty((8, 8), np.float64)
    for knot, values in enumerate(samples):
        if not len(values):
            bounds[knot].fill(math.inf)
        else:
            # A visible 5% calibration reserve avoids presenting a sample max
            # as a mathematical guarantee.  The independent holdout decides
            # whether this measured envelope is even eligible for codification.
            bounds[knot] = np.maximum(np.max(values, axis=0) * 1.05, 1.0e-12)
    return bounds


def coverage_summary(samples: list[np.ndarray], bounds: np.ndarray) -> dict[str, Any]:
    knot_rows = []
    all_inside = 0
    all_count = 0
    for knot, values in enumerate(samples):
        inside = values <= bounds[knot] if len(values) else np.empty((0, 8), bool)
        row_inside = np.all(inside, axis=1) if len(values) else np.empty(0, bool)
        all_inside += int(np.sum(row_inside))
        all_count += int(len(row_inside))
        knot_rows.append(
            {
                "knot": knot + 1,
                "sample_count": int(len(values)),
                "joint_coverage": float(np.mean(row_inside)) if len(row_inside) else 0.0,
                "per_state_coverage": {
                    name: float(np.mean(inside[:, column])) if len(values) else 0.0
                    for column, name in enumerate(STATE_NAMES)
                },
                "maximum_bound_ratio": float(np.max(values / bounds[knot]))
                if len(values)
                else 0.0,
            }
        )
    return {
        "joint_coverage": float(all_inside / all_count) if all_count else 0.0,
        "covered_samples": all_inside,
        "sample_count": all_count,
        "all_samples_inside": bool(all_count and all_inside == all_count),
        "by_knot": knot_rows,
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    selected = case_matrix()
    if args.cases:
        names = set(args.cases.split(","))
        selected = tuple(case for case in selected if case.name in names)
        missing = names - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    case_samples: dict[str, dict[str, Any]] = {}
    case_rows: dict[str, Any] = {}
    all_traces: list[dict[str, Any]] = []
    for index, case in enumerate(selected, start=1):
        trace = execute(model, case, args.duration)
        replay = execute(model, case, args.duration)
        samples = collect_case_samples(trace)
        case_samples[case.name] = samples
        all_traces.append(trace)
        timing = np.asarray(trace["execution_forecast_step_ns"], np.uint64)
        timing = timing[np.asarray(trace["execution_forecast_path_valid"]) != 0]
        case_rows[case.name] = {
            "partition": "calibration" if case.name in CALIBRATION_CASES else "holdout",
            "valid_origins": samples["valid_origins"],
            "stable_samples_by_knot": samples["stable_samples_by_knot"],
            "quiescent_samples_by_knot": samples["quiescent_samples_by_knot"],
            "external_disturbance_samples_by_knot": samples[
                "external_disturbance_samples_by_knot"
            ],
            "transition_censored_by_knot": samples["transition_censored_by_knot"],
            "support_mismatch_censored_by_knot": samples[
                "support_mismatch_censored_by_knot"
            ],
            "truncated_by_knot": samples["truncated_by_knot"],
            "forecast_step_ns": scalar_distribution(timing.astype(np.float64)),
            "forecast_allocation_calls": int(
                np.sum(trace["execution_forecast_allocation_calls"])
            ),
            "forecast_allocated_bytes": int(
                np.sum(trace["execution_forecast_allocated_bytes"])
            ),
            "replay_exact": semantic_trace_equal(trace, replay),
            "termination_reason": trace["termination_reason"],
            "terminal_time_s": trace["terminal_time_s"],
        }
        print(
            f"[{index:02d}/{len(selected)}] {case.name}: "
            f"origins={samples['valid_origins']}, "
            f"k8={samples['stable_samples_by_knot'][-1]}, "
            f"replay={'exact' if case_rows[case.name]['replay_exact'] else 'DIFF'}",
            flush=True,
        )

    selected_names = set(case_samples)
    calibration_names = selected_names & set(CALIBRATION_CASES)
    holdout_names = selected_names - set(CALIBRATION_CASES)
    calibration = concatenate_samples(case_samples, calibration_names)
    holdout = concatenate_samples(case_samples, holdout_names)
    quiescent_calibration = concatenate_samples(
        case_samples, calibration_names, "quiescent_errors"
    )
    quiescent_holdout = concatenate_samples(
        case_samples, holdout_names, "quiescent_errors"
    )
    bounds = calibrated_bounds(calibration)
    holdout_coverage = coverage_summary(holdout, bounds)
    quiescent_bounds = calibrated_bounds(quiescent_calibration)
    quiescent_holdout_coverage = coverage_summary(
        quiescent_holdout, quiescent_bounds
    )
    expected_times = np.arange(1, 9, dtype=np.float64) * 0.03
    expected_offsets = np.arange(1, 9, dtype=np.int64) * 6
    all_times = [np.asarray(value["knot_times_s"]) for value in case_samples.values()]
    all_offsets = [np.asarray(value["offset_ticks"]) for value in case_samples.values()]
    full_matrix = len(case_rows) == len(case_matrix())
    mechanism_gates = {
        "matrix_complete": full_matrix,
        "frozen_calibration_holdout_split_complete": (
            len(calibration_names) == 10 and len(holdout_names) == 10
        ),
        "candidate_replay_exact": all(row["replay_exact"] for row in case_rows.values()),
        "eight_fixed_knots": all(
            np.array_equal(offsets, expected_offsets) for offsets in all_offsets
        ),
        "knot_times_are_30_to_240_ms": all(
            np.allclose(times, expected_times, rtol=0.0, atol=1.0e-12)
            for times in all_times
        ),
        "calibration_has_every_knot": all(len(values) for values in calibration),
        "holdout_has_every_knot": all(len(values) for values in holdout),
        "quiescent_calibration_has_every_knot": all(
            len(values) for values in quiescent_calibration
        ),
        "quiescent_holdout_has_every_knot": all(
            len(values) for values in quiescent_holdout
        ),
        "finite_realization_error": all(
            np.all(np.isfinite(values)) for values in (*calibration, *holdout)
        ),
        "zero_rust_allocation": all(
            row["forecast_allocation_calls"] == 0
            and row["forecast_allocated_bytes"] == 0
            for row in case_rows.values()
        ),
    }
    calibration_complete = all(mechanism_gates.values())
    certificate_eligible = calibration_complete and quiescent_holdout_coverage[
        "all_samples_inside"
    ]
    # r157 is evidence only.  Even a green frozen holdout does not mutate live
    # authority until a typed Rust certificate and revocation tests land.
    authority_promoted = False
    aggregate_timing = np.concatenate(
        [
            np.asarray(trace["execution_forecast_step_ns"], np.float64)[
                np.asarray(trace["execution_forecast_path_valid"]) != 0
            ]
            for trace in all_traces
        ]
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "controller": "r156 confirmed multistep measured-contact candidate",
        "case_count": len(case_rows),
        "calibration_cases": sorted(calibration_names),
        "holdout_cases": sorted(holdout_names),
        "state_columns": list(STATE_NAMES),
        "state_normalization_scales": STATE_SCALES.tolist(),
        "knot_times_s": expected_times.tolist(),
        "offset_ticks": expected_offsets.tolist(),
        "support_contract": {
            "origin_requires_exact_raw_equals_admitted_support": True,
            "raw_support_change_before_knot": "censor_and_revoke",
            "transition_error_included_in_dynamics_bound": False,
        },
        "disturbance_contract": {
            "unmodeled_external_force_inside_horizon": "out_of_domain",
            "force_at_origin_affects_first_prediction_interval": True,
            "unconditional_error_retained_separately": True,
        },
        "mechanism_gates": mechanism_gates,
        "calibration_complete": calibration_complete,
        "calibration_error": partition_summary(calibration),
        "holdout_error": partition_summary(holdout),
        "calibrated_absolute_bounds": {
            "reserve_fraction": 0.05,
            "by_knot_and_state": bounds.tolist(),
        },
        "holdout_coverage": holdout_coverage,
        "quiescent_calibration_error": partition_summary(quiescent_calibration),
        "quiescent_holdout_error": partition_summary(quiescent_holdout),
        "quiescent_calibrated_absolute_bounds": {
            "reserve_fraction": 0.05,
            "by_knot_and_state": quiescent_bounds.tolist(),
        },
        "quiescent_holdout_coverage": quiescent_holdout_coverage,
        "certificate_eligible_for_codification": certificate_eligible,
        "authority_promoted": authority_promoted,
        "forecast_step_ns": scalar_distribution(aggregate_timing),
        "total_valid_origins": sum(row["valid_origins"] for row in case_rows.values()),
        "total_stable_samples": sum(
            sum(row["stable_samples_by_knot"]) for row in case_rows.values()
        ),
        "total_quiescent_samples": sum(
            sum(row["quiescent_samples_by_knot"]) for row in case_rows.values()
        ),
        "total_external_disturbance_samples": sum(
            sum(row["external_disturbance_samples_by_knot"])
            for row in case_rows.values()
        ),
        "total_transition_censored": sum(
            sum(row["transition_censored_by_knot"]) for row in case_rows.values()
        ),
        "total_support_mismatch_censored": sum(
            sum(row["support_mismatch_censored_by_knot"])
            for row in case_rows.values()
        ),
        "total_truncated": sum(
            sum(row["truncated_by_knot"]) for row in case_rows.values()
        ),
        "cases": case_rows,
    }

    knot_table = []
    quiescent_knot_table = []
    for knot in range(8):
        calibration_row = metrics["calibration_error"][knot]
        holdout_row = metrics["holdout_error"][knot]
        coverage_row = holdout_coverage["by_knot"][knot]
        knot_table.append(
            [
                f"{expected_times[knot] * 1e3:.0f}",
                calibration_row["sample_count"],
                f"{calibration_row['maximum_normalized_error']['p99']:.3f}",
                f"{calibration_row['maximum_normalized_error']['maximum']:.3f}",
                holdout_row["sample_count"],
                f"{holdout_row['maximum_normalized_error']['p99']:.3f}",
                f"{holdout_row['maximum_normalized_error']['maximum']:.3f}",
                f"{100.0 * coverage_row['joint_coverage']:.3f}%",
                f"{coverage_row['maximum_bound_ratio']:.3f}",
            ]
        )
        quiescent_calibration_row = metrics["quiescent_calibration_error"][knot]
        quiescent_holdout_row = metrics["quiescent_holdout_error"][knot]
        quiescent_coverage_row = quiescent_holdout_coverage["by_knot"][knot]
        quiescent_knot_table.append(
            [
                f"{expected_times[knot] * 1e3:.0f}",
                quiescent_calibration_row["sample_count"],
                f"{quiescent_calibration_row['maximum_normalized_error']['p99']:.3f}",
                f"{quiescent_calibration_row['maximum_normalized_error']['maximum']:.3f}",
                quiescent_holdout_row["sample_count"],
                f"{quiescent_holdout_row['maximum_normalized_error']['p99']:.3f}",
                f"{quiescent_holdout_row['maximum_normalized_error']['maximum']:.3f}",
                f"{100.0 * quiescent_coverage_row['joint_coverage']:.3f}%",
                f"{quiescent_coverage_row['maximum_bound_ratio']:.3f}",
            ]
        )
    case_table = [
        [
            name,
            row["partition"],
            row["valid_origins"],
            row["stable_samples_by_knot"][-1],
            sum(row["transition_censored_by_knot"]),
            sum(row["external_disturbance_samples_by_knot"]),
            sum(row["support_mismatch_censored_by_knot"]),
            sum(row["truncated_by_knot"]),
            f"{row['forecast_step_ns']['p99'] / 1e3:.2f}",
            "YES" if row["replay_exact"] else "NO",
        ]
        for name, row in case_rows.items()
    ]
    report = "\n".join(
        [
            "# Bonesaw executed-command forecast-versus-realized contract · r159",
            "",
            f"> Measurement mechanism **{'PASS' if calibration_complete else 'FAIL'}** · unconditional frozen holdout **{'PASS' if holdout_coverage['all_samples_inside'] else 'FAIL'}** · force-quiescent frozen holdout **{'PASS' if certificate_eligible else 'FAIL'}** · live authority promotion **NO**.",
            "",
            "## Result",
            "",
            f"The final admitted WBC emitted **{metrics['total_valid_origins']:,}** allocation-free eight-knot path witnesses. After exact-support provenance, transition revocation, and terminal censoring, **{metrics['total_stable_samples']:,}** forecast/realization pairs remain. The unconditional independent holdout covered **{holdout_coverage['covered_samples']:,}/{holdout_coverage['sample_count']:,} ({100.0 * holdout_coverage['joint_coverage']:.4f}%)** pairs jointly across all eight state components. This failure is retained: future external wrench is absent from the forecast state.",
            "",
            "R157 first measured the 26 shadow-confirmed proposal paths and exposed hybrid-support error. R158 adds and causally tests a support-dwell/direction/load guard. R159 broadens the realization question to every final fresh exact-WBC command and freezes a separate calibration/holdout corpus. It does not let either envelope command the robot. A typed Rust certificate still has to consume an admitted bound, expire on sequence/evidence/support changes, and pass a second causal plant A/B before authority can move.",
            "",
            "## Error over execution time",
            "",
            *markdown_table(
                [
                    "horizon ms",
                    "cal n",
                    "cal norm p99",
                    "cal norm max",
                    "holdout n",
                    "holdout norm p99",
                    "holdout norm max",
                    "joint coverage",
                    "worst bound ratio",
                ],
                knot_table,
            ),
            "",
            "Normalized error is the maximum of absolute roll/pitch/yaw error divided by 45°, angular-rate errors divided by 4 rad/s, lateral-position error divided by 0.10 m, and lateral-velocity error divided by 1 m/s. Absolute per-state distributions and all 64 calibrated bounds are retained in JSON.",
            "",
            "## Conditional force-quiescent envelope",
            "",
            f"When no unmodeled external force acts anywhere inside the forecast interval, **{metrics['total_quiescent_samples']:,}** pairs remain and the independent holdout covers **{quiescent_holdout_coverage['covered_samples']:,}/{quiescent_holdout_coverage['sample_count']:,} ({100.0 * quiescent_holdout_coverage['joint_coverage']:.4f}%)** jointly. The **{metrics['total_external_disturbance_samples']:,}** excluded pairs remain in the unconditional table above; they are not relabeled as model success.",
            "",
            *markdown_table(
                [
                    "horizon ms",
                    "cal n",
                    "cal norm p99",
                    "cal norm max",
                    "holdout n",
                    "holdout norm p99",
                    "holdout norm max",
                    "joint coverage",
                    "worst bound ratio",
                ],
                quiescent_knot_table,
            ),
            "",
            "## Cases and censoring",
            "",
            *markdown_table(
                [
                    "case",
                    "partition",
                    "origins",
                    "240 ms pairs",
                    "transition censored",
                    "forced pairs",
                    "support mismatch",
                    "terminal censored",
                    "forecast p99 µs",
                    "replay",
                ],
                case_table,
            ),
            "",
            "A raw support change is a certificate revocation, not a large model residual. An origin whose exact raw support differs from the WBC's admitted support is also censored. This keeps contact-authority errors visible rather than laundering them into a permissive dynamics bound.",
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in mechanism_gates.items()],
            ),
            "",
            "## Dataflow and allocation",
            "",
            "Python owns the immutable case split, MuJoCo sequencing, future-state joins, censoring, statistics, and report generation. Rust owns the final exact WBC, fixed-size path integration, output validation, timing, and allocation witness. The hot path writes a caller-owned `[8, 9]` array; no path list, dictionary, or per-knot Python object is created in the control loop.",
            "",
            f"Forecast emission timing across valid origins: p50 **{metrics['forecast_step_ns']['p50'] / 1e3:.3f} µs**, p99 **{metrics['forecast_step_ns']['p99'] / 1e3:.3f} µs**, max **{metrics['forecast_step_ns']['maximum'] / 1e3:.3f} µs**. Rust allocation calls and bytes are zero by gate.",
            "",
            "## Certificate boundary",
            "",
            "The eventual certificate may erode a forecast by an admitted per-knot, per-state error bound only while input validation, exact observation provenance, monotonically increasing sequence, and unchanged support all hold. It must fail closed on expiry or any evidence discontinuity. R159 deliberately contains no path from `certificate_eligible_for_codification` to executable request authority. The frozen envelope misses holdout outliers, so it is not yet admissible.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-forecast-realization-contract-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_FORECAST_REALIZATION_CONTRACT_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "calibration_complete": calibration_complete,
                "certificate_eligible_for_codification": certificate_eligible,
                "authority_promoted": authority_promoted,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if calibration_complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
