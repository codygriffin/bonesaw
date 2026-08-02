#!/usr/bin/env python3
"""r162 causal one-step execution-residual monitor on a third holdout.

The Rust monitor sees only a prediction authored on the preceding control tick,
the current exact reduced state, and the current exact support mask.  It checks
against the pre-update rolling envelope before inserting the new residual.  It
is a diagnostic confidence signal and never grants command authority.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

import bonesaw
from cpu_reference_report import markdown_table, render_report_html
from upkie_conditioned_forecast_certificate import STATE_NAMES, STATE_SCALES
from upkie_disturbance_envelope import CaseSpec, semantic_trace_equal
from upkie_forecast_realization_contract import execute, raw_support_masks
from upkie_mujoco_plant_report import CONTROL_DT
from upkie_paired_state_forecast_certificate import new_holdout_cases as r161_cases


REVISION = "upkie-execution-residual-monitor-r162"
MINIMUM_SAMPLES = 8
RESERVE_MULTIPLIER = 2.0
MINIMUM_NORMALIZED_BOUND = np.full(8, 1.0e-4, np.float64)
MAXIMUM_NORMALIZED_BOUND = np.full(8, 0.25, np.float64)


def third_holdout_cases() -> tuple[CaseSpec, ...]:
    """Frozen before the first r162 plant execution; disjoint from r161."""
    diagonal = 3.75 / math.sqrt(2.0)
    return (
        CaseSpec("r162_forward_0p75n", "new_force", (0.75, 0.0, 0.0), 0.10),
        CaseSpec("r162_forward_2n", "new_force", (2.0, 0.0, 0.0), 0.10),
        CaseSpec("r162_forward_4p25n", "new_force", (4.25, 0.0, 0.0), 0.10),
        CaseSpec("r162_backward_2p25n", "new_force", (-2.25, 0.0, 0.0), 0.10),
        CaseSpec("r162_backward_4p25n", "new_force", (-4.25, 0.0, 0.0), 0.10),
        CaseSpec("r162_left_2n", "new_force", (0.0, 2.0, 0.0), 0.10),
        CaseSpec("r162_right_4n", "new_force", (0.0, -4.0, 0.0), 0.10),
        CaseSpec(
            "r162_diagonal_3p75n",
            "new_direction",
            (diagonal, -diagonal, 0.0),
            0.10,
        ),
        CaseSpec("r162_up_2p5n", "new_vertical", (0.0, 0.0, 2.5), 0.10),
        CaseSpec("r162_down_2p5n", "new_vertical", (0.0, 0.0, -2.5), 0.10),
        CaseSpec("r162_forward_2p5n_200ms", "new_duration", (2.5, 0.0, 0.0), 0.20),
        CaseSpec(
            "r162_handle_backward_2p5n",
            "new_application_point",
            (-2.5, 0.0, 0.0),
            0.10,
            body="handle",
        ),
        CaseSpec(
            "r162_right_2n_two_pulses",
            "new_repeated",
            (0.0, -2.0, 0.0),
            0.10,
            repetitions=2,
            repeat_interval_s=0.65,
        ),
    )


def predict_one_step(state: np.ndarray, acceleration: np.ndarray) -> np.ndarray:
    result = np.asarray(state, np.float64).copy()
    # Exact-WBC acceleration layout is roll, lateral, yaw, pitch.
    component_acceleration = (acceleration[0], acceleration[3], acceleration[1], acceleration[2])
    for pair, value in enumerate(component_acceleration):
        position = 2 * pair
        velocity = position + 1
        result[position] += CONTROL_DT * state[velocity] + 0.5 * CONTROL_DT**2 * value
        result[velocity] += CONTROL_DT * value
    return result


def monitor_trace(model: pathlib.Path, trace: dict[str, Any]) -> dict[str, Any]:
    state = np.asarray(trace["execution_forecast_reduced_state"], np.float64)
    acceleration = np.asarray(trace["execution_forecast_achieved_acceleration"], np.float64)
    valid = np.asarray(trace["execution_forecast_path_valid"], np.uint8) != 0
    forecast_support = np.asarray(trace["execution_forecast_support_mask"], np.uint8)
    support = raw_support_masks(trace)
    session = bonesaw.UpkieBalanceSession(str(model))
    session.configure_viability_execution_monitor(
        MINIMUM_SAMPLES,
        RESERVE_MULTIPLIER,
        MINIMUM_NORMALIZED_BOUND,
        MAXIMUM_NORMALIZED_BOUND,
    )
    diagnostics = np.zeros(26, np.float64)
    statuses = np.zeros(len(state), np.uint8)
    certificate = np.zeros(len(state), np.uint8)
    maximum_ratio = np.zeros(len(state), np.float64)
    sample_count = np.zeros(len(state), np.uint32)
    elapsed_ns = np.zeros(len(state), np.uint64)
    allocation_calls = np.zeros(len(state), np.uint64)
    allocated_bytes = np.zeros(len(state), np.uint64)
    predictions = np.zeros_like(state)
    prediction_available = np.zeros(len(state), np.uint8)
    for tick in range(len(state)):
        available = bool(
            tick > 0
            and valid[tick - 1]
            and int(forecast_support[tick - 1]) == int(support[tick - 1])
            and int(support[tick - 1]) == int(support[tick])
        )
        if available:
            predictions[tick] = predict_one_step(state[tick - 1], acceleration[tick - 1])
            prediction_available[tick] = 1
        else:
            predictions[tick] = state[tick]
        elapsed, calls, byte_count = session.step_viability_execution_monitor(
            tick,
            True,
            available,
            int(support[tick]),
            predictions[tick],
            state[tick],
            STATE_SCALES,
            diagnostics,
        )
        statuses[tick] = int(diagnostics[0])
        certificate[tick] = int(diagnostics[1])
        sample_count[tick] = int(diagnostics[3])
        maximum_ratio[tick] = diagnostics[4]
        elapsed_ns[tick] = elapsed
        allocation_calls[tick] = calls
        allocated_bytes[tick] = byte_count

    comparable = prediction_available != 0
    ready = comparable & (sample_count > MINIMUM_SAMPLES)
    covered = statuses == 1
    exceeded = statuses == 2
    exceed_ticks = np.flatnonzero(exceeded)
    recovered = 0
    for tick in exceed_ticks:
        endpoint = min(len(state), int(tick) + 1 + int(round(0.25 / CONTROL_DT)))
        recovered += int(np.any(covered[int(tick) + 1 : endpoint]))
    return {
        "ticks": len(state),
        "comparable_ticks": int(np.sum(comparable)),
        "ready_ticks": int(np.sum(ready)),
        "certificate_ticks": int(np.sum(certificate)),
        "covered_ticks": int(np.sum(covered)),
        "exceeded_ticks": int(np.sum(exceeded)),
        "warmup_ticks": int(np.sum(statuses == 0)),
        "support_reset_ticks": int(np.sum(statuses == 4)),
        "exceedances_recovered_within_250ms": recovered,
        "maximum_error_ratio": float(np.max(maximum_ratio, initial=0.0)),
        "p99_error_ratio": float(np.quantile(maximum_ratio[ready], 0.99)) if np.any(ready) else 0.0,
        "maximum_step_ns": int(np.max(elapsed_ns, initial=0)),
        "p99_step_ns": float(np.quantile(elapsed_ns, 0.99)),
        "allocation_calls": int(np.sum(allocation_calls)),
        "allocated_bytes": int(np.sum(allocated_bytes)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_EXECUTION_RESIDUAL_MONITOR_R162.html")
    parser.add_argument("--cases", help="comma-separated debug-only holdout subset")
    return parser.parse_args()


def select_cases(cases: tuple[CaseSpec, ...], value: str | None) -> tuple[CaseSpec, ...]:
    if value is None:
        return cases
    requested = set(value.split(","))
    selected = tuple(case for case in cases if case.name in requested)
    missing = requested - {case.name for case in selected}
    if missing:
        raise SystemExit(f"unknown cases: {sorted(missing)}")
    return selected


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    cases = select_cases(third_holdout_cases(), args.cases)
    case_results: dict[str, Any] = {}
    replay_exact: dict[str, bool] = {}
    for index, case in enumerate(cases, 1):
        trace = execute(model, case, args.duration)
        replay = execute(model, case, args.duration)
        case_results[case.name] = monitor_trace(model, trace)
        replay_exact[case.name] = semantic_trace_equal(trace, replay)
        result = case_results[case.name]
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: {result['certificate_ticks']} covered, "
            f"{result['exceeded_ticks']} exceeded, replay={'exact' if replay_exact[case.name] else 'DIFF'}",
            flush=True,
        )

    aggregate = {
        key: sum(int(result[key]) for result in case_results.values())
        for key in (
            "ticks",
            "comparable_ticks",
            "ready_ticks",
            "certificate_ticks",
            "covered_ticks",
            "exceeded_ticks",
            "warmup_ticks",
            "support_reset_ticks",
            "exceedances_recovered_within_250ms",
            "allocation_calls",
            "allocated_bytes",
        )
    }
    aggregate.update(
        {
            "maximum_error_ratio": max((v["maximum_error_ratio"] for v in case_results.values()), default=0.0),
            "maximum_step_ns": max((v["maximum_step_ns"] for v in case_results.values()), default=0),
            "maximum_p99_step_ns": max((v["p99_step_ns"] for v in case_results.values()), default=0.0),
        }
    )
    full_holdout = len(cases) == len(third_holdout_cases())
    gates = {
        "third_holdout_complete": full_holdout,
        "third_holdout_disjoint_from_r161": not ({c.name for c in cases} & {c.name for c in r161_cases()}),
        "third_holdout_replay_exact": all(replay_exact.values()),
        "causal_preupdate_monitor_exercised": aggregate["ready_ticks"] > 0,
        "disturbance_exceedance_exercised": aggregate["exceeded_ticks"] > 0,
        "recovery_after_exceedance_exercised": aggregate["exceedances_recovered_within_250ms"] > 0,
        "rust_transition_zero_allocation": aggregate["allocation_calls"] == 0 and aggregate["allocated_bytes"] == 0,
        "bounded_monitor_latency_under_100us": aggregate["maximum_step_ns"] < 100_000,
    }
    mechanism_pass = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "state_names": list(STATE_NAMES),
        "configuration_frozen_before_holdout": True,
        "configuration": {
            "minimum_samples": MINIMUM_SAMPLES,
            "window_samples": 32,
            "reserve_multiplier": RESERVE_MULTIPLIER,
            "minimum_normalized_bound": MINIMUM_NORMALIZED_BOUND.tolist(),
            "maximum_normalized_bound": MAXIMUM_NORMALIZED_BOUND.tolist(),
            "control_dt_s": CONTROL_DT,
        },
        "third_holdout_cases": [case.name for case in cases],
        "replay_exact": replay_exact,
        "case_results": case_results,
        "aggregate": aggregate,
        "gates": gates,
        "mechanism_pass": mechanism_pass,
        "authority_promoted": False,
    }
    rows = [
        [
            name,
            result["comparable_ticks"],
            result["certificate_ticks"],
            result["exceeded_ticks"],
            result["exceedances_recovered_within_250ms"],
            f"{result['maximum_error_ratio']:.3f}",
            f"{result['p99_step_ns']:.0f}",
            "YES" if replay_exact[name] else "NO",
        ]
        for name, result in case_results.items()
    ]
    report = "\n".join(
        [
            "# Bonesaw causal execution-residual monitor · r162",
            "",
            f"> Mechanism gate **{'PASS' if mechanism_pass else 'FAIL'}** · command authority **NOT PROMOTED**.",
            "",
            "## Result",
            "",
            "Static forecast-error cells failed on r160 and r161. This replacement is a fixed-capacity Rust monitor: a 5 ms prediction is authored from the final exact-WBC acceleration, checked against the envelope that existed before the observation arrived, and only then admitted into a 32-sample rolling window. Exact support changes and missing evidence reset warmup.",
            "",
            f"The fresh third holdout contains **{aggregate['comparable_ticks']:,}** comparable ticks: **{aggregate['certificate_ticks']:,}** covered ticks and **{aggregate['exceeded_ticks']:,}** causal exceedances. **{aggregate['exceedances_recovered_within_250ms']:,}** exceedances saw a later covered tick within 250 ms. Worst observed error/bound was **{aggregate['maximum_error_ratio']:.3f}×**; the maximum Rust transition time was **{aggregate['maximum_step_ns']:,} ns**, with **{aggregate['allocation_calls']} allocation calls / {aggregate['allocated_bytes']} bytes** in the measured transition.",
            "",
            "## Third holdout",
            "",
            *markdown_table(
                ["case", "comparable", "covered", "exceeded", "recovered ≤250ms", "worst ratio", "p99 ns", "replay"],
                rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]),
            "",
            "## Authority boundary",
            "",
            "This signal may lower forecast confidence or color a UI health bar. It cannot admit a candidate, alter torque, or turn a previously rejected request into an executable command. A rolling residual envelope is empirical observability, not a reachability proof; promotion still requires a separate causal plant gate.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-execution-residual-monitor-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (output / "UPKIE_EXECUTION_RESIDUAL_MONITOR_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"mechanism_pass": mechanism_pass, "authority_promoted": False, "web_report": str(web_report)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
