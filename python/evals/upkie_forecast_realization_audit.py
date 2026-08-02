#!/usr/bin/env python3
"""Calibrate the r156 reduced forecast against realized Upkie plant state."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, run_case, semantic_trace_equal
from upkie_mujoco_plant_report import CONTROL_DT


REVISION = "upkie-forecast-realization-envelope-r157"
DEFAULT_OUTPUT = f"benchmarks/results/{REVISION}"
DEFAULT_WEB = "web/UPKIE_FORECAST_REALIZATION_ENVELOPE_R157.html"
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
STATE_LIMITS = np.asarray(
    [math.pi / 4.0, 4.0, math.pi / 4.0, 4.0, 0.10, 1.0, math.pi / 4.0, 4.0],
    np.float64,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--web-report", default=DEFAULT_WEB)
    parser.add_argument("--cases", help="comma-separated subset for smoke/debug")
    return parser.parse_args()


def support_masks(trace: dict[str, Any]) -> np.ndarray:
    observed = np.asarray(trace["observed_contact_active"], np.uint8)
    return observed[:, 0] | (observed[:, 1] << 1)


def realized_state(trace: dict[str, Any], tick: int, nominal_y: float) -> np.ndarray:
    rotation = np.asarray(trace["rotation_vector"])[tick]
    twist = np.asarray(trace["root_twist"])[tick]
    position = np.asarray(trace["root_position"])[tick]
    return np.asarray(
        (
            rotation[0],
            twist[0],
            rotation[1],
            twist[1],
            position[1] - nominal_y,
            twist[4],
            rotation[2],
            twist[2],
        ),
        np.float64,
    )


def analyze_trace(trace: dict[str, Any]) -> dict[str, Any]:
    paths = np.asarray(trace["viability_forecast_path"])
    valid = np.asarray(trace["viability_forecast_path_valid"]) != 0
    confirmed = np.asarray(trace["viability_confirmation_executable"]) != 0
    selected = np.flatnonzero(valid & confirmed)
    masks = support_masks(trace)
    nominal_y = float(np.asarray(trace["root_position"])[0, 1])
    samples: list[dict[str, Any]] = []
    incomplete = 0
    for origin in selected:
        origin_mask = int(masks[origin])
        for knot in range(paths.shape[1]):
            time_s = float(paths[origin, knot, 0])
            offset = int(round(time_s / CONTROL_DT))
            if origin + offset >= len(masks):
                incomplete += 1
                continue
            future = origin + offset
            predicted = paths[origin, knot, 1:]
            actual = realized_state(trace, future, nominal_y)
            absolute = np.abs(actual - predicted)
            changed = bool(np.any(masks[origin + 1 : future + 1] != origin_mask))
            samples.append(
                {
                    "origin": int(origin),
                    "knot": knot + 1,
                    "time_s": time_s,
                    "support_changed": changed,
                    "absolute_error": absolute,
                    "normalized_max_error": float(np.max(absolute / STATE_LIMITS)),
                }
            )
    return {
        "selected_path_count": int(len(selected)),
        "complete_knot_samples": len(samples),
        "incomplete_knot_samples": incomplete,
        "samples": samples,
    }


def distribution(values: np.ndarray) -> dict[str, float]:
    if values.size == 0:
        return {key: 0.0 for key in ("p50", "p95", "p99", "max")}
    return {
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected = case_matrix()
    if args.cases:
        names = set(args.cases.split(","))
        selected = tuple(case for case in selected if case.name in names)
        missing = names - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    model = pathlib.Path(args.model).resolve()
    case_results: dict[str, Any] = {}
    all_samples: list[dict[str, Any]] = []
    exact_replay = True
    zero_allocation = True
    for index, case in enumerate(selected, 1):
        kwargs = dict(
            balance_mode="capture",
            fall_safe_enabled=True,
            fall_safe_primary_blend=False,
            measured_contact_admission=True,
            execute_reduced_support=True,
            viability_planner_enabled=True,
            viability_support_requires_active_request=False,
            viability_planner_strategy="multistep_budgeted",
            viability_confirmation_updates=2,
        )
        trace = run_case(model, case, args.duration, **kwargs)
        replay = run_case(model, case, args.duration, **kwargs)
        analysis = analyze_trace(trace)
        replay_equal = semantic_trace_equal(trace, replay)
        exact_replay &= replay_equal
        zero_allocation &= bool(np.all(np.asarray(trace["allocation_calls"]) == 0))
        case_samples = analysis.pop("samples")
        for sample in case_samples:
            sample["case"] = case.name
        all_samples.extend(case_samples)
        case_results[case.name] = {**analysis, "exact_replay": replay_equal}
        print(
            f"[{index:02d}/{len(selected)}] {case.name}: "
            f"paths={analysis['selected_path_count']}, "
            f"knots={analysis['complete_knot_samples']}, replay={replay_equal}",
            flush=True,
        )

    errors = (
        np.stack([sample["absolute_error"] for sample in all_samples])
        if all_samples
        else np.empty((0, len(STATE_NAMES)), np.float64)
    )
    normalized = np.asarray(
        [sample["normalized_max_error"] for sample in all_samples], np.float64
    )
    changed = np.asarray(
        [sample["support_changed"] for sample in all_samples], bool
    )
    knot_rows = []
    knot_metrics: dict[str, Any] = {}
    for knot in range(1, 9):
        mask = np.asarray([sample["knot"] == knot for sample in all_samples], bool)
        knot_values = normalized[mask]
        stats = distribution(knot_values)
        knot_metrics[str(knot)] = {
            "time_s": 0.03 * knot,
            "sample_count": int(np.sum(mask)),
            "support_changed_count": int(np.sum(changed[mask])),
            "normalized_max_error": stats,
        }
        knot_rows.append(
            [
                knot,
                f"{0.03 * knot:.2f}",
                int(np.sum(mask)),
                int(np.sum(changed[mask])),
                f"{stats['p50']:.3f}",
                f"{stats['p95']:.3f}",
                f"{stats['p99']:.3f}",
                f"{stats['max']:.3f}",
            ]
        )

    state_metrics = {
        name: distribution(errors[:, axis])
        for axis, name in enumerate(STATE_NAMES)
    }
    same_support_stats = distribution(normalized[~changed])
    changed_support_stats = distribution(normalized[changed])
    full_matrix = len(case_results) == len(case_matrix())
    gates = {
        "matrix_complete": full_matrix,
        "forecast_paths_exercised": len(all_samples) > 0,
        "support_transition_paths_exercised": bool(np.any(changed)),
        "path_replay_exact": exact_replay,
        "finite": bool(np.all(np.isfinite(errors)) and np.all(np.isfinite(normalized))),
        "zero_timed_rust_allocation": zero_allocation,
        "fixed_knot_times_exact": all(
            abs(sample["time_s"] - 0.03 * sample["knot"]) <= 1.0e-12
            for sample in all_samples
        ),
    }
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "passed": passed,
        "gates": gates,
        "case_count": len(case_results),
        "forecast_path_count": sum(
            row["selected_path_count"] for row in case_results.values()
        ),
        "complete_knot_samples": len(all_samples),
        "incomplete_knot_samples": sum(
            row["incomplete_knot_samples"] for row in case_results.values()
        ),
        "support_changed_samples": int(np.sum(changed)),
        "same_support_samples": int(np.sum(~changed)),
        "normalized_max_error": distribution(normalized),
        "same_support_normalized_max_error": same_support_stats,
        "changed_support_normalized_max_error": changed_support_stats,
        "state_absolute_error": state_metrics,
        "by_knot": knot_metrics,
        "cases": case_results,
        "scope": "r156 confirmed planner paths compared with later MuJoCo state; calibration evidence only, no online bound or promoted action",
    }

    state_rows = [
        [
            name,
            f"{state_metrics[name]['p50']:.5g}",
            f"{state_metrics[name]['p95']:.5g}",
            f"{state_metrics[name]['p99']:.5g}",
            f"{state_metrics[name]['max']:.5g}",
        ]
        for name in STATE_NAMES
    ]
    report = "\n".join(
        [
            "# Bonesaw forecast-versus-realized envelope · r157",
            "",
            f"> Measurement gate **{'PASS' if passed else 'FAIL'}**. This is calibration evidence, not an executable safety certificate.",
            "",
            "## Result",
            "",
            f"The retained r156 two-update confirmation produced **{metrics['forecast_path_count']}** executable forecast origins and **{len(all_samples)}** complete knot comparisons. **{int(np.sum(changed))}** comparisons cross at least one raw support change. Replay is **{'exact' if exact_replay else 'not exact'}** and timed Rust allocation is **{'zero' if zero_allocation else 'nonzero'}**.",
            "",
            f"Same-support normalized max error reaches p50/p95/p99/max **{same_support_stats['p50']:.3f}/{same_support_stats['p95']:.3f}/{same_support_stats['p99']:.3f}/{same_support_stats['max']:.3f}** of the forecast limits. Support-changing comparisons reach **{changed_support_stats['p50']:.3f}/{changed_support_stats['p95']:.3f}/{changed_support_stats['p99']:.3f}/{changed_support_stats['max']:.3f}**. These empirical errors are descriptive and must not be consumed online until coverage, uncertainty, and an upper-confidence rule are admitted.",
            "",
            "## Error by knot",
            "",
            *markdown_table(
                ["knot", "t s", "samples", "support changed", "p50", "p95", "p99", "max"],
                knot_rows,
            ),
            "",
            "## Absolute state error",
            "",
            *markdown_table(["state", "p50", "p95", "p99", "max"], state_rows),
            "",
            "## Measurement gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Contract",
            "",
            "- Rust emits the exact eight fixed reduced-order knots used by scoring into caller-owned `[8, 9]` storage with validation-before-mutation and zero hot-path allocation.",
            "- Python aligns each confirmed origin with later 200 Hz plant observations at 30 ms intervals and keeps same-support and support-changing evidence separate.",
            "- This experiment does not train a policy, alter the live r137 worker, infer a worst-case bound from a percentile, or claim that MuJoCo is hardware.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-forecast-realization-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_FORECAST_REALIZATION_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "web_report": str(web)}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
