#!/usr/bin/env python3
"""Retained plant sweep for the Upkie DCM velocity-reference fraction."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

import upkie_mujoco_plant_report as plant
from cpu_reference_report import markdown_table, render_report_html


REVISION = "upkie-capture-fraction-sweep-r130"
DEFAULT_FRACTIONS = (0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--push-start", type=float, default=1.0)
    parser.add_argument("--push-duration", type=float, default=0.1)
    parser.add_argument("--push-force", type=float, default=4.0)
    parser.add_argument(
        "--fractions",
        default=",".join(str(value) for value in DEFAULT_FRACTIONS),
    )
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_CAPTURE_FRACTION_SWEEP_R130.html"
    )
    return parser.parse_args()


def fraction_key(value: float) -> str:
    return f"fraction_{value:.3f}".replace(".", "p")


def qualifies(summary: dict[str, Any], duration: float) -> tuple[bool, dict[str, bool]]:
    startup_ticks = int(round(plant.STARTUP_TRANSIENT_S / plant.CONTROL_DT))
    gates = {
        "no_fall": not summary["fell"],
        "tilt_recovery": summary["recovery_time_s"] is not None,
        "station_reentry": summary["station_reentry_time_s"] is not None
        and abs(summary["final_station_error_m"]) < 0.05
        and summary["final_station_authority"] > 0.99,
        "bounded_startup": summary["maximum_consecutive_nonadmitted_steps"]
        <= startup_ticks
        and summary["last_nonadmitted_time_s"] is not None
        and summary["last_nonadmitted_time_s"]
        <= plant.STARTUP_TRANSIENT_S + 1.0e-12,
        "admitted_after_startup": summary["post_startup_nonadmitted_steps"] == 0,
        "allocation_free": summary["controller_allocation_calls"] == 0
        and summary["controller_allocated_bytes"] == 0,
        "loop_budget": summary["execution_over_5ms_steps"] == 0,
        "contact_reacquired": summary["maximum_contactless_duration_s"] <= 0.25
        and (
            summary["last_contactless_time_s"] is None
            or summary["last_contactless_time_s"] < duration - 0.4
        ),
    }
    return all(gates.values()), gates


def make_report(metrics: dict[str, Any]) -> str:
    rows = []
    for result in metrics["results"]:
        rows.append(
            [
                f'{result["fraction"]:.2f}',
                result["qualified"],
                f'{math.degrees(result["maximum_tilt_rad"]):.2f}',
                "—"
                if result["recovery_time_s"] is None
                else f'{result["recovery_time_s"]:.3f}',
                "—"
                if result["station_reentry_time_s"] is None
                else f'{result["station_reentry_time_s"]:.3f}',
                f'{result["maximum_root_excursion_m"]:.4f}',
                f'{result["final_station_error_m"]:.6f}',
                result["post_startup_nonadmitted_steps"],
                f'{result["controller_step_ns"]["p99"] / 1e3:.1f}',
                result["execution_over_5ms_steps"],
            ]
        )
    largest = metrics["largest_qualified_fraction"]
    largest_text = "none" if largest is None else f"{largest:.2f}"
    selected = metrics["recommended_fraction"]
    selected_text = "none" if selected is None else f"{selected:.2f}"
    return "\n".join(
        [
            f'# Upkie capture-fraction sweep · {metrics["revision"]}',
            "",
            f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** Seven predeclared actuator-facing fractions of the full DCM velocity offset are exercised against the same 10-second, 4 N × 100 ms forward-push MuJoCo plant. This is a parameter robustness study, not a learned-policy comparison. Python owns the plant and scoring; Rust owns capture/station semantics, reference PI state, WBC, admission, and allocation evidence.',
            "",
            "The full DCM remains the viability-pressure witness at every fraction. The swept value only changes how much of `com_velocity / omega` is presented as a pitch reference to the Upkie-matched PI loop.",
            "",
            "## Results",
            "",
            *markdown_table(
                [
                    "fraction",
                    "qualified",
                    "peak tilt deg",
                    "tilt recovery s",
                    "station re-entry s",
                    "peak Δx m",
                    "final station error m",
                    "later rejects",
                    "Rust p99 µs",
                    ">5 ms",
                ],
                rows,
            ),
            "",
            f'- Largest plant-qualified fraction: **{largest_text}**.',
            f'- Recommended fraction: **{selected_text}**, selected for the fastest station re-entry among qualified rows rather than maximum velocity feedback.',
            f'- Frozen default under review: **{metrics["default_fraction"]:.2f}**; qualified: **{metrics["default_fraction_qualified"]}**.',
            f'- Sweep is discriminating above the qualified envelope: **{metrics["higher_fraction_failure_observed"]}**.',
            "- Every qualified row requires no fall, tilt recovery, station-authority re-entry, final station error below 5 cm, bounded fail-closed startup, continuous admission afterward, contact reacquisition, zero Rust timed-region allocations, and zero 5 ms loop overruns.",
            "",
            "## Boundary and limits",
            "",
            "Each fraction receives a fresh plant, WBC session, PI state, and balanced-standing projection. The retained NPZ contains root pose, capture pressure, station error/authority, status, and controller/loop timing for every tick and fraction. This sweep covers one sagittal impulse, one contact model, and one ideal-observation condition; it does not establish lateral, terrain, delay/noise, thermal, or hardware robustness.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    fractions = tuple(float(value) for value in args.fractions.split(","))
    if not fractions or any(not math.isfinite(value) or not 0.0 <= value <= 1.0 for value in fractions):
        raise ValueError("fractions must be finite values inside [0, 1]")
    if tuple(sorted(set(fractions))) != fractions:
        raise ValueError("fractions must be unique and sorted")
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    raw: dict[str, np.ndarray] = {}
    for fraction in fractions:
        trace = plant.run_case(
            model_path,
            duration=args.duration,
            push_start=args.push_start,
            push_duration=args.push_duration,
            push_force=args.push_force,
            contact_model="soft",
            balance_mode="capture",
            capture_velocity_fraction=fraction,
        )
        summary = plant.summarize_case(
            trace, args.push_start + args.push_duration
        )
        finite = all(
            np.all(np.isfinite(value))
            for value in trace.values()
            if isinstance(value, np.ndarray) and value.dtype.kind == "f"
        )
        qualified, gates = qualifies(summary, args.duration)
        gates["finite_trace"] = finite
        qualified = qualified and finite
        results.append(
            {
                "fraction": fraction,
                "qualified": qualified,
                "gates": gates,
                **summary,
            }
        )
        key = fraction_key(fraction)
        for field in (
            "time_s",
            "root_position",
            "rotation_vector",
            "capture_error",
            "capture_pressure",
            "station_error",
            "station_authority",
            "commanded_ground_velocity",
            "status",
            "controller_step_ns",
            "loop_ns",
        ):
            raw[f"{key}_{field}"] = np.asarray(trace[field])
    qualified_fractions = [
        result["fraction"] for result in results if result["qualified"]
    ]
    largest_qualified = max(qualified_fractions, default=None)
    recommended_result = min(
        (result for result in results if result["qualified"]),
        key=lambda result: result["station_reentry_time_s"],
        default=None,
    )
    recommended_fraction = (
        None if recommended_result is None else recommended_result["fraction"]
    )
    default_fraction = 0.2
    default_result = next(
        (result for result in results if result["fraction"] == default_fraction), None
    )
    higher_failure = largest_qualified is not None and any(
        result["fraction"] > largest_qualified and not result["qualified"]
        for result in results
    )
    admission = bool(
        default_result is not None
        and default_result["qualified"]
        and recommended_fraction == default_fraction
        and largest_qualified is not None
        and higher_failure
    )
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "machine": platform.machine()},
        "mujoco_version": mujoco.__version__,
        "model": str(model_path),
        "model_sha256": plant.sha256(model_path),
        "control_dt_s": plant.CONTROL_DT,
        "physics_dt_s": plant.PHYSICS_DT,
        "duration_s": args.duration,
        "push": {
            "start_s": args.push_start,
            "duration_s": args.push_duration,
            "force_x_n": args.push_force,
            "impulse_ns": args.push_force * args.push_duration,
        },
        "fractions": fractions,
        "default_fraction": default_fraction,
        "default_fraction_qualified": bool(
            default_result is not None and default_result["qualified"]
        ),
        "largest_qualified_fraction": largest_qualified,
        "recommended_fraction": recommended_fraction,
        "higher_fraction_failure_observed": higher_failure,
        "results": results,
        "admission": admission,
    }
    np.savez_compressed(output / "upkie-capture-fraction-sweep-raw.npz", **raw)
    metrics_path = output / "upkie-capture-fraction-sweep-metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path = output / "UPKIE_CAPTURE_FRACTION_SWEEP_AUDIT.md"
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Upkie capture-fraction sweep")
    )
    print(
        json.dumps(
            {
                "admission": admission,
                "largest_qualified_fraction": largest_qualified,
                "metrics": str(metrics_path),
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if admission else 1


if __name__ == "__main__":
    raise SystemExit(main())
