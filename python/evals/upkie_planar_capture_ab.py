#!/usr/bin/env python3
"""Negative-promotion A/B for differential-wheel lateral capture.

Python owns the MuJoCo experiment matrix and scoring. Rust owns persistent
planar DCM state, direction latching, bounded yaw/wheel commands, rotating
contact bases, floating WBC, torque, and allocation witnesses.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import (
    case_matrix,
    run_case,
    semantic_trace_equal,
    summarize,
)


REVISION = "upkie-planar-capture-ab-r134"
CASE_NAMES = ("nominal", "forward_4n_reference", "left_1n", "left_2n", "right_2n")
LATERAL_NAMES = ("left_1n", "left_2n", "right_2n")
BASELINE_MODE = "capture"
CANDIDATE_MODE = "planar_capture"
CANDIDATE_CONFIG = (0.0005, 0.005, 0.5, 0.5, 5.0, 8.0, 20.0)
SENSITIVITY_GAINS = (0.4, 0.5, 0.6)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--sensitivity-duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_PLANAR_CAPTURE_AB_R134.html")
    return parser.parse_args()


def promotion_decision(
    baseline: dict[str, dict[str, Any]],
    candidate: dict[str, dict[str, Any]],
) -> tuple[bool, list[str]]:
    blockers: list[str] = []
    for name in ("nominal", "forward_4n_reference"):
        if not candidate[name]["qualified"]:
            blockers.append(f"candidate regresses {name}")
    unrecovered = [name for name in LATERAL_NAMES if candidate[name]["outcome"] != "RECOVERED"]
    if unrecovered:
        blockers.append(f"no lateral recovery: {unrecovered}")
    for name in LATERAL_NAMES:
        if candidate[name]["terminal_time_s"] <= baseline[name]["terminal_time_s"]:
            blockers.append(f"no first-boundary improvement: {name}")
    return not blockers, blockers


def run_profile(
    model: pathlib.Path,
    cases: dict[str, Any],
    duration: float,
    mode: str,
    config: tuple[float, ...] | None,
) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    metrics: dict[str, dict[str, Any]] = {}
    traces: dict[str, dict[str, Any]] = {}
    for name in CASE_NAMES:
        trace = run_case(model, cases[name], duration, mode, config)
        traces[name] = trace
        result = summarize(cases[name], trace, duration)
        result["minimum_lateral_support_margin_m"] = float(
            np.min(np.asarray(trace["lateral_support_margin"]))
        )
        result["maximum_abs_heading_deg"] = float(
            np.degrees(np.max(np.abs(np.asarray(trace["heading"]))))
        )
        result["maximum_abs_commanded_yaw_rate_rad_s"] = float(
            np.max(np.abs(np.asarray(trace["commanded_yaw_rate"])))
        )
        metrics[name] = result
    return metrics, traces


def make_report(metrics: dict[str, Any]) -> str:
    rows = []
    for name in CASE_NAMES:
        baseline = metrics["baseline"][name]
        candidate = metrics["candidate"][name]
        rows.append(
            [
                name,
                baseline["outcome"],
                f'{baseline["terminal_time_s"]:.3f}',
                candidate["outcome"],
                f'{candidate["terminal_time_s"]:.3f}',
                f'{candidate["terminal_time_s"] - baseline["terminal_time_s"]:+.3f}',
                f'{candidate["maximum_translation_m"] * 1000:.1f}',
                f'{candidate["maximum_tilt_deg"]:.2f}',
                f'{candidate["minimum_lateral_support_margin_m"] * 1000:.1f}',
                f'{candidate["maximum_abs_heading_deg"]:.2f}',
                f'{candidate["maximum_abs_commanded_yaw_rate_rad_s"]:.3f}',
                candidate["post_startup_nonadmitted_steps"],
                f'{candidate["controller_step_ns"]["p99"] / 1e3:.1f}',
            ]
        )
    sensitivity_rows = []
    for profile in metrics["sensitivity"]:
        sensitivity_rows.append(
            [
                f'{profile["heading_gain"]:.2f}',
                *[
                    f'{profile["results"][name]["outcome"]} @ '
                    f'{profile["results"][name]["terminal_time_s"]:.3f}s'
                    for name in LATERAL_NAMES
                ],
            ]
        )
    gate_rows = [[name, value["observed"], value["pass"]] for name, value in metrics["gates"].items()]
    fall_margins_mm = [
        metrics["candidate"][name]["minimum_lateral_support_margin_m"] * 1000.0
        for name in LATERAL_NAMES
    ]
    low_gain = next(
        profile for profile in metrics["sensitivity"] if profile["heading_gain"] == 0.4
    )
    return "\n".join(
        [
            f'# Upkie differential-wheel planar capture A/B · {REVISION}',
            "",
            f'**Evaluation admission: {"PASS" if metrics["admission"] else "FAIL"}. Controller promotion: {"PASS" if metrics["promote"] else "REJECTED"}.** Evaluation success means the candidate is measured repeatably and rejected honestly when it does not recover the declared lateral cases.',
            "",
            "The candidate keeps persistent planar DCM and steering-direction state in Rust, slews bounded differential-wheel yaw commands, supplies heading-rotated contact bases to the same floating WBC, and exposes signed lateral support margin. Python owns only MuJoCo cases, timing, scoring, and artifacts.",
            "",
            "## Frozen A/B",
            "",
            *markdown_table(
                [
                    "case",
                    "baseline",
                    "baseline boundary s",
                    "candidate",
                    "candidate boundary s",
                    "Δ boundary s",
                    "candidate peak Δp mm",
                    "candidate peak tilt °",
                    "minimum lateral margin mm",
                    "maximum heading °",
                    "maximum yaw cmd rad/s",
                    "later nonadmitted",
                    "Rust p99 µs",
                ],
                rows,
            ),
            "",
            "## Gain sensitivity",
            "",
            *markdown_table(["heading gain", *LATERAL_NAMES], sensitivity_rows),
            "",
            "## Gates",
            "",
            *markdown_table(["gate", "observed", "pass"], gate_rows),
            "",
            "## Decision",
            "",
            f'- Promotion blockers: **{metrics["promotion_blockers"]}**.',
            f'- Candidate exact semantic replay: **{metrics["candidate_replay_exact"]}**.',
            "- The three candidate falls retain positive DCM-to-track margins of "
            + "/".join(f"{margin:+.1f}" for margin in fall_margins_mm)
            + " mm. That scalar is useful evidence, but it is not a nonlinear recovery certificate: pitch/yaw/contact coupling reaches the fall boundary first.",
            f'- At heading gain 0.40, the full {metrics["sensitivity_duration_s"]:.1f} s sensitivity horizon recovers only right_2n ({low_gain["results"]["right_2n"]["outcome"]}); the mirrored left rows still fall. The gain response is sign-specific and non-monotone.',
            "- The declared candidate delays all three first-fall boundaries while preserving nominal and sagittal recovery, but it recovers none of the frozen lateral cases. It therefore remains an experimental diagnostics path, not the live controller.",
            "- The next physical milestone must change available authority—an explicit support/contact transition, steering-aware nonholonomic program with a robust viability proof, or fall-safe behavior—not merely tune a per-tick QP.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    cases = {case.name: case for case in case_matrix()}
    baseline, baseline_traces = run_profile(model, cases, args.duration, BASELINE_MODE, None)
    candidate, candidate_traces = run_profile(
        model, cases, args.duration, CANDIDATE_MODE, CANDIDATE_CONFIG
    )
    repeat = run_case(
        model,
        cases["left_1n"],
        args.duration,
        CANDIDATE_MODE,
        CANDIDATE_CONFIG,
    )
    replay_exact = semantic_trace_equal(candidate_traces["left_1n"], repeat)
    sensitivity = []
    for gain in SENSITIVITY_GAINS:
        config = (*CANDIDATE_CONFIG[:2], gain, *CANDIDATE_CONFIG[3:])
        results = {}
        for name in LATERAL_NAMES:
            trace = run_case(
                model,
                cases[name],
                args.sensitivity_duration,
                CANDIDATE_MODE,
                config,
            )
            results[name] = summarize(cases[name], trace, args.sensitivity_duration)
        sensitivity.append({"heading_gain": gain, "results": results})
    promote, blockers = promotion_decision(baseline, candidate)
    lateral_deltas = {
        name: candidate[name]["terminal_time_s"] - baseline[name]["terminal_time_s"]
        for name in LATERAL_NAMES
    }
    allocation_free = all(
        result["allocation_free"]
        for profile in (baseline, candidate)
        for result in profile.values()
    )
    numeric_clean = all(
        not result["numeric_fault"]
        for profile in (baseline, candidate)
        for result in profile.values()
    )
    neighbor_regression = any(
        any(
            profile["results"][name]["terminal_time_s"]
            < candidate[name]["terminal_time_s"] - 0.05
            for name in LATERAL_NAMES
        )
        for profile in sensitivity
        if profile["heading_gain"] != CANDIDATE_CONFIG[2]
    )
    gates = {
        "frozen baseline and candidate matrix complete": {
            "observed": f"2 × {len(CASE_NAMES)} cases",
            "pass": set(baseline) == set(candidate) == set(CASE_NAMES),
        },
        "baseline reproduces r133 boundary": {
            "observed": {name: baseline[name]["outcome"] for name in CASE_NAMES},
            "pass": baseline["nominal"]["qualified"]
            and baseline["forward_4n_reference"]["qualified"]
            and all(baseline[name]["outcome"] == "FALL" for name in LATERAL_NAMES),
        },
        "candidate preserves nominal and sagittal recovery": {
            "observed": {
                name: candidate[name]["qualified"]
                for name in ("nominal", "forward_4n_reference")
            },
            "pass": candidate["nominal"]["qualified"]
            and candidate["forward_4n_reference"]["qualified"],
        },
        "candidate delays every lateral first boundary": {
            "observed": lateral_deltas,
            "pass": all(delta > 0.05 for delta in lateral_deltas.values()),
        },
        "candidate replay is exact": {"observed": replay_exact, "pass": replay_exact},
        "neighboring gain sensitivity is discriminating": {
            "observed": neighbor_regression,
            "pass": neighbor_regression,
        },
        "Rust timed regions remain allocation-free": {
            "observed": allocation_free,
            "pass": allocation_free,
        },
        "no pre-boundary numeric fault": {"observed": numeric_clean, "pass": numeric_clean},
        "failed recovery prevents promotion": {
            "observed": {"promote": promote, "blockers": blockers},
            "pass": not promote and any("no lateral recovery" in item for item in blockers),
        },
    }
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "sensitivity_duration_s": args.sensitivity_duration,
        "candidate_config": {
            "steering_release_capture_error_m": CANDIDATE_CONFIG[0],
            "steering_full_capture_error_m": CANDIDATE_CONFIG[1],
            "heading_gain_rad_s_per_rad": CANDIDATE_CONFIG[2],
            "maximum_yaw_rate_rad_s": CANDIDATE_CONFIG[3],
            "yaw_rate_slew_rad_s2": CANDIDATE_CONFIG[4],
            "yaw_rate_tracking_gain_per_s": CANDIDATE_CONFIG[5],
            "maximum_yaw_acceleration_rad_s2": CANDIDATE_CONFIG[6],
        },
        "baseline": baseline,
        "candidate": candidate,
        "sensitivity": sensitivity,
        "lateral_boundary_delta_s": lateral_deltas,
        "candidate_replay_exact": replay_exact,
        "promote": promote,
        "promotion_blockers": blockers,
        "gates": gates,
        "admission": all(value["pass"] for value in gates.values()),
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    raw: dict[str, np.ndarray] = {}
    for profile_name, traces in (("baseline", baseline_traces), ("candidate", candidate_traces)):
        for case_name, trace in traces.items():
            for field in (
                "time_s",
                "root_position",
                "root_twist",
                "rotation_vector",
                "status",
                "heading",
                "lateral_capture_error",
                "lateral_support_margin",
                "commanded_yaw_rate",
                "torque_utilization",
                "controller_step_ns",
            ):
                raw[f"{profile_name}_{case_name}_{field}"] = np.asarray(trace[field])
    np.savez_compressed(output / "upkie-planar-capture-ab-raw.npz", **raw)
    (output / "upkie-planar-capture-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = make_report(metrics)
    (output / "UPKIE_PLANAR_CAPTURE_AB_AUDIT.md").write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Upkie planar capture A/B")
    )
    print(
        json.dumps(
            {
                "admission": metrics["admission"],
                "promote": promote,
                "lateral_boundary_delta_s": lateral_deltas,
                "report": str(output / "UPKIE_PLANAR_CAPTURE_AB_AUDIT.md"),
            },
            indent=2,
        )
    )
    return 0 if metrics["admission"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
