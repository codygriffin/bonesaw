#!/usr/bin/env python3
"""Separate steady sensor delay from a delay-onset timestamp discontinuity."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_current_support_realization_plant_ab import (
    exact_terminal_outcome,
    plant_command_trace_equal,
)
from upkie_disturbance_envelope import semantic_trace_equal


REVISION = "upkie-observation-delay-startup-ab-r183"
DURATION_S = 6.0
PROFILES: dict[str, dict[str, int]] = {
    "exact": {},
    "delay_onset_20ms": {"contact_observation_delay_ticks": 4},
    "steady_delay_20ms": {
        "contact_observation_delay_ticks": 4,
        "contact_observation_prestart_age_ticks": 4,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_OBSERVATION_DELAY_STARTUP_AB_R183.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def enrich(run: dict[str, Any]) -> dict[str, Any]:
    trace = run["trace"]
    metrics = run["metrics"]
    status = np.asarray(trace["contact_observation_status"])
    torque = np.asarray(trace["torque"])
    metrics.update(
        {
            "timestamp_fault_ticks": int(np.sum(status == 7)),
            "zero_torque_ticks": int(
                np.sum(np.linalg.norm(torque, axis=1) == 0.0)
            ),
            "first_selection": int(
                trace["contact_program_authority_selection"][0]
            ),
        }
    )
    return run


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(
            case for case in selected_cases if case.name in requested
        )
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    model = pathlib.Path(args.model).resolve()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs = {
            name: enrich(execute(model, case, args.duration, profile))
            for name, profile in PROFILES.items()
        }
        replays = {
            name: enrich(execute(model, case, args.duration, profile))
            for name, profile in PROFILES.items()
        }
        exact = runs["exact"]
        rows[case.name] = {}
        for name, run in runs.items():
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
                "plant_command_exact_to_exact": plant_command_trace_equal(
                    exact["trace"], run["trace"]
                ),
                "terminal_exact_to_exact": exact_terminal_outcome(
                    exact["metrics"], run["metrics"]
                ),
            }
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            f"exact={outcome(exact['metrics'])}, "
            f"onset={outcome(runs['delay_onset_20ms']['metrics'])}, "
            f"steady={outcome(runs['steady_delay_20ms']['metrics'])}",
            flush=True,
        )

    full_matrix = len(rows) == len(cases())
    all_arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    onset = [case_rows["delay_onset_20ms"] for case_rows in rows.values()]
    steady = [case_rows["steady_delay_20ms"] for case_rows in rows.values()]
    gates = {
        "matrix_complete": full_matrix,
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in all_arms),
        "delay_onset_exercises_timestamp_fault": all(
            arm["metrics"]["timestamp_fault_ticks"] == 4 for arm in onset
        ),
        "delay_onset_timestamp_faults_fail_closed": all(
            arm["metrics"]["withheld_ticks"]
            >= arm["metrics"]["timestamp_fault_ticks"]
            and arm["metrics"]["zero_torque_ticks"]
            >= arm["metrics"]["timestamp_fault_ticks"]
            for arm in onset
        ),
        "steady_delay_is_20ms_from_first_tick": all(
            arm["metrics"]["maximum_observation_age_ns"] == 20_000_000
            and arm["metrics"]["timestamp_fault_ticks"] == 0
            for arm in steady
        ),
        "steady_delay_never_withholds": all(
            arm["metrics"]["withheld_ticks"] == 0 for arm in steady
        ),
        "steady_delay_plant_and_command_are_bit_exact": all(
            arm["plant_command_exact_to_exact"] for arm in steady
        ),
        "steady_delay_terminal_outcome_is_exact": all(
            arm["terminal_exact_to_exact"] for arm in steady
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in all_arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            for arm in all_arms
        ),
    }
    timing = {
        "loop_overruns": sum(
            arm["metrics"]["loop_overruns"] for arm in all_arms
        ),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in all_arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in all_arms
        ),
    }
    timing_gates = {
        "zero_5ms_loop_overruns": timing["loop_overruns"] == 0,
        "all_controller_calls_below_5ms": (
            timing["controller_step_ns_maximum"] <= 5_000_000
        ),
    }
    mechanism_passed = all(gates.values())
    synchronous_profile_admitted = mechanism_passed and all(timing_gates.values())

    table = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact"]["metrics"]
        for profile_name in ("delay_onset_20ms", "steady_delay_20ms"):
            arm = case_rows[profile_name]
            metrics = arm["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not metrics["fell"]
                else f"{float(metrics['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            table.append(
                [
                    case_name,
                    profile_name,
                    outcome(exact),
                    outcome(metrics),
                    delta,
                    metrics["timestamp_fault_ticks"],
                    metrics["withheld_ticks"],
                    "YES" if arm["plant_command_exact_to_exact"] else "NO",
                ]
            )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "gates": gates,
        "timing": timing,
        "timing_gates": timing_gates,
        "profiles": PROFILES,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw observation-delay startup A/B · r183",
            "",
            f"> Causal classification **{'PASS' if mechanism_passed else 'FAIL'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**. This audit distinguishes a steady delayed stream from introducing delay discontinuously when actuator authority begins.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "case",
                    "profile",
                    "exact",
                    "candidate",
                    "fall Δ s",
                    "timestamp faults",
                    "withheld",
                    "plant/command exact",
                ],
                table,
            ),
            "",
            "## Causal gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Timing",
            "",
            *markdown_table(
                ["metric", "value"],
                [
                    ["loop misses >5 ms", timing["loop_overruns"]],
                    ["worst loop", f"{timing['loop_ns_maximum'] / 1e6:.3f} ms"],
                    [
                        "worst controller call",
                        f"{timing['controller_step_ns_maximum'] / 1e6:.3f} ms",
                    ],
                ],
            ),
            "",
            "## Interpretation",
            "",
            "- The r182 20 ms arm is a delay-onset fault: zero-age prestart samples are followed by older runtime samples. Rust correctly rejects and withholds on four backwards timestamps; most rows need one additional activation tick before authority returns.",
            "- The steady-delay arm advances the receiver clock, then primes three already-aged samples before actuator authority. Every runtime sample is exactly 20 ms old and monotonically newer than the previous acquisition.",
            "- Exact plant/command equality under steady delay is specific to r181 fixed-effective-effort realization: observation age changes the proof route, but it cannot change the applied torque. It is not a generic claim that delayed contact is harmless.",
            "- Periodic unavailability remains a real open consequence problem; this audit does not reclassify the r182 dropout regressions.",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-observation-delay-startup-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_OBSERVATION_DELAY_STARTUP_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "synchronous_profile_admitted": synchronous_profile_admitted,
                "gates": gates,
                "timing_gates": timing_gates,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
