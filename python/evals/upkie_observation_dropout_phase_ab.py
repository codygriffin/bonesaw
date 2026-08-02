#!/usr/bin/env python3
"""Separate startup dropout from established-authority periodic dropout."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_current_support_realization_plant_ab import exact_terminal_outcome
from upkie_disturbance_envelope import semantic_trace_equal


REVISION = "upkie-observation-dropout-phase-ab-r184"
DURATION_S = 6.0
PROFILES: dict[str, dict[str, int]] = {
    "exact": {},
    "startup_5ms_per_250ms": {
        "contact_observation_dropout_period_ticks": 50,
        "contact_observation_dropout_burst_ticks": 1,
    },
    "established_5ms_per_250ms": {
        "contact_observation_dropout_period_ticks": 50,
        "contact_observation_dropout_burst_ticks": 1,
        "contact_observation_dropout_start_tick": 50,
    },
    "startup_10ms_per_500ms": {
        "contact_observation_dropout_period_ticks": 100,
        "contact_observation_dropout_burst_ticks": 2,
    },
    "established_10ms_per_500ms": {
        "contact_observation_dropout_period_ticks": 100,
        "contact_observation_dropout_burst_ticks": 2,
        "contact_observation_dropout_start_tick": 100,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_OBSERVATION_DROPOUT_PHASE_AB_R184.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


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
            name: execute(model, case, args.duration, profile)
            for name, profile in PROFILES.items()
        }
        replays = {
            name: execute(model, case, args.duration, profile)
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
                "terminal_exact_to_exact": exact_terminal_outcome(
                    exact["metrics"], run["metrics"]
                ),
            }
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
            ),
            flush=True,
        )

    all_arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    dropout_arms = [
        (case_name, profile_name, arm)
        for case_name, case_rows in rows.items()
        for profile_name, arm in case_rows.items()
        if profile_name != "exact"
    ]
    gates = {
        "matrix_complete": len(rows) == len(cases()),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in all_arms),
        "all_dropout_profiles_exercised": all(
            arm["metrics"]["unavailable_observation_ticks"] > 0
            for _, _, arm in dropout_arms
        ),
        "every_unavailable_tick_fails_closed": all(
            arm["metrics"]["unavailable_never_executes"]
            and arm["metrics"]["no_current_selection_when_unavailable"]
            and arm["metrics"]["withheld_is_zero_torque"]
            for _, _, arm in dropout_arms
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
    earlier: dict[str, float] = {}
    new_green_falls: list[str] = []
    for case_name, profile_name, arm in dropout_arms:
        exact = rows[case_name]["exact"]["metrics"]
        candidate = arm["metrics"]
        if exact["qualified"] and candidate["fell"]:
            new_green_falls.append(f"{case_name}/{profile_name}")
        if exact["fell"] and candidate["fell"]:
            delta = float(candidate["terminal_time_s"]) - float(
                exact["terminal_time_s"]
            )
            if delta < -1.0e-12:
                earlier[f"{case_name}/{profile_name}"] = delta
    consequence_gates = {
        "no_exact_green_case_becomes_a_fall": not new_green_falls,
        "no_existing_fall_boundary_moves_earlier": not earlier,
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
    consequence_admitted = mechanism_passed and all(consequence_gates.values())
    synchronous_profile_admitted = consequence_admitted and all(
        timing_gates.values()
    )

    table = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact"]["metrics"]
        for profile_name, arm in case_rows.items():
            if profile_name == "exact":
                continue
            candidate = arm["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            table.append(
                [
                    case_name,
                    profile_name,
                    outcome(exact),
                    outcome(candidate),
                    delta,
                    candidate["unavailable_observation_ticks"],
                    candidate["withheld_ticks"],
                    "YES" if arm["terminal_exact_to_exact"] else "NO",
                ]
            )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "consequence_admitted": consequence_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "gates": gates,
        "consequence_gates": consequence_gates,
        "timing_gates": timing_gates,
        "timing": timing,
        "new_green_falls": new_green_falls,
        "earlier_boundaries_s": earlier,
        "profiles": PROFILES,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw observation-dropout phase A/B · r184",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · consequence **{'ADMITTED' if consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**. This audit separates dropout on actuator tick zero from the same periodic loss after authority is established.",
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
                    "unavailable",
                    "withheld",
                    "terminal exact",
                ],
                table,
            ),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Consequence and timing gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in {**consequence_gates, **timing_gates}.items()
                ],
            ),
            "",
            "## Interpretation",
            "",
            "- Startup profiles lose the first observation and repeat at the declared period. Established profiles defer their first loss by one full period, after primary authority has executed continuously.",
            "- Both profiles retain a zero-tick command lease. Unavailable evidence must withhold; this audit classifies phase sensitivity before introducing a replacement command.",
            f"- Earlier existing-fall boundaries: **{len(earlier)}** ({', '.join(earlier) if earlier else 'none'}). New exact-green falls: **{len(new_green_falls)}** ({', '.join(new_green_falls) if new_green_falls else 'none'}).",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-observation-dropout-phase-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_OBSERVATION_DROPOUT_PHASE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "consequence_admitted": consequence_admitted,
                "synchronous_profile_admitted": synchronous_profile_admitted,
                "earlier_boundaries_s": earlier,
                "new_green_falls": new_green_falls,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
