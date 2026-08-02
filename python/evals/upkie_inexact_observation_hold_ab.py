#!/usr/bin/env python3
"""Evaluate a Rust-owned bounded effective-effort hold under observation loss."""

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
from upkie_current_support_realization_plant_ab import exact_terminal_outcome
from upkie_disturbance_envelope import semantic_trace_equal


REVISION = "upkie-inexact-observation-hold-ab-r185"
DURATION_S = 6.0
ESTABLISHED_5MS = {
    "contact_observation_dropout_period_ticks": 50,
    "contact_observation_dropout_burst_ticks": 1,
    "contact_observation_dropout_start_tick": 50,
}
ESTABLISHED_10MS = {
    "contact_observation_dropout_period_ticks": 100,
    "contact_observation_dropout_burst_ticks": 2,
    "contact_observation_dropout_start_tick": 100,
}
PROFILES: dict[str, tuple[int, dict[str, int]]] = {
    "exact_hold0": (0, {}),
    "exact_hold1": (1, {}),
    "exact_hold2": (2, {}),
    "drop5_hold0": (0, ESTABLISHED_5MS),
    "drop5_hold1": (1, ESTABLISHED_5MS),
    "drop10_hold0": (0, ESTABLISHED_10MS),
    "drop10_hold1": (1, ESTABLISHED_10MS),
    "drop10_hold2": (2, ESTABLISHED_10MS),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_OBSERVATION_HOLD_AB_R185.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def hold_contract(trace: dict[str, Any]) -> dict[str, Any]:
    selection = np.asarray(trace["contact_program_authority_selection"])
    available = np.asarray(trace["contact_observation_available"]) != 0
    torque = np.asarray(trace["torque"])
    held = np.flatnonzero(selection == 4)
    held_has_predecessor = bool(np.all(held > 0))
    held_matches_predecessor = bool(
        held_has_predecessor
        and all(np.array_equal(torque[tick], torque[tick - 1]) for tick in held)
    )
    return {
        "typed_hold_ticks": int(len(held)),
        "typed_hold_only_when_unavailable": bool(np.all(~available[held])),
        "typed_hold_matches_preceding_effort": held_matches_predecessor,
    }


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
            name: execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=hold_ticks,
            )
            for name, (hold_ticks, profile) in PROFILES.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=hold_ticks,
            )
            for name, (hold_ticks, profile) in PROFILES.items()
        }
        baseline = runs["exact_hold0"]
        rows[case.name] = {}
        for name, run in runs.items():
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "hold_contract": hold_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
                "terminal_exact_to_baseline": exact_terminal_outcome(
                    baseline["metrics"], run["metrics"]
                ),
                "semantic_exact_to_baseline": semantic_trace_equal(
                    baseline["trace"], run["trace"]
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
    exact_holds = [
        case_rows[name]
        for case_rows in rows.values()
        for name in ("exact_hold1", "exact_hold2")
    ]
    hold_arms = [
        case_rows[name]
        for case_rows in rows.values()
        for name in ("drop5_hold1", "drop10_hold1", "drop10_hold2")
    ]
    expected_counts = {
        "drop5_hold1": lambda unavailable: unavailable,
        "drop10_hold1": lambda unavailable: (unavailable + 1) // 2,
        "drop10_hold2": lambda unavailable: unavailable,
    }
    gates = {
        "matrix_complete": len(rows) == len(cases()),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in all_arms),
        "exact_stream_is_unchanged_by_dormant_hold_config": all(
            arm["semantic_exact_to_baseline"] for arm in exact_holds
        ),
        "typed_hold_is_only_unavailable_and_replays_preceding_effort": all(
            arm["hold_contract"]["typed_hold_only_when_unavailable"]
            and arm["hold_contract"]["typed_hold_matches_preceding_effort"]
            for arm in hold_arms
        ),
        "typed_hold_count_matches_burst_budget": all(
            case_rows[name]["hold_contract"]["typed_hold_ticks"]
            == expected_counts[name](
                case_rows[name]["metrics"]["unavailable_observation_ticks"]
            )
            for case_rows in rows.values()
            for name in expected_counts
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
    candidate_names = ("drop5_hold1", "drop10_hold1", "drop10_hold2")
    earlier: dict[str, float] = {}
    new_green_falls: list[str] = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_hold0"]["metrics"]
        for name in candidate_names:
            candidate = case_rows[name]["metrics"]
            if exact["qualified"] and candidate["fell"]:
                new_green_falls.append(f"{case_name}/{name}")
            if exact["fell"] and candidate["fell"]:
                delta = float(candidate["terminal_time_s"]) - float(
                    exact["terminal_time_s"]
                )
                if delta < -1.0e-12:
                    earlier[f"{case_name}/{name}"] = delta
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
        "all_controller_calls_below_5ms": timing["controller_step_ns_maximum"] <= 5_000_000,
    }
    mechanism_passed = all(gates.values())
    consequence_admitted = mechanism_passed and all(consequence_gates.values())
    synchronous_profile_admitted = consequence_admitted and all(timing_gates.values())

    table = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_hold0"]["metrics"]
        for name in ("drop5_hold0", *candidate_names, "drop10_hold0"):
            arm = case_rows[name]
            candidate = arm["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            table.append(
                [
                    case_name,
                    name,
                    outcome(exact),
                    outcome(candidate),
                    delta,
                    candidate["unavailable_observation_ticks"],
                    candidate["withheld_ticks"],
                    arm["hold_contract"]["typed_hold_ticks"],
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
        "profiles": {name: {"hold_ticks": hold, **profile} for name, (hold, profile) in PROFILES.items()},
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw inexact-observation hold A/B · r185",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · consequence **{'ADMITTED' if consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**. Rust may replay only the last already-admitted effective effort for a separately configured number of non-exact observation ticks.",
            "",
            "## Result",
            "",
            *markdown_table(
                ["case", "profile", "exact", "candidate", "fall Δ s", "unavailable", "withheld", "typed holds"],
                table,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in {**gates, **consequence_gates, **timing_gates}.items()],
            ),
            "",
            "## Contract",
            "",
            "- The hold is a distinct Rust authority selection and provenance. It cannot execute at startup, on an exact contact transition, after its tick budget, or without a prior admitted command.",
            "- Held ticks replay the preceding effective actuator effort exactly, emit no contact-force witness, and do not refresh primary-command health.",
            "- Hold configuration is dormant under exact observation; exact hold-0/1/2 semantic traces must be identical.",
            f"- Earlier candidate boundaries: **{len(earlier)}** ({', '.join(earlier) if earlier else 'none'}). New green falls: **{len(new_green_falls)}** ({', '.join(new_green_falls) if new_green_falls else 'none'}).",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-observation-hold-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_OBSERVATION_HOLD_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({"mechanism_passed": mechanism_passed, "consequence_admitted": consequence_admitted, "synchronous_profile_admitted": synchronous_profile_admitted, "earlier_boundaries_s": earlier, "new_green_falls": new_green_falls}, sort_keys=True))
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
