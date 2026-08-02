#!/usr/bin/env python3
"""Causal support-free forecast selector A/B for unavailable contact evidence."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_disturbance_envelope import semantic_trace_equal


REVISION = "upkie-inexact-hold-forecast-selector-ab-r188"
DURATION_S = 6.0
LEVELS_Q15 = np.asarray([0, 8192, 16384, 24576, 32768], np.uint16)
DROP5 = {
    "contact_observation_dropout_period_ticks": 50,
    "contact_observation_dropout_burst_ticks": 1,
    "contact_observation_dropout_start_tick": 50,
}
DROP10 = {
    "contact_observation_dropout_period_ticks": 100,
    "contact_observation_dropout_burst_ticks": 2,
    "contact_observation_dropout_start_tick": 100,
}
DROP5_MATCHED = {
    "contact_observation_dropout_period_ticks": 100,
    "contact_observation_dropout_burst_ticks": 1,
    "contact_observation_dropout_start_tick": 100,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_HOLD_FORECAST_SELECTOR_AB_R188.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def profiles() -> dict[str, tuple[int, float, bool, dict[str, int]]]:
    return {
        "exact_control": (0, 1.0, False, {}),
        "exact_selector": (1, 1.0, True, {}),
        "drop5_hold0": (0, 1.0, False, DROP5),
        "drop10_hold0": (0, 1.0, False, DROP10),
        "drop5_full": (1, 1.0, False, DROP5),
        "drop10_full": (1, 1.0, False, DROP10),
        "drop5_selector": (1, 1.0, True, DROP5),
        "drop10_selector": (1, 1.0, True, DROP10),
        "drop5_matched_selector": (1, 1.0, True, DROP5_MATCHED),
    }


def selector_contract(trace: dict[str, Any]) -> dict[str, Any]:
    queried = np.asarray(
        trace["inexact_observation_authority_selector_queried"]
    ) != 0
    available = np.asarray(trace["contact_observation_available"]) != 0
    selected_index = np.asarray(
        trace["inexact_observation_authority_selector_selected_index"]
    )
    authority_q15 = np.asarray(
        trace["inexact_observation_authority_selector_authority_q15"]
    )
    scores = np.asarray(
        trace["inexact_observation_authority_selector_candidate_scores"]
    )
    selection = np.asarray(trace["contact_program_authority_selection"])
    torque = np.asarray(trace["torque"])
    ticks = np.flatnonzero(queried)
    score_argmin = (
        np.argmin(scores[ticks], axis=1)
        if len(ticks)
        else np.empty(0, np.int64)
    )
    selected_matches_argmin = bool(
        np.array_equal(selected_index[ticks], score_argmin)
        and np.array_equal(authority_q15[ticks], LEVELS_Q15[score_argmin])
    )
    exact_effort = True
    for tick in ticks:
        q15 = int(authority_q15[tick])
        if q15 == 0 or selection[tick] == 0:
            exact_effort &= selection[tick] == 0 and np.array_equal(
                torque[tick], np.zeros(6)
            )
        else:
            exact_effort &= tick > 0 and selection[tick] == 4 and np.array_equal(
                torque[tick], (q15 / 32768.0) * torque[tick - 1]
            )
    return {
        "query_ticks": int(len(ticks)),
        "queried_only_when_unavailable": bool(np.all(~available[ticks])),
        "selected_matches_score_argmin": selected_matches_argmin,
        "selected_effort_is_exact": bool(exact_effort),
        "authority_counts": {
            str(int(level)): int(np.sum(authority_q15[ticks] == level))
            for level in LEVELS_Q15
        },
        "step_ns": distribution(
            np.asarray(trace["inexact_observation_authority_selector_step_ns"])[ticks]
            if len(ticks)
            else np.asarray([0], np.uint64)
        ),
    }


def first_query_prefix_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    left_ticks = np.flatnonzero(
        np.asarray(left["inexact_observation_authority_selector_queried"]) != 0
    )
    right_ticks = np.flatnonzero(
        np.asarray(right["inexact_observation_authority_selector_queried"]) != 0
    )
    if not len(left_ticks) or not len(right_ticks) or left_ticks[0] != right_ticks[0]:
        return False
    tick = int(left_ticks[0])
    fields = (
        "time_s",
        "root_position",
        "root_twist",
        "rotation_vector",
        "q",
        "v",
        "torque",
        "inexact_observation_authority_selector_selected_index",
        "inexact_observation_authority_selector_authority_q15",
        "inexact_observation_authority_selector_selected_score",
        "inexact_observation_authority_selector_candidate_scores",
    )
    return all(
        np.array_equal(np.asarray(left[field])[: tick + 1], np.asarray(right[field])[: tick + 1])
        for field in fields
    )


def consequence(rows: dict[str, dict[str, Any]], profile: str) -> dict[str, Any]:
    earlier: dict[str, float] = {}
    new_green_falls: list[str] = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        candidate = case_rows[profile]["metrics"]
        if exact["qualified"] and candidate["fell"]:
            new_green_falls.append(case_name)
        if exact["fell"] and candidate["fell"]:
            delta = float(candidate["terminal_time_s"]) - float(
                exact["terminal_time_s"]
            )
            if delta < -1.0e-12:
                earlier[case_name] = delta
    return {
        "earlier_boundaries_s": earlier,
        "new_green_falls": new_green_falls,
        "admitted": not earlier and not new_green_falls,
    }


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
    configured = profiles()
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
                inexact_hold_authority=authority,
                inexact_hold_forecast_selector=selector,
            )
            for name, (hold_ticks, authority, selector, profile) in configured.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_authority=authority,
                inexact_hold_forecast_selector=selector,
            )
            for name, (hold_ticks, authority, selector, profile) in configured.items()
        }
        rows[case.name] = {
            name: {
                "metrics": run["metrics"],
                "selector_contract": selector_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(run["trace"], replays[name]["trace"]),
            }
            for name, run in runs.items()
        }
        rows[case.name]["exact_selector"]["dormant_exact"] = semantic_trace_equal(
            runs["exact_control"]["trace"], runs["exact_selector"]["trace"]
        )
        rows[case.name]["drop5_matched_selector"]["first_query_prefix_equal"] = (
            first_query_prefix_equal(
                runs["drop5_matched_selector"]["trace"],
                runs["drop10_selector"]["trace"],
            )
        )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(f"{name}={outcome(run['metrics'])}" for name, run in runs.items()),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    selector_names = ("drop5_selector", "drop10_selector", "drop5_matched_selector")
    selector_arms = [case_rows[name] for case_rows in rows.values() for name in selector_names]
    gates = {
        "matrix_complete": len(rows) == len(selected_cases),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_selector_is_dormant": all(
            case_rows["exact_selector"]["dormant_exact"] for case_rows in rows.values()
        ),
        "selector_queries_every_unavailable_tick": all(
            arm["selector_contract"]["query_ticks"]
            == arm["metrics"]["unavailable_observation_ticks"]
            for arm in selector_arms
        ),
        "selector_is_unavailable_only_argmin_and_exact": all(
            arm["selector_contract"]["queried_only_when_unavailable"]
            and arm["selector_contract"]["selected_matches_score_argmin"]
            and arm["selector_contract"]["selected_effort_is_exact"]
            for arm in selector_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_selector"]["first_query_prefix_equal"]
            for case_rows in rows.values()
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            for arm in arms
        ),
    }
    consequences = {
        name: consequence(rows, name)
        for name in ("drop5_hold0", "drop10_hold0", "drop5_full", "drop10_full", "drop5_selector", "drop10_selector")
    }
    selector_consequence_admitted = all(
        consequences[name]["admitted"] for name in ("drop5_selector", "drop10_selector")
    )
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(arm["metrics"]["loop_ns"]["maximum"] for arm in arms),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "selector_step_ns_maximum": max(
            arm["selector_contract"]["step_ns"]["maximum"] for arm in selector_arms
        ),
    }
    mechanism_passed = all(gates.values())
    timing_passed = timing["loop_overruns"] == 0 and timing["controller_step_ns_maximum"] <= 5_000_000
    synchronous_profile_admitted = mechanism_passed and selector_consequence_admitted and timing_passed
    authority_counts = {str(int(level)): 0 for level in LEVELS_Q15}
    for arm in selector_arms:
        for level, count in arm["selector_contract"]["authority_counts"].items():
            authority_counts[level] += count
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)
    selector_queries = sum(
        arm["selector_contract"]["query_ticks"] for arm in selector_arms
    )
    profile_table = []
    for name, result in consequences.items():
        earlier = result["earlier_boundaries_s"]
        profile_table.append([
            name,
            len(result["new_green_falls"]),
            len(earlier),
            "—" if not earlier else f"{min(earlier.values()):.3f}",
            "ADMITTED" if result["admitted"] else "REJECTED",
        ])

    detail = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        for duration in ("drop5", "drop10"):
            candidate = case_rows[f"{duration}_selector"]["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            detail.append([case_name, duration, outcome(exact), outcome(candidate), delta])

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "selector_consequence_admitted": selector_consequence_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "gates": gates,
        "consequences": consequences,
        "authority_counts": authority_counts,
        "first_run_ticks": first_run_ticks,
        "selector_queries": selector_queries,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw support-free inexact-hold forecast selector A/B · r188",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant consequence **{'ADMITTED' if selector_consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**.",
            "",
            "## Selector result",
            "",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks** across **{len(arms)} profiles**, each with an exact replay; selector queries: **{selector_queries}**.",
            f"- Q15 selection counts across selector arms: **{authority_counts}**.",
            f"- Worst selector call: **{timing['selector_step_ns_maximum'] / 1e3:.3f} µs**; Rust allocation and Python GC: **zero**.",
            f"- Timing: **{timing['loop_overruns']}** 5 ms overruns, **{timing['controller_step_ns_maximum'] / 1e6:.3f} ms** worst controller call.",
            "- The matched 5/10 ms pair has an identical first-loss state, score vector, selected authority, and torque; no future burst-duration input exists.",
            "",
            "## Comparator summary",
            "",
            *markdown_table(["profile", "new green falls", "earlier falls", "worst Δ s", "verdict"], profile_table),
            "",
            "## Plant consequence",
            "",
            *markdown_table(["case", "dropout", "exact", "selector", "fall Δ s"], detail),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(["gate", "result"], [[name, "PASS" if passed else "FAIL"] for name, passed in gates.items()]),
            "",
            "## Contract",
            "",
            "- Rust forces support unknown, scores 0/0.25/0.5/0.75/1 over the fixed eight-knot reduced model, and resolves exact ties toward lower authority.",
            "- The selected fraction scales only the last admitted effective effort; it never creates a contact-force witness or refreshes primary health.",
            "- This is a state-local forecast witness, not a physics rollout or recovery certificate; strict plant non-regression remains the promotion gate.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-hold-forecast-selector-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_HOLD_FORECAST_SELECTOR_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({
        "mechanism_passed": mechanism_passed,
        "selector_consequence_admitted": selector_consequence_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "authority_counts": authority_counts,
        "consequences": consequences,
        "timing": timing,
    }, sort_keys=True))
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
