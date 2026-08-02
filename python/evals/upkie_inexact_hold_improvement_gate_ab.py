#!/usr/bin/env python3
"""Sweep a causal forecast-improvement gate over retained-effort authority."""

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
from upkie_inexact_hold_forecast_selector_ab import (
    DROP5,
    DROP5_MATCHED,
    DROP10,
    LEVELS_Q15,
    first_query_prefix_equal,
)


REVISION = "upkie-inexact-hold-improvement-gate-ab-r189"
DURATION_S = 6.0
THRESHOLDS = (0.0, 0.001, 0.005, 0.025, 0.1, 0.25, 1.0, 1.0e9)


def threshold_name(threshold: float) -> str:
    if threshold >= 1.0e8:
        return "withheld"
    return f"m{int(round(threshold * 1000)):04d}"


def profiles() -> dict[str, tuple[int, bool, float, dict[str, int]]]:
    result: dict[str, tuple[int, bool, float, dict[str, int]]] = {
        "exact_control": (0, False, 0.0, {}),
        "exact_selector_m0000": (1, True, 0.0, {}),
        "exact_selector_withheld": (1, True, 1.0e9, {}),
        "drop5_hold0": (0, False, 0.0, DROP5),
        "drop10_hold0": (0, False, 0.0, DROP10),
    }
    for duration, perturbation in (("drop5", DROP5), ("drop10", DROP10)):
        for threshold in THRESHOLDS:
            result[f"{duration}_{threshold_name(threshold)}"] = (
                1,
                True,
                threshold,
                perturbation,
            )
    result["drop5_matched_m0025"] = (1, True, 0.025, DROP5_MATCHED)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_HOLD_IMPROVEMENT_GATE_AB_R189.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def selector_contract(trace: dict[str, Any], threshold: float) -> dict[str, Any]:
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
    raw_argmin = (
        np.argmin(scores[ticks], axis=1)
        if len(ticks)
        else np.empty(0, np.int64)
    )
    raw_improvement = (
        scores[ticks, 0] - scores[ticks, raw_argmin]
        if len(ticks)
        else np.empty(0, np.float64)
    )
    expected_index = np.where(raw_improvement >= threshold, raw_argmin, 0)
    selected_matches_gate = bool(
        np.array_equal(selected_index[ticks], expected_index)
        and np.array_equal(authority_q15[ticks], LEVELS_Q15[expected_index])
    )
    exact_effort = True
    for tick in ticks:
        q15 = int(authority_q15[tick])
        if selection[tick] == 0:
            exact_effort &= np.array_equal(
                torque[tick], np.zeros(6)
            )
        elif selection[tick] == 4 and q15 > 0:
            source = tick - 1
            while source >= 0 and selection[source] not in (1, 2):
                source -= 1
            exact_effort &= (
                source >= 0
                and selection[tick] == 4
                and np.array_equal(
                    torque[tick], (q15 / 32768.0) * torque[source]
                )
            )
        else:
            exact_effort = False
    values = raw_improvement if len(raw_improvement) else np.asarray([0.0])
    return {
        "threshold": threshold,
        "query_ticks": int(len(ticks)),
        "queried_only_when_unavailable": bool(np.all(~available[ticks])),
        "selected_matches_improvement_gate": selected_matches_gate,
        "selected_effort_is_exact": bool(exact_effort),
        "margin_rejected_ticks": int(np.sum(expected_index == 0)),
        "raw_improvement": distribution(values),
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


def execution_trace_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    fields = (
        "time_s",
        "root_position",
        "root_twist",
        "rotation_vector",
        "q",
        "v",
        "torque",
        "physical_contact_active",
        "observed_contact_active",
        "contact_program_authority_selection",
        "contact_program_authority_executable",
    )
    return all(
        np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
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
        selected_cases = tuple(
            case for case in selected_cases if case.name in requested
        )
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
                perturbation,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_forecast_selector=selector,
                inexact_hold_forecast_minimum_improvement=threshold,
            )
            for name, (hold_ticks, selector, threshold, perturbation) in configured.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                perturbation,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_forecast_selector=selector,
                inexact_hold_forecast_minimum_improvement=threshold,
            )
            for name, (hold_ticks, selector, threshold, perturbation) in configured.items()
        }
        rows[case.name] = {}
        for name, run in runs.items():
            threshold = configured[name][2]
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "selector_contract": selector_contract(run["trace"], threshold),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
            }
        for name in ("exact_selector_m0000", "exact_selector_withheld"):
            rows[case.name][name]["dormant_exact"] = semantic_trace_equal(
                runs["exact_control"]["trace"], runs[name]["trace"]
            )
        for duration in ("drop5", "drop10"):
            rows[case.name][f"{duration}_withheld"][
                "execution_exact_to_hold0"
            ] = execution_trace_equal(
                runs[f"{duration}_withheld"]["trace"],
                runs[f"{duration}_hold0"]["trace"],
            )
        rows[case.name]["drop5_matched_m0025"][
            "first_query_prefix_equal"
        ] = first_query_prefix_equal(
            runs["drop5_matched_m0025"]["trace"],
            runs["drop10_m0025"]["trace"],
        )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
                if name.startswith("drop5_m") or name.startswith("drop10_m")
            ),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    selector_names = [
        f"{duration}_{threshold_name(threshold)}"
        for duration in ("drop5", "drop10")
        for threshold in THRESHOLDS
    ] + ["drop5_matched_m0025"]
    selector_arms = [
        case_rows[name] for case_rows in rows.values() for name in selector_names
    ]
    gates = {
        "matrix_complete": len(rows) == len(selected_cases),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_selector_is_dormant": all(
            case_rows[name]["dormant_exact"]
            for case_rows in rows.values()
            for name in ("exact_selector_m0000", "exact_selector_withheld")
        ),
        "selector_queries_every_unavailable_tick": all(
            arm["selector_contract"]["query_ticks"]
            == arm["metrics"]["unavailable_observation_ticks"]
            for arm in selector_arms
        ),
        "selector_is_unavailable_only_gated_and_exact": all(
            arm["selector_contract"]["queried_only_when_unavailable"]
            and arm["selector_contract"]["selected_matches_improvement_gate"]
            and arm["selector_contract"]["selected_effort_is_exact"]
            for arm in selector_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_m0025"]["first_query_prefix_equal"]
            for case_rows in rows.values()
        ),
        "withheld_endpoint_is_execution_exact": all(
            case_rows[f"{duration}_withheld"]["execution_exact_to_hold0"]
            for case_rows in rows.values()
            for duration in ("drop5", "drop10")
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
        f"{duration}_{threshold_name(threshold)}": consequence(
            rows, f"{duration}_{threshold_name(threshold)}"
        )
        for duration in ("drop5", "drop10")
        for threshold in THRESHOLDS
    }
    admitted_thresholds = [
        threshold
        for threshold in THRESHOLDS
        if all(
            consequences[f"{duration}_{threshold_name(threshold)}"]["admitted"]
            for duration in ("drop5", "drop10")
        )
    ]
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "selector_step_ns_maximum": max(
            arm["selector_contract"]["step_ns"]["maximum"]
            for arm in selector_arms
        ),
    }
    mechanism_passed = all(gates.values())
    timing_passed = (
        timing["loop_overruns"] == 0
        and timing["controller_step_ns_maximum"] <= 5_000_000
    )
    # Thresholds are chosen from and evaluated on the same seven-case corpus.
    # Even a consequence-passing row would remain calibration evidence only.
    policy_admitted = False
    synchronous_profile_admitted = False
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)
    selector_queries = sum(
        arm["selector_contract"]["query_ticks"] for arm in selector_arms
    )
    authority_counts = {str(int(level)): 0 for level in LEVELS_Q15}
    for arm in selector_arms:
        for level, count in arm["selector_contract"]["authority_counts"].items():
            authority_counts[level] += count

    summary_rows = []
    for threshold in THRESHOLDS:
        name = threshold_name(threshold)
        drop5 = consequences[f"drop5_{name}"]
        drop10 = consequences[f"drop10_{name}"]
        summary_rows.append(
            [
                "WITHHELD ENDPOINT" if threshold >= 1.0e8 else f"{threshold:.3f}",
                str(len(drop5["new_green_falls"])),
                str(len(drop5["earlier_boundaries_s"])),
                str(len(drop10["new_green_falls"])),
                str(len(drop10["earlier_boundaries_s"])),
                "YES" if threshold in admitted_thresholds else "NO",
            ]
        )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "thresholds": THRESHOLDS,
        "mechanism_passed": mechanism_passed,
        "admitted_thresholds_same_corpus": admitted_thresholds,
        "policy_admitted": policy_admitted,
        "timing_passed": timing_passed,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "first_run_ticks": first_run_ticks,
        "selector_queries": selector_queries,
        "authority_counts": authority_counts,
        "gates": gates,
        "consequences": consequences,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw inexact-hold forecast-improvement gate A/B · r189",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · same-corpus consequence thresholds **{admitted_thresholds or 'NONE'}** · policy **REJECTED** · synchronous profile **REJECTED**.",
            "",
            "## Threshold result",
            "",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks** across **{len(arms)} profiles**, each with an exact replay; selector queries: **{selector_queries:,}**.",
            f"- Q15 selection counts across gated selector arms: **{authority_counts}**.",
            *markdown_table(
                [
                    "minimum improvement",
                    "5 ms new green falls",
                    "5 ms earlier",
                    "10 ms new green falls",
                    "10 ms earlier",
                    "both pass",
                ],
                summary_rows,
            ),
            "",
            f"- Worst selector call: **{timing['selector_step_ns_maximum'] / 1e3:.3f} µs**; Rust allocation and Python GC: **zero**.",
            f"- Timing: **{timing['loop_overruns']}** 5 ms overruns, **{timing['loop_ns_maximum'] / 1e6:.3f} ms** worst loop, and **{timing['controller_step_ns_maximum'] / 1e6:.3f} ms** worst controller call.",
            "- The 1e9 endpoint selects zero authority on every query and is execution-exact to the explicit hold-0 control.",
            "- Threshold points were derived from and evaluated on this same corpus; they are a calibration ablation, never independent promotion evidence.",
            "",
            "## Contract",
            "",
            "- Rust first chooses the exact lower-authority score argmin, then spends it only when predicted improvement over zero authority meets the configured finite nonnegative margin.",
            "- The gate consumes no burst duration, future sample, physics rollout, policy state, hidden time, or heap allocation. The independent one-tick hold budget still withholds later missing samples even if the selector proposes nonzero authority.",
            "- Rejection installs zero retained authority; it does not fall through to a cached lease, contact-force witness, or renewed primary health.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-hold-improvement-gate-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_HOLD_IMPROVEMENT_GATE_AUDIT.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "admitted_thresholds_same_corpus": admitted_thresholds,
                "policy_admitted": policy_admitted,
                "synchronous_profile_admitted": synchronous_profile_admitted,
                "consequences": consequences,
                "timing": timing,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
