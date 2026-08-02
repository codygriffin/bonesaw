#!/usr/bin/env python3
"""Delay, dropout, bit-chatter, and mirrored plant stress for contact authority."""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_disturbance_envelope import (
    CaseSpec,
    case_matrix,
    run_case,
    semantic_trace_equal,
    summarize,
)


REVISION = "upkie-contact-program-robustness-ab-r182"
DURATION_S = 6.0
CASE_NAMES = (
    "nominal",
    "forward_4n_reference",
    "backward_4n",
    "left_1n",
    "right_1n_mirror",
    "handle_forward_4n",
    "forward_4n_friction_0p03",
)
PROFILES: dict[str, dict[str, int]] = {
    "exact": {},
    "delay_5ms": {"contact_observation_delay_ticks": 1},
    "delay_20ms": {"contact_observation_delay_ticks": 4},
    "dropout_5ms_per_250ms": {
        "contact_observation_dropout_period_ticks": 50,
        "contact_observation_dropout_burst_ticks": 1,
    },
    "dropout_10ms_per_500ms": {
        "contact_observation_dropout_period_ticks": 100,
        "contact_observation_dropout_burst_ticks": 2,
    },
    "left_bit_chatter_5ms_per_250ms": {
        "contact_observation_flip_period_ticks": 50,
        "contact_observation_flip_burst_ticks": 1,
        "contact_observation_flip_contact": 0,
    },
    "right_bit_chatter_5ms_per_250ms": {
        "contact_observation_flip_period_ticks": 50,
        "contact_observation_flip_burst_ticks": 1,
        "contact_observation_flip_contact": 1,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_PROGRAM_ROBUSTNESS_AB_R182.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--profiles", help="comma-separated profile subset")
    return parser.parse_args()


def cases() -> tuple[CaseSpec, ...]:
    source = {case.name: case for case in case_matrix()}
    source["right_1n_mirror"] = dataclasses.replace(
        source["left_1n"],
        name="right_1n_mirror",
        force_world_n=(0.0, -1.0, 0.0),
    )
    return tuple(source[name] for name in CASE_NAMES)


def execute(
    model: pathlib.Path,
    case: CaseSpec,
    duration_s: float,
    profile: dict[str, int],
    *,
    authority_ticks: int = 0,
    inexact_hold_ticks: int = 0,
    inexact_hold_authority: float = 1.0,
    inexact_hold_forecast_selector: bool = False,
    inexact_hold_forecast_minimum_improvement: float = 0.0,
    inexact_support_free_brake: bool = False,
    inexact_terminal_chooser: bool = False,
    inexact_terminal_zero_effort_baseline: bool = False,
    inexact_terminal_support_hypothesis_envelope: bool = False,
    inexact_terminal_maximum_component_regression: float = 0.0,
    inexact_terminal_minimum_component_improvement: float = 0.01,
    record_physical_contact_impulses: bool = False,
    record_physical_contact_prestate: bool = False,
    record_physical_prospective_contact_state: bool = False,
) -> dict[str, Any]:
    trace = run_case(
        model,
        case,
        duration_s,
        balance_mode="capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=True,
        execute_reduced_support=True,
        support_contingency_enabled=True,
        support_contingency_execute=True,
        support_contingency_realize_primary_torque=True,
        contact_program_authority_ticks=authority_ticks,
        contact_program_inexact_hold_ticks=inexact_hold_ticks,
        contact_program_inexact_hold_authority=inexact_hold_authority,
        contact_program_inexact_hold_forecast_selector=(
            inexact_hold_forecast_selector
        ),
        contact_program_inexact_hold_forecast_minimum_improvement=(
            inexact_hold_forecast_minimum_improvement
        ),
        contact_program_inexact_support_free_brake=inexact_support_free_brake,
        contact_program_inexact_terminal_chooser=inexact_terminal_chooser,
        contact_program_inexact_terminal_zero_effort_baseline=(
            inexact_terminal_zero_effort_baseline
        ),
        contact_program_inexact_terminal_support_hypothesis_envelope=(
            inexact_terminal_support_hypothesis_envelope
        ),
        contact_program_inexact_terminal_maximum_component_regression=(
            inexact_terminal_maximum_component_regression
        ),
        contact_program_inexact_terminal_minimum_component_improvement=(
            inexact_terminal_minimum_component_improvement
        ),
        record_physical_contact_impulses=record_physical_contact_impulses,
        record_physical_contact_prestate=record_physical_contact_prestate,
        record_physical_prospective_contact_state=(
            record_physical_prospective_contact_state
        ),
        contact_observation_prestart_samples=3,
        use_feasibility_row_spans=True,
        **profile,
    )
    metrics = summarize(case, trace, duration_s)
    selection = np.asarray(trace["contact_program_authority_selection"])
    executable = np.asarray(trace["contact_program_authority_executable"]) != 0
    available = np.asarray(trace["contact_observation_available"]) != 0
    torque = np.asarray(trace["torque"])
    physical = np.asarray(trace["physical_contact_active"])
    observed = np.asarray(trace["observed_contact_active"])
    selector_queried = np.asarray(
        trace["inexact_observation_authority_selector_queried"]
    ) != 0
    selector_authority = np.asarray(
        trace["inexact_observation_authority_selector_authority_q15"]
    )
    metrics.update(
        {
            "withheld_ticks": int(np.sum(selection == 0)),
            "primary_ticks": int(np.sum(selection == 1)),
            "current_support_ticks": int(np.sum(selection == 2)),
            "lease_ticks": int(np.sum(selection == 3)),
            "inexact_hold_ticks": int(np.sum(selection == 4)),
            "inexact_support_free_brake_ticks": int(np.sum(selection == 5)),
            "inexact_terminal_selector_queries": int(
                np.sum(trace["inexact_observation_terminal_selector_queried"])
            ),
            "inexact_terminal_selector_action_counts": {
                str(int(action)): int(
                    np.sum(
                        np.asarray(
                            trace["inexact_observation_terminal_selector_queried"]
                        )
                        & (
                            np.asarray(
                                trace["inexact_observation_terminal_selector_action"]
                            )
                            == action
                        )
                    )
                )
                for action in np.unique(
                    np.asarray(trace["inexact_observation_terminal_selector_action"])[
                        np.asarray(
                            trace["inexact_observation_terminal_selector_queried"]
                        )
                        != 0
                    ]
                )
            },
            "unavailable_observation_ticks": int(np.sum(~available)),
            "observation_mismatch_ticks": int(
                np.sum(np.any(physical != observed, axis=1))
            ),
            "maximum_observation_age_ns": int(
                np.max(trace["contact_observation_age_ns"])
            ),
            "maximum_observation_status": int(
                np.max(trace["contact_observation_status"])
            ),
            "unavailable_never_executes": bool(np.all(available | ~executable)),
            "withheld_is_zero_torque": bool(np.all(torque[~executable] == 0.0)),
            "no_current_selection_when_unavailable": bool(
                np.all(available | (selection != 2))
            ),
            "inexact_selector_queries": int(np.sum(selector_queried)),
            "inexact_selector_authority_counts": {
                str(int(authority)): int(
                    np.sum(selector_queried & (selector_authority == authority))
                )
                for authority in np.unique(selector_authority[selector_queried])
            },
            "inexact_selector_step_ns": distribution(
                np.asarray(trace["inexact_observation_authority_selector_step_ns"])[
                    selector_queried
                ]
            ) if np.any(selector_queried) else distribution(np.asarray([0], np.uint64)),
        }
    )
    return {"trace": trace, "metrics": metrics}


def outcome(metrics: dict[str, Any]) -> str:
    if metrics["fell"]:
        return f"FALL {float(metrics['terminal_time_s']):.3f}s"
    return str(metrics["outcome"])


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        names = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in names)
        missing = names - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")
    profiles = PROFILES
    if args.profiles:
        names = set(args.profiles.split(","))
        missing = names - set(PROFILES)
        if missing:
            raise SystemExit(f"unknown profiles: {sorted(missing)}")
        profiles = {name: PROFILES[name] for name in PROFILES if name in names}

    model = pathlib.Path(args.model).resolve()
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        rows[case.name] = {}
        for profile_index, (profile_name, profile) in enumerate(profiles.items(), 1):
            first = execute(model, case, args.duration, profile)
            replay = execute(model, case, args.duration, profile)
            rows[case.name][profile_name] = {
                "metrics": first["metrics"],
                "replay_exact": semantic_trace_equal(first["trace"], replay["trace"]),
            }
            print(
                f"[{case_index:02d}/{len(selected_cases)}:{profile_index}/{len(profiles)}] "
                f"{case.name}/{profile_name}: {outcome(first['metrics'])}, "
                f"withheld={first['metrics']['withheld_ticks']}",
                flush=True,
            )

    full_matrix = len(selected_cases) == len(CASE_NAMES) and len(profiles) == len(PROFILES)
    exact = {name: row["exact"]["metrics"] for name, row in rows.items()}
    exact_green = [name for name, metrics in exact.items() if metrics["qualified"]]
    exact_falls = [name for name, metrics in exact.items() if metrics["fell"]]
    perturbed = [
        (case_name, profile_name, value["metrics"])
        for case_name, case_rows in rows.items()
        for profile_name, value in case_rows.items()
        if profile_name != "exact"
    ]
    new_green_falls = [
        f"{case_name}/{profile_name}"
        for case_name, profile_name, metrics in perturbed
        if case_name in exact_green and metrics["fell"]
    ]
    earlier_boundaries = {
        f"{case_name}/{profile_name}": float(metrics["terminal_time_s"])
        - float(exact[case_name]["terminal_time_s"])
        for case_name, profile_name, metrics in perturbed
        if case_name in exact_falls
        and metrics["fell"]
        and float(metrics["terminal_time_s"])
        < float(exact[case_name]["terminal_time_s"])
    }
    all_results = [value for case_rows in rows.values() for value in case_rows.values()]
    dropout_results = [
        value["metrics"]
        for case_rows in rows.values()
        for name, value in case_rows.items()
        if name.startswith("dropout_")
    ]
    delay_20 = [case_rows["delay_20ms"]["metrics"] for case_rows in rows.values()]
    gates = {
        "matrix_complete": full_matrix,
        "exact_replay": all(value["replay_exact"] for value in all_results),
        "dropout_exercised": sum(
            value["unavailable_observation_ticks"] for value in dropout_results
        )
        > 0,
        "dropout_is_fail_closed": all(
            value["unavailable_never_executes"]
            and value["no_current_selection_when_unavailable"]
            and value["withheld_is_zero_torque"]
            for value in dropout_results
        ),
        "inclusive_20ms_age_is_exercised": all(
            value["maximum_observation_age_ns"] == 20_000_000
            for value in delay_20
        ),
        "bit_chatter_and_both_sides_exercised": all(
            rows[case_name][profile]["metrics"]["observation_mismatch_ticks"] > 0
            for case_name in rows
            for profile in (
                "left_bit_chatter_5ms_per_250ms",
                "right_bit_chatter_5ms_per_250ms",
            )
        ),
        "finite_without_numeric_fault": all(
            value["metrics"]["finite"] and not value["metrics"]["numeric_fault"]
            for value in all_results
        ),
        "zero_rust_allocation_and_python_gc": all(
            value["metrics"]["allocation_free"]
            and value["metrics"]["python_gc_collections"] == 0
            for value in all_results
        ),
    }
    robustness_gates = {
        "no_exact_green_case_becomes_a_fall": not new_green_falls,
        "no_exact_fall_boundary_moves_earlier": not earlier_boundaries,
        "zero_5ms_loop_overruns": all(
            value["metrics"]["loop_overruns"] == 0 for value in all_results
        ),
    }
    mechanism_passed = all(gates.values())
    robustness_admitted = mechanism_passed and all(robustness_gates.values())

    table = []
    for case_name, case_rows in rows.items():
        control = case_rows["exact"]["metrics"]
        for profile_name, value in case_rows.items():
            if profile_name == "exact":
                continue
            metrics = value["metrics"]
            delta = (
                "—"
                if not control["fell"] or not metrics["fell"]
                else f"{float(metrics['terminal_time_s']) - float(control['terminal_time_s']):+.3f}"
            )
            table.append(
                [
                    case_name,
                    profile_name,
                    outcome(control),
                    outcome(metrics),
                    delta,
                    metrics["unavailable_observation_ticks"],
                    metrics["observation_mismatch_ticks"],
                    metrics["withheld_ticks"],
                    metrics["loop_overruns"],
                ]
            )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "robustness_admitted": robustness_admitted,
        "gates": gates,
        "robustness_gates": robustness_gates,
        "exact_green_cases": exact_green,
        "exact_fall_cases": exact_falls,
        "new_green_falls": new_green_falls,
        "earlier_boundaries_s": earlier_boundaries,
        "profiles": profiles,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw contact-program observation robustness A/B · r182",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · robustness profile **{'ADMITTED' if robustness_admitted else 'REJECTED'}**. Python owns deterministic sensor perturbation and MuJoCo; Rust owns timestamp/provenance admission, debounce, current-support WBC proof, and program authority.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "case",
                    "profile",
                    "exact",
                    "perturbed",
                    "fall Δ s",
                    "unavailable",
                    "mask mismatch",
                    "withheld",
                    ">5 ms",
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
            "## Robustness gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in robustness_gates.items()
                ],
            ),
            "",
            "## Contract",
            "",
            "- Delay profiles retain the sample mask and expose its acquisition age to Rust. The 20 ms row exercises the configured inclusive age boundary; it does not pretend delayed contact is current truth.",
            "- Dropout profiles mark the observation unavailable. With a zero-tick command lease, unavailable evidence must produce withheld authority and exactly zero torque on that tick.",
            "- Bit-chatter profiles are accepted sensor claims with deliberately wrong left or right bits. Their consequence measures sensitivity; software cannot infer that an authenticated but incorrect sensor bit is physically false.",
            "- The mirrored ±1 N rows and left/right bit profiles expose both contact directions without averaging them into a symmetry claim.",
            f"- New falls from exact-green cases: **{len(new_green_falls)}** ({', '.join(new_green_falls) if new_green_falls else 'none'}). Earlier exact-fall boundaries: **{len(earlier_boundaries)}** ({', '.join(earlier_boundaries) if earlier_boundaries else 'none'}).",
        ]
    ) + "\n"
    (destination / "upkie-contact-program-robustness-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_PROGRAM_ROBUSTNESS_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "robustness_admitted": robustness_admitted,
                "gates": gates,
                "robustness_gates": robustness_gates,
                "new_green_falls": new_green_falls,
                "earlier_boundaries_s": earlier_boundaries,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
