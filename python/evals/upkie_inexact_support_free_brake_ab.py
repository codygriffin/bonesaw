#!/usr/bin/env python3
"""Causal A/B for a freshly solved support-free observation-loss brake."""

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
)
from upkie_inexact_hold_improvement_gate_ab import consequence


REVISION = "upkie-inexact-support-free-brake-ab-r190"
DURATION_S = 6.0


def profiles() -> dict[str, tuple[int, bool, float, bool, dict[str, int]]]:
    return {
        "exact_control": (0, False, 0.0, False, {}),
        "exact_brake_config": (0, False, 0.0, True, {}),
        "drop5_hold0": (0, False, 0.0, False, DROP5),
        "drop10_hold0": (0, False, 0.0, False, DROP10),
        "drop5_argmin": (1, True, 0.0, False, DROP5),
        "drop10_argmin": (1, True, 0.0, False, DROP10),
        "drop5_margin001": (1, True, 0.001, False, DROP5),
        "drop10_margin001": (1, True, 0.001, False, DROP10),
        "drop5_brake": (0, False, 0.0, True, DROP5),
        "drop10_brake": (0, False, 0.0, True, DROP10),
        "drop5_matched_brake": (0, False, 0.0, True, DROP5_MATCHED),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_SUPPORT_FREE_BRAKE_AB_R190.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def brake_contract(trace: dict[str, Any]) -> dict[str, Any]:
    available = np.asarray(trace["contact_observation_available"]) != 0
    selection = np.asarray(trace["contact_program_authority_selection"])
    executable = np.asarray(trace["contact_program_authority_executable"]) != 0
    torque = np.asarray(trace["torque"])
    candidate_torque = np.asarray(trace["support_contingency_candidate_torque"])
    status = np.asarray(trace["support_contingency_status"])
    violation = np.asarray(
        trace["support_contingency_maximum_constraint_violation"]
    )
    mode = np.asarray(trace["support_contingency_mode"])
    support_mask = np.asarray(trace["support_contingency_support_mask"])
    contact_force = np.asarray(trace["wbc_normal_force"])
    age = np.asarray(trace["contact_program_authority_age_ticks"])
    lease_status = np.asarray(trace["contact_program_authority_lease_status"])
    lease_provenance = np.asarray(
        trace["contact_program_authority_lease_provenance"]
    )
    ticks = np.flatnonzero(~available)
    return {
        "unavailable_ticks": int(len(ticks)),
        "selected_on_every_unavailable_tick": bool(
            np.all(selection[ticks] == 5) and np.all(executable[ticks])
        ),
        "torque_equals_independent_wbc": bool(
            np.array_equal(torque[ticks], candidate_torque[ticks])
        ),
        "wbc_admitted": bool(
            np.all(status[ticks] <= 1) and np.all(violation[ticks] < 1.0e-8)
        ),
        "support_is_explicitly_zero_and_ballistic": bool(
            np.all(mode[ticks] == 2) and np.all(support_mask[ticks] == 0)
        ),
        "emits_no_contact_force_witness": bool(
            np.array_equal(contact_force[ticks], np.zeros_like(contact_force[ticks]))
        ),
        "typed_without_primary_refresh": bool(
            np.all(lease_status[ticks] == 10)
            and np.all(lease_provenance[ticks] == 4)
            and (len(ticks) == 0 or np.all(age[ticks] >= 1))
        ),
        "author_step_ns": distribution(
            np.asarray(trace["support_contingency_author_step_ns"])[ticks]
            if len(ticks)
            else np.asarray([0], np.uint64)
        ),
        "wbc_step_ns": distribution(
            np.asarray(trace["support_contingency_wbc_step_ns"])[ticks]
            if len(ticks)
            else np.asarray([0], np.uint64)
        ),
    }


def first_brake_prefix_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    left_ticks = np.flatnonzero(
        np.asarray(left["contact_program_authority_selection"]) == 5
    )
    right_ticks = np.flatnonzero(
        np.asarray(right["contact_program_authority_selection"]) == 5
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
        "support_contingency_candidate_torque",
        "support_contingency_candidate_generalized_acceleration",
        "contact_program_authority_selection",
        "contact_program_authority_lease_status",
        "contact_program_authority_lease_provenance",
    )
    return all(
        np.array_equal(np.asarray(left[field])[: tick + 1], np.asarray(right[field])[: tick + 1])
        for field in fields
    )


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
                inexact_hold_forecast_minimum_improvement=margin,
                inexact_support_free_brake=brake,
            )
            for name, (hold_ticks, selector, margin, brake, perturbation) in configured.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                perturbation,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_forecast_selector=selector,
                inexact_hold_forecast_minimum_improvement=margin,
                inexact_support_free_brake=brake,
            )
            for name, (hold_ticks, selector, margin, brake, perturbation) in configured.items()
        }
        rows[case.name] = {
            name: {
                "metrics": run["metrics"],
                "brake_contract": brake_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
            }
            for name, run in runs.items()
        }
        rows[case.name]["exact_brake_config"]["dormant_exact"] = (
            semantic_trace_equal(
                runs["exact_control"]["trace"],
                runs["exact_brake_config"]["trace"],
            )
        )
        rows[case.name]["drop5_matched_brake"]["first_query_prefix_equal"] = (
            first_brake_prefix_equal(
                runs["drop5_matched_brake"]["trace"],
                runs["drop10_brake"]["trace"],
            )
        )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
                if name.startswith("drop")
            ),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    brake_names = ("drop5_brake", "drop10_brake", "drop5_matched_brake")
    brake_arms = [
        case_rows[name] for case_rows in rows.values() for name in brake_names
    ]
    gates = {
        "matrix_complete": len(rows) == len(selected_cases),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_brake_configuration_is_dormant": all(
            case_rows["exact_brake_config"]["dormant_exact"]
            for case_rows in rows.values()
        ),
        "brake_is_unavailable_only_typed_and_executable": all(
            arm["brake_contract"]["selected_on_every_unavailable_tick"]
            for arm in brake_arms
        ),
        "brake_equals_independently_admitted_wbc": all(
            arm["brake_contract"]["torque_equals_independent_wbc"]
            and arm["brake_contract"]["wbc_admitted"]
            for arm in brake_arms
        ),
        "brake_is_support_free_without_force_witness": all(
            arm["brake_contract"]["support_is_explicitly_zero_and_ballistic"]
            and arm["brake_contract"]["emits_no_contact_force_witness"]
            for arm in brake_arms
        ),
        "brake_does_not_refresh_primary": all(
            arm["brake_contract"]["typed_without_primary_refresh"]
            for arm in brake_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_brake"]["first_query_prefix_equal"]
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
        for name in (
            "drop5_hold0",
            "drop10_hold0",
            "drop5_argmin",
            "drop10_argmin",
            "drop5_margin001",
            "drop10_margin001",
            "drop5_brake",
            "drop10_brake",
        )
    }
    brake_consequence_admitted = all(
        consequences[name]["admitted"]
        for name in ("drop5_brake", "drop10_brake")
    )
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "brake_author_step_ns_maximum": max(
            arm["brake_contract"]["author_step_ns"]["maximum"]
            for arm in brake_arms
        ),
        "brake_wbc_step_ns_maximum": max(
            arm["brake_contract"]["wbc_step_ns"]["maximum"]
            for arm in brake_arms
        ),
    }
    mechanism_passed = all(gates.values())
    timing_passed = (
        timing["loop_overruns"] == 0
        and timing["controller_step_ns_maximum"] <= 5_000_000
    )
    synchronous_profile_admitted = (
        mechanism_passed and brake_consequence_admitted and timing_passed
    )
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)
    brake_ticks = sum(
        arm["metrics"]["inexact_support_free_brake_ticks"] for arm in arms
    )

    detail = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        for duration in ("drop5", "drop10"):
            candidate = case_rows[f"{duration}_brake"]["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            detail.append(
                [case_name, duration, outcome(exact), outcome(candidate), delta]
            )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "brake_consequence_admitted": brake_consequence_admitted,
        "timing_passed": timing_passed,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "first_run_ticks": first_run_ticks,
        "brake_ticks": brake_ticks,
        "gates": gates,
        "consequences": consequences,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw support-free observation-loss brake A/B · r190",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant consequence **{'ADMITTED' if brake_consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**.",
            "",
            "## Plant consequence",
            "",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks** across **{len(arms)} profiles**, each with an exact replay; fresh brake selections: **{brake_ticks}**.",
            *markdown_table(
                ["case", "dropout", "exact", "support-free brake", "fall Δ s"],
                detail,
            ),
            "",
            "## Runtime",
            "",
            f"- Rust author / independent WBC maximum: **{timing['brake_author_step_ns_maximum'] / 1e3:.3f} / {timing['brake_wbc_step_ns_maximum'] / 1e3:.3f} µs**.",
            f"- Full loop: **{timing['loop_overruns']}** 5 ms overruns; loop/controller maxima **{timing['loop_ns_maximum'] / 1e6:.3f} / {timing['controller_step_ns_maximum'] / 1e6:.3f} ms**.",
            "- Rust allocation and Python GC inside the measured authority path: **zero**.",
            "",
            "## Contract",
            "",
            "- Missing contact evidence authors a current-state flight-mode request: ballistic gravity, attitude damping, and joint damping, with no support centroid or fictitious contact impulse.",
            "- A separate zero-contact floating WBC must admit the command on every missing tick before generic Rust authority emits typed selection 5.",
            "- The command carries no contact-force witness and never refreshes cached Primary command age, health, or contact evidence.",
            "- The matched 5/10 ms first-loss pair is bit-exact through the first brake command; no burst-duration oracle exists.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-support-free-brake-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_SUPPORT_FREE_BRAKE_AUDIT.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "brake_consequence_admitted": brake_consequence_admitted,
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
