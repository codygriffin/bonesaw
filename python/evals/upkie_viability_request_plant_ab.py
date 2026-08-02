#!/usr/bin/env python3
"""Retained plant A/B for the supervised bounded viability planner."""

from __future__ import annotations

import argparse
import json
import math
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


R149_REVISION = "upkie-viability-request-plant-ab-r149"
R154_REVISION = "upkie-multistep-measured-contact-plant-ab-r154"
R155_REVISION = "upkie-paired-multistep-measured-contact-plant-ab-r155"
R156_REVISION = "upkie-confirmed-multistep-measured-contact-plant-ab-r156"
R158_REVISION = "upkie-hybrid-guard-measured-contact-plant-ab-r158"
DURATION_S = 6.0
REQUEST_LIMIT = np.asarray([250.0, 250.0, 80.0], np.float64)
SLEW_LIMIT = np.asarray([40.0, 40.0, 20.0], np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output")
    parser.add_argument("--web-report")
    parser.add_argument(
        "--planner-strategy",
        choices=(
            "coordinate",
            "multistep_budgeted",
            "lateral_paired_multistep_budgeted",
            "hybrid_confirmed_multistep_budgeted",
        ),
        default="coordinate",
    )
    parser.add_argument(
        "--cases",
        help="comma-separated subset for smoke/debug; admission requires all 20",
    )
    parser.add_argument(
        "--confirmation-updates",
        type=int,
        default=1,
        help="consistent forecast proposals required before request supervision",
    )
    return parser.parse_args()


def execute(
    model: pathlib.Path,
    case: Any,
    duration: float,
    *,
    measured_contact: bool,
    planner: bool,
    viability_support_requires_active_request: bool = False,
    viability_planner_strategy: str = "coordinate",
    viability_planner_update_period_ticks: int | None = None,
    viability_confirmation_updates: int = 1,
    maximum_feasibility_projection_sweeps: int | None = None,
    repair_feasibility_equalities_before_inequalities: bool = False,
    use_feasibility_row_spans: bool = False,
    viability_planner_maximum_feasibility_projection_sweeps: int | None = None,
    viability_planner_projection_continuation_violation_threshold: float | None = None,
    viability_planner_reuse_identical_hard_feasibility_seed: bool = False,
    viability_planner_transfer_hard_feasibility_witness: bool = False,
    execution_residual_veto: bool = False,
) -> dict[str, Any]:
    trace = run_case(
        model,
        case,
        duration,
        "capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=measured_contact,
        execute_reduced_support=True,
        viability_planner_enabled=planner,
        viability_support_requires_active_request=(
            viability_support_requires_active_request
        ),
        viability_planner_strategy=viability_planner_strategy,
        viability_planner_update_period_ticks=viability_planner_update_period_ticks,
        viability_confirmation_updates=viability_confirmation_updates,
        maximum_feasibility_projection_sweeps=maximum_feasibility_projection_sweeps,
        repair_feasibility_equalities_before_inequalities=repair_feasibility_equalities_before_inequalities,
        use_feasibility_row_spans=use_feasibility_row_spans,
        viability_planner_maximum_feasibility_projection_sweeps=viability_planner_maximum_feasibility_projection_sweeps,
        viability_planner_projection_continuation_violation_threshold=viability_planner_projection_continuation_violation_threshold,
        viability_planner_reuse_identical_hard_feasibility_seed=viability_planner_reuse_identical_hard_feasibility_seed,
        viability_planner_transfer_hard_feasibility_witness=viability_planner_transfer_hard_feasibility_witness,
        execution_residual_veto=execution_residual_veto,
    )
    metrics = summarize(case, trace, duration)
    request = np.asarray(trace["viability_request"])
    status = np.asarray(trace["viability_request_status"])
    executable = np.asarray(trace["viability_request_executable"]) != 0
    query_count = np.asarray(trace["viability_planner_query_count"])
    planner_updates = np.asarray(trace["viability_planner_update"]) != 0
    search_cardinality = {
        "coordinate": 40,
        "multistep_budgeted": 4,
        "lateral_paired_multistep_budgeted": 2,
        "hybrid_confirmed_multistep_budgeted": 4,
    }[viability_planner_strategy]
    searches = query_count == search_cardinality
    admitted_searches = searches & (status == 1)
    zero_pressure = np.asarray(trace["viability_planner_zero_pressure"])
    candidate_pressure = np.asarray(trace["viability_planner_candidate_pressure"])
    request_delta = np.abs(np.diff(request, axis=0))
    normal_transition = ~np.isin(status[1:], [4, 5, 6, 7, 8])
    confirmation_status = np.asarray(trace["viability_confirmation_status"])
    confirmation_executable = (
        np.asarray(trace["viability_confirmation_executable"]) != 0
    )
    confirmation_flags = np.asarray(trace["viability_confirmation_flags"])
    hybrid_guard_status = np.asarray(trace["viability_hybrid_guard_status"])
    hybrid_guard_executable = (
        np.asarray(trace["viability_hybrid_guard_executable"]) != 0
    )
    hybrid_guard_shadow_admissible = (
        np.asarray(trace["viability_hybrid_guard_shadow_admissible"]) != 0
    )
    observed_slew = (
        np.max(request_delta[normal_transition], axis=0)
        if np.any(normal_transition)
        else np.zeros(3, np.float64)
    )
    metrics.update(
        {
            "planner_enabled": planner,
            "maximum_feasibility_projection_sweeps": (
                maximum_feasibility_projection_sweeps
            ),
            "repair_feasibility_equalities_before_inequalities": (
                repair_feasibility_equalities_before_inequalities
            ),
            "use_feasibility_row_spans": use_feasibility_row_spans,
            "viability_planner_maximum_feasibility_projection_sweeps": (
                viability_planner_maximum_feasibility_projection_sweeps
            ),
            "viability_planner_projection_continuation_violation_threshold": (
                viability_planner_projection_continuation_violation_threshold
            ),
            "execution_residual_veto": execution_residual_veto,
            "execution_residual_veto_ticks": int(
                np.sum(np.asarray(trace["execution_residual_veto_active"]) != 0)
            ),
            "execution_residual_veto_request_overlap_ticks": int(
                np.sum(
                    (np.asarray(trace["execution_residual_veto_active"]) != 0)
                    & executable
                )
            ),
            "execution_residual_exceeded_ticks": int(
                np.sum(np.asarray(trace["execution_residual_status"]) == 2)
            ),
            "execution_residual_allocation_calls": int(
                np.sum(np.asarray(trace["execution_residual_allocation_calls"]))
            ),
            "execution_residual_allocated_bytes": int(
                np.sum(np.asarray(trace["execution_residual_allocated_bytes"]))
            ),
            "planner_strategy": viability_planner_strategy,
            "planner_update_period_ticks": viability_planner_update_period_ticks,
            "search_query_cardinality": search_cardinality,
            "measured_contact": measured_contact,
            "planner_update_steps": int(np.sum(planner_updates)),
            "planner_skipped_steps": int(np.sum(~planner_updates)),
            "maximum_observed_planner_update_gap_ticks": int(
                np.max(np.diff(np.flatnonzero(planner_updates)))
            )
            if np.sum(planner_updates) > 1
            else 0,
            "planner_search_steps": int(np.sum(searches)),
            "total_planner_wbc_queries": int(np.sum(query_count)),
            "planner_query_cardinalities": sorted(
                int(value) for value in np.unique(query_count)
            ),
            "active_request_steps": int(
                np.sum(np.asarray(trace["viability_request_active"]) != 0)
            ),
            "executable_request_steps": int(np.sum(executable)),
            "request_status_counts": {
                str(int(value)): int(np.sum(status == value))
                for value in np.unique(status)
            },
            "maximum_abs_request_roll_lateral_yaw": np.max(
                np.abs(request), axis=0
            ).tolist(),
            "maximum_observed_normal_slew_roll_lateral_yaw": observed_slew.tolist(),
            "maximum_candidate_age_ticks": int(
                np.max(np.asarray(trace["viability_request_age_ticks"]))
            ),
            "strict_descent_searches": int(
                np.sum(admitted_searches & (candidate_pressure < zero_pressure))
            ),
            "admitted_search_steps": int(np.sum(admitted_searches)),
            "all_admitted_searches_strict_descent": bool(
                not np.any(admitted_searches)
                or np.all(
                    candidate_pressure[admitted_searches]
                    < zero_pressure[admitted_searches]
                )
            ),
            "minimum_candidate_over_zero_pressure": float(
                np.min(
                    candidate_pressure[searches]
                    / np.maximum(zero_pressure[searches], 1.0e-12)
                )
            )
            if np.any(searches)
            else 1.0,
            "executable_request_wbc_rejections": int(
                np.sum(executable & ~np.isin(np.asarray(trace["status"]), [0, 1]))
            ),
            "support_transition_count": int(
                np.asarray(trace["support_transition_count"])[-1]
            ),
            "confirmation_updates": viability_confirmation_updates,
            "confirmation_status_counts": {
                str(int(value)): int(np.sum(confirmation_status == value))
                for value in np.unique(confirmation_status)
            },
            "confirmation_shadow_steps": int(np.sum(confirmation_status == 1)),
            "confirmation_executable_steps": int(np.sum(confirmation_executable)),
            "confirmation_support_revocations": int(
                np.sum(confirmation_status == 4)
            ),
            "confirmation_evidence_revocations": int(
                np.sum(confirmation_status == 3)
            ),
            "confirmation_direction_resets": int(
                np.sum((confirmation_flags & (1 << 7)) != 0)
            ),
            "request_only_after_confirmation": bool(
                not np.any(executable & ~confirmation_executable)
            )
            if viability_confirmation_updates > 1
            else True,
            "hybrid_guard_status_counts": {
                str(int(value)): int(np.sum(hybrid_guard_status == value))
                for value in np.unique(hybrid_guard_status)
            },
            "hybrid_guard_admitted_steps": int(np.sum(hybrid_guard_status == 1)),
            "hybrid_guard_shadow_only_steps": int(
                np.sum(hybrid_guard_shadow_admissible & ~hybrid_guard_executable)
            ),
            "hybrid_guard_support_age_rejections": int(
                np.sum(hybrid_guard_status == 3)
            ),
            "hybrid_guard_direction_rejections": int(
                np.sum(hybrid_guard_status == 4)
            ),
            "hybrid_guard_load_rejections": int(np.sum(hybrid_guard_status == 5)),
            "request_only_after_hybrid_guard": bool(
                not np.any(executable & ~hybrid_guard_executable)
            )
            if viability_planner_strategy == "hybrid_confirmed_multistep_budgeted"
            else True,
        }
    )
    return {"trace": trace, "metrics": metrics}


def outcome(metrics: dict[str, Any]) -> str:
    if not metrics["fell"]:
        return str(metrics["outcome"])
    return f"FALL {float(metrics['terminal_time_s']):.3f}s"


def execution_trace_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    fields = (
        "time_s",
        "root_position",
        "root_twist",
        "rotation_vector",
        "q",
        "v",
        "torque",
        "wbc_normal_force",
        "status",
        "command_age_steps",
        "contact_count",
        "observed_contact_active",
        "admitted_contact_active",
        "support_transition_count",
        "fall_safe_mode",
        "fall_safe_primary_authority",
        "fall_safe_fresh_command_authority",
        "torque_utilization",
        "allocation_calls",
        "allocated_bytes",
        "maximum_abs_qacc",
        "external_force_world",
        "application_point_world",
    )
    return bool(
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in fields
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
    )


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    if args.confirmation_updates < 1:
        raise SystemExit("--confirmation-updates must be at least one")
    if args.confirmation_updates > 1 and args.planner_strategy not in (
        "multistep_budgeted",
        "hybrid_confirmed_multistep_budgeted",
    ):
        raise SystemExit(
            "confirmation evaluation requires a multistep or hybrid-confirmed strategy"
        )
    if (
        args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
        and args.confirmation_updates < 2
    ):
        raise SystemExit("hybrid-confirmed evaluation requires at least two updates")
    model = pathlib.Path(args.model).resolve()
    revision = (
        R158_REVISION
        if args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
        else (
            R156_REVISION
            if args.confirmation_updates > 1
            else {
                "coordinate": R149_REVISION,
                "multistep_budgeted": R154_REVISION,
                "lateral_paired_multistep_budgeted": R155_REVISION,
            }[args.planner_strategy]
        )
    )
    output_path = args.output or f"benchmarks/results/{revision}"
    web_report_path = args.web_report or (
        "web/UPKIE_HYBRID_GUARD_MEASURED_CONTACT_PLANT_AB_R158.html"
        if args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
        else (
            "web/UPKIE_CONFIRMED_MULTISTEP_MEASURED_CONTACT_PLANT_AB_R156.html"
            if args.confirmation_updates > 1
            else {
                "coordinate": "web/UPKIE_VIABILITY_REQUEST_PLANT_AB_R149.html",
                "multistep_budgeted": "web/UPKIE_MULTISTEP_MEASURED_CONTACT_PLANT_AB_R154.html",
                "lateral_paired_multistep_budgeted": "web/UPKIE_PAIRED_MULTISTEP_MEASURED_CONTACT_PLANT_AB_R155.html",
            }[args.planner_strategy]
        )
    )
    budgeted = args.planner_strategy != "coordinate"
    request_limit = (
        np.asarray([40.0, 40.0, 20.0], np.float64)
        if budgeted
        else REQUEST_LIMIT
    )
    expected_cardinalities = (
        (
            {0, 1, 2}
            if args.planner_strategy == "lateral_paired_multistep_budgeted"
            else {0, 1, 4}
        )
        if budgeted
        else {0, 1, 40}
    )
    maximum_candidate_age = 1 if budgeted else 4
    selected = case_matrix()
    if args.cases:
        names = set(args.cases.split(","))
        selected = tuple(case for case in selected if case.name in names)
        missing = names - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    rows: dict[str, Any] = {}
    for index, case in enumerate(selected, start=1):
        legacy = execute(
            model, case, args.duration, measured_contact=False, planner=False
        )
        control = execute(
            model, case, args.duration, measured_contact=True, planner=False
        )
        candidate = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            planner=True,
            viability_planner_strategy=args.planner_strategy,
            viability_confirmation_updates=args.confirmation_updates,
        )
        replay = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            planner=True,
            viability_planner_strategy=args.planner_strategy,
            viability_confirmation_updates=args.confirmation_updates,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "r137_sentinel": legacy["metrics"],
            "measured_contact_control": control["metrics"],
            "candidate": candidate["metrics"],
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
            "control_candidate_execution_exact": execution_trace_equal(
                control["trace"], candidate["trace"]
            ),
        }
        print(
            f"[{index:02d}/{len(selected)}] {case.name}: "
            f"r137={outcome(legacy['metrics'])}, "
            f"control={outcome(control['metrics'])}, "
            f"candidate={outcome(candidate['metrics'])}, "
            f"searches={candidate['metrics']['planner_search_steps']}",
            flush=True,
        )

    full_matrix = len(rows) == len(case_matrix())
    controls = [row["measured_contact_control"] for row in rows.values()]
    candidates = [row["candidate"] for row in rows.values()]
    green_controls = [
        name
        for name, row in rows.items()
        if row["measured_contact_control"]["qualified"]
    ]
    control_falls = [
        name
        for name, row in rows.items()
        if row["measured_contact_control"]["fell"]
    ]
    boundary_delta_s = {
        name: float(rows[name]["candidate"]["terminal_time_s"])
        - float(rows[name]["measured_contact_control"]["terminal_time_s"])
        for name in control_falls
    }
    earlier_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta < -1.0e-12
    }
    later_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta > 1.0e-12
    }
    consequence_keys = (
        "maximum_translation_m",
        "maximum_tilt_deg",
        "maximum_torque_utilization",
        "final_station_error_m",
    )
    consequence_deltas = {
        key: {
            "mean": float(
                np.mean(
                    [
                        row["candidate"][key]
                        - row["measured_contact_control"][key]
                        for row in rows.values()
                    ]
                )
            ),
            "maximum": float(
                np.max(
                    [
                        row["candidate"][key]
                        - row["measured_contact_control"][key]
                        for row in rows.values()
                    ]
                )
            ),
        }
        for key in consequence_keys
    }
    mechanism_gates = {
        "matrix_complete": full_matrix,
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "planner_search_exercised": any(
            candidate["planner_search_steps"] > 0 for candidate in candidates
        ),
        "inactive_planner_is_execution_neutral": all(
            row["control_candidate_execution_exact"]
            for row in rows.values()
            if row["candidate"]["executable_request_steps"] == 0
        ),
        "query_cardinality_is_bounded": all(
            set(candidate["planner_query_cardinalities"]) <= expected_cardinalities
            for candidate in candidates
        ),
        "every_admitted_search_strictly_descends": all(
            candidate["all_admitted_searches_strict_descent"]
            for candidate in candidates
        ),
        "request_bounds_hold": all(
            np.all(
                np.asarray(candidate["maximum_abs_request_roll_lateral_yaw"])
                <= request_limit + 1.0e-12
            )
            for candidate in candidates
        ),
        "normal_slew_bounds_hold": all(
            np.all(
                np.asarray(
                    candidate["maximum_observed_normal_slew_roll_lateral_yaw"]
                )
                <= SLEW_LIMIT + 1.0e-12
            )
            for candidate in candidates
        ),
        "candidate_age_is_bounded": all(
            candidate["maximum_candidate_age_ticks"] <= maximum_candidate_age
            for candidate in candidates
        ),
        "zero_rust_allocation": all(
            side["allocation_free"] for side in (*controls, *candidates)
        ),
        "finite": all(side["finite"] for side in (*controls, *candidates)),
    }
    if args.confirmation_updates > 1:
        mechanism_gates.update(
            {
                "confirmation_shadow_exercised": any(
                    candidate["confirmation_shadow_steps"] > 0
                    for candidate in candidates
                ),
                "confirmation_execution_exercised": any(
                    candidate["confirmation_executable_steps"] > 0
                    for candidate in candidates
                ),
                "request_only_after_confirmation": all(
                    candidate["request_only_after_confirmation"]
                    for candidate in candidates
                ),
            }
        )
        if args.planner_strategy != "hybrid_confirmed_multistep_budgeted":
            mechanism_gates["support_change_revocation_exercised"] = any(
                candidate["confirmation_support_revocations"] > 0
                for candidate in candidates
            )
    if args.planner_strategy == "hybrid_confirmed_multistep_budgeted":
        mechanism_gates.update(
            {
                "hybrid_guard_admission_exercised": any(
                    candidate["hybrid_guard_admitted_steps"] > 0
                    for candidate in candidates
                ),
                "hybrid_guard_rejection_exercised": any(
                    candidate["hybrid_guard_support_age_rejections"]
                    + candidate["hybrid_guard_direction_rejections"]
                    + candidate["hybrid_guard_load_rejections"]
                    > 0
                    for candidate in candidates
                ),
                "support_discontinuity_prevention_exercised": any(
                    candidate["hybrid_guard_support_age_rejections"] > 0
                    for candidate in candidates
                ),
                "hybrid_guard_shadow_only_exercised": any(
                    candidate["hybrid_guard_shadow_only_steps"] > 0
                    for candidate in candidates
                ),
                "request_only_after_hybrid_guard": all(
                    candidate["request_only_after_hybrid_guard"]
                    for candidate in candidates
                ),
            }
        )
    physical_gates = {
        "no_new_numeric_fault": all(
            not row["candidate"]["numeric_fault"]
            for row in rows.values()
            if not row["measured_contact_control"]["numeric_fault"]
        ),
        "no_new_fall": all(
            not row["candidate"]["fell"]
            for row in rows.values()
            if not row["measured_contact_control"]["fell"]
        ),
        "no_earlier_fall_boundary": all(
            float(rows[name]["candidate"]["terminal_time_s"])
            >= float(rows[name]["measured_contact_control"]["terminal_time_s"])
            for name in control_falls
        ),
        "green_control_qualification_preserved": all(
            rows[name]["candidate"]["qualified"] for name in green_controls
        ),
        "no_loop_deadline_overrun": all(
            candidate["loop_overruns"] == 0 for candidate in candidates
        ),
    }
    mechanism_passed = all(mechanism_gates.values())
    physical_passed = all(physical_gates.values())
    promoted = mechanism_passed and physical_passed and full_matrix

    table_rows = []
    for name, row in rows.items():
        control = row["measured_contact_control"]
        candidate = row["candidate"]
        table_rows.append(
            [
                name,
                outcome(row["r137_sentinel"]),
                outcome(control),
                outcome(candidate),
                candidate["planner_search_steps"],
                candidate["hybrid_guard_admitted_steps"],
                candidate["hybrid_guard_support_age_rejections"]
                + candidate["hybrid_guard_direction_rejections"]
                + candidate["hybrid_guard_load_rejections"],
                candidate["confirmation_shadow_steps"],
                candidate["confirmation_executable_steps"],
                candidate["active_request_steps"],
                "/".join(
                    f"{value:.0f}"
                    for value in candidate[
                        "maximum_abs_request_roll_lateral_yaw"
                    ]
                ),
                f"{candidate['loop_ns']['p99'] / 1.0e3:.0f}",
                f"{candidate['viability_planner_step_ns']['p99'] / 1.0e3:.0f}",
                f"{candidate['final_wbc_step_ns']['p99'] / 1.0e3:.0f}",
                candidate["loop_overruns"],
                "YES" if row["candidate_replay_exact"] else "NO",
            ]
        )

    reference_summary: dict[str, Any] | None = None
    if budgeted:
        reference_path = pathlib.Path(
            "benchmarks/results/upkie-viability-request-plant-ab-r149/"
            "upkie-viability-request-plant-ab-metrics.json"
        )
        if reference_path.exists():
            reference = json.loads(reference_path.read_text())
            reference_boundary_delta = {
                name: float(row["candidate"]["terminal_time_s"])
                - float(row["measured_contact_control"]["terminal_time_s"])
                for name, row in reference["rows"].items()
                if row["measured_contact_control"]["fell"]
            }
            reference_summary = {
                "revision": reference["revision"],
                "earlier_fall_boundaries": sum(
                    delta < -1.0e-12 for delta in reference_boundary_delta.values()
                ),
                "fall_boundary_count": len(reference_boundary_delta),
                "worst_fall_boundary_delta_s": min(
                    reference_boundary_delta.values()
                ),
                "total_exact_planner_queries": reference["aggregate"][
                    "total_exact_planner_queries"
                ],
                "candidate_loop_overruns": reference["aggregate"][
                    "candidate_loop_overruns"
                ],
                "candidate_gc_collections": reference["aggregate"][
                    "candidate_gc_collections"
                ],
                "candidate_rss_delta_bytes_sum": reference["aggregate"][
                    "candidate_rss_delta_bytes_sum"
                ],
            }

    predecessor_summary: dict[str, Any] | None = None
    if args.confirmation_updates > 1:
        predecessor_path = pathlib.Path(
            (
                "benchmarks/results/upkie-confirmed-multistep-measured-contact-plant-ab-r156/"
                if args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
                else "benchmarks/results/upkie-multistep-measured-contact-plant-ab-r154/"
            )
            + "upkie-viability-request-plant-ab-metrics.json"
        )
        if predecessor_path.exists():
            predecessor = json.loads(predecessor_path.read_text())
            predecessor_summary = {
                "revision": predecessor["revision"],
                **predecessor["aggregate"],
            }

    metrics = {
        "revision": revision,
        "planner_strategy": args.planner_strategy,
        "confirmation_updates": args.confirmation_updates,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "physical_passed": physical_passed,
        "promoted": promoted,
        "mechanism_gates": mechanism_gates,
        "physical_gates": physical_gates,
        "aggregate": {
            "case_count": len(rows),
            "green_measured_controls": len(green_controls),
            "total_searches": sum(
                candidate["planner_search_steps"] for candidate in candidates
            ),
            "total_exact_planner_queries": sum(
                candidate["total_planner_wbc_queries"] for candidate in candidates
            ),
            "confirmation_shadow_steps": sum(
                candidate["confirmation_shadow_steps"] for candidate in candidates
            ),
            "confirmation_executable_steps": sum(
                candidate["confirmation_executable_steps"] for candidate in candidates
            ),
            "confirmation_support_revocations": sum(
                candidate["confirmation_support_revocations"]
                for candidate in candidates
            ),
            "confirmation_evidence_revocations": sum(
                candidate["confirmation_evidence_revocations"]
                for candidate in candidates
            ),
            "hybrid_guard_admitted_steps": sum(
                candidate["hybrid_guard_admitted_steps"] for candidate in candidates
            ),
            "hybrid_guard_shadow_only_steps": sum(
                candidate["hybrid_guard_shadow_only_steps"]
                for candidate in candidates
            ),
            "hybrid_guard_support_age_rejections": sum(
                candidate["hybrid_guard_support_age_rejections"]
                for candidate in candidates
            ),
            "hybrid_guard_direction_rejections": sum(
                candidate["hybrid_guard_direction_rejections"]
                for candidate in candidates
            ),
            "hybrid_guard_load_rejections": sum(
                candidate["hybrid_guard_load_rejections"]
                for candidate in candidates
            ),
            "candidate_gc_collections": sum(
                candidate["python_gc_collections"] for candidate in candidates
            ),
            "candidate_rss_delta_bytes_sum": sum(
                candidate["rss_delta_bytes"] for candidate in candidates
            ),
            "candidate_loop_overruns": sum(
                candidate["loop_overruns"] for candidate in candidates
            ),
            "worst_candidate_planner_p99_ns": max(
                candidate["viability_planner_step_ns"]["p99"]
                for candidate in candidates
            ),
            "worst_candidate_final_wbc_p99_ns": max(
                candidate["final_wbc_step_ns"]["p99"] for candidate in candidates
            ),
            "earlier_fall_boundaries": len(earlier_boundaries),
            "later_fall_boundaries": len(later_boundaries),
            "neutral_fall_boundaries": len(boundary_delta_s)
            - len(earlier_boundaries)
            - len(later_boundaries),
            "worst_fall_boundary_delta_s": min(boundary_delta_s.values()),
            "best_fall_boundary_delta_s": max(boundary_delta_s.values()),
            "median_fall_boundary_delta_s": float(
                np.median(list(boundary_delta_s.values()))
            ),
        },
        "fall_boundary_delta_s": boundary_delta_s,
        "consequence_delta_candidate_minus_control": consequence_deltas,
        "r149_reference": reference_summary,
        "direct_predecessor": predecessor_summary,
        "rows": rows,
        "interpretation": {
            "promotion": "promoted" if promoted else "rejected",
            "scope": "bounded state-local viability request layered on the r137 freshness controller; measured-contact control separates contact authority from planner consequence",
            "policy_or_physics_free_precondition": "r146/r147 frozen-state oracle and coordinate planner; r151 adds the fixed-knot reduced forecast and r152 separates lateral wake authority without advancing the plant during selection",
        },
    }

    report = "\n".join(
        [
            (
                "# Bonesaw hybrid-support guarded measured-contact plant A/B · r158"
                if args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
                else (
                    "# Bonesaw confirmed multi-step measured-contact plant A/B · r156"
                    if args.confirmation_updates > 1
                    else (
                        "# Bonesaw paired-query multi-step measured-contact plant A/B · r155"
                        if args.planner_strategy == "lateral_paired_multistep_budgeted"
                        else (
                            "# Bonesaw budgeted multi-step measured-contact plant A/B · r154"
                            if args.planner_strategy == "multistep_budgeted"
                            else "# Bonesaw supervised viability-request plant A/B · r149"
                        )
                    )
                )
            ),
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · physical consequence **{'PASS' if physical_passed else 'FAIL'}** · live promotion **{'YES' if promoted else 'NO'}**.",
            "",
            "## Causal comparison",
            "",
            "Each row has four deterministic runs: the retained r137 assumed-contact sentinel, a measured-contact control with no planner, the supervised planner candidate, and an exact candidate replay. The physical verdict compares candidate against the measured-contact control so contact-observer consequences are not credited to or blamed on the planner. The r137 arm remains visible as the current-behavior reference.",
            "",
            *markdown_table(
                [
                    "case",
                    "r137",
                    "measured control",
                    "candidate",
                    "searches",
                    "guard admit",
                    "guard reject",
                    "shadow ticks",
                    "confirmed ticks",
                    "active ticks",
                    "max r/l/y",
                    "loop p99 µs",
                    "planner p99 µs",
                    "final WBC p99 µs",
                    ">5 ms",
                    "replay",
                ],
                table_rows,
            ),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in mechanism_gates.items()
                ],
            ),
            "",
            "## Physical gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in physical_gates.items()
                ],
            ),
            "",
            "## Consequence deltas",
            "",
            f"- Earlier falling boundaries: **{len(earlier_boundaries)}/{len(boundary_delta_s)}**; later: **{len(later_boundaries)}/{len(boundary_delta_s)}**; neutral: **{len(boundary_delta_s) - len(earlier_boundaries) - len(later_boundaries)}/{len(boundary_delta_s)}**.",
            f"- Worst boundary delta: **{min(boundary_delta_s.values()):+.3f} s** ({min(boundary_delta_s, key=boundary_delta_s.get)}); best: **{max(boundary_delta_s.values()):+.3f} s** ({max(boundary_delta_s, key=boundary_delta_s.get)}); median: **{np.median(list(boundary_delta_s.values())):+.3f} s**.",
            "- Candidate-minus-control mean / worst increases: "
            + "; ".join(
                f"{key} {value['mean']:+.4g} / {value['maximum']:+.4g}"
                for key, value in consequence_deltas.items()
            )
            + ".",
            (
                f"- Against retained r149: earlier boundaries fall from **{reference_summary['earlier_fall_boundaries']}/{reference_summary['fall_boundary_count']}** to **{len(earlier_boundaries)}/{len(boundary_delta_s)}**; worst regression contracts from **{reference_summary['worst_fall_boundary_delta_s']:+.3f} s** to **{min(boundary_delta_s.values()):+.3f} s**; >5 ms loop events fall from **{reference_summary['candidate_loop_overruns']}** to **{sum(candidate['loop_overruns'] for candidate in candidates)}**. Total exact queries are **{sum(candidate['total_planner_wbc_queries'] for candidate in candidates):,}** versus **{reference_summary['total_exact_planner_queries']:,}**, so the win is burst size and jitter, not less aggregate work."
                if reference_summary is not None
                else "- The retained r149 artifact was unavailable; no cross-revision comparison is claimed."
            ),
            (
                f"- Against direct predecessor {predecessor_summary['revision']}, earlier boundaries change **{predecessor_summary['earlier_fall_boundaries']} → {len(earlier_boundaries)}**, later boundaries **{predecessor_summary['later_fall_boundaries']} → {len(later_boundaries)}**, neutral boundaries **{predecessor_summary['neutral_fall_boundaries']} → {len(boundary_delta_s) - len(earlier_boundaries) - len(later_boundaries)}**, exact planner queries **{predecessor_summary['total_exact_planner_queries']:,} → {sum(candidate['total_planner_wbc_queries'] for candidate in candidates):,}**, and >5 ms loops **{predecessor_summary['candidate_loop_overruns']} → {sum(candidate['loop_overruns'] for candidate in candidates)}**. Worst boundary delta changes **{predecessor_summary['worst_fall_boundary_delta_s']:+.3f} → {min(boundary_delta_s.values()):+.3f} s**."
                if predecessor_summary is not None
                else "- The direct predecessor artifact was unavailable; no direct comparison is claimed."
            ),
            "",
            "## Authority and timing contract",
            "",
            "- Python owns retained MuJoCo experiment sequencing. Rust owns each WBC query, fixed-knot scoring, the r155 signed-proposal phase, the r156 confirmation transition, the r158 hybrid-support guard, the final command solve, request activation hysteresis, maximum age, per-tick slew, exact-evidence revocation, and allocation witnesses.",
            (
                "- The r158 guard requires the established three exact raw-support samples used by hard contact eligibility and at least 5% candidate-WBC load on each wheel before double-support execution. A single-support roll request that opens the missing wheel may advance the bounded confirmation shadow but never receives physical execution authority; age, load, evidence, input, and sequence rejections cannot seed or refresh that shadow."
                if args.planner_strategy == "hybrid_confirmed_multistep_budgeted"
                else "- This revision does not insert the r158 hybrid-support guard."
            ),
            (
                f"- The r156 Rust confirmation layer requires **{args.confirmation_updates}** consecutive, directionally aligned improving proposals under the same exact raw support mask. Shadow proposals cannot reach request supervision; evidence loss, support change, failed update, expiry, invalid input, or reordered sequence revokes execution immediately."
                if args.confirmation_updates > 1
                else "- This revision does not insert the r156 confirmation layer."
            ),
            (
                "- The r155 planner performs one baseline query when inactive or two exact WBC queries when searching: the same-state zero baseline plus one Rust-scheduled signed proposal. The final current-support command WBC remains a separate full-budget solve."
                if args.planner_strategy == "lateral_paired_multistep_budgeted"
                else (
                    "- The r154 planner performs one baseline query when inactive or four exact WBC queries when searching: baseline plus one three-point local coordinate question. Rust scores each candidate at eight fixed future knots, and work advances every control tick rather than arriving as a 40-query burst."
                    if args.planner_strategy
                    in ("multistep_budgeted", "hybrid_confirmed_multistep_budgeted")
                    else "- A planner event costs one baseline query when inactive or 40 exact WBC queries when searching. Between events, no planner query runs; the Rust supervisor either holds a fresh candidate, slews release, or revokes it."
                )
            ),
            (
                "- The r155 proposal phase and clipping witness are fixed-size Rust state. Only current roll/lateral capture wakes it; pitch-path non-regression and all other forecast pressures remain independent proposal vetoes."
                if args.planner_strategy == "lateral_paired_multistep_budgeted"
                else (
                    "- The r154 score retains roll/lateral capture, pitch-path non-regression, yaw, single-support closing direction, actuator utilization, joint headroom, action magnitude, and request delta as separate evidence. A 40/40/20 one-tick trust region prevents the lattice-edge/execution mismatch seen in r149."
                    if args.planner_strategy
                    in ("multistep_budgeted", "hybrid_confirmed_multistep_budgeted")
                    else "- The coordinate score is the retained r149 local forecast pressure."
                )
            ),
            "- A nonzero request is not a command. The downstream WBC re-solves every control tick against the current measured hard-contact mask; rejected results cannot become fresh torque.",
            f"- Candidate Python GC collections: **{metrics['aggregate']['candidate_gc_collections']}**; summed within-run RSS delta: **{metrics['aggregate']['candidate_rss_delta_bytes_sum'] / 1_048_576:.2f} MiB**; loop overruns: **{metrics['aggregate']['candidate_loop_overruns']}**.",
            f"- Worst per-case p99 split: planner queries **{metrics['aggregate']['worst_candidate_planner_p99_ns'] / 1.0e3:.1f} µs**; final current-support WBC **{metrics['aggregate']['worst_candidate_final_wbc_p99_ns'] / 1.0e3:.1f} µs**. These clocks are measured separately inside the Rust query boundary; their sum is the controller solve work, not Python loop overhead.",
            "",
            "## Scope",
            "",
            "This plant A/B is downstream of the policy- and physics-free r146/r147 frozen-state evaluations. A green mechanism gate proves bounded execution semantics, not universal recovery. Live promotion additionally requires every physical regression gate; otherwise the planner remains evaluation-only and the retained controller is unchanged.",
        ]
    ) + "\n"

    output = pathlib.Path(output_path)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-viability-request-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_VIABILITY_REQUEST_PLANT_AB_AUDIT.md").write_text(report)
    web_report = pathlib.Path(web_report_path)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "physical_passed": physical_passed,
                "promoted": promoted,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if promoted else 1


if __name__ == "__main__":
    raise SystemExit(main())
