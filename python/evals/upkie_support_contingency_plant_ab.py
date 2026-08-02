#!/usr/bin/env python3
"""Four-authority causal plant A/B for the independently admitted R175 action."""

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


REVISION = "upkie-support-contingency-plant-ab-r176"
DURATION_S = 6.0
PHYSICAL_FIELDS = (
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
    "maximum_abs_qacc",
    "external_force_world",
    "application_point_world",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_SUPPORT_CONTINGENCY_PLANT_AB_R176.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def execute(
    model: pathlib.Path,
    case: Any,
    duration: float,
    *,
    measured_contact: bool,
    contingency_shadow: bool,
    contingency_execute: bool,
    preserve_primary_support: bool = False,
    query_every_tick: bool = True,
    flight_only: bool = False,
    single_evaluation_lease: bool = False,
    forecast_guard: bool = False,
    contact_program_authority_ticks: int | None = None,
    project_primary: bool = False,
    realize_primary_torque: bool = False,
    contact_observation_prestart_samples: int = 0,
    use_feasibility_row_spans: bool = False,
) -> dict[str, Any]:
    trace = run_case(
        model,
        case,
        duration,
        balance_mode="capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=measured_contact,
        execute_reduced_support=True,
        support_contingency_enabled=contingency_shadow,
        support_contingency_execute=contingency_execute,
        support_contingency_preserve_primary_support=preserve_primary_support,
        support_contingency_query_every_tick=query_every_tick,
        support_contingency_flight_only=flight_only,
        support_contingency_single_evaluation_lease=single_evaluation_lease,
        support_contingency_forecast_guard=forecast_guard,
        support_contingency_project_primary=project_primary,
        support_contingency_realize_primary_torque=realize_primary_torque,
        contact_program_authority_ticks=contact_program_authority_ticks,
        contact_observation_prestart_samples=contact_observation_prestart_samples,
        use_feasibility_row_spans=use_feasibility_row_spans,
    )
    metrics = summarize(case, trace, duration)
    selected = np.asarray(trace["support_contingency_selected"]) != 0
    admitted = np.asarray(trace["support_contingency_admitted"]) != 0
    armed = np.asarray(trace["support_contingency_armed"]) != 0
    requested = np.asarray(trace["support_contingency_requested"]) != 0
    support_mask = np.asarray(trace["support_contingency_support_mask"])
    contingency_status = np.asarray(trace["support_contingency_status"])
    forecast_passed = np.asarray(
        trace["support_contingency_forecast_guard_passed"]
    ) != 0
    forecast_baseline = np.asarray(
        trace["support_contingency_forecast_baseline_score"]
    )
    forecast_candidate = np.asarray(
        trace["support_contingency_forecast_candidate_score"]
    )
    candidate_acceleration = np.asarray(
        trace["support_contingency_candidate_generalized_acceleration"]
    )
    program_selection = np.asarray(
        trace["contact_program_authority_selection"]
    )
    program_executable = np.asarray(
        trace["contact_program_authority_executable"]
    ) != 0
    realization_fallback = np.asarray(
        trace["support_contingency_realization_fallback"]
    ) != 0
    primary_torque = np.asarray(trace["support_contingency_primary_torque"])
    executed_torque = np.asarray(trace["torque"])
    hard_mask = np.asarray(trace["contact_program_authority_hard_mask"])
    stable_mask = np.asarray(trace["contact_program_authority_stable_mask"])
    observed_contact = np.asarray(trace["observed_contact_active"])
    observed_mask = observed_contact[:, 0] | (observed_contact[:, 1] << 1)
    metrics.update(
        {
            "measured_contact": measured_contact,
            "contingency_shadow": contingency_shadow,
            "contingency_execute": contingency_execute,
            "support_contingency_realize_primary_torque": (
                realize_primary_torque
            ),
            "contact_observation_prestart_samples": (
                contact_observation_prestart_samples
            ),
            "use_feasibility_row_spans": use_feasibility_row_spans,
            "contingency_admitted_ticks": int(np.sum(admitted)),
            "contingency_armed_ticks": int(np.sum(armed)),
            "contingency_requested_ticks": int(np.sum(requested)),
            "contingency_selected_ticks": int(np.sum(selected)),
            "contingency_guard_rejected_ticks": int(
                np.sum(requested & ~selected)
            ),
            "contingency_selected_support_masks": sorted(
                int(value) for value in np.unique(support_mask[selected])
            ),
            "contingency_status_counts": {
                str(int(value)): int(np.sum(contingency_status == value))
                for value in np.unique(contingency_status)
            },
            "contingency_maximum_constraint_violation": float(
                np.max(trace["support_contingency_maximum_constraint_violation"])
            ),
            "contingency_author_step_ns_p99": float(
                np.percentile(trace["support_contingency_author_step_ns"], 99)
            ),
            "contingency_wbc_step_ns_p99": float(
                np.percentile(trace["support_contingency_wbc_step_ns"], 99)
            ),
            "contingency_wbc_step_ns_maximum": int(
                np.max(trace["support_contingency_wbc_step_ns"])
            ),
            "contingency_forecast_step_ns_p99": float(
                np.percentile(trace["support_contingency_forecast_step_ns"], 99)
            ),
            "contingency_forecast_step_ns_maximum": int(
                np.max(trace["support_contingency_forecast_step_ns"])
            ),
            "contingency_requested_at_most_once": bool(np.sum(requested) <= 1),
            "selection_only_in_flight": bool(
                np.all(~selected | (support_mask == 0))
            ),
            "selection_only_when_forecast_guard_passed": bool(
                np.all(~selected | forecast_passed)
            ),
            "minimum_selected_forecast_improvement": (
                float(np.min(forecast_baseline[selected] - forecast_candidate[selected]))
                if np.any(selected)
                else None
            ),
            "requested_forecast_improvement": (
                float(
                    forecast_baseline[requested][0]
                    - forecast_candidate[requested][0]
                )
                if np.any(requested)
                else None
            ),
            "requested_root_angular_acceleration_rad_s2": (
                float(np.max(np.abs(candidate_acceleration[requested][0, :3])))
                if np.any(requested)
                else None
            ),
            "maximum_selected_root_angular_acceleration_rad_s2": (
                float(np.max(np.abs(candidate_acceleration[selected, :3])))
                if np.any(selected)
                else 0.0
            ),
            "selection_only_after_admission": bool(np.all(~selected | admitted)),
            "selection_only_after_causal_request": bool(
                np.all(~selected | requested)
            ),
            "request_only_after_double_support_arming": bool(
                np.all(~requested | (armed & (support_mask != 3)))
            ),
            "selection_only_after_support_loss": bool(
                np.all(~selected | (support_mask != 3))
            ),
            "selected_status_is_executable": bool(
                np.all(~selected | np.isin(contingency_status, (0, 1)))
            ),
            "contact_program_primary_ticks": int(
                np.sum(program_selection == 1)
            ),
            "contact_program_current_support_ticks": int(
                np.sum(program_selection == 2)
            ),
            "contact_program_lease_ticks": int(
                np.sum(program_selection == 3)
            ),
            "contact_program_withheld_ticks": int(
                np.sum(program_selection == 0)
            ),
            "contact_program_executable_ticks": int(
                np.sum(program_executable)
            ),
            "contact_program_masks_consistent": bool(
                np.all(
                    np.asarray(
                        trace["contact_program_authority_masks_consistent"]
                    )
                    != 0
                )
            ),
            "contact_program_hard_mask_matches_raw_stable_intersection": bool(
                np.array_equal(hard_mask, observed_mask & stable_mask)
            ),
            "support_contingency_realization_fallback_ticks": int(
                np.sum(realization_fallback)
            ),
            "maximum_primary_execution_torque_error": (
                float(
                    np.max(
                        np.abs(
                            executed_torque[program_executable]
                            - primary_torque[program_executable]
                        )
                    )
                )
                if np.any(program_executable)
                else 0.0
            ),
        }
    )
    return {"trace": trace, "metrics": metrics}


def physical_trace_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return bool(
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in PHYSICAL_FIELDS
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
        and np.array_equal(
            left["terminal_root_position"], right["terminal_root_position"]
        )
        and np.array_equal(
            left["terminal_rotation_vector"], right["terminal_rotation_vector"]
        )
    )


def outcome(metrics: dict[str, Any]) -> str:
    if metrics["fell"]:
        return f"FALL {float(metrics['terminal_time_s']):.3f}s"
    return str(metrics["outcome"])


def main() -> None:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    selected_cases = case_matrix()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in requested)
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    checkpoint = destination / "upkie-support-contingency-plant-ab-rows.json"
    rows: dict[str, Any] = {}
    if checkpoint.exists():
        retained = json.loads(checkpoint.read_text())
        if (
            retained.get("model") == str(model)
            and retained.get("duration_s") == args.duration
        ):
            rows = retained.get("rows", {})
    for index, case in enumerate(selected_cases, 1):
        if case.name in rows:
            print(
                f"[{index:02d}/{len(selected_cases)}] {case.name}: retained checkpoint",
                flush=True,
            )
            continue
        sentinel = execute(
            model,
            case,
            args.duration,
            measured_contact=False,
            contingency_shadow=False,
            contingency_execute=False,
        )
        control = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=False,
            contingency_execute=False,
        )
        shadow = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=False,
        )
        candidate = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=True,
        )
        replay = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=True,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "r137_sentinel": sentinel["metrics"],
            "measured_contact_control": control["metrics"],
            "contingency_shadow": shadow["metrics"],
            "candidate": candidate["metrics"],
            "shadow_execution_exact": physical_trace_equal(
                control["trace"], shadow["trace"]
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }
        checkpoint.write_text(
            json.dumps(
                {"model": str(model), "duration_s": args.duration, "rows": rows},
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        print(
            f"[{index:02d}/{len(selected_cases)}] {case.name}: "
            f"r137={outcome(sentinel['metrics'])}, "
            f"control={outcome(control['metrics'])}, "
            f"candidate={outcome(candidate['metrics'])}, "
            f"selected={candidate['metrics']['contingency_selected_ticks']}",
            flush=True,
        )

    full_matrix = len(rows) == len(case_matrix())
    controls = [row["measured_contact_control"] for row in rows.values()]
    candidates = [row["candidate"] for row in rows.values()]
    retained_green = [
        name for name, row in rows.items() if row["r137_sentinel"]["qualified"]
    ]
    measured_green = [
        name
        for name, row in rows.items()
        if row["measured_contact_control"]["qualified"]
    ]
    lost_retained_green = [
        name for name in retained_green if not rows[name]["candidate"]["qualified"]
    ]
    lost_measured_green = [
        name for name in measured_green if not rows[name]["candidate"]["qualified"]
    ]
    control_falls = [
        name for name, row in rows.items() if row["measured_contact_control"]["fell"]
    ]
    boundary_delta_s = {
        name: float(rows[name]["candidate"]["terminal_time_s"])
        - float(rows[name]["measured_contact_control"]["terminal_time_s"])
        for name in control_falls
        if rows[name]["candidate"]["fell"]
    }
    earlier_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta < -1.0e-12
    }
    later_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta > 1.0e-12
    }
    mechanism_gates = {
        "matrix_complete": full_matrix,
        "shadow_execution_is_exactly_neutral": all(
            row["shadow_execution_exact"] for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "selection_exercised": any(
            row["contingency_selected_ticks"] > 0 for row in candidates
        ),
        "selection_only_after_admission": all(
            row["selection_only_after_admission"] for row in candidates
        ),
        "selection_only_after_causal_arming_request": all(
            row["selection_only_after_causal_request"]
            and row["request_only_after_double_support_arming"]
            for row in candidates
        ),
        "selection_only_after_exact_support_loss": all(
            row["selection_only_after_support_loss"] for row in candidates
        ),
        "selected_rows_are_typed_executable": all(
            row["selected_status_is_executable"] for row in candidates
        ),
        "candidate_finite": all(row["finite"] for row in candidates),
        "zero_timed_rust_allocation": all(row["allocation_free"] for row in candidates),
        "zero_python_gc": all(row["python_gc_collections"] == 0 for row in candidates),
    }
    promotion_gates = {
        "retained_green_rows_preserved": not lost_retained_green,
        "measured_control_green_rows_preserved": not lost_measured_green,
        "no_existing_fall_boundary_earlier": not earlier_boundaries,
        "no_candidate_numeric_fault": not any(row["numeric_fault"] for row in candidates),
        "zero_5ms_loop_overruns": sum(row["loop_overruns"] for row in candidates) == 0,
    }
    mechanism_passed = all(mechanism_gates.values())
    controller_promoted = mechanism_passed and all(promotion_gates.values())
    report_rows = []
    for name, row in rows.items():
        delta = boundary_delta_s.get(name)
        report_rows.append(
            [
                name,
                outcome(row["r137_sentinel"]),
                outcome(row["measured_contact_control"]),
                outcome(row["candidate"]),
                row["candidate"]["contingency_selected_ticks"],
                "—" if delta is None else f"{delta:+.3f}",
            ]
        )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "promotion_gates": promotion_gates,
        "retained_green_rows": retained_green,
        "lost_retained_green_rows": lost_retained_green,
        "measured_green_rows": measured_green,
        "lost_measured_green_rows": lost_measured_green,
        "earlier_boundaries_s": earlier_boundaries,
        "later_boundaries_s": later_boundaries,
        "total_selected_ticks": sum(
            row["contingency_selected_ticks"] for row in candidates
        ),
        "total_candidate_loop_overruns": sum(row["loop_overruns"] for row in candidates),
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw causal support-contingency plant A/B · r176",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · controller promotion **{'PASS' if controller_promoted else 'REJECTED'}**. Four authority arms retain r137 sentinel, measured-contact control, non-executing R175 shadow, and causal R175 selection; the selected arm is rerun for exact replay.",
            "",
            "## Consequence",
            "",
            *markdown_table(
                ["case", "r137", "measured control", "selected candidate", "selected ticks", "fall Δ s"],
                report_rows,
            ),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in mechanism_gates.items()],
            ),
            "",
            "## Promotion gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in promotion_gates.items()],
            ),
            "",
            "## Authority contract",
            "",
            "- The shadow arm runs Rust FK/request authoring and a second floating WBC every tick but cannot affect torque; physical equality with measured-contact control is an explicit gate.",
            "- Selection is causal and state-minimal: one exact `11` observation arms the run, exact current hard support later leaves `11`, the independent candidate status is `Solved`/`SolvedWithSlack`, and hard violation is below `1e-8`. No elapsed-time dwell, blend, timeout extension, cache resurrection, plant reset, or rejected-row execution participates.",
            "- A mechanism pass proves only that selection is typed and reproducible. Promotion additionally requires every retained green row, every measured-control green row, no earlier existing fall boundary, no numeric fault, and zero 5 ms loop overruns.",
        ]
    ) + "\n"
    (destination / "upkie-support-contingency-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_SUPPORT_CONTINGENCY_PLANT_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "controller_promoted": controller_promoted,
                "mechanism_gates": mechanism_gates,
                "promotion_gates": promotion_gates,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
