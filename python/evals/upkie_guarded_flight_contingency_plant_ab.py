#!/usr/bin/env python3
"""Admit one forecast-improving flight action without regressing r137."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_support_contingency_plant_ab import execute, outcome


REVISION = "upkie-guarded-flight-contingency-plant-ab-r179"
COMMAND_FIELDS = (
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
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_GUARDED_FLIGHT_CONTINGENCY_PLANT_AB_R179.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def command_trace_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return bool(
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in COMMAND_FIELDS
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
    )


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
    checkpoint = destination / "upkie-guarded-flight-contingency-rows.json"
    rows: dict[str, Any] = {}
    if checkpoint.exists():
        retained = json.loads(checkpoint.read_text())
        if retained.get("model") == str(model) and retained.get("duration_s") == args.duration:
            rows = retained.get("rows", {})

    for index, case in enumerate(selected_cases, 1):
        if case.name in rows:
            print(f"[{index:02d}/{len(selected_cases)}] {case.name}: retained checkpoint", flush=True)
            continue
        sentinel = execute(
            model, case, args.duration,
            measured_contact=False, contingency_shadow=False,
            contingency_execute=False,
        )
        measured = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=False,
            contingency_execute=False,
        )
        shadow = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=True,
            contingency_execute=False, preserve_primary_support=True,
            query_every_tick=False,
            flight_only=True, single_evaluation_lease=True,
            forecast_guard=True,
        )
        candidate = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=True,
            contingency_execute=True, preserve_primary_support=True,
            query_every_tick=False,
            flight_only=True, single_evaluation_lease=True,
            forecast_guard=True,
        )
        replay = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=True,
            contingency_execute=True, preserve_primary_support=True,
            query_every_tick=False,
            flight_only=True, single_evaluation_lease=True,
            forecast_guard=True,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "r137_sentinel": sentinel["metrics"],
            "measured_contact_negative_control": measured["metrics"],
            "conditional_shadow": shadow["metrics"],
            "candidate": candidate["metrics"],
            "shadow_command_exact": command_trace_equal(
                sentinel["trace"], shadow["trace"]
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }
        checkpoint.write_text(
            json.dumps(
                {"model": str(model), "duration_s": args.duration, "rows": rows},
                indent=2, sort_keys=True,
            ) + "\n"
        )
        print(
            f"[{index:02d}/{len(selected_cases)}] {case.name}: "
            f"r137={outcome(sentinel['metrics'])}, "
            f"measured={outcome(measured['metrics'])}, "
            f"candidate={outcome(candidate['metrics'])}, "
            f"selected={candidate['metrics']['contingency_selected_ticks']}",
            flush=True,
        )

    full_matrix = len(rows) == len(case_matrix())
    candidates = [row["candidate"] for row in rows.values()]
    retained_green = [
        name for name, row in rows.items() if row["r137_sentinel"]["qualified"]
    ]
    lost_green = [
        name for name in retained_green if not rows[name]["candidate"]["qualified"]
    ]
    sentinel_falls = [
        name for name, row in rows.items() if row["r137_sentinel"]["fell"]
    ]
    boundary_delta_s = {
        name: float(rows[name]["candidate"]["terminal_time_s"])
        - float(rows[name]["r137_sentinel"]["terminal_time_s"])
        for name in sentinel_falls
        if rows[name]["candidate"]["fell"]
    }
    earlier = {name: delta for name, delta in boundary_delta_s.items() if delta < -1e-12}
    later = {name: delta for name, delta in boundary_delta_s.items() if delta > 1e-12}
    recovered = [
        name for name in sentinel_falls if not rows[name]["candidate"]["fell"]
    ]
    sentinel_overruns = sum(
        row["r137_sentinel"]["loop_overruns"] for row in rows.values()
    )
    candidate_overruns = sum(row["loop_overruns"] for row in candidates)
    retained_green_metrics = [rows[name]["candidate"] for name in retained_green]
    requested_metrics = [
        row for row in candidates if row["contingency_requested_ticks"]
    ]
    wbc_query_ns = np.asarray(
        [row["contingency_wbc_step_ns_maximum"] for row in requested_metrics],
        np.float64,
    )
    forecast_query_ns = np.asarray(
        [row["contingency_forecast_step_ns_maximum"] for row in requested_metrics],
        np.float64,
    )

    def query_timing(values: np.ndarray) -> dict[str, float]:
        return {
            "minimum": float(np.min(values)),
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values)),
        }

    conditional_timing = {
        "candidate_wbc_step_ns": query_timing(wbc_query_ns),
        "forecast_guard_step_ns": query_timing(forecast_query_ns),
        "maximum_green_loop_ns": max(
            row["loop_ns"]["maximum"] for row in retained_green_metrics
        ),
    }
    mechanism_gates = {
        "matrix_complete": full_matrix,
        "shadow_preserves_r137_command_and_plant_exactly": all(
            row["shadow_command_exact"] for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "selection_exercised": any(row["contingency_selected_ticks"] for row in candidates),
        "selection_only_after_admission": all(
            row["selection_only_after_admission"] for row in candidates
        ),
        "selection_only_after_exact_support_loss": all(
            row["selection_only_after_support_loss"] for row in candidates
        ),
        "single_evaluation_lease_observed": all(
            row["contingency_requested_at_most_once"] for row in candidates
        ),
        "selection_only_in_exact_flight": all(
            row["selection_only_in_flight"] for row in candidates
        ),
        "selection_only_after_forecast_guard": all(
            row["selection_only_when_forecast_guard_passed"] for row in candidates
        ),
        "guard_acceptance_and_rejection_exercised": (
            any(row["contingency_selected_ticks"] for row in candidates)
            and any(row["contingency_guard_rejected_ticks"] for row in candidates)
        ),
        "selected_forecast_margin_at_least_0p10": all(
            row["minimum_selected_forecast_improvement"] is None
            or row["minimum_selected_forecast_improvement"] >= 0.10 - 1.0e-12
            for row in candidates
        ),
        "selected_angular_acceleration_within_40_rad_s2": all(
            row["maximum_selected_root_angular_acceleration_rad_s2"]
            <= 40.0 + 1.0e-12
            for row in candidates
        ),
        "candidate_finite": all(row["finite"] for row in candidates),
        "zero_timed_rust_allocation": all(row["allocation_free"] for row in candidates),
        "zero_python_gc": all(row["python_gc_collections"] == 0 for row in candidates),
    }
    action_promotion_gates = {
        "all_r137_green_rows_preserved": not lost_green,
        "no_r137_fall_boundary_earlier": not earlier,
        "no_numeric_fault": not any(row["numeric_fault"] for row in candidates),
        "no_incremental_total_loop_overruns": candidate_overruns <= sentinel_overruns,
        "green_rows_keep_5ms_loop_budget": all(
            row["loop_ns"]["maximum"] < 5.0e6 for row in retained_green_metrics
        ),
        "at_least_one_fall_boundary_improved": bool(later or recovered),
    }
    mechanism_passed = all(mechanism_gates.values())
    scaffold_action_promoted = mechanism_passed and all(
        action_promotion_gates.values()
    )
    controller_promotion_gates = {
        "conditioned_action_passes_preserved_primary_scaffold": (
            scaffold_action_promoted
        ),
        "generic_current_observation_authority_integrated": False,
    }
    controller_promoted = all(controller_promotion_gates.values())
    table = []
    for name, row in rows.items():
        delta = boundary_delta_s.get(name)
        table.append([
            name,
            outcome(row["r137_sentinel"]),
            outcome(row["measured_contact_negative_control"]),
            outcome(row["candidate"]),
            row["candidate"]["contingency_selected_ticks"],
            (
                "—"
                if row["candidate"]["requested_forecast_improvement"] is None
                else f"{row['candidate']['requested_forecast_improvement']:+.3f}"
            ),
            "RECOVERED" if name in recovered else "—" if delta is None else f"{delta:+.3f}",
        ])
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "scaffold_action_promoted": scaffold_action_promoted,
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "action_promotion_gates": action_promotion_gates,
        "controller_promotion_gates": controller_promotion_gates,
        "retained_green_rows": retained_green,
        "lost_green_rows": lost_green,
        "recovered_fall_rows": recovered,
        "earlier_boundaries_s": earlier,
        "later_boundaries_s": later,
        "total_selected_ticks": sum(row["contingency_selected_ticks"] for row in candidates),
        "total_requested_ticks": sum(row["contingency_requested_ticks"] for row in candidates),
        "total_guard_rejected_ticks": sum(row["contingency_guard_rejected_ticks"] for row in candidates),
        "sentinel_loop_overruns": sentinel_overruns,
        "total_loop_overruns": candidate_overruns,
        "conditional_timing": conditional_timing,
        "rows": rows,
    }
    report = "\n".join([
        "# Bonesaw guarded flight-contingency authority A/B · r179", "",
        f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · conditioned action in the preserved-primary scaffold **{'PASS' if scaffold_action_promoted else 'REJECTED'}** · plant-controller promotion **{'PASS' if controller_promoted else 'REJECTED'}**. One exact flight transition may transfer authority only when the Rust eight-knot forecast improves by at least `0.10` inside a `40 rad/s²` angular-acceleration trust region.", "",
        "## Consequence", "",
        *markdown_table(["case", "r137", "measured negative", "guarded candidate", "selected ticks", "forecast Δ", "r137 fall Δ s"], table), "",
        "## Mechanism gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in mechanism_gates.items()]), "",
        "## Conditioned-action gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in action_promotion_gates.items()]), "",
        "## Plant-controller gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in controller_promotion_gates.items()]), "",
        "## Conditional timing", "",
        *markdown_table(
            ["work", "min µs", "p50 µs", "p95 µs", "max µs"],
            [
                [
                    name,
                    f"{values['minimum'] / 1e3:.3f}",
                    f"{values['p50'] / 1e3:.3f}",
                    f"{values['p95'] / 1e3:.3f}",
                    f"{values['maximum'] / 1e3:.3f}",
                ]
                for name, values in (
                    ("candidate WBC", conditional_timing["candidate_wbc_step_ns"]),
                    ("Rust forecast guard", conditional_timing["forecast_guard_step_ns"]),
                )
            ],
        ), "",
        f"Maximum complete loop on any retained green row: `{conditional_timing['maximum_green_loop_ns'] / 1e6:.3f} ms`. Whole red-case overrun counts are `{sentinel_overruns}→{candidate_overruns}`; these include post-instability solver tails and are not substituted for the conditional query measurements.", "",
        "## Authority contract", "",
        "- The primary retains r137 double-support authority. After exact double-support arming, only the first exact mask-0 flight transition receives a candidate query; the evaluation lease is consumed whether the guard accepts or rejects it.",
        "- The R175 flight author requests exact ballistic gravity and no fictitious horizontal support force. Selection also requires typed WBC admission, hard violation below `1e-8`, at least `0.10` Rust forecast improvement over inertial continuation, and achieved root angular acceleration no greater than `40 rad/s²`.",
        "- Rejection executes the ordinary r137 fresh command. There is no blend, unilateral-support transfer, repeated opportunistic query, elapsed-time dwell, cache resurrection, or rejected-row execution.",
        "- This deliberately remains a preserved-primary scaffold. R178 contact-program authority is not enabled in this A/B, so the primary still lacks current observation authority during true flight; controller promotion remains rejected until the guarded action and bounded reacquisition are composed without regressing these results.",
    ]) + "\n"
    (destination / "upkie-guarded-flight-contingency-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_GUARDED_FLIGHT_CONTINGENCY_PLANT_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({
        "mechanism_passed": mechanism_passed,
        "scaffold_action_promoted": scaffold_action_promoted,
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "action_promotion_gates": action_promotion_gates,
        "controller_promotion_gates": controller_promotion_gates,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
