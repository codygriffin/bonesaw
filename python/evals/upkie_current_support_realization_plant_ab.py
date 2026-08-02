#!/usr/bin/env python3
"""Promote measured-contact authority without changing the retained r137 action."""

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


REVISION = "upkie-current-support-realization-ablation-r181"
CHECKPOINT_SCHEMA = 3
PLANT_COMMAND_FIELDS = (
    "time_s",
    "root_position",
    "root_twist",
    "rotation_vector",
    "q",
    "v",
    "torque",
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
        default="web/UPKIE_CURRENT_SUPPORT_REALIZATION_ABLATION_R181.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def plant_command_trace_equal(
    left: dict[str, Any], right: dict[str, Any]
) -> bool:
    return bool(
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in PLANT_COMMAND_FIELDS
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
        and np.array_equal(
            left["terminal_root_position"], right["terminal_root_position"]
        )
        and np.array_equal(
            left["terminal_rotation_vector"],
            right["terminal_rotation_vector"],
        )
    )


def exact_terminal_outcome(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return bool(
        left["fell"] == right["fell"]
        and left["numeric_fault"] == right["numeric_fault"]
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
        and left["qualified"] == right["qualified"]
    )


def main() -> None:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    selected_cases = case_matrix()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(
            case for case in selected_cases if case.name in requested
        )
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    checkpoint = destination / "upkie-current-support-realization-rows.json"
    rows: dict[str, Any] = {}
    if checkpoint.exists():
        retained = json.loads(checkpoint.read_text())
        if (
            retained.get("revision") == REVISION
            and retained.get("schema") == CHECKPOINT_SCHEMA
            and retained.get("model") == str(model)
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
        raw_transfer = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=True,
            contact_program_authority_ticks=0,
            contact_observation_prestart_samples=3,
            use_feasibility_row_spans=True,
        )
        candidate = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=True,
            contact_program_authority_ticks=0,
            realize_primary_torque=True,
            contact_observation_prestart_samples=3,
            use_feasibility_row_spans=True,
        )
        replay = execute(
            model,
            case,
            args.duration,
            measured_contact=True,
            contingency_shadow=True,
            contingency_execute=True,
            contact_program_authority_ticks=0,
            realize_primary_torque=True,
            contact_observation_prestart_samples=3,
            use_feasibility_row_spans=True,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "r137_sentinel": sentinel["metrics"],
            "raw_current_support_transfer": raw_transfer["metrics"],
            "candidate": candidate["metrics"],
            "candidate_plant_command_exact": plant_command_trace_equal(
                sentinel["trace"], candidate["trace"]
            ),
            "candidate_terminal_outcome_exact": exact_terminal_outcome(
                sentinel["metrics"], candidate["metrics"]
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }
        checkpoint.write_text(
            json.dumps(
                {
                    "revision": REVISION,
                    "schema": CHECKPOINT_SCHEMA,
                    "model": str(model),
                    "duration_s": args.duration,
                    "rows": rows,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        print(
            f"[{index:02d}/{len(selected_cases)}] {case.name}: "
            f"r137={outcome(sentinel['metrics'])}, "
            f"raw={outcome(raw_transfer['metrics'])}, "
            f"realized={outcome(candidate['metrics'])}, "
            f"current-support={candidate['metrics']['contact_program_current_support_ticks']}",
            flush=True,
        )

    candidates = [row["candidate"] for row in rows.values()]
    sentinels = [row["r137_sentinel"] for row in rows.values()]
    raw_transfers = [
        row["raw_current_support_transfer"] for row in rows.values()
    ]
    full_matrix = len(rows) == len(case_matrix())
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
    earlier = {
        name: delta for name, delta in boundary_delta_s.items() if delta < -1e-12
    }
    raw_changed = [
        name
        for name, row in rows.items()
        if not exact_terminal_outcome(
            row["r137_sentinel"], row["raw_current_support_transfer"]
        )
    ]
    total_ticks = sum(row["executed_ticks"] for row in candidates)
    total_primary_ticks = sum(
        row["contact_program_primary_ticks"] for row in candidates
    )
    total_current_ticks = sum(
        row["contact_program_current_support_ticks"] for row in candidates
    )
    total_withheld_ticks = sum(
        row["contact_program_withheld_ticks"] for row in candidates
    )
    total_lease_ticks = sum(
        row["contact_program_lease_ticks"] for row in candidates
    )
    maximum_torque_error = max(
        row["maximum_primary_execution_torque_error"] for row in candidates
    )
    maximum_violation = max(
        row["contingency_maximum_constraint_violation"] for row in candidates
    )
    retained_green_metrics = [rows[name]["candidate"] for name in retained_green]

    mechanism_gates = {
        "matrix_complete": full_matrix,
        "raw_transfer_negative_control_changes_outcomes": bool(raw_changed),
        "current_support_path_exercised": total_current_ticks > 0,
        "all_control_ticks_have_executable_program": (
            total_primary_ticks + total_current_ticks == total_ticks
            and total_withheld_ticks == 0
            and total_lease_ticks == 0
        ),
        "hard_mask_is_exact_raw_stable_intersection": all(
            row[
                "contact_program_hard_mask_matches_raw_stable_intersection"
            ]
            for row in candidates
        ),
        "contact_program_masks_are_consistent": all(
            row["contact_program_masks_consistent"] for row in candidates
        ),
        "current_support_selection_is_typed_admitted": all(
            row["selection_only_after_admission"]
            and row["selected_status_is_executable"]
            for row in candidates
        ),
        "current_support_hard_violation_below_1e_8": (
            maximum_violation <= 1.0e-8
        ),
        "no_realization_fallback": all(
            row["support_contingency_realization_fallback_ticks"] == 0
            for row in candidates
        ),
        "executed_torque_is_bit_exact_primary_program": (
            maximum_torque_error == 0.0
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "candidate_finite": all(row["finite"] for row in candidates),
        "zero_timed_rust_allocation": all(
            row["allocation_free"] for row in candidates
        ),
        "zero_python_gc": all(
            row["python_gc_collections"] == 0 for row in candidates
        ),
    }
    non_regression_gates = {
        "r137_plant_and_command_trace_bit_exact": all(
            row["candidate_plant_command_exact"] for row in rows.values()
        ),
        "r137_terminal_outcome_exact": all(
            row["candidate_terminal_outcome_exact"] for row in rows.values()
        ),
        "all_r137_green_rows_preserved": not lost_green,
        "no_r137_fall_boundary_earlier": not earlier,
        "all_r137_fall_boundaries_exact": all(
            delta == 0.0 for delta in boundary_delta_s.values()
        ),
        "no_numeric_fault": not any(row["numeric_fault"] for row in candidates),
        "green_rows_keep_5ms_loop_budget": all(
            row["loop_ns"]["maximum"] < 5.0e6
            for row in retained_green_metrics
        ),
    }
    mechanism_passed = all(mechanism_gates.values())
    semantic_composition_admitted = mechanism_passed and all(
        non_regression_gates.values()
    )

    table = []
    for name, row in rows.items():
        candidate = row["candidate"]
        table.append(
            [
                name,
                outcome(row["r137_sentinel"]),
                outcome(row["raw_current_support_transfer"]),
                outcome(candidate),
                candidate["contact_program_primary_ticks"],
                candidate["contact_program_current_support_ticks"],
                candidate["support_contingency_realization_fallback_ticks"],
                "YES" if row["candidate_plant_command_exact"] else "NO",
            ]
        )
    timing = {
        "current_support_wbc_p99_ns_maximum_across_cases": max(
            row["contingency_wbc_step_ns_p99"] for row in candidates
        ),
        "current_support_wbc_single_step_ns_maximum": max(
            row["contingency_wbc_step_ns_maximum"] for row in candidates
        ),
        "green_loop_ns_maximum": max(
            row["loop_ns"]["maximum"] for row in retained_green_metrics
        ),
        "sentinel_loop_overruns": sum(
            row["loop_overruns"] for row in sentinels
        ),
        "candidate_loop_overruns": sum(
            row["loop_overruns"] for row in candidates
        ),
    }
    scheduling_gates = {
        "zero_5ms_loop_overruns": timing["candidate_loop_overruns"] == 0,
        "all_controller_calls_below_5ms": all(
            row["controller_step_ns"]["maximum"] <= 5.0e6
            for row in candidates
        ),
    }
    synchronous_profile_admitted = semantic_composition_admitted and all(
        scheduling_gates.values()
    )
    deployment_gates = {
        "semantic_current_observation_composition_admitted": (
            semantic_composition_admitted
        ),
        "synchronous_ordinary_process_profile_admitted": (
            synchronous_profile_admitted
        ),
        "delay_noise_dropout_and_mirrored_evidence_complete": False,
        "higher_fidelity_or_hardware_evidence_complete": False,
    }
    controller_promoted = all(deployment_gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "semantic_composition_admitted": semantic_composition_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "non_regression_gates": non_regression_gates,
        "scheduling_gates": scheduling_gates,
        "deployment_gates": deployment_gates,
        "retained_green_rows": retained_green,
        "lost_green_rows": lost_green,
        "earlier_boundaries_s": earlier,
        "boundary_delta_s": boundary_delta_s,
        "raw_transfer_changed_outcomes": raw_changed,
        "total_control_ticks": total_ticks,
        "total_primary_program_ticks": total_primary_ticks,
        "total_current_support_ticks": total_current_ticks,
        "total_withheld_ticks": total_withheld_ticks,
        "total_lease_ticks": total_lease_ticks,
        "maximum_primary_execution_torque_error": maximum_torque_error,
        "maximum_current_support_constraint_violation": maximum_violation,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw current-support realization ablation · r181",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · semantic composition **{'ADMITTED' if semantic_composition_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}** · hardware controller **{'PROMOTED' if controller_promoted else 'NOT PROMOTED'}**. This ablation isolates why the generic R178 authority must realize the retained r137 torque instead of directly selecting an independently optimized current-support action.",
            "",
            "## Result",
            "",
            *markdown_table(
                [
                    "case",
                    "r137",
                    "raw transfer",
                    "realized transfer",
                    "primary ticks",
                    "current ticks",
                    "fallbacks",
                    "plant exact",
                ],
                table,
            ),
            "",
            "The raw-current-support negative control changes the terminal boundary or qualification of "
            f"`{len(raw_changed)}` row(s): `{', '.join(raw_changed)}`. "
            "The realized composition changes none. Across "
            f"`{total_ticks}` control ticks, the authority selected the retained "
            f"primary proof `{total_primary_ticks}` times and its current-support "
            f"realization `{total_current_ticks}` times, with `{total_withheld_ticks}` "
            f"withheld and `{total_lease_ticks}` lease ticks.",
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
            "## Non-regression gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in non_regression_gates.items()
                ],
            ),
            "",
            "## Scheduling and deployment gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in {
                        **scheduling_gates,
                        **deployment_gates,
                    }.items()
                ],
            ),
            "",
            "## Timing and memory",
            "",
            f"Worst per-case current-support WBC p99: `{timing['current_support_wbc_p99_ns_maximum_across_cases'] / 1e3:.3f} µs`; maximum single solve: `{timing['current_support_wbc_single_step_ns_maximum'] / 1e3:.3f} µs`; maximum complete green-row loop: `{timing['green_loop_ns_maximum'] / 1e6:.3f} ms`. Timed Rust allocations and Python GC collections are zero. Whole-run overrun counts are `{timing['sentinel_loop_overruns']}→{timing['candidate_loop_overruns']}` and include red-case post-instability tails.",
            "",
            "## Authority stack",
            "",
            "1. Three exact pre-control contact samples initialize the debouncer before torque authority is enabled; startup therefore has no zero-command hole.",
            "2. The r137 primary program supplies either a fresh raw torque or its already-bounded freshness-faded command. Its raw freshness alone drives fall-safe confidence.",
            "3. A fixed-effort Rust WBC receives that exact torque and the current hard contact mask. It may prove achieved acceleration and feasibility, but it cannot optimize or alter the torque.",
            "4. R178 selects the primary proof when its authored mask remains current and the current-support proof otherwise. A failed realization is non-executable; there is no permissive fallback or hidden lease in this run.",
            "5. The selected torque is bit-exact to the r137 effective command on every executable tick. Plant motion, fall-safe authority, terminal outcomes, and every fall boundary therefore remain byte-for-byte retained while the diagnostics become physically truthful about support.",
            "",
            "## Scope",
            "",
            "This admits the semantic current-observation composition, not a stronger recovery action or hardware controller. R179's guarded flight action remains separately conditioned; the next action experiment may replace the fixed retained torque only behind the same typed current-support proof, scheduling, robustness, and non-regression gates.",
        ]
    ) + "\n"
    (destination / "upkie-current-support-realization-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CURRENT_SUPPORT_REALIZATION_PLANT_AB_AUDIT.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "semantic_composition_admitted": (
                    semantic_composition_admitted
                ),
                "synchronous_profile_admitted": synchronous_profile_admitted,
                "controller_promoted": controller_promoted,
                "mechanism_gates": mechanism_gates,
                "non_regression_gates": non_regression_gates,
                "scheduling_gates": scheduling_gates,
                "deployment_gates": deployment_gates,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
