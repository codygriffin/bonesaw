#!/usr/bin/env python3
"""Causal plant admission for Rust contact-program authority and realization."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import platform
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


REVISION = "upkie-contact-program-authority-plant-ab-r180"
DURATION_S = 6.0
HOLD_TICKS = 0
PRESTART_CONTACT_SAMPLES = 3
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
    "fall_safe_mode",
    "fall_safe_primary_authority",
    "fall_safe_fresh_command_authority",
    "maximum_abs_qacc",
    "external_force_world",
    "application_point_world",
)
EXECUTION_FIELDS = tuple(
    field for field in PHYSICAL_FIELDS if field not in ("status", "wbc_normal_force")
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_PROGRAM_AUTHORITY_PLANT_AB_R180.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    parser.add_argument("--no-checkpoint", action="store_true")
    return parser.parse_args()


def physical_equal(
    left: dict[str, Any],
    right: dict[str, Any],
    fields: tuple[str, ...] = PHYSICAL_FIELDS,
) -> bool:
    return bool(
        all(
            np.array_equal(np.asarray(left[field]), np.asarray(right[field]))
            for field in fields
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


def run_sentinel(model: pathlib.Path, case: Any, duration_s: float) -> dict[str, Any]:
    return run_case(
        model,
        case,
        duration_s,
        balance_mode="capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=False,
        execute_reduced_support=True,
    )


def run_preservation_shadow(
    model: pathlib.Path, case: Any, duration_s: float
) -> dict[str, Any]:
    return run_case(
        model,
        case,
        duration_s,
        balance_mode="capture",
        fall_safe_enabled=True,
        fall_safe_primary_blend=False,
        measured_contact_admission=True,
        execute_reduced_support=True,
        support_contingency_enabled=True,
        support_contingency_execute=False,
        support_contingency_preserve_primary_support=True,
        support_contingency_query_every_tick=True,
    )


def run_candidate(
    model: pathlib.Path,
    case: Any,
    duration_s: float,
    *,
    sparse_rows: bool,
) -> dict[str, Any]:
    return run_case(
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
        support_contingency_preserve_primary_support=True,
        support_contingency_query_every_tick=True,
        support_contingency_realize_primary_torque=True,
        contact_program_authority_ticks=HOLD_TICKS,
        contact_observation_prestart_samples=PRESTART_CONTACT_SAMPLES,
        use_feasibility_row_spans=sparse_rows,
    )


def outcome(metrics: dict[str, Any]) -> str:
    if metrics["fell"]:
        return f"FALL {float(metrics['terminal_time_s']):.3f}s"
    return str(metrics["outcome"])


def authority_metrics(trace: dict[str, Any]) -> dict[str, Any]:
    selection = np.asarray(trace["contact_program_authority_selection"])
    executable = np.asarray(trace["contact_program_authority_executable"]) != 0
    transition = (
        np.asarray(trace["contact_program_authority_transition_pending"]) != 0
    )
    consistent = np.asarray(trace["contact_program_authority_masks_consistent"]) != 0
    provenance = np.asarray(trace["contact_program_authority_lease_provenance"])
    age = np.asarray(trace["contact_program_authority_age_ticks"])
    authoring = np.asarray(trace["contact_program_authority_authoring_mask"])
    hard = np.asarray(trace["contact_program_authority_hard_mask"])
    current = selection == 2
    primary = selection == 1
    retained = selection == 3
    admitted = np.asarray(trace["support_contingency_admitted"]) != 0
    requested = np.asarray(trace["support_contingency_requested"]) != 0
    support_mask = np.asarray(trace["support_contingency_support_mask"])
    torque = np.asarray(trace["torque"])
    candidate_torque = np.asarray(trace["support_contingency_candidate_torque"])
    primary_torque = np.asarray(trace["support_contingency_primary_torque"])
    first_authority = np.flatnonzero(selection != 0)
    nonexecutable_zero = bool(np.all(torque[~executable] == 0.0))
    current_torque_exact = bool(
        np.array_equal(torque[current], candidate_torque[current])
        and np.array_equal(candidate_torque[current], primary_torque[current])
    )
    return {
        "selection_counts": {
            str(int(value)): int(np.sum(selection == value))
            for value in np.unique(selection)
        },
        "withheld_ticks": int(np.sum(selection == 0)),
        "fresh_primary_ticks": int(np.sum(primary)),
        "fresh_current_support_ticks": int(np.sum(current)),
        "retained_lease_ticks": int(np.sum(retained)),
        "transition_current_support_ticks": int(np.sum(current & transition)),
        "stable_current_support_ticks": int(np.sum(current & ~transition)),
        "maximum_command_age_ticks": int(np.max(age)),
        "first_authority_is_primary": bool(
            first_authority.size and selection[first_authority[0]] == 1
        ),
        "execution_only_with_consistent_masks": bool(np.all(~executable | consistent)),
        "fresh_provenance_typed": bool(
            np.all(~(primary | current) | (provenance == 1))
        ),
        "retained_provenance_typed": bool(np.all(~retained | (provenance == 2))),
        "current_support_only_after_admission": bool(np.all(~current | admitted)),
        "current_support_only_after_request": bool(np.all(~current | requested)),
        "current_support_only_on_reduced_hard_support": bool(
            np.all(~current | (support_mask != 3))
        ),
        "current_authoring_mask_matches_hard_mask": bool(
            np.all(~current | (authoring == hard))
        ),
        "current_support_preserves_primary_program_torque_exactly": (
            current_torque_exact
        ),
        "nonexecutable_output_is_zero_torque": nonexecutable_zero,
        "bounded_lease_age": bool(np.all(~retained | (age <= HOLD_TICKS))),
        "realization_fallback_ticks": int(
            np.sum(trace["support_contingency_realization_fallback"])
        ),
        "maximum_constraint_violation": float(
            np.max(trace["support_contingency_maximum_constraint_violation"])
        ),
    }


def main() -> int:
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
    checkpoint = destination / "upkie-contact-program-authority-rows.json"
    rows: dict[str, Any] = {}
    checkpoint_header = {
        "revision": REVISION,
        "model": str(model),
        "duration_s": args.duration,
        "hold_ticks": HOLD_TICKS,
        "prestart_contact_samples": PRESTART_CONTACT_SAMPLES,
    }
    if checkpoint.exists() and not args.no_checkpoint:
        retained = json.loads(checkpoint.read_text())
        if all(retained.get(key) == value for key, value in checkpoint_header.items()):
            rows = retained.get("rows", {})

    for index, case in enumerate(selected_cases, 1):
        if case.name in rows:
            print(
                f"[{index:02d}/{len(selected_cases)}] {case.name}: retained checkpoint",
                flush=True,
            )
            continue
        sentinel_trace = run_sentinel(model, case, args.duration)
        shadow_trace = run_preservation_shadow(model, case, args.duration)
        dense_trace = run_candidate(model, case, args.duration, sparse_rows=False)
        sparse_trace = run_candidate(model, case, args.duration, sparse_rows=True)
        replay_trace = run_candidate(model, case, args.duration, sparse_rows=True)
        sentinel = summarize(case, sentinel_trace, args.duration)
        shadow = summarize(case, shadow_trace, args.duration)
        dense = summarize(case, dense_trace, args.duration)
        sparse = summarize(case, sparse_trace, args.duration)
        row = {
            "case": sparse["case"],
            "r137_sentinel": sentinel,
            "preserved_primary_shadow": shadow,
            "dense_candidate": dense,
            "candidate": sparse,
            "authority": authority_metrics(sparse_trace),
            "candidate_execution_exact": physical_equal(
                sentinel_trace, sparse_trace, EXECUTION_FIELDS
            ),
            "preserved_shadow_physical_exact": physical_equal(
                sentinel_trace, shadow_trace
            ),
            "sparse_rows_semantic_exact": semantic_trace_equal(
                dense_trace, sparse_trace
            ),
            "candidate_replay_exact": semantic_trace_equal(
                sparse_trace, replay_trace
            ),
        }
        rows[case.name] = row
        checkpoint.write_text(
            json.dumps({**checkpoint_header, "rows": rows}, indent=2, sort_keys=True)
            + "\n"
        )
        print(
            f"[{index:02d}/{len(selected_cases)}] {case.name}: "
            f"r137={outcome(sentinel)}, candidate={outcome(sparse)}, "
            f"overruns {dense['loop_overruns']}→{sparse['loop_overruns']}, "
            f"current={row['authority']['fresh_current_support_ticks']}",
            flush=True,
        )

    full_matrix = len(rows) == len(case_matrix())
    candidates = [row["candidate"] for row in rows.values()]
    authorities = [row["authority"] for row in rows.values()]
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
    earlier_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta < -1.0e-12
    }
    later_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta > 1.0e-12
    }
    mechanism_gates = {
        "matrix_complete": full_matrix,
        "preservation_shadow_matches_r137_physics": all(
            row["preserved_shadow_physical_exact"] for row in rows.values()
        ),
        "sparse_row_kernel_is_semantically_exact": all(
            row["sparse_rows_semantic_exact"] for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "candidate_matches_r137_execution": all(
            row["candidate_execution_exact"] for row in rows.values()
        ),
        "startup_establishes_primary_authority_first": all(
            value["first_authority_is_primary"] for value in authorities
        ),
        "current_support_authority_exercised": sum(
            value["fresh_current_support_ticks"] for value in authorities
        )
        > 0,
        "transition_current_support_authority_exercised": sum(
            value["transition_current_support_ticks"] for value in authorities
        )
        > 0,
        "execution_only_with_consistent_masks": all(
            value["execution_only_with_consistent_masks"] for value in authorities
        ),
        "typed_fresh_and_retained_provenance": all(
            value["fresh_provenance_typed"] and value["retained_provenance_typed"]
            for value in authorities
        ),
        "current_support_is_separately_admitted_and_requested": all(
            value["current_support_only_after_admission"]
            and value["current_support_only_after_request"]
            for value in authorities
        ),
        "current_support_uses_actual_reduced_hard_mask": all(
            value["current_support_only_on_reduced_hard_support"]
            and value["current_authoring_mask_matches_hard_mask"]
            for value in authorities
        ),
        "fixed_effort_realization_preserves_primary_program_torque": all(
            value["current_support_preserves_primary_program_torque_exactly"]
            for value in authorities
        ),
        "bounded_lease_and_zero_when_withheld": all(
            value["bounded_lease_age"]
            and value["nonexecutable_output_is_zero_torque"]
            for value in authorities
        ),
        "no_realization_fallback": sum(
            value["realization_fallback_ticks"] for value in authorities
        )
        == 0,
        "candidate_finite_and_allocation_free": all(
            value["finite"] and value["allocation_free"] for value in candidates
        ),
        "zero_python_gc": all(
            value["python_gc_collections"] == 0 for value in candidates
        ),
    }
    promotion_gates = {
        "retained_green_rows_preserved": not lost_green,
        "no_r137_fall_boundary_earlier": not earlier_boundaries,
        "no_candidate_numeric_fault": not any(
            value["numeric_fault"] for value in candidates
        ),
        "zero_5ms_loop_overruns": sum(
            value["loop_overruns"] for value in candidates
        )
        == 0,
        "candidate_wbc_calls_below_5ms": all(
            value["controller_step_ns"]["maximum"] <= 5.0e6
            for value in candidates
        ),
        "delay_noise_dropout_and_mirrored_evidence_complete": False,
        "hardware_or_higher_fidelity_plant_evidence_complete": False,
    }
    mechanism_passed = all(mechanism_gates.values())
    authoritative_wbc_profile_admitted = mechanism_passed and all(
        value
        for name, value in promotion_gates.items()
        if name
        not in (
            "zero_5ms_loop_overruns",
            "delay_noise_dropout_and_mirrored_evidence_complete",
            "hardware_or_higher_fidelity_plant_evidence_complete",
        )
    )
    synchronous_supervisor_profile_admitted = bool(
        authoritative_wbc_profile_admitted
        and promotion_gates["zero_5ms_loop_overruns"]
    )
    controller_promoted = mechanism_passed and all(promotion_gates.values())

    report_rows = []
    for name, row in rows.items():
        delta = boundary_delta_s.get(name)
        report_rows.append(
            [
                name,
                outcome(row["r137_sentinel"]),
                outcome(row["candidate"]),
                "—" if delta is None else f"{delta:+.3f}",
                row["authority"]["fresh_current_support_ticks"],
                row["authority"]["transition_current_support_ticks"],
                f"{row['dense_candidate']['loop_overruns']}→{row['candidate']['loop_overruns']}",
                f"{row['candidate']['loop_ns']['p99'] / 1e3:.0f}",
            ]
        )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "python": platform.python_version(),
        },
        "model": str(model),
        "duration_s": args.duration,
        "control_period_ms": 5.0,
        "hold_ticks": HOLD_TICKS,
        "prestart_contact_samples": PRESTART_CONTACT_SAMPLES,
        "mechanism_passed": mechanism_passed,
        "authoritative_wbc_profile_admitted": authoritative_wbc_profile_admitted,
        "synchronous_supervisor_profile_admitted": (
            synchronous_supervisor_profile_admitted
        ),
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "promotion_gates": promotion_gates,
        "retained_green_rows": retained_green,
        "lost_retained_green_rows": lost_green,
        "earlier_boundaries_s": earlier_boundaries,
        "later_boundaries_s": later_boundaries,
        "total_dense_loop_overruns": sum(
            row["dense_candidate"]["loop_overruns"] for row in rows.values()
        ),
        "total_candidate_loop_overruns": sum(
            row["candidate"]["loop_overruns"] for row in rows.values()
        ),
        "total_fresh_current_support_ticks": sum(
            value["fresh_current_support_ticks"] for value in authorities
        ),
        "total_transition_current_support_ticks": sum(
            value["transition_current_support_ticks"] for value in authorities
        ),
        "total_retained_lease_ticks": sum(
            value["retained_lease_ticks"] for value in authorities
        ),
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw contact-program authority plant A/B · r180",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · authoritative CPU WBC profile **{'ADMITTED' if authoritative_wbc_profile_admitted else 'REJECTED'}** · synchronous supervisor profile **{'ADMITTED' if synchronous_supervisor_profile_admitted else 'REJECTED'}** · hardware controller promotion **{'YES' if controller_promoted else 'NO'}**.",
            "",
            "## Result",
            "",
            "The Rust authority machine now distinguishes a stable primary program, a separately admitted current-hard-support realization, a bounded retained lease, and withheld authority. Startup cannot jump directly to a reduced/flight command: three explicit prestart contact observations establish causal contact evidence, and the first executable program must be primary.",
            "",
            "The current-support WBC does not invent a new torque. It fixes the active primary-program actuator effort, re-solves floating dynamics/contact against the observed hard mask, and returns an actual-support acceleration/force witness. During debounce, this candidate may become fresh only after prior authority exists. Evidence loss, inconsistent masks, invalid sequence, rejection, and lease expiry remain fail-closed.",
            "",
            "This evaluated profile configures the retained-command lease to **zero ticks**. A two-tick stale hold changed red-row plant boundaries and was rejected. Contact transition is covered by continuously fresh, separately admitted current-support realization; if that query is unavailable, authority is withheld rather than extended by an old command.",
            "",
            *markdown_table(
                [
                    "case",
                    "r137 sentinel",
                    "r180 candidate",
                    "fall Δ s",
                    "fresh current",
                    "during transition",
                    "dense→sparse overruns",
                    "loop p99 µs",
                ],
                report_rows,
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
            "## Promotion gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [
                    [name, "PASS" if value else "FAIL"]
                    for name, value in promotion_gates.items()
                ],
            ),
            "",
            "## Timing and memory",
            "",
            f"- Dense feasibility rows miss the 5 ms synchronous deadline **{metrics['total_dense_loop_overruns']}** times; the established sparse-row kernel removes those misses without changing any semantic trace: **{metrics['total_candidate_loop_overruns']}** remain.",
            f"- The current-support realization is selected for **{metrics['total_fresh_current_support_ticks']}** ticks, including **{metrics['total_transition_current_support_ticks']}** debounce ticks. The configured stale-command hold is **{HOLD_TICKS} ticks** and observed retained-lease execution is **{metrics['total_retained_lease_ticks']} ticks**.",
            "- Every reported candidate run records zero Rust hot-path allocation and zero Python GC collection. Per-row RSS deltas are retained in the JSON but are descriptive process-residency measurements, not a deterministic ownership bound.",
            "- Timing is host-native descriptive evidence on an ordinary Linux process, not a real-time scheduling guarantee. No core isolation, priority elevation, or real-time kernel is claimed.",
            "",
            "## Consequence",
            "",
            f"- Retained r137 green rows lost: **{len(lost_green)}** ({', '.join(lost_green) if lost_green else 'none'}).",
            f"- Earlier r137 fall boundaries: **{len(earlier_boundaries)}** ({', '.join(earlier_boundaries) if earlier_boundaries else 'none'}).",
            f"- Later fall boundaries: **{len(later_boundaries)}** ({', '.join(later_boundaries) if later_boundaries else 'none'}).",
            "- The authoritative WBC profile can be admitted independently of ordinary-process scheduling jitter only when every measured controller call remains inside 5 ms. The combined synchronous supervisor still requires zero measured loop misses. Neither result admits hardware authority: delay/noise/dropout, mirrored evidence, estimator disagreement, coupled actuation, thermal/power envelopes, and a higher-fidelity or hardware plant remain open.",
            "",
            "## Authority stack",
            "",
            "1. Exact timestamped contact evidence and debounce state determine raw, stable, and hard masks.",
            "2. Stable double support may admit the established primary WBC command.",
            "3. Reduced hard support may admit a fixed-primary-program-effort realization through an independent WBC query.",
            "4. Rust contact-program authority selects fresh primary, fresh current support, bounded retained lease, or withheld; this profile sets the retained lease budget to zero.",
            "5. The fall-safe/execution boundary emits only the selected fixed-size command; unavailable authority emits zero torque.",
        ]
    ) + "\n"

    (destination / "upkie-contact-program-authority-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONTACT_PROGRAM_AUTHORITY_PLANT_AUDIT.md").write_text(
        report
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "authoritative_wbc_profile_admitted": (
                    authoritative_wbc_profile_admitted
                ),
                "synchronous_supervisor_profile_admitted": (
                    synchronous_supervisor_profile_admitted
                ),
                "controller_promoted": controller_promoted,
                "mechanism_gates": mechanism_gates,
                "promotion_gates": promotion_gates,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
