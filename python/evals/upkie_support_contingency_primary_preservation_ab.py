#!/usr/bin/env python3
"""Diagnose primary-support discontinuity ahead of contingency selection."""

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


REVISION = "upkie-support-contingency-primary-preservation-ab-r177"
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
        default="web/UPKIE_SUPPORT_CONTINGENCY_PRIMARY_PRESERVATION_AB_R177.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def physical_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    """Compare plant/command consequence while excluding observation telemetry."""
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


def run_arm(
    model: pathlib.Path,
    case: Any,
    duration: float,
    *,
    observed: bool,
    shadow: bool,
    selected: bool,
    preserve: bool,
) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration,
        measured_contact=observed,
        contingency_shadow=shadow,
        contingency_execute=selected,
        preserve_primary_support=preserve,
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
    checkpoint = destination / "upkie-support-contingency-primary-preservation-rows.json"
    rows: dict[str, Any] = {}
    if checkpoint.exists():
        retained = json.loads(checkpoint.read_text())
        if retained.get("model") == str(model) and retained.get("duration_s") == args.duration:
            rows = retained.get("rows", {})

    for index, case in enumerate(selected_cases, 1):
        if case.name in rows:
            print(f"[{index:02d}/{len(selected_cases)}] {case.name}: retained checkpoint", flush=True)
            continue
        sentinel = run_arm(
            model, case, args.duration,
            observed=False, shadow=False, selected=False, preserve=False,
        )
        measured = run_arm(
            model, case, args.duration,
            observed=True, shadow=False, selected=False, preserve=False,
        )
        preserved_shadow = run_arm(
            model, case, args.duration,
            observed=True, shadow=True, selected=False, preserve=True,
        )
        candidate = run_arm(
            model, case, args.duration,
            observed=True, shadow=True, selected=True, preserve=True,
        )
        replay = run_arm(
            model, case, args.duration,
            observed=True, shadow=True, selected=True, preserve=True,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "r137_sentinel": sentinel["metrics"],
            "measured_contact_control": measured["metrics"],
            "preserved_primary_shadow": preserved_shadow["metrics"],
            "candidate": candidate["metrics"],
            "preserved_shadow_physical_exact": physical_equal(
                sentinel["trace"], preserved_shadow["trace"]
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
            f"measured={outcome(measured['metrics'])}, "
            f"candidate={outcome(candidate['metrics'])}, "
            f"selected={candidate['metrics']['contingency_selected_ticks']}",
            flush=True,
        )

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
    earlier_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta < -1.0e-12
    }
    later_boundaries = {
        name: delta for name, delta in boundary_delta_s.items() if delta > 1.0e-12
    }
    mechanism_gates = {
        "matrix_complete": len(rows) == len(case_matrix()),
        "preserved_primary_shadow_matches_r137_physics": all(
            row["preserved_shadow_physical_exact"] for row in rows.values()
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
        "primary_support_has_current_observation_authority": False,
        "retained_green_rows_preserved": not lost_green,
        "no_r137_fall_boundary_earlier": not earlier_boundaries,
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
                outcome(row["preserved_primary_shadow"]),
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
        "lost_retained_green_rows": lost_green,
        "earlier_boundaries_s": earlier_boundaries,
        "later_boundaries_s": later_boundaries,
        "total_selected_ticks": sum(row["contingency_selected_ticks"] for row in candidates),
        "total_r137_loop_overruns": sum(
            row["r137_sentinel"]["loop_overruns"] for row in rows.values()
        ),
        "total_preserved_shadow_loop_overruns": sum(
            row["preserved_primary_shadow"]["loop_overruns"] for row in rows.values()
        ),
        "total_candidate_loop_overruns": sum(row["loop_overruns"] for row in candidates),
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw primary-support preservation diagnostic · r177",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · controller promotion **{'PASS' if controller_promoted else 'REJECTED'}**. This diagnostic isolates the primary-support discontinuity found by r176; preservation is an experiment, not authority to ignore observed support.",
            "",
            "## Consequence",
            "",
            *markdown_table(
                ["case", "r137", "measured", "preserved shadow", "selected", "selected ticks", "fall Δ vs r137 s"],
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
            "## Interpretation",
            "",
            "- The r139 filter intentionally begins with no hard-contact authority and needs three positive samples to activate contact. R176 let the primary WBC consume that startup/reacquisition state, which changed torque before the contingency selector was armed.",
            "- This arm keeps the established double-support primary solve physically identical to r137 while the exact observed mask remains available only to the independent contingency query and selector. Exact equality is checked on plant state, torque, status, contact count, fall-safe state, and terminal consequence; observation telemetry is deliberately excluded.",
            f"- The zero-overrun gate is not a candidate-only regression: retained r137, preserved shadow, and selected candidate record {sum(row['r137_sentinel']['loop_overruns'] for row in rows.values())}, {sum(row['preserved_primary_shadow']['loop_overruns'] for row in rows.values())}, and {sum(row['candidate']['loop_overruns'] for row in rows.values())} misses respectively. Candidate termination shortens several red traces, so these totals are not normalized speed ratios.",
            "- Preserving a fictitious primary support row is not itself promotable. If it restores the envelope, the follow-up must replace it with an explicit enable/reacquisition state and bounded retained-command authority, then repeat the same plant gates with delay/noise/dropout.",
        ]
    ) + "\n"
    (destination / "upkie-support-contingency-primary-preservation-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_SUPPORT_CONTINGENCY_PRIMARY_PRESERVATION_AUDIT.md").write_text(report)
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
