#!/usr/bin/env python3
"""Keep r137 primary support until the independently admitted action owns execution."""

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


REVISION = "upkie-conditional-support-contingency-plant-ab-r177"
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
        default="web/UPKIE_CONDITIONAL_SUPPORT_CONTINGENCY_PLANT_AB_R177.html",
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
    checkpoint = destination / "upkie-conditional-support-contingency-rows.json"
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
        )
        candidate = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=True,
            contingency_execute=True, preserve_primary_support=True,
            query_every_tick=False,
        )
        replay = execute(
            model, case, args.duration,
            measured_contact=True, contingency_shadow=True,
            contingency_execute=True, preserve_primary_support=True,
            query_every_tick=False,
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
        "candidate_finite": all(row["finite"] for row in candidates),
        "zero_timed_rust_allocation": all(row["allocation_free"] for row in candidates),
        "zero_python_gc": all(row["python_gc_collections"] == 0 for row in candidates),
    }
    promotion_gates = {
        "all_r137_green_rows_preserved": not lost_green,
        "no_r137_fall_boundary_earlier": not earlier,
        "no_numeric_fault": not any(row["numeric_fault"] for row in candidates),
        "zero_5ms_loop_overruns": sum(row["loop_overruns"] for row in candidates) == 0,
    }
    mechanism_passed = all(mechanism_gates.values())
    controller_promoted = mechanism_passed and all(promotion_gates.values())
    table = []
    for name, row in rows.items():
        delta = boundary_delta_s.get(name)
        table.append([
            name,
            outcome(row["r137_sentinel"]),
            outcome(row["measured_contact_negative_control"]),
            outcome(row["candidate"]),
            row["candidate"]["contingency_selected_ticks"],
            "RECOVERED" if name in recovered else "—" if delta is None else f"{delta:+.3f}",
        ])
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
        "lost_green_rows": lost_green,
        "recovered_fall_rows": recovered,
        "earlier_boundaries_s": earlier,
        "later_boundaries_s": later,
        "total_selected_ticks": sum(row["contingency_selected_ticks"] for row in candidates),
        "total_loop_overruns": sum(row["loop_overruns"] for row in candidates),
        "rows": rows,
    }
    report = "\n".join([
        "# Bonesaw conditional primary/contingency authority A/B · r177", "",
        f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · controller promotion **{'PASS' if controller_promoted else 'REJECTED'}**. Primary r137 contact authority remains unchanged until the separately admitted R175 action owns the command.", "",
        "## Consequence", "",
        *markdown_table(["case", "r137", "measured negative", "conditional candidate", "selected ticks", "r137 fall Δ s"], table), "",
        "## Mechanism gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in mechanism_gates.items()]), "",
        "## Promotion gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in promotion_gates.items()]), "",
        "## Authority contract", "",
        "- Exact measured support is consumed by the non-executing R175 author/WBC. The r137 primary WBC retains its established hard-support program until the admitted contingency is selected; a rejected candidate falls back to ordinary r137 freshness rather than a measured-contact primary command.",
        "- The shadow must preserve r137 torque and plant state bit-for-bit. Selection still requires exact double-support arming, later exact support loss, typed candidate admission, and hard violation below `1e-8`.",
        "- No blend, elapsed-time dwell, command-lifetime extension, cache resurrection, rejected-row execution, or reset participates.",
    ]) + "\n"
    (destination / "upkie-conditional-support-contingency-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_CONDITIONAL_SUPPORT_CONTINGENCY_PLANT_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({
        "mechanism_passed": mechanism_passed,
        "controller_promoted": controller_promoted,
        "mechanism_gates": mechanism_gates,
        "promotion_gates": promotion_gates,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
