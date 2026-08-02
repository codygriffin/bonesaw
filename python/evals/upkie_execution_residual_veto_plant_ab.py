#!/usr/bin/env python3
"""r163 causal plant A/B for a confidence-lowering execution-residual veto."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-execution-residual-veto-plant-ab-r163"


def boundary_delta(control: dict[str, Any], candidate: dict[str, Any], duration: float) -> float:
    control_fall = bool(control["fell"])
    candidate_fall = bool(candidate["fell"])
    if control_fall and candidate_fall:
        return float(candidate["terminal_time_s"] - control["terminal_time_s"])
    if control_fall:
        return float(duration - control["terminal_time_s"])
    if candidate_fall:
        return float(candidate["terminal_time_s"] - duration)
    return 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_EXECUTION_RESIDUAL_VETO_PLANT_AB_R163.html")
    parser.add_argument("--cases", help="comma-separated debug-only retained-case subset")
    return parser.parse_args()


def select_cases(value: str | None) -> tuple[Any, ...]:
    cases = case_matrix()
    if value is None:
        return cases
    requested = set(value.split(","))
    selected = tuple(case for case in cases if case.name in requested)
    missing = requested - {case.name for case in selected}
    if missing:
        raise SystemExit(f"unknown cases: {sorted(missing)}")
    return selected


def run(model: pathlib.Path, case: Any, duration: float, veto: bool) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration,
        measured_contact=True,
        planner=True,
        viability_support_requires_active_request=True,
        viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
        viability_confirmation_updates=2,
        execution_residual_veto=veto,
    )


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    cases = select_cases(args.cases)
    rows: list[dict[str, Any]] = []
    for index, case in enumerate(cases, 1):
        control = run(model, case, args.duration, False)
        candidate = run(model, case, args.duration, True)
        replay = run(model, case, args.duration, True)
        exact = semantic_trace_equal(candidate["trace"], replay["trace"])
        delta = boundary_delta(control["metrics"], candidate["metrics"], args.duration)
        row = {
            "case": case.name,
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "replay_exact": exact,
            "fall_boundary_delta_s": delta,
        }
        rows.append(row)
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: {outcome(control['metrics'])} → "
            f"{outcome(candidate['metrics'])}, Δ={delta:+.3f}s, "
            f"veto={candidate['metrics']['execution_residual_veto_ticks']}, "
            f"replay={'exact' if exact else 'DIFF'}",
            flush=True,
        )

    control_green = [row for row in rows if bool(row["control"]["qualified"])]
    lost_green = [row["case"] for row in control_green if not bool(row["candidate"]["qualified"])]
    earlier = [row["case"] for row in rows if row["fall_boundary_delta_s"] < -1.0e-12]
    later = [row["case"] for row in rows if row["fall_boundary_delta_s"] > 1.0e-12]
    total_veto = sum(row["candidate"]["execution_residual_veto_ticks"] for row in rows)
    total_exceeded = sum(row["candidate"]["execution_residual_exceeded_ticks"] for row in rows)
    control_executable = sum(row["control"]["executable_request_steps"] for row in rows)
    candidate_executable = sum(row["candidate"]["executable_request_steps"] for row in rows)
    allocation_calls = sum(row["candidate"]["execution_residual_allocation_calls"] for row in rows)
    allocated_bytes = sum(row["candidate"]["execution_residual_allocated_bytes"] for row in rows)
    veto_request_overlap = sum(
        row["candidate"]["execution_residual_veto_request_overlap_ticks"] for row in rows
    )
    loop_overruns = sum(row["candidate"]["loop_overruns"] for row in rows)
    full_matrix = len(cases) == len(case_matrix())
    gates = {
        "retained_matrix_complete": full_matrix,
        "exact_candidate_replay": all(row["replay_exact"] for row in rows),
        "residual_exceedance_exercised": total_exceeded > 0,
        "veto_path_exercised": total_veto > 0,
        "vetoed_tick_never_executes_request": veto_request_overlap == 0,
        "rust_monitor_zero_allocation": allocation_calls == 0 and allocated_bytes == 0,
        "all_control_green_rows_preserved": not lost_green,
        "no_earlier_fall_boundary": not earlier,
        "five_ms_control_deadline": loop_overruns == 0,
    }
    physical_pass = all(gates.values())
    authority_promoted = False
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "rows": rows,
        "aggregate": {
            "control_green_rows": len(control_green),
            "lost_green_rows": lost_green,
            "earlier_fall_boundaries": earlier,
            "later_fall_boundaries": later,
            "neutral_fall_boundaries": len(rows) - len(earlier) - len(later),
            "worst_fall_boundary_delta_s": min((row["fall_boundary_delta_s"] for row in rows), default=0.0),
            "best_fall_boundary_delta_s": max((row["fall_boundary_delta_s"] for row in rows), default=0.0),
            "control_executable_request_ticks": control_executable,
            "candidate_executable_request_ticks": candidate_executable,
            "residual_exceeded_ticks": total_exceeded,
            "residual_veto_ticks": total_veto,
            "monitor_allocation_calls": allocation_calls,
            "monitor_allocated_bytes": allocated_bytes,
            "veto_request_overlap_ticks": veto_request_overlap,
            "candidate_loop_overruns": loop_overruns,
        },
        "gates": gates,
        "physical_pass": physical_pass,
        "authority_promoted": authority_promoted,
    }
    table_rows = [
        [
            row["case"],
            outcome(row["control"]),
            outcome(row["candidate"]),
            f"{row['fall_boundary_delta_s']:+.3f}",
            row["control"]["executable_request_steps"],
            row["candidate"]["executable_request_steps"],
            row["candidate"]["execution_residual_veto_ticks"],
            "YES" if row["replay_exact"] else "NO",
        ]
        for row in rows
    ]
    aggregate = metrics["aggregate"]
    report = "\n".join(
        [
            "# Bonesaw execution-residual veto plant A/B · r163",
            "",
            f"> Causal physical gate **{'PASS' if physical_pass else 'FAIL'}** · live authority promotion **NO**.",
            "",
            "## Result",
            "",
            "The control and candidate run the same r158 hybrid-confirmed planner. The candidate adds only r162's confidence-lowering rule: an execution residual that exceeds the pre-update rolling envelope makes the current proposal unavailable before confirmation/request supervision. It cannot create a request, reuse rejected torque, or alter the retained r137 fallback.",
            "",
            f"The candidate reports **{total_exceeded}** exceedances / **{total_veto}** veto ticks and changes later trajectory-dependent executable-request exposure **{control_executable} → {candidate_executable} ticks**, while executable-request overlap on the vetoed tick itself remains **{veto_request_overlap}**. Green rows lost: **{len(lost_green)}**. Earlier/later/neutral first-boundary rows: **{len(earlier)}/{len(later)}/{len(rows) - len(earlier) - len(later)}**. Worst/best boundary delta: **{aggregate['worst_fall_boundary_delta_s']:+.3f}/{aggregate['best_fall_boundary_delta_s']:+.3f} s**. Timed Rust monitor allocation: **{allocation_calls} calls / {allocated_bytes} bytes**; 5 ms loop overruns: **{loop_overruns}**.",
            "",
            "## Retained causal matrix",
            "",
            *markdown_table(
                ["case", "control", "veto", "boundary Δs", "control exec", "veto exec", "veto ticks", "replay"],
                table_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]),
            "",
            "## Authority boundary",
            "",
            "Even a physical pass would admit only a fail-lowering veto mechanism. It would not prove the planner action reachable or promote r158. Live use remains withheld until deterministic deadline and independently solved support/contingency action gates pass.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-execution-residual-veto-plant-ab-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (output / "UPKIE_EXECUTION_RESIDUAL_VETO_PLANT_AB_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"physical_pass": physical_pass, "authority_promoted": authority_promoted, "web_report": str(web_report)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
