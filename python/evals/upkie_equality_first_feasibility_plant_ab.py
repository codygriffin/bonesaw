#!/usr/bin/env python3
"""r165 causal plant A/B for equality-first active-set feasibility repair."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_execution_residual_veto_plant_ab import boundary_delta
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-equality-first-feasibility-plant-ab-r165"


def run(model: pathlib.Path, case: Any, duration: float, repair: bool) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration,
        measured_contact=True,
        planner=True,
        viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
        viability_confirmation_updates=2,
        repair_feasibility_equalities_before_inequalities=repair,
        use_feasibility_row_spans=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_EQUALITY_FIRST_FEASIBILITY_PLANT_AB_R165.html")
    parser.add_argument("--cases", help="comma-separated debug-only retained-case subset")
    return parser.parse_args()


def select_cases(value: str | None) -> tuple[Any, ...]:
    cases = case_matrix()
    if value is None:
        return cases
    names = set(value.split(","))
    selected = tuple(case for case in cases if case.name in names)
    missing = names - {case.name for case in selected}
    if missing:
        raise SystemExit(f"unknown cases: {sorted(missing)}")
    return selected


def main() -> int:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    cases = select_cases(args.cases)
    rows = []
    for index, case in enumerate(cases, 1):
        control = run(model, case, args.duration, False)
        candidate = run(model, case, args.duration, True)
        replay = run(model, case, args.duration, True)
        control_sweeps = np.asarray(control["trace"]["feasibility_projection_sweeps"], np.uint64)
        candidate_sweeps = np.asarray(candidate["trace"]["feasibility_projection_sweeps"], np.uint64)
        control_projections = np.asarray(control["trace"]["feasibility_halfspace_projections"], np.uint64)
        candidate_projections = np.asarray(candidate["trace"]["feasibility_halfspace_projections"], np.uint64)
        row = {
            "case": case.name,
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "candidate_replay_exact": semantic_trace_equal(candidate["trace"], replay["trace"]),
            "fall_boundary_delta_s": boundary_delta(control["metrics"], candidate["metrics"], args.duration),
            "control_maximum_projection_sweeps": int(np.max(control_sweeps)),
            "candidate_maximum_projection_sweeps": int(np.max(candidate_sweeps)),
            "control_halfspace_projections": int(np.sum(control_projections)),
            "candidate_halfspace_projections": int(np.sum(candidate_projections)),
        }
        rows.append(row)
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: {outcome(control['metrics'])} → "
            f"{outcome(candidate['metrics'])}, overruns {control['metrics']['loop_overruns']}→"
            f"{candidate['metrics']['loop_overruns']}, sweeps {row['control_maximum_projection_sweeps']}→"
            f"{row['candidate_maximum_projection_sweeps']}, Δ={row['fall_boundary_delta_s']:+.3f}s",
            flush=True,
        )

    control_green = [row for row in rows if bool(row["control"]["qualified"])]
    lost_green = [row["case"] for row in control_green if not bool(row["candidate"]["qualified"])]
    earlier = [row["case"] for row in rows if row["fall_boundary_delta_s"] < -1.0e-12]
    later = [row["case"] for row in rows if row["fall_boundary_delta_s"] > 1.0e-12]
    control_overruns = sum(row["control"]["loop_overruns"] for row in rows)
    candidate_overruns = sum(row["candidate"]["loop_overruns"] for row in rows)
    control_projections = sum(row["control_halfspace_projections"] for row in rows)
    candidate_projections = sum(row["candidate_halfspace_projections"] for row in rows)
    gates = {
        "retained_matrix_complete": len(rows) == len(case_matrix()),
        "candidate_replay_exact": all(row["candidate_replay_exact"] for row in rows),
        "fallback_repair_exercised": candidate_projections < control_projections,
        "zero_deadline_overruns": candidate_overruns == 0,
        "zero_rust_allocation": all(row[side]["allocation_free"] for row in rows for side in ("control", "candidate")),
        "zero_python_gc": all(row[side]["python_gc_collections"] == 0 for row in rows for side in ("control", "candidate")),
        "finite": all(row[side]["finite"] for row in rows for side in ("control", "candidate")),
        "all_control_green_rows_preserved": not lost_green,
        "no_earlier_fall_boundary": not earlier,
    }
    timing_mechanism_pass = all(gates[name] for name in (
        "retained_matrix_complete",
        "candidate_replay_exact",
        "fallback_repair_exercised",
        "zero_deadline_overruns",
        "zero_rust_allocation",
        "zero_python_gc",
        "finite",
    ))
    physical_pass = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "configuration": {"sparse_nonzero_rows": True, "candidate_equality_first_repair": True},
        "rows": rows,
        "aggregate": {
            "control_green_rows": len(control_green),
            "lost_green_rows": lost_green,
            "earlier_fall_boundaries": earlier,
            "later_fall_boundaries": later,
            "neutral_fall_boundaries": len(rows) - len(earlier) - len(later),
            "worst_fall_boundary_delta_s": min((row["fall_boundary_delta_s"] for row in rows), default=0.0),
            "best_fall_boundary_delta_s": max((row["fall_boundary_delta_s"] for row in rows), default=0.0),
            "control_loop_overruns": control_overruns,
            "candidate_loop_overruns": candidate_overruns,
            "control_halfspace_projections": control_projections,
            "candidate_halfspace_projections": candidate_projections,
            "worst_control_loop_p99_ns": max((row["control"]["loop_ns"]["p99"] for row in rows), default=0.0),
            "worst_candidate_loop_p99_ns": max((row["candidate"]["loop_ns"]["p99"] for row in rows), default=0.0),
        },
        "gates": gates,
        "timing_mechanism_pass": timing_mechanism_pass,
        "physical_pass": physical_pass,
        "profile_promoted": False,
        "controller_promoted": False,
    }
    table = [
        [
            row["case"], outcome(row["control"]), outcome(row["candidate"]),
            row["control"]["loop_overruns"], row["candidate"]["loop_overruns"],
            row["control_maximum_projection_sweeps"], row["candidate_maximum_projection_sweeps"],
            f"{row['fall_boundary_delta_s']:+.3f}", "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for row in rows
    ]
    aggregate = metrics["aggregate"]
    report = "\n".join([
        "# Bonesaw equality-first feasibility plant A/B · r165", "",
        f"> Timing mechanism **{'PASS' if timing_mechanism_pass else 'FAIL'}** · physical gate **{'PASS' if physical_pass else 'FAIL'}** · profile/controller promotion **NO/NO**.", "",
        "## Result", "",
        "The candidate changes one default-off feasibility rule: when the bounded Dykstra prefix hands the active-set accelerator a point displaced from hard equalities, it solves the equality block before searching inequality violations. Exact sparse-nonzero row traversal is enabled on both arms. This may change solver/plant execution, so timing and physical gates are independent.", "",
        f"Logical half-space projections change **{control_projections:,} → {candidate_projections:,}** and >5 ms loops **{control_overruns} → {candidate_overruns}**. Green rows lost: **{len(lost_green)}**; earlier/later/neutral boundaries: **{len(earlier)}/{len(later)}/{len(rows)-len(earlier)-len(later)}**; worst/best delta **{aggregate['worst_fall_boundary_delta_s']:+.3f}/{aggregate['best_fall_boundary_delta_s']:+.3f} s**.", "",
        "## Retained causal matrix", "",
        *markdown_table(["case", "control", "repair", "overrun C", "overrun R", "max sweep C", "max sweep R", "boundary Δs", "replay"], table), "",
        "## Gates", "",
        *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]), "",
        "## Authority boundary", "",
        "A timing-mechanism pass cannot admit this profile if the physical gate fails. A changed feasible seed is not an arithmetic-only optimization; it remains default-off until both exact solver semantics and downstream consequences are accepted.",
    ]) + "\n"
    output = pathlib.Path(args.output); output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-equality-first-feasibility-plant-ab-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (output / "UPKIE_EQUALITY_FIRST_FEASIBILITY_PLANT_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report); web.parent.mkdir(parents=True, exist_ok=True); web.write_text(render_report_html(report))
    print(json.dumps({"timing_mechanism_pass": timing_mechanism_pass, "physical_pass": physical_pass, "profile_promoted": False, "controller_promoted": False, "web_report": str(web)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
