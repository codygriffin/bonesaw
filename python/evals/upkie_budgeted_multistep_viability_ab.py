#!/usr/bin/env python3
"""Retained plant A/B for the fixed-budget multistep viability planner."""

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
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-budgeted-multistep-viability-ab-r151"
FORECAST_PRESSURES = (
    "peak_capture",
    "peak_sagittal",
    "terminal_capture",
    "terminal_sagittal",
    "terminal_rate",
    "yaw",
    "resource",
    "action",
    "action_delta",
    "support",
)
TRACE_PRESSURES = tuple(
    f"viability_forecast_{name}_pressure" for name in FORECAST_PRESSURES
)
REQUEST_LIMIT = np.asarray([250.0, 250.0, 80.0], np.float64)
TRUST_REGION_LIMIT = np.asarray([40.0, 40.0, 20.0], np.float64)


def parse_args(revision: str, web_report: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{revision}")
    parser.add_argument(
        "--web-report",
        default=web_report,
    )
    parser.add_argument(
        "--cases",
        help="comma-separated subset for smoke/debug; admission requires all 20",
    )
    return parser.parse_args()


def execute_arm(
    model: pathlib.Path,
    case: Any,
    duration: float,
    *,
    candidate: bool,
    planner_strategy: str = "multistep_budgeted",
) -> tuple[dict[str, Any], dict[str, Any]]:
    result = execute(
        model,
        case,
        duration,
        measured_contact=candidate,
        planner=candidate,
        viability_support_requires_active_request=candidate,
        viability_planner_strategy=(planner_strategy if candidate else "coordinate"),
    )
    metrics = result["metrics"]
    trace = result["trace"]
    if not candidate:
        return metrics, trace

    query_count = np.asarray(trace["viability_planner_query_count"])
    request = np.asarray(trace["viability_request"])
    forecast = np.column_stack([np.asarray(trace[field]) for field in TRACE_PRESSURES])
    dominant = np.argmax(forecast, axis=1)
    scored = np.any(forecast != 0.0, axis=1)
    metrics.update(
        {
            "planner_strategy": planner_strategy,
            "planner_query_cardinalities": sorted(
                int(value) for value in np.unique(query_count)
            ),
            "planner_queries": int(np.sum(query_count)),
            "planner_update_ticks": int(np.count_nonzero(query_count)),
            "planner_poll_ticks": int(np.count_nonzero(query_count > 1)),
            "maximum_queries_per_tick": int(np.max(query_count)),
            "planner_deadline_misses": int(
                np.count_nonzero(np.asarray(trace["controller_step_ns"]) > 5_000_000)
            ),
            "active_request_ticks": int(
                np.count_nonzero(trace["viability_request_active"])
            ),
            "executable_request_ticks": int(
                np.count_nonzero(trace["viability_request_executable"])
            ),
            "support_program_ticks": int(
                np.count_nonzero(
                    np.any(np.asarray(trace["admitted_contact_active"]) != 1, axis=1)
                )
            ),
            "request_edge_ticks": int(
                np.count_nonzero(np.any(np.isclose(np.abs(request), REQUEST_LIMIT), axis=1))
            ),
            "trust_region_edge_ticks": int(
                np.count_nonzero(
                    np.any(np.isclose(np.abs(request), TRUST_REGION_LIMIT), axis=1)
                )
            ),
            "strict_forecast_descent_ticks": int(
                np.count_nonzero(
                    (query_count > 1)
                    & (
                        np.asarray(trace["viability_planner_candidate_pressure"])
                        < np.asarray(trace["viability_planner_zero_pressure"])
                        - 1.0e-12
                    )
                )
            ),
            "dominant_forecast_pressure_counts": {
                name: int(np.count_nonzero(scored & (dominant == index)))
                for index, name in enumerate(FORECAST_PRESSURES)
            },
            "maximum_forecast_pressures": {
                name: float(np.max(forecast[:, index]))
                for index, name in enumerate(FORECAST_PRESSURES)
            },
            "minimum_forecast_capture_margin": float(
                np.min(trace["viability_forecast_minimum_capture_margin"])
            ),
        }
    )
    return metrics, trace


def main(
    *,
    revision: str = REVISION,
    planner_strategy: str = "multistep_budgeted",
    web_report: str = "web/UPKIE_BUDGETED_MULTISTEP_VIABILITY_AB_R151.html",
    report_title: str = "Bonesaw fixed-budget multistep viability plant A/B · r151",
    activation_description: str = (
        "Capture, sagittal, or actionable single-support pressure may activate the poll."
    ),
    metrics_name: str = "upkie-budgeted-multistep-viability-ab-metrics.json",
    audit_name: str = "UPKIE_BUDGETED_MULTISTEP_VIABILITY_AB_AUDIT.md",
) -> None:
    args = parse_args(revision, web_report)
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    selected = list(case_matrix())
    if args.cases:
        names = set(args.cases.split(","))
        selected = [case for case in selected if case.name in names]
        missing = names - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    retained = json.loads(
        pathlib.Path(
            "benchmarks/results/upkie-contact-command-freshness-composition-r145/"
            "upkie-contact-command-freshness-composition-metrics.json"
        ).read_text()
    )
    rows: dict[str, Any] = {}
    for case in selected:
        baseline, _ = execute_arm(model, case, args.duration, candidate=False)
        candidate, trace = execute_arm(
            model,
            case,
            args.duration,
            candidate=True,
            planner_strategy=planner_strategy,
        )
        _, replay = execute_arm(
            model,
            case,
            args.duration,
            candidate=True,
            planner_strategy=planner_strategy,
        )
        rows[case.name] = {
            "baseline": baseline,
            "candidate": candidate,
            "boundary_delta_s": float(
                candidate["terminal_time_s"] - baseline["terminal_time_s"]
            ),
            "candidate_replay_exact": semantic_trace_equal(trace, replay),
        }
        print(
            f"{case.name}: {outcome(baseline)} -> {outcome(candidate)} "
            f"@ {candidate['terminal_time_s']:.3f}s; "
            f"queries {candidate['planner_queries']}",
            flush=True,
        )

    green = [
        name for name, row in retained["rows"].items() if row["live"]["qualified"]
    ]
    adverse = [name for name in rows if name not in green]
    lost_green = [name for name in green if not rows[name]["candidate"]["qualified"]]
    new_falls = [
        name
        for name, row in rows.items()
        if not row["baseline"]["fell"] and row["candidate"]["fell"]
    ]
    earlier = [
        name
        for name in adverse
        if rows[name]["candidate"]["fell"]
        and rows[name]["candidate"]["terminal_time_s"]
        < rows[name]["baseline"]["terminal_time_s"] - 1.0e-12
    ]
    newly_recovered = [
        name
        for name in adverse
        if rows[name]["baseline"]["outcome"] != "RECOVERED"
        and rows[name]["candidate"]["outcome"] == "RECOVERED"
    ]
    mechanism_gates = {
        "complete_frozen_matrix": len(rows) == 20,
        "current_live_reproduced": len(rows) == 20
        and all(
            row["baseline"]["outcome"] == retained["rows"][name]["live"]["outcome"]
            and math.isclose(
                row["baseline"]["terminal_time_s"],
                retained["rows"][name]["live"]["terminal_time_s"],
                abs_tol=1.0e-12,
            )
            for name, row in rows.items()
        ),
        "fixed_query_budget_at_most_four": all(
            row["candidate"]["maximum_queries_per_tick"] <= 4
            for row in rows.values()
        ),
        "multistep_pressure_stack_finite": all(
            all(math.isfinite(value) for value in row["candidate"]["maximum_forecast_pressures"].values())
            and math.isfinite(row["candidate"]["minimum_forecast_capture_margin"])
            for row in rows.values()
        ),
        "candidate_exact_replay": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "zero_timed_rust_allocation": all(
            row[side]["allocation_free"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
        "zero_python_gc": all(
            row["candidate"]["python_gc_collections"] == 0 for row in rows.values()
        ),
        "finite_and_numeric_clean": all(
            row[side]["finite"] and not row[side]["numeric_fault"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
    }
    physical_gates = {
        "every_green_qualification_preserved": not lost_green,
        "no_new_fall": not new_falls,
        "no_earlier_adverse_boundary": not earlier,
        "at_least_one_new_adverse_recovery": bool(newly_recovered),
        "candidate_p99_inside_5ms": all(
            row["candidate"]["controller_step_ns"]["p99"] <= 5_000_000
            for row in rows.values()
        ),
    }
    evaluation_passed = all(mechanism_gates.values())
    deployment_passed = evaluation_passed and all(physical_gates.values())
    aggregate = {
        "case_count": len(rows),
        "total_planner_queries": sum(
            row["candidate"]["planner_queries"] for row in rows.values()
        ),
        "total_planner_deadline_misses": sum(
            row["candidate"]["planner_deadline_misses"] for row in rows.values()
        ),
        "total_request_edge_ticks": sum(
            row["candidate"]["request_edge_ticks"] for row in rows.values()
        ),
        "total_trust_region_edge_ticks": sum(
            row["candidate"]["trust_region_edge_ticks"] for row in rows.values()
        ),
        "worst_boundary_delta_s": min(
            (row["boundary_delta_s"] for row in rows.values()), default=0.0
        ),
        "worst_candidate_p99_ns": max(
            (row["candidate"]["controller_step_ns"]["p99"] for row in rows.values()),
            default=0.0,
        ),
    }
    metrics = {
        "revision": revision,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "evaluation_passed": evaluation_passed,
        "deployment_passed": deployment_passed,
        "mechanism_gates": mechanism_gates,
        "physical_gates": physical_gates,
        "contract": {
            "planner_period_ticks": 1,
            "activation_authority": activation_description,
            "maximum_planner_wbc_queries_per_tick": 4,
            "forecast_knots": 8,
            "forecast_horizon_s": 0.24,
            "acceleration_hold_s": 0.06,
            "reduced_support_requires_executable_request": True,
            "policy_model": None,
            "controller_physics_engine": None,
            "external_consequence_plant": "MuJoCo",
        },
        "classification": {
            "green": green,
            "lost_green": lost_green,
            "adverse": adverse,
            "new_falls": new_falls,
            "earlier_adverse": earlier,
            "newly_recovered_adverse": newly_recovered,
        },
        "aggregate": aggregate,
        "rows": rows,
    }
    table = [
        [
            name,
            outcome(row["baseline"]),
            outcome(row["candidate"]),
            f"{row['boundary_delta_s']:+.3f}",
            row["candidate"]["planner_queries"],
            row["candidate"]["maximum_queries_per_tick"],
            max(
                row["candidate"]["dominant_forecast_pressure_counts"],
                key=row["candidate"]["dominant_forecast_pressure_counts"].get,
            ),
            f"{row['candidate']['controller_step_ns']['p99'] / 1e3:.1f}",
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for name, row in rows.items()
    ]
    report = "\n".join(
        [
            f"# {report_title}",
            "",
            f"> Mechanism evaluation **{'PASS' if evaluation_passed else 'FAIL'}**. Live deployment **{'PASS' if deployment_passed else 'REJECTED'}**. The candidate is never promoted when any physical gate fails.",
            "",
            "## Outcome",
            "",
            "The r146 one-shot maximum-pressure score is replaced by an eight-knot Rust forecast. Acceleration is held for 60 ms, then coasted through a 240 ms horizon. Capture, terminal rate, yaw, support, torque/joint resource, action magnitude, and action change remain separate witnesses. Python schedules one three-point coordinate poll per tick; including the baseline, no tick performs more than four planner WBC queries, and the final current-support WBC solve remains mandatory.",
            "",
            activation_description,
            "",
            f"Green regressions: **{lost_green or 'none'}**. New falls: **{new_falls or 'none'}**. Earlier adverse boundaries: **{earlier or 'none'}**. Newly recovered adverse rows: **{newly_recovered or 'none'}**.",
            "",
            *markdown_table(
                ["case", "r137", "candidate", "Δ s", "queries", "max/tick", "dominant", "p99 µs", "replay"],
                table,
            ),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in mechanism_gates.items()],
            ),
            "",
            "## Physical gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in physical_gates.items()],
            ),
            "",
            "## Interpretation",
            "",
            "- This is a reduced-order forecast scorer, not a learned policy, physics engine, or proof that future contact follows the forecast. MuJoCo is used only as the external consequence gate.",
            "- Request lifetime, exact evidence, slew, reduced-support admission, and final WBC verification remain the independent r148/r150 authority boundaries.",
            "- Each pressure component is retained independently so a capture, support, joint/torque resource, yaw, rate, or action limit can be displayed without hiding it in the total score.",
            f"- Total exact planner queries: **{aggregate['total_planner_queries']}**; >5 ms controller ticks: **{aggregate['total_planner_deadline_misses']}**; global-request-edge ticks: **{aggregate['total_request_edge_ticks']}**; one-tick trust-region-edge ticks: **{aggregate['total_trust_region_edge_ticks']}**.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / metrics_name).write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / audit_name).write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "evaluation_passed": evaluation_passed,
                "deployment_passed": deployment_passed,
                "mechanism_gates": mechanism_gates,
                "physical_gates": physical_gates,
                "classification": metrics["classification"],
                "aggregate": aggregate,
            },
            sort_keys=True,
        )
    )
    if not evaluation_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
