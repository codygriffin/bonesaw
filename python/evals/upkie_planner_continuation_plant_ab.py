#!/usr/bin/env python3
"""r166 causal plant A/B for bounded planner projection continuation."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execute, execution_trace_equal, outcome


REVISION = "upkie-planner-continuation-plant-ab-r166"


def run(
    model: pathlib.Path,
    case: Any,
    duration: float,
    projection_sweeps: int | None,
    continuation_threshold: float | None,
) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration,
        measured_contact=True,
        planner=True,
        viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
        viability_confirmation_updates=2,
        maximum_feasibility_projection_sweeps=None,
        repair_feasibility_equalities_before_inequalities=False,
        use_feasibility_row_spans=False,
        viability_planner_maximum_feasibility_projection_sweeps=projection_sweeps,
        viability_planner_projection_continuation_violation_threshold=(
            continuation_threshold
        ),
    )


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--planner-projection-sweeps", type=int, default=512)
    parser.add_argument(
        "--continuation-violation-threshold", type=float, default=0.1
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_PLANNER_CONTINUATION_PLANT_AB_R166.html",
    )
    parser.add_argument("--cases", help="comma-separated debug-only case subset")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if args.planner_projection_sweeps <= 0:
        raise SystemExit("--planner-projection-sweeps must be positive")
    if args.continuation_violation_threshold < 0.0:
        raise SystemExit("--continuation-violation-threshold must be nonnegative")

    model = pathlib.Path(args.model).resolve()
    cases = select_cases(args.cases)
    rows: list[dict[str, Any]] = []
    for index, case in enumerate(cases, 1):
        control = run(model, case, args.duration, None, None)
        candidate = run(
            model,
            case,
            args.duration,
            args.planner_projection_sweeps,
            args.continuation_violation_threshold,
        )
        replay = run(
            model,
            case,
            args.duration,
            args.planner_projection_sweeps,
            args.continuation_violation_threshold,
        )
        row = {
            "case": case.name,
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "control_candidate_exact": semantic_trace_equal(
                control["trace"], candidate["trace"]
            ),
            "plant_execution_exact": execution_trace_equal(
                control["trace"], candidate["trace"]
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }
        rows.append(row)
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: "
            f"{outcome(control['metrics'])} → {outcome(candidate['metrics'])}, "
            f"overruns {control['metrics']['loop_overruns']}→"
            f"{candidate['metrics']['loop_overruns']}, "
            f"authority exact={row['control_candidate_exact']}, "
            f"plant exact={row['plant_execution_exact']}",
            flush=True,
        )

    control_overruns = sum(row["control"]["loop_overruns"] for row in rows)
    candidate_overruns = sum(row["candidate"]["loop_overruns"] for row in rows)
    exact = all(row["control_candidate_exact"] for row in rows)
    plant_exact = all(row["plant_execution_exact"] for row in rows)
    gates = {
        "retained_matrix_complete": len(rows) == len(case_matrix()),
        "authority_trace_exact": exact,
        "plant_execution_trace_exact": plant_exact,
        "candidate_replay_exact": all(row["candidate_replay_exact"] for row in rows),
        "deadline_overruns_reduced": candidate_overruns < control_overruns,
        "zero_deadline_overruns": candidate_overruns == 0,
        "zero_rust_allocation": all(
            row[side]["allocation_free"]
            for row in rows
            for side in ("control", "candidate")
        ),
        "zero_python_gc": all(
            row[side]["python_gc_collections"] == 0
            for row in rows
            for side in ("control", "candidate")
        ),
        "finite": all(
            row[side]["finite"]
            for row in rows
            for side in ("control", "candidate")
        ),
    }
    admitted = all(gates.values())
    aggregate = {
        "control_loop_overruns": control_overruns,
        "candidate_loop_overruns": candidate_overruns,
        "worst_control_planner_p99_ns": max(
            row["control"]["viability_planner_step_ns"]["p99"] for row in rows
        ),
        "worst_candidate_planner_p99_ns": max(
            row["candidate"]["viability_planner_step_ns"]["p99"] for row in rows
        ),
        "worst_control_loop_p99_ns": max(
            row["control"]["loop_ns"]["p99"] for row in rows
        ),
        "worst_candidate_loop_p99_ns": max(
            row["candidate"]["loop_ns"]["p99"] for row in rows
        ),
    }
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "configuration": {
            "planner_projection_sweeps": args.planner_projection_sweeps,
            "continuation_violation_threshold": args.continuation_violation_threshold,
            "sparse_nonzero_rows": False,
            "executable_wbc_projection_sweeps": None,
        },
        "rows": rows,
        "aggregate": aggregate,
        "gates": gates,
        "profile_admitted": admitted,
        "controller_promoted": False,
    }
    table = [
        [
            row["case"],
            outcome(row["control"]),
            outcome(row["candidate"]),
            row["control"]["loop_overruns"],
            row["candidate"]["loop_overruns"],
            round(row["control"]["viability_planner_step_ns"]["p99"] / 1e3),
            round(row["candidate"]["viability_planner_step_ns"]["p99"] / 1e3),
            "YES" if row["control_candidate_exact"] else "NO",
            "YES" if row["plant_execution_exact"] else "NO",
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for row in rows
    ]
    report = "\n".join(
        [
            "# Bonesaw bounded planner continuation plant A/B · r166",
            "",
            f"> Profile **{'PASS' if admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            f"The candidate caps each speculative planner solve at **{args.planner_projection_sweeps} Dykstra sweeps**. If the exact prefix ends within **{args.continuation_violation_threshold:g}** normalized violation, Rust resumes the same point and multipliers to the established bound; otherwise the proposal remains typed `MaxIterations` and has no authority. The executable WBC remains uncapped and dense/full-row on both arms.",
            f"Across **{len(rows)}** frozen plant cases, >5 ms loops change **{control_overruns} → {candidate_overruns}**. Plant execution is **{'exact' if plant_exact else 'not exact'}**; the complete non-timing authority trace is **{'exact' if exact else 'not exact'}**.",
            "",
            "## Retained causal matrix",
            "",
            *markdown_table(
                [
                    "case",
                    "control",
                    "candidate",
                    "overrun C",
                    "overrun B",
                    "planner p99 C µs",
                    "planner p99 B µs",
                    "authority exact",
                    "plant exact",
                    "replay exact",
                ],
                table,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Authority boundary",
            "",
            "Only speculative planner work is bounded. Budget exhaustion cannot create or refresh a request, and no candidate torque crosses this boundary. Promotion remains false until the timing profile passes the frozen matrix and actual-browser/hardware scheduling is separately established.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-planner-continuation-plant-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_PLANNER_CONTINUATION_PLANT_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "profile_admitted": admitted,
                "controller_promoted": False,
                "web_report": str(web),
            },
            sort_keys=True,
        )
    )
    return 0 if admitted else 1


if __name__ == "__main__":
    raise SystemExit(main())
