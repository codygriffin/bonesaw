#!/usr/bin/env python3
"""Causal A/B for a deterministic cross-tick viability-planner cadence."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-cross-tick-planner-cadence-ab-r171"
DURATION_S = 6.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--candidate-period-ticks", type=int, default=3)
    parser.add_argument("--planner-projection-sweeps", type=int, default=512)
    parser.add_argument(
        "--planner-continuation-violation-threshold", type=float, default=0.1
    )
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CROSS_TICK_PLANNER_CADENCE_AB_R171.html"
    )
    parser.add_argument("--cases", help="comma-separated subset; admission needs all 20")
    return parser.parse_args()


def run(
    model: pathlib.Path,
    case: Any,
    duration_s: float,
    period_ticks: int,
    projection_sweeps: int,
    continuation_threshold: float,
) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration_s,
        measured_contact=True,
        planner=True,
        viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
        viability_planner_update_period_ticks=period_ticks,
        viability_confirmation_updates=2,
        maximum_feasibility_projection_sweeps=None,
        repair_feasibility_equalities_before_inequalities=False,
        use_feasibility_row_spans=True,
        viability_planner_maximum_feasibility_projection_sweeps=projection_sweeps,
        viability_planner_projection_continuation_violation_threshold=(
            continuation_threshold
        ),
        viability_planner_reuse_identical_hard_feasibility_seed=True,
    )


def cadence_exact(trace: dict[str, Any], period_ticks: int) -> bool:
    update = np.asarray(trace["viability_planner_update"], np.uint8)
    expected = (np.arange(len(update), dtype=np.uint64) % period_ticks == 0).astype(
        np.uint8
    )
    query_count = np.asarray(trace["viability_planner_query_count"], np.uint64)
    return bool(
        np.array_equal(update, expected)
        and not np.any((update == 0) & (query_count != 0))
    )


def boundary_delta_s(control: dict[str, Any], candidate: dict[str, Any], duration: float) -> float:
    control_time = (
        float(control["terminal_time_s"]) if control["fell"] else duration
    )
    candidate_time = (
        float(candidate["terminal_time_s"]) if candidate["fell"] else duration
    )
    return candidate_time - control_time


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    if not 2 <= args.candidate_period_ticks <= 16:
        raise SystemExit("--candidate-period-ticks must be in 2..=16")
    if args.planner_projection_sweeps <= 0:
        raise SystemExit("--planner-projection-sweeps must be positive")
    if (
        not np.isfinite(args.planner_continuation_violation_threshold)
        or args.planner_continuation_violation_threshold < 0.0
    ):
        raise SystemExit("continuation threshold must be finite and nonnegative")

    selected = case_matrix()
    if args.cases:
        names = set(args.cases.split(","))
        selected = tuple(case for case in selected if case.name in names)
        missing = names - {case.name for case in selected}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    model = pathlib.Path(args.model).resolve()
    rows: dict[str, Any] = {}
    for index, case in enumerate(selected, start=1):
        control = run(
            model,
            case,
            args.duration,
            1,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
        )
        candidate = run(
            model,
            case,
            args.duration,
            args.candidate_period_ticks,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
        )
        replay = run(
            model,
            case,
            args.duration,
            args.candidate_period_ticks,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
        )
        delta = boundary_delta_s(control["metrics"], candidate["metrics"], args.duration)
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "boundary_delta_s": delta,
            "control_cadence_exact": cadence_exact(control["trace"], 1),
            "candidate_cadence_exact": cadence_exact(
                candidate["trace"], args.candidate_period_ticks
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
            "candidate_cadence_replay_exact": np.array_equal(
                np.asarray(candidate["trace"]["viability_planner_update"]),
                np.asarray(replay["trace"]["viability_planner_update"]),
            ),
        }
        print(
            f"[{index:02d}/{len(selected)}] {case.name}: "
            f"{control['metrics']['loop_overruns']}→{candidate['metrics']['loop_overruns']} misses, "
            f"boundary {delta:+.3f}s, "
            f"requests {control['metrics']['executable_request_steps']}→"
            f"{candidate['metrics']['executable_request_steps']}",
            flush=True,
        )

    controls = [row["control"] for row in rows.values()]
    candidates = [row["candidate"] for row in rows.values()]
    gates = {
        "matrix_complete": len(rows) == len(case_matrix()),
        "control_cadence_exact": all(
            row["control_cadence_exact"] for row in rows.values()
        ),
        "candidate_cadence_exact": all(
            row["candidate_cadence_exact"] for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"]
            and row["candidate_cadence_replay_exact"]
            for row in rows.values()
        ),
        "zero_synchronous_deadline_misses": all(
            side["loop_overruns"] == 0 for side in candidates
        ),
        "zero_final_wbc_deadline_misses": all(
            side["final_wbc_step_ns"]["maximum"] <= 5.0e6 for side in candidates
        ),
        "no_earlier_physical_boundary": all(
            row["boundary_delta_s"] >= -1.0e-12 for row in rows.values()
        ),
        "no_new_numeric_fault": all(
            not candidate["numeric_fault"]
            for control, candidate in zip(controls, candidates, strict=True)
            if not control["numeric_fault"]
        ),
        "request_only_after_confirmation": all(
            side["request_only_after_confirmation"] for side in candidates
        ),
        "request_only_after_hybrid_guard": all(
            side["request_only_after_hybrid_guard"] for side in candidates
        ),
        "candidate_age_bounded_by_period": all(
            side["maximum_candidate_age_ticks"] <= args.candidate_period_ticks
            for side in candidates
        ),
        "zero_rust_allocation": all(
            side["allocation_free"] for side in (*controls, *candidates)
        ),
        "zero_python_gc": all(
            side["python_gc_collections"] == 0 for side in (*controls, *candidates)
        ),
        "finite": all(side["finite"] for side in (*controls, *candidates)),
    }
    timing_admitted = all(
        gates[name]
        for name in (
            "matrix_complete",
            "candidate_cadence_exact",
            "candidate_replay_exact",
            "zero_synchronous_deadline_misses",
            "zero_final_wbc_deadline_misses",
            "zero_rust_allocation",
            "zero_python_gc",
            "finite",
        )
    )
    physical_admitted = all(gates.values())
    control_misses = sum(side["loop_overruns"] for side in controls)
    candidate_misses = sum(side["loop_overruns"] for side in candidates)
    worst_boundary = min(row["boundary_delta_s"] for row in rows.values())
    control_queries = sum(side["total_planner_wbc_queries"] for side in controls)
    candidate_queries = sum(side["total_planner_wbc_queries"] for side in candidates)
    control_requests = sum(side["executable_request_steps"] for side in controls)
    candidate_requests = sum(side["executable_request_steps"] for side in candidates)

    table_rows = [
        [
            name,
            outcome(row["control"]),
            outcome(row["candidate"]),
            f"{row['boundary_delta_s']:+.3f}",
            row["control"]["loop_overruns"],
            row["candidate"]["loop_overruns"],
            row["control"]["total_planner_wbc_queries"],
            row["candidate"]["total_planner_wbc_queries"],
            row["control"]["executable_request_steps"],
            row["candidate"]["executable_request_steps"],
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for name, row in rows.items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "host": platform.platform(),
        "rustflags": os.environ.get("RUSTFLAGS", ""),
        "control_period_ticks": 1,
        "candidate_period_ticks": args.candidate_period_ticks,
        "timing_profile_admitted": timing_admitted,
        "physical_profile_admitted": physical_admitted,
        "controller_promoted": False,
        "gates": gates,
        "aggregate": {
            "case_count": len(rows),
            "control_loop_misses": control_misses,
            "candidate_loop_misses": candidate_misses,
            "control_planner_wbc_queries": control_queries,
            "candidate_planner_wbc_queries": candidate_queries,
            "control_executable_request_ticks": control_requests,
            "candidate_executable_request_ticks": candidate_requests,
            "worst_boundary_delta_s": worst_boundary,
            "worst_candidate_final_wbc_max_ns": max(
                side["final_wbc_step_ns"]["maximum"] for side in candidates
            ),
            "worst_candidate_loop_max_ns": max(
                side["loop_ns"]["maximum"] for side in candidates
            ),
        },
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw deterministic cross-tick planner cadence A/B · r171",
            "",
            f"> Timing profile **{'PASS' if timing_admitted else 'FAIL'}** · physical profile **{'PASS' if physical_admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            f"The candidate runs the speculative planner once every **{args.candidate_period_ticks} control ticks**. Rust request supervision treats intervening ticks as intentional no-update holds, while confirmation expires only beyond the same explicit period. Scheduling depends only on tick sequence, never measured wall time.",
            f"Across **{len(rows)}** frozen cases, synchronous misses change **{control_misses} → {candidate_misses}** and planner WBC queries change **{control_queries:,} → {candidate_queries:,}**. The worst physical boundary moves **{worst_boundary:+.3f} s** and executable-request ticks change **{control_requests} → {candidate_requests}**.",
            "",
            *markdown_table(
                [
                    "case",
                    "period 1",
                    f"period {args.candidate_period_ticks}",
                    "boundary Δs",
                    "miss P1",
                    "miss Px",
                    "queries P1",
                    "queries Px",
                    "request P1",
                    "request Px",
                    "replay",
                ],
                table_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Interpretation",
            "",
            "- The mechanism is a bounded temporal contract, not an asynchronous thread and not a wall-clock heuristic.",
            "- A skipped tick cannot be reported as a failed planner update; candidate age, confirmation gap, and request freshness remain explicit Rust state.",
            "- Timing admission and physical non-regression are separate. A zero-miss cadence is rejected if it suppresses useful requests or moves any retained boundary earlier.",
            "- No result promotes the experimental viability action, r169 cache, or controller. R137 remains live.",
            "",
            "## Scope",
            "",
            f"Host timing is descriptive for `{os.environ.get('RUSTFLAGS', '')}`. The evaluation uses the retained Upkie MuJoCo plant and has no hardware, estimator delay/noise, real-time kernel, or authenticated transport claim.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-cross-tick-planner-cadence-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CROSS_TICK_PLANNER_CADENCE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "timing_profile_admitted": timing_admitted,
                "physical_profile_admitted": physical_admitted,
                "controller_promoted": False,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if physical_admitted else 1


if __name__ == "__main__":
    raise SystemExit(main())
