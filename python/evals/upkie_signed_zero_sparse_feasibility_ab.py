#!/usr/bin/env python3
"""r170 exact plant A/B for signed-zero-preserving sparse feasibility rows."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-signed-zero-sparse-feasibility-ab-r170"


def run(model: pathlib.Path, case: Any, duration: float, sparse: bool) -> dict[str, Any]:
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
        use_feasibility_row_spans=sparse,
        viability_planner_maximum_feasibility_projection_sweeps=None,
        viability_planner_projection_continuation_violation_threshold=None,
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
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_SIGNED_ZERO_SPARSE_FEASIBILITY_AB_R170.html",
    )
    parser.add_argument("--cases", help="comma-separated debug-only case subset")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    cases = select_cases(args.cases)
    rows: list[dict[str, Any]] = []
    for index, case in enumerate(cases, 1):
        control = run(model, case, args.duration, False)
        candidate = run(model, case, args.duration, True)
        replay = run(model, case, args.duration, True)
        control_projections = int(
            np.sum(
                np.asarray(
                    control["trace"]["feasibility_halfspace_projections"], np.uint64
                )
            )
        )
        candidate_projections = int(
            np.sum(
                np.asarray(
                    candidate["trace"]["feasibility_halfspace_projections"],
                    np.uint64,
                )
            )
        )
        row = {
            "case": case.name,
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "control_candidate_exact": semantic_trace_equal(
                control["trace"], candidate["trace"]
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
            "control_halfspace_projections": control_projections,
            "candidate_halfspace_projections": candidate_projections,
        }
        rows.append(row)
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: "
            f"overruns {control['metrics']['loop_overruns']}→"
            f"{candidate['metrics']['loop_overruns']}, "
            f"projections {control_projections}→{candidate_projections}, "
            f"complete exact={row['control_candidate_exact']}",
            flush=True,
        )

    control_overruns = sum(row["control"]["loop_overruns"] for row in rows)
    candidate_overruns = sum(row["candidate"]["loop_overruns"] for row in rows)
    control_projections = sum(row["control_halfspace_projections"] for row in rows)
    candidate_projections = sum(row["candidate_halfspace_projections"] for row in rows)
    gates = {
        "retained_matrix_complete": len(rows) == len(case_matrix()),
        "complete_trace_exact": all(row["control_candidate_exact"] for row in rows),
        "projection_work_exact": candidate_projections == control_projections,
        "candidate_replay_exact": all(row["candidate_replay_exact"] for row in rows),
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
        "control_halfspace_projections": control_projections,
        "candidate_halfspace_projections": candidate_projections,
        "worst_control_final_wbc_p99_ns": max(
            row["control"]["final_wbc_step_ns"]["p99"] for row in rows
        ),
        "worst_candidate_final_wbc_p99_ns": max(
            row["candidate"]["final_wbc_step_ns"]["p99"] for row in rows
        ),
        "worst_control_loop_max_ns": max(
            row["control"]["loop_ns"]["maximum"] for row in rows
        ),
        "worst_candidate_loop_max_ns": max(
            row["candidate"]["loop_ns"]["maximum"] for row in rows
        ),
    }
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "configuration": {
            "candidate_sparse_nonzero_rows": True,
            "candidate_preserves_dense_signed_zero_updates": True,
            "planner_projection_cap": None,
            "equality_first_repair": False,
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
            round(row["control"]["final_wbc_step_ns"]["p99"] / 1e3),
            round(row["candidate"]["final_wbc_step_ns"]["p99"] / 1e3),
            "YES" if row["control_candidate_exact"] else "NO",
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for row in rows
    ]
    report = "\n".join(
        [
            "# Bonesaw signed-zero-preserving sparse feasibility A/B · r170",
            "",
            f"> Exact CPU profile **{'PASS' if admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            "The candidate traverses compiled nonzero coordinate/value lists for Dykstra dot and transpose work, but explicitly reproduces the dense kernel's `-0 → +0` transitions for omitted signed-zero coefficients. This preserves bitwise active-set tie/freeze inputs without executing every zero multiply. Both planner and executable WBC use the same kernel; neither has a projection cap or equality-first repair.",
            f"Across **{len(rows)}** frozen cases, >5 ms loops change **{control_overruns} → {candidate_overruns}**, logical projections remain **{control_projections:,} → {candidate_projections:,}**, and the complete non-timing trace is **{'exact' if gates['complete_trace_exact'] else 'not exact'}**.",
            "",
            "## Retained causal matrix",
            "",
            *markdown_table(
                [
                    "case",
                    "dense",
                    "sparse signed-zero",
                    "overrun D",
                    "overrun S",
                    "final WBC p99 D µs",
                    "final WBC p99 S µs",
                    "complete exact",
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
            "## Scope",
            "",
            "This may admit an exact default-off CPU kernel profile only. Host timing is unisolated; controller promotion remains false and the retained live r137 authority is unchanged.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-signed-zero-sparse-feasibility-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_SIGNED_ZERO_SPARSE_FEASIBILITY_AUDIT.md").write_text(report)
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
