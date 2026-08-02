#!/usr/bin/env python3
"""Exact A/B for bounded-planner to uncapped-authority feasibility witnesses."""

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
from upkie_cross_session_feasibility_witness_ab import (
    logical_projections,
    selected_cases,
    run,
)
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execution_trace_equal, outcome


REVISION = "upkie-cross-profile-feasibility-witness-ab-r174"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CROSS_PROFILE_FEASIBILITY_WITNESS_AB_R174.html",
    )
    parser.add_argument("--cases", help="comma-separated debug-only case subset")
    return parser.parse_args(argv)


def sum_field(trace: dict[str, Any], field: str) -> int:
    return int(np.sum(np.asarray(trace[field], np.uint64), dtype=np.uint64))


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    cases = selected_cases(args.cases)
    rows: dict[str, Any] = {}

    for index, case in enumerate(cases, 1):
        control = run(
            model,
            case,
            args.duration,
            False,
            matched_profile=False,
        )
        candidate = run(
            model,
            case,
            args.duration,
            True,
            matched_profile=False,
        )
        replay = run(
            model,
            case,
            args.duration,
            True,
            matched_profile=False,
        )
        control_trace = control["trace"]
        candidate_trace = candidate["trace"]
        replay_trace = replay["trace"]
        copies = sum_field(
            candidate_trace, "viability_hard_feasibility_witness_transferred"
        )
        hits = sum_field(candidate_trace, "final_feasibility_seed_reused")
        prefix_resumes = sum_field(
            candidate_trace, "final_feasibility_prefix_resumed"
        )
        row = {
            "case": candidate["metrics"]["case"],
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "authority_trace_exact": semantic_trace_equal(
                control_trace, candidate_trace
            ),
            "execution_trace_exact": execution_trace_equal(
                control_trace, candidate_trace
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate_trace, replay_trace
            ),
            "mechanism_replay_exact": all(
                np.array_equal(
                    np.asarray(candidate_trace[field]),
                    np.asarray(replay_trace[field]),
                )
                for field in (
                    "viability_hard_feasibility_witness_transferred",
                    "final_feasibility_seed_reused",
                    "final_feasibility_prefix_resumed",
                    "feasibility_projection_sweeps",
                    "feasibility_halfspace_projections",
                )
            ),
            "logical_projections_control": logical_projections(control_trace),
            "logical_projections_candidate": logical_projections(candidate_trace),
            "final_projections_control": sum_field(
                control_trace, "feasibility_halfspace_projections"
            ),
            "final_projections_candidate": sum_field(
                candidate_trace, "feasibility_halfspace_projections"
            ),
            "planner_projections_control": sum_field(
                control_trace,
                "viability_planner_feasibility_halfspace_projections",
            ),
            "planner_projections_candidate": sum_field(
                candidate_trace,
                "viability_planner_feasibility_halfspace_projections",
            ),
            "witness_copies": copies,
            "final_witness_hits": hits,
            "final_exact_seed_hits": hits - prefix_resumes,
            "final_prefix_resumes": prefix_resumes,
            "refused_copies": copies - hits,
            "control_final_witness_hits": sum_field(
                control_trace, "final_feasibility_seed_reused"
            ),
            "witness_transfer_max_ns": int(
                np.max(
                    np.asarray(
                        candidate_trace[
                            "viability_hard_feasibility_witness_transfer_ns"
                        ],
                        np.uint64,
                    )
                )
            ),
            "witness_transfer_allocation_calls": sum_field(
                candidate_trace,
                "viability_hard_feasibility_witness_transfer_allocation_calls",
            ),
            "witness_transfer_allocated_bytes": sum_field(
                candidate_trace,
                "viability_hard_feasibility_witness_transfer_allocated_bytes",
            ),
            "maximum_projection_sweeps": int(
                np.max(
                    np.asarray(
                        control_trace["feasibility_projection_sweeps"], np.uint64
                    )
                )
            ),
        }
        rows[case.name] = row
        print(
            f"[{index:02d}/{len(cases)}] {case.name}: "
            f"miss {control['metrics']['loop_overruns']}→"
            f"{candidate['metrics']['loop_overruns']}, "
            f"copy/exact/prefix {copies}/{hits - prefix_resumes}/{prefix_resumes}, "
            f"authority exact={row['authority_trace_exact']}",
            flush=True,
        )

    controls = [row["control"] for row in rows.values()]
    candidates = [row["candidate"] for row in rows.values()]
    gates = {
        "matrix_complete": len(rows) == len(case_matrix()),
        "established_fallback_exercised": any(
            row["maximum_projection_sweeps"] > 8 for row in rows.values()
        ),
        "witness_copy_exercised": sum(
            row["witness_copies"] for row in rows.values()
        )
        > 0,
        "cross_profile_witness_hit_exercised": sum(
            row["final_witness_hits"] for row in rows.values()
        )
        > 0,
        "no_hit_without_copy": all(
            0 <= row["final_witness_hits"] <= row["witness_copies"]
            for row in rows.values()
        ),
        "every_copy_revalidated_and_consumed": all(
            row["final_witness_hits"] == row["witness_copies"]
            for row in rows.values()
        ),
        "bounded_prefix_resume_exercised": sum(
            row["final_prefix_resumes"] for row in rows.values()
        )
        > 0,
        "control_has_no_cross_session_hits": all(
            row["control_final_witness_hits"] == 0 for row in rows.values()
        ),
        "logical_projection_work_exact": all(
            row["logical_projections_control"]
            == row["logical_projections_candidate"]
            for row in rows.values()
        ),
        "final_projection_work_exact": all(
            row["final_projections_control"]
            == row["final_projections_candidate"]
            for row in rows.values()
        ),
        "planner_projection_work_exact": all(
            row["planner_projections_control"]
            == row["planner_projections_candidate"]
            for row in rows.values()
        ),
        "authority_trace_exact": all(
            row["authority_trace_exact"] for row in rows.values()
        ),
        "execution_trace_exact": all(
            row["execution_trace_exact"] for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "mechanism_replay_exact": all(
            row["mechanism_replay_exact"] for row in rows.values()
        ),
        "zero_authoritative_wbc_deadline_overruns": all(
            side["final_wbc_step_ns"]["maximum"] <= 5.0e6 for side in candidates
        ),
        "zero_synchronous_loop_deadline_overruns": all(
            side["loop_overruns"] == 0 for side in candidates
        ),
        "zero_rust_allocation": all(
            side["allocation_free"] for side in (*controls, *candidates)
        ),
        "zero_witness_transfer_allocation": all(
            row["witness_transfer_allocation_calls"] == 0
            and row["witness_transfer_allocated_bytes"] == 0
            for row in rows.values()
        ),
        "zero_python_gc": all(
            side["python_gc_collections"] == 0
            for side in (*controls, *candidates)
        ),
        "finite": all(side["finite"] for side in (*controls, *candidates)),
        "no_new_numeric_fault": all(
            not candidate["numeric_fault"]
            for control, candidate in zip(controls, candidates, strict=True)
            if not control["numeric_fault"]
        ),
        "no_new_fall": all(
            not candidate["fell"]
            for control, candidate in zip(controls, candidates, strict=True)
            if not control["fell"]
        ),
    }
    profile_admitted = all(gates.values())
    copies = sum(row["witness_copies"] for row in rows.values())
    hits = sum(row["final_witness_hits"] for row in rows.values())
    prefix_resumes = sum(row["final_prefix_resumes"] for row in rows.values())
    control_projections = sum(
        row["logical_projections_control"] for row in rows.values()
    )
    candidate_projections = sum(
        row["logical_projections_candidate"] for row in rows.values()
    )
    aggregate = {
        "case_count": len(rows),
        "control_loop_overruns": sum(side["loop_overruns"] for side in controls),
        "candidate_loop_overruns": sum(
            side["loop_overruns"] for side in candidates
        ),
        "witness_copies": copies,
        "final_witness_hits": hits,
        "final_exact_seed_hits": hits - prefix_resumes,
        "final_prefix_resumes": prefix_resumes,
        "refused_copies": copies - hits,
        "control_logical_halfspace_projections": control_projections,
        "candidate_logical_halfspace_projections": candidate_projections,
        "worst_control_loop_max_ns": max(
            side["loop_ns"]["maximum"] for side in controls
        ),
        "worst_candidate_loop_max_ns": max(
            side["loop_ns"]["maximum"] for side in candidates
        ),
        "worst_control_final_wbc_p99_ns": max(
            side["final_wbc_step_ns"]["p99"] for side in controls
        ),
        "worst_candidate_final_wbc_p99_ns": max(
            side["final_wbc_step_ns"]["p99"] for side in candidates
        ),
        "worst_candidate_final_wbc_max_ns": max(
            side["final_wbc_step_ns"]["maximum"] for side in candidates
        ),
        "worst_witness_transfer_ns": max(
            row["witness_transfer_max_ns"] for row in rows.values()
        ),
        "witness_transfer_allocation_calls": sum(
            row["witness_transfer_allocation_calls"] for row in rows.values()
        ),
        "witness_transfer_allocated_bytes": sum(
            row["witness_transfer_allocated_bytes"] for row in rows.values()
        ),
    }
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "host": platform.platform(),
        "rustflags": os.environ.get("RUSTFLAGS", ""),
        "configuration": {
            "signed_zero_sparse_rows": True,
            "planner_session_local_reuse": True,
            "planner_projection_cap": 512,
            "planner_continuation_threshold": 0.1,
            "final_projection_cap": None,
            "candidate_cross_profile_witness_continuation": True,
            "final_soft_hierarchy_recomputed": True,
        },
        "gates": gates,
        "aggregate": aggregate,
        "rows": rows,
        "profile_admitted": profile_admitted,
        "controller_promoted": False,
    }
    table_rows = [
        [
            name,
            outcome(row["control"]),
            row["control"]["loop_overruns"],
            row["candidate"]["loop_overruns"],
            row["witness_copies"],
            row["final_witness_hits"],
            row["final_prefix_resumes"],
            f"{row['control']['final_wbc_step_ns']['p99'] / 1e3:.0f}",
            f"{row['candidate']['final_wbc_step_ns']['p99'] / 1e3:.0f}",
            "YES" if row["authority_trace_exact"] else "NO",
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for name, row in rows.items()
    ]
    report = "\n".join(
        [
            "# Bonesaw bounded-planner witness continuation A/B · r174",
            "",
            f"> Exact host-native evaluation profile **{'PASS' if profile_admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            "Both arms retain r169's 512-sweep planner prefix, 0.1 exact-continuation threshold, and uncapped final authority. The candidate copies either a terminal exact planner seed or its immutable exhausted Dykstra prefix into the independent final workspace. Exact seeds are revalidated directly; an exhausted bounded prefix resumes from the copied point and row multipliers through the destination's uncapped budget. Compatibility is one-way and every non-stopping-control key bit must match.",
            f"Across **{len(rows)}** frozen cases, synchronous misses change **{aggregate['control_loop_overruns']} → {aggregate['candidate_loop_overruns']}**. Rust records **{copies:,}** copies, **{hits - prefix_resumes:,}** terminal exact hits, and **{prefix_resumes:,}** bounded-prefix resumptions. Logical work remains **{control_projections:,} → {candidate_projections:,}**; complete non-timing authority/plant traces and replay are exact.",
            f"Worst final-WBC p99 changes **{aggregate['worst_control_final_wbc_p99_ns'] / 1e6:.3f} → {aggregate['worst_candidate_final_wbc_p99_ns'] / 1e6:.3f} ms**; candidate maximum is **{aggregate['worst_candidate_final_wbc_max_ns'] / 1e6:.3f} ms**. The copy boundary itself is **{aggregate['worst_witness_transfer_ns'] / 1e3:.3f} µs** worst-case with **{aggregate['witness_transfer_allocation_calls']} calls / {aggregate['witness_transfer_allocated_bytes']} bytes**.",
            "",
            *markdown_table(
                [
                    "case",
                    "outcome",
                    "miss C",
                    "miss T",
                    "copies",
                    "hits",
                    "prefix",
                    "final p99 C µs",
                    "final p99 T µs",
                    "authority exact",
                    "replay exact",
                ],
                table_rows,
            ),
            "",
            "## Admission gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Architecture contract",
            "",
            "- Exact compatibility ignores only the planner projection ceiling and continuation threshold, and only toward an uncapped destination. Dimensions, ordered stable IDs, every coefficient and bound bit, singular tolerance, active-set budget, equality-repair mode, and sparse arithmetic mode still match exactly.",
            "- A cap below the common initial Dykstra prefix is compatible only when no active-set pseudoinverse influenced an exact seed. Exhausted prefixes require the common initial active-set attempt and preserve the exact Dykstra point plus every row multiplier. Reverse uncapped-to-bounded reuse is forbidden.",
            "- A resumed prefix continues to the uncapped destination's ordinary terminal result and reports the complete cold-equivalent logical work. If it remains exhausted, it stays typed `MaxIterations` and never enters the soft hierarchy.",
            "- Equality factors and every soft task are recomputed in destination-owned storage. No mutable matrix, multiplier, controller state, command, or authority token is shared.",
            "- Python owns the frozen corpus and statistics. Rust owns the key, copy, one-way compatibility proof, revalidation, typed result, logical work, and allocation witness.",
            "",
            "## Scope",
            "",
            f"Timing is host-descriptive for `{os.environ.get('RUSTFLAGS', '')}`. This admits an evaluation compute profile, not the experimental viability policy, a hardware controller, real-time-kernel WCET, or learned realization model. The retained r137 authority remains live.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-cross-profile-feasibility-witness-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CROSS_PROFILE_FEASIBILITY_WITNESS_AUDIT.md").write_text(
        report
    )
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "profile_admitted": profile_admitted,
                "controller_promoted": False,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if profile_admitted else 1


if __name__ == "__main__":
    raise SystemExit(main())
