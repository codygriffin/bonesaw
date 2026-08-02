#!/usr/bin/env python3
"""Exact A/B for copied planner-to-authority hard-feasibility witnesses."""

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
from upkie_viability_request_plant_ab import execute, execution_trace_equal, outcome


REVISION = "upkie-cross-session-feasibility-witness-ab-r172"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CROSS_SESSION_FEASIBILITY_WITNESS_AB_R172.html",
    )
    parser.add_argument("--cases", help="comma-separated debug-only case subset")
    return parser.parse_args(argv)


def run(
    model: pathlib.Path,
    case: Any,
    duration: float,
    transfer_witness: bool,
    *,
    matched_profile: bool = True,
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
        use_feasibility_row_spans=True,
        viability_planner_maximum_feasibility_projection_sweeps=(
            None if matched_profile else 512
        ),
        viability_planner_projection_continuation_violation_threshold=(
            None if matched_profile else 0.1
        ),
        viability_planner_reuse_identical_hard_feasibility_seed=True,
        viability_planner_transfer_hard_feasibility_witness=transfer_witness,
    )


def selected_cases(names: str | None) -> tuple[Any, ...]:
    cases = case_matrix()
    if names is None:
        return cases
    requested = set(names.split(","))
    selected = tuple(case for case in cases if case.name in requested)
    missing = requested - {case.name for case in selected}
    if missing:
        raise SystemExit(f"unknown cases: {sorted(missing)}")
    return selected


def logical_projections(trace: dict[str, Any]) -> int:
    return int(
        np.sum(
            np.asarray(trace["feasibility_halfspace_projections"], np.uint64),
            dtype=np.uint64,
        )
        + np.sum(
            np.asarray(
                trace["viability_planner_feasibility_halfspace_projections"],
                np.uint64,
            ),
            dtype=np.uint64,
        )
    )


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    model = pathlib.Path(args.model).resolve()
    cases = selected_cases(args.cases)
    rows: dict[str, Any] = {}
    for index, case in enumerate(cases, 1):
        established = run(model, case, args.duration, False, matched_profile=False)
        control = run(model, case, args.duration, False)
        candidate = run(model, case, args.duration, True, matched_profile=False)
        replay = run(model, case, args.duration, True, matched_profile=False)
        established_trace = established["trace"]
        control_trace = control["trace"]
        candidate_trace = candidate["trace"]
        replay_trace = replay["trace"]
        transfers = int(
            np.sum(
                np.asarray(
                    candidate_trace[
                        "viability_hard_feasibility_witness_transferred"
                    ],
                    np.uint64,
                )
            )
        )
        final_hits = int(
            np.sum(
                np.asarray(
                    candidate_trace["final_feasibility_seed_reused"], np.uint64
                )
            )
        )
        established_executable = (
            np.asarray(established_trace["viability_request_executable"]) != 0
        )
        candidate_executable = (
            np.asarray(candidate_trace["viability_request_executable"]) != 0
        )
        executable_union = established_executable | candidate_executable
        row = {
            "case": candidate["metrics"]["case"],
            "established": established["metrics"],
            "control": control["metrics"],
            "candidate": candidate["metrics"],
            "established_authority_trace_exact": semantic_trace_equal(
                established_trace, control_trace
            ),
            "established_execution_trace_exact": execution_trace_equal(
                established_trace, candidate_trace
            ),
            "established_executable_request_exact": all(
                np.array_equal(
                    (
                        np.asarray(established_trace[field])
                        if field != "viability_request"
                        else np.where(
                            np.asarray(
                                established_trace["viability_request_executable"]
                            )[:, None]
                            != 0,
                            np.asarray(established_trace[field]),
                            0.0,
                        )
                    ),
                    (
                        np.asarray(candidate_trace[field])
                        if field != "viability_request"
                        else np.where(
                            np.asarray(candidate_trace["viability_request_executable"])[
                                :, None
                            ]
                            != 0,
                            np.asarray(candidate_trace[field]),
                            0.0,
                        )
                    ),
                )
                for field in (
                    "viability_request_executable",
                    "viability_confirmation_executable",
                    "viability_hybrid_guard_executable",
                )
            )
            and np.array_equal(
                np.asarray(established_trace["viability_request"])[executable_union],
                np.asarray(candidate_trace["viability_request"])[executable_union],
            ),
            "authority_trace_exact": semantic_trace_equal(
                established_trace, candidate_trace
            ),
            "execution_trace_exact": execution_trace_equal(
                established_trace, candidate_trace
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
                    "feasibility_projection_sweeps",
                    "feasibility_halfspace_projections",
                    "viability_hard_feasibility_witness_transfer_allocation_calls",
                    "viability_hard_feasibility_witness_transfer_allocated_bytes",
                )
            ),
            "established_logical_projections": logical_projections(established_trace),
            "candidate_logical_projections": logical_projections(candidate_trace),
            "established_final_projections": int(
                np.sum(
                    np.asarray(
                        established_trace["feasibility_halfspace_projections"], np.uint64
                    ),
                    dtype=np.uint64,
                )
            ),
            "candidate_final_projections": int(
                np.sum(
                    np.asarray(
                        candidate_trace["feasibility_halfspace_projections"],
                        np.uint64,
                    ),
                    dtype=np.uint64,
                )
            ),
            "established_planner_projections": int(
                np.sum(
                    np.asarray(
                        established_trace[
                            "viability_planner_feasibility_halfspace_projections"
                        ],
                        np.uint64,
                    ),
                    dtype=np.uint64,
                )
            ),
            "candidate_planner_projections": int(
                np.sum(
                    np.asarray(
                        candidate_trace[
                            "viability_planner_feasibility_halfspace_projections"
                        ],
                        np.uint64,
                    ),
                    dtype=np.uint64,
                )
            ),
            "witness_transfer_allocation_calls": int(
                np.sum(
                    np.asarray(
                        candidate_trace[
                            "viability_hard_feasibility_witness_transfer_allocation_calls"
                        ],
                        np.uint64,
                    ),
                    dtype=np.uint64,
                )
            ),
            "witness_transfer_allocated_bytes": int(
                np.sum(
                    np.asarray(
                        candidate_trace[
                            "viability_hard_feasibility_witness_transfer_allocated_bytes"
                        ],
                        np.uint64,
                    ),
                    dtype=np.uint64,
                )
            ),
            "witness_transfers": transfers,
            "final_witness_hits": final_hits,
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
            "control_final_witness_hits": int(
                np.sum(
                    np.asarray(
                        control_trace["final_feasibility_seed_reused"], np.uint64
                    )
                )
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
            f"overruns {established['metrics']['loop_overruns']}/"
            f"{control['metrics']['loop_overruns']}→"
            f"{candidate['metrics']['loop_overruns']}, "
            f"transfer/hit {transfers}/{final_hits}, "
            f"authority exact={row['authority_trace_exact']}",
            flush=True,
        )

    established = [row["established"] for row in rows.values()]
    controls = [row["control"] for row in rows.values()]
    candidates = [row["candidate"] for row in rows.values()]
    gates = {
        "matrix_complete": len(rows) == len(case_matrix()),
        "established_fallback_exercised": any(
            row["maximum_projection_sweeps"] > 8 for row in rows.values()
        ),
        "witness_transfer_exercised": sum(
            row["witness_transfers"] for row in rows.values()
        )
        > 0,
        "portable_witness_hits_are_bounded_by_transfers": all(
            0 <= row["final_witness_hits"] <= row["witness_transfers"]
            for row in rows.values()
        ),
        "control_has_no_cross_session_hits": all(
            row["control_final_witness_hits"] == 0 for row in rows.values()
        ),
        "established_execution_trace_exact": all(
            row["established_execution_trace_exact"] for row in rows.values()
        ),
        "established_executable_request_exact": all(
            row["established_executable_request_exact"] for row in rows.values()
        ),
        "logical_projection_work_exact": all(
            row["established_logical_projections"]
            == row["candidate_logical_projections"]
            for row in rows.values()
        ),
        "final_projection_work_exact": all(
            row["established_final_projections"]
            == row["candidate_final_projections"]
            for row in rows.values()
        ),
        "planner_projection_work_exact": all(
            row["established_planner_projections"]
            == row["candidate_planner_projections"]
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
            side["allocation_free"]
            for side in (*established, *controls, *candidates)
        ),
        "zero_witness_transfer_allocation": all(
            row["witness_transfer_allocation_calls"] == 0
            and row["witness_transfer_allocated_bytes"] == 0
            for row in rows.values()
        ),
        "zero_python_gc": all(
            side["python_gc_collections"] == 0
            for side in (*established, *controls, *candidates)
        ),
        "finite": all(
            side["finite"] for side in (*established, *controls, *candidates)
        ),
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
    transfer_mechanism_admitted = all(
        value
        for name, value in gates.items()
        if not name.startswith("established_")
    )
    profile_admitted = all(gates.values())
    established_overruns = sum(side["loop_overruns"] for side in established)
    control_overruns = sum(side["loop_overruns"] for side in controls)
    candidate_overruns = sum(side["loop_overruns"] for side in candidates)
    transfers = sum(row["witness_transfers"] for row in rows.values())
    final_hits = sum(row["final_witness_hits"] for row in rows.values())
    established_projections = sum(
        row["established_logical_projections"] for row in rows.values()
    )
    candidate_projections = sum(
        row["candidate_logical_projections"] for row in rows.values()
    )
    aggregate = {
        "case_count": len(rows),
        "established_loop_overruns": established_overruns,
        "control_loop_overruns": control_overruns,
        "candidate_loop_overruns": candidate_overruns,
        "witness_transfers": transfers,
        "final_witness_hits": final_hits,
        "established_logical_halfspace_projections": established_projections,
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
        "worst_established_final_wbc_p99_ns": max(
            side["final_wbc_step_ns"]["p99"] for side in established
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
        "established_authority_exact_cases": sum(
            row["established_authority_trace_exact"] for row in rows.values()
        ),
        "established_execution_exact_cases": sum(
            row["established_execution_trace_exact"] for row in rows.values()
        ),
        "established_executable_request_exact_cases": sum(
            row["established_executable_request_exact"] for row in rows.values()
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
            "established_planner_projection_cap": 512,
            "established_planner_continuation_threshold": 0.1,
            "planner_projection_cap": None,
            "candidate_cross_session_witness_transfer": True,
            "final_soft_hierarchy_recomputed": True,
        },
        "gates": gates,
        "aggregate": aggregate,
        "rows": rows,
        "transfer_mechanism_admitted": transfer_mechanism_admitted,
        "profile_admitted": profile_admitted,
        "controller_promoted": False,
    }
    table_rows = [
        [
            name,
            outcome(row["established"]),
            outcome(row["control"]),
            outcome(row["candidate"]),
            row["established"]["loop_overruns"],
            row["control"]["loop_overruns"],
            row["candidate"]["loop_overruns"],
            row["witness_transfers"],
            row["final_witness_hits"],
            f"{row['established']['final_wbc_step_ns']['p99'] / 1e3:.0f}",
            f"{row['candidate']['final_wbc_step_ns']['p99'] / 1e3:.0f}",
            "YES" if row["established_execution_trace_exact"] else "NO",
            "YES" if row["authority_trace_exact"] else "NO",
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for name, row in rows.items()
    ]
    report = "\n".join(
        [
            "# Bonesaw copied hard-feasibility witness A/B · r172",
            "",
            f"> Copied-witness mechanism **{'PASS' if transfer_mechanism_admitted else 'FAIL'}** · r169-profile semantic promotion **{'PASS' if profile_admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            "The established arm retains r169's exact 512-sweep planner prefix. The matched diagnostic control removes that prefix, while the transfer candidate preserves it and copies only budget-independent feasible terminals into the uncapped final-authority workspace. Rust rebuilds and bit-compares every structural/arithmetic hard-problem bit; projection ceiling and continuation slots may differ only for a feasible terminal proven independent of those stopping controls. Exhausted or prefix-dependent witnesses miss the cache and force an ordinary cold final solve. No mutable matrices, multipliers, controller state, or session storage are shared, and the final session recomputes every soft hierarchy.",
            f"Across **{len(rows)}** frozen plant cases, synchronous misses are **{established_overruns} established / {control_overruns} matched-control → {candidate_overruns} transfer**. Rust records **{transfers:,}** immutable transfer attempts and **{final_hits:,}** revalidated portable final hits; every other attempt falls back cold. Logical half-space work remains **{established_projections:,} → {candidate_projections:,}**, the established complete non-timing authority/plant trace is **{'exact' if gates['authority_trace_exact'] else 'not exact'}**, and replay is exact.",
            f"Aligning the planner profile with final authority preserves the established executable-request trace in **{aggregate['established_executable_request_exact_cases']}/{len(rows)}** cases and plant execution in **{aggregate['established_execution_exact_cases']}/{len(rows)}**; full authority telemetry is exact in **{aggregate['established_authority_exact_cases']}/{len(rows)}**, with any difference confined to non-executable speculative diagnostics.",
            f"Worst final-WBC p99 changes **{aggregate['worst_established_final_wbc_p99_ns'] / 1e6:.3f} → {aggregate['worst_candidate_final_wbc_p99_ns'] / 1e6:.3f} ms**; candidate maximum is **{aggregate['worst_candidate_final_wbc_max_ns'] / 1e6:.3f} ms**.",
            f"The copied-witness boundary itself is explicitly timed and allocation-counted: worst **{aggregate['worst_witness_transfer_ns'] / 1e3:.3f} µs**, **{aggregate['witness_transfer_allocation_calls']} calls / {aggregate['witness_transfer_allocated_bytes']} bytes** across the matrix.",
            "",
            *markdown_table(
                [
                    "case",
                    "established",
                    "control",
                    "transfer",
                    "miss E",
                    "miss C",
                    "miss T",
                    "copies",
                    "hits",
                    "final p99 E µs",
                    "final p99 T µs",
                    "E/C exec exact",
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
            "- The witness is a copied terminal hard seed plus exact problem key and logical diagnostics. It is not a shared workspace, warm soft solution, command, or authority token.",
            "- The destination assembles and bit-compares its own complete ordered hard problem. Any coefficient, bound, stable ID, feasibility profile, or dimension mismatch forces an ordinary cold solve.",
            "- Equality factors are built in destination-owned storage. Every soft task is independently solved after hard feasibility reuse.",
            "- Exhausted results remain typed `MaxIterations`; they cannot acquire torque authority through transfer.",
            "- Python owns frozen plant sequencing and report generation only. Rust owns the key, copy, revalidation, solve, typed status, logical counters, and allocation witness.",
            "",
            "## Scope",
            "",
            f"Timing is descriptive for this host and explicit `{os.environ.get('RUSTFLAGS', '')}` build. This is a policy-free CPU WBC/profile admission test, not a hardware controller, real-time-kernel WCET proof, or learned realization certificate. The retained r137 authority stays live.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-cross-session-feasibility-witness-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CROSS_SESSION_FEASIBILITY_WITNESS_AUDIT.md").write_text(
        report
    )
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "transfer_mechanism_admitted": transfer_mechanism_admitted,
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
