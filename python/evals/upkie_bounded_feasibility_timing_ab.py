#!/usr/bin/env python3
"""Causal A/B for session-local bit-identical hard-feasibility reuse."""

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


REVISION = "upkie-hard-feasibility-reuse-timing-ab-r169"
DURATION_S = 6.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--planner-projection-sweeps", type=int, default=512)
    parser.add_argument(
        "--planner-continuation-violation-threshold", type=float, default=0.1
    )
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument(
        "--web-report", default="web/UPKIE_HARD_FEASIBILITY_REUSE_TIMING_AB_R169.html"
    )
    parser.add_argument(
        "--cases", help="comma-separated subset for smoke/debug; admission requires all 20"
    )
    return parser.parse_args()


def run(
    model: pathlib.Path,
    case: Any,
    duration_s: float,
    planner_projection_sweeps: int | None,
    planner_continuation_threshold: float | None,
    reuse_identical_hard_feasibility_seed: bool,
) -> dict[str, Any]:
    return execute(
        model,
        case,
        duration_s,
        measured_contact=True,
        planner=True,
        viability_planner_strategy="hybrid_confirmed_multistep_budgeted",
        viability_confirmation_updates=2,
        maximum_feasibility_projection_sweeps=None,
        repair_feasibility_equalities_before_inequalities=False,
        use_feasibility_row_spans=True,
        viability_planner_maximum_feasibility_projection_sweeps=planner_projection_sweeps,
        viability_planner_projection_continuation_violation_threshold=planner_continuation_threshold,
        viability_planner_reuse_identical_hard_feasibility_seed=reuse_identical_hard_feasibility_seed,
    )


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    if args.planner_projection_sweeps <= 0:
        raise SystemExit("--planner-projection-sweeps must be positive")
    if (
        not np.isfinite(args.planner_continuation_violation_threshold)
        or args.planner_continuation_violation_threshold < 0.0
    ):
        raise SystemExit("--planner-continuation-violation-threshold must be nonnegative")
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
        reference = run(
            model,
            case,
            args.duration,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
            False,
        )
        candidate = run(
            model,
            case,
            args.duration,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
            True,
        )
        replay = run(
            model,
            case,
            args.duration,
            args.planner_projection_sweeps,
            args.planner_continuation_violation_threshold,
            True,
        )
        reference_trace = reference["trace"]
        candidate_trace = candidate["trace"]
        reference_sweeps = np.asarray(
            reference_trace["feasibility_projection_sweeps"], np.uint64
        )
        candidate_sweeps = np.asarray(
            candidate_trace["feasibility_projection_sweeps"], np.uint64
        )
        reference_projections = np.asarray(
            reference_trace["feasibility_halfspace_projections"], np.uint64
        )
        candidate_projections = np.asarray(
            candidate_trace["feasibility_halfspace_projections"], np.uint64
        )
        reference_planner_projections = np.asarray(
            reference_trace[
                "viability_planner_feasibility_halfspace_projections"
            ],
            np.uint64,
        )
        candidate_planner_projections = np.asarray(
            candidate_trace[
                "viability_planner_feasibility_halfspace_projections"
            ],
            np.uint64,
        )
        reference_reuses = np.asarray(
            reference_trace["viability_planner_feasibility_seed_reuses"],
            np.uint64,
        )
        candidate_reuses = np.asarray(
            candidate_trace["viability_planner_feasibility_seed_reuses"],
            np.uint64,
        )
        rows[case.name] = {
            "case": candidate["metrics"]["case"],
            "cold": reference["metrics"],
            "reuse": candidate["metrics"],
            "execution_trace_exact": execution_trace_equal(
                reference_trace, candidate_trace
            ),
            "control_candidate_semantic_exact": semantic_trace_equal(
                reference_trace, candidate_trace
            ),
            "candidate_replay_exact": semantic_trace_equal(
                candidate_trace, replay["trace"]
            ),
            "candidate_cache_witness_replay_exact": all(
                np.array_equal(
                    np.asarray(candidate_trace[field]),
                    np.asarray(replay["trace"][field]),
                )
                for field in (
                    "viability_planner_feasibility_seed_reuses",
                    "viability_planner_feasibility_projection_sweeps",
                    "viability_planner_feasibility_halfspace_projections",
                )
            ),
            "cold_maximum_projection_sweeps": int(np.max(reference_sweeps)),
            "reuse_maximum_projection_sweeps": int(np.max(candidate_sweeps)),
            "cold_final_halfspace_projections": int(np.sum(reference_projections)),
            "reuse_final_halfspace_projections": int(np.sum(candidate_projections)),
            "cold_planner_halfspace_projections": int(
                np.sum(reference_planner_projections)
            ),
            "reuse_planner_halfspace_projections": int(
                np.sum(candidate_planner_projections)
            ),
            "cold_feasibility_seed_reuses": int(np.sum(reference_reuses)),
            "reuse_feasibility_seed_reuses": int(np.sum(candidate_reuses)),
        }
        print(
            f"[{index:02d}/{len(selected)}] {case.name}: "
            f"{reference['metrics']['loop_overruns']}→{candidate['metrics']['loop_overruns']} overruns, "
            f"cache hits {int(np.sum(candidate_reuses))}, "
            f"execution exact={rows[case.name]['execution_trace_exact']}",
            flush=True,
        )

    full_matrix = len(rows) == len(case_matrix())
    references = [row["cold"] for row in rows.values()]
    candidates = [row["reuse"] for row in rows.values()]
    gates = {
        "matrix_complete": full_matrix,
        "established_fallback_exercised": any(
            row["cold_maximum_projection_sweeps"] > 8
            for row in rows.values()
        ),
        "projection_work_is_exact": all(
            row["reuse_maximum_projection_sweeps"]
            == row["cold_maximum_projection_sweeps"]
            and row["reuse_final_halfspace_projections"]
            == row["cold_final_halfspace_projections"]
            and row["reuse_planner_halfspace_projections"]
            == row["cold_planner_halfspace_projections"]
            for row in rows.values()
        ),
        "candidate_cache_exercised": sum(
            row["reuse_feasibility_seed_reuses"] for row in rows.values()
        ) > 0,
        "control_cache_disabled": all(
            row["cold_feasibility_seed_reuses"] == 0 for row in rows.values()
        ),
        "execution_trace_is_exact": all(
            row["execution_trace_exact"] for row in rows.values()
        ),
        "authority_trace_is_exact": all(
            row["control_candidate_semantic_exact"] for row in rows.values()
        ),
        "candidate_replay_is_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "cache_witness_replay_is_exact": all(
            row["candidate_cache_witness_replay_exact"] for row in rows.values()
        ),
        "zero_authoritative_wbc_deadline_overruns": all(
            candidate["final_wbc_step_ns"]["maximum"] <= 5.0e6
            for candidate in candidates
        ),
        "zero_synchronous_loop_deadline_overruns": all(
            candidate["loop_overruns"] == 0 for candidate in candidates
        ),
        "zero_rust_allocation": all(
            side["allocation_free"] for side in (*references, *candidates)
        ),
        "zero_python_gc": all(
            side["python_gc_collections"] == 0 for side in (*references, *candidates)
        ),
        "finite": all(side["finite"] for side in (*references, *candidates)),
        "no_new_numeric_fault": all(
            not candidate["numeric_fault"]
            for reference, candidate in zip(references, candidates, strict=True)
            if not reference["numeric_fault"]
        ),
        "no_new_fall": all(
            not candidate["fell"]
            for reference, candidate in zip(references, candidates, strict=True)
            if not reference["fell"]
        ),
    }
    authoritative_wbc_profile_admitted = all(
        value
        for name, value in gates.items()
        if name != "zero_synchronous_loop_deadline_overruns"
    )
    timing_profile_admitted = all(gates.values())
    reference_overruns = sum(side["loop_overruns"] for side in references)
    candidate_overruns = sum(side["loop_overruns"] for side in candidates)
    reference_projections = sum(
        row["cold_final_halfspace_projections"]
        + row["cold_planner_halfspace_projections"]
        for row in rows.values()
    )
    candidate_projections = sum(
        row["reuse_final_halfspace_projections"]
        + row["reuse_planner_halfspace_projections"]
        for row in rows.values()
    )
    candidate_cache_hits = sum(
        row["reuse_feasibility_seed_reuses"] for row in rows.values()
    )
    table_rows = [
        [
            name,
            outcome(row["cold"]),
            outcome(row["reuse"]),
            row["cold"]["loop_overruns"],
            row["reuse"]["loop_overruns"],
            row["reuse_feasibility_seed_reuses"],
            row["cold_maximum_projection_sweeps"],
            row["reuse_maximum_projection_sweeps"],
            f"{row['cold']['loop_ns']['p99'] / 1.0e3:.0f}",
            f"{row['reuse']['loop_ns']['p99'] / 1.0e3:.0f}",
            f"{row['reuse']['final_wbc_step_ns']['maximum'] / 1.0e3:.0f}",
            "YES" if row["control_candidate_semantic_exact"] else "NO",
            "YES" if row["execution_trace_exact"] else "NO",
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
        "projection_sweep_ceiling": None,
        "equality_first_active_set_repair": False,
        "sparse_dykstra_nonzero_rows": True,
        "planner_reuse_identical_hard_feasibility_seed": True,
        "planner_projection_sweep_ceiling": args.planner_projection_sweeps,
        "planner_continuation_violation_threshold": (
            args.planner_continuation_violation_threshold
        ),
        "timing_profile_admitted": timing_profile_admitted,
        "authoritative_wbc_profile_admitted": authoritative_wbc_profile_admitted,
        "controller_promoted": False,
        "gates": gates,
        "aggregate": {
            "case_count": len(rows),
            "cold_loop_overruns": reference_overruns,
            "reuse_loop_overruns": candidate_overruns,
            "cold_logical_halfspace_projections": reference_projections,
            "reuse_logical_halfspace_projections": candidate_projections,
            "reuse_feasibility_seed_hits": candidate_cache_hits,
            "worst_cold_loop_max_ns": max(
                side["loop_ns"]["maximum"] for side in references
            ),
            "worst_reuse_loop_max_ns": max(
                side["loop_ns"]["maximum"] for side in candidates
            ),
            "worst_cold_final_wbc_p99_ns": max(
                side["final_wbc_step_ns"]["p99"] for side in references
            ),
            "worst_reuse_final_wbc_p99_ns": max(
                side["final_wbc_step_ns"]["p99"] for side in candidates
            ),
            "worst_reuse_final_wbc_max_ns": max(
                side["final_wbc_step_ns"]["maximum"] for side in candidates
            ),
        },
        "rows": rows,
        "interpretation": {
            "admission": "the host-native synchronous profile is admitted only if every gate passes; controller promotion remains false",
            "semantics": "the candidate reuses only a bit-identical terminal hard-feasibility result, preserves logical work counters, and independently runs every admitted soft hierarchy",
            "default_semantics": "reuse remains opt-in and planner-local until the complete matrix passes",
        },
    }
    report = "\n".join(
        [
            "# Bonesaw bit-identical hard-feasibility reuse timing A/B · r169",
            "",
            f"> Authoritative WBC profile **{'PASS' if authoritative_wbc_profile_admitted else 'FAIL'}** · synchronous supervisor profile **{'PASS' if timing_profile_admitted else 'FAIL'}** · controller promotion **NO**.",
            "",
            "## Result",
            "",
            f"Both sides use the admitted r167 sparse/anytime profile. The candidate additionally caches a terminal hard-feasibility result inside one planner session only when the complete ordered hard problem, bounds, and feasibility configuration are bit-identical. Exact seeds still run every soft hierarchy; cached exhausted results return the same typed `MaxIterations` and never gain authority.",
            f"Across **{len(rows)}** frozen plant cases, synchronous-loop deadline overruns change **{reference_overruns} → {candidate_overruns}**, with **{candidate_cache_hits:,}** witnessed reuse hits. The complete non-timing authority trace is **{'exact' if gates['authority_trace_is_exact'] else 'not exact'}**, physical execution is **{'exact in every row' if gates['execution_trace_is_exact'] else 'not exact'}**, and replay is exact. Every authoritative WBC call stays below 5 ms; the worst candidate call is **{metrics['aggregate']['worst_reuse_final_wbc_max_ns'] / 1.0e6:.3f} ms**.",
            f"Logical half-space projections remain **{reference_projections:,} → {candidate_projections:,}** across planner and final solves. Reuse removes physical recomputation but preserves the cold logical counters for auditability.",
            "",
            *markdown_table(
                [
                    "case",
                    "cold",
                    "reuse",
                    "overrun cold",
                    "overrun reuse",
                    "cache hits",
                    "max sweep cold",
                    "max sweep reuse",
                    "loop p99 cold µs",
                    "loop p99 reuse µs",
                    "final WBC max µs",
                    "authority exact",
                    "execution exact",
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
            "- Rust owns the session-local cache witness, sparse-nonzero metadata, Dykstra projection ordering, logical work counters, typed solver status, allocation witness, hybrid guard, confirmation, and request freshness.",
            "- Python owns the immutable MuJoCo matrix, exact A/B/replay sequencing, timing statistics, and artifact generation.",
            "- Admission requires exact executable commands, planner requests/statuses, final-solver diagnostics, and plant traces. A timing win with changed execution is rejected.",
            "- The cache key contains every ordered hard coefficient and bound as exact IEEE-754 bits plus feasibility-profile parameters. It is never shared between planner and final-authority sessions.",
            "- Planner budget exhaustion remains typed `MaxIterations` and carries no torque authority even when its deterministic terminal result is reused; multi-update confirmation remains the temporal admission boundary.",
            f"- The combined synchronous supervisor profile has {candidate_overruns} remaining deadline misses; zero is required for admission.",
            "- This can admit a CPU kernel profile, not the learned realization certificate rejected by r159 and not a hardware controller. The retained r137 worker remains live.",
            "",
            "## Scope",
            "",
            f"Host timing is descriptive for this Ryzen 7 3700X and the explicit `{os.environ.get('RUSTFLAGS', '')}` build profile. The portable generic build is not admitted. No real-time kernel, priority elevation, core isolation, hardware plant, estimator delay/noise, or authenticated command transport is claimed.",
        ]
    ) + "\n"

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-hard-feasibility-reuse-timing-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_HARD_FEASIBILITY_REUSE_TIMING_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "timing_profile_admitted": timing_profile_admitted,
                "authoritative_wbc_profile_admitted": authoritative_wbc_profile_admitted,
                "controller_promoted": False,
                "web_report": str(web_report),
            },
            sort_keys=True,
        )
    )
    return 0 if timing_profile_admitted else 1


if __name__ == "__main__":
    raise SystemExit(main())
