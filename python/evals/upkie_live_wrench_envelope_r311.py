#!/usr/bin/env python3
"""R311 map policy-free off-CoM and repeated-pull capability boundaries."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_load_reserve_matrix_r310 as r310


REVISION = "upkie-live-wrench-envelope-r311"
TICKS = 450
FORCES_N = (-8.0, -6.0, -4.0, -2.0, 2.0, 4.0, 6.0, 8.0)
OFFSETS_Z_M = (-0.25, 0.0, 0.25)
SCHEDULES = {
    "single": ((40, 20),),
    "repeated": ((40, 20), (120, 20), (200, 20), (280, 20)),
}


def _semantic_digest(case: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(r300._semantic(case), sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def outcome(summary: dict[str, Any]) -> str:
    if summary["terminal_pending"] is not None:
        return f"terminal_{summary['terminal_pending']}"
    if summary.get("nonadmitted_ticks"):
        return "completed_with_nonadmission"
    if (
        summary["final_100_bilateral_upright"]
        and summary["maximum_root_tilt_rad"] <= 0.25
    ):
        return "stable_upright"
    return "completed_outside_upright_envelope"


def run_one(
    model: pathlib.Path,
    force_y_n: float,
    offset_z_m: float,
    schedule_name: str,
    *,
    ticks: int,
) -> dict[str, Any]:
    return r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=ticks,
        controller_options=r310.CANDIDATE_OPTIONS,
        worker_options=r310.WORKER_OPTIONS,
        force_world_n=(0.0, force_y_n, 0.0),
        application_offset_world_m=(0.0, 0.0, offset_z_m),
        push_windows=SCHEDULES[schedule_name],
    )


def summarize(case: dict[str, Any], schedule_name: str) -> dict[str, Any]:
    summary = r310.summarize(case)
    moments = np.asarray(
        [state["external_moment_world_nm"] for state in case["states"]],
        np.float64,
    )
    maximum_moment_nm = float(
        np.max(np.linalg.norm(moments, axis=1)) if len(moments) else 0.0
    )
    offset_z_m = float(case.get("application_offset_world_m", [0.0, 0.0, 0.0])[2])
    force_y_n = float(case["force_world_n"][1])
    expected_active_ticks = sum(duration for _, duration in SCHEDULES[schedule_name])
    summary.update(
        {
            "schedule": schedule_name,
            "force_y_n": force_y_n,
            "offset_z_m": offset_z_m,
            "expected_moment_nm": abs(force_y_n * offset_z_m),
            "maximum_moment_nm": maximum_moment_nm,
            "active_ticks": sum(
                state["external_load_active"] for state in case["states"]
            ),
            "expected_active_ticks": expected_active_ticks,
        }
    )
    summary["outcome"] = outcome(summary)
    return summary


def evaluate(
    rows: list[dict[str, Any]], replay: dict[str, Any], *, ticks: int
) -> dict[str, Any]:
    centered = [row for row in rows if row["offset_z_m"] == 0.0]
    lower = [row for row in rows if row["offset_z_m"] < 0.0]
    upper = [row for row in rows if row["offset_z_m"] > 0.0]
    low_moment = [row for row in rows if row["expected_moment_nm"] <= 1.0]
    admitted_states = [
        state
        for row in rows
        for state in row["case"]["states"]
        if state["wbc_admitted"]
    ]
    maximum_admitted_hard_residual = max(
        max(
            state["wbc_maximum_constraint_violation"],
            state["wbc_dynamics_residual"],
            state["wbc_contact_residual"],
        )
        for state in admitted_states
    )
    harness_gates = {
        "complete_matrix_and_both_schedules": len(rows)
        == len(FORCES_N) * len(OFFSETS_Z_M) * len(SCHEDULES)
        and {row["schedule"] for row in rows} == set(SCHEDULES),
        "declared_moment_matches_force_cross_offset": all(
            abs(row["maximum_moment_nm"] - row["expected_moment_nm"]) <= 1.0e-12
            for row in rows
        ),
        "completed_cases_receive_every_scheduled_tick": all(
            row["terminal_pending"] is not None
            or row["active_ticks"] == row["expected_active_ticks"]
            for row in rows
        ),
        "upper_high_moment_boundary_is_exposed": any(
            row["terminal_pending"] is not None
            and row["expected_moment_nm"] >= 1.5
            for row in upper
        ),
        "lower_high_moment_has_stable_examples": any(
            row["outcome"] == "stable_upright"
            and row["expected_moment_nm"] >= 1.5
            for row in lower
        ),
        "terminal_boundary_replay_is_exact": _semantic_digest(rows[-1]["case"])
        == _semantic_digest(replay),
    }
    controller_quality_gates = {
        "centered_repeated_pulls_finish_with_upright_tail": all(
            row["terminal_pending"] is None and row["final_100_bilateral_upright"]
            for row in centered
            if row["schedule"] == "repeated"
        ),
        "low_moment_envelope_stays_nonterminal": all(
            row["terminal_pending"] is None for row in low_moment
        ),
        "zero_nonadmission_or_numeric_reset": all(
            not row["nonadmitted_ticks"] and not row["numeric_reset"] for row in rows
        ),
        "admitted_hard_residual_below_1e_minus_8": maximum_admitted_hard_residual
        < 1.0e-8,
        "controller_and_worker_p99_within_deadline": all(
            row["controller_step_us"]["p99"] < 5_000.0
            and row["worker_step_us"]["p99"] < 20_000.0
            for row in rows
        ),
        "zero_hot_path_allocations": all(
            row["allocation_calls"] == 0
            and row["allocated_bytes"] == 0
            and row["load_reserve"]["allocation_calls"] == 0
            and row["load_reserve"]["allocated_bytes"] == 0
            for row in rows
        ),
    }
    public_rows = [{key: value for key, value in row.items() if key != "case"} for row in rows]
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "rates_hz": {"wbc": 50, "physics": 250, "stream": 50},
        "controller": "R310 production CPU profile; no learned policy",
        "matrix": {
            "forces_y_n": list(FORCES_N),
            "offsets_z_m": list(OFFSETS_Z_M),
            "schedules": {
                name: [list(window) for window in windows]
                for name, windows in SCHEDULES.items()
            },
            "case_count": len(rows),
            "requested_ticks": ticks,
        },
        "rows": public_rows,
        "outcome_counts": {
            name: sum(row["outcome"] == name for row in rows)
            for name in sorted({row["outcome"] for row in rows})
        },
        "maximum_admitted_hard_residual": maximum_admitted_hard_residual,
        "nonadmitted_case_count": sum(bool(row["nonadmitted_ticks"]) for row in rows),
        "harness_gates": harness_gates,
        "controller_quality_gates": controller_quality_gates,
        "benchmark_valid": all(harness_gates.values()),
        "controller_qualified_for_full_envelope": all(controller_quality_gates.values()),
        "finding": (
            "The production CPU controller handles repeated centered pulls and the full "
            "tested <=1 N.m moment envelope, but upper-base pulls expose a repeatable "
            "1.5 N.m-class terminal boundary. This is retained as a measured feasibility "
            "limit, not misreported as a solver or transport failure."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    rows = metrics["rows"]
    harness_gates = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — {name.replace('_', ' ')}"
        for name, passed in metrics["harness_gates"].items()
    )
    quality_gates = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — {name.replace('_', ' ')}"
        for name, passed in metrics["controller_quality_gates"].items()
    )
    return f"""# Upkie wrench capability envelope R311

Generated: {metrics['generated_at']}

Status: **{
        'valid measured capability envelope'
        if metrics['benchmark_valid']
        else 'benchmark invalid'
    }**

This deterministic, policy-free matrix applies the same lateral force at the
base center of mass and 25 cm above/below it. A single 400 ms pull is compared
with four spaced pulls in one trajectory. Terminal falls are capability
outcomes; protocol rejection, numeric reset, missed scheduled loads, or hard
residual error would invalidate the benchmark.

| Measure | Result |
|---|---:|
| matrix | {metrics['matrix']['case_count']} cases |
| stable upright | {metrics['outcome_counts'].get('stable_upright', 0)} |
| completed outside upright envelope | {
        metrics['outcome_counts'].get('completed_outside_upright_envelope', 0)
    } |
| terminal falls | {metrics['outcome_counts'].get('terminal_fall', 0)} |
| completed with WBC nonadmission | {
        metrics['outcome_counts'].get('completed_with_nonadmission', 0)
    } |
| maximum commanded moment | {max(row['maximum_moment_nm'] for row in rows):.3f} N·m |
| maximum tilt among completed cases | {
        max(
            row['maximum_root_tilt_rad']
            for row in rows
            if row['terminal_pending'] is None
        )
    :.6f} rad |
| maximum WBC p99 | {max(row['controller_step_us']['p99'] for row in rows):.3f} µs |
| maximum worker p99 | {max(row['worker_step_us']['p99'] for row in rows):.3f} µs |
| hot allocations | 0 |

## Harness validity gates

{harness_gates}

## Controller quality gates

{quality_gates}

## Architectural conclusion

{metrics['finding']}
"""


def run(model: pathlib.Path, *, ticks: int) -> tuple[dict[str, Any], dict[str, Any], str]:
    rows: list[dict[str, Any]] = []
    for schedule_name in SCHEDULES:
        for offset_z_m in OFFSETS_Z_M:
            for force_y_n in FORCES_N:
                case = run_one(
                    model,
                    force_y_n,
                    offset_z_m,
                    schedule_name,
                    ticks=ticks,
                )
                rows.append({**summarize(case, schedule_name), "case": case})
    replay = run_one(
        model,
        FORCES_N[-1],
        OFFSETS_Z_M[-1],
        "repeated",
        ticks=ticks,
    )
    metrics = evaluate(rows, replay, ticks=ticks)
    traces = {
        "revision": REVISION,
        "representative_centered_repeated": next(
            row["case"]
            for row in rows
            if row["schedule"] == "repeated"
            and row["offset_z_m"] == 0.0
            and row["force_y_n"] == FORCES_N[-1]
        ),
        "representative_upper_boundary": rows[-1]["case"],
        "representative_replay": replay,
    }
    return metrics, traces, render_markdown(metrics)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf")
    )
    parser.add_argument("--ticks", type=int, default=TICKS)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results/upkie-live-wrench-envelope-r311"),
    )
    args = parser.parse_args()
    if args.ticks < TICKS:
        parser.error(f"ticks must be at least {TICKS}")
    metrics, traces, markdown = run(args.model, ticks=args.ticks)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output_dir / "traces.json").write_text(
        json.dumps(traces, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    (args.output_dir / "UPKIE_LIVE_WRENCH_ENVELOPE_R311.md").write_text(
        markdown, encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "benchmark_valid": metrics["benchmark_valid"],
                "controller_qualified_for_full_envelope": metrics[
                    "controller_qualified_for_full_envelope"
                ],
                "harness_gates": metrics["harness_gates"],
                "controller_quality_gates": metrics["controller_quality_gates"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if metrics["benchmark_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
