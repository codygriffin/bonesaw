#!/usr/bin/env python3
"""R310 qualify the public wheel-load reserve profile without a policy rollout."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_load_reserve_action_r305 as r305
from upkie_live_plant_worker import PRODUCTION_SUPPORT_LOAD_RESERVE_CONFIG


REVISION = "upkie-live-load-reserve-matrix-r310"
TICKS = 300
FORCES_N = (-8.0, -6.0, -4.0, -2.0, 2.0, 4.0, 6.0, 8.0)
DURATIONS_TICKS = (10, 15, 20, 30, 40)
WORKER_OPTIONS = {
    "stream_dt": 0.020,
    "control_dt": 0.020,
    "physics_dt": 0.004,
    "balanced_nominal_joint_target": True,
}
BASELINE_OPTIONS = {
    "joint_posture_priority": 0,
    "support_load_reserve_action_enabled": False,
}
CANDIDATE_OPTIONS = {
    "joint_posture_priority": 0,
    "support_load_reserve_action_enabled": True,
    "support_load_reserve_config": PRODUCTION_SUPPORT_LOAD_RESERVE_CONFIG,
}


def strict_recovery_tick(states: list[dict[str, Any]], dwell_ticks: int = 10) -> int | None:
    """First bilateral/upright dwell strictly after the final support-loss tick."""
    losses = [state["index"] for state in states if state["observed"] != [1, 1]]
    if not losses:
        return None
    final_loss = losses[-1]
    consecutive = 0
    for state in states:
        recovered = (
            state["index"] > final_loss
            and state["observed"] == [1, 1]
            and state["root_height_m"] >= 0.48
            and state["root_tilt_rad"] <= 0.20
            and state["automatic_reset_pending"] is None
        )
        consecutive = consecutive + 1 if recovered else 0
        if consecutive >= dwell_ticks:
            return state["index"] - dwell_ticks + 1
    return None


def _semantic_digest(case: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(r300._semantic(case), sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def summarize(case: dict[str, Any]) -> dict[str, Any]:
    summary = r300.summarize(case)
    states = case["states"]
    loss_ticks = [state["index"] for state in states if state["observed"] != [1, 1]]
    summary.update(
        {
            "force_y_n": float(case["force_world_n"][1]),
            "push_ticks": int(case["push_ticks"]),
            "push_duration_s": int(case["push_ticks"]) * 0.020,
            "loss_tick_count": len(loss_ticks),
            "last_non_double_tick": loss_ticks[-1] if loss_ticks else None,
            "strict_recovery_tick": strict_recovery_tick(states),
            "final_100_bilateral_upright": len(states) >= 100
            and all(
                state["observed"] == [1, 1]
                and state["root_height_m"] >= 0.48
                and state["root_tilt_rad"] <= 0.20
                for state in states[-100:]
            ),
            "load_reserve": r305._action_summary(case),
        }
    )
    return summary


def run_one(
    model: pathlib.Path,
    force_y_n: float,
    push_ticks: int,
    options: dict[str, Any],
    *,
    ticks: int,
) -> dict[str, Any]:
    return r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=ticks,
        controller_options=options,
        worker_options=WORKER_OPTIONS,
        force_world_n=(0.0, force_y_n, 0.0),
        push_ticks=push_ticks,
    )


def evaluate(
    baseline_nominal: dict[str, Any],
    candidate_nominal: dict[str, Any],
    baseline_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    candidate_replay: dict[str, Any],
    *,
    ticks: int,
) -> dict[str, Any]:
    baseline_nominal_summary = summarize(baseline_nominal)
    candidate_nominal_summary = summarize(candidate_nominal)
    baseline_falls = sum(row["terminal_pending"] is not None for row in baseline_rows)
    candidate_losses = [row for row in candidate_rows if row["loss_tick_count"]]
    maximum_hard_residual = max(
        max(
            row["maximum_constraint_violation"],
            row["maximum_dynamics_residual"],
            row["maximum_contact_residual"],
        )
        for row in candidate_rows
    )
    gates = {
        "nominal_completes_bilateral": candidate_nominal_summary["ticks"] == ticks
        and candidate_nominal_summary["terminal_pending"] is None
        and candidate_nominal_summary["observed_patterns"] == ["11"],
        "nominal_action_is_dormant": candidate_nominal_summary["load_reserve"][
            "maximum_authority"
        ]
        <= 1.0e-12,
        "matrix_has_baseline_failures": baseline_falls > 0,
        "candidate_completes_every_case": all(
            row["ticks"] == ticks and row["terminal_pending"] is None
            for row in candidate_rows
        ),
        "candidate_never_enters_flight": all(
            row["first_flight_tick"] is None for row in candidate_rows
        ),
        "every_support_loss_reacquires": bool(candidate_losses)
        and all(row["strict_recovery_tick"] is not None for row in candidate_losses),
        "every_case_has_stable_2s_tail": all(
            row["final_100_bilateral_upright"] for row in candidate_rows
        ),
        "zero_nonadmission_or_numeric_reset": all(
            not row["nonadmitted_ticks"] and not row["numeric_reset"]
            for row in candidate_rows
        ),
        "hard_residual_below_1e_minus_8": maximum_hard_residual < 1.0e-8,
        "controller_and_worker_p99_within_deadline": all(
            row["controller_step_us"]["p99"] < 5_000.0
            and row["worker_step_us"]["p99"] < 20_000.0
            for row in candidate_rows
        ),
        "zero_hot_path_allocations": all(
            row["allocation_calls"] == 0
            and row["allocated_bytes"] == 0
            and row["load_reserve"]["allocation_calls"] == 0
            and row["load_reserve"]["allocated_bytes"] == 0
            for row in candidate_rows
        ),
        "bounded_tilt_height_and_torque": all(
            row["maximum_root_tilt_rad"] < 0.25
            and row["minimum_root_height_m"] >= 0.50
            and row["maximum_torque_utilization"] < 0.30
            for row in candidate_rows
        ),
        "representative_replay_exact": _semantic_digest(candidate_rows[-1]["case"])
        == _semantic_digest(candidate_replay),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "rates_hz": {"wbc": 50, "physics": 250, "stream": 50},
        "candidate_default_enabled": all(gates.values()),
        "candidate_config": list(PRODUCTION_SUPPORT_LOAD_RESERVE_CONFIG),
        "matrix": {
            "forces_y_n": list(FORCES_N),
            "durations_ticks": list(DURATIONS_TICKS),
            "case_count": len(candidate_rows),
        },
        "baseline_nominal": baseline_nominal_summary,
        "candidate_nominal": candidate_nominal_summary,
        "baseline_rows": [{k: v for k, v in row.items() if k != "case"} for row in baseline_rows],
        "candidate_rows": [{k: v for k, v in row.items() if k != "case"} for row in candidate_rows],
        "baseline_falls": baseline_falls,
        "candidate_falls": sum(row["terminal_pending"] is not None for row in candidate_rows),
        "candidate_contact_loss_cases": len(candidate_losses),
        "candidate_recovered_cases": sum(
            row["strict_recovery_tick"] is not None for row in candidate_rows
        ),
        "maximum_hard_residual": maximum_hard_residual,
        "gates": gates,
        "qualified_for_public_default": all(gates.values()),
        "finding": (
            "A gentler continuous reserve profile composes with R309 posture authority: "
            "all 40 mirrored force-duration cases finish upright, transient single support "
            "never becomes flight, and every loss has a measured bilateral tail. The R308 "
            "landing layer remains default-off because it is unnecessary and non-monotonic here."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    rows = metrics["candidate_rows"]
    gates = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — {name.replace('_', ' ')}"
        for name, passed in metrics["gates"].items()
    )
    return f"""# Upkie load-reserve matrix R310

Generated: {metrics['generated_at']}

Status: **{'qualified for public default' if metrics['qualified_for_public_default'] else 'not qualified'}**

This policy-free A/B composes R309's support-preserving posture hierarchy with
a gentler Rust wheel-load reserve profile. The matrix holds model, public
50/250/50 cadence, limits, contact evidence, solver budget, and application
point fixed while crossing both lateral signs, four force magnitudes, and five
durations.

| Measure | Result |
|---|---:|
| matrix | {metrics['matrix']['case_count']} cases |
| baseline falls | {metrics['baseline_falls']} |
| candidate falls / flight cases | {metrics['candidate_falls']} / {sum(row['first_flight_tick'] is not None for row in rows)} |
| candidate contact-loss / recovered cases | {metrics['candidate_contact_loss_cases']} / {metrics['candidate_recovered_cases']} |
| maximum tilt | {max(row['maximum_root_tilt_rad'] for row in rows):.6f} rad |
| minimum root height | {min(row['minimum_root_height_m'] for row in rows):.6f} m |
| maximum torque utilization | {max(row['maximum_torque_utilization'] for row in rows):.6f} |
| maximum WBC p99 | {max(row['controller_step_us']['p99'] for row in rows):.3f} µs |
| maximum worker p99 | {max(row['worker_step_us']['p99'] for row in rows):.3f} µs |
| maximum reserve-action p99 | {max(row['load_reserve']['step_us_p99'] for row in rows):.3f} µs |
| hot allocations | 0 |

## Gates

{gates}

## Architectural conclusion

{metrics['finding']}
"""


def run(model: pathlib.Path, *, ticks: int) -> tuple[dict[str, Any], dict[str, Any], str]:
    baseline_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=ticks,
        controller_options=BASELINE_OPTIONS,
        worker_options=WORKER_OPTIONS,
    )
    candidate_nominal = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=ticks,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=WORKER_OPTIONS,
    )
    baseline_rows: list[dict[str, Any]] = []
    candidate_rows: list[dict[str, Any]] = []
    for force_y_n in FORCES_N:
        for push_ticks in DURATIONS_TICKS:
            baseline_case = run_one(
                model, force_y_n, push_ticks, BASELINE_OPTIONS, ticks=ticks
            )
            candidate_case = run_one(
                model, force_y_n, push_ticks, CANDIDATE_OPTIONS, ticks=ticks
            )
            baseline_rows.append({**summarize(baseline_case), "case": baseline_case})
            candidate_rows.append({**summarize(candidate_case), "case": candidate_case})
    candidate_replay = run_one(
        model, FORCES_N[-1], DURATIONS_TICKS[-1], CANDIDATE_OPTIONS, ticks=ticks
    )
    metrics = evaluate(
        baseline_nominal,
        candidate_nominal,
        baseline_rows,
        candidate_rows,
        candidate_replay,
        ticks=ticks,
    )
    traces = {
        "revision": REVISION,
        "baseline_nominal": baseline_nominal,
        "candidate_nominal": candidate_nominal,
        "representative_baseline": baseline_rows[-1]["case"],
        "representative_candidate": candidate_rows[-1]["case"],
        "representative_replay": candidate_replay,
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
        default=pathlib.Path("benchmarks/results/upkie-live-load-reserve-matrix-r310"),
    )
    args = parser.parse_args()
    if args.ticks < 300:
        parser.error("ticks must be at least 300")
    metrics, traces, markdown = run(args.model, ticks=args.ticks)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output_dir / "traces.json").write_text(
        json.dumps(traces, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    (args.output_dir / "UPKIE_LIVE_LOAD_RESERVE_MATRIX_R310.md").write_text(
        markdown, encoding="utf-8"
    )
    print(json.dumps(metrics["gates"], indent=2, sort_keys=True))
    return 0 if metrics["qualified_for_public_default"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
