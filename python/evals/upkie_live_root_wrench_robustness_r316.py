#!/usr/bin/env python3
"""R316 robustness audit for the confidence-scaled root-wrench candidate.

R314's exact 0.70 profile clears its frozen eight-force matrix. This audit
keeps that matrix as selection evidence, then asks two independent questions:
whether adjacent scales form a stable interval and whether the selected scale
survives the remaining 40 rows of R311's schedule/lever matrix.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_load_reserve_matrix_r310 as r310
import upkie_live_moment_rejection_r314 as r314
import upkie_live_wrench_envelope_r311 as r311


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "upkie-live-root-wrench-robustness-r316"
FROZEN_MUJOCO_VERSION = "3.3.7"
SELECTED_SCALE = 0.70
SCALES = (0.0, 0.40, 0.60, 0.62, 0.64, 0.68, 0.70, 0.72, 0.74, 0.80, 1.0)
TERMINAL_FORCES_N = (-8.0, -6.0, 6.0, 8.0)


def _tail(case: dict[str, Any], ticks: int = 100) -> bool:
    states = case["states"]
    return len(states) >= ticks and all(
        state["observed"] == [1, 1]
        and state["root_height_m"] >= 0.48
        and state["root_tilt_rad"] <= 0.20
        and state["automatic_reset_pending"] is None
        for state in states[-ticks:]
    )


def _case_summary(case: dict[str, Any]) -> dict[str, Any]:
    summary = r314.summarize(case)
    return {
        "terminal_tick": summary["terminal_tick"],
        "nonadmitted_ticks": summary["nonadmitted_ticks"],
        "loss_tick_count": summary["loss_tick_count"],
        "force_qualified_relock_count": summary["force_qualified_relock_count"],
        "final_100_bilateral_upright": _tail(case),
        "maximum_root_tilt_rad": summary["maximum_root_tilt_rad"],
        "maximum_torque_utilization": summary["maximum_torque_utilization"],
        "maximum_dynamics_residual": summary["maximum_dynamics_residual"],
        "maximum_contact_residual": summary["maximum_contact_residual"],
        "controller_step_p99_us": summary["controller_step_us"]["p99"],
        "worker_step_p99_us": summary["worker_step_us"]["p99"],
    }


def _run(
    model: pathlib.Path,
    force_n: float,
    offset_m: float,
    windows: tuple[tuple[int, int], ...],
    scale: float,
) -> dict[str, Any]:
    return r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=450,
        controller_options={
            **r314.CANDIDATE_OPTIONS,
            "external_wrench_feedforward_scale": scale,
        },
        worker_options=r310.WORKER_OPTIONS,
        force_world_n=(0.0, force_n, 0.0),
        application_offset_world_m=(0.0, 0.0, offset_m),
        push_windows=windows,
    )


def evaluate(model: pathlib.Path) -> dict[str, Any]:
    scale_rows = []
    for scale in SCALES:
        consequences = []
        for force_n in TERMINAL_FORCES_N:
            case = _run(
                model,
                force_n,
                r311.OFFSETS_Z_M[-1],
                r311.SCHEDULES["repeated"],
                scale,
            )
            consequences.append({"force_y_n": force_n, **_case_summary(case)})
        scale_rows.append(
            {
                "scale": scale,
                "fall_count": sum(
                    row["terminal_tick"] is not None for row in consequences
                ),
                "nonadmitted_tick_count": sum(
                    len(row["nonadmitted_ticks"]) for row in consequences
                ),
                "every_final_tail_upright": all(
                    row["final_100_bilateral_upright"] for row in consequences
                ),
                "maximum_torque_utilization": max(
                    row["maximum_torque_utilization"] for row in consequences
                ),
                "consequences": consequences,
            }
        )

    schedule_rows = []
    replay_source: dict[str, Any] | None = None
    for schedule, windows in r311.SCHEDULES.items():
        for offset_m in r311.OFFSETS_Z_M:
            for force_n in r311.FORCES_N:
                case = _run(model, force_n, offset_m, windows, SELECTED_SCALE)
                selection_row = schedule == "repeated" and offset_m == 0.25
                schedule_rows.append(
                    {
                        "schedule": schedule,
                        "offset_z_m": offset_m,
                        "force_y_n": force_n,
                        "selection_row": selection_row,
                        **_case_summary(case),
                    }
                )
                if schedule == "repeated" and offset_m == 0.0 and force_n == 8.0:
                    replay_source = case
    assert replay_source is not None
    replay = _run(
        model,
        8.0,
        0.0,
        r311.SCHEDULES["repeated"],
        SELECTED_SCALE,
    )
    holdout = [row for row in schedule_rows if not row["selection_row"]]
    passing_scales = [
        row["scale"]
        for row in scale_rows
        if row["fall_count"] == 0
        and row["nonadmitted_tick_count"] == 0
        and row["every_final_tail_upright"]
    ]
    gates = {
        "frozen_mujoco_version_matches": mujoco.__version__ == FROZEN_MUJOCO_VERSION,
        "candidate_is_default_off": (
            "external_wrench_feedforward_enabled"
            not in r314.r312.PUBLIC_CONTROLLER_OPTIONS
        ),
        "selected_scale_clears_terminal_rows": SELECTED_SCALE in passing_scales,
        "adjacent_scale_interval_qualifies": 0.68 in passing_scales
        and 0.72 in passing_scales,
        "independent_40_case_holdout_has_zero_falls": all(
            row["terminal_tick"] is None for row in holdout
        ),
        "independent_40_case_holdout_has_zero_nonadmission": all(
            not row["nonadmitted_ticks"] for row in holdout
        ),
        "independent_completed_rows_finish_upright": all(
            row["terminal_tick"] is not None
            or row["final_100_bilateral_upright"]
            for row in holdout
        ),
        "independent_holdout_meets_deadlines": all(
            row["controller_step_p99_us"] < 5_000.0
            and row["worker_step_p99_us"] < 20_000.0
            for row in holdout
        ),
        "failed_holdout_replays_exactly": (
            r314.semantic_digest(replay_source) == r314.semantic_digest(replay)
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "mujoco_version": mujoco.__version__,
            "frozen_mujoco_version": FROZEN_MUJOCO_VERSION,
        },
        "selected_scale": SELECTED_SCALE,
        "scale_rows": scale_rows,
        "passing_scales": passing_scales,
        "schedule_matrix": {
            "case_count": len(schedule_rows),
            "selection_case_count": sum(row["selection_row"] for row in schedule_rows),
            "independent_holdout_case_count": len(holdout),
            "holdout_fall_count": sum(
                row["terminal_tick"] is not None for row in holdout
            ),
            "holdout_nonadmitted_tick_count": sum(
                len(row["nonadmitted_ticks"]) for row in holdout
            ),
            "maximum_holdout_torque_utilization": max(
                row["maximum_torque_utilization"] for row in holdout
            ),
            "maximum_holdout_controller_step_p99_us": max(
                row["controller_step_p99_us"] for row in holdout
            ),
            "rows": schedule_rows,
        },
        "gates": gates,
        "public_promotion": all(gates.values()),
        "finding": (
            "Scale 0.70 clears the selected frozen terminal matrix, but the scale "
            "response is discontinuous and the independent 40-case R311 holdout "
            "retains one failure: repeated centered +8 N reaches a nonadmitted "
            "solve at tick 338 and falls at tick 351. The mechanism remains "
            "default-off pending a robust interval and independent holdout pass."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    matrix = metrics["schedule_matrix"]
    lines = [
        "# Upkie root-wrench robustness — R316",
        "",
        "Status: **FROZEN MATRIX RECOVERY; ROBUSTNESS REJECTED; DEFAULT-OFF**.",
        "",
        metrics["finding"],
        "",
        f'Passing sampled scales: **{metrics["passing_scales"]}**.',
        "",
        "| scale | falls | nonadmitted ticks | all upright tails | max torque |",
        "|---:|---:|---:|:---:|---:|",
    ]
    for row in metrics["scale_rows"]:
        lines.append(
            f'| {row["scale"]:.2f} | {row["fall_count"]} | '
            f'{row["nonadmitted_tick_count"]} | '
            f'{"yes" if row["every_final_tail_upright"] else "no"} | '
            f'{row["maximum_torque_utilization"]:.3f} |'
        )
    lines += [
        "",
        "## Independent schedule/lever holdout",
        "",
        f'- {matrix["independent_holdout_case_count"]} rows exclude the eight upper/repeated selection rows.',
        f'- Falls: **{matrix["holdout_fall_count"]}**; nonadmitted ticks: **{matrix["holdout_nonadmitted_tick_count"]}**.',
        f'- Max torque utilization: **{matrix["maximum_holdout_torque_utilization"]:.3f}**.',
        f'- Max controller p99: **{matrix["maximum_holdout_controller_step_p99_us"]:.1f} µs**.',
        "",
        "## Gates",
        "",
    ]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["gates"].items()
    ]
    lines += [
        "",
        "R310 remains the public profile. R314's exact 0.70 result is retained as "
        "behavior-positive selection evidence, not generalized into authority.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=ROOT / "benchmarks/results/upkie-live-root-wrench-robustness-r316",
    )
    args = parser.parse_args()
    metrics = evaluate(args.model.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (args.output / "UPKIE_LIVE_ROOT_WRENCH_ROBUSTNESS_R316.md").write_text(
        markdown(metrics)
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
