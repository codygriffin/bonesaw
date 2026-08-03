#!/usr/bin/env python3
"""R318 exploratory force/moment confidence-vector probe.

R316 showed that one scalar root-wrench confidence does not transfer between a
centered force and the same force applied at a lever.  This screen exercises
the new Rust-owned six-axis scale seam on a small, predeclared holdout.  It is
evidence only: no profile is promoted and the public R310 controller is not
changed.
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
from cpu_reference_report import render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
REVISION = "upkie-live-root-wrench-axis-scale-probe-r318"
FROZEN_MUJOCO_VERSION = "3.3.7"
WINDOWS = r311.SCHEDULES["repeated"]
CASES = (("centered_plus8", 8.0, 0.0), ("upper_plus8", 8.0, 0.25), ("upper_minus6", -6.0, 0.25))
PROFILES: dict[str, dict[str, Any]] = {
    "scalar_0.70": {"external_wrench_feedforward_scale": 0.70},
    "moment_0.70_force_0.52": {
        "external_wrench_feedforward_scale": 1.0,
        "external_wrench_feedforward_axis_scales": (0.70, 0.70, 0.70, 0.52, 0.52, 0.52),
    },
    "moment_0.70_force_0.68": {
        "external_wrench_feedforward_scale": 1.0,
        "external_wrench_feedforward_axis_scales": (0.70, 0.70, 0.70, 0.68, 0.68, 0.68),
    },
    "moment_0.68_force_0.70": {
        "external_wrench_feedforward_scale": 1.0,
        "external_wrench_feedforward_axis_scales": (0.68, 0.68, 0.68, 0.70, 0.70, 0.70),
    },
}


def final_tail(case: dict[str, Any], ticks: int = 100) -> bool:
    states = case["states"]
    return len(states) >= ticks and all(
        state["observed"] == [1, 1]
        and state["root_height_m"] >= 0.48
        and state["root_tilt_rad"] <= 0.20
        and state["automatic_reset_pending"] is None
        for state in states[-ticks:]
    )


def summarize(case: dict[str, Any]) -> dict[str, Any]:
    summary = r314.summarize(case)
    return {
        "terminal_tick": summary["terminal_tick"],
        "nonadmitted_ticks": summary["nonadmitted_ticks"],
        "loss_tick_count": summary["loss_tick_count"],
        "final_100_bilateral_upright": final_tail(case),
        "maximum_root_tilt_rad": summary["maximum_root_tilt_rad"],
        "maximum_torque_utilization": summary["maximum_torque_utilization"],
        "controller_step_p99_us": summary["controller_step_us"]["p99"],
        "worker_step_p99_us": summary["worker_step_us"]["p99"],
    }


def run_case(
    model: pathlib.Path,
    force_n: float,
    offset_z_m: float,
    profile: dict[str, Any],
) -> dict[str, Any]:
    return r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=450,
        controller_options={**r314.CANDIDATE_OPTIONS, **profile},
        worker_options=r310.WORKER_OPTIONS,
        force_world_n=(0.0, force_n, 0.0),
        application_offset_world_m=(0.0, 0.0, offset_z_m),
        push_windows=WINDOWS,
    )


def evaluate(model: pathlib.Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for profile_name, profile in PROFILES.items():
        for case_name, force_n, offset_z_m in CASES:
            result = run_case(model, force_n, offset_z_m, profile)
            rows.append(
                {
                    "profile": profile_name,
                    "case": case_name,
                    "force_y_n": force_n,
                    "offset_z_m": offset_z_m,
                    **summarize(result),
                }
            )
    scalar = {
        row["case"]: row
        for row in rows
        if row["profile"] == "scalar_0.70"
    }
    vector_rows = [row for row in rows if row["profile"] != "scalar_0.70"]
    gates = {
        "frozen_mujoco_version_matches": mujoco.__version__ == FROZEN_MUJOCO_VERSION,
        "public_controller_remains_default_off": (
            "external_wrench_feedforward_enabled"
            not in r314.r312.PUBLIC_CONTROLLER_OPTIONS
        ),
        "vector_path_changes_at_least_one_observed_consequence": any(
            row["terminal_tick"] != scalar[row["case"]]["terminal_tick"]
            or row["nonadmitted_ticks"] != scalar[row["case"]]["nonadmitted_ticks"]
            for row in vector_rows
        ),
        "all_vector_rows_meet_zero_fall_holdout": all(
            row["terminal_tick"] is None for row in vector_rows
        ),
        "all_vector_rows_meet_zero_nonadmission_holdout": all(
            not row["nonadmitted_ticks"] for row in vector_rows
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "mujoco_version": mujoco.__version__,
            "frozen_mujoco_version": FROZEN_MUJOCO_VERSION,
            "case_count": len(CASES),
            "profile_count": len(PROFILES),
            "axis_order": "root_moment_xyz_then_root_force_xyz",
        },
        "profiles": PROFILES,
        "rows": rows,
        "gates": gates,
        "public_promotion": False,
        "finding": (
            "The optional Rust-owned six-axis scale vector changes the measured "
            "consequence without changing public authority. This small screen "
            "is not a robustness qualification: the full R311 schedule/lever "
            "holdout, contiguous scale neighborhood, and hardware calibration "
            "remain required before any promotion."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Upkie root-wrench force/moment scale probe — R318",
        "",
        "Status: **EXPLORATORY EVIDENCE; NO AUTHORITY PROMOTION**.",
        "",
        metrics["finding"],
        "",
        "| profile | case | terminal tick | nonadmitted | upright tail | max tilt | max torque |",
        "|:--|:--|--:|:--|:--|--:|--:|",
    ]
    for row in metrics["rows"]:
        lines.append(
            f'| {row["profile"]} | {row["case"]} | '
            f'{row["terminal_tick"] if row["terminal_tick"] is not None else "—"} | '
            f'{row["nonadmitted_ticks"] or "—"} | '
            f'{"yes" if row["final_100_bilateral_upright"] else "no"} | '
            f'{row["maximum_root_tilt_rad"]:.3f} | {row["maximum_torque_utilization"]:.3f} |'
        )
    lines += ["", "## Gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["gates"].items()
    ]
    lines += ["", "R310 remains public; no vector profile is executable authority.", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=ROOT / "benchmarks/results/upkie-live-root-wrench-axis-scale-probe-r318",
    )
    parser.add_argument(
        "--web-report",
        type=pathlib.Path,
        default=ROOT / "web/UPKIE_LIVE_ROOT_WRENCH_AXIS_SCALE_PROBE_R318.html",
    )
    args = parser.parse_args()
    metrics = evaluate(args.model.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (args.output / "UPKIE_LIVE_ROOT_WRENCH_AXIS_SCALE_PROBE_R318.md").write_text(
        markdown(metrics)
    )
    args.web_report.write_text(
        render_report_html(
            markdown(metrics), title="Upkie root-wrench axis-scale probe · R318"
        )
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
