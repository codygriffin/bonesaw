#!/usr/bin/env python3
"""R302 measured-contact recovery calibration and strict consequence audit."""

from __future__ import annotations

import argparse
import html
import itertools
import json
import math
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any

import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-support-recovery-r302"
MAXIMUM_TICKS = 700
CALIBRATION_TICKS = 200
SELECTED_CONFIG = (9.81, 4.0, 16.0, 12.0, 80.0, 14.0, 8.0, 8.0, 20.0, 80.0, 120.0)


def controller_options(config: tuple[float, ...] | None) -> dict[str, Any]:
    options: dict[str, Any] = {
        "support_contingency_enabled": True,
        "support_contingency_execute": True,
    }
    if config is not None:
        options["support_contingency_config"] = config
    return options


def calibration_profiles() -> list[tuple[float, ...]]:
    profiles: list[tuple[float, ...]] = []
    for position, linear, attitude, angular, maximum_horizontal in itertools.product(
        (4.0, 8.0, 16.0, 32.0),
        (2.0, 4.0, 8.0, 12.0),
        (20.0, 40.0, 80.0),
        (8.0, 14.0),
        (8.0, 16.0),
    ):
        if abs(angular - 2.0 * math.sqrt(attitude)) > 6.5:
            continue
        profiles.append(
            (
                9.81,
                4.0,
                position,
                linear,
                attitude,
                angular,
                8.0,
                maximum_horizontal,
                20.0,
                80.0,
                120.0,
            )
        )
    return profiles


def screen_row(config: tuple[float, ...], case: dict[str, Any]) -> dict[str, Any]:
    summary = r300.summarize(case)
    return {
        "config": list(config),
        "terminal_tick": summary["terminal_tick"],
        "supported_upright_recovery_tick": summary[
            "supported_upright_recovery_tick"
        ],
        "maximum_body_ground_stall_ticks": summary[
            "maximum_body_ground_stall_ticks"
        ],
        "maximum_root_tilt_rad": summary["maximum_root_tilt_rad"],
        "minimum_root_height_m": summary["minimum_root_height_m"],
        "maximum_torque_utilization": summary["maximum_torque_utilization"],
        "max_iterations_ticks": summary["max_iterations_ticks"],
        "maximum_dynamics_residual": summary["maximum_dynamics_residual"],
        "maximum_contact_residual": summary["maximum_contact_residual"],
        "controller_p99_us": summary["controller_step_us"]["p99"],
        "allocation_calls": summary["allocation_calls"],
    }


def run_screen(model: pathlib.Path) -> dict[str, Any]:
    phase_one: list[dict[str, Any]] = []
    shortlist: list[tuple[float, ...]] = []
    profiles = calibration_profiles()
    for index, config in enumerate(profiles, 1):
        case = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=CALIBRATION_TICKS,
            controller_options=controller_options(config),
        )
        row = screen_row(config, case)
        phase_one.append(row)
        if (
            row["terminal_tick"] is None
            and not row["max_iterations_ticks"]
            and row["controller_p99_us"] < 5_000.0
            and row["maximum_dynamics_residual"] <= 1.0e-8
            and row["maximum_contact_residual"] <= 1.0e-8
            and row["allocation_calls"] == 0
        ):
            shortlist.append(config)
        print(
            f"screen {index:03d}/{len(profiles)} terminal={row['terminal_tick']} "
            f"recovery={row['supported_upright_recovery_tick']} stall={row['maximum_body_ground_stall_ticks']}",
            flush=True,
        )
    phase_two: list[dict[str, Any]] = []
    for index, config in enumerate(shortlist, 1):
        case = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=MAXIMUM_TICKS,
            controller_options=controller_options(config),
        )
        row = screen_row(config, case)
        phase_two.append(row)
        print(
            f"extend {index:02d}/{len(shortlist)} terminal={row['terminal_tick']} "
            f"recovery={row['supported_upright_recovery_tick']} stall={row['maximum_body_ground_stall_ticks']}",
            flush=True,
        )
    selected = max(
        phase_two,
        key=lambda row: (
            row["terminal_tick"] if row["terminal_tick"] is not None else MAXIMUM_TICKS,
            -row["maximum_body_ground_stall_ticks"],
        ),
    )
    return {
        "declared_profile_count": len(profiles),
        "shortlist_count": len(shortlist),
        "phase_one": phase_one,
        "phase_two": phase_two,
        "selected_config": selected["config"],
        "selection_objective": "latest fall boundary, then least low-body/no-wheel stall; calibration only",
    }


def compact_trace(case: dict[str, Any]) -> list[dict[str, Any]]:
    fields = (
        "index",
        "time_s",
        "root_position",
        "root_tilt_rad",
        "root_height_m",
        "observed",
        "hard",
        "hard_executable",
        "physics_contact_active",
        "wbc_raw_status",
        "wbc_admitted",
        "wbc_maximum_constraint_violation",
        "wbc_dynamics_residual",
        "wbc_contact_residual",
        "torque_utilization",
        "controller_step_us",
        "worker_step_us",
        "ground_contact_count",
        "automatic_reset_pending",
    )
    return [{field: state[field] for field in fields} for state in case["states"]]


def physical_trace(case: dict[str, Any]) -> list[dict[str, Any]]:
    """Exact plant/command consequence, excluding intentional shadow telemetry."""
    fields = (
        "root_position",
        "root_quaternion_wxyz",
        "root_twist_world",
        "joint_positions",
        "joint_velocities",
        "actuator_effort_nm",
        "generalized_acceleration",
        "observed",
        "hard",
        "hard_executable",
        "physics_contact_active",
        "wbc_raw_status",
        "wbc_admitted",
        "automatic_reset_pending",
        "numeric_reset",
    )
    return [{field: state[field] for field in fields} for state in case["states"]]


def evaluate(
    baseline: dict[str, Any],
    default: dict[str, Any],
    candidate: dict[str, Any],
    replay: dict[str, Any],
    dormant_control: dict[str, Any],
    dormant_candidate: dict[str, Any],
) -> dict[str, bool]:
    baseline_summary = r300.summarize(baseline)
    default_summary = r300.summarize(default)
    summary = r300.summarize(candidate)
    return {
        "semantic_replay_exact": r300._semantic(candidate) == r300._semantic(replay),
        "configured_profile_is_physically_dormant_in_nominal_double_support": (
            physical_trace(dormant_control) == physical_trace(dormant_candidate)
        ),
        "causal_prior_window_contract_preserved": r300._causal_window_contract(
            candidate["states"]
        ),
        "all_outputs_finite": all(
            math.isfinite(float(state[field]))
            for state in candidate["states"]
            for field in (
                "root_height_m",
                "root_tilt_rad",
                "wbc_maximum_constraint_violation",
                "wbc_dynamics_residual",
                "wbc_contact_residual",
                "controller_step_us",
            )
        ),
        "zero_timed_rust_allocation": summary["allocation_calls"] == 0
        and summary["allocated_bytes"] == 0,
        "no_max_iterations": not summary["max_iterations_ticks"],
        "controller_p99_under_5ms": summary["controller_step_us"]["p99"]
        < 5_000.0,
        "hard_residuals_under_1e8": summary["maximum_constraint_violation"]
        <= 1.0e-8
        and summary["maximum_dynamics_residual"] <= 1.0e-8
        and summary["maximum_contact_residual"] <= 1.0e-8,
        "fall_boundary_delayed_vs_no_contingency": (
            summary["terminal_tick"] is not None
            and baseline_summary["terminal_tick"] is not None
            and summary["terminal_tick"] > baseline_summary["terminal_tick"]
        ),
        "fall_boundary_delayed_vs_default_contingency": (
            summary["terminal_tick"] is not None
            and default_summary["terminal_tick"] is not None
            and summary["terminal_tick"] > default_summary["terminal_tick"]
        ),
        "candidate_has_no_fall_boundary": summary["terminal_pending"] is None,
        "candidate_recovers_supported_upright": summary[
            "supported_upright_recovery_tick"
        ]
        is not None,
        "candidate_never_stalls_on_body_without_wheel_support": summary[
            "maximum_body_ground_stall_ticks"
        ]
        == 0,
    }


def render_report(metrics: dict[str, Any]) -> str:
    summaries = metrics["summaries"]
    gates = metrics["gates"]
    rows = []
    for name in ("no_contingency", "default_contingency", "calibrated_candidate"):
        summary = summaries[name]
        rows.append(
            "| {} | {} | {} | {} | {} | {:.1f} | {:.3e} | {:.3f} |".format(
                name.replace("_", " "),
                summary["terminal_tick"] if summary["terminal_tick"] is not None else "—",
                summary["supported_upright_recovery_tick"]
                if summary["supported_upright_recovery_tick"] is not None
                else "—",
                summary["maximum_body_ground_stall_ticks"],
                summary["max_iterations_ticks"] or "—",
                summary["controller_step_us"]["p99"],
                max(
                    summary["maximum_constraint_violation"],
                    summary["maximum_dynamics_residual"],
                    summary["maximum_contact_residual"],
                ),
                summary["maximum_torque_utilization"],
            )
        )
    mechanism_names = (
        "semantic_replay_exact",
        "configured_profile_is_physically_dormant_in_nominal_double_support",
        "causal_prior_window_contract_preserved",
        "all_outputs_finite",
        "zero_timed_rust_allocation",
        "no_max_iterations",
        "controller_p99_under_5ms",
        "hard_residuals_under_1e8",
        "fall_boundary_delayed_vs_no_contingency",
        "fall_boundary_delayed_vs_default_contingency",
    )
    behavior_names = tuple(name for name in gates if name not in mechanism_names)
    lines = [
        "# Upkie live support recovery calibration · R302",
        "",
        f"> Mechanism {'PASS' if metrics['mechanism_passed'] else 'FAIL'} · controller behavior {'PASS' if metrics['controller_behavior_passed'] else 'REJECTED'} · evaluation-only profile · no learned policy.",
        "",
        "R302 calibrates only the existing allocation-free Rust observed-support action admitted by R301. Python declares the bounded gain grid, sequences MuJoCo, and scores outcomes; Rust authors the acceleration request and the ordinary floating WBC retains execution authority. The selected profile is not promoted and does not alter the public/default controller.",
        "",
        "A reset-free trace is not called recovery. Recovery requires ten consecutive 50 Hz ticks with both measured wheel contacts, root height at least 0.48 m, root tilt at most 0.20 rad, and no pending reset. A separate metric counts low-body intervals below 0.40 m with neither wheel in contact.",
        "The retained calibration artifact evaluates 160 declared gain profiles for 200 ticks, extends the six finite/residual/deadline-clean boundary survivors to 700 ticks, and then replays the selected profile exactly.",
        "",
        "| arm | fall tick | supported-upright recovery | longest body-ground stall ticks | MaxIterations ticks | controller p99 µs | worst hard residual | max torque utilization |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        *rows,
        "",
        f"Selected Rust configuration: `{metrics['selected_config']}`.",
        "",
        "## Mechanism and consequence gates",
        "",
        "| gate | result |",
        "|---|:---:|",
        *(f"| {name} | {'PASS' if gates[name] else 'FAIL'} |" for name in mechanism_names),
        "",
        "## Recovery gates",
        "",
        "| gate | result |",
        "|---|:---:|",
        *(f"| {name} | {'PASS' if gates[name] else 'FAIL'} |" for name in behavior_names),
        "",
        "## Conclusion",
        "",
        "The calibrated action removes the R300 infeasible/residual failure mode and materially delays the first fall boundary, but it does not recover. It spends a long interval supported by non-wheel body geometry, never satisfies the sustained wheel-supported upright predicate, and eventually crosses the same fall boundary. The next controller milestone is therefore contact reacquisition—not more QP iterations or timeout tolerance. It needs an independently admitted action that preserves or deliberately regains a wheel contact, with this strict state predicate retained as the promotion gate.",
        "",
    ]
    return "\n".join(lines)


def render_html(report: str, passed: bool) -> str:
    return "\n".join(
        (
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>Bonesaw Upkie support recovery R302</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px}.pass{color:#86efac}.fail{color:#fca5a5}</style>",
            f"<h1 class='{'pass' if passed else 'fail'}'>Upkie live support recovery · R302</h1>",
            f"<pre>{html.escape(report)}</pre>",
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_LIVE_SUPPORT_RECOVERY_R302.html")
    parser.add_argument("--screen", action="store_true")
    parser.add_argument("--require-behavior", action="store_true")
    args = parser.parse_args()
    model = pathlib.Path(args.model).resolve()
    screen = run_screen(model) if args.screen else None
    selected = tuple(screen["selected_config"]) if screen is not None else SELECTED_CONFIG
    if selected != SELECTED_CONFIG:
        raise SystemExit(
            f"calibration selected {selected}, expected frozen profile {SELECTED_CONFIG}"
        )

    no_contingency = r300.run_case(model, disturbed=True, maximum_ticks=MAXIMUM_TICKS)
    default = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=MAXIMUM_TICKS,
        controller_options=controller_options(None),
    )
    candidate = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=MAXIMUM_TICKS,
        controller_options=controller_options(selected),
    )
    replay = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=MAXIMUM_TICKS,
        controller_options=controller_options(selected),
    )
    # The reference capture controller itself leaves double support after tick
    # 64 on this long open-loop fixture. Keep the dormancy control inside the
    # shared, measured-11 prefix instead of conflating later support action.
    dormant_control = r300.run_case(model, disturbed=False, maximum_ticks=60)
    dormant_candidate = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=60,
        controller_options=controller_options(selected),
    )
    gates = evaluate(
        no_contingency,
        default,
        candidate,
        replay,
        dormant_control,
        dormant_candidate,
    )
    behavior_names = (
        "candidate_has_no_fall_boundary",
        "candidate_recovers_supported_upright",
        "candidate_never_stalls_on_body_without_wheel_support",
    )
    mechanism_passed = all(value for name, value in gates.items() if name not in behavior_names)
    behavior_passed = mechanism_passed and all(gates[name] for name in behavior_names)
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mechanism_passed": mechanism_passed,
        "controller_behavior_passed": behavior_passed,
        "learned_policy_steps": 0,
        "maximum_ticks": MAXIMUM_TICKS,
        "selected_config": list(selected),
        "calibration_screen": screen,
        "gates": gates,
        "summaries": {
            "no_contingency": r300.summarize(no_contingency),
            "default_contingency": r300.summarize(default),
            "calibrated_candidate": r300.summarize(candidate),
            "dormant_control": r300.summarize(dormant_control),
            "dormant_candidate": r300.summarize(dormant_candidate),
        },
        "traces": {
            "no_contingency": compact_trace(no_contingency),
            "default_contingency": compact_trace(default),
            "calibrated_candidate": compact_trace(candidate),
        },
    }
    report = render_report(metrics)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-live-support-recovery-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_LIVE_SUPPORT_RECOVERY_R302.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report, behavior_passed))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "controller_behavior_passed": behavior_passed,
                "gates": gates,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed and (not args.require_behavior or behavior_passed) else 1


if __name__ == "__main__":
    sys.exit(main())
