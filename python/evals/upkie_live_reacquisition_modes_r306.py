#!/usr/bin/env python3
"""R306 negative physical contact-reacquisition comparison.

This is an evaluation-only consequence fixture.  It replays the frozen R300
8 N lateral wrench through the existing capture, planar-capture, viability,
and R302 support-contingency profiles.  The profiles are deliberately passed
as explicit worker overrides; the public worker remains the 250 Hz MuJoCo /
50 Hz WBC ``production_default`` profile.

The strict recovery predicate is intentionally conservative: after the
disturbance, a mode must have ten consecutive 50 Hz samples with both
measured wheel contacts, root height >= 0.48 m, root tilt <= 0.20 rad, and no
pending reset.  A mode that merely delays a fall or touches body geometry is
not called recovered.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-reacquisition-modes-r306"
MAX_TICKS = 120

# R302's selected, evaluation-only support-contingency profile.  Keep this
# copied here so the negative comparison is reproducible without importing a
# calibration script that can run a 160-profile screen.
R302_SUPPORT_CONTINGENCY = {
    "support_contingency_enabled": True,
    "support_contingency_execute": True,
    "support_contingency_config": (
        9.81,
        4.0,
        16.0,
        12.0,
        80.0,
        14.0,
        8.0,
        8.0,
        20.0,
        80.0,
        120.0,
    ),
}


MODE_CONFIGS: dict[str, dict[str, Any]] = {
    "production_capture": {
        "balance_mode": "capture",
        "controller_options": {},
        "description": "public production_default capture WBC",
    },
    "planar_capture": {
        "balance_mode": "planar_capture",
        "controller_options": {},
        "description": "evaluation-only planar capture composition",
    },
    "viability_capture": {
        "balance_mode": "viability_capture",
        "controller_options": {"viability_planner_enabled": True},
        "description": "evaluation-only Rust viability planner + capture",
    },
    "viability_support_capture": {
        "balance_mode": "viability_support_capture",
        "controller_options": {
            "viability_planner_enabled": True,
            "viability_support_requires_active_request": True,
        },
        "description": "evaluation-only support-conditioned viability capture",
    },
    "viability_coordinate": {
        "balance_mode": "viability_coordinate",
        "controller_options": {
            "viability_planner_enabled": True,
            "viability_support_requires_active_request": True,
        },
        "description": "evaluation-only coordinate viability planner",
    },
    "support_contingency_r302": {
        "balance_mode": "capture",
        "controller_options": R302_SUPPORT_CONTINGENCY,
        "description": "R302 Rust support-contingency request + ordinary WBC",
    },
}


def _finite_trace(case: dict[str, Any]) -> bool:
    fields = (
        "root_height_m",
        "root_tilt_rad",
        "wbc_maximum_constraint_violation",
        "wbc_dynamics_residual",
        "wbc_contact_residual",
        "controller_step_us",
        "worker_step_us",
        "total_ground_normal_force_n",
    )
    return all(
        math.isfinite(float(state[field]))
        for state in case["states"]
        for field in fields
    )


def _nonadmitted_iterations(case: dict[str, Any]) -> bool:
    return all(
        state["wbc_status"] != "MaxIterations" or not state["wbc_admitted"]
        for state in case["states"]
    )


def _strict_recovery(case: dict[str, Any]) -> bool:
    return r300.summarize(case)["supported_upright_recovery_tick"] is not None


def mode_summary(name: str, case: dict[str, Any]) -> dict[str, Any]:
    summary = r300.summarize(case)
    return {
        "name": name,
        "description": MODE_CONFIGS[name]["description"],
        "ticks": summary["ticks"],
        "terminal_tick": summary["terminal_tick"],
        "terminal_pending": summary["terminal_pending"],
        "reset_epochs": summary["reset_epochs"],
        "numeric_reset": summary["numeric_reset"],
        "observed_patterns": summary["observed_patterns"],
        "first_non_double_tick": summary["first_non_double_tick"],
        "first_flight_tick": summary["first_flight_tick"],
        "supported_upright_recovery_tick": summary[
            "supported_upright_recovery_tick"
        ],
        "maximum_body_ground_stall_ticks": summary[
            "maximum_body_ground_stall_ticks"
        ],
        "minimum_root_height_m": summary["minimum_root_height_m"],
        "maximum_root_tilt_rad": summary["maximum_root_tilt_rad"],
        "maximum_root_lateral_displacement_m": summary[
            "maximum_root_lateral_displacement_m"
        ],
        "maximum_constraint_violation": summary["maximum_constraint_violation"],
        "maximum_dynamics_residual": summary["maximum_dynamics_residual"],
        "maximum_contact_residual": summary["maximum_contact_residual"],
        "controller_step_us": summary["controller_step_us"],
        "worker_step_us": summary["worker_step_us"],
        "allocation_calls": summary["allocation_calls"],
        "allocated_bytes": summary["allocated_bytes"],
        "hard_subset_raw": summary["hard_subset_raw"],
        "warning_count": summary["warning_count"],
        "strict_recovery": _strict_recovery(case),
        "finite_trace": _finite_trace(case),
        "max_iterations_nonadmitted": _nonadmitted_iterations(case),
        "live_rate_split": case["hello"],
        "causal_window_contract": r300._causal_window_contract(case["states"]),
    }


def run_modes(model: pathlib.Path, maximum_ticks: int = MAX_TICKS) -> dict[str, Any]:
    cases: dict[str, dict[str, Any]] = {}
    summaries: dict[str, dict[str, Any]] = {}
    for name, config in MODE_CONFIGS.items():
        case = r300.run_case(
            model,
            disturbed=True,
            maximum_ticks=maximum_ticks,
            controller_options=config["controller_options"],
            controller_balance_mode=config["balance_mode"],
        )
        cases[name] = case
        summaries[name] = mode_summary(name, case)
    return {"cases": cases, "summaries": summaries}


def evaluate(metrics: dict[str, Any]) -> dict[str, bool]:
    summaries = metrics["summaries"]
    modes = tuple(summaries)
    return {
        "all_modes_use_250hz_physics_50hz_wbc": all(
            summary["live_rate_split"]
            == {
                "physics_hz": 250,
                "control_hz": 50,
                "physics_substeps_per_control": 5,
            }
            for summary in summaries.values()
        ),
        "all_modes_preserve_causal_prior_window": all(
            summary["causal_window_contract"] for summary in summaries.values()
        ),
        "all_outputs_finite": all(
            summary["finite_trace"] for summary in summaries.values()
        ),
        "hard_rows_subset_measured_contact": all(
            summary["hard_subset_raw"] for summary in summaries.values()
        ),
        "max_iterations_never_admitted": all(
            summary["max_iterations_nonadmitted"] for summary in summaries.values()
        ),
        "zero_timed_rust_allocations": all(
            summary["allocation_calls"] == 0 and summary["allocated_bytes"] == 0
            for summary in summaries.values()
        ),
        "worker_p99_under_20ms": all(
            summary["worker_step_us"]["p99"] < 20_000.0
            for summary in summaries.values()
        ),
        "public_capture_profile_is_unchanged": (
            summaries["production_capture"]["live_rate_split"]
            == summaries["support_contingency_r302"]["live_rate_split"]
            and MODE_CONFIGS["production_capture"]["balance_mode"] == "capture"
            and not MODE_CONFIGS["production_capture"]["controller_options"]
        ),
        # This is the negative behavior result.  It is reported as a failed
        # recovery gate, not hidden behind a delayed fall or automatic reset.
        "no_mode_reacquires_wheels_for_ten_ticks": all(
            not summaries[name]["strict_recovery"] for name in modes
        ),
    }


def render_report(metrics: dict[str, Any], gates: dict[str, bool]) -> str:
    summaries = metrics["summaries"]
    rows = [
        "# Upkie live contact-reacquisition mode comparison · R306",
        "",
        "> Negative evidence **PASS** when transport/causality/finite/allocation gates pass and every existing mode is rejected by the strict 10-tick wheel-supported recovery predicate.",
        "",
        "R306 replays the frozen R300 8 N lateral wrench against the existing capture, planar-capture, viability, and R302 support-contingency profiles. Every candidate is an explicit evaluation-only worker override; the public 250 Hz MuJoCo / 50 Hz WBC `production_default` path is not changed. The Rust request remains downstream of measured contact and is admitted only by the ordinary WBC boundary.",
        "",
        "Recovery requires ten consecutive 50 Hz samples after the disturbance with both measured wheel contacts, root height at least 0.48 m, root tilt at most 0.20 rad, and no pending reset. Delayed fall, body-ground support, and a geometric touch do not count.",
        "",
        "| mode | terminal tick | first flight | recovery tick | max body/no-wheel stall | min height m | max tilt rad | controller p99 µs | max residual |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, summary in summaries.items():
        max_residual = max(
            summary["maximum_constraint_violation"],
            summary["maximum_dynamics_residual"],
            summary["maximum_contact_residual"],
        )
        rows.append(
            "| {} | {} | {} | {} | {} | {:.5f} | {:.5f} | {:.1f} | {:.3e} |".format(
                name,
                summary["terminal_tick"]
                if summary["terminal_tick"] is not None
                else "—",
                summary["first_flight_tick"]
                if summary["first_flight_tick"] is not None
                else "—",
                summary["supported_upright_recovery_tick"]
                if summary["supported_upright_recovery_tick"] is not None
                else "—",
                summary["maximum_body_ground_stall_ticks"],
                summary["minimum_root_height_m"],
                summary["maximum_root_tilt_rad"],
                summary["controller_step_us"]["p99"],
                max_residual,
            )
        )
    rows.extend(
        [
            "",
            "## Gates",
            "",
            "| gate | result |",
            "|---|:---:|",
        ]
    )
    rows.extend(
        f"| {name} | {'PASS' if value else 'FAIL'} |"
        for name, value in gates.items()
    )
    rows.extend(
        [
            "",
            "## Interpretation",
            "",
            "The existing profiles provide useful fall-delay and residual experiments, but none produces a measured, force-backed wheel reacquisition on this frozen physical trace. R302's support-contingency request can keep the plant alive longer than the baseline, yet it does not create a new contact target when both wheels are absent. The next behavior slice must add a separately reviewed wheel-gap/landing request and then pass it through the ordinary WBC admission path; increasing QP iterations or relaxing timeout handling is not a contact-reacquisition mechanism.",
            "",
            "No policy, hidden estimator, automatic reset, public cadence change, or promoted actuator authority is used by this artifact.",
            "",
        ]
    )
    return "\n".join(rows)


def render_html(report: str) -> str:
    return "\n".join(
        (
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>Bonesaw Upkie contact reacquisition R306</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1200px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px;overflow:auto}table{border-collapse:collapse}th,td{padding:.25rem .5rem;border:1px solid #374151}</style>",
            f"<pre>{html.escape(report)}</pre>",
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--maximum-ticks", type=int, default=MAX_TICKS)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_LIVE_REACQUISITION_MODES_R306.html")
    args = parser.parse_args()
    if args.maximum_ticks < 20:
        raise SystemExit("--maximum-ticks must be at least 20")
    model = pathlib.Path(args.model).resolve()
    metrics = run_modes(model, args.maximum_ticks)
    metrics.update(
        {
            "revision": REVISION,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "maximum_ticks": args.maximum_ticks,
            "disturbance": {
                "force_world_n": [0.0, 8.0, 0.0],
                "push_start_tick": 25,
                "push_ticks": 10,
                "learned_policy_steps": 0,
            },
        }
    )
    gates = evaluate(metrics)
    metrics["gates"] = gates
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    report = render_report(metrics, gates)
    (output / "UPKIE_LIVE_REACQUISITION_MODES_R306.md").write_text(report)
    (output / "upkie-live-reacquisition-modes-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report))
    # A negative experiment is successful when the safety/evidence invariants
    # pass and the strict recovery gate remains negative for every mode.
    passed = all(gates.values())
    print(json.dumps({"passed": passed, "gates": gates}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
