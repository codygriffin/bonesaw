#!/usr/bin/env python3
"""Measured 250/50 support-contingency handoff on the live Upkie plant.

R300 demonstrated that measured support loss reaches the WBC but that the
primary solve can exhaust its iteration budget on the resulting one-wheel /
flight trace.  R301 enables the existing default-off Rust contingency author
and executes its candidate only after the ordinary admission checks pass.  It
is a graceful degraded-mode experiment: the candidate must stay finite,
allocation-free, and avoid an unsolved command, but it is not called a
recovery or walking result because the run is still expected to fall later.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any

from upkie_live_dynamic_contact_transition_r300 import (
    _semantic,
    compare,
    run_case,
    summarize,
)


REVISION = "upkie-live-support-contingency-r301"
MAX_TICKS = 100
SUPPORT_CONTINGENCY_OPTIONS: dict[str, Any] = {
    "support_contingency_enabled": True,
    "support_contingency_execute": True,
    "support_contingency_flight_only": True,
    "support_contingency_forecast_guard": True,
    # Explicitly record the bounded degraded-mode gains.  The public worker
    # passes no override and therefore remains production_default.
    "support_contingency_config": (
        9.81,  # gravity m/s²
        4.0,  # double-support CoM position gain
        8.0,  # single-support CoM position gain
        4.0,  # supported linear damping
        80.0,  # attitude stiffness
        30.0,  # angular damping
        12.0,  # joint damping
        8.0,  # horizontal acceleration limit
        20.0,  # vertical braking acceleration limit
        80.0,  # angular acceleration limit
        120.0,  # joint acceleration limit
    ),
}
MODE_NAMES = {0: "double_support", 1: "single_support", 2: "flight"}


def _support_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    base = summarize(case)
    selected = [state for state in states if state["support_contingency_selected"]]
    requested = [state for state in states if state["support_contingency_requested"]]
    admitted = [state for state in states if state["support_contingency_admitted"]]
    modes = sorted(
        {
            MODE_NAMES.get(state["support_contingency_mode"], "unknown")
            for state in states
            if state["support_contingency_requested"]
        }
    )
    status_counts = {
        status: sum(state["support_contingency_status"] == status for state in states)
        for status in sorted(
            {state["support_contingency_status"] for state in states}
        )
    }
    base.update(
        {
            "support_contingency_requested_ticks": [
                state["index"] for state in requested
            ],
            "support_contingency_admitted_ticks": [
                state["index"] for state in admitted
            ],
            "support_contingency_selected_ticks": [
                state["index"] for state in selected
            ],
            "support_contingency_first_selected_tick": (
                selected[0]["index"] if selected else None
            ),
            "support_contingency_modes": modes,
            "support_contingency_status_counts": status_counts,
            "support_contingency_maximum_constraint_violation": max(
                (
                    state["support_contingency_maximum_constraint_violation"]
                    for state in states
                ),
                default=0.0,
            ),
            "support_contingency_maximum_power_w": max(
                (
                    abs(state["support_contingency_candidate_power_w"])
                    for state in states
                ),
                default=0.0,
            ),
            "support_contingency_maximum_incremental_power_w": max(
                (
                    abs(state["support_contingency_incremental_power_w"])
                    for state in states
                ),
                default=0.0,
            ),
        }
    )
    return base


def _support_step_distribution(case: dict[str, Any]) -> dict[str, float]:
    """Return a small timing distribution without reusing WBC timing fields."""
    values = sorted(
        float(state["support_contingency_step_us"]) for state in case["states"]
    )
    if not values:
        return {"minimum": 0.0, "mean": 0.0, "p99": 0.0, "maximum": 0.0}
    index = lambda fraction: values[min(int(math.ceil(fraction * len(values))) - 1, len(values) - 1)]
    return {
        "minimum": values[0],
        "mean": sum(values) / len(values),
        "p99": index(0.99),
        "maximum": values[-1],
    }


def evaluate(
    baseline: dict[str, Any],
    candidate: dict[str, Any],
    replay: dict[str, Any],
) -> dict[str, bool]:
    baseline_summary = _support_summary(baseline)
    candidate_summary = _support_summary(candidate)
    candidate_states = candidate["states"]
    baseline_terminal = baseline_summary["terminal_tick"]
    candidate_terminal = candidate_summary["terminal_tick"]
    candidate_selected = [
        state for state in candidate_states if state["support_contingency_selected"]
    ]
    return {
        "live_rate_split_is_250_50": candidate["hello"]
        == {"physics_hz": 250, "control_hz": 50, "physics_substeps_per_control": 5},
        "baseline_reproduces_primary_fall": baseline_summary["terminal_pending"]
        == "fall"
        and baseline_terminal is not None,
        "candidate_also_reports_boundary_without_reset": candidate_summary[
            "terminal_pending"
        ]
        == "fall"
        and candidate_terminal is not None
        and candidate_summary["reset_epochs"] == [0],
        "candidate_delays_boundary_by_at_least_20_ticks": (
            candidate_terminal is not None
            and baseline_terminal is not None
            and candidate_terminal - baseline_terminal >= 20
        ),
        "candidate_measures_all_contact_modes": candidate_summary[
            "observed_patterns"
        ]
        == ["00", "01", "10", "11"],
        "candidate_contingency_is_flight_only": candidate_summary[
            "support_contingency_modes"
        ]
        == ["flight"],
        "selection_only_after_request_and_admission": all(
            state["support_contingency_requested"]
            and state["support_contingency_admitted"]
            and state["support_contingency_status"] in {"Solved", "SolvedWithSlack"}
            for state in candidate_selected
        ),
        "selection_only_without_double_support": all(
            state["observed"] != [1, 1] for state in candidate_selected
        ),
        "candidate_eliminates_primary_max_iterations": not candidate_summary[
            "max_iterations_ticks"
        ],
        "candidate_residuals_under_1e8": candidate_summary[
            "maximum_constraint_violation"
        ]
        <= 1.0e-8
        and candidate_summary["maximum_dynamics_residual"] <= 1.0e-8
        and candidate_summary["maximum_contact_residual"] <= 1.0e-8
        and candidate_summary["support_contingency_maximum_constraint_violation"]
        <= 1.0e-8,
        "candidate_controller_p99_under_5ms": candidate_summary[
            "controller_step_us"
        ]["p99"]
        < 5_000.0,
        "candidate_support_query_p99_under_1ms": _support_step_distribution(candidate)[
            "p99"
        ]
        < 1_000.0,
        "candidate_zero_rust_allocations": candidate_summary["allocation_calls"]
        == 0
        and candidate_summary["allocated_bytes"] == 0,
        "candidate_no_numeric_reset_or_warning": not candidate_summary[
            "numeric_reset"
        ]
        and candidate_summary["warning_count"] == 0,
        "candidate_replay_is_semantically_exact": _semantic(candidate)
        == _semantic(replay),
        "hard_rows_remain_subset_of_measured_contact": candidate_summary[
            "hard_subset_raw"
        ],
    }


def render_report(
    baseline: dict[str, Any], candidate: dict[str, Any], gates: dict[str, bool]
) -> str:
    b = _support_summary(baseline)
    c = _support_summary(candidate)
    candidate_support_timing = _support_step_distribution(candidate)
    paired = compare(baseline, candidate)
    lines = [
        "# Upkie live support-contingency handoff · R301",
        "",
        "> Degraded-mode handoff **PASS** when all gates below pass; recovery is deliberately **NOT CLAIMED**.",
        "",
        "Both cases use the measured 250 Hz MuJoCo / 50 Hz Rust WBC worker, the Upkie reference model, the same 8 N lateral wrench for 200 ms, and no policy. The candidate enables the default-off Rust support-contingency author with an explicit bounded degraded-mode gain profile. The ordinary WBC admission boundary remains authoritative: a contingency request is executed only when it is finite, hard-feasible, and admitted.",
        "",
        "| metric | primary production default | contingency candidate |",
        "|---|---:|---:|",
        f"| terminal boundary tick | {b['terminal_tick']} | {c['terminal_tick']} |",
        f"| terminal time s | {b['duration_s']:.3f} | {c['duration_s']:.3f} |",
        f"| boundary delay ticks / ms | — | {(c['terminal_tick'] - b['terminal_tick']) if c['terminal_tick'] is not None and b['terminal_tick'] is not None else '—'} / {((c['terminal_tick'] - b['terminal_tick']) * 20) if c['terminal_tick'] is not None and b['terminal_tick'] is not None else '—'} |",
        f"| contact patterns | {', '.join(b['observed_patterns'])} | {', '.join(c['observed_patterns'])} |",
        f"| primary MaxIterations ticks | {b['max_iterations_ticks'] or '—'} | {c['max_iterations_ticks'] or '—'} |",
        f"| max dynamics / contact residual | {b['maximum_dynamics_residual']:.3e} / {b['maximum_contact_residual']:.3e} | {c['maximum_dynamics_residual']:.3e} / {c['maximum_contact_residual']:.3e} |",
        f"| controller p50 / p99 / max µs | {b['controller_step_us']['p50']:.1f} / {b['controller_step_us']['p99']:.1f} / {b['controller_step_us']['maximum']:.1f} | {c['controller_step_us']['p50']:.1f} / {c['controller_step_us']['p99']:.1f} / {c['controller_step_us']['maximum']:.1f} |",
        f"| contingency selected ticks | — | {len(c['support_contingency_selected_ticks'])} ({c['support_contingency_first_selected_tick']}…{c['support_contingency_selected_ticks'][-1] if c['support_contingency_selected_ticks'] else '—'}) |",
        f"| contingency modes | — | {', '.join(c['support_contingency_modes']) or '—'} |",
        f"| contingency query p50 / p99 / max µs | — | {candidate_support_timing['mean']:.1f} / {candidate_support_timing['p99']:.1f} / {candidate_support_timing['maximum']:.1f} |",
        f"| candidate root trace delta RMS m | — | {paired['root_trace_delta_rms_m']:.5f} |",
        f"| Rust allocation calls / bytes | {b['allocation_calls']} / {b['allocated_bytes']} | {c['allocation_calls']} / {c['allocated_bytes']} |",
        "",
        "## Interpretation",
        "",
        "The candidate is a bounded degraded-mode handoff: it avoids forwarding the primary solver's unsolved command, keeps the residual witnesses finite, and buys a measurable additional fall horizon. It does not stabilize the robot indefinitely; the terminal fall is retained as a required boundary and is not hidden by an automatic reset. The profile remains evaluation-only until a stronger support-feasible braking/recovery criterion exists.",
        "",
        "## Gates",
        "",
        "| gate | result |",
        "|---|:---:|",
    ]
    lines.extend(
        f"| {name} | {'PASS' if value else 'FAIL'} |"
        for name, value in gates.items()
    )
    lines.append("")
    return "\n".join(lines)


def render_html(report: str, passed: bool) -> str:
    return "\n".join(
        (
            "<!doctype html>",
            "<meta name='viewport' content='width=device-width,initial-scale=1'>",
            "<title>Bonesaw Upkie support-contingency R301</title>",
            "<style>body{font:15px system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem;background:#111827;color:#e5e7eb}pre{white-space:pre-wrap;line-height:1.45;background:#1f2937;padding:1rem;border-radius:8px}.pass{color:#86efac}.fail{color:#fca5a5}</style>",
            f"<h1 class='{'pass' if passed else 'fail'}'>Upkie live support-contingency handoff · R301</h1>",
            f"<pre>{html.escape(report)}</pre>",
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/UPKIE_LIVE_SUPPORT_CONTINGENCY_R301.html")
    args = parser.parse_args()
    model = pathlib.Path(args.model)
    baseline = run_case(model, disturbed=True, maximum_ticks=MAX_TICKS)
    candidate = run_case(
        model,
        disturbed=True,
        maximum_ticks=MAX_TICKS,
        controller_options=SUPPORT_CONTINGENCY_OPTIONS,
    )
    replay = run_case(
        model,
        disturbed=True,
        maximum_ticks=MAX_TICKS,
        controller_options=SUPPORT_CONTINGENCY_OPTIONS,
    )
    gates = evaluate(baseline, candidate, replay)
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "passed": passed,
        "options": SUPPORT_CONTINGENCY_OPTIONS,
        "gates": gates,
        "baseline_summary": _support_summary(baseline),
        "candidate_summary": _support_summary(candidate),
        "paired_comparison": compare(baseline, candidate),
        "baseline": baseline,
        "candidate": candidate,
        "candidate_replay": replay,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-live-support-contingency-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    report = render_report(baseline, candidate, gates)
    (output / "UPKIE_LIVE_SUPPORT_CONTINGENCY_R301.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_html(report, passed))
    print(json.dumps({"passed": passed, "gates": gates}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
