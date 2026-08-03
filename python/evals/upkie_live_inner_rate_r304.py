#!/usr/bin/env python3
"""R304 qualify the live 250 Hz WBC / 1 kHz plant inner loop.

The browser transport remains 50 Hz.  This checkpoint promotes only the
transport/control/physics rate separation and the balanced-pose target needed
for nominal standing.  The existing 8 N lateral disturbance remains an
explicit red recovery holdout.
"""

from __future__ import annotations

import argparse
import html
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import upkie_live_dynamic_contact_transition_r300 as r300


REVISION = "upkie-live-inner-rate-r304"
FAST_WORKER_OPTIONS = {
    "stream_dt": 0.020,
    "control_dt": 0.004,
    "physics_dt": 0.001,
    "balanced_nominal_joint_target": True,
}


def _rates(case: dict[str, Any]) -> dict[str, int]:
    return {
        "stream_hz": 50,
        "control_hz": int(case["hello"]["control_hz"]),
        "physics_hz": int(case["hello"]["physics_hz"]),
        "physics_substeps_per_control": int(
            case["hello"]["physics_substeps_per_control"]
        ),
    }


def evaluate_cases(
    legacy_control: dict[str, Any],
    fast_control: dict[str, Any],
    fast_disturbed: dict[str, Any],
    *,
    nominal_ticks: int,
) -> dict[str, Any]:
    legacy = r300.summarize(legacy_control)
    nominal = r300.summarize(fast_control)
    disturbed = r300.summarize(fast_disturbed)
    nominal_states = fast_control["states"]
    disturbed_states = fast_disturbed["states"]
    qualification_gates = {
        "public_rate_contract_is_50_250_1000": _rates(fast_control)
        == {
            "stream_hz": 50,
            "control_hz": 250,
            "physics_hz": 1000,
            "physics_substeps_per_control": 4,
        },
        "public_uses_rust_balanced_nominal_joint_target": (
            fast_control["nominal_joint_target"] == "rust_balanced_initial_pose"
            and legacy_control["nominal_joint_target"]
            == "legacy_adapter_standing_pose"
        ),
        "legacy_50hz_control_reaches_fall_boundary": (
            legacy["terminal_pending"] == "fall"
        ),
        "fast_nominal_completes_horizon": (
            nominal["ticks"] == nominal_ticks
            and nominal["terminal_pending"] is None
            and not nominal["numeric_reset"]
        ),
        "fast_nominal_stays_bilateral": nominal["observed_patterns"] == ["11"],
        "fast_nominal_stays_upright": (
            nominal["minimum_root_height_m"] >= 0.48
            and nominal["maximum_root_tilt_rad"] <= 0.20
        ),
        "fast_nominal_all_solved_and_admitted": (
            nominal["wbc_status_counts"] == {"Solved": nominal_ticks}
            and not nominal["nonadmitted_ticks"]
        ),
        "fast_nominal_hard_residuals_under_1e8": max(
            nominal["maximum_constraint_violation"],
            nominal["maximum_dynamics_residual"],
            nominal["maximum_contact_residual"],
        )
        <= 1.0e-8,
        "fast_nominal_rust_hot_loop_zero_allocations": (
            nominal["allocation_calls"] == 0 and nominal["allocated_bytes"] == 0
        ),
        "fast_nominal_controller_p99_under_1ms": (
            nominal["controller_step_us"]["p99"] < 1_000.0
        ),
        "fast_nominal_worker_p99_under_20ms_stream_budget": (
            nominal["worker_step_us"]["p99"] < 20_000.0
        ),
        "disturbance_trace_is_finite_and_fail_closed": (
            bool(disturbed_states)
            and not disturbed["numeric_reset"]
            and disturbed["warning_count"] == 0
            and disturbed["hard_subset_raw"]
            and all(
                state["wbc_status"] != "MaxIterations"
                or not state["wbc_admitted"]
                for state in disturbed_states
            )
        ),
    }
    open_recovery_gates = {
        "eight_newton_lateral_pull_recovers": (
            disturbed["terminal_pending"] is None
            and disturbed["supported_upright_recovery_tick"] is not None
            and disturbed["maximum_body_ground_stall_ticks"] == 0
        ),
        "eight_newton_lateral_pull_has_no_max_iterations": (
            not disturbed["max_iterations_ticks"]
        ),
        "eight_newton_lateral_pull_stays_within_20ms_stream_budget": (
            disturbed["worker_step_us"]["p99"] < 20_000.0
        ),
    }
    first_fast_loss = disturbed["first_non_double_tick"]
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "promotion": {
            "transport_control_physics_rate_separation": True,
            "balanced_pose_joint_target": True,
            "lateral_recovery_action": False,
        },
        "rates": {
            "legacy": _rates(legacy_control),
            "public": _rates(fast_control),
        },
        "horizons": {
            "requested_nominal_ticks": nominal_ticks,
            "requested_nominal_seconds": nominal_ticks * 0.020,
            "legacy_completed_seconds": legacy["duration_s"],
            "fast_nominal_completed_seconds": nominal["duration_s"],
            "disturbed_completed_seconds": disturbed["duration_s"],
        },
        "legacy_control": legacy,
        "fast_nominal": nominal,
        "fast_disturbed": disturbed,
        "disturbance_boundary": {
            "force_world_n": list(r300.FORCE_WORLD_N),
            "start_stream_tick": r300.PUSH_START_TICK,
            "duration_stream_ticks": r300.PUSH_TICKS,
            "first_non_bilateral_stream_tick": first_fast_loss,
            "first_non_bilateral_time_s": (
                None if first_fast_loss is None else first_fast_loss * 0.020
            ),
            "fall_boundary_stream_tick": disturbed["terminal_tick"],
            "max_iterations_stream_ticks": disturbed["max_iterations_ticks"],
        },
        "qualification_gates": qualification_gates,
        "open_recovery_gates": open_recovery_gates,
        "qualified": all(qualification_gates.values()),
        "recovery_promoted": all(open_recovery_gates.values()),
        "authority_statement": (
            "R304 changes the public execution cadence and nominal joint target only. "
            "It does not add a Python policy or promote disturbance-recovery authority."
        ),
        "trace_counts": {
            "legacy_control": len(legacy_control["states"]),
            "fast_nominal": len(nominal_states),
            "fast_disturbed": len(disturbed_states),
        },
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    legacy = metrics["legacy_control"]
    nominal = metrics["fast_nominal"]
    disturbed = metrics["fast_disturbed"]
    gates = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} `{name}`"
        for name, passed in metrics["qualification_gates"].items()
    )
    recovery = "\n".join(
        f"- {'PASS' if passed else 'OPEN'} `{name}`"
        for name, passed in metrics["open_recovery_gates"].items()
    )
    return f"""# Upkie live inner-rate qualification — R304

Status: **{'QUALIFIED' if metrics['qualified'] else 'REJECTED'} for nominal live execution; lateral recovery remains unpromoted.**

## Outcome

The browser still receives 50 Hz state frames, while each frame now contains five
250 Hz Rust WBC ticks and each WBC tick contains four 1 kHz MuJoCo substeps. With
the Rust-balanced initialization also used as the nominal joint target, the live
plant completes {metrics['horizons']['fast_nominal_completed_seconds']:.3f} s in
bilateral support. The frozen 50 Hz WBC / 250 Hz plant baseline reaches its fall
boundary after {metrics['horizons']['legacy_completed_seconds']:.3f} s.

This promotes an execution architecture, not a recovery policy. The same 8 N
lateral pull still loses bilateral support at stream tick
{metrics['disturbance_boundary']['first_non_bilateral_stream_tick']} and reaches a
fall boundary at tick {metrics['disturbance_boundary']['fall_boundary_stream_tick']}.

## Rate contract

| path | browser stream | Rust WBC | MuJoCo | substeps / WBC |
|---|---:|---:|---:|---:|
| frozen baseline | 50 Hz | 50 Hz | 250 Hz | 5 |
| public R304 | 50 Hz | 250 Hz | 1000 Hz | 4 |

## Nominal 20-second hold

| measure | result |
|---|---:|
| completed ticks | {nominal['ticks']} |
| measured contact patterns | {', '.join(nominal['observed_patterns'])} |
| minimum root height | {nominal['minimum_root_height_m']:.6f} m |
| maximum root tilt | {nominal['maximum_root_tilt_rad']:.6f} rad |
| maximum hard residual | {max(nominal['maximum_constraint_violation'], nominal['maximum_dynamics_residual'], nominal['maximum_contact_residual']):.3e} |
| WBC status counts | `{json.dumps(nominal['wbc_status_counts'], sort_keys=True)}` |
| Rust allocations / bytes | {nominal['allocation_calls']} / {nominal['allocated_bytes']} |
| controller p50 / p95 / p99 | {nominal['controller_step_us']['p50']:.3f} / {nominal['controller_step_us']['p95']:.3f} / {nominal['controller_step_us']['p99']:.3f} us |
| controller adjacent jitter p99 | {nominal['controller_adjacent_jitter_us']['p99']:.3f} us |
| worker p50 / p95 / p99 | {nominal['worker_step_us']['p50']:.3f} / {nominal['worker_step_us']['p95']:.3f} / {nominal['worker_step_us']['p99']:.3f} us |
| worker adjacent jitter p99 | {nominal['worker_adjacent_jitter_us']['p99']:.3f} us |

## Frozen baseline consequence

The baseline completes {legacy['ticks']} stream ticks, loses bilateral wheel
support at tick {legacy['first_non_double_tick']}, and reaches
`{legacy['terminal_pending']}` at tick {legacy['terminal_tick']}. Its peak tilt is
{legacy['maximum_root_tilt_rad']:.6f} rad and minimum root height is
{legacy['minimum_root_height_m']:.6f} m. This comparison isolates execution rate;
the model, WBC formulation, and 50 Hz observation/visualization cadence are unchanged.

## Disturbance holdout

The 8 N lateral wrench runs for {r300.PUSH_TICKS * 0.020:.3f} s. It first loses
bilateral support at tick {disturbed['first_non_double_tick']}, reaches flight at
tick {disturbed['first_flight_tick']}, and terminates with
`{disturbed['terminal_pending']}` at tick {disturbed['terminal_tick']}.
MaxIterations occurs at stream ticks
`{json.dumps(disturbed['max_iterations_ticks'])}`; those outputs are non-admitted.
The hot Rust solve still reports {disturbed['allocation_calls']} allocations.

## Qualification gates

{gates}

## Deliberately open recovery gates

{recovery}

## Authority boundary

{metrics['authority_statement']} Python declares the immutable cases, aggregates
metrics, and renders this report. The WBC and actuator solve remain Rust-owned.
Full per-tick state summaries are retained in `traces.json` beside `metrics.json`.
"""


def render_html(markdown: str, metrics: dict[str, Any]) -> str:
    nominal = metrics["fast_nominal"]
    disturbed = metrics["fast_disturbed"]
    gate_cards = "".join(
        f'<li class="{"pass" if passed else "fail"}"><span>{"PASS" if passed else "FAIL"}</span>{html.escape(name)}</li>'
        for name, passed in metrics["qualification_gates"].items()
    )
    open_cards = "".join(
        f'<li class="{"pass" if passed else "open"}"><span>{"PASS" if passed else "OPEN"}</span>{html.escape(name)}</li>'
        for name, passed in metrics["open_recovery_gates"].items()
    )
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>R304 inner-rate qualification</title>
<style>
:root{{--bg:#071015;--panel:#101c23;--ink:#e8f1f3;--muted:#9db1b7;--green:#53e0a2;--orange:#ffb45e;--red:#ff6474;--line:#29404a}}
*{{box-sizing:border-box}} body{{margin:0;background:radial-gradient(circle at 80% 0,#18313a 0,transparent 35%),var(--bg);color:var(--ink);font:15px/1.5 system-ui,sans-serif}}
main{{max-width:1080px;margin:auto;padding:clamp(18px,4vw,54px)}} h1{{font-size:clamp(2rem,7vw,5rem);line-height:.94;margin:.2em 0}} h2{{margin-top:2rem}} .kicker{{color:var(--green);letter-spacing:.16em;text-transform:uppercase;font-weight:700}}
.lede{{max-width:760px;color:var(--muted);font-size:1.08rem}} .grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:28px 0}} .card{{background:color-mix(in srgb,var(--panel) 92%,transparent);border:1px solid var(--line);border-radius:14px;padding:18px}} .card b{{display:block;font-size:1.7rem}} .card small{{color:var(--muted)}}
table{{width:100%;border-collapse:collapse;background:var(--panel);border-radius:12px;overflow:hidden}} th,td{{padding:11px 13px;border-bottom:1px solid var(--line);text-align:left}} th{{color:var(--muted);font-weight:600}}
ul.gates{{list-style:none;padding:0;display:grid;grid-template-columns:repeat(2,1fr);gap:8px}} .gates li{{border:1px solid var(--line);border-radius:9px;padding:10px;background:var(--panel);overflow-wrap:anywhere}} .gates span{{display:inline-block;min-width:54px;color:var(--green);font-weight:800}} .gates .open span{{color:var(--orange)}} .gates .fail span{{color:var(--red)}}
details{{margin-top:30px}} pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#050a0d;border:1px solid var(--line);padding:16px;border-radius:10px;color:#bfd0d5}}
@media(max-width:720px){{.grid{{grid-template-columns:repeat(2,1fr)}} ul.gates{{grid-template-columns:1fr}} th,td{{padding:9px 8px;font-size:.85rem}}}}
</style></head><body><main><div class="kicker">Bonesaw · R304 · measured</div><h1>Fast inside.<br>Calm outside.</h1>
<p class="lede">The browser remains a smooth 50 Hz observer. The plant now runs five allocation-free Rust WBC solves per visible frame against a 1 kHz MuJoCo consequence loop. Nominal standing qualifies; lateral recovery does not.</p>
<div class="grid"><div class="card"><b>250 Hz</b><small>Rust WBC</small></div><div class="card"><b>1 kHz</b><small>MuJoCo physics</small></div><div class="card"><b>{nominal['duration_s']:.1f} s</b><small>bilateral nominal hold</small></div><div class="card"><b>{nominal['controller_step_us']['p99']:.1f} µs</b><small>controller p99</small></div></div>
<h2>Measured contrast</h2><table><tr><th>Case</th><th>Duration</th><th>First support loss</th><th>Boundary</th><th>Peak tilt</th></tr>
<tr><td>50 Hz WBC baseline</td><td>{metrics['legacy_control']['duration_s']:.3f} s</td><td>{metrics['legacy_control']['first_non_double_tick']}</td><td>{metrics['legacy_control']['terminal_pending']}</td><td>{metrics['legacy_control']['maximum_root_tilt_rad']:.4f} rad</td></tr>
<tr><td>250 Hz WBC nominal</td><td>{nominal['duration_s']:.3f} s</td><td>none</td><td>none</td><td>{nominal['maximum_root_tilt_rad']:.4f} rad</td></tr>
<tr><td>250 Hz + 8 N lateral</td><td>{disturbed['duration_s']:.3f} s</td><td>{disturbed['first_non_double_tick']}</td><td>{disturbed['terminal_pending']}</td><td>{disturbed['maximum_root_tilt_rad']:.4f} rad</td></tr></table>
<h2>Qualification</h2><ul class="gates">{gate_cards}</ul><h2>Recovery stays red</h2><ul class="gates">{open_cards}</ul>
<details><summary>Full engineering report</summary><pre>{html.escape(markdown)}</pre></details></main></body></html>"""


def run(
    model: pathlib.Path,
    *,
    nominal_ticks: int,
    disturbed_ticks: int,
) -> tuple[dict[str, Any], dict[str, Any], str, str]:
    legacy_control = r300.run_case(
        model, disturbed=False, maximum_ticks=nominal_ticks
    )
    fast_control = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=nominal_ticks,
        worker_options=FAST_WORKER_OPTIONS,
    )
    fast_disturbed = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=disturbed_ticks,
        worker_options=FAST_WORKER_OPTIONS,
    )
    metrics = evaluate_cases(
        legacy_control,
        fast_control,
        fast_disturbed,
        nominal_ticks=nominal_ticks,
    )
    traces = {
        "revision": REVISION,
        "legacy_control": legacy_control,
        "fast_nominal": fast_control,
        "fast_disturbed": fast_disturbed,
    }
    markdown = render_markdown(metrics)
    return metrics, traces, markdown, render_html(markdown, metrics)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf"))
    parser.add_argument("--nominal-ticks", type=int, default=1000)
    parser.add_argument("--disturbed-ticks", type=int, default=300)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results/upkie-live-inner-rate-r304"),
    )
    parser.add_argument(
        "--web-output",
        type=pathlib.Path,
        default=pathlib.Path("web/UPKIE_LIVE_INNER_RATE_R304.html"),
    )
    args = parser.parse_args()
    if args.nominal_ticks < 100 or args.disturbed_ticks < 50:
        parser.error("nominal-ticks must be >=100 and disturbed-ticks >=50")
    metrics, traces, markdown, html_report = run(
        args.model,
        nominal_ticks=args.nominal_ticks,
        disturbed_ticks=args.disturbed_ticks,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output_dir / "traces.json").write_text(
        json.dumps(traces, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    report_path = args.output_dir / "UPKIE_LIVE_INNER_RATE_R304.md"
    report_path.write_text(markdown, encoding="utf-8")
    args.web_output.write_text(html_report, encoding="utf-8")
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["qualified"] and not metrics["recovery_promoted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
