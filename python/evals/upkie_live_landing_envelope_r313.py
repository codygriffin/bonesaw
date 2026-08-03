#!/usr/bin/env python3
"""R313 measure a continuous landing-request safety envelope.

R312 established a causal measured-contact landing boundary but its free-leg
request can spend support margin during an off-CoM moment. R313 keeps the
request in Rust and continuously attenuates it as measured root tilt,
horizontal speed, or height approaches a declared safety boundary. This is a
default-off falsifier: promotion requires monotonic behavior on the R311
upper-base holdout, not merely finite or visually smoother output.
"""

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
import upkie_live_measured_landing_r312 as r312


REVISION = "upkie-live-landing-envelope-r313"
TICKS = 450
FORCES_N = r312.COMPOSITION_FORCES_N
ENVELOPE_CONFIG = (0.20, 2.0, 0.34, 0.08)
BASE_OPTIONS = dict(r312.PUBLIC_CONTROLLER_OPTIONS)
R312_OPTIONS = dict(r312.PUBLIC_LANDING_CONTROLLER_OPTIONS)
CANDIDATE_OPTIONS = {
    **R312_OPTIONS,
    "measured_landing_envelope_config": ENVELOPE_CONFIG,
}
WORKER_OPTIONS = dict(r312.PUBLIC_WORKER_OPTIONS)


def _semantic_digest(case: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(r300._semantic(case), sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _request_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    authority = np.asarray(
        [
            state["measured_landing_diagnostics"][
                r312.DIAGNOSTIC_INDEX["request_authority"]
            ]
            for state in states
        ],
        np.float64,
    )
    active = [
        int(state["index"])
        for state in states
        if state["measured_landing_diagnostics"][
            r312.DIAGNOSTIC_INDEX["request_active"]
        ]
        > 0.5
    ]
    return {
        "active_ticks": len(active),
        "first_active_tick": active[0] if active else None,
        "last_active_tick": active[-1] if active else None,
        "maximum_effective_authority": float(np.max(authority, initial=0.0)),
        "partial_authority_ticks": int(
            np.count_nonzero((authority > 0.0) & (authority < 1.0))
        ),
        "step_us_p99": float(
            np.percentile(
                np.asarray(
                    [state["measured_landing_step_us"] for state in states],
                    np.float64,
                ),
                99.0,
            )
        ),
        "allocation_calls": int(
            sum(state["measured_landing_allocation_calls"] for state in states)
        ),
        "allocated_bytes": int(
            sum(state["measured_landing_allocated_bytes"] for state in states)
        ),
    }


def _row(
    force_y_n: float,
    baseline: dict[str, Any],
    r312_case: dict[str, Any],
    candidate: dict[str, Any],
) -> dict[str, Any]:
    baseline_summary = r310.summarize(baseline)
    r312_summary = r310.summarize(r312_case)
    candidate_summary = r310.summarize(candidate)
    return {
        "force_y_n": float(force_y_n),
        "baseline": baseline_summary,
        "r312": r312_summary,
        "candidate": candidate_summary,
        "r312_request": _request_summary(r312_case),
        "candidate_request": _request_summary(candidate),
        "terminal_delta_vs_baseline": (
            candidate_summary["terminal_tick"] - baseline_summary["terminal_tick"]
            if candidate_summary["terminal_tick"] is not None
            and baseline_summary["terminal_tick"] is not None
            else None
        ),
        "terminal_delta_vs_r312": (
            candidate_summary["terminal_tick"] - r312_summary["terminal_tick"]
            if candidate_summary["terminal_tick"] is not None
            and r312_summary["terminal_tick"] is not None
            else None
        ),
        "baseline_case": baseline,
        "r312_case": r312_case,
        "candidate_case": candidate,
    }


def evaluate(rows: list[dict[str, Any]], replay: dict[str, Any], *, ticks: int) -> dict[str, Any]:
    candidate_cases = [row["candidate_case"] for row in rows]
    candidate_summaries = [row["candidate"] for row in rows]
    candidate_requests = [row["candidate_request"] for row in rows]
    harness_gates = {
        "complete_r311_force_holdout": len(rows) == len(FORCES_N),
        "causal_measured_window": all(
            r312._causal_measured_window(case["states"])
            for case in candidate_cases
        ),
        "mode_firewall": all(
            r312._mode_firewall(case["states"]) for case in candidate_cases
        ),
        "finite_outputs": all(
            r312._finite_outputs(case["states"]) for case in candidate_cases
        ),
        "zero_landing_allocations": all(
            request["allocation_calls"] == 0 and request["allocated_bytes"] == 0
            for request in candidate_requests
        ),
        "landing_deadline": all(
            request["step_us_p99"] < 100.0 for request in candidate_requests
        ),
        "controller_and_worker_deadlines": all(
            summary["controller_step_us"]["p99"] < 5_000.0
            and summary["worker_step_us"]["p99"] < 20_000.0
            for summary in candidate_summaries
        ),
        "candidate_replay_exact": _semantic_digest(rows[-1]["candidate_case"])
        == _semantic_digest(replay),
    }
    def terminal_boundary_is_not_earlier(row: dict[str, Any]) -> bool:
        """Treat completion as strictly better than a baseline terminal fall."""
        baseline = row["baseline"]
        candidate = row["candidate"]
        baseline_terminal = baseline["terminal_pending"] is not None
        candidate_terminal = candidate["terminal_pending"] is not None
        if not baseline_terminal:
            # A candidate may not introduce a terminal boundary where the
            # baseline completed the declared horizon.
            return not candidate_terminal
        if not candidate_terminal:
            return True
        return candidate["terminal_tick"] >= baseline["terminal_tick"]

    promotion_gates = {
        "candidate_never_moves_terminal_boundary_earlier": all(
            terminal_boundary_is_not_earlier(row) for row in rows
        ),
        "candidate_reduces_terminal_fall_count": sum(
            row["candidate"]["terminal_pending"] is not None for row in rows
        )
        < sum(row["baseline"]["terminal_pending"] is not None for row in rows),
        "candidate_completes_every_case": all(
            summary["ticks"] == ticks and summary["terminal_pending"] is None
            for summary in candidate_summaries
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "rates_hz": {"wbc": 50, "physics": 250, "stream": 50},
        "candidate_default_enabled": False,
        "envelope_config": list(ENVELOPE_CONFIG),
        "forces_y_n": list(FORCES_N),
        "ticks": ticks,
        "rows": [
            {key: value for key, value in row.items() if not key.endswith("_case")}
            for row in rows
        ],
        "harness_gates": harness_gates,
        "promotion_gates": promotion_gates,
        "harness_valid": all(harness_gates.values()),
        "qualified_for_public_default": all(harness_gates.values())
        and all(promotion_gates.values()),
        "finding": (
            "R313 supplies a Rust-owned continuous landing-request envelope, but "
            "the measured holdout remains negative: attenuation does not remove "
            "the terminal cases and still moves at least one boundary earlier. "
            "The envelope remains default-off safety/research evidence."
        ),
    }


def render_markdown(metrics: dict[str, Any]) -> str:
    lines = [
        f"# Upkie continuous landing-request envelope — R313",
        "",
        f"Status: **{'QUALIFIED DEFAULT-OFF MECHANISM' if metrics['harness_valid'] else 'FIXTURE FAILED'}**; public promotion **{'PROMOTED' if metrics['qualified_for_public_default'] else 'REJECTED'}**.",
        "",
        "R313 keeps R312's measured-contact phase boundary and adds a Rust-owned, "
        "allocation-free continuous authority envelope. Authority attenuates "
        "smoothly as root tilt, horizontal speed, or height approaches the limits "
        f"`{metrics['envelope_config']}`. No reset or solver-budget change is allowed.",
        "",
        "| force Y (N) | R310 baseline terminal | R312 terminal | R313 terminal | Δ vs baseline | R313 active ticks |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for row in metrics["rows"]:
        delta = row["terminal_delta_vs_baseline"]
        delta_text = f"{delta:+g}" if delta is not None else "—"
        lines.append(
            f"| {row['force_y_n']:+g} | {row['baseline']['terminal_tick'] or '—'} | "
            f"{row['r312']['terminal_tick'] or '—'} | {row['candidate']['terminal_tick'] or '—'} | "
            f"{delta_text} | {row['candidate_request']['active_ticks']} |"
        )
    lines += ["", "## Mechanism gates", ""]
    lines += [
        f"- {'PASS' if value else 'FAIL'} `{name}`"
        for name, value in metrics["harness_gates"].items()
    ]
    lines += ["", "## Physical promotion gates", ""]
    lines += [
        f"- {'PASS' if value else 'OPEN'} `{name}`"
        for name, value in metrics["promotion_gates"].items()
    ]
    lines += ["", "## Conclusion", "", metrics["finding"], ""]
    return "\n".join(lines)


def run(model: pathlib.Path, *, ticks: int = TICKS) -> tuple[dict[str, Any], dict[str, Any], str]:
    rows: list[dict[str, Any]] = []
    for force_y_n in FORCES_N:
        common = {
            "model_path": model,
            "disturbed": True,
            "maximum_ticks": ticks,
            "worker_options": WORKER_OPTIONS,
            "force_world_n": (0.0, force_y_n, 0.0),
            "application_offset_world_m": (0.0, 0.0, r312.COMPOSITION_OFFSET_Z_M),
            "push_windows": r312.COMPOSITION_WINDOWS,
        }
        baseline = r300.run_case(controller_options=BASE_OPTIONS, **common)
        r312_case = r300.run_case(controller_options=R312_OPTIONS, **common)
        candidate = r300.run_case(controller_options=CANDIDATE_OPTIONS, **common)
        rows.append(_row(force_y_n, baseline, r312_case, candidate))
    replay = r300.run_case(
        model,
        disturbed=True,
        maximum_ticks=ticks,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=WORKER_OPTIONS,
        force_world_n=(0.0, FORCES_N[-1], 0.0),
        application_offset_world_m=(0.0, 0.0, r312.COMPOSITION_OFFSET_Z_M),
        push_windows=r312.COMPOSITION_WINDOWS,
    )
    metrics = evaluate(rows, replay, ticks=ticks)
    traces = {
        "revision": REVISION,
        "representative_baseline": rows[-1]["baseline_case"],
        "representative_r312": rows[-1]["r312_case"],
        "representative_r313": rows[-1]["candidate_case"],
        "representative_r313_replay": replay,
    }
    return metrics, traces, render_markdown(metrics)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf"))
    parser.add_argument("--ticks", type=int, default=TICKS)
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results/upkie-live-landing-envelope-r313"),
    )
    args = parser.parse_args()
    if args.ticks < 100:
        parser.error("ticks must be >=100")
    metrics, traces, markdown = run(args.model, ticks=args.ticks)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (args.output_dir / "traces.json").write_text(json.dumps(traces, separators=(",", ":")) + "\n")
    (args.output_dir / "UPKIE_LIVE_LANDING_ENVELOPE_R313.md").write_text(markdown)
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["harness_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
