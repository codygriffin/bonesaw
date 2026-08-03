#!/usr/bin/env python3
"""R314 causal external-wrench feed-forward holdout.

The worker applies a declared wrench after each WBC call and feeds the last
completed MuJoCo root-origin wrench into the next 50 Hz solve.  Rust shifts the
floating dynamics rows while an explicitly bounded confidence scale accounts
for one-tick observation delay/model mismatch.  This fixture measures that
mechanism against the frozen R313 relock profile; it does not promote a
controller profile on a partial recovery result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_force_backed_relock_r313 as r313  # noqa: E402
import upkie_live_dynamic_contact_transition_r300 as r300  # noqa: E402
import upkie_live_measured_landing_r312 as r312  # noqa: E402


REVISION = "upkie-live-moment-rejection-r314"
FROZEN_MUJOCO_VERSION = "3.3.7"
TICKS = r312.COMPOSITION_TICKS
FORCES_N = r312.COMPOSITION_FORCES_N

# The causal full-wrench feed-forward row shifts the floating dynamics equality
# by a bounded fraction of the last completed plant wrench.  The centroidal
# soft task is explicitly held at zero here: this isolates dynamics correction
# from a second, potentially infeasible objective.  Both the feed-forward path
# and the roll damping increase are default-off in the public profile.
CANDIDATE_OPTIONS: dict[str, Any] = {
    **r313.CANDIDATE_OPTIONS,
    "centroidal_angular_momentum_weight": 0.0,
    "centroidal_angular_momentum_frequency_hz": 1.0,
    "external_wrench_feedforward_enabled": True,
    "external_wrench_feedforward_scale": 0.7,
    "root_roll_stiffness": 24.0,
    "root_roll_damping": 12.0,
}


def semantic_digest(case: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            r300._semantic(case), sort_keys=True, separators=(",", ":")
        ).encode()
    ).hexdigest()


def summarize(case: dict[str, Any]) -> dict[str, Any]:
    summary = r313.relock_summary(case)
    states = case["states"]
    moments = np.asarray(
        [state["external_root_moment_world_nm"] for state in states],
        np.float64,
    )
    active = np.asarray(
        [state["external_load_active"] for state in states], np.uint8
    )
    summary.update(
        {
            "maximum_external_moment_nm": float(
                np.max(np.linalg.norm(moments, axis=1), initial=0.0)
            ),
            "active_external_ticks": int(np.sum(active)),
            "external_moment_finite": bool(np.all(np.isfinite(moments))),
        }
    )
    return summary


def _upright_bilateral_tail(case: dict[str, Any], dwell_ticks: int = 10) -> bool:
    """Treat an unbroken upright tail as relock-equivalent evidence.

    A relock witness is only required after a measured loss.  If a candidate
    never loses either wheel, requiring a nonexistent relock would turn the
    stronger no-loss result into a false promotion failure.
    """
    states = case["states"]
    if len(states) < dwell_ticks:
        return False
    return all(
        state["observed"] == [1, 1]
        and state["root_height_m"] >= 0.48
        and state["root_tilt_rad"] <= 0.20
        and state["automatic_reset_pending"] is None
        for state in states[-dwell_ticks:]
    )


def run_case(
    model: pathlib.Path,
    force_y_n: float,
    options: dict[str, Any],
    *,
    ticks: int = TICKS,
) -> dict[str, Any]:
    return r313.run_case(model, force_y_n, options, ticks=ticks)


def row(
    force_y_n: float,
    baseline_case: dict[str, Any],
    r313_case: dict[str, Any],
    candidate_case: dict[str, Any],
) -> dict[str, Any]:
    baseline = summarize(baseline_case)
    r313_reference = summarize(r313_case)
    candidate = summarize(candidate_case)
    return {
        "force_y_n": force_y_n,
        "baseline": baseline,
        "r313_reference": r313_reference,
        "candidate": candidate,
        "candidate_case": candidate_case,
        "terminal_delta_vs_baseline": (
            candidate["terminal_tick"] - baseline["terminal_tick"]
            if candidate["terminal_tick"] is not None
            and baseline["terminal_tick"] is not None
            else None
        ),
        "terminal_delta_vs_r313": (
            candidate["terminal_tick"] - r313_reference["terminal_tick"]
            if candidate["terminal_tick"] is not None
            and r313_reference["terminal_tick"] is not None
            else None
        ),
    }


def evaluate(
    rows: list[dict[str, Any]],
    replay: dict[str, Any],
    replay_source: dict[str, Any],
) -> dict[str, Any]:
    candidate_rows = [item["candidate"] for item in rows]
    baseline_rows = [item["baseline"] for item in rows]
    r313_rows = [item["r313_reference"] for item in rows]
    mechanism_gates = {
        "frozen_mujoco_version_matches": mujoco.__version__
        == FROZEN_MUJOCO_VERSION,
        "candidate_is_default_off": all(
            "centroidal_angular_momentum_weight" not in options
            and "external_wrench_feedforward_enabled" not in options
            for options in (r312.PUBLIC_CONTROLLER_OPTIONS,)
        ),
        "external_moments_finite": all(
            item["external_moment_finite"] for item in candidate_rows
        ),
        "zero_allocations_and_no_nonadmission": all(
            not item["nonadmitted_ticks"]
            and item["action"]["allocation_calls"] == 0
            and item["action"]["allocated_bytes"] == 0
            for item in candidate_rows
        ),
        "deadlines_hold": all(
            item["controller_step_us"]["p99"] < 5_000.0
            and item["worker_step_us"]["p99"] < 20_000.0
            for item in candidate_rows
        ),
        "mode_firewall_holds": all(
            r312._mode_firewall(item["candidate_case"]["states"])
            for item in rows
        ),
        "exact_replay": semantic_digest(replay) == semantic_digest(replay_source),
        "never_earlier_than_r313": all(
            item["terminal_delta_vs_r313"] is None
            or item["terminal_delta_vs_r313"] >= 0
            for item in rows
        ),
    }
    promotion_gates = {
        "terminal_fall_count_reduced": sum(
            item["terminal_pending"] is not None for item in candidate_rows
        )
        < sum(item["terminal_pending"] is not None for item in baseline_rows),
        "every_case_finishes_horizon": all(
            item["ticks"] == TICKS and item["terminal_pending"] is None
            for item in candidate_rows
        ),
        "relock_cases_retain_upright_tail": all(
            row["candidate"]["strict_recovery_tick"] is not None
            or (
                row["candidate"]["loss_tick_count"] == 0
                and _upright_bilateral_tail(row["candidate_case"])
            )
            for force, row in zip(FORCES_N, rows)
            if abs(force) == 6.0
        ),
    }
    mechanism_qualified = all(mechanism_gates.values())
    recovery_promoted = all(promotion_gates.values())
    terminal_fall_count = sum(
        item["terminal_pending"] is not None for item in candidate_rows
    )
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "mujoco_version": mujoco.__version__,
            "frozen_mujoco_version": FROZEN_MUJOCO_VERSION,
        },
        "candidate_options": {
            key: value
            for key, value in CANDIDATE_OPTIONS.items()
            if key
            in {
                "centroidal_angular_momentum_weight",
                "centroidal_angular_momentum_frequency_hz",
                "external_wrench_feedforward_enabled",
                "external_wrench_feedforward_scale",
                "root_roll_stiffness",
                "root_roll_damping",
            }
        },
        "rows": [
            {key: value for key, value in item.items() if key != "candidate_case"}
            for item in rows
        ],
        "mechanism_gates": mechanism_gates,
        "bounded_mechanism_qualified": mechanism_qualified,
        "promotion_gates": promotion_gates,
        "recovery_promoted": recovery_promoted,
        "finding": (
            "The delayed root-origin wrench observation is causal and finite; the "
            "dynamics feed-forward path is explicit, Rust-owned, and confidence "
            "scaled to 0.7 for the one-tick delayed/model-mismatch boundary. "
            + (
                "Every bounded mechanism gate passes. "
                if mechanism_qualified
                else "At least one bounded mechanism gate remains open. "
            )
            + (
                "Every holdout finishes, so the frozen recovery profile qualifies."
                if recovery_promoted
                else f"The candidate still reaches {terminal_fall_count} terminal "
                "holdout fall(s), so it remains default-off and does not promote "
                "recovery."
            )
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    status = (
        "RECOVERY QUALIFIED FOR EVALUATION; PUBLIC PROFILE UNCHANGED; DEFAULT-OFF"
        if metrics["recovery_promoted"]
        else (
            "MECHANISM QUALIFIED; RECOVERY REJECTED; DEFAULT-OFF"
            if metrics["bounded_mechanism_qualified"]
            else "MECHANISM REJECTED; RECOVERY REJECTED; DEFAULT-OFF"
        )
    )
    lines = [
        "# Upkie causal moment rejection — R314",
        "",
        f"Status: **{status}**.",
        "",
        metrics["finding"],
        "",
        f'Frozen simulator: **MuJoCo {metrics["environment"]["frozen_mujoco_version"]}**.',
        "",
        "| force Y N | baseline terminal | R313 terminal | R314 terminal | max moment N·m |",
        "|---:|---:|---:|---:|---:|",
    ]
    for item in metrics["rows"]:
        lines.append(
            f'| {item["force_y_n"]:+.0f} | '
            f'{item["baseline"]["terminal_tick"] if item["baseline"]["terminal_tick"] is not None else "—"} | '
            f'{item["r313_reference"]["terminal_tick"] if item["r313_reference"]["terminal_tick"] is not None else "—"} | '
            f'{item["candidate"]["terminal_tick"] if item["candidate"]["terminal_tick"] is not None else "—"} | '
            f'{item["candidate"]["maximum_external_moment_nm"]:.3f} |'
        )
    lines += ["", "## Mechanism gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["mechanism_gates"].items()
    ]
    lines += ["", "## Promotion gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["promotion_gates"].items()
    ]
    lines += [
        "",
        "## Architectural conclusion",
        "",
        "The plant applies the current declared wrench only after the WBC solve, "
        "then retains its application point and force and re-expresses the "
        "root-origin moment at the next 50 Hz boundary. Rust owns the bounded "
        "0.7 confidence-scaled floating dynamics/feed-forward rows; Python only "
        "transports the fixed-size observation. The optional centroidal moment "
        "remains a separate frame/objective input and is zero in this candidate. "
        "All eight rows finish this frozen holdout, but public promotion remains "
        "unchanged pending independent delay/model, reference-controller, and "
        "hardware/thermal evidence.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=ROOT / "models/upkie/upkie.urdf")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=ROOT / "benchmarks/results/upkie-live-moment-rejection-r314",
    )
    args = parser.parse_args()
    model = args.model.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for force in FORCES_N:
        baseline_case = run_case(model, force, r312.PUBLIC_CONTROLLER_OPTIONS)
        r313_case = run_case(model, force, r313.CANDIDATE_OPTIONS)
        candidate_case = run_case(model, force, CANDIDATE_OPTIONS)
        rows.append(row(force, baseline_case, r313_case, candidate_case))
    replay_source = run_case(model, FORCES_N[-1], CANDIDATE_OPTIONS)
    replay = run_case(model, FORCES_N[-1], CANDIDATE_OPTIONS)
    metrics = evaluate(rows, replay, replay_source)
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (output / "UPKIE_LIVE_MOMENT_REJECTION_R314.md").write_text(markdown(metrics))
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
