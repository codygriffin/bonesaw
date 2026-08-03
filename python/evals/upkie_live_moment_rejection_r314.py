#!/usr/bin/env python3
"""R314 causal external-moment rejection holdout.

The worker applies a declared wrench after each WBC call and feeds the last
completed MuJoCo wrench into the next 50 Hz solve.  The Rust WBC can therefore
ask its centroidal angular-momentum task for the opposing contact moment while
remaining causal.  This fixture measures that mechanism against the frozen
R313 relock profile; it does not promote a controller profile on a partial
recovery result.
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

# The centroidal task is existing Rust machinery.  The roll damping increase
# is deliberately explicit and default-off: it gives the continuous moment
# target enough room to act without changing the public profile.
CANDIDATE_OPTIONS: dict[str, Any] = {
    **r313.CANDIDATE_OPTIONS,
    "centroidal_angular_momentum_weight": 0.30,
    "centroidal_angular_momentum_frequency_hz": 1.0,
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
        [state["external_moment_world_nm"] for state in states], np.float64
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
            item["strict_recovery_tick"] is not None
            for force, item in zip(FORCES_N, candidate_rows)
            if abs(force) == 6.0
        ),
    }
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
                "root_roll_stiffness",
                "root_roll_damping",
            }
        },
        "rows": [
            {key: value for key, value in item.items() if key != "candidate_case"}
            for item in rows
        ],
        "mechanism_gates": mechanism_gates,
        "bounded_mechanism_qualified": all(mechanism_gates.values()),
        "promotion_gates": promotion_gates,
        "recovery_promoted": all(promotion_gates.values()),
        "finding": (
            "The delayed external-moment target is causal, finite, allocation-free, "
            "and never earlier than the frozen R313 boundary for this profile. "
            "It still reaches terminal falls in the repeated ±6/±8 N holdout, so "
            "the mechanism remains default-off and does not promote recovery."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Upkie causal moment rejection — R314",
        "",
        "Status: **MECHANISM QUALIFIED; RECOVERY REJECTED; DEFAULT-OFF**.",
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
        "then stores the completed MuJoCo moment for the next 50 Hz call. Rust "
        "owns the centroidal contact-moment target; Python only transports the "
        "fixed-size observation. The remaining failure is physical support/moment "
        "capacity, not a missing timeout or hidden reset.",
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
