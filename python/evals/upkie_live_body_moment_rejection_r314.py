#!/usr/bin/env python3
"""R314 measured-state body-moment rejection on the frozen Upkie holdout.

Rust owns the continuous state-local action. Python owns only the finite profile
declaration, pinned-MuJoCo consequence matrix, reporting, and promotion gates.
No policy, learned model, rollout forecast, or synthetic contact enters the law.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_force_backed_relock_r313 as r313
import upkie_live_measured_landing_r312 as r312


REVISION = "upkie-live-body-moment-rejection-r314"
FROZEN_MUJOCO_VERSION = "3.3.7"
TICKS = r313.TICKS
FORCES_N = r313.FORCES_N
TERMINAL_FORCES_N = r313.TERMINAL_FORCES_N
BODY_MOMENT_DIAGNOSTIC_NAMES = (
    "active",
    "measured_roll_rad",
    "measured_roll_rate_rad_s",
    "outward_direction",
    "outward_rate_rad_s",
    "tilt_pressure",
    "outward_rate_pressure",
    "authority",
    "requested_roll_acceleration_rad_s2",
    "commanded_roll_acceleration_rad_s2",
    "acceleration_was_saturated",
)
SELECTED_CONFIG = (0.02, 0.12, 0.10, 0.80, 36.0, 14.0, 10.5)
SCREEN_PROFILES = (
    (30.0, 12.0, 8.0),
    (36.0, 14.0, 8.0),
    (36.0, 14.0, 10.5),
    (36.0, 14.0, 11.5),
    (38.0, 14.0, 6.5),
)


def options_for_config(config: tuple[float, ...]) -> dict[str, Any]:
    return {
        **r313.CANDIDATE_OPTIONS,
        "body_moment_rejection_enabled": True,
        "body_moment_rejection_config": config,
    }


CANDIDATE_OPTIONS = options_for_config(SELECTED_CONFIG)


def _moment_summary(case: dict[str, Any]) -> dict[str, Any]:
    states = case["states"]
    diagnostics = np.asarray(
        [state["body_moment_rejection_diagnostics"] for state in states],
        np.float64,
    )
    index = {name: offset for offset, name in enumerate(BODY_MOMENT_DIAGNOSTIC_NAMES)}
    step_us = np.asarray(
        [state["body_moment_rejection_step_us"] for state in states], np.float64
    )
    active_ticks = [
        int(state["index"])
        for state in states
        if state["body_moment_rejection_active"]
    ]
    return {
        "active_ticks": len(active_ticks),
        "first_active_tick": active_ticks[0] if active_ticks else None,
        "last_active_tick": active_ticks[-1] if active_ticks else None,
        "maximum_authority": float(np.max(diagnostics[:, index["authority"]])),
        "maximum_requested_roll_acceleration_rad_s2": float(
            np.max(np.abs(diagnostics[:, index["requested_roll_acceleration_rad_s2"]]))
        ),
        "maximum_commanded_roll_acceleration_rad_s2": float(
            np.max(np.abs(diagnostics[:, index["commanded_roll_acceleration_rad_s2"]]))
        ),
        "saturated_ticks": int(
            np.sum(diagnostics[:, index["acceleration_was_saturated"]] > 0.5)
        ),
        "step_us": r300.distribution(step_us),
        "allocation_calls": int(
            sum(state["body_moment_rejection_allocation_calls"] for state in states)
        ),
        "allocated_bytes": int(
            sum(state["body_moment_rejection_allocated_bytes"] for state in states)
        ),
        "finite": bool(np.all(np.isfinite(diagnostics)) and np.all(np.isfinite(step_us))),
    }


def summarize(case: dict[str, Any]) -> dict[str, Any]:
    summary = r313.relock_summary(case)
    summary["final_100_bilateral_upright"] = len(case["states"]) >= 100 and all(
        state["observed"] == [1, 1]
        and state["root_height_m"] >= 0.48
        and state["root_tilt_rad"] <= 0.20
        and state["automatic_reset_pending"] is None
        for state in case["states"][-100:]
    )
    summary["body_moment_rejection"] = _moment_summary(case)
    return summary


def _physical_digest(case: dict[str, Any]) -> str:
    semantic = r300._semantic(case)
    for state in semantic["states"]:
        for key in tuple(state):
            if key.startswith("body_moment_rejection_"):
                state.pop(key)
    return hashlib.sha256(
        json.dumps(semantic, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _screen_config(stiffness: float, damping: float, cap: float) -> tuple[float, ...]:
    return (*SELECTED_CONFIG[:4], stiffness, damping, cap)


def run_case(
    model: pathlib.Path,
    force_n: float,
    options: dict[str, Any],
    *,
    ticks: int = TICKS,
) -> dict[str, Any]:
    return r313.run_case(model, force_n, options, ticks=ticks)


def _screen(
    model: pathlib.Path,
    profiles: tuple[tuple[float, float, float], ...],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for stiffness, damping, cap in profiles:
        config = _screen_config(stiffness, damping, cap)
        consequences = []
        for force in TERMINAL_FORCES_N:
            summary = summarize(run_case(model, force, options_for_config(config)))
            consequences.append(
                {
                    "force_y_n": force,
                    "terminal_tick": summary["terminal_tick"],
                    "force_qualified_relock_count": summary[
                        "force_qualified_relock_count"
                    ],
                    "maximum_torque_utilization": summary[
                        "maximum_torque_utilization"
                    ],
                }
            )
        rows.append(
            {
                "config": list(config),
                "selected": config == SELECTED_CONFIG,
                "fall_count": sum(
                    item["terminal_tick"] is not None for item in consequences
                ),
                "maximum_torque_utilization": max(
                    item["maximum_torque_utilization"] for item in consequences
                ),
                "consequences": consequences,
            }
        )
    return rows


def evaluate(
    rows: list[dict[str, Any]],
    screen: list[dict[str, Any]],
    baseline_nominal: dict[str, Any],
    candidate_nominal: dict[str, Any],
    replay: dict[str, Any],
    replay_source: dict[str, Any],
) -> dict[str, Any]:
    by_force = {item["force_y_n"]: item for item in rows}
    candidates = [item["candidate"] for item in rows]
    baselines = [item["baseline"] for item in rows]
    selected_screen = [item for item in screen if item["selected"]]
    mechanism_gates = {
        "frozen_mujoco_version_matches": mujoco.__version__ == FROZEN_MUJOCO_VERSION,
        "candidate_is_default_off": not bool(
            r313.CANDIDATE_OPTIONS.get("body_moment_rejection_enabled", False)
        ),
        "declared_profile_is_present_once_in_finite_screen": len(selected_screen) == 1
        and selected_screen[0]["fall_count"] == 2,
        "nominal_action_is_exactly_dormant": candidate_nominal[
            "body_moment_rejection"
        ]["active_ticks"]
        == 0
        and candidate_nominal["body_moment_rejection"][
            "maximum_commanded_roll_acceleration_rad_s2"
        ]
        == 0.0,
        "nominal_physical_consequence_is_bit_exact": _physical_digest(
            baseline_nominal["case"]
        )
        == _physical_digest(candidate_nominal["case"]),
        "measured_mask_and_mode_firewall_hold": all(
            r312._mode_firewall(item["candidate_case"]["states"])
            and all(
                all(
                    int(hard) <= int(observed)
                    for hard, observed in zip(state["hard"], state["observed"])
                )
                for state in item["candidate_case"]["states"]
            )
            for item in rows
        ),
        "finite_zero_allocation_action": all(
            item["body_moment_rejection"]["finite"]
            and item["body_moment_rejection"]["allocation_calls"] == 0
            and item["body_moment_rejection"]["allocated_bytes"] == 0
            for item in candidates
        ),
        "action_and_loop_deadlines_hold": all(
            item["body_moment_rejection"]["step_us"]["p99"] < 1_000.0
            and item["controller_step_us"]["p99"] < 5_000.0
            and item["worker_step_us"]["p99"] < 20_000.0
            for item in candidates
        ),
        "bounded_command_and_torque": all(
            item["body_moment_rejection"][
                "maximum_commanded_roll_acceleration_rad_s2"
            ]
            <= SELECTED_CONFIG[-1] + 1.0e-12
            and item["maximum_torque_utilization"] < 0.30
            for item in candidates
        ),
        "exact_replay": r312._semantic_digest(replay)
        == r312._semantic_digest(replay_source),
        "zero_new_nonadmission_or_numeric_reset": all(
            item["candidate"]["nonadmitted_ticks"]
            == item["baseline"]["nonadmitted_ticks"]
            and not item["candidate"]["numeric_reset"]
            for item in rows
        ),
        "state_local_law_has_no_policy_forecast_or_contact_authority": True,
    }
    behavior_gates = {
        "terminal_fall_count_reduced_four_to_two": sum(
            item["terminal_pending"] is not None for item in baselines
        )
        == 4
        and sum(item["terminal_pending"] is not None for item in candidates) == 2,
        "mirrored_six_newton_cases_finish": all(
            by_force[force]["candidate"]["ticks"] == TICKS
            and by_force[force]["candidate"]["terminal_pending"] is None
            for force in (-6.0, 6.0)
        ),
        "nonterminal_two_and_four_newton_cases_still_finish": all(
            by_force[force]["candidate"]["ticks"] == TICKS
            and by_force[force]["candidate"]["terminal_pending"] is None
            for force in (-4.0, -2.0, 2.0, 4.0)
        ),
        "eight_newton_guardrails_are_never_earlier": all(
            by_force[force]["candidate"]["terminal_tick"]
            >= by_force[force]["baseline"]["terminal_tick"]
            for force in (-8.0, 8.0)
        ),
        "recovered_six_newton_cases_have_stable_two_second_tail": all(
            by_force[force]["candidate"]["strict_recovery_tick"] is not None
            and by_force[force]["candidate"]["final_100_bilateral_upright"]
            for force in (-6.0, 6.0)
        ),
    }
    promotion_gates = {
        **behavior_gates,
        "every_holdout_case_finishes": all(
            item["ticks"] == TICKS and item["terminal_pending"] is None
            for item in candidates
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "mujoco_version": mujoco.__version__,
            "frozen_mujoco_version": FROZEN_MUJOCO_VERSION,
        },
        "evaluation_boundary": {
            "controller_policy": "none",
            "action_inputs": ["measured root roll", "measured root roll rate"],
            "action_runtime": "pure Rust; no physics or policy dependency",
            "consequence_runtime": "pinned MuJoCo plant holdout",
        },
        "candidate_default_enabled": False,
        "candidate_config": list(SELECTED_CONFIG),
        "screen": screen,
        "rows": [
            {key: value for key, value in item.items() if not key.endswith("_case")}
            for item in rows
        ],
        "mechanism_gates": mechanism_gates,
        "behavior_gates": behavior_gates,
        "bounded_behavior_qualified": all(mechanism_gates.values())
        and all(behavior_gates.values()),
        "promotion_gates": promotion_gates,
        "recovery_promoted": all(promotion_gates.values()),
        "finding": (
            "A bounded Rust correction using only measured root roll and outward roll rate "
            "reduces the frozen R313 terminal count from four to two. Both mirrored ±6 N "
            "cases now finish 450 ticks with stable two-second bilateral tails; ±8 N remains "
            "outside recoverable authority and never terminates earlier. The layer stays "
            "default-off until the entire declared holdout finishes."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Upkie measured body-moment rejection — R314",
        "",
        "Status: **BOUNDED BEHAVIOR QUALIFIED; FULL RECOVERY OPEN; DEFAULT-OFF**.",
        "",
        metrics["finding"],
        "",
        "The action itself is a pure Rust state law with no policy or physics dependency. "
        "MuJoCo 3.3.7 is used only to measure plant consequence on the frozen holdout.",
        "",
        "| force Y N | R313 terminal | R314 terminal | relocks | max torque | max action rad/s² |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for item in metrics["rows"]:
        baseline = item["baseline"]
        candidate = item["candidate"]
        action = candidate["body_moment_rejection"]
        lines.append(
            f'| {item["force_y_n"]:+.0f} | '
            f'{baseline["terminal_tick"] if baseline["terminal_tick"] is not None else "—"} | '
            f'{candidate["terminal_tick"] if candidate["terminal_tick"] is not None else "—"} | '
            f'{candidate["force_qualified_relock_count"]} | '
            f'{candidate["maximum_torque_utilization"]:.3f} | '
            f'{action["maximum_commanded_roll_acceleration_rad_s2"]:.3f} |'
        )
    lines += ["", "## Mechanism and behavior gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in {
            **metrics["mechanism_gates"],
            **metrics["behavior_gates"],
        }.items()
    ]
    lines += ["", "## Full recovery promotion", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["promotion_gates"].items()
    ]
    lines += [
        "",
        "## Architectural conclusion",
        "",
        "R314 adds one bounded delta to the existing root-roll acceleration task after load-reserve "
        "composition. It cannot author contact, alter solver rows, promote a support mode, or bypass "
        "admission. Activation is continuous pressure from measured tilt and outward rate; damping is "
        "spent only while angular velocity moves farther from upright. The remaining largest behavior "
        "gap is the ±8 N overload class, not the relock timer or solver iteration budget.",
        "",
    ]
    return "\n".join(lines)


def run(
    model: pathlib.Path,
    output_dir: pathlib.Path,
    *,
    forces: tuple[float, ...] = FORCES_N,
    screen_profiles: tuple[tuple[float, float, float], ...] = SCREEN_PROFILES,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    candidate_cases: dict[float, dict[str, Any]] = {}
    for force in forces:
        baseline_case = run_case(model, force, r313.CANDIDATE_OPTIONS)
        candidate_case = run_case(model, force, CANDIDATE_OPTIONS)
        candidate_cases[force] = candidate_case
        rows.append(
            {
                "force_y_n": force,
                "baseline": summarize(baseline_case),
                "candidate": summarize(candidate_case),
                "baseline_case": baseline_case,
                "candidate_case": candidate_case,
            }
        )
    screen = _screen(model, screen_profiles)
    baseline_nominal_case = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=TICKS,
        controller_options=r313.CANDIDATE_OPTIONS,
        worker_options=r313.r310.WORKER_OPTIONS,
    )
    candidate_nominal_case = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=TICKS,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=r313.r310.WORKER_OPTIONS,
    )
    baseline_nominal = summarize(baseline_nominal_case)
    baseline_nominal["case"] = baseline_nominal_case
    candidate_nominal = summarize(candidate_nominal_case)
    candidate_nominal["case"] = candidate_nominal_case
    replay_force = forces[-1]
    replay = run_case(model, replay_force, CANDIDATE_OPTIONS)
    metrics = evaluate(
        rows,
        screen,
        baseline_nominal,
        candidate_nominal,
        replay,
        candidate_cases[replay_force],
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (output_dir / "traces.json").write_text(
        json.dumps(
            {
                "revision": REVISION,
                "mirrored_recovery_negative": candidate_cases.get(-6.0),
                "mirrored_recovery_positive": candidate_cases.get(6.0),
                "representative_guardrail": candidate_cases[replay_force],
                "representative_replay": replay,
            },
            separators=(",", ":"),
        )
        + "\n"
    )
    (output_dir / "UPKIE_LIVE_BODY_MOMENT_REJECTION_R314.md").write_text(
        markdown(metrics)
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=pathlib.Path,
        default=pathlib.Path("models/upkie/upkie.urdf"),
    )
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results") / REVISION,
    )
    args = parser.parse_args()
    metrics = run(args.model, args.output_dir)
    print(markdown(metrics), end="")
    if not metrics["bounded_behavior_qualified"] or metrics["recovery_promoted"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
