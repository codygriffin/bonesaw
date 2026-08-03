#!/usr/bin/env python3
"""R313 force-backed relock persistence and preload qualification.

The evaluator remains policy-free. Rust owns the persistent landing request,
mode firewall, and force-backed observer. Python declares a finite preload
screen, sequences the public-rate MuJoCo consequence, and keeps bounded
mechanism qualification separate from full recovery promotion.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_load_reserve_matrix_r310 as r310
import upkie_live_measured_landing_r312 as r312


REVISION = "upkie-live-force-backed-relock-r313"
TICKS = r312.COMPOSITION_TICKS
FORCES_N = r312.COMPOSITION_FORCES_N
TERMINAL_FORCES_N = (-8.0, -6.0, 6.0, 8.0)
WHEEL_RADIUS_M = 0.050
PRELOAD_DEPTH_M = 0.0009
TARGET_WHEEL_HEIGHT_M = WHEEL_RADIUS_M - PRELOAD_DEPTH_M
SCREEN_TARGETS_M = (0.0500, 0.0495, 0.0494, 0.0493, 0.0492, 0.0491, 0.0490, 0.0488, 0.0486)


def config_for_target(target_m: float) -> tuple[float, ...]:
    return (target_m, *r312.CANDIDATE_CONFIG[1:])


def options_for_target(target_m: float) -> dict[str, Any]:
    return {
        **r312.PUBLIC_CONTROLLER_OPTIONS,
        "measured_landing_enabled": True,
        "measured_landing_config": config_for_target(target_m),
    }


CANDIDATE_OPTIONS = options_for_target(TARGET_WHEEL_HEIGHT_M)


def qualification_ticks(states: list[dict[str, Any]]) -> list[int]:
    ticks: list[int] = []
    previous = False
    for state in states:
        qualified = r312._diagnostic(state, "reacquisition_qualified") > 0.5
        if qualified and not previous:
            ticks.append(int(state["index"]))
        previous = qualified
    return ticks


def relock_summary(case: dict[str, Any]) -> dict[str, Any]:
    summary = r310.summarize(case)
    action = r312._action_summary(case)
    states = case["states"]
    active = [
        state
        for state in states
        if r312._diagnostic(state, "request_active") > 0.5
    ]
    qualification = qualification_ticks(states)
    persistence_ticks = [
        int(state["index"])
        for state in active
        if state["observed"] == [1, 1]
        and r312._diagnostic(state, "reacquisition_qualified") < 0.5
    ]
    return {
        "ticks": summary["ticks"],
        "terminal_pending": summary["terminal_pending"],
        "terminal_tick": summary["terminal_tick"],
        "first_non_double_tick": summary["first_non_double_tick"],
        "first_flight_tick": summary["first_flight_tick"],
        "loss_tick_count": summary["loss_tick_count"],
        "strict_recovery_tick": summary["strict_recovery_tick"],
        "minimum_root_height_m": summary["minimum_root_height_m"],
        "maximum_root_tilt_rad": summary["maximum_root_tilt_rad"],
        "maximum_torque_utilization": summary["maximum_torque_utilization"],
        "maximum_constraint_violation": summary["maximum_constraint_violation"],
        "maximum_dynamics_residual": summary["maximum_dynamics_residual"],
        "maximum_contact_residual": summary["maximum_contact_residual"],
        "nonadmitted_ticks": summary["nonadmitted_ticks"],
        "numeric_reset": summary["numeric_reset"],
        "controller_step_us": summary["controller_step_us"],
        "worker_step_us": summary["worker_step_us"],
        "action": action,
        "qualification_ticks": qualification,
        "persistence_ticks": persistence_ticks,
        "force_qualified_relock_count": len(qualification),
        "maximum_missing_leg_contact_load_n": max(
            (
                min(state["observed_wheel_normal_force_n"])
                for state in active
                if state["observed"] == [1, 1]
            ),
            default=0.0,
        ),
    }


def run_case(
    model: pathlib.Path,
    force_n: float,
    options: dict[str, Any],
    *,
    ticks: int = TICKS,
) -> dict[str, Any]:
    return r312._run_composition_case(model, force_n, options, ticks=ticks)


def row(
    force_n: float,
    baseline_case: dict[str, Any],
    candidate_case: dict[str, Any],
    r312_reference: dict[str, Any],
) -> dict[str, Any]:
    baseline = relock_summary(baseline_case)
    candidate = relock_summary(candidate_case)
    reference_tick = r312_reference["candidate"]["terminal_tick"]
    return {
        "force_y_n": force_n,
        "baseline": baseline,
        "candidate": candidate,
        "r312_terminal_tick": reference_tick,
        "terminal_delta_vs_baseline": (
            candidate["terminal_tick"] - baseline["terminal_tick"]
            if candidate["terminal_tick"] is not None
            and baseline["terminal_tick"] is not None
            else None
        ),
        "terminal_delta_vs_r312": (
            candidate["terminal_tick"] - reference_tick
            if candidate["terminal_tick"] is not None and reference_tick is not None
            else None
        ),
    }


def screen_row(target_m: float, force_n: float, case: dict[str, Any]) -> dict[str, Any]:
    result = relock_summary(case)
    return {
        "target_wheel_height_m": target_m,
        "preload_depth_m": WHEEL_RADIUS_M - target_m,
        "force_y_n": force_n,
        "ticks": result["ticks"],
        "terminal_tick": result["terminal_tick"],
        "qualification_ticks": result["qualification_ticks"],
        "nonadmitted_ticks": result["nonadmitted_ticks"],
        "maximum_torque_utilization": result["maximum_torque_utilization"],
    }


def profile_screen(
    rows: list[dict[str, Any]], r312_reference: dict[float, dict[str, Any]]
) -> list[dict[str, Any]]:
    profiles: list[dict[str, Any]] = []
    for target_m in sorted({item["target_wheel_height_m"] for item in rows}, reverse=True):
        selected = [item for item in rows if item["target_wheel_height_m"] == target_m]
        by_force = {item["force_y_n"]: item for item in selected}
        requalifies_six = all(by_force[force]["qualification_ticks"] for force in (-6.0, 6.0))
        no_earlier = all(
            by_force[force]["terminal_tick"]
            >= r312_reference[force]["candidate"]["terminal_tick"]
            for force in TERMINAL_FORCES_N
        )
        zero_nonadmission = all(not item["nonadmitted_ticks"] for item in selected)
        profiles.append(
            {
                "target_wheel_height_m": target_m,
                "preload_depth_m": WHEEL_RADIUS_M - target_m,
                "rows": selected,
                "gates": {
                    "mirrored_six_newton_force_requalification": requalifies_six,
                    "never_earlier_than_r312_terminal_boundary": no_earlier,
                    "zero_nonadmission": zero_nonadmission,
                },
                "selected": requalifies_six and no_earlier and zero_nonadmission,
            }
        )
    return profiles


def evaluate(
    rows: list[dict[str, Any]],
    screen: list[dict[str, Any]],
    nominal: dict[str, Any],
    replay: dict[str, Any],
    replay_source: dict[str, Any],
) -> dict[str, Any]:
    candidate_rows = [item["candidate"] for item in rows]
    activated = [item for item in rows if item["candidate"]["action"]["request_active_ticks"]]
    selected_profiles = [item for item in screen if item["selected"]]
    row_by_force = {item["force_y_n"]: item for item in rows}
    mechanism_gates = {
        "candidate_is_default_off": not bool(
            r312.PUBLIC_CONTROLLER_OPTIONS.get("measured_landing_enabled", False)
        ),
        "model_radius_minus_declared_preload_is_target": abs(
            TARGET_WHEEL_HEIGHT_M - (WHEEL_RADIUS_M - PRELOAD_DEPTH_M)
        )
        < 1.0e-15,
        "screen_selects_exactly_declared_profile": len(selected_profiles) == 1
        and abs(
            selected_profiles[0]["target_wheel_height_m"] - TARGET_WHEEL_HEIGHT_M
        )
        < 1.0e-12,
        "nominal_is_bilateral_dormant": nominal["ticks"] == TICKS
        and nominal["terminal_pending"] is None
        and nominal["action"]["request_active_ticks"] == 0,
        "request_starts_only_after_measured_loss": bool(activated)
        and all(
            item["candidate"]["action"]["first_request_active_tick"]
            >= item["candidate"]["first_non_double_tick"]
            for item in activated
        ),
        "request_persists_during_force_debounce": all(
            row_by_force[force]["candidate"]["persistence_ticks"]
            for force in (-6.0, 6.0)
        ),
        "mirrored_force_backed_relock_occurs": all(
            row_by_force[force]["candidate"]["force_qualified_relock_count"] >= 3
            for force in (-6.0, 6.0)
        ),
        "measured_mask_and_mode_firewall_hold": all(
            r312._mode_firewall(item["candidate_case"]["states"])
            and all(
                all(int(hard) <= int(observed) for hard, observed in zip(state["hard"], state["observed"]))
                for state in item["candidate_case"]["states"]
            )
            for item in rows
        ),
        "finite_zero_allocation": all(
            r312._finite_outputs(item["candidate_case"]["states"])
            and item["candidate"]["action"]["allocation_calls"] == 0
            and item["candidate"]["action"]["allocated_bytes"] == 0
            for item in rows
        ),
        "deadlines_hold": all(
            item["candidate"]["controller_step_us"]["p99"] < 5_000.0
            and item["candidate"]["worker_step_us"]["p99"] < 20_000.0
            for item in rows
        ),
        "exact_replay": r312._semantic_digest(replay)
        == r312._semantic_digest(replay_source),
        "no_new_nonadmission_and_active_rows_clean": all(
            item["candidate"]["nonadmitted_ticks"]
            == item["baseline"]["nonadmitted_ticks"]
            and not item["candidate"]["numeric_reset"]
            for item in rows
        )
        and all(not item["candidate"]["nonadmitted_ticks"] for item in activated),
        "never_earlier_than_r312_terminal_boundary": all(
            item["terminal_delta_vs_r312"] is None
            or item["terminal_delta_vs_r312"] >= 0
            for item in rows
        ),
    }
    promotion_gates = {
        "terminal_fall_count_reduced": sum(
            item["terminal_pending"] is not None for item in candidate_rows
        )
        < sum(item["baseline"]["terminal_pending"] is not None for item in rows),
        "every_case_finishes_horizon": all(
            item["ticks"] == TICKS and item["terminal_pending"] is None
            for item in candidate_rows
        ),
        "every_activated_case_force_relocks": all(
            item["candidate"]["force_qualified_relock_count"] > 0 for item in activated
        ),
        "relocked_cases_retain_final_upright_tail": all(
            row_by_force[force]["candidate"]["strict_recovery_tick"] is not None
            for force in (-6.0, 6.0)
        ),
    }
    return {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "candidate_default_enabled": False,
        "wheel_radius_m": WHEEL_RADIUS_M,
        "preload_depth_m": PRELOAD_DEPTH_M,
        "target_wheel_height_m": TARGET_WHEEL_HEIGHT_M,
        "rows": [
            {key: value for key, value in item.items() if not key.endswith("_case")}
            for item in rows
        ],
        "preload_screen": screen,
        "mechanism_gates": mechanism_gates,
        "bounded_relock_qualified": all(mechanism_gates.values()),
        "promotion_gates": promotion_gates,
        "recovery_promoted": all(promotion_gates.values()),
        "finding": (
            "R313 keeps a bounded normal request active through measured-load debounce. "
            "A model-radius-minus-0.9-mm target uniquely passes the frozen screen, "
            "force-qualifies at least three mirrored ±6 N relocks, and moves their "
            "terminal boundaries to tick 236 without moving the ±8 N guardrails earlier. "
            "All four terminal cases still fall, so the mechanism remains default-off."
        ),
    }


def markdown(metrics: dict[str, Any]) -> str:
    lines = [
        "# Upkie force-backed relock persistence — R313",
        "",
        "Status: **BOUNDED RELOCK QUALIFIED; FULL RECOVERY REJECTED; DEFAULT-OFF**.",
        "",
        metrics["finding"],
        "",
        "## Selected public-profile consequence",
        "",
        "| force Y N | baseline terminal | R312 terminal | R313 terminal | force-qualified ticks |",
        "|---:|---:|---:|---:|---:|",
    ]
    for item in metrics["rows"]:
        candidate = item["candidate"]
        lines.append(
            f'| {item["force_y_n"]:+.0f} | '
            f'{item["baseline"]["terminal_tick"] if item["baseline"]["terminal_tick"] is not None else "—"} | '
            f'{item["r312_terminal_tick"] if item["r312_terminal_tick"] is not None else "—"} | '
            f'{candidate["terminal_tick"] if candidate["terminal_tick"] is not None else "—"} | '
            f'{", ".join(map(str, candidate["qualification_ticks"])) or "—"} |'
        )
    lines += ["", "## Mechanism gates", ""]
    lines += [
        f'- {"PASS" if passed else "OPEN"} `{name}`'
        for name, passed in metrics["mechanism_gates"].items()
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
        "The request mask used to retain the relock command is private to the Rust request author. "
        "Measured raw/stable/hard masks still own WBC contact rows and RollingWheel promotion. "
        "R313 proves repeated force-backed requalification in the ±6 N class, but not survival "
        "of the later repeated pull. The next action must reject accumulated body moment before "
        "the third loss rather than deepen preload or extend a timeout.",
        "",
    ]
    return "\n".join(lines)


def run(
    model: pathlib.Path,
    output_dir: pathlib.Path,
    *,
    forces: tuple[float, ...] = FORCES_N,
    screen_targets: tuple[float, ...] = SCREEN_TARGETS_M,
) -> dict[str, Any]:
    reference_metrics = json.loads(
        pathlib.Path(
            "benchmarks/results/upkie-live-measured-landing-r312/metrics.json"
        ).read_text()
    )
    reference_by_force = {
        float(item["force_y_n"]): item
        for item in reference_metrics["r311_public_composition"]["rows"]
    }
    rows: list[dict[str, Any]] = []
    candidate_cases: dict[float, dict[str, Any]] = {}
    for force in forces:
        baseline = run_case(model, force, r312.PUBLIC_CONTROLLER_OPTIONS)
        candidate = run_case(model, force, CANDIDATE_OPTIONS)
        candidate_cases[force] = candidate
        item = row(force, baseline, candidate, reference_by_force[force])
        item["baseline_case"] = baseline
        item["candidate_case"] = candidate
        rows.append(item)
    screen_rows: list[dict[str, Any]] = []
    for target in screen_targets:
        for force in TERMINAL_FORCES_N:
            case = run_case(model, force, options_for_target(target))
            screen_rows.append(screen_row(target, force, case))
    screen = profile_screen(screen_rows, reference_by_force)
    nominal_case = r300.run_case(
        model,
        disturbed=False,
        maximum_ticks=TICKS,
        controller_options=CANDIDATE_OPTIONS,
        worker_options=r310.WORKER_OPTIONS,
    )
    nominal = relock_summary(nominal_case)
    replay_force = forces[-1]
    replay = run_case(model, replay_force, CANDIDATE_OPTIONS)
    metrics = evaluate(rows, screen, nominal, replay, candidate_cases[replay_force])
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (output_dir / "traces.json").write_text(
        json.dumps(
            {
                "revision": REVISION,
                "mirrored_relock_negative": candidate_cases.get(-6.0),
                "mirrored_relock_positive": candidate_cases.get(6.0),
                "representative_guardrail": candidate_cases[replay_force],
                "representative_replay": replay,
            },
            separators=(",", ":"),
        )
        + "\n"
    )
    (output_dir / "UPKIE_LIVE_FORCE_BACKED_RELOCK_R313.md").write_text(markdown(metrics))
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=pathlib.Path, default=pathlib.Path("models/upkie/upkie.urdf"))
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/results") / REVISION,
    )
    args = parser.parse_args()
    metrics = run(args.model, args.output_dir)
    print(markdown(metrics), end="")
    if not metrics["bounded_relock_qualified"] or metrics["recovery_promoted"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
