#!/usr/bin/env python3
"""R193 A/B: model the typed withhold action as zero actuator effort."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_disturbance_envelope import semantic_trace_equal
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_hold_improvement_gate_ab import consequence
from upkie_inexact_terminal_chooser_ab import (
    ROOT_IMPACT_PLANE_M,
    first_terminal_prefix_equal,
    model_limits,
    physical_impact_audit,
    terminal_contract,
)


REVISION = "upkie-inexact-zero-effort-baseline-ab-r193"
DURATION_S = 6.0


def profiles() -> dict[str, dict[str, Any]]:
    terminal = {"inexact_hold_ticks": 1, "inexact_terminal_chooser": True}
    realized = {
        **terminal,
        "inexact_terminal_zero_effort_baseline": True,
    }
    return {
        "exact_control": {"profile": {}},
        "exact_zero_effort_config": {"profile": {}, **realized},
        "drop5_r191_zero_acceleration": {"profile": DROP5, **terminal},
        "drop10_r191_zero_acceleration": {"profile": DROP10, **terminal},
        "drop5_r193_zero_effort": {"profile": DROP5, **realized},
        "drop10_r193_zero_effort": {"profile": DROP10, **realized},
        "drop5_matched_r193_zero_effort": {
            "profile": DROP5_MATCHED,
            **realized,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_ZERO_EFFORT_BASELINE_AB_R193.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def zero_effort_contract(trace: dict[str, Any]) -> dict[str, Any]:
    unavailable = np.asarray(trace["contact_observation_available"]) == 0
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    ticks = np.flatnonzero(queried)
    available = np.asarray(
        trace["inexact_observation_terminal_zero_effort_available"]
    ) != 0
    realized = np.asarray(
        trace["inexact_observation_terminal_zero_effort_acceleration"]
    )
    candidate_root = np.asarray(
        trace["inexact_observation_terminal_selector_root_acceleration"]
    )
    candidate_joint = np.asarray(
        trace["inexact_observation_terminal_selector_joint_acceleration"]
    )
    effort = np.asarray(
        trace["inexact_observation_terminal_selector_effort_utilization"]
    )
    root_exact = bool(
        np.array_equal(candidate_root[ticks, 0], realized[ticks, :2])
    )
    joint_exact = bool(
        np.array_equal(candidate_joint[ticks, 0], realized[ticks, 6:])
    )
    nonzero = (
        float(np.max(np.abs(realized[ticks]))) if len(ticks) else 0.0
    )
    return {
        "unavailable_ticks": int(np.sum(unavailable)),
        "queried_ticks": int(len(ticks)),
        "available_every_query": bool(np.all(available[ticks])),
        "query_every_unavailable_tick": bool(np.array_equal(queried, unavailable)),
        "candidate_root_acceleration_exact": root_exact,
        "candidate_joint_acceleration_exact": joint_exact,
        "baseline_effort_utilization_zero": bool(np.all(effort[ticks, 0] == 0.0)),
        "maximum_absolute_zero_effort_acceleration_rad_or_m_s2": nonzero,
        "nontrivial_dynamics_witness": nonzero > 1.0,
        "zero_allocation": bool(
            np.all(
                np.asarray(
                    trace[
                        "inexact_observation_terminal_zero_effort_allocation_calls"
                    ]
                )[ticks]
                == 0
            )
            and np.all(
                np.asarray(
                    trace[
                        "inexact_observation_terminal_zero_effort_allocated_bytes"
                    ]
                )[ticks]
                == 0
            )
        ),
        "step_ns_maximum": int(
            np.max(
                np.asarray(
                    trace["inexact_observation_terminal_zero_effort_step_ns"]
                )[ticks]
            )
            if len(ticks)
            else 0
        ),
    }


def first_action_divergence(
    old: dict[str, Any], new: dict[str, Any], diagnostic_names: tuple[str, ...]
) -> dict[str, Any] | None:
    old_queried = np.asarray(
        old["inexact_observation_terminal_selector_queried"], np.uint8
    ) != 0
    new_queried = np.asarray(
        new["inexact_observation_terminal_selector_queried"], np.uint8
    ) != 0
    old_action = np.asarray(
        old["inexact_observation_terminal_selector_action"], np.uint8
    )
    new_action = np.asarray(
        new["inexact_observation_terminal_selector_action"], np.uint8
    )
    common = min(len(old_action), len(new_action))
    changed = np.flatnonzero(
        (old_queried[:common] != new_queried[:common])
        | (old_action[:common] != new_action[:common])
    )
    if not len(changed):
        return None
    tick = int(changed[0])
    name_index = {name: index for index, name in enumerate(diagnostic_names)}
    old_diagnostics = np.asarray(
        old["inexact_observation_terminal_selector_candidate_diagnostics"],
        np.float64,
    )[tick]
    new_diagnostics = np.asarray(
        new["inexact_observation_terminal_selector_candidate_diagnostics"],
        np.float64,
    )[tick]
    fields = (
        "tilt_pressure",
        "angular_rate_pressure",
        "joint_position_pressure",
        "joint_velocity_pressure",
        "maximum_terminal_harm_pressure",
        "aggregate_score",
    )
    return {
        "tick": tick,
        "time_s": float(np.asarray(new["time_s"])[tick]),
        "old_queried": bool(old_queried[tick]),
        "new_queried": bool(new_queried[tick]),
        "old_action": int(old_action[tick]),
        "new_action": int(new_action[tick]),
        "zero_effort_acceleration": np.asarray(
            new["inexact_observation_terminal_zero_effort_acceleration"]
        )[tick].tolist(),
        "zero_effort_acceleration_linf": float(
            np.max(
                np.abs(
                    np.asarray(
                        new["inexact_observation_terminal_zero_effort_acceleration"]
                    )[tick]
                )
            )
        ),
        "old_baseline": {
            field: float(old_diagnostics[0, name_index[field]]) for field in fields
        },
        "new_baseline": {
            field: float(new_diagnostics[0, name_index[field]]) for field in fields
        },
        "new_support_free": {
            field: float(new_diagnostics[2, name_index[field]]) for field in fields
        },
        "old_selection": np.asarray(
            old["inexact_observation_terminal_selector_selection_diagnostics"]
        )[tick].tolist(),
        "new_selection": np.asarray(
            new["inexact_observation_terminal_selector_selection_diagnostics"]
        )[tick].tolist(),
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(case for case in selected_cases if case.name in requested)
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    model = pathlib.Path(args.model).resolve()
    limits = model_limits(model)
    import bonesaw

    impact_balance = bonesaw.UpkieBalanceSession(str(model))
    diagnostic_names = tuple(impact_balance.terminal_impact_diagnostic_names)
    configured = profiles()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs: dict[str, dict[str, Any]] = {}
        replays: dict[str, dict[str, Any]] = {}
        for name, config in configured.items():
            kwargs = {key: value for key, value in config.items() if key != "profile"}
            runs[name] = execute(
                model, case, args.duration, config["profile"], **kwargs
            )
            replays[name] = execute(
                model, case, args.duration, config["profile"], **kwargs
            )
        rows[case.name] = {}
        for name, run in runs.items():
            row: dict[str, Any] = {
                "metrics": run["metrics"],
                "terminal_contract": terminal_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
            }
            if "zero_effort" in name and name != "exact_zero_effort_config":
                row["zero_effort_contract"] = zero_effort_contract(run["trace"])
                row["physical_impact_audit"] = physical_impact_audit(
                    run["trace"], impact_balance, limits, 0.01
                )
            rows[case.name][name] = row
        rows[case.name]["exact_zero_effort_config"]["dormant_exact"] = (
            semantic_trace_equal(
                runs["exact_control"]["trace"],
                runs["exact_zero_effort_config"]["trace"],
            )
        )
        rows[case.name]["drop5_matched_r193_zero_effort"][
            "first_query_prefix_equal"
        ] = first_terminal_prefix_equal(
            runs["drop5_matched_r193_zero_effort"]["trace"],
            runs["drop10_r193_zero_effort"]["trace"],
        )
        for duration in ("drop5", "drop10"):
            rows[case.name][f"{duration}_r193_zero_effort"][
                "first_action_divergence_from_r191"
            ] = first_action_divergence(
                runs[f"{duration}_r191_zero_acceleration"]["trace"],
                runs[f"{duration}_r193_zero_effort"]["trace"],
                diagnostic_names,
            )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
                if name.startswith("drop") and "matched" not in name
            ),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    realized_names = (
        "drop5_r193_zero_effort",
        "drop10_r193_zero_effort",
        "drop5_matched_r193_zero_effort",
    )
    realized_arms = [
        case_rows[name] for case_rows in rows.values() for name in realized_names
    ]
    gates = {
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_configuration_is_dormant": all(
            case_rows["exact_zero_effort_config"]["dormant_exact"]
            for case_rows in rows.values()
        ),
        "zero_effort_witness_is_available_exact_and_nontrivial": all(
            arm["zero_effort_contract"]["available_every_query"]
            and arm["zero_effort_contract"]["query_every_unavailable_tick"]
            and arm["zero_effort_contract"]["candidate_root_acceleration_exact"]
            and arm["zero_effort_contract"]["candidate_joint_acceleration_exact"]
            and arm["zero_effort_contract"]["baseline_effort_utilization_zero"]
            and arm["zero_effort_contract"]["nontrivial_dynamics_witness"]
            for arm in realized_arms
        ),
        "typed_choice_maps_to_authority": all(
            arm["terminal_contract"]["typed_action_maps_to_authority"]
            and arm["terminal_contract"]["selected_action_is_pareto_admissible"]
            for arm in realized_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_r193_zero_effort"][
                "first_query_prefix_equal"
            ]
            for case_rows in rows.values()
        ),
        "independent_impact_audit_is_exact": all(
            arm["physical_impact_audit"]["zero_allocation"]
            and arm["physical_impact_audit"]["online_reaudit_diagnostics_exact"]
            and arm["physical_impact_audit"]["online_reaudit_disagreement_ticks"]
            == 0
            for arm in realized_arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            and arm["terminal_contract"]["zero_allocation"]
            for arm in arms
        )
        and all(arm["zero_effort_contract"]["zero_allocation"] for arm in realized_arms),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in arms
        ),
    }
    consequence_names = (
        "drop5_r191_zero_acceleration",
        "drop10_r191_zero_acceleration",
        "drop5_r193_zero_effort",
        "drop10_r193_zero_effort",
    )
    consequences = {name: consequence(rows, name) for name in consequence_names}
    consequence_admitted = all(
        consequences[name]["admitted"]
        for name in ("drop5_r193_zero_effort", "drop10_r193_zero_effort")
    )
    mechanism_passed = all(gates.values())
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "zero_effort_query_step_ns_maximum": max(
            arm["zero_effort_contract"]["step_ns_maximum"]
            for arm in realized_arms
        ),
        "terminal_selector_step_ns_maximum": max(
            arm["terminal_contract"]["step_ns"]["maximum"]
            for arm in realized_arms
        ),
    }
    timing_passed = (
        timing["loop_overruns"] == 0
        and timing["controller_step_ns_maximum"] <= 5_000_000
    )
    synchronous_admitted = mechanism_passed and consequence_admitted and timing_passed
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)
    terminal_queries = sum(
        arm["physical_impact_audit"]["query_count"] for arm in realized_arms
    )

    detail: list[list[Any]] = []
    divergence_detail: list[list[Any]] = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        for duration in ("drop5", "drop10"):
            old = case_rows[f"{duration}_r191_zero_acceleration"]["metrics"]
            new = case_rows[f"{duration}_r193_zero_effort"]["metrics"]
            detail.append(
                [
                    case_name,
                    duration,
                    outcome(exact),
                    outcome(old),
                    outcome(new),
                    (
                        "—"
                        if not exact["fell"] or not new["fell"]
                        else f"{float(new['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
                    ),
                ]
            )
            divergence = case_rows[f"{duration}_r193_zero_effort"][
                "first_action_divergence_from_r191"
            ]
            if divergence is not None:
                divergence_detail.append(
                    [
                        case_name,
                        duration,
                        f"{divergence['time_s']:.3f}",
                        f"{divergence['old_action']}→{divergence['new_action']}",
                        f"{divergence['old_baseline']['tilt_pressure']:.3f}→{divergence['new_baseline']['tilt_pressure']:.3f}",
                        f"{divergence['new_support_free']['tilt_pressure']:.3f}",
                        f"{divergence['new_baseline']['joint_velocity_pressure']:.3f}",
                        f"{divergence['new_support_free']['joint_velocity_pressure']:.3f}",
                        f"{divergence['zero_effort_acceleration_linf']:.3f}",
                    ]
                )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "root_impact_plane_m": ROOT_IMPACT_PLANE_M,
        "mechanism_passed": mechanism_passed,
        "zero_effort_consequence_admitted": consequence_admitted,
        "timing_passed": timing_passed,
        "synchronous_profile_admitted": synchronous_admitted,
        "first_run_ticks": first_run_ticks,
        "terminal_queries": terminal_queries,
        "gates": gates,
        "consequences": consequences,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw zero-effort terminal baseline A/B · r193",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant consequence **{'ADMITTED' if consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_admitted else 'REJECTED'}**.",
            "",
            "## Semantic correction",
            "",
            "- Typed withhold executes zero actuator effort. R191 scored it as zero generalized acceleration; r193 instead runs a separate no-contact fixed-zero-effort dynamics query and supplies that admitted acceleration to the unchanged terminal chooser.",
            "- The query is state-local Rust model evaluation. It does not call MuJoCo, integrate a plant, infer future dropout duration, refresh Primary health, or make zero torque executable through a different authority type.",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks**. Independently re-audited **{terminal_queries}** r193 queries.",
            "",
            "## Plant consequence",
            "",
            *markdown_table(
                ["case", "dropout", "exact", "r191 zero-qdd", "r193 zero-effort", "r193 fall Δ s"],
                detail,
            ),
            "",
            "## First changed terminal decisions",
            "",
            *markdown_table(
                [
                    "case",
                    "dropout",
                    "time s",
                    "action old→new",
                    "baseline tilt old→new",
                    "support-free tilt",
                    "baseline joint-v",
                    "support-free joint-v",
                    "|zero-effort qdd|∞",
                ],
                divergence_detail,
            ),
            "",
            "Action indices are 0 withhold, 1 retained, and 2 support-free. The table reports the first causal choice that changes in each row; it does not average later trajectory divergence.",
            "",
            "## Runtime",
            "",
            f"- Zero-effort dynamics / terminal-selector maxima: **{timing['zero_effort_query_step_ns_maximum'] / 1e3:.3f} / {timing['terminal_selector_step_ns_maximum'] / 1e3:.3f} µs**.",
            f"- Full loop: **{timing['loop_overruns']}** 5 ms overruns; loop/controller maxima **{timing['loop_ns_maximum'] / 1e6:.3f} / {timing['controller_step_ns_maximum'] / 1e6:.3f} ms**.",
            "- Fixed-effort query, terminal chooser, and measured controller hot paths report zero Rust allocation; Python GC remains zero.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-zero-effort-baseline-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_ZERO_EFFORT_BASELINE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "zero_effort_consequence_admitted": consequence_admitted,
                "synchronous_profile_admitted": synchronous_admitted,
                "consequences": consequences,
                "timing": timing,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
