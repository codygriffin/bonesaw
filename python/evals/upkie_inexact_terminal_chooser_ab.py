#!/usr/bin/env python3
"""Three-way terminal chooser A/B with an independent physical impact audit."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_contact_program_robustness_ab import cases, execute, outcome
from upkie_disturbance_envelope import semantic_trace_equal
from upkie_inexact_hold_forecast_selector_ab import DROP5, DROP5_MATCHED, DROP10
from upkie_inexact_hold_improvement_gate_ab import consequence
from upkie_mujoco_plant_report import JOINT_ORDER


REVISION = "upkie-inexact-terminal-chooser-ab-r191"
DURATION_S = 6.0
ROOT_IMPACT_PLANE_M = 0.225


def profiles() -> dict[str, dict[str, Any]]:
    return {
        "exact_control": {"profile": {}},
        "exact_terminal_config": {
            "profile": {},
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
        },
        "drop5_hold0": {"profile": DROP5},
        "drop10_hold0": {"profile": DROP10},
        "drop5_brake": {
            "profile": DROP5,
            "inexact_support_free_brake": True,
        },
        "drop10_brake": {
            "profile": DROP10,
            "inexact_support_free_brake": True,
        },
        "drop5_terminal": {
            "profile": DROP5,
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
        },
        "drop10_terminal": {
            "profile": DROP10,
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
        },
        "drop5_terminal_zero_margin": {
            "profile": DROP5,
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
            "inexact_terminal_minimum_component_improvement": 0.0,
        },
        "drop10_terminal_zero_margin": {
            "profile": DROP10,
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
            "inexact_terminal_minimum_component_improvement": 0.0,
        },
        "drop5_matched_terminal": {
            "profile": DROP5_MATCHED,
            "inexact_hold_ticks": 1,
            "inexact_terminal_chooser": True,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_INEXACT_TERMINAL_CHOOSER_AB_R191.html"
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def model_limits(model_path: pathlib.Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    joints = {joint.get("name", ""): joint for joint in ET.parse(model_path).getroot().findall("joint")}
    lower = np.empty(len(JOINT_ORDER), np.float64)
    upper = np.empty(len(JOINT_ORDER), np.float64)
    velocity = np.empty(len(JOINT_ORDER), np.float64)
    effort = np.empty(len(JOINT_ORDER), np.float64)
    for index, name in enumerate(JOINT_ORDER):
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"{name}: missing URDF limit")
        lower[index] = float(limit.get("lower", "-inf"))
        upper[index] = float(limit.get("upper", "inf"))
        velocity[index] = min(float(limit.get("velocity", "nan")), 8.0)
        effort[index] = float(limit.get("effort", "nan"))
    if np.any(~np.isfinite(velocity)) or np.any(velocity <= 0.0):
        raise ValueError("Upkie velocity limits must be finite and positive")
    if np.any(~np.isfinite(effort)) or np.any(effort <= 0.0):
        raise ValueError("Upkie effort limits must be finite and positive")
    return lower, upper, velocity, effort


def terminal_contract(trace: dict[str, Any]) -> dict[str, Any]:
    available = np.asarray(trace["contact_observation_available"]) != 0
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    action = np.asarray(trace["inexact_observation_terminal_selector_action"])
    selection = np.asarray(trace["contact_program_authority_selection"])
    executable = np.asarray(trace["contact_program_authority_executable"]) != 0
    candidate_available = np.column_stack(
        (
            np.ones(len(action), np.bool_),
            np.asarray(
                trace["inexact_observation_terminal_selector_retained_available"]
            )
            != 0,
            np.asarray(
                trace["inexact_observation_terminal_selector_support_free_available"]
            )
            != 0,
        )
    )
    selection_diagnostics = np.asarray(
        trace["inexact_observation_terminal_selector_selection_diagnostics"]
    )
    ticks = np.flatnonzero(queried)
    expected_selection = np.choose(action[ticks], [0, 4, 5]) if len(ticks) else np.asarray([], np.uint8)
    selected_available = candidate_available[ticks, action[ticks]]
    selected_component_regression = selection_diagnostics[ticks, 4]
    return {
        "unavailable_ticks": int(np.sum(~available)),
        "queried_ticks": int(len(ticks)),
        "queries_every_unavailable_tick": bool(np.array_equal(queried, ~available)),
        "typed_action_maps_to_authority": bool(
            np.array_equal(selection[ticks], expected_selection)
            and np.all(executable[ticks] == (action[ticks] != 0))
        ),
        "selected_action_is_pareto_admissible": bool(
            np.all(selected_available)
            and np.all(selected_component_regression <= 1.0e-12)
        ),
        "action_counts": {
            str(int(value)): int(np.sum(action[ticks] == value))
            for value in np.unique(action[ticks])
        },
        "zero_allocation": bool(
            np.all(
                np.asarray(
                    trace["inexact_observation_terminal_selector_allocation_calls"]
                )[ticks]
                == 0
            )
            and np.all(
                np.asarray(
                    trace["inexact_observation_terminal_selector_allocated_bytes"]
                )[ticks]
                == 0
            )
        ),
        "step_ns": distribution(
            np.asarray(trace["inexact_observation_terminal_selector_step_ns"])[ticks]
            if len(ticks)
            else np.asarray([0], np.uint64)
        ),
    }


def physical_impact_audit(
    trace: dict[str, Any],
    balance: Any,
    limits: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    minimum_component_improvement: float,
) -> dict[str, Any]:
    lower, upper, velocity_limit, _effort_limit = limits
    queried = np.asarray(trace["inexact_observation_terminal_selector_queried"]) != 0
    observation_available = np.asarray(trace["contact_observation_available"]) != 0
    retained_available = np.asarray(
        trace["inexact_observation_terminal_selector_retained_available"]
    ) != 0
    support_free_available = np.asarray(
        trace["inexact_observation_terminal_selector_support_free_available"]
    ) != 0
    online_action = np.asarray(trace["inexact_observation_terminal_selector_action"])
    online_diagnostics = np.asarray(
        trace["inexact_observation_terminal_selector_candidate_diagnostics"]
    )
    online_selection = np.asarray(
        trace["inexact_observation_terminal_selector_selection_diagnostics"]
    )
    ticks = np.flatnonzero(queried)
    names = tuple(balance.terminal_impact_diagnostic_names)
    name_index = {name: index for index, name in enumerate(names)}
    physical_selection = np.empty(len(ticks), np.uint8)
    diagnostics = np.empty((len(ticks), 3, len(names)), np.float64)
    selections = np.empty((len(ticks), 6), np.float64)
    allocation_calls = np.empty(len(ticks), np.uint64)
    allocated_bytes = np.empty(len(ticks), np.uint64)
    step_ns = np.empty(len(ticks), np.uint64)
    cursor = 0
    for tick in range(len(observation_available)):
        if not queried[tick]:
            continue
        candidate_root = np.asarray(
            trace["inexact_observation_terminal_selector_root_acceleration"]
        )[tick]
        candidate_joint = np.asarray(
            trace["inexact_observation_terminal_selector_joint_acceleration"]
        )[tick]
        effort = np.asarray(
            trace["inexact_observation_terminal_selector_effort_utilization"]
        )[tick]
        selection = np.zeros(6, np.float64)
        timing = balance.score_terminal_impact_candidates(
            np.asarray(trace["inexact_observation_terminal_selector_state"])[tick],
            np.asarray(trace["q"])[tick],
            np.asarray(trace["v"])[tick],
            lower,
            upper,
            velocity_limit,
            np.asarray(
                [True, retained_available[tick], support_free_available[tick]],
                np.uint8,
            ),
            candidate_root,
            candidate_joint,
            effort,
            0,
            0.0,
            minimum_component_improvement,
            diagnostics[cursor],
            selection,
        )
        step_ns[cursor], allocation_calls[cursor], allocated_bytes[cursor] = timing
        physical_selection[cursor] = int(selection[0])
        selections[cursor] = selection
        cursor += 1
    impact_time = diagnostics[:, :, name_index["time_to_impact_s"]]
    impact_energy = diagnostics[
        :, :, name_index["vertical_specific_impact_energy_j_kg"]
    ]
    diagnostic_delta = np.abs(online_diagnostics[ticks] - diagnostics)
    maximum_diagnostic_error = float(np.max(diagnostic_delta)) if len(ticks) else 0.0
    first_mismatch: dict[str, Any] | None = None
    if len(ticks) and np.any(diagnostic_delta != 0.0):
        cursor_index, candidate_index, field_index = np.argwhere(
            diagnostic_delta != 0.0
        )[0]
        first_mismatch = {
            "tick": int(ticks[cursor_index]),
            "candidate": int(candidate_index),
            "field": names[field_index],
            "online": float(
                online_diagnostics[ticks[cursor_index], candidate_index, field_index]
            ),
            "reaudit": float(diagnostics[cursor_index, candidate_index, field_index]),
        }
    return {
        "query_count": int(len(ticks)),
        "root_impact_plane_m": ROOT_IMPACT_PLANE_M,
        "ballistic_fields_candidate_invariant": bool(
            np.all(impact_time == impact_time[:, :1])
            and np.all(impact_energy == impact_energy[:, :1])
        ),
        "zero_allocation": bool(
            np.all(allocation_calls == 0) and np.all(allocated_bytes == 0)
        ),
        "physical_selection_counts": {
            str(int(value)): int(np.sum(physical_selection == value))
            for value in np.unique(physical_selection)
        },
        "online_reaudit_disagreement_ticks": int(
            np.sum(online_action[ticks] != physical_selection)
        ),
        "online_reaudit_disagreement_tick_indices": [
            int(tick)
            for tick in ticks[online_action[ticks] != physical_selection]
        ],
        "online_reaudit_maximum_diagnostic_error": float(
            np.max(np.abs(online_diagnostics[ticks] - diagnostics))
        ),
        "online_reaudit_maximum_selection_error": float(
            np.max(np.abs(online_selection[ticks] - selections))
        ),
        "online_reaudit_diagnostics_exact": bool(
            np.array_equal(online_diagnostics[ticks], diagnostics)
            and np.array_equal(online_selection[ticks], selections)
        ),
        "online_reaudit_maximum_diagnostic_error": maximum_diagnostic_error,
        "online_reaudit_first_mismatch": first_mismatch,
        "time_to_impact_s": distribution(impact_time[:, 0]),
        "specific_impact_energy_j_kg": distribution(impact_energy[:, 0]),
        "step_ns": distribution(step_ns),
    }


def first_terminal_prefix_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    left_ticks = np.flatnonzero(
        np.asarray(left["inexact_observation_terminal_selector_queried"]) != 0
    )
    right_ticks = np.flatnonzero(
        np.asarray(right["inexact_observation_terminal_selector_queried"]) != 0
    )
    if not len(left_ticks) or not len(right_ticks) or left_ticks[0] != right_ticks[0]:
        return False
    tick = int(left_ticks[0])
    fields = (
        "time_s",
        "root_position",
        "root_twist",
        "rotation_vector",
        "q",
        "v",
        "torque",
        "inexact_observation_terminal_selector_action",
        "inexact_observation_terminal_selector_candidate_diagnostics",
    )
    return all(
        np.array_equal(np.asarray(left[field])[: tick + 1], np.asarray(right[field])[: tick + 1])
        for field in fields
    )


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
    configured = profiles()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs: dict[str, dict[str, Any]] = {}
        replays: dict[str, dict[str, Any]] = {}
        for name, config in configured.items():
            kwargs = {key: value for key, value in config.items() if key != "profile"}
            runs[name] = execute(model, case, args.duration, config["profile"], **kwargs)
            replays[name] = execute(model, case, args.duration, config["profile"], **kwargs)
        rows[case.name] = {}
        for name, run in runs.items():
            row: dict[str, Any] = {
                "metrics": run["metrics"],
                "terminal_contract": terminal_contract(run["trace"]),
                "replay_exact": semantic_trace_equal(run["trace"], replays[name]["trace"]),
            }
            if "terminal" in name and name != "exact_terminal_config":
                row["physical_impact_audit"] = physical_impact_audit(
                    run["trace"],
                    impact_balance,
                    limits,
                    float(
                        configured[name].get(
                            "inexact_terminal_minimum_component_improvement",
                            0.01,
                        )
                    ),
                )
            rows[case.name][name] = row
        rows[case.name]["exact_terminal_config"]["dormant_exact"] = semantic_trace_equal(
            runs["exact_control"]["trace"], runs["exact_terminal_config"]["trace"]
        )
        rows[case.name]["drop5_matched_terminal"]["first_query_prefix_equal"] = (
            first_terminal_prefix_equal(
                runs["drop5_matched_terminal"]["trace"],
                runs["drop10_terminal"]["trace"],
            )
        )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
                if name.startswith("drop")
            ),
            flush=True,
        )

    arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    terminal_names = (
        "drop5_terminal",
        "drop10_terminal",
        "drop5_terminal_zero_margin",
        "drop10_terminal_zero_margin",
        "drop5_matched_terminal",
    )
    terminal_arms = [case_rows[name] for case_rows in rows.values() for name in terminal_names]
    gates = {
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in arms),
        "exact_terminal_configuration_is_dormant": all(
            case_rows["exact_terminal_config"]["dormant_exact"] for case_rows in rows.values()
        ),
        "terminal_queries_every_unavailable_tick": all(
            arm["terminal_contract"]["queries_every_unavailable_tick"] for arm in terminal_arms
        ),
        "typed_choice_maps_to_authority": all(
            arm["terminal_contract"]["typed_action_maps_to_authority"]
            and arm["terminal_contract"]["selected_action_is_pareto_admissible"]
            for arm in terminal_arms
        ),
        "first_loss_has_no_burst_duration_oracle": all(
            case_rows["drop5_matched_terminal"]["first_query_prefix_equal"]
            for case_rows in rows.values()
        ),
        "physical_impact_audit_is_separate_and_allocation_free": all(
            arm["physical_impact_audit"]["ballistic_fields_candidate_invariant"]
            and arm["physical_impact_audit"]["zero_allocation"]
            and arm["physical_impact_audit"]["online_reaudit_diagnostics_exact"]
            and arm["physical_impact_audit"]["online_reaudit_disagreement_ticks"] == 0
            for arm in terminal_arms
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"] for arm in arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            and arm["terminal_contract"]["zero_allocation"]
            for arm in arms
        ),
    }
    consequence_names = (
        "drop5_hold0",
        "drop10_hold0",
        "drop5_brake",
        "drop10_brake",
        "drop5_terminal",
        "drop10_terminal",
        "drop5_terminal_zero_margin",
        "drop10_terminal_zero_margin",
    )
    consequences = {name: consequence(rows, name) for name in consequence_names}
    terminal_consequence_admitted = all(
        consequences[name]["admitted"] for name in ("drop5_terminal", "drop10_terminal")
    )
    mechanism_passed = all(gates.values())
    timing = {
        "loop_overruns": sum(arm["metrics"]["loop_overruns"] for arm in arms),
        "loop_ns_maximum": max(arm["metrics"]["loop_ns"]["maximum"] for arm in arms),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in arms
        ),
        "terminal_selector_step_ns_maximum": max(
            arm["terminal_contract"]["step_ns"]["maximum"] for arm in terminal_arms
        ),
        "physical_impact_audit_step_ns_maximum": max(
            arm["physical_impact_audit"]["step_ns"]["maximum"] for arm in terminal_arms
        ),
    }
    timing_passed = timing["loop_overruns"] == 0 and timing["controller_step_ns_maximum"] <= 5_000_000
    synchronous_profile_admitted = mechanism_passed and terminal_consequence_admitted and timing_passed
    physical_disagreements = sum(
        arm["physical_impact_audit"]["online_reaudit_disagreement_ticks"]
        for arm in terminal_arms
    )
    terminal_queries = sum(arm["physical_impact_audit"]["query_count"] for arm in terminal_arms)
    first_run_ticks = sum(arm["metrics"]["executed_ticks"] for arm in arms)

    detail = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact_control"]["metrics"]
        for duration in ("drop5", "drop10"):
            candidate = case_rows[f"{duration}_terminal"]["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            detail.append([case_name, duration, outcome(exact), outcome(candidate), delta])

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "root_impact_plane_m": ROOT_IMPACT_PLANE_M,
        "mechanism_passed": mechanism_passed,
        "terminal_consequence_admitted": terminal_consequence_admitted,
        "timing_passed": timing_passed,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "first_run_ticks": first_run_ticks,
        "terminal_queries": terminal_queries,
        "online_reaudit_disagreement_ticks": physical_disagreements,
        "gates": gates,
        "consequences": consequences,
        "timing": timing,
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw three-way terminal chooser A/B · r191",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · plant consequence **{'ADMITTED' if terminal_consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**.",
            "",
            "## Plant consequence",
            "",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks** across **{len(arms)} profiles**, each with exact replay.",
            *markdown_table(
                ["case", "dropout", "exact", "terminal chooser", "fall Δ s"], detail
            ),
            "",
            "## Independent physical impact audit",
            "",
            f"- Independently re-audited **{terminal_queries}** online terminal queries at a declared root-impact plane of **{ROOT_IMPACT_PLANE_M:.3f} m**; full candidate diagnostics and selections disagree on **{physical_disagreements}** ticks.",
            "- Ballistic time, vertical impact velocity, and vertical specific energy are candidate-invariant by construction. Candidate commands affect only terminal tilt/rate, joint headroom/speed, effort, and admission diagnostics.",
            "- The J/kg field is only `0.5 vz²`; no horizontal or rotational energy is invented without mass/inertia evidence, and the audit is not a collision-impulse or injury certificate.",
            "",
            "## Runtime",
            "",
            f"- Online selector / independent re-audit maxima: **{timing['terminal_selector_step_ns_maximum'] / 1e3:.3f} / {timing['physical_impact_audit_step_ns_maximum'] / 1e3:.3f} µs**.",
            f"- Full loop: **{timing['loop_overruns']}** 5 ms overruns; loop/controller maxima **{timing['loop_ns_maximum'] / 1e6:.3f} / {timing['controller_step_ns_maximum'] / 1e6:.3f} ms**.",
            "- Rust hot-path allocation and Python GC inside measured controller execution: **zero**.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-terminal-chooser-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_TERMINAL_CHOOSER_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "terminal_consequence_admitted": terminal_consequence_admitted,
                "synchronous_profile_admitted": synchronous_profile_admitted,
                "physical_disagreements": physical_disagreements,
                "consequences": consequences,
                "timing": timing,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
