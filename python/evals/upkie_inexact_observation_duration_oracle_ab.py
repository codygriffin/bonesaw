#!/usr/bin/env python3
"""Ablate a noncausal burst-duration oracle over fixed Q15 hold authority."""

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
from upkie_current_support_realization_plant_ab import exact_terminal_outcome
from upkie_disturbance_envelope import semantic_trace_equal
from upkie_inexact_observation_hold_ab import ESTABLISHED_5MS, ESTABLISHED_10MS


REVISION = "upkie-inexact-observation-duration-oracle-ab-r187"
DURATION_S = 6.0
ONE_Q15 = 32_768
GAINS = (0.0, 0.25, 0.45, 0.50, 0.75, 0.95, 1.0)


def gain_name(gain: float) -> str:
    return f"g{int(round(gain * 100)):03d}"


def profiles() -> dict[str, tuple[int, float, dict[str, int]]]:
    result: dict[str, tuple[int, float, dict[str, int]]] = {
        "exact_hold0": (0, 1.0, {}),
        "exact_g000": (1, 0.0, {}),
        "exact_g045": (1, 0.45, {}),
        "exact_g100": (1, 1.0, {}),
        "drop5_hold0": (0, 1.0, ESTABLISHED_5MS),
    }
    result.update(
        {
            f"drop5_{gain_name(gain)}": (1, gain, ESTABLISHED_5MS)
            for gain in GAINS
        }
    )
    result["drop10_hold0"] = (0, 1.0, ESTABLISHED_10MS)
    result.update(
        {
            f"drop10_{gain_name(gain)}": (1, gain, ESTABLISHED_10MS)
            for gain in GAINS
        }
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_OBSERVATION_DURATION_ORACLE_AB_R187.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def scaled_hold_contract(trace: dict[str, Any], gain: float) -> dict[str, Any]:
    selection = np.asarray(trace["contact_program_authority_selection"])
    available = np.asarray(trace["contact_observation_available"]) != 0
    torque = np.asarray(trace["torque"])
    held = np.flatnonzero(selection == 4)
    q15 = int(round(gain * ONE_Q15))
    exact = True
    for tick in held:
        if tick == 0:
            exact = False
            break
        expected = np.asarray(
            [
                float(value) * float(q15) / float(ONE_Q15)
                for value in torque[tick - 1]
            ],
            np.float64,
        )
        if not np.array_equal(torque[tick], expected):
            exact = False
            break
    return {
        "configured_gain": gain,
        "configured_q15": q15,
        "typed_hold_ticks": int(len(held)),
        "typed_hold_only_when_unavailable": bool(np.all(~available[held])),
        "typed_hold_matches_scaled_preceding_effort": exact,
    }


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be finite and positive")
    selected_cases = cases()
    if args.cases:
        requested = set(args.cases.split(","))
        selected_cases = tuple(
            case for case in selected_cases if case.name in requested
        )
        missing = requested - {case.name for case in selected_cases}
        if missing:
            raise SystemExit(f"unknown cases: {sorted(missing)}")

    matrix = profiles()
    model = pathlib.Path(args.model).resolve()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs = {
            name: execute(
                model,
                case,
                args.duration,
                perturbation,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_authority=gain,
            )
            for name, (hold_ticks, gain, perturbation) in matrix.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                perturbation,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_authority=gain,
            )
            for name, (hold_ticks, gain, perturbation) in matrix.items()
        }
        baseline = runs["exact_hold0"]
        rows[case.name] = {}
        for name, run in runs.items():
            gain = matrix[name][1]
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "hold_contract": scaled_hold_contract(run["trace"], gain),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
                "semantic_exact_to_baseline": semantic_trace_equal(
                    baseline["trace"], run["trace"]
                ),
                "terminal_exact_to_baseline": exact_terminal_outcome(
                    baseline["metrics"], run["metrics"]
                ),
            }
        for duration in ("drop5", "drop10"):
            rows[case.name][f"{duration}_g000"][
                "semantic_exact_to_withheld"
            ] = semantic_trace_equal(
                runs[f"{duration}_g000"]["trace"],
                runs[f"{duration}_hold0"]["trace"],
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

    all_arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    exact_config_arms = [
        case_rows[name]
        for case_rows in rows.values()
        for name in ("exact_g000", "exact_g045", "exact_g100")
    ]
    scaled_arms = [
        arm
        for case_rows in rows.values()
        for name, arm in case_rows.items()
        if name.startswith("drop") and "_g" in name
    ]
    gates = {
        "matrix_complete": len(rows) == len(cases()),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in all_arms),
        "exact_stream_is_unchanged_by_dormant_gain": all(
            arm["semantic_exact_to_baseline"] for arm in exact_config_arms
        ),
        "typed_hold_is_unavailable_only_and_q15_exact": all(
            arm["hold_contract"]["typed_hold_only_when_unavailable"]
            and arm["hold_contract"]["typed_hold_matches_scaled_preceding_effort"]
            for arm in scaled_arms
        ),
        "gain_zero_is_withheld_semantics": all(
            arm["semantic_exact_to_withheld"]
            for case_rows in rows.values()
            for name, arm in case_rows.items()
            if name in ("drop5_g000", "drop10_g000")
        ),
        "finite_without_numeric_fault": all(
            arm["metrics"]["finite"] and not arm["metrics"]["numeric_fault"]
            for arm in all_arms
        ),
        "zero_rust_allocation_and_python_gc": all(
            arm["metrics"]["allocation_free"]
            and arm["metrics"]["python_gc_collections"] == 0
            for arm in all_arms
        ),
    }

    decisions: dict[str, dict[str, Any]] = {}
    baseline_name = "exact_hold0"
    for duration in ("drop5", "drop10"):
        for gain in GAINS:
            name = f"{duration}_{gain_name(gain)}"
            earlier: dict[str, float] = {}
            new_green_falls: list[str] = []
            for case_name, case_rows in rows.items():
                exact = case_rows[baseline_name]["metrics"]
                candidate = case_rows[name]["metrics"]
                if exact["qualified"] and candidate["fell"]:
                    new_green_falls.append(case_name)
                if exact["fell"] and candidate["fell"]:
                    delta = float(candidate["terminal_time_s"]) - float(
                        exact["terminal_time_s"]
                    )
                    if delta < -1.0e-12:
                        earlier[case_name] = delta
            decisions[name] = {
                "new_green_falls": new_green_falls,
                "earlier_boundaries_s": earlier,
                "consequence_admitted": not new_green_falls and not earlier,
            }

    duration_admitted = {
        duration: [
            gain
            for gain in GAINS
            if decisions[f"{duration}_{gain_name(gain)}"][
                "consequence_admitted"
            ]
        ]
        for duration in ("drop5", "drop10")
    }
    global_admitted = [
        gain
        for gain in GAINS
        if all(
            decisions[f"{duration}_{gain_name(gain)}"]["consequence_admitted"]
            for duration in ("drop5", "drop10")
        )
    ]
    timing = {
        "loop_overruns": sum(
            arm["metrics"]["loop_overruns"] for arm in all_arms
        ),
        "loop_ns_maximum": max(
            arm["metrics"]["loop_ns"]["maximum"] for arm in all_arms
        ),
        "controller_step_ns_maximum": max(
            arm["metrics"]["controller_step_ns"]["maximum"] for arm in all_arms
        ),
    }
    mechanism_passed = all(gates.values())
    duration_specific_consequence_exists = any(duration_admitted.values())
    global_policy_admitted = bool(mechanism_passed and global_admitted)
    synchronous_profile_admitted = bool(
        global_policy_admitted
        and timing["loop_overruns"] == 0
        and timing["controller_step_ns_maximum"] <= 5_000_000
    )

    decision_table = []
    for name, decision in decisions.items():
        earlier = decision["earlier_boundaries_s"]
        decision_table.append(
            [
                name,
                "ADMITTED" if decision["consequence_admitted"] else "REJECTED",
                len(decision["new_green_falls"]),
                len(earlier),
                f"{min(earlier.values()):+.3f}" if earlier else "—",
            ]
        )
    row_table = []
    for case_name, case_rows in rows.items():
        exact = case_rows[baseline_name]["metrics"]
        for name, arm in case_rows.items():
            if not name.startswith("drop") or "_g" not in name:
                continue
            candidate = arm["metrics"]
            delta = (
                "—"
                if not exact["fell"] or not candidate["fell"]
                else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
            )
            row_table.append(
                [
                    case_name,
                    name,
                    outcome(candidate),
                    delta,
                    arm["hold_contract"]["configured_q15"],
                    arm["hold_contract"]["typed_hold_ticks"],
                ]
            )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "duration_specific_consequence_exists": duration_specific_consequence_exists,
        "duration_admitted_gains": duration_admitted,
        "global_admitted_gains": global_admitted,
        "global_policy_admitted": global_policy_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "gates": gates,
        "decisions": decisions,
        "timing": timing,
        "gains": GAINS,
        "profiles": {
            name: {
                "hold_ticks": hold_ticks,
                "gain": gain,
                **perturbation,
            }
            for name, (hold_ticks, gain, perturbation) in matrix.items()
        },
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw observation-loss duration-oracle ablation · r187",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · duration-specific consequence **{'EXISTS' if duration_specific_consequence_exists else 'ABSENT'}** · one gain across both burst classes **{'ADMITTED' if global_policy_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**.",
            "",
            "## Gain decisions",
            "",
            *markdown_table(
                ["profile", "decision", "new green falls", "earlier falls", "worst Δ s"],
                decision_table,
            ),
            "",
            "## Plant rows",
            "",
            *markdown_table(
                ["case", "profile", "outcome", "fall Δ s", "gain Q15", "typed holds"],
                row_table,
            ),
            "",
            "## Mechanism gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Interpretation",
            "",
            "- Gain is quantized once to Q15 by Rust and applied to the cached admitted effort without compounding. Exact-observation behavior is dormant and must remain bit-identical for zero, partial, and full configured gain.",
            "- Zero gain disables retained authority and is required to be semantically identical to the corresponding withheld arm; executable retained-zero provenance is forbidden.",
            f"- Duration-specific admitted gains: **{duration_admitted}**. Gains admitted across both loss durations: **{global_admitted or 'none'}**.",
            "- The burst duration is not known on the first unavailable tick. A gain that passes one declared fault class is not a causal global selector; the next candidate must use only current state/history or a separately authenticated transport guarantee.",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-observation-duration-oracle-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "UPKIE_INEXACT_OBSERVATION_DURATION_ORACLE_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "duration_admitted_gains": duration_admitted,
                "global_admitted_gains": global_admitted,
                "global_policy_admitted": global_policy_admitted,
                "synchronous_profile_admitted": synchronous_profile_admitted,
            },
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
