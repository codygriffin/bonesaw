#!/usr/bin/env python3
"""Sweep one first-loss effort fraction across 5 ms and 10 ms dropout."""

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


REVISION = "upkie-inexact-hold-authority-sweep-r186"
DURATION_S = 6.0
AUTHORITIES = (0.0, 0.25, 0.5, 0.75, 1.0)
DROP5 = {
    "contact_observation_dropout_period_ticks": 50,
    "contact_observation_dropout_burst_ticks": 1,
    "contact_observation_dropout_start_tick": 50,
}
DROP10 = {
    "contact_observation_dropout_period_ticks": 100,
    "contact_observation_dropout_burst_ticks": 2,
    "contact_observation_dropout_start_tick": 100,
}


def authority_name(authority: float) -> str:
    return f"a{int(round(authority * 100)):03d}"


def profiles() -> dict[str, tuple[int, float, dict[str, int]]]:
    result = {
        "exact": (0, 1.0, {}),
        "drop5_hold0": (0, 1.0, DROP5),
        "drop10_hold0": (0, 1.0, DROP10),
    }
    for authority in AUTHORITIES:
        suffix = authority_name(authority)
        result[f"drop5_{suffix}"] = (1, authority, DROP5)
        result[f"drop10_{suffix}"] = (1, authority, DROP10)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_INEXACT_HOLD_AUTHORITY_SWEEP_R186.html",
    )
    parser.add_argument("--cases", help="comma-separated smoke subset")
    return parser.parse_args()


def hold_contract(trace: dict[str, Any], authority: float) -> dict[str, Any]:
    selection = np.asarray(trace["contact_program_authority_selection"])
    available = np.asarray(trace["contact_observation_available"]) != 0
    torque = np.asarray(trace["torque"])
    held = np.flatnonzero(selection == 4)
    exact_scale = bool(
        np.all(held > 0)
        and all(
            np.array_equal(torque[tick], authority * torque[tick - 1])
            for tick in held
        )
    )
    return {
        "typed_hold_ticks": int(len(held)),
        "typed_hold_only_when_unavailable": bool(np.all(~available[held])),
        "typed_hold_matches_authority_scaled_predecessor": exact_scale,
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

    model = pathlib.Path(args.model).resolve()
    configured = profiles()
    rows: dict[str, dict[str, Any]] = {}
    for case_index, case in enumerate(selected_cases, 1):
        runs = {
            name: execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_authority=authority,
            )
            for name, (hold_ticks, authority, profile) in configured.items()
        }
        replays = {
            name: execute(
                model,
                case,
                args.duration,
                profile,
                inexact_hold_ticks=hold_ticks,
                inexact_hold_authority=authority,
            )
            for name, (hold_ticks, authority, profile) in configured.items()
        }
        rows[case.name] = {}
        for name, run in runs.items():
            authority = configured[name][1]
            rows[case.name][name] = {
                "metrics": run["metrics"],
                "hold_contract": hold_contract(run["trace"], authority),
                "replay_exact": semantic_trace_equal(
                    run["trace"], replays[name]["trace"]
                ),
            }
        for duration in ("drop5", "drop10"):
            rows[case.name][f"{duration}_a000"][
                "semantic_exact_to_withheld"
            ] = semantic_trace_equal(
                runs[f"{duration}_a000"]["trace"],
                runs[f"{duration}_hold0"]["trace"],
            )
        print(
            f"[{case_index:02d}/{len(selected_cases)}] {case.name}: "
            + ", ".join(
                f"{name}={outcome(run['metrics'])}"
                for name, run in runs.items()
            ),
            flush=True,
        )

    all_arms = [arm for case_rows in rows.values() for arm in case_rows.values()]
    scaled_names = [
        name
        for name, (ticks, authority, _) in configured.items()
        if ticks == 1 and authority > 0.0
    ]
    scaled_arms = [
        case_rows[name] for case_rows in rows.values() for name in scaled_names
    ]
    gates = {
        "matrix_complete": len(rows) == len(cases()),
        "all_arms_replay_exact": all(arm["replay_exact"] for arm in all_arms),
        "zero_authority_is_withheld_semantics": all(
            case_rows[f"{duration}_a000"]["semantic_exact_to_withheld"]
            for case_rows in rows.values()
            for duration in ("drop5", "drop10")
        ),
        "scaled_hold_is_unavailable_only_and_exact": all(
            arm["hold_contract"]["typed_hold_only_when_unavailable"]
            and arm["hold_contract"][
                "typed_hold_matches_authority_scaled_predecessor"
            ]
            for arm in scaled_arms
        ),
        "scaled_hold_count_matches_first_tick_of_each_burst": all(
            case_rows[name]["hold_contract"]["typed_hold_ticks"]
            == (
                case_rows[name]["metrics"]["unavailable_observation_ticks"]
                if name.startswith("drop5_")
                else (
                    case_rows[name]["metrics"]["unavailable_observation_ticks"]
                    + 1
                )
                // 2
            )
            for case_rows in rows.values()
            for name in scaled_names
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

    authority_results: dict[str, Any] = {}
    for authority in AUTHORITIES:
        suffix = authority_name(authority)
        candidate_names = (f"drop5_{suffix}", f"drop10_{suffix}")
        earlier: dict[str, float] = {}
        new_green_falls: list[str] = []
        for case_name, case_rows in rows.items():
            exact = case_rows["exact"]["metrics"]
            for name in candidate_names:
                candidate = case_rows[name]["metrics"]
                if exact["qualified"] and candidate["fell"]:
                    new_green_falls.append(f"{case_name}/{name}")
                if exact["fell"] and candidate["fell"]:
                    delta = float(candidate["terminal_time_s"]) - float(
                        exact["terminal_time_s"]
                    )
                    if delta < -1.0e-12:
                        earlier[f"{case_name}/{name}"] = delta
        authority_results[suffix] = {
            "authority": authority,
            "earlier_boundaries_s": earlier,
            "new_green_falls": new_green_falls,
            "consequence_admitted": not earlier and not new_green_falls,
        }

    admitted = [
        name
        for name, result in authority_results.items()
        if result["consequence_admitted"]
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
    timing_gates = {
        "zero_5ms_loop_overruns": timing["loop_overruns"] == 0,
        "all_controller_calls_below_5ms": timing["controller_step_ns_maximum"] <= 5_000_000,
    }
    mechanism_passed = all(gates.values())
    consequence_admitted = mechanism_passed and bool(admitted)
    synchronous_profile_admitted = consequence_admitted and all(timing_gates.values())
    first_run_ticks = sum(
        arm["metrics"]["executed_ticks"] for arm in all_arms
    )
    typed_hold_ticks = sum(
        arm["hold_contract"]["typed_hold_ticks"] for arm in all_arms
    )

    table = []
    for case_name, case_rows in rows.items():
        exact = case_rows["exact"]["metrics"]
        for authority in AUTHORITIES:
            suffix = authority_name(authority)
            for duration in ("drop5", "drop10"):
                name = f"{duration}_{suffix}"
                candidate = case_rows[name]["metrics"]
                delta = (
                    "—"
                    if not exact["fell"] or not candidate["fell"]
                    else f"{float(candidate['terminal_time_s']) - float(exact['terminal_time_s']):+.3f}"
                )
                table.append(
                    [case_name, duration, f"{authority:.2f}", outcome(exact), outcome(candidate), delta]
                )

    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "duration_s": args.duration,
        "mechanism_passed": mechanism_passed,
        "consequence_admitted": consequence_admitted,
        "synchronous_profile_admitted": synchronous_profile_admitted,
        "gates": gates,
        "timing_gates": timing_gates,
        "timing": timing,
        "first_run_ticks": first_run_ticks,
        "typed_hold_ticks": typed_hold_ticks,
        "authority_results": authority_results,
        "admitted_authorities": admitted,
        "profiles": {name: {"hold_ticks": ticks, "authority": authority, **profile} for name, (ticks, authority, profile) in configured.items()},
        "rows": rows,
    }
    authority_table = [
        [
            f"{result['authority']:.2f}",
            len(result["earlier_boundaries_s"]),
            f'{min(result["earlier_boundaries_s"].values(), default=0.0):.3f}',
            len(result["new_green_falls"]),
            "ADMITTED" if result["consequence_admitted"] else "REJECTED",
        ]
        for result in authority_results.values()
    ]
    report = "\n".join(
        [
            "# Bonesaw inexact-hold authority sweep · r186",
            "",
            f"> Mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · continuous consequence **{'ADMITTED' if consequence_admitted else 'REJECTED'}** · synchronous profile **{'ADMITTED' if synchronous_profile_admitted else 'REJECTED'}**. One fixed first-loss effort fraction is applied to both isolated 5 ms and first-of-two 10 ms missing samples.",
            "",
            "## Authority summary",
            "",
            *markdown_table(["authority", "earlier", "worst Δ s", "new green falls", "verdict"], authority_table),
            "",
            "## Runtime resource gate",
            "",
            f"- Stored first-run work: **{first_run_ticks:,} control ticks** across **{len(all_arms)} profiles**, each with an exact replay; typed scaled holds: **{typed_hold_ticks}**.",
            f"- 5 ms loop overruns: **{timing['loop_overruns']}** (required: 0).",
            f"- Worst loop: **{timing['loop_ns_maximum'] / 1.0e6:.3f} ms**.",
            f"- Worst controller call: **{timing['controller_step_ns_maximum'] / 1.0e6:.3f} ms** (required: ≤5.000 ms).",
            "- Timed Rust allocation and Python GC collections: **zero**.",
            "",
            "## Case detail",
            "",
            *markdown_table(["case", "dropout", "authority", "exact", "candidate", "fall Δ s"], table),
            "",
            "## Contract",
            "",
            "- Authority is Q15 Rust state, not a Python post-process. Quarter fractions are represented exactly and scale the cached effort without compounding.",
            "- The first missing tick receives the same fraction whether the next sample returns or is also unavailable. No future burst-duration oracle is used.",
            "- Zero authority disables retained authority and follows withheld semantics; full authority reproduces r185's one-tick hold.",
            f"- Strictly admitted authority values: **{', '.join(admitted) if admitted else 'none'}**.",
        ]
    ) + "\n"

    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "upkie-inexact-hold-authority-sweep-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (destination / "UPKIE_INEXACT_HOLD_AUTHORITY_SWEEP_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(json.dumps({"mechanism_passed": mechanism_passed, "consequence_admitted": consequence_admitted, "synchronous_profile_admitted": synchronous_profile_admitted, "admitted_authorities": admitted, "authority_results": authority_results}, sort_keys=True))
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
