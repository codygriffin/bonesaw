#!/usr/bin/env python3
"""Sweep contact-command grace composed ahead of r137 freshness fade."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, run_case, semantic_trace_equal, summarize


REVISION = "upkie-contact-command-freshness-composition-r145"
DURATION_S = 6.0
DEFAULT_TICKS = (0, 2, 4, 8, 16)
LEASE_FIELDS = {
    "contact_command_lease_status",
    "contact_command_lease_executable",
    "contact_command_lease_age_ticks",
    "contact_command_lease_remaining_ticks",
    "contact_command_lease_flags",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", default=",".join(map(str, DEFAULT_TICKS)))
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONTACT_COMMAND_FRESHNESS_COMPOSITION_R145.html",
    )
    return parser.parse_args()


def execution_trace_equal(left: dict[str, Any], right: dict[str, Any]) -> bool:
    array_fields = [
        name
        for name, value in left.items()
        if isinstance(value, np.ndarray)
        and value.shape
        and name not in LEASE_FIELDS
        and name not in {"controller_step_ns", "loop_ns"}
    ]
    return bool(
        all(
            name in right
            and np.array_equal(np.asarray(left[name]), np.asarray(right[name]))
            for name in array_fields
        )
        and left["termination_reason"] == right["termination_reason"]
        and left["terminal_time_s"] == right["terminal_time_s"]
    )


def longest_true_run(mask: np.ndarray) -> int:
    longest = current = 0
    for value in mask:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def execute(model: pathlib.Path, case: Any, mode: str, ticks: int | None = None) -> dict[str, Any]:
    measured = mode != "live"
    trace = run_case(
        model,
        case,
        DURATION_S,
        "capture",
        None,
        True,
        False,
        measured,
        not measured,
        ticks if mode == "composed" else None,
        mode == "composed",
    )
    metrics = summarize(case, trace, DURATION_S)
    status = np.asarray(trace["contact_command_lease_status"])
    leased = status == 2
    fresh = status == 1
    support_count = np.sum(np.asarray(trace["admitted_contact_active"]), axis=1)
    metrics.update(
        {
            "physical_qualified": bool(
                not metrics["numeric_fault"]
                and not metrics["fell"]
                and (case.force_magnitude_n == 0.0 or metrics["recovery_time_s"] is not None)
                and metrics["allocation_free"]
                and metrics["loop_overruns"] == 0
            ),
            "wbc_nonadmitted_steps": int(
                np.sum(~np.isin(np.asarray(trace["status"]), [0, 1]))
            ),
            "leased_steps": int(np.sum(leased)),
            "maximum_leased_age_ticks": int(
                np.max(np.asarray(trace["contact_command_lease_age_ticks"])[leased])
            )
            if np.any(leased)
            else 0,
            "maximum_consecutive_leased_steps": longest_true_run(leased),
            "bounded": bool(
                ticks is None
                or (
                    (not np.any(leased) or np.all(
                        np.asarray(trace["contact_command_lease_age_ticks"])[leased] <= ticks
                    ))
                    and longest_true_run(leased) <= ticks
                )
            ),
            "fresh_only_full_support": bool(
                not np.any(fresh) or np.all(support_count[fresh] == 2)
            ),
            "leased_only_changed_support": bool(
                not np.any(leased) or np.all(support_count[leased] < 2)
            ),
        }
    )
    return {"trace": trace, "metrics": metrics}


def label(metrics: dict[str, Any]) -> str:
    return (
        f"FALL {float(metrics['terminal_time_s']):.3f}s"
        if metrics["fell"]
        else str(metrics["outcome"])
    )


def main() -> None:
    args = parse_args()
    ticks = tuple(int(value) for value in args.ticks.split(","))
    if not ticks or any(value < 0 for value in ticks) or len(set(ticks)) != len(ticks):
        raise SystemExit("--ticks must contain unique nonnegative integers")
    model = pathlib.Path(args.model).resolve()
    rows: dict[str, Any] = {}
    for index, case in enumerate(case_matrix(), start=1):
        live = execute(model, case, "live")
        withheld = execute(model, case, "withheld")
        candidates: dict[str, Any] = {}
        for hold in ticks:
            candidate = execute(model, case, "composed", hold)
            replay = execute(model, case, "composed", hold)
            candidates[str(hold)] = {
                "metrics": candidate["metrics"],
                "replay_exact": semantic_trace_equal(candidate["trace"], replay["trace"]),
                "zero_execution_equivalent": hold != 0
                or execution_trace_equal(withheld["trace"], candidate["trace"]),
            }
        rows[case.name] = {
            "case": live["metrics"]["case"],
            "live": live["metrics"],
            "withheld": withheld["metrics"],
            "candidates": candidates,
        }
        print(
            f"[{index:02d}/20] {case.name}: "
            + ", ".join(f"{hold}={label(candidates[str(hold)]['metrics'])}" for hold in ticks),
            flush=True,
        )

    green = [name for name, row in rows.items() if row["live"]["qualified"]]
    falls = [name for name, row in rows.items() if row["live"]["fell"]]
    aggregate: dict[str, Any] = {}
    for hold in ticks:
        key = str(hold)
        candidates = [row["candidates"][key] for row in rows.values()]
        earlier = [
            name
            for name in falls
            if rows[name]["candidates"][key]["metrics"]["terminal_time_s"]
            < rows[name]["live"]["terminal_time_s"] - 1.0e-12
        ]
        new_falls = [
            name
            for name, row in rows.items()
            if not row["live"]["fell"] and row["candidates"][key]["metrics"]["fell"]
        ]
        green_failures = [
            name
            for name in green
            if not rows[name]["candidates"][key]["metrics"]["physical_qualified"]
        ]
        deltas = [
            rows[name]["candidates"][key]["metrics"]["terminal_time_s"]
            - rows[name]["live"]["terminal_time_s"]
            for name in falls
        ]
        gates = {
            "green_physical_outcomes_preserved": not green_failures,
            "no_new_fall": not new_falls,
            "no_earlier_live_fall": not earlier,
            "exact_replay": all(item["replay_exact"] for item in candidates),
            "zero_tick_execution_equivalent": all(
                item["zero_execution_equivalent"] for item in candidates
            ),
            "finite": all(item["metrics"]["finite"] for item in candidates),
            "zero_rust_allocation": all(
                item["metrics"]["allocation_free"] for item in candidates
            ),
            "bounded": all(item["metrics"]["bounded"] for item in candidates),
            "fresh_only_full_support": all(
                item["metrics"]["fresh_only_full_support"] for item in candidates
            ),
            "leased_only_changed_support": all(
                item["metrics"]["leased_only_changed_support"] for item in candidates
            ),
        }
        aggregate[key] = {
            "ticks": hold,
            "milliseconds": hold * 5,
            "gates": gates,
            "passed": all(gates.values()),
            "green_failures": green_failures,
            "new_falls": new_falls,
            "earlier_falls": earlier,
            "minimum_fall_delta_s": min(deltas),
            "sum_fall_delta_s": sum(deltas),
            "leased_steps": sum(item["metrics"]["leased_steps"] for item in candidates),
            "maximum_leased_age_ticks": max(
                item["metrics"]["maximum_leased_age_ticks"] for item in candidates
            ),
        }

    nonzero = [hold for hold in ticks if hold > 0]
    selected = min(
        nonzero,
        key=lambda hold: (
            len(aggregate[str(hold)]["new_falls"]),
            len(aggregate[str(hold)]["green_failures"]),
            len(aggregate[str(hold)]["earlier_falls"]),
            -aggregate[str(hold)]["sum_fall_delta_s"],
            hold,
        ),
    )
    passed = aggregate[str(selected)]["passed"] and aggregate[str(selected)]["leased_steps"] > 0
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "ticks": ticks,
        "selected_ticks": selected,
        "passed": passed,
        "green_rows": green,
        "baseline_fall_rows": falls,
        "aggregate": aggregate,
        "rows": rows,
        "interpretation": {
            "promotion": "admitted" if passed else "rejected",
            "composition": "r143 contact grace then r137 continuous freshness fade",
            "reduced_support_wbc_execution": False,
            "hardware_estimator_claim": False,
        },
    }
    case_rows = [
        [
            name,
            label(row["live"]),
            label(row["withheld"]),
            *[label(row["candidates"][str(hold)]["metrics"]) for hold in ticks],
        ]
        for name, row in rows.items()
    ]
    aggregate_rows = [
        [
            f"{hold} ({hold * 5} ms)",
            len(aggregate[str(hold)]["green_failures"]),
            len(aggregate[str(hold)]["new_falls"]),
            len(aggregate[str(hold)]["earlier_falls"]),
            f"{aggregate[str(hold)]['minimum_fall_delta_s']:+.3f}",
            f"{aggregate[str(hold)]['sum_fall_delta_s']:+.3f}",
            aggregate[str(hold)]["leased_steps"],
            "PASS" if aggregate[str(hold)]["passed"] else "FAIL",
        ]
        for hold in ticks
    ]
    report = "\n".join(
        [
            "# Bonesaw contact-grace + freshness composition sweep · r145",
            "",
            f"> Deployment **{'PASS' if passed else 'REJECTED'}**. Best nonzero row: {selected} ticks ({selected * 5} ms).",
            "",
            "## Outcome",
            "",
            "This gate keeps r143's exact-evidence contact-command lease separate from r137's already admitted continuous solver-freshness fade. A retained command executes first; after its bounded contact grace expires, r137 begins its five-to-twelve-tick fade. Reduced-support WBC output remains diagnostic-only. The zero-tick row must be execution-bit-exact with r141 withholding, while every nonzero row must preserve all r137 green outcomes and never move an existing fall earlier.",
            "",
            *markdown_table(
                ["case", "r137 live", "r141 withheld", *[f"grace {hold}" for hold in ticks]],
                case_rows,
            ),
            "",
            "## Aggregate gates",
            "",
            *markdown_table(
                ["grace", "green failures", "new falls", "earlier falls", "min Δ s", "sum Δ s", "leased ticks", "gate"],
                aggregate_rows,
            ),
            "",
            "## Boundary",
            "",
            "- Exact absence never re-adds a hard row, and reduced-support diagnostics never execute.",
            "- Contact grace and solver freshness are independent typed authorities; this experiment composes them sequentially instead of treating either as recovery.",
            "- A negative result closes timeout tuning. The next candidate must change the solved physical action before sustained support loss.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-command-freshness-composition-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONTACT_COMMAND_FRESHNESS_COMPOSITION_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "selected_ticks": selected, "aggregate": aggregate}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
