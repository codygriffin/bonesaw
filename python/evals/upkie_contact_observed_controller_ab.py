#!/usr/bin/env python3
"""Gate causal measured-contact hard rows against the current live controller."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import (
    case_matrix,
    run_case,
    semantic_trace_equal,
    summarize,
)


REVISION = "upkie-contact-observed-controller-ab-r140"
WITHHELD_REVISION = "upkie-reduced-support-command-gate-ab-r141"
DURATION_S = 6.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_OBSERVED_CONTROLLER_AB_R140.html"
    )
    parser.add_argument(
        "--withhold-reduced-support",
        action="store_true",
        help="keep reduced-support WBC results diagnostic-only",
    )
    return parser.parse_args()


def execute(
    model: pathlib.Path,
    case: Any,
    measured: bool,
    execute_reduced_support: bool = True,
) -> dict[str, Any]:
    trace = run_case(
        model,
        case,
        DURATION_S,
        "capture",
        None,
        True,
        False,
        measured,
        execute_reduced_support,
    )
    metrics = summarize(case, trace, DURATION_S)
    metrics.update(
        {
            "maximum_command_age_steps": int(
                np.max(np.asarray(trace["command_age_steps"]))
            ),
            "nonadmitted_steps": int(
                np.sum(~np.isin(np.asarray(trace["status"]), [0, 1]))
            ),
            "support_transition_count": int(
                np.asarray(trace["support_transition_count"])[-1]
            ),
            "minimum_active_contact_count": int(
                np.min(np.sum(np.asarray(trace["admitted_contact_active"]), axis=1))
            ),
            "contact_mask_mismatch_steps": int(
                np.sum(
                    np.any(
                        np.asarray(trace["observed_contact_active"])
                        != np.asarray(trace["admitted_contact_active"]),
                        axis=1,
                    )
                )
            ),
        }
    )
    return {"trace": trace, "metrics": metrics}


def outcome(metrics: dict[str, Any]) -> str:
    if metrics["outcome"] != "FALL":
        return str(metrics["outcome"])
    return f"FALL {float(metrics['terminal_time_s']):.3f}s"


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    revision = WITHHELD_REVISION if args.withhold_reduced_support else REVISION
    rows: dict[str, Any] = {}
    for case in case_matrix():
        baseline = execute(model, case, False)
        candidate = execute(
            model, case, True, not args.withhold_reduced_support
        )
        replay = execute(model, case, True, not args.withhold_reduced_support)
        rows[case.name] = {
            "case": baseline["metrics"]["case"],
            "baseline": baseline["metrics"],
            "candidate": candidate["metrics"],
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }

    fall_rows = [name for name, row in rows.items() if row["baseline"]["fell"]]
    green_rows = [
        name
        for name, row in rows.items()
        if row["baseline"]["qualified"]
    ]
    gates = {
        "matrix_complete": len(rows) == 20,
        "no_new_numeric_fault": all(
            not row["candidate"]["numeric_fault"] for row in rows.values()
        ),
        "green_qualification_preserved": all(
            rows[name]["candidate"]["qualified"] for name in green_rows
        ),
        "no_new_fall": all(
            not row["candidate"]["fell"]
            for name, row in rows.items()
            if not row["baseline"]["fell"]
        ),
        "no_earlier_fall_boundary": all(
            float(rows[name]["candidate"]["terminal_time_s"])
            >= float(rows[name]["baseline"]["terminal_time_s"])
            for name in fall_rows
        ),
        "nonadmission_not_increased": sum(
            row["candidate"]["nonadmitted_steps"] for row in rows.values()
        )
        <= sum(row["baseline"]["nonadmitted_steps"] for row in rows.values()),
        "contact_transition_discriminates": any(
            row["candidate"]["support_transition_count"] > 0
            and row["candidate"]["minimum_active_contact_count"] < 2
            for row in rows.values()
        ),
        "candidate_replay_exact": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "zero_rust_allocation": all(
            row[side]["allocation_free"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
        "finite": all(
            row[side]["finite"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
    }
    passed = all(gates.values())
    table_rows = []
    for name, row in rows.items():
        baseline = row["baseline"]
        candidate = row["candidate"]
        boundary_delta = (
            "—"
            if not baseline["fell"]
            else f"{float(candidate['terminal_time_s']) - float(baseline['terminal_time_s']):+.3f}"
        )
        table_rows.append(
            [
                name,
                outcome(baseline),
                outcome(candidate),
                boundary_delta,
                f"{baseline['nonadmitted_steps']} → {candidate['nonadmitted_steps']}",
                candidate["minimum_active_contact_count"],
                candidate["support_transition_count"],
                candidate["contact_mask_mismatch_steps"],
                "YES" if row["candidate_replay_exact"] else "NO",
            ]
        )
    metrics = {
        "revision": revision,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "green_rows": green_rows,
        "baseline_fall_rows": fall_rows,
        "baseline_nonadmitted_steps": sum(
            row["baseline"]["nonadmitted_steps"] for row in rows.values()
        ),
        "candidate_nonadmitted_steps": sum(
            row["candidate"]["nonadmitted_steps"] for row in rows.values()
        ),
        "rows": rows,
        "interpretation": {
            "promotion": "measured-contact controller admitted" if passed else "rejected",
            "reduced_support_execution": not args.withhold_reduced_support,
            "contact_source": "exact MuJoCo wheel-tire/world observation",
            "hardware_estimator_claim": False,
            "recovery_claim": False,
        },
    }
    report = "\n".join(
        [
            (
                "# Bonesaw reduced-support command gate A/B · r141"
                if args.withhold_reduced_support
                else "# Bonesaw measured-contact controller A/B · r140"
            ),
            "",
            f"> Deployment **{'PASS' if passed else 'REJECTED'}**. The candidate consumes r139 exact contact evidence; simulator truth is not a hardware estimator.",
            "",
            "## Outcome",
            "",
            (
                "The frozen 20-case r133 consequence matrix is rerun as baseline/current-live freshness versus causal measured-contact hard masks whose reduced-support WBC result remains diagnostic-only. The prior admitted double-support command expires through r137 until exact double support is re-established. Promotion requires every baseline-qualified row, no new or earlier fall, no numeric fault, non-increased non-admission, exact replay, and zero Rust allocation."
                if args.withhold_reduced_support
                else "The frozen 20-case r133 consequence matrix is rerun as baseline/current-live freshness versus causal measured-contact hard masks, with an exact candidate replay. A new contact requires three exact samples; exact absence removes its hard row immediately; rejected WBC candidates remain non-executable and prior torque still expires through r137. Promotion requires every baseline-qualified row, no new or earlier fall, no numeric fault, non-increased non-admission, exact replay, and zero Rust allocation."
            ),
            "",
            *markdown_table(
                ["case", "baseline", "candidate", "boundary Δ s", "nonadmitted", "min contacts", "transitions", "mask mismatch ticks", "replay"],
                table_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Contract",
            "",
            "- Baseline and candidate share plant, capture policy, WBC, effort limits, fall-safe command lease, disturbances, and first-boundary scoring. Only contact observation/admission differs.",
            "- Exact absence cannot retain a hard rolling row. Debounced mode and hard eligibility remain separate.",
            "- A passing simulator A/B may promote the software boundary for the toy only; hardware still needs calibrated sensing, delay/noise/dropout, and source-authentication evidence.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_name = (
        "upkie-reduced-support-command-gate-ab-metrics.json"
        if args.withhold_reduced_support
        else "upkie-contact-observed-controller-ab-metrics.json"
    )
    audit_name = (
        "UPKIE_REDUCED_SUPPORT_COMMAND_GATE_AB_AUDIT.md"
        if args.withhold_reduced_support
        else "UPKIE_CONTACT_OBSERVED_CONTROLLER_AB_AUDIT.md"
    )
    (output / metrics_name).write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / audit_name).write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
