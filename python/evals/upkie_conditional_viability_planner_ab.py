#!/usr/bin/env python3
"""Full plant A/B with reduced support gated by a supervised viability request."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from datetime import datetime, timezone

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, semantic_trace_equal
from upkie_viability_request_plant_ab import execute, outcome


REVISION = "upkie-conditional-viability-planner-ab-r150"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report",
        default="web/UPKIE_CONDITIONAL_VIABILITY_PLANNER_AB_R150.html",
    )
    return parser.parse_args()


def execute_arm(
    model: pathlib.Path,
    case: object,
    duration: float,
    *,
    candidate: bool,
) -> tuple[dict, dict]:
    result = execute(
        model,
        case,
        duration,
        measured_contact=candidate,
        planner=candidate,
        viability_support_requires_active_request=candidate,
    )
    metrics = result["metrics"]
    trace = result["trace"]
    if candidate:
        metrics["active_request_ticks"] = metrics["active_request_steps"]
        metrics["planner_queries"] = metrics["total_planner_wbc_queries"]
        metrics["support_program_ticks"] = int(
            sum(any(value != 1 for value in row) for row in trace["admitted_contact_active"])
        )
    return metrics, trace


def verdict(row: dict) -> str:
    return outcome(row)


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    retained = json.loads(
        pathlib.Path(
            "benchmarks/results/upkie-contact-command-freshness-composition-r145/"
            "upkie-contact-command-freshness-composition-metrics.json"
        ).read_text()
    )
    rows = {}
    for case in case_matrix():
        baseline, _ = execute_arm(
            model, case, args.duration, candidate=False
        )
        candidate, trace = execute_arm(
            model, case, args.duration, candidate=True
        )
        _, replay = execute_arm(model, case, args.duration, candidate=True)
        rows[case.name] = {
            "baseline": baseline,
            "candidate": candidate,
            "boundary_delta_s": candidate["terminal_time_s"]
            - baseline["terminal_time_s"],
            "candidate_replay_exact": semantic_trace_equal(trace, replay),
        }
        print(
            f'{case.name}: {baseline["outcome"]} -> {candidate["outcome"]} '
            f'@ {candidate["terminal_time_s"]:.3f}s',
            flush=True,
        )

    green = [name for name, row in retained["rows"].items() if row["live"]["qualified"]]
    adverse = [name for name in rows if name not in green]
    lost_green = [name for name in green if not rows[name]["candidate"]["qualified"]]
    earlier = [
        name
        for name in adverse
        if rows[name]["candidate"]["fell"]
        and rows[name]["candidate"]["terminal_time_s"]
        < rows[name]["baseline"]["terminal_time_s"] - 1.0e-12
    ]
    newly_recovered = [
        name
        for name in adverse
        if rows[name]["baseline"]["outcome"] != "RECOVERED"
        and rows[name]["candidate"]["outcome"] == "RECOVERED"
    ]
    gates = {
        "complete_frozen_matrix": len(rows) == 20,
        "current_live_reproduced": all(
            row["baseline"]["outcome"] == retained["rows"][name]["live"]["outcome"]
            and math.isclose(
                row["baseline"]["terminal_time_s"],
                retained["rows"][name]["live"]["terminal_time_s"],
                abs_tol=1.0e-12,
            )
            for name, row in rows.items()
        ),
        "every_green_qualification_preserved": not lost_green,
        "no_new_or_earlier_adverse_boundary": not earlier,
        "at_least_one_new_adverse_recovery": bool(newly_recovered),
        "candidate_exact_replay": all(
            row["candidate_replay_exact"] for row in rows.values()
        ),
        "zero_timed_rust_allocation": all(
            row[side]["allocation_free"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
        "finite_and_numeric_clean": all(
            row[side]["finite"] and not row[side]["numeric_fault"]
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
        "candidate_p99_inside_5ms": all(
            row["candidate"]["controller_step_ns"]["p99"] <= 5_000_000
            for row in rows.values()
        ),
    }
    evaluation_passed = all(
        gates[name]
        for name in (
            "complete_frozen_matrix",
            "current_live_reproduced",
            "candidate_exact_replay",
            "zero_timed_rust_allocation",
            "finite_and_numeric_clean",
        )
    )
    deployment_passed = all(gates.values())
    table = [
        [
            name,
            verdict(row["baseline"]),
            verdict(row["candidate"]),
            f'{row["boundary_delta_s"]:+.3f}',
            row["candidate"]["active_request_ticks"],
            row["candidate"]["support_program_ticks"],
            row["candidate"]["planner_queries"],
            f'{row["candidate"]["controller_step_ns"]["p99"] / 1e3:.1f}',
            f'{row["candidate"]["controller_step_ns"]["maximum"] / 1e3:.1f}',
            "YES" if row["candidate_replay_exact"] else "NO",
        ]
        for name, row in rows.items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "evaluation_passed": evaluation_passed,
        "deployment_passed": deployment_passed,
        "gates": gates,
        "contract": {
            "activation_pressure": 0.10,
            "release_pressure": 0.05,
            "planner_period_ticks": 4,
            "reduced_support_requires_executable_request": True,
            "policy_model": None,
            "controller_physics_model": None,
        },
        "classification": {
            "green": green,
            "lost_green": lost_green,
            "adverse": adverse,
            "earlier_adverse": earlier,
            "newly_recovered_adverse": newly_recovered,
        },
        "rows": rows,
    }
    report = "\n".join(
        [
            "# Bonesaw request-gated viability planner plant A/B · r150",
            "",
            f"> Evaluation **{'PASS' if evaluation_passed else 'FAIL'}**. Live deployment **{'PASS' if deployment_passed else 'REJECTED'}**. This composes the r147 selector and r148 supervisor without allowing an inactive planner to replace r137 double-support execution.",
            "",
            "## Outcome",
            "",
            "Measured reduced support is used only when the physical roll/lateral capture pressure has crossed activation and the Rust supervisor has a fresh executable request. Otherwise the admitted r137 double-support program remains unchanged. This isolates whether request-gated support switching repairs r149's green regressions while retaining any adverse recovery.",
            "",
            f"Green regressions: **{lost_green or 'none'}**. Earlier adverse boundaries: **{earlier or 'none'}**. Newly recovered adverse rows: **{newly_recovered or 'none'}**.",
            "",
            *markdown_table(
                ["case", "r137", "candidate", "Δ s", "active", "reduced", "queries", "p99 µs", "max µs", "replay"],
                table,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Boundary",
            "",
            "- Preserving green behavior is necessary but not sufficient: every existing adverse boundary must also be no earlier, and at least one previously falling row must recover.",
            "- The 250 ms local score remains a proposal heuristic. Rust owns request lifetime and final WBC admission; MuJoCo remains external consequence evidence.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-conditional-viability-planner-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONDITIONAL_VIABILITY_PLANNER_AB_AUDIT.md").write_text(report)
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "evaluation_passed": evaluation_passed,
                "deployment_passed": deployment_passed,
                "gates": gates,
                "classification": metrics["classification"],
            },
            sort_keys=True,
        )
    )
    if not evaluation_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
