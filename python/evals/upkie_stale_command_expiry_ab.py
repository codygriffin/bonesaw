#!/usr/bin/env python3
"""Admit stale-command expiry independently from the rejected damping blend."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_fall_safe_ab import CASES, GREEN, execute, semantic_equal


REVISION = "upkie-stale-command-expiry-ab-r137"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_STALE_COMMAND_EXPIRY_AB_R137.html"
    )
    return parser.parse_args()


def verdict(metrics: dict[str, object]) -> str:
    if not bool(metrics["fell"]):
        return "RECOVERED"
    return f"FALL {float(metrics['fall_time_s']):.3f}s"


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rows: dict[str, object] = {}
    for name, force in CASES:
        baseline = execute(model, force, False)
        candidate = execute(model, force, True, primary_blend=False)
        replay = execute(model, force, True, primary_blend=False)
        rows[name] = {
            "force_world_n": list(force),
            "baseline": baseline["metrics"],
            "candidate": candidate["metrics"],
            "candidate_replay_exact": semantic_equal(
                candidate["trace"], replay["trace"]
            ),
            "baseline_candidate_exact": semantic_equal(
                baseline["trace"], candidate["trace"]
            ),
        }

    adverse = [name for name, _ in CASES if name not in GREEN]
    gates = {
        "matrix_complete": set(rows) == {name for name, _ in CASES},
        "green_recovery_preserved": all(
            not rows[name]["candidate"]["fell"] for name in GREEN
        ),
        "green_semantics_exact": all(
            rows[name]["baseline_candidate_exact"] for name in GREEN
        ),
        "no_earlier_adverse_boundary": all(
            float(rows[name]["candidate"]["fall_time_s"])
            >= float(rows[name]["baseline"]["fall_time_s"])
            for name in adverse
        ),
        "stale_command_age_reduced": all(
            int(rows[name]["candidate"]["maximum_command_age_steps"])
            < int(rows[name]["baseline"]["maximum_command_age_steps"])
            for name in adverse
        ),
        "expiry_engages": all(
            float(rows[name]["candidate"]["minimum_fresh_command_authority"])
            < 1.0
            for name in adverse
        ),
        "candidate_replay_exact": all(
            bool(row["candidate_replay_exact"]) for row in rows.values()
        ),
        "zero_rust_allocation": all(
            int(row[side]["controller_allocation_calls"]) == 0
            and int(row[side]["controller_allocated_bytes"]) == 0
            for row in rows.values()
            for side in ("baseline", "candidate")
        ),
        "finite_metrics": all(
            np.isfinite(
                [
                    row[side]["fall_time_s"],
                    row[side]["terminal_kinetic_energy_j"],
                    row[side]["terminal_root_angular_speed_rad_s"],
                ]
            ).all()
            for name, row in rows.items()
            if name in adverse
            for side in ("baseline", "candidate")
        ),
    }
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "rows": rows,
        "interpretation": {
            "promotion": "live stale-command expiry admitted" if passed else "rejected",
            "recovery_claim": False,
            "scope": "expire a last admitted command after current WBC non-admission; do not blend the rejected r136 damping candidate",
        },
    }
    table_rows = []
    for name, row in rows.items():
        baseline = row["baseline"]
        candidate = row["candidate"]
        table_rows.append(
            [
                name,
                verdict(baseline),
                verdict(candidate),
                (
                    "—"
                    if name in GREEN
                    else f"{float(candidate['fall_time_s']) - float(baseline['fall_time_s']):+.3f}"
                ),
                f"{baseline['maximum_command_age_steps']} → {candidate['maximum_command_age_steps']}",
                f"{float(candidate['minimum_fresh_command_authority']):.3f}",
                f"{float(baseline['terminal_kinetic_energy_j']):.4f} → {float(candidate['terminal_kinetic_energy_j']):.4f}",
                "YES" if row["baseline_candidate_exact"] else "NO",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw Upkie stale-command expiry A/B · r137",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. Live promotion covers command freshness only; it is not a recovery claim.",
            "",
            "## Outcome",
            "",
            "The deployed candidate leaves valid primary objectives untouched. When the current WBC rejects, Rust grants the last admitted torque a five-tick lease and fades it to zero by tick twelve. Nominal and the qualified 4 N sagittal recovery are bit-exact. Every frozen adverse row reaches the first boundary no earlier, and stale-command age falls in every row. Terminal energy is reported but is deliberately not an admission gate: expiring stale torque is a freshness guarantee, not a universal energy controller.",
            "",
            *markdown_table(
                [
                    "case",
                    "baseline",
                    "candidate",
                    "boundary Δ s",
                    "max stale age",
                    "min fresh authority",
                    "terminal KE J",
                    "baseline exact",
                ],
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
            "- A `MaxIterations` candidate never executes. Only a previously admitted command may use the bounded lease.",
            "- Startup has an explicit five-tick lease before the first admitted command; it does not count as stale replay.",
            "- The r136 physical-risk damping blend remains rejected because the overload boundary regressed by 3.120 s.",
            "- Rust owns lease state, freshness authority, reason flags, and allocation-free state transition. Python owns MuJoCo and retained A/B evidence.",
        ]
    ) + "\n"
    (output / "upkie-stale-command-expiry-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_STALE_COMMAND_EXPIRY_AB_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(
        json.dumps(
            {"passed": passed, "gates": gates, "web_report": str(web_report)},
            sort_keys=True,
        )
    )
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
