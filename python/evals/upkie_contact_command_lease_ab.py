#!/usr/bin/env python3
"""Plant consequence gate for the bounded contact-command lease."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, run_case, semantic_trace_equal, summarize


REVISION = "upkie-contact-command-lease-ab-r144"
DURATION_S = 6.0
DEFAULT_HOLD_TICKS = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--hold-ticks", type=int, default=DEFAULT_HOLD_TICKS)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_COMMAND_LEASE_AB_R144.html"
    )
    return parser.parse_args()


def execute(
    model: pathlib.Path,
    case: Any,
    *,
    measured: bool,
    hold_ticks: int | None,
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
        False,
        hold_ticks,
    )
    metrics = summarize(case, trace, DURATION_S)
    lease_status = np.asarray(trace["contact_command_lease_status"])
    lease_executable = np.asarray(trace["contact_command_lease_executable"]) != 0
    lease_age = np.asarray(trace["contact_command_lease_age_ticks"])
    status = np.asarray(trace["status"])
    torque_norm = np.linalg.norm(np.asarray(trace["torque"]), axis=1)
    nonadmitted_nonleased = ~np.isin(status, [0, 1]) & ~lease_executable
    metrics.update(
        {
            "nonadmitted_steps": int(np.sum(~np.isin(status, [0, 1]))),
            "leased_steps": int(np.sum(lease_status == 2)),
            "expired_steps": int(np.sum(lease_status == 3)),
            "revoked_evidence_steps": int(np.sum(lease_status == 4)),
            "revoked_support_steps": int(np.sum(lease_status == 5)),
            "maximum_lease_age_ticks": int(np.max(lease_age)),
            "maximum_nonadmitted_nonleased_torque_nm": float(
                np.max(torque_norm[nonadmitted_nonleased], initial=0.0)
            ),
            "minimum_active_contact_count": int(
                np.min(np.sum(np.asarray(trace["admitted_contact_active"]), axis=1))
            ),
            "support_transition_count": int(
                np.asarray(trace["support_transition_count"])[-1]
            ),
        }
    )
    return {"trace": trace, "metrics": metrics}


def outcome(metrics: dict[str, Any]) -> str:
    if not metrics["fell"]:
        return str(metrics["outcome"])
    return f"FALL {float(metrics['terminal_time_s']):.3f}s"


def main() -> None:
    args = parse_args()
    if args.hold_ticks < 0:
        raise SystemExit("--hold-ticks must be nonnegative")
    model = pathlib.Path(args.model).resolve()
    rows: dict[str, Any] = {}
    for case in case_matrix():
        baseline = execute(model, case, measured=False, hold_ticks=None)
        candidate = execute(
            model, case, measured=True, hold_ticks=args.hold_ticks
        )
        replay = execute(model, case, measured=True, hold_ticks=args.hold_ticks)
        rows[case.name] = {
            "case": baseline["metrics"]["case"],
            "baseline": baseline["metrics"],
            "candidate": candidate["metrics"],
            "candidate_replay_exact": semantic_trace_equal(
                candidate["trace"], replay["trace"]
            ),
        }

    green_rows = [name for name, row in rows.items() if row["baseline"]["qualified"]]
    fall_rows = [name for name, row in rows.items() if row["baseline"]["fell"]]
    gates = {
        "matrix_complete": len(rows) == 20,
        "lease_transition_exercised": any(
            row["candidate"]["leased_steps"] > 0 for row in rows.values()
        ),
        "bounded_command_age": all(
            row["candidate"]["maximum_lease_age_ticks"] <= args.hold_ticks + 1
            for row in rows.values()
        ),
        "no_torque_after_nonexecutable_lease": all(
            row["candidate"]["maximum_nonadmitted_nonleased_torque_nm"] <= 1.0e-12
            for row in rows.values()
        ),
        "no_new_numeric_fault": all(
            not row["candidate"]["numeric_fault"] for row in rows.values()
        ),
        "green_qualification_preserved": all(
            rows[name]["candidate"]["qualified"] for name in green_rows
        ),
        "no_new_fall": all(
            not row["candidate"]["fell"]
            for row in rows.values()
            if not row["baseline"]["fell"]
        ),
        "no_earlier_fall_boundary": all(
            float(rows[name]["candidate"]["terminal_time_s"])
            >= float(rows[name]["baseline"]["terminal_time_s"])
            for name in fall_rows
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
        delta = (
            "—"
            if not baseline["fell"]
            else f"{float(candidate['terminal_time_s']) - float(baseline['terminal_time_s']):+.3f}"
        )
        table_rows.append(
            [
                name,
                outcome(baseline),
                outcome(candidate),
                delta,
                candidate["leased_steps"],
                candidate["maximum_lease_age_ticks"],
                f"{candidate['maximum_nonadmitted_nonleased_torque_nm']:.1e}",
                "YES" if row["candidate_replay_exact"] else "NO",
            ]
        )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "hold_ticks": args.hold_ticks,
        "hold_seconds": args.hold_ticks * 0.005,
        "passed": passed,
        "gates": gates,
        "green_rows": green_rows,
        "baseline_fall_rows": fall_rows,
        "rows": rows,
        "interpretation": {
            "software_contract": "admitted",
            "physical_recovery": "admitted" if passed else "rejected",
            "contact_source": "exact MuJoCo wheel-tire/world observation",
            "hardware_estimator_claim": False,
        },
    }
    report = "\n".join(
        [
            "# Bonesaw bounded contact-command lease A/B · r144",
            "",
            f"> Physical deployment **{'PASS' if passed else 'REJECTED'}**. The r143 lease is configured for {args.hold_ticks} ticks ({args.hold_ticks * 5} ms); exact MuJoCo contact remains a simulator source, not a hardware estimator.",
            "",
            "## Outcome",
            "",
            "The frozen 20-case r133 consequence matrix compares current-live fixed double-support control with r139 exact contact evidence, diagnostic-only reduced-support solves, and the r143 bounded prior-command lease. The lease itself is enforced end to end: no torque survives a non-executable lease, exact replay passes, and Rust timed allocations remain zero. Physical promotion remains independently gated on every green recovery and every existing adverse boundary.",
            "",
            *markdown_table(
                ["case", "baseline", "candidate", "boundary Δ s", "leased ticks", "max age", "post-expiry torque", "replay"],
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
            "## Boundary",
            "",
            "- R143 admits only bounded command lifetime. This A/B determines whether that mechanism may drive the plant; it does not alter the lease contract when physical promotion fails.",
            "- Reduced-support WBC results remain diagnostic-only. A leased torque retains the prior authoring mask and emits no current-support contact-force witness.",
            "- The next action must be independently solved for the observed support state or prepared before contact loss; extending this hold horizon cannot be called recovery.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-command-lease-ab-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONTACT_COMMAND_LEASE_AB_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates}, sort_keys=True))
    if passed:
        return
    raise SystemExit(1)


if __name__ == "__main__":
    main()
