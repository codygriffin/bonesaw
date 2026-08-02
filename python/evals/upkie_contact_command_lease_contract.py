#!/usr/bin/env python3
"""Admit the generic bounded contact-command lease without policy or physics."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


REVISION = "upkie-contact-command-lease-contract-r143"
STATUS = {
    0: "Unavailable",
    1: "Fresh",
    2: "Leased",
    3: "Expired",
    4: "RevokedEvidence",
    5: "RevokedSupport",
    6: "RejectedSequence",
    7: "RejectedCommand",
    8: "InvalidConfig",
}
PROVENANCE = {0: "Unavailable", 1: "Fresh", 2: "Leased"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_COMMAND_LEASE_CONTRACT_R143.html"
    )
    return parser.parse_args()


def run_rows(session: Any, rows: list[dict[str, Any]], hold_ticks: int) -> list[dict[str, Any]]:
    session.reset()
    session.configure_contact_command_lease(hold_ticks)
    names = list(session.contact_command_lease_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    stable = np.empty(2, np.uint8)
    hard = np.empty(2, np.uint8)
    candidate = np.empty(6, np.float64)
    command = np.empty(6, np.float64)
    diagnostics = np.empty(len(names), np.int64)
    outputs: list[dict[str, Any]] = []
    for row in rows:
        stable[:] = row["stable"]
        hard[:] = row["hard"]
        candidate[:] = row.get("command", (1.0, -2.0, 3.0, -4.0, 5.0, -6.0))
        session.step_contact_command_lease_from_masks(
            row["tick"],
            row.get("exact", True),
            stable,
            hard,
            row.get("fresh", False),
            candidate,
            command,
            diagnostics,
        )
        outputs.append(
            {
                "status": STATUS[int(diagnostics[index["status"]])],
                "provenance": PROVENANCE[int(diagnostics[index["provenance"]])],
                "executable": bool(diagnostics[index["executable"]]),
                "age_ticks": int(diagnostics[index["command_age_ticks"]]),
                "remaining_ticks": int(diagnostics[index["remaining_hold_ticks"]]),
                "authoring_mask": int(diagnostics[index["authoring_mask"]]),
                "stable_mask": int(diagnostics[index["stable_mask"]]),
                "hard_mask": int(diagnostics[index["hard_mask"]]),
                "transition_count": int(diagnostics[index["transition_count"]]),
                "flags": int(diagnostics[index["flags"]]),
                "command": command.tolist(),
            }
        )
    return outputs


def transition_rows(changed: tuple[int, int]) -> list[dict[str, Any]]:
    return [
        {"tick": 10, "stable": (1, 1), "hard": (1, 1), "fresh": True},
        {"tick": 11, "stable": (1, 1), "hard": changed},
        {"tick": 12, "stable": changed, "hard": changed},
        {"tick": 13, "stable": changed, "hard": changed},
    ]


def main() -> None:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    session = bonesaw.UpkieBalanceSession(str(model))
    cases = {
        "left_only": (transition_rows((1, 0)), 2),
        "right_only": (transition_rows((0, 1)), 2),
        "zero_support": (transition_rows((0, 0)), 2),
        "support_return": (
            [
                {"tick": 1, "stable": (1, 1), "hard": (1, 1), "fresh": True},
                {"tick": 2, "stable": (1, 0), "hard": (1, 0)},
                {"tick": 3, "stable": (1, 1), "hard": (1, 1)},
                {"tick": 4, "stable": (1, 0), "hard": (1, 0)},
            ],
            8,
        ),
        "missing_evidence": (
            [
                {"tick": 1, "stable": (1, 1), "hard": (1, 1), "fresh": True},
                {"tick": 2, "stable": (1, 1), "hard": (1, 0), "exact": False},
                {"tick": 3, "stable": (1, 0), "hard": (1, 0)},
            ],
            8,
        ),
        "zero_hold": (transition_rows((1, 0))[:2], 0),
        "invalid_command": (
            [
                {
                    "tick": 1,
                    "stable": (1, 1),
                    "hard": (1, 1),
                    "fresh": True,
                    "command": (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0),
                }
            ],
            8,
        ),
    }
    results: dict[str, Any] = {}
    for name, (rows, hold_ticks) in cases.items():
        first = run_rows(session, rows, hold_ticks)
        replay = run_rows(session, rows, hold_ticks)
        results[name] = {"rows": first, "exact_replay": first == replay}

    duplicate_rows = [
        {"tick": 5, "stable": (1, 1), "hard": (1, 1), "fresh": True},
        {"tick": 5, "stable": (1, 0), "hard": (1, 0)},
        {"tick": 6, "stable": (1, 0), "hard": (1, 0)},
    ]
    duplicate = run_rows(session, duplicate_rows, 8)
    left = results["left_only"]["rows"]
    right = results["right_only"]["rows"]
    zero = results["zero_support"]["rows"]
    returned = results["support_return"]["rows"]
    missing = results["missing_evidence"]["rows"]
    invalid = results["invalid_command"]["rows"]
    zero_hold = results["zero_hold"]["rows"]
    zero_command = [0.0] * 6
    gates = {
        "fresh_command_exact": all(
            rows[0]["status"] == "Fresh"
            and rows[0]["provenance"] == "Fresh"
            and rows[0]["executable"]
            for rows in (left, right, zero)
        ),
        "left_right_zero_transitions_mirrored": all(
            rows[1]["status"] == "Leased"
            and rows[2]["status"] == "Leased"
            and rows[3]["status"] == "Expired"
            for rows in (left, right, zero)
        ),
        "bounded_expiry_zeroes_command": all(
            not rows[3]["executable"] and rows[3]["command"] == zero_command
            for rows in (left, right, zero)
        ),
        "support_return_requires_fresh_admission": returned[2]["status"]
        == "RevokedSupport"
        and returned[3]["status"] == "Unavailable"
        and returned[3]["command"] == zero_command,
        "nonexact_evidence_revokes_without_resurrection": missing[1]["status"]
        == "RevokedEvidence"
        and missing[2]["status"] == "Unavailable"
        and missing[2]["command"] == zero_command,
        "zero_hold_never_replays": zero_hold[1]["status"] == "Expired"
        and not zero_hold[1]["executable"],
        "invalid_command_nonexecutable": invalid[0]["status"] == "RejectedCommand"
        and not invalid[0]["executable"],
        "duplicate_tick_atomic": duplicate[1]["status"] == "RejectedSequence"
        and not duplicate[1]["executable"]
        and duplicate[2]["status"] == "Leased"
        and duplicate[2]["age_ticks"] == 1,
        "exact_replay": all(value["exact_replay"] for value in results.values()),
        "finite_outputs": all(
            all(np.isfinite(row["command"]))
            for value in results.values()
            for row in value["rows"]
        ),
    }
    passed = all(gates.values())
    summary_rows = [
        [
            name,
            value["rows"][-1]["status"],
            value["rows"][-1]["provenance"],
            value["rows"][-1]["age_ticks"],
            "YES" if value["rows"][-1]["executable"] else "NO",
            "YES" if value["exact_replay"] else "NO",
        ]
        for name, value in results.items()
    ]
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "cases": results,
        "duplicate_sequence": duplicate,
        "contract": {
            "command_coordinates": 6,
            "default_transition_hold_ticks": 2,
            "policy_steps": 0,
            "physics_steps": 0,
        },
    }
    report = "\n".join(
        [
            "# Bonesaw bounded contact-command lease · r143",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. No WBC, policy, simulator, integration, or clock participates; Python supplies immutable rows and Rust owns command bytes, authoring support, tick age, and revocation.",
            "",
            "## Outcome",
            "",
            "An already admitted six-actuator command can cross an exact contact-mask edge for a fixed number of explicit ticks. The lease is mirrored for left-only and right-only support, covers zero support without claiming recovery, emits zero after expiry, and cannot resurrect after missing evidence or support return. This closes a command-lifetime contract only; it does not admit the command as physically safe.",
            "",
            *markdown_table(
                ["case", "final status", "provenance", "age ticks", "executable", "replay"],
                summary_rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Admission boundary",
            "",
            "- Fresh commands must already be admitted by the caller against exact evidence; the lease never solves or approves one.",
            "- Non-exact evidence, invalid commands, returned authoring support, and age expiry produce no executable command.",
            "- A physical A/B must still choose the hold horizon and prove no earlier fall, no lost green recovery, and no unbounded stale execution.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-command-lease-contract-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONTACT_COMMAND_LEASE_CONTRACT_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
