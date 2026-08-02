#!/usr/bin/env python3
"""Admit the causal Rust contact-observation contract without a plant or WBC."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html


REVISION = "upkie-contact-observation-contract-r139"
DT_NS = 5_000_000
STATUS = {
    0: "Accepted",
    1: "Missing",
    2: "RejectedFuture",
    3: "RejectedStale",
    4: "RejectedUncertain",
    5: "RejectedSource",
    6: "RejectedSequence",
    7: "RejectedTimestamp",
    8: "InvalidConfig",
}
PROVENANCE = {0: "Exact", 1: "Held", 2: "Unavailable"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_OBSERVATION_CONTRACT_R139.html"
    )
    return parser.parse_args()


def observation(
    active: tuple[int, int],
    *,
    sequence: int,
    tick: int | None = None,
    mapped: int | None = None,
    available: bool = True,
    source: int = 1,
    uncertainty: int = 100,
) -> dict[str, Any]:
    default_time = sequence * DT_NS
    return {
        "active": active,
        "sequence": sequence,
        "tick": default_time if tick is None else tick,
        "mapped": default_time if mapped is None else mapped,
        "available": available,
        "source": source,
        "uncertainty": uncertainty,
    }


def run_rows(session: Any, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    names = list(session.contact_observation_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    raw = np.empty(2, np.uint8)
    debounced = np.empty(2, np.uint8)
    hard = np.empty(2, np.uint8)
    diagnostics = np.empty(len(names), np.int64)
    outputs: list[dict[str, Any]] = []
    for row in rows:
        raw[:] = row["active"]
        session.step_contact_observation_from_mask(
            row["tick"],
            row["available"],
            row["mapped"],
            row["sequence"],
            row["source"],
            row["uncertainty"],
            raw,
            debounced,
            hard,
            diagnostics,
        )
        outputs.append(
            {
                "status": STATUS[int(diagnostics[index["status"]])],
                "provenance": PROVENANCE[int(diagnostics[index["provenance"]])],
                "accepted": bool(diagnostics[index["accepted"]]),
                "hard_constraint_eligible": bool(
                    diagnostics[index["hard_constraint_eligible"]]
                ),
                "age_ns": int(diagnostics[index["age_ns"]]),
                "raw": [int(value) for value in raw],
                "debounced": [int(value) for value in debounced],
                "hard": [int(value) for value in hard],
                "pending": [
                    int(diagnostics[index["left_pending_samples"]]),
                    int(diagnostics[index["right_pending_samples"]]),
                ],
                "transition_count": int(diagnostics[index["transition_count"]]),
                "flags": int(diagnostics[index["flags"]]),
            }
        )
    return outputs


def timeline(active_rows: list[tuple[int, int]]) -> list[dict[str, Any]]:
    return [
        observation(active, sequence=index + 1)
        for index, active in enumerate(active_rows)
    ]


def main() -> None:
    import bonesaw

    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    session = bonesaw.UpkieBalanceSession(str(model))
    cases = {
        "activate_double": timeline([(1, 1)] * 4),
        "left_only": timeline([(1, 1)] * 3 + [(1, 0)] * 3),
        "right_only": timeline([(1, 1)] * 3 + [(0, 1)] * 3),
        "zero_support": timeline([(1, 1)] * 3 + [(0, 0)] * 3),
        "loss_chatter": timeline(
            [(1, 1)] * 3 + [(1, 0), (1, 1), (1, 0), (1, 1), (1, 1)]
        ),
        "activation_chatter": timeline(
            [(0, 0), (1, 0), (0, 0), (1, 0), (0, 0), (1, 0), (1, 0), (1, 0)]
        ),
    }
    results: dict[str, Any] = {}
    for name, rows in cases.items():
        session.reset()
        first = run_rows(session, rows)
        session.reset()
        replay = run_rows(session, rows)
        results[name] = {"rows": first, "exact_replay": first == replay}

    fault_prefix = timeline([(1, 1)] * 3)
    last_time = 3 * DT_NS
    fault_rows = {
        "future": observation(
            (0, 0), sequence=4, tick=4 * DT_NS, mapped=4 * DT_NS + 1
        ),
        "stale": observation(
            (0, 0),
            sequence=4,
            tick=9 * DT_NS,
            mapped=4 * DT_NS,
        ),
        "uncertain": observation(
            (0, 0), sequence=4, uncertainty=2_000_001
        ),
        "wrong_source": observation((0, 0), sequence=4, source=2),
        "duplicate_sequence": observation((0, 0), sequence=3, tick=4 * DT_NS),
        "old_timestamp": observation(
            (0, 0), sequence=4, tick=4 * DT_NS, mapped=last_time
        ),
        "missing_held": observation(
            (0, 0),
            sequence=4,
            tick=last_time + DT_NS,
            mapped=0,
            available=False,
        ),
        "missing_expired": observation(
            (0, 0),
            sequence=4,
            tick=last_time + 20_000_001,
            mapped=0,
            available=False,
        ),
    }
    faults: dict[str, Any] = {}
    for name, row in fault_rows.items():
        session.reset()
        prefix = run_rows(session, fault_prefix)
        output = run_rows(session, [row])[0]
        faults[name] = {
            "before": prefix[-1],
            "fault": output,
            "state_preserved": output["debounced"] == prefix[-1]["debounced"]
            and output["transition_count"] == prefix[-1]["transition_count"],
        }

    left = results["left_only"]["rows"]
    right = results["right_only"]["rows"]
    zero = results["zero_support"]["rows"]
    chatter = results["loss_chatter"]["rows"]
    activation_chatter = results["activation_chatter"]["rows"]
    expected_fault_status = {
        "future": "RejectedFuture",
        "stale": "RejectedStale",
        "uncertain": "RejectedUncertain",
        "wrong_source": "RejectedSource",
        "duplicate_sequence": "RejectedSequence",
        "old_timestamp": "RejectedTimestamp",
        "missing_held": "Missing",
        "missing_expired": "Missing",
    }
    gates = {
        "three_sample_activation": results["activate_double"]["rows"][1]["hard"]
        == [0, 0]
        and results["activate_double"]["rows"][2]["hard"] == [1, 1],
        "absence_removes_hard_row_immediately": left[3]["hard"] == [1, 0]
        and right[3]["hard"] == [0, 1]
        and zero[3]["hard"] == [0, 0],
        "two_sample_mode_deactivation": left[3]["debounced"] == [1, 1]
        and left[4]["debounced"] == [1, 0]
        and right[4]["debounced"] == [0, 1]
        and zero[4]["debounced"] == [0, 0],
        "mirrored_single_contact": left[4]["transition_count"]
        == right[4]["transition_count"]
        and left[4]["debounced"] == [1, 0]
        and right[4]["debounced"] == [0, 1],
        "loss_chatter_does_not_change_mode": chatter[-1]["debounced"] == [1, 1]
        and chatter[-1]["transition_count"] == 2,
        "activation_chatter_requires_consecutive_evidence": activation_chatter[5][
            "debounced"
        ]
        == [0, 0]
        and activation_chatter[-1]["debounced"] == [1, 0],
        "faults_typed_atomic_and_nonhard": all(
            value["fault"]["status"] == expected_fault_status[name]
            and value["fault"]["hard"] == [0, 0]
            and value["state_preserved"]
            for name, value in faults.items()
        ),
        "missing_provenance_expires": faults["missing_held"]["fault"]["provenance"]
        == "Held"
        and faults["missing_expired"]["fault"]["provenance"] == "Unavailable",
        "exact_replay": all(value["exact_replay"] for value in results.values()),
    }
    passed = all(gates.values())
    summary_rows = [
        [
            name,
            "".join(str(value) for value in value["rows"][-1]["raw"]),
            "".join(str(value) for value in value["rows"][-1]["debounced"]),
            "".join(str(value) for value in value["rows"][-1]["hard"]),
            value["rows"][-1]["transition_count"],
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
        "faults": faults,
        "contract": {
            "sample_period_ns": DT_NS,
            "activation_samples": 3,
            "deactivation_samples": 2,
            "maximum_age_ns": 20_000_000,
            "maximum_synchronization_uncertainty_ns": 2_000_000,
            "expected_source_identity": 1,
        },
    }
    report = "\n".join(
        [
            "# Bonesaw causal contact-observation contract · r139",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. No WBC, policy, simulator, integration, or clock participates; Python supplies immutable observation rows and Rust owns every state transition.",
            "",
            "## Outcome",
            "",
            "The contract separates exact raw contact, debounced support mode, and hard-row eligibility. A new contact needs three consecutive exact samples. Exact absence removes its hard row immediately, while the stable mode changes after two samples. Missing, held, future, stale, uncertain, wrong-source, duplicate, and time-reordered evidence never authors a hard row. Left-only and right-only transitions are mirrored and exact replay passes.",
            "",
            *markdown_table(
                ["case", "final raw", "final mode", "final hard", "transitions", "replay"],
                summary_rows,
            ),
            "",
            "## Fault admission",
            "",
            *markdown_table(
                ["fault", "status", "provenance", "hard", "state atomic"],
                [
                    [
                        name,
                        value["fault"]["status"],
                        value["fault"]["provenance"],
                        "".join(str(item) for item in value["fault"]["hard"]),
                        "YES" if value["state_preserved"] else "NO",
                    ]
                    for name, value in faults.items()
                ],
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
            "- The caller owns control time and observation clock mapping. Rust validates mapped time, age, source identity, sequence, and synchronization uncertainty.",
            "- Held mode state remains visible but is never hard-constraint eligible. A rejected observation is atomic.",
            "- Debounce state is not contact estimation. Live promotion still requires a calibrated source and a controller A/B for each support mode.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-observation-contract-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_CONTACT_OBSERVATION_CONTRACT_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
