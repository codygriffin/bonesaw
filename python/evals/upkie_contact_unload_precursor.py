#!/usr/bin/env python3
"""Measure whether WBC normal-load share precedes observed wheel contact loss."""

from __future__ import annotations

import argparse
import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import markdown_table, render_report_html
from upkie_disturbance_envelope import case_matrix, run_case, semantic_trace_equal, summarize


REVISION = "upkie-contact-unload-precursor-r142"
DURATION_S = 6.0
THRESHOLDS = (0.10, 0.20, 0.30)
WINDOW_S = 0.50
CONSECUTIVE_TICKS = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_CONTACT_UNLOAD_PRECURSOR_R142.html"
    )
    return parser.parse_args()


def first_stable(mask: np.ndarray, start: int, stop: int) -> int | None:
    for tick in range(start, max(start, stop - CONSECUTIVE_TICKS + 1)):
        if np.all(mask[tick : tick + CONSECUTIVE_TICKS]):
            return tick
    return None


def analyze(case: Any, trace: dict[str, Any]) -> dict[str, Any]:
    time_s = np.asarray(trace["time_s"])
    observed = np.asarray(trace["observed_contact_active"], np.uint8)
    force = np.maximum(np.asarray(trace["wbc_normal_force"], np.float64), 0.0)
    total = np.sum(force, axis=1)
    share = np.divide(force, total[:, None], out=np.full_like(force, 0.5), where=total[:, None] > 1.0e-12)
    post_start = int(np.searchsorted(time_s, 0.05, side="left"))
    loss_ticks = np.flatnonzero(np.any(observed[post_start:] == 0, axis=1))
    first_loss = None if len(loss_ticks) == 0 else int(loss_ticks[0] + post_start)
    lost_wheel = None
    if first_loss is not None:
        missing = np.flatnonzero(observed[first_loss] == 0)
        lost_wheel = int(missing[0]) if len(missing) else None
    leads: dict[str, float | None] = {}
    for threshold in THRESHOLDS:
        if first_loss is None or lost_wheel is None:
            leads[f"{threshold:.2f}"] = None
            continue
        window_ticks = int(round(WINDOW_S / 0.005))
        start = max(post_start, first_loss - window_ticks)
        crossing = first_stable(share[:, lost_wheel] <= threshold, start, first_loss)
        leads[f"{threshold:.2f}"] = (
            None
            if crossing is None
            else float(time_s[first_loss] - time_s[crossing])
        )
    false_alert = {
        f"{threshold:.2f}": bool(
            first_stable(np.min(share, axis=1) <= threshold, post_start, len(time_s))
            is not None
        )
        for threshold in THRESHOLDS
    }
    summary = summarize(case, trace, DURATION_S)
    return {
        "outcome": summary["outcome"],
        "qualified": summary["qualified"],
        "terminal_time_s": summary["terminal_time_s"],
        "first_contact_loss_time_s": None if first_loss is None else float(time_s[first_loss]),
        "lost_wheel": None if lost_wheel is None else ("left" if lost_wheel == 0 else "right"),
        "minimum_left_load_share": float(np.min(share[post_start:, 0])),
        "minimum_right_load_share": float(np.min(share[post_start:, 1])),
        "lead_time_s": leads,
        "threshold_observed_without_conditioning_on_loss": false_alert,
        "allocation_free": summary["allocation_free"],
        "finite": summary["finite"],
    }


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model).resolve()
    rows: dict[str, Any] = {}
    for case in case_matrix():
        trace = run_case(model, case, DURATION_S, "capture", None, True, False)
        replay = run_case(model, case, DURATION_S, "capture", None, True, False)
        rows[case.name] = {
            **analyze(case, trace),
            "exact_replay": semantic_trace_equal(trace, replay),
        }
    green = [name for name, row in rows.items() if row["qualified"]]
    failures = [name for name, row in rows.items() if row["outcome"] == "FALL"]
    loss_rows = [name for name, row in rows.items() if row["first_contact_loss_time_s"] is not None]
    coverage = {
        f"{threshold:.2f}": sum(
            rows[name]["lead_time_s"][f"{threshold:.2f}"] is not None
            for name in loss_rows
        )
        for threshold in THRESHOLDS
    }
    green_false_alerts = {
        f"{threshold:.2f}": sum(
            rows[name]["threshold_observed_without_conditioning_on_loss"][f"{threshold:.2f}"]
            for name in green
        )
        for threshold in THRESHOLDS
    }
    gates = {
        "matrix_complete": len(rows) == 20,
        "green_rows_retain_double_contact": all(
            rows[name]["first_contact_loss_time_s"] is None for name in green
        ),
        "failure_contact_loss_discriminates": len(loss_rows) > 0
        and set(loss_rows).issubset(set(failures)),
        "twenty_percent_precursor_has_coverage": coverage["0.20"] > 0,
        "twenty_percent_no_green_false_alert": green_false_alerts["0.20"] == 0,
        "exact_replay": all(row["exact_replay"] for row in rows.values()),
        "zero_rust_allocation": all(row["allocation_free"] for row in rows.values()),
        "finite": all(row["finite"] for row in rows.values()),
    }
    passed = all(gates.values())
    table_rows = []
    for name, row in rows.items():
        table_rows.append(
            [
                name,
                row["outcome"],
                "—" if row["first_contact_loss_time_s"] is None else f"{row['first_contact_loss_time_s']:.3f}",
                row["lost_wheel"] or "—",
                f"{row['minimum_left_load_share']:.3f}/{row['minimum_right_load_share']:.3f}",
                *[
                    "—" if row["lead_time_s"][f"{threshold:.2f}"] is None else f"{row['lead_time_s'][f'{threshold:.2f}']:.3f}"
                    for threshold in THRESHOLDS
                ],
                "YES" if row["exact_replay"] else "NO",
            ]
        )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "thresholds": THRESHOLDS,
        "window_s": WINDOW_S,
        "consecutive_ticks": CONSECUTIVE_TICKS,
        "green_rows": green,
        "failure_rows": failures,
        "contact_loss_rows": loss_rows,
        "coverage": coverage,
        "green_false_alerts": green_false_alerts,
        "rows": rows,
        "interpretation": {
            "controller_promotion": False,
            "signal": "per-wheel positive normal-force share from the current WBC",
            "contact_estimator_claim": False,
        },
    }
    report = "\n".join(
        [
            "# Bonesaw pre-contact-loss load precursor · r142",
            "",
            f"> Evaluation **{'PASS' if passed else 'FAIL'}**. This admits a continuous warning signal, not a contact estimator or recovery policy.",
            "",
            "## Outcome",
            "",
            "The current live r137 controller is replayed over the frozen 20-case matrix. Exact per-wheel plant contact remains the event label; the candidate warning is the positive WBC normal-force share on the wheel that later loses contact. Fixed 10/20/30% thresholds require two consecutive 5 ms samples inside a 500 ms pre-loss window. Green rows are checked for false alerts independently.",
            "",
            *markdown_table(
                ["case", "outcome", "first loss s", "wheel", "min L/R share", "10% lead s", "20% lead s", "30% lead s", "replay"],
                table_rows,
            ),
            "",
            "## Threshold discrimination",
            "",
            *markdown_table(
                ["threshold", "loss rows covered", "green false alerts"],
                [[f"{100*threshold:.0f}%", coverage[f"{threshold:.2f}"], green_false_alerts[f"{threshold:.2f}"]] for threshold in THRESHOLDS],
            ),
            "",
            "## Gates",
            "",
            *markdown_table(["gate", "result"], [[name, "PASS" if value else "FAIL"] for name, value in gates.items()]),
            "",
            "## Boundary",
            "",
            "- Normal-load share is model output and remains separate from exact contact observation, debounced mode, WBC admission, command freshness, and plant outcome.",
            "- A threshold crossing may authorize a viability layer to start preparing a support action; it cannot author a contact edge or execute an unadmitted reduced-support solve.",
            "- Hardware requires calibrated force/torque or motor-current evidence and delay/noise/dropout validation.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-contact-unload-precursor-metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    (output / "UPKIE_CONTACT_UNLOAD_PRECURSOR_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "gates": gates, "coverage": coverage, "green_false_alerts": green_false_alerts, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
