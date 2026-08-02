#!/usr/bin/env python3
"""Retained binding contract for the generic Rust viability-request supervisor."""

from __future__ import annotations

import argparse
import gc
import json
import pathlib
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from upkie_mujoco_plant_report import rss_bytes


REVISION = "upkie-viability-request-supervisor-r148"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", type=int, default=20_000)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/UPKIE_VIABILITY_REQUEST_SUPERVISOR_R148.html"
    )
    return parser.parse_args()


def session_and_buffers(bonesaw: Any, model: pathlib.Path) -> tuple[Any, ...]:
    session = bonesaw.UpkieBalanceSession(str(model))
    session.configure_viability_request(
        0.10,
        0.05,
        4,
        np.asarray([250.0, 250.0, 80.0], np.float64),
        np.asarray([40.0, 40.0, 20.0], np.float64),
    )
    names = list(session.viability_request_diagnostic_names)
    return (
        session,
        names,
        {name: slot for slot, name in enumerate(names)},
        np.zeros(3, np.float64),
        np.empty(3, np.float64),
        np.empty(len(names), np.float64),
    )


def run_events(bonesaw: Any, model: pathlib.Path) -> list[dict[str, Any]]:
    session, names, index, candidate, request, diagnostics = session_and_buffers(
        bonesaw, model
    )
    events = (
        ("activate", 1, True, True, 0.20, True, (100.0, -60.0, 30.0)),
        ("hold", 2, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("duplicate", 2, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("hold_after_reject", 3, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("evidence_loss", 4, False, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("reactivate", 5, True, True, 0.20, True, (80.0, 20.0, 0.0)),
        ("hold_after_reactivate", 6, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("release", 7, True, True, 0.04, False, (0.0, 0.0, 0.0)),
        ("release_hold", 8, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("inactive", 9, True, False, 0.0, False, (0.0, 0.0, 0.0)),
        ("bad_candidate", 10, True, True, 0.20, True, (251.0, 0.0, 0.0)),
    )
    rows: list[dict[str, Any]] = []
    for label, tick, exact, update, pressure, available, values in events:
        candidate[:] = values
        session.step_viability_request(
            tick,
            exact,
            update,
            pressure,
            available,
            candidate,
            request,
            diagnostics,
        )
        rows.append(
            {
                "event": label,
                "tick": tick,
                "status": int(diagnostics[index["status"]]),
                "provenance": int(diagnostics[index["provenance"]]),
                "active": bool(diagnostics[index["active"]]),
                "executable": bool(diagnostics[index["executable"]]),
                "age_ticks": int(diagnostics[index["candidate_age_ticks"]]),
                "remaining_ticks": int(
                    diagnostics[index["remaining_fresh_ticks"]]
                ),
                "slew_limited": bool(
                    diagnostics[index["request_was_slew_limited"]]
                ),
                "request": request.tolist(),
                "flags": int(diagnostics[index["flags"]]),
            }
        )
    assert len(names) == 17
    return rows


def benchmark(bonesaw: Any, model: pathlib.Path, ticks: int) -> dict[str, Any]:
    session, _, _, candidate, request, diagnostics = session_and_buffers(bonesaw, model)
    timing = np.empty(ticks, np.float64)
    gc.collect()
    gc_before = np.asarray([item["collections"] for item in gc.get_stats()])
    rss_before = rss_bytes()
    for offset in range(ticks):
        tick = offset + 1
        update = offset % 4 == 0
        if update:
            sign = 1.0 if (offset // 4) % 2 == 0 else -1.0
            candidate[:] = (80.0 * sign, -60.0 * sign, 20.0 * sign)
        started = time.perf_counter_ns()
        session.step_viability_request(
            tick,
            True,
            update,
            0.20 if update else 0.0,
            update,
            candidate,
            request,
            diagnostics,
        )
        timing[offset] = time.perf_counter_ns() - started
    gc_after = np.asarray([item["collections"] for item in gc.get_stats()])
    return {
        "ticks": ticks,
        "timing_ns": distribution(timing),
        "python_gc_collections": int(np.sum(gc_after - gc_before)),
        "rss_delta_bytes": rss_bytes() - rss_before,
        "binding_allocation_guard_passed": True,
    }


def main() -> None:
    import bonesaw

    args = parse_args()
    if args.ticks < 1:
        raise SystemExit("--ticks must be positive")
    model = pathlib.Path(args.model).resolve()
    events = run_events(bonesaw, model)
    replay = run_events(bonesaw, model)
    bench = benchmark(bonesaw, model, args.ticks)
    statuses = {row["event"]: row["status"] for row in events}
    gates = {
        "fresh_hold_release_sequence": statuses
        == {
            "activate": 1,
            "hold": 2,
            "duplicate": 6,
            "hold_after_reject": 2,
            "evidence_loss": 5,
            "reactivate": 1,
            "hold_after_reactivate": 2,
            "release": 3,
            "release_hold": 0,
            "inactive": 0,
            "bad_candidate": 7,
        },
        "duplicate_is_atomic": events[3]["request"] == [100.0, -60.0, 30.0],
        "evidence_loss_revokes_without_tail": not events[4]["executable"]
        and events[4]["request"] == [0.0, 0.0, 0.0],
        "bad_input_fails_closed": not events[-1]["executable"]
        and events[-1]["request"] == [0.0, 0.0, 0.0],
        "exact_replay": events == replay,
        "zero_rust_hot_path_allocation": bench["binding_allocation_guard_passed"],
        "zero_python_gc": bench["python_gc_collections"] == 0,
    }
    passed = all(gates.values())
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model),
        "passed": passed,
        "gates": gates,
        "events": events,
        "benchmark": bench,
    }
    rows = [
        [
            row["event"],
            row["tick"],
            row["status"],
            row["provenance"],
            row["active"],
            row["executable"],
            row["age_ticks"],
            "/".join(f"{value:+.0f}" for value in row["request"]),
        ]
        for row in events
    ]
    report = "\n".join(
        [
            "# Bonesaw viability-request supervisor contract · r148",
            "",
            f"> Contract **{'PASS' if passed else 'FAIL'}**. This admits the generic request-lifetime mechanism, not the planner or plant consequence.",
            "",
            "## State sequence",
            "",
            *markdown_table(
                [
                    "event",
                    "tick",
                    "status",
                    "provenance",
                    "active",
                    "exec",
                    "age",
                    "request r/l/y",
                ],
                rows,
            ),
            "",
            "## Gates",
            "",
            *markdown_table(
                ["gate", "result"],
                [[name, "PASS" if value else "FAIL"] for name, value in gates.items()],
            ),
            "",
            "## Timing and ownership",
            "",
            f"The caller-owned NumPy boundary ran **{args.ticks:,}** transitions at p50/p95/p99 **{bench['timing_ns']['p50'] / 1e3:.3f} / {bench['timing_ns']['p95'] / 1e3:.3f} / {bench['timing_ns']['p99'] / 1e3:.3f} µs**. Python GC collections were **{bench['python_gc_collections']}** and RSS delta was **{bench['rss_delta_bytes'] / 1_048_576:.3f} MiB**. The PyO3 method snapshots the Rust allocator counters around the state transition and raises if either changes; every retained call passed.",
            "",
            "Rust owns tick ordering, activation/release hysteresis, candidate age, per-axis bounds and slew, target/request state, typed provenance, transition count, and exact-evidence revocation. Python owns only event construction and reporting. A nonzero output is still a request: downstream WBC admission is required every tick.",
        ]
    ) + "\n"
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "upkie-viability-request-supervisor-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (output / "UPKIE_VIABILITY_REQUEST_SUPERVISOR_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report))
    print(json.dumps({"passed": passed, "web_report": str(web_report)}, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
