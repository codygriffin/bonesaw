#!/usr/bin/env python3
"""Retained report for bounded live application-point wrench semantics."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
import statistics
from datetime import datetime, timezone
from typing import Any, Callable

from cpu_reference_report import markdown_table, render_report_html
from live_wrench_application_probe import run


REVISION = "live-wrench-application-r132"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8777")
    parser.add_argument("--connect-address")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument("--web-report", default="web/LIVE_WRENCH_APPLICATION_R132.html")
    return parser.parse_args()


def condition(trial: dict[str, Any], index: int) -> dict[str, Any]:
    return trial["conditions"][index]


def values(trials: list[dict[str, Any]], getter: Callable[[dict[str, Any]], float]) -> list[float]:
    return [float(getter(trial)) for trial in trials]


def spread(samples: list[float]) -> float:
    return max(samples) - min(samples)


def make_report(metrics: dict[str, Any]) -> str:
    rows = []
    for trial_index, trial in enumerate(metrics["trials"], start=1):
        low, center, high = trial["conditions"]
        rows.append(
            [
                trial_index,
                f'{low["moment_y_nm"][-1]:.6f}',
                f'{center["moment_y_nm"][-1]:.6f}',
                f'{high["moment_y_nm"][-1]:.6f}',
                f'{low["final_pitch_rad"]:.6f}',
                f'{center["final_pitch_rad"]:.6f}',
                f'{high["final_pitch_rad"]:.6f}',
                f'{low["final_pitch_rate_rad_s"]:.6f}',
                f'{center["final_pitch_rate_rad_s"]:.6f}',
                f'{high["final_pitch_rate_rad_s"]:.6f}',
            ]
        )
    gates = [
        [name, gate["observed"], gate["pass"]]
        for name, gate in metrics["gates"].items()
    ]
    return "\n".join(
        [
            f'# Live application-point wrench audit · {metrics["revision"]}',
            "",
            f'Admission: {"PASS" if metrics["admission"] else "FAIL"}. Equal 2 N world-X forces are applied for three 20 ms stream frames at the base origin and at ±200 mm world-Z offsets. MuJoCo maps each point force through mj_applyFT; the worker separately streams the COM-relative moment and lever while Rust limits the point to 750 mm from the latest body origin.',
            "",
            "The expected differential moment is Δτ = Δp × F = 0.4 m × 2 N = 0.8 N·m. This audit requires both that wrench identity and a signed, ordered physical pitch consequence; it does not infer application-point semantics from a force arrow.",
            "",
            "## Repeated physical consequence",
            "",
            *markdown_table(
                [
                    "trial",
                    "τy −200 mm N·m",
                    "τy center N·m",
                    "τy +200 mm N·m",
                    "pitch −200 mm rad",
                    "pitch center rad",
                    "pitch +200 mm rad",
                    "rate −200 mm rad/s",
                    "rate center rad/s",
                    "rate +200 mm rad/s",
                ],
                rows,
            ),
            "",
            "Each condition owns a fresh isolated MuJoCo and persistent Rust capture/WBC session. Timing remains descriptive and is gated separately from the physical ordering.",
            "",
            "## Admission gates",
            "",
            *markdown_table(["gate", "observed", "pass"], gates),
            "",
            "## Ownership and limits",
            "",
            "Rust owns finite schema validation, body existence/readiness, the 8 N force cap, the 750 mm body-origin point lease, TTL, correlation, and clearing a rejected push. Python owns MuJoCo topology and therefore independently checks the exact body-COM lever before calling mj_applyFT. The browser now displays force and measured moment separately.",
            "",
            "This is one sagittal Upkie body, one force direction, three points, ideal observation, and 60 ms of forcing. It does not establish lateral recovery, arbitrary body surfaces, sustained/repeated wrench safety, friction or slope robustness, delay/noise tolerance, thermal limits, hardware behavior, or browser frame time.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.trials < 3:
        raise ValueError("trials must be at least three")
    trials = [run(args.url.rstrip("/"), args.connect_address) for _ in range(args.trials)]
    low_moment = values(trials, lambda trial: condition(trial, 0)["moment_y_nm"][-1])
    center_moment = values(trials, lambda trial: condition(trial, 1)["moment_y_nm"][-1])
    high_moment = values(trials, lambda trial: condition(trial, 2)["moment_y_nm"][-1])
    low_pitch = values(trials, lambda trial: condition(trial, 0)["final_pitch_rad"])
    center_pitch = values(trials, lambda trial: condition(trial, 1)["final_pitch_rad"])
    high_pitch = values(trials, lambda trial: condition(trial, 2)["final_pitch_rad"])
    low_rate = values(trials, lambda trial: condition(trial, 0)["final_pitch_rate_rad_s"])
    center_rate = values(trials, lambda trial: condition(trial, 1)["final_pitch_rate_rad_s"])
    high_rate = values(trials, lambda trial: condition(trial, 2)["final_pitch_rate_rad_s"])
    differential_moment = [high - low for high, low in zip(high_moment, low_moment)]
    controller_us = [
        value
        for trial in trials
        for item in trial["conditions"]
        for value in item["controller_step_us"]
    ]
    worker_us = [
        value
        for trial in trials
        for item in trial["conditions"]
        for value in item["worker_step_us"]
    ]
    gates = {
        "streamed moment has the expected sign": {
            "observed": f'low {max(low_moment):.6f}; center max abs {max(abs(value) for value in center_moment):.6f}; high {min(high_moment):.6f} N·m',
            "pass": max(low_moment) < -0.35
            and max(abs(value) for value in center_moment) < 0.01
            and min(high_moment) > 0.35,
        },
        "point-force differential matches cross product": {
            "observed": f'{min(differential_moment):.6f}–{max(differential_moment):.6f} N·m against 0.800000 N·m',
            "pass": max(abs(value - 0.8) for value in differential_moment) < 0.01,
        },
        "signed pitch consequence is ordered": {
            "observed": f'low {max(low_pitch):.6f}; center {statistics.mean(center_pitch):.6f}; high {min(high_pitch):.6f} rad',
            "pass": max(low_pitch) < -0.015
            and min(center_pitch) > 0.02
            and min(high_pitch) > 0.07
            and all(low < center < high for low, center, high in zip(low_pitch, center_pitch, high_pitch)),
        },
        "signed pitch-rate consequence is ordered": {
            "observed": f'low {max(low_rate):.6f}; center {statistics.mean(center_rate):.6f}; high {min(high_rate):.6f} rad/s',
            "pass": max(low_rate) < -0.5
            and min(center_rate) > 0.7
            and min(high_rate) > 2.0
            and all(low < center < high for low, center, high in zip(low_rate, center_rate, high_rate)),
        },
        "physical outputs repeat across fresh sessions": {
            "observed": f'max pitch spread {max(spread(low_pitch), spread(center_pitch), spread(high_pitch)):.3e} rad; moment spread {max(spread(low_moment), spread(center_moment), spread(high_moment)):.3e} N·m',
            "pass": max(spread(low_pitch), spread(center_pitch), spread(high_pitch)) < 1.0e-12
            and max(spread(low_moment), spread(center_moment), spread(high_moment)) < 1.0e-12,
        },
        "all bounded conditions remain upright": {
            "observed": sum(item["fallen"] for trial in trials for item in trial["conditions"]),
            "pass": not any(item["fallen"] for trial in trials for item in trial["conditions"]),
        },
        "excessive lever is rejected and cleared": {
            "observed": all(
                trial["excessive_offset_rejection"]["stream_survived"]
                and trial["excessive_offset_rejection"]["active_push_cleared"]
                for trial in trials
            ),
            "pass": all(
                trial["excessive_offset_rejection"]["stream_survived"]
                and trial["excessive_offset_rejection"]["active_push_cleared"]
                for trial in trials
            ),
        },
        "controller maximum below 1 ms": {
            "observed": f'{max(controller_us):.1f} µs',
            "pass": max(controller_us) < 1000.0,
        },
        "four-tick worker maximum below 5 ms": {
            "observed": f'{max(worker_us):.1f} µs',
            "pass": max(worker_us) < 5000.0,
        },
    }
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "machine": platform.machine()},
        "url": args.url,
        "trials": trials,
        "gates": gates,
        "admission": all(gate["pass"] for gate in gates.values()),
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_path = output / "live-wrench-application-metrics.json"
    report_path = output / "LIVE_WRENCH_APPLICATION_AUDIT.md"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Live application-point wrench")
    )
    print(
        json.dumps(
            {
                "admission": metrics["admission"],
                "metrics": str(metrics_path),
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if metrics["admission"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
