#!/usr/bin/env python3
"""Retained repeated end-to-end audit of the live Upkie PUSH gateway."""

from __future__ import annotations

import argparse
import json
import pathlib
import platform
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table, render_report_html
from live_plant_gateway_smoke import run_retained as run


REVISION = "live-upkie-plant-gateway-r131"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8777")
    parser.add_argument("--connect-address")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/LIVE_UPKIE_PLANT_GATEWAY_R131.html"
    )
    return parser.parse_args()


def make_report(metrics: dict[str, Any]) -> str:
    rows = []
    for index, trial in enumerate(metrics["trials"], start=1):
        rows.append(
            [
                index,
                f'{trial["maximum_root_x_delta_m"]:.4f}',
                f'{trial["maximum_tilt_delta_deg"]:.2f}',
                f'{trial["maximum_capture_pressure"]:.3f}',
                f'{trial["final_recovery"]["tilt_deg"]:.3f}',
                f'{trial["final_recovery"]["station_error_m"] * 1000:.2f}',
                f'{trial["release_ack_ms"]:.1f}',
                f'{trial["expiry_observed_ms"]:.1f}',
                f'{trial["controller_step_us"]["p99"]:.1f}',
                f'{trial["worker_step_us"]["p99"]:.1f}',
                f'{trial["stream_interval_ms"]["p99"]:.1f}',
            ]
        )
    gate_rows = [
        [name, gate["observed"], gate["pass"]]
        for name, gate in metrics["gates"].items()
    ]
    return "\n".join(
        [
            f'# Live Upkie plant gateway audit · {metrics["revision"]}',
            "",
            f'Admission: {"PASS" if metrics["admission"] else "FAIL"}. This is a repeated end-to-end transport and plant-response audit. Browser-protocol commands enter the Rust WebSocket server, which validates force and expiry, supervises a per-session Python MuJoCo worker, and streams measured transforms plus Rust capture/WBC evidence back. TARGET preview and PUSH physics remain separate sockets and command types.',
            "",
            "PUSH drag → Rust validation/TTL → Python MuJoCo plant → observed q/v/root → Rust capture + WBC → torque → MuJoCo → plant_state.",
            "",
            "## Repeated response",
            "",
            *markdown_table(
                [
                    "trial",
                    "peak Δx m",
                    "peak tilt Δ°",
                    "capture",
                    "final tilt °",
                    "final station mm",
                    "release ms",
                    "expiry ms",
                    "Rust p99 µs",
                    "worker p99 µs",
                    "stream p99 ms",
                ],
                rows,
            ),
            "",
            "Each trial starts a fresh isolated plant session, applies 4 N to the base for five 20 ms stream frames, releases it, observes 2.1 seconds of recovery, verifies one-shot force expiry, injects an over-limit command, proves the stream survives, and performs a correlated reset.",
            "",
            "## Admission gates",
            "",
            *markdown_table(["gate", "observed", "pass"], gate_rows),
            "",
            "## Deliberate limits",
            "",
            "This admits one sagittal body-center force on one Upkie soft-contact plant with ideal state observation. It does not establish arbitrary application-point torque, lateral recovery, slopes/friction variation, delay/noise, multiple simultaneous clients sharing one physical robot, thermal/hardware safety, or browser frame time. The in-app browser was unavailable for this CLI run, so visual/touch/mobile acceptance remains separate from these protocol and plant gates.",
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    if args.trials < 2:
        raise ValueError("trials must be at least two")
    trials = [run(args.url.rstrip("/"), args.connect_address) for _ in range(args.trials)]
    maximum = lambda path: max(
        path(trial) for trial in trials
    )
    minimum = lambda path: min(
        path(trial) for trial in trials
    )
    gates = {
        "typed ownership boundary": {
            "observed": sorted({trial["plant_boundary"] for trial in trials}),
            "pass": all(
                trial["plant_boundary"]
                == "python_mujoco_plant__rust_capture_wbc"
                for trial in trials
            ),
        },
        "physical push is observable": {
            "observed": f'min Δx {minimum(lambda trial: trial["maximum_root_x_delta_m"]):.4f} m; min tilt {minimum(lambda trial: trial["maximum_tilt_delta_deg"]):.2f} deg',
            "pass": minimum(lambda trial: trial["maximum_root_x_delta_m"]) > 0.05
            and minimum(lambda trial: trial["maximum_tilt_delta_deg"]) > 10.0,
        },
        "capture pressure is exercised": {
            "observed": minimum(lambda trial: trial["maximum_capture_pressure"]),
            "pass": minimum(lambda trial: trial["maximum_capture_pressure"]) >= 0.99,
        },
        "release and one-shot expiry are fail-safe": {
            "observed": f'max release {maximum(lambda trial: trial["release_ack_ms"]):.1f} ms; expiry {minimum(lambda trial: trial["expiry_observed_ms"]):.1f}–{maximum(lambda trial: trial["expiry_observed_ms"]):.1f} ms',
            "pass": maximum(lambda trial: trial["release_ack_ms"]) < 100.0
            and minimum(lambda trial: trial["expiry_observed_ms"]) >= 140.0
            and maximum(lambda trial: trial["expiry_observed_ms"]) < 220.0,
        },
        "invalid command does not kill stream": {
            "observed": all(
                trial["invalid_force_rejected_stream_survived"] for trial in trials
            ),
        },
        "tilt and odom station recover": {
            "observed": f'max final tilt {maximum(lambda trial: trial["final_recovery"]["tilt_deg"]):.3f} deg; max absolute station error {maximum(lambda trial: abs(trial["final_recovery"]["station_error_m"])) * 1000:.2f} mm',
            "pass": maximum(lambda trial: trial["final_recovery"]["tilt_deg"]) < 2.0
            and maximum(
                lambda trial: abs(trial["final_recovery"]["station_error_m"])
            )
            < 0.05
            and minimum(
                lambda trial: trial["final_recovery"]["station_authority"]
            )
            > 0.99,
        },
        "correlated reset restores seed": {
            "observed": f'max root error {maximum(lambda trial: trial["reset_root_error_m"]):.3e} m; max ack {maximum(lambda trial: trial["reset_ack_ms"]):.1f} ms',
            "pass": maximum(lambda trial: trial["reset_root_error_m"]) < 0.01
            and maximum(lambda trial: trial["reset_ack_ms"]) < 100.0,
        },
        "no numerical auto-reset": {
            "observed": maximum(lambda trial: trial["numeric_resets"]),
            "pass": maximum(lambda trial: trial["numeric_resets"]) == 0,
        },
        "Rust controller p99 below 1 ms": {
            "observed": f'{maximum(lambda trial: trial["controller_step_us"]["p99"]):.1f} µs',
            "pass": maximum(lambda trial: trial["controller_step_us"]["p99"]) < 1000.0,
        },
        "four-control-tick worker p99 below 5 ms": {
            "observed": f'{maximum(lambda trial: trial["worker_step_us"]["p99"]):.1f} µs',
            "pass": maximum(lambda trial: trial["worker_step_us"]["p99"]) < 5000.0,
        },
        "50 Hz stream p99 below 60 ms": {
            "observed": f'{maximum(lambda trial: trial["stream_interval_ms"]["p99"]):.1f} ms',
            "pass": maximum(lambda trial: trial["stream_interval_ms"]["p99"]) < 60.0,
        },
    }
    for gate in gates.values():
        gate["pass"] = bool(gate.get("pass", gate["observed"]))
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
    metrics_path = output / "live-upkie-plant-gateway-metrics.json"
    report_path = output / "LIVE_UPKIE_PLANT_GATEWAY_AUDIT.md"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="Live Upkie plant gateway")
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
