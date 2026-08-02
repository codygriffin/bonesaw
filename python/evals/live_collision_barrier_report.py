#!/usr/bin/env python3
"""Retain the live floating collision-barrier WebSocket authority row."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import platform
from collections import Counter
from datetime import datetime, timezone
from typing import Any

import numpy as np
from websocket import create_connection

from cpu_reference_report import distribution, markdown_table, render_report_html
from live_authority_stream_report import BASE_CAPABILITIES, receive_kind


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8800/ws")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--pull-m", type=float, default=0.08)
    parser.add_argument("--expected-support", choices=("measured", "unavailable"), default="measured")
    parser.add_argument("--output", default="benchmarks/results/live-collision-barrier-r108")
    parser.add_argument("--web-report", default="web/LIVE_COLLISION_BARRIER_R108.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def finite_series(frames: list[dict[str, Any]], key: str) -> np.ndarray:
    values = np.asarray([frame["metrics"][key] for frame in frames], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise AssertionError(f"non-finite live collision field: {key}")
    return values


def evaluate(url: str, frame_count: int, pull_m: float, expected_support: str) -> dict[str, Any]:
    socket = create_connection(url, timeout=5)
    try:
        hello = receive_kind(socket, "hello", 1)
        contract = hello["authority_contract"]
        capabilities = {
            signal["stable_id"]: signal["availability"]
            for signal in contract["signals"]
        }
        expected = {**BASE_CAPABILITIES, "finite_support": expected_support}
        if capabilities != expected:
            raise AssertionError("live authority capability contract changed")
        idle = receive_kind(socket, "state")
        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= pull_m
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))
        retained: list[dict[str, Any]] = []
        while len(retained) < frame_count:
            state = receive_kind(socket, "state")
            if state["metrics"]["authority_profile"] == "floating_dynamic_wbc_with_command_admission":
                retained.append(state)
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    distance = finite_series(retained, "collision_barrier_minimum_distance_m")
    margin = finite_series(retained, "collision_barrier_minimum_margin_m")
    relative_velocity = finite_series(retained, "collision_barrier_relative_velocity_mps")
    required = finite_series(retained, "collision_barrier_required_acceleration_mps2")
    achieved = finite_series(retained, "collision_barrier_achieved_acceleration_mps2")
    residual = finite_series(retained, "collision_barrier_residual_mps2")
    solve_us = finite_series(retained, "solve_us")
    dynamics = finite_series(retained, "dynamics_residual_linf")
    contact = finite_series(retained, "contact_residual_linf")
    command_clearance = finite_series(retained, "primary_continuous_clearance_m")
    active_counts = np.asarray(
        [frame["metrics"]["collision_barrier_active_pairs"] for frame in retained],
        dtype=np.int64,
    )
    unsupported = np.asarray(
        [frame["metrics"]["collision_barrier_unsupported_shapes"] for frame in retained],
        dtype=np.int64,
    )
    closest_pairs = [frame["metrics"]["collision_barrier_closest_pair"] for frame in retained]
    limiting_pairs = [frame["metrics"]["collision_barrier_limiting_pair"] for frame in retained]
    qualities = [frame["metrics"]["collision_barrier_quality"] for frame in retained]
    body_names = hello["body_names"]
    limiting_bodies = Counter(
        f"{body_names[frame['metrics']['collision_barrier_limiting_body_a']]}↔{body_names[frame['metrics']['collision_barrier_limiting_body_b']]}"
        for frame in retained
    )
    passed = bool(
        contract["source"].endswith(("-r108", "-r109"))
        and capabilities["self_collision_avoidance"] == "measured"
        and np.all(active_counts > 0)
        and np.all(unsupported == 0)
        and all(isinstance(pair, int) for pair in closest_pairs)
        and all(isinstance(pair, int) for pair in limiting_pairs)
        and all(isinstance(quality, str) for quality in qualities)
        and float(np.min(residual)) >= -1e-7
        and float(np.min(achieved - required)) >= -1e-7
        and float(np.max(dynamics)) < 1e-8
        and float(np.max(contact)) < 1e-8
        and distribution(solve_us)["p99"] < 5_000.0
        and float(np.min(command_clearance)) > 0.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "guided state-local floating WBC plus separate policy-/physics-free command query",
        "frames": frame_count,
        "pull_m": pull_m,
        "contract_source": contract["source"],
        "capability_count": len(capabilities),
        "minimum_distance_m": float(np.min(distance)),
        "minimum_margin_m": float(np.min(margin)),
        "minimum_active_pair_count": int(np.min(active_counts)),
        "maximum_active_pair_count": int(np.max(active_counts)),
        "closest_pair_counts": dict(Counter(str(pair) for pair in closest_pairs)),
        "limiting_pair_counts": dict(Counter(str(pair) for pair in limiting_pairs)),
        "limiting_body_counts": dict(limiting_bodies),
        "quality_counts": dict(Counter(qualities)),
        "relative_velocity_mps": distribution(relative_velocity),
        "required_acceleration_mps2": distribution(required),
        "achieved_acceleration_mps2": distribution(achieved),
        "barrier_residual_mps2": distribution(residual),
        "minimum_achieved_minus_required_mps2": float(np.min(achieved - required)),
        "maximum_unsupported_shape_count": int(np.max(unsupported)),
        "maximum_dynamics_residual": float(np.max(dynamics)),
        "maximum_contact_residual": float(np.max(contact)),
        "minimum_separate_command_clearance_m": float(np.min(command_clearance)),
        "solve_timing_us": distribution(solve_us),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    timing = audit["solve_timing_us"]
    table = "\n".join(
        markdown_table(
            ["signal", "retained value"],
            [
                ["closest clearance / hard-margin reserve", f"{audit['minimum_distance_m'] * 1e3:.3f} / {audit['minimum_margin_m'] * 1e3:.3f} mm"],
                ["active pair count", f"{audit['minimum_active_pair_count']}–{audit['maximum_active_pair_count']}"],
                ["limiting pairs", json.dumps(audit["limiting_pair_counts"], sort_keys=True)],
                ["limiting bodies", json.dumps(audit["limiting_body_counts"], sort_keys=True)],
                ["quality", json.dumps(audit["quality_counts"], sort_keys=True)],
                ["minimum barrier residual", f"{audit['barrier_residual_mps2']['minimum']:.3e} m/s²"],
                ["raw WBC p50 / p99 / max", f"{timing['p50'] / 1000:.3f} / {timing['p99'] / 1000:.3f} / {timing['maximum'] / 1000:.3f} ms"],
            ],
        )
    )
    return f"""# Bonesaw live floating collision viability · r108

## Outcome

**{audit['status'].upper()}.** The r108 WebSocket contract exposes the floating closest-feature collision barrier as its own Viability signal across {audit['frames']} guided 50 Hz frames. It does not replace the sampled/adaptive command-clearance rows and does not turn the guided pose into plant response.

{table}

All retained frames report zero unsupported shapes. The minimum achieved-minus-required normal acceleration is `{audit['minimum_achieved_minus_required_mps2']:.3e}` m/s². Maximum dynamics/contact residuals are `{audit['maximum_dynamics_residual']:.3e}` / `{audit['maximum_contact_residual']:.3e}`, and the separate command certificate retains at least `{audit['minimum_separate_command_clearance_m'] * 1e3:.3f}` mm.

## Typed authority boundary

The live row carries closest pair/body, limiting active pair/body, active count, explicit primitive quality, signed clearance and hard-margin reserve, relative normal velocity, required/achieved normal acceleration, post-solve residual, and unsupported-shape count. Support, joint stopping, effort, solver time, trajectory clearance, command selection, thermal state, and realized body response remain separate rows.

## Remaining work

The barrier is a local rigid-feature linearization and excludes closest-feature switching and normal curvature. World collision, mesh CCD, authored exclusion matrices, closest-feature continuous rate certificates, calibrated actuator realization, and measured plant response remain separate milestones.
"""


def main() -> None:
    args = parse_args()
    if args.frames <= 0 or not math.isfinite(args.pull_m) or args.pull_m <= 0.0:
        raise SystemExit("invalid live trace length or pull")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(args.url, args.frames, args.pull_m, args.expected_support)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/collision.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_wbc.rs"),
        pathlib.Path("crates/bonesaw-tools/src/bin/server.rs"),
        pathlib.Path("web/motion-rig-r16.js"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "live-collision-barrier-r108",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "live-collision-barrier-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "LIVE_COLLISION_BARRIER_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(
        render_report_html(report, title="Bonesaw live floating collision viability · r108")
    )
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
