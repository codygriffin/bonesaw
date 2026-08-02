#!/usr/bin/env python3
"""Retain the live sampled/continuous command-authority WebSocket contract."""

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
    parser.add_argument("--url", default="ws://127.0.0.1:8798/ws")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--pull-m", type=float, default=0.08)
    parser.add_argument("--expected-support", choices=("measured", "unavailable"), default="measured")
    parser.add_argument("--output", default="benchmarks/results/live-command-authority-r106")
    parser.add_argument("--web-report", default="web/LIVE_COMMAND_AUTHORITY_R106.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table(headers, rows))


def finite_series(frames: list[dict[str, Any]], key: str) -> np.ndarray:
    values = np.asarray([frame["metrics"][key] for frame in frames], np.float64)
    if not np.all(np.isfinite(values)):
        raise AssertionError(f"non-finite streamed field: {key}")
    return values


def evaluate(url: str, frames: int, pull_m: float, expected_support: str) -> dict[str, Any]:
    socket = create_connection(url, timeout=5)
    try:
        hello = receive_kind(socket, "hello", 1)
        contract = hello["authority_contract"]
        capabilities = {
            signal["stable_id"]: signal["availability"]
            for signal in contract["signals"]
        }
        expected = {**BASE_CAPABILITIES, "finite_support": expected_support}
        idle = receive_kind(socket, "state")
        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= pull_m
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))
        retained: list[dict[str, Any]] = []
        while len(retained) < frames:
            state = receive_kind(socket, "state")
            if state["metrics"]["authority_profile"] == "floating_dynamic_wbc_with_command_admission":
                retained.append(state)
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    sampled = finite_series(retained, "primary_sampled_clearance_m")
    continuous = finite_series(retained, "primary_continuous_clearance_m")
    brake = finite_series(retained, "contingency_continuous_clearance_m")
    primary_pair_speed = finite_series(retained, "primary_continuous_relative_speed_m_s")
    contingency_pair_speed = finite_series(
        retained, "contingency_continuous_relative_speed_m_s"
    )
    primary_refinements = np.asarray(
        [frame["metrics"]["primary_refinement_pair_samples"] for frame in retained],
        np.int64,
    )
    primary_unresolved = np.asarray(
        [frame["metrics"]["primary_continuity_unresolved_intervals"] for frame in retained],
        np.int64,
    )
    primary_leaves = np.asarray(
        [frame["metrics"]["primary_continuity_leaf_intervals"] for frame in retained],
        np.int64,
    )
    primary_depth = np.asarray(
        [frame["metrics"]["primary_continuity_maximum_subdivision_depth"] for frame in retained],
        np.int64,
    )
    raw_solve = finite_series(retained, "solve_us")
    command_query = finite_series(retained, "command_admission_us")
    command_batch = finite_series(retained, "command_admission_batch_us")
    dynamics = finite_series(retained, "dynamics_residual_linf")
    contacts = finite_series(retained, "contact_residual_linf")
    flags = [int(frame["metrics"]["command_admission_flags"]) for frame in retained]
    selections = [frame["metrics"]["command_selection"] for frame in retained]
    first_witnesses = [
        (
            frame["metrics"]["primary_first_collision_pair"],
            frame["metrics"]["primary_first_collision_time_ns"],
            frame["metrics"]["primary_first_collision_body_a"],
            frame["metrics"]["primary_first_collision_body_b"],
        )
        for frame in retained
    ]
    first_pairs = [pair for pair, _, _, _ in first_witnesses if pair is not None]
    minimum_pairs = [
        frame["metrics"]["primary_minimum_collision_pair"] for frame in retained
    ]
    minimum_body_pairs = [
        (
            hello["body_names"][frame["metrics"]["primary_minimum_collision_body_a"]],
            hello["body_names"][frame["metrics"]["primary_minimum_collision_body_b"]],
        )
        for frame in retained
    ]
    continuous_pairs = [
        frame["metrics"]["primary_continuous_limiting_pair"] for frame in retained
    ]
    continuous_body_pairs = [
        (
            hello["body_names"][frame["metrics"]["primary_continuous_limiting_body_a"]],
            hello["body_names"][frame["metrics"]["primary_continuous_limiting_body_b"]],
        )
        for frame in retained
    ]
    first_body_pairs = [
        (
            hello["body_names"][body_a],
            hello["body_names"][body_b],
        )
        for _, _, body_a, body_b in first_witnesses
        if body_a is not None and body_b is not None
    ]
    first_times = [time_ns for _, time_ns, _, _ in first_witnesses if time_ns is not None]
    requirement = float(retained[0]["metrics"]["command_clearance_requirement_m"])
    passed = bool(
        contract["schema"] == 1
        and capabilities == expected
        and len(capabilities) == len(contract["signals"]) == 13
        and "health" not in capabilities
        and idle["metrics"]["command_selection"] is None
        and all(selection in {"primary", "contingency", "rejected"} for selection in selections)
        and all(
            (pair is None and time_ns is None and body_a is None and body_b is None)
            or all(isinstance(value, int) for value in (pair, time_ns, body_a, body_b))
            for pair, time_ns, body_a, body_b in first_witnesses
        )
        and all(isinstance(pair, int) for pair in minimum_pairs)
        and all(isinstance(pair, int) for pair in continuous_pairs)
        and np.all(primary_refinements >= 0)
        and np.all(primary_unresolved >= 0)
        and np.all(primary_leaves >= 0)
        and np.all((primary_depth >= 0) & (primary_depth <= 3))
        and all(all(isinstance(body, str) for body in pair) for pair in first_body_pairs)
        and all(isinstance(time_ns, int) for time_ns in first_times)
        and np.max(dynamics) < 1e-7
        and np.max(contacts) < 1e-7
        and np.max(raw_solve) < 5_000.0
        and np.max(command_query) < 5_000.0
        and np.max(command_batch) < 20_000.0
    )
    return {
        "status": "pass" if passed else "fail",
        "scope": "live WebSocket authority telemetry; guided pose and command admission remain separate",
        "frames": frames,
        "pull_m": pull_m,
        "contract_source": contract["source"],
        "capabilities": capabilities,
        "selection_counts": dict(Counter(selections)),
        "flag_counts": {f"0x{flag:03x}": count for flag, count in sorted(Counter(flags).items())},
        "clearance_requirement_m": requirement,
        "primary_sampled_clearance_m": distribution(sampled),
        "primary_continuous_clearance_m": distribution(continuous),
        "contingency_continuous_clearance_m": distribution(brake),
        "primary_continuous_pair_speed_m_s": distribution(primary_pair_speed),
        "contingency_continuous_pair_speed_m_s": distribution(contingency_pair_speed),
        "primary_refinement_pair_samples": distribution(primary_refinements.astype(np.float64)),
        "primary_unresolved_intervals": distribution(primary_unresolved.astype(np.float64)),
        "primary_leaf_intervals": distribution(primary_leaves.astype(np.float64)),
        "primary_subdivision_depth_counts": dict(Counter(map(int, primary_depth))),
        "first_collision_pairs": dict(Counter(first_pairs)),
        "clear_sample_frames": sum(pair is None for pair, _, _, _ in first_witnesses),
        "minimum_collision_pairs": dict(Counter(minimum_pairs)),
        "minimum_collision_body_pairs": {
            " ↔ ".join(pair): count for pair, count in Counter(minimum_body_pairs).items()
        },
        "continuous_limiting_pairs": dict(Counter(continuous_pairs)),
        "continuous_limiting_body_pairs": {
            " ↔ ".join(pair): count for pair, count in Counter(continuous_body_pairs).items()
        },
        "first_collision_body_pairs": {
            " ↔ ".join(pair): count for pair, count in Counter(first_body_pairs).items()
        },
        "first_collision_times_ns": dict(Counter(first_times)),
        "raw_wbc_query_us": distribution(raw_solve),
        "command_admission_query_us": distribution(command_query),
        "command_admission_four_query_batch_us": distribution(command_batch),
        "maximum_dynamics_residual": float(np.max(dynamics)),
        "maximum_contact_residual": float(np.max(contacts)),
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    rows = [
        ["primary sampled primitive clearance", audit["primary_sampled_clearance_m"]["minimum"] * 1e3, "direct 21-point evidence"],
        ["primary continuous lower bound", audit["primary_continuous_clearance_m"]["minimum"] * 1e3, "conservative half-interval guard"],
        ["contingency continuous lower bound", audit["contingency_continuous_clearance_m"]["minimum"] * 1e3, "independent brake certificate"],
        ["primary limiting-pair speed", audit["primary_continuous_pair_speed_m_s"]["maximum"], "m/s conservative maximum"],
        ["primary midpoint pair queries", audit["primary_refinement_pair_samples"]["maximum"], "maximum bounded refinement work"],
        ["primary unresolved leaves", audit["primary_unresolved_intervals"]["maximum"], "maximum after depth-3 budget"],
        ["raw WBC max query", audit["raw_wbc_query_us"]["maximum"], "µs; separate 5 ms clock"],
        ["command-admission max query", audit["command_admission_query_us"]["maximum"], "µs; separate 5 ms clock"],
        ["four command-query batch", audit["command_admission_four_query_batch_us"]["maximum"], "µs; separate 20 ms clock"],
    ]
    return f"""# Bonesaw adaptive command-authority stream · r106

## Outcome

**{audit['status'].upper()}.** The 13-signal WebSocket authority contract keeps base sampled geometry, bounded adaptive continuity, and final command selection independent. Pairs already certified by the cheap R105 bound skip interval work; only globally unresolved pairs evaluate analytic subinterval velocity extrema and deterministic midpoint primitive samples, to a fixed maximum depth of three.

The retained {audit['frames']}-frame, {audit['pull_m'] * 1e3:.0f} mm torso-pull trace uses the guided browser pose while running a parallel policy-/physics-free command-admission query. The primary is admitted on {audit['selection_counts'].get('primary', 0)} frames, the independently valid braking contingency is selected on {audit['selection_counts'].get('contingency', 0)}, and {audit['selection_counts'].get('rejected', 0)} frames reject both; no frame is labelled realized plant motion.

{table(['signal', 'retained minimum/max', 'interpretation'], rows)}

Selections: `{audit['selection_counts']}`. Flags: `{audit['flag_counts']}`. Minimum sampled pairs: `{audit['minimum_collision_pairs']}` / `{audit['minimum_collision_body_pairs']}`. Continuous limiting pairs: `{audit['continuous_limiting_pairs']}` / `{audit['continuous_limiting_body_pairs']}`; pair-speed range `{audit['primary_continuous_pair_speed_m_s']['minimum']:.3f}–{audit['primary_continuous_pair_speed_m_s']['maximum']:.3f}` m/s. Refinement maximum/mean: `{audit['primary_refinement_pair_samples']['maximum']:.0f}` / `{audit['primary_refinement_pair_samples']['mean']:.2f}` midpoint pair queries; unresolved-leaf maximum: `{audit['primary_unresolved_intervals']['maximum']:.0f}`; reached-depth counts: `{audit['primary_subdivision_depth_counts']}`. First violating pairs: `{audit['first_collision_pairs']}` / `{audit['first_collision_body_pairs']}`; first times: `{audit['first_collision_times_ns']}`. Maximum dynamics/contact residuals remain `{audit['maximum_dynamics_residual']:.3e}` / `{audit['maximum_contact_residual']:.3e}`.

## Architectural finding

R105 made the broad certificate pair-consistent but its less-pessimistic selections changed persistent command state and exposed later sampled violations. R106 preserves those sampled failures as hard evidence: midpoint refinement may add a real first-violation pair/time, but can never erase one. On clear-but-uncertain intervals, subdivision spends bounded work to distinguish certified clearance from unresolved reserve. The represented sampled minimum is {audit['primary_sampled_clearance_m']['minimum'] * 1e3:.3f} mm; {audit['clear_sample_frames']} frames are clear and {audit['frames'] - audit['clear_sample_frames']} expose a sampled violation.

The adaptive continuous lower bound reaches {audit['primary_continuous_clearance_m']['minimum'] * 1e3:.3f} mm against the 20 mm requirement, while the braking certificate reaches {audit['contingency_continuous_clearance_m']['minimum'] * 1e3:.3f} mm. The browser retains 120 ticks and now annotates current refinement work and unresolved leaves. Closest-feature rate bounds, mesh CCD, world collision, calibrated actuation, and plant response remain unavailable.
"""


def main() -> None:
    args = parse_args()
    if args.frames <= 1 or not math.isfinite(args.pull_m) or args.pull_m <= 0.0:
        raise SystemExit("invalid frames or pull distance")
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(args.url, args.frames, args.pull_m, args.expected_support)
    sources = [
        pathlib.Path("crates/bonesaw-tools/src/bin/server.rs"),
        pathlib.Path("web/index.html"),
        pathlib.Path("web/motion-rig-r16.js"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "adaptive-command-authority-r106",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in sources},
        "audit": audit,
    }
    (output / "live-command-authority-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "LIVE_COMMAND_AUTHORITY_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw adaptive command-authority stream · r106"))
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
