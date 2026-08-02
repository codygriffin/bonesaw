#!/usr/bin/env python3
"""Repeated soft joint-drag limit/retreat/release recovery over the live WebSocket."""

from __future__ import annotations

import argparse
import html
import json
import math
import pathlib
import statistics
import time
from datetime import datetime, timezone
from typing import Any

from live_editor_smoke import (
    RawWebSocket,
    http_probe,
    percentile,
    receive_kind,
)


LIMIT_COVERAGE_THRESHOLD_RAD = 0.03
APPROACH_STATES_PER_UPDATE = 5
MAX_APPROACH_UPDATES = 64
RETREAT_STATES = 10
SETTLE_STATES = 6
RELEASE_COOLDOWN_STATES = 10


def finite_tree(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(finite_tree(item) for item in value)
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    return False


def assert_joint_bounds(state: dict[str, Any], limits: list[dict[str, Any]]) -> None:
    positions = state["joint_positions"]
    velocities = state["joint_velocities"]
    assert len(positions) == len(velocities)
    assert all(math.isfinite(value) for value in positions + velocities)
    for limit in limits:
        coordinate = limit["coordinate"]
        position = positions[coordinate]
        if limit["lower"] is not None:
            assert position >= limit["lower"] - 1e-10, (
                limit["name"], position, limit["lower"]
            )
        if limit["upper"] is not None:
            assert position <= limit["upper"] + 1e-10, (
                limit["name"], position, limit["upper"]
            )


def receive_states(
    websocket: RawWebSocket,
    count: int,
    limits: list[dict[str, Any]],
    reset_epoch: int,
) -> list[dict[str, Any]]:
    states = []
    for _ in range(count):
        state = receive_kind(websocket, "state", attempts=200)
        assert state["reset_epoch"] == reset_epoch, "unexpected reset during recovery replay"
        assert_joint_bounds(state, limits)
        assert finite_tree(state["metrics"]), "nonfinite live authority metric"
        if states:
            assert state["tick"] > states[-1]["tick"], "state stream stopped or rewound"
        states.append(state)
    return states


def receive_command_states(
    websocket: RawWebSocket,
    command_id: int,
    count: int,
    limits: list[dict[str, Any]],
    reset_epoch: int,
    active_frame: str | None,
) -> list[dict[str, Any]]:
    states: list[dict[str, Any]] = []
    for _ in range(400):
        state = receive_kind(websocket, "state", attempts=200)
        assert state["reset_epoch"] == reset_epoch
        assert_joint_bounds(state, limits)
        if state.get("command_id") != command_id:
            continue
        assert state.get("active_frame") == active_frame
        states.append(state)
        break
    assert states, f"command {command_id} was not acknowledged"
    if count > 1:
        states.extend(receive_states(websocket, count - 1, limits, reset_epoch))
    assert all(state.get("command_id") == command_id for state in states)
    assert all(state.get("active_frame") == active_frame for state in states)
    return states


def signed_headroom(
    state: dict[str, Any], limit: dict[str, Any], toward_upper: bool
) -> float:
    position = state["joint_positions"][limit["coordinate"]]
    bound = limit["upper"] if toward_upper else limit["lower"]
    assert bound is not None
    return (bound - position) if toward_upper else (position - bound)


def interaction_frame(joint_name: str) -> str:
    side, joint = joint_name.split("_", 1)
    if joint == "hip":
        return f"{side}_knee_qdd100_rotor"
    if joint == "knee":
        return f"{side}_ankle_mj5208_rotor"
    raise AssertionError(f"no limit interaction frame for {joint_name}")


def state_witness(state: dict[str, Any]) -> dict[str, Any]:
    metrics = state["metrics"]
    return {
        "tick": state["tick"],
        "status": state["status"],
        "joint_positions": state["joint_positions"],
        "joint_velocities": state["joint_velocities"],
        "commanded_joint_velocity": state.get("commanded_joint_velocity"),
        "intent_residual": metrics["intent_residual"],
        "minimum_joint_margin_rad": metrics.get("minimum_joint_margin_rad"),
        "limiting_joint": metrics.get("limiting_joint"),
        "joint_position_headroom_rad": metrics.get("joint_position_headroom_rad", []),
        "minimum_joint_velocity_stopping_headroom_rad": metrics.get(
            "minimum_joint_velocity_stopping_headroom_rad"
        ),
        "limiting_joint_velocity_stopping": metrics.get(
            "limiting_joint_velocity_stopping"
        ),
        "joint_velocity_stopping_headroom_rad": metrics.get(
            "joint_velocity_stopping_headroom_rad", []
        ),
        "trajectory_velocity_scale": metrics.get("trajectory_velocity_scale"),
        "trajectory_backtrack_steps": metrics.get("trajectory_backtrack_steps"),
        "active_constraint_rows": metrics.get("active_constraint_rows", []),
        "clipped_levels": metrics.get("clipped_levels", []),
        "task_residuals": metrics.get("task_residuals", []),
    }


def run(base_url: str, cycles: int, connect_address: str | None = None) -> dict[str, Any]:
    http_probe(base_url, connect_address)
    websocket = RawWebSocket.connect(base_url, connect_address)
    try:
        hello = receive_kind(websocket, "hello")
        limits = [
            item
            for item in hello["joint_limits"]
            if item["lower"] is not None and item["upper"] is not None
        ]
        assert limits, "no finite joint limits"
        handle_names = {
            item["frame"]
            for item in hello["interaction_handles"]
            if item["kind"] == "joint"
        }
        for limit in limits:
            assert interaction_frame(limit["name"]) in handle_names
        initial = receive_kind(websocket, "state")
        reset_epoch = initial["reset_epoch"]
        rows: list[dict[str, Any]] = []
        recovery_frames: list[int] = []
        minimum_margin = math.inf
        maximum_residual = 0.0
        maximum_observed_speed = 0.0
        command_request_id = 0
        cases = [(limit, side) for limit in limits for side in ("lower", "upper")]
        coverage = {
            limit["name"]: {
                side: {
                    "cycles": 0,
                    "minimum_headroom_rad": math.inf,
                    "maximum_recovery_frames": 0,
                }
                for side in ("lower", "upper")
            }
            for limit in limits
        }

        def check_sequence(states: list[dict[str, Any]]) -> None:
            nonlocal maximum_observed_speed
            for state in states:
                assert state["reset_epoch"] == reset_epoch
                assert_joint_bounds(state, limits)
                assert finite_tree(state["metrics"])
                stopping = state["metrics"].get(
                    "minimum_joint_velocity_stopping_headroom_rad"
                )
                assert stopping is not None and math.isfinite(stopping)
            for before, after in zip(states, states[1:]):
                tick_delta = after["tick"] - before["tick"]
                assert tick_delta > 0
                for limit in limits:
                    coordinate = limit["coordinate"]
                    speed = abs(
                        after["joint_positions"][coordinate]
                        - before["joint_positions"][coordinate]
                    ) / (0.02 * tick_delta)
                    maximum_observed_speed = max(maximum_observed_speed, speed)
                    assert speed <= limit["velocity"] + 1e-8, (
                        limit["name"],
                        speed,
                        limit["velocity"],
                    )

        for cycle in range(cycles):
            limit, side = cases[cycle % len(cases)]
            coordinate = limit["coordinate"]
            toward_upper = side == "upper"
            direction = 1.0 if toward_upper else -1.0
            frame = interaction_frame(limit["name"])
            before = receive_kind(websocket, "state")
            assert_joint_bounds(before, limits)
            outward: list[dict[str, Any]] = []
            approach_updates = 0
            for approach_updates in range(1, MAX_APPROACH_UPDATES + 1):
                command_request_id += 1
                websocket.send_json(
                    {
                        "type": "joint_drag",
                        "request_id": command_request_id,
                        "frame": frame,
                        "coordinate": coordinate,
                        "target_position": (
                            limit["upper"] + 0.5
                            if toward_upper
                            else limit["lower"] - 0.5
                        ),
                    }
                )
                batch = receive_command_states(
                    websocket,
                    command_request_id,
                    APPROACH_STATES_PER_UPDATE,
                    limits,
                    reset_epoch,
                    frame,
                )
                assert all(state.get("active_frame") == frame for state in batch)
                outward.extend(batch)
                if signed_headroom(batch[-1], limit, toward_upper) <= LIMIT_COVERAGE_THRESHOLD_RAD:
                    break
            check_sequence(outward)
            boundary = outward[-1]
            headroom_before = signed_headroom(boundary, limit, toward_upper)
            assert headroom_before <= LIMIT_COVERAGE_THRESHOLD_RAD, (
                f"cycle {cycle}: {limit['name']} {side} stopped "
                f"{headroom_before:.6g} rad from the limit; "
                f"before={state_witness(before)}; boundary={state_witness(boundary)}; "
                f"statuses={sorted({state['status'] for state in outward})}"
            )
            position_headrooms = boundary["metrics"].get(
                "joint_position_headroom_rad", []
            )
            assert len(position_headrooms) == len(boundary["joint_positions"])
            targeted_position_headroom = position_headrooms[coordinate]
            assert targeted_position_headroom is not None
            assert abs(targeted_position_headroom - headroom_before) <= 1e-10
            stopping_headrooms = boundary["metrics"].get(
                "joint_velocity_stopping_headroom_rad", []
            )
            assert len(stopping_headrooms) == len(boundary["joint_positions"])
            targeted_stopping_headroom = stopping_headrooms[coordinate]
            assert targeted_stopping_headroom is not None
            assert -1e-10 <= targeted_stopping_headroom <= LIMIT_COVERAGE_THRESHOLD_RAD
            outward_margin = min(
                state["metrics"]["minimum_joint_margin_rad"] for state in outward
            )
            minimum_margin = min(minimum_margin, outward_margin)
            outward_residual = max(
                state["metrics"]["intent_residual"] for state in outward
            )
            maximum_residual = max(maximum_residual, outward_residual)

            command_request_id += 1
            websocket.send_json(
                {
                    "type": "joint_drag",
                    "request_id": command_request_id,
                    "frame": frame,
                    "coordinate": coordinate,
                    "target_position": (
                        limit["upper"] - 0.5
                        if toward_upper
                        else limit["lower"] + 0.5
                    ),
                }
            )
            retreat = receive_command_states(
                websocket,
                command_request_id,
                RETREAT_STATES,
                limits,
                reset_epoch,
                frame,
            )
            check_sequence(retreat)
            assert all(state.get("active_frame") == frame for state in retreat)
            recovered_at = next(
                (
                    index
                    for index, state in enumerate(retreat, start=1)
                    if signed_headroom(state, limit, toward_upper)
                    > headroom_before + 1e-4
                ),
                None,
            )
            assert recovered_at is not None, (
                f"cycle {cycle}: no motion away from {limit['name']} {side}; "
                f"boundary={state_witness(boundary)}, "
                f"retreat={[state_witness(state) for state in retreat]}"
            )
            recovery_frames.append(recovered_at)
            headroom_after = signed_headroom(retreat[recovered_at - 1], limit, toward_upper)

            settled_target = retreat[-1]["joint_positions"][coordinate]
            command_request_id += 1
            websocket.send_json(
                {
                    "type": "joint_drag",
                    "request_id": command_request_id,
                    "frame": frame,
                    "coordinate": coordinate,
                    "target_position": settled_target,
                }
            )
            settled = receive_command_states(
                websocket,
                command_request_id,
                SETTLE_STATES,
                limits,
                reset_epoch,
                frame,
            )
            check_sequence(settled)
            settled_residual = min(
                state["metrics"]["intent_residual"] for state in settled
            )
            assert settled_residual < outward_residual, (
                f"cycle {cycle}: intent residual did not fall after retreat: "
                f"{outward_residual} -> {settled_residual}"
            )

            command_request_id += 1
            websocket.send_json({"type": "release", "request_id": command_request_id})
            released = receive_command_states(
                websocket,
                command_request_id,
                1,
                limits,
                reset_epoch,
                None,
            )[0]
            command_request_id += 1
            websocket.send_json({"type": "release", "request_id": command_request_id})
            released_again = receive_command_states(
                websocket,
                command_request_id,
                RELEASE_COOLDOWN_STATES,
                limits,
                reset_epoch,
                None,
            )[-1]
            assert released_again["tick"] > released["tick"]
            released = released_again
            assert released["metrics"]["intent_residual"] <= outward_residual
            case_coverage = coverage[limit["name"]][side]
            case_coverage["cycles"] += 1
            case_coverage["minimum_headroom_rad"] = min(
                case_coverage["minimum_headroom_rad"], headroom_before
            )
            case_coverage["maximum_recovery_frames"] = max(
                case_coverage["maximum_recovery_frames"], recovered_at
            )
            rows.append(
                {
                    "cycle": cycle,
                    "handle": frame,
                    "joint": limit["name"],
                    "bound": side,
                    "approach_updates": approach_updates,
                    "outward_statuses": sorted({state["status"] for state in outward}),
                    "outward_margin_rad": outward_margin,
                    "outward_intent_residual": outward_residual,
                    "headroom_before_rad": headroom_before,
                    "headroom_after_rad": headroom_after,
                    "settled_intent_residual": settled_residual,
                    "recovery_frames": recovered_at,
                    "release_tick": released["tick"],
                    "transition_witnesses": {
                        "before": state_witness(before),
                        "boundary": state_witness(boundary),
                        "recovered": state_witness(retreat[recovered_at - 1]),
                        "settled": state_witness(settled[-1]),
                        "released": state_witness(released),
                    },
                }
            )
        assert all(
            entry["cycles"] > 0
            and entry["minimum_headroom_rad"] <= LIMIT_COVERAGE_THRESHOLD_RAD
            for joint in coverage.values()
            for entry in joint.values()
        )
        handle_counts = {
            frame: sum(row["handle"] == frame for row in rows)
            for frame in sorted(handle_names)
        }
        return {
            "schema_version": 2,
            "revision": "live-editor-limit-recovery-r126",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "url": base_url,
            "cycles": cycles,
            "reset_epoch": reset_epoch,
            "unexpected_resets": 0,
            "limited_joints": [item["name"] for item in limits],
            "joint_bound_coverage": coverage,
            "limit_coverage_threshold_rad": LIMIT_COVERAGE_THRESHOLD_RAD,
            "handle_counts": handle_counts,
            "minimum_joint_margin_rad": minimum_margin,
            "maximum_intent_residual": maximum_residual,
            "maximum_observed_joint_speed_rad_s": maximum_observed_speed,
            "recovery_frames": {
                "p50": statistics.median(recovery_frames),
                "p95": percentile([float(value) for value in recovery_frames], 0.95),
                "p99": percentile([float(value) for value in recovery_frames], 0.99),
                "maximum": max(recovery_frames),
            },
            "all_joint_states_finite_and_bounded": True,
            "idempotent_release_cycles": cycles,
            "acknowledged_commands": command_request_id,
            "rows": rows,
            "admission": True,
        }
    finally:
        websocket.close()


def markdown(metrics: dict[str, Any]) -> str:
    coverage_lines = []
    for joint, sides in metrics["joint_bound_coverage"].items():
        lower = sides["lower"]
        upper = sides["upper"]
        coverage_lines.append(
            f'- `{joint}`: lower {lower["cycles"]}× / '
            f'{lower["minimum_headroom_rad"]:.6g} rad; upper '
            f'{upper["cycles"]}× / {upper["minimum_headroom_rad"]:.6g} rad; '
            f'max recovery {max(lower["maximum_recovery_frames"], upper["maximum_recovery_frames"])} frame(s)'
        )
    return "\n".join(
        [
            "# Live editor joint-limit recovery · r126",
            "",
            f'> **Admission: {"PASS" if metrics["admission"] else "FAIL"}.** '
            f'{metrics["cycles"]} soft joint-drag hold/retreat/idempotent-release cycles '
            "completed without a reset, nonfinite state, hard-bound crossing, or latched drag.",
            "",
            f'- Limited joints checked: {", ".join(metrics["limited_joints"])}',
            f'- Lower/upper coverage threshold: '
            f'`≤ {metrics["limit_coverage_threshold_rad"]:.3f} rad`',
            *coverage_lines,
            f'- Handle coverage: `{metrics["handle_counts"]}`',
            f'- Minimum streamed joint headroom: `{metrics["minimum_joint_margin_rad"]:.6g} rad`',
            f'- Maximum intent residual: `{metrics["maximum_intent_residual"]:.6g}`',
            f'- Maximum represented joint speed: '
            f'`{metrics["maximum_observed_joint_speed_rad_s"]:.6g} rad/s`',
            f'- Retreat recovery frames p50/p95/p99/max: `{metrics["recovery_frames"]}`',
            f'- Idempotent releases: `{metrics["idempotent_release_cycles"]}`',
            f'- Correlated drag/settle/release acknowledgements: '
            f'`{metrics["acknowledged_commands"]}`',
            "",
            "Each row retains before/boundary/recovery/settled/released transition witnesses: "
            "solve status, active constraint rows, position and stopping margins, residuals, "
            "commanded velocity, backtracking scale/work, joint q/v, recovery latency, and "
            "release tick. The server streams joint q/v directly, so bounds are asserted on "
            "represented state rather than inferred from rendered pixels.",
            "",
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--connect-address")
    parser.add_argument(
        "--output", default="benchmarks/results/live-editor-limit-recovery-r126"
    )
    parser.add_argument(
        "--web-report", default="web/LIVE_EDITOR_LIMIT_RECOVERY_R126.html"
    )
    args = parser.parse_args()
    if args.cycles < 8:
        raise ValueError("cycles must be at least eight to cover every finite joint/bound pair")
    metrics = run(args.url.rstrip("/"), args.cycles, args.connect_address)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "live-editor-limit-recovery-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    report = markdown(metrics)
    (output / "LIVE_EDITOR_LIMIT_RECOVERY.md").write_text(report)
    pathlib.Path(args.web_report).write_text(
        "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Live editor joint-limit recovery r126</title>"
        "<style>body{max-width:900px;margin:40px auto;padding:0 18px;background:#0b0d10;"
        "color:#e8ecf2;font:16px/1.55 system-ui}pre{white-space:pre-wrap}</style>"
        f"<pre>{html.escape(report)}</pre>"
    )
    print(json.dumps({key: value for key, value in metrics.items() if key != "rows"}, indent=2))


if __name__ == "__main__":
    main()
