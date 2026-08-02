#!/usr/bin/env python3
"""Audit conservative observation-error propagation through the live Rust WBC."""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any

from websocket import create_connection

from live_observation_transport_report import (
    collect_mode_states,
    receive,
    receive_kind,
    set_mode,
    timing_summary,
)


ERROR_FIELDS = {
    "joint_position_rad": "robot_observation_joint_position_error_rad",
    "joint_velocity_rad_s": "robot_observation_joint_velocity_error_rad_s",
    "root_translation_m": "robot_observation_root_translation_error_m",
    "root_rotation_rad": "robot_observation_root_rotation_error_rad",
    "point_position_m": "robot_observation_point_position_error_m",
    "center_of_mass_position_m": "robot_observation_center_of_mass_position_error_m",
}

MARGIN_PAIRS = {
    "joint_position_rad": (
        "raw_minimum_joint_margin_rad",
        "minimum_joint_margin_rad",
        1.0,
    ),
    "support_m": (
        "raw_minimum_support_margin_m",
        "minimum_support_margin_m",
        1.0,
    ),
    "self_collision_m": (
        "raw_collision_barrier_minimum_margin_m",
        "collision_barrier_minimum_margin_m",
        2.0,
    ),
    "world_collision_m": (
        "raw_world_collision_minimum_margin_m",
        "world_collision_minimum_margin_m",
        1.0,
    ),
    "joint_stopping_rad_s2": (
        "raw_minimum_joint_stopping_margin_rad_s2",
        "minimum_joint_stopping_margin_rad_s2",
        None,
    ),
}

EXPECTED = {
    "exact": {
        "exposure_ns": 0,
        "joint_position_rad": 0.0,
        "joint_velocity_rad_s": 0.0,
        "root_translation_m": 0.0,
        "root_rotation_rad": 0.0,
        "point_position_m": 0.0,
        "center_of_mass_position_m": 0.0,
    },
    "interpolated": {
        "exposure_ns": 2_500_000,
        "joint_position_rad": 0.00021875,
        "joint_velocity_rad_s": 0.015,
        "root_translation_m": 0.0000775,
        "root_rotation_rad": 0.000103125,
        "point_position_m": 0.000128125,
        "center_of_mass_position_m": 0.0000765625,
    },
    "predicted": {
        "exposure_ns": 5_000_000,
        "joint_position_rad": 0.000475,
        "joint_velocity_rad_s": 0.03,
        "root_translation_m": 0.00016,
        "root_rotation_rad": 0.0002125,
        "point_position_m": 0.0002625,
        "center_of_mass_position_m": 0.00015625,
    },
}


def close(left: float, right: float, tolerance: float = 1e-12) -> bool:
    return math.isclose(left, right, rel_tol=0.0, abs_tol=tolerance)


def assert_uncertainty_mode(mode: str, states: list[dict[str, Any]]) -> None:
    expected = EXPECTED[mode]
    for state in states:
        metrics = state["metrics"]
        assert metrics["robot_observation_error_exposure_ns"] == expected["exposure_ns"]
        assert metrics["robot_observation_reconstruction_hard_eligible"] is True
        for name, field in ERROR_FIELDS.items():
            assert close(metrics[field], expected[name]), (mode, field, metrics[field])

        q_erosion = (
            metrics["raw_minimum_joint_margin_rad"]
            - metrics["minimum_joint_margin_rad"]
        )
        support_erosion = (
            metrics["raw_minimum_support_margin_m"]
            - metrics["minimum_support_margin_m"]
        )
        self_erosion = (
            metrics["raw_collision_barrier_minimum_margin_m"]
            - metrics["collision_barrier_minimum_margin_m"]
        )
        world_erosion = (
            metrics["raw_world_collision_minimum_margin_m"]
            - metrics["world_collision_minimum_margin_m"]
        )
        stopping_erosion = (
            metrics["raw_minimum_joint_stopping_margin_rad_s2"]
            - metrics["minimum_joint_stopping_margin_rad_s2"]
        )
        assert close(q_erosion, expected["joint_position_rad"])
        assert close(support_erosion, expected["center_of_mass_position_m"])
        assert close(self_erosion, 2.0 * expected["point_position_m"])
        assert close(world_erosion, expected["point_position_m"])
        assert stopping_erosion >= -1e-10
        if mode == "exact":
            assert close(stopping_erosion, 0.0)


def summarize_mode(states: list[dict[str, Any]]) -> dict[str, Any]:
    first = states[0]["metrics"]
    erosions: dict[str, dict[str, float]] = {}
    for name, (raw_field, robust_field, _) in MARGIN_PAIRS.items():
        values = [
            state["metrics"][raw_field] - state["metrics"][robust_field]
            for state in states
        ]
        erosions[name] = {"minimum": min(values), "maximum": max(values)}
    return {
        "frames": len(states),
        "provenance": first["robot_observation_reconstruction_provenance"],
        "exposure_ns": first["robot_observation_error_exposure_ns"],
        "error_bound": {name: first[field] for name, field in ERROR_FIELDS.items()},
        "margin_erosion": erosions,
        "selection_counts": {
            selection: sum(
                state["metrics"]["command_selection"] == selection for state in states
            )
            for selection in ("primary", "contingency", "rejected")
        },
        "solve": timing_summary(states, "solve_us"),
        "command_admission": timing_summary(states, "command_admission_us"),
        "minimum_robust_margins": {
            name: min(state["metrics"][robust] for state in states)
            for name, (_, robust, _) in MARGIN_PAIRS.items()
        },
    }


def markdown_report(audit: dict[str, Any]) -> str:
    rows = []
    for mode in ("exact", "interpolated", "predicted"):
        item = audit["modes"][mode]
        error = item["error_bound"]
        erosion = item["margin_erosion"]
        rows.append(
            f"| {mode} | {item['exposure_ns'] / 1e6:.3f} | "
            f"{error['joint_position_rad'] * 1e3:.6f} / {error['joint_velocity_rad_s']:.6f} | "
            f"{error['point_position_m'] * 1e3:.6f} / {error['center_of_mass_position_m'] * 1e3:.6f} | "
            f"{erosion['self_collision_m']['minimum'] * 1e3:.6f} / {erosion['world_collision_m']['minimum'] * 1e3:.6f} / {erosion['support_m']['minimum'] * 1e3:.6f} | "
            f"{erosion['joint_stopping_rad_s2']['minimum']:.6f}–{erosion['joint_stopping_rad_s2']['maximum']:.6f} | "
            f"`{json.dumps(item['selection_counts'], sort_keys=True)}` |"
        )
    return f"""# Bonesaw live observation uncertainty authority · r119

**PASS.** A caller-authored deterministic error-growth envelope now propagates
from canonical reconstruction into the actual Rust WBC hard rows. The eval uses
the real WebSocket/WBC path and guided state without a policy or physics rollout.
Raw geometric/state margins remain visible beside robust margins; no aggregate
health score or probabilistic confidence claim is introduced.

| mode | exposure ms | q / v error (mrad / rad/s) | point / CoM error (mm) | self / world / support erosion (mm) | stopping erosion rad/s² | selection |
|---|---:|---:|---:|---:|---:|---|
{chr(10).join(rows)}

## Propagation contract

- Joint stopping intersects the safe acceleration interval over all four
  corners of the `q ± error`, `v ± error` box.
- Finite-support rows add the CoM-position error radius to the authored erosion.
- Self-collision rows add two represented-point radii; world-SDF rows add one.
- Exact reconstruction has zero configured exposure and bit-identical raw and
  robust margins. Interpolation uses distance to the nearest bracket endpoint.
  Prediction grows monotonically from the newest source sample.
- Held or stale evidence remains ineligible and never reaches these hard rows.

## Retained audit

```json
{json.dumps(audit, indent=2, sort_keys=True)}
```

The configured growth numbers are conservative editor contracts, not calibrated
estimator covariance, confidence intervals, network statistics, or hardware
safety certification. Command-trajectory collision admission retains its
separate root-prediction/error witness and is not relabeled as plant response.
"""


def html_report(report: str, audit: dict[str, Any]) -> str:
    rows = "".join(
        f"<tr><td>{mode}</td><td>{item['exposure_ns'] / 1e6:.3f}</td>"
        f"<td>{item['error_bound']['joint_position_rad'] * 1e3:.6f}</td>"
        f"<td>{item['error_bound']['joint_velocity_rad_s']:.6f}</td>"
        f"<td>{item['error_bound']['point_position_m'] * 1e3:.6f}</td>"
        f"<td>{item['margin_erosion']['self_collision_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{item['margin_erosion']['world_collision_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{item['margin_erosion']['support_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{html.escape(json.dumps(item['selection_counts'], sort_keys=True))}</td></tr>"
        for mode, item in audit["modes"].items()
    )
    return f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Bonesaw observation uncertainty authority · r119</title><style>
:root{{color-scheme:dark;font-family:ui-monospace,SFMono-Regular,Consolas,monospace}}body{{margin:0;background:#0b0d10;color:#dce6e1}}main{{max-width:1100px;margin:auto;padding:28px 18px 60px}}h1{{font:600 24px system-ui;color:#eff8f3}}h2{{color:#8fe6c0}}.pass{{padding:13px;border-left:3px solid #8fe6c0;background:#14231d;color:#bdebd7}}.table{{overflow-x:auto}}table{{width:100%;min-width:900px;border-collapse:collapse;font-size:12px}}th,td{{padding:9px;border:1px solid #2a3039;text-align:right}}th:first-child,td:first-child{{text-align:left}}th{{color:#8fe6c0;background:#151a1d}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;padding:16px;border:1px solid #2a3039;background:#11151a;line-height:1.5}}@media(max-width:600px){{main{{padding:18px 10px 40px}}}}
</style></head><body><main><h1>Bonesaw live observation uncertainty authority · r119</h1><p class="pass">PASS · exact, interpolation, and prediction error bounds erode real hard margins deterministically.</p><div class="table"><table><thead><tr><th>mode</th><th>exposure ms</th><th>q error mrad</th><th>v error rad/s</th><th>point error mm</th><th>self erosion mm</th><th>world erosion mm</th><th>support erosion mm</th><th>selection</th></tr></thead><tbody>{rows}</tbody></table></div><h2>Complete retained audit</h2><pre>{html.escape(json.dumps(audit, indent=2, sort_keys=True))}</pre><h2>Method</h2><pre>{html.escape(report.split('## Retained audit', 1)[0])}</pre></main></body></html>"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8819/ws")
    parser.add_argument("--frames-per-mode", type=int, default=20)
    parser.add_argument(
        "--output", default="benchmarks/results/live-observation-uncertainty-r119"
    )
    parser.add_argument(
        "--web-report", default="web/LIVE_OBSERVATION_UNCERTAINTY_R119.html"
    )
    args = parser.parse_args()
    if args.frames_per_mode < 2:
        raise ValueError("--frames-per-mode must be at least two")

    socket = create_connection(args.url, timeout=10)
    try:
        hello = receive_kind(socket, "hello", 1)
        assert hello["squat_execution"] == "guided_preview"
        assert hello["authority_contract"]["source"].endswith(("-r119", "-r121"))
        idle = receive_kind(socket, "state")
        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= 0.06
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))

        summaries: dict[str, Any] = {}
        for mode in ("exact", "interpolated", "predicted"):
            set_mode(socket, mode)
            states = collect_mode_states(socket, mode, args.frames_per_mode)
            assert_uncertainty_mode(mode, states)
            summaries[mode] = summarize_mode(states)

        set_mode(socket, "stale")
        stale = None
        for _ in range(100):
            message = receive(socket)
            if message.get("type") == "observation_withheld":
                stale = message
                break
        assert stale is not None and stale["transport_mode"] == "stale"
        set_mode(socket, "exact")
        recovery = collect_mode_states(socket, "exact", 1)[0]
        assert recovery["tick"] > stale["tick"]
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    assert summaries["exact"]["error_bound"]["point_position_m"] == 0.0
    assert (
        summaries["interpolated"]["error_bound"]["point_position_m"]
        < summaries["predicted"]["error_bound"]["point_position_m"]
    )
    audit = {
        "schema": 1,
        "revision": "live-observation-uncertainty-r119",
        "status": "pass",
        "url": args.url,
        "execution": "guided_preview_without_policy_or_physics_rollout",
        "contract_source": hello["authority_contract"]["source"],
        "program_fingerprint": hello["program_fingerprint"],
        "modes": summaries,
        "stale_withheld": stale,
        "recovery_tick": recovery["tick"],
    }
    report = markdown_report(audit)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "audit.json").write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n")
    (output / "LIVE_OBSERVATION_UNCERTAINTY_AUDIT.md").write_text(report)
    web_report = Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(html_report(report, audit))
    print(json.dumps(audit, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
