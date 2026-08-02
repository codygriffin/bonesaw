#!/usr/bin/env python3
"""Audit reconstruction uncertainty at primary/brake command admission."""

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
from live_observation_uncertainty_report import ERROR_FIELDS, EXPECTED, close


COMMAND_MARGIN_PAIRS = {
    "self_sampled_m": (
        "primary_sampled_clearance_m",
        "primary_robust_sampled_clearance_m",
        2.0,
    ),
    "self_continuous_m": (
        "primary_continuous_clearance_m",
        "primary_robust_continuous_clearance_m",
        2.0,
    ),
    "world_sampled_m": (
        "primary_world_sampled_clearance_m",
        "primary_world_robust_sampled_clearance_m",
        1.0,
    ),
    "world_continuous_m": (
        "primary_world_continuous_clearance_m",
        "primary_world_robust_continuous_clearance_m",
        1.0,
    ),
    "brake_self_sampled_m": (
        "contingency_sampled_clearance_m",
        "contingency_robust_sampled_clearance_m",
        2.0,
    ),
    "brake_self_continuous_m": (
        "contingency_continuous_clearance_m",
        "contingency_robust_continuous_clearance_m",
        2.0,
    ),
    "brake_world_sampled_m": (
        "contingency_world_sampled_clearance_m",
        "contingency_world_robust_sampled_clearance_m",
        1.0,
    ),
    "brake_world_continuous_m": (
        "contingency_world_continuous_clearance_m",
        "contingency_world_robust_continuous_clearance_m",
        1.0,
    ),
}


def assert_command_uncertainty_mode(
    mode: str,
    states: list[dict[str, Any]],
) -> None:
    expected = EXPECTED[mode]
    for state in states:
        metrics = state["metrics"]
        assert metrics["robot_observation_error_exposure_ns"] == expected["exposure_ns"]
        assert metrics["robot_observation_reconstruction_hard_eligible"] is True
        assert (
            metrics["robot_observation_transport_query_time_ns"]
            == metrics["robot_observation_mapped_time_ns"]
        )
        assert metrics["command_tracking_action"] in {
            "nominal",
            "contingency",
            "rejected",
        }
        for name, field in ERROR_FIELDS.items():
            assert close(metrics[field], expected[name]), (mode, field, metrics[field])
        for name, (raw_field, robust_field, multiplier) in COMMAND_MARGIN_PAIRS.items():
            raw = metrics[raw_field]
            robust = metrics[robust_field]
            assert math.isfinite(raw), (mode, name, raw)
            assert math.isfinite(robust), (mode, name, robust)
            erosion = raw - robust
            assert close(
                erosion,
                multiplier * expected["point_position_m"],
                tolerance=2e-12,
            ), (mode, name, erosion)


def summarize_mode(states: list[dict[str, Any]]) -> dict[str, Any]:
    first = states[0]["metrics"]
    erosions = {}
    minima = {}
    for name, (raw_field, robust_field, _) in COMMAND_MARGIN_PAIRS.items():
        losses = [
            state["metrics"][raw_field] - state["metrics"][robust_field]
            for state in states
        ]
        erosions[name] = {"minimum": min(losses), "maximum": max(losses)}
        minima[name] = min(state["metrics"][robust_field] for state in states)
    return {
        "frames": len(states),
        "provenance": first["robot_observation_reconstruction_provenance"],
        "exposure_ns": first["robot_observation_error_exposure_ns"],
        "error_bound": {name: first[field] for name, field in ERROR_FIELDS.items()},
        "command_margin_erosion": erosions,
        "minimum_robust_command_margin": minima,
        "selection_counts": {
            selection: sum(
                state["metrics"]["command_selection"] == selection for state in states
            )
            for selection in ("primary", "contingency", "rejected")
        },
        "tracking_action_counts": {
            action: sum(
                state["metrics"]["command_tracking_action"] == action for state in states
            )
            for action in ("nominal", "contingency", "rejected")
        },
        "root_prediction_error": {
            "translation_radius_m": first[
                "primary_root_prediction_translation_error_radius_m"
            ],
            "rotation_radius_rad": first[
                "primary_root_prediction_rotation_error_radius_rad"
            ],
            "maximum_world_clearance_erosion_m": first[
                "primary_world_prediction_clearance_erosion_m"
            ],
        },
        "command_admission": timing_summary(states, "command_admission_us"),
        "raw_wbc": timing_summary(states, "solve_us"),
    }


def markdown_report(audit: dict[str, Any]) -> str:
    rows = []
    for mode in ("exact", "interpolated", "predicted"):
        item = audit["modes"][mode]
        loss = item["command_margin_erosion"]
        root = item["root_prediction_error"]
        rows.append(
            f"| {mode} | {item['exposure_ns'] / 1e6:.3f} | "
            f"{item['error_bound']['point_position_m'] * 1e3:.6f} | "
            f"{loss['self_sampled_m']['minimum'] * 1e3:.6f} / "
            f"{loss['self_continuous_m']['minimum'] * 1e3:.6f} | "
            f"{loss['world_sampled_m']['minimum'] * 1e3:.6f} / "
            f"{loss['world_continuous_m']['minimum'] * 1e3:.6f} | "
            f"{root['translation_radius_m'] * 1e3:.6f} / "
            f"{root['rotation_radius_rad'] * 1e3:.6f} | "
            f"`{json.dumps(item['selection_counts'], sort_keys=True)}` |"
        )
    return f"""# Bonesaw command reconstruction-uncertainty authority · r121

**PASS.** Canonical reconstruction uncertainty now reaches the independently
validated Primary and braking command trajectories. This live Python eval uses
the production Rust WebSocket boundary with guided state-local queries and no
policy, rigid-body physics, or plant integration.

| mode | exposure ms | point radius mm | self sampled / continuous loss mm | world sampled / continuous loss mm | root translation / rotation radius (mm / mrad) | selection |
|---|---:|---:|---:|---:|---:|---|
{chr(10).join(rows)}

## Admission contract

- Self-collision consumes two body-local represented-point radii for both the
  sampled grid and bounded between-sample certificate.
- World collision carries root translation/rotation reconstruction error in the
  root-prediction initial radius, then consumes one body-local point radius
  through the immutable field Lipschitz bound.
- Raw command geometry, robust command geometry, tracking mismatch, scene
  validity, solver status, and final Primary/brake/reject selection remain
  distinct witnesses in the example authority stack.
- Exact reconstruction is the zero-error compatibility case. Held, stale, or
  horizon-invalid reconstruction is withheld before command admission.

## Retained audit

```json
{json.dumps(audit, indent=2, sort_keys=True)}
```

These caller-authored deterministic radii are not estimator covariance,
probability, calibrated hardware safety, or evidence that the plant realizes an
admitted command.
"""


def html_report(report: str, audit: dict[str, Any]) -> str:
    rows = "".join(
        "<tr>"
        f"<td>{mode}</td><td>{item['exposure_ns'] / 1e6:.3f}</td>"
        f"<td>{item['error_bound']['point_position_m'] * 1e3:.6f}</td>"
        f"<td>{item['command_margin_erosion']['self_sampled_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{item['command_margin_erosion']['self_continuous_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{item['command_margin_erosion']['world_sampled_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{item['command_margin_erosion']['world_continuous_m']['minimum'] * 1e3:.6f}</td>"
        f"<td>{html.escape(json.dumps(item['selection_counts'], sort_keys=True))}</td>"
        "</tr>"
        for mode, item in audit["modes"].items()
    )
    method = report.split("## Retained audit", 1)[0]
    return f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Bonesaw command reconstruction uncertainty · r121</title><style>
:root{{color-scheme:dark;font-family:ui-monospace,SFMono-Regular,Consolas,monospace}}body{{margin:0;background:#0b0d10;color:#dce6e1}}main{{max-width:1120px;margin:auto;padding:28px 18px 60px}}h1{{font:600 24px system-ui;color:#eff8f3}}h2{{color:#8fe6c0}}.pass{{padding:13px;border-left:3px solid #8fe6c0;background:#14231d;color:#bdebd7}}.table{{overflow-x:auto}}table{{width:100%;min-width:930px;border-collapse:collapse;font-size:12px}}th,td{{padding:9px;border:1px solid #2a3039;text-align:right}}th:first-child,td:first-child{{text-align:left}}th{{color:#8fe6c0;background:#151a1d}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;padding:16px;border:1px solid #2a3039;background:#11151a;line-height:1.5}}@media(max-width:600px){{main{{padding:18px 10px 40px}}}}
</style></head><body><main><h1>Bonesaw command reconstruction uncertainty · r121</h1><p class="pass">PASS · Primary and brake command geometry consume typed reconstruction radii without folding them into tracking mismatch.</p><div class="table"><table><thead><tr><th>mode</th><th>exposure ms</th><th>point radius mm</th><th>self sampled loss mm</th><th>self continuous loss mm</th><th>world sampled loss mm</th><th>world continuous loss mm</th><th>selection</th></tr></thead><tbody>{rows}</tbody></table></div><h2>Method</h2><pre>{html.escape(method)}</pre><h2>Complete retained audit</h2><pre>{html.escape(json.dumps(audit, indent=2, sort_keys=True))}</pre></main></body></html>"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8819/ws")
    parser.add_argument("--frames-per-mode", type=int, default=20)
    parser.add_argument(
        "--output", default="benchmarks/results/live-command-observation-uncertainty-r121"
    )
    parser.add_argument(
        "--web-report", default="web/LIVE_COMMAND_OBSERVATION_UNCERTAINTY_R121.html"
    )
    args = parser.parse_args()
    if args.frames_per_mode < 2:
        raise ValueError("--frames-per-mode must be at least two")

    socket = create_connection(args.url, timeout=10)
    try:
        hello = receive_kind(socket, "hello", 1)
        assert hello["squat_execution"] == "guided_preview"
        assert hello["authority_contract"]["source"].endswith("-r121")
        idle = receive_kind(socket, "state")
        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= 0.06
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))

        summaries = {}
        for mode in ("exact", "interpolated", "predicted"):
            set_mode(socket, mode)
            states = collect_mode_states(socket, mode, args.frames_per_mode)
            assert_command_uncertainty_mode(mode, states)
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
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    audit = {
        "schema": 1,
        "revision": "live-command-observation-uncertainty-r121",
        "status": "pass",
        "url": args.url,
        "execution": "guided_state_local_queries_without_policy_physics_or_plant_integration",
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
    (output / "LIVE_COMMAND_OBSERVATION_UNCERTAINTY_AUDIT.md").write_text(report)
    web_report = Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(html_report(report, audit))
    print(json.dumps(audit, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
