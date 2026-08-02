#!/usr/bin/env python3
"""Exercise live exact/interpolated/predicted/stale observation authority.

The eval uses the real WebSocket adapter and Rust WBC boundary, but the guided
editor state path intentionally supplies no policy or physics rollout.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import statistics
from pathlib import Path
from typing import Any

from websocket import create_connection


def receive(socket: Any, attempts: int = 200) -> dict[str, Any]:
    for _ in range(attempts):
        message = json.loads(socket.recv())
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "server error"))
        return message
    raise RuntimeError("WebSocket did not produce a message")


def receive_kind(socket: Any, kind: str, attempts: int = 200) -> dict[str, Any]:
    for _ in range(attempts):
        message = receive(socket)
        if message.get("type") == kind:
            return message
    raise RuntimeError(f"did not receive {kind!r} within {attempts} messages")


def set_mode(socket: Any, mode: str) -> None:
    socket.send(json.dumps({"type": "set_observation_transport", "mode": mode}))


def collect_mode_states(
    socket: Any, mode: str, count: int, attempts: int = 600
) -> list[dict[str, Any]]:
    states: list[dict[str, Any]] = []
    for _ in range(attempts):
        message = receive(socket)
        if message.get("type") != "state":
            continue
        metrics = message["metrics"]
        if metrics.get("authority_profile") != "floating_dynamic_wbc_with_command_admission":
            continue
        if metrics.get("robot_observation_transport_mode") != mode:
            continue
        states.append(message)
        if len(states) == count:
            return states
    raise RuntimeError(f"collected only {len(states)}/{count} {mode} states")


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def timing_summary(states: list[dict[str, Any]], field: str) -> dict[str, float]:
    values = [float(state["metrics"][field]) for state in states]
    return {
        "p50_us": statistics.median(values),
        "p99_us": percentile(values, 0.99),
        "maximum_us": max(values),
    }


def mode_summary(states: list[dict[str, Any]]) -> dict[str, Any]:
    provenance: dict[str, int] = {}
    query_counts = {"exact": 0, "interpolated": 0, "predicted": 0, "held": 0}
    for state in states:
        metrics = state["metrics"]
        key = metrics["robot_observation_reconstruction_provenance"]
        provenance[key] = provenance.get(key, 0) + 1
        for name in query_counts:
            query_counts[name] += metrics[f"robot_observation_frame_{name}_queries"]
    return {
        "frames": len(states),
        "streamed_command_provenance": provenance,
        "command_query_counts": query_counts,
        "source_age_ns": sorted(
            {state["metrics"]["robot_observation_reconstruction_source_age_ns"] for state in states}
        ),
        "observation_admission_age_ns": sorted(
            {state["metrics"]["robot_observation_age_ns"] for state in states}
        ),
        "solve": timing_summary(states, "solve_us"),
        "command_admission": timing_summary(states, "command_admission_us"),
        "hard_eligible": all(
            state["metrics"]["robot_observation_reconstruction_hard_eligible"] is True
            for state in states
        ),
        "selection_counts": {
            selection: sum(
                state["metrics"]["command_selection"] == selection for state in states
            )
            for selection in ("primary", "contingency", "rejected")
        },
    }


def assert_mode(mode: str, states: list[dict[str, Any]]) -> None:
    for state in states:
        metrics = state["metrics"]
        assert metrics["robot_observation_reconstruction_hard_eligible"] is True
        assert metrics["robot_observation_frame_held_queries"] == 0
        assert metrics["robot_observation_ingest_rejected"] == 0
        assert metrics["robot_observation_causal"] is True
        assert metrics["robot_observation_age_valid"] is True
        assert metrics["robot_observation_synchronization_valid"] is True
        query = metrics["robot_observation_transport_query_time_ns"]
        assert query == metrics["robot_observation_mapped_time_ns"]
        lower = metrics["robot_observation_reconstruction_lower_time_ns"]
        upper = metrics["robot_observation_reconstruction_upper_time_ns"]
        assert isinstance(query, int)
        if mode == "exact":
            assert metrics["robot_observation_reconstruction_provenance"] == "exact"
            assert lower == query == upper
            assert metrics["robot_observation_reconstruction_source_age_ns"] == 0
            assert metrics["robot_observation_frame_exact_queries"] == 4
        elif mode == "interpolated":
            assert metrics["robot_observation_reconstruction_provenance"] == "interpolated"
            assert lower < query < upper
            assert query - lower == 2_500_000
            assert upper - query == 2_500_000
            assert metrics["robot_observation_reconstruction_source_age_ns"] == 2_500_000
            assert metrics["robot_observation_age_ns"] == 2_500_000
            assert metrics["robot_observation_frame_interpolated_queries"] == 4
        elif mode == "predicted":
            assert metrics["robot_observation_reconstruction_provenance"] == "predicted"
            assert lower == upper
            assert query - lower == 5_000_000
            assert metrics["robot_observation_reconstruction_source_age_ns"] == 5_000_000
            assert metrics["robot_observation_frame_predicted_queries"] == 2
            assert metrics["robot_observation_frame_exact_queries"] == 2
        else:
            raise AssertionError(mode)


def markdown_report(audit: dict[str, Any]) -> str:
    modes = audit["modes"]
    rows = []
    for mode in ("exact", "interpolated", "predicted"):
        item = modes[mode]
        rows.append(
            f"| {mode} | {item['frames']} | "
            f"`{json.dumps(item['streamed_command_provenance'], sort_keys=True)}` | "
            f"`{json.dumps(item['command_query_counts'], sort_keys=True)}` | "
            f"{item['solve']['p50_us']:.3f} / {item['solve']['p99_us']:.3f} | "
            f"{item['command_admission']['p50_us']:.3f} / {item['command_admission']['p99_us']:.3f} |"
        )
    stale = audit["stale_withheld"]
    recovery = audit["recovery"]
    return f"""# Bonesaw live observation transaction authority · r121

**PASS.** The real WebSocket/Rust WBC boundary exercised exact, local cubic
interpolation, bounded constant-velocity prediction, and a deliberately stale
producer without a policy or physics rollout. Every admitted reconstructed
state remained hard-eligible. Stale evidence was withheld before WBC at query
`{stale['query_time_ns']}` ns with newest sample `{stale['newest_sample_time_ns']}` ns,
then exact delivery recovered at tick `{recovery['tick']}` without restarting the session.

| transport | frames | command provenance | four command queries E/I/P/H | solve p50 / p99 µs | admission p50 / p99 µs |
|---|---:|---|---|---:|---:|
{chr(10).join(rows)}

## Typed stale witness

```json
{json.dumps(stale, indent=2, sort_keys=True)}
```

## Interpretation

- `exact` is the zero-lookback 5 ms producer path.
- `interpolated` deliberately queries 2.5 ms behind control time with two
  bracketing samples; downstream observation admission sees the same 2.5 ms age.
- `predicted` uses a 10 ms producer. The fourth 5 ms command query is a genuine
  5 ms prediction inside the declared horizon, and the streamed reconstruction,
  WBC, command admission, and selection retain that same query timestamp.
- `stale` pauses the producer. The controller clock continues advancing and
  emits a structured `observation_withheld` event when extrapolation exceeds
  5 ms; no held state reaches hard rows.

This measures state reconstruction and command authority, not plant tracking,
closed-loop stability, probabilistic uncertainty, or a physics simulator.
"""


def html_report(markdown: str, audit: dict[str, Any]) -> str:
    summary = html.escape(markdown.split("## Typed stale witness", 1)[0])
    return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Bonesaw live observation transaction · r121</title>
<style>
:root{{color-scheme:dark;font-family:ui-monospace,SFMono-Regular,Consolas,monospace}}
body{{margin:0;background:#0b0d10;color:#dce6e1}}main{{max-width:1000px;margin:auto;padding:28px 18px 60px}}
h1{{font:600 24px system-ui;color:#eff8f3}}h2{{margin-top:30px;color:#8fe6c0;font-size:15px}}
pre{{white-space:pre-wrap;overflow-wrap:anywhere;padding:16px;border:1px solid #2a3039;background:#11151a;line-height:1.55}}
.pass{{padding:13px;border-left:3px solid #8fe6c0;background:#14231d;color:#bdebd7}}
table{{width:100%;border-collapse:collapse;font-size:12px}}th,td{{padding:9px;border:1px solid #2a3039;text-align:left}}
th{{color:#8fe6c0;background:#151a1d}}code{{color:#f3c879}}
@media(max-width:600px){{main{{padding:18px 10px 40px}}.table{{overflow-x:auto}}table{{min-width:780px}}}}
</style></head><body><main><h1>Bonesaw live observation transaction authority · r121</h1>
<p class="pass">PASS · exact, interpolation, prediction, stale fail-closed, and in-session recovery all verified.</p>
<div class="table"><table><thead><tr><th>mode</th><th>frames</th><th>boundary provenance</th><th>source ages (ms)</th><th>solve p50 / p99 (µs)</th><th>admission p50 / p99 (µs)</th></tr></thead><tbody>
{''.join(f'<tr><td>{mode}</td><td>{audit["modes"][mode]["frames"]}</td><td>{html.escape(json.dumps(audit["modes"][mode]["streamed_command_provenance"], sort_keys=True))}</td><td>{html.escape(json.dumps([value / 1e6 for value in audit["modes"][mode]["source_age_ns"]]))}</td><td>{audit["modes"][mode]["solve"]["p50_us"]:.3f} / {audit["modes"][mode]["solve"]["p99_us"]:.3f}</td><td>{audit["modes"][mode]["command_admission"]["p50_us"]:.3f} / {audit["modes"][mode]["command_admission"]["p99_us"]:.3f}</td></tr>' for mode in ("exact", "interpolated", "predicted"))}
</tbody></table></div><h2>Typed stale witness</h2><pre>{html.escape(json.dumps(audit['stale_withheld'], indent=2, sort_keys=True))}</pre>
<h2>Complete retained audit</h2><pre>{html.escape(json.dumps(audit, indent=2, sort_keys=True))}</pre>
<h2>Method note</h2><pre>{summary}</pre></main></body></html>"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8818/ws")
    parser.add_argument("--frames-per-mode", type=int, default=12)
    parser.add_argument(
        "--output", default="benchmarks/results/live-observation-transport-r121"
    )
    parser.add_argument("--web-report", default="web/LIVE_OBSERVATION_TRANSPORT_R121.html")
    args = parser.parse_args()
    if args.frames_per_mode < 2:
        raise ValueError("--frames-per-mode must be at least two")

    socket = create_connection(args.url, timeout=10)
    try:
        hello = receive_kind(socket, "hello", 1)
        assert hello["squat_execution"] == "guided_preview"
        assert hello["authority_contract"]["source"].endswith(("-r118", "-r119", "-r121"))
        idle = receive_kind(socket, "state")
        handle = hello["interaction_handles"][0]
        frame_id = hello["frame_names"].index(handle["frame"])
        frame = next(item for item in idle["frames"] if item["id"] == frame_id)
        target = list(frame["translation"])
        target[2] -= 0.04
        socket.send(json.dumps({"type": "drag", "frame": handle["frame"], "target": target}))

        modes: dict[str, Any] = {}
        for mode in ("exact", "interpolated", "predicted"):
            set_mode(socket, mode)
            states = collect_mode_states(socket, mode, args.frames_per_mode)
            assert_mode(mode, states)
            modes[mode] = mode_summary(states)

        set_mode(socket, "stale")
        stale = None
        for _ in range(100):
            message = receive(socket)
            if message.get("type") == "observation_withheld" and message.get("transport_mode") == "stale":
                stale = message
                break
        assert stale is not None
        assert isinstance(stale["query_time_ns"], int)
        assert isinstance(stale["newest_sample_time_ns"], int)
        assert stale["query_time_ns"] - stale["newest_sample_time_ns"] > 5_000_000
        assert "reconstruction" in stale["reason"]

        set_mode(socket, "exact")
        recovered = collect_mode_states(socket, "exact", 1)[0]
        assert recovered["tick"] > stale["tick"]
        assert recovered["metrics"]["robot_observation_reconstruction_provenance"] == "exact"
        socket.send(json.dumps({"type": "release"}))
    finally:
        socket.close()

    audit = {
        "schema": 1,
        "revision": "live-observation-transport-r121",
        "status": "pass",
        "url": args.url,
        "execution": "guided_preview_without_policy_or_physics_rollout",
        "contract_source": hello["authority_contract"]["source"],
        "program_fingerprint": hello["program_fingerprint"],
        "modes": modes,
        "stale_withheld": stale,
        "recovery": {
            "tick": recovered["tick"],
            "provenance": recovered["metrics"]["robot_observation_reconstruction_provenance"],
            "transport_mode": recovered["metrics"]["robot_observation_transport_mode"],
        },
    }
    report = markdown_report(audit)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "audit.json").write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n")
    (output / "LIVE_OBSERVATION_TRANSPORT_AUDIT.md").write_text(report)
    web_report = Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(html_report(report, audit))
    print(json.dumps(audit, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
