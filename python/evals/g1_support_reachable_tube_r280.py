#!/usr/bin/env python3
"""Validate and render the R280 reachable-support-tube qualification.

The replay itself is intentionally frozen and report-only: no policy or physics
steps are executed here.  This module keeps the evidence contract in Python so
the Rust WBC remains the only timed implementation path.
"""

from __future__ import annotations

import argparse
import json
import pathlib
from typing import Any

from cpu_reference_report import markdown_table, render_report_html


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-support-reachable-tube-r280/g1-support-reachable-tube-r280-metrics.json"
REPORT = RESULT.with_name("G1_SUPPORT_REACHABLE_TUBE_R280.md")
WEB_REPORT = ROOT / "web/G1_SUPPORT_REACHABLE_TUBE_R280.html"
REVISION = "g1-support-reachable-tube-r280"


def load_result(path: pathlib.Path = RESULT) -> dict[str, Any]:
    return json.loads(path.read_text())


def validate_result(result: dict[str, Any]) -> None:
    """Fail loudly if a report is accidentally regenerated from weak evidence."""
    if result.get("revision") != REVISION:
        raise ValueError(f"unexpected revision: {result.get('revision')!r}")
    execution = result["execution"]
    if execution["policy_steps"] != 0 or execution["physics_steps"] != 0:
        raise ValueError("R280 must remain a simulator-free replay")
    profiles = result["profiles"]
    dormant = profiles["dormant"]
    soft = profiles["soft_two_axis"]
    hard = profiles["hard"]
    repeats = result["soft_pinned_repeats"]
    if not result["verdict"]["mechanism_passed"]:
        raise ValueError("reachable-tube mechanism gate is not passing")
    if not result["verdict"]["soft_behavior_passed"]:
        raise ValueError("soft behavior gate is not passing")
    if result["verdict"]["authority_admitted"]:
        raise ValueError("R280 must not admit authority")
    if not (soft["release"] > dormant["release"]):
        raise ValueError("soft profile did not delay release")
    for field in ("root_rms_m", "com_rms_m", "foot_rms_m"):
        if not soft[field] < dormant[field]:
            raise ValueError(f"soft profile regressed {field}")
    deadline = result["gates"]["p99_deadline_us"]
    if len(repeats) != 5 or not all(row["semantic_exact"] for row in repeats):
        raise ValueError("pinned repeats are not semantically exact")
    if not all(row["p99_us"] > deadline for row in repeats):
        raise ValueError("R280 timing rejection must remain visible")
    if hard["solved"] + hard["unresolved"] != hard["active"]:
        raise ValueError("hard solved/unresolved accounting is inconsistent")
    if hard["unresolved"] <= 0:
        raise ValueError("hard profile unexpectedly admitted all active ticks")


def _f(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def _metric(row: dict[str, Any], key: str, digits: int = 3) -> str:
    value = row.get(key)
    return "n/a" if value is None else _f(value, digits)


def build_report(result: dict[str, Any]) -> str:
    p = result["profiles"]
    rows = []
    for name in ("dormant", "observer", "request_only", "soft_two_axis", "hard"):
        row = p[name]
        rows.append(
            [
                name.replace("_", " "),
                row["active"],
                row.get("clipped", 0),
                row["conflict"],
                row["release"],
                "exact" if name in ("observer", "request_only") else _metric(row, "root_rms_m"),
                "exact" if name in ("observer", "request_only") else _metric(row, "com_rms_m"),
                "exact" if name in ("observer", "request_only") else _metric(row, "foot_rms_m"),
                "repeated below" if name == "soft_two_axis" else _f(row["p99_us"], 1),
            ]
        )
    repeat_rows = [
        [
            row["repeat"],
            _f(row["p99_us"], 1),
            _f(row["maximum_us"], 1),
            row["over_5ms"],
            _f(row["wall_ms_per_tick"], 3),
            _f(row["cpu_ms_per_tick"], 3),
            _f(row["rss_delta_mib"], 3),
            row["gc"],
        ]
        for row in result["soft_pinned_repeats"]
    ]
    improvements = p["soft_two_axis"]["improvement_percent"]
    summary = result["summary"]
    report_lines = [
        "# G1 reachable support tube timing qualification · R280",
        "",
        "> Mechanism **PASS** · soft behavior **PASS** · soft timing **REJECTED** · hard authority **REJECTED**.",
        "",
        "R280 qualifies the default-off R279 exact discrete DCM backward-reachable support tube on the frozen 2,317-tick G1 morphology replay. It executes zero policy and zero physics steps. Observer, request shaping, soft tracking, and hard acceleration rows remain independent authority layers.",
        "",
        "## Behavior and switch independence",
        "",
        *markdown_table(
            ["profile", "active", "clipped", "conflict", "release", "root RMS m", "CoM RMS m", "foot RMS m", "p99 µs"],
            rows,
        ),
        "",
        f"The two-axis soft profile delays release {p['dormant']['release']}→{p['soft_two_axis']['release']} and improves root/CoM/foot RMS by {_f(improvements[0], 2)}%/{_f(improvements[1], 2)}%/{_f(improvements[2], 2)}%. Observer and request-only integrated state/status are bit-exact to dormant; request-only changes bounded command telemetry on {p['request_only']['clipped']:,} ticks without gaining execution authority.",
        "",
        "## Pinned CPU, jitter, memory, and GC",
        "",
        *markdown_table(
            ["repeat", "p99 µs", "max µs", ">5 ms", "wall ms/tick", "CPU ms/tick", "RSS Δ MiB", "Python GC"],
            repeat_rows,
        ),
        "",
        f"All five CPU-4-pinned repeats are semantically bit-exact and all miss the {result['gates']['p99_deadline_us']:,.0f} µs p99 gate. Mean p99 is {summary['p99_mean_us']:.1f} µs (population σ {summary['p99_stddev_us']:.1f} µs), with {summary['deadline_misses']} deadline misses across {len(result['soft_pinned_repeats']) * result['execution']['ticks_per_profile']:,} steps. Every repeat has deterministic late status-5 events at ticks 1694 and 2296 (188–197 ms). They do not set p99, but remain a separate controller-path tail defect. The Rust fold/WBC uses fixed-capacity storage; Python owns orchestration and retained trace arrays, so RSS delta is not a per-step Rust allocation claim.",
        "",
        "## Hard evidence and rejected shortcut",
        "",
        f"Hard enforcement admits {p['hard']['solved']}/{p['hard']['active']} active ticks and leaves {p['hard']['unresolved']} explicitly unresolved. Admitted rows have zero face leakage; the negative first-solve witness remains visible and is never integrated. Conflict/release {p['hard']['conflict']}/{p['hard']['release']} rejects hard authority.",
        "",
        f"A world-Y-only task lowered one p99 sample to {p['rejected_lateral_only']['p99_us']:.1f} µs but regressed release to {p['rejected_lateral_only']['release']} and root/CoM/foot RMS to {p['rejected_lateral_only']['root_rms_m']:.3f}/{p['rejected_lateral_only']['com_rms_m']:.3f}/{p['rejected_lateral_only']['foot_rms_m']:.3f} m, all worse than dormant. It was removed. Timing is not purchased by deleting behavior-critical task rows.",
        "",
        "## Decision",
        "",
        "No actuator, contact, or walking authority is admitted. The immediate work is exact-trace CPU optimization of the 0.23–0.73 ms p99 excess and localization of the deterministic release path. Then hard support, contact, joint, actuator, thermal, and plant envelopes can be composed.",
    ]
    return "\n".join(report_lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    result = load_result()
    validate_result(result)
    if not args.check_only:
        report = build_report(result)
        REPORT.write_text(report)
        WEB_REPORT.write_text(
            render_report_html(report, title="G1 reachable support tube · R280")
        )
    print(f"validated {RESULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
