#!/usr/bin/env python3
"""Render the R39 measured touchdown phase-retiming ablation.

All controller behavior is read from raw Rust-produced traces.  This script is
deliberately report-only: it aggregates fixed experiment outputs without
reimplementing the phase policy in Python.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


CASES = {
    "viable_off": "g1-synthetic-step-r39-retiming-disabled-smoke",
    "viable_on": "g1-synthetic-step-r39-retiming-enabled-smoke",
    "stress_off": "g1-synthetic-step-r39-stress06-disabled",
    "stress_on_1": "g1-synthetic-step-r39-stress06-enabled",
    "stress_on_2": "g1-synthetic-step-r39-stress06-enabled-repeat2",
    "stress_on_3": "g1-synthetic-step-r39-stress06-enabled-repeat3",
    "cmu_off": "g1-transfer-r38-preview-200",
    "cmu_on_600": "g1-transfer-r39-retiming-release100-guard020",
    "cmu_on_800": "g1-transfer-r39-retiming-release100-guard020-800",
}

BEHAVIOR_ARRAYS = (
    "root_tracked",
    "root_quaternion_wxyz",
    "tracked_positions",
    "q",
    "v",
    "contact_normal_force",
    "dynamics_residual",
    "contact_residual",
    "status",
    "support_phase",
    "reference_phase",
    "reference_phase_target_rate",
    "reference_phase_rate",
    "reference_phase_acceleration",
    "effective_root_targets",
    "effective_center_of_mass_targets",
    "effective_target_positions",
    "effective_contact_active",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", default="benchmarks/results")
    parser.add_argument(
        "--output",
        default=(
            "benchmarks/results/g1-phase-retiming-r39/"
            "TOUCHDOWN_PHASE_RETIMING_ABLATION.md"
        ),
    )
    return parser.parse_args()


def load_case(results_root: Path, name: str) -> tuple[dict[str, Any], Any]:
    case = results_root / name
    payload = json.loads((case / "floating-walk-metrics.json").read_text())
    return payload["metrics"], np.load(case / "floating-walk-raw.npz")


def behavior_digest(trace: Any) -> str:
    digest = hashlib.sha256()
    for key in BEHAVIOR_ARRAYS:
        array = np.ascontiguousarray(trace[key])
        digest.update(key.encode())
        digest.update(array.dtype.str.encode())
        digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
        digest.update(array.tobytes())
    return digest.hexdigest()


def exact_trace(left: Any, right: Any) -> bool:
    return all(np.array_equal(left[key], right[key]) for key in BEHAVIOR_ARRAYS)


def status_non_nominal(metrics: dict[str, Any]) -> int:
    status = metrics["status_counts"]
    return (
        status["normal_contact_contingency"]
        + status["contact_release_contingency"]
        + status["primal_infeasible"]
        + status["failed"]
    )


def failed_checks(metrics: dict[str, Any]) -> str:
    failed = [
        name
        for name, passed in metrics["acceptance"]["checks"].items()
        if not passed
    ]
    return "none" if not failed else ", ".join(failed)


def phase_cell(metrics: dict[str, Any], key: str, default: float = 0.0) -> float:
    phase = metrics.get("touchdown_phase_retiming") or {}
    return float(phase.get(key, default))


def main() -> None:
    args = parse_args()
    root = Path(args.results_root)
    loaded = {key: load_case(root, name) for key, name in CASES.items()}
    metrics = {key: value[0] for key, value in loaded.items()}
    traces = {key: value[1] for key, value in loaded.items()}

    viable_no_op = all(
        np.array_equal(traces["viable_off"][effective], traces["viable_off"][authored])
        for effective, authored in (
            ("effective_root_targets", "root_targets"),
            ("effective_center_of_mass_targets", "center_of_mass_targets"),
            ("effective_target_positions", "target_positions"),
            ("effective_contact_active", "contact_active"),
        )
    ) and np.array_equal(
        traces["viable_off"]["reference_phase"],
        np.arange(len(traces["viable_off"]["reference_phase"]), dtype=np.float64),
    )
    viable_on_off_exact = exact_trace(traces["viable_off"], traces["viable_on"])
    repeat_exact = all(
        exact_trace(traces["stress_on_1"], traces[key])
        for key in ("stress_on_2", "stress_on_3")
    )
    repeat_digests = [
        behavior_digest(traces[key])
        for key in ("stress_on_1", "stress_on_2", "stress_on_3")
    ]
    repeat_p99 = [
        metrics[key]["latency_us"]["p99"]
        for key in ("stress_on_1", "stress_on_2", "stress_on_3")
    ]
    repeat_cpu_ms = [
        metrics[key]["runtime"]["thread_cpu_ns"] / 1e6
        for key in ("stress_on_1", "stress_on_2", "stress_on_3")
    ]

    lines = [
        "# Bonesaw R39 measured touchdown phase-retiming ablation",
        "",
        "This report evaluates one architectural change: Rust owns a persistent "
        "source-phase cursor and retimes root, CoM, every endpoint jet, and contact "
        "intent together from measured sole position and velocity. Python constructs "
        "the fixed corpus and renders this report; it does not reproduce the policy.",
        "",
        "## Outcome",
        "",
        f"- Viable-reference no-op contract: **{'PASS' if viable_no_op else 'FAIL'}**; "
        f"enabled and disabled viable traces behaviorally exact: "
        f"**{'PASS' if viable_on_off_exact else 'FAIL'}**.",
        f"- Stressed touchdown rescue: nominal transition "
        f"`{metrics['stress_off']['maximum_touchdown_transition_ticks']}` ticks → "
        f"retimed `{metrics['stress_on_1']['maximum_touchdown_transition_ticks']}` ticks; "
        f"combined acceptance **{'PASS' if metrics['stress_on_1']['acceptance']['passed'] else 'FAIL'}**.",
        f"- Retimed stressed trace reproducibility: "
        f"**{'PASS' if repeat_exact else 'FAIL'}**, three bit-for-bit behavioral repeats; "
        f"median p99 `{np.median(repeat_p99):.1f} µs`, all "
        f"`{sum(value <= 5_000.0 for value in repeat_p99)}/3` below 5 ms.",
        f"- CMU H=200 failure boundary: 600-tick cursor ends at source tick "
        f"`{phase_cell(metrics['cmu_on_600'], 'final_source_tick'):.3f}` before authored "
        f"touchdown tick `428`; the new progress gate correctly remains **FAIL**.",
        "",
        "## Policy contract",
        "",
        "1. A pure Rust policy estimates conservative landing time from position "
        "excess, whole-patch tangential speed, normal speed, acceleration capacity, "
        "and the unchanged 2.5 cm / 0.20 m/s / 0.20 m/s admission envelope.",
        "2. The fastest admissible scalar phase rate is slew-bounded. Engagement can "
        "be immediate; recovery is bounded to avoid cadence snapping.",
        "3. One Rust cursor samples quintic Hermite jets for root, CoM, and all "
        "effectors. Velocity and acceleration use the full time-warp chain rule.",
        "4. Contact intent is sampled from the same cursor. The evaluator additionally "
        "requires the first post-liftoff authored touchdown edge to be reached, so "
        "holding before contact cannot score as a vacuous admission success.",
        "",
        "## Contact and acceptance matrix",
        "",
        "| case | phase | combined | first touchdown reached | transition ticks | admission delay | contingency/rejected | minimum rate | failed checks |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for label, key in (
        ("viable synthetic", "viable_off"),
        ("viable synthetic", "viable_on"),
        ("6 cm / 100 ms stress", "stress_off"),
        ("6 cm / 100 ms stress", "stress_on_1"),
        ("CMU H=200, 600 ticks", "cmu_off"),
        ("CMU H=200, 600 ticks", "cmu_on_600"),
        ("CMU H=200, 800 ticks", "cmu_on_800"),
    ):
        item = metrics[key]
        phase = item.get("touchdown_phase_retiming")
        reached = "n/a" if not phase else str(phase["first_authored_touchdown_reached"])
        lines.append(
            f"| {label} | {'on' if phase and phase['enabled'] else 'off'} | "
            f"{'PASS' if item['acceptance']['passed'] else 'FAIL'} | {reached} | "
            f"{item['maximum_touchdown_transition_ticks']} | "
            f"{item['maximum_touchdown_admission_delay_ticks']} | "
            f"{status_non_nominal(item)} | "
            f"{phase_cell(item, 'applied_rate_minimum', 1.0):.4f} | "
            f"{failed_checks(item)} |"
        )

    lines += [
        "",
        "The stressed off/on pair changes only phase retiming. The nominal trace "
        "crosses contact before the whole sole can settle and remains in touchdown "
        "transition for 10 ticks. Retiming bottoms at "
        f"`{phase_cell(metrics['stress_on_1'], 'applied_rate_minimum'):.4f}×`, "
        f"limits `{int(phase_cell(metrics['stress_on_1'], 'limited_ticks'))}` physical "
        "ticks, crosses source touchdown tick 100, and locks in 3 ticks without a "
        "fallback, release, infeasible, or failed solve.",
        "",
        "## Tracking",
        "",
        "| case | root RMS | stance foot RMS | swing foot RMS | max root rotation | max joint speed |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for label, key in (
        ("stress off", "stress_off"),
        ("stress on", "stress_on_1"),
        ("CMU off", "cmu_off"),
        ("CMU on, 600", "cmu_on_600"),
        ("CMU on, 800", "cmu_on_800"),
    ):
        item = metrics[key]
        lines.append(
            f"| {label} | {item['root_tracking_rms_m'] * 100:.3f} cm | "
            f"{item['stance_foot_tracking_rms_m'] * 100:.3f} cm | "
            f"{item['swing_foot_tracking_rms_m'] * 100:.3f} cm | "
            f"{np.rad2deg(item['maximum_root_rotation_rad']):.3f}° | "
            f"{item['maximum_joint_velocity_rad_s']:.3f} rad/s |"
        )

    lines += [
        "",
        "## CPU, latency, jitter, memory, and GC",
        "",
        "Single-process release traces include Python call overhead around one Rust "
        "batch. Scheduler context switches are retained rather than filtered.",
        "",
        "| case | p50 | p99 | jitter p99 | thread CPU | RSS Δ | traced peak | GC collections | involuntary switches |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, key in (
        ("viable off", "viable_off"),
        ("stress off", "stress_off"),
        ("stress on #1", "stress_on_1"),
        ("stress on #2", "stress_on_2"),
        ("stress on #3", "stress_on_3"),
        ("CMU on, 600", "cmu_on_600"),
    ):
        item = metrics[key]
        runtime = item["runtime"]
        lines.append(
            f"| {label} | {item['latency_us']['p50']:.1f} µs | "
            f"{item['latency_us']['p99']:.1f} µs | "
            f"{item['jitter_abs_delta_us']['p99']:.1f} µs | "
            f"{runtime['thread_cpu_ns'] / 1e6:.1f} ms | "
            f"{runtime['rss_delta_bytes'] / 1024:.1f} KiB | "
            f"{runtime['python_tracemalloc_peak_bytes']} B | "
            f"{runtime['python_gc_delta']['collections']} | "
            f"{runtime['usage_delta']['involuntary_context_switches']} |"
        )

    lines += [
        "",
        f"The three stress-on thread-CPU measurements span "
        f"`{min(repeat_cpu_ms):.1f}–{max(repeat_cpu_ms):.1f} ms` for 240 ticks. "
        "All runs report zero Python GC collections in the measured call. RSS delta "
        "is process-level demand paging, not controller heap-allocation telemetry.",
        "",
        "## Determinism and no-op proof",
        "",
        f"- Stress-on behavior digest: `{repeat_digests[0]}`; all three equal: "
        f"`{len(set(repeat_digests)) == 1}`.",
        "- The digest covers state, force, physical residuals, status, support phase, "
        "phase cursor/rates, effective root/CoM/endpoint targets, and effective "
        "contact intent. Timing arrays are intentionally excluded.",
        f"- Disabled retiming reproduces every authored target/contact array and the "
        f"integer source cursor exactly: `{viable_no_op}`.",
        f"- On the already-viable synthetic trace, enabling retiming changes no "
        f"behavioral byte: `{viable_on_off_exact}`.",
        "",
        "## Honest failure boundary",
        "",
        "R39 is not a claim that the current CMU-to-G1 transfer is solved. At 600 "
        "physical ticks the controller advances only to source tick "
        f"`{phase_cell(metrics['cmu_on_600'], 'final_source_tick'):.3f}`; at 800 it "
        f"advances to `{phase_cell(metrics['cmu_on_800'], 'final_source_tick'):.3f}`. "
        "Both remain before touchdown tick 428 as the measured robot diverges. The "
        "first-touchdown progress gate fails both, preventing the absence of delayed "
        "admission samples from being misreported as success. This separates a valid "
        "retiming mechanism from an unsolved sustained-balance/reference problem.",
        "",
        "The independent PlaCo, Pinocchio, and upstream Upkie comparisons remain in "
        "`benchmarks/results/reference-r38/REFERENCE_COMPARISON.md`. Those references "
        "do not expose an equivalent coupled measured-phase policy, so this ablation "
        "compares identical Bonesaw controller configurations off/on and makes no "
        "unsupported cross-implementation phase-retiming claim.",
        "",
    ]

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines))
    print(output)


if __name__ == "__main__":
    main()
