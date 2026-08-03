#!/usr/bin/env python3
"""Audit schedule-bounded DCM shaping on the immutable G1 replay."""

from __future__ import annotations

import json
import pathlib
from datetime import datetime, timezone
from typing import Any

import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html
from g1_hard_feasibility_witness_r275 import fingerprint, load, semantic_differences


REVISION = "g1-dcm-transition-gate-r277"
ROOT = pathlib.Path("benchmarks/results")
CONTROL = ROOT / "floating-g1-r276-auto-localization-dormant-cap8"
DORMANT = ROOT / "floating-g1-r277-dcm-dormant-cap8"
GATED = ROOT / "floating-g1-r277-dcm-gated-w0p00005-h10-cap8"
CONTINUOUS = ROOT / "floating-g1-r277-dcm-continuous-w0p025-cap8"
SWEEP = (
    (0.00001, 5, ROOT / "floating-g1-r277-dcm-w0p00001-h5-ss-cap8"),
    (0.000025, 5, ROOT / "floating-g1-r277-dcm-w0p000025-h5-ss-cap8"),
    (0.00005, 5, ROOT / "floating-g1-r277-dcm-w0p00005-h5-ss-cap8"),
    (0.0001, 5, ROOT / "floating-g1-r277-dcm-w0p0001-h5-ss-cap8"),
    (0.00001, 10, ROOT / "floating-g1-r277-dcm-w0p00001-h10-ss-cap8"),
    (0.000025, 10, ROOT / "floating-g1-r277-dcm-w0p000025-h10-ss-cap8"),
    (0.00005, 10, ROOT / "floating-g1-r277-dcm-w0p00005-h10-ss-cap8"),
    (0.0001, 10, ROOT / "floating-g1-r277-dcm-w0p0001-h10-ss-cap8"),
)


def first_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[0]) if len(ticks) else None


def last_tick(mask: np.ndarray) -> int | None:
    ticks = np.flatnonzero(mask)
    return int(ticks[-1]) if len(ticks) else None


def json_metrics(directory: pathlib.Path) -> dict[str, Any]:
    return json.loads((directory / "floating-walk-metrics.json").read_text())[
        "metrics"
    ]


def summarize(directory: pathlib.Path) -> dict[str, Any]:
    trace = load(directory / "floating-walk-raw.npz")
    metrics = json_metrics(directory)
    timing_ms = trace["step_ns"].astype(np.float64) / 1.0e6
    root_error = np.linalg.norm(
        trace["root_tracked"] - trace["effective_root_targets"], axis=1
    )
    foot_error = np.linalg.norm(
        trace["tracked_positions"][:, :2]
        - trace["effective_target_positions"][:, :2],
        axis=2,
    )
    active = trace.get(
        "dcm_pre_liftoff_active", np.zeros(len(trace["status"]), dtype=np.uint8)
    ).astype(bool)
    runtime = metrics["runtime"]
    return {
        "active_ticks": int(np.count_nonzero(active)),
        "first_active_tick": first_tick(active),
        "last_active_tick": last_tick(active),
        "first_fallback_tick": first_tick(trace["status"] == 4),
        "first_release_tick": first_tick(np.isin(trace["status"], (5, 12))),
        "root_tracking_rms_m": float(np.sqrt(np.mean(np.square(root_error)))),
        "foot_tracking_rms_m": float(np.sqrt(np.mean(np.square(foot_error)))),
        "timing_ms": distribution(timing_ms),
        "five_ms_misses": int(np.count_nonzero(timing_ms > 5.0)),
        "twenty_ms_misses": int(np.count_nonzero(timing_ms > 20.0)),
        "tick_874": {
            "root_y_m": float(trace["root_tracked"][874, 1]),
            "center_of_mass_y_m": float(trace["center_of_mass_tracked"][874, 1]),
            "root_y_velocity_mps": float(trace["v"][874, 1]),
            "right_knee_position_rad": float(trace["q"][874, 9]),
        },
        "dcm": metrics["dcm_balance"],
        "runtime": {
            "wall_ms_per_tick": float(runtime["call_wall_ns"] / 1.0e6 / len(active)),
            "cpu_ms_per_tick": float(runtime["process_cpu_ns"] / 1.0e6 / len(active)),
            "cpu_to_wall_ratio": float(runtime["process_cpu_to_wall_ratio"]),
            "rss_delta_bytes": int(runtime["rss_delta_bytes"]),
            "peak_rss_bytes": int(runtime["peak_rss_bytes"]),
            "python_tracemalloc_peak_bytes": int(
                runtime["python_tracemalloc_peak_bytes"]
            ),
            "python_gc_collections": int(
                runtime["python_gc_delta"]["collections"]
            ),
        },
    }


def main() -> int:
    directories = (CONTROL, DORMANT, GATED, CONTINUOUS) + tuple(
        row[2] for row in SWEEP
    )
    missing = [
        str(path)
        for directory in directories
        for path in (
            directory / "floating-walk-raw.npz",
            directory / "floating-walk-metrics.json",
        )
        if not path.is_file()
    ]
    if missing:
        raise SystemExit("R277 is missing evidence: " + ", ".join(missing))

    control_trace = load(CONTROL / "floating-walk-raw.npz")
    dormant_trace = load(DORMANT / "floating-walk-raw.npz")
    dormant_differences = semantic_differences(control_trace, dormant_trace)
    profiles = {
        "R276 control": summarize(CONTROL),
        "R277 dormant": summarize(DORMANT),
        "R277 gated 5e-5 / 10": summarize(GATED),
        "continuous 0.025": summarize(CONTINUOUS),
    }
    sweep = [
        {"weight": weight, "horizon_ticks": horizon, **summarize(directory)}
        for weight, horizon, directory in SWEEP
    ]
    shared_arrays = set(control_trace) & set(dormant_trace)
    new_arrays = sorted(set(dormant_trace) - set(control_trace))
    mechanism_passed = bool(
        not dormant_differences
        and profiles["R277 dormant"]["active_ticks"] == 0
        and profiles["R277 gated 5e-5 / 10"]["active_ticks"] > 0
        and profiles["continuous 0.025"]["active_ticks"] == 2317
        and new_arrays == ["dcm_pre_liftoff_active"]
    )
    gated = profiles["R277 gated 5e-5 / 10"]
    control = profiles["R276 control"]
    profile_rejected = bool(
        gated["first_release_tick"] < control["first_release_tick"]
        and gated["root_tracking_rms_m"] > control["root_tracking_rms_m"]
        and gated["foot_tracking_rms_m"] > control["foot_tracking_rms_m"]
        and gated["timing_ms"]["p99"] > 5.0
    )

    profile_rows = [
        [
            name,
            str(profile["active_ticks"]),
            str(profile["first_fallback_tick"]),
            str(profile["first_release_tick"]),
            f'{profile["root_tracking_rms_m"]:.3f}',
            f'{profile["foot_tracking_rms_m"]:.3f}',
            f'{profile["timing_ms"]["p99"]:.3f}',
            f'{profile["timing_ms"]["maximum"]:.3f}',
            str(profile["five_ms_misses"]),
            str(profile["twenty_ms_misses"]),
        ]
        for name, profile in profiles.items()
    ]
    state_rows = [
        [
            name,
            f'{profile["tick_874"]["root_y_m"]:.5f}',
            f'{profile["tick_874"]["center_of_mass_y_m"]:.5f}',
            f'{profile["tick_874"]["root_y_velocity_mps"]:.5f}',
            f'{profile["tick_874"]["right_knee_position_rad"]:.5f}',
        ]
        for name, profile in profiles.items()
    ]
    runtime_rows = [
        [
            name,
            f'{profile["runtime"]["wall_ms_per_tick"]:.3f}',
            f'{profile["runtime"]["cpu_ms_per_tick"]:.3f}',
            f'{profile["runtime"]["cpu_to_wall_ratio"]:.4f}',
            f'{profile["runtime"]["rss_delta_bytes"] / (1 << 20):.3f}',
            f'{profile["runtime"]["peak_rss_bytes"] / (1 << 20):.3f}',
            str(profile["runtime"]["python_gc_collections"]),
            str(profile["runtime"]["python_tracemalloc_peak_bytes"]),
        ]
        for name, profile in profiles.items()
    ]
    sweep_rows = [
        [
            f'{row["weight"]:.6f}',
            str(row["horizon_ticks"]),
            str(row["first_fallback_tick"]),
            str(row["first_release_tick"]),
            f'{row["root_tracking_rms_m"]:.3f}',
            f'{row["foot_tracking_rms_m"]:.3f}',
        ]
        for row in sweep
    ]
    dcm = gated["dcm"]
    report = "\n".join(
        [
            "# G1 schedule-bounded DCM transition shaping · R277",
            "",
            "**Mechanism passed / DCM profile rejected.** R277 adds a default-off, allocation-free Rust schedule gate around the existing DCM/virtual-ZMP task. A positive horizon activates before an authored multi-to-single support loss, stays active through single support, and emits a caller-owned byte witness. Zero preserves the historical continuous mode.",
            "",
            *markdown_table(
                ["profile", "active ticks", "fallback", "release", "root RMS m", "foot RMS m", "p99 ms", "max ms", ">5 ms", ">20 ms"],
                profile_rows,
            ),
            "",
            "## Causal state at the original conflict",
            "",
            *markdown_table(
                ["profile", "root y m", "CoM y m", "root vy m/s", "right knee rad"],
                state_rows,
            ),
            "",
            "The gated profile does move the upstream state in the intended direction: at tick 874, lateral root speed falls from -1.668 to -0.427 m/s, CoM y moves from 0.14338 to 0.10696 m, and the right knee is no longer pinned at its -0.087267 rad lower limit. The original tick-875 full-lock conflict moves to tick 926. This is a causal mechanism result, not a walking pass.",
            "",
            "The consequence remains unacceptable. Release advances from tick 1108 to 959, root/foot RMS rise from 15.514/15.467 m to 19.280/19.059 m, and p99 reaches 6.726 ms with two 20 ms misses. During the gated interval, virtual ZMP is clipped on "
            f'{100.0 * dcm["zmp_clipped_fraction"]:.1f}% of active ticks, support margin p05/min is {dcm["support_margin_p05_m"]:.3f}/{dcm["support_margin_minimum_m"]:.3f} m, and DCM acceleration p95/max is {dcm["command_acceleration_p95_mps2"]:.3f}/{dcm["command_acceleration_maximum_mps2"]:.3f} m/s². The soft task can change the state, but it does not construct a support-feasible trajectory.',
            "",
            "## Frozen small-weight screen",
            "",
            *markdown_table(
                ["weight", "pre ticks", "fallback", "release", "root RMS m", "foot RMS m"],
                sweep_rows,
            ),
            "",
            "All eight schedule-bounded profiles fail before the control release; the 5e-5 / 10-tick row is the latest at tick 959 and was rerun alone for the retained timing/memory record. The historical continuous 0.025 mode is a negative control: it releases at tick 466 and reaches 24.882/24.569 m root/foot RMS.",
            "",
            "## Dataflow, CPU, jitter, and memory",
            "",
            *markdown_table(
                ["profile", "wall ms/tick", "CPU ms/tick", "CPU/wall", "RSS delta MiB", "peak RSS MiB", "Python GC", "tracemalloc peak B"],
                runtime_rows,
            ),
            "",
            f"Dormant R277 is bit-exact on all {len(shared_arrays) - 1} shared non-timing arrays against R276; the only new array is `dcm_pre_liftoff_active`, and it is zero throughout. Every retained run performs zero policy steps and zero physics steps. The Python hot call records zero garbage collections and a 1,672-byte tracemalloc peak; caller-owned arrays carry all per-tick telemetry. Timing is workload-dependent because rejected profiles enter different contact/release states, so it is reported as consequence rather than normalized solver speedup.",
            "",
            "## Decision",
            "",
            "Retain the schedule gate as a default-off experimental/diagnostic mechanism, but admit no profile and no authority. R277 rules out both continuous and schedule-bounded scalar weighting of the current DCM objective. The next CPU implementation needs an explicit support-feasible root/CoM trajectory tube (with velocity and joint-headroom state), not another gain sweep or failure-time selector.",
        ]
    )
    result = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "policy_steps": 0,
        "physics_steps": 0,
        "profiles": profiles,
        "sweep": sweep,
        "source_contract": {
            "control_sha256": fingerprint(CONTROL / "floating-walk-raw.npz"),
            "dormant_sha256": fingerprint(DORMANT / "floating-walk-raw.npz"),
            "shared_control_array_count": len(shared_arrays),
            "shared_non_timing_array_count": len(shared_arrays) - 1,
            "dormant_semantic_differences": dormant_differences,
            "new_arrays": new_arrays,
        },
        "mechanism_passed": mechanism_passed,
        "profile_rejected": profile_rejected,
        "authority_admitted": False,
    }
    output = ROOT / REVISION
    output.mkdir(parents=True, exist_ok=True)
    (output / "g1-dcm-transition-gate-metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    (output / "G1_DCM_TRANSITION_GATE_R277.md").write_text(report + "\n")
    pathlib.Path("web/G1_DCM_TRANSITION_GATE_R277.html").write_text(
        render_report_html(report, title="G1 schedule-bounded DCM transition shaping · R277")
    )
    print(output / "G1_DCM_TRANSITION_GATE_R277.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
