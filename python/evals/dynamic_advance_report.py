#!/usr/bin/env python3
"""Retain the policy-/physics-free floating WBC→servo-block CPU transaction."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import platform
import resource
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np

from bonesaw import DynamicAdvanceSession
from cpu_reference_report import distribution, markdown_table as markdown_table_rows, render_report_html


def table(headers: list[str], rows: list[list[Any]]) -> str:
    return "\n".join(markdown_table_rows(headers, rows))


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", type=int, default=500)
    parser.add_argument("--output", default="benchmarks/results/dynamic-advance-r99")
    parser.add_argument("--web-report", default="web/DYNAMIC_ADVANCE_R99.html")
    return parser.parse_args()


def make_reference(ticks: int, generalized_dof: int) -> np.ndarray:
    time_seconds = np.arange(ticks, dtype=np.float64) * 0.02
    period_seconds = 2.0
    omega = 2.0 * np.pi / period_seconds
    amplitude_rad = 0.02
    acceleration = amplitude_rad * omega * omega * np.cos(omega * time_seconds)
    desired = np.zeros((ticks, generalized_dof), np.float64)
    desired[:, 6] = acceleration
    if generalized_dof > 7:
        desired[:, 7] = -acceleration
    return desired


def run_session(model: pathlib.Path, ticks: int) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    session = DynamicAdvanceSession(
        str(model), ["left_wheel_center", "right_wheel_center"]
    )
    dof = session.dof
    generalized_dof = session.generalized_dof
    actuators = session.actuator_count
    samples = session.samples_per_tick
    arrays: dict[str, np.ndarray] = {
        "observed_q": np.zeros((ticks, dof), np.float64),
        "observed_v": np.zeros((ticks, dof), np.float64),
        "root_twist": np.zeros((ticks, 6), np.float64),
        "desired": make_reference(ticks, generalized_dof),
        "sample_position": np.empty((ticks, samples, actuators), np.float64),
        "sample_velocity": np.empty((ticks, samples, actuators), np.float64),
        "sample_acceleration": np.empty((ticks, samples, actuators), np.float64),
        "solved_acceleration": np.empty((ticks, generalized_dof), np.float64),
        "splice_position": np.empty((ticks, actuators), np.float64),
        "splice_velocity": np.empty((ticks, actuators), np.float64),
        "splice_acceleration": np.empty((ticks, actuators), np.float64),
        "admitted_effort": np.empty((ticks, actuators), np.float64),
        "selected_extrema": np.empty((ticks, 3), np.float64),
        "selected_position_headroom": np.empty(ticks, np.float64),
        "admission_flags": np.empty(ticks, np.uint32),
        "primary_collision_distance": np.empty(ticks, np.float64),
        "contingency_collision_distance": np.empty(ticks, np.float64),
        "primary_continuous_clearance": np.empty(ticks, np.float64),
        "contingency_continuous_clearance": np.empty(ticks, np.float64),
        "primary_relative_speed_bound": np.empty(ticks, np.float64),
        "contingency_relative_speed_bound": np.empty(ticks, np.float64),
        "primary_collision_pair": np.empty(ticks, np.int64),
        "primary_collision_time_ns": np.empty(ticks, np.int64),
        "status": np.empty(ticks, np.uint8),
        "selection": np.empty(ticks, np.uint8),
        "solve_status": np.empty(ticks, np.uint8),
        "primary_valid": np.empty(ticks, np.uint8),
        "contingency_valid": np.empty(ticks, np.uint8),
        "previous_expired": np.empty(ticks, np.uint8),
        "dynamics_residual": np.empty(ticks, np.float64),
        "contact_residual": np.empty(ticks, np.float64),
        "command_divergence": np.empty(ticks, np.float64),
        "step_ns": np.empty(ticks, np.uint64),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
    }
    rss_before_kib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    thread_before = time.thread_time_ns()
    wall_before = time.perf_counter_ns()
    session.run_trace(
        0,
        arrays["observed_q"],
        arrays["observed_v"],
        arrays["root_twist"],
        arrays["desired"],
        arrays["sample_position"],
        arrays["sample_velocity"],
        arrays["sample_acceleration"],
        arrays["solved_acceleration"],
        arrays["splice_position"],
        arrays["splice_velocity"],
        arrays["splice_acceleration"],
        arrays["admitted_effort"],
        arrays["selected_extrema"],
        arrays["selected_position_headroom"],
        arrays["admission_flags"],
        arrays["primary_collision_distance"],
        arrays["contingency_collision_distance"],
        arrays["primary_continuous_clearance"],
        arrays["contingency_continuous_clearance"],
        arrays["primary_relative_speed_bound"],
        arrays["contingency_relative_speed_bound"],
        arrays["primary_collision_pair"],
        arrays["primary_collision_time_ns"],
        arrays["status"],
        arrays["selection"],
        arrays["solve_status"],
        arrays["primary_valid"],
        arrays["contingency_valid"],
        arrays["previous_expired"],
        arrays["dynamics_residual"],
        arrays["contact_residual"],
        arrays["command_divergence"],
        arrays["step_ns"],
        arrays["allocation_calls"],
        arrays["allocated_bytes"],
    )
    wall_ns = time.perf_counter_ns() - wall_before
    thread_ns = time.thread_time_ns() - thread_before
    rss_after_kib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    metadata = {
        "dof": dof,
        "generalized_dof": generalized_dof,
        "actuators": actuators,
        "samples_per_tick": samples,
        "array_bytes": int(sum(array.nbytes for array in arrays.values())),
        "thread_cpu_ms": thread_ns / 1e6,
        "wall_ms": wall_ns / 1e6,
        "thread_cpu_to_wall": thread_ns / wall_ns if wall_ns else None,
        "maximum_rss_before_kib": rss_before_kib,
        "maximum_rss_after_kib": rss_after_kib,
        "maximum_rss_growth_kib": max(0, rss_after_kib - rss_before_kib),
    }
    return arrays, metadata


def exact_replay(left: dict[str, np.ndarray], right: dict[str, np.ndarray]) -> bool:
    excluded = {"step_ns", "allocation_calls", "allocated_bytes"}
    return all(
        left[name].dtype == right[name].dtype
        and left[name].shape == right[name].shape
        and left[name].tobytes() == right[name].tobytes()
        for name in left.keys() - excluded
    )


def evaluate(model: pathlib.Path, ticks: int) -> dict[str, Any]:
    # Exclude extension/library lazy initialization from both retained runs.
    run_session(model, min(20, ticks))
    first, first_meta = run_session(model, ticks)
    second, second_meta = run_session(model, ticks)
    splice_position_error = float(
        np.max(np.abs(first["splice_position"][1:] - first["sample_position"][:-1, -1]))
    )
    splice_velocity_error = float(
        np.max(np.abs(first["splice_velocity"][1:] - first["sample_velocity"][:-1, -1]))
    )
    splice_acceleration_error = float(
        np.max(
            np.abs(
                first["splice_acceleration"][1:]
                - first["sample_acceleration"][:-1, -1]
            )
        )
    )
    joint_error = (
        first["solved_acceleration"][:, 6:] - first["desired"][:, 6:]
    )
    step_us = first["step_ns"].astype(np.float64) / 1e3
    hard_residual = np.maximum(first["dynamics_residual"], first["contact_residual"])
    all_finite = all(
        np.all(np.isfinite(first[name]))
        for name in (
            "sample_position",
            "sample_velocity",
            "sample_acceleration",
            "solved_acceleration",
            "admitted_effort",
            "selected_extrema",
            "selected_position_headroom",
            "dynamics_residual",
            "contact_residual",
            "command_divergence",
        )
    )
    replay = exact_replay(first, second)
    passed = bool(
        all_finite
        and replay
        and np.all(first["selection"] == 0)
        and np.all(first["admission_flags"] == 0)
        and np.all(first["status"] <= 1)
        and np.all(first["solve_status"] <= 1)
        and np.all(first["primary_valid"] == 1)
        and np.all(first["contingency_valid"] == 1)
        and np.all(first["previous_expired"] == 0)
        and splice_position_error == 0.0
        and splice_velocity_error == 0.0
        and splice_acceleration_error == 0.0
        and np.max(hard_residual) <= 1e-7
        and np.max(first["allocation_calls"]) == 0
        and np.max(first["allocated_bytes"]) == 0
        and np.max(step_us) < 20_000.0
    )
    timing = distribution(step_us)
    timing["jitter_p99_minus_p50"] = timing["p99"] - timing["p50"]
    return {
        "status": "pass" if passed else "fail",
        "scope": "policy-free, physics-free observed-state queries and actuator command synthesis",
        "model": str(model),
        "ticks": ticks,
        "control_horizon_ms": 20.0,
        "sample_period_ms": 1.0,
        "layout": first_meta,
        "second_replay_layout": second_meta,
        "selection_counts": {
            "primary": int(np.sum(first["selection"] == 0)),
            "contingency": int(np.sum(first["selection"] == 1)),
            "rejected": int(np.sum(first["selection"] == 2)),
        },
        "status_counts": {str(code): int(np.sum(first["status"] == code)) for code in np.unique(first["status"])},
        "solve_status_counts": {str(code): int(np.sum(first["solve_status"] == code)) for code in np.unique(first["solve_status"])},
        "d1_complete_semantic_output_bytes_exact": replay,
        "splice": {
            "maximum_position_error": splice_position_error,
            "maximum_velocity_error": splice_velocity_error,
            "maximum_acceleration_error": splice_acceleration_error,
        },
        "validation": {
            "all_primary_valid": bool(np.all(first["primary_valid"] == 1)),
            "all_contingency_valid": bool(np.all(first["contingency_valid"] == 1)),
            "expired_plan_count": int(np.sum(first["previous_expired"])),
            "all_numeric_outputs_finite": all_finite,
            "maximum_analytic_velocity": float(np.max(first["selected_extrema"][:, 0])),
            "maximum_analytic_acceleration": float(np.max(first["selected_extrema"][:, 1])),
            "maximum_analytic_jerk": float(np.max(first["selected_extrema"][:, 2])),
            "minimum_joint_position_headroom": float(np.min(first["selected_position_headroom"])),
            "nonzero_admission_flag_count": int(np.count_nonzero(first["admission_flags"])),
        },
        "wbc": {
            "joint_acceleration_tracking_rms": float(np.sqrt(np.mean(joint_error * joint_error))),
            "joint_acceleration_tracking_max_abs": float(np.max(np.abs(joint_error))),
            "maximum_dynamics_residual": float(np.max(first["dynamics_residual"])),
            "maximum_contact_acceleration_residual": float(np.max(first["contact_residual"])),
            "maximum_hard_residual": float(np.max(hard_residual)),
            "maximum_admitted_effort_abs": float(np.max(np.abs(first["admitted_effort"]))),
        },
        "command_consequence": {
            "maximum_position_abs": float(np.max(np.abs(first["sample_position"]))),
            "maximum_velocity_abs": float(np.max(np.abs(first["sample_velocity"]))),
            "maximum_acceleration_abs": float(np.max(np.abs(first["sample_acceleration"]))),
            "maximum_observed_command_position_divergence": float(np.max(first["command_divergence"])),
            "final_observed_command_position_divergence": float(first["command_divergence"][-1]),
            "interpretation": "command-space consequence only; no plant state was integrated",
        },
        "execution": {
            "step_us": timing,
            "deadline_us": 20_000.0,
            "deadline_overrun_count": int(np.sum(step_us > 20_000.0)),
            "maximum_over_execution_us": float(np.max(step_us - 20_000.0)),
            "allocation_call_maximum": int(np.max(first["allocation_calls"])),
            "allocated_byte_maximum": int(np.max(first["allocated_bytes"])),
        },
    }


def markdown(metrics: dict[str, Any]) -> str:
    audit = metrics["audit"]
    layout = audit["layout"]
    execution = audit["execution"]
    timing = execution["step_us"]
    wbc = audit["wbc"]
    splice = audit["splice"]
    validation = audit["validation"]
    consequence = audit["command_consequence"]
    return f"""# Bonesaw integrated floating CPU advance · r99

## Outcome

**{audit['status'].upper()}.** R99 composes the admitted strict floating WBC query with explicit actuator mapping, exact commanded-state splice, primary and braking-contingency quintics, analytic segment validation, and one contiguous 20×1 ms actuator block. Rust owns the complete transaction; Python owns corpus construction, arrays, statistics, and this report.

This is intentionally policy- and physics-free. Every tick consumes an authored observed state. The command consequence is measured, but it is never fed back as if it were a simulated robot.

## Transaction and authority boundary

{table(['stage', 'retained behavior'], [
    ['Observed authority query', 'Upkie floating WBC with two RollingPoint wheel contacts'],
    ['Admission', 'only Solved/SolvedWithSlack and analytically valid segment may select primary'],
    ['Actuation', 'generalized joint acceleration and effort mapped through immutable CompiledActuation'],
    ['Splice', 'new segment begins at exact previous commanded position/velocity/acceleration'],
    ['Contingency', 'independently synthesized braking segment; feed-forward effort becomes exactly zero'],
    ['Dense output', '20 samples at deterministic 1 ms spacing per 20 ms transaction'],
    ['Plant boundary', 'no state integration, contact response, estimator, or policy'],
])}

The example authority stack therefore gains a distinct **command synthesis and admission** row. WBC feasibility, actuator-space effort, segment validity, compute budget, and observed/commanded divergence remain separate signals.

## Corpus and fixed layout

{table(['signal', 'value'], [
    ['reference model', audit['model']],
    ['ticks / authored duration', f"{audit['ticks']} / {audit['ticks'] * 0.02:.3f} s"],
    ['joint / generalized / actuator coordinates', f"{layout['dof']} / {layout['generalized_dof']} / {layout['actuators']}"],
    ['samples per tick', layout['samples_per_tick']],
    ['Python-owned corpus/output bytes', layout['array_bytes']],
    ['primary / contingency / rejected', f"{audit['selection_counts']['primary']} / {audit['selection_counts']['contingency']} / {audit['selection_counts']['rejected']}"],
    ['D1 complete semantic output bytes exact', audit['d1_complete_semantic_output_bytes_exact']],
])}

The reference is a five-cycle, 2 s cosine-derived acceleration command over the first two joint coordinates. Both observations and root twist remain zero by construction, so the run asks only: what command block does this state-local WBC transaction emit?

## Exact splice and segment validity

{table(['gate', 'result'], [
    ['position splice maximum absolute error', splice['maximum_position_error']],
    ['velocity splice maximum absolute error', splice['maximum_velocity_error']],
    ['acceleration splice maximum absolute error', splice['maximum_acceleration_error']],
    ['all primary segments analytically valid', validation['all_primary_valid']],
    ['all contingency segments analytically valid', validation['all_contingency_valid']],
    ['expired prior plans', validation['expired_plan_count']],
    ['maximum analytic |velocity|', validation['maximum_analytic_velocity']],
    ['maximum analytic |acceleration|', validation['maximum_analytic_acceleration']],
    ['maximum analytic |jerk|', validation['maximum_analytic_jerk']],
    ['minimum analytic joint-position headroom', validation['minimum_joint_position_headroom']],
    ['ticks with nonzero admission flags', validation['nonzero_admission_flag_count']],
])}

The splice comparisons are against the prior block's exact twentieth sample and are byte-equivalent as f64 values. Analytic extrema come from polynomial roots, not a sampling-only approximation.

## WBC tracking and physical invariants

{table(['signal', 'result'], [
    ['joint-acceleration objective RMS', wbc['joint_acceleration_tracking_rms']],
    ['joint-acceleration objective maximum absolute error', wbc['joint_acceleration_tracking_max_abs']],
    ['maximum floating-dynamics residual', wbc['maximum_dynamics_residual']],
    ['maximum contact-acceleration residual', wbc['maximum_contact_acceleration_residual']],
    ['maximum hard residual', wbc['maximum_hard_residual']],
    ['maximum admitted actuator effort magnitude', wbc['maximum_admitted_effort_abs']],
])}

Tracking residual is not hidden when dynamics/contact consume authority. It is an objective consequence, while the two hard residuals remain the admission evidence.

## Command consequence without a plant

{table(['signal', 'result'], [
    ['maximum command position magnitude', consequence['maximum_position_abs']],
    ['maximum command velocity magnitude', consequence['maximum_velocity_abs']],
    ['maximum sampled acceleration magnitude', consequence['maximum_acceleration_abs']],
    ['maximum observed/commanded position divergence', consequence['maximum_observed_command_position_divergence']],
    ['final observed/commanded position divergence', consequence['final_observed_command_position_divergence']],
])}

The divergence is supposed to be visible: observations remain fixed while commands evolve. No tracking-success, stability, disturbance-recovery, power, or reliability claim is inferred from this trace.

## Time, jitter, CPU, memory, and allocation

{table(['metric', 'result'], [
    ['mean / p50 / p95 / p99 / max step µs', f"{timing['mean']:.3f} / {timing['p50']:.3f} / {timing['p95']:.3f} / {timing['p99']:.3f} / {timing['maximum']:.3f}"],
    ['p99 − p50 jitter µs', timing['jitter_p99_minus_p50']],
    ['20 ms deadline overruns', execution['deadline_overrun_count']],
    ['maximum over-execution relative to 20 ms µs', execution['maximum_over_execution_us']],
    ['maximum Rust allocation calls / bytes per step', f"{execution['allocation_call_maximum']} / {execution['allocated_byte_maximum']}"],
    ['calling-thread CPU / wall ms', f"{layout['thread_cpu_ms']:.3f} / {layout['wall_ms']:.3f}"],
    ['calling-thread CPU / wall ratio', layout['thread_cpu_to_wall']],
    ['maximum RSS before / after / growth KiB', f"{layout['maximum_rss_before_kib']} / {layout['maximum_rss_after_kib']} / {layout['maximum_rss_growth_kib']}"],
])}

These are untrimmed same-process release-build measurements of the Rust transaction inside the GIL-detached boundary, not target-hardware WCET. PyO3 detach/reattach bookkeeping is intentionally outside the core allocation counter and is represented in the enclosing calling-thread CPU/wall measurement. Array bytes are explicit Python corpus/output storage; RSS is a process high-water mark and therefore not attributed solely to Rust.

## Remaining admission work

The transaction still needs observed root pose/twist arrays beyond this zero-root eval adapter, effort or impedance feed-forward sampling as an explicit command schema, collision validation on the dynamic segment, state-history ingest, frame snapshots, typed program/input epochs in every exported block, and long recorded-replay/fault corpora. Closed-loop tracking still requires a separately named external plant or hardware run.
"""


def main() -> None:
    args = parse_args()
    model = pathlib.Path(args.model)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    audit = evaluate(model, args.ticks)
    sources = [
        pathlib.Path("crates/bonesaw-core/src/dynamic_controller.rs"),
        pathlib.Path("crates/bonesaw-core/src/dynamic_wbc.rs"),
        pathlib.Path("crates/bonesaw-core/src/trajectory.rs"),
        pathlib.Path("crates/bonesaw-py/src/lib.rs"),
        pathlib.Path(__file__),
    ]
    metrics = {
        "schema": 1,
        "revision": "dynamic-advance-r99",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "artifacts": {str(path): sha256(path) for path in [model, *sources]},
        "audit": audit,
    }
    (output / "dynamic-advance-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    report = markdown(metrics)
    (output / "DYNAMIC_ADVANCE_AUDIT.md").write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.parent.mkdir(parents=True, exist_ok=True)
    web_report.write_text(render_report_html(report, title="Bonesaw integrated floating CPU advance · r99"))
    print(json.dumps(metrics, indent=2))
    if audit["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
