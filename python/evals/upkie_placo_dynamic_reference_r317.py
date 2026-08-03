#!/usr/bin/env python3
"""Isolated Bonesaw/PlaCo floating-dynamics comparison on frozen Upkie states."""

from __future__ import annotations

import argparse
import gc
import hashlib
import importlib.metadata
import json
import os
import pathlib
import platform
import resource
import subprocess
import sys
import time
from typing import Any

import numpy as np
import psutil

from cpu_reference_report import distribution, markdown_table, render_report_html


REPEATS = 8
CONTACT_FRAMES = ("left_wheel_center", "right_wheel_center")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument(
        "--input",
        default="benchmarks/results/upkie-state-local-wbc-r123/upkie-state-local-wbc-raw.npz",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/upkie-placo-dynamic-reference-r317"
    )
    parser.add_argument("--web-report", default="web/UPKIE_PLACO_DYNAMIC_REFERENCE_R317.html")
    parser.add_argument("--worker", choices=("bonesaw", "placo"))
    return parser.parse_args()


def corpus_from_raw(path: pathlib.Path) -> dict[str, np.ndarray]:
    source = np.load(path)
    return {
        name: np.ascontiguousarray(source[f"input_{name}"])
        for name in (
            "root_positions",
            "root_velocities",
            "root_accelerations",
            "q",
            "v",
            "joint_accelerations",
            "contact_active",
            "slip",
        )
    }


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def rss_bytes() -> int:
    return psutil.Process().memory_info().rss


def gc_collections() -> int:
    return sum(item["collections"] for item in gc.get_stats())


def resource_summary(
    *,
    wall_start_ns: int,
    cpu_start_ns: int,
    rss_before: int,
    peak_before_kib: int,
    gc_before: int,
    queries: int,
) -> dict[str, Any]:
    wall_ns = time.perf_counter_ns() - wall_start_ns
    cpu_ns = time.process_time_ns() - cpu_start_ns
    rss_after = rss_bytes()
    peak_after_kib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return {
        "queries": queries,
        "wall_seconds": wall_ns / 1e9,
        "cpu_seconds": cpu_ns / 1e9,
        "cpu_to_wall": cpu_ns / wall_ns,
        "queries_per_second": queries * 1e9 / wall_ns,
        "full_query_mean_us": wall_ns / queries / 1000.0,
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": rss_after - rss_before,
        "peak_rss_before_bytes": peak_before_kib * 1024,
        "peak_rss_after_bytes": peak_after_kib * 1024,
        "gc_collections": gc_collections() - gc_before,
    }


def reference_acceleration(corpus: dict[str, np.ndarray]) -> np.ndarray:
    ticks = len(corpus["q"])
    return np.column_stack(
        (
            np.zeros((ticks, 3), dtype=np.float64),
            corpus["root_accelerations"],
            corpus["joint_accelerations"],
        )
    )


def tracking_summary(qdd: np.ndarray, reference: np.ndarray) -> dict[str, Any]:
    groups = {
        "root_angular_rad_s2": slice(0, 3),
        "root_horizontal_m_s2": slice(3, 5),
        "root_height_m_s2": slice(5, 6),
        "joint_rad_s2": slice(6, 12),
    }
    result: dict[str, Any] = {}
    for name, columns in groups.items():
        error = qdd[:, columns] - reference[:, columns]
        norms = np.linalg.norm(error, axis=1)
        result[name] = {
            "rms": float(np.sqrt(np.mean(error * error))),
            "norm": distribution(norms),
        }
    return result


def timing_summary(step_ns: np.ndarray) -> dict[str, Any]:
    latency_us = step_ns.astype(np.float64) / 1000.0
    jitter_us = np.abs(np.diff(latency_us))
    return {
        "latency_us": distribution(latency_us),
        "adjacent_jitter_us": distribution(jitter_us),
        "deadline_misses": {
            f"{limit / 1000:g}ms": int(np.sum(latency_us > limit))
            for limit in (500, 1000, 2000, 5000, 10000, 20000)
        },
    }


def run_bonesaw(args: argparse.Namespace) -> None:
    from upkie_state_local_wbc_report import (
        allocate_outputs,
        new_session,
        rolling_descriptors,
        run_case,
    )

    model = pathlib.Path(args.model).resolve()
    corpus = corpus_from_raw(pathlib.Path(args.input))
    session = new_session(model)
    joint_names = list(session.joint_names)
    frame_names = list(session.frame_names)
    frame_ids = np.asarray([frame_names.index(name) for name in CONTACT_FRAMES], np.int64)
    modes, coordinates, coefficients = rolling_descriptors(
        model, joint_names, list(CONTACT_FRAMES)
    )
    outputs = allocate_outputs(session, len(corpus["q"]))
    run_case(session, corpus, frame_ids, modes, coordinates, coefficients, outputs)
    gc.collect()
    gc_before = gc_collections()
    rss_before = rss_bytes()
    peak_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wall_start = time.perf_counter_ns()
    cpu_start = time.process_time_ns()
    timings = np.empty(REPEATS * len(corpus["q"]), dtype=np.uint64)
    allocation_calls = 0
    allocated_bytes = 0
    for repeat in range(REPEATS):
        run_case(session, corpus, frame_ids, modes, coordinates, coefficients, outputs)
        start = repeat * len(corpus["q"])
        timings[start : start + len(corpus["q"])] = outputs["step_ns"]
        allocation_calls += int(np.sum(outputs["allocation_calls"]))
        allocated_bytes += int(np.sum(outputs["allocated_bytes"]))
    resources = resource_summary(
        wall_start_ns=wall_start,
        cpu_start_ns=cpu_start,
        rss_before=rss_before,
        peak_before_kib=peak_before,
        gc_before=gc_before,
        queries=len(timings),
    )
    resources["native_allocation_calls"] = allocation_calls
    resources["native_allocated_bytes"] = allocated_bytes
    qdd = outputs["generalized_acceleration"].copy()
    metrics = {
        "implementation": "Bonesaw Rust floating rolling WBC",
        "version": importlib.metadata.version("bonesaw"),
        "contact_model": "two nonholonomic RollingWheel YZ+rolling-X rows",
        "samples": len(qdd),
        "success_count": int(np.sum(np.isin(outputs["status"], (0, 1)))),
        "status_counts": {
            str(int(value)): int(np.sum(outputs["status"] == value))
            for value in np.unique(outputs["status"])
        },
        "tracking": tracking_summary(qdd, reference_acceleration(corpus)),
        "timing": timing_summary(timings),
        "resources": resources,
    }
    output = pathlib.Path(args.output)
    np.savez_compressed(
        output / "bonesaw-raw.npz",
        generalized_acceleration=qdd,
        actuator_torque=outputs["actuator_torque"],
        contact_force=outputs["contact_force_basis"],
        status=outputs["status"],
        step_ns=timings,
    )
    (output / "bonesaw-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")


def configure_placo(model: pathlib.Path, corpus: dict[str, np.ndarray]):
    import placo

    source = model.read_text().replace(
        "package://upkie_description", str(model.parent.resolve())
    )
    robot = placo.RobotWrapper("", placo.Flags.ignore_collisions, source)
    solver = placo.DynamicsSolver(robot)
    solver.dt = 0.02
    solver.mask_fbase(False)
    solver.damping = 1e-8
    contact_tasks = []
    contacts = []
    set_placo_state(robot, corpus, 0)
    for frame in CONTACT_FRAMES:
        task = solver.add_position_task(frame, robot.get_T_world_frame(frame)[:3, 3])
        task.configure(f"contact_{frame}", "soft", 1.0)
        task.kp = 1e5
        contact = solver.add_unilateral_point_contact(task)
        contact.mu = 0.8
        contact_tasks.append(task)
        contacts.append(contact)
    root_orientation = solver.add_orientation_task("base", np.eye(3))
    root_orientation.configure("root_angular", "hard")
    root_orientation.kp = 0.0
    root_orientation.kd = 0.0
    root_height = solver.add_position_task("base", corpus["root_positions"][0])
    root_height.mask.set_axises("z", "world")
    root_height.configure("root_height", "hard")
    root_height.kp = 0.0
    root_height.kd = 0.0
    root_horizontal = solver.add_position_task("base", corpus["root_positions"][0])
    root_horizontal.mask.set_axises("xy", "world")
    root_horizontal.configure("root_horizontal", "soft", 10.0)
    root_horizontal.kp = 0.0
    root_horizontal.kd = 0.0
    joints = solver.add_joints_task()
    joints.configure("joint_acceleration", "soft", 1.0)
    joints.kp = 0.0
    joints.kd = 0.0
    for name in robot.joint_names():
        solver.set_torque_limit(name, 2000.0)
    solver.enable_torque_limits(True)
    return (
        robot,
        solver,
        contact_tasks,
        contacts,
        root_orientation,
        root_height,
        root_horizontal,
        joints,
    )


def set_placo_state(robot: Any, corpus: dict[str, np.ndarray], tick: int) -> None:
    transform = np.eye(4)
    transform[:3, 3] = corpus["root_positions"][tick]
    robot.set_T_world_fbase(transform)
    for name, q, velocity in zip(
        robot.joint_names(), corpus["q"][tick], corpus["v"][tick], strict=True
    ):
        robot.set_joint(name, float(q))
        robot.set_joint_velocity(name, float(velocity))
    robot.state.qd[:3] = corpus["root_velocities"][tick]
    robot.state.qd[3:6] = 0.0
    robot.update_kinematics()


def solve_placo_tick(configured: tuple[Any, ...], corpus: dict[str, np.ndarray], tick: int):
    (
        robot,
        solver,
        contact_tasks,
        contacts,
        root_orientation,
        root_height,
        root_horizontal,
        joints,
    ) = configured
    set_placo_state(robot, corpus, tick)
    for frame, task in zip(CONTACT_FRAMES, contact_tasks, strict=True):
        task.target_world = robot.get_T_world_frame(frame)[:3, 3]
        task.dtarget_world = np.zeros(3)
        task.ddtarget_world = np.zeros(3)
    root_orientation.R_world_frame = np.eye(3)
    root_orientation.omega_world = np.zeros(3)
    root_orientation.domega_world = np.zeros(3)
    for task in (root_height, root_horizontal):
        task.target_world = corpus["root_positions"][tick]
        task.dtarget_world = corpus["root_velocities"][tick]
        task.ddtarget_world = corpus["root_accelerations"][tick]
    for name, q, velocity, acceleration in zip(
        robot.joint_names(),
        corpus["q"][tick],
        corpus["v"][tick],
        corpus["joint_accelerations"][tick],
        strict=True,
    ):
        joints.set_joint(name, float(q), float(velocity), float(acceleration))
    started = time.perf_counter_ns()
    result = solver.solve(False)
    elapsed = time.perf_counter_ns() - started
    if not result.success:
        return elapsed, None, None, None
    permutation = np.asarray([3, 4, 5, 0, 1, 2, *range(6, 12)], dtype=np.int64)
    force = np.stack([contact.wrench for contact in contacts])
    return elapsed, result.qdd[permutation].copy(), result.tau[6:].copy(), force


def run_placo(args: argparse.Namespace) -> None:
    model = pathlib.Path(args.model).resolve()
    corpus = corpus_from_raw(pathlib.Path(args.input))
    configured = configure_placo(model, corpus)
    ticks = len(corpus["q"])
    for tick in range(ticks):
        solve_placo_tick(configured, corpus, tick)
    gc.collect()
    gc_before = gc_collections()
    rss_before = rss_bytes()
    peak_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wall_start = time.perf_counter_ns()
    cpu_start = time.process_time_ns()
    timing = np.empty(REPEATS * ticks, dtype=np.uint64)
    qdd = np.full((ticks, 12), np.nan, dtype=np.float64)
    torque = np.full((ticks, 6), np.nan, dtype=np.float64)
    force = np.full((ticks, 2, 3), np.nan, dtype=np.float64)
    success = np.zeros(ticks, dtype=np.uint8)
    for repeat in range(REPEATS):
        for tick in range(ticks):
            elapsed, tick_qdd, tick_torque, tick_force = solve_placo_tick(
                configured, corpus, tick
            )
            timing[repeat * ticks + tick] = elapsed
            if tick_qdd is not None:
                success[tick] = 1
                qdd[tick] = tick_qdd
                torque[tick] = tick_torque
                force[tick] = tick_force
    resources = resource_summary(
        wall_start_ns=wall_start,
        cpu_start_ns=cpu_start,
        rss_before=rss_before,
        peak_before_kib=peak_before,
        gc_before=gc_before,
        queries=len(timing),
    )
    finite = success.astype(bool)
    metrics = {
        "implementation": "PlaCo floating DynamicsSolver",
        "version": importlib.metadata.version("placo"),
        "contact_model": "two stiff unilateral material point contacts at wheel centers",
        "samples": ticks,
        "success_count": int(np.sum(success)),
        "tracking": tracking_summary(qdd[finite], reference_acceleration(corpus)[finite]),
        "timing": timing_summary(timing),
        "resources": resources,
    }
    output = pathlib.Path(args.output)
    np.savez_compressed(
        output / "placo-raw.npz",
        generalized_acceleration=qdd,
        actuator_torque=torque,
        contact_force=force,
        success=success,
        step_ns=timing,
    )
    (output / "placo-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")


def divergence(bonesaw: Any, placo: Any) -> dict[str, Any]:
    valid = placo["success"].astype(bool)
    result = {}
    for name in ("generalized_acceleration", "actuator_torque"):
        delta = placo[name][valid] - bonesaw[name][valid]
        result[name] = {
            "rms": float(np.sqrt(np.mean(delta * delta))),
            "maximum_absolute": float(np.max(np.abs(delta))),
        }
    normal_delta = placo["contact_force"][valid, :, 2] - bonesaw["contact_force"][valid, :, 2]
    result["contact_normal_force"] = {
        "rms": float(np.sqrt(np.mean(normal_delta * normal_delta))),
        "maximum_absolute": float(np.max(np.abs(normal_delta))),
    }
    return result


def execution_windows(
    corpus: dict[str, np.ndarray], bonesaw: Any, placo: Any, width: int = 32
) -> list[dict[str, Any]]:
    reference = reference_acceleration(corpus)
    rows = []
    for start in range(0, len(reference), width):
        stop = min(len(reference), start + width)
        row: dict[str, Any] = {"start": start, "stop": stop}
        for name, raw in (("bonesaw", bonesaw), ("placo", placo)):
            qdd = raw["generalized_acceleration"][start:stop]
            torque = raw["actuator_torque"][start:stop]
            force = raw["contact_force"][start:stop]
            latency_us = raw["step_ns"][start:stop].astype(np.float64) / 1000.0
            valid = np.all(np.isfinite(qdd), axis=1)
            horizontal = qdd[valid, 3:5] - reference[start:stop][valid, 3:5]
            joint = qdd[valid, 6:] - reference[start:stop][valid, 6:]
            row[name] = {
                "successful": int(np.sum(valid)),
                "latency_p50_us": float(np.percentile(latency_us, 50)),
                "latency_p99_us": float(np.percentile(latency_us, 99)),
                "root_horizontal_rms": float(np.sqrt(np.mean(horizontal * horizontal))),
                "joint_rms": float(np.sqrt(np.mean(joint * joint))),
                "maximum_abs_torque_nm": float(np.nanmax(np.abs(torque))),
                "mean_normal_force_n": float(np.nanmean(force[:, :, 2])),
            }
        rows.append(row)
    return rows


def report(metrics: dict[str, Any]) -> str:
    rows = []
    for key in ("bonesaw", "placo"):
        item = metrics[key]
        latency = item["timing"]["latency_us"]
        jitter = item["timing"]["adjacent_jitter_us"]
        resources = item["resources"]
        rows.append(
            [
                key.title(),
                f"{item['success_count']}/{item['samples']}",
                f"{latency['p50']:.1f} / {latency['p99']:.1f} / {latency['maximum']:.1f}",
                f"{jitter['p99']:.1f}",
                f"{resources['full_query_mean_us']:.1f}",
                f"{resources['queries_per_second']:.0f}",
                f"{resources['peak_rss_after_bytes'] / 1048576:.2f}",
                str(resources["gc_collections"]),
            ]
        )
    tracking_rows = []
    for signal in metrics["bonesaw"]["tracking"]:
        tracking_rows.append(
            [
                signal,
                f"{metrics['bonesaw']['tracking'][signal]['rms']:.6f}",
                f"{metrics['placo']['tracking'][signal]['rms']:.6f}",
            ]
        )
    divergence_rows = [
        [name, f"{value['rms']:.6f}", f"{value['maximum_absolute']:.6f}"]
        for name, value in metrics["divergence"].items()
    ]
    resource_rows = []
    for key in ("bonesaw", "placo"):
        resources = metrics[key]["resources"]
        resource_rows.append(
            [
                key.title(),
                f"{resources['wall_seconds']:.6f}",
                f"{resources['cpu_seconds']:.6f}",
                f"{resources['cpu_to_wall']:.6f}",
                f"{resources['rss_before_bytes'] / 1048576:.2f}",
                f"{resources['rss_after_bytes'] / 1048576:.2f}",
                f"{resources['rss_delta_bytes'] / 1048576:.4f}",
            ]
        )
    provenance_rows = [[name, value] for name, value in metrics["provenance"].items()]
    window_rows = []
    for window in metrics["execution_windows"]:
        for implementation in ("bonesaw", "placo"):
            item = window[implementation]
            window_rows.append(
                [
                    f"{window['start']}–{window['stop'] - 1}",
                    implementation.title(),
                    f"{item['latency_p50_us']:.1f} / {item['latency_p99_us']:.1f}",
                    f"{item['root_horizontal_rms']:.4f}",
                    f"{item['joint_rms']:.2f}",
                    f"{item['maximum_abs_torque_nm']:.3f}",
                    f"{item['mean_normal_force_n']:.2f}",
                ]
            )
    return f"""# Upkie floating reference comparison · R317

## Result

This is the first isolated full-body Upkie comparison against PlaCo's floating
inverse-dynamics implementation. Both workers consume the same 256 frozen
states and acceleration targets; policy, integration, simulator, and physics
rollout are all absent. Bonesaw retains its Rust nonholonomic rolling-contact
model. PlaCo uses its independently implemented unilateral material-point
contacts at the two wheel centers.

**Reference availability: PASS. Exact controller parity: NOT CLAIMED.** The
contact laws are intentionally visible and materially different, so output
divergence is evidence rather than a failure hidden by tuning.

## CPU, memory, jitter, and success

{"\n".join(markdown_table(['implementation', 'successful states', 'solve p50 / p99 / max µs', 'jitter p99 µs', 'full query mean µs', 'queries/s', 'peak RSS MiB', 'GC'], rows))}

Bonesaw's measured solve loop reports
`{metrics['bonesaw']['resources']['native_allocation_calls']}` Rust allocation
calls and `{metrics['bonesaw']['resources']['native_allocated_bytes']}` bytes.
Timing and RSS are isolated-process observations on this host; they are not a
universal speedup claim. Bonesaw's full-query mean amortizes one fixed-shape
PyO3 batch boundary over 256 rows; PlaCo's includes Python state/task updates
for every row. Solve-only and full-query columns are therefore both retained.

{"\n".join(markdown_table(['implementation', 'wall s', 'CPU s', 'CPU/wall', 'RSS before MiB', 'RSS after MiB', 'RSS delta MiB'], resource_rows))}

## Shared-target tracking

{"\n".join(markdown_table(['acceleration target', 'Bonesaw RMS', 'PlaCo RMS'], tracking_rows))}

## Output divergence

{"\n".join(markdown_table(['quantity', 'RMS delta', 'maximum absolute delta'], divergence_rows))}

The normal-force comparison is the closest contact quantity shared by both
models. Tangential force is not declared equivalent: Bonesaw's force belongs
to a rolling basis, while PlaCo anchors a material wheel-center point.

## Execution over corpus order

{"\n".join(markdown_table(['ticks', 'implementation', 'solve p50 / p99 µs', 'root XY RMS', 'joint RMS', 'max |τ| Nm', 'mean Fz N'], window_rows))}

Each row is a lossless 32-state partition of the frozen corpus. The window
table keeps phase-local latency, tracking, torque, and normal-load changes
visible instead of reducing the run to one aggregate score.

## Boundary and next gate

- Corpus: `{metrics['input']}`; {metrics['samples']} independent rows.
- Bonesaw: exact rolling rows, hierarchical task stack, preallocated Rust hot loop.
- PlaCo: independent weighted QP, floating dynamics, unilateral frictional point contacts.
- No learned policy, physics engine, integration, state propagation, or reset behavior is in this comparison.
- The next parity gate is an independent solver with the same nonholonomic
  wheel row and contact-force basis, followed separately by a shared closed-loop
  plant corpus. Hardware timing and thermal/reliability authority remain open.

## Provenance

{"\n".join(markdown_table(['artifact', 'version or SHA-256'], provenance_rows))}
"""


def parent(args: argparse.Namespace) -> None:
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    script = pathlib.Path(__file__).resolve()
    for worker in ("bonesaw", "placo"):
        subprocess.run(
            [
                sys.executable,
                str(script),
                "--worker",
                worker,
                "--model",
                args.model,
                "--input",
                args.input,
                "--output",
                args.output,
            ],
            check=True,
            cwd=pathlib.Path.cwd(),
            env={**os.environ, "PYTHONPATH": str(pathlib.Path("python/evals").resolve())},
        )
    bonesaw_metrics = json.loads((output / "bonesaw-metrics.json").read_text())
    placo_metrics = json.loads((output / "placo-metrics.json").read_text())
    bonesaw_raw = np.load(output / "bonesaw-raw.npz")
    placo_raw = np.load(output / "placo-raw.npz")
    metrics = {
        "schema_version": 1,
        "revision": "upkie-placo-dynamic-reference-r317",
        "host": {"platform": platform.platform(), "python": sys.version},
        "input": args.input,
        "samples": int(bonesaw_metrics["samples"]),
        "policy": None,
        "physics": None,
        "integration": None,
        "bonesaw": bonesaw_metrics,
        "placo": placo_metrics,
        "divergence": divergence(bonesaw_raw, placo_raw),
        "execution_windows": execution_windows(
            corpus_from_raw(pathlib.Path(args.input)), bonesaw_raw, placo_raw
        ),
        "provenance": {
            "Bonesaw": bonesaw_metrics["version"],
            "PlaCo": placo_metrics["version"],
            "model": sha256(pathlib.Path(args.model)),
            "input corpus": sha256(pathlib.Path(args.input)),
            "evaluator": sha256(script),
            "Bonesaw raw": sha256(output / "bonesaw-raw.npz"),
            "PlaCo raw": sha256(output / "placo-raw.npz"),
        },
    }
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    markdown = report(metrics)
    (output / "UPKIE_PLACO_DYNAMIC_REFERENCE_R317.md").write_text(markdown)
    pathlib.Path(args.web_report).write_text(
        render_report_html(markdown, title="Upkie floating reference comparison · R317")
    )
    print(json.dumps(metrics, indent=2))


def main() -> None:
    args = parse_args()
    if args.worker == "bonesaw":
        run_bonesaw(args)
    elif args.worker == "placo":
        run_placo(args)
    else:
        parent(args)


if __name__ == "__main__":
    main()
