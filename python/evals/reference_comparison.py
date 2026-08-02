#!/usr/bin/env python3
"""Reproducible Bonesaw ↔ PlaCo WBC comparison.

The parent process builds one fixed NumPy input corpus. Separate worker
processes then load exactly one implementation each, execute the corpus, and
write raw per-step traces plus resource metrics. The parent renders JSON and a
Markdown report. This keeps experiment logic in Python while the Bonesaw hot
loop remains in Rust.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import importlib.metadata
import json
import math
import pathlib
import platform
import resource
import subprocess
import sys
import time
import tracemalloc
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np
import psutil

from cmu_mocap import retarget_subject_37_walk


SCENARIOS = ("end_effector_reach", "bimanual_priority_conflict", "walking_motion_retarget")
DT = 0.02


@dataclass(frozen=True)
class Scenario:
    name: str
    frame_names: tuple[str, ...]
    priorities: tuple[int, ...]
    weights: tuple[float, ...]


SCENARIO_SPECS = {
    "end_effector_reach": Scenario(
        "end_effector_reach",
        ("left_hand", "right_hand"),
        (2, 2),
        (1.0, 1.0),
    ),
    "bimanual_priority_conflict": Scenario(
        "bimanual_priority_conflict",
        ("left_hand", "right_hand"),
        (2, 2),
        (1.0, 1.0),
    ),
    "walking_motion_retarget": Scenario(
        "walking_motion_retarget",
        ("left_foot", "right_foot", "left_hand", "right_hand"),
        (2, 2, 3, 3),
        (1.0, 1.0, 0.35, 0.35),
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/toy_humanoid.urdf")
    parser.add_argument("--upkie-model", default="models/upkie/upkie.urdf")
    parser.add_argument("--ticks", type=int, default=5_000)
    parser.add_argument("--warmup", type=int, default=250)
    parser.add_argument("--chunk", type=int, default=256)
    parser.add_argument("--output", default="benchmarks/results/reference-latest")
    parser.add_argument("--cmu-cache", default="benchmarks/cache/cmu-37")
    parser.add_argument(
        "--g1-liftoff",
        default="benchmarks/results/floating-g1-liftoff-latest",
        help="directory containing the green-prefix G1 metrics and raw NPZ",
    )
    parser.add_argument(
        "--g1-transfer",
        default="benchmarks/results/floating-g1-transfer-latest",
        help="directory containing the full-transfer G1 metrics and raw NPZ",
    )
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="re-render from existing worker/native artifacts without retiming them",
    )
    parser.add_argument("--worker", choices=("bonesaw", "placo"))
    parser.add_argument("--input")
    return parser.parse_args()


def standing_posture(joint_names: list[str]) -> np.ndarray:
    values = {
        "left_hip_pitch": -0.15,
        "right_hip_pitch": -0.15,
        "left_knee": 0.3,
        "right_knee": 0.3,
        "left_ankle": -0.15,
        "right_ankle": -0.15,
        "left_elbow": -0.25,
        "right_elbow": 0.25,
        # Official Unitree G1 nominal posture from unitree_rl_gym. The 23-DOF
        # description uses the `_joint` names and needs a bent-knee seed; its
        # all-zero posture is a near-singular fully extended stance.
        "left_hip_pitch_joint": -0.1,
        "right_hip_pitch_joint": -0.1,
        "left_knee_joint": 0.3,
        "right_knee_joint": 0.3,
        "left_ankle_pitch_joint": -0.2,
        "right_ankle_pitch_joint": -0.2,
    }
    q = np.zeros(len(joint_names), dtype=np.float64)
    for name, value in values.items():
        if name in joint_names:
            q[joint_names.index(name)] = value
    return q


def build_input_corpus(
    model: pathlib.Path,
    model_label: str,
    ticks: int,
    output: pathlib.Path,
    cmu_cache: pathlib.Path,
    cmu_source_record: dict[str, Any],
) -> pathlib.Path:
    import bonesaw

    session = bonesaw.ControllerSession(str(model))
    frame_names = list(session.frame_names)
    joint_names = list(session.joint_names)
    q = standing_posture(joint_names)
    origins = np.empty((session.bodies, 3), dtype=np.float64)
    session.frame_positions(q, origins)
    zero_com = np.empty(3, dtype=np.float64)
    session.center_of_mass(np.zeros(session.dof, dtype=np.float64), zero_com)
    arrays: dict[str, np.ndarray] = {
        "standing_q": q,
        "zero_v": np.zeros(session.dof, dtype=np.float64),
    }
    times = np.arange(ticks, dtype=np.float64) * DT
    walking_metadata: dict[str, Any] | None = None

    for name in SCENARIOS:
        spec = SCENARIO_SPECS[name]
        ids = np.asarray([frame_names.index(frame) for frame in spec.frame_names], dtype=np.int64)
        targets = np.empty((ticks, len(ids), 3), dtype=np.float64)
        active = np.ones((ticks, len(ids)), dtype=np.uint8)
        com_targets = np.zeros((ticks, 3), dtype=np.float64)
        com_active = np.zeros(ticks, dtype=np.uint8)
        if name == "end_effector_reach":
            phase = 2.0 * math.pi * 0.25 * times
            for index, side in enumerate((1.0, -1.0)):
                targets[:, index, :] = origins[ids[index]]
                targets[:, index, 0] += 0.10 + 0.04 * np.cos(phase)
                targets[:, index, 1] += side * (0.12 + 0.03 * np.sin(phase))
                targets[:, index, 2] += 0.12 + 0.03 * np.sin(phase)
        elif name == "bimanual_priority_conflict":
            shared = 0.5 * (origins[ids[0]] + origins[ids[1]]) + np.array([0.18, 0.0, 0.12])
            targets[:] = shared
            com_targets[:] = zero_com
            com_active[:] = 1
        else:
            pelvis = frame_names.index("pelvis")
            walking = retarget_subject_37_walk(
                cmu_cache / "37.asf",
                cmu_cache / "37_01.amc",
                origins[ids],
                origins[pelvis],
                ticks,
                DT,
            )
            targets[:] = walking.targets
            arrays[f"{name}_reference_stance"] = walking.stance
            arrays[f"{name}_cadence_scale"] = walking.cadence_scale
            arrays[f"{name}_source_phase_frames"] = walking.source_phase_frames
            walking_metadata = {
                **cmu_source_record,
                **walking.metadata,
            }

        prefix = name
        arrays[f"{prefix}_frame_ids"] = ids
        arrays[f"{prefix}_target_positions"] = targets
        arrays[f"{prefix}_target_velocities"] = np.gradient(
            targets,
            DT,
            axis=0,
            edge_order=2 if ticks >= 3 else 1,
        )
        arrays[f"{prefix}_target_active"] = active
        arrays[f"{prefix}_priorities"] = np.asarray(spec.priorities, dtype=np.uint8)
        arrays[f"{prefix}_weights"] = np.asarray(spec.weights, dtype=np.float64)
        arrays[f"{prefix}_com_targets"] = com_targets
        arrays[f"{prefix}_com_active"] = com_active

    input_path = output / "input-corpus.npz"
    np.savez_compressed(input_path, **arrays)
    manifest = {
        "model": model_label,
        "ticks": ticks,
        "dt_seconds": DT,
        "frame_names": frame_names,
        "joint_names": joint_names,
        "scenarios": {name: asdict(SCENARIO_SPECS[name]) for name in SCENARIOS},
        "walking_reference": walking_metadata,
    }
    (output / "input-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return input_path


def process_snapshot(process: psutil.Process) -> dict[str, int]:
    memory = process.memory_full_info()
    return {
        "rss_bytes": int(memory.rss),
        "uss_bytes": int(getattr(memory, "uss", 0)),
        "pss_bytes": int(getattr(memory, "pss", 0)),
        "vms_bytes": int(memory.vms),
    }


def usage_snapshot(process: psutil.Process) -> dict[str, Any]:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    context = process.num_ctx_switches()
    return {
        "user_cpu_seconds": usage.ru_utime,
        "system_cpu_seconds": usage.ru_stime,
        "minor_faults": usage.ru_minflt,
        "major_faults": usage.ru_majflt,
        "voluntary_context_switches": context.voluntary,
        "involuntary_context_switches": context.involuntary,
        "max_rss_kib": usage.ru_maxrss,
    }


def delta(after: dict[str, Any], before: dict[str, Any]) -> dict[str, Any]:
    return {key: after[key] - before[key] for key in before}


def gc_snapshot() -> dict[str, Any]:
    return {
        "count": list(gc.get_count()),
        "collections": [generation["collections"] for generation in gc.get_stats()],
        "collected": [generation["collected"] for generation in gc.get_stats()],
        "uncollectable": [generation["uncollectable"] for generation in gc.get_stats()],
    }


def gc_delta(after: dict[str, Any], before: dict[str, Any]) -> dict[str, Any]:
    return {
        key: [a - b for a, b in zip(after[key], before[key], strict=True)]
        for key in after
    }


def scenario_arrays(corpus: Any, name: str) -> dict[str, np.ndarray]:
    result = {
        key: corpus[f"{name}_{key}"]
        for key in (
            "frame_ids",
            "target_positions",
            "target_velocities",
            "target_active",
            "priorities",
            "weights",
            "com_targets",
            "com_active",
        )
    }
    for key in ("reference_stance", "cadence_scale", "source_phase_frames"):
        full_key = f"{name}_{key}"
        if full_key in corpus.files:
            result[key] = corpus[full_key]
    return result


def run_bonesaw_worker(args: argparse.Namespace) -> None:
    process = psutil.Process()
    baseline_memory = process_snapshot(process)
    import_started = time.perf_counter_ns()
    import bonesaw
    import_wall_ns = time.perf_counter_ns() - import_started
    import_memory = process_snapshot(process)
    corpus = np.load(args.input)
    model = pathlib.Path(args.model).resolve()
    setup_started = time.perf_counter_ns()
    session = bonesaw.ControllerSession(str(model))
    setup_wall_ns = time.perf_counter_ns() - setup_started
    setup_memory = process_snapshot(process)
    standing = corpus["standing_q"]
    zero_v = corpus["zero_v"]
    raw: dict[str, np.ndarray] = {}
    scenarios: dict[str, Any] = {}

    for name in SCENARIOS:
        arrays = scenario_arrays(corpus, name)
        total_ticks = arrays["target_positions"].shape[0]
        session.reset(standing, zero_v)
        warmup_ticks = min(args.warmup, total_ticks)
        if warmup_ticks:
            warm_q = np.empty((warmup_ticks, session.dof), dtype=np.float64)
            warm_v = np.empty_like(warm_q)
            warm_tracked = np.empty(
                (warmup_ticks, len(arrays["frame_ids"]), 3), dtype=np.float64
            )
            warm_ns = np.empty(warmup_ticks, dtype=np.uint64)
            warm_status = np.empty(warmup_ticks, dtype=np.uint8)
            session.run_trace(
                -warmup_ticks * 20_000_000,
                arrays["frame_ids"],
                arrays["target_positions"][:warmup_ticks],
                arrays["target_velocities"][:warmup_ticks],
                arrays["target_active"][:warmup_ticks],
                arrays["priorities"],
                arrays["weights"],
                arrays["com_targets"][:warmup_ticks],
                arrays["com_active"][:warmup_ticks],
                standing,
                warm_q,
                warm_v,
                warm_tracked,
                warm_ns,
                warm_status,
            )
            session.reset(standing, zero_v)

        q_out = np.empty((total_ticks, session.dof), dtype=np.float64)
        v_out = np.empty_like(q_out)
        tracked = np.empty(
            (total_ticks, len(arrays["frame_ids"]), 3), dtype=np.float64
        )
        step_ns = np.empty(total_ticks, dtype=np.uint64)
        status = np.empty(total_ticks, dtype=np.uint8)
        gc.collect()
        gc_before = gc_snapshot()
        usage_before = usage_snapshot(process)
        memory_before = process_snapshot(process)
        peak_rss = memory_before["rss_bytes"]
        tracemalloc.start()
        wall_started = time.perf_counter_ns()
        cpu_started = time.process_time_ns()
        thread_cpu_started = time.thread_time_ns()
        for start in range(0, total_ticks, args.chunk):
            stop = min(start + args.chunk, total_ticks)
            session.run_trace(
                start * 20_000_000,
                arrays["frame_ids"],
                arrays["target_positions"][start:stop],
                arrays["target_velocities"][start:stop],
                arrays["target_active"][start:stop],
                arrays["priorities"],
                arrays["weights"],
                arrays["com_targets"][start:stop],
                arrays["com_active"][start:stop],
                standing,
                q_out[start:stop],
                v_out[start:stop],
                tracked[start:stop],
                step_ns[start:stop],
                status[start:stop],
            )
            peak_rss = max(peak_rss, process.memory_info().rss)
        thread_cpu_ns = time.thread_time_ns() - thread_cpu_started
        cpu_ns = time.process_time_ns() - cpu_started
        wall_ns = time.perf_counter_ns() - wall_started
        traced_current, traced_peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        memory_after = process_snapshot(process)
        usage_after = usage_snapshot(process)
        gc_after = gc_snapshot()
        error = np.linalg.norm(tracked - arrays["target_positions"], axis=2)
        prefix = f"bonesaw_{name}"
        raw[f"{prefix}_step_ns"] = step_ns
        raw[f"{prefix}_status"] = status
        raw[f"{prefix}_tracked_positions"] = tracked
        raw[f"{prefix}_tracking_error_m"] = error
        raw[f"{prefix}_q"] = q_out
        raw[f"{prefix}_v"] = v_out
        metric = summarize_scenario(
            step_ns,
            error,
            status,
            arrays["target_active"],
            wall_ns,
            cpu_ns,
            thread_cpu_ns,
            memory_before,
            memory_after,
            peak_rss,
            usage_before,
            usage_after,
            gc_before,
            gc_after,
            traced_current,
            traced_peak,
        )
        if "reference_stance" in arrays:
            metric["walking_retarget"] = summarize_walking_retarget(
                tracked,
                arrays["target_positions"],
                arrays["reference_stance"],
                arrays["cadence_scale"],
            )
            acceptance = metric["walking_retarget"]["acceptance"]
            acceptance["checks"]["no_contingency_or_rejected_ticks"] = (
                metric["status_counts"]["contingency"] == 0
                and metric["status_counts"]["failed_or_rejected"] == 0
            )
            acceptance["passed"] = all(acceptance["checks"].values())
            raw[f"{prefix}_reference_stance"] = arrays["reference_stance"]
            raw[f"{prefix}_cadence_scale"] = arrays["cadence_scale"]
            raw[f"{prefix}_source_phase_frames"] = arrays["source_phase_frames"]
        scenarios[name] = metric

    output = pathlib.Path(args.output)
    np.savez_compressed(output / "bonesaw-raw.npz", **raw)
    result = {
        "implementation": "Bonesaw",
        "version": importlib.metadata.version("bonesaw"),
        "binding": "PyO3 0.28 + rust-numpy 0.28, fixed-shape batch writes",
        "baseline_memory": baseline_memory,
        "import_memory": import_memory,
        "setup_memory": setup_memory,
        "import_wall_ns": import_wall_ns,
        "setup_wall_ns": setup_wall_ns,
        "scenarios": scenarios,
    }
    (output / "bonesaw-metrics.json").write_text(json.dumps(result, indent=2) + "\n")


def configure_placo(model: pathlib.Path, spec: Scenario, initial_targets: np.ndarray, standing: np.ndarray):
    import placo

    source = model.read_text()
    robot = placo.RobotWrapper("", placo.Flags.ignore_collisions, source)
    names = list(robot.joint_names())
    for index, name in enumerate(names):
        robot.set_joint(name, float(standing[index]))
    robot.update_kinematics()
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.dt = DT
    solver.enable_joint_limits(True)
    solver.enable_velocity_limits(True)
    tasks = []
    for index, frame in enumerate(spec.frame_names):
        task = solver.add_position_task(frame, initial_targets[index])
        task.configure(f"target_{index}", "soft", spec.weights[index])
        tasks.append(task)
    posture = solver.add_joints_task()
    posture.set_joints({name: float(standing[index]) for index, name in enumerate(names)})
    # PlaCo has one weighted soft objective here rather than Bonesaw's strict
    # lower-priority null-space posture. Keep only enough regularization to
    # choose a stable solution without visibly competing with effectors.
    posture.configure("posture", "soft", 1e-3)
    return robot, solver, tasks


def run_placo_worker(args: argparse.Namespace) -> None:
    process = psutil.Process()
    baseline_memory = process_snapshot(process)
    import_started = time.perf_counter_ns()
    import placo
    import_wall_ns = time.perf_counter_ns() - import_started
    import_memory = process_snapshot(process)
    corpus = np.load(args.input)
    model = pathlib.Path(args.model).resolve()
    standing = corpus["standing_q"]
    raw: dict[str, np.ndarray] = {}
    scenarios: dict[str, Any] = {}
    setup_wall_ns = 0
    setup_memory = import_memory

    for scenario_index, name in enumerate(SCENARIOS):
        spec = SCENARIO_SPECS[name]
        arrays = scenario_arrays(corpus, name)
        total_ticks = arrays["target_positions"].shape[0]
        setup_started = time.perf_counter_ns()
        robot, solver, tasks = configure_placo(
            model, spec, arrays["target_positions"][0], standing
        )
        com_task = None
        if np.any(arrays["com_active"]):
            com_task = solver.add_com_task(arrays["com_targets"][0])
            com_task.configure("center_of_mass", "soft", 10.0)
        setup_wall_ns += time.perf_counter_ns() - setup_started
        setup_memory = process_snapshot(process)

        for tick in range(min(args.warmup, total_ticks)):
            for target, task in enumerate(tasks):
                task.target_world = arrays["target_positions"][tick, target]
            if com_task is not None:
                com_task.target_world = arrays["com_targets"][tick]
            robot.update_kinematics()
            solver.solve(True)
            robot.update_kinematics()
        robot, solver, tasks = configure_placo(
            model, spec, arrays["target_positions"][0], standing
        )
        if np.any(arrays["com_active"]):
            com_task = solver.add_com_task(arrays["com_targets"][0])
            com_task.configure("center_of_mass", "soft", 10.0)
        else:
            com_task = None

        tracked = np.empty(
            (total_ticks, len(arrays["frame_ids"]), 3), dtype=np.float64
        )
        step_ns = np.empty(total_ticks, dtype=np.uint64)
        solve_ns = np.empty(total_ticks, dtype=np.uint64)
        status = np.zeros(total_ticks, dtype=np.uint8)
        gc.collect()
        gc_before = gc_snapshot()
        usage_before = usage_snapshot(process)
        memory_before = process_snapshot(process)
        peak_rss = memory_before["rss_bytes"]
        tracemalloc.start()
        wall_started = time.perf_counter_ns()
        cpu_started = time.process_time_ns()
        thread_cpu_started = time.thread_time_ns()
        for tick in range(total_ticks):
            step_started = time.perf_counter_ns()
            for target, task in enumerate(tasks):
                task.target_world = arrays["target_positions"][tick, target]
            if com_task is not None:
                com_task.target_world = arrays["com_targets"][tick]
            robot.update_kinematics()
            solve_started = time.perf_counter_ns()
            try:
                solver.solve(True)
            except Exception:
                status[tick] = 3
            solve_ns[tick] = time.perf_counter_ns() - solve_started
            robot.update_kinematics()
            for target, frame in enumerate(spec.frame_names):
                tracked[tick, target] = robot.get_T_world_frame(frame)[:3, 3]
            step_ns[tick] = time.perf_counter_ns() - step_started
            if tick % args.chunk == 0:
                peak_rss = max(peak_rss, process.memory_info().rss)
        thread_cpu_ns = time.thread_time_ns() - thread_cpu_started
        cpu_ns = time.process_time_ns() - cpu_started
        wall_ns = time.perf_counter_ns() - wall_started
        traced_current, traced_peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        memory_after = process_snapshot(process)
        usage_after = usage_snapshot(process)
        gc_after = gc_snapshot()
        error = np.linalg.norm(tracked - arrays["target_positions"], axis=2)
        prefix = f"placo_{name}"
        raw[f"{prefix}_step_ns"] = step_ns
        raw[f"{prefix}_solve_ns"] = solve_ns
        raw[f"{prefix}_status"] = status
        raw[f"{prefix}_tracked_positions"] = tracked
        raw[f"{prefix}_tracking_error_m"] = error
        metric = summarize_scenario(
            step_ns,
            error,
            status,
            arrays["target_active"],
            wall_ns,
            cpu_ns,
            thread_cpu_ns,
            memory_before,
            memory_after,
            peak_rss,
            usage_before,
            usage_after,
            gc_before,
            gc_after,
            traced_current,
            traced_peak,
            solve_ns=solve_ns,
        )
        if "reference_stance" in arrays:
            metric["walking_retarget"] = summarize_walking_retarget(
                tracked,
                arrays["target_positions"],
                arrays["reference_stance"],
                arrays["cadence_scale"],
            )
            acceptance = metric["walking_retarget"]["acceptance"]
            acceptance["checks"]["no_contingency_or_rejected_ticks"] = (
                metric["status_counts"]["contingency"] == 0
                and metric["status_counts"]["failed_or_rejected"] == 0
            )
            acceptance["passed"] = all(acceptance["checks"].values())
            raw[f"{prefix}_reference_stance"] = arrays["reference_stance"]
            raw[f"{prefix}_cadence_scale"] = arrays["cadence_scale"]
            raw[f"{prefix}_source_phase_frames"] = arrays["source_phase_frames"]
        scenarios[name] = metric

    output = pathlib.Path(args.output)
    np.savez_compressed(output / "placo-raw.npz", **raw)
    result = {
        "implementation": "PlaCo",
        "version": importlib.metadata.version("placo"),
        "pinocchio_version": importlib.metadata.version("pin"),
        "binding": "Python loop over C++ QP solver",
        "baseline_memory": baseline_memory,
        "import_memory": import_memory,
        "setup_memory": setup_memory,
        "import_wall_ns": import_wall_ns,
        "setup_wall_ns": setup_wall_ns,
        "scenarios": scenarios,
    }
    (output / "placo-metrics.json").write_text(json.dumps(result, indent=2) + "\n")


def distribution(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "stddev": float(np.std(values)),
        "median": float(np.median(values)),
        "mad": float(np.median(np.abs(values - np.median(values)))),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "p99_9": float(np.percentile(values, 99.9)),
        "p99_99": float(np.percentile(values, 99.99)),
        "max": float(np.max(values)),
    }


def summarize_scenario(
    step_ns: np.ndarray,
    error: np.ndarray,
    status: np.ndarray,
    active: np.ndarray,
    wall_ns: int,
    cpu_ns: int,
    thread_cpu_ns: int,
    memory_before: dict[str, int],
    memory_after: dict[str, int],
    peak_rss: int,
    usage_before: dict[str, Any],
    usage_after: dict[str, Any],
    gc_before: dict[str, Any],
    gc_after: dict[str, Any],
    traced_current: int,
    traced_peak: int,
    *,
    solve_ns: np.ndarray | None = None,
) -> dict[str, Any]:
    selected = error[active.astype(bool)]
    steady_tick = min(int(round(1.0 / DT)), error.shape[0])
    steady = error[steady_tick:][active[steady_tick:].astype(bool)]
    latency = distribution(step_ns)
    jitter = distribution(np.abs(np.diff(step_ns.astype(np.int64)))) if len(step_ns) > 1 else distribution(step_ns)
    tracking = distribution(selected)
    windows = []
    for indices in np.array_split(np.arange(len(step_ns)), 10):
        window_active = active[indices].astype(bool)
        window_error = error[indices][window_active]
        windows.append(
            {
                "tick_start": int(indices[0]),
                "tick_stop": int(indices[-1] + 1),
                "latency_p50_ns": float(np.percentile(step_ns[indices], 50)),
                "latency_p99_ns": float(np.percentile(step_ns[indices], 99)),
                "tracking_rms_m": float(np.sqrt(np.mean(window_error**2))),
                "tracking_p99_m": float(np.percentile(window_error, 99)),
            }
        )
    result = {
        "ticks": int(len(step_ns)),
        "wall_ns": wall_ns,
        "cpu_ns": cpu_ns,
        "thread_cpu_ns": thread_cpu_ns,
        "cpu_to_wall_ratio": cpu_ns / max(wall_ns, 1),
        "thread_cpu_to_wall_ratio": thread_cpu_ns / max(wall_ns, 1),
        "throughput_steps_per_second": len(step_ns) * 1e9 / max(wall_ns, 1),
        "latency_ns": latency,
        "jitter_abs_delta_ns": jitter,
        "deadline_misses": {
            "1ms": int(np.count_nonzero(step_ns > 1_000_000)),
            "5ms": int(np.count_nonzero(step_ns > 5_000_000)),
            "20ms": int(np.count_nonzero(step_ns > 20_000_000)),
        },
        "tracking_error_m": tracking,
        "tracking_rms_m": float(np.sqrt(np.mean(selected**2))),
        "steady_state_rms_m_after_1s": float(np.sqrt(np.mean(steady**2))),
        "integrated_absolute_error_m_s": float(np.sum(selected) * DT),
        "integrated_squared_error_m2_s": float(np.sum(selected**2) * DT),
        "fraction_over_1cm": float(np.mean(selected > 0.01)),
        "fraction_over_3cm": float(np.mean(selected > 0.03)),
        "fraction_over_5cm": float(np.mean(selected > 0.05)),
        "status_counts": {
            "ok": int(np.count_nonzero(status == 0)),
            "degraded": int(np.count_nonzero(status == 1)),
            "contingency": int(np.count_nonzero(status == 2)),
            "failed_or_rejected": int(np.count_nonzero(status == 3)),
        },
        "memory_before": memory_before,
        "memory_after": memory_after,
        "peak_rss_bytes": peak_rss,
        "rss_delta_bytes": memory_after["rss_bytes"] - memory_before["rss_bytes"],
        "usage_delta": delta(usage_after, usage_before),
        "python_gc_delta": gc_delta(gc_after, gc_before),
        "python_tracemalloc_current_bytes": traced_current,
        "python_tracemalloc_peak_bytes": traced_peak,
        "temporal_windows": windows,
    }
    if solve_ns is not None:
        result["solver_only_ns"] = distribution(solve_ns)
    return result


def summarize_walking_retarget(
    tracked: np.ndarray,
    targets: np.ndarray,
    stance: np.ndarray,
    cadence_scale: np.ndarray,
) -> dict[str, Any]:
    error = np.linalg.norm(tracked - targets, axis=2)
    foot_error = error[:, :2]
    hand_error = error[:, 2:]
    stance_mask = stance.astype(bool)
    swing_mask = ~stance_mask
    target_ground = np.min(targets[:, :2, 2], axis=0)
    target_clearance = targets[:, :2, 2] - target_ground
    achieved_clearance = tracked[:, :2, 2] - target_ground
    clearance_error = np.abs(achieved_clearance - target_clearance)
    transition = np.zeros_like(stance_mask)
    transition[1:] = stance_mask[1:] != stance_mask[:-1]
    target_step = np.linalg.norm(np.diff(targets[:, :2], axis=0), axis=2)
    tracked_step = np.linalg.norm(np.diff(tracked[:, :2], axis=0), axis=2)
    transition_target_steps = np.concatenate(
        [target_step[:, foot][transition[1:, foot]] for foot in range(2)]
    )
    transition_tracked_steps = np.concatenate(
        [tracked_step[:, foot][transition[1:, foot]] for foot in range(2)]
    )
    if len(transition_target_steps) == 0:
        transition_target_steps = np.zeros(1, dtype=np.float64)
        transition_tracked_steps = np.zeros(1, dtype=np.float64)

    cadence: dict[str, Any] = {}
    for scale in sorted(set(float(value) for value in cadence_scale)):
        selected = cadence_scale == scale
        cadence[f"{scale:.2f}x"] = {
            "ticks": int(np.count_nonzero(selected)),
            "tracking_rms_m": float(np.sqrt(np.mean(error[selected] ** 2))),
            "foot_tracking_rms_m": float(np.sqrt(np.mean(foot_error[selected] ** 2))),
            "hand_tracking_rms_m": float(np.sqrt(np.mean(hand_error[selected] ** 2))),
        }

    per_effector = {}
    for index, name in enumerate(("left_foot", "right_foot", "left_hand", "right_hand")):
        per_effector[name] = {
            "tracking_rms_m": float(np.sqrt(np.mean(error[:, index] ** 2))),
            "tracking_p95_m": float(np.percentile(error[:, index], 95)),
            "tracking_max_m": float(np.max(error[:, index])),
            "reference_axis_range_m": np.ptp(targets[:, index], axis=0).tolist(),
            "achieved_axis_range_m": np.ptp(tracked[:, index], axis=0).tolist(),
        }

    result = {
        "foot_tracking_rms_m": float(np.sqrt(np.mean(foot_error**2))),
        "hand_tracking_rms_m": float(np.sqrt(np.mean(hand_error**2))),
        "stance_foot_tracking_rms_m": float(
            np.sqrt(np.mean(foot_error[stance_mask] ** 2))
        ),
        "swing_foot_tracking_rms_m": float(
            np.sqrt(np.mean(foot_error[swing_mask] ** 2))
        ),
        "swing_clearance_error_rms_m": float(
            np.sqrt(np.mean(clearance_error[swing_mask] ** 2))
        ),
        "minimum_swing_clearance_m": float(np.min(achieved_clearance[swing_mask])),
        "maximum_swing_clearance_m": float(np.max(achieved_clearance[swing_mask])),
        "reference_maximum_swing_clearance_m": float(
            np.max(target_clearance[swing_mask])
        ),
        "double_support_fraction": float(np.mean(np.all(stance_mask, axis=1))),
        "flight_fraction": float(np.mean(~np.any(stance_mask, axis=1))),
        "contact_transition_count": int(np.count_nonzero(transition)),
        "contact_transition_target_step_p99_m": float(
            np.percentile(transition_target_steps, 99)
        ),
        "contact_transition_tracked_step_p99_m": float(
            np.percentile(transition_tracked_steps, 99)
        ),
        "cadence": cadence,
        "per_effector": per_effector,
    }
    cadence_foot_limit_m = 0.06
    checks = {
        "foot_tracking_rms_le_5cm": result["foot_tracking_rms_m"] <= 0.05,
        "hand_tracking_rms_le_3cm": result["hand_tracking_rms_m"] <= 0.03,
        "stance_foot_tracking_rms_le_6cm": (
            result["stance_foot_tracking_rms_m"] <= 0.06
        ),
        "swing_foot_tracking_rms_le_6cm": (
            result["swing_foot_tracking_rms_m"] <= 0.06
        ),
        "swing_clearance_error_rms_le_3cm": (
            result["swing_clearance_error_rms_m"] <= 0.03
        ),
        "minimum_swing_clearance_ge_minus_1cm": (
            result["minimum_swing_clearance_m"] >= -0.01
        ),
        "each_cadence_foot_tracking_rms_le_6cm": all(
            value["foot_tracking_rms_m"] <= cadence_foot_limit_m
            for value in cadence.values()
        ),
    }
    result["acceptance"] = {
        "passed": all(checks.values()),
        "checks": checks,
        "thresholds": {
            "foot_tracking_rms_m": 0.05,
            "hand_tracking_rms_m": 0.03,
            "stance_foot_tracking_rms_m": 0.06,
            "swing_foot_tracking_rms_m": 0.06,
            "swing_clearance_error_rms_m": 0.03,
            "minimum_swing_clearance_m": -0.01,
            "per_cadence_foot_tracking_rms_m": cadence_foot_limit_m,
        },
    }
    return result


def cpu_model() -> str:
    cpuinfo = pathlib.Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def fmt_us(value_ns: float) -> str:
    return f"{value_ns / 1e3:.1f}"


def fmt_mb(value_bytes: float) -> str:
    return f"{value_bytes / (1024 * 1024):.2f}"


def fmt_optional_mb(value_bytes: int | None) -> str:
    return "n/a" if value_bytes is None else fmt_mb(value_bytes)


def load_g1_profile(path: pathlib.Path) -> dict[str, Any]:
    metrics_path = path / "floating-walk-metrics.json"
    raw_path = path / "floating-walk-raw.npz"
    if not metrics_path.is_file() or not raw_path.is_file():
        raise SystemExit(
            f"missing G1 profile artifacts under {path}; run "
            "scripts/run-floating-walk-corpus.sh first"
        )
    document = json.loads(metrics_path.read_text())
    if document.get("schema", 0) < 2:
        raise SystemExit(
            f"{metrics_path} predates detailed runtime schema 2; regenerate it"
        )
    with np.load(raw_path) as raw:
        required = {
            "step_ns",
            "status",
            "root_targets",
            "root_tracked",
            "target_positions",
            "tracked_positions",
            "dynamics_residual",
            "contact_residual",
        }
        missing = sorted(required - set(raw.files))
        if missing:
            raise SystemExit(f"{raw_path} is missing arrays: {missing}")
        ticks = int(len(raw["step_ns"]))
        if "task_jacobi_sweeps" in raw.files:
            sweep_counts = raw["task_jacobi_sweeps"]
            for window in document["metrics"].get("temporal_windows", []):
                start = int(window["tick_start"])
                stop = int(window["tick_stop"])
                window["task_jacobi_sweeps_mean"] = float(
                    np.mean(sweep_counts[start:stop])
                )
    if ticks != document["metrics"]["ticks"]:
        raise SystemExit(
            f"G1 profile tick mismatch between {metrics_path} and {raw_path}"
        )
    return {
        "directory": str(path),
        "metrics_sha256": hashlib.sha256(metrics_path.read_bytes()).hexdigest(),
        "raw_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        "document": document,
    }


def render_report(
    output: pathlib.Path,
    bonesaw: dict[str, Any],
    placo: dict[str, Any],
    native: dict[str, Any],
    upkie_native: dict[str, Any],
    ticks: int,
    warmup: int,
    upkie_controller: dict[str, Any] | None,
    g1_profiles: dict[str, dict[str, Any]],
    pinocchio_oracles: dict[str, dict[str, Any]],
) -> str:
    input_manifest = json.loads((output / "input-manifest.json").read_text())
    walking_reference = input_manifest["walking_reference"]
    lines = [
        "# Bonesaw reference implementation comparison",
        "",
        "This report compares the CPU kinematic WBC path against PlaCo, keeps",
        "Pinocchio as the rigid-body correctness oracle, and executes Upkie's",
        "pinned C++ WheelBalancer as a direct rolling-controller oracle. Raw",
        "per-step traces are stored beside this report in compressed NumPy archives.",
        "The official Unitree G1 section adds the current floating inverse-dynamics",
        "liftoff proof and deliberately red full-transfer stress, including execution",
        "windows and process-resource measurements.",
        "",
        "## Method and comparability",
        "",
        f"- Same `models/toy_humanoid.urdf`, fixed base, 18 coordinates, 50 Hz, {ticks:,} measured ticks per scenario.",
        f"- {warmup:,} unreported warmup ticks precede every measured run.",
        "- Targets, initial posture, sample times, joint/velocity limits, and tracked frame names are identical.",
        "- Bonesaw uses strict lexicographic priorities and a bandwidth-shaped velocity target plus the finite-difference velocity jet of the shared position reference.",
        "- PlaCo uses its independent C++ QP kinematics solver with soft task weights, a 1e-3 near-null posture regularizer, and a weight-10 CoM task in the conflict case.",
        "- Walking targets come from the pinned CMU Graphics Lab subject-37/trial-1 slow walk, not a procedural sine wave. A periodic six-harmonic reconstruction is morphology-scaled and time-warped through 0.75×, 1.0×, and 1.25× cadence blocks.",
        "- PlaCo's public position task accepts the shared position reference but exposes no target-velocity field. Therefore timing and closed-loop tracking are measurable, while exact command equality and identical feedforward are not expected: the optimization semantics and task interfaces are deliberately different.",
        "- Bonesaw `latency` is timed inside the Rust batch loop. PlaCo `latency` includes Python target assignment, kinematic updates, the C++ solve, and frame extraction; its separate solver-only distribution is also reported.",
        "- Python `tracemalloc` excludes native allocations. RSS/USS and `ru_maxrss` include native memory.",
        "",
        "PlaCo describes itself as a C++ whole-body inverse-kinematics/dynamics QP implementation built on Pinocchio and eiquadprog; its documented loop updates kinematics, solves, integrates, and updates kinematics again.",
        "",
        "## Reference scope matrix",
        "",
        "These references answer different questions; rows that do not share a model,",
        "decision vector, or task API are not presented as command-for-command parity.",
        "",
        "| Reference | Shared boundary | What it proves | Deliberate limitation |",
        "|---|---|---|---|",
        "| PlaCo | Same toy-humanoid model, targets, initial state, frames, and 50 Hz scenario corpus | Independent WBC tracking, latency, jitter, and process-resource comparison | Soft weighted QP and public position-only task API differ from Bonesaw's strict hierarchy and target jets |",
        "| Pinocchio 4.0 | Same URDF states and rigid-body convention adapters | FK, Jacobian, CoM, mass, bias, inverse dynamics, and centroidal product correctness | Product oracle, not a closed-loop controller |",
        "| Upkie C++ WheelBalancer | Same sequential balance inputs and official parameter profile | Canonical bitwise controller-law parity plus live-tuning delta | Compares the typed rolling law, not the whole inverse-dynamics WBC |",
        "| Official Unitree G1 + CMU 37/01 | Authored G1 mass/inertia/limits/sole geometry and a pinned walking source | Floating contact behavior, strict residuals, transitions, and scaling cost on a realistic 23-DOF morphology | Behavior/evaluation reference; no public G1 WBC exposes identical strict task semantics for direct command parity |",
        "",
        "## Environment",
        "",
        "| Field | Value |",
        "|---|---|",
        f"| CPU | {cpu_model()} |",
        f"| OS | {platform.platform()} |",
        f"| Python | {platform.python_version()} |",
        f"| Bonesaw | {bonesaw['version']} |",
        f"| PlaCo | {placo['version']} |",
        f"| PlaCo Pinocchio | {placo['pinocchio_version']} |",
        f"| Logical CPUs | {psutil.cpu_count(logical=True)} |",
        f"| Physical CPUs | {psutil.cpu_count(logical=False)} |",
        "",
        "## Aggregate outcome",
        "",
        "| Scenario | Implementation | RMS error cm | steady RMS cm | p50 µs | p99 µs | p99.9 µs | max µs | >20 ms | steps/s |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            metric = report["scenarios"][name]
            latency = metric["latency_ns"]
            lines.append(
                f"| {name} | {implementation} | {metric['tracking_rms_m'] * 100:.3f} | "
                f"{metric['steady_state_rms_m_after_1s'] * 100:.3f} | {fmt_us(latency['median'])} | "
                f"{fmt_us(latency['p99'])} | {fmt_us(latency['p99_9'])} | {fmt_us(latency['max'])} | "
                f"{metric['deadline_misses']['20ms']} | {metric['throughput_steps_per_second']:.0f} |"
            )

    g1_labels = {
        "liftoff": "moving liftoff",
        "transfer": "full transfer stress",
    }
    lines += [
        "",
        "## Official G1 floating inverse-dynamics WBC",
        "",
        "Both profiles use the checksum-pinned Unitree 23-DOF mode-10 URDF, four",
        "friction-limited force points per sole, six rank-minimal kinematic rows per",
        "locked foot, and the same CMU 37/01 retarget. The short profile proves a",
        "moving liftoff without fallback. The longer profile is retained precisely",
        "because it exposes the current single-support/contact-transfer failure.",
        "Hand error is observational because the canonical hand task weight is zero.",
        "",
        "### Behavior, feasibility, and tracking",
        "",
        "| profile | behavior gate | overall gate | ticks | nominal prefix | root RMS cm | stance RMS cm | swing RMS cm | max root deg | max joint rad/s | solved/slack/touchdown | fallback/release/infeasible/failed | dynamics max | contact max |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        metric = g1_profiles[key]["document"]["metrics"]
        counts = metric["status_counts"]
        behavior_passed = all(
            passed
            for name, passed in metric["acceptance"]["checks"].items()
            if name != "p99_tick_le_5ms"
        )
        lines.append(
            f"| {g1_labels[key]} | "
            f"{'PASS' if behavior_passed else 'FAIL'} | "
            f"{'PASS' if metric['acceptance']['passed'] else 'FAIL'} | "
            f"{metric['ticks']:,} | {metric['nominal_prefix']['ticks']:,} | "
            f"{metric['root_tracking_rms_m'] * 100:.3f} | "
            f"{metric['stance_foot_tracking_rms_m'] * 100:.3f} | "
            f"{metric['swing_foot_tracking_rms_m'] * 100:.3f} | "
            f"{math.degrees(metric['maximum_root_rotation_rad']):.3f} | "
            f"{metric['maximum_joint_velocity_rad_s']:.3f} | "
            f"{counts['solved']}/{counts['solved_with_slack']}/"
            f"{counts['touchdown_transition']} | "
            f"{counts['normal_contact_contingency']}/"
            f"{counts['contact_release_contingency']}/"
            f"{counts['primal_infeasible']}/{counts['failed']} | "
            f"{metric['maximum_dynamics_residual']:.2e} | "
            f"{metric['maximum_contact_acceleration_residual']:.2e} |"
        )

    liftoff_acceptance = g1_profiles["liftoff"]["document"]["metrics"]["acceptance"]
    transfer_acceptance = g1_profiles["transfer"]["document"]["metrics"]["acceptance"]
    liftoff_summary = (
        "The moving-liftoff row passes both the behavioral and unchanged 5 ms p99 CPU gates."
        if liftoff_acceptance["passed"]
        else "The moving-liftoff row remains behaviorally green but fails the unchanged 5 ms p99 CPU gate."
    )
    transfer_summary = (
        "The transfer row passes its combined gate."
        if transfer_acceptance["passed"]
        else "The transfer row remains a deliberately retained failing stress case."
    )
    lines += [
        "",
        liftoff_summary,
        f"{transfer_summary} Rejected ticks",
        "retain raw residuals and state rather than being omitted from aggregates.",
        "",
        "### G1 latency, jitter, and deadlines",
        "",
        "| profile | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        metric = g1_profiles[key]["document"]["metrics"]
        latency = metric["latency_us"]
        jitter = metric["jitter_abs_delta_us"]
        deadline = metric["deadline_misses"]
        lines.append(
            f"| {g1_labels[key]} | {latency['mean']:.1f} | "
            f"{latency['stddev']:.1f} | {latency['mad']:.1f} | "
            f"{latency['p50']:.1f} | {latency['p95']:.1f} | "
            f"{latency['p99']:.1f} | {latency['p99_9']:.1f} | "
            f"{latency['p99_99']:.1f} | {latency['max']:.1f} | "
            f"{jitter['p99']:.1f} | {deadline['1ms']:,} | "
            f"{deadline['5ms']:,} | {deadline['20ms']:,} | "
            f"{metric['hot_loop_throughput_ticks_per_second']:.1f} |"
        )

    lines += [
        "",
        "### G1 process resources",
        "",
        "Buffers and the Rust session are allocated before this measurement. Python",
        "`tracemalloc` excludes native allocation; zero hot-loop allocation is gated",
        "separately by the native Rust sentinels later in this report.",
        "",
        "| profile | wall s | process CPU s | thread CPU s | process CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor/major faults | voluntary/involuntary ctx |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        runtime = g1_profiles[key]["document"]["metrics"]["runtime"]
        usage = runtime["usage_delta"]
        lines.append(
            f"| {g1_labels[key]} | {runtime['call_wall_ns'] / 1e9:.3f} | "
            f"{runtime['process_cpu_ns'] / 1e9:.3f} | "
            f"{runtime['thread_cpu_ns'] / 1e9:.3f} | "
            f"{runtime['process_cpu_to_wall_ratio']:.3f} | "
            f"{fmt_optional_mb(runtime['rss_before_bytes'])} | "
            f"{fmt_optional_mb(runtime['rss_after_bytes'])} | "
            f"{fmt_optional_mb(runtime['rss_delta_bytes'])} | "
            f"{fmt_optional_mb(runtime['peak_rss_bytes'])} | "
            f"{fmt_optional_mb(runtime['python_tracemalloc_peak_bytes'])} | "
            f"{runtime['python_gc_delta']['collections']:,} | "
            f"{usage['minor_faults']:,}/{usage['major_faults']:,} | "
            f"{usage['voluntary_context_switches']:,}/"
            f"{usage['involuntary_context_switches']:,} |"
        )

    lines += [
        "",
        "### G1 latency by solver/contact status",
        "",
        "| profile | status | ticks | p50 µs | p95 µs | p99 µs | max µs |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        status_latency = g1_profiles[key]["document"]["metrics"][
            "status_latency_us"
        ]
        for status_name, latency in status_latency.items():
            lines.append(
                f"| {g1_labels[key]} | {status_name} | {latency['ticks']:,} | "
                f"{latency['p50']:.1f} | {latency['p95']:.1f} | "
                f"{latency['p99']:.1f} | {latency['max']:.1f} |"
            )

    lines += [
        "",
        "### G1 strict-solver work attribution",
        "",
        "Task-level pseudoinverse and Jacobi-sweep counts expose the dense-kernel",
        "work behind each tick. The solver reuses the feasibility seed's exact",
        "equality factorization, reuses a projected inverse while its nullspace is",
        "unchanged, and skips the terminal Style projector because no lower priority",
        "can consume it. Near-feasible seeds still polish and refactor. Optimizations",
        "that change floating-point summation order are evaluated as algorithmic",
        "revisions; this report does not claim bitwise behavior/status-trace parity.",
        "",
        "| profile | pseudoinverse mean/p95/p99/max | Jacobi sweeps mean/p95/p99/max | sweeps/pseudoinverse | clipped-step mean/p95/p99/max | calls↔latency correlation |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        work = g1_profiles[key]["document"]["metrics"]["solver_work"]
        calls = work["task_pseudoinverse_calls"]
        sweeps = work["task_jacobi_sweeps"]
        clipped = work["clipped_steps"]
        lines.append(
            f"| {g1_labels[key]} | {calls['mean']:.2f}/{calls['p95']:.1f}/"
            f"{calls['p99']:.1f}/{calls['max']:.0f} | "
            f"{sweeps['mean']:.2f}/{sweeps['p95']:.1f}/"
            f"{sweeps['p99']:.1f}/{sweeps['max']:.0f} | "
            f"{work['jacobi_sweeps_per_pseudoinverse']:.2f} | "
            f"{clipped['mean']:.2f}/{clipped['p95']:.1f}/"
            f"{clipped['p99']:.1f}/{clipped['max']:.0f} | "
            f"{work['pseudoinverse_calls_to_latency_correlation']:.4f} |"
        )

    lines += [
        "",
        "| profile | priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        by_priority = g1_profiles[key]["document"]["metrics"]["solver_work"][
            "by_priority"
        ]
        for priority, work in by_priority.items():
            calls = work["task_pseudoinverse_calls"]
            sweeps = work["task_jacobi_sweeps"]
            clipped = work["clipped_steps"]
            lines.append(
                f"| {g1_labels[key]} | {priority} | "
                f"{calls['mean']:.2f}/{calls['p99']:.1f}/{calls['max']:.0f} | "
                f"{sweeps['mean']:.2f}/{sweeps['p99']:.1f}/{sweeps['max']:.0f} | "
                f"{clipped['mean']:.2f}/{clipped['p99']:.1f}/{clipped['max']:.0f} | "
                f"{work['ticks_with_clipping']:,} |"
            )

    lines += [
        "",
        "### G1 execution over time",
        "",
        "| profile | ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in ("liftoff", "transfer"):
        windows = g1_profiles[key]["document"]["metrics"]["temporal_windows"]
        for window in windows:
            lines.append(
                f"| {g1_labels[key]} | {window['tick_start']}–"
                f"{window['tick_stop'] - 1} | {window['latency_p50_us']:.1f} | "
                f"{window['latency_p99_us']:.1f} | "
                f"{window['task_pseudoinverse_calls_mean']:.2f} | "
                f"{window.get('task_jacobi_sweeps_mean', 0.0):.2f} | "
                f"{window['clipped_steps_mean']:.2f} | "
                f"{window['root_tracking_rms_m'] * 100:.3f} | "
                f"{window['foot_tracking_rms_m'] * 100:.3f} | "
                f"{window['maximum_dynamics_residual']:.2e} | "
                f"{window['maximum_contact_acceleration_residual']:.2e} | "
                f"{window['contingency_or_rejected_ticks']:,} |"
            )

    lines += [
        "",
        "### G1 artifact integrity",
        "",
        "| profile | metrics SHA-256 | raw NPZ SHA-256 | directory |",
        "|---|---|---|---|",
    ]
    for key in ("liftoff", "transfer"):
        profile = g1_profiles[key]
        lines.append(
            f"| {g1_labels[key]} | `{profile['metrics_sha256']}` | "
            f"`{profile['raw_sha256']}` | `{profile['directory']}` |"
        )

    lines += [
        "",
        "## Pinocchio 4.0 rigid-body differential oracles",
        "",
        "The Rust fixture and Pinocchio run in separate processes. Every row compares",
        "fixed and floating products at identical deterministic states after explicit",
        "conversion between Bonesaw's `[angular; linear]` world-classical root tangent",
        "and Pinocchio's free-flyer convention.",
        "",
        "| model | gate | states | frame samples | frame position | frame rotation | Jacobian | CoM | mass | gravity | inverse dynamics | centroidal map | floating mass | floating bias | floating inverse dynamics |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in ("upkie", "g1"):
        oracle = pinocchio_oracles[key]
        error = oracle["maximum_absolute_error"]
        lines.append(
            f"| {oracle['model']} | {'PASS' if oracle['passed'] else 'FAIL'} | "
            f"{oracle['samples']:,} | {oracle['frames_compared']:,} | "
            f"{error['frame_translation_m']:.2e} | "
            f"{error['frame_rotation_matrix']:.2e} | "
            f"{error['frame_jacobian']:.2e} | "
            f"{error['center_of_mass']:.2e} | {error['mass_matrix']:.2e} | "
            f"{error['generalized_gravity']:.2e} | "
            f"{error['inverse_dynamics']:.2e} | "
            f"{error['centroidal_map']:.2e} | "
            f"{error['floating_mass_matrix']:.2e} | "
            f"{error['floating_bias']:.2e} | "
            f"{error['floating_inverse_dynamics']:.2e} |"
        )

    lines += [
        "",
        "## Latency and jitter distributions",
        "",
        "| Scenario | Impl | mean µs | std µs | MAD µs | p90 µs | p95 µs | p99 µs | p99.99 µs | jitter p99 µs |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            metric = report["scenarios"][name]
            latency = metric["latency_ns"]
            jitter = metric["jitter_abs_delta_ns"]
            lines.append(
                f"| {name} | {implementation} | {fmt_us(latency['mean'])} | {fmt_us(latency['stddev'])} | "
                f"{fmt_us(latency['mad'])} | {fmt_us(latency['p90'])} | {fmt_us(latency['p95'])} | "
                f"{fmt_us(latency['p99'])} | {fmt_us(latency['p99_99'])} | {fmt_us(jitter['p99'])} |"
            )

    lines += [
        "",
        "## Tracking quality",
        "",
        "| Scenario | Impl | median cm | p95 cm | p99 cm | max cm | IAE m·s | ISE m²·s | >1 cm | >3 cm | >5 cm |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            metric = report["scenarios"][name]
            error = metric["tracking_error_m"]
            lines.append(
                f"| {name} | {implementation} | {error['median'] * 100:.3f} | {error['p95'] * 100:.3f} | "
                f"{error['p99'] * 100:.3f} | {error['max'] * 100:.3f} | "
                f"{metric['integrated_absolute_error_m_s']:.3f} | {metric['integrated_squared_error_m2_s']:.4f} | "
                f"{metric['fraction_over_1cm']:.1%} | {metric['fraction_over_3cm']:.1%} | "
                f"{metric['fraction_over_5cm']:.1%} |"
            )

    lines += [
        "",
        "## Data-backed walking retarget",
        "",
        f"Source: CMU Graphics Lab Motion Capture Database, subject "
        f"{walking_reference['subject']}, trial {walking_reference['trial']} "
        f"(`{walking_reference['description']}`, "
        f"{walking_reference['source_rate_hz']} Hz). The ASF/AMC bytes are "
        "checksum-pinned and fetched into the ignored benchmark cache.",
        "",
        f"The selected cycle is {walking_reference['cycle_duration_seconds']:.3f} s; "
        f"raw same-phase endpoint closure is "
        f"{walking_reference['closure_position_rms_m'] * 100:.3f} cm RMS. "
        f"Leg/arm morphology scales are "
        f"{walking_reference['leg_morphology_scale']:.3f}/"
        f"{walking_reference['arm_morphology_scale']:.3f}. "
        f"Maximum reference speed/acceleration across cadence blocks are "
        f"{walking_reference['maximum_target_speed_m_s']:.3f} m/s and "
        f"{walking_reference['maximum_target_acceleration_m_s2']:.3f} m/s².",
        "",
        "Because the toy benchmark holds the pelvis fixed, the retarget removes "
        "the source pelvis-bob component by grounding the lower foot at every "
        "phase. Corpus construction rejects flight or missing bilateral "
        "stance/swing phases. The retained labels contain "
        f"{walking_reference['reference_double_support_fraction']:.1%} double "
        f"support, {walking_reference['reference_flight_fraction']:.1%} flight, "
        f"and {walking_reference['reference_contact_transition_count']} bilateral "
        "contact transitions.",
        "",
        "The predeclared acceptance envelope requires overall foot/hand RMS ≤5/3 cm, "
        "stance and swing foot RMS ≤6 cm, clearance RMS ≤3 cm, minimum swing "
        "clearance ≥−1 cm, each cadence foot RMS ≤6 cm, and no contingency/rejected ticks.",
        "",
        "| Impl | Gate | foot RMS cm | hand RMS cm | stance foot RMS cm | swing foot RMS cm | clearance RMS cm | min/peak achieved clearance cm | reference peak cm | transitions | target/tracked transition p99 cm |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
        gait = report["scenarios"]["walking_motion_retarget"]["walking_retarget"]
        lines.append(
            f"| {implementation} | "
            f"{'PASS' if gait['acceptance']['passed'] else 'FAIL'} | "
            f"{gait['foot_tracking_rms_m'] * 100:.3f} | "
            f"{gait['hand_tracking_rms_m'] * 100:.3f} | "
            f"{gait['stance_foot_tracking_rms_m'] * 100:.3f} | "
            f"{gait['swing_foot_tracking_rms_m'] * 100:.3f} | "
            f"{gait['swing_clearance_error_rms_m'] * 100:.3f} | "
            f"{gait['minimum_swing_clearance_m'] * 100:.3f}/"
            f"{gait['maximum_swing_clearance_m'] * 100:.3f} | "
            f"{gait['reference_maximum_swing_clearance_m'] * 100:.3f} | "
            f"{gait['contact_transition_count']} | "
            f"{gait['contact_transition_target_step_p99_m'] * 100:.3f}/"
            f"{gait['contact_transition_tracked_step_p99_m'] * 100:.3f} |"
        )
    lines += [
        "",
        "| Cadence | Impl | all-effector RMS cm | foot RMS cm | hand RMS cm | ticks |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for cadence in ("0.75x", "1.00x", "1.25x"):
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            metric = report["scenarios"]["walking_motion_retarget"][
                "walking_retarget"
            ]["cadence"].get(cadence)
            if metric is None:
                lines.append(f"| {cadence} | {implementation} | — | — | — | 0 |")
                continue
            lines.append(
                f"| {cadence} | {implementation} | "
                f"{metric['tracking_rms_m'] * 100:.3f} | "
                f"{metric['foot_tracking_rms_m'] * 100:.3f} | "
                f"{metric['hand_tracking_rms_m'] * 100:.3f} | "
                f"{metric['ticks']:,} |"
            )
    for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
        acceptance = report["scenarios"]["walking_motion_retarget"][
            "walking_retarget"
        ]["acceptance"]
        failed = [name for name, passed in acceptance["checks"].items() if not passed]
        lines.append(
            f"- {implementation} walking gate: "
            f"{'PASS' if not failed else 'FAIL — ' + ', '.join(failed)}"
        )

    lines += [
        "",
        "## Memory, CPU, faults, context switches, and Python GC",
        "",
        "| Scenario | Impl | RSS before MB | peak RSS MB | RSS Δ MB | thread CPU/wall | process CPU/wall | minor faults | major faults | voluntary ctx | involuntary ctx | Python peak MB | GC collections |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            metric = report["scenarios"][name]
            usage = metric["usage_delta"]
            collections = sum(metric["python_gc_delta"]["collections"])
            lines.append(
                f"| {name} | {implementation} | {fmt_mb(metric['memory_before']['rss_bytes'])} | "
                f"{fmt_mb(metric['peak_rss_bytes'])} | {fmt_mb(metric['rss_delta_bytes'])} | "
                f"{metric['thread_cpu_to_wall_ratio']:.3f} | {metric['cpu_to_wall_ratio']:.3f} | "
                f"{usage['minor_faults']} | {usage['major_faults']} | "
                f"{usage['voluntary_context_switches']} | {usage['involuntary_context_switches']} | "
                f"{fmt_mb(metric['python_tracemalloc_peak_bytes'])} | {collections} |"
            )

    lines += [
        "",
        "## Native controller heap-allocation sentinel",
        "",
        "The standalone Rust executable samples its counting global allocator immediately",
        "around each reusable controller transition. Corpus construction, statistics, and",
        "state copying outside the controller call are excluded. Collision mode evaluates",
        "all compiled pairs while reusing distance, task, constraint, solver, trajectory,",
        "and output workspaces.",
        "",
        "| Scenario | allocation calls/tick | allocated bytes/tick |",
        "|---|---:|---:|",
    ]
    for scenario in native["scenarios"]:
        lines.append(
            f"| {scenario['name']} | {scenario['allocations_per_tick']:.1f} | "
            f"{scenario['allocated_bytes_per_tick']:.1f} |"
        )
    lines.append(
        f"| collision-enabled controller ({native['collision']['self_pairs']} pairs) | "
        f"{native['collision']['controller_allocations_per_tick']:.1f} | "
        f"{native['collision']['controller_allocated_bytes_per_tick']:.1f} |"
    )
    signal_graph = native["signal_graph"]
    lines.append(
        f"| compiled signal graph ({signal_graph['nodes']} nodes, "
        f"{signal_graph['memory_slots']} memory slots) | "
        f"{signal_graph['allocations_per_step']:.1f} | "
        f"{signal_graph['allocated_bytes_per_step']:.1f} |"
    )
    compiled_rig = native["compiled_rig"]
    lines.append(
        f"| compiled signal→task→controller rig "
        f"({compiled_rig['task_slots']} task slots) | "
        f"{compiled_rig['allocations_per_tick']:.1f} | "
        f"{compiled_rig['allocated_bytes_per_tick']:.1f} |"
    )
    lines.append(
        f"| dynamic WBC ({native['dynamic_wbc']['decision_variables']} variables, "
        f"{native['dynamic_wbc']['contacts']} contacts) | "
        f"{native['dynamic_wbc']['allocations_per_tick']:.1f} | "
        f"{native['dynamic_wbc']['allocated_bytes_per_tick']:.1f} |"
    )
    lines.append(
        f"| floating dynamic WBC ({native['floating_dynamic_wbc']['decision_variables']} "
        f"variables, {native['floating_dynamic_wbc']['contacts']} contacts) | "
        f"{native['floating_dynamic_wbc']['allocations_per_tick']:.1f} | "
        f"{native['floating_dynamic_wbc']['allocated_bytes_per_tick']:.1f} |"
    )
    if upkie_native["floating_squat"] is not None:
        lines.append(
            f"| Upkie contact-IK floating squat preview | "
            f"{upkie_native['floating_squat']['allocations_per_step']:.1f} | "
            f"{upkie_native['floating_squat']['allocated_bytes_per_step']:.1f} |"
        )
    if upkie_native.get("floating_unprojected_squat") is not None:
        lines.append(
            f"| Upkie raw integrated balance + squat | "
            f"{upkie_native['floating_unprojected_squat']['allocations_per_step']:.1f} | "
            f"{upkie_native['floating_unprojected_squat']['allocated_bytes_per_step']:.1f} |"
        )

    lines += [
        "",
        "## Compiled signal-jet graph sentinel",
        "",
        "This profile evaluates a flat scalar/vector graph with input and constant jets,",
        "add, scale, analytic blend derivatives, deadband, clamp, low-pass, and critically",
        "damped spring nodes. Stateful memory is explicit and double-buffered; graph topology,",
        "stable IDs, types, memory slots, and outputs are frozen into MotionProgram archive v5",
        "and its SHA-256 content fingerprint.",
        "",
        "| ticks | nodes | outputs | state slots | max output accel | p50 us | p99 us | max us | bitwise repeat |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {signal_graph['ticks']:,} | {signal_graph['nodes']} | "
        f"{signal_graph['outputs']} | {signal_graph['memory_slots']} | "
        f"{signal_graph['maximum_output_acceleration_abs']:.3f} | "
        f"{signal_graph['p50_step_us']:.3f} | {signal_graph['p99_step_us']:.3f} | "
        f"{signal_graph['max_step_us']:.3f} | "
        f"{'yes' if signal_graph['bitwise_repeat'] else 'NO'} |",
        "",
        "## Compiled signal-to-task controller sentinel",
        "",
        "This profile runs the complete explicit transition: vector and rotation inputs,",
        "low-pass and SO(3) spring state, resolved point, CoM, and orientation task slots,",
        "FK/Jacobians, strict hierarchy,",
        "quintic synthesis, and next-state signal-memory commit. Two independently sized",
        "state/scratch/output streams receive identical inputs.",
        "",
        "| ticks | signal nodes | task slots | point RMS cm | orientation RMS deg | p50 us | p99 us | max us | bitwise repeat |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {compiled_rig['ticks']:,} | {compiled_rig['signal_nodes']} | "
        f"{compiled_rig['task_slots']} | {100.0 * compiled_rig['tracking_rms_m']:.4f} | "
        f"{np.degrees(compiled_rig['orientation_tracking_rms_rad']):.4f} | "
        f"{compiled_rig['p50_tick_us']:.1f} | {compiled_rig['p99_tick_us']:.1f} | "
        f"{compiled_rig['max_tick_us']:.1f} | "
        f"{'yes' if compiled_rig['bitwise_repeat'] else 'NO'} |",
    ]

    dynamic = native["dynamic_wbc"]
    floating_dynamic = native["floating_dynamic_wbc"]
    lines += [
        "",
        "## Unified inverse-dynamics WBC sentinel",
        "",
        "The CPU dynamic profiles solve one strict problem over `[q̈, τ, contact force]`.",
        "Hard rows enforce the rigid-body equation and locked/rolling contact acceleration;",
        "bounds and inequalities enforce acceleration, torque, unilateral normal load, and",
        "a four-sided friction pyramid. The floating profile prepends six unactuated root",
        "accelerations and enforces all six free-body equilibrium rows without a root torque.",
        "",
        "| profile | ticks | variables | contacts | accel tracking RMS | max abs(q̈) | max abs(τ) | max abs(f) | dynamics max | contact accel max | min friction margin | friction active | min torque margin | p50 µs | p99 µs | max µs | infeasible | bitwise repeat |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| fixed | {dynamic['ticks']:,} | {dynamic['decision_variables']} | {dynamic['contacts']} | "
        f"{dynamic['acceleration_tracking_rms']:.3e} | "
        f"{dynamic['maximum_acceleration_abs']:.3e} | "
        f"{dynamic['maximum_torque_abs']:.3e} | "
        f"{dynamic['maximum_contact_force_abs']:.3e} | "
        f"{dynamic['maximum_dynamics_residual']:.2e} | "
        f"{dynamic['maximum_contact_acceleration_residual']:.2e} | "
        f"{dynamic['minimum_friction_margin']:.2e} | "
        f"{dynamic['friction_active_ticks']:,} | "
        f"{dynamic['minimum_torque_margin']:.2e} | "
        f"{dynamic['p50_tick_us']:.1f} | {dynamic['p99_tick_us']:.1f} | "
        f"{dynamic['max_tick_us']:.1f} | {dynamic['infeasible_ticks']} | "
        f"{'yes' if dynamic['bitwise_repeat'] else 'NO'} |",
        f"| floating | {floating_dynamic['ticks']:,} | "
        f"{floating_dynamic['decision_variables']} | {floating_dynamic['contacts']} | "
        f"{floating_dynamic['acceleration_tracking_rms']:.3e} | "
        f"{floating_dynamic['maximum_acceleration_abs']:.3e} | "
        f"{floating_dynamic['maximum_torque_abs']:.3e} | "
        f"{floating_dynamic['maximum_contact_force_abs']:.3e} | "
        f"{floating_dynamic['maximum_dynamics_residual']:.2e} | "
        f"{floating_dynamic['maximum_contact_acceleration_residual']:.2e} | "
        f"{floating_dynamic['minimum_friction_margin']:.2e} | "
        f"{floating_dynamic['friction_active_ticks']:,} | "
        f"{floating_dynamic['minimum_torque_margin']:.2e} | "
        f"{floating_dynamic['p50_tick_us']:.1f} | "
        f"{floating_dynamic['p99_tick_us']:.1f} | "
        f"{floating_dynamic['max_tick_us']:.1f} | "
        f"{floating_dynamic['infeasible_ticks']} | "
        f"{'yes' if floating_dynamic['bitwise_repeat'] else 'NO'} |",
    ]
    if upkie_native["floating_squat"] is not None:
        floating_squat = upkie_native["floating_squat"]
        raw_squat = upkie_native.get("floating_unprojected_squat")
        lines += [
            "",
            "## Upkie floating balance and squat",
            "",
            "Both 200 Hz sentinels generate a two-contact planar posture with allocation-free",
            "damped Gauss-Newton IK and solve the same floating WBC. The projected row is",
            "retained as an explicit guided diagnostic; the raw row is the default browser",
            "and dynamic regression path. Root attitude, root translation, and CoM feedback are emitted by",
            "the same fixed-slot compiled policy used by the server. A compiled 1 Hz",
            "critically damped root-translation spring shapes discontinuous editor targets. The adapter",
            "supplies target jets and the articulation-aware axle-to-CoM pitch observation.",
            "The raw row integrates only the solver output and uses canonical wheel-center",
            "rolling constraints plus a bounded wheel-acceleration task derived from Upkie's",
            "velocity PI law.",
            "`Slip`",
            "measures only constrained lateral/normal motion; wheel-axis travel is reported",
            "separately and is physically permitted.",
            "",
            "| path | ticks | target cm | achieved cm | final z err mm | z RMS cm | xy RMS cm | CoM RMS cm | slip mm | wheel travel cm | max rotation deg | max qdd | max tau | max force | dynamics max | contact accel max | min friction | min torque | degraded | infeasible | p50 us | p99 us | max us |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            f"| projected adapter | {floating_squat['ticks']:,} | {floating_squat['target_lowering_m'] * 100:.2f} | "
            f"{floating_squat['achieved_lowering_m'] * 100:.2f} | "
            f"{floating_squat['final_root_height_error_m'] * 1000:.3f} | "
            f"{floating_squat['root_height_tracking_rms_m'] * 100:.2f} | "
            f"{floating_squat['root_horizontal_tracking_rms_m'] * 100:.2f} | "
            f"{floating_squat['center_of_mass_tracking_rms_m'] * 100:.2f} | "
            f"{floating_squat['maximum_constrained_contact_drift_m'] * 1000:.3f} | "
            f"{floating_squat['maximum_permitted_rolling_travel_m'] * 100:.2f} | "
            f"{math.degrees(floating_squat['maximum_root_rotation_rad']):.3f} | "
            f"{floating_squat['maximum_generalized_acceleration_abs']:.2f} | "
            f"{floating_squat['maximum_actuator_torque_abs']:.2f} | "
            f"{floating_squat['maximum_contact_force_abs']:.2f} | "
            f"{floating_squat['maximum_dynamics_residual']:.2e} | "
            f"{floating_squat['maximum_contact_acceleration_residual']:.2e} | "
            f"{floating_squat['minimum_friction_margin']:.2e} | "
            f"{floating_squat['minimum_torque_margin']:.2e} | "
            f"{floating_squat['degraded_ticks']:,} | {floating_squat['infeasible_ticks']:,} | "
            f"{floating_squat['p50_step_us']:.1f} | {floating_squat['p99_step_us']:.1f} | "
            f"{floating_squat['max_step_us']:.1f} |",
        ]
        if raw_squat is not None:
            lines.append(
                f"| raw integrated WBC | {raw_squat['ticks']:,} | "
                f"{raw_squat['target_lowering_m'] * 100:.2f} | "
                f"{raw_squat['achieved_lowering_m'] * 100:.2f} | "
                f"{raw_squat['final_root_height_error_m'] * 1000:.3f} | "
                f"{raw_squat['root_height_tracking_rms_m'] * 100:.2f} | "
                f"{raw_squat['root_horizontal_tracking_rms_m'] * 100:.2f} | "
                f"{raw_squat['center_of_mass_tracking_rms_m'] * 100:.2f} | "
                f"{raw_squat['maximum_constrained_contact_drift_m'] * 1000:.3f} | "
                f"{raw_squat['maximum_permitted_rolling_travel_m'] * 100:.2f} | "
                f"{math.degrees(raw_squat['maximum_root_rotation_rad']):.3f} | "
                f"{raw_squat['maximum_generalized_acceleration_abs']:.2f} | "
                f"{raw_squat['maximum_actuator_torque_abs']:.2f} | "
                f"{raw_squat['maximum_contact_force_abs']:.2f} | "
                f"{raw_squat['maximum_dynamics_residual']:.2e} | "
                f"{raw_squat['maximum_contact_acceleration_residual']:.2e} | "
                f"{raw_squat['minimum_friction_margin']:.2e} | "
                f"{raw_squat['minimum_torque_margin']:.2e} | "
                f"{raw_squat['degraded_ticks']:,} | {raw_squat['infeasible_ticks']:,} | "
                f"{raw_squat['p50_step_us']:.1f} | {raw_squat['p99_step_us']:.1f} | "
                f"{raw_squat['max_step_us']:.1f} |"
            )
            lines += [
                "",
                f"The raw policy contains {raw_squat['compiled_signal_nodes']} signal nodes and "
                f"{raw_squat['compiled_task_slots']} resolved floating task slots. Its emitted "
                f"root/CoM acceleration differs from the direct formula oracle by at most "
                f"`{raw_squat['maximum_policy_oracle_acceleration_delta']:.3e}`; the full "
                "IK → signals → task emission → WBC → SE(3) integration loop reports zero "
                "allocator calls and bytes per step.",
            ]

    if upkie_controller is not None:
        parity = upkie_controller["comparisons"]["official_aligned"]
        live = upkie_controller["comparisons"]["live_tuned"]
        upkie_latency = upkie_controller["latency"]["upkie_cpp"]
        rust_latency = upkie_controller["latency"]["bonesaw_live"]
        lines += [
            "",
            "## Official Upkie wheel-controller oracle",
            "",
            "A separate 100,000-step worker links the pinned upstream",
            "`WheelBalancer.cpp` class unchanged. On the common floor-contact,",
            "stationary-target subset, Bonesaw is evaluated once with upstream",
            "parameters for an exact implementation gate and once with live tuned",
            "parameters to expose the intentional policy delta.",
            "",
            "| Gate | Result |",
            "|---|---:|",
            f"| Official-parameter wheel commands | {'PASS' if parity['canonical_bitwise_equal'] else 'FAIL'} · {parity['canonical_bit_mismatches']:,} canonical bit mismatches |",
            f"| Live-tuned ground-velocity delta | RMS {live['rms_error']:.4f} m/s · max {live['maximum_absolute_error']:.4f} m/s · correlation {live['correlation']:.6f} |",
            f"| Official C++ adapter latency | p50 {upkie_latency['p50_us']:.3f} µs · p99 {upkie_latency['p99_us']:.3f} µs |",
            f"| Rust typed-law latency | p50 {rust_latency['p50_us']:.3f} µs · p99 {rust_latency['p99_us']:.3f} µs |",
            f"| Rust hot-loop allocation | {upkie_controller['bonesaw_hot_loop_allocations']['live']['allocation_calls']} calls · {upkie_controller['bonesaw_hot_loop_allocations']['live']['allocated_bytes']} bytes |",
            "",
            "See [UPKIE_CONTROLLER_COMPARISON.md](UPKIE_CONTROLLER_COMPARISON.md)",
            "for per-region behavior, p99.99 latency, jitter, ten temporal windows,",
            "RSS/CPU/fault/context-switch metrics, provenance hashes, and raw artifacts.",
        ]

    lines += [
        "",
        "## PlaCo solver-only latency",
        "",
        "This removes Python target assignment, both kinematics updates, and frame",
        "extraction from PlaCo's end-to-end step timing.",
        "",
        "| Scenario | mean µs | p50 µs | p95 µs | p99 µs | p99.9 µs | max µs |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        latency = placo["scenarios"][name]["solver_only_ns"]
        lines.append(
            f"| {name} | {fmt_us(latency['mean'])} | {fmt_us(latency['median'])} | "
            f"{fmt_us(latency['p95'])} | {fmt_us(latency['p99'])} | "
            f"{fmt_us(latency['p99_9'])} | {fmt_us(latency['max'])} |"
        )

    lines += [
        "",
        "## Temporal drift by execution window",
        "",
        "Each row is one tenth of a run. This exposes warm drift, allocator/GC episodes,",
        "thermal/scheduler outliers, and tracking degradation hidden by one aggregate.",
        "",
        "| Scenario | Impl | tick range | p50 µs | p99 µs | RMS cm | p99 error cm |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for name in SCENARIOS:
        for implementation, report in (("Bonesaw", bonesaw), ("PlaCo", placo)):
            for window in report["scenarios"][name]["temporal_windows"]:
                lines.append(
                    f"| {name} | {implementation} | {window['tick_start']}–{window['tick_stop'] - 1} | "
                    f"{fmt_us(window['latency_p50_ns'])} | {fmt_us(window['latency_p99_ns'])} | "
                    f"{window['tracking_rms_m'] * 100:.3f} | {window['tracking_p99_m'] * 100:.3f} |"
                )

    lines += [
        "",
        "## Process startup and resident footprint",
        "",
        "| Implementation | import ms | setup ms | baseline RSS MB | after import MB | after setup MB | import Δ MB | setup Δ MB |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for report in (bonesaw, placo):
        lines.append(
            f"| {report['implementation']} | {report['import_wall_ns'] / 1e6:.2f} | "
            f"{report['setup_wall_ns'] / 1e6:.2f} | {fmt_mb(report['baseline_memory']['rss_bytes'])} | "
            f"{fmt_mb(report['import_memory']['rss_bytes'])} | {fmt_mb(report['setup_memory']['rss_bytes'])} | "
            f"{fmt_mb(report['import_memory']['rss_bytes'] - report['baseline_memory']['rss_bytes'])} | "
            f"{fmt_mb(report['setup_memory']['rss_bytes'] - report['import_memory']['rss_bytes'])} |"
        )

    lines += [
        "",
        "## Artifacts and interpretation",
        "",
        "- `input-corpus.npz`: exact shared targets, activation masks, CMU gait phase, cadence, and reference stance labels.",
        "- `input-manifest.json`: CMU source/license/checksum provenance plus cycle, morphology, and target-bandwidth metadata.",
        "- `bonesaw-raw.npz`: native per-step latency, status, q/v, tracked positions, and errors.",
        "- `placo-raw.npz`: end-to-end and solver-only per-step latency, status, tracked positions, and errors.",
        "- `native-cpu-eval.json`: Rust allocator, dynamics, collision, constraint, determinism, and query sentinels.",
        "- `native-upkie-eval.json`: Upkie floating dynamics and contact-consistent interactive squat sentinel.",
        "- `reference-metrics.json`: complete machine-readable aggregates and ten temporal windows.",
        "- `UPKIE_CONTROLLER_COMPARISON.md`: direct pinned C++ controller-law oracle with per-step traces.",
        "- `upkie-controller-raw.npz`: shared balance inputs, commands, integral states, and latency samples.",
        "- `pinocchio-upkie.json` and `pinocchio-g1.json`: complete fixed/floating product maxima, thresholds, state/frame counts, and coordinate provenance.",
        "- `../floating-g1-liftoff-latest/`: official-G1 green moving-liftoff metrics, full state/residual/latency trace, and standalone report.",
        "- `../floating-g1-transfer-latest/`: official-G1 red full-transfer stress with every fallback/release/rejected tick retained.",
        "- `../../../docs/PINOCCHIO_ORACLE.md`: independent FK/Jacobian/mass/gravity/inverse-dynamics correctness.",
        "",
        "The comparison is a regression instrument, not a claim that one formulation is",
        "universally better. Bonesaw's strict hierarchy is the intended product behavior;",
        "PlaCo's QP is an intentionally independent reference with different tradeoffs.",
        "",
        "Sources: [PlaCo repository](https://github.com/Rhoban/placo), "
        "[PlaCo kinematics loop documentation](https://placo.readthedocs.io/en/stable/kinematics/getting_started.html), "
        "[Pinocchio repository](https://github.com/stack-of-tasks/pinocchio), "
        "[Upkie repository](https://github.com/upkie/upkie).",
        "",
    ]
    return "\n".join(lines)


def run_parent(args: argparse.Namespace) -> None:
    root = pathlib.Path(__file__).resolve().parents[2]
    model_arg = pathlib.Path(args.model)
    upkie_model_arg = pathlib.Path(args.upkie_model)
    output_arg = pathlib.Path(args.output)
    cmu_cache_arg = pathlib.Path(args.cmu_cache)
    g1_liftoff_arg = pathlib.Path(args.g1_liftoff)
    g1_transfer_arg = pathlib.Path(args.g1_transfer)
    model = (root / model_arg).resolve() if not model_arg.is_absolute() else model_arg
    upkie_model = (
        (root / upkie_model_arg).resolve()
        if not upkie_model_arg.is_absolute()
        else upkie_model_arg
    )
    output = (root / output_arg).resolve() if not output_arg.is_absolute() else output_arg
    cmu_cache = (
        (root / cmu_cache_arg).resolve()
        if not cmu_cache_arg.is_absolute()
        else cmu_cache_arg
    )
    g1_liftoff = (
        (root / g1_liftoff_arg).resolve()
        if not g1_liftoff_arg.is_absolute()
        else g1_liftoff_arg
    )
    g1_transfer = (
        (root / g1_transfer_arg).resolve()
        if not g1_transfer_arg.is_absolute()
        else g1_transfer_arg
    )
    cmu_source_record = json.loads(
        (root / "benchmarks/references/cmu-37-walk.json").read_text()
    )
    for filename, checksum_key in (
        ("37.asf", "skeleton_sha256"),
        ("37_01.amc", "motion_sha256"),
    ):
        reference_path = cmu_cache / filename
        if not reference_path.is_file():
            raise SystemExit(
                f"missing {reference_path}; run scripts/fetch-cmu-walk-reference.sh"
            )
        actual = hashlib.sha256(reference_path.read_bytes()).hexdigest()
        expected = cmu_source_record[checksum_key]
        if actual != expected:
            raise SystemExit(
                f"CMU reference checksum mismatch for {reference_path}: "
                f"expected {expected}, got {actual}"
            )
    try:
        model_label = str(model.relative_to(root))
    except ValueError:
        model_label = str(model)
    output.mkdir(parents=True, exist_ok=True)
    if not args.report_only:
        input_path = build_input_corpus(
            model,
            model_label,
            args.ticks,
            output,
            cmu_cache,
            cmu_source_record,
        )
        for implementation in ("bonesaw", "placo"):
            subprocess.run(
                [
                    sys.executable,
                    str(pathlib.Path(__file__).resolve()),
                    "--worker",
                    implementation,
                    "--model",
                    str(model),
                    "--ticks",
                    str(args.ticks),
                    "--warmup",
                    str(args.warmup),
                    "--chunk",
                    str(args.chunk),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output),
                ],
                check=True,
                cwd=root,
            )
        native_process = subprocess.run(
            [
                "cargo",
                "run",
                "--release",
                "-p",
                "bonesaw-tools",
                "--bin",
                "bonesaw-eval",
                "--",
                "--model",
                str(model),
                "--ticks",
                str(args.ticks),
                "--json",
            ],
            check=True,
            cwd=root,
            capture_output=True,
            text=True,
        )
        native = json.loads(native_process.stdout)
        (output / "native-cpu-eval.json").write_text(
            json.dumps(native, indent=2) + "\n"
        )
        upkie_native_process = subprocess.run(
            [
                "cargo",
                "run",
                "--release",
                "-p",
                "bonesaw-tools",
                "--bin",
                "bonesaw-eval",
                "--",
                "--model",
                str(upkie_model),
                "--ticks",
                str(args.ticks),
                "--json",
            ],
            check=True,
            cwd=root,
            capture_output=True,
            text=True,
        )
        upkie_native = json.loads(upkie_native_process.stdout)
        (output / "native-upkie-eval.json").write_text(
            json.dumps(upkie_native, indent=2) + "\n"
        )
    required_existing = (
        "input-manifest.json",
        "bonesaw-metrics.json",
        "placo-metrics.json",
        "native-cpu-eval.json",
        "native-upkie-eval.json",
        "pinocchio-upkie.json",
        "pinocchio-g1.json",
    )
    missing_existing = [
        name for name in required_existing if not (output / name).is_file()
    ]
    if missing_existing:
        raise SystemExit(
            f"missing comparison artifacts under {output}: {missing_existing}"
        )
    bonesaw = json.loads((output / "bonesaw-metrics.json").read_text())
    placo = json.loads((output / "placo-metrics.json").read_text())
    native = json.loads((output / "native-cpu-eval.json").read_text())
    upkie_native = json.loads((output / "native-upkie-eval.json").read_text())
    pinocchio_oracles = {
        "upkie": json.loads((output / "pinocchio-upkie.json").read_text()),
        "g1": json.loads((output / "pinocchio-g1.json").read_text()),
    }
    upkie_controller_path = output / "upkie-controller-metrics.json"
    upkie_controller = (
        json.loads(upkie_controller_path.read_text())
        if upkie_controller_path.exists()
        else None
    )
    g1_profiles = {
        "liftoff": load_g1_profile(g1_liftoff),
        "transfer": load_g1_profile(g1_transfer),
    }
    combined = {
        "schema": 2,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "method": {
            "ticks": args.ticks,
            "warmup_ticks": args.warmup,
            "dt_seconds": DT,
            "model": model_label,
            "upkie_model": str(upkie_model),
            "walking_reference": json.loads(
                (output / "input-manifest.json").read_text()
            )["walking_reference"],
        },
        "environment": {
            "cpu": cpu_model(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "logical_cpus": psutil.cpu_count(logical=True),
            "physical_cpus": psutil.cpu_count(logical=False),
        },
        "bonesaw": bonesaw,
        "placo": placo,
        "native_cpu_eval": native,
        "native_upkie_eval": upkie_native,
        "upkie_controller_oracle": upkie_controller,
        "g1_floating_profiles": g1_profiles,
        "pinocchio_oracles": pinocchio_oracles,
    }
    (output / "reference-metrics.json").write_text(json.dumps(combined, indent=2) + "\n")
    report = render_report(
        output,
        bonesaw,
        placo,
        native,
        upkie_native,
        args.ticks,
        args.warmup,
        upkie_controller,
        g1_profiles,
        pinocchio_oracles,
    )
    (output / "REFERENCE_COMPARISON.md").write_text(report)
    print(output / "REFERENCE_COMPARISON.md")


def main() -> None:
    args = parse_args()
    if args.ticks <= 0 or args.warmup < 0 or args.chunk <= 0:
        raise SystemExit("ticks/chunk must be positive and warmup non-negative")
    if args.worker == "bonesaw":
        run_bonesaw_worker(args)
    elif args.worker == "placo":
        run_placo_worker(args)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
