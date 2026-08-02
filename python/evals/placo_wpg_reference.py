#!/usr/bin/env python3
"""Generate a PlaCo WPG reference without IK, control, or physics integration."""

from __future__ import annotations

import argparse
import contextlib
import gc
import importlib.metadata
import io
import json
from pathlib import Path
import resource
import time
import xml.etree.ElementTree as ET

import numpy as np
import placo


DT = 0.005
SOURCE_DIRECTORY = "g1-transfer-r39-retiming-release100-guard020-800"
LEFT = 0
RIGHT = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"
    )
    parser.add_argument("--results-root", default="benchmarks/results")
    parser.add_argument("--source", default=SOURCE_DIRECTORY)
    parser.add_argument("--output", default="benchmarks/results/g1-placo-wpg-r41")
    parser.add_argument("--ticks", type=int, default=600)
    return parser.parse_args()


def adapted_g1_urdf(path: Path) -> str:
    """Strip unavailable meshes and add PlaCo's three conventional frame aliases."""
    tree = ET.parse(path)
    robot = tree.getroot()
    for link in robot.findall("link"):
        for tag in ("visual", "collision"):
            for element in list(link.findall(tag)):
                link.remove(element)
    aliases = (
        ("left_foot", "left_ankle_roll_link", "0.035 0 -0.035"),
        ("right_foot", "right_ankle_roll_link", "0.035 0 -0.035"),
        ("trunk", "torso_link", "0 0 0"),
        ("left_hip_yaw", "left_hip_yaw_link", "0 0 0"),
        ("head_base", "head_link", "0 0 0"),
        ("head_pitch", "head_link", "0 0 0"),
        ("camera", "d435_link", "0 0 0"),
    )
    names = {element.attrib["name"] for element in robot.findall("link")}
    for child, parent, xyz in aliases:
        if child in names:
            continue
        ET.SubElement(robot, "link", {"name": child})
        joint = ET.SubElement(
            robot,
            "joint",
            {"name": f"bonesaw_placo_{child}", "type": "fixed"},
        )
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": child})
        ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    return ET.tostring(robot, encoding="unicode")


def footstep(side: object, ankle_position: np.ndarray, width: float, length: float) -> object:
    step = placo.Footstep(width, length)
    step.side = side
    frame = np.eye(4)
    frame[:3, 3] = ankle_position + np.array([0.035, 0.0, -0.035])
    step.frame = frame
    return step


def support_state(trajectory: object, t: float) -> np.ndarray:
    # Treat a support boundary as belonging to the new phase, as the authored
    # corpus does at its integer contact edge.
    query_time = min(t + 1.0e-10, float(trajectory.t_end))
    if trajectory.support_is_both(query_time):
        return np.array([1, 1], dtype=np.uint8)
    side = trajectory.support_side(query_time)
    if side == placo.HumanoidRobot_Side.left:
        return np.array([1, 0], dtype=np.uint8)
    return np.array([0, 1], dtype=np.uint8)


def generate(
    model: Path,
    source_raw: object,
    source_motion: dict[str, object],
    ticks: int,
) -> tuple[dict[str, np.ndarray], dict[str, object]]:
    if ticks < 429:
        raise ValueError("matched first-step reference requires at least 429 ticks")
    stance = source_raw["reference_stance"].astype(bool)
    feet_source = source_raw["target_positions"][:, :2]
    com_source = source_raw["center_of_mass_targets"]
    root_source = source_raw["root_targets"]
    liftoff = int(np.flatnonzero(~stance[1:, LEFT] & stance[:-1, LEFT])[0] + 1)
    touchdown = int(np.flatnonzero(stance[1:, LEFT] & ~stance[:-1, LEFT])[0] + 1)
    single_ticks = touchdown - liftoff
    start_ticks = liftoff

    model_warnings = io.StringIO()
    with contextlib.redirect_stderr(model_warnings):
        robot = placo.HumanoidRobot(
            "", placo.Flags.ignore_collisions, adapted_g1_urdf(model)
        )
    parameters = placo.HumanoidParameters()
    parameters.single_support_duration = single_ticks * DT
    parameters.single_support_timesteps = single_ticks
    parameters.double_support_ratio = 0.0
    parameters.startend_double_support_ratio = start_ticks / single_ticks
    parameters.planned_timesteps = start_ticks + single_ticks + start_ticks
    parameters.walk_com_height = float(com_source[0, 2])
    parameters.walk_foot_height = float(
        np.max(feet_source[liftoff : touchdown + 1, LEFT, 2])
        - feet_source[liftoff - 1, LEFT, 2]
    )
    parameters.walk_foot_rise_ratio = 0.2
    parameters.walk_trunk_pitch = 0.0
    parameters.foot_length = 2.0 * float(source_motion["contact_patch_half_length_m"])
    parameters.foot_width = 2.0 * float(source_motion["contact_patch_half_width_m"])
    parameters.feet_spacing = float(
        abs(feet_source[0, LEFT, 1] - feet_source[0, RIGHT, 1])
    )
    parameters.zmp_margin = float(source_motion.get("support_margin_m", 0.01))
    parameters.foot_zmp_target_x = 0.0
    parameters.foot_zmp_target_y = 0.0

    footsteps = placo.Footsteps()
    footsteps.append(
        footstep(
            placo.HumanoidRobot_Side.left,
            feet_source[0, LEFT],
            parameters.foot_width,
            parameters.foot_length,
        )
    )
    footsteps.append(
        footstep(
            placo.HumanoidRobot_Side.right,
            feet_source[0, RIGHT],
            parameters.foot_width,
            parameters.foot_length,
        )
    )
    footsteps.append(
        footstep(
            placo.HumanoidRobot_Side.left,
            feet_source[touchdown, LEFT],
            parameters.foot_width,
            parameters.foot_length,
        )
    )
    supports = placo.FootstepsPlanner.make_supports(
        footsteps, 0.0, True, False, True
    )
    planner = placo.WalkPatternGenerator(robot, parameters)
    plan_started = time.perf_counter_ns()
    trajectory = planner.plan(supports, com_source[0], 0.0)
    plan_ns = time.perf_counter_ns() - plan_started

    times = np.arange(ticks, dtype=np.float64) * DT
    if times[-1] > trajectory.t_end:
        raise ValueError("requested sample extends beyond PlaCo trajectory")
    com = np.empty((ticks, 3), dtype=np.float64)
    com_velocity = np.empty_like(com)
    com_acceleration = np.empty_like(com)
    feet = np.empty((ticks, 2, 3), dtype=np.float64)
    foot_velocity = np.empty_like(feet)
    reference_stance = np.empty((ticks, 2), dtype=np.uint8)
    sample_started = time.perf_counter_ns()
    for tick, t in enumerate(times):
        query_time = min(float(t) + 1.0e-10, float(trajectory.t_end))
        com[tick] = trajectory.get_p_world_CoM(query_time)
        com_velocity[tick] = trajectory.get_v_world_CoM(query_time)
        com_acceleration[tick] = trajectory.get_a_world_CoM(query_time)
        for foot, side in (
            (LEFT, placo.HumanoidRobot_Side.left),
            (RIGHT, placo.HumanoidRobot_Side.right),
        ):
            transform = (
                trajectory.get_T_world_left(query_time)
                if foot == LEFT
                else trajectory.get_T_world_right(query_time)
            )
            # Convert PlaCo's sole-center frame back to the G1 ankle-roll frame
            # used by the shared contract.
            feet[tick, foot] = transform[:3, 3] - transform[:3, :3] @ np.array(
                [0.035, 0.0, -0.035]
            )
            if trajectory.support_is_both(query_time):
                foot_velocity[tick, foot] = 0.0
            else:
                foot_velocity[tick, foot] = trajectory.get_v_world_foot(
                    side, query_time
                )
        reference_stance[tick] = support_state(trajectory, float(t))
    sample_ns = time.perf_counter_ns() - sample_started
    foot_acceleration = np.gradient(foot_velocity, DT, axis=0, edge_order=2)
    foot_velocity[reference_stance.astype(bool)] = 0.0
    foot_acceleration[reference_stance.astype(bool)] = 0.0
    root_offset = com_source[0] - root_source[0]
    root = com - root_offset[None, :]

    arrays = {
        "root_targets": root,
        "center_of_mass_targets": com,
        "center_of_mass_target_velocities": com_velocity,
        "center_of_mass_target_accelerations": com_acceleration,
        "target_positions": feet,
        "target_velocities": foot_velocity,
        "target_accelerations": foot_acceleration,
        "reference_stance": reference_stance,
    }
    details: dict[str, object] = {
        "plan_ns": plan_ns,
        "sample_ns": sample_ns,
        "trajectory_start_seconds": float(trajectory.t_start),
        "trajectory_end_seconds": float(trajectory.t_end),
        "matched_liftoff_tick": liftoff,
        "matched_touchdown_tick": touchdown,
        "matched_single_support_ticks": single_ticks,
        "matched_start_double_support_ticks": start_ticks,
        "pelvis_translation_semantics": (
            "derived from PlaCo CoM minus the identical initial G1 CoM-to-root offset"
        ),
        "optional_model_warnings": model_warnings.getvalue().splitlines(),
    }
    return arrays, details


def main() -> None:
    args = parse_args()
    if args.ticks <= 0:
        raise SystemExit("ticks must be positive")
    results_root = Path(args.results_root)
    source_directory = results_root / args.source
    source_payload = json.loads(
        (source_directory / "floating-walk-metrics.json").read_text()
    )
    source_raw = np.load(source_directory / "floating-walk-raw.npz")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    gc.collect()
    gc_before = tuple(gc.get_count())
    usage_before = resource.getrusage(resource.RUSAGE_SELF)
    arrays, details = generate(
        Path(args.model), source_raw, source_payload["motion"], args.ticks
    )
    gc_after = tuple(gc.get_count())
    usage_after = resource.getrusage(resource.RUSAGE_SELF)
    repeat, _ = generate(
        Path(args.model), source_raw, source_payload["motion"], args.ticks
    )
    exact_repeat = all(
        np.array_equal(arrays[key], repeat[key], equal_nan=True)
        for key in arrays
    )
    np.savez_compressed(output / "reference-inputs.npz", **arrays)
    motion = {
        "contact_patch_center_x_m": source_payload["motion"][
            "contact_patch_center_x_m"
        ],
        "contact_patch_half_length_m": source_payload["motion"][
            "contact_patch_half_length_m"
        ],
        "contact_patch_half_width_m": source_payload["motion"][
            "contact_patch_half_width_m"
        ],
        "contact_patch_z_m": source_payload["motion"]["contact_patch_z_m"],
        "support_margin_m": source_payload["motion"]["support_margin_m"],
    }
    metadata = {
        "schema": 1,
        "implementation": "PlaCo WalkPatternGenerator",
        "placo_version": importlib.metadata.version("placo"),
        "placo_source_revision": "e6c288604639d67b979a16cb2ad26913413c8e3a",
        "placo_license": "MIT",
        "policy_or_physics_rollout": False,
        "ik_or_wbc_solve": False,
        "source_reference": args.source,
        "ticks": args.ticks,
        "dt_seconds": DT,
        "exact_repeat": exact_repeat,
        "runtime": {
            **details,
            "gc_count_delta": [after - before for before, after in zip(gc_before, gc_after)],
            "user_cpu_seconds": usage_after.ru_utime - usage_before.ru_utime,
            "system_cpu_seconds": usage_after.ru_stime - usage_before.ru_stime,
            "maximum_rss_kib": usage_after.ru_maxrss,
        },
        "motion": motion,
    }
    (output / "reference-metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(output / "reference-inputs.npz")


if __name__ == "__main__":
    main()
