#!/usr/bin/env python3
"""Generate a G1 reference with the Rust support-constrained LIPM planner."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import resource
import subprocess
import time
from typing import Any

import numpy as np

from g1_reference_contract import convex_hull, sole_vertices


DT = 0.005
GRAVITY = 9.81
LEFT = 0
RIGHT = 1
SOURCE_DIRECTORY = "g1-transfer-r39-retiming-release100-guard020-800"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", default="benchmarks/results")
    parser.add_argument("--source", default=SOURCE_DIRECTORY)
    parser.add_argument("--output", default="benchmarks/results/g1-bonesaw-lipm-r44")
    parser.add_argument("--binary", default="target/release/bonesaw-lipm-reference")
    parser.add_argument("--ticks", type=int, default=600)
    parser.add_argument("--maximum-root-to-landing-reach", type=float, default=0.76)
    parser.add_argument("--root-horizontal-follow-ratio", type=float, default=0.75)
    return parser.parse_args()


def build_request(
    raw: Any,
    motion: dict[str, Any],
    ticks: int,
    maximum_root_to_landing_reach_m: float,
    root_horizontal_follow_ratio: float,
) -> dict[str, Any]:
    stance = raw["reference_stance"].astype(bool)
    feet = raw["target_positions"][:, :2]
    com = raw["center_of_mass_targets"]
    com_velocity = raw["center_of_mass_target_velocities"]
    root = raw["root_targets"]
    liftoff = int(np.flatnonzero(~stance[1:, LEFT] & stance[:-1, LEFT])[0] + 1)
    touchdown = int(np.flatnonzero(stance[1:, LEFT] & ~stance[:-1, LEFT])[0] + 1)
    if ticks <= touchdown:
        raise ValueError("reference must include the first touchdown")

    center_x = float(motion["contact_patch_center_x_m"])
    eroded_length = float(motion["contact_patch_half_length_m"]) - float(
        motion["support_margin_m"]
    )
    eroded_width = float(motion["contact_patch_half_width_m"]) - float(
        motion["support_margin_m"]
    )
    if eroded_length <= 0.0 or eroded_width <= 0.0:
        raise ValueError("support margin erases the sole patch")
    initial_centers = feet[0, :, :2].copy()
    initial_centers[:, 0] += center_x
    terminal_centers = initial_centers[[RIGHT]]
    opening = convex_hull(
        sole_vertices(initial_centers, eroded_length, eroded_width)
    )
    terminal = convex_hull(
        sole_vertices(terminal_centers, eroded_length, eroded_width)
    )
    swing_height = float(
        np.max(feet[liftoff : touchdown + 1, LEFT, 2]) - feet[0, LEFT, 2]
    )
    support_plane_z = float(
        np.mean(feet[0, :, 2]) + float(motion["contact_patch_z_m"])
    )
    return {
        "ticks": ticks,
        "dt_seconds": DT,
        "liftoff_tick": liftoff,
        "touchdown_tick": touchdown,
        "swing_foot": LEFT,
        "terminal_support_foot": RIGHT,
        "gravity_mps2": GRAVITY,
        "com_height_m": float(com[0, 2] - support_plane_z),
        "initial_com": com[0].tolist(),
        "initial_com_velocity": com_velocity[0].tolist(),
        "initial_root": root[0].tolist(),
        "initial_feet": feet[0].tolist(),
        "swing_touchdown": feet[touchdown, LEFT].tolist(),
        "swing_height_m": swing_height,
        "contact_patch_center_x_m": center_x,
        "maximum_root_to_landing_reach_m": maximum_root_to_landing_reach_m,
        "root_horizontal_follow_ratio": root_horizontal_follow_ratio,
        "cop_transition_duration_seconds": 0.10,
        "opening_support_ccw": opening.tolist(),
        "terminal_support_ccw": terminal.tolist(),
        "minimum_switch_ratio": 0.05,
        "maximum_switch_ratio": 0.95,
        "switch_candidates": 91,
        "minimum_cop_margin_m": 0.0,
    }


def run_generator(binary: Path, request: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started = time.perf_counter_ns()
    completed = subprocess.run(
        [str(binary)],
        input=json.dumps(request),
        text=True,
        capture_output=True,
        check=False,
    )
    wall_ns = time.perf_counter_ns() - started
    usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    if completed.returncode != 0:
        raise RuntimeError(
            f"Rust LIPM generator failed with code {completed.returncode}:\n"
            f"{completed.stderr}"
        )
    payload = json.loads(completed.stdout)
    process = {
        "process_wall_ns": wall_ns,
        "child_user_cpu_seconds": usage_after.ru_utime - usage_before.ru_utime,
        "child_system_cpu_seconds": usage_after.ru_stime - usage_before.ru_stime,
        "child_maximum_rss_kib": usage_after.ru_maxrss,
        "stderr": completed.stderr.splitlines(),
    }
    return payload, process


def main() -> None:
    args = parse_args()
    if args.ticks <= 0:
        raise SystemExit("ticks must be positive")
    root = Path(args.results_root)
    source = root / args.source
    source_payload = json.loads((source / "floating-walk-metrics.json").read_text())
    source_raw = np.load(source / "floating-walk-raw.npz")
    if (
        not np.isfinite(args.maximum_root_to_landing_reach)
        or args.maximum_root_to_landing_reach <= 0.0
    ):
        raise SystemExit("--maximum-root-to-landing-reach must be positive")
    if not np.isfinite(args.root_horizontal_follow_ratio) or not (
        0.0 <= args.root_horizontal_follow_ratio <= 1.0
    ):
        raise SystemExit("--root-horizontal-follow-ratio must be in [0, 1]")
    request = build_request(
        source_raw,
        source_payload["motion"],
        args.ticks,
        args.maximum_root_to_landing_reach,
        args.root_horizontal_follow_ratio,
    )
    payload, process = run_generator(Path(args.binary), request)

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    array_keys = (
        "root_targets",
        "root_target_velocities",
        "root_target_accelerations",
        "center_of_mass_targets",
        "center_of_mass_target_velocities",
        "center_of_mass_target_accelerations",
        "target_positions",
        "target_velocities",
        "target_accelerations",
        "reference_stance",
    )
    arrays = {
        key: np.asarray(
            payload[key], dtype=np.uint8 if key == "reference_stance" else np.float64
        )
        for key in array_keys
    }
    np.savez_compressed(output / "reference-inputs.npz", **arrays)
    motion = {
        key: source_payload["motion"][key]
        for key in (
            "contact_patch_center_x_m",
            "contact_patch_half_length_m",
            "contact_patch_half_width_m",
            "contact_patch_z_m",
            "support_margin_m",
        )
    }
    metadata = {
        "schema": 1,
        "implementation": payload["implementation"],
        "generator_family": "rust-native-lipm",
        "bonesaw_version": "0.1.0",
        "bonesaw_license": "Apache-2.0",
        "planner_source": "crates/bonesaw-core/src/lipm.rs",
        "policy_or_physics_rollout": payload["policy_or_physics_rollout"],
        "ik_or_wbc_solve": payload["ik_or_wbc_solve"],
        "exact_repeat": payload["exact_repeat"],
        "source_reference": args.source,
        "ticks": args.ticks,
        "dt_seconds": DT,
        "matched_liftoff_tick": request["liftoff_tick"],
        "matched_touchdown_tick": request["touchdown_tick"],
        "runtime": {**payload["runtime"], **process},
        "plan": payload["plan"],
        "motion": motion,
    }
    (output / "reference-metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(output / "reference-inputs.npz")


if __name__ == "__main__":
    main()
