#!/usr/bin/env python3
"""Compose an alternating G1 walk from Rust support-constrained LIPM segments.

Python owns the evaluation sequence and artifact format.  Every horizontal CoM
transfer and swing-foot jet is planned and sampled by the Rust reference binary.
There is no feedback policy, state integration, contact simulation, or WBC solve
in this generator; those remain separate evaluation stages.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import time
from typing import Any

import numpy as np

from g1_reference_contract import convex_hull, sole_vertices


DT = 0.005
ARRAY_KEYS = (
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--seed-reference",
        default="benchmarks/results/g1-bonesaw-lipm-r44/reference-inputs.npz",
    )
    parser.add_argument(
        "--seed-metadata",
        default="benchmarks/results/g1-bonesaw-lipm-r44/reference-metadata.json",
    )
    parser.add_argument(
        "--binary", default="target/release/bonesaw-lipm-reference"
    )
    parser.add_argument(
        "--output", default="benchmarks/results/g1-multistep-reference-r53"
    )
    parser.add_argument("--steps", type=int, default=4)
    parser.add_argument("--transfer-ticks", type=int, default=300)
    parser.add_argument("--swing-ticks", type=int, default=229)
    parser.add_argument("--settle-ticks", type=int, default=50)
    parser.add_argument("--step-length", type=float, default=0.062)
    parser.add_argument("--swing-clearance", type=float, default=0.048)
    parser.add_argument(
        "--root-height-offset",
        type=float,
        default=-0.02,
        help="vertical offset applied to the seed root and CoM before planning",
    )
    parser.add_argument("--support-margin", type=float, default=0.005)
    parser.add_argument("--maximum-root-to-landing-reach", type=float, default=0.765)
    parser.add_argument("--root-horizontal-follow-ratio", type=float, default=0.80)
    parser.add_argument("--cop-startup-duration", type=float, default=0.30)
    parser.add_argument("--cop-transition-duration", type=float, default=0.30)
    parser.add_argument("--switch-candidates", type=int, default=193)
    return parser.parse_args()


def fingerprint(arrays: dict[str, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for key in ARRAY_KEYS:
        value = np.ascontiguousarray(arrays[key])
        digest.update(key.encode())
        digest.update(str(value.dtype).encode())
        digest.update(str(value.shape).encode())
        digest.update(value.view(np.uint8))
    return digest.hexdigest()


def run_segment(binary: pathlib.Path, request: dict[str, Any]) -> dict[str, Any]:
    completed = subprocess.run(
        [str(binary)],
        input=json.dumps(request),
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"Rust LIPM segment failed with code {completed.returncode}:\n"
            f"{completed.stderr}"
        )
    return json.loads(completed.stdout)


def sequence_request(
    *,
    feet: np.ndarray,
    center_of_mass: np.ndarray,
    center_of_mass_velocity: np.ndarray,
    root: np.ndarray,
    swing_foot: int,
    motion: dict[str, float],
    args: argparse.Namespace,
) -> dict[str, Any]:
    support_foot = 1 - swing_foot
    touchdown = feet[swing_foot].copy()
    touchdown[0] = float(np.max(feet[:, 0]) + args.step_length)
    contact_center = float(motion["contact_patch_center_x_m"])
    half_length = float(motion["contact_patch_half_length_m"])
    half_width = float(motion["contact_patch_half_width_m"])
    patch_z = float(motion["contact_patch_z_m"])
    eroded_length = half_length - args.support_margin
    eroded_width = half_width - args.support_margin
    if eroded_length <= 0.0 or eroded_width <= 0.0:
        raise ValueError("support margin erases the foot patch")
    centers = feet[:, :2].copy()
    centers[:, 0] += contact_center
    support_plane_z = float(np.mean(feet[:, 2]) + patch_z)
    touchdown_tick = args.transfer_ticks + args.swing_ticks
    return {
        "ticks": touchdown_tick + args.settle_ticks + 1,
        "dt_seconds": DT,
        "liftoff_tick": args.transfer_ticks,
        "touchdown_tick": touchdown_tick,
        "swing_foot": swing_foot,
        "terminal_support_foot": support_foot,
        "gravity_mps2": 9.81,
        "com_height_m": float(center_of_mass[2] - support_plane_z),
        "initial_com": center_of_mass.tolist(),
        "initial_com_velocity": center_of_mass_velocity.tolist(),
        "initial_root": root.tolist(),
        "initial_feet": feet.tolist(),
        "swing_touchdown": touchdown.tolist(),
        "swing_height_m": args.swing_clearance,
        "contact_patch_center_x_m": contact_center,
        "root_horizontal_follow_ratio": args.root_horizontal_follow_ratio,
        "maximum_root_to_landing_reach_m": args.maximum_root_to_landing_reach,
        "cop_startup_duration_seconds": args.cop_startup_duration,
        "cop_transition_duration_seconds": args.cop_transition_duration,
        "opening_support_ccw": convex_hull(
            sole_vertices(centers, eroded_length, eroded_width)
        ).tolist(),
        "terminal_support_ccw": convex_hull(
            sole_vertices(centers[[support_foot]], eroded_length, eroded_width)
        ).tolist(),
        "minimum_switch_ratio": 0.02,
        "maximum_switch_ratio": 0.98,
        "switch_candidates": args.switch_candidates,
        "minimum_cop_margin_m": 0.0,
    }


def generate_sequence(
    args: argparse.Namespace,
    seed: dict[str, np.ndarray],
    motion: dict[str, float],
) -> tuple[dict[str, np.ndarray], list[dict[str, Any]], list[int]]:
    binary = pathlib.Path(args.binary).resolve()
    feet = seed["target_positions"][0, :2].astype(np.float64, copy=True)
    center_of_mass = seed["center_of_mass_targets"][0].astype(
        np.float64, copy=True
    )
    center_of_mass_velocity = seed["center_of_mass_target_velocities"][0].astype(
        np.float64, copy=True
    )
    root = seed["root_targets"][0].astype(np.float64, copy=True)
    chunks: dict[str, list[np.ndarray]] = {key: [] for key in ARRAY_KEYS}
    reports: list[dict[str, Any]] = []
    segment_start_ticks: list[int] = []
    output_ticks = 0
    for step in range(args.steps):
        request = sequence_request(
            feet=feet,
            center_of_mass=center_of_mass,
            center_of_mass_velocity=center_of_mass_velocity,
            root=root,
            swing_foot=step % 2,
            motion=motion,
            args=args,
        )
        response = run_segment(binary, request)
        segment = {
            key: np.asarray(
                response[key],
                dtype=np.uint8 if key == "reference_stance" else np.float64,
            )
            for key in ARRAY_KEYS
        }
        # The boundary state is present at both the end of the previous segment
        # and tick zero of the next one.  Keep it once so time remains uniform.
        start = 0 if step == 0 else 1
        segment_start_ticks.append(output_ticks)
        for key in ARRAY_KEYS:
            chunks[key].append(segment[key][start:])
        output_ticks += len(segment["root_targets"]) - start
        feet = segment["target_positions"][-1].copy()
        center_of_mass = segment["center_of_mass_targets"][-1].copy()
        center_of_mass_velocity = segment[
            "center_of_mass_target_velocities"
        ][-1].copy()
        root = segment["root_targets"][-1].copy()
        reports.append(
            {
                "step": step,
                "swing_foot": step % 2,
                "segment_ticks": len(segment["root_targets"]),
                "global_start_tick": segment_start_ticks[-1],
                "global_liftoff_tick": segment_start_ticks[-1]
                + args.transfer_ticks
                - (1 if step else 0),
                "global_touchdown_tick": segment_start_ticks[-1]
                + args.transfer_ticks
                + args.swing_ticks
                - (1 if step else 0),
                "exact_repeat": bool(response["exact_repeat"]),
                "initial_center_of_mass_acceleration_mps2": float(
                    np.linalg.norm(segment["center_of_mass_target_accelerations"][0])
                ),
                "runtime": response["runtime"],
                "plan": response["plan"],
            }
        )
    return (
        {key: np.concatenate(chunks[key], axis=0) for key in ARRAY_KEYS},
        reports,
        segment_start_ticks,
    )


def maximum_boundary_delta(values: np.ndarray, starts: list[int]) -> float:
    if len(starts) <= 1:
        return 0.0
    return max(
        float(np.max(np.abs(values[start] - values[start - 1])))
        for start in starts[1:]
    )


def main() -> int:
    args = parse_args()
    if (
        args.steps < 2
        or args.transfer_ticks < 2
        or args.swing_ticks < 2
        or args.settle_ticks < 1
        or args.switch_candidates < 2
    ):
        raise ValueError("multi-step timing and candidate counts are too small")
    for value in (
        args.step_length,
        args.swing_clearance,
        args.root_height_offset,
        args.support_margin,
        args.maximum_root_to_landing_reach,
        args.root_horizontal_follow_ratio,
        args.cop_startup_duration,
        args.cop_transition_duration,
    ):
        if not np.isfinite(value):
            raise ValueError("multi-step configuration must be finite")
    if (
        args.step_length <= 0.0
        or args.swing_clearance < 0.0
        or args.support_margin < 0.0
        or args.maximum_root_to_landing_reach <= 0.0
        or not 0.0 <= args.root_horizontal_follow_ratio <= 1.0
        or args.cop_startup_duration <= 0.0
        or args.cop_transition_duration < 0.0
        or args.cop_startup_duration + args.cop_transition_duration
        >= args.transfer_ticks * DT
    ):
        raise ValueError("multi-step configuration is outside its valid range")

    with np.load(pathlib.Path(args.seed_reference)) as source:
        seed = {key: np.asarray(source[key]).copy() for key in source.files}
    seed["root_targets"][:, 2] += args.root_height_offset
    seed["center_of_mass_targets"][:, 2] += args.root_height_offset
    seed_metadata = json.loads(pathlib.Path(args.seed_metadata).read_text())
    motion = {
        key: float(seed_metadata["motion"][key])
        for key in (
            "contact_patch_center_x_m",
            "contact_patch_half_length_m",
            "contact_patch_half_width_m",
            "contact_patch_z_m",
        )
    }

    started = time.perf_counter_ns()
    arrays, segments, starts = generate_sequence(args, seed, motion)
    wall_ns = time.perf_counter_ns() - started
    repeat_arrays, repeat_segments, repeat_starts = generate_sequence(args, seed, motion)
    exact_repeat = starts == repeat_starts and all(
        np.array_equal(arrays[key], repeat_arrays[key]) for key in ARRAY_KEYS
    )
    stance = arrays["reference_stance"].astype(bool)
    liftoff = np.argwhere(stance[:-1] & ~stance[1:])
    touchdown = np.argwhere(~stance[:-1] & stance[1:])
    liftoff_feet = liftoff[:, 1].astype(int).tolist()
    touchdown_feet = touchdown[:, 1].astype(int).tolist()
    stance_velocity = np.where(
        stance[:, :, None], arrays["target_velocities"], 0.0
    )
    touchdown_speeds = [
        float(np.linalg.norm(arrays["target_velocities"][tick, foot]))
        for tick, foot in touchdown
    ]
    net_progress = float(
        np.mean(arrays["target_positions"][-1, :, 0])
        - np.mean(arrays["target_positions"][0, :, 0])
    )
    checks = {
        "at_least_two_steps": args.steps >= 2,
        "one_liftoff_and_touchdown_per_step": len(liftoff) == args.steps
        and len(touchdown) == args.steps,
        "strictly_alternating_swing_feet": liftoff_feet
        == [step % 2 for step in range(args.steps)],
        "touchdown_matches_liftoff_order": touchdown_feet == liftoff_feet,
        "no_flight_ticks": bool(np.all(np.any(stance, axis=1))),
        "stance_foot_velocity_is_zero": float(np.max(np.abs(stance_velocity)))
        <= 1e-12,
        "touchdown_speed_le_0_02mps": max(touchdown_speeds, default=0.0)
        <= 0.02,
        "segment_position_boundary_delta_le_0_1mm": max(
            maximum_boundary_delta(arrays["root_targets"], starts),
            maximum_boundary_delta(arrays["center_of_mass_targets"], starts),
            maximum_boundary_delta(arrays["target_positions"], starts),
        )
        <= 1e-4,
        "segment_velocity_boundary_delta_le_0_02mps": max(
            maximum_boundary_delta(arrays["root_target_velocities"], starts),
            maximum_boundary_delta(
                arrays["center_of_mass_target_velocities"], starts
            ),
            maximum_boundary_delta(arrays["target_velocities"], starts),
        )
        <= 0.02,
        "all_rust_segments_repeat_exactly": all(
            segment["exact_repeat"] for segment in segments
        ),
        "complete_sequence_repeats_bitwise": exact_repeat,
        "rust_hot_sampling_has_zero_allocations": sum(
            int(segment["runtime"]["sample_allocation_calls"])
            for segment in segments
        )
        == 0,
        "each_transfer_starts_at_zero_com_acceleration": max(
            segment["initial_center_of_mass_acceleration_mps2"]
            for segment in segments
        )
        <= 1e-12,
        "net_forward_progress_ge_0_20m": net_progress >= 0.20,
    }
    checks = {key: bool(value) for key, value in checks.items()}
    metadata = {
        "schema": 1,
        "implementation": "Python-authored alternating sequence of Rust Bonesaw LIPM boundary plans",
        "policy_or_physics_rollout": False,
        "state_integration": False,
        "ik_or_wbc_solve": False,
        "reference_generator": "target/release/bonesaw-lipm-reference",
        "seed_reference": str(pathlib.Path(args.seed_reference).resolve()),
        "ticks": len(stance),
        "dt_seconds": DT,
        "steps": args.steps,
        "contact_transitions": len(liftoff) + len(touchdown),
        "liftoff_feet": liftoff_feet,
        "touchdown_feet": touchdown_feet,
        "net_forward_progress_m": net_progress,
        "maximum_touchdown_speed_mps": max(touchdown_speeds, default=0.0),
        "maximum_stance_foot_speed_mps": float(np.max(np.abs(stance_velocity))),
        "maximum_segment_boundary_delta": {
            "root_position_m": maximum_boundary_delta(
                arrays["root_targets"], starts
            ),
            "center_of_mass_position_m": maximum_boundary_delta(
                arrays["center_of_mass_targets"], starts
            ),
            "foot_position_m": maximum_boundary_delta(
                arrays["target_positions"], starts
            ),
            "root_velocity_mps": maximum_boundary_delta(
                arrays["root_target_velocities"], starts
            ),
            "center_of_mass_velocity_mps": maximum_boundary_delta(
                arrays["center_of_mass_target_velocities"], starts
            ),
            "foot_velocity_mps": maximum_boundary_delta(
                arrays["target_velocities"], starts
            ),
        },
        "exact_repeat": exact_repeat,
        "fingerprint": fingerprint(arrays),
        "runtime": {
            "first_sequence_wall_ns": wall_ns,
            "rust_plan_ns": sum(
                int(segment["runtime"]["plan_ns"]) for segment in segments
            ),
            "rust_sample_ns": sum(
                int(segment["runtime"]["sample_ns"]) for segment in segments
            ),
            "rust_plan_allocation_calls": sum(
                int(segment["runtime"]["plan_allocation_calls"])
                for segment in segments
            ),
            "rust_sample_allocation_calls": sum(
                int(segment["runtime"]["sample_allocation_calls"])
                for segment in segments
            ),
            "rust_sample_allocated_bytes": sum(
                int(segment["runtime"]["sample_allocated_bytes"])
                for segment in segments
            ),
        },
        "configuration": {
            "transfer_ticks": args.transfer_ticks,
            "swing_ticks": args.swing_ticks,
            "settle_ticks": args.settle_ticks,
            "step_length_m": args.step_length,
            "swing_clearance_m": args.swing_clearance,
            "root_height_offset_m": args.root_height_offset,
            "support_margin_m": args.support_margin,
            "maximum_root_to_landing_reach_m": args.maximum_root_to_landing_reach,
            "root_horizontal_follow_ratio": args.root_horizontal_follow_ratio,
            "cop_startup_duration_seconds": args.cop_startup_duration,
            "cop_transition_duration_seconds": args.cop_transition_duration,
        },
        "motion": {**motion, "support_margin_m": args.support_margin},
        "segments": segments,
        "checks": checks,
        "passed": all(checks.values()),
    }
    if [segment["plan"] for segment in segments] != [
        segment["plan"] for segment in repeat_segments
    ]:
        raise RuntimeError("repeated Rust plan metadata is not deterministic")

    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "reference-inputs.npz", **arrays)
    (output / "reference-metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n"
    )
    report = [
        "# G1 alternating multi-step reference",
        "",
        f"**{'PASS' if metadata['passed'] else 'RED'} · {sum(checks.values())}/{len(checks)} gates**",
        "",
        "This is an offline, policy-free and physics-free reference. Python authors the alternating sequence; Rust plans and analytically samples every support transfer and swing jet. No integrated robot state or simulated contact is used.",
        "",
        f"- `{args.steps}` alternating steps, `{metadata['contact_transitions']}` contact edges, `{len(stance)}` ticks / `{len(stance) * DT:.3f} s`.",
        f"- Net foot-center progress: `{net_progress:.3f} m`.",
        f"- Maximum touchdown speed: `{metadata['maximum_touchdown_speed_mps']:.6f} m/s`; stance-foot speed: `{metadata['maximum_stance_foot_speed_mps']:.2e} m/s`.",
        f"- Rust analytic sampling: `{metadata['runtime']['rust_sample_ns'] / 1e6:.3f} ms` total, `{metadata['runtime']['rust_sample_allocation_calls']}` allocations / `{metadata['runtime']['rust_sample_allocated_bytes']}` B.",
        f"- Complete reference bitwise repeat: `{exact_repeat}` (`{metadata['fingerprint']}`).",
        "",
        "## Per-step plan",
        "",
        "| step | swing | global liftoff | global touchdown | first / second CoP margin | applied landing x | retargeted |",
        "|---:|---:|---:|---:|---:|---:|---:|",
        *[
            f"| {segment['step']} | {'left' if segment['swing_foot'] == 0 else 'right'} | {segment['global_liftoff_tick']} | {segment['global_touchdown_tick']} | {segment['plan']['first_cop_margin_m'] * 1000:.2f} / {segment['plan']['second_cop_margin_m'] * 1000:.2f} mm | {segment['plan']['applied_touchdown'][0]:.3f} m | {segment['plan']['touchdown_was_retargeted']} |"
            for segment in segments
        ],
        "",
        "## Predeclared gates",
        "",
        *[f"- {'PASS' if passed else 'FAIL'} `{name}`" for name, passed in checks.items()],
        "",
    ]
    (output / "MULTISTEP_REFERENCE.md").write_text("\n".join(report))
    print(
        json.dumps(
            {"passed": metadata["passed"], "checks": checks, "output": str(output)},
            indent=2,
        )
    )
    return 0 if metadata["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
