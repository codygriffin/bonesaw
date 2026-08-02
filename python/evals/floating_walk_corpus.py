#!/usr/bin/env python3
"""Moving-root, contact-aware CMU walking corpus for the Rust floating WBC."""

from __future__ import annotations

import argparse
import dataclasses
import gc
import json
import math
import os
import pathlib
import platform
import resource
import sys
import time
import tracemalloc
from typing import Any

import numpy as np

from cmu_mocap import FloatingRetargetedWalk, retarget_subject_37_walk_floating
from reference_comparison import standing_posture


DT = 0.005
PRIORITY_NAMES = ("invariant", "viability", "intent", "preference", "style")
SUPPORT_PHASE_NAMES = (
    "swing",
    "precontact",
    "touchdown_normal",
    "locked",
    "normal_fallback",
)
FRAME_NAMES = (
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "left_wrist_roll_rubber_hand",
    "right_wrist_roll_rubber_hand",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf",
    )
    parser.add_argument("--left-foot-frame", default=FRAME_NAMES[0])
    parser.add_argument("--right-foot-frame", default=FRAME_NAMES[1])
    parser.add_argument("--left-hand-frame", default=FRAME_NAMES[2])
    parser.add_argument("--right-hand-frame", default=FRAME_NAMES[3])
    parser.add_argument("--cmu-cache", default="benchmarks/cache/cmu-37")
    parser.add_argument(
        "--motion-profile",
        choices=("cmu-walk", "synthetic-step"),
        default="cmu-walk",
    )
    parser.add_argument(
        "--reference-inputs",
        help="immutable standalone reference-inputs.npz admitted by the open-loop contract",
    )
    parser.add_argument("--synthetic-step-length", type=float, default=0.04)
    parser.add_argument("--synthetic-step-clearance", type=float, default=0.03)
    parser.add_argument("--synthetic-swing-ticks", type=int, default=40)
    parser.add_argument(
        "--synthetic-root-transfer-scale", type=float, default=0.0
    )
    parser.add_argument("--ticks", type=int, default=6_400)
    parser.add_argument("--friction", type=float, default=1.0)
    parser.add_argument("--maximum-acceleration", type=float, default=200.0)
    parser.add_argument("--maximum-torque", type=float, default=2_000.0)
    parser.add_argument("--maximum-normal-force-multiple", type=float, default=3.0)
    parser.add_argument(
        "--maximum-feasibility-iterations",
        type=int,
        default=64,
        help="bounded active-set polish iterations per WBC query",
    )
    parser.add_argument(
        "--maximum-feasibility-projection-sweeps",
        type=int,
        default=None,
        help="optional total Dykstra sweeps per WBC query; finite values fail closed when exhausted",
    )
    parser.add_argument(
        "--feasibility-projection-continuation-violation-threshold",
        type=float,
        default=None,
        help="optional normalized violation threshold for continuing a bounded Dykstra prefix",
    )
    parser.add_argument("--joint-limit-braking", action="store_true")
    parser.add_argument("--root-frequency-hz", type=float, default=2.0)
    parser.add_argument("--root-angular-task-weight", type=float, default=1.0)
    parser.add_argument("--root-height-task-weight", type=float, default=10.0)
    parser.add_argument("--root-horizontal-task-weight", type=float, default=1.0)
    parser.add_argument(
        "--root-horizontal-task-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=2,
        help="strict priority for pelvis horizontal intent; DCM/CoM viability may remain above it",
    )
    parser.add_argument("--point-frequency-hz", type=float, default=4.0)
    parser.add_argument("--joint-posture-weight", type=float, default=0.25)
    parser.add_argument(
        "--joint-posture-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=3,
    )
    parser.add_argument("--center-of-mass-task-weight", type=float, default=0.0)
    parser.add_argument(
        "--center-of-mass-task-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=1,
    )
    parser.add_argument("--center-of-mass-frequency-hz", type=float, default=2.0)
    parser.add_argument(
        "--center-of-mass-controller",
        choices=("tracking", "dcm-zmp"),
        default="tracking",
        help="Rust horizontal balance law; dcm-zmp derives DCM from the CoM jet and clips virtual ZMP to measured support",
    )
    parser.add_argument("--dcm-feedback-gain-per-second", type=float, default=3.5)
    parser.add_argument("--dcm-support-margin", type=float, default=0.01)
    parser.add_argument(
        "--dcm-maximum-horizontal-acceleration", type=float, default=25.0
    )
    parser.add_argument(
        "--center-of-mass-zero-reference-derivatives",
        action="store_true",
        help="track only the CoM position reference; zero preview velocity/acceleration jets",
    )
    parser.add_argument(
        "--centroidal-angular-momentum-weight", type=float, default=0.0
    )
    parser.add_argument(
        "--centroidal-angular-momentum-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=2,
    )
    parser.add_argument(
        "--centroidal-angular-momentum-frequency-hz", type=float, default=1.0
    )
    parser.add_argument("--upper-body-posture-weight", type=float, default=0.0)
    parser.add_argument(
        "--protected-posture-include-leg-yaw",
        action="store_true",
        help="include hip-yaw coordinates in the protected posture task",
    )
    parser.add_argument(
        "--upper-body-posture-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=2,
    )
    parser.add_argument("--joint-velocity-envelope-weight", type=float, default=0.0)
    parser.add_argument(
        "--joint-velocity-envelope-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=1,
    )
    parser.add_argument(
        "--joint-velocity-envelope-activation-fraction", type=float, default=0.75
    )
    parser.add_argument(
        "--joint-velocity-envelope-frequency-hz", type=float, default=2.0
    )
    parser.add_argument(
        "--joint-velocity-envelope-phase-policy",
        choices=("always", "multi-support", "feedback"),
        default="always",
        help="schedule the Rust velocity-envelope task from measured support state",
    )
    parser.add_argument(
        "--joint-velocity-envelope-phase-transition-ticks",
        type=int,
        default=0,
        help="bounded ticks to release authority after leaving its measured phase; engagement is immediate",
    )
    parser.add_argument(
        "--center-of-mass-reference",
        choices=(
            "rooted",
            "support-preview",
            "support-centroid-preview",
            "dcm-backward-preview",
        ),
        default="rooted",
    )
    parser.add_argument(
        "--root-reference",
        choices=("mocap", "support-preview"),
        default="mocap",
    )
    parser.add_argument("--support-preview-ticks", type=int, default=40)
    parser.add_argument("--support-margin", type=float, default=0.01)
    parser.add_argument("--support-reference-blend", type=float, default=1.0)
    parser.add_argument("--precontact-ticks", type=int, default=0)
    parser.add_argument(
        "--precontact-maximum-acceleration", type=float, default=25.0
    )
    parser.add_argument(
        "--capture-landing-retarget",
        action="store_true",
        help="retarget the latched sole-center landing in Rust from measured DCM",
    )
    parser.add_argument("--capture-landing-activation-margin", type=float, default=0.0)
    parser.add_argument("--capture-landing-full-scale-margin", type=float, default=-0.10)
    parser.add_argument("--capture-landing-maximum-offset", type=float, default=0.12)
    parser.add_argument("--capture-landing-maximum-root-reach", type=float, default=0.90)
    parser.add_argument("--capture-landing-maximum-anchor-speed", type=float, default=0.50)
    parser.add_argument("--capture-landing-freeze-ticks", type=int, default=40)
    parser.add_argument(
        "--touchdown-phase-retiming",
        action="store_true",
        help="let Rust slow one coherent reference phase from measured touchdown viability",
    )
    parser.add_argument("--touchdown-phase-minimum-rate", type=float, default=0.0)
    parser.add_argument(
        "--touchdown-phase-guard-time-seconds", type=float, default=0.02
    )
    parser.add_argument("--touchdown-phase-release-ticks", type=int, default=100)
    parser.add_argument("--touchdown-phase-engagement-ticks", type=int, default=0)
    parser.add_argument(
        "--balance-phase-retiming",
        action="store_true",
        help="slow the same Rust reference cursor from measured signed DCM support margin",
    )
    parser.add_argument("--balance-phase-hold-margin", type=float, default=-0.02)
    parser.add_argument("--balance-phase-full-rate-margin", type=float, default=0.0)
    parser.add_argument("--touchdown-blend-ticks", type=int, default=0)
    parser.add_argument("--material-touchdown-task", action="store_true")
    parser.add_argument("--contact-patch-center-x", type=float, default=0.035)
    parser.add_argument("--contact-patch-half-length", type=float, default=0.085)
    parser.add_argument("--contact-patch-half-width", type=float, default=0.0275)
    parser.add_argument("--contact-patch-z", type=float, default=-0.035)
    parser.add_argument(
        "--minimum-contact-cop-margin",
        type=float,
        default=0.0,
        help="hard inward erosion applied independently to every loaded finite sole",
    )
    parser.add_argument("--foot-task-weight", type=float, default=1.0)
    parser.add_argument(
        "--foot-task-priority",
        type=int,
        choices=range(len(PRIORITY_NAMES)),
        default=1,
        help="lexicographic priority for non-contact foot tracking",
    )
    parser.add_argument("--hand-task-weight", type=float, default=0.0)
    parser.add_argument("--forward-motion-scale", type=float, default=0.35)
    parser.add_argument("--lateral-motion-scale", type=float, default=0.35)
    parser.add_argument(
        "--maximum-root-to-foot-reach",
        type=float,
        default=0.0,
        help="project retargeted touchdown endpoints to this root-frame reach; 0 disables",
    )
    parser.add_argument("--startup-balance-ticks", type=int, default=160)
    parser.add_argument("--startup-ramp-ticks", type=int, default=200)
    parser.add_argument("--cadence-multiplier", type=float, default=1.0)
    parser.add_argument(
        "--first-liftoff-hold-ticks",
        type=int,
        default=0,
        help="insert a whole-reference double-support hold at the first authored liftoff",
    )
    parser.add_argument(
        "--first-liftoff-leg-phase-gate-ticks",
        type=int,
        default=0,
        help="eval-only: advance root/CoM intent while delaying both leg/contact phases",
    )
    parser.add_argument(
        "--output", default="benchmarks/results/floating-walk-latest"
    )
    return parser.parse_args()


def finite_difference(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    edge_order = 2 if len(values) >= 3 else 1
    velocity = np.gradient(values, DT, axis=0, edge_order=edge_order)
    acceleration = np.gradient(velocity, DT, axis=0, edge_order=edge_order)
    return velocity, acceleration


def synthetic_g1_step(
    initial_positions: np.ndarray,
    root_translation: np.ndarray,
    ticks: int,
    *,
    step_length: float,
    step_clearance: float,
    root_transfer_scale: float,
    swing_duration_ticks: int,
) -> FloatingRetargetedWalk:
    """Create a modest, deterministic left-foot step for controller acceptance.

    This is deliberately not a learned or mocap-derived motion.  Its contact
    schedule and zero-endpoint-velocity trajectory are fixed before running the
    controller, making failures attributable to the WBC rather than retargeting
    ambiguity.
    """
    if ticks < 160:
        raise ValueError("synthetic-step requires at least 160 ticks")
    if initial_positions.shape != (4, 3):
        raise ValueError("synthetic-step expects four initial effectors")
    if not np.isfinite(step_length) or step_length <= 0.0:
        raise ValueError("synthetic step length must be finite and positive")
    if not np.isfinite(step_clearance) or step_clearance <= 0.0:
        raise ValueError("synthetic step clearance must be finite and positive")
    if not np.isfinite(root_transfer_scale) or not 0.0 <= root_transfer_scale <= 1.0:
        raise ValueError("synthetic root transfer scale must be in [0, 1]")
    if swing_duration_ticks < 20:
        raise ValueError("synthetic swing duration must be at least 20 ticks")

    liftoff_tick = ticks // 3
    touchdown_tick = liftoff_tick + swing_duration_ticks
    if touchdown_tick > ticks - 40:
        raise ValueError("synthetic-step needs at least 40 post-touchdown ticks")

    targets = np.broadcast_to(initial_positions, (ticks, 4, 3)).copy()
    stance = np.ones((ticks, 2), dtype=np.uint8)
    stance[liftoff_tick:touchdown_tick, 0] = 0
    swing_ticks = np.arange(
        liftoff_tick, touchdown_tick + 1, dtype=np.int64
    )


    phase = (swing_ticks - liftoff_tick) / (touchdown_tick - liftoff_tick)
    smootherstep = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    lift = 16.0 * phase**2 * (1.0 - phase) ** 2
    targets[swing_ticks, 0, 0] += step_length * smootherstep
    targets[swing_ticks, 0, 2] += step_clearance * lift
    targets[touchdown_tick + 1 :, 0, 0] += step_length

    root_targets = np.broadcast_to(root_translation, (ticks, 3)).copy()
    transfer_start = max(20, ticks // 20)
    transfer_stop = liftoff_tick - 10
    support_root = root_translation.copy()
    support_root[1] += root_transfer_scale * (
        initial_positions[1, 1] - root_translation[1]
    )
    transfer_ticks = np.arange(transfer_start, transfer_stop + 1)
    transfer_phase = (transfer_ticks - transfer_start) / (
        transfer_stop - transfer_start
    )
    transfer_blend = (
        10.0 * transfer_phase**3
        - 15.0 * transfer_phase**4
        + 6.0 * transfer_phase**5
    )[:, None]
    root_targets[transfer_ticks] = (
        (1.0 - transfer_blend) * root_translation
        + transfer_blend * support_root
    )
    root_targets[transfer_stop + 1 :] = support_root

    recenter_start = touchdown_tick + 10
    recenter_stop = ticks - 20
    final_root = root_translation.copy()
    final_root[0] += root_transfer_scale * 0.5 * step_length
    recenter_ticks = np.arange(recenter_start, recenter_stop + 1)
    recenter_phase = (recenter_ticks - recenter_start) / (
        recenter_stop - recenter_start
    )
    recenter_blend = (
        10.0 * recenter_phase**3
        - 15.0 * recenter_phase**4
        + 6.0 * recenter_phase**5
    )[:, None]
    root_targets[recenter_ticks] = (
        (1.0 - recenter_blend) * support_root
        + recenter_blend * final_root
    )
    root_targets[recenter_stop + 1 :] = final_root
    cadence_scale = np.ones(ticks, dtype=np.float64)
    source_phase_frames = np.arange(ticks, dtype=np.float64)
    duration_seconds = ticks * DT
    swing_seconds = (touchdown_tick - liftoff_tick) * DT
    return FloatingRetargetedWalk(
        root_targets=root_targets,
        targets=targets,
        stance=stance,
        cadence_scale=cadence_scale,
        source_phase_frames=source_phase_frames,
        metadata={
            "profile": "synthetic_g1_step",
            "motion_profile": "synthetic-step",
            "subject": "deterministic",
            "trial": "left-step-v1",
            "description": "predeclared G1-sized single-step acceptance trace",
            "source_rate_hz": 1.0 / DT,
            "cycle_duration_seconds": duration_seconds,
            "target_stride_displacement_m": [step_length, 0.0, 0.0],
            "target_mean_forward_speed_m_s": step_length / duration_seconds,
            "cadence_scale_pattern": "constant synthetic phase rate",
            "cadence_block_seconds": duration_seconds,
            "forward_motion_scale": 1.0,
            "lateral_motion_scale": 1.0,
            "synthetic_liftoff_tick": liftoff_tick,
            "synthetic_touchdown_tick": touchdown_tick,
            "synthetic_root_transfer_ticks": [transfer_start, transfer_stop],
            "synthetic_root_recenter_ticks": [recenter_start, recenter_stop],
            "synthetic_support_root_translation_m": support_root.tolist(),
            "synthetic_final_root_translation_m": final_root.tolist(),
            "synthetic_swing_seconds": swing_seconds,
            "synthetic_swing_ticks": swing_duration_ticks,
            "synthetic_step_length_m": step_length,
            "synthetic_step_clearance_m": step_clearance,
            "synthetic_root_transfer_scale": root_transfer_scale,
            "synthetic_trajectory": (
                "quintic smootherstep forward with quartic zero-endpoint lift"
            ),
            "synthetic_contact_schedule": (
                "double support, right-foot support during left swing, double support"
            ),
        },
    )


@dataclasses.dataclass(frozen=True)
class StandaloneReference:
    walk: FloatingRetargetedWalk
    root_velocities: np.ndarray
    root_accelerations: np.ndarray
    center_of_mass_targets: np.ndarray
    center_of_mass_velocities: np.ndarray
    center_of_mass_accelerations: np.ndarray
    target_positions: np.ndarray
    target_velocities: np.ndarray
    target_accelerations: np.ndarray
    contacts: np.ndarray


def load_standalone_reference(
    path: pathlib.Path,
    initial_positions: np.ndarray,
    root_translation: np.ndarray,
    ticks: int,
) -> StandaloneReference:
    """Load an already-admitted authored trace without rebuilding any jet."""
    raw = np.load(path)
    required = (
        "root_targets",
        "center_of_mass_targets",
        "center_of_mass_target_velocities",
        "center_of_mass_target_accelerations",
        "target_positions",
        "target_velocities",
        "target_accelerations",
        "reference_stance",
    )
    missing = [key for key in required if key not in raw.files]
    if missing:
        raise ValueError(f"standalone reference is missing arrays: {missing}")
    if len(raw["root_targets"]) != ticks:
        raise ValueError(
            "--ticks must exactly match the standalone reference; slicing or padding is forbidden"
        )
    root_targets = np.asarray(raw["root_targets"], dtype=np.float64)
    center_of_mass_targets = np.asarray(
        raw["center_of_mass_targets"], dtype=np.float64
    )
    center_of_mass_velocities = np.asarray(
        raw["center_of_mass_target_velocities"], dtype=np.float64
    )
    center_of_mass_accelerations = np.asarray(
        raw["center_of_mass_target_accelerations"], dtype=np.float64
    )
    feet = np.asarray(raw["target_positions"], dtype=np.float64)
    foot_velocities = np.asarray(raw["target_velocities"], dtype=np.float64)
    foot_accelerations = np.asarray(raw["target_accelerations"], dtype=np.float64)
    stance = np.asarray(raw["reference_stance"], dtype=np.uint8)
    expected_shapes = {
        "root_targets": (ticks, 3),
        "center_of_mass_targets": (ticks, 3),
        "center_of_mass_target_velocities": (ticks, 3),
        "center_of_mass_target_accelerations": (ticks, 3),
        "target_positions": (ticks, 2, 3),
        "target_velocities": (ticks, 2, 3),
        "target_accelerations": (ticks, 2, 3),
        "reference_stance": (ticks, 2),
    }
    arrays = {
        "root_targets": root_targets,
        "center_of_mass_targets": center_of_mass_targets,
        "center_of_mass_target_velocities": center_of_mass_velocities,
        "center_of_mass_target_accelerations": center_of_mass_accelerations,
        "target_positions": feet,
        "target_velocities": foot_velocities,
        "target_accelerations": foot_accelerations,
        "reference_stance": stance,
    }
    mismatched = {
        key: (value.shape, expected_shapes[key])
        for key, value in arrays.items()
        if value.shape != expected_shapes[key]
    }
    if mismatched:
        raise ValueError(f"standalone reference shapes are invalid: {mismatched}")
    if not all(np.all(np.isfinite(value)) for key, value in arrays.items() if key != "reference_stance"):
        raise ValueError("standalone reference contains NaN or infinity")
    if not np.all(np.any(stance.astype(bool), axis=1)):
        raise ValueError("standalone reference contains a flight tick")
    if np.max(np.abs(root_targets[0] - root_translation)) > 1.0e-9:
        raise ValueError("standalone root does not match the initialized G1 root")
    if np.max(np.abs(feet[0] - initial_positions[:2])) > 1.0e-9:
        raise ValueError("standalone soles do not match the initialized G1 frames")

    targets = np.broadcast_to(initial_positions, (ticks, 4, 3)).copy()
    target_velocities = np.zeros_like(targets)
    target_accelerations = np.zeros_like(targets)
    targets[:, :2] = feet
    target_velocities[:, :2] = foot_velocities
    target_accelerations[:, :2] = foot_accelerations
    contacts = np.zeros((ticks, 4), dtype=np.uint8)
    contacts[:, :2] = stance
    metadata_path = path.with_name("reference-metadata.json")
    generator_metadata = (
        json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    )
    step_displacement = targets[-1, 0] - targets[0, 0]
    duration_seconds = ticks * DT
    metadata = {
        "profile": "standalone_admitted_reference",
        "motion_profile": "standalone-reference",
        "subject": "none",
        "trial": "rust-lipm-r42",
        "description": generator_metadata.get(
            "implementation", "standalone authored reference"
        ),
        "source_rate_hz": 1.0 / DT,
        "cycle_duration_seconds": duration_seconds,
        "target_stride_displacement_m": step_displacement.tolist(),
        "target_mean_forward_speed_m_s": float(step_displacement[0] / duration_seconds),
        "cadence_scale_pattern": "immutable authored tick sequence",
        "cadence_block_seconds": duration_seconds,
        "cadence_multiplier": 1.0,
        "forward_motion_scale": 1.0,
        "lateral_motion_scale": 1.0,
        "root_reference": "standalone-authored",
        "center_of_mass_reference": "standalone-authored",
        "standalone_reference_inputs": str(path),
        "standalone_reference_metadata": str(metadata_path),
        "standalone_generator": generator_metadata.get("implementation"),
        "standalone_open_loop_exact_repeat": generator_metadata.get("exact_repeat"),
        "standalone_policy_or_physics_rollout": generator_metadata.get(
            "policy_or_physics_rollout"
        ),
    }
    walk = FloatingRetargetedWalk(
        root_targets=root_targets,
        targets=targets,
        stance=stance,
        cadence_scale=np.ones(ticks, dtype=np.float64),
        source_phase_frames=np.arange(ticks, dtype=np.float64),
        metadata=metadata,
    )
    # The planner keeps a constant CoM-to-root offset, so the authored CoM jet
    # is also the exact root-translation jet.
    return StandaloneReference(
        walk=walk,
        root_velocities=center_of_mass_velocities.copy(),
        root_accelerations=center_of_mass_accelerations.copy(),
        center_of_mass_targets=center_of_mass_targets,
        center_of_mass_velocities=center_of_mass_velocities,
        center_of_mass_accelerations=center_of_mass_accelerations,
        target_positions=targets,
        target_velocities=target_velocities,
        target_accelerations=target_accelerations,
        contacts=contacts,
    )

def anchored_contact_targets(
    targets: np.ndarray,
    stance: np.ndarray,
    initial_positions: np.ndarray,
    touchdown_blend_ticks: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if initial_positions.shape != targets.shape[1:]:
        raise ValueError("initial contact positions must match target effectors")
    if touchdown_blend_ticks < 0:
        raise ValueError("touchdown blend ticks must be nonnegative")
    anchored = targets.copy()
    contact = np.zeros(targets.shape[:2], dtype=np.uint8)
    contact[:, :2] = stance
    for foot in range(2):
        was_stance = bool(stance[0, foot])
        anchor = initial_positions[foot].copy()
        raw_liftoff = targets[0, foot].copy()
        output_liftoff = anchor.copy()
        for tick in range(len(targets)):
            in_stance = bool(stance[tick, foot])
            if in_stance:
                if not was_stance:
                    anchor[:] = anchored[max(0, tick - 1), foot]
                    anchor[2] = initial_positions[foot, 2]
                anchored[tick, foot] = anchor
            else:
                if was_stance:
                    raw_liftoff[:] = targets[tick, foot]
                    output_liftoff[:] = anchor
                delta = targets[tick, foot] - raw_liftoff
                delta[2] = max(0.0, delta[2])
                anchored[tick, foot] = output_liftoff + delta
            was_stance = in_stance
    if touchdown_blend_ticks > 0:
        for foot in range(2):
            contact_edges = np.flatnonzero(
                stance[1:, foot].astype(bool)
                & ~stance[:-1, foot].astype(bool)
            ) + 1
            for edge in contact_edges:
                swing_start = int(edge - 1)
                while swing_start > 0 and not bool(stance[swing_start - 1, foot]):
                    swing_start -= 1
                start = max(swing_start, int(edge) - touchdown_blend_ticks)
                duration = int(edge) - start
                if duration <= 0:
                    continue
                original = anchored[start : edge + 1, foot].copy()
                phase = np.arange(duration + 1, dtype=np.float64) / duration
                blend = (
                    10.0 * phase**3
                    - 15.0 * phase**4
                    + 6.0 * phase**5
                )[:, None]
                endpoint = anchored[edge, foot].copy()
                anchored[start : edge + 1, foot] = (
                    (1.0 - blend) * original + blend * endpoint
                )
    velocity, acceleration = finite_difference(anchored)
    for foot in range(2):
        selected = stance[:, foot].astype(bool)
        velocity[selected, foot] = 0.0
        acceleration[selected, foot] = 0.0
    return anchored, velocity, acceleration, contact


def center_of_mass_reference(
    root_targets: np.ndarray,
    center_of_mass_offset_from_root: np.ndarray,
    foot_targets: np.ndarray,
    stance: np.ndarray,
    *,
    mode: str,
    patch_center_x: float,
    patch_half_length: float,
    patch_half_width: float,
    patch_z: float,
    preview_ticks: int,
    margin: float,
    blend: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build an explicit CoM jet, optionally previewing the support schedule.

    The support-preview reference starts moving during the preceding
    double-support interval. It averages future support centers over a fixed
    horizon, blends that preview with the rooted motion, and finally clamps the
    horizontal target to the support-patch envelope active at the current tick.
    Python owns this experiment/reference policy; Rust owns tracking and every
    measured solve.
    """
    nominal = root_targets + center_of_mass_offset_from_root[None, :]
    if mode == "rooted":
        velocity, acceleration = finite_difference(nominal)
        return nominal, velocity, acceleration
    if mode not in (
        "support-preview",
        "support-centroid-preview",
        "dcm-backward-preview",
    ):
        raise ValueError(f"unknown center-of-mass reference mode: {mode}")
    if preview_ticks < 0:
        raise ValueError("support preview ticks must be nonnegative")
    if margin < 0.0 or margin >= min(patch_half_length, patch_half_width):
        raise ValueError("support margin must fit inside both patch half extents")
    if not 0.0 <= blend <= 1.0:
        raise ValueError("support reference blend must be in [0, 1]")

    active = stance.astype(bool)
    patch_centers = foot_targets[:, :2, :2].copy()
    patch_centers[:, :, 0] += patch_center_x
    support_center = np.empty((len(root_targets), 2), dtype=np.float64)
    for tick in range(len(root_targets)):
        feet = active[tick]
        if not np.any(feet):
            raise ValueError("support-preview reference encountered a flight tick")
        support_center[tick] = np.mean(patch_centers[tick, feet], axis=0)

    if mode == "dcm-backward-preview":
        support_plane_z = np.empty(len(root_targets), dtype=np.float64)
        for tick in range(len(root_targets)):
            support_plane_z[tick] = np.mean(
                foot_targets[tick, :2, 2][active[tick]] + patch_z
            )
        height = np.maximum(nominal[:, 2] - support_plane_z, 0.2)
        omega = np.sqrt(9.81 / height)
        target = nominal.copy()
        # Solve the stable DCM boundary condition on a fixed receding horizon,
        # never from the arbitrary end of the evaluation buffer. This makes
        # every tick outside the final preview window prefix-invariant when a
        # caller requests a longer trace from the same motion program.
        for tick in range(len(target)):
            stop = min(len(target) - 1, tick + preview_ticks)
            preview_dcm = support_center[stop].copy()
            for preview_tick in range(stop - 1, tick - 1, -1):
                decay = math.exp(-omega[preview_tick] * DT)
                preview_dcm = support_center[preview_tick] + decay * (
                    preview_dcm - support_center[preview_tick]
                )
            target[tick, :2] = preview_dcm
        velocity = np.zeros_like(target)
        velocity[:, :2] = omega[:, None] * (
            target[:, :2] - support_center
        )
        nominal_velocity, _ = finite_difference(nominal)
        velocity[:, 2] = nominal_velocity[:, 2]
        acceleration = finite_difference(velocity)[0]
        return target, velocity, acceleration

    prefix = np.vstack(
        (
            np.zeros((1, 2), dtype=np.float64),
            np.cumsum(support_center, axis=0),
        )
    )
    preview = np.empty_like(support_center)
    for tick in range(len(root_targets)):
        stop = min(len(root_targets), tick + preview_ticks + 1)
        preview[tick] = (prefix[stop] - prefix[tick]) / (stop - tick)

    target = nominal.copy()
    target[:, :2] = (1.0 - blend) * nominal[:, :2] + blend * preview
    if mode == "support-preview":
        # Retained as a deliberately static control. This treats CoM like a
        # center-of-pressure target and is too restrictive for dynamic walking:
        # at a support edge the admissible box can move discontinuously.
        for tick in range(len(root_targets)):
            centers = patch_centers[tick, active[tick]]
            lower = np.min(
                centers
                - np.array(
                    [patch_half_length - margin, patch_half_width - margin]
                ),
                axis=0,
            )
            upper = np.max(
                centers
                + np.array(
                    [patch_half_length - margin, patch_half_width - margin]
                ),
                axis=0,
            )
            target[tick, :2] = np.clip(target[tick, :2], lower, upper)
    velocity, acceleration = finite_difference(target)
    return target, velocity, acceleration


def rebase_to_double_support(
    walk: Any,
    target_root: np.ndarray,
    target_origins: np.ndarray,
    ticks: int,
    startup_balance_ticks: int = 160,
    startup_ramp_ticks: int = 200,
    forward_motion_scale: float = 0.35,
    lateral_motion_scale: float = 0.35,
    cadence_multiplier: float = 1.0,
) -> Any:
    candidates = np.flatnonzero(np.all(walk.stance.astype(bool), axis=1))
    phase_wrap = np.flatnonzero(np.diff(walk.source_phase_frames) < 0.0)
    if len(phase_wrap):
        candidates = candidates[candidates < int(phase_wrap[0] + 1)]
    if len(candidates) == 0:
        raise ValueError(
            "floating walking source has no first-cycle double-support phase"
        )
    candidate_foot_height = (
        walk.targets[candidates, :2, 2] - target_origins[None, :2, 2]
    )
    offset = int(
        candidates[
            np.argmin(np.max(np.abs(candidate_foot_height), axis=1))
        ]
    )
    if startup_balance_ticks < 1 or startup_ramp_ticks < 1:
        raise ValueError("startup balance and ramp ticks must be positive")
    if (
        not np.isfinite(forward_motion_scale)
        or forward_motion_scale <= 0.0
        or not np.isfinite(lateral_motion_scale)
        or lateral_motion_scale <= 0.0
        or not np.isfinite(cadence_multiplier)
        or cadence_multiplier <= 0.0
    ):
        raise ValueError("motion scales must be finite and positive")
    motion_ticks = np.maximum(
        np.arange(ticks, dtype=np.float64) - startup_balance_ticks,
        0.0,
    )
    normalized = np.minimum(
        motion_ticks / startup_ramp_ticks,
        1.0,
    )
    phase_rate = (
        10.0 * normalized**3
        - 15.0 * normalized**4
        + 6.0 * normalized**5
    ) * cadence_multiplier
    sample = offset + np.cumsum(np.r_[0.0, phase_rate[:-1]])
    if int(np.ceil(sample[-1])) >= len(walk.root_targets):
        raise ValueError("floating walking source lacks phase-rebase padding")

    def interpolate(values: np.ndarray) -> np.ndarray:
        flattened = values.reshape(len(values), -1)
        sampled = np.empty((ticks, flattened.shape[1]), dtype=np.float64)
        source = np.arange(len(values), dtype=np.float64)
        for column in range(flattened.shape[1]):
            sampled[:, column] = np.interp(sample, source, flattened[:, column])
        return sampled.reshape((ticks, *values.shape[1:]))

    root_targets = interpolate(walk.root_targets)
    targets = interpolate(walk.targets)
    grounded_foot_height = (
        targets[:, :2, 2] - target_origins[None, :2, 2]
    )
    discrete_sample = np.clip(
        np.rint(sample).astype(np.int64), 0, len(walk.stance) - 1
    )
    root_targets -= root_targets[0] - target_root
    targets -= targets[0] - target_origins
    targets[:, :2, 2] = (
        target_origins[None, :2, 2] + grounded_foot_height
    )
    root_targets[:, 0] = target_root[0] + forward_motion_scale * (
        root_targets[:, 0] - target_root[0]
    )
    root_targets[:, 1] = target_root[1] + lateral_motion_scale * (
        root_targets[:, 1] - target_root[1]
    )
    targets[:, :, 0] = target_origins[None, :, 0] + forward_motion_scale * (
        targets[:, :, 0] - target_origins[None, :, 0]
    )
    targets[:, :, 1] = target_origins[None, :, 1] + lateral_motion_scale * (
        targets[:, :, 1] - target_origins[None, :, 1]
    )
    return dataclasses.replace(
        walk,
        root_targets=root_targets,
        targets=targets,
        stance=walk.stance[discrete_sample].copy(),
        cadence_scale=(
            walk.cadence_scale[discrete_sample].copy() * cadence_multiplier
        ),
        source_phase_frames=np.interp(
            sample,
            np.arange(len(walk.source_phase_frames), dtype=np.float64),
            walk.source_phase_frames,
        ),
        metadata={
            **walk.metadata,
            "initialization": (
                "lowest double-support phase registered to the target rig root; "
                "only horizontal capture-frame deltas transfer while the "
                "already-grounded foot-height channel is preserved"
            ),
            "phase_rebase_ticks": offset,
            "phase_rebase_seconds": offset * DT,
            "startup_balance_ramp": "neutral rooted double-support hold",
            "startup_balance_ticks": startup_balance_ticks,
            "startup_balance_seconds": startup_balance_ticks * DT,
            "startup_phase_ramp": "quintic smootherstep from zero to nominal cadence",
            "startup_phase_ramp_ticks": startup_ramp_ticks,
            "startup_phase_ramp_seconds": startup_ramp_ticks * DT,
            "cadence_multiplier": cadence_multiplier,
            "forward_motion_scale": forward_motion_scale,
            "lateral_motion_scale": lateral_motion_scale,
        },
    )


def insert_first_liftoff_hold(walk: Any, hold_ticks: int) -> Any:
    """Delay the first release while preserving phase alignment of every jet.

    This is an eval-side prototype for measured liftoff gating. The complete
    rooted pose, endpoint motion, contact schedule, cadence, and source phase
    are held together; no foot target is allowed to run ahead of contact.
    """
    if hold_ticks == 0:
        return walk
    if hold_ticks < 0 or hold_ticks >= len(walk.stance):
        raise ValueError("first-liftoff hold must fit inside the trace")
    stance = walk.stance.astype(bool)
    edges = np.argwhere(stance[:-1] & ~stance[1:])
    if len(edges) == 0:
        raise ValueError("trace has no authored liftoff to delay")
    edge = int(edges[0, 0] + 1)

    def hold(values: np.ndarray) -> np.ndarray:
        result = np.empty_like(values)
        result[:edge] = values[:edge]
        result[edge : edge + hold_ticks] = values[edge - 1]
        result[edge + hold_ticks :] = values[edge : len(values) - hold_ticks]
        return result

    return dataclasses.replace(
        walk,
        root_targets=hold(walk.root_targets),
        targets=hold(walk.targets),
        stance=hold(walk.stance),
        cadence_scale=hold(walk.cadence_scale),
        source_phase_frames=hold(walk.source_phase_frames),
        metadata={
            **walk.metadata,
            "first_liftoff_hold_ticks": hold_ticks,
            "first_liftoff_hold_seconds": hold_ticks * DT,
            "first_liftoff_original_tick": edge,
            "first_liftoff_delayed_tick": edge + hold_ticks,
        },
    )


def gate_first_liftoff_leg_phase(
    walk: Any,
    target_positions: np.ndarray,
    contacts: np.ndarray,
    hold_ticks: int,
) -> tuple[Any, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Delay the coupled leg/contact phase while higher-level intent advances.

    This is a causal eval prototype, not the production scheduler. Both feet
    and both contact labels pause together, so support topology and leg jets
    cannot disagree. Root, CoM, hands, cadence, and source phase continue on
    the authored timeline. A measured Rust policy should replace the fixed
    delay only if this split improves the retained transfer corpus.
    """
    if hold_ticks == 0:
        target_velocities, target_accelerations = finite_difference(target_positions)
        for foot in range(2):
            selected = contacts[:, foot].astype(bool)
            target_velocities[selected, foot] = 0.0
            target_accelerations[selected, foot] = 0.0
        return walk, target_positions, target_velocities, target_accelerations, contacts
    if hold_ticks < 0 or hold_ticks >= len(walk.stance):
        raise ValueError("first-liftoff leg-phase gate must fit inside the trace")
    stance = walk.stance.astype(bool)
    edges = np.argwhere(stance[:-1] & ~stance[1:])
    if len(edges) == 0:
        raise ValueError("trace has no authored liftoff to gate")
    edge = int(edges[0, 0] + 1)

    gated_positions = target_positions.copy()
    gated_contacts = contacts.copy()
    gated_stance = walk.stance.copy()
    gated_walk_targets = walk.targets.copy()
    for values in (gated_positions[:, :2], gated_walk_targets[:, :2]):
        original = values.copy()
        values[edge : edge + hold_ticks] = original[edge - 1]
        values[edge + hold_ticks :] = original[edge : len(values) - hold_ticks]
    original_contacts = gated_contacts[:, :2].copy()
    gated_contacts[edge : edge + hold_ticks, :2] = original_contacts[edge - 1]
    gated_contacts[edge + hold_ticks :, :2] = original_contacts[
        edge : len(original_contacts) - hold_ticks
    ]
    original_stance = gated_stance[:, :2].copy()
    gated_stance[edge : edge + hold_ticks, :2] = original_stance[edge - 1]
    gated_stance[edge + hold_ticks :, :2] = original_stance[
        edge : len(original_stance) - hold_ticks
    ]
    target_velocities, target_accelerations = finite_difference(gated_positions)
    for foot in range(2):
        selected = gated_contacts[:, foot].astype(bool)
        target_velocities[selected, foot] = 0.0
        target_accelerations[selected, foot] = 0.0
    return (
        dataclasses.replace(
            walk,
            targets=gated_walk_targets,
            stance=gated_stance,
            metadata={
                **walk.metadata,
                "first_liftoff_leg_phase_gate_ticks": hold_ticks,
                "first_liftoff_leg_phase_gate_seconds": hold_ticks * DT,
                "first_liftoff_leg_phase_original_tick": edge,
                "first_liftoff_leg_phase_released_tick": edge + hold_ticks,
            },
        ),
        gated_positions,
        target_velocities,
        target_accelerations,
        gated_contacts,
    )


def project_touchdowns_to_root_reach(
    walk: Any, anchored_targets: np.ndarray, maximum_reach: float
) -> tuple[Any, np.ndarray]:
    """Make retargeted foot landings reachable without changing contact timing.

    Source-morphology scaling alone does not guarantee that a target root and
    swing endpoint fit the target robot's leg envelope. Each touchdown is
    projected only in the horizontal plane, then its correction is introduced
    with a quintic blend across the preceding swing and retained for subsequent
    samples. This keeps position and the first two endpoint derivatives smooth
    while preserving ground height, support labels, and the authored cadence.
    """
    if not np.isfinite(maximum_reach) or maximum_reach <= 0.0:
        raise ValueError("maximum root-to-foot reach must be finite and positive")
    targets = anchored_targets.copy()
    corrections: list[dict[str, float | int]] = []
    maximum_before = 0.0
    maximum_after = 0.0
    for foot in range(2):
        contact_edges = np.flatnonzero(
            walk.stance[1:, foot].astype(bool)
            & ~walk.stance[:-1, foot].astype(bool)
        ) + 1
        for edge_value in contact_edges:
            edge = int(edge_value)
            endpoint = edge - 1
            swing_start = endpoint
            while swing_start > 0 and not bool(walk.stance[swing_start - 1, foot]):
                swing_start -= 1
            delta = targets[endpoint, foot] - walk.root_targets[edge]
            reach_before = float(np.linalg.norm(delta))
            maximum_before = max(maximum_before, reach_before)
            horizontal = float(np.linalg.norm(delta[:2]))
            horizontal_limit_squared = maximum_reach**2 - float(delta[2] ** 2)
            if horizontal_limit_squared <= 0.0:
                raise ValueError(
                    "root-to-foot reach is shorter than the touchdown height offset"
                )
            horizontal_limit = math.sqrt(horizontal_limit_squared)
            correction = np.zeros(3, dtype=np.float64)
            if horizontal > horizontal_limit:
                correction[:2] = delta[:2] * (horizontal_limit / horizontal - 1.0)
                duration = endpoint - swing_start
                phase = (
                    np.ones(1, dtype=np.float64)
                    if duration == 0
                    else np.arange(duration + 1, dtype=np.float64) / duration
                )
                blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
                targets[swing_start : endpoint + 1, foot] += (
                    blend[:, None] * correction[None, :]
                )
                targets[edge:, foot] += correction
            reach_after = float(
                np.linalg.norm(targets[endpoint, foot] - walk.root_targets[edge])
            )
            maximum_after = max(maximum_after, reach_after)
            corrections.append(
                {
                    "foot": foot,
                    "touchdown_tick": edge,
                    "swing_start_tick": swing_start,
                    "reach_before_m": reach_before,
                    "reach_after_m": reach_after,
                    "horizontal_correction_m": float(np.linalg.norm(correction[:2])),
                }
            )
    return (
        dataclasses.replace(
            walk,
            metadata={
                **walk.metadata,
                "root_to_foot_reach_projection": {
                    "maximum_reach_m": maximum_reach,
                    "maximum_before_m": maximum_before,
                    "maximum_after_m": maximum_after,
                    "touchdowns": corrections,
                },
            },
        ),
        targets,
    )


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values**2)))


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def distribution_us(step_ns: np.ndarray) -> dict[str, float]:
    values = np.asarray(step_ns, dtype=np.float64) / 1_000.0
    median = float(np.median(values))
    return {
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "stddev": float(np.std(values)),
        "median": median,
        "p50": median,
        "mad": float(np.median(np.abs(values - median))),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "p99_9": float(np.percentile(values, 99.9)),
        "p99_99": float(np.percentile(values, 99.99)),
        "max": float(np.max(values)),
    }


def count_distribution(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def longest_true_run(values: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in np.asarray(values, dtype=bool):
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def current_rss_bytes() -> int | None:
    statm = pathlib.Path("/proc/self/statm")
    if not statm.is_file():
        return None
    resident_pages = int(statm.read_text().split()[1])
    return resident_pages * os.sysconf("SC_PAGE_SIZE")


def usage_snapshot() -> dict[str, float | int]:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return {
        "user_seconds": usage.ru_utime,
        "system_seconds": usage.ru_stime,
        "minor_faults": usage.ru_minflt,
        "major_faults": usage.ru_majflt,
        "voluntary_context_switches": usage.ru_nvcsw,
        "involuntary_context_switches": usage.ru_nivcsw,
        "maximum_rss_bytes": int(usage.ru_maxrss * 1024),
    }


def usage_delta(
    after: dict[str, float | int], before: dict[str, float | int]
) -> dict[str, float | int]:
    return {
        key: after[key] - before[key]
        for key in after
        if key != "maximum_rss_bytes"
    }


def gc_snapshot() -> dict[str, int]:
    stats = gc.get_stats()
    return {
        "collections": sum(generation["collections"] for generation in stats),
        "collected": sum(generation["collected"] for generation in stats),
        "uncollectable": sum(
            generation["uncollectable"] for generation in stats
        ),
    }


def integer_delta(after: dict[str, int], before: dict[str, int]) -> dict[str, int]:
    return {key: after[key] - before[key] for key in after}


def runtime_measurement(
    *,
    wall_ns: int,
    process_cpu_ns: int,
    thread_cpu_ns: int,
    rss_before: int | None,
    rss_after: int | None,
    usage_before: dict[str, float | int],
    usage_after: dict[str, float | int],
    gc_before: dict[str, int],
    gc_after: dict[str, int],
    traced_current: int,
    traced_peak: int,
) -> dict[str, Any]:
    peak_candidates = [int(usage_after["maximum_rss_bytes"])]
    peak_candidates.extend(
        value for value in (rss_before, rss_after) if value is not None
    )
    return {
        "call_wall_ns": wall_ns,
        "process_cpu_ns": process_cpu_ns,
        "thread_cpu_ns": thread_cpu_ns,
        "process_cpu_to_wall_ratio": process_cpu_ns / max(wall_ns, 1),
        "thread_cpu_to_wall_ratio": thread_cpu_ns / max(wall_ns, 1),
        "rss_before_bytes": rss_before,
        "rss_after_bytes": rss_after,
        "rss_delta_bytes": (
            rss_after - rss_before
            if rss_before is not None and rss_after is not None
            else None
        ),
        "peak_rss_bytes": max(peak_candidates),
        "usage_delta": usage_delta(usage_after, usage_before),
        "python_gc_delta": integer_delta(gc_after, gc_before),
        "python_tracemalloc_current_bytes": traced_current,
        "python_tracemalloc_peak_bytes": traced_peak,
    }


def summarize(
    walk: Any,
    target_positions: np.ndarray,
    root_targets: np.ndarray,
    effective_contact_active: np.ndarray,
    cadence_scale: np.ndarray,
    root_out: np.ndarray,
    center_of_mass_targets: np.ndarray,
    center_of_mass_tracked: np.ndarray,
    tracked: np.ndarray,
    root_quaternion: np.ndarray,
    joint_velocity: np.ndarray,
    contact_force: np.ndarray,
    task_rms: np.ndarray,
    dynamics_residual: np.ndarray,
    contact_residual: np.ndarray,
    minimum_support_margin: np.ndarray,
    required_support_margin: float,
    step_ns: np.ndarray,
    status: np.ndarray,
    support_phase: np.ndarray,
    support_phase_ticks: np.ndarray,
    support_tangential_speed: np.ndarray,
    support_touchdown_position_error: np.ndarray,
    support_touchdown_normal_speed: np.ndarray,
    touchdown_position_limit: float,
    touchdown_tangential_speed_limit: float,
    touchdown_normal_speed_limit: float,
    task_pseudoinverse_calls: np.ndarray,
    clipped_steps: np.ndarray,
    task_pseudoinverse_calls_by_level: np.ndarray,
    clipped_steps_by_level: np.ndarray,
    task_jacobi_sweeps: np.ndarray,
    task_jacobi_sweeps_by_level: np.ndarray,
    feasibility_projection_sweeps: np.ndarray,
    feasibility_halfspace_projections: np.ndarray,
    feasibility_polish_iterations: np.ndarray,
    feasibility_polish_pseudoinverse_calls: np.ndarray,
    feasibility_polish_jacobi_sweeps: np.ndarray,
    dcm: np.ndarray,
    target_dcm: np.ndarray,
    virtual_zmp: np.ndarray,
    clipped_zmp: np.ndarray,
    center_of_mass_command: np.ndarray,
    dcm_natural_frequency: np.ndarray,
    dcm_measured_height: np.ndarray,
    dcm_height_clamped: np.ndarray,
    dcm_zmp_clipped: np.ndarray,
    dcm_support_vertices: np.ndarray,
    dcm_support_margin: np.ndarray,
    landing_retarget_anchor: np.ndarray,
    landing_retarget_capture_scale: np.ndarray,
    landing_retarget_offset: np.ndarray,
    landing_retarget_reach: np.ndarray,
    landing_retarget_flags: np.ndarray,
    contact_phase_authority: np.ndarray,
    joint_velocity_envelope_target_scale: np.ndarray,
    joint_velocity_envelope_scale: np.ndarray,
    joint_velocity_envelope_active_coordinates: np.ndarray,
    reference_phase: np.ndarray,
    reference_phase_target_rate: np.ndarray,
    reference_phase_rate: np.ndarray,
    reference_phase_acceleration: np.ndarray,
    reference_phase_required_time: np.ndarray,
    reference_phase_flags: np.ndarray,
    touchdown_phase_retiming_enabled: bool,
    balance_phase_retiming_enabled: bool,
    hand_task_weight: float,
    runtime: dict[str, Any],
) -> dict[str, Any]:
    root_error = np.linalg.norm(root_out - root_targets, axis=1)
    center_of_mass_error = np.linalg.norm(
        center_of_mass_tracked - center_of_mass_targets, axis=1
    )
    dcm_valid = np.isfinite(dcm[:, 0])
    if np.any(dcm_valid):
        dcm_error = np.linalg.norm(
            dcm[dcm_valid, :2] - target_dcm[dcm_valid, :2], axis=1
        )
        zmp_clip_distance = np.linalg.norm(
            virtual_zmp[dcm_valid, :2] - clipped_zmp[dcm_valid, :2], axis=1
        )
        center_of_mass_command_norm = np.linalg.norm(
            center_of_mass_command[dcm_valid, :2], axis=1
        )
        dcm_metrics = {
            "enabled": True,
            "tracking_rms_m": rms(dcm_error),
            "tracking_p95_m": percentile(dcm_error, 95),
            "zmp_clipped_fraction": float(np.mean(dcm_zmp_clipped[dcm_valid])),
            "zmp_clip_distance_rms_m": rms(zmp_clip_distance),
            "zmp_clip_distance_maximum_m": float(np.max(zmp_clip_distance)),
            "natural_frequency_p50_rad_s": percentile(
                dcm_natural_frequency[dcm_valid], 50
            ),
            "natural_frequency_minimum_rad_s": float(
                np.min(dcm_natural_frequency[dcm_valid])
            ),
            "natural_frequency_maximum_rad_s": float(
                np.max(dcm_natural_frequency[dcm_valid])
            ),
            "measured_com_height_minimum_m": float(
                np.min(dcm_measured_height[dcm_valid])
            ),
            "height_clamped_ticks": int(
                np.count_nonzero(dcm_height_clamped[dcm_valid])
            ),
            "command_acceleration_p95_mps2": percentile(
                center_of_mass_command_norm, 95
            ),
            "command_acceleration_maximum_mps2": float(
                np.max(center_of_mass_command_norm)
            ),
            "support_vertices_minimum": int(
                np.min(dcm_support_vertices[dcm_valid])
            ),
            "support_vertices_maximum": int(
                np.max(dcm_support_vertices[dcm_valid])
            ),
            "support_margin_minimum_m": float(
                np.min(dcm_support_margin[dcm_valid])
            ),
            "support_margin_p05_m": percentile(
                dcm_support_margin[dcm_valid], 5
            ),
            "support_margin_positive_fraction": float(
                np.mean(dcm_support_margin[dcm_valid] >= 0.0)
            ),
        }
    else:
        dcm_metrics = {"enabled": False}
    tracking_error = np.linalg.norm(tracked - target_positions, axis=2)
    stance = effective_contact_active[:, :2].astype(bool)
    swing = ~stance
    foot_error = tracking_error[:, :2]
    hand_error = tracking_error[:, 2:]
    quaternion_w = np.clip(np.abs(root_quaternion[:, 0]), 0.0, 1.0)
    rotation_angle = 2.0 * np.arccos(quaternion_w)
    cadence = {}
    for scale in sorted(set(float(value) for value in cadence_scale)):
        selected = cadence_scale == scale
        cadence[f"{scale:.2f}x"] = {
            "ticks": int(np.count_nonzero(selected)),
            "root_tracking_rms_m": rms(root_error[selected]),
            "foot_tracking_rms_m": rms(foot_error[selected]),
            "hand_tracking_rms_m": rms(hand_error[selected]),
        }
    status_counts = {
        "solved": int(np.count_nonzero(status == 0)),
        "solved_with_slack": int(np.count_nonzero(status == 1)),
        "primal_infeasible": int(np.count_nonzero(status == 2)),
        "failed": int(np.count_nonzero(status == 3)),
        "normal_contact_contingency": int(np.count_nonzero(status == 4)),
        "contact_release_contingency": int(np.count_nonzero(status == 5)),
        "touchdown_transition": int(np.count_nonzero(status == 6)),
        "precontact_transition": int(np.count_nonzero(status == 7)),
    }
    maximum_touchdown_transition_ticks = longest_true_run(status == 6)
    maximum_precontact_transition_ticks = longest_true_run(status == 7)
    delayed_touchdown_admission = stance & (support_phase[:, :2] == 1)
    maximum_touchdown_admission_delay_ticks = max(
        (
            longest_true_run(delayed_touchdown_admission[:, target])
            for target in range(delayed_touchdown_admission.shape[1])
        ),
        default=0,
    )
    delayed_touchdown_admission_ticks = int(
        np.count_nonzero(delayed_touchdown_admission)
    )
    support_phase_counts = {
        name: int(np.count_nonzero(support_phase == code))
        for code, name in enumerate(SUPPORT_PHASE_NAMES)
    }
    support_phase_maximum_ticks = {
        name: int(
            np.max(support_phase_ticks[support_phase == code])
            if np.any(support_phase == code)
            else 0
        )
        for code, name in enumerate(SUPPORT_PHASE_NAMES)
    }
    support_phase_counts_by_target = [
        {
            name: int(np.count_nonzero(support_phase[:, target] == code))
            for code, name in enumerate(SUPPORT_PHASE_NAMES)
        }
        for target in range(support_phase.shape[1])
    ]
    support_speed = {
        name: {
            "samples": int(np.count_nonzero(selected)),
            "minimum_mps": float(np.min(support_tangential_speed[selected])),
            "p50_mps": percentile(support_tangential_speed[selected], 50),
            "p95_mps": percentile(support_tangential_speed[selected], 95),
            "maximum_mps": float(np.max(support_tangential_speed[selected])),
        }
        for code, name in enumerate(SUPPORT_PHASE_NAMES)
        if np.any(
            selected := (
                (support_phase == code) & np.isfinite(support_tangential_speed)
            )
        )
    }
    physical_tick = np.isin(status, (0, 1, 4, 5, 6, 7))
    if not np.any(physical_tick):
        physical_tick = np.ones_like(status, dtype=bool)
    non_nominal = np.flatnonzero(~np.isin(status, (0, 1, 6, 7)))
    nominal_prefix_ticks = (
        int(non_nominal[0]) if len(non_nominal) else len(status)
    )
    nominal = slice(0, nominal_prefix_ticks)
    nominal_stance = stance[nominal]
    nominal_swing = swing[nominal]
    nominal_prefix = {
        "ticks": nominal_prefix_ticks,
        "duration_seconds": nominal_prefix_ticks * DT,
        "root_tracking_rms_m": rms(root_error[nominal]),
        "stance_foot_tracking_rms_m": rms(
            foot_error[nominal][nominal_stance]
        ),
        "swing_foot_tracking_rms_m": rms(
            foot_error[nominal][nominal_swing]
        ),
        "hand_tracking_rms_m": rms(hand_error[nominal]),
        "maximum_root_rotation_rad": float(np.max(rotation_angle[nominal])),
        "maximum_joint_velocity_rad_s": float(
            np.max(np.abs(joint_velocity[nominal]))
        ),
        "latency_p99_us": percentile(step_ns[nominal], 99) / 1_000.0,
        "maximum_dynamics_residual": float(
            np.max(dynamics_residual[nominal])
        ),
        "maximum_contact_acceleration_residual": float(
            np.max(contact_residual[nominal])
        ),
    }
    latency = distribution_us(step_ns)
    jitter = distribution_us(
        np.abs(np.diff(step_ns.astype(np.int64)))
        if len(step_ns) > 1
        else step_ns
    )
    status_latency = {}
    status_solver_work = {}
    status_names = {
        0: "solved",
        1: "solved_with_slack",
        2: "primal_infeasible",
        3: "failed",
        4: "normal_contact_contingency",
        5: "contact_release_contingency",
        6: "touchdown_transition",
        7: "precontact_transition",
    }
    for code, name in status_names.items():
        selected = step_ns[status == code]
        if len(selected):
            status_latency[name] = {
                "ticks": int(len(selected)),
                **distribution_us(selected),
            }
            status_solver_work[name] = {
                "task_pseudoinverse_calls": count_distribution(
                    task_pseudoinverse_calls[status == code]
                ),
                "clipped_steps": count_distribution(clipped_steps[status == code]),
                "feasibility_projection_sweeps": count_distribution(
                    feasibility_projection_sweeps[status == code]
                ),
                "feasibility_halfspace_projections": count_distribution(
                    feasibility_halfspace_projections[status == code]
                ),
            }
    temporal_windows = []
    for indices in np.array_split(np.arange(len(step_ns)), min(10, len(step_ns))):
        window_status = status[indices]
        temporal_windows.append(
            {
                "tick_start": int(indices[0]),
                "tick_stop": int(indices[-1] + 1),
                "latency_p50_us": percentile(step_ns[indices], 50) / 1_000.0,
                "latency_p99_us": percentile(step_ns[indices], 99) / 1_000.0,
                "root_tracking_rms_m": rms(root_error[indices]),
                "foot_tracking_rms_m": rms(foot_error[indices]),
                "maximum_dynamics_residual": float(
                    np.max(dynamics_residual[indices])
                ),
                "maximum_contact_acceleration_residual": float(
                    np.max(contact_residual[indices])
                ),
                "contingency_or_rejected_ticks": int(
                    np.count_nonzero(~np.isin(window_status, (0, 1, 6)))
                ),
                "task_pseudoinverse_calls_mean": float(
                    np.mean(task_pseudoinverse_calls[indices])
                ),
                "task_jacobi_sweeps_mean": float(
                    np.mean(task_jacobi_sweeps[indices])
                ),
                "clipped_steps_mean": float(np.mean(clipped_steps[indices])),
                "feasibility_projection_sweeps_mean": float(
                    np.mean(feasibility_projection_sweeps[indices])
                ),
                "feasibility_halfspace_projections_mean": float(
                    np.mean(feasibility_halfspace_projections[indices])
                ),
            }
        )
    latency_us_values = np.asarray(step_ns, dtype=np.float64) / 1_000.0
    pseudoinverse_latency_correlation = (
        float(np.corrcoef(task_pseudoinverse_calls, latency_us_values)[0, 1])
        if np.std(task_pseudoinverse_calls) > 0.0
        else 0.0
    )
    feasibility_projection_latency_correlation = (
        float(
            np.corrcoef(feasibility_halfspace_projections, latency_us_values)[
                0, 1
            ]
        )
        if np.std(feasibility_halfspace_projections) > 0.0
        else 0.0
    )
    authority_phase_counts = {
        name: int(np.count_nonzero(contact_phase_authority == index))
        for index, name in enumerate(
            ("unsupported", "single_support", "precontact", "multi_support")
        )
    }
    landing_retarget_valid = np.isfinite(landing_retarget_offset)
    landing_retarget_updated = np.isfinite(landing_retarget_capture_scale)
    landing_retarget_metrics = {
        "active_ticks": int(np.count_nonzero(landing_retarget_valid)),
        "policy_update_ticks": int(np.count_nonzero(landing_retarget_updated)),
        "maximum_applied_offset_m": (
            float(np.max(landing_retarget_offset[landing_retarget_valid]))
            if np.any(landing_retarget_valid)
            else 0.0
        ),
        "maximum_root_to_landing_reach_m": (
            float(np.max(landing_retarget_reach[landing_retarget_valid]))
            if np.any(landing_retarget_valid)
            else 0.0
        ),
        "capture_scale_mean": (
            float(np.mean(landing_retarget_capture_scale[landing_retarget_updated]))
            if np.any(landing_retarget_updated)
            else 0.0
        ),
        "authored_offset_limited_ticks": int(
            np.count_nonzero(landing_retarget_flags & 1)
        ),
        "reach_limited_ticks": int(np.count_nonzero(landing_retarget_flags & 2)),
        "slew_limited_ticks": int(np.count_nonzero(landing_retarget_flags & 4)),
        "unreachable_authored_geometry_ticks": int(
            np.count_nonzero(landing_retarget_flags & 8)
        ),
        "frozen_target_ticks": int(np.count_nonzero(landing_retarget_flags & 16)),
    }
    pending_touchdown = np.zeros_like(support_phase, dtype=bool)
    pending_touchdown[:, :2] = effective_contact_active[:, :2].astype(bool)
    pending_touchdown &= support_phase == SUPPORT_PHASE_NAMES.index("precontact")
    touchdown_observation_valid = (
        pending_touchdown
        & np.isfinite(support_touchdown_position_error)
        & np.isfinite(support_tangential_speed)
        & np.isfinite(support_touchdown_normal_speed)
    )
    if np.any(touchdown_observation_valid):
        touchdown_position = support_touchdown_position_error[
            touchdown_observation_valid
        ]
        touchdown_tangential = support_tangential_speed[touchdown_observation_valid]
        touchdown_normal = support_touchdown_normal_speed[touchdown_observation_valid]
        touchdown_viability = {
            "observed_target_ticks": int(np.count_nonzero(touchdown_observation_valid)),
            "position_error_minimum_m": float(np.min(touchdown_position)),
            "position_error_p50_m": percentile(touchdown_position, 50),
            "tangential_speed_minimum_mps": float(np.min(touchdown_tangential)),
            "normal_speed_minimum_mps": float(np.min(touchdown_normal)),
            "position_viable_ticks": int(
                np.count_nonzero(touchdown_position <= touchdown_position_limit)
            ),
            "tangential_viable_ticks": int(
                np.count_nonzero(
                    touchdown_tangential <= touchdown_tangential_speed_limit
                )
            ),
            "normal_viable_ticks": int(
                np.count_nonzero(touchdown_normal <= touchdown_normal_speed_limit)
            ),
            "jointly_viable_ticks": int(
                np.count_nonzero(
                    (touchdown_position <= touchdown_position_limit)
                    & (touchdown_tangential <= touchdown_tangential_speed_limit)
                    & (touchdown_normal <= touchdown_normal_speed_limit)
                )
            ),
        }
    else:
        touchdown_viability = {
            "observed_target_ticks": 0,
            "position_error_minimum_m": 0.0,
            "position_error_p50_m": 0.0,
            "tangential_speed_minimum_mps": 0.0,
            "normal_speed_minimum_mps": 0.0,
            "position_viable_ticks": 0,
            "tangential_viable_ticks": 0,
            "normal_viable_ticks": 0,
            "jointly_viable_ticks": 0,
        }
    source_stance = np.asarray(walk.stance, dtype=bool)
    first_authored_touchdown_source_tick: int | None = None
    for target in range(source_stance.shape[1]):
        liftoff_edges = np.flatnonzero(
            source_stance[1:, target] < source_stance[:-1, target]
        ) + 1
        touchdown_edges = np.flatnonzero(
            source_stance[1:, target] > source_stance[:-1, target]
        ) + 1
        if len(liftoff_edges) == 0:
            continue
        touchdown_after_liftoff = touchdown_edges[
            touchdown_edges > liftoff_edges[0]
        ]
        if len(touchdown_after_liftoff) == 0:
            continue
        candidate = int(touchdown_after_liftoff[0])
        first_authored_touchdown_source_tick = (
            candidate
            if first_authored_touchdown_source_tick is None
            else min(first_authored_touchdown_source_tick, candidate)
        )
    first_authored_touchdown_reached = (
        first_authored_touchdown_source_tick is None
        or reference_phase[-1] + 1e-9 >= first_authored_touchdown_source_tick
    )
    reference_phase_retiming_enabled = (
        touchdown_phase_retiming_enabled or balance_phase_retiming_enabled
    )
    phase_retiming_metrics = {
        "enabled": reference_phase_retiming_enabled,
        "touchdown_enabled": touchdown_phase_retiming_enabled,
        "balance_enabled": balance_phase_retiming_enabled,
        "final_source_tick": float(reference_phase[-1]),
        "source_progress_ticks": float(reference_phase[-1] - reference_phase[0]),
        "first_authored_touchdown_source_tick": (
            first_authored_touchdown_source_tick
        ),
        "first_authored_touchdown_reached": bool(
            first_authored_touchdown_reached
        ),
        "target_rate_minimum": float(np.min(reference_phase_target_rate)),
        "applied_rate_minimum": float(np.min(reference_phase_rate)),
        "applied_rate_mean": float(np.mean(reference_phase_rate)),
        "applied_rate_p50": percentile(reference_phase_rate, 50),
        "maximum_abs_rate_acceleration_per_second": float(
            np.max(np.abs(reference_phase_acceleration))
        ),
        "maximum_required_time_seconds": float(
            np.max(reference_phase_required_time)
        ),
        "limited_ticks": int(np.count_nonzero(reference_phase_flags & 16)),
        "held_ticks": int(np.count_nonzero(reference_phase_rate <= 1e-9)),
        "position_limited_ticks": int(
            np.count_nonzero(reference_phase_flags & 1)
        ),
        "tangential_limited_ticks": int(
            np.count_nonzero(reference_phase_flags & 2)
        ),
        "normal_limited_ticks": int(
            np.count_nonzero(reference_phase_flags & 4)
        ),
        "unsafe_edge_ticks": int(np.count_nonzero(reference_phase_flags & 8)),
        "balance_limited_ticks": int(
            np.count_nonzero(reference_phase_flags & 64)
        ),
    }
    metrics: dict[str, Any] = {
        "ticks": len(step_ns),
        "dt_seconds": DT,
        "duration_seconds": len(step_ns) * DT,
        "root_tracking_rms_m": rms(root_error),
        "root_tracking_p95_m": percentile(root_error, 95),
        "center_of_mass_tracking_rms_m": rms(center_of_mass_error),
        "center_of_mass_tracking_p95_m": percentile(center_of_mass_error, 95),
        "dcm_balance": dcm_metrics,
        "capture_landing_retarget": landing_retarget_metrics,
        "touchdown_phase_retiming": phase_retiming_metrics,
        "touchdown_viability": touchdown_viability,
        "foot_tracking_rms_m": rms(foot_error),
        "hand_tracking_rms_m": rms(hand_error),
        "hand_task_commanded": hand_task_weight > 0.0,
        "stance_foot_tracking_rms_m": rms(foot_error[stance]),
        "swing_foot_tracking_rms_m": rms(foot_error[swing]),
        "maximum_root_rotation_rad": float(np.max(rotation_angle)),
        "maximum_joint_velocity_rad_s": float(np.max(np.abs(joint_velocity))),
        "contact_phase_authority": {
            "phase_ticks": authority_phase_counts,
            "joint_velocity_envelope_active_ticks": int(
                np.count_nonzero(joint_velocity_envelope_active_coordinates)
            ),
            "joint_velocity_envelope_active_coordinates_maximum": int(
                np.max(joint_velocity_envelope_active_coordinates)
            ),
            "joint_velocity_envelope_scale_mean": float(
                np.mean(joint_velocity_envelope_scale)
            ),
            "joint_velocity_envelope_target_scale_mean": float(
                np.mean(joint_velocity_envelope_target_scale)
            ),
        },
        "maximum_dynamics_residual": float(
            np.max(dynamics_residual[physical_tick])
        ),
        "maximum_contact_acceleration_residual": float(
            np.max(contact_residual[physical_tick])
        ),
        "maximum_raw_dynamics_residual": float(np.max(dynamics_residual)),
        "maximum_raw_contact_acceleration_residual": float(
            np.max(contact_residual)
        ),
        "finite_support": {
            "enabled": required_support_margin > 0.0,
            "required_margin_m": required_support_margin,
            "loaded_ticks": int(
                np.count_nonzero(np.isfinite(minimum_support_margin))
            ),
            "minimum_margin_m": (
                float(
                    np.min(
                        minimum_support_margin[
                            np.isfinite(minimum_support_margin)
                        ]
                    )
                )
                if np.any(np.isfinite(minimum_support_margin))
                else None
            ),
        },
        "minimum_active_normal_force_n": float(
            np.min(contact_force[contact_force > 0.0])
            if np.any(contact_force > 0.0)
            else 0.0
        ),
        "maximum_normal_force_n": float(np.max(contact_force)),
        "centroidal_angular_momentum_rate_residual_nm": {
            "rms": rms(task_rms[:, 6]),
            "maximum": float(np.max(task_rms[:, 6])),
        },
        "maximum_frame_angular_task_rms_rad_s2": float(np.max(task_rms[:, 9])),
        "maximum_point_task_rms_m_s2": float(np.max(task_rms[:, -4:])),
        "latency_us": latency,
        "jitter_abs_delta_us": jitter,
        "deadline_misses": {
            "1ms": int(np.count_nonzero(step_ns > 1_000_000)),
            "5ms": int(np.count_nonzero(step_ns > 5_000_000)),
            "20ms": int(np.count_nonzero(step_ns > 20_000_000)),
        },
        "hot_loop_seconds": float(np.sum(step_ns, dtype=np.float64) / 1e9),
        "hot_loop_throughput_ticks_per_second": float(
            len(step_ns) * 1e9 / max(float(np.sum(step_ns)), 1.0)
        ),
        "status_latency_us": status_latency,
        "status_solver_work": status_solver_work,
        "solver_work": {
            "task_pseudoinverse_calls": count_distribution(
                task_pseudoinverse_calls
            ),
            "clipped_steps": count_distribution(clipped_steps),
            "task_jacobi_sweeps": count_distribution(task_jacobi_sweeps),
            "jacobi_sweeps_per_pseudoinverse": float(
                np.sum(task_jacobi_sweeps, dtype=np.float64)
                / max(np.sum(task_pseudoinverse_calls, dtype=np.float64), 1.0)
            ),
            "pseudoinverse_calls_to_latency_correlation": (
                pseudoinverse_latency_correlation
            ),
            "feasibility_projection_sweeps": count_distribution(
                feasibility_projection_sweeps
            ),
            "feasibility_halfspace_projections": count_distribution(
                feasibility_halfspace_projections
            ),
            "feasibility_polish_iterations": count_distribution(
                feasibility_polish_iterations
            ),
            "feasibility_polish_pseudoinverse_calls": count_distribution(
                feasibility_polish_pseudoinverse_calls
            ),
            "feasibility_polish_jacobi_sweeps": count_distribution(
                feasibility_polish_jacobi_sweeps
            ),
            "feasibility_projections_to_latency_correlation": (
                feasibility_projection_latency_correlation
            ),
            "by_priority": {
                name: {
                    "task_pseudoinverse_calls": count_distribution(
                        task_pseudoinverse_calls_by_level[:, index]
                    ),
                    "clipped_steps": count_distribution(
                        clipped_steps_by_level[:, index]
                    ),
                    "task_jacobi_sweeps": count_distribution(
                        task_jacobi_sweeps_by_level[:, index]
                    ),
                    "ticks_with_clipping": int(
                        np.count_nonzero(clipped_steps_by_level[:, index])
                    ),
                }
                for index, name in enumerate(PRIORITY_NAMES)
            },
        },
        "temporal_windows": temporal_windows,
        "runtime": runtime,
        "status_counts": status_counts,
        "support_phase_counts": support_phase_counts,
        "support_phase_counts_by_target": support_phase_counts_by_target,
        "support_phase_maximum_ticks": support_phase_maximum_ticks,
        "support_tangential_speed_mps": support_speed,
        "maximum_touchdown_transition_ticks": maximum_touchdown_transition_ticks,
        "maximum_touchdown_admission_delay_ticks": (
            maximum_touchdown_admission_delay_ticks
        ),
        "delayed_touchdown_admission_ticks": delayed_touchdown_admission_ticks,
        "maximum_precontact_transition_ticks": maximum_precontact_transition_ticks,
        "nominal_prefix": nominal_prefix,
        "cadence": cadence,
    }
    functional_checks = {
        "no_infeasible_or_failed_ticks": (
            status_counts["primal_infeasible"] == 0
            and status_counts["failed"] == 0
        ),
        "no_contact_contingency_ticks": (
            status_counts["normal_contact_contingency"] == 0
            and status_counts["contact_release_contingency"] == 0
        ),
        "touchdown_transition_completes_within_8_ticks": (
            metrics["maximum_touchdown_transition_ticks"] <= 8
        ),
        "touchdown_admission_completes_within_8_ticks": (
            metrics["maximum_touchdown_admission_delay_ticks"] <= 8
        ),
        "root_tracking_rms_le_5cm": metrics["root_tracking_rms_m"] <= 0.05,
        "stance_foot_tracking_rms_le_2cm": (
            metrics["stance_foot_tracking_rms_m"] <= 0.02
        ),
        "swing_foot_tracking_rms_le_8cm": (
            metrics["swing_foot_tracking_rms_m"] <= 0.08
        ),
        "root_rotation_le_5deg": metrics["maximum_root_rotation_rad"]
        <= np.deg2rad(5.0),
        "joint_velocity_le_8rad_s": (
            metrics["maximum_joint_velocity_rad_s"] <= 8.0 + 1e-8
        ),
        "dynamics_residual_le_1e_8": metrics["maximum_dynamics_residual"]
        <= 1e-8,
        "contact_residual_le_1e_8": (
            metrics["maximum_contact_acceleration_residual"] <= 1e-8
        ),
    }
    if hand_task_weight > 0.0:
        functional_checks["hand_tracking_rms_le_5cm"] = (
            metrics["hand_tracking_rms_m"] <= 0.05
        )
    if required_support_margin > 0.0:
        measured_support_margin = metrics["finite_support"]["minimum_margin_m"]
        functional_checks["finite_support_margin_respected"] = (
            measured_support_margin is not None
            and measured_support_margin >= required_support_margin - 1e-8
        )
    if (
        reference_phase_retiming_enabled
        and first_authored_touchdown_source_tick is not None
    ):
        functional_checks["retiming_reaches_first_authored_touchdown"] = (
            first_authored_touchdown_reached
        )
    checks = {
        **functional_checks,
        "p99_tick_le_5ms": metrics["latency_us"]["p99"] <= 5_000.0,
    }
    checks = {name: bool(passed) for name, passed in checks.items()}
    metrics["acceptance"] = {
        "passed": all(checks.values()),
        "functional_passed": all(functional_checks.values()),
        "real_time_passed": checks["p99_tick_le_5ms"],
        "checks": checks,
        "thresholds": {
            "root_tracking_rms_m": 0.05,
            "stance_foot_tracking_rms_m": 0.02,
            "swing_foot_tracking_rms_m": 0.08,
            "hand_tracking_rms_m": 0.05,
            "maximum_root_rotation_rad": float(np.deg2rad(5.0)),
            "maximum_joint_velocity_rad_s": 8.0,
            "maximum_dynamics_residual": 1e-8,
            "maximum_contact_acceleration_residual": 1e-8,
            "minimum_support_margin_m": required_support_margin,
            "maximum_touchdown_transition_ticks": 8,
            "p99_tick_us": 5_000.0,
        },
    }
    return metrics


def render_report(metrics: dict[str, Any], metadata: dict[str, Any]) -> str:
    gate = metrics["acceptance"]
    runtime = metrics["runtime"]
    failed = [name for name, passed in gate["checks"].items() if not passed]
    dcm_metrics = metrics["dcm_balance"]
    dcm_lines = (
        [
            "## DCM and virtual-ZMP balance",
            "",
            f"- DCM RMS / p95: `{dcm_metrics['tracking_rms_m'] * 100:.3f}` / "
            f"`{dcm_metrics['tracking_p95_m'] * 100:.3f} cm`.",
            f"- Virtual ZMP clipped on `{dcm_metrics['zmp_clipped_fraction'] * 100:.2f}%` "
            f"of ticks; clip-distance RMS / max "
            f"`{dcm_metrics['zmp_clip_distance_rms_m'] * 100:.3f}` / "
            f"`{dcm_metrics['zmp_clip_distance_maximum_m'] * 100:.3f} cm`.",
            f"- Measured-height natural frequency min / p50 / max: "
            f"`{dcm_metrics['natural_frequency_minimum_rad_s']:.3f}` / "
            f"`{dcm_metrics['natural_frequency_p50_rad_s']:.3f}` / "
            f"`{dcm_metrics['natural_frequency_maximum_rad_s']:.3f} rad/s`.",
            f"- Minimum measured CoM height: "
            f"`{dcm_metrics['measured_com_height_minimum_m']:.3f} m`; height-floor "
            f"ticks: `{dcm_metrics['height_clamped_ticks']}`.",
            f"- CoM command acceleration p95 / max: "
            f"`{dcm_metrics['command_acceleration_p95_mps2']:.3f}` / "
            f"`{dcm_metrics['command_acceleration_maximum_mps2']:.3f} m/s²`; "
            f"support hull `{dcm_metrics['support_vertices_minimum']}–"
            f"{dcm_metrics['support_vertices_maximum']}` vertices.",
            f"- Signed measured DCM support margin min / p05: "
            f"`{dcm_metrics['support_margin_minimum_m'] * 100:.3f}` / "
            f"`{dcm_metrics['support_margin_p05_m'] * 100:.3f} cm`; inside on "
            f"`{dcm_metrics['support_margin_positive_fraction'] * 100:.2f}%` of ticks.",
            "",
        ]
        if dcm_metrics["enabled"]
        else []
    )

    def mib(value: int | None) -> str:
        return "n/a" if value is None else f"{value / (1024 * 1024):.3f}"

    synthetic = metadata.get("motion_profile") == "synthetic-step"
    standalone = metadata.get("motion_profile") == "standalone-reference"
    source_lines = (
        [
            f"- Source: immutable open-loop-admitted artifact "
            f"`{metadata['standalone_reference_inputs']}` from "
            f"`{metadata['standalone_generator']}`.",
            "- Admission contract: authored root, CoM, foot jets, and contact "
            "schedule are consumed unchanged; eval-side reconstruction, projection, "
            "and retiming are rejected.",
        ]
        if standalone
        else
        [
            "- Source: deterministic, predeclared G1-sized left step; no mocap "
            "or learned policy contributes contact labels or target motion.",
            f"- Contact schedule: liftoff tick "
            f"`{metadata['synthetic_liftoff_tick']}`, touchdown tick "
            f"`{metadata['synthetic_touchdown_tick']}`, step "
            f"`{metadata['synthetic_step_length_m']:.3f} m`, clearance "
            f"`{metadata['synthetic_step_clearance_m']:.3f} m`.",
        ]
        if synthetic
        else [
            f"- Source: CMU subject {metadata['subject']}, trial {metadata['trial']} "
            f"(`{metadata['description']}`, {metadata['source_rate_hz']} Hz)."
        ]
    )
    lines = [
        (
            "# Bonesaw floating G1 admitted-reference tracking"
            if standalone
            else (
                "# Bonesaw floating G1 synthetic-step acceptance"
                if synthetic
                else "# Bonesaw floating CMU walking corpus"
            )
        ),
        "",
        "This is a moving-root, contact-aware trace through the Rust floating "
        "inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, "
        "and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown "
        "anchors, target shaping, Jacobians, hard contact rows, the solve, "
        "SE(3) integration, and every measured tick.",
        "",
        "## Source and motion profile",
        "",
        *source_lines,
        f"- Target model: `{metadata['target_model']}` with "
        f"{metadata['contact_patch_points_per_foot']} sole contact points per foot "
        f"(`x={metadata['contact_patch_center_x_m']:.3f}±"
        f"{metadata['contact_patch_half_length_m']:.3f} m`, "
        f"`y=±{metadata['contact_patch_half_width_m']:.4f} m`, "
        f"`z={metadata['contact_patch_z_m']:.3f} m` in the foot frame).",
        f"- Applied target displacement: "
        f"`{metadata['target_stride_displacement_m'][0] * metadata['forward_motion_scale']:.3f} m` "
        f"forward per `{metadata['cycle_duration_seconds']:.3f} s` source cycle.",
        f"- Applied mean forward speed: "
        f"`{metadata['target_mean_forward_speed_m_s'] * metadata['forward_motion_scale']:.3f} m/s` "
        f"(`{metadata['forward_motion_scale']:.2f}×` forward, "
        f"`{metadata['lateral_motion_scale']:.2f}×` lateral retarget scale).",
        f"- Cadence schedule: `{metadata['cadence_scale_pattern']}` in "
        f"`{metadata['cadence_block_seconds']:.1f} s` blocks with an applied "
        f"`{metadata.get('cadence_multiplier', 1.0):.2f}×` multiplier.",
        "- Each four-point sole retains four independent friction-limited force "
        "slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins "
        "in a planned normal-only transition before tangential lock; fallback "
        "and contact release remain separately observable.",
        f"- Finite-support CoP constraint: "
        f"`{'enabled' if metrics['finite_support']['enabled'] else 'disabled'}`; "
        f"required `{metrics['finite_support']['required_margin_m'] * 1000:.3f} mm`, "
        f"measured minimum "
        f"`{metrics['finite_support']['minimum_margin_m'] * 1000:.3f} mm` "
        f"over `{metrics['finite_support']['loaded_ticks']}` loaded ticks."
        if metrics["finite_support"]["minimum_margin_m"] is not None
        else "- Finite-support CoP constraint: disabled; no loaded finite patch was declared.",
        f"- Balance task: `{metadata['center_of_mass_reference']}` "
        f"`{metadata['center_of_mass_reference_semantics']}` reference at "
        f"`{metadata['center_of_mass_task_priority']}` priority with weight "
        f"`{metadata['center_of_mass_task_weight']:.3f}` and "
        f"`{metadata['center_of_mass_frequency_hz']:.3f} Hz` response "
        f"through `{metadata['center_of_mass_controller']}` horizontal control "
        f"(`{'position-only' if metadata['center_of_mass_zero_reference_derivatives'] else 'full position/velocity/acceleration jet'}`). "
        f"Foot tracking: `{metadata['foot_task_priority']}` priority at weight "
        f"`{metadata['foot_task_weight']:.3f}`. "
        f"Hand task weight: `{metadata['hand_task_weight']:.3f}` "
        "(hand error is observational when zero).",
        f"- Root horizontal reference: `{metadata['root_reference']}`.",
        f"- Root / swing-point response: `{metadata['root_frequency_hz']:.3f}` / "
        f"`{metadata['point_frequency_hz']:.3f} Hz` critically damped; root "
        f"angular/height/horizontal task weights "
        f"`{metadata['root_angular_task_weight']:.3f}` / "
        f"`{metadata['root_height_task_weight']:.3f}` / "
        f"`{metadata['root_horizontal_task_weight']:.3f}`.",
        f"- Whole-body posture: `{metadata['joint_posture_priority']}` priority "
        f"with weight `{metadata['joint_posture_weight']:.3f}`.",
        f"- Protected upper-body posture: "
        f"`{metadata['upper_body_posture_priority']}` priority with weight "
        f"`{metadata['upper_body_posture_weight']:.3f}` over "
        f"{len(metadata['upper_body_posture_joints'])} waist/arm coordinates.",
        f"- Joint-velocity envelope: `{metadata['joint_velocity_envelope_priority']}` "
        f"priority with weight `{metadata['joint_velocity_envelope_weight']:.3f}`, "
        f"activating at `{metadata['joint_velocity_envelope_activation_fraction']:.1%}` "
        f"of each effective limit with `{metadata['joint_velocity_envelope_frequency_hz']:.3f} Hz` "
        f"response and `{metadata['joint_velocity_envelope_phase_policy']}` measured-phase policy "
        f"with immediate engagement and bounded release over "
        f"`{metadata['joint_velocity_envelope_phase_transition_ticks']}` ticks.",
        f"- Centroidal angular-momentum damping: "
        f"`{metadata['centroidal_angular_momentum_priority']}` priority with weight "
        f"`{metadata['centroidal_angular_momentum_weight']:.3f}` and "
        f"`{metadata['centroidal_angular_momentum_frequency_hz']:.3f} Hz` response.",
        f"- Pre-contact viability preview: `{metadata['precontact_ticks']}` ticks "
        f"(`{metadata['precontact_seconds']:.3f} s`) with a receding cubic "
        f"landing law capped at "
        f"`{metadata['precontact_maximum_acceleration_m_s2']:.3f} m/s²`; "
        "the authored edge requests touchdown, while measured sole proximity "
        "and velocity admit physical contact.",
        f"- Capture landing retarget: "
        f"`{'enabled' if metadata['capture_landing_retarget_enabled'] else 'disabled'}`; "
        f"authored offset ≤ `{metadata['capture_landing_maximum_offset_m']:.3f} m`, "
        f"root reach ≤ `{metadata['capture_landing_maximum_root_reach_m']:.3f} m`, "
        f"anchor speed ≤ `{metadata['capture_landing_maximum_anchor_speed_mps']:.3f} m/s`.",
        f"- Landing commitment: freeze the retargeted anchor for the final "
        f"`{metadata['capture_landing_freeze_ticks']}` scheduled ticks and throughout "
        "delayed admission.",
        f"- Coupled touchdown phase retiming: "
        f"`{'enabled' if metadata['touchdown_phase_retiming_enabled'] else 'disabled'}`; "
        f"minimum rate `{metadata['touchdown_phase_minimum_rate']:.3f}`, guard "
        f"`{metadata['touchdown_phase_guard_time_seconds']:.3f} s`, engagement / "
        f"release `{metadata['touchdown_phase_engagement_ticks']} / "
        f"{metadata['touchdown_phase_release_ticks']}` ticks. Rust samples root, "
        "CoM, all endpoint jets, and contact intent from one explicit cursor.",
        f"- Coupled balance phase retiming: "
        f"`{'enabled' if metadata['balance_phase_retiming_enabled'] else 'disabled'}`; "
        f"hold at signed DCM margin ≤ "
        f"`{metadata['balance_phase_hold_margin_m']:.3f} m`, recover nominal rate at "
        f"`{metadata['balance_phase_full_rate_margin_m']:.3f} m`.",
        f"- Touchdown admission limits: "
        f"`{metadata['maximum_touchdown_position_error_m']:.3f} m` position, "
        f"`{metadata['maximum_touchdown_tangential_speed_mps']:.3f} m/s` "
        f"tangential, and "
        f"`{metadata['maximum_touchdown_normal_speed_mps']:.3f} m/s` normal; "
        "the outgoing support is retained until its replacement locks.",
        f"- Reference touchdown blend: `{metadata['touchdown_blend_ticks']}` ticks "
        f"(`{metadata['touchdown_blend_seconds']:.3f} s`) with a quintic "
        "zero-velocity endpoint.",
        f"- Touchdown stabilization point: "
        f"`{'sole center' if metadata['material_touchdown_task'] else 'ankle origin'}`.",
        "",
        "## Acceptance",
        "",
        f"Functional: **{'PASS' if gate['functional_passed'] else 'FAIL'}**  ",
        f"5 ms p99 deadline: **{'PASS' if gate['real_time_passed'] else 'FAIL'}**  ",
        f"Combined: **{'PASS' if gate['passed'] else 'FAIL'}**",
        "",
        "| Check | Result |",
        "|---|---:|",
    ]
    for name, passed in gate["checks"].items():
        lines.append(f"| `{name}` | {'PASS' if passed else 'FAIL'} |")
    if failed:
        lines += ["", "Failed checks: " + ", ".join(f"`{name}`" for name in failed) + "."]
    lines += [
        "",
        "## Nominal prefix before first contingency",
        "",
        "This window ends immediately before the first normal-only, "
        "contact-release, infeasible, or numerical-failure status.",
        "",
        "| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {metrics['nominal_prefix']['ticks']:,} | "
        f"{metrics['nominal_prefix']['duration_seconds']:.3f} s | "
        f"{metrics['nominal_prefix']['root_tracking_rms_m'] * 100:.3f} cm | "
        f"{metrics['nominal_prefix']['stance_foot_tracking_rms_m'] * 100:.3f} cm | "
        f"{metrics['nominal_prefix']['swing_foot_tracking_rms_m'] * 100:.3f} cm | "
        f"{metrics['nominal_prefix']['hand_tracking_rms_m'] * 100:.3f} cm | "
        f"{np.rad2deg(metrics['nominal_prefix']['maximum_root_rotation_rad']):.3f}° | "
        f"{metrics['nominal_prefix']['maximum_joint_velocity_rad_s']:.3f} rad/s | "
        f"{metrics['nominal_prefix']['latency_p99_us']:.1f} µs |",
        "",
        f"Nominal hard residual maxima: dynamics "
        f"`{metrics['nominal_prefix']['maximum_dynamics_residual']:.3e}`, "
        f"contact acceleration "
        f"`{metrics['nominal_prefix']['maximum_contact_acceleration_residual']:.3e}`.",
        "",
        "## Tracking and physical residuals",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| root RMS | {metrics['root_tracking_rms_m'] * 100:.3f} cm |",
        f"| authored reference vs measured CoM RMS / p95 | "
        f"{metrics['center_of_mass_tracking_rms_m'] * 100:.3f} / "
        f"{metrics['center_of_mass_tracking_p95_m'] * 100:.3f} cm |",
        f"| stance foot RMS | {metrics['stance_foot_tracking_rms_m'] * 100:.3f} cm |",
        f"| swing foot RMS | {metrics['swing_foot_tracking_rms_m'] * 100:.3f} cm |",
        f"| hand RMS | {metrics['hand_tracking_rms_m'] * 100:.3f} cm |",
        f"| maximum root rotation | {np.rad2deg(metrics['maximum_root_rotation_rad']):.3f}° |",
        f"| maximum joint velocity | "
        f"{metrics['maximum_joint_velocity_rad_s']:.3f} rad/s |",
        f"| dynamics residual | {metrics['maximum_dynamics_residual']:.3e} |",
        f"| contact acceleration residual | "
        f"{metrics['maximum_contact_acceleration_residual']:.3e} |",
        f"| raw max dynamics residual, including rejected ticks | "
        f"{metrics['maximum_raw_dynamics_residual']:.3e} |",
        f"| raw max contact residual, including rejected ticks | "
        f"{metrics['maximum_raw_contact_acceleration_residual']:.3e} |",
        f"| active normal force range | "
        f"{metrics['minimum_active_normal_force_n']:.3f}–"
        f"{metrics['maximum_normal_force_n']:.3f} N |",
        f"| centroidal momentum-rate residual RMS / max | "
        f"{metrics['centroidal_angular_momentum_rate_residual_nm']['rms']:.3f} / "
        f"{metrics['centroidal_angular_momentum_rate_residual_nm']['maximum']:.3f} N·m |",
        f"| point-task acceleration RMS max | "
        f"{metrics['maximum_point_task_rms_m_s2']:.3f} m/s² |",
        f"| frame-angular acceleration RMS max | "
        f"{metrics['maximum_frame_angular_task_rms_rad_s2']:.3f} rad/s² |",
        f"| longest pre-contact / touchdown transition | "
        f"{metrics['maximum_precontact_transition_ticks']} / "
        f"{metrics['maximum_touchdown_transition_ticks']} ticks |",
        f"| delayed touchdown admission ticks / longest delay | "
        f"{metrics['delayed_touchdown_admission_ticks']} / "
        f"{metrics['maximum_touchdown_admission_delay_ticks']} ticks |",
        "",
        *dcm_lines,
        "## Coupled touchdown phase retiming",
        "",
        f"- Enabled: `{metrics['touchdown_phase_retiming']['enabled']}` "
        f"(touchdown `{metrics['touchdown_phase_retiming']['touchdown_enabled']}`, "
        f"balance `{metrics['touchdown_phase_retiming']['balance_enabled']}`); final source "
        f"tick `{metrics['touchdown_phase_retiming']['final_source_tick']:.3f}`, "
        f"progress `{metrics['touchdown_phase_retiming']['source_progress_ticks']:.3f}` ticks.",
        f"- First post-liftoff authored touchdown source tick: "
        f"`{metrics['touchdown_phase_retiming']['first_authored_touchdown_source_tick']}`; "
        f"reached before trace end: "
        f"`{metrics['touchdown_phase_retiming']['first_authored_touchdown_reached']}`.",
        f"- Target / applied minimum rate: "
        f"`{metrics['touchdown_phase_retiming']['target_rate_minimum']:.4f}` / "
        f"`{metrics['touchdown_phase_retiming']['applied_rate_minimum']:.4f}`; "
        f"mean / p50 applied `{metrics['touchdown_phase_retiming']['applied_rate_mean']:.4f}` / "
        f"`{metrics['touchdown_phase_retiming']['applied_rate_p50']:.4f}`.",
        f"- Limited / zero-rate hold ticks: "
        f"`{metrics['touchdown_phase_retiming']['limited_ticks']}` / "
        f"`{metrics['touchdown_phase_retiming']['held_ticks']}`; maximum required "
        f"landing time `{metrics['touchdown_phase_retiming']['maximum_required_time_seconds']:.4f} s`.",
        f"- Position / tangential / normal limiting ticks: "
        f"`{metrics['touchdown_phase_retiming']['position_limited_ticks']}` / "
        f"`{metrics['touchdown_phase_retiming']['tangential_limited_ticks']}` / "
        f"`{metrics['touchdown_phase_retiming']['normal_limited_ticks']}`; unsafe-edge "
        f"ticks `{metrics['touchdown_phase_retiming']['unsafe_edge_ticks']}`.",
        f"- Balance-margin limited ticks: "
        f"`{metrics['touchdown_phase_retiming']['balance_limited_ticks']}`.",
        "",
        "## Capture-aware landing",
        "",
        f"- Active target-ticks: "
        f"`{metrics['capture_landing_retarget']['active_ticks']}`; policy updates "
        f"`{metrics['capture_landing_retarget']['policy_update_ticks']}`, frozen "
        f"`{metrics['capture_landing_retarget']['frozen_target_ticks']}`.",
        f"- Maximum applied offset / root reach: "
        f"`{metrics['capture_landing_retarget']['maximum_applied_offset_m']:.4f} / "
        f"{metrics['capture_landing_retarget']['maximum_root_to_landing_reach_m']:.4f} m`.",
        f"- Authored-offset / reach / slew limited ticks: "
        f"`{metrics['capture_landing_retarget']['authored_offset_limited_ticks']} / "
        f"{metrics['capture_landing_retarget']['reach_limited_ticks']} / "
        f"{metrics['capture_landing_retarget']['slew_limited_ticks']}`.",
        f"- Authored geometry outside configured reach: "
        f"`{metrics['capture_landing_retarget']['unreachable_authored_geometry_ticks']}` target-ticks.",
        f"- Pending-touchdown observations / jointly viable: "
        f"`{metrics['touchdown_viability']['observed_target_ticks']} / "
        f"{metrics['touchdown_viability']['jointly_viable_ticks']}` target-ticks.",
        f"- Minimum position error / tangential speed / normal speed: "
        f"`{metrics['touchdown_viability']['position_error_minimum_m']:.4f} m / "
        f"{metrics['touchdown_viability']['tangential_speed_minimum_mps']:.4f} / "
        f"{metrics['touchdown_viability']['normal_speed_minimum_mps']:.4f} m/s`.",
        f"- Individually viable position / tangential / normal target-ticks: "
        f"`{metrics['touchdown_viability']['position_viable_ticks']} / "
        f"{metrics['touchdown_viability']['tangential_viable_ticks']} / "
        f"{metrics['touchdown_viability']['normal_viable_ticks']}`.",
        "",
        "## Measured contact-phase authority",
        "",
        f"- Phase ticks: unsupported "
        f"`{metrics['contact_phase_authority']['phase_ticks']['unsupported']}`, "
        f"single support `{metrics['contact_phase_authority']['phase_ticks']['single_support']}`, "
        f"precontact `{metrics['contact_phase_authority']['phase_ticks']['precontact']}`, "
        f"multi-support `{metrics['contact_phase_authority']['phase_ticks']['multi_support']}`.",
        f"- Joint-velocity envelope active on "
        f"`{metrics['contact_phase_authority']['joint_velocity_envelope_active_ticks']}` ticks; "
        f"maximum active coordinates "
        f"`{metrics['contact_phase_authority']['joint_velocity_envelope_active_coordinates_maximum']}`; "
        f"mean target/applied scale "
        f"`{metrics['contact_phase_authority']['joint_velocity_envelope_target_scale_mean']:.3f}` / "
        f"`{metrics['contact_phase_authority']['joint_velocity_envelope_scale_mean']:.3f}`.",
        "",
        "## Runtime",
        "",
        "| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {metrics['ticks']:,} | {metrics['duration_seconds']:.1f} s | "
        f"{metrics['latency_us']['p50']:.1f} µs | "
        f"{metrics['latency_us']['p95']:.1f} µs | "
        f"{metrics['latency_us']['p99']:.1f} µs | "
        f"{metrics['latency_us']['max']:.1f} µs | "
        f"{metrics['status_counts']['solved']:,} | "
        f"{metrics['status_counts']['solved_with_slack']:,} | "
        f"{metrics['status_counts']['precontact_transition']:,} | "
        f"{metrics['status_counts']['touchdown_transition']:,} | "
        f"{metrics['status_counts']['normal_contact_contingency']:,} | "
        f"{metrics['status_counts']['contact_release_contingency']:,} | "
        f"{metrics['status_counts']['primal_infeasible']:,} | "
        f"{metrics['status_counts']['failed']:,} |",
        "",
        "### Latency distribution and deadlines",
        "",
        "The per-tick timer is inside the Rust batch loop. Call-level wall/CPU "
        "measurements wrap the single PyO3 call and therefore include only one "
        "Python→Rust boundary crossing, not per-tick Python work.",
        "",
        "| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {metrics['latency_us']['mean']:.1f} | "
        f"{metrics['latency_us']['stddev']:.1f} | "
        f"{metrics['latency_us']['mad']:.1f} | "
        f"{metrics['latency_us']['p90']:.1f} | "
        f"{metrics['latency_us']['p99_9']:.1f} | "
        f"{metrics['latency_us']['p99_99']:.1f} | "
        f"{metrics['jitter_abs_delta_us']['p99']:.1f} | "
        f"{metrics['deadline_misses']['1ms']:,} | "
        f"{metrics['deadline_misses']['5ms']:,} | "
        f"{metrics['deadline_misses']['20ms']:,} | "
        f"{metrics['hot_loop_throughput_ticks_per_second']:.1f} |",
        "",
        "### Latency by solver/contact status",
        "",
        "| status | ticks | p50 µs | p95 µs | p99 µs | max µs |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, values in metrics["status_latency_us"].items():
        lines.append(
            f"| {name} | {values['ticks']:,} | {values['p50']:.1f} | "
            f"{values['p95']:.1f} | {values['p99']:.1f} | "
            f"{values['max']:.1f} |"
        )
    solver_work = metrics["solver_work"]
    lines += [
        "",
        "### Strict-solver work attribution",
        "",
        "A task pseudoinverse is the dominant dense kernel. Each active priority "
        "normally needs one call after exact projected-inverse reuse; accepted "
        "bound/inequality truncations require another projected solve.",
        "",
        "| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {solver_work['task_pseudoinverse_calls']['mean']:.2f} | "
        f"{solver_work['task_pseudoinverse_calls']['p95']:.1f} | "
        f"{solver_work['task_pseudoinverse_calls']['p99']:.1f} | "
        f"{solver_work['task_pseudoinverse_calls']['max']:.0f} | "
        f"{solver_work['clipped_steps']['mean']:.2f} | "
        f"{solver_work['clipped_steps']['p99']:.1f} | "
        f"{solver_work['clipped_steps']['max']:.0f} | "
        f"{solver_work['pseudoinverse_calls_to_latency_correlation']:.4f} |",
        "",
        "The feasibility seed is a cyclic hard-halfspace projection before "
        "semantic task solving. Near-feasible seeds may then enter a dense "
        "active-set polish; these counters make that previously hidden work "
        "visible without timing inside the solver.",
        "",
        "| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |",
        "|---:|---:|---:|---:|---:|---:|",
        f"| {solver_work['feasibility_projection_sweeps']['mean']:.2f}/"
        f"{solver_work['feasibility_projection_sweeps']['p95']:.1f}/"
        f"{solver_work['feasibility_projection_sweeps']['p99']:.1f}/"
        f"{solver_work['feasibility_projection_sweeps']['max']:.0f} | "
        f"{solver_work['feasibility_halfspace_projections']['mean']:.2f}/"
        f"{solver_work['feasibility_halfspace_projections']['p95']:.1f}/"
        f"{solver_work['feasibility_halfspace_projections']['p99']:.1f}/"
        f"{solver_work['feasibility_halfspace_projections']['max']:.0f} | "
        f"{solver_work['feasibility_polish_iterations']['mean']:.2f}/"
        f"{solver_work['feasibility_polish_iterations']['p99']:.1f}/"
        f"{solver_work['feasibility_polish_iterations']['max']:.0f} | "
        f"{solver_work['feasibility_polish_pseudoinverse_calls']['mean']:.2f}/"
        f"{solver_work['feasibility_polish_pseudoinverse_calls']['p99']:.1f}/"
        f"{solver_work['feasibility_polish_pseudoinverse_calls']['max']:.0f} | "
        f"{solver_work['feasibility_polish_jacobi_sweeps']['mean']:.2f}/"
        f"{solver_work['feasibility_polish_jacobi_sweeps']['p99']:.1f}/"
        f"{solver_work['feasibility_polish_jacobi_sweeps']['max']:.0f} | "
        f"{solver_work['feasibility_projections_to_latency_correlation']:.4f} |",
        "",
        "| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |",
        "|---|---:|---:|---:|",
    ]
    for name, work in metrics["status_solver_work"].items():
        calls = work["task_pseudoinverse_calls"]
        clipped = work["clipped_steps"]
        lines.append(
            f"| {name} | {metrics['status_latency_us'][name]['ticks']:,} | "
            f"{calls['mean']:.2f}/{calls['p99']:.1f}/{calls['max']:.0f} | "
            f"{clipped['mean']:.2f}/{clipped['p99']:.1f}/{clipped['max']:.0f} |"
        )
    lines += [
        "",
        "| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, work in solver_work["by_priority"].items():
        calls = work["task_pseudoinverse_calls"]
        clipped = work["clipped_steps"]
        sweeps = work["task_jacobi_sweeps"]
        lines.append(
            f"| {name} | {calls['mean']:.2f}/{calls['p99']:.1f}/{calls['max']:.0f} | "
            f"{sweeps['mean']:.2f}/{sweeps['p99']:.1f}/{sweeps['max']:.0f} | "
            f"{clipped['mean']:.2f}/{clipped['p99']:.1f}/{clipped['max']:.0f} | "
            f"{work['ticks_with_clipping']:,} |"
        )
    lines += [
        "",
        "### Per-target support-phase telemetry",
        "",
        "These are Rust-owned phase states sampled after each tick; solver "
        "contingencies remain separate from planned touchdown.",
        "",
        "| target | swing | precontact | touchdown normal | locked | normal fallback |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for target, counts in zip(
        metadata["target_frames"],
        metrics["support_phase_counts_by_target"],
        strict=True,
    ):
        lines.append(
            f"| {target} | {counts['swing']:,} | {counts['precontact']:,} | "
            f"{counts['touchdown_normal']:,} | {counts['locked']:,} | "
            f"{counts['normal_fallback']:,} |"
        )
    phase_maximum = metrics["support_phase_maximum_ticks"]
    lines += [
        "",
        f"Maximum contiguous phase ages: precontact "
        f"`{phase_maximum['precontact']}` ticks, planned normal touchdown "
        f"`{phase_maximum['touchdown_normal']}` ticks, normal fallback "
        f"`{phase_maximum['normal_fallback']}` ticks.",
    ]
    for phase in ("precontact", "touchdown_normal"):
        if phase in metrics["support_tangential_speed_mps"]:
            speed = metrics["support_tangential_speed_mps"][phase]
            lines.append(
                f"{phase.replace('_', ' ').title()} sole-center tangential speed: "
                f"p50 `{speed['p50_mps']:.4f} m/s`, p95 "
                f"`{speed['p95_mps']:.4f} m/s`, max "
                f"`{speed['maximum_mps']:.4f} m/s` over "
                f"{speed['samples']:,} samples."
            )
    lines += [
        "",
        "### CPU, memory, faults, context switches, and Python runtime",
        "",
        "All large NumPy input/output buffers and the Rust session are constructed "
        "before this measurement. Python `tracemalloc` does not observe native "
        "Rust allocations; the standalone native controller sentinels remain the "
        "authoritative hot-loop allocation gate.",
        "",
        "| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {runtime['call_wall_ns'] / 1e9:.3f} | "
        f"{runtime['process_cpu_ns'] / 1e9:.3f} | "
        f"{runtime['thread_cpu_ns'] / 1e9:.3f} | "
        f"{runtime['process_cpu_to_wall_ratio']:.3f} | "
        f"{runtime['thread_cpu_to_wall_ratio']:.3f} | "
        f"{mib(runtime['rss_before_bytes'])} | "
        f"{mib(runtime['rss_after_bytes'])} | "
        f"{mib(runtime['rss_delta_bytes'])} | "
        f"{mib(runtime['peak_rss_bytes'])} | "
        f"{mib(runtime['python_tracemalloc_peak_bytes'])} | "
        f"{runtime['python_gc_delta']['collections']:,} | "
        f"{runtime['usage_delta']['minor_faults']:,} | "
        f"{runtime['usage_delta']['major_faults']:,} | "
        f"{runtime['usage_delta']['voluntary_context_switches']:,} | "
        f"{runtime['usage_delta']['involuntary_context_switches']:,} |",
        "",
        "### Execution over time",
        "",
        "Each row is one tenth of the run so warm drift, scheduler tails, tracking "
        "loss, and the exact onset of contact contingency remain visible.",
        "",
        "| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for window in metrics["temporal_windows"]:
        lines.append(
            f"| {window['tick_start']}–{window['tick_stop'] - 1} | "
            f"{window['latency_p50_us']:.1f} | "
            f"{window['latency_p99_us']:.1f} | "
            f"{window['task_pseudoinverse_calls_mean']:.2f} | "
            f"{window['task_jacobi_sweeps_mean']:.2f} | "
            f"{window['clipped_steps_mean']:.2f} | "
            f"{window['feasibility_projection_sweeps_mean']:.2f} | "
            f"{window['feasibility_halfspace_projections_mean']:.2f} | "
            f"{window['root_tracking_rms_m'] * 100:.3f} | "
            f"{window['foot_tracking_rms_m'] * 100:.3f} | "
            f"{window['maximum_dynamics_residual']:.2e} | "
            f"{window['maximum_contact_acceleration_residual']:.2e} | "
            f"{window['contingency_or_rejected_ticks']:,} |"
        )
    lines += [
        "",
        "## Cadence sensitivity",
        "",
        "| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |",
        "|---|---:|---:|---:|---:|",
    ]
    for cadence, values in metrics["cadence"].items():
        lines.append(
            f"| {cadence} | {values['ticks']:,} | "
            f"{values['root_tracking_rms_m'] * 100:.3f} | "
            f"{values['foot_tracking_rms_m'] * 100:.3f} | "
            f"{values['hand_tracking_rms_m'] * 100:.3f} |"
        )
    lines += [
        "",
        "Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and "
        "source/retarget metadata are in `floating-walk-metrics.json`.",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    if args.ticks <= 0:
        raise SystemExit("--ticks must be positive")
    if args.startup_balance_ticks <= 0 or args.startup_ramp_ticks <= 0:
        raise SystemExit("--startup-balance-ticks and --startup-ramp-ticks must be positive")
    if args.precontact_ticks < 0 or args.touchdown_blend_ticks < 0:
        raise SystemExit("transition tick settings must be nonnegative")
    if (
        not np.isfinite(args.precontact_maximum_acceleration)
        or args.precontact_maximum_acceleration <= 0.0
    ):
        raise SystemExit("--precontact-maximum-acceleration must be positive")
    if (
        not np.isfinite(args.forward_motion_scale)
        or args.forward_motion_scale <= 0.0
        or not np.isfinite(args.lateral_motion_scale)
        or args.lateral_motion_scale <= 0.0
    ):
        raise SystemExit("--forward-motion-scale and --lateral-motion-scale must be positive")
    if (
        not np.isfinite(args.maximum_root_to_foot_reach)
        or args.maximum_root_to_foot_reach < 0.0
    ):
        raise SystemExit("--maximum-root-to-foot-reach must be finite and nonnegative")
    if (
        not np.isfinite(args.synthetic_step_length)
        or args.synthetic_step_length <= 0.0
        or not np.isfinite(args.synthetic_step_clearance)
        or args.synthetic_step_clearance <= 0.0
        or not np.isfinite(args.synthetic_root_transfer_scale)
        or not 0.0 <= args.synthetic_root_transfer_scale <= 1.0
        or args.synthetic_swing_ticks < 20
    ):
        raise SystemExit(
            "synthetic step length/clearance must be positive and root transfer in [0, 1]"
        )
    if (
        not np.isfinite(args.center_of_mass_task_weight)
        or args.center_of_mass_task_weight < 0.0
        or not np.isfinite(args.center_of_mass_frequency_hz)
        or args.center_of_mass_frequency_hz <= 0.0
        or not np.isfinite(args.dcm_feedback_gain_per_second)
        or args.dcm_feedback_gain_per_second <= 0.0
        or not np.isfinite(args.dcm_support_margin)
        or args.dcm_support_margin < 0.0
        or not np.isfinite(args.dcm_maximum_horizontal_acceleration)
        or args.dcm_maximum_horizontal_acceleration <= 0.0
        or not np.isfinite(args.upper_body_posture_weight)
        or args.upper_body_posture_weight < 0.0
        or not np.isfinite(args.joint_velocity_envelope_weight)
        or args.joint_velocity_envelope_weight < 0.0
        or not np.isfinite(args.joint_velocity_envelope_activation_fraction)
        or not 0.0 <= args.joint_velocity_envelope_activation_fraction < 1.0
        or not np.isfinite(args.joint_velocity_envelope_frequency_hz)
        or args.joint_velocity_envelope_frequency_hz <= 0.0
        or args.joint_velocity_envelope_phase_transition_ticks < 0
        or not np.isfinite(args.capture_landing_activation_margin)
        or not np.isfinite(args.capture_landing_full_scale_margin)
        or args.capture_landing_full_scale_margin
        >= args.capture_landing_activation_margin
        or not np.isfinite(args.capture_landing_maximum_offset)
        or args.capture_landing_maximum_offset < 0.0
        or not np.isfinite(args.capture_landing_maximum_root_reach)
        or args.capture_landing_maximum_root_reach <= 0.0
        or not np.isfinite(args.capture_landing_maximum_anchor_speed)
        or args.capture_landing_maximum_anchor_speed <= 0.0
        or args.capture_landing_freeze_ticks < 0
        or not np.isfinite(args.touchdown_phase_minimum_rate)
        or not 0.0 <= args.touchdown_phase_minimum_rate <= 1.0
        or not np.isfinite(args.touchdown_phase_guard_time_seconds)
        or args.touchdown_phase_guard_time_seconds < 0.0
        or args.touchdown_phase_release_ticks < 0
        or args.touchdown_phase_engagement_ticks < 0
        or not np.isfinite(args.balance_phase_hold_margin)
        or not np.isfinite(args.balance_phase_full_rate_margin)
        or args.balance_phase_hold_margin
        >= args.balance_phase_full_rate_margin
        or not np.isfinite(args.centroidal_angular_momentum_weight)
        or args.centroidal_angular_momentum_weight < 0.0
        or not np.isfinite(args.centroidal_angular_momentum_frequency_hz)
        or args.centroidal_angular_momentum_frequency_hz <= 0.0
    ):
        raise SystemExit("CoM, centroidal-momentum, and posture settings are invalid")
    if args.center_of_mass_controller == "dcm-zmp" and (
        args.center_of_mass_task_weight <= 0.0
        or args.contact_patch_half_length <= 0.0
        or args.contact_patch_half_width <= 0.0
    ):
        raise SystemExit(
            "dcm-zmp requires a positive CoM task weight and finite support patch"
        )
    if args.capture_landing_retarget and (
        args.center_of_mass_controller != "dcm-zmp" or args.precontact_ticks == 0
    ):
        raise SystemExit(
            "capture landing retargeting requires dcm-zmp and a positive precontact horizon"
        )
    if args.touchdown_phase_retiming and args.precontact_ticks == 0:
        raise SystemExit(
            "touchdown phase retiming requires a positive precontact horizon"
        )
    if (
        args.balance_phase_retiming
        and args.center_of_mass_controller != "dcm-zmp"
    ):
        raise SystemExit("balance phase retiming requires dcm-zmp")
    if args.reference_inputs and (
        args.first_liftoff_hold_ticks != 0
        or args.first_liftoff_leg_phase_gate_ticks != 0
        or args.touchdown_blend_ticks != 0
        or args.maximum_root_to_foot_reach != 0.0
        or args.root_reference != "mocap"
        or args.center_of_mass_reference != "rooted"
        or args.center_of_mass_zero_reference_derivatives
        or args.capture_landing_retarget
        or args.touchdown_phase_retiming
        or args.balance_phase_retiming
    ):
        raise SystemExit(
            "standalone references forbid eval-side reconstruction, projection, or retiming"
        )
    if (
        args.support_preview_ticks < 0
        or not np.isfinite(args.support_margin)
        or args.support_margin < 0.0
        or not np.isfinite(args.support_reference_blend)
        or not 0.0 <= args.support_reference_blend <= 1.0
    ):
        raise SystemExit("support preview settings are invalid")
    if not np.isfinite(args.friction) or args.friction < 0.0:
        raise SystemExit("--friction must be finite and nonnegative")
    if (
        not np.isfinite(args.maximum_acceleration)
        or args.maximum_acceleration <= 0.0
        or not np.isfinite(args.maximum_torque)
        or args.maximum_torque <= 0.0
        or not np.isfinite(args.maximum_normal_force_multiple)
        or args.maximum_normal_force_multiple <= 0.0
        or not np.isfinite(args.root_horizontal_task_weight)
        or args.root_horizontal_task_weight < 0.0
        or not np.isfinite(args.root_angular_task_weight)
        or args.root_angular_task_weight < 0.0
        or not np.isfinite(args.root_height_task_weight)
        or args.root_height_task_weight < 0.0
        or not np.isfinite(args.root_frequency_hz)
        or args.root_frequency_hz <= 0.0
        or not np.isfinite(args.point_frequency_hz)
        or args.point_frequency_hz <= 0.0
        or not np.isfinite(args.joint_posture_weight)
        or args.joint_posture_weight < 0.0
    ):
        raise SystemExit("acceleration, torque, and normal-force limits must be positive")
    patch_parameters = np.asarray(
        [
            args.contact_patch_center_x,
            args.contact_patch_half_length,
            args.contact_patch_half_width,
            args.contact_patch_z,
        ],
        dtype=np.float64,
    )
    if (
        not np.all(np.isfinite(patch_parameters))
        or args.contact_patch_half_length < 0.0
        or args.contact_patch_half_width < 0.0
        or not np.isfinite(args.minimum_contact_cop_margin)
        or args.minimum_contact_cop_margin < 0.0
        or not np.isfinite(args.foot_task_weight)
        or not np.isfinite(args.hand_task_weight)
        or args.foot_task_weight < 0.0
        or args.hand_task_weight < 0.0
    ):
        raise SystemExit(
            "contact-patch dimensions and task weights must be finite and nonnegative"
        )
    if args.minimum_contact_cop_margin > 0.0 and (
        args.minimum_contact_cop_margin >= args.contact_patch_half_length
        or args.minimum_contact_cop_margin >= args.contact_patch_half_width
    ):
        raise SystemExit(
            "minimum contact CoP margin must be smaller than both patch half extents"
        )
    import bonesaw

    model = pathlib.Path(args.model)
    cmu_cache = pathlib.Path(args.cmu_cache)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    query = bonesaw.ControllerSession(str(model))
    names = list(query.frame_names)
    joint_names = list(query.joint_names)
    frame_names = (
        args.left_foot_frame,
        args.right_foot_frame,
        args.left_hand_frame,
        args.right_hand_frame,
    )
    missing_frames = [name for name in frame_names if name not in names]
    if missing_frames:
        raise SystemExit(f"model is missing requested frames: {missing_frames}")
    q = standing_posture(joint_names)
    protected_posture_patterns = [
        "waist",
        "shoulder",
        "elbow",
        "wrist",
    ]
    if args.protected_posture_include_leg_yaw:
        protected_posture_patterns.append("hip_yaw")
    upper_body_coordinates = np.asarray(
        [
            index
            for index, name in enumerate(joint_names)
            if any(part in name for part in protected_posture_patterns)
        ],
        dtype=np.int64,
    )
    origins = np.empty((query.bodies, 3), dtype=np.float64)
    query.frame_positions(q, origins)
    frame_ids = np.asarray([names.index(name) for name in frame_names], dtype=np.int64)
    foot_z = origins[frame_ids[:2], 2]
    root_translation = np.array(
        [0.0, 0.0, -float(np.min(foot_z)) - args.contact_patch_z]
    )
    world_origins = origins[frame_ids] + root_translation[None, :]
    standalone_reference: StandaloneReference | None = None
    if args.reference_inputs:
        standalone_reference = load_standalone_reference(
            pathlib.Path(args.reference_inputs),
            world_origins,
            root_translation,
            args.ticks,
        )
        walk = standalone_reference.walk
    elif args.motion_profile == "synthetic-step":
        walk = synthetic_g1_step(
            world_origins,
            root_translation,
            args.ticks,
            step_length=args.synthetic_step_length,
            step_clearance=args.synthetic_step_clearance,
            root_transfer_scale=args.synthetic_root_transfer_scale,
            swing_duration_ticks=args.synthetic_swing_ticks,
        )
    else:
        walk = retarget_subject_37_walk_floating(
            cmu_cache / "37.asf",
            cmu_cache / "37_01.amc",
            world_origins,
            root_translation,
            args.ticks + 512,
            DT,
        )
        walk = rebase_to_double_support(
            walk,
            root_translation,
            world_origins,
            args.ticks,
            startup_balance_ticks=args.startup_balance_ticks,
            startup_ramp_ticks=args.startup_ramp_ticks,
            forward_motion_scale=args.forward_motion_scale,
            lateral_motion_scale=args.lateral_motion_scale,
            cadence_multiplier=args.cadence_multiplier,
        )
    walk = insert_first_liftoff_hold(walk, args.first_liftoff_hold_ticks)
    target_positions, target_velocities, target_accelerations, contacts = (
        anchored_contact_targets(
            walk.targets,
            walk.stance,
            world_origins,
            touchdown_blend_ticks=args.touchdown_blend_ticks,
        )
    )
    if args.maximum_root_to_foot_reach > 0.0:
        walk, target_positions = project_touchdowns_to_root_reach(
            walk, target_positions, args.maximum_root_to_foot_reach
        )
        target_velocities, target_accelerations = finite_difference(target_positions)
        for foot in range(2):
            selected = walk.stance[:, foot].astype(bool)
            target_velocities[selected, foot] = 0.0
            target_accelerations[selected, foot] = 0.0
    center_of_mass_at_root = np.empty(3, dtype=np.float64)
    query.center_of_mass(q, center_of_mass_at_root)
    if args.root_reference == "support-preview":
        support_center_of_mass, _, _ = center_of_mass_reference(
            walk.root_targets,
            center_of_mass_at_root,
            target_positions,
            walk.stance,
            mode="support-preview",
            patch_center_x=args.contact_patch_center_x,
            patch_half_length=args.contact_patch_half_length,
            patch_half_width=args.contact_patch_half_width,
            patch_z=args.contact_patch_z,
            preview_ticks=args.support_preview_ticks,
            margin=args.support_margin,
            blend=args.support_reference_blend,
        )
        support_root_targets = walk.root_targets.copy()
        support_root_targets[:, :2] = (
            support_center_of_mass[:, :2] - center_of_mass_at_root[None, :2]
        )
        walk = dataclasses.replace(
            walk,
            root_targets=support_root_targets,
            metadata={**walk.metadata, "root_reference": args.root_reference},
        )
    else:
        walk = dataclasses.replace(
            walk,
            metadata={**walk.metadata, "root_reference": args.root_reference},
        )
    root_velocities, root_accelerations = finite_difference(walk.root_targets)
    (
        center_of_mass_targets,
        center_of_mass_velocities,
        center_of_mass_accelerations,
    ) = center_of_mass_reference(
        walk.root_targets,
        center_of_mass_at_root,
        target_positions,
        walk.stance,
        mode=args.center_of_mass_reference,
        patch_center_x=args.contact_patch_center_x,
        patch_half_length=args.contact_patch_half_length,
        patch_half_width=args.contact_patch_half_width,
        patch_z=args.contact_patch_z,
        preview_ticks=args.support_preview_ticks,
        margin=args.support_margin,
        blend=args.support_reference_blend,
    )
    if args.center_of_mass_zero_reference_derivatives:
        center_of_mass_velocities.fill(0.0)
        center_of_mass_accelerations.fill(0.0)
    (
        walk,
        target_positions,
        target_velocities,
        target_accelerations,
        contacts,
    ) = gate_first_liftoff_leg_phase(
        walk,
        target_positions,
        contacts,
        args.first_liftoff_leg_phase_gate_ticks,
    )
    if standalone_reference is not None:
        # Downstream admission is byte-semantic: every authored root, CoM,
        # foot, and contact array comes from the already-gated artifact.
        walk = standalone_reference.walk
        root_velocities = standalone_reference.root_velocities
        root_accelerations = standalone_reference.root_accelerations
        center_of_mass_targets = standalone_reference.center_of_mass_targets
        center_of_mass_velocities = standalone_reference.center_of_mass_velocities
        center_of_mass_accelerations = (
            standalone_reference.center_of_mass_accelerations
        )
        target_positions = standalone_reference.target_positions
        target_velocities = standalone_reference.target_velocities
        target_accelerations = standalone_reference.target_accelerations
        contacts = standalone_reference.contacts
    target_active = np.ones((args.ticks, len(frame_ids)), dtype=np.uint8)
    priorities = np.asarray(
        [args.foot_task_priority, args.foot_task_priority, 4, 4],
        dtype=np.uint8,
    )
    weights = np.asarray(
        [
            args.foot_task_weight,
            args.foot_task_weight,
            args.hand_task_weight,
            args.hand_task_weight,
        ],
        dtype=np.float64,
    )
    contact_points_per_foot = (
        4
        if args.contact_patch_half_length > 0.0
        or args.contact_patch_half_width > 0.0
        else 1
    )

    session = bonesaw.FloatingWbcSession(
        str(model),
        maximum_contacts=2 * contact_points_per_foot,
        friction_coefficient=args.friction,
        maximum_acceleration=args.maximum_acceleration,
        maximum_torque=args.maximum_torque,
        maximum_normal_force_multiple=args.maximum_normal_force_multiple,
        maximum_feasibility_iterations=args.maximum_feasibility_iterations,
        maximum_feasibility_projection_sweeps=(
            args.maximum_feasibility_projection_sweeps
        ),
        feasibility_projection_continuation_violation_threshold=(
            args.feasibility_projection_continuation_violation_threshold
        ),
        joint_limit_braking=args.joint_limit_braking,
        root_frequency_hz=args.root_frequency_hz,
        root_angular_task_weight=args.root_angular_task_weight,
        root_height_task_weight=args.root_height_task_weight,
        root_horizontal_task_weight=args.root_horizontal_task_weight,
        root_horizontal_task_priority=args.root_horizontal_task_priority,
        point_frequency_hz=args.point_frequency_hz,
        joint_posture_weight=args.joint_posture_weight,
        joint_posture_priority=args.joint_posture_priority,
        center_of_mass_task_weight=args.center_of_mass_task_weight,
        center_of_mass_task_priority=args.center_of_mass_task_priority,
        center_of_mass_frequency_hz=args.center_of_mass_frequency_hz,
        dcm_balance_enabled=args.center_of_mass_controller == "dcm-zmp",
        dcm_feedback_gain_per_second=args.dcm_feedback_gain_per_second,
        dcm_support_margin_m=args.dcm_support_margin,
        dcm_maximum_horizontal_acceleration_mps2=(
            args.dcm_maximum_horizontal_acceleration
        ),
        protected_joint_posture_weight=args.upper_body_posture_weight,
        protected_joint_posture_priority=args.upper_body_posture_priority,
        joint_velocity_envelope_weight=args.joint_velocity_envelope_weight,
        joint_velocity_envelope_priority=args.joint_velocity_envelope_priority,
        joint_velocity_envelope_activation_fraction=(
            args.joint_velocity_envelope_activation_fraction
        ),
        joint_velocity_envelope_frequency_hz=(
            args.joint_velocity_envelope_frequency_hz
        ),
        joint_velocity_envelope_multi_support_only=(
            args.joint_velocity_envelope_phase_policy != "always"
        ),
        joint_velocity_envelope_phase_transition_ticks=(
            args.joint_velocity_envelope_phase_transition_ticks
        ),
        balance_feedback_authority_enabled=(
            args.joint_velocity_envelope_phase_policy == "feedback"
        ),
        capture_landing_retarget_enabled=args.capture_landing_retarget,
        capture_landing_activation_margin_m=(
            args.capture_landing_activation_margin
        ),
        capture_landing_full_scale_margin_m=(
            args.capture_landing_full_scale_margin
        ),
        capture_landing_maximum_authored_offset_m=(
            args.capture_landing_maximum_offset
        ),
        capture_landing_maximum_root_reach_m=(
            args.capture_landing_maximum_root_reach
        ),
        capture_landing_maximum_anchor_speed_mps=(
            args.capture_landing_maximum_anchor_speed
        ),
        capture_landing_freeze_ticks=args.capture_landing_freeze_ticks,
        touchdown_phase_retiming_enabled=args.touchdown_phase_retiming,
        touchdown_phase_minimum_rate=args.touchdown_phase_minimum_rate,
        touchdown_phase_guard_time_seconds=(
            args.touchdown_phase_guard_time_seconds
        ),
        touchdown_phase_release_ticks=args.touchdown_phase_release_ticks,
        touchdown_phase_engagement_ticks=(
            args.touchdown_phase_engagement_ticks
        ),
        balance_phase_retiming_enabled=args.balance_phase_retiming,
        balance_phase_hold_margin_m=args.balance_phase_hold_margin,
        balance_phase_full_rate_margin_m=(
            args.balance_phase_full_rate_margin
        ),
        centroidal_angular_momentum_weight=(
            args.centroidal_angular_momentum_weight
        ),
        centroidal_angular_momentum_priority=(
            args.centroidal_angular_momentum_priority
        ),
        centroidal_angular_momentum_frequency_hz=(
            args.centroidal_angular_momentum_frequency_hz
        ),
        precontact_ticks=args.precontact_ticks,
        precontact_maximum_acceleration=args.precontact_maximum_acceleration,
        material_touchdown_task=args.material_touchdown_task,
        contact_patch_center_x=args.contact_patch_center_x,
        contact_patch_half_length=args.contact_patch_half_length,
        contact_patch_half_width=args.contact_patch_half_width,
        contact_patch_z=args.contact_patch_z,
        minimum_contact_cop_margin_m=args.minimum_contact_cop_margin,
    )
    session.reset(q, np.zeros_like(q), root_translation)
    root_out = np.empty((args.ticks, 3), dtype=np.float64)
    root_quaternion = np.empty((args.ticks, 4), dtype=np.float64)
    center_of_mass_tracked = np.empty((args.ticks, 3), dtype=np.float64)
    q_out = np.empty((args.ticks, len(q)), dtype=np.float64)
    v_out = np.empty_like(q_out)
    tracked = np.empty_like(target_positions)
    contact_force = np.empty(
        (args.ticks, 2 * contact_points_per_foot), dtype=np.float64
    )
    task_rms = np.empty(
        (args.ticks, session.task_diagnostic_capacity), dtype=np.float64
    )
    dynamics_residual = np.empty(args.ticks, dtype=np.float64)
    contact_residual = np.empty(args.ticks, dtype=np.float64)
    minimum_support_margin = np.empty(args.ticks, dtype=np.float64)
    step_ns = np.empty(args.ticks, dtype=np.uint64)
    status = np.empty(args.ticks, dtype=np.uint8)
    support_phase = np.empty((args.ticks, len(frame_ids)), dtype=np.uint8)
    support_phase_ticks = np.empty((args.ticks, len(frame_ids)), dtype=np.uint16)
    support_tangential_speed = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    support_touchdown_position_error = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    support_touchdown_normal_speed = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    task_pseudoinverse_calls = np.empty(args.ticks, dtype=np.uint16)
    clipped_steps = np.empty(args.ticks, dtype=np.uint16)
    task_pseudoinverse_calls_by_level = np.empty(
        (args.ticks, len(PRIORITY_NAMES)), dtype=np.uint16
    )
    clipped_steps_by_level = np.empty_like(task_pseudoinverse_calls_by_level)
    task_jacobi_sweeps = np.empty(args.ticks, dtype=np.uint16)
    task_jacobi_sweeps_by_level = np.empty_like(
        task_pseudoinverse_calls_by_level
    )
    feasibility_projection_sweeps = np.empty(args.ticks, dtype=np.uint16)
    feasibility_halfspace_projections = np.empty(args.ticks, dtype=np.uint32)
    feasibility_polish_iterations = np.empty(args.ticks, dtype=np.uint16)
    feasibility_polish_pseudoinverse_calls = np.empty(
        args.ticks, dtype=np.uint16
    )
    feasibility_polish_jacobi_sweeps = np.empty(args.ticks, dtype=np.uint16)
    center_of_mass_velocity = np.empty((args.ticks, 3), dtype=np.float64)
    dcm = np.empty((args.ticks, 3), dtype=np.float64)
    target_dcm = np.empty((args.ticks, 3), dtype=np.float64)
    virtual_zmp = np.empty((args.ticks, 3), dtype=np.float64)
    clipped_zmp = np.empty((args.ticks, 3), dtype=np.float64)
    center_of_mass_command = np.empty((args.ticks, 3), dtype=np.float64)
    dcm_natural_frequency = np.empty(args.ticks, dtype=np.float64)
    dcm_measured_height = np.empty(args.ticks, dtype=np.float64)
    dcm_height_clamped = np.empty(args.ticks, dtype=np.uint8)
    dcm_zmp_clipped = np.empty(args.ticks, dtype=np.uint8)
    dcm_support_vertices = np.empty(args.ticks, dtype=np.uint8)
    dcm_support_margin = np.empty(args.ticks, dtype=np.float64)
    landing_retarget_anchor = np.empty(
        (args.ticks, len(frame_ids), 3), dtype=np.float64
    )
    landing_retarget_capture_scale = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    landing_retarget_offset = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    landing_retarget_reach = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.float64
    )
    landing_retarget_flags = np.empty(
        (args.ticks, len(frame_ids)), dtype=np.uint8
    )
    contact_phase_authority = np.empty(args.ticks, dtype=np.uint8)
    joint_velocity_envelope_target_scale = np.empty(
        args.ticks, dtype=np.float64
    )
    joint_velocity_envelope_scale = np.empty(args.ticks, dtype=np.float64)
    joint_velocity_envelope_active_coordinates = np.empty(
        args.ticks, dtype=np.uint8
    )
    effective_root_targets = np.empty((args.ticks, 3), dtype=np.float64)
    effective_center_of_mass_targets = np.empty(
        (args.ticks, 3), dtype=np.float64
    )
    effective_target_positions = np.empty_like(target_positions)
    effective_contact_active = np.empty_like(contacts)
    reference_phase = np.empty(args.ticks, dtype=np.float64)
    reference_phase_target_rate = np.empty(args.ticks, dtype=np.float64)
    reference_phase_rate = np.empty(args.ticks, dtype=np.float64)
    reference_phase_acceleration = np.empty(args.ticks, dtype=np.float64)
    reference_phase_required_time = np.empty(args.ticks, dtype=np.float64)
    reference_phase_flags = np.empty(args.ticks, dtype=np.uint8)
    gc.collect()
    rss_before = current_rss_bytes()
    usage_before = usage_snapshot()
    gc_before = gc_snapshot()
    tracemalloc.start()
    wall_before_ns = time.perf_counter_ns()
    process_before_ns = time.process_time_ns()
    thread_before_ns = time.thread_time_ns()
    session.run_trace(
        DT,
        walk.root_targets,
        root_velocities,
        root_accelerations,
        center_of_mass_targets,
        center_of_mass_velocities,
        center_of_mass_accelerations,
        frame_ids,
        target_positions,
        target_velocities,
        target_accelerations,
        target_active,
        contacts,
        priorities,
        weights,
        q,
        upper_body_coordinates,
        root_out,
        root_quaternion,
        center_of_mass_tracked,
        q_out,
        v_out,
        tracked,
        contact_force,
        task_rms,
        dynamics_residual,
        contact_residual,
        minimum_support_margin,
        step_ns,
        status,
        support_phase,
        support_phase_ticks,
        support_tangential_speed,
        support_touchdown_position_error,
        support_touchdown_normal_speed,
        task_pseudoinverse_calls,
        clipped_steps,
        task_pseudoinverse_calls_by_level,
        clipped_steps_by_level,
        task_jacobi_sweeps,
        task_jacobi_sweeps_by_level,
        feasibility_projection_sweeps,
        feasibility_halfspace_projections,
        feasibility_polish_iterations,
        feasibility_polish_pseudoinverse_calls,
        feasibility_polish_jacobi_sweeps,
        center_of_mass_velocity,
        dcm,
        target_dcm,
        virtual_zmp,
        clipped_zmp,
        center_of_mass_command,
        dcm_natural_frequency,
        dcm_measured_height,
        dcm_height_clamped,
        dcm_zmp_clipped,
        dcm_support_vertices,
        dcm_support_margin,
        landing_retarget_anchor,
        landing_retarget_capture_scale,
        landing_retarget_offset,
        landing_retarget_reach,
        landing_retarget_flags,
        contact_phase_authority,
        joint_velocity_envelope_target_scale,
        joint_velocity_envelope_scale,
        joint_velocity_envelope_active_coordinates,
        effective_root_targets,
        effective_center_of_mass_targets,
        effective_target_positions,
        effective_contact_active,
        reference_phase,
        reference_phase_target_rate,
        reference_phase_rate,
        reference_phase_acceleration,
        reference_phase_required_time,
        reference_phase_flags,
    )
    thread_cpu_ns = time.thread_time_ns() - thread_before_ns
    process_cpu_ns = time.process_time_ns() - process_before_ns
    wall_ns = time.perf_counter_ns() - wall_before_ns
    traced_current, traced_peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    gc_after = gc_snapshot()
    usage_after = usage_snapshot()
    rss_after = current_rss_bytes()
    runtime = runtime_measurement(
        wall_ns=wall_ns,
        process_cpu_ns=process_cpu_ns,
        thread_cpu_ns=thread_cpu_ns,
        rss_before=rss_before,
        rss_after=rss_after,
        usage_before=usage_before,
        usage_after=usage_after,
        gc_before=gc_before,
        gc_after=gc_after,
        traced_current=traced_current,
        traced_peak=traced_peak,
    )
    effective_reference_indices = np.floor(reference_phase).astype(np.int64)
    effective_cadence_scale = walk.cadence_scale[effective_reference_indices]
    metrics = summarize(
        walk,
        effective_target_positions,
        effective_root_targets,
        effective_contact_active,
        effective_cadence_scale,
        root_out,
        effective_center_of_mass_targets,
        center_of_mass_tracked,
        tracked,
        root_quaternion,
        v_out,
        contact_force,
        task_rms,
        dynamics_residual,
        contact_residual,
        minimum_support_margin,
        args.minimum_contact_cop_margin,
        step_ns,
        status,
        support_phase,
        support_phase_ticks,
        support_tangential_speed,
        support_touchdown_position_error,
        support_touchdown_normal_speed,
        session.maximum_touchdown_position_error_m,
        session.maximum_touchdown_tangential_speed_mps,
        session.maximum_touchdown_normal_speed_mps,
        task_pseudoinverse_calls,
        clipped_steps,
        task_pseudoinverse_calls_by_level,
        clipped_steps_by_level,
        task_jacobi_sweeps,
        task_jacobi_sweeps_by_level,
        feasibility_projection_sweeps,
        feasibility_halfspace_projections,
        feasibility_polish_iterations,
        feasibility_polish_pseudoinverse_calls,
        feasibility_polish_jacobi_sweeps,
        dcm,
        target_dcm,
        virtual_zmp,
        clipped_zmp,
        center_of_mass_command,
        dcm_natural_frequency,
        dcm_measured_height,
        dcm_height_clamped,
        dcm_zmp_clipped,
        dcm_support_vertices,
        dcm_support_margin,
        landing_retarget_anchor,
        landing_retarget_capture_scale,
        landing_retarget_offset,
        landing_retarget_reach,
        landing_retarget_flags,
        contact_phase_authority,
        joint_velocity_envelope_target_scale,
        joint_velocity_envelope_scale,
        joint_velocity_envelope_active_coordinates,
        reference_phase,
        reference_phase_target_rate,
        reference_phase_rate,
        reference_phase_acceleration,
        reference_phase_required_time,
        reference_phase_flags,
        args.touchdown_phase_retiming,
        args.balance_phase_retiming,
        args.hand_task_weight,
        runtime,
    )
    report_metadata = {
        **walk.metadata,
        "motion_profile": walk.metadata.get("motion_profile", args.motion_profile),
        "target_model": str(model),
        "target_frames": list(frame_names),
        "contact_patch_points_per_foot": contact_points_per_foot,
        "contact_patch_center_x_m": args.contact_patch_center_x,
        "contact_patch_half_length_m": args.contact_patch_half_length,
        "contact_patch_half_width_m": args.contact_patch_half_width,
        "contact_patch_z_m": args.contact_patch_z,
        "minimum_contact_cop_margin_m": args.minimum_contact_cop_margin,
        "foot_task_weight": args.foot_task_weight,
        "foot_task_priority": PRIORITY_NAMES[args.foot_task_priority],
        "hand_task_weight": args.hand_task_weight,
        "root_frequency_hz": args.root_frequency_hz,
        "root_angular_task_weight": args.root_angular_task_weight,
        "root_height_task_weight": args.root_height_task_weight,
        "root_horizontal_task_weight": args.root_horizontal_task_weight,
        "root_horizontal_task_priority": PRIORITY_NAMES[
            args.root_horizontal_task_priority
        ],
        "point_frequency_hz": args.point_frequency_hz,
        "joint_posture_weight": args.joint_posture_weight,
        "joint_posture_priority": PRIORITY_NAMES[args.joint_posture_priority],
        "center_of_mass_task_weight": args.center_of_mass_task_weight,
        "center_of_mass_task_priority": PRIORITY_NAMES[
            args.center_of_mass_task_priority
        ],
        "center_of_mass_frequency_hz": args.center_of_mass_frequency_hz,
        "center_of_mass_controller": args.center_of_mass_controller,
        "center_of_mass_reference": walk.metadata.get(
            "center_of_mass_reference", args.center_of_mass_reference
        ),
        "center_of_mass_reference_semantics": (
            "DCM" if args.center_of_mass_controller == "dcm-zmp" else "CoM"
        ),
        "dcm_feedback_gain_per_second": args.dcm_feedback_gain_per_second,
        "dcm_support_margin_m": args.dcm_support_margin,
        "dcm_maximum_horizontal_acceleration_mps2": (
            args.dcm_maximum_horizontal_acceleration
        ),
        "center_of_mass_zero_reference_derivatives": (
            args.center_of_mass_zero_reference_derivatives
        ),
        "upper_body_posture_weight": args.upper_body_posture_weight,
        "upper_body_posture_priority": PRIORITY_NAMES[
            args.upper_body_posture_priority
        ],
        "upper_body_posture_joints": [
            joint_names[index] for index in upper_body_coordinates
        ],
        "joint_velocity_envelope_weight": args.joint_velocity_envelope_weight,
        "joint_velocity_envelope_priority": PRIORITY_NAMES[
            args.joint_velocity_envelope_priority
        ],
        "joint_velocity_envelope_activation_fraction": (
            args.joint_velocity_envelope_activation_fraction
        ),
        "joint_velocity_envelope_frequency_hz": (
            args.joint_velocity_envelope_frequency_hz
        ),
        "joint_velocity_envelope_phase_policy": (
            args.joint_velocity_envelope_phase_policy
        ),
        "joint_velocity_envelope_phase_transition_ticks": (
            args.joint_velocity_envelope_phase_transition_ticks
        ),
        "protected_posture_include_leg_yaw": (
            args.protected_posture_include_leg_yaw
        ),
        "centroidal_angular_momentum_weight": (
            args.centroidal_angular_momentum_weight
        ),
        "centroidal_angular_momentum_priority": PRIORITY_NAMES[
            args.centroidal_angular_momentum_priority
        ],
        "centroidal_angular_momentum_frequency_hz": (
            args.centroidal_angular_momentum_frequency_hz
        ),
        "center_of_mass_reference": walk.metadata.get(
            "center_of_mass_reference", args.center_of_mass_reference
        ),
        "root_reference": walk.metadata.get("root_reference", args.root_reference),
        "support_preview_ticks": args.support_preview_ticks,
        "support_preview_seconds": args.support_preview_ticks * DT,
        "support_margin_m": args.support_margin,
        "support_reference_blend": args.support_reference_blend,
        "precontact_ticks": args.precontact_ticks,
        "precontact_seconds": args.precontact_ticks * DT,
        "precontact_maximum_acceleration_m_s2": (
            args.precontact_maximum_acceleration
        ),
        "capture_landing_retarget_enabled": args.capture_landing_retarget,
        "capture_landing_activation_margin_m": (
            args.capture_landing_activation_margin
        ),
        "capture_landing_full_scale_margin_m": (
            args.capture_landing_full_scale_margin
        ),
        "capture_landing_maximum_offset_m": (
            args.capture_landing_maximum_offset
        ),
        "capture_landing_maximum_root_reach_m": (
            args.capture_landing_maximum_root_reach
        ),
        "capture_landing_maximum_anchor_speed_mps": (
            args.capture_landing_maximum_anchor_speed
        ),
        "capture_landing_freeze_ticks": args.capture_landing_freeze_ticks,
        "touchdown_phase_retiming_enabled": args.touchdown_phase_retiming,
        "touchdown_phase_minimum_rate": args.touchdown_phase_minimum_rate,
        "touchdown_phase_guard_time_seconds": (
            args.touchdown_phase_guard_time_seconds
        ),
        "touchdown_phase_release_ticks": args.touchdown_phase_release_ticks,
        "touchdown_phase_engagement_ticks": (
            args.touchdown_phase_engagement_ticks
        ),
        "balance_phase_retiming_enabled": args.balance_phase_retiming,
        "balance_phase_hold_margin_m": args.balance_phase_hold_margin,
        "balance_phase_full_rate_margin_m": (
            args.balance_phase_full_rate_margin
        ),
        "maximum_touchdown_position_error_m": (
            session.maximum_touchdown_position_error_m
        ),
        "maximum_touchdown_tangential_speed_mps": (
            session.maximum_touchdown_tangential_speed_mps
        ),
        "maximum_touchdown_normal_speed_mps": (
            session.maximum_touchdown_normal_speed_mps
        ),
        "maximum_feasibility_iterations": args.maximum_feasibility_iterations,
        "maximum_feasibility_projection_sweeps": (
            args.maximum_feasibility_projection_sweeps
        ),
        "feasibility_projection_continuation_violation_threshold": (
            args.feasibility_projection_continuation_violation_threshold
        ),
        "joint_limit_braking": args.joint_limit_braking,
        "touchdown_blend_ticks": args.touchdown_blend_ticks,
        "touchdown_blend_seconds": args.touchdown_blend_ticks * DT,
        "material_touchdown_task": args.material_touchdown_task,
    }
    document = {
        "schema": 2,
        "environment": {
            "cpu": platform.processor(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "bonesaw": getattr(bonesaw, "__version__", "unknown"),
        },
        "model": str(model),
        "friction_coefficient": args.friction,
        "maximum_acceleration": args.maximum_acceleration,
        "maximum_torque": args.maximum_torque,
        "maximum_normal_force_multiple": args.maximum_normal_force_multiple,
        "maximum_feasibility_iterations": args.maximum_feasibility_iterations,
        "maximum_feasibility_projection_sweeps": (
            args.maximum_feasibility_projection_sweeps
        ),
        "feasibility_projection_continuation_violation_threshold": (
            args.feasibility_projection_continuation_violation_threshold
        ),
        "center_of_mass_task_weight": args.center_of_mass_task_weight,
        "motion": report_metadata,
        "metrics": metrics,
    }
    (output / "floating-walk-metrics.json").write_text(
        json.dumps(document, indent=2) + "\n"
    )
    np.savez_compressed(
        output / "floating-walk-raw.npz",
        root_targets=walk.root_targets,
        effective_root_targets=effective_root_targets,
        root_target_velocities=root_velocities,
        root_target_accelerations=root_accelerations,
        center_of_mass_targets=center_of_mass_targets,
        effective_center_of_mass_targets=effective_center_of_mass_targets,
        center_of_mass_target_velocities=center_of_mass_velocities,
        center_of_mass_target_accelerations=center_of_mass_accelerations,
        center_of_mass_tracked=center_of_mass_tracked,
        center_of_mass_velocity=center_of_mass_velocity,
        center_of_mass_command=center_of_mass_command,
        dcm=dcm,
        target_dcm=target_dcm,
        virtual_zmp=virtual_zmp,
        clipped_zmp=clipped_zmp,
        dcm_natural_frequency=dcm_natural_frequency,
        dcm_measured_height=dcm_measured_height,
        dcm_height_clamped=dcm_height_clamped,
        dcm_zmp_clipped=dcm_zmp_clipped,
        dcm_support_vertices=dcm_support_vertices,
        dcm_support_margin=dcm_support_margin,
        minimum_support_margin=minimum_support_margin,
        landing_retarget_anchor=landing_retarget_anchor,
        landing_retarget_capture_scale=landing_retarget_capture_scale,
        landing_retarget_offset=landing_retarget_offset,
        landing_retarget_reach=landing_retarget_reach,
        landing_retarget_flags=landing_retarget_flags,
        contact_phase_authority=contact_phase_authority,
        joint_velocity_envelope_target_scale=(
            joint_velocity_envelope_target_scale
        ),
        joint_velocity_envelope_scale=joint_velocity_envelope_scale,
        joint_velocity_envelope_active_coordinates=(
            joint_velocity_envelope_active_coordinates
        ),
        root_tracked=root_out,
        root_quaternion_wxyz=root_quaternion,
        target_positions=target_positions,
        effective_target_positions=effective_target_positions,
        target_velocities=target_velocities,
        target_accelerations=target_accelerations,
        reference_stance=walk.stance,
        contact_active=contacts,
        effective_contact_active=effective_contact_active,
        cadence_scale=walk.cadence_scale,
        source_phase_frames=walk.source_phase_frames,
        reference_phase=reference_phase,
        reference_phase_target_rate=reference_phase_target_rate,
        reference_phase_rate=reference_phase_rate,
        reference_phase_acceleration=reference_phase_acceleration,
        reference_phase_required_time=reference_phase_required_time,
        reference_phase_flags=reference_phase_flags,
        tracked_positions=tracked,
        q=q_out,
        v=v_out,
        contact_normal_force=contact_force,
        task_rms=task_rms,
        dynamics_residual=dynamics_residual,
        contact_residual=contact_residual,
        step_ns=step_ns,
        status=status,
        support_phase=support_phase,
        support_phase_ticks=support_phase_ticks,
        support_tangential_speed=support_tangential_speed,
        support_touchdown_position_error=support_touchdown_position_error,
        support_touchdown_normal_speed=support_touchdown_normal_speed,
        task_pseudoinverse_calls=task_pseudoinverse_calls,
        clipped_steps=clipped_steps,
        task_pseudoinverse_calls_by_level=task_pseudoinverse_calls_by_level,
        clipped_steps_by_level=clipped_steps_by_level,
        task_jacobi_sweeps=task_jacobi_sweeps,
        task_jacobi_sweeps_by_level=task_jacobi_sweeps_by_level,
        feasibility_projection_sweeps=feasibility_projection_sweeps,
        feasibility_halfspace_projections=feasibility_halfspace_projections,
        feasibility_polish_iterations=feasibility_polish_iterations,
        feasibility_polish_pseudoinverse_calls=(
            feasibility_polish_pseudoinverse_calls
        ),
        feasibility_polish_jacobi_sweeps=feasibility_polish_jacobi_sweeps,
    )
    report = output / "FLOATING_WALK_CORPUS.md"
    report.write_text(render_report(metrics, report_metadata))
    print(report)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"floating walk corpus failed: {error}", file=sys.stderr)
        raise
