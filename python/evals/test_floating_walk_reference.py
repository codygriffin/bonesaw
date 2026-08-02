#!/usr/bin/env python3
"""Regression tests for causal bounded walking-reference construction."""

from __future__ import annotations

import json
import pathlib
import tempfile
import unittest

import numpy as np

from cmu_mocap import retarget_subject_37_walk_floating
from floating_walk_corpus import center_of_mass_reference, load_standalone_reference


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
CMU_SKELETON = REPOSITORY_ROOT / "benchmarks/cache/cmu-37/37.asf"
CMU_MOTION = REPOSITORY_ROOT / "benchmarks/cache/cmu-37/37_01.amc"


def reference(length: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    root = np.zeros((length, 3), dtype=np.float64)
    root[:, 2] = 0.8
    feet = np.zeros((length, 2, 3), dtype=np.float64)
    feet[:, 0, 1] = 0.1
    feet[:, 1, 1] = -0.1
    stance = np.ones((length, 2), dtype=np.uint8)
    stance[40:70, 0] = 0
    stance[85:, 1] = 0
    return root, feet, stance


def dcm_preview(
    root: np.ndarray, feet: np.ndarray, stance: np.ndarray, horizon: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return center_of_mass_reference(
        root,
        np.array([0.0, 0.0, 0.0]),
        feet,
        stance,
        mode="dcm-backward-preview",
        patch_center_x=0.035,
        patch_half_length=0.085,
        patch_half_width=0.0275,
        patch_z=0.0,
        preview_ticks=horizon,
        margin=0.01,
        blend=1.0,
    )


class FixedHorizonDcmPreviewTest(unittest.TestCase):
    def test_extending_trace_does_not_change_completed_preview_prefix(self) -> None:
        short = reference(100)
        long = reference(140)
        short_result = dcm_preview(*short, horizon=20)
        long_result = dcm_preview(*long, horizon=20)
        for short_array, long_array in zip(short_result, long_result, strict=True):
            np.testing.assert_array_equal(short_array[:78], long_array[:78])

    def test_future_support_changes_have_bounded_influence(self) -> None:
        root, feet, stance = reference(120)
        changed = stance.copy()
        changed[70:, 0] = 1
        changed[70:, 1] = 0
        baseline = dcm_preview(root, feet, stance, horizon=20)
        modified = dcm_preview(root, feet, changed, horizon=20)
        for baseline_array, modified_array in zip(baseline, modified, strict=True):
            np.testing.assert_array_equal(baseline_array[:48], modified_array[:48])


class StandaloneReferenceAdmissionTest(unittest.TestCase):
    def test_loader_preserves_every_authored_jet_and_contact_byte_semantics(self) -> None:
        ticks = 8
        root = np.tile([0.0, 0.0, 0.8], (ticks, 1))
        com = np.tile([0.035, 0.0, 0.7], (ticks, 1))
        com_velocity = np.zeros_like(com)
        com_acceleration = np.zeros_like(com)
        initial_positions = np.array(
            [
                [0.0, 0.1, 0.035],
                [0.0, -0.1, 0.035],
                [0.2, 0.25, 0.9],
                [0.2, -0.25, 0.9],
            ],
            dtype=np.float64,
        )
        feet = np.broadcast_to(initial_positions[:2], (ticks, 2, 3)).copy()
        foot_velocity = np.zeros_like(feet)
        foot_acceleration = np.zeros_like(feet)
        stance = np.ones((ticks, 2), dtype=np.uint8)
        stance[3:6, 0] = 0
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            artifact = directory / "reference-inputs.npz"
            np.savez_compressed(
                artifact,
                root_targets=root,
                center_of_mass_targets=com,
                center_of_mass_target_velocities=com_velocity,
                center_of_mass_target_accelerations=com_acceleration,
                target_positions=feet,
                target_velocities=foot_velocity,
                target_accelerations=foot_acceleration,
                reference_stance=stance,
            )
            (directory / "reference-metadata.json").write_text(
                json.dumps(
                    {
                        "implementation": "test Rust planner",
                        "exact_repeat": True,
                        "policy_or_physics_rollout": False,
                    }
                )
            )
            loaded = load_standalone_reference(
                artifact, initial_positions, root[0], ticks
            )
        np.testing.assert_array_equal(loaded.walk.root_targets, root)
        np.testing.assert_array_equal(loaded.center_of_mass_targets, com)
        np.testing.assert_array_equal(loaded.center_of_mass_velocities, com_velocity)
        np.testing.assert_array_equal(
            loaded.center_of_mass_accelerations, com_acceleration
        )
        np.testing.assert_array_equal(loaded.target_positions[:, :2], feet)
        np.testing.assert_array_equal(loaded.target_velocities[:, :2], foot_velocity)
        np.testing.assert_array_equal(
            loaded.target_accelerations[:, :2], foot_acceleration
        )
        np.testing.assert_array_equal(loaded.contacts[:, :2], stance)
        self.assertEqual(loaded.walk.metadata["motion_profile"], "standalone-reference")


@unittest.skipUnless(
    CMU_SKELETON.is_file() and CMU_MOTION.is_file(),
    "pinned CMU subject 37 corpus is not cached",
)
class FloatingMocapPrefixTest(unittest.TestCase):
    def test_extending_trace_does_not_recalibrate_lateral_root_prefix(self) -> None:
        target_origins = np.array(
            [
                [0.0, 0.12, 0.0],
                [0.0, -0.12, 0.0],
                [0.18, 0.25, 0.92],
                [0.18, -0.25, 0.92],
            ],
            dtype=np.float64,
        )
        target_root = np.array([0.0, 0.0, 0.80], dtype=np.float64)
        short = retarget_subject_37_walk_floating(
            CMU_SKELETON,
            CMU_MOTION,
            target_origins,
            target_root,
            600,
            0.001,
        )
        long = retarget_subject_37_walk_floating(
            CMU_SKELETON,
            CMU_MOTION,
            target_origins,
            target_root,
            800,
            0.001,
        )

        np.testing.assert_array_equal(short.root_targets, long.root_targets[:600])
        np.testing.assert_array_equal(short.targets, long.targets[:600])
        np.testing.assert_array_equal(short.stance, long.stance[:600])
        np.testing.assert_array_equal(short.cadence_scale, long.cadence_scale[:600])
        np.testing.assert_array_equal(
            short.source_phase_frames, long.source_phase_frames[:600]
        )
        self.assertEqual(
            short.metadata["floating_lateral_support_retarget"],
            long.metadata["floating_lateral_support_retarget"],
        )


if __name__ == "__main__":
    unittest.main()
