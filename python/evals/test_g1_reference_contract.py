#!/usr/bin/env python3
"""Unit tests for the policy-free walking-reference contract."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from g1_reference_contract import (
    convex_hull,
    score_reference,
    signed_polygon_margin,
    sole_vertices,
)


class SupportGeometryTest(unittest.TestCase):
    def test_signed_margin_is_positive_inside_and_negative_outside(self) -> None:
        hull = convex_hull(
            sole_vertices(np.array([[0.0, 0.0]]), half_length=0.1, half_width=0.05)
        )
        self.assertAlmostEqual(signed_polygon_margin(hull, np.array([0.0, 0.0])), 0.05)
        self.assertAlmostEqual(signed_polygon_margin(hull, np.array([0.0, 0.08])), -0.03)

    def test_double_support_hull_spans_both_soles(self) -> None:
        hull = convex_hull(
            sole_vertices(
                np.array([[0.0, -0.1], [0.0, 0.1]]),
                half_length=0.1,
                half_width=0.04,
            )
        )
        self.assertAlmostEqual(signed_polygon_margin(hull, np.array([0.0, 0.0])), 0.1)

    def test_standalone_reference_uses_only_authored_arrays(self) -> None:
        ticks = 4
        root = np.tile([0.0, 0.0, 0.8], (ticks, 1))
        com = np.tile([0.035, 0.0, 0.7], (ticks, 1))
        feet = np.zeros((ticks, 2, 3))
        feet[:, 0] = [0.0, 0.1, 0.035]
        feet[:, 1] = [0.0, -0.1, 0.035]
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            np.savez_compressed(
                directory / "reference-inputs.npz",
                root_targets=root,
                center_of_mass_targets=com,
                center_of_mass_target_velocities=np.zeros_like(com),
                center_of_mass_target_accelerations=np.zeros_like(com),
                target_positions=feet,
                target_velocities=np.zeros_like(feet),
                target_accelerations=np.zeros_like(feet),
                reference_stance=np.ones((ticks, 2), dtype=np.uint8),
            )
            (directory / "reference-metadata.json").write_text(
                json.dumps(
                    {
                        "implementation": "test",
                        "generator_family": "test",
                        "policy_or_physics_rollout": False,
                        "ik_or_wbc_solve": False,
                        "exact_repeat": True,
                        "runtime": {},
                        "motion": {
                            "contact_patch_center_x_m": 0.035,
                            "contact_patch_half_length_m": 0.085,
                            "contact_patch_half_width_m": 0.0275,
                            "contact_patch_z_m": -0.035,
                            "support_margin_m": 0.01,
                        },
                    }
                )
            )
            metrics, _ = score_reference(directory)
        self.assertTrue(metrics["pass"])
        self.assertEqual(metrics["flight_ticks"], 0)
        self.assertEqual(metrics["generator"]["generator_family"], "test")


if __name__ == "__main__":
    unittest.main()
