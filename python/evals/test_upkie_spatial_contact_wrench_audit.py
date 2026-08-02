from __future__ import annotations

import math
import unittest

import numpy as np

from upkie_spatial_contact_wrench_audit import (
    reconstruction_score,
    translate_moment_from_world_origin,
)


class SpatialContactWrenchAuditTests(unittest.TestCase):
    def test_moment_translation_preserves_force_line(self) -> None:
        point = np.asarray([[1.0, 0.0, 0.0]], np.float64)
        impulse = np.asarray([[0.0, 2.0, 0.0]], np.float64)
        moment_origin = np.asarray([[0.0, 0.0, 2.0]], np.float64)
        translated = translate_moment_from_world_origin(
            moment_origin, point, impulse
        )
        np.testing.assert_array_equal(translated, np.zeros((1, 3), np.float64))

    def test_reconstruction_score_keeps_coordinate_groups_separate(self) -> None:
        score = reconstruction_score(
            [
                np.asarray(
                    [3.0, 4.0, 0.0, 0.0, 0.0, 12.0, 5.0, 12.0],
                    np.float64,
                )
            ]
        )
        self.assertEqual(score["root_angular_norm"]["p50"], 5.0)
        self.assertEqual(score["root_linear_norm"]["p50"], 12.0)
        self.assertEqual(score["joint_norm"]["p50"], 13.0)
        self.assertEqual(score["norm"]["p50"], math.sqrt(338.0))


if __name__ == "__main__":
    unittest.main()
