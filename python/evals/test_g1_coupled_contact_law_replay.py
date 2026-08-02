from __future__ import annotations

import math
import unittest

import numpy as np

from g1_contact_law_momentum_holdout import CONTROL_DT
from g1_coupled_contact_law_replay import (
    REGULARIZATION_TIME_SCALE,
    contact_law_parameters,
    prospective_step_velocity,
)


class G1CoupledContactLawReplayTests(unittest.TestCase):
    def test_contact_law_mapping_is_continuous_and_stiffens_monotonically(self) -> None:
        hard = contact_law_parameters(0.001)
        soft = contact_law_parameters(0.030)
        self.assertGreater(hard[0], soft[0])
        self.assertLess(hard[1], soft[1])
        self.assertAlmostEqual(soft[1], 0.030 / (REGULARIZATION_TIME_SCALE * CONTROL_DT))
        self.assertTrue(all(math.isfinite(value) for value in (*hard, *soft)))

    def test_gap_shifts_only_normal_end_step_velocity(self) -> None:
        points = np.zeros((8, 3), np.float64)
        points[:, 2] = np.linspace(-0.001, 0.006, 8)
        velocity = np.arange(24, dtype=np.float64).reshape(8, 3) * 0.01
        adjusted = prospective_step_velocity(points, velocity)
        np.testing.assert_array_equal(adjusted[:, :2], velocity[:, :2])
        np.testing.assert_allclose(
            adjusted[:, 2], velocity[:, 2] + np.maximum(points[:, 2], 0.0) / CONTROL_DT
        )


if __name__ == "__main__":
    unittest.main()
