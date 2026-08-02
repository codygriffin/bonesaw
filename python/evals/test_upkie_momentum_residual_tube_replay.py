from __future__ import annotations

import unittest

import numpy as np

from upkie_momentum_residual_tube_replay import loco_momentum_bounds


class MomentumResidualTubeReplayTests(unittest.TestCase):
    def test_loco_bounds_exclude_held_out_case_and_include_zero(self) -> None:
        cases = np.asarray(["a", "a", "b", "b", "fresh"])
        fresh = np.asarray([False, False, False, False, True])
        counts = np.ones(5, np.uint8)
        residuals = np.full((5, 4, 12), np.nan, np.float64)
        residuals[:2, 0, 0] = 10.0
        residuals[2:4, 0, 0] = -2.0
        residuals[4, 0, 0] = 7.0
        residuals[:, 0, 1:] = 0.0
        lower, upper = loco_momentum_bounds(
            cases, fresh, counts, residuals, "nearest_candidate", 1.0
        )
        np.testing.assert_array_equal(lower[:2, 0], -2.0)
        np.testing.assert_array_equal(upper[:2, 0], 0.0)
        np.testing.assert_array_equal(lower[2:4, 0], 0.0)
        np.testing.assert_array_equal(upper[2:4, 0], 10.0)
        self.assertEqual(lower[4, 0], -2.0)
        self.assertEqual(upper[4, 0], 10.0)


if __name__ == "__main__":
    unittest.main()
