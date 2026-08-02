from __future__ import annotations

import unittest

import numpy as np

from upkie_spatial_conditioned_momentum_residual_audit import calibrate_loco_boxes


class GeneralizedMomentumReachableSetAuditTests(unittest.TestCase):
    def test_loco_calibration_excludes_named_evaluation_fold(self) -> None:
        cases = np.asarray(["a", "a", "b", "b"])
        residual = np.asarray(
            [
                [100.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                [200.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
                [-4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0],
                [-5.0, -4.0, -3.0, -2.0, 2.0, 3.0, 4.0],
            ],
            np.float64,
        )
        boxes = calibrate_loco_boxes(cases, residual)
        lower_a, upper_a = boxes["a"]["coordinate"]
        self.assertEqual(lower_a[0], -5.0)
        self.assertEqual(upper_a[0], 0.0)
        lower_b, upper_b = boxes["b"]["coordinate"]
        self.assertEqual(lower_b[0], 0.0)
        self.assertEqual(upper_b[0], 200.0)

    def test_grouped_box_is_symmetric_and_uses_group_maximum(self) -> None:
        cases = np.asarray(["a", "b"])
        residual = np.asarray(
            [
                [1.0, -2.0, 3.0, 4.0, -5.0, 6.0, 7.0, -8.0],
                [-9.0, 2.0, 1.0, -3.0, 4.0, 2.0, -5.0, 6.0],
            ],
            np.float64,
        )
        boxes = calibrate_loco_boxes(cases, residual)
        lower, upper = boxes["a"]["group_symmetric"]
        np.testing.assert_array_equal(lower, -upper)
        np.testing.assert_array_equal(upper[:3], np.full(3, 9.0))
        np.testing.assert_array_equal(upper[3:6], np.full(3, 4.0))
        np.testing.assert_array_equal(upper[6:], np.full(2, 6.0))


if __name__ == "__main__":
    unittest.main()
