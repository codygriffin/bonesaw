from __future__ import annotations

import unittest

import numpy as np

from floating_momentum_cross_morphology_oracle import interval_hull


class FloatingMomentumCrossMorphologyOracleTests(unittest.TestCase):
    def test_interval_hull_maps_every_corner(self) -> None:
        matrix = np.asarray([[2.0, -1.0], [0.5, 3.0]])
        lower = np.asarray([-2.0, 1.0])
        upper = np.asarray([3.0, 4.0])
        mapped_lower, mapped_upper = interval_hull(matrix, lower, upper)
        corners = np.asarray(
            [[x, y] for x in (lower[0], upper[0]) for y in (lower[1], upper[1])]
        )
        mapped = corners @ matrix.T
        np.testing.assert_allclose(mapped_lower, np.min(mapped, axis=0))
        np.testing.assert_allclose(mapped_upper, np.max(mapped, axis=0))

    def test_interval_hull_rejects_inversion(self) -> None:
        with self.assertRaises(ValueError):
            interval_hull(np.eye(2), np.asarray([1.0, 0.0]), np.asarray([0.0, 1.0]))


if __name__ == "__main__":
    unittest.main()
