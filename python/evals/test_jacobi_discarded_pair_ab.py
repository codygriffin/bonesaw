#!/usr/bin/env python3
"""Unit tests for r60 A/B arithmetic."""

import unittest

from jacobi_discarded_pair_ab import relative_delta


class JacobiDiscardedPairAbTests(unittest.TestCase):
    def test_relative_delta_preserves_direction(self) -> None:
        self.assertAlmostEqual(relative_delta(100.0, 105.0), 0.05)
        self.assertAlmostEqual(relative_delta(100.0, 95.0), -0.05)


if __name__ == "__main__":
    unittest.main()
