#!/usr/bin/env python3
"""Unit tests for r65 A/B report helpers."""

import unittest

from zero_task_row_compaction_ab import distribution, relative_delta


class ZeroTaskRowCompactionAbTests(unittest.TestCase):
    def test_relative_delta_has_signed_candidate_direction(self) -> None:
        self.assertAlmostEqual(relative_delta(100.0, 97.0), -0.03)

    def test_distribution_retains_span(self) -> None:
        result = distribution([9.0, 10.0, 11.0])
        self.assertEqual(result["median"], 10.0)
        self.assertEqual(result["relative_span"], 0.2)


if __name__ == "__main__":
    unittest.main()
