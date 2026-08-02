from __future__ import annotations

import unittest

import numpy as np

from upkie_grouped_acceleration_reachable_set_replay import (
    reserve_vector,
    score_envelope,
)


class GroupedAccelerationReachableSetReplayTests(unittest.TestCase):
    def test_reserve_vector_uses_floating_group_order(self) -> None:
        reserve = reserve_vector((50.0, 10.0, 25.0), 9)
        np.testing.assert_array_equal(
            reserve,
            np.asarray([50.0] * 3 + [10.0] * 3 + [25.0] * 3),
        )

    def test_scoring_keeps_sample_and_component_coverage_separate(self) -> None:
        actual = np.asarray([[0.0] * 9, [0.0] * 8 + [2.0]])
        lower = np.full_like(actual, -1.0)
        upper = np.full_like(actual, 1.0)
        score = score_envelope(actual, lower, upper)
        self.assertEqual(score["covered_sample_count"], 1)
        self.assertEqual(score["sample_coverage"], 0.5)
        self.assertEqual(score["component_coverage"], 17.0 / 18.0)
        self.assertEqual(score["maximum_exceedance"], 1.0)

    def test_scoring_split_uses_selected_rows_only(self) -> None:
        actual = np.asarray([[0.0] * 9, [4.0] * 9])
        lower = np.full_like(actual, -1.0)
        upper = np.full_like(actual, 1.0)
        score = score_envelope(
            actual, lower, upper, np.asarray([True, False], np.bool_)
        )
        self.assertEqual(score["sample_count"], 1)
        self.assertEqual(score["sample_coverage"], 1.0)


if __name__ == "__main__":
    unittest.main()
