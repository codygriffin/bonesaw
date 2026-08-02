from __future__ import annotations

import unittest

import numpy as np

from g1_stiff_contact_activation_localization import (
    first_activation_cohort,
    free_flight_activation_ticks,
)


class G1StiffContactActivationLocalizationTests(unittest.TestCase):
    def test_ballistic_ticks_and_first_cohort_are_label_free(self) -> None:
        gap = np.asarray([[0.0020, 0.0029, 0.0500]], np.float64)
        velocity = np.asarray([[-0.5, -0.5, -0.1]], np.float64)
        acceleration = np.zeros_like(gap)
        ticks = free_flight_activation_ticks(
            gap, velocity, acceleration, step_s=0.001, steps=5
        )
        np.testing.assert_array_equal(ticks, [[4, 0, 0]])
        np.testing.assert_array_equal(first_activation_cohort(ticks), [[1, 0, 0]])

    def test_tied_first_crossings_share_a_cohort(self) -> None:
        ticks = np.asarray([[3, 1, 1, 4, 0]], np.uint8)
        np.testing.assert_array_equal(
            first_activation_cohort(ticks), [[0, 1, 1, 0, 0]]
        )

    def test_invalid_shapes_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            free_flight_activation_ticks(
                np.zeros((1, 2)), np.zeros((2, 1)), np.zeros((1, 2))
            )
        with self.assertRaises(ValueError):
            first_activation_cohort(np.zeros(2, np.uint8))


if __name__ == "__main__":
    unittest.main()
