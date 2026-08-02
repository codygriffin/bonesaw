#!/usr/bin/env python3
"""Unit tests for the independent fixed-effort reference mathematics."""

import unittest

import numpy as np

from g1_pinocchio_fixed_effort_reference import (
    lexicographic_equality_solve,
    selected_ticks,
)


class G1PinocchioFixedEffortReferenceTests(unittest.TestCase):
    def test_equality_is_preserved_across_soft_levels(self) -> None:
        equality = np.array([[1.0, 1.0, 0.0]])
        target = np.array([3.0])
        invariant = (np.array([[1.0, 0.0, 0.0]]), np.array([1.0]))
        style = (np.array([[0.0, 0.0, 1.0]]), np.array([4.0]))

        solution = lexicographic_equality_solve(equality, target, [invariant, style])

        np.testing.assert_allclose(equality @ solution, target, atol=1e-12)
        np.testing.assert_allclose(solution, np.array([1.0, 2.0, 4.0]), atol=1e-12)

    def test_higher_priority_nullspace_is_frozen(self) -> None:
        equality = np.empty((0, 2))
        target = np.empty(0)
        invariant = (np.array([[1.0, 0.0]]), np.array([2.0]))
        preference = (np.array([[1.0, 1.0]]), np.array([9.0]))

        solution = lexicographic_equality_solve(equality, target, [invariant, preference])

        self.assertAlmostEqual(solution[0], 2.0, places=12)
        self.assertAlmostEqual(solution[1], 7.0, places=12)

    def test_selection_keeps_three_ticks_around_every_contact_edge(self) -> None:
        contacts = np.ones((40, 2), dtype=np.uint8)
        contacts[10:20, 0] = 0
        contacts[30:, 1] = 0
        pressure = np.linspace(0.0, 1.0, len(contacts))

        ticks = selected_ticks(contacts, pressure, 24)

        for expected in (9, 10, 11, 19, 20, 21, 29, 30, 31):
            self.assertIn(expected, ticks)


if __name__ == "__main__":
    unittest.main()
