#!/usr/bin/env python3
"""Unit tests for actuator-realization persistence metrics."""

import unittest

import numpy as np

from g1_actuator_realization import contact_edges, edge_recovery_ticks, longest_true_run


class G1ActuatorRealizationTests(unittest.TestCase):
    def test_contact_edges_are_half_open_change_indices(self) -> None:
        contacts = np.array([[1, 1], [1, 1], [0, 1], [0, 1], [1, 1]], dtype=np.uint8)

        np.testing.assert_array_equal(contact_edges(contacts), np.array([2, 4]))

    def test_edge_recovery_requires_declared_dwell(self) -> None:
        pressure = np.array([0.0, 0.2, 0.1, 0.04, 0.03, 0.02, 0.2])

        self.assertEqual(edge_recovery_ticks(pressure, np.array([1]), 0.05, 3), [2])

    def test_longest_true_run_handles_boundaries(self) -> None:
        self.assertEqual(longest_true_run(np.array([True, True, False, True])), 2)


if __name__ == "__main__":
    unittest.main()
