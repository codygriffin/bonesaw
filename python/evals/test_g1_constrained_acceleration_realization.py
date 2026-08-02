#!/usr/bin/env python3
"""Unit tests for constrained-acceleration persistence metrics."""

import unittest

import numpy as np

from g1_constrained_acceleration_realization import (
    contact_edges,
    edge_recovery_ticks,
    extended_distribution,
    inequality_traces,
    longest_true_run,
    replay_delta_contract,
)


class G1ConstrainedAccelerationRealizationTests(unittest.TestCase):
    def test_contact_edges_are_change_indices(self) -> None:
        contacts = np.array([[1, 1], [1, 1], [0, 1], [0, 1], [1, 1]], dtype=np.uint8)
        np.testing.assert_array_equal(contact_edges(contacts), np.array([2, 4]))

    def test_recovery_requires_consecutive_dwell(self) -> None:
        pressure = np.array([0.0, 0.2, 0.1, 0.009, 0.008, 0.007, 0.2])
        self.assertEqual(edge_recovery_ticks(pressure, np.array([1]), 0.01, 3), [2])

    def test_unrecovered_edge_is_explicit(self) -> None:
        pressure = np.array([0.2, 0.1, 0.02, 0.03])
        self.assertEqual(edge_recovery_ticks(pressure, np.array([0]), 0.01, 2), [None])

    def test_longest_true_run_handles_boundaries(self) -> None:
        self.assertEqual(longest_true_run(np.array([True, True, False, True])), 2)

    def test_extended_distribution_keeps_unavailable_counts(self) -> None:
        result = extended_distribution(np.array([1.0, 2.0, np.inf, -np.inf]))
        self.assertEqual(result["finite"]["maximum"], 2.0)
        self.assertEqual(result["positive_infinity"], 1)
        self.assertEqual(result["negative_infinity"], 1)

    def test_inequality_traces_recover_patch_margin_and_normal_force(self) -> None:
        acceleration = np.zeros((1, 8))
        normal_force = np.zeros((1, 8))
        normal_force[0, :4] = 10.0
        traces = inequality_traces(
            acceleration,
            normal_force,
            np.array([2.0]),
            np.array([[1, 0]], dtype=np.uint8),
        )
        self.assertAlmostEqual(traces["minimum_support_margin"][0], 0.0275)
        self.assertEqual(traces["minimum_normal_force"][0], 10.0)
        self.assertEqual(traces["acceleration_bound_excess"][0], 0.0)

    def test_replay_delta_contract_keeps_undeclared_fields_exact(self) -> None:
        source = {
            "generalized_acceleration": np.array([1.0]),
            "status": np.array([0], dtype=np.uint8),
        }
        replay = {
            "generalized_acceleration": np.array([1.0 + 1e-7]),
            "status": np.array([1], dtype=np.uint8),
        }

        passed, details = replay_delta_contract(
            replay, source, ("generalized_acceleration", "status")
        )

        self.assertFalse(passed)
        self.assertTrue(details["generalized_acceleration"]["within_contract"])
        self.assertFalse(details["status"]["within_contract"])


if __name__ == "__main__":
    unittest.main()
