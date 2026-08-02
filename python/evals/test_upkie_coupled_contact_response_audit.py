from __future__ import annotations

import unittest

import numpy as np

from upkie_coupled_contact_response_audit import fit_loco_residual, score_envelope


class CoupledContactResponseAuditTests(unittest.TestCase):
    def test_score_envelope_separates_sample_and_component_coverage(self) -> None:
        actual = np.zeros((2, 12), np.float64)
        lower = np.full((2, 12), -1.0)
        upper = np.full((2, 12), 1.0)
        actual[1, 7] = 2.0
        score = score_envelope(actual, lower, upper)
        self.assertEqual(score["sample_coverage"], 0.5)
        self.assertEqual(score["component_coverage"], 23.0 / 24.0)
        self.assertEqual(score["maximum_exceedance"], 1.0)

    def test_loco_residual_never_uses_the_held_out_named_case(self) -> None:
        cases = np.asarray(["a", "a", "b", "b", "fresh"])
        fresh = np.asarray([False, False, False, False, True])
        lower = np.zeros((5, 12), np.float64)
        upper = np.zeros((5, 12), np.float64)
        actual = np.zeros((5, 12), np.float64)
        actual[:2, 0] = 10.0
        actual[2:4, 0] = 2.0
        actual[4, 0] = 7.0
        expanded_lower, expanded_upper = fit_loco_residual(
            cases, fresh, actual, lower, upper
        )
        np.testing.assert_array_equal(expanded_lower, lower)
        np.testing.assert_array_equal(expanded_upper[:2, 0], 2.0)
        np.testing.assert_array_equal(expanded_upper[2:4, 0], 10.0)
        np.testing.assert_array_equal(expanded_upper[4, 0], 10.0)


if __name__ == "__main__":
    unittest.main()
