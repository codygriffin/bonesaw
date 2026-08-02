from __future__ import annotations

import unittest

import numpy as np

from g1_center_response_split_residual_replay import (
    CANDIDATES,
    center_weights,
    predicted_center_impulse,
)


class G1CenterResponseSplitResidualReplayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.points = np.zeros((8, 3), np.float64)
        self.points[:, 2] = [0.04, 0.03, 0.02, 0.01] * 2
        self.velocity = np.zeros((8, 3), np.float64)
        self.velocity[:, 2] = [-1.0, -2.0, -3.0, -4.0] * 2

    def test_causal_center_rules_are_convex_and_label_free(self) -> None:
        for kind in ("geometric", "closing_weighted", "earliest", "lowest"):
            weights = center_weights(kind, self.points, self.velocity)
            self.assertEqual(weights.shape, (2, 4))
            np.testing.assert_allclose(np.sum(weights, axis=1), 1.0)
            self.assertTrue(np.all(weights >= 0.0))
        with self.assertRaisesRegex(ValueError, "completed impulses"):
            center_weights("impulse_oracle", self.points, self.velocity)

    def test_plastic_prediction_opposes_slip_and_obeys_friction(self) -> None:
        velocity = np.asarray([[3.0, -4.0, -2.0], [-5.0, 6.0, 1.0]])
        effective_mass = np.ones((2, 3), np.float64)
        impulse = predicted_center_impulse(
            velocity,
            effective_mass,
            friction=0.5,
            supported_weight_impulse=2.0,
            load_fraction=0.5,
        )
        np.testing.assert_allclose(impulse[:, 2], [3.0, 1.0])
        self.assertLessEqual(abs(impulse[0, 0]), 1.5)
        self.assertLessEqual(abs(impulse[0, 1]), 1.5)
        self.assertLess(impulse[0, 0] * velocity[0, 0], 0.0)
        self.assertLess(impulse[0, 1] * velocity[0, 1], 0.0)

    def test_candidate_family_keeps_oracle_explicit(self) -> None:
        self.assertEqual(len(CANDIDATES), 14)
        self.assertEqual(len({candidate.name for candidate in CANDIDATES}), 14)
        self.assertEqual(sum(not candidate.causal for candidate in CANDIDATES), 2)


if __name__ == "__main__":
    unittest.main()
