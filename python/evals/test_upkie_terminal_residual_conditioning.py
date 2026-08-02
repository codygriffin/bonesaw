from __future__ import annotations

import unittest

import numpy as np

import upkie_terminal_residual_conditioning as conditioning


class TerminalResidualConditioningTests(unittest.TestCase):
    def test_lipschitz_cones_interpolate_declared_training_slope(self) -> None:
        train = {
            "features": np.asarray([[0.0], [1.0]]),
            "positive_residual": np.asarray([[1.0], [3.0]]),
            "actions": np.zeros(2, np.uint8),
        }
        held = {
            "features": np.asarray([[0.5]]),
            "positive_residual": np.asarray([[2.0]]),
            "actions": np.zeros(1, np.uint8),
        }

        bound = conditioning.lipschitz_action_bound(train, held)

        np.testing.assert_allclose(bound, np.asarray([[2.0]]))

    def test_leave_one_case_out_never_uses_held_residual(self) -> None:
        samples = {
            "case": np.asarray(["a", "a", "b", "b"], object),
            "profile": np.asarray(["p", "p", "p", "p"], object),
            "ticks": np.asarray([0, 1, 0, 1], np.int64),
            "positive_residual": np.asarray([[1.0], [1.0], [10.0], [10.0]]),
            "actions": np.zeros(4, np.uint8),
            "support_masks": np.full(4, 3, np.uint8),
            "features": np.asarray([[0.0], [0.1], [0.0], [0.1]]),
        }

        audit = conditioning.leave_one_case_out(samples, ("a", "b"))

        self.assertEqual(audit["global_max"]["sample_coverage"], 0.5)
        self.assertEqual(
            audit["global_max"]["folds"]["b"]["maximum_exceedance"],
            9.0,
        )
        self.assertFalse(audit["global_max"]["strict_holdout_admitted"])

    def test_bound_score_requires_every_pressure_component(self) -> None:
        residual = np.asarray([[1.0, 2.0], [1.0, 2.0]])
        bound = np.asarray([[1.0, 2.0], [1.0, 1.5]])

        score = conditioning.score_bound(residual, bound)

        self.assertEqual(score["covered_samples"], 1)
        self.assertEqual(score["sample_coverage"], 0.5)
        self.assertEqual(score["component_coverage"], 0.75)
        self.assertEqual(score["maximum_exceedance"], 0.5)


if __name__ == "__main__":
    unittest.main()
