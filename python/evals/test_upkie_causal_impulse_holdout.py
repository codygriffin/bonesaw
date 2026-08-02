from __future__ import annotations

import unittest

import numpy as np

import upkie_causal_impulse_holdout as holdout


class CausalImpulseHoldoutTests(unittest.TestCase):
    def test_feature_is_fixed_shape_and_uses_selected_action(self) -> None:
        acceleration = np.zeros((4, 3, 12), np.float64)
        left = holdout.causal_feature(
            np.zeros(6),
            np.zeros(6),
            np.zeros(6),
            -np.ones(6),
            np.ones(6),
            np.ones(6),
            0,
            acceleration,
        )
        right = holdout.causal_feature(
            np.zeros(6),
            np.zeros(6),
            np.zeros(6),
            -np.ones(6),
            np.ones(6),
            np.ones(6),
            2,
            acceleration,
        )
        self.assertEqual(left.shape, (37,))
        self.assertEqual(right.shape, (37,))
        self.assertFalse(np.array_equal(left, right))
        np.testing.assert_array_equal(left[-3:], np.asarray([1.0, 0.0, 0.0]))
        np.testing.assert_array_equal(right[-3:], np.asarray([0.0, 0.0, 1.0]))

    def test_holdout_never_trains_on_held_target(self) -> None:
        samples = {
            "case": np.asarray(["a", "a", "b", "b"], object),
            "profile": np.asarray(["p", "p", "p", "p"], object),
            "ticks": np.asarray([0, 1, 0, 1]),
            "actions": np.zeros(4, np.uint8),
            "support_masks": np.full(4, 3, np.uint8),
            "features": np.asarray([[0.0], [0.1], [0.0], [0.1]]),
            "positive_residual": np.asarray(
                [[1.0] * 4, [1.0] * 4, [10.0] * 4, [10.0] * 4]
            ),
        }
        audit = holdout.leave_one_case_out(samples, ("a", "b"))
        self.assertEqual(audit["global_max"]["sample_coverage"], 0.5)
        self.assertFalse(audit["global_max"]["strict_holdout_admitted"])
        self.assertEqual(
            audit["global_max"]["folds"]["b"]["worst_sample"]["exceedance"],
            9.0,
        )

    def test_score_requires_all_four_targets(self) -> None:
        target = np.asarray([[1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0]])
        bound = target.copy()
        bound[1, 3] -= 0.5
        result = holdout.score_bound(target, bound)
        self.assertEqual(result["covered_samples"], 1)
        self.assertEqual(result["sample_coverage"], 0.5)
        self.assertEqual(result["component_coverage"], 0.875)


if __name__ == "__main__":
    unittest.main()
