from __future__ import annotations

import unittest

import numpy as np

import upkie_lagged_impulse_conditioning as audit


class LaggedImpulseConditioningTests(unittest.TestCase):
    def test_feature_sets_never_replace_the_frozen_state_feature(self) -> None:
        samples = {
            "features": np.asarray([[1.0, 2.0], [3.0, 4.0]]),
            "state_only_features": np.asarray([[1.0, 2.0], [3.0, 4.0]]),
            "lagged_impulse": np.asarray([[0.25, 0.10, 0.50], [0.0, 0.0, 0.0]]),
        }
        linear = audit.select_feature_set(samples, "lagged_impulse_linear")
        logged = audit.select_feature_set(samples, "lagged_impulse_log")

        np.testing.assert_array_equal(linear["features"][:, :2], samples["features"])
        np.testing.assert_array_equal(linear["features"][0, 2:], np.ones(3))
        np.testing.assert_array_equal(logged["features"][:, :2], samples["features"])
        self.assertIsNot(linear["features"], samples["features"])
        np.testing.assert_array_equal(samples["features"], samples["state_only_features"])


if __name__ == "__main__":
    unittest.main()
