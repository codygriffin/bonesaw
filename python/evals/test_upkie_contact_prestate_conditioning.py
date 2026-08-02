from __future__ import annotations

import unittest

import numpy as np

import upkie_contact_prestate_conditioning as audit


class ContactPrestateConditioningTests(unittest.TestCase):
    def test_append_uses_current_tick_and_masks_unavailable_payload(self) -> None:
        samples = {
            "ticks": np.asarray([1, 3]),
            "features": np.asarray([[1.0, 2.0], [3.0, 4.0]]),
        }
        trace = {
            "physical_wheel_contact_prestate_available": np.asarray(
                [[0, 0], [1, 0], [1, 1], [0, 1]], np.uint8
            ),
            "physical_wheel_contact_distance_m": np.asarray(
                [[np.inf, np.inf], [-0.001, np.inf], [-0.002, -0.003], [np.inf, 0.004]]
            ),
            "physical_wheel_contact_relative_velocity_m_s": np.asarray(
                [
                    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                    [[-2.0, 0.5, 0.0], [99.0, 99.0, 99.0]],
                    [[8.0, 8.0, 8.0], [8.0, 8.0, 8.0]],
                    [[99.0, 99.0, 99.0], [3.0, -0.25, 0.125]],
                ]
            ),
        }
        audit.append_contact_prestate_features(samples, trace)

        self.assertEqual(samples["contact_prestate"].shape, (2, 10))
        np.testing.assert_array_equal(samples["contact_prestate"][:, :2], [[1, 0], [0, 1]])
        np.testing.assert_allclose(samples["contact_prestate"][:, 2:4], [[-0.1, 0.0], [0.0, 0.4]])
        np.testing.assert_allclose(samples["contact_prestate"][0, 4:], [-2.0, 0.5, 0.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(samples["contact_prestate"][1, 4:], [0.0, 0.0, 0.0, 3.0, -0.25, 0.125])

    def test_feature_sets_preserve_frozen_state_prefix(self) -> None:
        samples = {
            "features": np.asarray([[1.0, 2.0]]),
            "state_only_features": np.asarray([[1.0, 2.0]]),
            "contact_prestate": np.asarray(
                [[1.0, 0.0, -0.1, 0.0, -2.0, 0.5, 0.0, 0.0, 0.0, 0.0]]
            ),
        }
        linear = audit.select_feature_set(samples, "contact_prestate_linear")
        logged = audit.select_feature_set(samples, "contact_prestate_signed_log")

        np.testing.assert_array_equal(linear["features"][:, :2], samples["features"])
        np.testing.assert_array_equal(logged["features"][:, :2], samples["features"])
        np.testing.assert_array_equal(logged["features"][:, 2:4], [[1.0, 0.0]])
        self.assertLess(logged["features"][0, 6], 0.0)
        np.testing.assert_array_equal(samples["features"], samples["state_only_features"])


if __name__ == "__main__":
    unittest.main()
