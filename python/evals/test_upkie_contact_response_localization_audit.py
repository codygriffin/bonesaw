from __future__ import annotations

import unittest

import numpy as np

import upkie_contact_response_localization_audit as audit


class ContactResponseLocalizationAuditTests(unittest.TestCase):
    def test_impulse_weighted_centroid_falls_back_without_contact(self) -> None:
        prospective = np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        centroid, available = audit.impulse_weighted_centroid(
            prospective,
            np.asarray([[0.2, 0.4, 0.6], [0.0, 0.0, 0.0]]),
            np.asarray([0.2, 0.0]),
        )
        np.testing.assert_allclose(centroid[0], [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(centroid[1], prospective[1])
        np.testing.assert_array_equal(available, [True, False])

    def test_response_projection_preserves_contact_axis_layout(self) -> None:
        response = np.arange(24, dtype=np.float64).reshape(4, 2, 3)
        impulse = np.asarray([[1.0, 2.0, 3.0], [-1.0, 0.5, 2.0]])
        expected = np.asarray(
            [np.sum(response[row] * impulse) for row in range(4)]
        )
        np.testing.assert_array_equal(
            audit.project_impulse_response(response, impulse), expected
        )

    def test_candidate_interval_adds_structured_reserve(self) -> None:
        candidates = np.zeros((2, 12), np.float64)
        candidates[1] = 1.0
        lower, upper = audit.candidate_interval(candidates, np.zeros(12))
        np.testing.assert_allclose(lower, -audit.STRUCTURED_ACCELERATION_RESERVE * 0.005)
        np.testing.assert_allclose(
            upper,
            (np.ones(12) + audit.STRUCTURED_ACCELERATION_RESERVE) * 0.005,
        )


if __name__ == "__main__":
    unittest.main()
