from __future__ import annotations

import unittest

import numpy as np

from g1_causal_center_split_kinetic_replay import (
    causal_center_prediction,
    fit_affine_envelope,
)


class G1CausalCenterSplitKineticReplayTests(unittest.TestCase):
    def test_zero_closing_speed_has_zero_impulse_and_geometric_center(self) -> None:
        foot = np.asarray(
            [
                [-0.1, -0.05, 0.01],
                [-0.1, 0.05, 0.01],
                [0.1, -0.05, 0.01],
                [0.1, 0.05, 0.01],
            ],
            np.float64,
        )
        points = np.concatenate((foot, foot + [0.0, 0.2, 0.0]))
        velocity = np.zeros((8, 3), np.float64)
        effective_mass = np.ones((8, 3), np.float64)
        centers, center_velocity, normal, remaining = causal_center_prediction(
            points, velocity, effective_mass, 0.5, 0.01
        )
        np.testing.assert_allclose(centers[0], np.mean(foot, axis=0))
        np.testing.assert_allclose(centers[1], np.mean(foot, axis=0) + [0, 0.2, 0])
        np.testing.assert_array_equal(center_velocity, 0.0)
        np.testing.assert_array_equal(normal, 0.0)
        np.testing.assert_array_equal(remaining, 0.0)

    def test_sooner_closing_corner_moves_center_without_poststep_input(self) -> None:
        points = np.zeros((8, 3), np.float64)
        points[:, 2] = 0.002
        points[:4, 0] = [-0.1, -0.1, 0.1, 0.1]
        points[4:, 0] = points[:4, 0]
        velocity = np.zeros((8, 3), np.float64)
        velocity[:, 2] = -0.5
        velocity[2, 2] = -1.0
        effective_mass = np.ones((8, 3), np.float64)
        centers, _, normal, _ = causal_center_prediction(
            points, velocity, effective_mass, 0.5, 0.01
        )
        self.assertGreater(centers[0, 0], 0.0)
        self.assertGreater(normal[0], normal[1])

    def test_affine_fit_is_deterministic_and_conservative(self) -> None:
        severity = np.asarray([0.0, 0.1, 0.4, 0.8], np.float64)
        required = np.asarray([0.2, 0.4, 0.9, 2.5], np.float64)
        first = fit_affine_envelope(severity, required)
        second = fit_affine_envelope(severity, required)
        self.assertEqual(first["intercept"], second["intercept"])
        self.assertEqual(first["slope"], second["slope"])
        np.testing.assert_array_equal(first["fraction"], second["fraction"])
        self.assertTrue(np.all(first["fraction"] >= required - 1.0e-12))


if __name__ == "__main__":
    unittest.main()
