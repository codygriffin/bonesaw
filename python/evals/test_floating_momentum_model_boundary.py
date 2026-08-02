from __future__ import annotations

import unittest

import numpy as np

import bonesaw


class FloatingMomentumModelBoundaryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.session = bonesaw.ContactTransitionModelSession(
            "models/upkie/upkie.urdf", ["left_wheel_center"]
        )
        self.root = np.zeros(3, np.float64)
        self.quaternion = np.asarray([1.0, 0.0, 0.0, 0.0])
        self.q = np.zeros(self.session.joint_dof(), np.float64)

    def test_singleton_box_and_residual_round_trip_without_allocation(self) -> None:
        n = self.session.generalized_dof()
        momentum = np.linspace(-0.01, 0.02, n)
        lower = np.empty(n, np.float64)
        upper = np.empty(n, np.float64)
        timing = self.session.generalized_velocity_interval_from_momentum_box(
            self.root, self.quaternion, self.q, momentum, momentum, lower, upper
        )
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_array_equal(lower, upper)
        predicted = np.stack((np.zeros(n), lower))
        residual = np.empty_like(predicted)
        timing = self.session.generalized_momentum_impulse_residuals(
            self.root, self.quaternion, self.q, lower, predicted, residual
        )
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_allclose(residual[0], momentum, atol=1e-11, rtol=0.0)
        np.testing.assert_allclose(residual[1], 0.0, atol=1e-11, rtol=0.0)

    def test_inverted_box_is_atomic(self) -> None:
        n = self.session.generalized_dof()
        lower_momentum = np.zeros(n, np.float64)
        upper_momentum = np.zeros(n, np.float64)
        lower_momentum[0] = 1.0
        upper_momentum[0] = -1.0
        lower = np.full(n, 7.0)
        upper = np.full(n, 8.0)
        with self.assertRaises(ValueError):
            self.session.generalized_velocity_interval_from_momentum_box(
                self.root,
                self.quaternion,
                self.q,
                lower_momentum,
                upper_momentum,
                lower,
                upper,
            )
        np.testing.assert_array_equal(lower, 7.0)
        np.testing.assert_array_equal(upper, 8.0)

    def test_kinetic_impulse_ellipsoid_is_symmetric_zero_alloc_and_atomic(self) -> None:
        n = self.session.generalized_dof()
        lower = np.empty(n, np.float64)
        upper = np.empty(n, np.float64)
        timing = self.session.generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
            self.root, self.quaternion, self.q, 0.025, lower, upper
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertTrue(np.all(upper > 0.0))
        np.testing.assert_array_equal(lower, -upper)
        lower.fill(7.0)
        upper.fill(8.0)
        with self.assertRaises(ValueError):
            self.session.generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
                self.root, self.quaternion, self.q, -1.0, lower, upper
            )
        np.testing.assert_array_equal(lower, 7.0)
        np.testing.assert_array_equal(upper, 8.0)


if __name__ == "__main__":
    unittest.main()
