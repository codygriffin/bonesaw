from __future__ import annotations

import pathlib
import unittest

import numpy as np


class ContactTransitionBoundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import bonesaw

        model = pathlib.Path("models/upkie/upkie.urdf").resolve()
        cls.session = bonesaw.UpkieBalanceSession(str(model))

    def test_python_boundary_matches_analytic_interval_without_allocation(self) -> None:
        impulse = np.empty((1, 3), np.float64)
        lower = np.empty(1, np.float64)
        upper = np.empty(1, np.float64)
        timing = self.session.bound_contact_transition_velocity_jump(
            np.asarray([0.01, 0.02]),
            0.5,
            np.asarray([[2.0, 3.0, 10.0, 0.5]]),
            np.asarray([4.0]),
            np.asarray([[[2.0, -1.0, 0.5]]]),
            impulse,
            lower,
            upper,
        )
        np.testing.assert_allclose(impulse, np.asarray([[4.6, 4.6, 9.2]]))
        np.testing.assert_allclose(lower, np.asarray([-13.76]))
        np.testing.assert_allclose(upper, np.asarray([18.48]))
        self.assertGreaterEqual(timing[0], 0)
        self.assertEqual(timing[1:], (0, 0))

    def test_boundary_rejects_wrong_response_shape(self) -> None:
        with self.assertRaises(ValueError):
            self.session.bound_contact_transition_velocity_jump(
                np.asarray([0.0, 0.005]),
                1.0,
                np.ones((2, 4), np.float64),
                np.zeros(3, np.float64),
                np.zeros((3, 1, 3), np.float64),
                np.empty((2, 3), np.float64),
                np.empty(3, np.float64),
                np.empty(3, np.float64),
            )

    def test_invalid_witness_does_not_mutate_outputs(self) -> None:
        impulse = np.full((1, 3), 7.0)
        lower = np.full(1, 8.0)
        upper = np.full(1, 9.0)
        with self.assertRaises(ValueError):
            self.session.bound_contact_transition_velocity_jump(
                np.asarray([0.0, 0.005]),
                1.0,
                np.asarray([[1.0, 2.0, 3.0, -0.1]]),
                np.zeros(1),
                np.zeros((1, 1, 3)),
                impulse,
                lower,
                upper,
            )
        np.testing.assert_array_equal(impulse, np.full((1, 3), 7.0))
        np.testing.assert_array_equal(lower, np.full(1, 8.0))
        np.testing.assert_array_equal(upper, np.full(1, 9.0))

    def test_model_owns_upkie_response_and_effective_mass_without_allocation(self) -> None:
        root = np.asarray([0.0, 0.0, 0.539], np.float64)
        q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
        points = np.asarray(
            [[0.0, 0.09, 0.0], [0.0, -0.09, 0.0]], np.float64
        )
        bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
        response = np.empty((12, 2, 3), np.float64)
        effective_mass = np.empty((2, 3), np.float64)
        timing = self.session.model_contact_impulse_velocity_response(
            root,
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            q,
            points,
            bases,
            response,
            effective_mass,
        )
        self.assertTrue(np.all(np.isfinite(response)))
        self.assertTrue(np.all(np.isfinite(effective_mass)))
        self.assertTrue(np.all(effective_mass > 0.0))
        self.assertGreaterEqual(timing[0], 0)
        self.assertEqual(timing[1:], (0, 0))
        self.assertGreater(float(np.max(np.abs(response))), 0.0)
        self.assertGreater(float(np.max(np.abs(response[:, 0] - response[:, 1]))), 0.0)

    def test_model_response_rejects_nonorthonormal_contact_basis(self) -> None:
        response = np.full((12, 2, 3), 7.0)
        effective_mass = np.full((2, 3), 8.0)
        bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
        bases[0, 1] = bases[0, 0]
        with self.assertRaisesRegex(ValueError, "InvalidContact"):
            self.session.model_contact_impulse_velocity_response(
                np.asarray([0.0, 0.0, 0.539], np.float64),
                np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
                np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64),
                np.asarray([[0.0, 0.09, 0.0], [0.0, -0.09, 0.0]], np.float64),
                bases,
                response,
                effective_mass,
            )
        np.testing.assert_array_equal(response, np.full((12, 2, 3), 7.0))
        np.testing.assert_array_equal(effective_mass, np.full((2, 3), 8.0))

    def test_model_exposes_complete_delassus_without_allocation(self) -> None:
        response = np.empty((12, 2, 3), np.float64)
        effective_mass = np.empty((2, 3), np.float64)
        delassus = np.empty((6, 6), np.float64)
        timing = self.session.model_contact_impulse_velocity_response_with_delassus(
            np.asarray([0.0, 0.0, 0.539], np.float64),
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64),
            np.asarray([[0.0, 0.09, 0.0], [0.0, -0.09, 0.0]], np.float64),
            np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0),
            response,
            effective_mass,
            delassus,
        )
        np.testing.assert_array_equal(delassus, delassus.T)
        np.testing.assert_allclose(
            np.diag(delassus), 1.0 / effective_mass.reshape(-1), rtol=1.0e-12
        )
        self.assertGreater(abs(float(delassus[2, 5])), 1.0e-9)
        self.assertEqual(timing[1:], (0, 0))

    def test_model_spatial_response_preserves_force_and_moment_without_allocation(
        self,
    ) -> None:
        root = np.asarray([0.0, 0.0, 0.539], np.float64)
        quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
        q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
        contact_points = np.asarray(
            [[0.03, 0.09, -0.01], [-0.02, -0.09, 0.015]], np.float64
        )
        references = np.asarray(
            [[0.0, 0.09, 0.08], [0.0, -0.09, 0.08]], np.float64
        )
        bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
        point_response = np.empty((12, 2, 3), np.float64)
        effective_mass = np.empty((2, 3), np.float64)
        self.session.model_contact_impulse_velocity_response(
            root,
            quaternion,
            q,
            contact_points,
            bases,
            point_response,
            effective_mass,
        )
        spatial_response = np.empty((12, 2, 6), np.float64)
        spatial_delassus = np.empty((12, 12), np.float64)
        timing = self.session.model_contact_spatial_impulse_velocity_response(
            root,
            quaternion,
            q,
            references,
            bases,
            spatial_response,
            spatial_delassus,
        )
        forces = np.asarray([[0.8, -0.3, 1.4], [-0.2, 0.5, 0.9]])
        moments = np.cross(contact_points - references, forces)
        point_delta = np.einsum("dcx,cx->d", point_response, forces)
        spatial_impulses = np.concatenate((moments, forces), axis=1)
        spatial_delta = np.einsum(
            "dcx,cx->d", spatial_response, spatial_impulses
        )
        np.testing.assert_allclose(spatial_delta, point_delta, atol=1.0e-11)
        np.testing.assert_array_equal(spatial_delassus, spatial_delassus.T)
        self.assertTrue(np.all(np.diag(spatial_delassus) > 0.0))
        self.assertEqual(timing[1:], (0, 0))

    def test_generalized_momentum_residual_is_independent_and_allocation_free(
        self,
    ) -> None:
        observed = np.linspace(-0.03, 0.04, 12, dtype=np.float64)
        predicted = np.stack((observed, np.zeros(12, np.float64)))
        residual = np.empty_like(predicted)
        timing = self.session.model_generalized_momentum_impulse_residuals(
            np.asarray([0.0, 0.0, 0.539], np.float64),
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64),
            observed,
            predicted,
            residual,
        )
        np.testing.assert_allclose(residual[0], 0.0, atol=1.0e-15)
        self.assertTrue(np.all(np.isfinite(residual[1])))
        self.assertGreater(float(np.linalg.norm(residual[1])), 0.0)
        self.assertEqual(timing[1:], (0, 0))

    def test_momentum_box_round_trips_through_inverse_mass_without_allocation(
        self,
    ) -> None:
        root = np.asarray([0.0, 0.0, 0.539], np.float64)
        quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
        q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
        momentum = np.linspace(-0.02, 0.03, 12, dtype=np.float64)
        velocity_lower = np.empty(12, np.float64)
        velocity_upper = np.empty(12, np.float64)
        timing = self.session.model_generalized_velocity_interval_from_momentum_box(
            root,
            quaternion,
            q,
            momentum,
            momentum,
            velocity_lower,
            velocity_upper,
        )
        np.testing.assert_array_equal(velocity_lower, velocity_upper)
        residual = np.empty((1, 12), np.float64)
        self.session.model_generalized_momentum_impulse_residuals(
            root,
            quaternion,
            q,
            velocity_lower,
            np.zeros((1, 12), np.float64),
            residual,
        )
        np.testing.assert_allclose(residual[0], momentum, atol=1.0e-12)
        self.assertEqual(timing[1:], (0, 0))

    def test_momentum_box_rejects_inversion_atomically(self) -> None:
        lower = np.zeros(12, np.float64)
        upper = np.zeros(12, np.float64)
        lower[4] = 1.0
        upper[4] = -1.0
        velocity_lower = np.full(12, 7.0)
        velocity_upper = np.full(12, 8.0)
        with self.assertRaisesRegex(ValueError, "Dimension"):
            self.session.model_generalized_velocity_interval_from_momentum_box(
                np.asarray([0.0, 0.0, 0.539], np.float64),
                np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
                np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64),
                lower,
                upper,
                velocity_lower,
                velocity_upper,
            )
        np.testing.assert_array_equal(velocity_lower, np.full(12, 7.0))
        np.testing.assert_array_equal(velocity_upper, np.full(12, 8.0))

    def test_coupled_solver_uses_cross_contact_response_without_allocation(self) -> None:
        delassus = np.eye(6, dtype=np.float64)
        delassus[2, 5] = 0.5
        delassus[5, 2] = 0.5
        impulse = np.empty((2, 3), np.float64)
        after = np.empty((2, 3), np.float64)
        timing = self.session.solve_coupled_contact_impulse(
            np.asarray([[0.0, 0.0, -1.0], [0.0, 0.0, -1.0]], np.float64),
            delassus,
            np.asarray([[0.0, 0.0, 10.0], [0.0, 0.0, 10.0]], np.float64),
            np.zeros(2, np.float64),
            0.0,
            0.0,
            16,
            impulse,
            after,
        )
        np.testing.assert_allclose(impulse[:, 2], 2.0 / 3.0, atol=1.0e-9)
        np.testing.assert_allclose(after[:, 2], 0.0, atol=1.0e-9)
        np.testing.assert_array_equal(impulse[:, :2], 0.0)
        self.assertEqual(timing[1:], (0, 0))

    def test_coupled_solver_rejects_asymmetry_atomically(self) -> None:
        delassus = np.eye(6, dtype=np.float64)
        delassus[0, 1] = 0.2
        impulse = np.full((2, 3), 7.0)
        after = np.full((2, 3), 8.0)
        with self.assertRaisesRegex(ValueError, "InvalidDelassus"):
            self.session.solve_coupled_contact_impulse(
                np.zeros((2, 3), np.float64),
                delassus,
                np.ones((2, 3), np.float64),
                np.ones(2, np.float64),
                0.0,
                0.0,
                1,
                impulse,
                after,
            )
        np.testing.assert_array_equal(impulse, np.full((2, 3), 7.0))
        np.testing.assert_array_equal(after, np.full((2, 3), 8.0))

    def test_acceleration_interval_is_independent_of_zero_impulse(self) -> None:
        impulse = np.empty((1, 3), np.float64)
        lower = np.empty(1, np.float64)
        upper = np.empty(1, np.float64)
        timing = self.session.bound_contact_transition_velocity_jump_with_acceleration_interval(
            np.asarray([0.01, 0.02]),
            0.0,
            np.asarray([[0.0, 1.0, 0.0, 0.0]]),
            np.asarray([-4.0]),
            np.asarray([6.0]),
            np.zeros((1, 1, 3), np.float64),
            impulse,
            lower,
            upper,
        )
        np.testing.assert_array_equal(impulse, np.zeros((1, 3), np.float64))
        np.testing.assert_allclose(lower, np.asarray([-0.08]))
        np.testing.assert_allclose(upper, np.asarray([0.12]))
        self.assertEqual(timing[1:], (0, 0))

    def test_acceleration_interval_rejects_inverted_component(self) -> None:
        impulse = np.full((1, 3), 7.0)
        lower = np.full(1, 8.0)
        upper = np.full(1, 9.0)
        with self.assertRaisesRegex(ValueError, "InvalidResponse"):
            self.session.bound_contact_transition_velocity_jump_with_acceleration_interval(
                np.asarray([0.0, 0.005]),
                1.0,
                np.asarray([[1.0, 1.0, 0.0, 0.0]]),
                np.asarray([2.0]),
                np.asarray([1.0]),
                np.zeros((1, 1, 3), np.float64),
                impulse,
                lower,
                upper,
            )
        np.testing.assert_array_equal(impulse, np.full((1, 3), 7.0))
        np.testing.assert_array_equal(lower, np.full(1, 8.0))
        np.testing.assert_array_equal(upper, np.full(1, 9.0))

    def test_directional_tangent_uses_slip_bound_before_coulomb_capacity(self) -> None:
        impulse = np.empty((1, 3), np.float64)
        lower = np.empty(12, np.float64)
        upper = np.empty(12, np.float64)
        response = np.zeros((12, 1, 3), np.float64)
        response[0, 0] = [1.0, 1.0, 0.0]
        timing = self.session.bound_directional_contact_transition_velocity_jump(
            np.asarray([0.01, 0.02]),
            0.5,
            np.asarray(
                [[0.1, 3.0, 1.0, 1.0, 5.0, 6.0, 0.0, 20.0, 30.0, 0.5]]
            ),
            np.zeros(12),
            np.zeros(12),
            response,
            impulse,
            lower,
            upper,
        )
        np.testing.assert_allclose(impulse, np.asarray([[0.1, 4.8, 9.6]]))
        self.assertAlmostEqual(lower[0], -4.9)
        self.assertAlmostEqual(upper[0], 4.9)
        np.testing.assert_array_equal(lower[1:], np.zeros(11))
        np.testing.assert_array_equal(upper[1:], np.zeros(11))
        self.assertEqual(timing[1:], (0, 0))

    def test_directional_tube_rejects_inverted_acceleration_before_outputs_change(self) -> None:
        impulse = np.full((1, 3), 7.0)
        lower = np.full(12, 8.0)
        upper = np.full(12, 9.0)
        with self.assertRaisesRegex(ValueError, "InvalidResponse"):
            self.session.bound_directional_contact_transition_velocity_jump(
                np.asarray([0.0, 0.005]),
                0.0,
                np.asarray([[0.0] * 9 + [0.5]]),
                np.ones(12),
                np.zeros(12),
                np.zeros((12, 1, 3)),
                impulse,
                lower,
                upper,
            )
        np.testing.assert_array_equal(impulse, np.full((1, 3), 7.0))
        np.testing.assert_array_equal(lower, np.full(12, 8.0))
        np.testing.assert_array_equal(upper, np.full(12, 9.0))

    def test_directional_bound_uses_passive_slip_limit_without_allocation(self) -> None:
        impulse = np.empty((1, 3), np.float64)
        lower = np.empty(1, np.float64)
        upper = np.empty(1, np.float64)
        timing = self.session.bound_directional_contact_transition_velocity_jump(
            np.asarray([0.01, 0.02]),
            0.5,
            np.asarray([[0.1, 3.0, 1.0, 1.0, 5.0, 6.0, 0.0, 20.0, 30.0, 0.5]]),
            np.asarray([0.0]),
            np.asarray([0.0]),
            np.asarray([[[1.0, 1.0, 0.0]]]),
            impulse,
            lower,
            upper,
        )
        np.testing.assert_allclose(impulse, np.asarray([[0.1, 4.8, 9.6]]))
        np.testing.assert_allclose(lower, np.asarray([-4.9]))
        np.testing.assert_allclose(upper, np.asarray([4.9]))
        self.assertEqual(timing[1:], (0, 0))


if __name__ == "__main__":
    unittest.main()
