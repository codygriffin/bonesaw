from __future__ import annotations

import pathlib
import unittest

import numpy as np

from bonesaw import ContactTransitionModelSession, UpkieBalanceSession


ROOT = pathlib.Path(__file__).resolve().parents[2]
UPKIE_URDF = ROOT / "models" / "upkie" / "upkie.urdf"


class ContactTransitionModelSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.generic = ContactTransitionModelSession(
            str(UPKIE_URDF), ["left_wheel_center", "right_wheel_center"]
        )
        self.upkie = UpkieBalanceSession(str(UPKIE_URDF))
        self.root = np.asarray([0.1, -0.2, 0.55], np.float64)
        self.quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
        self.q = np.zeros(self.generic.joint_dof(), np.float64)
        self.points = np.asarray(
            [[0.1, 0.08, 0.05], [0.1, -0.08, 0.05]], np.float64
        )
        self.bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)

    def test_metadata_and_unknown_frame_validation(self) -> None:
        self.assertEqual(self.generic.contact_count(), 2)
        self.assertEqual(self.generic.generalized_dof(), self.generic.joint_dof() + 6)
        with self.assertRaisesRegex(ValueError, "missing_frame"):
            ContactTransitionModelSession(str(UPKIE_URDF), ["missing_frame"])

    def test_point_and_spatial_queries_match_upkie_boundary(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        generic_point = np.empty((generalized_dof, 2, 3), np.float64)
        upkie_point = np.empty_like(generic_point)
        generic_mass = np.empty((2, 3), np.float64)
        upkie_mass = np.empty_like(generic_mass)
        generic_delassus = np.empty((6, 6), np.float64)
        upkie_delassus = np.empty_like(generic_delassus)

        point_timing = self.generic.point_impulse_velocity_response(
            self.root,
            self.quaternion,
            self.q,
            self.points,
            self.bases,
            generic_point,
            generic_mass,
            generic_delassus,
        )
        upkie_timing = self.upkie.model_contact_impulse_velocity_response_with_delassus(
            self.root,
            self.quaternion,
            self.q,
            self.points,
            self.bases,
            upkie_point,
            upkie_mass,
            upkie_delassus,
        )
        np.testing.assert_array_equal(generic_point, upkie_point)
        np.testing.assert_array_equal(generic_mass, upkie_mass)
        np.testing.assert_array_equal(generic_delassus, upkie_delassus)
        self.assertEqual(point_timing[1:], (0, 0))
        self.assertEqual(upkie_timing[1:], (0, 0))

        generic_spatial = np.empty((generalized_dof, 2, 6), np.float64)
        upkie_spatial = np.empty_like(generic_spatial)
        generic_spatial_delassus = np.empty((12, 12), np.float64)
        upkie_spatial_delassus = np.empty_like(generic_spatial_delassus)
        spatial_timing = self.generic.spatial_impulse_velocity_response(
            self.root,
            self.quaternion,
            self.q,
            self.points,
            self.bases,
            generic_spatial,
            generic_spatial_delassus,
        )
        self.upkie.model_contact_spatial_impulse_velocity_response(
            self.root,
            self.quaternion,
            self.q,
            self.points,
            self.bases,
            upkie_spatial,
            upkie_spatial_delassus,
        )
        np.testing.assert_array_equal(generic_spatial, upkie_spatial)
        np.testing.assert_array_equal(generic_spatial_delassus, upkie_spatial_delassus)
        self.assertEqual(spatial_timing[1:], (0, 0))

    def test_momentum_queries_match_upkie_boundary(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        observed = np.linspace(-0.2, 0.3, generalized_dof, dtype=np.float64)
        predicted = np.stack((observed * 0.25, observed * -0.5))
        generic_residual = np.empty_like(predicted)
        upkie_residual = np.empty_like(predicted)
        timing = self.generic.generalized_momentum_impulse_residuals(
            self.root,
            self.quaternion,
            self.q,
            observed,
            predicted,
            generic_residual,
        )
        self.upkie.model_generalized_momentum_impulse_residuals(
            self.root,
            self.quaternion,
            self.q,
            observed,
            predicted,
            upkie_residual,
        )
        np.testing.assert_array_equal(generic_residual, upkie_residual)
        self.assertEqual(timing[1:], (0, 0))

        lower = -np.linspace(0.01, 0.03, generalized_dof, dtype=np.float64)
        upper = np.linspace(0.02, 0.04, generalized_dof, dtype=np.float64)
        generic_lower = np.empty(generalized_dof, np.float64)
        generic_upper = np.empty(generalized_dof, np.float64)
        upkie_lower = np.empty(generalized_dof, np.float64)
        upkie_upper = np.empty(generalized_dof, np.float64)
        box_timing = self.generic.generalized_velocity_interval_from_momentum_box(
            self.root,
            self.quaternion,
            self.q,
            lower,
            upper,
            generic_lower,
            generic_upper,
        )
        self.upkie.model_generalized_velocity_interval_from_momentum_box(
            self.root,
            self.quaternion,
            self.q,
            lower,
            upper,
            upkie_lower,
            upkie_upper,
        )
        np.testing.assert_array_equal(generic_lower, upkie_lower)
        np.testing.assert_array_equal(generic_upper, upkie_upper)
        self.assertEqual(box_timing[1:], (0, 0))

    def test_split_kinetic_bounds_add_partition_support_without_allocation(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        root_lower = np.empty(generalized_dof, np.float64)
        root_upper = np.empty(generalized_dof, np.float64)
        joint_lower = np.empty(generalized_dof, np.float64)
        joint_upper = np.empty(generalized_dof, np.float64)
        combined_lower = np.empty(generalized_dof, np.float64)
        combined_upper = np.empty(generalized_dof, np.float64)
        for root_energy, joint_energy, lower, upper in (
            (0.04, 0.0, root_lower, root_upper),
            (0.0, 0.01, joint_lower, joint_upper),
            (0.04, 0.01, combined_lower, combined_upper),
        ):
            timing = self.generic.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                self.root,
                self.quaternion,
                self.q,
                root_energy,
                joint_energy,
                lower,
                upper,
            )
            self.assertEqual(timing[1:], (0, 0))
            np.testing.assert_array_equal(lower, -upper)
        np.testing.assert_allclose(
            combined_upper,
            root_upper + joint_upper,
            atol=1.0e-12,
            rtol=0.0,
        )
        unchanged_lower = np.full(generalized_dof, 7.0, np.float64)
        unchanged_upper = np.full(generalized_dof, 8.0, np.float64)
        with self.assertRaisesRegex(ValueError, "split kinetic"):
            self.generic.generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                self.root,
                self.quaternion,
                self.q,
                0.04,
                -0.01,
                unchanged_lower,
                unchanged_upper,
            )
        np.testing.assert_array_equal(unchanged_lower, 7.0)
        np.testing.assert_array_equal(unchanged_upper, 8.0)

    def test_directional_bound_matches_upkie_boundary(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        response = np.empty((generalized_dof, 2, 3), np.float64)
        effective_mass = np.empty((2, 3), np.float64)
        delassus = np.empty((6, 6), np.float64)
        self.generic.point_impulse_velocity_response(
            self.root,
            self.quaternion,
            self.q,
            self.points,
            self.bases,
            response,
            effective_mass,
            delassus,
        )
        witnesses = np.empty((2, 10), np.float64)
        witnesses[:, :3] = [[0.2, 0.1, 0.4], [0.3, 0.2, 0.5]]
        witnesses[:, 3:6] = effective_mass
        witnesses[:, 6:9] = [8.0, 8.0, 100.0]
        witnesses[:, 9] = 0.6
        acceleration = np.linspace(-2.0, 3.0, generalized_dof)
        reserve = np.linspace(5.0, 50.0, generalized_dof)
        generic_impulse = np.empty((2, 3), np.float64)
        generic_lower = np.empty(generalized_dof, np.float64)
        generic_upper = np.empty(generalized_dof, np.float64)
        upkie_impulse = np.empty((2, 3), np.float64)
        upkie_lower = np.empty(generalized_dof, np.float64)
        upkie_upper = np.empty(generalized_dof, np.float64)
        timing = self.generic.bound_directional_contact_transition_velocity_jump(
            np.asarray([0.0, 0.005], np.float64),
            1.0,
            witnesses,
            acceleration - reserve,
            acceleration + reserve,
            response,
            generic_impulse,
            generic_lower,
            generic_upper,
        )
        self.upkie.bound_directional_contact_transition_velocity_jump(
            np.asarray([0.0, 0.005], np.float64),
            1.0,
            witnesses,
            acceleration - reserve,
            acceleration + reserve,
            response,
            upkie_impulse,
            upkie_lower,
            upkie_upper,
        )
        np.testing.assert_array_equal(generic_impulse, upkie_impulse)
        np.testing.assert_array_equal(generic_lower, upkie_lower)
        np.testing.assert_array_equal(generic_upper, upkie_upper)
        self.assertEqual(timing[1:], (0, 0))

    def test_generic_coupled_impulse_uses_all_configured_contacts(self) -> None:
        velocity = np.asarray(
            [[0.2, 0.0, -1.0], [-0.1, 0.0, -0.5]], np.float64
        )
        delassus = np.eye(6, dtype=np.float64)
        delassus[2, 5] = delassus[5, 2] = 0.2
        upper = np.full((2, 3), 10.0, np.float64)
        friction = np.full(2, 0.5, np.float64)
        impulse = np.empty((2, 3), np.float64)
        after = np.empty((2, 3), np.float64)
        timing = self.generic.solve_coupled_contact_impulse(
            velocity,
            delassus,
            upper,
            friction,
            0.0,
            0.0,
            16,
            impulse,
            after,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertTrue(np.all(impulse[:, 2] >= 0.0))
        self.assertGreater(impulse[0, 2], impulse[1, 2])
        self.assertTrue(np.all(np.linalg.norm(impulse[:, :2], axis=1) <= 0.5 * impulse[:, 2] + 1e-12))

    def test_contact_hypothesis_envelope_matches_declared_scenarios_without_allocation(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        velocity = np.asarray(
            [
                [[0.0, 0.0, -1.0], [0.0, 0.0, -0.5]],
                [[0.0, 0.0, -2.0], [0.0, 0.0, -1.5]],
            ],
            np.float64,
        )
        delassus = np.eye(6, dtype=np.float64)
        upper = np.full((2, 2, 3), 10.0, np.float64)
        friction = np.zeros((2, 2), np.float64)
        restitution = np.zeros(2, np.float64)
        regularization = np.zeros(2, np.float64)
        response = np.zeros((generalized_dof, 2, 3), np.float64)
        response[0, 0, 2] = 1.0
        response[0, 1, 2] = 2.0
        response[1, 0, 2] = -3.0
        response[1, 1, 2] = 0.5
        lower = np.empty(generalized_dof, np.float64)
        envelope_upper = np.empty(generalized_dof, np.float64)
        timing = self.generic.coupled_contact_hypothesis_velocity_envelope(
            velocity,
            delassus,
            upper,
            friction,
            restitution,
            regularization,
            response,
            2,
            lower,
            envelope_upper,
        )
        self.assertEqual(timing[1:], (0, 0))

        expected = []
        impulse = np.empty((2, 3), np.float64)
        after = np.empty((2, 3), np.float64)
        for hypothesis in range(2):
            self.generic.solve_coupled_contact_impulse(
                velocity[hypothesis],
                delassus,
                upper[hypothesis],
                friction[hypothesis],
                restitution[hypothesis],
                regularization[hypothesis],
                2,
                impulse,
                after,
            )
            expected.append(np.einsum("dca,ca->d", response, impulse))
        expected = np.asarray(expected)
        np.testing.assert_array_equal(lower, np.min(expected, axis=0))
        np.testing.assert_array_equal(envelope_upper, np.max(expected, axis=0))

        unchanged_lower = np.full(generalized_dof, 7.0, np.float64)
        unchanged_upper = np.full(generalized_dof, 8.0, np.float64)
        invalid_velocity = velocity.copy()
        invalid_velocity[-1, -1, -1] = np.nan
        with self.assertRaisesRegex(ValueError, "hypothesis envelope"):
            self.generic.coupled_contact_hypothesis_velocity_envelope(
                invalid_velocity,
                delassus,
                upper,
                friction,
                restitution,
                regularization,
                response,
                2,
                unchanged_lower,
                unchanged_upper,
            )
        np.testing.assert_array_equal(unchanged_lower, 7.0)
        np.testing.assert_array_equal(unchanged_upper, 8.0)

    def test_substepped_compliance_evolves_declared_gap_without_allocation(self) -> None:
        gap = np.asarray([-0.01, 0.10], np.float64)
        velocity = np.asarray(
            [[0.0, 0.0, -1.0], [0.0, 0.0, 0.0]], np.float64
        )
        delassus = np.eye(6, dtype=np.float64)
        upper = np.ones((2, 3), np.float64)
        friction = np.full(2, 0.5, np.float64)
        stiffness = np.full(2, 100.0, np.float64)
        damping = np.zeros(2, np.float64)
        impulse = np.empty((2, 3), np.float64)
        after = np.empty((2, 3), np.float64)
        gap_after = np.empty(2, np.float64)
        timing = self.generic.solve_substepped_compliant_contact_impulse(
            gap,
            velocity,
            delassus,
            upper,
            friction,
            stiffness,
            damping,
            0.01,
            1,
            impulse,
            after,
            gap_after,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertAlmostEqual(impulse[0, 2], 0.02)
        self.assertAlmostEqual(after[0, 2], -0.98)
        self.assertAlmostEqual(gap_after[0], -0.0198)
        np.testing.assert_array_equal(impulse[1], 0.0)
        self.assertEqual(gap_after[1], gap[1])

        unchanged_impulse = np.full((2, 3), 7.0, np.float64)
        unchanged_after = np.full((2, 3), 8.0, np.float64)
        unchanged_gap = np.full(2, 9.0, np.float64)
        invalid_stiffness = stiffness.copy()
        invalid_stiffness[-1] = np.nan
        with self.assertRaisesRegex(ValueError, "substepped compliant"):
            self.generic.solve_substepped_compliant_contact_impulse(
                gap,
                velocity,
                delassus,
                upper,
                friction,
                invalid_stiffness,
                damping,
                0.01,
                1,
                unchanged_impulse,
                unchanged_after,
                unchanged_gap,
            )
        np.testing.assert_array_equal(unchanged_impulse, 7.0)
        np.testing.assert_array_equal(unchanged_after, 8.0)
        np.testing.assert_array_equal(unchanged_gap, 9.0)

    def test_positive_reference_compliance_is_typed_atomic_and_allocation_free(self) -> None:
        contacts = self.generic.contact_count()
        gap = np.asarray([-0.001, -1.0e-6], np.float64)
        velocity = np.asarray(
            [[0.3, -0.2, -0.5], [-0.1, 0.2, -0.5]], np.float64
        )
        acceleration = np.zeros_like(velocity)
        delassus = np.eye(3 * contacts, dtype=np.float64)
        upper = np.full((contacts, 3), 10.0, np.float64)
        friction = np.full(contacts, 0.5, np.float64)
        mass = np.full(contacts, 2.0, np.float64)
        time_constant = np.full(contacts, 0.02, np.float64)
        damping_ratio = np.ones(contacts, np.float64)
        impedance_min = np.full(contacts, 0.8, np.float64)
        impedance_max = np.full(contacts, 0.96, np.float64)
        impedance_width = np.full(contacts, 0.001, np.float64)
        midpoint = np.full(contacts, 0.5, np.float64)
        power = np.full(contacts, 2.0, np.float64)
        impulse = np.empty((contacts, 3), np.float64)
        after = np.empty((contacts, 3), np.float64)
        gap_after = np.empty(contacts, np.float64)
        arguments = (
            gap,
            velocity,
            acceleration,
            delassus,
            upper,
            friction,
            mass,
            time_constant,
            damping_ratio,
            impedance_min,
            impedance_max,
            impedance_width,
            midpoint,
            power,
            0.002,
            0.005,
            16,
            1,
            2,
            impulse,
            after,
            gap_after,
        )
        timing = self.generic.solve_positive_reference_compliant_contact_impulse(
            *arguments
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertGreater(impulse[0, 2], impulse[1, 2])
        self.assertTrue(
            np.all(np.abs(impulse[:, 0]) + np.abs(impulse[:, 1])
                   <= friction * impulse[:, 2] + 1.0e-12)
        )
        first = (impulse.copy(), after.copy(), gap_after.copy())
        self.generic.solve_positive_reference_compliant_contact_impulse(*arguments)
        np.testing.assert_array_equal(impulse, first[0])
        np.testing.assert_array_equal(after, first[1])
        np.testing.assert_array_equal(gap_after, first[2])

        invalid_max = impedance_max.copy()
        invalid_max[-1] = 1.0
        unchanged_impulse = np.full_like(impulse, 7.0)
        unchanged_after = np.full_like(after, 8.0)
        unchanged_gap = np.full_like(gap_after, 9.0)
        with self.assertRaisesRegex(ValueError, "positive reference"):
            self.generic.solve_positive_reference_compliant_contact_impulse(
                gap,
                velocity,
                acceleration,
                delassus,
                upper,
                friction,
                mass,
                time_constant,
                damping_ratio,
                impedance_min,
                invalid_max,
                impedance_width,
                midpoint,
                power,
                0.002,
                0.005,
                16,
                1,
                2,
                unchanged_impulse,
                unchanged_after,
                unchanged_gap,
            )
        np.testing.assert_array_equal(unchanged_impulse, 7.0)
        np.testing.assert_array_equal(unchanged_after, 8.0)
        np.testing.assert_array_equal(unchanged_gap, 9.0)

    def test_coupled_positive_reference_distribution_is_atomic_and_allocation_free(self) -> None:
        contacts = self.generic.contact_count()
        self.assertEqual(contacts, 2)
        gap = np.full(contacts, -0.001, np.float64)
        velocity = np.zeros((contacts, 3), np.float64)
        acceleration = np.zeros_like(velocity)
        delassus = np.eye(3 * contacts, dtype=np.float64)
        delassus[2, 5] = 0.5
        delassus[5, 2] = 0.5
        upper = np.asarray([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]], np.float64)
        friction = np.zeros(contacts, np.float64)
        parameter = lambda value: np.full(contacts, value, np.float64)
        impulse = np.empty((contacts, 3), np.float64)
        after = np.empty((contacts, 3), np.float64)
        gap_after = np.empty(contacts, np.float64)
        arguments = (
            gap,
            velocity,
            acceleration,
            delassus,
            upper,
            friction,
            parameter(0.02),
            parameter(1.0),
            parameter(0.8),
            parameter(0.8),
            parameter(0.001),
            parameter(0.5),
            parameter(2.0),
            0.002,
            0.001,
            1,
            32,
            0,
            0,
            impulse,
            after,
            gap_after,
        )
        timing = self.generic.solve_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_allclose(after[:, 2], after[0, 2], rtol=0.0, atol=1.0e-12)
        np.testing.assert_allclose(
            impulse[:, 2], 2.0 * after[:, 2] / 3.0, rtol=0.0, atol=1.0e-12
        )
        first = (impulse.copy(), after.copy(), gap_after.copy())
        self.generic.solve_coupled_positive_reference_compliant_contact_impulse(*arguments)
        np.testing.assert_array_equal(impulse, first[0])
        np.testing.assert_array_equal(after, first[1])
        np.testing.assert_array_equal(gap_after, first[2])

        unchanged_impulse = np.full_like(impulse, 7.0)
        unchanged_after = np.full_like(after, 8.0)
        unchanged_gap = np.full_like(gap_after, 9.0)
        invalid = (*arguments[:16], 0, *arguments[17:19], unchanged_impulse, unchanged_after, unchanged_gap)
        with self.assertRaisesRegex(ValueError, "coupled positive reference"):
            self.generic.solve_coupled_positive_reference_compliant_contact_impulse(*invalid)
        np.testing.assert_array_equal(unchanged_impulse, 7.0)
        np.testing.assert_array_equal(unchanged_after, 8.0)
        np.testing.assert_array_equal(unchanged_gap, 9.0)

    def test_model_coupled_positive_reference_refreshes_state_without_allocation(self) -> None:
        contacts = self.generic.contact_count()
        dof = self.generic.joint_dof()
        generalized_dof = self.generic.generalized_dof()
        generalized_velocity = np.zeros(generalized_dof, np.float64)
        generalized_velocity[5] = -10.0
        generalized_free_acceleration = np.zeros(generalized_dof, np.float64)
        generalized_free_acceleration[5] = -9.81
        contact_points = self.points.copy()
        contact_points[:, 2] = 0.02
        point_free_acceleration = np.zeros((contacts, 3), np.float64)
        point_free_acceleration[:, 2] = -9.81
        upper = np.full((contacts, 3), 100.0, np.float64)
        parameter = lambda value: np.full(contacts, value, np.float64)
        impulse = np.empty((contacts, 3), np.float64)
        velocity_after = np.empty((contacts, 3), np.float64)
        gap_after = np.empty(contacts, np.float64)
        root_after = np.empty(3, np.float64)
        quaternion_after = np.empty(4, np.float64)
        q_after = np.empty(dof, np.float64)
        generalized_velocity_after = np.empty(generalized_dof, np.float64)
        arguments = (
            self.root,
            self.quaternion,
            self.q,
            generalized_velocity,
            generalized_free_acceleration,
            contact_points,
            self.bases,
            np.zeros(contacts, np.float64),
            point_free_acceleration,
            upper,
            parameter(0.5),
            parameter(0.02),
            parameter(1.0),
            parameter(0.8),
            parameter(0.96),
            parameter(0.001),
            parameter(0.5),
            parameter(2.0),
            np.asarray([0.0, 0.0, 1.0], np.float64),
            0.0,
            0.002,
            0.005,
            5,
            8,
            8,
            0,
            1,
            impulse,
            velocity_after,
            gap_after,
            root_after,
            quaternion_after,
            q_after,
            generalized_velocity_after,
        )
        timing = self.generic.solve_model_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertTrue(np.all(np.isfinite(impulse)))
        self.assertTrue(np.any(impulse[:, 2] > 0.0))
        self.assertLess(root_after[2], self.root[2])
        self.assertAlmostEqual(np.linalg.norm(quaternion_after), 1.0)
        first = tuple(
            value.copy()
            for value in (
                impulse,
                velocity_after,
                gap_after,
                root_after,
                quaternion_after,
                q_after,
                generalized_velocity_after,
            )
        )
        self.generic.solve_model_coupled_positive_reference_compliant_contact_impulse(
            *arguments
        )
        for expected, actual in zip(
            first,
            (
                impulse,
                velocity_after,
                gap_after,
                root_after,
                quaternion_after,
                q_after,
                generalized_velocity_after,
            ),
            strict=True,
        ):
            np.testing.assert_array_equal(actual, expected)

        unchanged = [
            np.full_like(impulse, 7.0),
            np.full_like(velocity_after, 8.0),
            np.full_like(gap_after, 9.0),
            np.full_like(root_after, 10.0),
            np.full_like(quaternion_after, 11.0),
            np.full_like(q_after, 12.0),
            np.full_like(generalized_velocity_after, 13.0),
        ]
        invalid = (*arguments[:22], 0, *arguments[23:27], *unchanged)
        with self.assertRaisesRegex(ValueError, "model coupled positive reference"):
            self.generic.solve_model_coupled_positive_reference_compliant_contact_impulse(
                *invalid
            )
        for expected, actual in zip(range(7, 14), unchanged, strict=True):
            np.testing.assert_array_equal(actual, float(expected))

    def test_terminal_state_batch_is_generic_atomic_and_allocation_free(self) -> None:
        joints = self.generic.joint_dof()
        states = np.asarray(
            [
                [0.3, -0.2, 0.05, -0.03, 0.2, -0.1],
                [0.3, -0.2, 0.05, -0.03, 1.2, -0.8],
            ],
            np.float64,
        )
        q = np.zeros(joints, np.float64)
        joint_velocity = np.zeros((2, joints), np.float64)
        lower = np.full(joints, -1.0, np.float64)
        upper = np.full(joints, 1.0, np.float64)
        velocity_limit = np.full(joints, 2.0, np.float64)
        available = np.ones(2, np.uint8)
        root_acceleration = np.zeros((2, 2), np.float64)
        joint_acceleration = np.zeros((2, joints), np.float64)
        effort = np.zeros(2, np.float64)
        diagnostics = np.empty((2, 17), np.float64)
        timing = self.generic.score_terminal_impact_state_batch(
            states,
            q,
            joint_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        self.assertEqual(timing[1:], (0, 0))
        names = tuple(self.generic.terminal_impact_state_diagnostic_names)
        rate = names.index("terminal_angular_rate_rad_s")
        self.assertGreater(diagnostics[1, rate], diagnostics[0, rate])

        unchanged = np.full((2, 17), 7.0, np.float64)
        invalid_states = states.copy()
        invalid_states[-1, 0] = -0.1
        with self.assertRaisesRegex(ValueError, "state row"):
            self.generic.score_terminal_impact_state_batch(
                invalid_states,
                q,
                joint_velocity,
                lower,
                upper,
                velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                unchanged,
            )
        np.testing.assert_array_equal(unchanged, 7.0)

    def test_terminal_state_box_is_atomic_allocation_free_and_replayable(self) -> None:
        joints = self.generic.joint_dof()
        rows = 2
        root_lower = np.asarray(
            [
                [0.20, -0.8, -0.15, -0.10, -1.2, -0.9],
                [0.16, -0.4, -0.08, -0.06, -0.7, -0.5],
            ],
            np.float64,
        )
        root_upper = np.asarray(
            [
                [0.28, 0.2, 0.20, 0.18, 1.1, 0.8],
                [0.24, 0.3, 0.16, 0.14, 0.9, 0.7],
            ],
            np.float64,
        )
        joint_position_lower_state = np.full((rows, joints), -0.15, np.float64)
        joint_position_upper_state = np.full((rows, joints), 0.15, np.float64)
        joint_velocity_lower = np.full((rows, joints), -0.5, np.float64)
        joint_velocity_upper = np.full((rows, joints), 0.7, np.float64)
        joint_position_lower = np.full(joints, -1.0, np.float64)
        joint_position_upper = np.full(joints, 1.0, np.float64)
        joint_velocity_limit = np.full(joints, 2.5, np.float64)
        available = np.ones(rows, np.uint8)
        root_acceleration = np.asarray([[4.0, -3.0], [-2.0, 1.0]], np.float64)
        joint_acceleration = np.zeros((rows, joints), np.float64)
        effort = np.zeros(rows, np.float64)
        diagnostics = np.empty((rows, 17), np.float64)
        timing = self.generic.score_terminal_impact_state_box_batch(
            root_lower,
            root_upper,
            joint_position_lower_state,
            joint_position_upper_state,
            joint_velocity_lower,
            joint_velocity_upper,
            joint_position_lower,
            joint_position_upper,
            joint_velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            diagnostics,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertTrue(np.all(np.isfinite(diagnostics)))
        replay = np.empty_like(diagnostics)
        replay_timing = self.generic.score_terminal_impact_state_box_batch(
            root_lower,
            root_upper,
            joint_position_lower_state,
            joint_position_upper_state,
            joint_velocity_lower,
            joint_velocity_upper,
            joint_position_lower,
            joint_position_upper,
            joint_velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            replay,
        )
        self.assertEqual(replay_timing[1:], (0, 0))
        np.testing.assert_array_equal(diagnostics, replay)

        invalid_upper = root_upper.copy()
        invalid_upper[-1, 0] = root_lower[-1, 0] - 0.1
        unchanged = np.full_like(diagnostics, 7.0)
        with self.assertRaisesRegex(ValueError, "state box row 1"):
            self.generic.score_terminal_impact_state_box_batch(
                root_lower,
                invalid_upper,
                joint_position_lower_state,
                joint_position_upper_state,
                joint_velocity_lower,
                joint_velocity_upper,
                joint_position_lower,
                joint_position_upper,
                joint_velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                unchanged,
            )
        np.testing.assert_array_equal(unchanged, 7.0)

    def test_complete_terminal_state_box_bounds_points_and_is_atomic(self) -> None:
        joints = self.generic.joint_dof()
        rows = 2
        root_lower = np.asarray(
            [
                [0.10, -0.8, -0.20, -0.15, -1.2, -0.9],
                [0.08, -1.0, -0.15, -0.25, -1.5, -0.7],
            ],
            np.float64,
        )
        root_upper = np.asarray(
            [
                [0.30, 0.2, 0.25, 0.18, 1.4, 1.1],
                [0.25, 0.1, 0.20, 0.15, 1.2, 1.3],
            ],
            np.float64,
        )
        position_lower_state = np.full((rows, joints), -0.2, np.float64)
        position_upper_state = np.full((rows, joints), 0.3, np.float64)
        velocity_lower_state = np.full((rows, joints), -0.5, np.float64)
        velocity_upper_state = np.full((rows, joints), 0.7, np.float64)
        lower = np.full(joints, -1.0, np.float64)
        upper = np.full(joints, 1.0, np.float64)
        velocity_limit = np.full(joints, 2.0, np.float64)
        available = np.ones(rows, np.uint8)
        root_acceleration = np.asarray([[3.0, -2.0], [-1.0, 4.0]], np.float64)
        joint_acceleration = np.zeros((rows, joints), np.float64)
        effort = np.zeros(rows, np.float64)
        bounds = np.empty((rows, 17), np.float64)
        timing = self.generic.score_terminal_impact_state_box_batch(
            root_lower,
            root_upper,
            position_lower_state,
            position_upper_state,
            velocity_lower_state,
            velocity_upper_state,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            bounds,
        )
        self.assertEqual(timing[1:], (0, 0))

        point_state = 0.5 * (root_lower + root_upper)
        point_position = 0.5 * (position_lower_state[0] + position_upper_state[0])
        point_velocity = 0.5 * (velocity_lower_state + velocity_upper_state)
        points = np.empty_like(bounds)
        self.generic.score_terminal_impact_state_batch(
            point_state,
            point_position,
            point_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            points,
        )
        names = tuple(self.generic.terminal_impact_state_diagnostic_names)
        for name in (
            "vertical_specific_impact_energy_j_kg",
            "terminal_tilt_rad",
            "terminal_angular_rate_rad_s",
            "maximum_terminal_joint_velocity_utilization",
            "impact_speed_pressure",
            "joint_position_pressure",
            "maximum_terminal_harm_pressure",
            "aggregate_score",
        ):
            coordinate = names.index(name)
            self.assertTrue(np.all(bounds[:, coordinate] >= points[:, coordinate]))
        headroom = names.index("minimum_terminal_joint_headroom_fraction")
        self.assertTrue(np.all(bounds[:, headroom] <= points[:, headroom]))

        invalid_position_upper = position_upper_state.copy()
        invalid_position_upper[-1, -1] = position_lower_state[-1, -1] - 0.1
        unchanged = np.full_like(bounds, 7.0)
        with self.assertRaisesRegex(ValueError, "state box row 1"):
            self.generic.score_terminal_impact_state_box_batch(
                root_lower,
                root_upper,
                position_lower_state,
                invalid_position_upper,
                velocity_lower_state,
                velocity_upper_state,
                lower,
                upper,
                velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                unchanged,
            )
        np.testing.assert_array_equal(unchanged, 7.0)

    def test_terminal_velocity_box_bounds_points_and_is_atomic(self) -> None:
        joints = self.generic.joint_dof()
        rows = 2
        root_state = np.asarray(
            [[0.3, 0.05, -0.03], [0.2, -0.08, 0.06]], np.float64
        )
        root_lower = np.asarray(
            [[-0.6, -1.0, -0.8], [-0.9, -1.3, -0.5]], np.float64
        )
        root_upper = np.asarray(
            [[0.4, 1.2, 0.9], [0.1, 0.7, 1.1]], np.float64
        )
        q = np.zeros(joints, np.float64)
        joint_lower_velocity = np.full((rows, joints), -0.5, np.float64)
        joint_upper_velocity = np.full((rows, joints), 0.7, np.float64)
        lower = np.full(joints, -1.0, np.float64)
        upper = np.full(joints, 1.0, np.float64)
        velocity_limit = np.full(joints, 2.0, np.float64)
        available = np.ones(rows, np.uint8)
        root_acceleration = np.asarray([[3.0, -2.0], [-1.0, 4.0]], np.float64)
        joint_acceleration = np.zeros((rows, joints), np.float64)
        effort = np.zeros(rows, np.float64)
        bounds = np.empty((rows, 17), np.float64)
        timing = self.generic.score_terminal_impact_velocity_box_batch(
            root_state,
            root_lower,
            root_upper,
            q,
            joint_lower_velocity,
            joint_upper_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            bounds,
        )
        self.assertEqual(timing[1:], (0, 0))

        point_state = np.column_stack(
            (
                root_state[:, 0],
                0.5 * (root_lower[:, 0] + root_upper[:, 0]),
                root_state[:, 1:],
                0.5 * (root_lower[:, 1:] + root_upper[:, 1:]),
            )
        )
        point_velocity = 0.5 * (joint_lower_velocity + joint_upper_velocity)
        points = np.empty_like(bounds)
        self.generic.score_terminal_impact_state_batch(
            point_state,
            q,
            point_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            points,
        )
        names = tuple(self.generic.terminal_impact_state_diagnostic_names)
        for name in (
            "terminal_tilt_rad",
            "terminal_angular_rate_rad_s",
            "maximum_terminal_joint_velocity_utilization",
            "maximum_terminal_harm_pressure",
            "aggregate_score",
        ):
            coordinate = names.index(name)
            self.assertTrue(np.all(bounds[:, coordinate] >= points[:, coordinate]))
        headroom = names.index("minimum_terminal_joint_headroom_fraction")
        self.assertTrue(np.all(bounds[:, headroom] <= points[:, headroom]))

        invalid_upper = root_upper.copy()
        invalid_upper[-1, 1] = root_lower[-1, 1] - 0.1
        unchanged = np.full_like(bounds, 7.0)
        with self.assertRaisesRegex(ValueError, "velocity box row 1"):
            self.generic.score_terminal_impact_velocity_box_batch(
                root_state,
                root_lower,
                invalid_upper,
                q,
                joint_lower_velocity,
                joint_upper_velocity,
                lower,
                upper,
                velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                unchanged,
            )
        np.testing.assert_array_equal(unchanged, 7.0)

    def test_terminal_velocity_box_selection_is_rust_owned_and_atomic(self) -> None:
        joints = self.generic.joint_dof()
        root_state = np.asarray([0.3, 0.25, 0.20], np.float64)
        root_lower = np.asarray([[-1.0, 2.0, 1.8]] * 3, np.float64)
        root_upper = np.asarray([[-0.8, 2.2, 2.0]] * 3, np.float64)
        q = np.zeros(joints, np.float64)
        joint_lower_velocity = np.full((3, joints), -0.2, np.float64)
        joint_upper_velocity = np.full((3, joints), 0.2, np.float64)
        lower = np.full(joints, -1.0, np.float64)
        upper = np.full(joints, 1.0, np.float64)
        velocity_limit = np.full(joints, 2.0, np.float64)
        available = np.asarray([1, 1, 0], np.uint8)
        root_acceleration = np.asarray(
            [[0.0, 0.0], [-20.0, -18.0], [0.0, 0.0]], np.float64
        )
        joint_acceleration = np.zeros((3, joints), np.float64)
        effort = np.asarray([0.0, 0.1, 0.0], np.float64)
        diagnostics = np.empty((3, 17), np.float64)
        selection = np.empty(6, np.float64)
        timing = self.generic.select_terminal_impact_velocity_box_candidates(
            root_state,
            root_lower,
            root_upper,
            q,
            joint_lower_velocity,
            joint_upper_velocity,
            lower,
            upper,
            velocity_limit,
            available,
            root_acceleration,
            joint_acceleration,
            effort,
            0,
            0.0,
            0.01,
            diagnostics,
            selection,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertEqual(selection[0], 1.0)
        self.assertEqual(selection[1], 0.0)
        self.assertLessEqual(selection[4], 0.0)
        self.assertGreater(selection[5], 0.01)

        invalid_upper = root_upper.copy()
        invalid_upper[2, 1] = root_lower[2, 1] - 0.1
        unchanged_diagnostics = np.full((3, 17), 7.0, np.float64)
        unchanged_selection = np.full(6, 8.0, np.float64)
        with self.assertRaisesRegex(ValueError, "candidate 2"):
            self.generic.select_terminal_impact_velocity_box_candidates(
                root_state,
                root_lower,
                invalid_upper,
                q,
                joint_lower_velocity,
                joint_upper_velocity,
                lower,
                upper,
                velocity_limit,
                available,
                root_acceleration,
                joint_acceleration,
                effort,
                0,
                0.0,
                0.01,
                unchanged_diagnostics,
                unchanged_selection,
            )
        np.testing.assert_array_equal(unchanged_diagnostics, 7.0)
        np.testing.assert_array_equal(unchanged_selection, 8.0)

    def test_paired_terminal_delta_selector_is_atomic_and_allocation_free(self) -> None:
        lower = np.asarray(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [-0.5, -0.2, -0.1, -0.1, -0.1, -0.1],
                [-0.4, -0.3, -0.2, -0.2, -0.2, -0.2],
            ],
            np.float64,
        )
        upper = np.asarray(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [-0.2, -0.1, 0.0, 0.0, 0.0, 0.0],
            ],
            np.float64,
        )
        aggregate_lower = np.asarray([0.0, -0.4, -0.3], np.float64)
        aggregate_upper = np.asarray([0.0, 0.1, -0.1], np.float64)
        available = np.ones(3, np.uint8)
        diagnostics = np.empty((3, 14), np.float64)
        selection = np.empty(6, np.float64)
        timing = self.generic.select_terminal_impact_component_delta_box_candidates(
            lower,
            upper,
            aggregate_lower,
            aggregate_upper,
            available,
            0,
            0.0,
            0.05,
            diagnostics,
            selection,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertEqual(selection[0], 2.0)
        self.assertEqual(selection[1], 0.0)
        self.assertEqual(selection[2], 0.0)
        self.assertAlmostEqual(selection[3], 0.2)
        np.testing.assert_array_equal(diagnostics[:, :6], lower)
        np.testing.assert_array_equal(diagnostics[:, 6:12], upper)

        invalid_upper = upper.copy()
        invalid_upper[2, 4] = lower[2, 4] - 0.1
        unchanged_diagnostics = np.full((3, 14), 7.0, np.float64)
        unchanged_selection = np.full(6, 8.0, np.float64)
        with self.assertRaisesRegex(ValueError, "paired terminal-impact delta"):
            self.generic.select_terminal_impact_component_delta_box_candidates(
                lower,
                invalid_upper,
                aggregate_lower,
                aggregate_upper,
                available,
                0,
                0.0,
                0.05,
                unchanged_diagnostics,
                unchanged_selection,
            )
        np.testing.assert_array_equal(unchanged_diagnostics, 7.0)
        np.testing.assert_array_equal(unchanged_selection, 8.0)

    def test_paired_terminal_delta_hypotheses_preserve_shared_correlation(self) -> None:
        hypotheses = np.zeros((3, 4, 17), np.float64)
        hypotheses[:, :, 0] = 1.0
        # Candidate one improves all six paired components over four shared
        # baseline hypotheses. Candidate two has one positive tilt delta and
        # must therefore remain ineligible at zero regression tolerance.
        for hypothesis in range(4):
            hypotheses[1, hypothesis, 9] = -0.20 - 0.01 * hypothesis
            hypotheses[1, hypothesis, 10] = -0.10
            hypotheses[1, hypothesis, 11] = -0.08
            hypotheses[1, hypothesis, 12] = -0.07
            hypotheses[1, hypothesis, 13] = -0.06
            hypotheses[1, hypothesis, 6] = 0.20 + 0.01 * hypothesis
            hypotheses[1, hypothesis, 16] = -0.30 - 0.01 * hypothesis
            hypotheses[2, hypothesis, 9] = 0.05 if hypothesis == 3 else -0.10
            hypotheses[2, hypothesis, 10] = -0.08
            hypotheses[2, hypothesis, 11] = -0.07
            hypotheses[2, hypothesis, 12] = -0.06
            hypotheses[2, hypothesis, 13] = -0.05
            hypotheses[2, hypothesis, 6] = 0.10
            hypotheses[2, hypothesis, 16] = -0.20
        delta_lower = np.full((3, 6), 7.0, np.float64)
        delta_upper = np.full((3, 6), 7.0, np.float64)
        aggregate_lower = np.full(3, 7.0, np.float64)
        aggregate_upper = np.full(3, 7.0, np.float64)
        selection = np.full(6, 7.0, np.float64)
        timing = self.upkie.select_terminal_impact_delta_hypothesis_envelopes(
            hypotheses,
            0,
            0.0,
            0.05,
            delta_lower,
            delta_upper,
            aggregate_lower,
            aggregate_upper,
            selection,
        )
        self.assertEqual(timing[1:], (0, 0))
        self.assertEqual(selection[0], 1.0)
        self.assertEqual(selection[1], 0.0)
        self.assertAlmostEqual(delta_upper[1, 0], -0.20)
        self.assertAlmostEqual(delta_lower[1, 0], -0.23)
        self.assertAlmostEqual(delta_upper[1, 5], -0.20)
        self.assertAlmostEqual(aggregate_upper[1], -0.30)
        self.assertEqual(selection[2], -0.06)
        self.assertGreater(selection[3], 0.05)

        invalid = hypotheses.copy()
        invalid[2, 3, 16] = np.nan
        unchanged_lower = np.full((3, 6), 8.0, np.float64)
        unchanged_upper = np.full((3, 6), 8.0, np.float64)
        unchanged_aggregate_lower = np.full(3, 8.0, np.float64)
        unchanged_aggregate_upper = np.full(3, 8.0, np.float64)
        unchanged_selection = np.full(6, 8.0, np.float64)
        with self.assertRaisesRegex(ValueError, "paired terminal hypothesis"):
            self.upkie.select_terminal_impact_delta_hypothesis_envelopes(
                invalid,
                0,
                0.0,
                0.05,
                unchanged_lower,
                unchanged_upper,
                unchanged_aggregate_lower,
                unchanged_aggregate_upper,
                unchanged_selection,
            )
        np.testing.assert_array_equal(unchanged_lower, 8.0)
        np.testing.assert_array_equal(unchanged_upper, 8.0)
        np.testing.assert_array_equal(unchanged_aggregate_lower, 8.0)
        np.testing.assert_array_equal(unchanged_aggregate_upper, 8.0)
        np.testing.assert_array_equal(unchanged_selection, 8.0)

    def test_spatial_patch_bound_couples_force_and_moment_to_normal(self) -> None:
        generalized_dof = self.generic.generalized_dof()
        response = np.zeros((generalized_dof, 2, 6), np.float64)
        response[0, 0] = [1.0, -2.0, 0.5, 3.0, -4.0, 0.25]
        witnesses = np.asarray(
            [
                [
                    10.0, 10.0, 1.0,
                    1.0, 1.0, 2.0,
                    0.0, 0.0, 10.0,
                    0.4, 0.2, 0.1, 0.05,
                ],
                [
                    0.0, 0.0, 0.0,
                    1.0, 1.0, 1.0,
                    0.0, 0.0, 0.0,
                    0.4, 0.2, 0.1, 0.05,
                ],
            ],
            np.float64,
        )
        normal = np.empty(2, np.float64)
        lower = np.empty(generalized_dof, np.float64)
        upper = np.empty(generalized_dof, np.float64)
        timing = self.generic.bound_spatial_patch_contact_transition_velocity_jump(
            np.asarray([0.005, 0.010], np.float64),
            0.5,
            witnesses,
            np.zeros(generalized_dof, np.float64),
            np.zeros(generalized_dof, np.float64),
            response,
            normal,
            lower,
            upper,
        )
        self.assertEqual(timing[1:], (0, 0))
        np.testing.assert_allclose(normal, [3.1, 0.0], atol=1e-12, rtol=0.0)
        self.assertAlmostEqual(lower[0], -9.5325)
        self.assertAlmostEqual(upper[0], 11.0825)
        np.testing.assert_array_equal(lower[1:], 0.0)
        np.testing.assert_array_equal(upper[1:], 0.0)


if __name__ == "__main__":
    unittest.main()
