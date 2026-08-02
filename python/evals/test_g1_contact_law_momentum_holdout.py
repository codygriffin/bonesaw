from __future__ import annotations

import unittest

import numpy as np

from g1_contact_law_momentum_holdout import (
    FOOT_LOCAL_POINTS,
    GROUPED_ACCELERATION_RESERVE,
    RESERVE_FRACTION,
    contact_force_world,
    directional_witnesses,
    grouped_acceleration_reserve,
    group_maximum,
    quaternion_from_rpy,
)


class G1ContactLawMomentumHoldoutTests(unittest.TestCase):
    def test_quaternion_is_unit_and_identity_at_zero(self) -> None:
        np.testing.assert_array_equal(
            quaternion_from_rpy(0.0, 0.0, 0.0),
            np.asarray([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(quaternion_from_rpy(0.2, -0.1, 0.3))), 1.0
        )

    def test_group_maximum_keeps_units_separate(self) -> None:
        self.assertEqual(
            group_maximum(np.asarray([1, -2, 3, 4, -5, 6, 7, -8], np.float64)),
            (3.0, 6.0, 8.0),
        )

    def test_frozen_fixture_has_four_distinct_points_and_physical_reserve(self) -> None:
        self.assertEqual(FOOT_LOCAL_POINTS.shape, (4, 3))
        self.assertEqual(len(np.unique(FOOT_LOCAL_POINTS, axis=0)), 4)
        self.assertGreater(RESERVE_FRACTION, 0.0)
        self.assertLess(RESERVE_FRACTION, 1.0)

    def test_contact_force_rotates_from_contact_rows_to_world(self) -> None:
        class Contact:
            frame = np.asarray(
                [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            )

        world = contact_force_world(Contact(), np.asarray([2.0, 3.0, 4.0, 0, 0, 0]))
        np.testing.assert_array_equal(world, np.asarray([-3.0, 2.0, 4.0]))

    def test_grouped_acceleration_profile_is_frozen_in_bonesaw_order(self) -> None:
        reserve = grouped_acceleration_reserve(10)
        np.testing.assert_array_equal(
            reserve,
            np.asarray(
                [GROUPED_ACCELERATION_RESERVE[0]] * 3
                + [GROUPED_ACCELERATION_RESERVE[1]] * 3
                + [GROUPED_ACCELERATION_RESERVE[2]] * 4
            ),
        )

    def test_directional_witnesses_keep_contact_law_friction(self) -> None:
        velocity = np.asarray([[0.2, -0.1, -0.4], [0.0, 0.3, -0.5]])
        effective_mass = np.asarray([[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]])
        witnesses = directional_witnesses(
            velocity, effective_mass, total_mass=10.0, friction=0.35
        )
        self.assertEqual(witnesses.shape, (2, 10))
        np.testing.assert_array_equal(witnesses[:, 3:6], effective_mass)
        np.testing.assert_array_equal(witnesses[:, 9], [0.35, 0.35])


if __name__ == "__main__":
    unittest.main()
