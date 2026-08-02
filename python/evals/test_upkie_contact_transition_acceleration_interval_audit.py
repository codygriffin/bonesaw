from __future__ import annotations

import unittest

import numpy as np

import upkie_contact_transition_acceleration_interval_audit as audit


class ContactTransitionAccelerationIntervalAuditTests(unittest.TestCase):
    def test_primary_reserve_only_expands_root_linear_coordinates(self) -> None:
        reserve = audit.acceleration_reserve(
            audit.ACCELERATION_PROFILES[audit.PRIMARY_PROFILE]
        )
        np.testing.assert_array_equal(reserve[:3], np.zeros(3))
        np.testing.assert_array_equal(reserve[3:6], np.full(3, 5.0))
        np.testing.assert_array_equal(reserve[6:], np.zeros(6))
        np.testing.assert_array_equal(2.0 * reserve * 0.005, [0.0] * 3 + [0.05] * 3 + [0.0] * 6)

    def test_fresh_case_is_new_and_only_changes_declared_friction(self) -> None:
        rows = audit.selected_cases()
        names = [row.name for row in rows]
        self.assertEqual(names.count(audit.FRESH_HOLDOUT_NAME), 1)
        fresh = rows[-1]
        source = next(row for row in rows if row.name == "forward_4n_friction_0p03")
        self.assertEqual(fresh.friction, 0.02)
        self.assertEqual(fresh.force_world_n, source.force_world_n)
        self.assertEqual(fresh.duration_s, source.duration_s)
        self.assertEqual(fresh.body, source.body)


if __name__ == "__main__":
    unittest.main()
