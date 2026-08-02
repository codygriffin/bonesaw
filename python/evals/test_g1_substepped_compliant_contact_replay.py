from __future__ import annotations

import unittest

import numpy as np

from g1_substepped_compliant_contact_replay import (
    FROZEN_PROFILE_NAME,
    PROFILES,
    compliant_parameters,
)


class G1SubsteppedCompliantContactReplayTests(unittest.TestCase):
    def test_frozen_profile_is_unique_and_fixed_work(self) -> None:
        matches = [profile for profile in PROFILES if profile.name == FROZEN_PROFILE_NAME]
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].substeps, 128)
        self.assertEqual(matches[0].stiffness_scale, 1.0)
        self.assertEqual(matches[0].damping_scale, 1.0)
        self.assertEqual(matches[0].impulse_cap_scale, 16.0)

    def test_compliant_parameter_mapping_has_physical_scaling(self) -> None:
        mass = np.asarray([1.0, 2.0], np.float64)
        stiffness, damping = compliant_parameters(mass, 0.1, 0.81, 1.0, 1.0)
        np.testing.assert_allclose(stiffness, [81.0, 162.0], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(damping, [18.0, 36.0], rtol=0.0, atol=1e-12)
        with self.assertRaisesRegex(ValueError, "positive finite"):
            compliant_parameters(mass, 0.0, 0.81, 1.0, 1.0)


if __name__ == "__main__":
    unittest.main()
