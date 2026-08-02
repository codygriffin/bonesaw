from __future__ import annotations

import unittest

from g1_contact_law_momentum_holdout import CONTACT_LAWS as R213_LAWS
from g1_coupled_contact_law_holdout import FRESH_CONTACT_LAWS as R218_LAWS
from g1_kinetic_impulse_ellipsoid_holdout import FRESH_CONTACT_LAWS as R214_LAWS
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS as R215_LAWS
from g1_substepped_compliant_contact_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROFILE,
    SAMPLE_OFFSETS,
)


class G1SubsteppedCompliantContactHoldoutTests(unittest.TestCase):
    def test_laws_offsets_and_profile_are_fresh_and_frozen(self) -> None:
        prior = {law.name for law in (*R213_LAWS, *R214_LAWS, *R215_LAWS, *R218_LAWS)}
        names = {law.name for law in FRESH_CONTACT_LAWS}
        self.assertFalse(names & prior)
        self.assertEqual(SAMPLE_OFFSETS, (70_000, 80_000))
        self.assertEqual(FROZEN_PROFILE["substeps"], 128)
        self.assertEqual(FROZEN_PROFILE["stiffness_scale"], 1.0)
        self.assertEqual(FROZEN_PROFILE["damping_scale"], 1.0)
        self.assertEqual(FROZEN_PROFILE["impulse_cap_scale"], 16.0)
        self.assertLess(FROZEN_PROFILE["residual_half_width"]["root_angular_rad_s"] * 2.0, 2.0)
        self.assertLess(FROZEN_PROFILE["residual_half_width"]["root_linear_m_s"] * 2.0, 0.5)
        self.assertLess(FROZEN_PROFILE["residual_half_width"]["joint_rad_s"] * 2.0, 10.0)


if __name__ == "__main__":
    unittest.main()
