from __future__ import annotations

import unittest

from g1_contact_law_momentum_holdout import CONTACT_LAWS as R213_LAWS
from g1_coupled_contact_law_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_SPLIT_PROFILE,
    SAMPLE_OFFSETS,
)
from g1_kinetic_impulse_ellipsoid_holdout import FRESH_CONTACT_LAWS as R214_LAWS
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS as R215_LAWS


class G1CoupledContactLawHoldoutTests(unittest.TestCase):
    def test_laws_offsets_and_profile_are_fresh_and_frozen(self) -> None:
        prior = {law.name for law in (*R213_LAWS, *R214_LAWS, *R215_LAWS)}
        self.assertFalse({law.name for law in FRESH_CONTACT_LAWS} & prior)
        self.assertEqual(SAMPLE_OFFSETS, (50_000, 60_000))
        self.assertEqual(FROZEN_SPLIT_PROFILE["root_slope"], 1.5050000000000001)
        self.assertEqual(FROZEN_SPLIT_PROFILE["articulated_slope"], 2.91)


if __name__ == "__main__":
    unittest.main()
