from __future__ import annotations

import unittest

from g1_contact_law_momentum_holdout import SPATIAL_PATCH_PROFILE
from g1_spatial_patch_transition_holdout import FRESH_CONTACT_LAWS, SAMPLE_OFFSETS


class G1SpatialPatchTransitionHoldoutTests(unittest.TestCase):
    def test_fixture_is_fresh_and_profile_is_physical(self) -> None:
        self.assertEqual(len({law.name for law in FRESH_CONTACT_LAWS}), 2)
        self.assertEqual(SAMPLE_OFFSETS, (30_000, 40_000))
        self.assertEqual(SPATIAL_PATCH_PROFILE["normal_load_scale"], 1.0)
        self.assertEqual(SPATIAL_PATCH_PROFILE["torsion_radius_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
