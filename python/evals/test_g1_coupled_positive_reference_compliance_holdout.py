from __future__ import annotations

import unittest

from g1_coupled_positive_reference_compliance_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROFILE,
    SAMPLE_OFFSETS,
)


class G1CoupledPositiveReferenceComplianceHoldoutTests(unittest.TestCase):
    def test_profile_and_fresh_laws_are_frozen(self) -> None:
        self.assertEqual(FROZEN_PROFILE["substeps"], 32)
        self.assertEqual(FROZEN_PROFILE["projection_sweeps"], 32)
        self.assertEqual(SAMPLE_OFFSETS, (90_000, 100_000))
        self.assertEqual(
            [law.name for law in FRESH_CONTACT_LAWS],
            ["soft_pyramidal_euler", "stiff_elliptic_implicitfast"],
        )
        self.assertLessEqual(
            2.0 * FROZEN_PROFILE["residual_half_width"]["joint_rad_s"], 10.0
        )


if __name__ == "__main__":
    unittest.main()
