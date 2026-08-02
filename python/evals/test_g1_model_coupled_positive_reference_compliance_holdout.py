from __future__ import annotations

import unittest

from g1_model_coupled_positive_reference_compliance_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROFILE,
    SAMPLE_OFFSETS,
)


class G1ModelCoupledPositiveReferenceComplianceHoldoutTests(unittest.TestCase):
    def test_profile_laws_and_offsets_are_frozen(self) -> None:
        self.assertEqual(SAMPLE_OFFSETS, (110_000, 120_000))
        self.assertEqual(FROZEN_PROFILE["state_steps"], 5)
        self.assertEqual(FROZEN_PROFILE["compliance_updates_per_state_step"], 1)
        self.assertEqual(FROZEN_PROFILE["projection_sweeps"], 32)
        self.assertEqual(
            [law.name for law in FRESH_CONTACT_LAWS],
            ["medium_pyramidal_implicitfast", "hard_elliptic_rk4"],
        )
        self.assertLessEqual(
            2.0 * FROZEN_PROFILE["residual_half_width"]["joint_rad_s"], 10.0
        )


if __name__ == "__main__":
    unittest.main()
