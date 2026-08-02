from __future__ import annotations

import unittest

from g1_coupled_positive_reference_compliance_audit import (
    PROFILES,
    QUERY_DEADLINE_NS,
    SUBSTEP_REFINEMENT_GATE_FRACTION,
    SWEEP_REFINEMENT_GATE_FRACTION,
)


class G1CoupledPositiveReferenceComplianceAuditTests(unittest.TestCase):
    def test_construction_grid_is_fixed_and_bounded(self) -> None:
        self.assertEqual(len(PROFILES), 24)
        self.assertEqual({profile.substeps for profile in PROFILES}, {16, 32, 64})
        self.assertEqual(
            {profile.projection_sweeps for profile in PROFILES},
            {1, 2, 4, 8, 16, 32, 64, 128},
        )
        self.assertEqual(len({profile.name for profile in PROFILES}), len(PROFILES))
        self.assertEqual(SWEEP_REFINEMENT_GATE_FRACTION, 0.02)
        self.assertEqual(SUBSTEP_REFINEMENT_GATE_FRACTION, 0.20)
        self.assertEqual(QUERY_DEADLINE_NS, 5_000_000.0)


if __name__ == "__main__":
    unittest.main()
