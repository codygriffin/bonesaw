from __future__ import annotations

import unittest

import numpy as np

from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    grouped_width,
)


class G1ModelCoupledPositiveReferenceComplianceAuditTests(unittest.TestCase):
    def test_selection_view_excludes_completed_labels(self) -> None:
        self.assertEqual(
            CAUSAL_SOURCE_SUFFIXES,
            ("root_height", "prospective_velocity", "contact_points"),
        )

    def test_grouped_width_uses_full_extrema(self) -> None:
        values = np.zeros((2, 9), np.float64)
        values[0, 1] = -0.2
        values[1, 4] = 0.3
        values[1, 8] = -0.4
        self.assertEqual(
            grouped_width(values),
            {
                "root_angular_rad_s": 0.4,
                "root_linear_m_s": 0.6,
                "joint_rad_s": 0.8,
            },
        )


if __name__ == "__main__":
    unittest.main()
