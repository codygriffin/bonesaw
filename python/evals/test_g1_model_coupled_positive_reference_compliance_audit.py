from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np

EVALS = pathlib.Path(__file__).resolve().parent
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

from g1_model_coupled_positive_reference_compliance_audit import (
    CAUSAL_SOURCE_SUFFIXES,
    grouped_width,
    model_state_tracking_residual,
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

    def test_stateful_score_uses_returned_final_tangent(self) -> None:
        initial = np.asarray([[1.0, -2.0, 0.5, 0.25, -0.75, 0.1]], np.float64)
        acceleration = np.asarray([[2.0, 0.0, -1.0, 0.5, 0.0, 3.0]], np.float64)
        response = np.zeros((1, 6, 1, 3), np.float64)
        response[0, 0, 0, 2] = 4.0
        impulse = np.asarray([[[0.0, 0.0, 0.2]]], np.float64)
        raw = np.asarray([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]], np.float64)
        expected_final = (
            initial
            + raw
            + acceleration * 0.005
            + np.einsum("sdca,sca->sd", response, impulse)
        )
        np.testing.assert_allclose(
            model_state_tracking_residual(
                initial, acceleration, response, impulse, raw, expected_final
            ),
            np.zeros_like(initial),
            atol=2.0e-16,
            rtol=0.0,
        )


if __name__ == "__main__":
    unittest.main()
