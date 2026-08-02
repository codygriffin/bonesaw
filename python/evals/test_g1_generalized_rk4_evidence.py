from __future__ import annotations

import pathlib
import sys
import unittest

import mujoco

EVALS = pathlib.Path(__file__).resolve().parent
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

from g1_generalized_rk4_convergence_audit import (
    PROJECTION_SWEEPS,
    REFINEMENT_FRACTION_GATE,
    RK4_LAW,
)
from g1_generalized_rk4_holdout import (
    FRESH_CONTACT_LAWS,
    FROZEN_PROJECTION_SWEEPS,
    SAMPLE_OFFSETS,
)
from g1_rk4_final_tangent_localization import foot_wrench
import numpy as np
from g1_positive_reference_compliance_audit import MODEL_INTEGRATOR_IDS


class G1GeneralizedRk4EvidenceTests(unittest.TestCase):
    def test_prediction_only_grid_can_certify_the_selected_work(self) -> None:
        self.assertEqual(PROJECTION_SWEEPS, (1, 2, 4, 8, 16, 32, 64, 128))
        self.assertEqual(REFINEMENT_FRACTION_GATE, 0.02)
        self.assertEqual(FROZEN_PROJECTION_SWEEPS, 64)
        self.assertEqual(RK4_LAW.integrator, int(mujoco.mjtIntegrator.mjINT_RK4))
        self.assertEqual(
            MODEL_INTEGRATOR_IDS,
            {
                "explicit": 0,
                "implicit": 1,
                "exponential_trapezoidal": 2,
                "generalized_rk4": 3,
                "generalized_rk4_stage_force": 4,
            },
        )

    def test_holdout_laws_and_offsets_are_disjoint_and_rk4(self) -> None:
        self.assertEqual(SAMPLE_OFFSETS, (130_000, 140_000))
        self.assertEqual(
            [law.name for law in FRESH_CONTACT_LAWS],
            ["medium_elliptic_rk4", "hard_pyramidal_rk4"],
        )

    def test_foot_wrench_preserves_resultant_and_moment(self) -> None:
        points = np.zeros((1, 8, 3), np.float64)
        points[0, :4, 0] = [-1.0, 1.0, -1.0, 1.0]
        points[0, :4, 1] = [-1.0, -1.0, 1.0, 1.0]
        impulse = np.zeros_like(points)
        impulse[0, 0, 2] = 2.0
        wrench = foot_wrench(points, impulse)
        np.testing.assert_array_equal(wrench[0, 0, 3:], [0.0, 0.0, 2.0])
        np.testing.assert_array_equal(wrench[0, 0, :3], [-2.0, 2.0, 0.0])
        np.testing.assert_array_equal(wrench[0, 1], np.zeros(6))
        self.assertTrue(
            all(
                law.integrator == int(mujoco.mjtIntegrator.mjINT_RK4)
                for law in FRESH_CONTACT_LAWS
            )
        )


if __name__ == "__main__":
    unittest.main()
