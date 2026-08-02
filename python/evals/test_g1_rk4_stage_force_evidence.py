#!/usr/bin/env python3
"""Contract tests for the R237/R238 stage-force evidence boundary."""

from __future__ import annotations

import unittest

import mujoco

from g1_generalized_rk4_holdout import (
    FRESH_CONTACT_LAWS as R233_LAWS,
    SAMPLE_OFFSETS as R233_OFFSETS,
)
from g1_rk4_stage_force_convergence_audit import PROJECTION_SWEEPS
from g1_rk4_stage_force_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R238_LAWS,
    FROZEN_PROJECTION_SWEEPS,
    SAMPLE_OFFSETS as R238_OFFSETS,
)
from g1_constraint_rhs_convergence_audit import PROJECTION_SWEEPS as R240_SWEEPS
from g1_constraint_rhs_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R241_LAWS,
    FROZEN_PROJECTION_SWEEPS as R241_FROZEN_SWEEPS,
    SAMPLE_OFFSETS as R241_OFFSETS,
)
from g1_pyramid_edge_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R244_LAWS,
    SAMPLE_OFFSETS as R244_OFFSETS,
)
from g1_positive_reference_compliance_audit import (
    law_model_constraint_rhs_integrator_id,
    law_model_edge_cone_id,
    law_model_predicted_gap_activation_integrator_id,
    law_model_stage_force_integrator_id,
)


class G1Rk4StageForceEvidenceTests(unittest.TestCase):
    def test_holdout_uses_prediction_frozen_work(self) -> None:
        self.assertIn(FROZEN_PROJECTION_SWEEPS, PROJECTION_SWEEPS)
        self.assertEqual(FROZEN_PROJECTION_SWEEPS, 64)

    def test_holdout_laws_and_offsets_are_disjoint_from_spent_source(self) -> None:
        self.assertTrue(
            {law.name for law in R238_LAWS}.isdisjoint(
                {law.name for law in R233_LAWS}
            )
        )
        self.assertTrue(set(R238_OFFSETS).isdisjoint(R233_OFFSETS))

    def test_cross_integrator_rows_select_implicit_and_stage_force_abis(self) -> None:
        self.assertEqual(
            [law.integrator for law in R238_LAWS],
            [
                int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
                int(mujoco.mjtIntegrator.mjINT_RK4),
            ],
        )
        self.assertEqual(
            [law_model_stage_force_integrator_id(law) for law in R238_LAWS],
            [1, 4],
        )
        self.assertEqual(
            [law_model_constraint_rhs_integrator_id(law) for law in R238_LAWS],
            [0, 4],
        )

    def test_constraint_rhs_holdout_is_new_and_uses_causally_frozen_work(self) -> None:
        self.assertEqual(R241_FROZEN_SWEEPS, 128)
        self.assertIn(R241_FROZEN_SWEEPS, R240_SWEEPS)
        self.assertTrue(
            {law.name for law in R241_LAWS}.isdisjoint(
                {law.name for law in R238_LAWS}
            )
        )
        self.assertTrue(set(R241_OFFSETS).isdisjoint(R238_OFFSETS))
        self.assertEqual(
            [law_model_constraint_rhs_integrator_id(law) for law in R241_LAWS],
            [0, 4],
        )

    def test_predicted_gap_activation_candidate_is_opt_in(self) -> None:
        self.assertEqual(
            [law_model_predicted_gap_activation_integrator_id(law) for law in R241_LAWS],
            [6, 4],
        )

    def test_pyramid_edge_followup_is_new_and_diagnostic_only(self) -> None:
        self.assertTrue(
            {law.name for law in R244_LAWS}.isdisjoint(
                {law.name for law in R241_LAWS}
            )
        )
        self.assertTrue(set(R244_OFFSETS).isdisjoint(R241_OFFSETS))
        self.assertEqual(
            [law_model_constraint_rhs_integrator_id(law) for law in R244_LAWS],
            [0, 4],
        )
        self.assertEqual([law_model_edge_cone_id(law) for law in R244_LAWS], [2, 2])


if __name__ == "__main__":
    unittest.main()
