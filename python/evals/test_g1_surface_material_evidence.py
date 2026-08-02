#!/usr/bin/env python3
"""Contract tests for the R245/R246 freeze and untouched holdout."""

from __future__ import annotations

import json
import pathlib
import unittest

import mujoco

from g1_constraint_rhs_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R241_LAWS,
    SAMPLE_OFFSETS as R241_OFFSETS,
)
from g1_pyramid_edge_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R244_LAWS,
    SAMPLE_OFFSETS as R244_OFFSETS,
)
from g1_pyramid_edge_cross_profile_audit import FORCED_MODEL_CONE_ID
from g1_positive_reference_compliance_audit import law_model_constraint_rhs_integrator_id
from g1_surface_material_cross_integrator_holdout import (
    FRESH_CONTACT_LAWS as R246_LAWS,
    FROZEN_SWEEPS_BY_INTEGRATOR_ID,
    MODEL_CONE_ID,
    SAMPLE_OFFSETS as R246_OFFSETS,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-surface-material-cross-integrator-holdout-r246/g1-surface-material-cross-integrator-holdout-metrics.json"


class G1SurfaceMaterialEvidenceTests(unittest.TestCase):
    def test_fresh_laws_and_offsets_are_disjoint(self) -> None:
        old_names = {law.name for law in (*R241_LAWS, *R244_LAWS)}
        self.assertTrue(old_names.isdisjoint(law.name for law in R246_LAWS))
        self.assertTrue(set((*R241_OFFSETS, *R244_OFFSETS)).isdisjoint(R246_OFFSETS))

    def test_crossed_profile_is_explicit_and_frozen(self) -> None:
        self.assertEqual(FORCED_MODEL_CONE_ID, 2)
        self.assertEqual(MODEL_CONE_ID, 2)
        self.assertEqual(
            [law_model_constraint_rhs_integrator_id(law) for law in R246_LAWS],
            [0, 4],
        )
        self.assertEqual(FROZEN_SWEEPS_BY_INTEGRATOR_ID, {0: 64, 4: 32})
        self.assertTrue(
            all(law.cone == int(mujoco.mjtCone.mjCONE_PYRAMIDAL) for law in R246_LAWS)
        )

    def test_archived_fresh_holdout_passes_without_authority(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["fresh_laws_and_state_offsets"])
        self.assertTrue(metrics["profile_frozen_before_holdout_labels"])
        self.assertTrue(metrics["predictor_received_causal_arrays_only"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["strict_holdout_passed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual([row["sample_coverage"] for row in metrics["results"]], [1.0, 1.0])
        self.assertEqual([row["exact_active_sets"] for row in metrics["results"]], [48, 48])


if __name__ == "__main__":
    unittest.main()
