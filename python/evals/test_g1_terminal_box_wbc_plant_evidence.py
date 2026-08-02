#!/usr/bin/env python3
"""Contract tests for the rejected R248 fresh plant A/B."""

from __future__ import annotations

import json
import pathlib
import unittest

from g1_surface_material_cross_integrator_holdout import SAMPLE_OFFSETS as R246_OFFSETS
from g1_terminal_box_wbc_plant_ab import (
    FRESH_PLANT_LAWS,
    SAMPLE_OFFSETS,
    SAMPLES_PER_LAW,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/g1-terminal-box-wbc-plant-ab-metrics.json"


class G1TerminalBoxWbcPlantEvidenceTests(unittest.TestCase):
    def test_fresh_laws_offsets_and_sample_count_are_frozen(self) -> None:
        self.assertEqual(SAMPLES_PER_LAW, 48)
        self.assertTrue(set(SAMPLE_OFFSETS).isdisjoint(R246_OFFSETS))
        self.assertEqual(len(set(law.name for law in FRESH_PLANT_LAWS)), 2)
        self.assertTrue(all("r248" in law.name for law in FRESH_PLANT_LAWS))

    def test_archived_failure_remains_fail_closed(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["fresh_laws_and_offsets_frozen_before_outcomes"])
        self.assertTrue(metrics["source_immutable"])
        self.assertEqual(metrics["samples"], 96)
        self.assertEqual(metrics["physics_steps"], 960)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 96)
        self.assertEqual(metrics["wbc_state_local_queries"], 288)
        self.assertEqual(metrics["selector_queries"], 96)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertFalse(metrics["strict_plant_nonregression_passed"])
        self.assertFalse(metrics["action_profile_promoted"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertTrue(all(row["mujoco_warning_count"] == 0 for row in metrics["results"]))
        self.assertTrue(all(row["deadline_passed"] for row in metrics["results"]))
        self.assertTrue(all(row["plant_regression_samples"] > 0 for row in metrics["results"]))


if __name__ == "__main__":
    unittest.main()
