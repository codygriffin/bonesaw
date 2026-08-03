#!/usr/bin/env python3
"""Regression gates for the R290 post-transfer Style experiment."""

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-post-transfer-style-budget-r290/g1-post-transfer-style-budget-r290-metrics.json"


class PostTransferStyleBudgetR290Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_replay_is_policy_and_physics_free(self) -> None:
        execution = self.result["execution"]
        self.assertEqual(execution["policy_steps"], 0)
        self.assertEqual(execution["physics_steps"], 0)

    def test_gate_is_causal_and_prefix_exact(self) -> None:
        experiment = self.result["experiment"]
        self.assertEqual(experiment["first_support_tube_clip_tick"], 819)
        self.assertEqual(experiment["pretransfer_inclusive_end_tick"], 819)
        self.assertTrue(experiment["pretransfer_semantic_exact"])
        self.assertEqual(experiment["first_semantic_divergence_tick"], 1152)

    def test_style_exhaustion_is_typed_and_hard_residuals_hold(self) -> None:
        experiment = self.result["experiment"]
        self.assertEqual(experiment["post_transfer_exhaustion_ticks"], [1152])
        self.assertTrue(self.result["verdict"]["hard_residuals_passed"])
        self.assertFalse(self.result["verdict"]["authority_admitted"])

    def test_pinned_repeats_are_semantically_exact(self) -> None:
        for profile in ("unbounded", "post_transfer_style2"):
            timing = self.result["timing"][profile]
            self.assertEqual(len(timing["repeats"]), 5)
            self.assertTrue(timing["all_semantic_exact"])
            self.assertTrue(all(row["semantic_exact"] for row in timing["repeats"]))

    def test_candidate_is_not_promoted(self) -> None:
        verdict = self.result["verdict"]
        self.assertFalse(verdict["post_transfer_p99_under_5ms"])
        self.assertFalse(verdict["tracking_non_regressed"])
        self.assertFalse(verdict["promoted"])


if __name__ == "__main__":
    unittest.main()
