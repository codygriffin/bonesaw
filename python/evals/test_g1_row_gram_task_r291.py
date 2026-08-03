#!/usr/bin/env python3
"""Regression gates for the rejected R291 row-Gram path."""

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-row-gram-task-r291/g1-row-gram-task-r291-metrics.json"


class RowGramTaskR291Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_policy_and_physics_free(self) -> None:
        self.assertEqual(self.result["execution"]["policy_steps"], 0)
        self.assertEqual(self.result["execution"]["physics_steps"], 0)

    def test_semantic_divergence_is_visible(self) -> None:
        comparison = self.result["comparison"]
        self.assertEqual(comparison["first_divergence_tick"], 0)
        self.assertLess(comparison["semantic_exact_arrays"], comparison["total_arrays"])

    def test_hard_residuals_do_not_hide_tracking_failure(self) -> None:
        verdict = self.result["verdict"]
        self.assertTrue(verdict["hard_residuals_passed"])
        self.assertFalse(verdict["tracking_gate_passed"])
        self.assertFalse(verdict["promoted"])
        self.assertFalse(verdict["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
