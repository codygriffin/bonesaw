from __future__ import annotations

import json
import pathlib
import unittest

from g1_causal_contact_terminal_tube_r256 import (
    CANDIDATES,
    HYPOTHESES,
    REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/g1-causal-contact-terminal-tube-metrics.json"
)


class G1CausalContactTerminalTubeR256Tests(unittest.TestCase):
    def test_causal_shared_hypothesis_mechanism_is_policy_and_physics_free(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["candidate_count"], CANDIDATES)
        self.assertEqual(metrics["hypothesis_count"], HYPOTHESES)
        self.assertEqual(metrics["kinematic_forwards"], 96)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["selector_queries"], 96)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["source_immutable"])
        self.assertTrue(metrics["contact_zero_rust_allocation"])
        self.assertTrue(metrics["state_box_zero_rust_allocation"])
        self.assertTrue(metrics["paired_zero_rust_allocation"])
        self.assertTrue(metrics["semantic_repeat"])
        self.assertTrue(metrics["paired_semantic_repeat"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertGreaterEqual(metrics["terminal_candidate_coverage_fraction"], 0.0)
        self.assertLessEqual(metrics["terminal_candidate_coverage_fraction"], 1.0)
        self.assertGreaterEqual(metrics["upper_component_coverage"], 0.0)
        self.assertLessEqual(metrics["upper_component_coverage"], 1.0)
        self.assertGreaterEqual(metrics["headroom_coverage"], 0.0)
        self.assertLessEqual(metrics["headroom_coverage"], 1.0)
        self.assertFalse(metrics["profile_frozen"])
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
