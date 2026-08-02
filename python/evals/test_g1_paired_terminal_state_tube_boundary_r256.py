from __future__ import annotations

import json
import pathlib
import unittest

from g1_paired_terminal_state_tube_boundary_r256 import (
    CANDIDATES,
    HYPOTHESES,
    POINTS_PER_TUBE,
    REPEAT_CALLS,
    REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/g1-paired-terminal-state-tube-boundary-metrics.json"
)


class G1PairedTerminalStateTubeBoundaryR256Tests(unittest.TestCase):
    def test_direct_paired_boundary_contains_points_without_policy_or_physics(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["candidate_count"], CANDIDATES)
        self.assertEqual(metrics["hypothesis_count"], HYPOTHESES)
        self.assertEqual(metrics["points_per_tube"], POINTS_PER_TUBE)
        self.assertEqual(metrics["repeat_calls"], REPEAT_CALLS)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["semantic_repeat"])
        self.assertTrue(metrics["zero_rust_allocation"])
        self.assertTrue(metrics["exact_zero_baseline"])
        self.assertTrue(metrics["point_containment"])
        self.assertGreaterEqual(metrics["minimum_containment_slack"], -1.0e-12)
        self.assertGreaterEqual(
            metrics["minimum_nonbaseline_containment_slack"], -1.0e-12
        )
        self.assertLess(metrics["batch_timing_ns"]["p99"], 5_000_000.0)
        self.assertFalse(metrics["unsafe_upper_subtraction_used"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertFalse(metrics["profile_frozen"])
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
