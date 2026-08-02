from __future__ import annotations

import json
import pathlib
import unittest

from g1_terminal_state_box_boundary_r255 import (
    POINTS_PER_BOX,
    REPEAT_CALLS,
    REVISION,
    ROWS,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/g1-terminal-state-box-boundary-metrics.json"
)


class G1TerminalStateBoxBoundaryR255Tests(unittest.TestCase):
    def test_policy_and_physics_free_boundary_contains_points(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["rows"], ROWS)
        self.assertEqual(metrics["points_per_box"], POINTS_PER_BOX)
        self.assertEqual(metrics["repeat_calls"], REPEAT_CALLS)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["selector_queries"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["semantic_repeat"])
        self.assertTrue(metrics["zero_rust_allocation"])
        self.assertTrue(metrics["point_containment"])
        self.assertGreaterEqual(metrics["minimum_upper_slack"], -1.0e-12)
        self.assertGreaterEqual(metrics["minimum_headroom_slack"], -1.0e-12)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertFalse(metrics["profile_frozen"])
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
