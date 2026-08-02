from __future__ import annotations

import json
import pathlib
import unittest

from g1_terminal_state_tube_freeze_r259 import REVISION


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-terminal-state-tube-freeze-metrics.json"


class G1TerminalStateTubeFreezeR259Tests(unittest.TestCase):
    def test_complete_state_freeze_is_zero_physics_and_fail_closed(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["source_immutable"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["source"]["zero_rust_allocation"])
        self.assertTrue(metrics["spent_rehearsal"]["zero_rust_allocation"])
        self.assertTrue(metrics["source"]["all_component_boxes_covered"])
        self.assertTrue(metrics["source"]["all_aggregate_boxes_covered"])
        self.assertEqual(metrics["source"]["nonzero_actions"], 0)
        self.assertFalse(metrics["profile_frozen"])
        self.assertFalse(metrics["spent_rehearsal_transferred"])
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
