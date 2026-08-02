from __future__ import annotations

import json
import pathlib
import unittest

from g1_paired_state_tube_freeze_r257 import REVISION


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-paired-state-tube-freeze-metrics.json"


class G1PairedStateTubeFreezeR257Tests(unittest.TestCase):
    def test_freeze_is_policy_and_physics_free_and_fail_closed(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["source_immutable"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["source"]["zero_rust_allocation"])
        self.assertTrue(metrics["spent_rehearsal"]["zero_rust_allocation"])
        self.assertTrue(metrics["source"]["semantic_repeat"])
        self.assertTrue(metrics["spent_rehearsal"]["semantic_repeat"])
        self.assertLess(metrics["source"]["timing_ns"]["p99"], 5_000_000.0)
        self.assertLess(
            metrics["spent_rehearsal"]["timing_ns"]["p99"], 5_000_000.0
        )
        if metrics["profile_frozen"]:
            self.assertTrue(metrics["source_profile_useful"])
            self.assertTrue(metrics["source"]["all_component_boxes_covered"])
            self.assertTrue(metrics["source"]["all_aggregate_boxes_covered"])
            self.assertGreater(metrics["source"]["nonzero_actions"], 0)
            self.assertEqual(metrics["source"]["selected_component_regression_rows"], 0)
            self.assertEqual(metrics["source"]["selected_aggregate_regression_rows"], 0)
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
