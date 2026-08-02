from __future__ import annotations

import json
import pathlib
import unittest

from g1_paired_terminal_score_freeze_r253 import (
    CLOSING_SPEED_BINS_M_S,
    COMPONENT_NAMES,
    MINIMUM_COMPONENT_IMPROVEMENT,
    MINIMUM_GROUP_SAMPLES,
    REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-paired-terminal-score-freeze-metrics.json"


class G1PairedTerminalScoreFreezeR253Tests(unittest.TestCase):
    def test_six_component_profile_is_frozen_for_holdout_only(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(tuple(metrics["component_names"]), COMPONENT_NAMES)
        self.assertEqual(len(metrics["component_names"]), 6)
        self.assertEqual(tuple(metrics["closing_speed_bins_m_s"]), CLOSING_SPEED_BINS_M_S)
        self.assertEqual(metrics["minimum_group_samples"], MINIMUM_GROUP_SAMPLES)
        self.assertEqual(
            metrics["minimum_guaranteed_component_improvement"],
            MINIMUM_COMPONENT_IMPROVEMENT,
        )
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertTrue(metrics["design_audit_not_holdout"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["action_profile_frozen_for_fresh_holdout"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(
            metrics["frozen_profile"]["family"],
            "zero_wbc_and_neutral_recovery",
        )

    def test_frozen_family_has_useful_zero_regression_actions(self) -> None:
        metrics = json.loads(METRICS.read_text())
        frozen = next(
            row
            for row in metrics["families"]
            if row["family"] == metrics["frozen_profile"]["family"]
        )
        self.assertEqual(frozen["fallback_rows"], 0)
        self.assertTrue(frozen["source_boxes_cover_all_actual_deltas"])
        self.assertTrue(frozen["zero_rust_allocation"])
        self.assertEqual(frozen["semantic_repeat_samples"], 96)
        self.assertEqual(frozen["nonzero_actions"], 3)
        self.assertEqual(frozen["actual_aggregate_improved_actions"], 3)
        self.assertEqual(frozen["maximum_actual_component_regression"], 0.0)
        self.assertTrue(frozen["strict_profile_passed"])


if __name__ == "__main__":
    unittest.main()
