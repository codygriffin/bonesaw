from __future__ import annotations

import json
import pathlib
import unittest

from g1_paired_terminal_score_plant_holdout_r254 import (
    COMPONENT_NAMES,
    FRESH_PLANT_LAWS,
    REVISION,
    SAMPLE_OFFSETS,
    SOURCE_REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/"
    "g1-paired-terminal-score-plant-holdout-metrics.json"
)


class G1PairedTerminalScorePlantHoldoutR254Tests(unittest.TestCase):
    def test_fresh_no_refit_mechanism_passes_but_profile_is_rejected(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["source_revision"], SOURCE_REVISION)
        self.assertTrue(metrics["source_immutable"])
        self.assertTrue(metrics["fresh_laws_and_state_offsets"])
        self.assertTrue(metrics["no_refit"])
        self.assertEqual(tuple(metrics["component_names"]), COMPONENT_NAMES)
        self.assertEqual(len(COMPONENT_NAMES), 6)
        self.assertEqual(metrics["fresh_group_coverage_samples"], 96)
        self.assertEqual(metrics["physics_steps"], 1_440)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertFalse(metrics["all_candidate_boxes_covered"])
        self.assertFalse(metrics["profile_transferred"])
        self.assertFalse(metrics["authority_admitted"])

    def test_rejection_retains_useful_actions_and_localizes_transfer_misses(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["nonzero_actions"], 5)
        self.assertEqual(metrics["improved_nonzero_actions"], 3)
        self.assertEqual(
            metrics["strict_nonregressing_improved_nonzero_actions"], 1
        )
        self.assertEqual(metrics["selected_component_regression_samples"], 4)
        self.assertEqual(metrics["selected_aggregate_regression_samples"], 2)
        self.assertGreater(metrics["maximum_selected_component_regression"], 2.0)
        self.assertEqual(metrics["semantic_repeat_samples"], 96)
        self.assertTrue(metrics["zero_wbc_rust_allocation"])
        self.assertTrue(metrics["zero_realization_rust_allocation"])
        self.assertTrue(metrics["zero_selector_rust_allocation"])
        self.assertEqual(metrics["mujoco_warning_count"], 0)
        self.assertTrue(metrics["deadline_passed"])
        self.assertEqual(
            [row["law"] for row in metrics["laws"]],
            [law.name for law in FRESH_PLANT_LAWS],
        )
        self.assertEqual(
            [row["sample_offset"] for row in metrics["laws"]],
            list(SAMPLE_OFFSETS),
        )
        for law in metrics["laws"]:
            self.assertFalse(law["all_candidate_boxes_covered"])
            self.assertGreater(
                law["component_box_misses"]["joint_position_pressure"]["rows"],
                0,
            )
            self.assertGreater(
                law["component_box_misses"]["joint_headroom_loss"]["rows"],
                0,
            )
            self.assertGreater(law["aggregate_box_misses"]["rows"], 0)


if __name__ == "__main__":
    unittest.main()
