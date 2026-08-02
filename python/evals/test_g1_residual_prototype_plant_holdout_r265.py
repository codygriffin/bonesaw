from __future__ import annotations

import json
import pathlib
import unittest


METRICS = pathlib.Path(
    "benchmarks/results/g1-residual-prototype-plant-holdout-r265/"
    "g1-residual-prototype-plant-holdout-metrics.json"
)


class ResidualPrototypePlantHoldoutR265Test(unittest.TestCase):
    def test_exactly_once_holdout_is_retained_as_rejected_evidence(self) -> None:
        result = json.loads(METRICS.read_text())
        self.assertEqual(
            result["revision"], "g1-residual-prototype-plant-holdout-r265"
        )
        self.assertTrue(result["source_frozen_and_immutable"])
        self.assertTrue(result["fresh_laws_and_offsets_declared_before_labels"])
        self.assertTrue(result["no_refit_or_widening"])
        self.assertTrue(result["holdout_consumed"])
        self.assertFalse(result["repeat_holdout_allowed"])
        self.assertTrue(result["mechanism_passed"])
        self.assertFalse(result["profile_transferred"])
        self.assertFalse(result["authority_admitted"])

        self.assertEqual(result["samples"], 96)
        self.assertEqual(result["physics_steps"], 1_440)
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["plant_actions"], 0)
        self.assertEqual(result["distance_supported_rows"], 88)
        self.assertEqual(result["distance_rejected_rows"], 8)
        self.assertEqual(result["selected_counts"], {"0": 95, "1": 1, "2": 0})
        self.assertEqual(result["nonzero_actions"], 1)
        self.assertEqual(
            result["strict_nonregressing_improving_nonzero_actions"], 0
        )
        self.assertEqual(result["selected_component_regression_rows"], 1)
        self.assertEqual(result["selected_aggregate_regression_rows"], 0)
        self.assertFalse(result["all_supported_candidate_boxes_covered"])
        self.assertTrue(result["semantic_repeat"])
        self.assertTrue(result["zero_profile_rust_allocation"])
        self.assertEqual(result["mujoco_warning_count"], 0)

        self.assertEqual(
            [law["offset"] for law in result["laws"]], [330_000, 340_000]
        )
        self.assertEqual(
            [law["distance_supported_rows"] for law in result["laws"]], [43, 45]
        )


if __name__ == "__main__":
    unittest.main()
