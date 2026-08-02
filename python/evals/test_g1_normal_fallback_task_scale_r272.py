from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-normal-fallback-task-scale-r272/g1-normal-fallback-task-scale-metrics.json"


class NormalFallbackTaskScaleR272Test(unittest.TestCase):
    def test_partial_mechanism_is_rejected_and_default_preserved(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertTrue(metrics["source_contract"]["dormant_non_timing_arrays_equal"])
        baseline = metrics["profiles"]["r270_baseline"]
        for name in ("r272_scale0", "r272_scale025", "r272_scale05"):
            candidate = metrics["profiles"][name]
            self.assertTrue(candidate["finite_state"])
            self.assertEqual(
                metrics["source_contract"]["candidate_prefix_semantic_differences"][name],
                [],
            )
            self.assertEqual(candidate["first_fallback_tick"], baseline["first_fallback_tick"])
            self.assertLess(candidate["first_release_tick"], baseline["first_release_tick"])
            self.assertEqual(
                candidate["critical_joint_velocity_min_rad_s"],
                baseline["critical_joint_velocity_min_rad_s"],
            )
            self.assertEqual(
                candidate["critical_joint_first_lower_limit_tick"],
                baseline["critical_joint_first_lower_limit_tick"],
            )


if __name__ == "__main__":
    unittest.main()
