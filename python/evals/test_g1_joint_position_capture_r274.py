from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-joint-position-capture-r274/g1-joint-position-capture-metrics.json"


class JointPositionCaptureR274Test(unittest.TestCase):
    def test_mechanism_is_causal_but_profile_is_rejected(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        contract = metrics["source_contract"]
        self.assertTrue(contract["dormant_common_non_timing_arrays_equal"])
        self.assertEqual(metrics["profiles"]["r274_dormant"]["active_ticks"], 0)
        baseline = metrics["profiles"]["r273_baseline"]
        for name, candidate in metrics["profiles"].items():
            if not name.startswith("r274_w"):
                continue
            self.assertTrue(candidate["finite_state"])
            self.assertGreater(candidate["active_ticks"], 0)
            self.assertEqual(
                contract["candidate_activation_prefix_semantic_differences"][name],
                [],
            )
            self.assertIsNotNone(candidate["critical_joint_first_lower_limit_tick"])
            self.assertLess(candidate["first_release_tick"], baseline["first_release_tick"])


if __name__ == "__main__":
    unittest.main()
