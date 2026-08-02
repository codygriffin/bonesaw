from __future__ import annotations

import json
import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-support-load-floor-r271/g1-support-load-floor-metrics.json"
sys.path.insert(0, str(ROOT / "python/evals"))

from g1_support_load_floor_r271 import dormant_differences, support_load_contract


class SupportLoadFloorR271Test(unittest.TestCase):
    def test_floor_is_causal_but_rejected_and_default_off(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertTrue(metrics["dormant_non_timing_arrays_equal"])
        self.assertEqual(metrics["dormant_semantic_differences"], [])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        baseline = metrics["profiles"]["r270_lower_body_hard_baseline"]
        for name in ("r271_floor005", "r271_floor10", "r271_floor25"):
            candidate = metrics["profiles"][name]
            self.assertLess(candidate["first_fallback_tick"], baseline["first_fallback_tick"])
            self.assertLess(candidate["first_release_tick"], baseline["first_release_tick"])
            self.assertTrue(candidate["finite_state"])
            self.assertGreater(candidate["load_contract"]["checked_rows"], 0)
            self.assertEqual(candidate["load_contract"]["violations"], 0)
            self.assertIsNotNone(candidate["load_contract"]["first_binding_tick"])

    def test_compact_force_slots_follow_active_target_order(self) -> None:
        trace = {
            "status": np.array([0, 0, 5], dtype=np.uint8),
            "support_phase": np.array(
                [[0, 3, 0, 4], [3, 0, 0, 0], [3, 4, 0, 0]], dtype=np.uint8
            ),
            "contact_normal_force": np.array(
                [[5.0] * 8, [10.0] * 4 + [0.0] * 4, [0.0] * 8],
                dtype=np.float64,
            ),
        }
        audit = support_load_contract(
            trace, requested_fraction=0.2, supported_weight_n=200.0
        )
        self.assertEqual(audit["checked_rows"], 3)
        self.assertEqual(audit["violations"], 0)
        self.assertEqual(audit["first_binding_tick"], 0)

    def test_dormant_comparison_excludes_only_timing(self) -> None:
        baseline = {"q": np.array([1.0]), "step_ns": np.array([10], dtype=np.uint64)}
        dormant = {"q": np.array([1.0]), "step_ns": np.array([20], dtype=np.uint64)}
        self.assertEqual(dormant_differences(baseline, dormant), [])
        dormant["q"][0] = 2.0
        self.assertEqual(dormant_differences(baseline, dormant), ["q"])


if __name__ == "__main__":
    unittest.main()
