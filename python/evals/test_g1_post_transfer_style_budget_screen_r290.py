from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/g1-post-transfer-style-budget-screen-r290"
    / "g1-post-transfer-style-budget-screen-r290-metrics.json"
)


class PostTransferStyleBudgetScreenR290Test(unittest.TestCase):
    def test_causal_mechanism_is_retained_but_profiles_are_rejected(self) -> None:
        result = json.loads(RESULT.read_text())
        self.assertEqual(result["decision"], "RETAIN_MECHANISM_REJECT_PROFILES")
        self.assertEqual(result["execution"]["policy_steps"], 0)
        self.assertEqual(result["execution"]["physics_steps"], 0)
        self.assertTrue(result["checks"]["budget_cannot_exhaust_on_clipping_tick"])
        self.assertTrue(result["checks"]["style3_plus_are_inert_on_common_arrays"])
        self.assertFalse(result["checks"]["style1_all_repeats_below_5ms"])
        self.assertFalse(result["checks"]["style1_preserves_behavior"])
        self.assertFalse(result["checks"]["style2_all_repeats_below_5ms"])
        self.assertFalse(result["checks"]["style2_preserves_behavior"])
        self.assertEqual(result["profiles"]["1"]["first_clip_tick"], 819)
        self.assertEqual(result["profiles"]["1"]["first_budget_exhaustion_tick"], 820)
        self.assertEqual(result["profiles"]["2"]["first_budget_exhaustion_tick"], 1152)
        self.assertEqual(result["baseline"]["release_ticks"][0], 1234)
        self.assertEqual(result["profiles"]["1"]["release_ticks"][0], 875)
        self.assertEqual(result["profiles"]["2"]["release_ticks"][0], 1210)
        self.assertEqual(result["verdict"]["finite_profiles_promoted"], [])
        self.assertFalse(result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
