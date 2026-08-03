from __future__ import annotations

import json
import pathlib
import unittest

from g1_low_authority_budget_r285 import validate_result


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-low-authority-budget-r285/g1-low-authority-budget-r285-metrics.json"


class LowAuthorityBudgetR285Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_default_is_exact_and_exhaustion_is_typed(self) -> None:
        validate_result(self.result)
        self.assertTrue(self.result["contract"]["unbounded_default_exact"])
        self.assertTrue(
            self.result["contract"][
                "exhaustion_is_typed_per_final_attempt_and_cumulatively"
            ]
        )

    def test_style_two_is_hard_safe_but_behaviorally_rejected(self) -> None:
        style2 = self.result["profiles"]["style2"]
        self.assertEqual(style2["budget_exhaustion_ticks"], [155, 158])
        self.assertEqual(style2["failed_or_infeasible_ticks"], 0)
        self.assertLess(style2["maximum_dynamics_residual"], 1e-8)
        self.assertFalse(self.result["verdict"]["style2_profile_promoted"])
        self.assertFalse(self.result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
