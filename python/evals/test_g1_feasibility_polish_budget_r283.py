from __future__ import annotations

import json
import pathlib
import unittest

from g1_feasibility_polish_budget_r283 import validate_result


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-feasibility-polish-budget-r283/g1-feasibility-polish-budget-r283-metrics.json"


class FeasibilityPolishBudgetR283Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_exact_tail_profile_is_retained_without_authority(self) -> None:
        validate_result(self.result)
        self.assertTrue(self.result["semantic_contract"]["all_repeats_exact"])
        self.assertTrue(self.result["verdict"]["profile_retained"])
        self.assertFalse(self.result["verdict"]["default_changed"])
        self.assertFalse(self.result["verdict"]["authority_admitted"])

    def test_tail_is_bounded_but_p99_still_fails(self) -> None:
        self.assertLess(
            self.result["summary"]["candidate_tail_maximum_ms"], 10.0
        )
        self.assertGreater(
            self.result["summary"]["tail_latency_reduction_percent"], 90.0
        )
        self.assertFalse(self.result["verdict"]["p99_gate_passed"])


if __name__ == "__main__":
    unittest.main()
