from __future__ import annotations

import unittest

from g1_floating_projection_budget_profile_r262 import build_result, parse_args


class FloatingProjectionBudgetR262Test(unittest.TestCase):
    def test_bounded_profiles_meet_50hz_without_functional_admission(self) -> None:
        args = parse_args([])
        result, report = build_result(args)
        self.assertEqual(result["recommended_cap"], 8)
        self.assertFalse(result["gates"]["unbounded_50hz_budget"])
        self.assertTrue(result["gates"]["bounded_50hz_budget"])
        self.assertTrue(result["gates"]["bounded_no_failed_or_infeasible"])
        self.assertTrue(result["gates"]["bounded_no_contact_release"])
        self.assertFalse(result["gates"]["functional_full_transfer"])
        self.assertIn("authority is not admitted", report)

        rows = {row["label"]: row for row in result["profiles"]}
        self.assertEqual(rows["unbounded"]["deadline_misses"]["20ms"], 281)
        self.assertEqual(rows["cap8"]["deadline_misses"]["20ms"], 0)
        self.assertLess(rows["cap8"]["latency_us"]["p99"], 5_000.0)
        self.assertGreater(rows["cap8"]["status_counts"]["normal_contact_contingency"], 0)
        for label in ("cap8", "cap16", "cap32", "cap64"):
            self.assertTrue(rows[label]["finite_trace"])
            self.assertTrue(rows[label]["no_failed_or_infeasible_ticks"])


if __name__ == "__main__":
    unittest.main()
