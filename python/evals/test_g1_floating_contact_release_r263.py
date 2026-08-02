from __future__ import annotations

import unittest

from g1_floating_contact_release_r263 import build_result, parse_args


class FloatingContactReleaseR263Test(unittest.TestCase):
    def test_release_latch_prevents_freeze_and_repeat_work(self) -> None:
        result, report = build_result(parse_args([]))
        gates = result["gates"]
        self.assertTrue(gates["baseline_exhibits_state_stall"])
        self.assertTrue(gates["candidate_has_no_state_stall"])
        self.assertTrue(gates["candidate_moves_after_contingency"])
        self.assertTrue(gates["candidate_no_failed_or_infeasible"])
        self.assertTrue(gates["candidate_zero_20ms_misses"])
        self.assertTrue(gates["candidate_p99_under_5ms"])
        self.assertTrue(gates["candidate_release_is_bounded"])
        self.assertFalse(result["functional_admission"])
        self.assertIn("Functional walking admission remains", report)


if __name__ == "__main__":
    unittest.main()
