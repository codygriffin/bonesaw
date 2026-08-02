#!/usr/bin/env python3

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / "benchmarks/results/g1-bounded-contact-continuation-r269"
    / "g1-bounded-contact-continuation-metrics.json"
)


class BoundedContactContinuationR269Test(unittest.TestCase):
    def test_retained_mechanism_and_rejected_profile(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)

        baseline = metrics["profiles"]["r268_low_gain_baseline"]
        candidate = metrics["profiles"]["r269_bounded_continuation"]
        self.assertEqual(baseline["first_contingency_tick"], 863)
        self.assertEqual(candidate["first_contingency_tick"], 863)
        self.assertEqual(baseline["first_global_release_tick"], 869)
        self.assertEqual(candidate["first_global_release_tick"], 888)

        hold = metrics["hold_contract"]
        self.assertEqual(hold["statuses"], [8] * 12)
        self.assertTrue(hold["represented_state_bitwise_constant"])
        self.assertTrue(hold["aggregate_wbc_tick_cap_respected"])
        self.assertEqual(hold["per_wbc_tick_feasibility_sweeps"], [16] * 12)

        handoff = metrics["scheduled_handoff_contract"]
        self.assertEqual(handoff["tick"], 869)
        self.assertEqual(handoff["status"], 9)
        self.assertEqual(handoff["support_phase"], [3, 0])
        self.assertGreater(handoff["normal_force_n"], 0.0)
        self.assertLess(handoff["dynamics_residual"], 1.0e-8)
        self.assertLess(handoff["contact_residual"], 1.0e-8)
        self.assertLessEqual(handoff["aggregate_feasibility_sweeps"], 8)

        localized = metrics["localized_handoff_contract"]
        self.assertEqual(localized["tick"], 529)
        self.assertEqual(localized["status"], 9)
        self.assertEqual(localized["support_phase"], [2, 0])
        self.assertGreater(localized["normal_force_n"], 0.0)
        self.assertLess(localized["dynamics_residual"], 1.0e-8)
        self.assertLess(localized["contact_residual"], 1.0e-8)

        self.assertTrue(candidate["global_release"]["fail_closed"])
        self.assertGreater(candidate["root_error_rms_m"], 1.0)
        self.assertGreater(candidate["maximum_root_attitude_error_rad"], 1.5)


if __name__ == "__main__":
    unittest.main()
