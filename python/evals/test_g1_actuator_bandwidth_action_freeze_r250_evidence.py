from __future__ import annotations

import json
import pathlib
import unittest

from g1_actuator_bandwidth_action_freeze_r250 import PROFILE_CASES, REVISION


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-actuator-bandwidth-action-freeze-metrics.json"


class G1ActuatorBandwidthActionFreezeR250EvidenceTests(unittest.TestCase):
    def test_only_r249_diagnostic_reference_profile_is_audited(self) -> None:
        self.assertEqual(
            PROFILE_CASES,
            (("bandwidth_25hz_slew_1000_nm_s", 25.0, 1_000.0),),
        )

    def test_archived_spent_state_audit_rejects_every_action(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertTrue(metrics["design_audit_not_holdout"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 2 * 2 * 48 * 3 * 5)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertFalse(metrics["action_profile_frozen_for_fresh_holdout"])
        self.assertIsNone(metrics["selected_profile"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(len(metrics["cases"]), 2)
        for case in metrics["cases"]:
            self.assertEqual(
                case["unthresholded_selected_counts"],
                {
                    "zero_effort": 96,
                    "bandwidth_zero_wbc": 0,
                    "bandwidth_third_law": 0,
                },
            )
            self.assertIsNone(case["passing_threshold"])


if __name__ == "__main__":
    unittest.main()
