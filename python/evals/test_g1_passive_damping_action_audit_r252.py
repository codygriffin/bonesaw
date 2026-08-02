from __future__ import annotations

import json
import pathlib
import unittest

from g1_passive_damping_action_audit_r252 import (
    DAMPING_GAINS_NM_PER_RAD_S,
    MAXIMUM_POSITIVE_POWER_W,
    REVISION,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-passive-damping-action-audit-metrics.json"


class G1PassiveDampingActionAuditR252Tests(unittest.TestCase):
    def test_archived_mechanism_passes_but_action_does_not(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(tuple(metrics["damping_gains_nm_per_rad_s"]), DAMPING_GAINS_NM_PER_RAD_S)
        self.assertEqual(metrics["maximum_positive_mechanical_power_w"], MAXIMUM_POSITIVE_POWER_W)
        self.assertEqual(metrics["samples"], 96)
        self.assertEqual(metrics["physics_steps"], 4_320)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertIsNone(metrics["selected_gain_for_fresh_plant"])
        self.assertFalse(metrics["fresh_plant_action_selected"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(len(metrics["gains"]), 4)
        for gain in metrics["gains"]:
            self.assertEqual(gain["maximum_positive_mechanical_power_w"], 0.0)
            self.assertTrue(gain["zero_rust_allocation"])
            self.assertEqual(gain["semantic_repeat_samples"], 96)
            self.assertEqual(gain["mujoco_warning_count"], 0)
            self.assertFalse(gain["strict_action_passed"])


if __name__ == "__main__":
    unittest.main()
