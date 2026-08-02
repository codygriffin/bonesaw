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
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/g1-passive-damping-action-audit-metrics.json"
)


class G1PassiveDampingActionAuditR252Tests(unittest.TestCase):
    def test_passivity_audit_is_mechanism_only(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["source_revision"], "g1-actuator-bandwidth-action-freeze-r250")
        self.assertTrue(metrics["design_audit_not_holdout"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertEqual(metrics["samples"], 96)
        self.assertEqual(metrics["physics_steps"], 4320)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["maximum_positive_mechanical_power_w"], MAXIMUM_POSITIVE_POWER_W)
        self.assertIsNone(metrics["selected_gain_for_fresh_plant"])
        self.assertFalse(metrics["fresh_plant_action_selected"])
        self.assertFalse(metrics["authority_admitted"])

    def test_every_declared_gain_repeats_and_stays_power_limited(self) -> None:
        metrics = json.loads(METRICS.read_text())
        gains = metrics["gains"]
        self.assertEqual(
            [row["gain_nm_per_rad_s"] for row in gains],
            list(DAMPING_GAINS_NM_PER_RAD_S),
        )
        for row in gains:
            self.assertLessEqual(
                row["maximum_positive_mechanical_power_w"],
                MAXIMUM_POSITIVE_POWER_W + 1.0e-12,
            )
            self.assertTrue(row["zero_rust_allocation"])
            self.assertEqual(row["semantic_repeat_samples"], 96)
            self.assertEqual(row["mujoco_warning_count"], 0)
            self.assertFalse(row["strict_action_passed"])


if __name__ == "__main__":
    unittest.main()
