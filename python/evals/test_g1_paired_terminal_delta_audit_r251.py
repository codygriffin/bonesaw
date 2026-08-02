from __future__ import annotations

import json
import pathlib
import unittest

from g1_paired_terminal_delta_audit_r251 import REVISION


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / f"benchmarks/results/{REVISION}/g1-paired-terminal-delta-audit-metrics.json"
)
REPLAY = (
    ROOT / f"benchmarks/results/{REVISION}/g1-paired-terminal-delta-audit.npz"
)


class G1PairedTerminalDeltaAuditR251Tests(unittest.TestCase):
    def test_design_audit_is_causal_and_does_not_freeze_authority(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["source_revision"], "g1-actuator-bandwidth-action-freeze-r250")
        self.assertTrue(metrics["design_audit_not_holdout"])
        self.assertTrue(metrics["mechanism_passed"])
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["rust_selector_queries"], 0)
        self.assertFalse(metrics["action_profile_frozen_for_fresh_holdout"])
        self.assertFalse(metrics["authority_admitted"])
        contract = metrics["feature_contract"]
        self.assertFalse(contract["uses_completed_plant_labels_for_features"])
        self.assertTrue(contract["uses_completed_plant_labels_for_spent_fit"])
        self.assertFalse(contract["uses_sample_index"])
        self.assertFalse(contract["uses_contact_label"])

    def test_both_families_have_finite_covered_paired_tubes(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(len(metrics["families"]), 2)
        for family in metrics["families"]:
            self.assertEqual(family["samples"], 96)
            self.assertEqual(len(family["schemes"]), 8)
            self.assertGreater(family["paired_residual_max_abs"], 0.0)
            for row in family["schemes"]:
                self.assertTrue(row["source_rows_covered"])
                self.assertGreater(row["independent_span_max"], 0.0)
                self.assertLessEqual(
                    row["paired_residual_max_span"], row["independent_span_max"]
                )
        self.assertTrue(REPLAY.is_file())


if __name__ == "__main__":
    unittest.main()
