#!/usr/bin/env python3
"""Contract tests for the R247 state-local action freeze."""

from __future__ import annotations

import json
import pathlib
import unittest

from g1_terminal_box_wbc_action_audit import (
    ALLOWED_REPLAY_SUFFIXES,
    CANDIDATE_NAMES,
    MAXIMUM_COMPONENT_REGRESSION,
    MINIMUM_COMPONENT_IMPROVEMENT,
)


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-terminal-box-wbc-action-audit-r247/g1-terminal-box-wbc-action-audit-metrics.json"


class G1TerminalBoxWbcActionEvidenceTests(unittest.TestCase):
    def test_candidate_and_source_contract_is_frozen(self) -> None:
        self.assertEqual(
            CANDIDATE_NAMES,
            ("zero_acceleration", "velocity_damping", "neutral_recovery"),
        )
        self.assertEqual(ALLOWED_REPLAY_SUFFIXES, ("root_height", "predicted_active"))
        self.assertEqual(MAXIMUM_COMPONENT_REGRESSION, 0.0)
        self.assertEqual(MINIMUM_COMPONENT_IMPROVEMENT, 1.0e-6)

    def test_archived_audit_passes_without_plant_or_authority(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["design_audit_not_holdout"])
        self.assertFalse(metrics["completed_per_sample_contact_labels_opened"])
        self.assertTrue(metrics["frozen_uncertainty_width_was_fit_on_spent_source_labels"])
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertEqual(metrics["selected_torques_applied"], 0)
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["semantic_repeat"])
        self.assertTrue(metrics["action_profile_frozen_for_fresh_plant_ab"])
        self.assertFalse(metrics["plant_non_regression_passed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(sum(metrics["selected_counts"].values()), 96)
        self.assertTrue(all(row["all_candidates_wbc_admitted"] for row in metrics["results"]))
        self.assertTrue(all(row["zero_wbc_rust_allocation"] for row in metrics["results"]))
        self.assertTrue(all(row["zero_selector_rust_allocation"] for row in metrics["results"]))
        self.assertTrue(all(row["deadline_passed"] for row in metrics["results"]))
        self.assertTrue(
            all(
                row["maximum_component_regression"] <= MAXIMUM_COMPONENT_REGRESSION
                for row in metrics["results"]
            )
        )


if __name__ == "__main__":
    unittest.main()
