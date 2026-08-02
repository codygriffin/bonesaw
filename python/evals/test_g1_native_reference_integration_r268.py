#!/usr/bin/env python3

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / "benchmarks/results/g1-native-reference-integration-r268"
    / "g1-native-reference-integration-metrics.json"
)


class NativeReferenceIntegrationR268Test(unittest.TestCase):
    def test_retained_decision_and_causal_ordering(self) -> None:
        metrics = json.loads(METRICS.read_text())
        profiles = metrics["profiles"]
        initialization = profiles["initialization_only"]
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        for profile in profiles.values():
            self.assertGreater(profile["release_fallback_ticks"], 0)
            self.assertTrue(profile["release_fallback_diagnostics_fail_closed"])
            self.assertEqual(profile["release_fallback_dynamics_residual_max"], 0.0)
            self.assertEqual(profile["release_fallback_contact_residual_max"], 0.0)
        self.assertGreater(initialization["first_contingency_tick"], 265)
        self.assertLess(initialization["first_contingency_tick"], 529)
        self.assertLess(
            profiles["oracle_task_stack"]["first_contingency_tick"],
            initialization["first_contingency_tick"],
        )
        self.assertLess(
            profiles["morphology_posture_jet"]["first_contingency_tick"],
            initialization["first_contingency_tick"],
        )
        low_gain = profiles["morphology_posture_jet_low_gain"]
        self.assertGreater(low_gain["first_contingency_tick"], 529)
        self.assertLess(
            low_gain["first_contingency_tick"], metrics["source_contract"]["reference_ticks"]
        )
        self.assertLess(low_gain["root_error_rms_m_before_contingency"], 0.03)
        self.assertLess(
            initialization["maximum_accepted_dynamics_residual"], 1.0e-8
        )
        self.assertLess(
            initialization["maximum_accepted_contact_residual"], 1.0e-8
        )


if __name__ == "__main__":
    unittest.main()
