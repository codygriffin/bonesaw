#!/usr/bin/env python3

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / "benchmarks/results/g1-lower-body-velocity-envelope-r270"
    / "g1-lower-body-velocity-envelope-metrics.json"
)


class LowerBodyVelocityEnvelopeR270Test(unittest.TestCase):
    def test_safety_mechanism_passes_but_walking_profile_is_rejected(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["hard_bound_causal"])
        self.assertTrue(metrics["soft_hard_interaction_observed"])
        self.assertTrue(metrics["composition_rejected"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        baseline = metrics["profiles"]["r268_low_gain_baseline"]
        soft_only = metrics["profiles"]["r270_lower_body_soft_only"]
        hard_only = metrics["profiles"]["r270_lower_body_hard_only"]
        candidate = metrics["profiles"]["r270_lower_body_soft_plus_hard"]
        r269 = metrics["profiles"]["r269_bounded_continuation"]
        dormant = metrics["profiles"]["r270_dormant_r269_stack"]
        hard_composed = metrics["profiles"][
            "r270_hard_only_composed_r269_stack"
        ]
        composed = metrics["profiles"]["r270_soft_plus_hard_composed_r269_stack"]
        self.assertEqual(baseline["first_fallback_tick"], 863)
        self.assertGreaterEqual(hard_only["first_fallback_tick"], 883)
        self.assertEqual(soft_only["first_fallback_tick"], 864)
        self.assertGreater(candidate["first_release_tick"], hard_only["first_release_tick"])
        self.assertGreater(candidate["first_release_tick"], soft_only["first_release_tick"])
        self.assertEqual(candidate["envelope_active_early_max_coordinates"], 0)
        self.assertGreaterEqual(candidate["envelope_active_critical_max_coordinates"], 1)
        self.assertGreater(hard_only["critical_joint_velocity_min_rad_s"], -8.0)
        self.assertTrue(candidate["release_diagnostics_fail_closed"])
        self.assertEqual(dormant["status_counts"], r269["status_counts"])
        self.assertTrue(
            metrics["source_contract"]["dormant_semantic_arrays_equal_r269"]
        )
        self.assertEqual(
            metrics["source_contract"]["dormant_semantic_differences"], []
        )
        self.assertEqual(
            composed["first_fallback_tick"], candidate["first_fallback_tick"]
        )
        self.assertLess(composed["first_release_tick"], candidate["first_release_tick"])
        self.assertGreater(
            hard_composed["first_release_tick"], hard_only["first_release_tick"]
        )


if __name__ == "__main__":
    unittest.main()
