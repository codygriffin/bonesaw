from __future__ import annotations

import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = (
    ROOT
    / "benchmarks/results/g1-hard-feasibility-witness-r275/"
    / "g1-hard-feasibility-witness-metrics.json"
)


class HardFeasibilityWitnessR275Test(unittest.TestCase):
    def test_witness_localizes_rejected_capture_without_promoting_authority(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["hard_capture_profile_rejected"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)

        control = metrics["profiles"]["control_cap8"]
        hard = metrics["profiles"]["hard_capture_w025"]
        self.assertEqual(control["first_failed_tick"], 875)
        self.assertEqual(control["first_failed_linear_constraint_name"], "dynamics[3]")
        self.assertIsNone(control["first_failed_bound_coordinate"])
        self.assertEqual(hard["first_failed_tick"], 869)
        self.assertEqual(hard["first_failed_bound_coordinate"], 15)
        self.assertGreater(hard["root_tracking_rms_m"], control["root_tracking_rms_m"])

        raw = ROOT / "benchmarks/results/floating-g1-r275-position-capture-hard-w025-cap8/floating-walk-raw.npz"
        with np.load(raw, allow_pickle=False) as trace:
            knee = trace["q"][:, 9]
            lower_tick = int(np.flatnonzero(knee <= -0.087267 + 1.0e-9)[0])
            self.assertEqual(lower_tick, 903)
            self.assertEqual(int(trace["pre_contingency_limiting_bound_coordinate"][869]), 15)
            self.assertGreater(trace["pre_contingency_maximum_bound_violation"][869], 1.0)
            self.assertGreater(trace["pre_contingency_maximum_linear_violation"][869], 100.0)


if __name__ == "__main__":
    unittest.main()
