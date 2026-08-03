from __future__ import annotations

import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS = ROOT / "benchmarks/results"
REPORT = RESULTS / "g1-support-trajectory-tube-r279/g1-support-trajectory-tube-r279-metrics.json"


class SupportTrajectoryTubeR279Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(REPORT.read_text())

    def test_mechanisms_are_default_off_and_non_authoritative(self) -> None:
        result = self.result
        self.assertTrue(result["mechanism_passed"])
        self.assertTrue(result["profile_rejected"])
        self.assertFalse(result["authority_admitted"])
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["physics_steps"], 0)
        self.assertEqual(result["source_contract"]["dormant_semantic_differences"], [])

    def test_observer_hard_and_combined_layers_are_distinct(self) -> None:
        profiles = self.result["profiles"]
        control = profiles["dormant control"]
        observer = profiles["reachable observer h5"]
        hard = profiles["reachable hard h5"]
        combined = profiles["reachable combined h5"]
        self.assertEqual(control["active_ticks"], 0)
        self.assertGreater(observer["active_ticks"], 0)
        self.assertFalse(observer["hard_enabled"])
        self.assertEqual(observer["first_contact_release_tick"], control["first_contact_release_tick"])
        self.assertTrue(hard["hard_enabled"])
        self.assertGreater(hard["hard_witness_ticks"], 0)
        self.assertGreater(hard["hard_witness_violation_ticks"], 0)
        self.assertLess(hard["first_contact_release_tick"], control["first_contact_release_tick"])
        self.assertGreater(combined["intent_projected_ticks"], 0)

    def test_dormant_and_observer_raw_contract(self) -> None:
        dormant_path = RESULTS / "floating-g1-r279-dormant-cap8/floating-walk-raw.npz"
        observer_path = RESULTS / "floating-g1-r279-reachable-observer-h5-rate2-cap8/floating-walk-raw.npz"
        with np.load(dormant_path, allow_pickle=False) as dormant, np.load(
            observer_path, allow_pickle=False
        ) as observer:
            self.assertTrue(np.all(dormant["support_trajectory_tube_active"] == 0))
            self.assertTrue(np.all(np.isinf(dormant["minimum_center_of_mass_tube_margin"])))
            self.assertTrue(np.all(observer["support_trajectory_tube_active"] >= 0))
            self.assertTrue(np.all(observer["support_trajectory_tube_clipped"] == 0))
            for key in (
                "status",
                "pre_contingency_status",
                "root_tracked",
                "center_of_mass_tracked",
            ):
                np.testing.assert_array_equal(dormant[key], observer[key])


if __name__ == "__main__":
    unittest.main()
