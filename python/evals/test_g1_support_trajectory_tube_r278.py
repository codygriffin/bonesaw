from __future__ import annotations

import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULTS = ROOT / "benchmarks/results"
REPORT = RESULTS / "g1-support-trajectory-tube-r278/g1-support-trajectory-tube-metrics.json"


class SupportTrajectoryTubeR278Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(REPORT.read_text())

    def test_mechanism_is_default_off_and_non_authoritative(self) -> None:
        result = self.result
        self.assertTrue(result["mechanism_passed"])
        self.assertTrue(result["profile_rejected"])
        self.assertFalse(result["authority_admitted"])
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["physics_steps"], 0)
        self.assertEqual(result["source_contract"]["dormant_semantic_differences"], [])
        self.assertEqual(
            result["source_contract"]["new_arrays"],
            [
                "limiting_center_of_mass_tube_halfspace",
                "minimum_center_of_mass_tube_margin",
                "support_trajectory_tube_active",
                "support_trajectory_tube_clipped",
                "support_trajectory_tube_headroom_scale",
                "support_trajectory_tube_target",
            ],
        )

    def test_hard_and_intent_profiles_expose_fail_closed_evidence(self) -> None:
        profiles = self.result["profiles"]
        control = profiles["dormant control"]
        hard = profiles["hard h5"]
        combined = profiles["combined h5"]
        self.assertEqual(control["active_ticks"], 0)
        self.assertGreater(hard["active_ticks"], 0)
        self.assertGreater(hard["hard_solved_ticks"], 0)
        self.assertGreater(hard["hard_unresolved_ticks"], 0)
        self.assertEqual(hard["hard_boundary_violation_ticks"], 0)
        self.assertLess(hard["first_contact_release_tick"], control["first_contact_release_tick"])
        self.assertGreater(combined["intent_projected_ticks"], 0)

    def test_canonical_trace_shapes_and_neutral_dormant_telemetry(self) -> None:
        control_path = RESULTS / "floating-g1-r278-tube-dormant-cap8/floating-walk-raw.npz"
        hard_path = RESULTS / "floating-g1-r278-tube-hard-h5-cap8/floating-walk-raw.npz"
        with np.load(control_path, allow_pickle=False) as control:
            self.assertEqual(control["support_trajectory_tube_active"].dtype, np.uint8)
            self.assertEqual(control["support_trajectory_tube_clipped"].dtype, np.uint8)
            self.assertEqual(control["support_trajectory_tube_target"].shape[1], 2)
            self.assertTrue(np.all(control["support_trajectory_tube_active"] == 0))
            self.assertTrue(np.all(control["support_trajectory_tube_clipped"] == 0))
            self.assertTrue(np.all(np.isinf(control["minimum_center_of_mass_tube_margin"])))
            self.assertTrue(np.all(control["limiting_center_of_mass_tube_halfspace"] == -1))
        with np.load(hard_path, allow_pickle=False) as hard:
            self.assertTrue(
                np.all(
                    np.isin(
                        hard["limiting_center_of_mass_tube_halfspace"],
                        (-1, 0, 1, 2, 3),
                    )
                )
            )
            active = hard["support_trajectory_tube_active"] != 0
            self.assertGreater(int(np.count_nonzero(active)), 0)


if __name__ == "__main__":
    unittest.main()
