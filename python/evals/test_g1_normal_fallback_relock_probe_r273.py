from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-normal-fallback-relock-probe-r273/g1-normal-fallback-relock-probe-metrics.json"
MODEL = ROOT / "benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf"


class NormalFallbackRelockProbeR273Test(unittest.TestCase):
    def test_mechanism_recovers_but_profile_is_rejected(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["default_changed"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        contract = metrics["source_contract"]
        self.assertTrue(contract["dormant_non_timing_arrays_equal"])
        baseline = metrics["profiles"]["r272_baseline"]
        admitted_profiles = 0
        for interval in (1, 4, 8, 16, 32, 64):
            name = f"r273_interval{interval}"
            candidate = metrics["profiles"][name]
            self.assertTrue(candidate["finite_state"])
            self.assertEqual(contract["candidate_prefix_semantic_differences"][name], [])
            self.assertGreater(candidate["probe_attempts"], 0)
            self.assertEqual(
                candidate["first_fallback_tick"], baseline["first_fallback_tick"]
            )
            self.assertLessEqual(
                candidate["first_release_tick"], baseline["first_release_tick"]
            )
            if candidate["probe_admissions"]:
                admitted_profiles += 1
                self.assertGreater(candidate["maximum_relock_duration_ticks"], 0)
                self.assertLess(
                    candidate["first_release_tick"], baseline["first_release_tick"]
                )
        self.assertGreaterEqual(admitted_profiles, 2)
        self.assertEqual(metrics["profiles"]["r273_interval64"]["probe_admissions"], 0)

    def test_probe_interval_is_bounded_at_the_pyO3_boundary(self) -> None:
        import bonesaw

        with self.assertRaises(ValueError):
            bonesaw.FloatingWbcSession(
                str(MODEL), normal_fallback_relock_probe_interval_ticks=513
            )


if __name__ == "__main__":
    unittest.main()
