from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-support-reachable-tube-r280/g1-support-reachable-tube-r280-metrics.json"


class SupportReachableTubeR280Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_behavior_passes_without_authority(self) -> None:
        p = self.result["profiles"]
        self.assertGreater(p["soft_two_axis"]["release"], p["dormant"]["release"])
        for field in ("root_rms_m", "com_rms_m", "foot_rms_m"):
            self.assertLess(p["soft_two_axis"][field], p["dormant"][field])
        self.assertFalse(self.result["verdict"]["authority_admitted"])

    def test_observer_and_request_are_execution_neutral(self) -> None:
        p = self.result["profiles"]
        self.assertTrue(p["observer"]["state_exact"])
        self.assertTrue(p["request_only"]["state_exact"])
        self.assertEqual(p["observer"]["clipped"], 0)
        self.assertGreater(p["request_only"]["clipped"], 0)

    def test_all_repeats_are_exact_and_miss_timing(self) -> None:
        rows = self.result["soft_pinned_repeats"]
        deadline = self.result["gates"]["p99_deadline_us"]
        self.assertEqual(len(rows), 5)
        self.assertTrue(all(r["semantic_exact"] for r in rows))
        self.assertTrue(all(r["p99_us"] > deadline for r in rows))
        self.assertTrue(all(r["gc"] == 0 for r in rows))
        self.assertTrue(all(r["late_ticks"] == [1694, 2296] for r in rows))

    def test_hard_and_lateral_profiles_are_rejected(self) -> None:
        p = self.result["profiles"]
        self.assertEqual(p["hard"]["solved"] + p["hard"]["unresolved"], p["hard"]["active"])
        self.assertGreater(p["hard"]["unresolved"], 0)
        self.assertLess(p["rejected_lateral_only"]["p99_us"], 5000.0)
        self.assertGreater(p["rejected_lateral_only"]["root_rms_m"], p["dormant"]["root_rms_m"])


if __name__ == "__main__":
    unittest.main()
