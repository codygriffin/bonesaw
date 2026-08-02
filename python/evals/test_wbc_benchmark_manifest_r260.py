from __future__ import annotations

import json
import pathlib
import unittest

from wbc_benchmark_manifest_r260 import REVISION


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/wbc-benchmark-manifest-metrics.json"


class WbcBenchmarkManifestR260Tests(unittest.TestCase):
    def test_manifest_retains_behavior_red_and_reference_parity(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertFalse(metrics["authority_admitted"])
        self.assertTrue(metrics["gates"]["fixed_base_end_effector_and_conflict"])
        self.assertTrue(metrics["gates"]["floating_moving_liftoff"])
        self.assertFalse(metrics["gates"]["fixed_base_walking_retarget"])
        self.assertFalse(metrics["gates"]["floating_full_transfer"])
        self.assertTrue(metrics["gates"]["upkie_official_controller_parity"])
        self.assertEqual(
            metrics["upkie_reference"]["official_aligned"]["canonical_bit_mismatches"],
            0,
        )
        self.assertEqual(
            metrics["upkie_reference"]["zero_hot_loop_allocation"]["allocated_bytes"],
            0,
        )
        self.assertEqual(metrics["upkie_reference"]["temporal_windows"], 10)


if __name__ == "__main__":
    unittest.main()
