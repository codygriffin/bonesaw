from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/frame-query-allocation-r287"
    / "frame-query-allocation-r287-metrics.json"
)


class FrameQueryAllocationR287Test(unittest.TestCase):
    def test_contract(self) -> None:
        result = json.loads(RESULT.read_text())
        self.assertEqual(result["revision"], "frame-query-allocation-r287")
        verdict = result["verdict"]
        self.assertTrue(verdict["strict_historical_query_allocation_stable"])
        self.assertTrue(verdict["legacy_allocating_api_retained_for_compatibility"])
        self.assertFalse(verdict["wbc_authority_admitted"])
        self.assertFalse(verdict["physics_authority_admitted"])
        execution = result["execution"]
        self.assertGreaterEqual(execution["process_repeats"], 5)
        contract = result["allocation_contract"]
        self.assertEqual(contract["total_measured_allocation_calls"], 0)
        self.assertEqual(contract["total_measured_allocated_bytes"], 0)
        self.assertEqual(contract["total_measured_deallocation_calls"], 0)
        self.assertEqual(
            contract["repeats_with_zero_allocation_calls"],
            execution["process_repeats"],
        )
        self.assertEqual(contract["output_external_capacity"], [2, 2])
        self.assertEqual(contract["workspace_external_capacity"], [2, 2])
        self.assertEqual(contract["workspace_external_sample_capacity"], [2, 2])
        self.assertTrue(contract["all_results_bitwise_equal_within_repeat"])
        timing = result["performance_ns_per_query"]
        self.assertLessEqual(timing["minimum"], timing["median"])
        self.assertLessEqual(timing["median"], timing["p95"])
        self.assertLessEqual(timing["p95"], timing["p99"])
        self.assertLessEqual(timing["p99"], timing["maximum"])


if __name__ == "__main__":
    unittest.main()
