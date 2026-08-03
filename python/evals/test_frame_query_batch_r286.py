from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/frame-query-batch-r286"
    / "frame-query-batch-r286-metrics.json"
)


class FrameQueryBatchR286Test(unittest.TestCase):
    def test_contract(self) -> None:
        result = json.loads(RESULT.read_text())
        self.assertEqual(result["revision"], "frame-query-batch-r286")
        self.assertTrue(result["verdict"]["batch_surface_qualified"])
        self.assertFalse(result["verdict"]["wbc_authority_admitted"])
        self.assertFalse(result["verdict"]["physics_authority_admitted"])
        contract = result["semantic_contract"]
        self.assertTrue(contract["repeated_results_bitwise_equal"])
        self.assertEqual(
            contract["output_external_capacity_before"],
            contract["output_external_capacity_after"],
        )


if __name__ == "__main__":
    unittest.main()
