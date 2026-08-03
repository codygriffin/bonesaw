from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/g1-jacobi-column-pointer-r284"
    / "g1-jacobi-column-pointer-r284-metrics.json"
)


class JacobiColumnPointerR284Test(unittest.TestCase):
    def test_exact_replay_and_cpu_reduction_contract(self) -> None:
        result = json.loads(RESULT.read_text())
        self.assertEqual(result["revision"], "g1-jacobi-column-pointer-r284")
        contract = result["semantic_contract"]
        self.assertEqual(contract["non_timing_arrays"], 89)
        self.assertEqual(
            contract["non_timing_sha256"],
            "3489c58b1259bcb97feb87dde038de8df30da9c2d1de3cb77a4f169b9f228521",
        )
        self.assertTrue(contract["all_repeats_exact"])
        control = result["hardware_counter_average"]["control"]
        pointer = result["hardware_counter_average"]["pointer"]
        self.assertLess(pointer["instructions"], control["instructions"])
        self.assertLess(pointer["cycles"], control["cycles"])
        self.assertFalse(result["verdict"]["p99_gate_passed"])
        self.assertFalse(result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
