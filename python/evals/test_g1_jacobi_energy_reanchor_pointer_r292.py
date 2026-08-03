from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/g1-jacobi-energy-reanchor-pointer-r292"
    / "g1-jacobi-energy-reanchor-pointer-r292-metrics.json"
)


class JacobiEnergyReanchorPointerR292Test(unittest.TestCase):
    def test_exact_candidate_is_rejected_by_cpu_work_contract(self) -> None:
        result = json.loads(RESULT.read_text())
        self.assertEqual(result["decision"], "REJECT")
        self.assertTrue(result["semantic_contract"]["all_repeats_exact"])
        self.assertEqual(len(result["semantic_contract"]["pairs"]), 4)
        self.assertTrue(result["verdict"]["candidate_rejected"])
        self.assertTrue(result["verdict"]["r284_default_retained"])
        self.assertFalse(result["candidate"]["production_default"])
        counters = result["hardware_counters"]
        self.assertGreater(
            counters["candidate_average"]["instructions"],
            counters["control_average"]["instructions"],
        )
        self.assertTrue(
            all(value > 0.0 for value in counters["paired_instruction_relative_delta"])
        )
        self.assertFalse(result["verdict"]["p99_gate_passed"])
        self.assertFalse(result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
