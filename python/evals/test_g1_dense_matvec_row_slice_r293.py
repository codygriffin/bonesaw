#!/usr/bin/env python3
"""Regression gates for the promoted exact-order R293 CPU kernel."""

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/g1-dense-matvec-row-slice-r293"
    / "g1-dense-matvec-row-slice-r293-metrics.json"
)


class DenseMatvecRowSliceR293Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_replay_is_policy_physics_free_and_exact(self) -> None:
        execution = self.result["execution"]
        self.assertEqual(execution["policy_steps"], 0)
        self.assertEqual(execution["physics_steps"], 0)
        self.assertEqual(execution["timing_repeats"], 6)
        self.assertTrue(self.result["semantic_contract"]["all_repeats_exact"])
        self.assertTrue(all(pair["exact"] for pair in self.result["semantic_contract"]["pairs"]))

    def test_every_paired_instruction_count_improves(self) -> None:
        counters = self.result["hardware_counters"]
        self.assertEqual(len(counters["paired_instruction_relative_delta"]), 5)
        self.assertTrue(all(delta < 0.0 for delta in counters["paired_instruction_relative_delta"]))
        self.assertLess(counters["production_relative_delta"]["instructions"], -0.019)
        self.assertLess(counters["production_relative_delta"]["branches"], -0.08)

    def test_clean_timing_and_memory_series_improves_without_crossing_gate(self) -> None:
        timing = self.result["timing_us"]
        self.assertTrue(
            all(
                pair["production"]["p99"] < pair["control"]["p99"]
                for pair in timing["pairs"]
            )
        )
        process = self.result["process_observations"]["summary"]
        self.assertEqual(
            process["production"]["maximum_max_rss_kib"],
            process["control"]["maximum_max_rss_kib"],
        )

    def test_promotion_does_not_claim_deadline_or_authority(self) -> None:
        verdict = self.result["verdict"]
        self.assertTrue(verdict["semantic_gate_passed"])
        self.assertTrue(verdict["cpu_work_gate_passed"])
        self.assertTrue(verdict["production_default_promoted"])
        self.assertFalse(verdict["p99_under_5ms"])
        self.assertFalse(verdict["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
