#!/usr/bin/env python3
"""Regression gates for R297 host-native timing and the R296 rejected dot candidate."""

from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = (
    ROOT
    / "benchmarks/results/g1-host-native-timing-r297"
    / "g1-host-native-timing-r297-metrics.json"
)


class HostNativeTimingR297Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_replay_is_policy_physics_free_and_exact(self) -> None:
        execution = self.result["execution"]
        self.assertEqual(execution["policy_steps"], 0)
        self.assertEqual(execution["physics_steps"], 0)
        self.assertEqual(execution["timing_pairs"], 7)
        self.assertEqual(execution["rustflags"], "-C target-cpu=native")
        self.assertTrue(self.result["semantic_contract"]["all_repeats_exact"])
        self.assertEqual(self.result["semantic_contract"]["field_count"], 112)

    def test_host_native_production_passes_every_p99_gate(self) -> None:
        timing = self.result["timing_us"]
        self.assertTrue(all(pair["control"]["p99"] < 5000 for pair in timing["pairs"]))
        self.assertEqual(timing["p99_under_5ms_repeats"]["control"], 7)
        self.assertTrue(self.result["verdict"]["host_native_production_p99_gate_passed"])
        self.assertFalse(self.result["verdict"]["portable_generic_p99_gate_passed"])

    def test_fixed_shape_candidate_is_rejected_on_integrated_evidence(self) -> None:
        counters = self.result["hardware_counters"]
        self.assertLess(counters["narrow_native_sentinel_relative_delta"]["instructions"], 0)
        self.assertTrue(all(delta > 0 for delta in counters["paired_instruction_relative_delta"]))
        self.assertGreater(counters["integrated_candidate_relative_delta"]["instructions"], 0)
        self.assertEqual(self.result["timing_us"]["p99_under_5ms_repeats"]["candidate"], 6)
        self.assertFalse(self.result["verdict"]["candidate_promoted"])
        self.assertTrue(self.result["verdict"]["r293_source_default_retained"])

    def test_no_authority_claim_follows(self) -> None:
        self.assertFalse(self.result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
