from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / "benchmarks/results/g1-automatic-contact-localization-r276/g1-automatic-contact-localization-metrics.json"


class AutomaticContactLocalizationR276Test(unittest.TestCase):
    def test_mechanism_passes_without_promoting_profile(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertTrue(metrics["mechanism_passed"])
        self.assertTrue(metrics["profile_rejected"])
        self.assertFalse(metrics["authority_admitted"])
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["physics_steps"], 0)
        self.assertEqual(metrics["source_contract"]["shared_control_array_count"], 81)
        self.assertEqual(metrics["source_contract"]["dormant_semantic_differences"], [])
        automatic = metrics["profiles"]["automatic"]
        self.assertEqual(automatic["probe_ticks"], [875])
        self.assertEqual(automatic["total_probes"], 2)
        self.assertEqual(automatic["admitted_targets"], [1])
        self.assertEqual(automatic["typed_admission_ticks"], [875])


if __name__ == "__main__":
    unittest.main()
