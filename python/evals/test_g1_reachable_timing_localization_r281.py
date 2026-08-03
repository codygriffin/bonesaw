from __future__ import annotations

import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-reachable-timing-localization-r281/g1-reachable-timing-localization-r281-metrics.json"


class ReachableTimingLocalizationR281Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_exact_candidate_is_strictly_rejected(self) -> None:
        r = self.result
        self.assertTrue(r["semantic_comparison"]["bit_exact"])
        self.assertEqual(r["semantic_comparison"]["different_arrays"], [])
        self.assertGreater(r["profiles"]["two_column_candidate"]["p99_us"], r["profiles"]["established"]["p99_us"])
        self.assertTrue(r["verdict"]["candidate_rejected"])
        self.assertTrue(r["verdict"]["candidate_removed"])

    def test_tail_diagnostics_are_known_to_be_overwritten(self) -> None:
        tail = self.result["release_tail_localization"]
        self.assertEqual(tail["ticks"], [1694, 2296])
        self.assertEqual(tail["status"], 5)
        self.assertEqual(tail["final_reported_task_pseudoinverse_calls"], 0)
        self.assertTrue(self.result["verdict"]["release_tail_needs_cumulative_attempt_diagnostics"])

    def test_no_authority_follows(self) -> None:
        self.assertEqual(self.result["policy_steps"], 0)
        self.assertEqual(self.result["physics_steps"], 0)
        self.assertFalse(self.result["verdict"]["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
