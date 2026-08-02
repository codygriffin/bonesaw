from __future__ import annotations

import json
import pathlib
import unittest

from g1_spent_terminal_state_corpus_r258 import CANDIDATES, REVISION, ROWS


ROOT = pathlib.Path(__file__).resolve().parents[2]
METRICS = ROOT / f"benchmarks/results/{REVISION}/g1-spent-terminal-state-corpus-metrics.json"


class G1SpentTerminalStateCorpusR258Tests(unittest.TestCase):
    def test_spent_extraction_is_exact_and_non_authoritative(self) -> None:
        metrics = json.loads(METRICS.read_text())
        self.assertEqual(metrics["revision"], REVISION)
        self.assertEqual(metrics["rows"], ROWS)
        self.assertEqual(metrics["candidates"], CANDIDATES)
        self.assertEqual(metrics["physics_steps"], ROWS * CANDIDATES * 5)
        self.assertEqual(metrics["policy_steps"], 0)
        self.assertEqual(metrics["plant_actions"], 0)
        self.assertTrue(metrics["source_immutable"])
        self.assertTrue(metrics["all_fixed_effort_wbc_admitted"])
        self.assertEqual(metrics["mujoco_warning_count"], 0)
        self.assertTrue(metrics["exact_diagnostic_replay"])
        self.assertTrue(metrics["replay_zero_rust_allocation"])
        self.assertTrue(metrics["extraction_passed"])
        self.assertFalse(metrics["profile_frozen"])
        self.assertFalse(metrics["authority_admitted"])


if __name__ == "__main__":
    unittest.main()
