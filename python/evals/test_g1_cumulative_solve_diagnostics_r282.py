from __future__ import annotations

import json
import pathlib
import unittest

from g1_cumulative_solve_diagnostics_r282 import validate_result


ROOT = pathlib.Path(__file__).resolve().parents[2]
RESULT = ROOT / "benchmarks/results/g1-cumulative-solve-diagnostics-r282/g1-cumulative-solve-diagnostics-r282-metrics.json"


class CumulativeSolveDiagnosticsR282Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text())

    def test_telemetry_gate_and_no_authority(self) -> None:
        validate_result(self.result)
        self.assertTrue(self.result["verdict"]["telemetry_passed"])
        self.assertFalse(self.result["verdict"]["authority_admitted"])
        self.assertEqual(self.result["execution"]["policy_steps"], 0)
        self.assertEqual(self.result["execution"]["physics_steps"], 0)
        self.assertTrue(
            self.result["semantic_contract"]["r281_non_timing_exact"]
        )
        self.assertEqual(
            self.result["semantic_contract"]["r281_non_timing_arrays"], 89
        )

    def test_retries_and_release_tails_are_visible(self) -> None:
        work = self.result["cumulative_solver_work"]
        self.assertGreaterEqual(work["maximum_attempts"], 1)
        self.assertGreaterEqual(work["ticks_with_retry"], 1)
        tails = self.result["release_tail_work"]["rows"]
        self.assertTrue(tails)
        self.assertTrue(
            any(row["cumulative_feasibility_halfspace_projections"] > 0 for row in tails)
        )
        self.assertTrue(
            all(
                row["cumulative_task_pseudoinverse_calls"]
                >= row["final_task_pseudoinverse_calls"]
                for row in tails
            )
        )
        expensive = {row["tick"]: row for row in tails if row["tick"] in (1694, 2296)}
        self.assertEqual(set(expensive), {1694, 2296})
        self.assertTrue(
            all(
                row["cumulative_feasibility_polish_pseudoinverse_calls"] == 126
                for row in expensive.values()
            )
        )


if __name__ == "__main__":
    unittest.main()
