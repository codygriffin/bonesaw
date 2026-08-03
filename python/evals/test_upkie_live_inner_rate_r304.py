from __future__ import annotations

import pathlib
import sys
import unittest


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_dynamic_contact_transition_r300 as r300  # noqa: E402
import upkie_live_inner_rate_r304 as r304  # noqa: E402


class UpkieLiveInnerRateR304Tests(unittest.TestCase):
    def test_short_nominal_trace_exercises_exact_fast_eval_rate_contract(self) -> None:
        case = r300.run_case(
            ROOT / "models/upkie/upkie.urdf",
            disturbed=False,
            maximum_ticks=12,
            worker_options=r304.FAST_WORKER_OPTIONS,
        )
        summary = r300.summarize(case)
        self.assertEqual(
            r304._rates(case),
            {
                "stream_hz": 50,
                "control_hz": 250,
                "physics_hz": 1000,
                "physics_substeps_per_control": 4,
            },
        )
        self.assertEqual(summary["ticks"], 12)
        self.assertEqual(summary["observed_patterns"], ["11"])
        self.assertEqual(summary["allocation_calls"], 0)
        self.assertEqual(summary["allocated_bytes"], 0)
        self.assertIsNone(summary["terminal_pending"])

    def test_report_labels_lateral_recovery_as_open(self) -> None:
        metrics, _, markdown, html_report = r304.run(
            ROOT / "models/upkie/upkie.urdf",
            nominal_ticks=100,
            disturbed_ticks=60,
        )
        self.assertTrue(metrics["qualified"])
        self.assertFalse(metrics["recovery_promoted"])
        self.assertIn("lateral recovery remains unpromoted", markdown)
        self.assertIn("Recovery stays red", html_report)
        self.assertFalse(
            metrics["open_recovery_gates"][
                "eight_newton_lateral_pull_recovers"
            ]
        )


if __name__ == "__main__":
    unittest.main()
