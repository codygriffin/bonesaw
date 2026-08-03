from __future__ import annotations

import json
import pathlib
import sys
import unittest


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_root_wrench_robustness_r316 as r316  # noqa: E402


class UpkieLiveRootWrenchRobustnessR316Tests(unittest.TestCase):
    def test_selected_scale_is_explicit_and_public_profile_stays_off(self) -> None:
        self.assertEqual(r316.SELECTED_SCALE, 0.70)
        self.assertNotIn(
            "external_wrench_feedforward_enabled",
            r316.r314.r312.PUBLIC_CONTROLLER_OPTIONS,
        )

    def test_retained_audit_keeps_selection_and_holdout_verdicts_separate(self) -> None:
        metrics = json.loads(
            (
                ROOT
                / "benchmarks/results/upkie-live-root-wrench-robustness-r316/metrics.json"
            ).read_text()
        )
        self.assertTrue(metrics["gates"]["selected_scale_clears_terminal_rows"])
        self.assertFalse(metrics["gates"]["adjacent_scale_interval_qualifies"])
        self.assertFalse(
            metrics["gates"]["independent_40_case_holdout_has_zero_falls"]
        )
        self.assertFalse(metrics["public_promotion"])


if __name__ == "__main__":
    unittest.main()
