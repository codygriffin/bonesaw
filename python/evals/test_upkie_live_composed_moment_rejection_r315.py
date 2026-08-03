from __future__ import annotations

import json
import pathlib
import sys
import unittest


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_composed_moment_rejection_r315 as r315  # noqa: E402


class UpkieLiveComposedMomentRejectionR315Tests(unittest.TestCase):
    def test_profiles_keep_both_mechanisms_explicit_and_default_off(self) -> None:
        composed = r315.PROFILES["body_plus_root_wrench"]
        self.assertTrue(composed["body_moment_rejection_enabled"])
        self.assertTrue(composed["external_wrench_feedforward_enabled"])
        self.assertNotIn(
            "external_wrench_feedforward_enabled",
            r315.feed.r312.PUBLIC_CONTROLLER_OPTIONS,
        )

    def test_retained_screen_rejects_composition_without_hiding_regressions(self) -> None:
        metrics = json.loads(
            (
                ROOT
                / "benchmarks/results/upkie-live-composed-moment-rejection-r315/metrics.json"
            ).read_text()
        )
        self.assertFalse(metrics["composition_promoted"])
        self.assertFalse(metrics["gates"]["composition_finishes_every_case"])
        self.assertFalse(metrics["gates"]["composition_has_zero_nonadmission"])
        self.assertTrue(metrics["gates"]["semantic_replay_exact"])


if __name__ == "__main__":
    unittest.main()
