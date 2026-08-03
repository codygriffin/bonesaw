from __future__ import annotations

import pathlib
import sys
import unittest

EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_live_moment_rejection_r314 as r314  # noqa: E402


class UpkieLiveMomentRejectionR314Tests(unittest.TestCase):
    def test_candidate_is_explicit_and_default_off(self) -> None:
        self.assertEqual(r314.CANDIDATE_OPTIONS["centroidal_angular_momentum_weight"], 0.3)
        self.assertEqual(r314.CANDIDATE_OPTIONS["root_roll_damping"], 12.0)
        self.assertNotIn(
            "centroidal_angular_momentum_weight",
            r314.r312.PUBLIC_CONTROLLER_OPTIONS,
        )

    def test_short_holdout_reports_applied_moment_and_finite_outputs(self) -> None:
        case = r314.run_case(
            ROOT / "models" / "upkie" / "upkie.urdf",
            8.0,
            r314.CANDIDATE_OPTIONS,
            ticks=80,
        )
        summary = r314.summarize(case)
        self.assertGreater(summary["maximum_external_moment_nm"], 1.9)
        self.assertTrue(summary["external_moment_finite"])
        self.assertEqual(summary["nonadmitted_ticks"], [])
        self.assertFalse(summary["numeric_reset"])


if __name__ == "__main__":
    unittest.main()
