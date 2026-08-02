from __future__ import annotations

import unittest
import pathlib
import sys

EVALS = pathlib.Path(__file__).resolve().parent
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

from upkie_planar_capture_ab import (  # noqa: E402
    CASE_NAMES,
    LATERAL_NAMES,
    promotion_decision,
)


def result(outcome: str, qualified: bool, boundary: float) -> dict[str, object]:
    return {"outcome": outcome, "qualified": qualified, "terminal_time_s": boundary}


class UpkiePlanarCaptureAbTest(unittest.TestCase):
    def test_case_matrix_keeps_nominal_sagittal_and_both_lateral_signs(self) -> None:
        self.assertEqual(CASE_NAMES[:2], ("nominal", "forward_4n_reference"))
        self.assertEqual(LATERAL_NAMES, ("left_1n", "left_2n", "right_2n"))

    def test_no_lateral_recovery_rejects_promotion_despite_delayed_falls(self) -> None:
        baseline = {
            "nominal": result("RECOVERED", True, 6.0),
            "forward_4n_reference": result("RECOVERED", True, 6.0),
            "left_1n": result("FALL", False, 2.0),
            "left_2n": result("FALL", False, 1.8),
            "right_2n": result("FALL", False, 1.8),
        }
        candidate = {
            "nominal": result("RECOVERED", True, 6.0),
            "forward_4n_reference": result("RECOVERED", True, 6.0),
            "left_1n": result("FALL", False, 2.5),
            "left_2n": result("FALL", False, 2.1),
            "right_2n": result("FALL", False, 2.3),
        }
        promote, blockers = promotion_decision(baseline, candidate)
        self.assertFalse(promote)
        self.assertTrue(any("no lateral recovery" in blocker for blocker in blockers))


if __name__ == "__main__":
    unittest.main()
