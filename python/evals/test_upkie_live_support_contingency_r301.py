from __future__ import annotations

import pathlib
import sys
import unittest


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

try:
    import bonesaw  # noqa: F401
    import mujoco  # noqa: F401

    HAS_RUNTIME = True
except ImportError:
    HAS_RUNTIME = False

from upkie_live_dynamic_contact_transition_r300 import run_case  # noqa: E402
from upkie_live_support_contingency_r301 import (  # noqa: E402
    MAX_TICKS,
    SUPPORT_CONTINGENCY_OPTIONS,
    _support_summary,
    evaluate,
)


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveSupportContingencyR301Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = ROOT / "models/upkie/upkie.urdf"

    def test_flight_contingency_delays_fall_without_admitting_bad_rows(self) -> None:
        baseline = run_case(self.model, disturbed=True, maximum_ticks=MAX_TICKS)
        candidate = run_case(
            self.model,
            disturbed=True,
            maximum_ticks=MAX_TICKS,
            controller_options=SUPPORT_CONTINGENCY_OPTIONS,
        )
        replay = run_case(
            self.model,
            disturbed=True,
            maximum_ticks=MAX_TICKS,
            controller_options=SUPPORT_CONTINGENCY_OPTIONS,
        )
        gates = evaluate(baseline, candidate, replay)
        self.assertTrue(all(gates.values()), gates)
        baseline_summary = _support_summary(baseline)
        candidate_summary = _support_summary(candidate)
        self.assertGreaterEqual(
            candidate_summary["terminal_tick"] - baseline_summary["terminal_tick"],
            20,
        )
        self.assertEqual(candidate_summary["max_iterations_ticks"], [])
        self.assertEqual(candidate_summary["support_contingency_modes"], ["flight"])
        self.assertGreater(candidate_summary["support_contingency_first_selected_tick"], 0)

    def test_profile_is_explicit_and_not_the_public_default(self) -> None:
        self.assertTrue(SUPPORT_CONTINGENCY_OPTIONS["support_contingency_enabled"])
        self.assertTrue(SUPPORT_CONTINGENCY_OPTIONS["support_contingency_execute"])
        self.assertTrue(SUPPORT_CONTINGENCY_OPTIONS["support_contingency_flight_only"])
        self.assertTrue(SUPPORT_CONTINGENCY_OPTIONS["support_contingency_forecast_guard"])


if __name__ == "__main__":
    unittest.main()
