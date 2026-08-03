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

from upkie_live_dynamic_contact_transition_r300 import (  # noqa: E402
    behavior_passed,
    evaluate,
    fixture_passed,
    run_case,
)


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveDynamicContactTransitionR300Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = ROOT / "models/upkie/upkie.urdf"

    def test_dynamic_contact_fixture_is_exact_and_fail_closed(self) -> None:
        candidate = run_case(self.model, disturbed=True)
        control = run_case(
            self.model, disturbed=False, maximum_ticks=len(candidate["states"])
        )
        replay = run_case(self.model, disturbed=True)
        gates = evaluate(control, candidate, replay)
        self.assertTrue(fixture_passed(gates), gates)
        self.assertFalse(behavior_passed(gates), gates)
        self.assertFalse(gates["controller_p99_under_5ms"])
        self.assertFalse(gates["hard_residuals_under_1e8"])
        self.assertFalse(gates["candidate_recovers_without_boundary"])

    def test_boundary_is_reported_before_reset_is_consumed(self) -> None:
        candidate = run_case(self.model, disturbed=True)
        final = candidate["states"][-1]
        self.assertEqual(final["automatic_reset_pending"], "fall")
        self.assertFalse(final["numeric_reset"])
        self.assertEqual({state["reset_epoch"] for state in candidate["states"]}, {0})


if __name__ == "__main__":
    unittest.main()
