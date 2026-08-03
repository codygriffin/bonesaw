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

import upkie_live_load_reserve_action_r307 as r307  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieLiveLoadReserveActionR307Tests(unittest.TestCase):
    def test_default_off_candidate_is_causal_allocation_free_and_not_promoted(
        self,
    ) -> None:
        metrics, _, _ = r307.run(
            ROOT / "models/upkie/upkie.urdf",
            nominal_ticks=120,
            disturbed_ticks=250,
        )
        self.assertTrue(
            metrics["qualified_as_bounded_default_off_experiment"], metrics
        )
        self.assertFalse(metrics["candidate_default_enabled"])
        self.assertFalse(metrics["recovery_promoted"])
        self.assertEqual(metrics["nominal_action"]["maximum_authority"], 0.0)
        self.assertEqual(metrics["disturbed_action"]["allocation_calls"], 0)
        self.assertEqual(metrics["disturbed_action"]["allocated_bytes"], 0)
        self.assertLess(
            metrics["disturbed_action"]["first_active_tick"],
            metrics["candidate_disturbed"]["first_non_double_tick"],
        )


if __name__ == "__main__":
    unittest.main()
