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

    HAS_LIVE_RUNTIME = True
except ImportError:
    HAS_LIVE_RUNTIME = False

from upkie_live_contact_transition_replay import (  # noqa: E402
    PHYSICS_SUBSTEPS,
    control_masks,
    evaluate,
    run_replay,
)


@unittest.skipUnless(
    HAS_LIVE_RUNTIME,
    "the live MuJoCo/PyO3 runtime is not installed",
)
class UpkieLiveContactTransitionReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = (ROOT / "models/upkie/upkie.urdf").resolve()

    def test_sequential_250hz_source_reaches_50hz_wbc_without_reset(self) -> None:
        result = run_replay(self.model)
        gates = evaluate(result, result)
        self.assertTrue(all(gates.values()), gates)
        self.assertEqual(
            result["source"]["cursor"],
            len(control_masks()) * PHYSICS_SUBSTEPS,
        )

    def test_replay_keeps_all_single_and_zero_contact_edges(self) -> None:
        result = run_replay(self.model)
        observed = {tuple(state["observed"]) for state in result["states"]}
        self.assertEqual(observed, {(1, 1), (1, 0), (0, 1), (0, 0)})
        self.assertEqual(result["paused"]["hard"], [0, 0])
        self.assertFalse(result["paused"]["available"])


if __name__ == "__main__":
    unittest.main()
