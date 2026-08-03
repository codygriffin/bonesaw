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

from upkie_measured_contact_kinematic_r299 import evaluate, run_replay  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the live MuJoCo/PyO3 runtime is not installed")
class UpkieMeasuredContactKinematicR299Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = ROOT / "models/upkie/upkie.urdf"

    def test_actual_geometry_covers_double_single_and_flight(self) -> None:
        result = run_replay(self.model)
        self.assertEqual(
            {tuple(state["observed"]) for state in result["states"]},
            {(1, 1), (1, 0), (0, 1), (0, 0)},
        )
        self.assertEqual(result["physics_integration_steps"], 0)
        self.assertEqual(result["initial_simulator_time_s"], result["final_simulator_time_s"])

    def test_all_admission_and_replay_gates_pass(self) -> None:
        result = run_replay(self.model)
        replay = run_replay(self.model)
        gates = evaluate(result, replay)
        self.assertTrue(all(gates.values()), gates)


if __name__ == "__main__":
    unittest.main()
