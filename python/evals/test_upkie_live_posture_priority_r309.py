from __future__ import annotations

import pathlib
import unittest

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_posture_priority_r309 as r309
from upkie_live_plant_worker import LiveUpkiePlant


MODEL = pathlib.Path("models/upkie/upkie.urdf")


class LivePosturePriorityR309Tests(unittest.TestCase):
    def _run(self, disturbed: bool) -> list[dict[str, object]]:
        return r300.run_case(
            MODEL,
            disturbed=disturbed,
            maximum_ticks=300,
            controller_options=r309.CANDIDATE_OPTIONS,
            worker_options=r309.WORKER_OPTIONS,
        )["states"]

    def test_priority_zero_holds_nominal_bilateral_stance(self) -> None:
        states = self._run(False)
        self.assertEqual(len(states), 300)
        self.assertIsNone(states[-1]["automatic_reset_pending"])
        self.assertTrue(all(state["observed"] == [1, 1] for state in states))
        self.assertTrue(all(state["wbc_admitted"] for state in states))
        self.assertLess(max(state["root_tilt_rad"] for state in states), 0.02)

    def test_live_worker_default_uses_qualified_hierarchy(self) -> None:
        worker = LiveUpkiePlant(
            MODEL,
            stream_dt=0.020,
            control_dt=0.020,
            physics_dt=0.004,
            balanced_nominal_joint_target=True,
        )
        states = [worker.step({"type": "step", "command_id": tick}) for tick in range(300)]
        self.assertTrue(all(state["type"] == "plant_state" for state in states))
        self.assertTrue(
            all(state["automatic_reset_pending"] is None for state in states)
        )
        self.assertTrue(
            all(state["wbc_observed_contact_active"] == [1, 1] for state in states)
        )

    def test_priority_zero_rejects_lateral_wrench_without_contact_loss(self) -> None:
        states = self._run(True)
        self.assertEqual(len(states), 300)
        self.assertIsNone(states[-1]["automatic_reset_pending"])
        self.assertTrue(all(state["observed"] == [1, 1] for state in states))
        self.assertTrue(all(state["wbc_admitted"] for state in states))
        self.assertLess(max(state["root_tilt_rad"] for state in states), 0.05)
        self.assertLess(max(state["torque_utilization"] for state in states), 0.20)


if __name__ == "__main__":
    unittest.main()
