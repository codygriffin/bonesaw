from __future__ import annotations

import pathlib
import unittest

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_load_reserve_matrix_r310 as r310
from upkie_live_plant_worker import LiveUpkiePlant


MODEL = pathlib.Path("models/upkie/upkie.urdf")


class LiveLoadReserveMatrixR310Tests(unittest.TestCase):
    def test_strict_recovery_starts_after_final_loss(self) -> None:
        states = [
            {
                "index": index,
                "observed": [0, 1] if index in (2, 4) else [1, 1],
                "root_height_m": 0.52,
                "root_tilt_rad": 0.02,
                "automatic_reset_pending": None,
            }
            for index in range(16)
        ]
        self.assertEqual(r310.strict_recovery_tick(states, dwell_ticks=3), 5)
        self.assertIsNone(
            r310.strict_recovery_tick(
                [{**state, "observed": [1, 1]} for state in states],
                dwell_ticks=3,
            )
        )

    def test_public_worker_enables_qualified_reserve_profile(self) -> None:
        worker = LiveUpkiePlant(MODEL)
        state = worker.step({"type": "step", "command_id": 0})
        self.assertEqual(state["type"], "plant_state")
        self.assertTrue(state["metrics"]["wbc_support_load_reserve_action_enabled"])
        self.assertEqual(state["metrics"]["wbc_support_load_reserve_authority"], 0.0)

    def test_historical_fixture_explicitly_keeps_reserve_off(self) -> None:
        case = r300.run_case(MODEL, disturbed=False, maximum_ticks=1)
        self.assertFalse(case["states"][0]["support_load_reserve_action_enabled"])

    def test_mirrored_maximum_hold_recovers_without_flight(self) -> None:
        for force_y_n in (-8.0, 8.0):
            with self.subTest(force_y_n=force_y_n):
                case = r310.run_one(
                    MODEL,
                    force_y_n,
                    40,
                    r310.CANDIDATE_OPTIONS,
                    ticks=180,
                )
                summary = r310.summarize(case)
                self.assertEqual(summary["ticks"], 180)
                self.assertIsNone(summary["terminal_pending"])
                self.assertIsNone(summary["first_flight_tick"])
                self.assertIsNotNone(summary["strict_recovery_tick"])
                self.assertTrue(summary["final_100_bilateral_upright"])
                self.assertFalse(summary["nonadmitted_ticks"])


if __name__ == "__main__":
    unittest.main()
