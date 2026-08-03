from __future__ import annotations

import pathlib
import unittest

import upkie_live_dynamic_contact_transition_r300 as r300
import upkie_live_wrench_envelope_r311 as r311


MODEL = pathlib.Path("models/upkie/upkie.urdf")


class LiveWrenchEnvelopeR311Tests(unittest.TestCase):
    def test_offset_and_repeated_windows_reach_the_live_plant(self) -> None:
        windows = ((10, 3), (30, 2))
        case = r300.run_case(
            MODEL,
            disturbed=True,
            maximum_ticks=45,
            controller_options=r311.r310.CANDIDATE_OPTIONS,
            worker_options=r311.r310.WORKER_OPTIONS,
            force_world_n=(0.0, 4.0, 0.0),
            application_offset_world_m=(0.0, 0.0, 0.25),
            push_windows=windows,
        )
        active = [state for state in case["states"] if state["external_load_active"]]
        self.assertEqual(len(active), 5)
        self.assertTrue(
            all(state["external_moment_world_nm"] == [-1.0, 0.0, 0.0] for state in active)
        )
        self.assertEqual(case["push_windows"], [[10, 3], [30, 2]])

    def test_default_r300_case_keeps_frozen_schema(self) -> None:
        case = r300.run_case(MODEL, disturbed=False, maximum_ticks=1)
        self.assertNotIn("application_offset_world_m", case)
        self.assertNotIn("push_windows", case)

    def test_outcome_names_terminal_and_feasibility_separately(self) -> None:
        base = {
            "terminal_pending": None,
            "nonadmitted_ticks": [],
            "final_100_bilateral_upright": True,
            "maximum_root_tilt_rad": 0.1,
        }
        self.assertEqual(r311.outcome(base), "stable_upright")
        self.assertEqual(
            r311.outcome({**base, "maximum_root_tilt_rad": 0.3}),
            "completed_outside_upright_envelope",
        )
        self.assertEqual(
            r311.outcome({**base, "terminal_pending": "fall"}), "terminal_fall"
        )
        self.assertEqual(
            r311.outcome({**base, "nonadmitted_ticks": [68]}),
            "completed_with_nonadmission",
        )


if __name__ == "__main__":
    unittest.main()
