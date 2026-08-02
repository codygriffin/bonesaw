from __future__ import annotations

import unittest

import numpy as np

import upkie_terminal_realization_calibration as calibration


class TerminalRealizationCalibrationTests(unittest.TestCase):
    def test_selected_candidate_is_compared_with_measured_interval_velocity(self) -> None:
        trace = {
            "inexact_observation_terminal_selector_queried": np.asarray([1], np.uint8),
            "inexact_observation_terminal_selector_action": np.asarray([2], np.uint8),
            "inexact_observation_terminal_selector_root_acceleration": np.asarray(
                [[[0.0, 0.0], [1.0, 2.0], [3.0, 4.0]]], np.float64
            ),
            "inexact_observation_terminal_selector_joint_acceleration": np.asarray(
                [[[0.0] * 6, [1.0] * 6, [2.0] * 6]], np.float64
            ),
            "root_twist": np.zeros((1, 6), np.float64),
            "post_root_twist": np.asarray(
                [[0.02, 0.03, 0.0, 0.0, 0.0, 0.0]], np.float64
            ),
            "v": np.zeros((1, 6), np.float64),
            "post_v": np.full((1, 6), 0.015, np.float64),
        }
        samples = calibration.samples_from_trace("case", "profile", trace)
        self.assertEqual(len(samples), 1)
        np.testing.assert_array_equal(
            samples[0]["predicted"], np.asarray([3.0, 4.0, *([2.0] * 6)])
        )
        np.testing.assert_allclose(
            samples[0]["realized"], np.asarray([4.0, 6.0, *([3.0] * 6)])
        )
        np.testing.assert_allclose(
            samples[0]["error"], np.asarray([1.0, 2.0, *([1.0] * 6)])
        )

    def test_leave_one_case_out_never_uses_held_case_bound(self) -> None:
        samples = []
        for case, magnitude in (("a", 1.0), ("b", 3.0)):
            samples.append(
                {
                    "case": case,
                    "profile": "p",
                    "tick": 0,
                    "action": 0,
                    "predicted": np.zeros(8),
                    "realized": np.full(8, magnitude),
                    "error": np.full(8, magnitude),
                }
            )
        audit = calibration.leave_one_case_out(samples, ("a", "b"), 1.0)
        self.assertEqual(audit["fold_count"], 2)
        self.assertEqual(audit["sample_count"], 2)
        self.assertEqual(audit["all_component_sample_coverage"], 0.5)
        self.assertEqual(audit["maximum_exceedance_ratio"], 3.0)


if __name__ == "__main__":
    unittest.main()
