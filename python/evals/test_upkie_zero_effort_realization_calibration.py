from __future__ import annotations

import unittest

import numpy as np

import upkie_zero_effort_realization_calibration as calibration


def samples(pressure_error: np.ndarray, support_masks: np.ndarray) -> dict[str, np.ndarray]:
    count = len(pressure_error)
    return {
        "predicted_acceleration": np.zeros((count, 8), np.float64),
        "realized_acceleration": np.zeros((count, 8), np.float64),
        "acceleration_error": np.zeros((count, 8), np.float64),
        "pressure_error": pressure_error,
        "support_masks": support_masks,
        "score_ns": np.arange(1, count + 1, dtype=np.uint64),
        "allocation_calls": np.zeros(count, np.uint64),
        "allocated_bytes": np.zeros(count, np.uint64),
    }


class ZeroEffortRealizationCalibrationTests(unittest.TestCase):
    def test_summary_keeps_physical_support_and_allocation_witnesses(self) -> None:
        sample_set = samples(
            np.asarray(
                [
                    [1.0, -2.0, 0.0, 0.5, 0.0],
                    [-3.0, 1.0, 0.0, -0.5, 0.0],
                    [0.0, 0.0, 4.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 5.0, 0.0],
                ],
                np.float64,
            ),
            np.asarray([0, 1, 2, 3], np.uint8),
        )

        summary = calibration.summarize_samples(sample_set)

        self.assertEqual(summary["sample_count"], 4)
        self.assertEqual(
            summary["physical_support_mask_counts"],
            {"0": 1, "1": 1, "2": 1, "3": 1},
        )
        self.assertEqual(
            summary["pressure_error_maximum_absolute"],
            [3.0, 2.0, 4.0, 5.0, 0.0],
        )
        self.assertEqual(
            summary["pressure_error_positive_maximum"],
            [1.0, 1.0, 4.0, 5.0, 0.0],
        )
        self.assertTrue(summary["zero_allocation"])

    def test_leave_one_case_out_never_learns_from_held_case(self) -> None:
        case_sets = {
            "small": samples(
                np.full((2, len(calibration.PRESSURE_NAMES)), 1.0),
                np.asarray([3, 3], np.uint8),
            ),
            "large": samples(
                np.full((1, len(calibration.PRESSURE_NAMES)), -3.0),
                np.asarray([0], np.uint8),
            ),
        }

        audit = calibration.leave_one_case_out(case_sets)

        self.assertEqual(audit["small"]["coverage"], 1.0)
        self.assertEqual(audit["large"]["coverage"], 0.0)
        self.assertEqual(audit["large"]["maximum_exceedance"], 2.0)
        self.assertEqual(
            audit["large"]["calibrated_absolute_pressure_bound"],
            [1.0] * len(calibration.PRESSURE_NAMES),
        )


if __name__ == "__main__":
    unittest.main()
