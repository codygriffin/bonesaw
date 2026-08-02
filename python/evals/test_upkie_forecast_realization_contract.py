import unittest

import numpy as np

import upkie_forecast_realization_contract as contract


class ForecastRealizationContractTests(unittest.TestCase):
    def test_frozen_split_is_balanced_and_complete(self) -> None:
        names = {case.name for case in contract.case_matrix()}
        self.assertEqual(len(names), 20)
        self.assertEqual(len(contract.CALIBRATION_CASES), 10)
        self.assertEqual(len(names - contract.CALIBRATION_CASES), 10)

    def test_future_force_is_visible_but_excluded_from_quiescent_slice(self) -> None:
        ticks = 60
        paths = np.zeros((ticks, 8, 9), np.float64)
        paths[:, :, 0] = np.arange(1, 9) * 0.03
        valid = np.zeros(ticks, np.uint8)
        valid[0] = 1
        trace = {
            "execution_forecast_path_valid": valid,
            "execution_forecast_support_mask": np.full(ticks, 3, np.uint8),
            "execution_forecast_reduced_state": np.zeros((ticks, 8), np.float64),
            "execution_forecast_path": paths,
            "observed_contact_active": np.ones((ticks, 2), np.uint8),
            "external_force_world": np.zeros((ticks, 3), np.float64),
        }
        trace["external_force_world"][2, 0] = 1.0
        result = contract.collect_case_samples(trace)
        self.assertEqual(result["stable_samples_by_knot"], [1] * 8)
        self.assertEqual(result["quiescent_samples_by_knot"], [0] * 8)
        self.assertEqual(result["external_disturbance_samples_by_knot"], [1] * 8)

    def test_support_change_revokes_before_error_accounting(self) -> None:
        ticks = 60
        paths = np.zeros((ticks, 8, 9), np.float64)
        paths[:, :, 0] = np.arange(1, 9) * 0.03
        valid = np.zeros(ticks, np.uint8)
        valid[0] = 1
        observed = np.ones((ticks, 2), np.uint8)
        observed[4:, 1] = 0
        trace = {
            "execution_forecast_path_valid": valid,
            "execution_forecast_support_mask": np.full(ticks, 3, np.uint8),
            "execution_forecast_reduced_state": np.zeros((ticks, 8), np.float64),
            "execution_forecast_path": paths,
            "observed_contact_active": observed,
            "external_force_world": np.zeros((ticks, 3), np.float64),
        }
        result = contract.collect_case_samples(trace)
        self.assertEqual(result["stable_samples_by_knot"], [0] * 8)
        self.assertEqual(result["transition_censored_by_knot"], [1] * 8)


if __name__ == "__main__":
    unittest.main()
