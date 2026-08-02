import unittest

import numpy as np

import upkie_execution_residual_monitor as monitor


class ExecutionResidualMonitorTests(unittest.TestCase):
    def test_third_holdout_is_disjoint_from_r161(self) -> None:
        names = {case.name for case in monitor.third_holdout_cases()}
        self.assertEqual(len(names), 13)
        self.assertFalse(names & {case.name for case in monitor.r161_cases()})

    def test_prediction_uses_final_acceleration_layout(self) -> None:
        state = np.zeros(8)
        acceleration = np.asarray([1.0, 2.0, 3.0, 4.0])
        predicted = monitor.predict_one_step(state, acceleration)
        expected_rates = np.asarray([1.0, 4.0, 2.0, 3.0]) * monitor.CONTROL_DT
        np.testing.assert_allclose(predicted[[1, 3, 5, 7]], expected_rates)
        np.testing.assert_allclose(
            predicted[[0, 2, 4, 6]],
            0.5 * monitor.CONTROL_DT**2 * np.asarray([1.0, 4.0, 2.0, 3.0]),
        )


if __name__ == "__main__":
    unittest.main()
