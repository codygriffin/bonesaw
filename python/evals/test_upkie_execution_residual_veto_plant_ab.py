import unittest

import upkie_execution_residual_veto_plant_ab as plant_ab


class ExecutionResidualVetoPlantAbTests(unittest.TestCase):
    def test_boundary_delta_scores_recovery_and_new_fall_conservatively(self) -> None:
        fall = {"fell": True, "outcome": "FALL", "terminal_time_s": 2.0}
        recovered = {"fell": False, "outcome": "RECOVERED", "terminal_time_s": 6.0}
        self.assertEqual(plant_ab.boundary_delta(fall, recovered, 6.0), 4.0)
        self.assertEqual(plant_ab.boundary_delta(recovered, fall, 6.0), -4.0)


if __name__ == "__main__":
    unittest.main()
