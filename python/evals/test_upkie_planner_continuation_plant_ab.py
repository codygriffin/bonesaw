import unittest

import upkie_planner_continuation_plant_ab as plant_ab


class PlannerContinuationPlantAbTests(unittest.TestCase):
    def test_revision_and_frozen_matrix_are_current(self) -> None:
        self.assertTrue(plant_ab.REVISION.endswith("r166"))
        self.assertEqual(len(plant_ab.select_cases(None)), 20)

    def test_unknown_case_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            plant_ab.select_cases("not-a-retained-case")


if __name__ == "__main__":
    unittest.main()
