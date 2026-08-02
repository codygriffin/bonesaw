import unittest

import upkie_equality_first_feasibility_plant_ab as plant_ab


class EqualityFirstFeasibilityPlantAbTests(unittest.TestCase):
    def test_revision_is_unique_and_current(self) -> None:
        self.assertTrue(plant_ab.REVISION.endswith("r165"))


if __name__ == "__main__":
    unittest.main()
