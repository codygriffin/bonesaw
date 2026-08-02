import unittest

import numpy as np

from coincident_step_limit_audit import difference


class CoincidentStepLimitAuditTest(unittest.TestCase):
    def test_difference_counts_and_bounds_numeric_changes(self) -> None:
        result = difference(
            np.asarray([1, 2, 3], dtype=np.uint16),
            np.asarray([1, 4, 2], dtype=np.uint16),
        )
        self.assertEqual(result["changed_samples"], 2)
        self.assertEqual(result["maximum_absolute_delta"], 2.0)


if __name__ == "__main__":
    unittest.main()
