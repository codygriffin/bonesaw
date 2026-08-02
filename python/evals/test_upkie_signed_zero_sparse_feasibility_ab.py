import unittest

import upkie_signed_zero_sparse_feasibility_ab as sparse_ab


class SignedZeroSparseFeasibilityAbTests(unittest.TestCase):
    def test_revision_and_matrix_are_frozen(self) -> None:
        self.assertTrue(sparse_ab.REVISION.endswith("r170"))
        self.assertEqual(len(sparse_ab.select_cases(None)), 20)


if __name__ == "__main__":
    unittest.main()
