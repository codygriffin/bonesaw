import unittest

import upkie_cross_profile_feasibility_witness_ab as witness_ab


class CrossProfileFeasibilityWitnessAbTests(unittest.TestCase):
    def test_revision_and_default_report_are_collision_free(self) -> None:
        self.assertTrue(witness_ab.REVISION.endswith("r174"))
        args = witness_ab.parse_args([])
        self.assertIn(witness_ab.REVISION, args.output)
        self.assertTrue(args.web_report.endswith("_R174.html"))

    def test_sum_field_uses_unsigned_exact_accumulation(self) -> None:
        self.assertEqual(
            witness_ab.sum_field({"work": [1, 2, 3]}, "work"),
            6,
        )


if __name__ == "__main__":
    unittest.main()
