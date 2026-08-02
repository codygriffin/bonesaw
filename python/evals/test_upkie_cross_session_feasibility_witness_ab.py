import unittest

import upkie_cross_session_feasibility_witness_ab as witness_ab


class CrossSessionFeasibilityWitnessAbTests(unittest.TestCase):
    def test_revision_and_default_report_are_collision_free(self) -> None:
        self.assertTrue(witness_ab.REVISION.endswith("r172"))
        args = witness_ab.parse_args([])
        self.assertIn(witness_ab.REVISION, args.output)
        self.assertTrue(args.web_report.endswith("_R172.html"))

    def test_case_filter_is_exact(self) -> None:
        cases = witness_ab.selected_cases("nominal,left_2n")
        self.assertEqual([case.name for case in cases], ["nominal", "left_2n"])

        with self.assertRaises(SystemExit):
            witness_ab.selected_cases("not_a_case")


if __name__ == "__main__":
    unittest.main()
