from __future__ import annotations

import json
import pathlib
import unittest

import numpy as np


RESULT = pathlib.Path(
    "benchmarks/results/g1-residual-prototype-profile-r264/"
    "g1-residual-prototype-profile-metrics.json"
)
REPLAY = pathlib.Path(
    "benchmarks/results/g1-residual-prototype-profile-r264/"
    "g1-residual-prototype-profile.npz"
)


class ResidualPrototypeProfileR264Test(unittest.TestCase):
    def test_spent_cross_law_profile_is_frozen_without_authority(self) -> None:
        result = json.loads(RESULT.read_text())
        rehearsal = result["leave_one_law_out"]

        self.assertEqual(result["revision"], "g1-residual-prototype-profile-r264")
        self.assertTrue(result["source_immutable"])
        self.assertTrue(result["mechanism_passed"])
        self.assertTrue(result["spent_profile_passed"])
        self.assertTrue(result["profile_frozen_for_one_fresh_holdout"])
        self.assertFalse(result["authority_admitted"])
        self.assertEqual(result["physics_steps"], 0)
        self.assertEqual(result["policy_steps"], 0)
        self.assertEqual(result["plant_actions"], 0)

        self.assertEqual(rehearsal["rows"], 192)
        self.assertTrue(rehearsal["all_component_boxes_covered"])
        self.assertTrue(rehearsal["all_aggregate_boxes_covered"])
        self.assertTrue(rehearsal["all_rows_distance_supported"])
        self.assertTrue(rehearsal["zero_rust_allocation"])
        self.assertTrue(rehearsal["semantic_repeat"])
        self.assertTrue(rehearsal["rust_python_nearest_identity_match"])
        self.assertTrue(rehearsal["rust_python_distance_match"])
        self.assertEqual(rehearsal["selected_counts"], {"0": 189, "1": 0, "2": 3})
        self.assertEqual(rehearsal["nonzero_actions"], 3)
        self.assertEqual(
            rehearsal["strict_nonregressing_improving_nonzero_actions"], 3
        )
        self.assertEqual(rehearsal["selected_component_regression_rows"], 0)
        self.assertEqual(rehearsal["selected_aggregate_regression_rows"], 0)

        with np.load(REPLAY, allow_pickle=False) as replay:
            self.assertEqual(replay["prototype_features"].shape, (192, 55))
            self.assertEqual(
                replay["prototype_component_residuals"].shape, (192, 3, 6)
            )
            self.assertEqual(
                replay["leave_one_law_out_envelopes"].shape, (192, 3, 14)
            )


if __name__ == "__main__":
    unittest.main()
