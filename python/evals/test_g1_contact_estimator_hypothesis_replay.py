from __future__ import annotations

import pathlib
import unittest

import numpy as np

from g1_contact_estimator_hypothesis_replay import (
    PROFILES,
    SOURCE_REPLAY,
    build_hypotheses,
)
from g1_coupled_contact_law_holdout import FRESH_CONTACT_LAWS


ROOT = pathlib.Path(__file__).resolve().parents[2]


class G1ContactEstimatorHypothesisReplayTests(unittest.TestCase):
    def test_profiles_are_explicit_construction_grid(self) -> None:
        self.assertEqual(len(PROFILES), 8)
        self.assertEqual(len({profile.name for profile in PROFILES}), len(PROFILES))
        self.assertTrue(all(profile.gap_error_m > 0.0 for profile in PROFILES))
        self.assertTrue(all(profile.normal_velocity_error_m_s > 0.0 for profile in PROFILES))
        self.assertTrue(all(profile.tangent_velocity_error_m_s > 0.0 for profile in PROFILES))

    def test_hypothesis_counts_and_values_are_finite(self) -> None:
        replay = np.load(ROOT / SOURCE_REPLAY)
        law = FRESH_CONTACT_LAWS[0]
        prefix = law.name
        points = replay[f"{prefix}_contact_points"][0]
        velocity = replay[f"{prefix}_prospective_velocity"][0]
        upper = replay[f"{prefix}_grouped_acceleration_impulse_upper"][0]
        local = build_hypotheses(points, velocity, upper, law, PROFILES[0])
        enumerated = build_hypotheses(points, velocity, upper, law, PROFILES[1])
        self.assertEqual(local[0].shape, (59, 8, 3))
        self.assertEqual(enumerated[0].shape, (315, 8, 3))
        self.assertEqual(local[1].shape, local[0].shape)
        self.assertEqual(enumerated[2].shape, (315, 8))
        for arrays in (local, enumerated):
            self.assertTrue(all(np.all(np.isfinite(array)) for array in arrays))


if __name__ == "__main__":
    unittest.main()
