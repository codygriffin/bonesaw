import unittest

import numpy as np

from g1_correlated_state_exemplar_profile_r261 import (
    exemplar_states,
    fit_exemplars,
    predicted_transition_state,
)


class CorrelatedStateExemplarProfileTest(unittest.TestCase):
    def test_source_exemplar_reconstructs_complete_candidate_state(self) -> None:
        rows, candidates, joints = 16, 3, 2
        root_state = np.zeros((rows, 3), np.float64)
        root_state[:, 0] = 0.4
        q = np.zeros((rows, joints), np.float64)
        velocity = np.zeros((rows, 6 + joints), np.float64)
        acceleration = np.zeros((rows, candidates, 6 + joints), np.float64)
        predicted = predicted_transition_state(root_state, q, velocity, acceleration)
        actual_root = predicted[0] + np.arange(rows)[:, None, None] * 1.0e-4
        actual_q = predicted[1] + np.arange(rows)[:, None, None] * 2.0e-4
        actual_v = predicted[2] - np.arange(rows)[:, None, None] * 3.0e-4
        groups = np.arange(rows, dtype=np.int64)
        profile = fit_exemplars(
            groups, *predicted, actual_root, actual_q, actual_v
        )
        for row in range(rows):
            root, position, joint_velocity = exemplar_states(
                row,
                np.asarray([row]),
                *predicted,
                profile,
            )
            np.testing.assert_allclose(root[:, 0], actual_root[row], atol=1.0e-15)
            np.testing.assert_allclose(position[:, 0], actual_q[row], atol=1.0e-15)
            np.testing.assert_allclose(
                joint_velocity[:, 0], actual_v[row], atol=1.0e-15
            )


if __name__ == "__main__":
    unittest.main()
