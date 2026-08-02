from __future__ import annotations

import pathlib
import unittest

import numpy as np

from g1_terminal_velocity_box_audit import SOURCE_REPLAY, terminal_relevant_indices
from g1_substepped_compliant_contact_holdout import residual_half_width


ROOT = pathlib.Path(__file__).resolve().parents[2]


class G1TerminalVelocityBoxAuditTests(unittest.TestCase):
    def test_terminal_projection_is_explicit_and_deterministic(self) -> None:
        first = terminal_relevant_indices(29)
        repeat = terminal_relevant_indices(29)
        np.testing.assert_array_equal(first, repeat)
        np.testing.assert_array_equal(first[:3], [0, 1, 5])
        np.testing.assert_array_equal(first[3:], np.arange(6, 29))
        self.assertNotIn(2, first)
        self.assertNotIn(3, first)
        self.assertNotIn(4, first)

    def test_frozen_box_matches_replay_dimension_and_is_not_refit(self) -> None:
        replay = np.load(ROOT / SOURCE_REPLAY)
        residual = replay["mid_elliptic_implicitfast_compliant_residual"]
        width = residual_half_width(residual.shape[1])
        self.assertEqual(width.shape, (29,))
        np.testing.assert_array_equal(width[:3], width[0])
        np.testing.assert_array_equal(width[3:6], width[3])
        np.testing.assert_array_equal(width[6:], width[6])
        self.assertTrue(np.any(np.abs(residual) > width))

    def test_nonfloating_projection_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "floating root"):
            terminal_relevant_indices(5)


if __name__ == "__main__":
    unittest.main()
