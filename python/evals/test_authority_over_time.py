import unittest

import numpy as np

from authority_over_time import dwell_curve, ema, runs, signal_summary
from cpu_tail_stability import arrays_byte_exact


class AuthorityOverTimeTests(unittest.TestCase):
    def test_runs_keep_exact_half_open_episode_bounds(self) -> None:
        mask = np.asarray([False, True, True, False, True, False, True, True, True])
        self.assertEqual(runs(mask), [(1, 3), (4, 5), (6, 9)])

    def test_dwell_curve_does_not_turn_isolated_ticks_into_persistence(self) -> None:
        mask = np.asarray([True, False, True, True, True, True, False], dtype=bool)
        curve = dwell_curve(mask)
        self.assertEqual([row["episode_count"] for row in curve[:3]], [2, 1, 1])
        self.assertEqual(curve[2]["ticks_in_retained_episodes"], 4)

    def test_constant_pressure_has_exact_ema_and_duration(self) -> None:
        pressure = np.full(40, 0.9)
        np.testing.assert_array_equal(ema(pressure, 1.0), pressure)
        summary = signal_summary(pressure, 0.8, 1.0, "fixture")
        self.assertEqual(summary["warning"]["longest_run_ticks"], 40)
        self.assertAlmostEqual(summary["warning"]["longest_run_seconds"], 0.2)
        self.assertEqual(summary["critical"]["ticks"], 0)

    def test_tail_audit_compares_float_payload_bytes(self) -> None:
        left = np.asarray([0.0, -0.0], dtype=np.float64)
        right = np.asarray([0.0, 0.0], dtype=np.float64)
        self.assertFalse(arrays_byte_exact(left, right))
        self.assertTrue(arrays_byte_exact(left, left.copy()))


if __name__ == "__main__":
    unittest.main()
