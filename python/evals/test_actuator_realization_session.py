from __future__ import annotations

import unittest

import numpy as np

from bonesaw import ActuatorRealizationSession


class ActuatorRealizationSessionTests(unittest.TestCase):
    def test_reset_is_complete_and_atomic(self) -> None:
        profiles = np.asarray([[5.0, 250.0], [10.0, 1_000.0]], np.float64)
        session = ActuatorRealizationSession(profiles, np.zeros(2, np.float64))
        requested = np.full((1, 2), 10.0, np.float64)
        available = np.full((1, 2), 20.0, np.float64)

        def run() -> np.ndarray:
            limited = np.empty_like(requested)
            realized = np.empty_like(requested)
            error = np.empty_like(requested)
            clipped = np.empty((1, 2), np.uint8)
            slew = np.empty((1, 2), np.uint8)
            timing = np.empty(1, np.uint64)
            calls = np.empty(1, np.uint64)
            bytes_ = np.empty(1, np.uint64)
            session.run_trace(
                requested,
                available,
                0.005,
                limited,
                realized,
                error,
                clipped,
                slew,
                timing,
                calls,
                bytes_,
            )
            np.testing.assert_array_equal(calls, 0)
            np.testing.assert_array_equal(bytes_, 0)
            return realized.copy()

        first = run()
        session.reset(np.zeros(2, np.float64))
        np.testing.assert_array_equal(run(), first)

        sentinel = np.asarray([3.0, 4.0], np.float64)
        session.reset(sentinel)
        with self.assertRaisesRegex(ValueError, "one finite effort"):
            session.reset(np.asarray([7.0, np.nan], np.float64))
        session.reset(sentinel)
        after_invalid = run()

        control = ActuatorRealizationSession(profiles, sentinel)
        limited = np.empty_like(requested)
        expected = np.empty_like(requested)
        error = np.empty_like(requested)
        clipped = np.empty((1, 2), np.uint8)
        slew = np.empty((1, 2), np.uint8)
        timing = np.empty(1, np.uint64)
        calls = np.empty(1, np.uint64)
        bytes_ = np.empty(1, np.uint64)
        control.run_trace(
            requested,
            available,
            0.005,
            limited,
            expected,
            error,
            clipped,
            slew,
            timing,
            calls,
            bytes_,
        )
        np.testing.assert_array_equal(after_invalid, expected)


if __name__ == "__main__":
    unittest.main()
