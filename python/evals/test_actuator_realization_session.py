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

    def test_passivity_trace_caps_power_and_rejects_late_invalid_atomically(self) -> None:
        profiles = np.full((2, 2), np.inf, np.float64)
        session = ActuatorRealizationSession(profiles, np.zeros(2, np.float64))
        requested = np.asarray([[4.0, -4.0]], np.float64)
        available = np.full((1, 2), 10.0, np.float64)
        velocity = np.asarray([[2.0, 2.0]], np.float64)

        def outputs(fill: float = 0.0) -> tuple[np.ndarray, ...]:
            return (
                np.full((1, 2), fill, np.float64),
                np.full((1, 2), fill, np.float64),
                np.full((1, 2), fill, np.float64),
                np.full((1, 2), fill, np.float64),
                np.full((1, 2), int(fill), np.uint8),
                np.full((1, 2), int(fill), np.uint8),
                np.full((1, 2), int(fill), np.uint8),
                np.full(1, int(fill), np.uint64),
                np.full(1, int(fill), np.uint64),
                np.full(1, int(fill), np.uint64),
            )

        out = outputs()
        session.run_passivity_limited_trace(
            requested, available, velocity, 0.0, 0.005, *out
        )
        np.testing.assert_array_equal(out[1], [[0.0, -4.0]])
        np.testing.assert_array_equal(out[3], [[0.0, -8.0]])
        np.testing.assert_array_equal(out[6], [[1, 0]])
        np.testing.assert_array_equal(out[8], 0)
        np.testing.assert_array_equal(out[9], 0)

        session.reset(np.zeros(2, np.float64))
        sentinel = outputs(7.0)
        invalid_velocity = velocity.copy()
        invalid_velocity[0, 1] = np.nan
        with self.assertRaisesRegex(ValueError, "inputs must be finite"):
            session.run_passivity_limited_trace(
                requested,
                available,
                invalid_velocity,
                0.0,
                0.005,
                *sentinel,
            )
        for value in sentinel:
            np.testing.assert_array_equal(value, np.full_like(value, 7))

        valid_after = outputs()
        session.run_passivity_limited_trace(
            requested, available, velocity, 0.0, 0.005, *valid_after
        )
        control = ActuatorRealizationSession(profiles, np.zeros(2, np.float64))
        expected = outputs()
        control.run_passivity_limited_trace(
            requested, available, velocity, 0.0, 0.005, *expected
        )
        for actual, wanted in zip(valid_after[:7], expected[:7], strict=True):
            np.testing.assert_array_equal(actual, wanted)


if __name__ == "__main__":
    unittest.main()
