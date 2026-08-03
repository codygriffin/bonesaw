#!/usr/bin/env python3
"""Boundary regressions for the persistent Rust Upkie reference session."""

import pathlib
import unittest

import numpy as np

import bonesaw


MODEL = pathlib.Path(__file__).parents[2] / "models/upkie/upkie.urdf"


class UpkieBalanceSessionTests(unittest.TestCase):
    @staticmethod
    def balanced_state(session: bonesaw.UpkieBalanceSession) -> tuple[np.ndarray, np.ndarray]:
        root = np.asarray([0.0, 0.0, 0.539], np.float64)
        q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
        balanced_root = np.empty(3, np.float64)
        balanced_q = np.empty(6, np.float64)
        session.balanced_standing(root, q, balanced_root, balanced_q)
        return balanced_root, balanced_q

    @staticmethod
    def rooted_capture_step(
        session: bonesaw.UpkieBalanceSession,
        root: np.ndarray,
        q: np.ndarray,
        *,
        root_linear_velocity_x: float = 0.0,
        control_world_from_odom_x: float = 0.0,
        map_from_odom_x: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        wheel = np.empty(2, np.float64)
        diagnostics = np.empty(16, np.float64)
        root_twist = np.zeros(6, np.float64)
        root_twist[3] = root_linear_velocity_x
        session.step_rooted_capture_from_state(
            0.005,
            0.0,
            0.0,
            np.asarray([control_world_from_odom_x, 0.0, 0.0], np.float64),
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            np.asarray([map_from_odom_x, 0.0, 0.0], np.float64),
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            root,
            np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
            root_twist,
            q,
            np.zeros(6, np.float64),
            wheel,
            diagnostics,
        )
        return wheel, diagnostics

    def test_balanced_seed_is_symmetric_exactly_repeatable_and_centered(self) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        root = np.asarray([0.0, 0.0, 0.539], np.float64)
        q = np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)
        root_a = np.empty(3, np.float64)
        q_a = np.empty(6, np.float64)
        root_b = np.empty(3, np.float64)
        q_b = np.empty(6, np.float64)

        error_a = session.balanced_standing(root, q, root_a, q_a)
        error_b = session.balanced_standing(root, q, root_b, q_b)

        self.assertLessEqual(abs(error_a), 1.0e-6)
        self.assertEqual(error_a, error_b)
        np.testing.assert_array_equal(root_a, root_b)
        np.testing.assert_array_equal(q_a, q_b)
        self.assertAlmostEqual(q_a[0], -q_a[3], places=12)
        self.assertAlmostEqual(q_a[1], -q_a[4], places=12)
        self.assertEqual(tuple(session.coordinates), (2, 5))

    def test_reset_replays_zero_error_reference_step(self) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        velocity = np.zeros(6, np.float64)
        wheel_a = np.empty(2, np.float64)
        wheel_b = np.empty(2, np.float64)

        ground_velocity_a = session.step(0.005, 0.0, 0.0, 0.0, velocity, wheel_a)
        session.reset()
        ground_velocity_b = session.step(0.005, 0.0, 0.0, 0.0, velocity, wheel_b)

        self.assertEqual(ground_velocity_a, 0.0)
        self.assertEqual(ground_velocity_b, 0.0)
        np.testing.assert_array_equal(wheel_a, wheel_b)
        np.testing.assert_array_equal(wheel_a, np.zeros(2, np.float64))
        self.assertEqual(session.integral_velocity, 0.0)

    def test_wheel_load_reserve_binding_is_dormant_and_allocation_free_nominally(
        self,
    ) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        root, q = self.balanced_state(session)
        diagnostics = np.empty(
            len(session.wheel_load_reserve_diagnostic_names), np.float64
        )
        angular = np.empty(3, np.float64)
        linear = np.empty(3, np.float64)
        step_ns, allocation_calls, allocated_bytes = (
            session.step_wheel_load_reserve_from_state(
                0.004,
                True,
                0.0,
                root,
                np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
                np.zeros(6, np.float64),
                q,
                np.zeros(6, np.float64),
                np.asarray([26.0, 26.0], np.float64),
                diagnostics,
                angular,
                linear,
            )
        )
        index = {
            name: offset
            for offset, name in enumerate(
                session.wheel_load_reserve_diagnostic_names
            )
        }
        self.assertGreater(step_ns, 0)
        self.assertEqual(allocation_calls, 0)
        self.assertEqual(allocated_bytes, 0)
        self.assertEqual(diagnostics[index["evidence_available"]], 1.0)
        self.assertEqual(diagnostics[index["active"]], 0.0)
        self.assertEqual(diagnostics[index["authority"]], 0.0)
        # Candidate buffers may track the kinematic baseline while dormant;
        # the explicit zero authority is the sole execution blend.
        self.assertTrue(np.all(np.isfinite(angular)))
        self.assertTrue(np.all(np.isfinite(linear)))

    def test_wheel_load_reserve_binding_targets_the_measured_weaker_wheel(
        self,
    ) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        root, q = self.balanced_state(session)
        diagnostics = np.empty(
            len(session.wheel_load_reserve_diagnostic_names), np.float64
        )
        angular = np.empty(3, np.float64)
        linear = np.empty(3, np.float64)
        for _ in range(13):
            session.step_wheel_load_reserve_from_state(
                0.004,
                True,
                0.0,
                root,
                np.asarray([1.0, 0.0, 0.0, 0.0], np.float64),
                np.zeros(6, np.float64),
                q,
                np.zeros(6, np.float64),
                np.asarray([18.0, 34.0], np.float64),
                diagnostics,
                angular,
                linear,
            )
        index = {
            name: offset
            for offset, name in enumerate(
                session.wheel_load_reserve_diagnostic_names
            )
        }
        self.assertEqual(diagnostics[index["weaker_support_index"]], 0.0)
        self.assertGreater(diagnostics[index["authority"]], 0.0)
        restoring_zmp = diagnostics[index["restoring_zmp_m"]]
        acceleration = diagnostics[
            index["commanded_lateral_acceleration_m_s2"]
        ]
        self.assertNotEqual(restoring_zmp, 0.0)
        self.assertLess(restoring_zmp * acceleration, 0.0)

    def test_rejects_degenerate_root_quaternion(self) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        wheel = np.empty(2, np.float64)
        pitch = np.empty(1, np.float64)
        with self.assertRaisesRegex(ValueError, "quaternion is degenerate"):
            session.step_from_state(
                0.005,
                0.0,
                0.0,
                0.0,
                np.asarray([0.0, 0.0, 0.539], np.float64),
                np.zeros(4, np.float64),
                np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64),
                np.zeros(6, np.float64),
                wheel,
                pitch,
            )

    def test_rooted_capture_map_jump_changes_reporting_not_command(self) -> None:
        baseline = bonesaw.UpkieBalanceSession(str(MODEL))
        jumped = bonesaw.UpkieBalanceSession(str(MODEL))
        root, q = self.balanced_state(baseline)
        wheel_a, diagnostics_a = self.rooted_capture_step(baseline, root, q)
        wheel_b, diagnostics_b = self.rooted_capture_step(
            jumped, root, q, map_from_odom_x=10.0
        )

        np.testing.assert_array_equal(wheel_a, wheel_b)
        np.testing.assert_array_equal(diagnostics_a[:11], diagnostics_b[:11])
        np.testing.assert_array_equal(diagnostics_a[12:], diagnostics_b[12:])
        self.assertAlmostEqual(diagnostics_b[11] - diagnostics_a[11], 10.0)

    def test_rooted_capture_uses_velocity_and_releases_station(self) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        root, q = self.balanced_state(session)
        wheel, diagnostics = self.rooted_capture_step(
            session,
            root,
            q,
            root_linear_velocity_x=0.6,
            control_world_from_odom_x=-2.0,
        )
        named = dict(zip(session.capture_diagnostic_names, diagnostics, strict=True))

        self.assertGreater(named["capture_position_control_world_m"], 0.09)
        self.assertEqual(named["capture_pressure"], 1.0)
        self.assertEqual(named["station_authority"], 0.0)
        self.assertGreater(named["commanded_ground_velocity_mps"], 0.0)
        self.assertTrue(np.all(np.isfinite(wheel)))

    def test_smooth_odom_transform_moves_only_station_preference(self) -> None:
        baseline = bonesaw.UpkieBalanceSession(str(MODEL))
        shifted = bonesaw.UpkieBalanceSession(str(MODEL))
        root, q = self.balanced_state(baseline)
        _, diagnostics_a = self.rooted_capture_step(baseline, root, q)
        _, diagnostics_b = self.rooted_capture_step(
            shifted, root, q, control_world_from_odom_x=1.0
        )
        names = tuple(baseline.capture_diagnostic_names)
        station_target = names.index("station_position_control_world_m")
        capture_position = names.index("capture_position_control_world_m")
        self.assertAlmostEqual(diagnostics_b[station_target] - diagnostics_a[station_target], 1.0)
        self.assertEqual(diagnostics_b[capture_position], diagnostics_a[capture_position])

    def test_capture_velocity_fraction_is_explicit_and_validated(self) -> None:
        session = bonesaw.UpkieBalanceSession(str(MODEL))
        self.assertEqual(session.capture_velocity_fraction, 0.2)
        session.capture_velocity_fraction = 0.4
        self.assertEqual(session.capture_velocity_fraction, 0.4)
        for invalid in (-0.01, 1.01, float("nan")):
            with self.assertRaisesRegex(ValueError, r"inside \[0, 1\]"):
                session.capture_velocity_fraction = invalid
        self.assertEqual(session.capture_velocity_fraction, 0.4)


if __name__ == "__main__":
    unittest.main()
