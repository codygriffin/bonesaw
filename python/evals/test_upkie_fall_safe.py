from __future__ import annotations

import pathlib

import numpy as np
import pytest


def test_rust_fall_safe_lease_and_damping_are_explicit_and_bounded() -> None:
    bonesaw = pytest.importorskip("bonesaw")
    session = bonesaw.UpkieBalanceSession(
        str(pathlib.Path("models/upkie/upkie.urdf").resolve())
    )
    names = list(session.fall_safe_diagnostic_names)
    index = {name: slot for slot, name in enumerate(names)}
    diagnostics = np.empty(len(names), np.float64)
    angular = np.empty(3, np.float64)
    linear = np.empty(3, np.float64)
    joint = np.empty(6, np.float64)
    root = np.asarray([0.0, 0.0, 0.539], np.float64)
    quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
    twist = np.asarray([20.0, -2.0, 1.0, 20.0, -2.0, 1.0], np.float64)
    velocity = np.asarray([20.0, -2.0, 1.0, -20.0, 2.0, -1.0], np.float64)

    session.step_fall_safe_from_state(
        0.005,
        root,
        quaternion,
        twist,
        velocity,
        True,
        diagnostics,
        angular,
        linear,
        joint,
    )
    np.testing.assert_array_equal(angular, [-80.0, 16.0, -8.0])
    np.testing.assert_array_equal(linear, [-40.0, 8.0, -4.0])
    np.testing.assert_array_equal(joint, [-120.0, 16.0, -8.0, 120.0, -16.0, 8.0])

    session.reset()
    zero_twist = np.zeros(6, np.float64)
    zero_velocity = np.zeros(6, np.float64)
    for _ in range(5):
        session.step_fall_safe_from_state(
            0.005,
            root,
            quaternion,
            zero_twist,
            zero_velocity,
            False,
            diagnostics,
            angular,
            linear,
            joint,
        )
    assert diagnostics[index["fresh_command_authority"]] == 1.0
    for _ in range(7):
        session.step_fall_safe_from_state(
            0.005,
            root,
            quaternion,
            zero_twist,
            zero_velocity,
            False,
            diagnostics,
            angular,
            linear,
            joint,
        )
    assert diagnostics[index["fresh_command_authority"]] == 0.0
    assert diagnostics[index["solver_pressure"]] == 1.0
    assert diagnostics[index["limiting_reason_flags"]] > 0.0
