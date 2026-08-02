from __future__ import annotations

import pathlib

import numpy as np
import pytest


def test_wheel_contact_truth_uses_named_subtrees_and_world_contact() -> None:
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("bonesaw")
    from upkie_mujoco_plant_report import (
        make_plant,
        measured_wheel_ground_contacts,
        wheel_contact_body_sets,
    )

    model, data = make_plant(pathlib.Path("models/upkie/upkie.urdf").resolve())
    wheel_roots = np.asarray(
        [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_mj5208_rotor"
            ),
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_mj5208_rotor"
            ),
        ],
        np.int64,
    )
    body_sets = wheel_contact_body_sets(model, wheel_roots)
    left_tire = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_tire"
    )
    right_tire = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "right_wheel_tire"
    )
    assert left_tire in body_sets[0] and left_tire not in body_sets[1]
    assert right_tire in body_sets[1] and right_tire not in body_sets[0]
    np.testing.assert_array_equal(
        measured_wheel_ground_contacts(model, data, body_sets), [1, 1]
    )

    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    data.qpos[model.jnt_qposadr[root] + 2] += 0.5
    mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(
        measured_wheel_ground_contacts(model, data, body_sets), [0, 0]
    )
