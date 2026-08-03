import unittest

import numpy as np

import upkie_placo_dynamic_reference_r317 as r317


class UpkiePlacoDynamicReferenceTests(unittest.TestCase):
    def test_reference_layout_is_angular_linear_joints(self) -> None:
        corpus = {
            "q": np.zeros((2, 6)),
            "root_accelerations": np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
            "joint_accelerations": np.arange(12, dtype=np.float64).reshape(2, 6),
        }
        reference = r317.reference_acceleration(corpus)
        np.testing.assert_array_equal(reference[:, :3], 0.0)
        np.testing.assert_array_equal(reference[:, 3:6], corpus["root_accelerations"])
        np.testing.assert_array_equal(reference[:, 6:], corpus["joint_accelerations"])

    def test_execution_windows_preserve_every_state(self) -> None:
        ticks = 5
        corpus = {
            "q": np.zeros((ticks, 6)),
            "root_accelerations": np.zeros((ticks, 3)),
            "joint_accelerations": np.zeros((ticks, 6)),
        }
        raw = {
            "generalized_acceleration": np.zeros((ticks, 12)),
            "actuator_torque": np.zeros((ticks, 6)),
            "contact_force": np.ones((ticks, 2, 3)),
            "step_ns": np.arange(1, ticks + 1, dtype=np.uint64) * 1000,
        }
        windows = r317.execution_windows(corpus, raw, raw, width=2)
        self.assertEqual([(row["start"], row["stop"]) for row in windows], [(0, 2), (2, 4), (4, 5)])
        self.assertEqual(sum(row["bonesaw"]["successful"] for row in windows), ticks)


if __name__ == "__main__":
    unittest.main()
