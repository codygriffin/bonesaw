from __future__ import annotations

import math
import pathlib
import sys
import unittest

import numpy as np

EVALS = pathlib.Path(__file__).resolve().parent
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

import upkie_state_local_wbc_report as replay  # noqa: E402


MODEL = pathlib.Path("models/upkie/upkie.urdf").resolve()


def yaw_bases(ticks: int, targets: int, yaw: float) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    basis = np.asarray(
        [
            [cosine, sine, 0.0],
            [-sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        np.float64,
    )
    return np.ascontiguousarray(np.tile(basis, (ticks, targets, 1, 1)))


class UpkieContactBasisBoundaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.session = replay.new_session(MODEL)
        cls.joint_names = list(cls.session.joint_names)
        cls.frame_names = list(cls.session.frame_names)
        cls.contact_names = ["left_wheel_center", "right_wheel_center"]
        cls.frame_ids = np.asarray(
            [cls.frame_names.index(name) for name in cls.contact_names], np.int64
        )
        cls.modes, cls.coordinates, cls.coefficients = replay.rolling_descriptors(
            MODEL, cls.joint_names, cls.contact_names
        )
        cls.corpus = replay.make_corpus(
            16, cls.joint_names, cls.coordinates, cls.coefficients
        )

    def run_with(self, bases: np.ndarray, outputs=None):
        return replay.run_case(
            self.session,
            self.corpus,
            self.frame_ids,
            self.modes,
            self.coordinates,
            self.coefficients,
            outputs=outputs,
            contact_bases_world=bases,
        )

    def test_rotated_basis_is_repeatable_allocation_free_and_discriminating(self) -> None:
        identity = yaw_bases(16, 2, 0.0)
        rotated = yaw_bases(16, 2, 0.35)
        baseline = self.run_with(identity)
        first = self.run_with(rotated)
        second = self.run_with(rotated)
        self.assertTrue(np.array_equal(first["actuator_torque"], second["actuator_torque"]))
        self.assertFalse(np.array_equal(baseline["actuator_torque"], first["actuator_torque"]))
        self.assertEqual(int(np.sum(first["allocation_calls"])), 0)
        self.assertEqual(int(np.sum(first["allocated_bytes"])), 0)

    def test_invalid_basis_rejects_before_output_mutation(self) -> None:
        invalid = yaw_bases(16, 2, 0.35)
        invalid[7, 1, 1] = invalid[7, 1, 0]
        outputs = replay.allocate_outputs(self.session, 16)
        for value in outputs.values():
            value.fill(0x2A)
        before = {name: value.copy() for name, value in outputs.items()}
        with self.assertRaisesRegex(ValueError, "contact mode.*invalid"):
            self.run_with(invalid, outputs)
        for name, value in outputs.items():
            self.assertTrue(np.array_equal(value, before[name]), name)


if __name__ == "__main__":
    unittest.main()
