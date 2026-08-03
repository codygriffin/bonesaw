from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python" / "evals"))

import upkie_pinocchio_rolling_reference_r319 as r319


class UpkiePinocchioRollingReferenceR319Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = ROOT / "models" / "upkie" / "upkie.urdf"
        cls.input = (
            ROOT
            / "benchmarks"
            / "results"
            / "upkie-state-local-wbc-r123"
            / "upkie-state-local-wbc-raw.npz"
        )
        cls.corpus = r319.corpus_from_raw(cls.input)
        cls.joint_names = r319.model_joint_names(cls.model)

    def test_rolling_problem_reconstructs_bonesaw_hard_rows(self) -> None:
        _, coordinates, coefficients = r319.rolling_descriptors(
            self.model, self.joint_names, list(r319.CONTACT_FRAMES)
        )
        mass, bias, jacobian, contact_bias = r319.independent_products(
            self.model, self.joint_names, self.corpus
        )
        velocity = np.concatenate(
            (
                np.zeros(3),
                self.corpus["root_velocities"][0],
                self.corpus["v"][0],
            )
        )
        equality, target = r319.equality_problem(
            mass[0],
            bias[0],
            jacobian[0],
            contact_bias[0],
            velocity,
            coordinates,
            coefficients,
        )
        with np.load(
            ROOT
            / "benchmarks"
            / "results"
            / "upkie-placo-dynamic-reference-r317"
            / "bonesaw-raw.npz"
        ) as archive:
            solution = np.concatenate(
                (
                    archive["generalized_acceleration"][0],
                    archive["actuator_torque"][0],
                    archive["contact_force"][0].reshape(-1),
                )
            )
        self.assertLessEqual(float(np.max(np.abs(equality @ solution - target))), 2.0e-8)

    def test_constrained_reference_is_finite_and_hard_feasible(self) -> None:
        _, coordinates, coefficients = r319.rolling_descriptors(
            self.model, self.joint_names, list(r319.CONTACT_FRAMES)
        )
        mass, bias, jacobian, contact_bias = r319.independent_products(
            self.model, self.joint_names, self.corpus
        )
        velocity = np.concatenate(
            (
                np.zeros(3),
                self.corpus["root_velocities"][0],
                self.corpus["v"][0],
            )
        )
        equality, target = r319.equality_problem(
            mass[0],
            bias[0],
            jacobian[0],
            contact_bias[0],
            velocity,
            coordinates,
            coefficients,
        )
        desired = r319.reference_acceleration(self.corpus)[0]
        weight = r319.supported_weight(self.model)
        effort = r319.urdf_effort_limits(self.model, self.joint_names)
        solution = r319.constrained_lexicographic_solve(
            equality,
            target,
            r319.task_levels(desired, weight / 2.0),
            effort,
            r319.MAXIMUM_NORMAL_FORCE_MULTIPLE * weight,
        )
        self.assertTrue(np.all(np.isfinite(solution)))
        self.assertLessEqual(float(np.max(np.abs(equality @ solution - target))), 2.0e-8)
        margins = r319.hard_bound_margins(
            solution[None, :], effort, r319.MAXIMUM_NORMAL_FORCE_MULTIPLE * weight
        )
        self.assertTrue(r319.hard_bound_summary(margins)["all_satisfied"])

    def test_hierarchy_declares_all_five_authority_levels(self) -> None:
        desired = r319.reference_acceleration(self.corpus)[0]
        levels = r319.task_levels(desired, r319.supported_weight(self.model) / 2.0)
        self.assertEqual(len(levels), 5)
        self.assertEqual([matrix.shape[1] for matrix, _ in levels], [24] * 5)
        self.assertEqual([matrix.shape[0] for matrix, _ in levels], [4, 2, 6, 0, 12])


if __name__ == "__main__":
    unittest.main()
