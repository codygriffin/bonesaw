from __future__ import annotations

import pathlib
import unittest

import mujoco
import numpy as np

from g1_contact_law_momentum_holdout import (
    FOOT_FRAMES,
    build_plant,
    plant_layout,
)
from g1_positive_reference_compliance_audit import (
    CPU_QUERY_DEADLINE_NS,
    PROFILES,
    PROJECTION_SWEEPS,
    REFERENCE_DOCUMENTATION,
    SOLVER_FAMILIES,
    law_cone_id,
    law_reduced_integrator_id,
    reference_contact_acceleration,
)
from g1_substepped_compliant_contact_holdout import FRESH_CONTACT_LAWS


ROOT = pathlib.Path(__file__).resolve().parents[2]
MODEL = ROOT / "benchmarks" / "cache" / "unitree-g1" / "g1_23dof_mode_10.urdf"


class G1PositiveReferenceComplianceAuditTests(unittest.TestCase):
    def test_profile_grid_and_reference_mapping_are_frozen(self) -> None:
        self.assertEqual(len(PROFILES), 12)
        self.assertEqual({profile.substeps for profile in PROFILES}, {5, 8, 16, 32, 64, 128})
        self.assertEqual(sum(profile.use_free_acceleration for profile in PROFILES), 6)
        self.assertEqual(SOLVER_FAMILIES, ("diagonal_effective_mass", "coupled_delassus"))
        self.assertEqual(PROJECTION_SWEEPS, 32)
        self.assertEqual(CPU_QUERY_DEADLINE_NS, 5_000_000)
        self.assertTrue(REFERENCE_DOCUMENTATION.startswith("https://mujoco.readthedocs.io/"))
        self.assertEqual(law_cone_id(FRESH_CONTACT_LAWS[0]), 0)
        self.assertEqual(law_cone_id(FRESH_CONTACT_LAWS[1]), 1)
        self.assertEqual(law_reduced_integrator_id(FRESH_CONTACT_LAWS[0]), 1)
        self.assertEqual(law_reduced_integrator_id(FRESH_CONTACT_LAWS[1]), 2)

    def test_reference_free_acceleration_is_finite_and_state_local(self) -> None:
        law = FRESH_CONTACT_LAWS[0]
        model, data = build_plant(MODEL, law)
        import bonesaw

        session = bonesaw.ContactTransitionModelSession(
            str(MODEL), [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
        )
        _, _, _, _, geoms, _ = plant_layout(model, list(session.joint_names()))
        mujoco.mj_forward(model, data)
        points = np.asarray([data.geom_xpos[geom] for geom in geoms], np.float64)
        points[:, 2] -= 0.005
        jacobian = np.empty((3, model.nv), np.float64)
        jacobian_dot = np.empty((3, model.nv), np.float64)
        first = np.empty((8, 3), np.float64)
        repeat = np.empty_like(first)
        reference_contact_acceleration(
            model, data, points, geoms, jacobian, jacobian_dot, first
        )
        reference_contact_acceleration(
            model, data, points, geoms, jacobian, jacobian_dot, repeat
        )
        np.testing.assert_array_equal(first, repeat)
        self.assertTrue(np.all(np.isfinite(first)))


if __name__ == "__main__":
    unittest.main()
