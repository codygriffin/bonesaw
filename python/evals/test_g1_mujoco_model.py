from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np


EVALS = pathlib.Path(__file__).resolve().parent
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

try:
    import mujoco
except ImportError:  # pragma: no cover - exercised by environments without MuJoCo
    mujoco = None  # type: ignore[assignment]

import g1_mujoco_model as fixture


@unittest.skipIf(mujoco is None, "MuJoCo is not installed")
class G1MuJoCoModelTests(unittest.TestCase):
    def test_primitive_fixture_has_floating_root_hinges_and_contacts(self) -> None:
        model, data = fixture.load_g1_model(fixture.default_urdf_path())

        self.assertAlmostEqual(float(model.opt.timestep), 0.004)
        self.assertEqual(model.nq, 30)  # 7 free-root coordinates + 23 hinges
        self.assertEqual(model.nv, 29)  # 6 free-root velocities + 23 hinges
        self.assertEqual(model.nu, 23)
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        free = int(mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(int(np.count_nonzero(model.jnt_type == hinge)), 23)
        root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self.assertGreaterEqual(root_id, 0)
        self.assertEqual(int(model.jnt_type[root_id]), free)

        self.assertTrue(np.isfinite(model.body_mass).all())
        self.assertGreater(float(np.sum(model.body_mass)), 1.0)
        self.assertTrue(np.isfinite(model.jnt_range).all())
        self.assertTrue(np.isfinite(model.actuator_ctrlrange).all())

        ground_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, fixture.GROUND_GEOM_NAME
        )
        self.assertGreaterEqual(ground_id, 0)
        for name in fixture.FOOT_SPHERE_NAMES:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            self.assertGreaterEqual(geom_id, 0)
            self.assertEqual(int(model.geom_type[geom_id]), int(mujoco.mjtGeom.mjGEOM_SPHERE))

        self.assertGreaterEqual(data.ncon, len(fixture.FOOT_SPHERE_NAMES))
        distances = np.asarray([data.contact[index].dist for index in range(data.ncon)])
        self.assertTrue(np.isfinite(distances).all())

    def test_generated_xml_contains_no_mesh_dependency(self) -> None:
        xml = fixture.build_g1_mjcf()
        self.assertNotIn("<mesh", xml)
        self.assertEqual(xml.count('type="hinge"'), 23)
        self.assertEqual(sum(xml.count(name) for name in fixture.FOOT_SPHERE_NAMES), 4)


if __name__ == "__main__":
    unittest.main()
