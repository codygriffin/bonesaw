from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

from upkie_live_plant_worker import (  # noqa: E402
    CONTROL_DT,
    PHYSICS_DT,
    PHYSICS_STEPS_PER_CONTROL,
    MAX_APPLICATION_OFFSET_M,
    LiveUpkiePlant,
)


EVALUATION_PROVENANCE = {
    "source": "evaluation_harness",
    "load_class": "declared_continuous_wrench",
    "force_frame": "world",
    "application_point_frame": "world",
}


class LiveUpkiePlantWorkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.worker = LiveUpkiePlant(ROOT / "models/upkie/upkie.urdf")

    def setUp(self) -> None:
        self.worker.reset()

    def test_offset_force_streams_its_physical_moment(self) -> None:
        body = "base"
        body_id = self.worker.body_by_name[body]
        force = np.asarray([2.0, 0.0, 0.0])
        point = self.worker.data.xipos[body_id] + np.asarray([0.0, 0.0, 0.2])
        result = self.worker.step(
            {
                "type": "step",
                "command_id": 1,
                "external_load": {
                    "active": True,
                    "body": body,
                    "force_world": force.tolist(),
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 1,
                },
            }
        )
        self.assertEqual(result["type"], "plant_state")
        self.assertGreater(result["external_load"]["moment_world_nm"][1], 0.35)
        self.assertLess(result["external_load"]["moment_world_nm"][1], 0.45)
        self.assertGreater(result["external_load"]["maximum_moment_nm"], 0.35)
        self.assertLess(result["external_load"]["application_offset_m"], 0.25)
        self.assertEqual(
            result["external_load"]["provenance"], EVALUATION_PROVENANCE
        )
        self.assertFalse(result["measured_impact_impulse"]["available"])
        self.assertFalse(result["unobserved_model_reserve"]["available"])

    def test_excessive_lever_is_rejected_without_advancing_plant(self) -> None:
        body = "base"
        body_id = self.worker.body_by_name[body]
        before_qpos = self.worker.data.qpos.copy()
        point = self.worker.data.xipos[body_id] + np.asarray(
            [0.0, 0.0, MAX_APPLICATION_OFFSET_M + 0.001]
        )
        result = self.worker.step(
            {
                "type": "step",
                "command_id": 2,
                "external_load": {
                    "active": True,
                    "body": body,
                    "force_world": [2.0, 0.0, 0.0],
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 2,
                },
            }
        )
        self.assertEqual(result["type"], "plant_error")
        self.assertIn("body-COM offset limit", result["message"])
        np.testing.assert_array_equal(self.worker.data.qpos, before_qpos)

    def test_live_rate_split_and_ground_contact_state_are_explicit(self) -> None:
        hello = self.worker.hello()
        self.assertEqual(hello["control_hz"], 50)
        self.assertEqual(hello["physics_hz"], 250)
        self.assertEqual(hello["physics_substeps_per_control"], 5)
        np.testing.assert_allclose(
            hello["simulator"]["ground_plane_point_world"], [0.0, 0.0, 0.0]
        )
        np.testing.assert_allclose(
            hello["simulator"]["ground_plane_normal_world"], [0.0, 0.0, 1.0]
        )
        before = float(self.worker.data.time)
        result = self.worker.step({"type": "step"})
        self.assertAlmostEqual(float(self.worker.data.time) - before, CONTROL_DT, places=12)
        self.assertEqual(result["simulator"]["physics_dt_s"], PHYSICS_DT)
        self.assertEqual(result["simulator"]["physics_substeps"], PHYSICS_STEPS_PER_CONTROL)
        self.assertGreaterEqual(result["metrics"]["ground_contact_count"], 1)
        self.assertTrue(np.isfinite(result["metrics"]["maximum_penetration_m"]))
        self.assertEqual(len(result["actuator_effort_nm"]), 6)
        self.assertEqual(
            len(result["generalized_acceleration"]), self.worker.model.nv
        )
        self.assertEqual(
            len(result["constraint_generalized_force"]), self.worker.model.nv
        )
        self.assertEqual(len(result["actuator_generalized_force"]), self.worker.model.nv)
        self.assertEqual(len(result["passive_generalized_force"]), self.worker.model.nv)
        self.assertEqual(len(result["bias_generalized_force"]), self.worker.model.nv)
        self.assertEqual(len(result["actuator_force"]), 6)
        self.assertEqual(len(result["center_of_mass_world"]), 3)
        self.assertTrue(np.all(np.isfinite(result["center_of_mass_world"])))
        self.assertEqual(len(result["simulator"]["solver_forward_inverse"]), 2)
        self.assertEqual(
            result["simulator"]["constraint_count"],
            len(result["constraint_force"]),
        )
        self.assertEqual(
            len(result["constraint_position"]),
            result["simulator"]["constraint_count"],
        )
        self.assertEqual(
            len(result["constraint_velocity"]),
            result["simulator"]["constraint_count"],
        )
        self.assertGreater(result["metrics"]["total_ground_normal_force_n"], 0.0)
        for key in (
            "kinetic_energy_j",
            "potential_energy_j",
        ):
            self.assertTrue(np.isfinite(result["simulator"][key]), key)
        self.assertEqual(result["simulator"]["warning_count"], 0)
        for key in (
            "maximum_abs_joint_speed_rad_s",
            "maximum_abs_actuator_effort_nm",
            "maximum_abs_generalized_acceleration",
            "maximum_abs_constraint_force",
        ):
            self.assertTrue(np.isfinite(result["metrics"][key]), key)

    def test_wrench_accepts_a_non_base_mesh_body(self) -> None:
        body = "left_femur"
        self.assertIn(body, self.worker.body_by_name)
        body_id = self.worker.body_by_name[body]
        point = self.worker.data.xipos[body_id].copy()
        result = self.worker.step(
            {
                "type": "step",
                "external_load": {
                    "active": True,
                    "body": body,
                    "force_world": [0.0, 1.0, 0.0],
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 3,
                },
            }
        )
        self.assertEqual(result["type"], "plant_state")
        self.assertEqual(result["external_load"]["body"], body)

    def test_load_without_provenance_is_rejected_without_advancing_plant(self) -> None:
        before_qpos = self.worker.data.qpos.copy()
        body = "base"
        body_id = self.worker.body_by_name[body]
        result = self.worker.step(
            {
                "type": "step",
                "external_load": {
                    "active": True,
                    "body": body,
                    "force_world": [1.0, 0.0, 0.0],
                    "application_point_world": self.worker.data.xipos[
                        body_id
                    ].tolist(),
                    "request_id": 4,
                },
            }
        )
        self.assertEqual(result["type"], "plant_error")
        self.assertIn("provenance", result["message"])
        np.testing.assert_array_equal(self.worker.data.qpos, before_qpos)


if __name__ == "__main__":
    unittest.main()
