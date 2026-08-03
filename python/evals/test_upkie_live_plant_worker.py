from __future__ import annotations

import pathlib
import sys
import unittest
from unittest.mock import patch

import mujoco
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
        self.worker.paused = False

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
        self.assertEqual(hello["controller_profile"], "production_default")
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
        # The live controller consumes the measured MuJoCo wheel mask through
        # the Rust contact-observation boundary; it must not silently reuse an
        # authored two-wheel stance when a wheel has left the plane.
        np.testing.assert_array_equal(
            result["wbc_observed_contact_active"],
            result["metrics"]["wbc_observed_contact_active"],
        )
        np.testing.assert_array_equal(
            result["wbc_debounced_contact_active"],
            result["metrics"]["wbc_debounced_contact_active"],
        )
        np.testing.assert_array_equal(
            result["wbc_hard_contact_active"],
            result["metrics"]["wbc_hard_contact_active"],
        )
        np.testing.assert_array_equal(
            result["wbc_hard_contact_executable"],
            result["metrics"]["wbc_hard_contact_executable"],
        )
        self.assertTrue(
            np.all(
                np.asarray(result["wbc_hard_contact_executable"], np.uint8)
                <= np.asarray(result["wbc_hard_contact_active"], np.uint8)
            )
        )
        self.assertTrue(result["metrics"]["wbc_observed_contact_available"])
        self.assertIn(result["metrics"]["wbc_raw_status_code"], (0, 1, 2, 3))
        self.assertEqual(result["metrics"]["wbc_allocation_calls"], 0)
        self.assertEqual(result["metrics"]["wbc_allocated_bytes"], 0)
        for key in (
            "wbc_maximum_constraint_violation",
            "wbc_dynamics_residual",
            "wbc_contact_residual",
        ):
            self.assertTrue(np.isfinite(result["metrics"][key]), key)

        self.assertTrue(
            np.all(
                np.asarray(result["wbc_hard_contact_active"], np.uint8)
                <= np.asarray(result["wbc_observed_contact_active"], np.uint8)
            )
        )
        np.testing.assert_array_equal(result["wbc_debounced_contact_active"], [0, 0])
        self.assertLessEqual(
            result["metrics"]["wbc_support_active_count"],
            int(np.sum(result["wbc_observed_contact_active"])),
        )
        self.assertIn(result["metrics"]["wbc_contact_observation_status"], (0, 1, 2, 3))
        self.assertIn(result["metrics"]["wbc_contact_observation_provenance"], (0, 1, 2, 3))
        settled = result
        for _ in range(3):
            settled = self.worker.step({"type": "step"})
        np.testing.assert_array_equal(settled["wbc_debounced_contact_active"], [1, 1])
        self.assertGreaterEqual(settled["metrics"]["wbc_support_active_count"], 1)
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
            "wbc_observed_wheel_normal_force_n",
            "physics_wheel_normal_force_n",
            "wbc_predicted_normal_force_n",
        ):
            self.assertEqual(len(result[key]), 2, key)
            self.assertTrue(np.all(np.isfinite(result[key])), key)
            self.assertTrue(np.all(np.asarray(result[key]) >= 0.0), key)
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

    def test_public_inner_rate_runs_five_wbc_ticks_per_stream_tick(self) -> None:
        worker = LiveUpkiePlant(
            ROOT / "models/upkie/upkie.urdf",
            stream_dt=0.020,
            control_dt=0.004,
            physics_dt=0.001,
        )
        hello = worker.hello()
        self.assertEqual(hello["stream_hz"], 50)
        self.assertEqual(hello["control_hz"], 250)
        self.assertEqual(hello["physics_hz"], 1000)
        self.assertEqual(hello["physics_substeps_per_control"], 4)
        self.assertEqual(
            hello["contact_observation"],
            {
                "sample_hz": 1000,
                "consumed_hz": 250,
                "window_size": 4,
                "wbc_source": "latest_completed_1000hz_substep",
                "prestart_samples": 0,
            },
        )
        observation_tick = worker.controller.contact_observation_tick
        before = float(worker.data.time)
        state = worker.step({"type": "step"})
        self.assertAlmostEqual(float(worker.data.time) - before, 0.020, places=12)
        self.assertEqual(
            worker.controller.contact_observation_tick - observation_tick,
            5,
        )
        self.assertEqual(state["simulator"]["physics_frame_index"], 20)
        self.assertEqual(state["simulator"]["physics_dt_s"], 0.001)
        self.assertEqual(state["simulator"]["control_dt_s"], 0.004)
        self.assertEqual(state["simulator"]["physics_substeps"], 4)

    def test_rate_periods_require_positive_integer_ratios(self) -> None:
        model = ROOT / "models/upkie/upkie.urdf"
        invalid = (
            {"stream_dt": 0.0},
            {"control_dt": float("nan")},
            {"physics_dt": -0.001},
            {"stream_dt": 0.020, "control_dt": 0.003},
            {"control_dt": 0.004, "physics_dt": 0.0015},
        )
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ValueError):
                LiveUpkiePlant(model, **options)

    def test_controller_overrides_are_explicit_and_survive_reset(self) -> None:
        worker = LiveUpkiePlant(
            ROOT / "models/upkie/upkie.urdf",
            controller_options={"maximum_feasibility_iterations": 32},
        )
        self.assertEqual(worker.hello()["controller_profile"], "evaluation_override")
        worker.reset()
        self.assertEqual(worker.hello()["controller_profile"], "evaluation_override")
        mode_worker = LiveUpkiePlant(
            ROOT / "models/upkie/upkie.urdf",
            controller_balance_mode="planar_capture",
        )
        self.assertEqual(
            mode_worker.hello()["controller_profile"], "evaluation_override"
        )

    def test_contact_prestart_priming_is_bounded_and_causal(self) -> None:
        worker = LiveUpkiePlant(
            ROOT / "models/upkie/upkie.urdf",
            controller_options={"contact_observation_prestart_samples": 3},
        )
        self.assertEqual(
            worker.hello()["contact_observation"]["prestart_samples"], 3
        )
        result = worker.step({"type": "step"})
        np.testing.assert_array_equal(result["wbc_observed_contact_active"], [1, 1])
        np.testing.assert_array_equal(result["wbc_debounced_contact_active"], [1, 1])
        np.testing.assert_array_equal(result["wbc_hard_contact_active"], [1, 1])
        self.assertEqual(result["metrics"]["wbc_raw_status"], "Solved")
        self.assertEqual(result["metrics"]["wbc_contact_observation_status"], 0)
        self.assertEqual(result["metrics"]["wbc_contact_observation_provenance"], 0)

    def test_contact_ring_samples_each_substep_and_wbc_uses_boundary_mask(self) -> None:
        scripted = [
            [1, 1],  # boundary observation consumed by the first WBC call
            [1, 1],
            [1, 0],
            [0, 1],
            [0, 0],
            [1, 1],  # five post-step samples in the first control window
            [0, 0],
            [0, 1],
            [0, 1],
            [1, 1],
            [1, 1],
        ]
        calls: list[list[int]] = []

        def scripted_extractor(_model, _data, _body_sets, active):
            index = len(calls)
            self.assertLess(index, len(scripted))
            active[...] = scripted[index]
            calls.append(active.tolist())
            return active

        with patch(
            "upkie_mujoco_plant_report.measured_wheel_ground_contacts_into",
            side_effect=scripted_extractor,
        ):
            first = self.worker.step({"type": "step", "command_id": 20})
            second = self.worker.step({"type": "step", "command_id": 21})

        self.assertEqual(len(calls), 1 + 2 * PHYSICS_STEPS_PER_CONTROL)
        self.assertEqual(first["wbc_observed_contact_active"], scripted[0])
        self.assertEqual(
            first["wbc_observation"],
            {
                "contact_active": scripted[0],
                "wheel_normal_force_n": first[
                    "wbc_observed_wheel_normal_force_n"
                ],
                "physics_frame_index": 0,
                "source": "latest_completed_250hz_substep",
            },
        )
        self.assertEqual(
            first["simulator"]["contact_window_masks"], scripted[1:6]
        )
        self.assertEqual(first["simulator"]["physics_frame_index"], 5)
        self.assertEqual(first["simulator"]["contact_window_frame_start"], 1)
        self.assertEqual(first["simulator"]["contact_window_frame_end"], 5)
        self.assertTrue(first["simulator"]["contact_window_valid"])
        self.assertEqual(first["simulator"]["contact_window_loss_mask"], [1, 1])
        self.assertEqual(first["simulator"]["contact_window_gain_mask"], [1, 1])
        self.assertEqual(
            first["simulator"]["contact_window_loss_masks"],
            [[0, 0], [0, 1], [1, 0], [0, 1], [0, 0]],
        )
        self.assertEqual(
            first["simulator"]["contact_window_gain_masks"],
            [[0, 0], [0, 0], [0, 1], [0, 0], [1, 1]],
        )
        self.assertEqual(first["physics_contact_active"], scripted[5])
        self.assertEqual(
            len(first["simulator"]["wheel_normal_force_window_n"]),
            PHYSICS_STEPS_PER_CONTROL,
        )
        self.assertTrue(
            np.all(
                np.asarray(first["simulator"]["wheel_normal_force_window_n"])
                >= 0.0
            )
        )
        self.assertEqual(second["wbc_observed_contact_active"], scripted[5])
        self.assertEqual(second["wbc_observation"]["physics_frame_index"], 5)

    def test_wbc_hard_support_fails_closed_when_mujoco_loses_both_wheels(self) -> None:
        root = mujoco.mj_name2id(
            self.worker.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
        )
        self.worker.data.qpos[self.worker.model.jnt_qposadr[root] + 2] += 0.5
        mujoco.mj_forward(self.worker.model, self.worker.data)
        result = self.worker.step({"type": "step"})
        np.testing.assert_array_equal(result["wbc_observed_contact_active"], [0, 0])
        np.testing.assert_array_equal(result["wbc_hard_contact_active"], [0, 0])
        self.assertEqual(result["metrics"]["wbc_support_active_count"], 0)
        self.assertEqual(result["metrics"]["wbc_contact_observation_status"], 0)

    def test_pause_freezes_mujoco_time_but_keeps_stream_heartbeat(self) -> None:
        running = self.worker.step({"type": "step", "command_id": 10})
        time_before = float(self.worker.data.time)
        tick_before = int(running["tick"])
        qpos_before = self.worker.data.qpos.copy()
        qvel_before = self.worker.data.qvel.copy()

        paused = self.worker.step(
            {"type": "step", "command_id": 11, "paused": True}
        )
        self.assertEqual(paused["type"], "plant_state")
        self.assertTrue(paused["paused"])
        self.assertTrue(paused["simulator"]["paused"])
        self.assertTrue(paused["metrics"]["paused"])
        self.assertEqual(paused["metrics"]["wbc_status"], "paused")
        self.assertFalse(paused["metrics"]["wbc_observed_contact_available"])
        self.assertEqual(paused["metrics"]["wbc_raw_status"], "paused")
        self.assertEqual(paused["metrics"]["wbc_raw_status_code"], -1)
        self.assertEqual(paused["metrics"]["wbc_allocation_calls"], 0)
        self.assertEqual(paused["metrics"]["wbc_allocated_bytes"], 0)
        self.assertFalse(paused["simulator"]["contact_window_valid"])
        self.assertEqual(paused["physics_contact_active"], [0, 0])
        np.testing.assert_array_equal(
            paused["wbc_hard_contact_executable"], [0, 0]
        )
        np.testing.assert_array_equal(paused["wbc_debounced_contact_active"], [0, 0])
        np.testing.assert_array_equal(paused["wbc_hard_contact_active"], [0, 0])
        self.assertEqual(paused["metrics"]["wbc_support_active_count"], 0)
        self.assertEqual(int(paused["tick"]), tick_before + 1)
        self.assertEqual(float(self.worker.data.time), time_before)
        np.testing.assert_array_equal(self.worker.data.qpos, qpos_before)
        np.testing.assert_array_equal(self.worker.data.qvel, qvel_before)

        heartbeat = self.worker.step({"type": "step"})
        self.assertTrue(heartbeat["paused"])
        self.assertFalse(heartbeat["metrics"]["wbc_observed_contact_available"])
        np.testing.assert_array_equal(heartbeat["wbc_hard_contact_active"], [0, 0])
        self.assertEqual(heartbeat["metrics"]["wbc_support_active_count"], 0)

        resumed = self.worker.step(
            {"type": "step", "command_id": 12, "paused": False}
        )
        self.assertFalse(resumed["paused"])
        self.assertGreater(float(self.worker.data.time), time_before)
        self.assertNotEqual(resumed["metrics"]["wbc_status"], "paused")

    def test_reset_while_paused_rebuilds_pose_and_stays_paused(self) -> None:
        self.worker.step({"type": "step", "paused": True})
        epoch_before = self.worker.reset_epoch
        reset = self.worker.step(
            {"type": "step", "command_id": 13, "reset": True, "paused": True}
        )
        self.assertEqual(reset["reset_epoch"], epoch_before + 1)
        self.assertTrue(reset["paused"])
        self.assertEqual(float(reset["simulator"]["time_s"]), 0.0)
        self.assertEqual(reset["metrics"]["wbc_status"], "paused")
        self.assertFalse(reset["metrics"]["wbc_observed_contact_available"])
        np.testing.assert_array_equal(reset["wbc_debounced_contact_active"], [0, 0])
        np.testing.assert_array_equal(reset["wbc_hard_contact_active"], [0, 0])
        self.assertEqual(reset["metrics"]["wbc_support_active_count"], 0)
        self.assertFalse(reset["metrics"]["wbc_observed_contact_available"])
        np.testing.assert_array_equal(reset["wbc_observed_contact_active"], [0, 0])
        np.testing.assert_array_equal(reset["wbc_debounced_contact_active"], [0, 0])
        np.testing.assert_array_equal(reset["wbc_hard_contact_active"], [0, 0])

    def test_paused_worker_rejects_active_external_load(self) -> None:
        self.worker.step({"type": "step", "paused": True})
        body_id = self.worker.body_by_name["base"]
        point = self.worker.data.xipos[body_id].copy()
        result = self.worker.step(
            {
                "type": "step",
                "paused": True,
                "external_load": {
                    "active": True,
                    "body": "base",
                    "force_world": [1.0, 0.0, 0.0],
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 14,
                },
            }
        )
        self.assertEqual(result["type"], "plant_error")
        self.assertIn("disabled while MuJoCo is paused", result["message"])

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
