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
    COMMAND_DEFAULT_DURATION_MS,
    COMMAND_HOLD_CONSECUTIVE_TICKS,
    COMMAND_MAX_DISPLACEMENT_M,
    COMMAND_MIN_DURATION_MS,
    CONTROL_DT,
    PHYSICS_DT,
    PHYSICS_STEPS_PER_CONTROL,
    MAX_APPLICATION_OFFSET_M,
    LiveUpkiePlant,
    parse_args,
    quintic_profile,
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
        body_origin = self.worker.data.xipos[body_id].copy()
        center_of_mass = np.asarray(
            self.worker.data.subtree_com[0], dtype=np.float64
        ).copy()
        root_body_id = int(self.worker.model.body_rootid[body_id])
        root_origin = np.asarray(
            self.worker.data.xpos[root_body_id], dtype=np.float64
        ).copy()
        point = body_origin + np.asarray([0.0, 0.0, 0.2])
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
        expected_moment = np.cross(point - body_origin, force)
        np.testing.assert_allclose(
            result["external_load"]["moment_world_nm"], expected_moment, atol=1.0e-12
        )
        self.assertAlmostEqual(
            result["external_load"]["maximum_moment_nm"],
            float(np.linalg.norm(expected_moment)),
            places=12,
        )
        centroidal_moment = np.cross(
            point - center_of_mass,
            force,
        )
        np.testing.assert_allclose(
            result["external_load"]["centroidal_moment_world_nm"],
            centroidal_moment,
            atol=1.0e-12,
        )
        root_moment = np.cross(point - root_origin, force)
        np.testing.assert_allclose(
            result["external_load"]["root_moment_world_nm"],
            root_moment,
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            result["external_load"]["maximum_root_moment_nm"],
            float(np.linalg.norm(root_moment)),
            places=12,
        )
        self.assertLess(result["external_load"]["application_offset_m"], 0.25)
        self.assertEqual(
            result["external_load"]["provenance"], EVALUATION_PROVENANCE
        )
        self.assertFalse(result["measured_impact_impulse"]["available"])
        self.assertFalse(result["unobserved_model_reserve"]["available"])

    def test_external_wrench_is_fed_forward_same_tick_and_cleared_on_release(self) -> None:
        worker = LiveUpkiePlant(
            ROOT / "models/upkie/upkie.urdf",
            controller_options={
                "centroidal_angular_momentum_weight": 0.3,
                "centroidal_angular_momentum_frequency_hz": 1.0,
            },
        )
        body_id = worker.body_by_name["base"]
        force = np.asarray([2.0, 0.0, 0.0])
        point = worker.data.xipos[body_id] + np.asarray([0.0, 0.0, 0.2])
        root_body_id = int(worker.model.body_rootid[body_id])
        expected_root_moment = np.cross(
            point - worker.data.xpos[root_body_id], force
        )
        expected_centroidal_moment = np.cross(
            point - np.asarray(worker.data.subtree_com[0], dtype=np.float64),
            force,
        )
        np.testing.assert_array_equal(
            worker.controller.centroidal_angular_momentum_rate_world,
            np.zeros((1, 3)),
        )
        worker.step(
            {
                "type": "step",
                "command_id": 3,
                "external_load": {
                    "active": True,
                    "body": "base",
                    "force_world": force.tolist(),
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 3,
                },
            }
        )
        # The declared wrench is consumed by the same solve that precedes its
        # MuJoCo application.
        np.testing.assert_allclose(
            worker.controller.centroidal_angular_momentum_rate_world[0],
            -expected_centroidal_moment,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            worker.controller.external_wrench_world[0, :3],
            expected_root_moment,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            worker.controller.external_wrench_world[0, 3:], force, atol=1.0e-12
        )
        self.assertTrue(worker.last_external_wrench_valid)
        np.testing.assert_allclose(
            worker.last_external_wrench_world[:3],
            expected_root_moment,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            worker.last_external_centroidal_moment_world,
            expected_centroidal_moment,
            atol=1.0e-12,
        )
        worker.step(
            {
                "type": "step",
                "command_id": 4,
                "external_load": {"active": False, "request_id": 4},
            }
        )
        # Release clears feed-forward on the release solve; no previous wrench
        # survives for one extra control tick.
        np.testing.assert_array_equal(
            worker.controller.centroidal_angular_momentum_rate_world,
            np.zeros((1, 3)),
        )
        np.testing.assert_array_equal(
            worker.controller.external_wrench_world,
            np.zeros((1, 6)),
        )
        self.assertFalse(worker.last_external_wrench_valid)

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
        self.assertEqual(hello["actuator_names"], [
            "left_hip_motor",
            "left_knee_motor",
            "left_wheel_motor",
            "right_hip_motor",
            "right_knee_motor",
            "right_wheel_motor",
        ])
        np.testing.assert_allclose(
            hello["actuator_effort_limits_nm"], [16.0, 16.0, 1.7, 16.0, 16.0, 1.7]
        )
        self.assertEqual(hello["actuator_resource_models"], [False] * 6)
        self.assertEqual(
            hello["actuator_resource_contract"]["thermal_reliability"],
            "unmodeled; no calibrated electrical/thermal state",
        )
        self.assertEqual(
            hello["external_load_contract"]["wbc_external_moment_observation"],
            "external_load.root_moment_world_nm, re-expressed about the current root origin and consumed by the same 50 Hz solve",
        )
        self.assertIn(
            "current aggregate CoM",
            hello["external_load_contract"][
                "wbc_external_centroidal_moment_observation"
            ],
        )
        self.assertTrue(
            hello["external_load_contract"]["wbc_external_wrench_feedforward"][
                "enabled"
            ]
        )
        self.assertEqual(
            hello["external_load_contract"]["wbc_external_wrench_feedforward"][
                "scale"
            ],
            1.0,
        )
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
        self.assertEqual(len(result["actuator_effort_limit_nm"]), 6)
        self.assertEqual(len(result["actuator_effort_utilization"]), 6)
        self.assertEqual(len(result["actuator_velocity_rad_s"]), 6)
        self.assertEqual(len(result["actuator_mechanical_power_w"]), 6)
        self.assertTrue(
            np.all(np.isfinite(np.asarray(result["actuator_effort_utilization"])))
        )
        self.assertTrue(
            np.all(np.asarray(result["actuator_effort_utilization"]) >= 0.0)
        )
        self.assertTrue(
            np.all(
                np.asarray(result["actuator_effort_utilization"]) <= 1.0 + 1.0e-12
            )
        )
        self.assertTrue(
            np.isfinite(result["metrics"]["maximum_actuator_effort_utilization"])
        )
        self.assertTrue(
            np.isfinite(result["metrics"]["maximum_abs_actuator_mechanical_power_w"])
        )
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

    def test_cli_defaults_to_the_production_rate_profile(self) -> None:
        model = ROOT / "models/upkie/upkie.urdf"
        with patch.object(sys, "argv", ["worker", str(model)]):
            args = parse_args()
        self.assertEqual(args.stream_dt, 0.020)
        self.assertEqual(args.control_dt, 0.020)
        self.assertEqual(args.physics_dt, 0.004)

    def test_target_commit_uses_uniform_frame_quintic_and_holds_until_replaced(self) -> None:
        torso = self.worker.data.xpos[self.worker.body_by_name["torso"]].copy()
        down_target = torso + np.asarray([0.0, 0.0, -0.08])
        first = self.worker.step(
            {
                "type": "step",
                "command_id": 20,
                "target_command": {
                    "active": True,
                    "frame": "torso",
                    "target": down_target.tolist(),
                    "duration_ms": COMMAND_MIN_DURATION_MS,
                    "request_id": 20,
                },
            }
        )
        self.assertEqual(first["target_command"]["phase"], "executing")
        self.assertEqual(first["target_command"]["request_id"], 20)
        np.testing.assert_allclose(
            first["target_command"]["requested_position_world"], down_target
        )
        admitted = np.asarray(
            first["target_command"]["admitted_position_world"], dtype=np.float64
        )
        self.assertAlmostEqual(
            np.linalg.norm(admitted - torso), COMMAND_MAX_DISPLACEMENT_M
        )
        self.assertTrue(first["target_command"]["clamped"])
        self.assertGreaterEqual(first["target_command"]["progress"], 0.0)
        measured_after_step = np.asarray(
            first["target_command"]["measured_position_world"],
            dtype=np.float64,
        )
        np.testing.assert_allclose(
            measured_after_step,
            self.worker.data.xpos[self.worker.body_by_name["torso"]],
        )
        self.assertGreater(np.linalg.norm(measured_after_step - torso), 1.0e-6)

        # Endpoint hold qualification is measured and requires consecutive
        # executable ticks; it is not inferred from trajectory time alone.
        body_id = self.worker.body_by_name["torso"]
        self.worker.command_elapsed_s = self.worker.command_duration_s
        self.worker.data.xpos[body_id] = admitted
        executable = {
            "command_task_rms": 0.0,
            "command_task_clipped": False,
            "command_intent_executable": True,
            "command_intent_suppressed": False,
            "command_intent_suppression_reason": "",
        }
        for _ in range(COMMAND_HOLD_CONSECUTIVE_TICKS):
            self.worker._advance_target_command(1.0, executable)
        self.assertEqual(self.worker.command_phase, "holding")

        measured = self.worker.data.xpos[body_id].copy()
        up_target = (measured + np.asarray([0.0, 0.0, 0.02])).tolist()
        rising = self.worker.step(
            {
                "type": "step",
                "command_id": 21,
                "target_command": {
                    "active": True,
                    "frame": "torso",
                    "target": up_target,
                    "duration_ms": COMMAND_MIN_DURATION_MS,
                    "request_id": 21,
                },
            }
        )
        self.assertEqual(rising["target_command"]["phase"], "executing")
        self.assertEqual(rising["target_command"]["request_id"], 21)
        self.assertGreater(
            rising["target_command"]["admitted_position_world"][2], measured[2]
        )

    def test_push_freezes_target_bundle_then_rebases_without_new_identity(self) -> None:
        for _ in range(100):
            self.worker.step({"type": "step"})
        frame = "left_knee_qdd100_rotor"
        request_id = 91
        measured = self.worker.data.xpos[self.worker.body_by_name[frame]].copy()
        target = measured + np.asarray([0.015, 0.0, 0.0])
        target_command = {
            "active": True,
            "frame": frame,
            "handle_id": f"frame:{frame}",
            "target": target.tolist(),
            "duration_ms": COMMAND_DEFAULT_DURATION_MS,
            "request_id": request_id,
        }
        execution_states = [
            self.worker.step(
                {
                    "type": "step",
                    "target_command": target_command,
                }
            )
            for _ in range(40)
        ]
        self.assertTrue(
            all(
                state["target_command"]["command_descriptors_active"]
                for state in execution_states
            )
        )
        self.assertTrue(
            all(
                abs(
                    state["target_command"][
                        "cartesian_x_position_neutral_residual_at_solve_m"
                    ]
                )
                <= 1.0e-12
                and abs(
                    state["target_command"][
                        "cartesian_desired_ax_at_solve_m_s2"
                    ]
                )
                <= 1.0e-12
                and 0.0
                <= state["target_command"][
                    "cartesian_x_velocity_damping_beta_at_solve"
                ]
                <= 1.0
                and abs(
                    state["target_command"]["cartesian_desired_vx_at_solve_m_s"]
                    - state["target_command"][
                        "cartesian_x_velocity_damping_beta_at_solve"
                    ]
                    * state["target_command"][
                        "cartesian_measured_vx_at_solve_m_s"
                    ]
                )
                <= 1.0e-12
                for state in execution_states
            )
        )
        self.assertTrue(
            any(
                abs(
                    state["target_command"][
                        "station_requested_error_at_solve_m"
                    ]
                )
                > 1.0e-6
                for state in execution_states
            )
        )
        frozen_progress = execution_states[-1]["target_command"]["progress"]
        self.assertGreater(frozen_progress, 0.0)
        body_id = self.worker.body_by_name["base"]
        point = self.worker.data.xipos[body_id].copy()
        external_load = {
            "active": True,
            "body": "base",
            "force_world": [1.0, 0.0, 0.0],
            "application_point_world": point.tolist(),
            "provenance": EVALUATION_PROVENANCE,
            "request_id": 92,
        }
        first = self.worker.step(
            {
                "type": "step",
                "target_command": target_command,
                "external_load": external_load,
            }
        )
        self.assertEqual(first["target_command"]["request_id"], request_id)
        self.assertEqual(first["target_command"]["handle_id"], f"frame:{frame}")
        self.assertEqual(first["target_command"]["phase"], "suppressed")
        self.assertEqual(first["target_command"]["progress"], frozen_progress)
        self.assertFalse(self.worker.controller.command_active)
        self.assertFalse(first["target_command"]["command_descriptors_active"])
        self.assertTrue(first["target_command"]["execution_bundle_suppressed"])
        self.assertAlmostEqual(
            first["target_command"]["station_neutral_error_at_solve_m"],
            0.0,
            places=12,
        )
        np.testing.assert_allclose(
            self.worker.controller.external_wrench_world[0, 3:],
            [1.0, 0.0, 0.0],
            atol=1.0e-12,
        )

        push_states = [first]
        push_states.extend(
            self.worker.step(
                {
                    "type": "step",
                    "target_command": target_command,
                    "external_load": external_load,
                }
            )
            for _ in range(24)
        )
        second = push_states[-1]
        self.assertTrue(
            all(
                state["target_command"]["progress"] == frozen_progress
                for state in push_states
            )
        )
        self.assertTrue(
            all(
                state["target_command"]["execution_bundle_suppressed"]
                and not state["target_command"]["command_descriptors_active"]
                and abs(
                    state["target_command"]["station_neutral_error_at_solve_m"]
                )
                <= 1.0e-12
                for state in push_states
            )
        )
        self.assertTrue(
            all(
                state["external_load"]["wbc_feedforward_active"]
                and np.allclose(
                    state["external_load"]["wbc_observed_force_world_n"],
                    [1.0, 0.0, 0.0],
                    atol=1.0e-12,
                )
                for state in push_states
            )
        )
        self.assertEqual(second["target_command"]["progress"], frozen_progress)
        np.testing.assert_allclose(
            second["target_command"]["admitted_position_world"], target
        )
        released = self.worker.step(
            {
                "type": "step",
                "external_load": {"active": False, "request_id": 93},
            }
        )
        self.assertFalse(released["external_load"]["active"])
        self.assertEqual(released["target_command"]["request_id"], request_id)
        self.assertEqual(released["target_command"]["handle_id"], f"frame:{frame}")
        self.assertEqual(released["target_command"]["phase"], "suppressed")
        self.assertFalse(self.worker.controller.command_active)
        self.assertFalse(released["external_load"]["wbc_feedforward_active"])
        np.testing.assert_array_equal(
            released["external_load"]["wbc_observed_force_world_n"],
            [0.0, 0.0, 0.0],
        )
        np.testing.assert_array_equal(
            self.worker.controller.external_wrench_world,
            np.zeros((1, 6)),
        )
        recovery_states = [released]
        measured_before_resume = None
        for _ in range(200):
            measured_before_tick = self.worker.data.xpos[
                self.worker.body_by_name[frame]
            ].copy()
            resumed = self.worker.step({"type": "step"})
            recovery_states.append(resumed)
            if resumed["target_command"]["command_descriptors_active"]:
                measured_before_resume = measured_before_tick
                break
        self.assertIsNotNone(measured_before_resume)
        suppressed_recovery = [
            state
            for state in recovery_states[:-1]
            if state["target_command"]["execution_bundle_suppressed"]
        ]
        self.assertGreaterEqual(len(suppressed_recovery), 5)
        self.assertEqual(
            [
                state["target_command"]["recovery_safe_ticks"]
                for state in suppressed_recovery[-5:]
            ],
            [1, 2, 3, 4, 5],
        )
        self.assertTrue(
            all(
                not state["target_command"]["command_descriptors_active"]
                and state["target_command"]["progress"] == frozen_progress
                and abs(
                    state["target_command"]["station_neutral_error_at_solve_m"]
                )
                <= 1.0e-12
                for state in suppressed_recovery
            )
        )
        np.testing.assert_allclose(
            resumed["target_command"]["admitted_position_world"], target
        )
        np.testing.assert_allclose(
            self.worker.command_plan["start"], measured_before_resume, atol=1.0e-12
        )
        np.testing.assert_allclose(
            self.worker.command_plan["target"], target, atol=1.0e-12
        )
        np.testing.assert_allclose(
            self.worker.command_plan["target_wheel_position"],
            self.worker.command_plan["start_wheel_position"],
            atol=1.0e-12,
        )
        self.assertTrue(self.worker.command_plan["measured_neutral_realization"])
        self.assertTrue(self.worker.controller.command_active)
        self.assertTrue(resumed["target_command"]["command_descriptors_active"])
        self.assertFalse(resumed["target_command"]["execution_bundle_suppressed"])
        self.assertTrue(resumed["target_command"]["measured_neutral_realization"])
        np.testing.assert_allclose(
            resumed["target_command"]["sampled_position_world"],
            measured_before_resume,
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            resumed["target_command"]["progress"], frozen_progress, places=12
        )
        self.assertIn(
            resumed["target_command"]["phase"],
            ("executing", "authority_limited", "holding"),
        )
        resumed_states = [resumed]
        for _ in range(520):
            resumed = self.worker.step({"type": "step"})
            resumed_states.append(resumed)
            if resumed["target_command"]["phase"] == "holding":
                break
        self.assertEqual(resumed["target_command"]["phase"], "holding")
        hold_dwell = [resumed]
        for _ in range(49):
            hold_dwell.append(self.worker.step({"type": "step"}))
        resumed_states.extend(hold_dwell[1:])
        self.assertTrue(
            all(
                state["target_command"]["phase"] == "holding"
                and np.linalg.norm(
                    np.asarray(
                        state["target_command"]["measured_position_world"],
                        dtype=np.float64,
                    )
                    - target
                )
                <= 0.012
                for state in hold_dwell
            )
        )
        self.assertTrue(
            all(
                abs(
                    state["target_command"][
                        "station_admitted_error_at_solve_m"
                    ]
                )
                <= 0.03 + 1.0e-12
                for state in resumed_states
            )
        )
        self.assertTrue(
            all(
                abs(
                    state["target_command"][
                        "cartesian_x_position_neutral_residual_at_solve_m"
                    ]
                )
                <= 1.0e-12
                and abs(
                    state["target_command"][
                        "cartesian_desired_ax_at_solve_m_s2"
                    ]
                )
                <= 1.0e-12
                and 0.0
                <= state["target_command"][
                    "cartesian_x_velocity_damping_beta_at_solve"
                ]
                <= 1.0
                and abs(
                    state["target_command"]["cartesian_desired_vx_at_solve_m_s"]
                    - state["target_command"][
                        "cartesian_x_velocity_damping_beta_at_solve"
                    ]
                    * state["target_command"][
                        "cartesian_measured_vx_at_solve_m_s"
                    ]
                )
                <= 1.0e-12
                for state in resumed_states
            )
        )
        self.assertTrue(
            any(
                state["target_command"]["station_error_clamped_at_solve"]
                for state in resumed_states
            )
        )

    def test_all_published_body_handles_share_uniform_admission(self) -> None:
        frames = (
            "torso",
            "left_knee_qdd100_rotor",
            "left_ankle_mj5208_rotor",
            "right_knee_qdd100_rotor",
            "right_ankle_mj5208_rotor",
        )
        for request_id, frame in enumerate(frames, start=100):
            with self.subTest(frame=frame):
                measured = self.worker.data.xpos[
                    self.worker.body_by_name[frame]
                ].copy()
                target = measured + np.asarray([0.015, 0.0, 0.0])
                before_qpos = self.worker.data.qpos.copy()
                before_qvel = self.worker.data.qvel.copy()
                self.worker._accept_target_command(
                    {
                        "frame": frame,
                        "target": target.tolist(),
                        "duration_ms": COMMAND_MIN_DURATION_MS,
                        "request_id": request_id,
                    }
                )
                self.assertEqual(self.worker.command_frame, frame)
                self.assertEqual(
                    self.worker.command_handle_id, f"frame:{frame}"
                )
                np.testing.assert_allclose(
                    self.worker.command_start_position, measured
                )
                np.testing.assert_allclose(
                    self.worker.command_target_position, target
                )
                np.testing.assert_array_equal(self.worker.data.qpos, before_qpos)
                np.testing.assert_array_equal(self.worker.data.qvel, before_qvel)
                self.assertLessEqual(
                    self.worker.command_ik_target_residual_m, 2.0e-4
                )
                self.assertLessEqual(
                    self.worker.command_ik_support_residual_m, 2.0e-4
                )
                self.assertLessEqual(
                    self.worker.command_ik_balance_residual_m, 2.0e-4
                )

    def test_hello_advertises_only_normalized_frame_target_handles(self) -> None:
        contract = self.worker.hello()["target_command_contract"]
        self.assertEqual(contract["type"], "plant_frame_target_commit")
        self.assertEqual(
            contract["accepted_frames"],
            [
                "torso",
                "left_knee_qdd100_rotor",
                "left_ankle_mj5208_rotor",
                "right_knee_qdd100_rotor",
                "right_ankle_mj5208_rotor",
            ],
        )
        layers = contract["active_target_wbc_layers"]
        self.assertEqual(
            [layer["priority"] for layer in layers], [0, 1, 2, 3, 4]
        )
        self.assertEqual(
            contract["task_residual_semantics"],
            "unweighted RMS in each task's physical units",
        )
        self.assertEqual(
            layers[2]["weights"],
            [{"task": "Cartesian point", "weight": 0.1}],
        )
        self.assertEqual(
            [entry["weight"] for entry in layers[3]["weights"]],
            [10.0, 10.0, 1.0],
        )

    def test_rejected_replacement_preserves_target_and_external_push(self) -> None:
        torso = self.worker.data.xpos[self.worker.body_by_name["torso"]].copy()
        accepted_target = torso + np.asarray([0.0, 0.0, -0.02])
        self.worker._accept_target_command(
            {
                "frame": "torso",
                "target": accepted_target.tolist(),
                "duration_ms": COMMAND_MIN_DURATION_MS,
                "request_id": 29,
            }
        )
        preserved_target = self.worker.command_target_position.copy()
        body_id = self.worker.body_by_name["base"]
        point = self.worker.data.xipos[body_id].copy()
        result = self.worker.step(
            {
                "type": "step",
                "command_id": 30,
                "target_command": {
                    "active": True,
                    "frame": "not_a_mujoco_body",
                    "target": [0.0, 0.0, 0.4],
                    "duration_ms": COMMAND_MIN_DURATION_MS,
                    "request_id": 30,
                },
                "external_load": {
                    "active": True,
                    "body": "base",
                    "force_world": [1.0, 0.0, 0.0],
                    "application_point_world": point.tolist(),
                    "provenance": EVALUATION_PROVENANCE,
                    "request_id": 31,
                },
            }
        )
        self.assertEqual(result["target_command"]["request_id"], 29)
        self.assertEqual(result["target_command"]["frame"], "torso")
        np.testing.assert_allclose(
            result["target_command"]["admitted_position_world"],
            preserved_target,
        )
        self.assertEqual(
            result["target_command"]["last_admission"]["status"], "rejected"
        )
        self.assertEqual(
            result["target_command"]["last_rejection"]["request_id"], 30
        )
        self.assertIn(
            "advertised Upkie frame handle",
            result["target_command"]["last_rejection"]["reason"],
        )
        self.assertTrue(result["external_load"]["active"])

    def test_quintic_profile_is_zero_slope_at_both_endpoints(self) -> None:
        start = np.asarray([0.0, 0.0, 0.5])
        target = np.asarray([0.1, -0.02, 0.38])
        beginning = quintic_profile(start, target, 1.2, 0.0)
        end = quintic_profile(start, target, 1.2, 1.2)
        np.testing.assert_allclose(beginning[0], start)
        np.testing.assert_allclose(beginning[1], 0.0)
        np.testing.assert_allclose(beginning[2], 0.0)
        np.testing.assert_allclose(end[0], target)
        np.testing.assert_allclose(end[1], 0.0)
        np.testing.assert_allclose(end[2], 0.0)

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
