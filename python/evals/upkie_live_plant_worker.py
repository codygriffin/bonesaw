#!/usr/bin/env python3
"""Line-delimited worker for the live Upkie MuJoCo plant gateway.

The Rust HTTP/WebSocket server owns transport, validation, command expiry, and
client isolation. This process owns MuJoCo contact/integration and calls the
persistent Rust balance/WBC sessions through the narrow PyO3 boundary.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time
from typing import Any

import mujoco
import numpy as np

import upkie_mujoco_plant_report as plant


STREAM_DT = 0.020
CONTROL_DT = 0.020
PHYSICS_DT = 0.004
CONTROL_TICKS_PER_STREAM = int(round(STREAM_DT / CONTROL_DT))
PHYSICS_STEPS_PER_CONTROL = int(round(CONTROL_DT / PHYSICS_DT))
MAX_FORCE_N = 8.0
MAX_APPLICATION_OFFSET_M = 0.75
FALL_HEIGHT_M = 0.30
FALL_TILT_RAD = math.radians(75.0)
# A target commit is deliberately a small, bounded command surface for the
# live prototype. It commands one measured MuJoCo body-frame origin through a
# support-constrained IK realization plus a Cartesian WBC task; it never writes
# MuJoCo qpos/qvel and it never becomes a wrench.
COMMAND_DEFAULT_DURATION_MS = 3000
COMMAND_MIN_DURATION_MS = 1000
COMMAND_MAX_DURATION_MS = 5000
COMMAND_MAX_DISPLACEMENT_M = 0.05
COMMAND_HOLD_TOLERANCE_M = 0.012
COMMAND_HOLD_CONSECUTIVE_TICKS = 3
# A released PUSH may leave the freely rolling plant with appreciable common
# longitudinal velocity. Keep target execution out of the solve until the
# ordinary balance/station controller has recovered that motion; rebasing a
# Cartesian/station trajectory against a fast-moving measurement can inject a
# discontinuous reversal even though the public absolute goal is unchanged.
COMMAND_RECOVERY_SAFE_TICKS = 5
COMMAND_RECOVERY_MINIMUM_ROOT_HEIGHT_M = 0.45
COMMAND_RECOVERY_MAXIMUM_TILT_RAD = 0.05
COMMAND_RECOVERY_MAXIMUM_PITCH_RATE_RAD_S = 0.05
COMMAND_RECOVERY_MAXIMUM_FORWARD_SPEED_M_S = 0.025
COMMAND_RECOVERY_MAXIMUM_CAPTURE_PRESSURE = 0.1
COMMAND_RECOVERY_MINIMUM_STATION_AUTHORITY = 0.999
COMMAND_RECOVERY_MAXIMUM_STATION_ERROR_M = 0.03
COMMAND_STATION_ERROR_LIMIT_M = 0.03
COMMAND_X_DAMPING_FULL_ERROR_M = 0.004
COMMAND_X_DAMPING_NEUTRAL_ERROR_M = 0.020
COMMAND_FRAME_HANDLES = (
    "torso",
    "left_knee_qdd100_rotor",
    "left_ankle_mj5208_rotor",
    "right_knee_qdd100_rotor",
    "right_ankle_mj5208_rotor",
)
PRODUCTION_SUPPORT_LOAD_RESERVE_CONFIG = (
    0.47,  # activation release load fraction
    0.38,  # activation full load fraction
    0.005,  # activation release DCM [m]
    0.030,  # activation full DCM [m]
    0.040,  # load-filter time constant [s]
    0.012,  # prediction lookahead [s]
    1.50,  # unloading-rate release [/s]
    5.00,  # unloading-rate full [/s]
    0.020,  # support reserve [m]
    10.0,  # authority attack [/s]
    6.0,  # authority release [/s]
    8.0,  # maximum lateral acceleration [m/s2]
    4.0,  # lateral-acceleration slew [m/s3]
    math.radians(25.0),  # maximum bank angle
    3.0,  # bank-angle slew [rad/s]
    80.0,  # bank stiffness [/s2]
    14.0,  # bank damping [/s]
    80.0,  # maximum roll acceleration [rad/s2]
)


def finite_vector(value: object, length: int) -> np.ndarray | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    result = np.asarray(value, np.float64)
    return result if np.all(np.isfinite(result)) else None


def quintic_profile(
    start: np.ndarray,
    target: np.ndarray,
    duration_s: float,
    elapsed_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return a bounded C2 position/velocity/acceleration profile."""
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("quintic duration must be finite and positive")
    u = float(np.clip(elapsed_s / duration_s, 0.0, 1.0))
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration_s
    dds = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / duration_s**2
    delta = target - start
    return start + s * delta, ds * delta, dds * delta, u


class LiveUpkiePlant:
    def __init__(
        self,
        model_path: pathlib.Path,
        *,
        controller_options: dict[str, Any] | None = None,
        controller_balance_mode: str = "capture",
        stream_dt: float = STREAM_DT,
        control_dt: float = CONTROL_DT,
        physics_dt: float = PHYSICS_DT,
        balanced_nominal_joint_target: bool = False,
    ):
        for name, value in (
            ("stream_dt", stream_dt),
            ("control_dt", control_dt),
            ("physics_dt", physics_dt),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isinstance(balanced_nominal_joint_target, bool):
            raise ValueError("balanced_nominal_joint_target must be a boolean")
        self.control_ticks_per_stream = int(round(stream_dt / control_dt))
        self.physics_steps_per_control = int(round(control_dt / physics_dt))
        if (
            self.control_ticks_per_stream < 1
            or self.physics_steps_per_control < 1
            or abs(self.control_ticks_per_stream * control_dt - stream_dt) > 1.0e-12
            or abs(self.physics_steps_per_control * physics_dt - control_dt) > 1.0e-12
        ):
            raise ValueError(
                "stream/control/physics periods must form positive integer ratios"
            )
        self.stream_dt = float(stream_dt)
        self.control_dt = float(control_dt)
        self.physics_dt = float(physics_dt)
        self.balanced_nominal_joint_target = balanced_nominal_joint_target
        self.wbc_observation_source = (
            f"latest_completed_{int(round(1.0 / self.physics_dt))}hz_substep"
        )
        self.model_path = model_path
        # Evaluation-only profiles may opt into existing Rust controller
        # mechanisms. The public worker passes no overrides, so its controller
        # remains the frozen production default.
        self.controller_options = dict(controller_options or {})
        self.controller_balance_mode = str(controller_balance_mode)
        self.reset_epoch = 0
        self.tick = 0
        self.numeric_resets = 0
        self.fall_resets = 0
        self.pending_automatic_reset: str | None = None
        # Keep the worker alive while freezing MuJoCo time and WBC updates.
        self.paused = False
        self._build()

    def _build(self) -> None:
        import bonesaw

        self.model, self.data = plant.make_plant(
            self.model_path, physics_dt=self.physics_dt
        )
        balance = bonesaw.UpkieBalanceSession(str(self.model_path))
        root_position, _, _, q, _ = plant.read_state(self.model, self.data)
        balanced_root = np.empty(3, np.float64)
        balanced_q = np.empty(6, np.float64)
        error = balance.balanced_standing(
            root_position, q, balanced_root, balanced_q
        )
        if abs(error) > 1.0e-6:
            raise RuntimeError(
                f"balanced standing projection retained {error:.3e} m CoM error"
            )
        root_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
        )
        root_qpos = self.model.jnt_qposadr[root_joint]
        self.data.qpos[root_qpos : root_qpos + 3] = balanced_root
        for coordinate, name in enumerate(plant.JOINT_ORDER):
            joint = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.data.qpos[self.model.jnt_qposadr[joint]] = balanced_q[coordinate]
        mujoco.mj_forward(self.model, self.data)
        nominal_root, _, _, _, _ = plant.read_state(self.model, self.data)
        wheel_bodies = np.asarray(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in (
                    "left_ankle_mj5208_rotor",
                    "right_ankle_mj5208_rotor",
                )
            ],
            np.int64,
        )
        self.target_ground_position = float(
            np.mean(self.data.xpos[wheel_bodies, 0])
        )
        self.wheel_bodies = wheel_bodies
        # Contact identity is derived once from the MuJoCo body topology.  At
        # every 50 Hz WBC observation we then pass the measured wheel-to-ground
        # mask into the persistent Rust adapter.  The adapter owns debounce,
        # timestamp/provenance checks, and hard-row eligibility; this worker
        # must not turn an authored standing assumption into contact authority.
        self.wheel_contact_body_sets = plant.wheel_contact_body_sets(
            self.model, self.wheel_bodies
        )
        controller_options: dict[str, Any] = {
            "fall_safe_enabled": True,
            "fall_safe_primary_blend": False,
            "support_load_reserve_action_enabled": True,
            "support_load_reserve_config": PRODUCTION_SUPPORT_LOAD_RESERVE_CONFIG,
            # The declared wrench is known before the plant step. Feed that
            # same wrench into the WBC dynamics rows before applying it to
            # MuJoCo, rather than compensating one control tick late.
            "external_wrench_feedforward_enabled": True,
            "control_dt": self.control_dt,
        }
        # Keep the historical adapter standing target in the public profile;
        # the R304 fast-rate experiment opts into the Rust-balanced target
        # explicitly and is not a production-rate change.
        if self.balanced_nominal_joint_target:
            controller_options["nominal_joint_position"] = balanced_q
        controller_options.update(self.controller_options)
        # Contact priming is an evaluation-only receiver operation.  Remove it
        # before constructing the Rust adapter so a transport/profile option
        # cannot accidentally become an unknown solver keyword.  The public
        # worker leaves this at zero and therefore retains the cold-start
        # contract.
        prestart_samples = controller_options.pop(
            "contact_observation_prestart_samples", 0
        )
        if isinstance(prestart_samples, bool) or not isinstance(
            prestart_samples, (int, np.integer)
        ):
            raise ValueError(
                "contact_observation_prestart_samples must be an integer"
            )
        prestart_samples = int(prestart_samples)
        if not 0 <= prestart_samples <= 16:
            raise ValueError(
                "contact_observation_prestart_samples must be in 0..=16"
            )
        self.contact_observation_prestart_samples = prestart_samples
        self.controller = plant.RustWbcAdapter(
            self.model_path,
            nominal_root,
            self.target_ground_position,
            balance,
            self.controller_balance_mode,
            0.2,
            **controller_options,
        )
        if prestart_samples:
            prestart_contact = np.empty(2, np.uint8)
            plant.measured_wheel_ground_contacts_into(
                self.model,
                self.data,
                self.wheel_contact_body_sets,
                prestart_contact,
            )
            self.controller.prime_contact_observation(
                prestart_contact, samples=prestart_samples
            )
        self.actuator_ids = np.asarray(
            [
                mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"{name}_motor",
                )
                for name in plant.JOINT_ORDER
            ],
            np.int64,
        )
        # These are measured plant limits, not a second controller authority
        # path.  Keep the fixed actuator order and expose the authored MuJoCo
        # control range so the browser can show which coordinate is closest
        # to saturation without inventing calibrated thermal limits.
        self.actuator_names = [
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(actuator_id)
            )
            or f"actuator_{index}"
            for index, actuator_id in enumerate(self.actuator_ids)
        ]
        actuator_ranges = np.asarray(
            self.model.actuator_ctrlrange[self.actuator_ids], dtype=np.float64
        )
        actuator_limited = np.asarray(
            self.model.actuator_ctrllimited[self.actuator_ids], dtype=np.uint8
        )
        actuator_limits = np.max(np.abs(actuator_ranges), axis=1)
        actuator_limits[actuator_limited == 0] = np.inf
        self.actuator_effort_limits_nm = actuator_limits
        self.body_by_name = {
            name: body
            for body in range(1, self.model.nbody)
            if (name := mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body
            ))
            is not None
        }
        root_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
        )
        if root_joint < 0:
            raise RuntimeError("live Upkie model has no floating root joint")
        self.command_root_qpos = int(self.model.jnt_qposadr[root_joint])
        self.command_root_dof = int(self.model.jnt_dofadr[root_joint])
        self.command_joint_qpos = np.asarray(
            [
                self.model.jnt_qposadr[
                    mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                ]
                for name in plant.JOINT_ORDER
            ],
            dtype=np.int64,
        )
        self.command_joint_dof = np.asarray(
            [
                self.model.jnt_dofadr[
                    mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                ]
                for name in plant.JOINT_ORDER
            ],
            dtype=np.int64,
        )
        self.command_ik_dofs = np.concatenate(
            (
                np.arange(
                    self.command_root_dof,
                    self.command_root_dof + 3,
                    dtype=np.int64,
                ),
                self.command_joint_dof,
            )
        )
        self.command_wheel_center_bodies = np.asarray(
            [
                self.body_by_name[name]
                for name in ("left_wheel_center", "right_wheel_center")
            ],
            dtype=np.int64,
        )
        self.command_ik_data = mujoco.MjData(self.model)
        self.command_ik_jacobian = np.empty((9, 9), np.float64)
        self.command_ik_error = np.empty(9, np.float64)
        self.command_ik_body_jacobian = np.empty(
            (3, self.model.nv), np.float64
        )
        self.command_ik_body_angular_jacobian = np.empty(
            (3, self.model.nv), np.float64
        )
        self.command_wheel_velocity_jacobian = np.empty(
            (3, self.model.nv), np.float64
        )
        self.command_wheel_velocity_angular_jacobian = np.empty(
            (3, self.model.nv), np.float64
        )
        self.command_wheel_measured_position = np.empty(2, np.float64)
        self.command_wheel_measured_velocity = np.empty(2, np.float64)
        self.command_phase = "idle"
        self.command_request_id: int | None = None
        self.command_frame: str | None = None
        self.command_handle_id: str | None = None
        self.command_reason = "no target committed"
        self.command_requested_position: np.ndarray | None = None
        self.command_target_position: np.ndarray | None = None
        self.command_start_position: np.ndarray | None = None
        self.command_sampled_position: np.ndarray | None = None
        self.command_sampled_velocity = np.zeros(3, np.float64)
        self.command_sampled_acceleration = np.zeros(3, np.float64)
        self.command_sampled_progress = 0.0
        self.command_elapsed_s = 0.0
        self.command_duration_s = 0.0
        self.command_plan: dict[str, Any] | None = None
        self.command_clamped = False
        self.command_hold_ticks = 0
        self.command_last_admission_request_id: int | None = None
        self.command_last_admission_status = "none"
        self.command_last_admission_frame: str | None = None
        self.command_last_admission_handle_id: str | None = None
        self.command_last_admission_reason = "no target command received"
        self.command_last_rejection_request_id: int | None = None
        self.command_last_rejection_frame: str | None = None
        self.command_last_rejection_handle_id: str | None = None
        self.command_last_rejection_reason: str | None = None
        self.command_task_rms = 0.0
        self.command_task_clipped = False
        self.command_task_executable = False
        self.command_task_suppressed = False
        self.command_task_status = "inactive"
        self.command_execution_suspended_for_external_load = False
        self.command_execution_bundle_suppressed = False
        self.command_recovery_safe_ticks = 0
        self.command_station_target_at_solve_m = self.target_ground_position
        self.command_station_neutral_at_solve_m = self.target_ground_position
        self.command_station_requested_error_at_solve_m = 0.0
        self.command_station_admitted_error_at_solve_m = 0.0
        self.command_station_error_clamped_at_solve = False
        self.command_cartesian_requested_x_at_solve_m = 0.0
        self.command_cartesian_admitted_x_at_solve_m = 0.0
        self.command_cartesian_measured_x_at_solve_m = 0.0
        self.command_cartesian_admitted_vx_at_solve_m_s = 0.0
        self.command_cartesian_measured_vx_at_solve_m_s = 0.0
        self.command_cartesian_admitted_ax_at_solve_m_s2 = 0.0
        self.command_cartesian_x_velocity_damping_beta_at_solve = 1.0
        self.command_descriptors_active_at_solve = False
        self.command_measured_neutral_realization_at_solve = False
        self.command_ik_target_residual_m: float | None = None
        self.command_ik_support_residual_m: float | None = None
        self.command_ik_balance_residual_m: float | None = None
        self.ground_geom = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground"
        )
        ground_rotation = self.data.geom_xmat[self.ground_geom].reshape(3, 3)
        self.ground_plane_point_world = self.data.geom_xpos[
            self.ground_geom
        ].copy()
        self.ground_plane_normal_world = ground_rotation[:, 2].copy()
        self.zero_torque = np.zeros(3, np.float64)
        # Until MuJoCo has supplied a fresh observation, expose no contact.
        # The authored nominal stance must never be mistaken for measured
        # support authority, including on a paused/reset heartbeat.
        self.observed_contact_active = np.zeros(2, np.uint8)
        self.observed_contact_scratch = np.empty(2, np.uint8)
        # Fixed-capacity causal contact history.  The mask consumed by the
        # 50 Hz WBC is the latest completed 250 Hz observation from the prior
        # control window; each row below is written immediately after one
        # MuJoCo substep and never grows at runtime.
        self.physics_contact_window = np.zeros(
            (self.physics_steps_per_control, 2), np.uint8
        )
        self.physics_contact_loss_window = np.zeros(
            (self.physics_steps_per_control, 2), np.uint8
        )
        self.physics_contact_gain_window = np.zeros(
            (self.physics_steps_per_control, 2), np.uint8
        )
        self.physics_contact_loss_mask = np.zeros(2, np.uint8)
        self.physics_contact_gain_mask = np.zeros(2, np.uint8)
        self.physics_wheel_normal_force_window_n = np.zeros(
            (self.physics_steps_per_control, 2), np.float64
        )
        self.physics_contact_frame_index = 0
        self.wbc_observation_active = np.zeros(2, np.uint8)
        self.wbc_observation_wheel_normal_force_n = np.zeros(2, np.float64)
        self.observed_wheel_normal_force_n = np.zeros(2, np.float64)
        self.contact_wrench_scratch = np.zeros(6, np.float64)
        self.wbc_observation_frame_index = 0
        self.no_contact_active = np.zeros(2, np.uint8)
        self.no_wheel_normal_force_n = np.zeros(2, np.float64)
        # Plant-owned external-load observations. The declared current command
        # is presented to the WBC and then applied to MuJoCo in the same
        # control tick. Root-origin wrench drives the floating dynamics rows;
        # aggregate-CoM moment drives the independently switchable centroidal
        # objective. Keeping them separate prevents a reference-point mix-up.
        self.last_external_wrench_world = np.zeros(6, np.float64)
        self.last_external_centroidal_moment_world = np.zeros(3, np.float64)
        self.last_external_application_point_world = np.zeros(3, np.float64)
        self.last_external_force_world = np.zeros(3, np.float64)
        self.last_external_root_body_id = -1
        self.last_external_wrench_valid = False
        self.last_result: dict[str, Any] | None = None

    def reset(self, *, numeric: bool = False, fall: bool = False) -> None:
        self.reset_epoch += 1
        if numeric:
            self.numeric_resets += 1
        if fall:
            self.fall_resets += 1
        was_paused = self.paused
        self._build()
        # Resetting a paused simulation resets pose but does not implicitly
        # resume it. Running or automatic resets remain running.
        self.paused = was_paused

    def hello(self) -> dict[str, Any]:
        return {
            "type": "plant_hello",
            "protocol": 2,
            "model": "upkie",
            "stream_hz": int(round(1.0 / self.stream_dt)),
            "control_hz": int(round(1.0 / self.control_dt)),
            "physics_hz": int(round(1.0 / self.physics_dt)),
            "physics_substeps_per_control": self.physics_steps_per_control,
            "contact_observation": {
                "sample_hz": int(round(1.0 / self.physics_dt)),
                "consumed_hz": int(round(1.0 / self.control_dt)),
                "window_size": self.physics_steps_per_control,
                "wbc_source": self.wbc_observation_source,
                "prestart_samples": int(self.contact_observation_prestart_samples),
            },
            "controller_profile": (
                "production_default"
                if not self.controller_options
                and self.controller_balance_mode == "capture"
                else "evaluation_override"
            ),
            "nominal_joint_target": (
                "rust_balanced_initial_pose"
                if self.balanced_nominal_joint_target
                else "legacy_adapter_standing_pose"
            ),
            "paused": self.paused,
            "maximum_force_n": MAX_FORCE_N,
            "maximum_application_offset_m": MAX_APPLICATION_OFFSET_M,
            "command_ttl_ms": 140,
            "target_command_contract": {
                "type": "plant_frame_target_commit",
                "accepted_frames": list(COMMAND_FRAME_HANDLES),
                "target_frame": "world metres",
                "trajectory": "bounded quintic body-frame-origin position/velocity/acceleration",
                "default_duration_ms": COMMAND_DEFAULT_DURATION_MS,
                "duration_ms": [COMMAND_MIN_DURATION_MS, COMMAND_MAX_DURATION_MS],
                "maximum_displacement_m": COMMAND_MAX_DISPLACEMENT_M,
                "hold_tolerance_m": COMMAND_HOLD_TOLERANCE_M,
                "hold_consecutive_ticks": COMMAND_HOLD_CONSECUTIVE_TICKS,
                "endpoint_policy": "hold measured endpoint until next target or reset",
                "authority": "Rust WBC task under measured contact, collision, joint, actuator, and balance authorities",
            },
            "automatic_reset": {
                "fall_height_m": FALL_HEIGHT_M,
                "fall_tilt_rad": FALL_TILT_RAD,
                "policy": "report_fall_then_reset_on_next_stream_step",
            },
            "body_names": sorted(self.body_by_name),
            "actuator_names": list(self.actuator_names),
            "actuator_effort_limits_nm": [
                float(limit) if np.isfinite(limit) else None
                for limit in self.actuator_effort_limits_nm
            ],
            "actuator_resource_models": [False] * len(self.actuator_names),
            "actuator_resource_contract": {
                "effort_source": "MuJoCo actuator ctrl/force range",
                "mechanical_power_source": "measured actuator effort × actuator velocity",
                "thermal_reliability": "unmodeled; no calibrated electrical/thermal state",
            },
            "boundary": "python_mujoco_plant__rust_capture_wbc",
            "external_load_contract": {
                "executable_class": "declared_continuous_wrench",
                "accepted_sources": [
                    "interactive_operator",
                    "evaluation_harness",
                ],
                "force_frame": "world",
                "application_point_frame": "world",
                "measured_impact_impulse": "not_exposed_by_live_gateway",
                "unobserved_model_reserve": "not_estimated_by_live_gateway",
                "wbc_external_moment_observation": (
                    "external_load.root_moment_world_nm, re-expressed about the current root origin and consumed by the same 50 Hz solve"
                ),
                "wbc_external_centroidal_moment_observation": (
                    "external_load.centroidal_moment_world_nm, re-expressed about the current aggregate CoM and consumed by the same 50 Hz solve"
                ),
                "wbc_external_wrench_feedforward": {
                    "enabled": bool(self.controller.external_wrench_feedforward_enabled),
                    "scale": float(self.controller.external_wrench_feedforward_scale),
                    "axis_scales": (
                        self.controller.external_wrench_feedforward_axis_scales.tolist()
                        if self.controller.external_wrench_feedforward_axis_scales
                        is not None
                        else None
                    ),
                    "axis_order": "root_moment_xyz_then_root_force_xyz",
                    "moment_reference": "current root-body origin",
                },
            },
            "simulator": {
                "backend": "MuJoCo",
                "version": mujoco.__version__,
                "integrator": "implicitfast",
                "ground_plane_point_world": self.ground_plane_point_world.tolist(),
                "ground_plane_normal_world": self.ground_plane_normal_world.tolist(),
                "ground_plane_z_m": float(self.ground_plane_point_world[2]),
                "contact_model": "soft elliptic cone",
            },
        }

    def _validate_state(self) -> bool:
        return bool(
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.abs(self.data.qpos) < 1.0e6)
            and np.all(np.abs(self.data.qvel) < 1.0e6)
        )

    def _frames(self) -> list[dict[str, Any]]:
        frames = []
        for name, body in self.body_by_name.items():
            frames.append(
                {
                    "name": name,
                    "translation": self.data.xpos[body].tolist(),
                    "rotation": self.data.xquat[body].tolist(),
                }
            )
        return frames

    def _contacts(self) -> list[dict[str, Any]]:
        contacts: list[dict[str, Any]] = []
        wrench = np.zeros(6, np.float64)
        for index, contact in enumerate(self.data.contact[: self.data.ncon]):
            geom_a = int(contact.geom[0])
            geom_b = int(contact.geom[1])
            body_a = int(self.model.geom_bodyid[geom_a])
            body_b = int(self.model.geom_bodyid[geom_b])
            mujoco.mj_contactForce(self.model, self.data, index, wrench)
            contacts.append(
                {
                    "position_world": contact.pos.tolist(),
                    "normal_world": contact.frame[:3].tolist(),
                    "distance_m": float(contact.dist),
                    "normal_force_n": max(float(wrench[0]), 0.0),
                    "ground": geom_a == self.ground_geom
                    or geom_b == self.ground_geom,
                    "body_a": mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_BODY, body_a
                    )
                    or "world",
                    "body_b": mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_BODY, body_b
                    )
                    or "world",
                }
            )
        return contacts

    def _solve_target_endpoint_ik(
        self,
        frame: str,
        target: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, float] | None:
        """Solve one support-constrained endpoint from the measured state.

        The realization coordinates are identical for every command frame:
        floating-root translation and all six actuated joints. Root rotation
        is copied from the measured plant and is never an IK variable. Both
        wheel centers retain their measured lateral/vertical support anchors;
        longitudinal motion remains free because that is the wheel rolling
        coordinate.
        """
        data = self.command_ik_data
        data.qpos[:] = self.data.qpos
        data.qvel[:] = self.data.qvel
        mujoco.mj_forward(self.model, data)
        frame_body = self.body_by_name[frame]
        support_targets = np.asarray(
            self.data.xpos[self.command_wheel_center_bodies],
            dtype=np.float64,
        ).copy()
        balance_target = np.asarray(
            self.data.subtree_com[0, :2], dtype=np.float64
        ).copy()
        # The frame target and both rolling-aware support anchors are physical
        # endpoint constraints. The measured subtree-CoM XY target closes the
        # remaining null space so the realized posture retains balance.
        regularization = 1.0e-4
        regularization_weights = np.asarray(
            [100.0, 100.0, 1.0, 1.0, 1.0, 100.0, 1.0, 1.0, 100.0],
            dtype=np.float64,
        )
        for _ in range(80):
            mujoco.mj_forward(self.model, data)
            self.command_ik_error[:3] = target - np.asarray(
                data.xpos[frame_body], dtype=np.float64
            )
            mujoco.mj_jacBody(
                self.model,
                data,
                self.command_ik_body_jacobian,
                self.command_ik_body_angular_jacobian,
                frame_body,
            )
            self.command_ik_jacobian[:3] = self.command_ik_body_jacobian[
                :, self.command_ik_dofs
            ]
            for support_index, body_id in enumerate(
                self.command_wheel_center_bodies
            ):
                row = slice(3 + 2 * support_index, 5 + 2 * support_index)
                self.command_ik_error[row] = (
                    support_targets[support_index, 1:]
                    - np.asarray(data.xpos[body_id], dtype=np.float64)[1:]
                )
                mujoco.mj_jacBody(
                    self.model,
                    data,
                    self.command_ik_body_jacobian,
                    self.command_ik_body_angular_jacobian,
                    int(body_id),
                )
                self.command_ik_jacobian[row] = self.command_ik_body_jacobian[
                    1:, self.command_ik_dofs
                ]
            self.command_ik_error[7:9] = balance_target - np.asarray(
                data.subtree_com[0, :2], dtype=np.float64
            )
            mujoco.mj_jacSubtreeCom(
                self.model,
                data,
                self.command_ik_body_jacobian,
                0,
            )
            self.command_ik_jacobian[7:9] = self.command_ik_body_jacobian[
                :2, self.command_ik_dofs
            ]
            target_error = float(np.linalg.norm(self.command_ik_error[:3]))
            support_error = max(
                float(np.linalg.norm(self.command_ik_error[3:5])),
                float(np.linalg.norm(self.command_ik_error[5:7])),
            )
            balance_error = float(np.linalg.norm(self.command_ik_error[7:9]))
            if (
                target_error <= 1.0e-6
                and support_error <= 1.0e-6
                and balance_error <= 1.0e-6
            ):
                break
            augmented_jacobian = np.vstack(
                (
                    self.command_ik_jacobian,
                    regularization * np.diag(regularization_weights),
                )
            )
            augmented_error = np.concatenate(
                (self.command_ik_error, np.zeros(9, np.float64))
            )
            delta = np.linalg.lstsq(
                augmented_jacobian, augmented_error, rcond=1.0e-10
            )[0]
            delta_norm = float(np.linalg.norm(delta))
            if not math.isfinite(delta_norm):
                return None
            if delta_norm > 0.10:
                delta *= 0.10 / delta_norm
            data.qpos[
                self.command_root_qpos : self.command_root_qpos + 3
            ] += delta[:3]
            for coordinate, qpos_address in enumerate(
                self.command_joint_qpos
            ):
                joint_id = int(
                    self.model.dof_jntid[self.command_joint_dof[coordinate]]
                )
                value = float(data.qpos[qpos_address] + delta[3 + coordinate])
                if self.model.jnt_limited[joint_id]:
                    lower, upper = self.model.jnt_range[joint_id]
                    value = float(np.clip(value, lower + 1.0e-4, upper - 1.0e-4))
                data.qpos[qpos_address] = value
        mujoco.mj_forward(self.model, data)
        target_residual = float(
            np.linalg.norm(target - np.asarray(data.xpos[frame_body]))
        )
        support_residual = max(
            float(
                np.linalg.norm(
                    support_targets[index, 1:]
                    - np.asarray(data.xpos[body_id], dtype=np.float64)[1:]
                )
            )
            for index, body_id in enumerate(self.command_wheel_center_bodies)
        )
        balance_residual = float(
            np.linalg.norm(
                balance_target
                - np.asarray(data.subtree_com[0, :2], dtype=np.float64)
            )
        )
        if (
            not math.isfinite(target_residual)
            or not math.isfinite(support_residual)
            or not math.isfinite(balance_residual)
            or target_residual > 2.0e-4
            or support_residual > 2.0e-4
            or balance_residual > 2.0e-4
        ):
            self.command_ik_target_residual_m = target_residual
            self.command_ik_support_residual_m = support_residual
            self.command_ik_balance_residual_m = balance_residual
            return None
        root_target = np.asarray(
            data.qpos[
                self.command_root_qpos : self.command_root_qpos + 3
            ],
            dtype=np.float64,
        ).copy()
        joint_target = np.asarray(
            [data.qpos[address] for address in self.command_joint_qpos],
            dtype=np.float64,
        )
        wheel_position_target = np.asarray(
            data.xpos[self.command_wheel_center_bodies, 0], dtype=np.float64
        ).copy()
        return (
            root_target,
            joint_target,
            wheel_position_target,
            target_residual,
            support_residual,
            balance_residual,
        )

    def _reject_target_command(self, command: object, reason: str) -> None:
        request_id = command.get("request_id") if isinstance(command, dict) else None
        rejected_request_id = (
            int(request_id)
            if isinstance(request_id, int) and not isinstance(request_id, bool)
            else None
        )
        rejected_frame = (
            str(command.get("frame")) if isinstance(command, dict) else None
        )
        rejected_handle_id = (
            str(command.get("handle_id"))
            if isinstance(command, dict) and command.get("handle_id") is not None
            else (
                f"frame:{rejected_frame}"
                if rejected_frame in COMMAND_FRAME_HANDLES
                else None
            )
        )
        self.command_last_admission_request_id = rejected_request_id
        self.command_last_admission_status = "rejected"
        self.command_last_admission_frame = rejected_frame
        self.command_last_admission_handle_id = rejected_handle_id
        self.command_last_admission_reason = reason
        self.command_last_rejection_request_id = rejected_request_id
        self.command_last_rejection_frame = rejected_frame
        self.command_last_rejection_handle_id = rejected_handle_id
        self.command_last_rejection_reason = reason

    def _accept_target_command(self, command: object) -> None:
        if not isinstance(command, dict):
            self._reject_target_command(command, "target command must be an object")
            return
        request_id = command.get("request_id")
        if (
            isinstance(request_id, bool)
            or not isinstance(request_id, int)
            or request_id <= 0
        ):
            self._reject_target_command(
                command,
                "target command request_id must be a positive integer",
            )
            return
        frame = command.get("frame")
        if not isinstance(frame, str) or frame not in COMMAND_FRAME_HANDLES:
            self._reject_target_command(
                command,
                "target frame must name an advertised Upkie frame handle",
            )
            return
        handle_id = command.get("handle_id", f"frame:{frame}")
        if handle_id != f"frame:{frame}":
            self._reject_target_command(
                command,
                "target handle_id must be the normalized frame:<frame> identifier",
            )
            return
        measured = np.asarray(
            self.data.xpos[self.body_by_name[frame]], dtype=np.float64
        ).copy()
        measured_quaternion = np.asarray(
            self.data.xquat[self.body_by_name[frame]], dtype=np.float64
        )
        if not (
            np.all(np.isfinite(measured))
            and np.all(np.isfinite(measured_quaternion))
        ):
            self._reject_target_command(
                command, "target frame has no finite measured world origin"
            )
            return
        target = finite_vector(command.get("target"), 3)
        if target is None:
            self._reject_target_command(
                command, "target must contain three finite world-frame coordinates"
            )
            return
        duration_ms = command.get("duration_ms", COMMAND_DEFAULT_DURATION_MS)
        if (
            isinstance(duration_ms, bool)
            or not isinstance(duration_ms, (int, float))
            or not math.isfinite(float(duration_ms))
            or int(duration_ms) != duration_ms
            or not COMMAND_MIN_DURATION_MS
            <= int(duration_ms)
            <= COMMAND_MAX_DURATION_MS
        ):
            self._reject_target_command(
                command,
                f"target duration must be an integer in {COMMAND_MIN_DURATION_MS}..{COMMAND_MAX_DURATION_MS} ms",
            )
            return
        displacement = target - measured
        displacement_norm = float(np.linalg.norm(displacement))
        clamped = displacement_norm > COMMAND_MAX_DISPLACEMENT_M
        admitted_target = target.copy()
        if clamped:
            admitted_target = measured + (
                COMMAND_MAX_DISPLACEMENT_M / displacement_norm
            ) * displacement
        root_position, _, _, q, _ = plant.read_state(self.model, self.data)
        previous_ik_residuals = (
            self.command_ik_target_residual_m,
            self.command_ik_support_residual_m,
            self.command_ik_balance_residual_m,
        )
        self.command_ik_target_residual_m = None
        self.command_ik_support_residual_m = None
        self.command_ik_balance_residual_m = None
        # Upkie's common longitudinal degree of freedom is realized by the
        # capture controller's station target.  Solve the internal posture
        # only for the residual transverse/vertical frame displacement so a
        # world-X command does not also author an incompatible differential
        # wheel/posture motion.
        realization_target = admitted_target.copy()
        realization_target[0] = measured[0]
        ik_endpoint = self._solve_target_endpoint_ik(frame, realization_target)
        if ik_endpoint is None:
            target_residual = self.command_ik_target_residual_m
            support_residual = self.command_ik_support_residual_m
            balance_residual = self.command_ik_balance_residual_m
            residual_detail = (
                ""
                if (
                    target_residual is None
                    or support_residual is None
                    or balance_residual is None
                )
                else (
                    f" (frame residual {target_residual:.6g} m, "
                    f"support residual {support_residual:.6g} m, "
                    f"balance residual {balance_residual:.6g} m)"
                )
            )
            self._reject_target_command(
                command,
                "target rejected because measured support-constrained IK "
                f"was infeasible{residual_detail}",
            )
            (
                self.command_ik_target_residual_m,
                self.command_ik_support_residual_m,
                self.command_ik_balance_residual_m,
            ) = previous_ik_residuals
            return
        (
            target_root,
            target_joint,
            target_wheel_position,
            target_residual,
            support_residual,
            balance_residual,
        ) = ik_endpoint
        self.command_request_id = request_id
        self.command_frame = frame
        self.command_handle_id = handle_id
        self.command_reason = (
            "target clamped to uniform frame displacement envelope"
            if clamped
            else "target admitted"
        )
        self.command_requested_position = target.copy()
        self.command_target_position = admitted_target.copy()
        self.command_start_position = measured.copy()
        self.command_sampled_position = measured.copy()
        self.command_sampled_velocity.fill(0.0)
        self.command_sampled_acceleration.fill(0.0)
        self.command_sampled_progress = 0.0
        self.command_elapsed_s = 0.0
        self.command_duration_s = float(duration_ms) / 1000.0
        self.command_plan = {
            "start": measured.copy(),
            "target": admitted_target.copy(),
            "start_root": root_position.copy(),
            "target_root": target_root,
            "start_joint": q.copy(),
            "target_joint": target_joint,
            "start_wheel_position": np.asarray(
                self.data.xpos[self.command_wheel_center_bodies, 0],
                dtype=np.float64,
            ).copy(),
            # Longitudinal frame motion is authored through the balance
            # station reference.  Keep the per-wheel realization neutral so
            # a point command cannot manufacture a differential-yaw request.
            "target_wheel_position": np.asarray(
                self.data.xpos[self.command_wheel_center_bodies, 0],
                dtype=np.float64,
            ).copy(),
            "duration_s": self.command_duration_s,
            "progress_start": 0.0,
        }
        self.command_ik_target_residual_m = target_residual
        self.command_ik_support_residual_m = support_residual
        self.command_ik_balance_residual_m = balance_residual
        self.command_clamped = clamped
        self.command_hold_ticks = 0
        self.command_phase = "executing"
        self.command_last_admission_request_id = request_id
        self.command_last_admission_status = "accepted"
        self.command_last_admission_frame = frame
        self.command_last_admission_handle_id = handle_id
        self.command_last_admission_reason = self.command_reason

    def _rebase_target_command_from_measured_state(self) -> None:
        """Resume Cartesian intent with a measured-neutral realization bundle."""
        if (
            self.command_plan is None
            or self.command_frame is None
            or self.command_target_position is None
        ):
            return
        measured = np.asarray(
            self.data.xpos[self.body_by_name[self.command_frame]],
            dtype=np.float64,
        ).copy()
        root_position, _, _, q, _ = plant.read_state(self.model, self.data)
        progress_start = float(np.clip(self.command_sampled_progress, 0.0, 1.0))
        remaining_s = max(
            self.control_dt,
            self.command_duration_s * (1.0 - progress_start),
        )
        measured_wheel_position = np.asarray(
            self.data.xpos[self.command_wheel_center_bodies, 0],
            dtype=np.float64,
        ).copy()
        self.command_plan = {
            "start": measured.copy(),
            "target": self.command_target_position.copy(),
            "start_root": root_position.copy(),
            "target_root": root_position.copy(),
            "start_joint": q.copy(),
            "target_joint": q.copy(),
            "start_wheel_position": measured_wheel_position.copy(),
            # Preserve the uniform capture/station mapping: PUSH release does
            # not create a differential wheel command.
            "target_wheel_position": measured_wheel_position.copy(),
            "duration_s": remaining_s,
            "progress_start": progress_start,
            # Root, joint, and wheel realization references follow the
            # measured state with measured velocity and zero feed-forward
            # acceleration. Only the priority-2 Cartesian endpoint and the
            # uniform station-X mapping resume after PUSH recovery.
            "measured_neutral_realization": True,
        }
        # Resume the remaining local trajectory from the measured state. The
        # global progress offset stays frozen across PUSH/recovery, while the
        # measured-neutral realization bundle avoids restoring stale endpoint
        # IK posture references.
        self.command_elapsed_s = 0.0
        self.command_sampled_position = measured.copy()
        self.command_sampled_velocity.fill(0.0)
        self.command_sampled_acceleration.fill(0.0)
        self.command_hold_ticks = 0
        self.command_phase = "executing"
        self.command_reason = "target resumed from measured state after external load"

    def _target_recovery_tick_is_safe(self, result: dict[str, Any]) -> bool:
        """Evaluate one completed balance-only recovery tick."""
        root_position, root_quaternion, root_twist, _, _ = plant.read_state(
            self.model, self.data
        )
        tilt = float(
            np.linalg.norm(plant.quaternion_rotation_vector(root_quaternion)[:2])
        )
        physical_bilateral = bool(np.all(self.observed_contact_active == 1))
        executable_bilateral = bool(
            int(result.get("status", -1)) in (0, 1)
            and np.all(self.controller.contact_active[0] == 1)
        )
        return bool(
            physical_bilateral
            and executable_bilateral
            and float(root_position[2]) > COMMAND_RECOVERY_MINIMUM_ROOT_HEIGHT_M
            and tilt < COMMAND_RECOVERY_MAXIMUM_TILT_RAD
            and abs(float(root_twist[1]))
            < COMMAND_RECOVERY_MAXIMUM_PITCH_RATE_RAD_S
            and abs(float(root_twist[3]))
            < COMMAND_RECOVERY_MAXIMUM_FORWARD_SPEED_M_S
            and float(result.get("capture_pressure", math.inf))
            <= COMMAND_RECOVERY_MAXIMUM_CAPTURE_PRESSURE
            and float(result.get("station_authority", -math.inf))
            >= COMMAND_RECOVERY_MINIMUM_STATION_AUTHORITY
            and abs(float(result.get("station_error", math.inf)))
            <= COMMAND_RECOVERY_MAXIMUM_STATION_ERROR_M
        )

    def _sample_target_command(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, float]:
        if self.command_plan is None or self.command_target_position is None:
            return None, None, None, 0.0
        plan = self.command_plan
        if self.command_elapsed_s < float(plan["duration_s"]):
            desired, velocity, acceleration, local_progress = quintic_profile(
                plan["start"],
                plan["target"],
                float(plan["duration_s"]),
                self.command_elapsed_s,
            )
            progress_start = float(plan.get("progress_start", 0.0))
            progress = progress_start + (1.0 - progress_start) * local_progress
            return desired, velocity, acceleration, progress
        return (
            np.asarray(plan["target"], dtype=np.float64).copy(),
            np.zeros(3, np.float64),
            np.zeros(3, np.float64),
            1.0,
        )

    def _sample_realization_command(
        self,
    ) -> tuple[
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
        np.ndarray | None,
    ]:
        """Sample root/joint jets for the admitted uniform IK endpoint."""
        if self.command_plan is None:
            return (None, None, None, None, None, None, None, None, None)
        plan = self.command_plan
        duration_s = float(plan["duration_s"])
        if self.command_elapsed_s < duration_s:
            root, root_velocity, root_acceleration, _ = quintic_profile(
                np.asarray(plan["start_root"], dtype=np.float64),
                np.asarray(plan["target_root"], dtype=np.float64),
                duration_s,
                self.command_elapsed_s,
            )
            joint, joint_velocity, joint_acceleration, _ = quintic_profile(
                np.asarray(plan["start_joint"], dtype=np.float64),
                np.asarray(plan["target_joint"], dtype=np.float64),
                duration_s,
                self.command_elapsed_s,
            )
            wheel, wheel_velocity, wheel_acceleration, _ = quintic_profile(
                np.asarray(plan["start_wheel_position"], dtype=np.float64),
                np.asarray(plan["target_wheel_position"], dtype=np.float64),
                duration_s,
                self.command_elapsed_s,
            )
            return (
                root,
                root_velocity,
                root_acceleration,
                joint,
                joint_velocity,
                joint_acceleration,
                wheel,
                wheel_velocity,
                wheel_acceleration,
            )
        return (
            np.asarray(plan["target_root"], dtype=np.float64).copy(),
            np.zeros(3, np.float64),
            np.zeros(3, np.float64),
            np.asarray(plan["target_joint"], dtype=np.float64).copy(),
            np.zeros(6, np.float64),
            np.zeros(6, np.float64),
            np.asarray(plan["target_wheel_position"], dtype=np.float64).copy(),
            np.zeros(2, np.float64),
            np.zeros(2, np.float64),
        )

    def _advance_target_command(
        self, sampled_progress: float, result: dict[str, Any]
    ) -> None:
        if self.command_plan is None or self.command_frame is None:
            return
        self.command_elapsed_s = min(
            self.command_elapsed_s + self.control_dt,
            float(self.command_plan["duration_s"]),
        )
        self.command_task_rms = float(result.get("command_task_rms", 0.0))
        self.command_task_clipped = bool(result.get("command_task_clipped", False))
        self.command_task_executable = bool(
            result.get("command_intent_executable", False)
        )
        self.command_task_suppressed = bool(
            result.get("command_intent_suppressed", False)
        )
        self.command_task_status = str(
            result.get("command_intent_suppression_reason", "")
            or ("executable" if self.command_task_executable else "not executable")
        )
        if self.command_task_suppressed:
            self.command_phase = "suppressed"
            self.command_reason = (
                f"command intent suppressed: {self.command_task_status}"
            )
            return
        if not self.command_task_executable or self.command_task_clipped:
            self.command_phase = "authority_limited"
            self.command_reason = (
                f"command intent is not executable: {self.command_task_status}"
                if not self.command_task_executable
                else "command intent is clipped by higher controller authority"
            )
            return
        if sampled_progress < 1.0 - 1.0e-12:
            self.command_phase = "executing"
            return
        measured = np.asarray(
            self.data.xpos[self.body_by_name[self.command_frame]], dtype=np.float64
        )
        error = float(np.linalg.norm(self.command_target_position - measured))
        if (
            error <= COMMAND_HOLD_TOLERANCE_M
            and self.command_task_executable
            and not self.command_task_suppressed
        ):
            self.command_hold_ticks += 1
        else:
            self.command_hold_ticks = 0
        if self.command_hold_ticks >= COMMAND_HOLD_CONSECUTIVE_TICKS:
            self.command_phase = "holding"
            self.command_reason = "measured frame reached endpoint under executable command intent"
        else:
            self.command_phase = "authority_limited"
            self.command_reason = (
                f"measured frame error {error:.6f} m exceeds "
                f"{COMMAND_HOLD_TOLERANCE_M:.3f} m hold tolerance"
            )

    def _target_telemetry(
        self, latest_result: dict[str, Any] | None
    ) -> dict[str, Any]:
        measured = (
            np.asarray(
                self.data.xpos[self.body_by_name[self.command_frame]],
                dtype=np.float64,
            ).copy()
            if self.command_frame in self.body_by_name
            else None
        )
        error_vector = (
            self.command_target_position - measured
            if self.command_target_position is not None and measured is not None
            else np.zeros(3, np.float64)
        )
        wbc_status_code = (
            -1 if latest_result is None else int(latest_result.get("status", -1))
        )
        return {
            "phase": self.command_phase,
            "request_id": self.command_request_id,
            "frame": self.command_frame,
            "handle": self.command_frame,
            "handle_id": self.command_handle_id,
            "reason": self.command_reason,
            "progress": self.command_sampled_progress,
            "duration_s": self.command_duration_s,
            "requested_position_world": (
                self.command_requested_position.tolist()
                if self.command_requested_position is not None
                else None
            ),
            "admitted_position_world": (
                self.command_target_position.tolist()
                if self.command_target_position is not None
                else None
            ),
            "sampled_position_world": (
                self.command_sampled_position.tolist()
                if self.command_sampled_position is not None
                else None
            ),
            "sampled_velocity_world": self.command_sampled_velocity.tolist(),
            "sampled_acceleration_world": self.command_sampled_acceleration.tolist(),
            "measured_position_world": measured.tolist() if measured is not None else None,
            "start_position_world": (
                self.command_start_position.tolist()
                if self.command_start_position is not None
                else None
            ),
            "position_error_world": error_vector.tolist(),
            "position_error_m": float(np.linalg.norm(error_vector)),
            "clamped": self.command_clamped,
            "displacement_limit_m": COMMAND_MAX_DISPLACEMENT_M,
            "task_rms": self.command_task_rms,
            "task_clipped": self.command_task_clipped,
            "intent_executable": self.command_task_executable,
            "intent_suppressed": self.command_task_suppressed,
            "intent_status": self.command_task_status,
            "execution_bundle_suppressed": self.command_execution_bundle_suppressed,
            "command_descriptors_active": self.command_descriptors_active_at_solve,
            "measured_neutral_realization": (
                self.command_measured_neutral_realization_at_solve
            ),
            "station_target_at_solve_m": self.command_station_target_at_solve_m,
            "station_neutral_at_solve_m": self.command_station_neutral_at_solve_m,
            "station_neutral_error_at_solve_m": (
                self.command_station_target_at_solve_m
                - self.command_station_neutral_at_solve_m
            ),
            "station_requested_error_at_solve_m": (
                self.command_station_requested_error_at_solve_m
            ),
            "station_admitted_error_at_solve_m": (
                self.command_station_admitted_error_at_solve_m
            ),
            "station_error_limit_m": COMMAND_STATION_ERROR_LIMIT_M,
            "station_error_clamped_at_solve": (
                self.command_station_error_clamped_at_solve
            ),
            "cartesian_requested_x_at_solve_m": (
                self.command_cartesian_requested_x_at_solve_m
            ),
            "cartesian_admitted_x_at_solve_m": (
                self.command_cartesian_admitted_x_at_solve_m
            ),
            "cartesian_measured_x_at_solve_m": (
                self.command_cartesian_measured_x_at_solve_m
            ),
            "cartesian_x_position_neutral_residual_at_solve_m": (
                self.command_cartesian_admitted_x_at_solve_m
                - self.command_cartesian_measured_x_at_solve_m
            ),
            "cartesian_desired_vx_at_solve_m_s": (
                self.command_cartesian_admitted_vx_at_solve_m_s
            ),
            "cartesian_measured_vx_at_solve_m_s": (
                self.command_cartesian_measured_vx_at_solve_m_s
            ),
            "cartesian_vx_damping_residual_at_solve_m_s": (
                self.command_cartesian_admitted_vx_at_solve_m_s
                - self.command_cartesian_measured_vx_at_solve_m_s
            ),
            "cartesian_desired_ax_at_solve_m_s2": (
                self.command_cartesian_admitted_ax_at_solve_m_s2
            ),
            "cartesian_x_velocity_damping_beta_at_solve": (
                self.command_cartesian_x_velocity_damping_beta_at_solve
            ),
            "cartesian_x_damping_full_error_m": COMMAND_X_DAMPING_FULL_ERROR_M,
            "cartesian_x_damping_neutral_error_m": (
                COMMAND_X_DAMPING_NEUTRAL_ERROR_M
            ),
            "recovery_safe_ticks": self.command_recovery_safe_ticks,
            "recovery_required_ticks": COMMAND_RECOVERY_SAFE_TICKS,
            "hold_qualified_ticks": self.command_hold_ticks,
            "hold_required_ticks": COMMAND_HOLD_CONSECUTIVE_TICKS,
            "ik_target_residual_m": self.command_ik_target_residual_m,
            "ik_support_residual_m": self.command_ik_support_residual_m,
            "ik_balance_residual_m": self.command_ik_balance_residual_m,
            "last_admission": {
                "request_id": self.command_last_admission_request_id,
                "status": self.command_last_admission_status,
                "frame": self.command_last_admission_frame,
                "handle_id": self.command_last_admission_handle_id,
                "reason": self.command_last_admission_reason,
            },
            "last_rejection": {
                "request_id": self.command_last_rejection_request_id,
                "frame": self.command_last_rejection_frame,
                "handle_id": self.command_last_rejection_handle_id,
                "reason": self.command_last_rejection_reason,
            },
            "wbc_status_code": wbc_status_code,
            "wbc_status": (
                "unavailable"
                if wbc_status_code < 0
                else plant.STATUS_NAMES[wbc_status_code]
            ),
            "primary_status_code": (
                -1
                if latest_result is None
                else int(latest_result.get("command_primary_status", -1))
            ),
            "selected_primary": bool(
                latest_result is not None
                and latest_result.get("command_selected_primary", False)
            ),
            "selected_status_code": (
                -1
                if latest_result is None
                else int(latest_result.get("command_selected_status", -1))
            ),
        }

    def step(self, request: dict[str, Any]) -> dict[str, Any]:
        # Rust includes the desired state on every stream request, making the
        # worker's reported state authoritative even if a command arrives
        # between two 50 Hz ticks.
        requested_paused = bool(request.get("paused", self.paused))
        requested_reset = bool(request.get("reset", False))
        automatic_reset_reason = (
            None if requested_reset else self.pending_automatic_reset
        )
        self.pending_automatic_reset = None
        if automatic_reset_reason == "fall":
            self.reset(fall=True)
        if requested_reset:
            self.reset()
        self.paused = requested_paused
        if self.paused:
            self.last_external_wrench_world.fill(0.0)
            self.last_external_centroidal_moment_world.fill(0.0)
            self.last_external_application_point_world.fill(0.0)
            self.last_external_force_world.fill(0.0)
            self.last_external_root_body_id = -1
            self.last_external_wrench_valid = False
        command = request.get("external_load")
        active = isinstance(command, dict) and bool(command.get("active", False))
        if self.paused and active:
            return {
                "type": "plant_error",
                "message": "active external load is disabled while MuJoCo is paused",
            }
        request_id = command.get("request_id") if active else None
        body_name = str(command.get("body", "base")) if active else "base"
        force = finite_vector(command.get("force_world"), 3) if active else None
        point = (
            finite_vector(command.get("application_point_world"), 3)
            if active
            else None
        )
        provenance = command.get("provenance") if active else None
        valid_provenance = bool(
            isinstance(provenance, dict)
            and provenance.get("source")
            in ("interactive_operator", "evaluation_harness")
            and provenance.get("load_class") == "declared_continuous_wrench"
            and provenance.get("force_frame") == "world"
            and provenance.get("application_point_frame") == "world"
        )
        if active and not valid_provenance:
            return {
                "type": "plant_error",
                "message": "active external load requires executable declared-wrench provenance",
            }
        if active and (force is None or point is None):
            return {
                "type": "plant_error",
                "message": "active external load requires finite force_world[3] and application_point_world[3]",
            }
        if active and body_name not in self.body_by_name:
            return {
                "type": "plant_error",
                "message": f"unknown MuJoCo body: {body_name}",
            }
        if active and float(np.linalg.norm(force)) > MAX_FORCE_N + 1.0e-12:
            return {
                "type": "plant_error",
                "message": f"push force exceeds {MAX_FORCE_N:g} N",
            }
        body_id = self.body_by_name[body_name] if active else None
        if active:
            application_offset = point - self.data.xipos[body_id]
            if float(np.linalg.norm(application_offset)) > MAX_APPLICATION_OFFSET_M + 1.0e-12:
                return {
                    "type": "plant_error",
                    "message": (
                        "push application point exceeds the "
                        f"{MAX_APPLICATION_OFFSET_M:g} m body-COM offset limit"
                    ),
                }

        target_command = request.get("target_command")
        if (
            not requested_reset
            and automatic_reset_reason is None
            and isinstance(target_command, dict)
            and bool(target_command.get("active", True))
        ):
            target_request_id = target_command.get("request_id")
            valid_target_request_id = bool(
                isinstance(target_request_id, int)
                and not isinstance(target_request_id, bool)
                and target_request_id > 0
            )
            if (
                not valid_target_request_id
                or target_request_id != self.command_last_admission_request_id
            ):
                if self.paused:
                    self._reject_target_command(
                        target_command,
                        "target commit is disabled while MuJoCo is paused",
                    )
                else:
                    self._accept_target_command(target_command)

        suppress_target_execution = False
        if active and self.command_plan is not None:
            # PUSH owns the declared wrench for this tick. Preserve the
            # target's public identity and absolute goal, but temporarily
            # remove its complete execution bundle so balance/contact can
            # absorb the load without competing station or posture requests.
            self.command_execution_suspended_for_external_load = True
            self.command_recovery_safe_ticks = 0
            suppress_target_execution = True
            self.command_phase = "suppressed"
            self.command_reason = "target execution suspended for external load"
            self.command_sampled_velocity.fill(0.0)
            self.command_sampled_acceleration.fill(0.0)
            self.command_hold_ticks = 0
        elif self.command_execution_suspended_for_external_load:
            # Release clears the wrench on this very solve. Rebase the private
            # trajectory from the measured state while retaining request ID,
            # normalized handle, and the admitted absolute goal.
            if self.command_recovery_safe_ticks >= COMMAND_RECOVERY_SAFE_TICKS:
                self._rebase_target_command_from_measured_state()
                self.command_execution_suspended_for_external_load = False
                self.command_recovery_safe_ticks = 0
            else:
                suppress_target_execution = True
                self.command_phase = "suppressed"
                self.command_reason = (
                    "target execution waiting for balance recovery after external load"
                )
        self.command_execution_bundle_suppressed = suppress_target_execution

        started = time.perf_counter_ns()
        peak_capture_pressure = 0.0
        minimum_station_authority = 1.0
        maximum_torque_utilization = 0.0
        maximum_fall_safe_risk = 0.0
        minimum_fall_safe_primary_authority = 1.0
        minimum_fall_safe_fresh_command_authority = 1.0
        maximum_fall_safe_mode = 0
        fall_safe_reason_flags = 0
        maximum_controller_step_ns = 0
        latest_observed_contact_active = self.no_contact_active
        latest_physics_contact_active = self.no_contact_active
        latest_wbc_observed_wheel_normal_force_n = self.no_wheel_normal_force_n
        latest_physics_wheel_normal_force_n = self.no_wheel_normal_force_n
        wbc_observation_frame_index = self.physics_contact_frame_index
        physics_contact_window_start = self.physics_contact_frame_index + 1
        physics_contact_window_end = self.physics_contact_frame_index
        physics_contact_window_valid = False
        physics_contact_loss_mask = self.physics_contact_loss_mask
        physics_contact_gain_mask = self.physics_contact_gain_mask
        physics_contact_loss_mask.fill(0)
        physics_contact_gain_mask.fill(0)
        # A paused frame keeps the last WBC diagnostics while explicitly
        # labelling the solve as paused below; no controller call occurs.
        latest_result: dict[str, Any] | None = self.last_result
        # Preserve the historical application-body inertial telemetry while
        # keeping centroidal and root-origin moments separate. The three
        # reference points are intentional: UI/operator compatibility uses
        # the application body's inertial origin, centroidal objectives use
        # the aggregate system CoM, and the Rust floating dynamics equality
        # uses the root-body origin that defines its generalized tangent.
        applied_moment_world = np.zeros(3, np.float64)
        applied_centroidal_moment_world = np.zeros(3, np.float64)
        applied_root_moment_world = np.zeros(3, np.float64)
        observed_external_wrench_world = np.zeros(6, np.float64)
        observed_external_centroidal_moment_world = np.zeros(3, np.float64)
        observed_external_wrench_valid = False
        application_offset_m = 0.0
        maximum_moment_nm = 0.0
        maximum_centroidal_moment_nm = 0.0
        maximum_root_moment_nm = 0.0
        for _ in range(self.control_ticks_per_stream):
            if self.paused:
                # Stream heartbeats still carry a frozen state while the
                # underlying simulator and WBC do no work.
                break
            root_position, root_quaternion, root_twist, q, v = plant.read_state(
                self.model, self.data
            )
            # This is the latest completed physical observation.  It is the
            # only contact mask passed to Rust for this WBC call; the masks
            # produced by the five upcoming physics steps belong to the next
            # control boundary.
            if self.physics_contact_frame_index == 0:
                # Frame zero is the only startup sample outside a completed
                # physics window. After this call the WBC consumes the cached
                # final observation of the prior five-substep window.
                plant.measured_wheel_ground_contacts_into(
                    self.model,
                    self.data,
                    self.wheel_contact_body_sets,
                    self.observed_contact_scratch,
                )
                np.copyto(
                    self.wbc_observation_active,
                    self.observed_contact_scratch,
                )
                plant.measured_wheel_ground_normal_forces_into(
                    self.model,
                    self.data,
                    self.wheel_contact_body_sets,
                    self.observed_wheel_normal_force_n,
                    self.contact_wrench_scratch,
                )
                np.copyto(
                    self.wbc_observation_wheel_normal_force_n,
                    self.observed_wheel_normal_force_n,
                )
            else:
                np.copyto(
                    self.wbc_observation_active,
                    self.observed_contact_active,
                )
                np.copyto(
                    self.wbc_observation_wheel_normal_force_n,
                    self.observed_wheel_normal_force_n,
                )
            np.copyto(self.observed_contact_active, self.wbc_observation_active)
            self.wbc_observation_frame_index = self.physics_contact_frame_index
            wbc_observation_frame_index = self.wbc_observation_frame_index
            ground_position = float(np.mean(self.data.xpos[self.wheel_bodies, 0]))
            ground_height = float(np.mean(self.data.xpos[self.wheel_bodies, 2]))
            if suppress_target_execution:
                command_target_position = None
                command_target_velocity = None
                command_target_acceleration = None
                command_progress = self.command_sampled_progress
                command_root_position = None
                command_root_velocity = None
                command_root_acceleration = None
                command_joint_position = None
                command_joint_velocity = None
                command_joint_acceleration = None
                command_wheel_reference_position = None
                command_wheel_reference_velocity = None
                command_wheel_reference_acceleration = None
            else:
                (
                    command_target_position,
                    command_target_velocity,
                    command_target_acceleration,
                    command_progress,
                ) = self._sample_target_command()
                (
                    command_root_position,
                    command_root_velocity,
                    command_root_acceleration,
                    command_joint_position,
                    command_joint_velocity,
                    command_joint_acceleration,
                    command_wheel_reference_position,
                    command_wheel_reference_velocity,
                    command_wheel_reference_acceleration,
                ) = self._sample_realization_command()
            station_neutral_position = float(
                np.mean(self.data.xpos[self.command_wheel_center_bodies, 0])
            )
            if suppress_target_execution:
                # Full target suppression includes the balance-owned common
                # rolling/station component. Neutralize it at the current
                # measured wheel-center position before every PUSH/recovery
                # solve, rather than retaining an earlier target station.
                self.controller.target_ground_position = station_neutral_position
                station_requested_error = 0.0
                station_admitted_error = 0.0
            elif command_target_position is not None and self.command_plan is not None:
                # A world-X frame request moves through Upkie's ordinary
                # capture/station authority.  This is uniform for every
                # handle and keeps rolling balance in the controller that
                # already owns both wheels, instead of synthesizing a
                # differential wheel command from static IK.
                station_requested_error = float(
                    command_target_position[0]
                    - self.data.xpos[self.body_by_name[self.command_frame], 0]
                )
                station_admitted_error = float(
                    np.clip(
                        station_requested_error,
                        -COMMAND_STATION_ERROR_LIMIT_M,
                        COMMAND_STATION_ERROR_LIMIT_M,
                    )
                )
                self.controller.target_ground_position = (
                    station_neutral_position + station_admitted_error
                )
            else:
                station_requested_error = 0.0
                station_admitted_error = 0.0
            self.command_station_neutral_at_solve_m = station_neutral_position
            self.command_station_target_at_solve_m = float(
                self.controller.target_ground_position
            )
            self.command_station_requested_error_at_solve_m = (
                station_requested_error
            )
            self.command_station_admitted_error_at_solve_m = station_admitted_error
            self.command_station_error_clamped_at_solve = bool(
                abs(station_requested_error - station_admitted_error) > 1.0e-15
            )
            self.command_wheel_measured_position[:] = self.data.xpos[
                self.command_wheel_center_bodies, 0
            ]
            for wheel_index, wheel_body in enumerate(
                self.command_wheel_center_bodies
            ):
                mujoco.mj_jacBody(
                    self.model,
                    self.data,
                    self.command_wheel_velocity_jacobian,
                    self.command_wheel_velocity_angular_jacobian,
                    int(wheel_body),
                )
                self.command_wheel_measured_velocity[wheel_index] = float(
                    np.dot(
                        self.command_wheel_velocity_jacobian[0],
                        self.data.qvel,
                    )
                )
            if (
                command_target_position is not None
                and self.command_plan is not None
                and bool(
                    self.command_plan.get("measured_neutral_realization", False)
                )
            ):
                # Do not restore the pre-PUSH endpoint IK realization. Track
                # the measured state at priority 3 so this bundle contributes
                # zero feedback acceleration while Cartesian intent and the
                # balance-owned station mapping resume.
                command_root_position = root_position.copy()
                command_root_velocity = root_twist[3:6].copy()
                command_root_acceleration = np.zeros(3, np.float64)
                command_joint_position = q.copy()
                command_joint_velocity = v.copy()
                command_joint_acceleration = np.zeros(6, np.float64)
                command_wheel_reference_position = (
                    self.command_wheel_measured_position.copy()
                )
                command_wheel_reference_velocity = (
                    self.command_wheel_measured_velocity.copy()
                )
                command_wheel_reference_acceleration = np.zeros(2, np.float64)
            if command_target_position is not None:
                self.command_sampled_position = command_target_position.copy()
                np.copyto(self.command_sampled_velocity, command_target_velocity)
                np.copyto(
                    self.command_sampled_acceleration,
                    command_target_acceleration,
                )
                self.command_sampled_progress = command_progress
            self.command_descriptors_active_at_solve = bool(
                command_target_position is not None
            )
            self.command_measured_neutral_realization_at_solve = bool(
                self.command_descriptors_active_at_solve
                and self.command_plan is not None
                and self.command_plan.get("measured_neutral_realization", False)
            )
            command_wbc_target_position = command_target_position
            command_wbc_target_velocity = command_target_velocity
            command_wbc_target_acceleration = command_target_acceleration
            if command_target_position is not None and self.command_frame is not None:
                measured_frame_x = float(
                    self.data.xpos[self.body_by_name[self.command_frame], 0]
                )
                mujoco.mj_jacBody(
                    self.model,
                    self.data,
                    self.command_ik_body_jacobian,
                    self.command_ik_body_angular_jacobian,
                    self.body_by_name[self.command_frame],
                )
                measured_frame_vx = float(
                    np.dot(self.command_ik_body_jacobian[0], self.data.qvel)
                )
                command_wbc_target_position = command_target_position.copy()
                command_wbc_target_position[0] = measured_frame_x
                command_wbc_target_velocity = command_target_velocity.copy()
                damping_coordinate = float(
                    np.clip(
                        (
                            abs(station_admitted_error)
                            - COMMAND_X_DAMPING_FULL_ERROR_M
                        )
                        / (
                            COMMAND_X_DAMPING_NEUTRAL_ERROR_M
                            - COMMAND_X_DAMPING_FULL_ERROR_M
                        ),
                        0.0,
                        1.0,
                    )
                )
                damping_beta = damping_coordinate * damping_coordinate * (
                    3.0 - 2.0 * damping_coordinate
                )
                # Preserve velocity-neutral Cartesian X realization while
                # the station executes the trajectory. Once the sample
                # reaches its endpoint, smoothly remove measured X velocity
                # feed-through as station error approaches zero.
                # This adds only passive damping; station remains the sole
                # owner of target-directed world-X motion.
                if command_progress < 1.0 - 1.0e-12:
                    damping_beta = 1.0
                command_wbc_target_velocity[0] = damping_beta * measured_frame_vx
                command_wbc_target_acceleration = command_target_acceleration.copy()
                command_wbc_target_acceleration[0] = 0.0
                self.command_cartesian_requested_x_at_solve_m = float(
                    command_target_position[0]
                )
                self.command_cartesian_admitted_x_at_solve_m = measured_frame_x
                self.command_cartesian_measured_x_at_solve_m = measured_frame_x
                self.command_cartesian_admitted_vx_at_solve_m_s = float(
                    command_wbc_target_velocity[0]
                )
                self.command_cartesian_measured_vx_at_solve_m_s = measured_frame_vx
                self.command_cartesian_admitted_ax_at_solve_m_s2 = 0.0
                self.command_cartesian_x_velocity_damping_beta_at_solve = (
                    damping_beta
                )
            else:
                self.command_cartesian_requested_x_at_solve_m = 0.0
                self.command_cartesian_admitted_x_at_solve_m = 0.0
                self.command_cartesian_measured_x_at_solve_m = 0.0
                self.command_cartesian_admitted_vx_at_solve_m_s = 0.0
                self.command_cartesian_measured_vx_at_solve_m_s = 0.0
                self.command_cartesian_admitted_ax_at_solve_m_s2 = 0.0
                self.command_cartesian_x_velocity_damping_beta_at_solve = 1.0
            if active:
                application_offset = point - self.data.xipos[body_id]
                application_offset_m = float(np.linalg.norm(application_offset))
                applied_moment_world = np.cross(application_offset, force)
                applied_centroidal_moment_world = np.cross(
                    point - np.asarray(self.data.subtree_com[0], dtype=np.float64),
                    force,
                )
                root_body_id = int(self.model.body_rootid[body_id])
                applied_root_moment_world = np.cross(
                    point - np.asarray(self.data.xpos[root_body_id], dtype=np.float64),
                    force,
                )
                observed_external_wrench_world[:3] = applied_root_moment_world
                observed_external_wrench_world[3:] = force
                np.copyto(
                    observed_external_centroidal_moment_world,
                    applied_centroidal_moment_world,
                )
                observed_external_wrench_valid = True
            else:
                observed_external_wrench_world.fill(0.0)
                observed_external_centroidal_moment_world.fill(0.0)
                observed_external_wrench_valid = False
            result = self.controller.solve(
                root_position,
                root_quaternion,
                root_twist,
                q,
                v,
                ground_position,
                ground_height,
                observed_contact_active=self.wbc_observation_active,
                observed_wheel_normal_force_n=(
                    self.wbc_observation_wheel_normal_force_n
                ),
                observed_external_wrench_world=(
                    observed_external_wrench_world
                    if observed_external_wrench_valid
                    else None
                ),
                observed_external_centroidal_moment_world=(
                    observed_external_centroidal_moment_world
                    if observed_external_wrench_valid
                    else None
                ),
                observed_contact_available=True,
                command_frame=(
                    self.command_frame
                    if command_target_position is not None
                    else None
                ),
                command_target_position=command_wbc_target_position,
                command_target_velocity=command_wbc_target_velocity,
                command_target_acceleration=command_wbc_target_acceleration,
                command_wheel_reference_position=command_wheel_reference_position,
                command_wheel_reference_velocity=command_wheel_reference_velocity,
                command_wheel_reference_acceleration=command_wheel_reference_acceleration,
                command_wheel_measured_position=(
                    self.command_wheel_measured_position
                    if command_wheel_reference_position is not None
                    else None
                ),
                command_wheel_measured_velocity=(
                    self.command_wheel_measured_velocity
                    if command_wheel_reference_position is not None
                    else None
                ),
                command_root_position=command_root_position,
                command_root_velocity=command_root_velocity,
                command_root_acceleration=command_root_acceleration,
                command_joint_position=command_joint_position,
                command_joint_velocity=command_joint_velocity,
                command_joint_acceleration=command_joint_acceleration,
            )
            latest_result = result
            self.data.ctrl[self.actuator_ids] = result["torque"]
            self.data.qfrc_applied.fill(0.0)
            self.data.xfrc_applied.fill(0.0)
            if active:
                maximum_moment_nm = max(
                    maximum_moment_nm, float(np.linalg.norm(applied_moment_world))
                )
                maximum_centroidal_moment_nm = max(
                    maximum_centroidal_moment_nm,
                    float(np.linalg.norm(applied_centroidal_moment_world)),
                )
                maximum_root_moment_nm = max(
                    maximum_root_moment_nm,
                    float(np.linalg.norm(applied_root_moment_world)),
                )
                mujoco.mj_applyFT(
                    self.model,
                    self.data,
                    force,
                    self.zero_torque,
                    point,
                    body_id,
                    self.data.qfrc_applied,
                )
            previous_contact = self.wbc_observation_active
            physics_contact_window_start = self.physics_contact_frame_index + 1
            physics_contact_loss_mask.fill(0)
            physics_contact_gain_mask.fill(0)
            for substep in range(self.physics_steps_per_control):
                mujoco.mj_step(self.model, self.data)
                # mj_step leaves collision data at its internal solve stage;
                # refresh it at the newly integrated state before declaring
                # this frame a completed 250 Hz observation.
                mujoco.mj_forward(self.model, self.data)
                self.physics_contact_frame_index += 1
                plant.measured_wheel_ground_contacts_into(
                    self.model,
                    self.data,
                    self.wheel_contact_body_sets,
                    self.physics_contact_window[substep],
                )
                plant.measured_wheel_ground_normal_forces_into(
                    self.model,
                    self.data,
                    self.wheel_contact_body_sets,
                    self.physics_wheel_normal_force_window_n[substep],
                    self.contact_wrench_scratch,
                )
                current_contact = self.physics_contact_window[substep]
                loss = self.physics_contact_loss_window[substep]
                gain = self.physics_contact_gain_window[substep]
                loss[...] = previous_contact & ~current_contact
                gain[...] = ~previous_contact & current_contact
                physics_contact_loss_mask |= loss
                physics_contact_gain_mask |= gain
                np.copyto(self.observed_contact_active, current_contact)
                previous_contact = current_contact
            np.copyto(self.observed_contact_active, self.physics_contact_window[-1])
            np.copyto(
                self.observed_wheel_normal_force_n,
                self.physics_wheel_normal_force_window_n[-1],
            )
            physics_contact_window_end = self.physics_contact_frame_index
            physics_contact_window_valid = True
            if active:
                self.last_external_wrench_world[:3] = applied_root_moment_world
                self.last_external_wrench_world[3:] = force
                np.copyto(
                    self.last_external_centroidal_moment_world,
                    applied_centroidal_moment_world,
                )
                np.copyto(self.last_external_application_point_world, point)
                np.copyto(self.last_external_force_world, force)
                self.last_external_root_body_id = int(
                    self.model.body_rootid[body_id]
                )
                self.last_external_wrench_valid = True
            else:
                self.last_external_wrench_world.fill(0.0)
                self.last_external_centroidal_moment_world.fill(0.0)
                self.last_external_application_point_world.fill(0.0)
                self.last_external_force_world.fill(0.0)
                self.last_external_root_body_id = -1
                self.last_external_wrench_valid = False
            latest_observed_contact_active = self.wbc_observation_active
            latest_physics_contact_active = self.observed_contact_active
            latest_wbc_observed_wheel_normal_force_n = (
                self.wbc_observation_wheel_normal_force_n
            )
            latest_physics_wheel_normal_force_n = self.observed_wheel_normal_force_n
            peak_capture_pressure = max(
                peak_capture_pressure, float(result["capture_pressure"])
            )
            minimum_station_authority = min(
                minimum_station_authority, float(result["station_authority"])
            )
            maximum_torque_utilization = max(
                maximum_torque_utilization, float(result["torque_utilization"])
            )
            maximum_fall_safe_risk = max(
                maximum_fall_safe_risk, float(result["fall_safe_risk"])
            )
            minimum_fall_safe_primary_authority = min(
                minimum_fall_safe_primary_authority,
                float(result["fall_safe_primary_authority"]),
            )
            minimum_fall_safe_fresh_command_authority = min(
                minimum_fall_safe_fresh_command_authority,
                float(result["fall_safe_fresh_command_authority"]),
            )
            maximum_fall_safe_mode = max(
                maximum_fall_safe_mode, int(result["fall_safe_mode"])
            )
            fall_safe_reason_flags |= int(result["fall_safe_reason_flags"])
            maximum_controller_step_ns = max(
                maximum_controller_step_ns, int(result["step_ns"])
            )
            if (
                suppress_target_execution
                and self.command_execution_suspended_for_external_load
                and not active
            ):
                if self._target_recovery_tick_is_safe(result):
                    self.command_recovery_safe_ticks += 1
                else:
                    self.command_recovery_safe_ticks = 0
            if suppress_target_execution:
                self.command_task_rms = 0.0
                self.command_task_clipped = False
                self.command_task_executable = False
                self.command_task_suppressed = True
                self.command_task_status = "external load owns target execution pause"
            else:
                self._advance_target_command(command_progress, result)

        numeric_reset = False
        if not self._validate_state():
            self.reset(numeric=True)
            numeric_reset = True
            latest_result = None
            latest_observed_contact_active = self.no_contact_active
            latest_physics_contact_active = self.no_contact_active
            latest_wbc_observed_wheel_normal_force_n = self.no_wheel_normal_force_n
            latest_physics_wheel_normal_force_n = self.no_wheel_normal_force_n
            physics_contact_window_valid = False
        # The final substep's explicit `mj_forward` leaves kinematics and
        # contact data valid. Avoiding another pass here keeps the final window
        # sample identical to the state consumed at the next WBC boundary.
        root_position, root_quaternion, root_twist, q, v = plant.read_state(
            self.model, self.data
        )
        rotation = plant.quaternion_rotation_vector(root_quaternion)
        tilt = float(np.linalg.norm(rotation[:2]))
        fallen = bool(
            float(root_position[2]) < FALL_HEIGHT_M or tilt > FALL_TILT_RAD
        )
        if fallen:
            self.pending_automatic_reset = "fall"
        self.tick += 1
        self.last_result = latest_result
        contacts = self._contacts()
        # The adapter retains its internal debounce state across an ordinary
        # pause so a later fresh observation can be processed causally.  That
        # retained state is not current hard authority while no WBC solve ran,
        # so the stream must fail closed instead of publishing it as active.
        published_contact_state = latest_result is not None and not self.paused
        if not published_contact_state:
            latest_observed_contact_active = self.no_contact_active
            latest_physics_contact_active = self.no_contact_active
            latest_wbc_observed_wheel_normal_force_n = self.no_wheel_normal_force_n
            latest_physics_wheel_normal_force_n = self.no_wheel_normal_force_n
            physics_contact_window_valid = False
            physics_contact_loss_mask.fill(0)
            physics_contact_gain_mask.fill(0)
        published_debounced_contact_active = (
            self.controller.contact_debounced
            if published_contact_state
            else self.no_contact_active
        )
        published_hard_contact_active = (
            self.controller.contact_active[0]
            if published_contact_state
            else self.no_contact_active
        )
        published_wbc_admitted = bool(
            published_contact_state
            and latest_result is not None
            and int(latest_result["status"]) in (0, 1)
        )
        target_telemetry = self._target_telemetry(
            latest_result if published_contact_state else None
        )
        # Keep the debounced/hard mask as a diagnostic of the authority stack,
        # but never advertise it as executable when the solve was not
        # admitted (for example MaxIterations or an explicit pause).
        published_hard_contact_executable = (
            published_hard_contact_active
            if published_wbc_admitted
            else self.no_contact_active
        )
        # Keep the optional support-contingency witness alongside the primary
        # WBC metrics.  This is deliberately evaluation-only: the public
        # worker constructs the adapter with no controller overrides, but a
        # measured profile can now tell the browser whether a fallback solve
        # was merely queried, admitted, or actually selected.  Never publish
        # a stale contingency result across pause/reset boundaries.
        support_contingency_result = (
            latest_result if published_contact_state else None
        )
        ground_contacts = [contact for contact in contacts if contact["ground"]]
        total_ground_normal_force_n = sum(
            float(contact["normal_force_n"]) for contact in ground_contacts
        )
        minimum_contact_distance_m = min(
            (contact["distance_m"] for contact in contacts), default=0.0
        )
        solver_niter = np.asarray(getattr(self.data, "solver_niter", [0]))
        solver_fwdinv = np.asarray(
            getattr(self.data, "solver_fwdinv", [0.0, 0.0]), dtype=np.float64
        ).copy()
        # These are simulator witnesses, not controller-owned estimates. Keep
        # them adjacent to the MuJoCo timing/contact record so the browser can
        # distinguish measured plant motion from the guided WBC preview.
        actuator_effort_nm = np.asarray(
            self.data.ctrl[self.actuator_ids], dtype=np.float64
        ).copy()
        actuator_velocity_rad_s = np.asarray(
            self.data.actuator_velocity[self.actuator_ids], dtype=np.float64
        ).copy()
        actuator_mechanical_power_w = (
            actuator_effort_nm * actuator_velocity_rad_s
        )
        actuator_effort_utilization = np.divide(
            np.abs(actuator_effort_nm),
            self.actuator_effort_limits_nm,
            out=np.zeros_like(actuator_effort_nm),
            where=np.isfinite(self.actuator_effort_limits_nm)
            & (self.actuator_effort_limits_nm > 0.0),
        )
        actuator_force = np.asarray(self.data.actuator_force, dtype=np.float64).copy()
        generalized_acceleration = np.asarray(
            self.data.qacc, dtype=np.float64
        ).copy()
        actuator_generalized_force = np.asarray(
            self.data.qfrc_actuator, dtype=np.float64
        ).copy()
        passive_generalized_force = np.asarray(
            self.data.qfrc_passive, dtype=np.float64
        ).copy()
        bias_generalized_force = np.asarray(
            self.data.qfrc_bias, dtype=np.float64
        ).copy()
        constraint_generalized_force = np.asarray(
            self.data.qfrc_constraint, dtype=np.float64
        ).copy()
        constraint_force = np.asarray(self.data.efc_force, dtype=np.float64).copy()
        constraint_position = np.asarray(self.data.efc_pos, dtype=np.float64).copy()
        constraint_velocity = np.asarray(self.data.efc_vel, dtype=np.float64).copy()
        center_of_mass_world = np.asarray(
            self.data.subtree_com[0], dtype=np.float64
        ).copy()
        mujoco.mj_energyPos(self.model, self.data)
        mujoco.mj_energyVel(self.model, self.data)
        warning_count = sum(int(warning.number) for warning in self.data.warning)
        response = {
            "type": "plant_state",
            "tick": self.tick,
            "reset_epoch": self.reset_epoch,
            "paused": self.paused,
            "numeric_reset": numeric_reset,
            "automatic_reset_reason": automatic_reset_reason,
            "automatic_reset_pending": self.pending_automatic_reset,
            "command_id": request.get("command_id"),
            "command_expired": bool(request.get("command_expired", False)),
            "target_command": target_telemetry,
            "frames": self._frames(),
            "root_position": root_position.tolist(),
            "root_quaternion_wxyz": root_quaternion.tolist(),
            "root_twist_world": root_twist.tolist(),
            "center_of_mass_world": center_of_mass_world.tolist(),
            "joint_positions": q.tolist(),
            "joint_velocities": v.tolist(),
            "wbc_observed_contact_active": latest_observed_contact_active.tolist(),
            "wbc_observed_wheel_normal_force_n": (
                latest_wbc_observed_wheel_normal_force_n.tolist()
            ),
            "wbc_observation": {
                "contact_active": latest_observed_contact_active.tolist(),
                "wheel_normal_force_n": (
                    latest_wbc_observed_wheel_normal_force_n.tolist()
                ),
                "physics_frame_index": int(wbc_observation_frame_index),
                "source": self.wbc_observation_source,
            },
            "physics_contact_active": latest_physics_contact_active.tolist(),
            "physics_wheel_normal_force_n": (
                latest_physics_wheel_normal_force_n.tolist()
            ),
            "wbc_predicted_normal_force_n": (
                self.no_wheel_normal_force_n.tolist()
                if latest_result is None or self.paused
                else np.asarray(latest_result["normal_force"], np.float64).tolist()
            ),
            "wbc_debounced_contact_active": published_debounced_contact_active.tolist(),
            "wbc_hard_contact_active": published_hard_contact_active.tolist(),
            "wbc_hard_contact_executable": published_hard_contact_executable.tolist(),
            "actuator_effort_nm": actuator_effort_nm.tolist(),
            "actuator_effort_limit_nm": [
                float(limit) if np.isfinite(limit) else None
                for limit in self.actuator_effort_limits_nm
            ],
            "actuator_effort_utilization": actuator_effort_utilization.tolist(),
            "actuator_velocity_rad_s": actuator_velocity_rad_s.tolist(),
            "actuator_mechanical_power_w": actuator_mechanical_power_w.tolist(),
            "actuator_force": actuator_force.tolist(),
            "generalized_acceleration": generalized_acceleration.tolist(),
            "actuator_generalized_force": actuator_generalized_force.tolist(),
            "passive_generalized_force": passive_generalized_force.tolist(),
            "bias_generalized_force": bias_generalized_force.tolist(),
            "constraint_generalized_force": constraint_generalized_force.tolist(),
            "constraint_force": constraint_force.tolist(),
            "constraint_position": constraint_position.tolist(),
            "constraint_velocity": constraint_velocity.tolist(),
            "contacts": contacts,
            "simulator": {
                "backend": "MuJoCo",
                "paused": self.paused,
                "time_s": float(self.data.time),
                "physics_dt_s": self.physics_dt,
                "control_dt_s": self.control_dt,
                "physics_substeps": self.physics_steps_per_control,
                "physics_frame_index": int(self.physics_contact_frame_index),
                "contact_window_frame_start": int(physics_contact_window_start),
                "contact_window_frame_end": int(physics_contact_window_end),
                "contact_window_valid": physics_contact_window_valid,
                "contact_window_masks": self.physics_contact_window.tolist(),
                "contact_window_loss_masks": self.physics_contact_loss_window.tolist(),
                "contact_window_gain_masks": self.physics_contact_gain_window.tolist(),
                "contact_window_loss_mask": physics_contact_loss_mask.tolist(),
                "contact_window_gain_mask": physics_contact_gain_mask.tolist(),
                "wheel_normal_force_window_n": (
                    self.physics_wheel_normal_force_window_n.tolist()
                ),
                "solver_iterations": int(np.max(solver_niter)),
                "solver_forward_inverse": solver_fwdinv.tolist(),
                "constraint_count": int(self.data.nefc),
                "ground_plane_point_world": self.ground_plane_point_world.tolist(),
                "ground_plane_normal_world": self.ground_plane_normal_world.tolist(),
                "ground_plane_z_m": float(self.ground_plane_point_world[2]),
                "kinetic_energy_j": float(self.data.energy[1]),
                "potential_energy_j": float(self.data.energy[0]),
                "warning_count": warning_count,
            },
            "external_load": {
                "active": active,
                "request_id": request_id,
                "provenance": provenance if active else None,
                "body": body_name if active else None,
                "force_world": force.tolist() if active else [0.0, 0.0, 0.0],
                "application_point_world": point.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "moment_world_nm": applied_moment_world.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "centroidal_moment_world_nm": applied_centroidal_moment_world.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "root_moment_world_nm": applied_root_moment_world.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "wbc_observed_root_moment_world_nm": (
                    observed_external_wrench_world[:3].tolist()
                    if observed_external_wrench_valid
                    else [0.0, 0.0, 0.0]
                ),
                "wbc_observed_force_world_n": (
                    observed_external_wrench_world[3:].tolist()
                    if observed_external_wrench_valid
                    else [0.0, 0.0, 0.0]
                ),
                "wbc_feedforward_active": observed_external_wrench_valid,
                "wbc_observed_centroidal_moment_world_nm": (
                    observed_external_centroidal_moment_world.tolist()
                    if observed_external_wrench_valid
                    else [0.0, 0.0, 0.0]
                ),
                "application_offset_m": application_offset_m if active else 0.0,
                "maximum_moment_nm": maximum_moment_nm if active else 0.0,
                "maximum_centroidal_moment_nm": (
                    maximum_centroidal_moment_nm if active else 0.0
                ),
                "maximum_root_moment_nm": maximum_root_moment_nm if active else 0.0,
            },
            "measured_impact_impulse": {
                "available": False,
                "reason": "live gateway exposes MuJoCo contact force, not a typed impact impulse",
            },
            "unobserved_model_reserve": {
                "available": False,
                "reason": "live gateway does not estimate an unobserved-model reserve",
            },
            "metrics": {
                "paused": self.paused,
                "wbc_status": "paused"
                if self.paused
                else ("unavailable"
                if latest_result is None
                else plant.STATUS_NAMES[int(latest_result["status"])]),
                "wbc_admitted": published_wbc_admitted,
                "wbc_raw_status": "paused"
                if self.paused
                else ("unavailable"
                if latest_result is None
                else plant.STATUS_NAMES[int(latest_result["raw_wbc_status"])]),
                "wbc_raw_status_code": -1
                if latest_result is None or self.paused
                else int(latest_result["raw_wbc_status"]),
                "wbc_allocation_calls": 0
                if latest_result is None or self.paused
                else int(latest_result["allocation_calls"]),
                "wbc_allocated_bytes": 0
                if latest_result is None or self.paused
                else int(latest_result["allocated_bytes"]),
                "wbc_maximum_constraint_violation": 0.0
                if latest_result is None or self.paused
                else float(latest_result["maximum_constraint_violation"]),
                "wbc_dynamics_residual": 0.0
                if latest_result is None or self.paused
                else float(latest_result["dynamics_residual"]),
                "wbc_contact_residual": 0.0
                if latest_result is None or self.paused
                else float(latest_result["contact_residual"]),
                "wbc_support_contingency_enabled": bool(
                    self.controller.support_contingency_enabled
                ),
                "wbc_support_contingency_requested": bool(
                    support_contingency_result is not None
                    and support_contingency_result[
                        "support_contingency_requested"
                    ]
                ),
                "wbc_support_contingency_admitted": bool(
                    support_contingency_result is not None
                    and support_contingency_result[
                        "support_contingency_admitted"
                    ]
                ),
                "wbc_support_contingency_selected": bool(
                    support_contingency_result is not None
                    and support_contingency_result[
                        "support_contingency_selected"
                    ]
                ),
                "wbc_support_contingency_armed": bool(
                    support_contingency_result is not None
                    and support_contingency_result[
                        "support_contingency_armed"
                    ]
                ),
                "wbc_support_contingency_mode": 0
                if support_contingency_result is None
                else int(support_contingency_result["support_contingency_mode"]),
                "wbc_support_contingency_support_mask": 3
                if support_contingency_result is None
                else int(
                    support_contingency_result[
                        "support_contingency_support_mask"
                    ]
                ),
                "wbc_support_contingency_status_code": -1
                if support_contingency_result is None
                else int(support_contingency_result["support_contingency_status"]),
                "wbc_support_contingency_status": "unavailable"
                if support_contingency_result is None
                else plant.STATUS_NAMES[
                    int(support_contingency_result["support_contingency_status"])
                ],
                "wbc_support_contingency_maximum_constraint_violation": 0.0
                if support_contingency_result is None
                else float(
                    support_contingency_result[
                        "support_contingency_maximum_constraint_violation"
                    ]
                ),
                "wbc_support_contingency_author_step_us": 0.0
                if support_contingency_result is None
                else float(
                    support_contingency_result[
                        "support_contingency_author_step_ns"
                    ]
                )
                / 1.0e3,
                "wbc_support_contingency_step_us": 0.0
                if support_contingency_result is None
                else float(
                    support_contingency_result["support_contingency_wbc_step_ns"]
                )
                / 1.0e3,
                "wbc_support_contingency_candidate_power_w": 0.0
                if support_contingency_result is None
                else float(
                    support_contingency_result[
                        "support_contingency_candidate_power_w"
                    ]
                ),
                "wbc_support_contingency_incremental_power_w": 0.0
                if support_contingency_result is None
                else float(
                    support_contingency_result[
                        "support_contingency_incremental_power_w"
                    ]
                ),
                "wbc_support_contingency_forecast_guard_passed": bool(
                    support_contingency_result is not None
                    and support_contingency_result[
                        "support_contingency_forecast_guard_passed"
                    ]
                ),
                "wbc_support_load_guard_enabled": bool(
                    self.controller.support_load_guard_enabled
                ),
                "wbc_support_load_guard_active": bool(
                    published_contact_state
                    and latest_result is not None
                    and latest_result["support_load_guard_active"]
                ),
                "wbc_support_load_guard_authority": 0.0
                if latest_result is None or self.paused
                else float(latest_result["support_load_guard_authority"]),
                "wbc_support_load_reserve_action_enabled": bool(
                    self.controller.support_load_reserve_action_enabled
                ),
                "wbc_support_load_reserve_active": bool(
                    latest_result is not None
                    and not self.paused
                    and latest_result["support_load_reserve_active"]
                ),
                "wbc_support_load_reserve_authority": 0.0
                if latest_result is None or self.paused
                else float(latest_result["support_load_reserve_authority"]),
                "wbc_support_load_reserve_diagnostics": (
                    self.controller.support_load_reserve_diagnostics.tolist()
                ),
                "wbc_support_load_reserve_step_us": 0.0
                if latest_result is None or self.paused
                else float(latest_result["support_load_reserve_step_ns"]) / 1.0e3,
                "wbc_support_load_reserve_allocation_calls": 0
                if latest_result is None or self.paused
                else int(latest_result["support_load_reserve_allocation_calls"]),
                "wbc_support_load_reserve_allocated_bytes": 0
                if latest_result is None or self.paused
                else int(latest_result["support_load_reserve_allocated_bytes"]),
                "wbc_body_moment_rejection_enabled": bool(
                    self.controller.body_moment_rejection_enabled
                ),
                "wbc_body_moment_rejection_active": bool(
                    latest_result is not None
                    and not self.paused
                    and latest_result["body_moment_rejection_active"]
                ),
                "wbc_body_moment_rejection_authority": 0.0
                if latest_result is None or self.paused
                else float(latest_result["body_moment_rejection_authority"]),
                "wbc_body_moment_rejection_diagnostics": (
                    self.controller.body_moment_rejection_diagnostics.tolist()
                ),
                "wbc_body_moment_rejection_step_us": 0.0
                if latest_result is None or self.paused
                else float(latest_result["body_moment_rejection_step_ns"]) / 1.0e3,
                "wbc_body_moment_rejection_allocation_calls": 0
                if latest_result is None or self.paused
                else int(latest_result["body_moment_rejection_allocation_calls"]),
                "wbc_body_moment_rejection_allocated_bytes": 0
                if latest_result is None or self.paused
                else int(latest_result["body_moment_rejection_allocated_bytes"]),
                "wbc_single_support_reacquisition_enabled": bool(
                    self.controller.single_support_reacquisition_enabled
                ),
                "wbc_single_support_reacquisition_active": bool(
                    latest_result is not None
                    and not self.paused
                    and latest_result["single_support_reacquisition_active"]
                ),
                "wbc_single_support_reacquisition_authority": 0.0
                if latest_result is None or self.paused
                else float(
                    latest_result["single_support_reacquisition_authority"]
                ),
                "wbc_single_support_reacquisition_diagnostics": (
                    self.controller.single_support_reacquisition_diagnostics.tolist()
                ),
                "wbc_single_support_reacquisition_step_us": 0.0
                if latest_result is None or self.paused
                else float(
                    latest_result["single_support_reacquisition_step_ns"]
                )
                / 1.0e3,
                "wbc_single_support_reacquisition_allocation_calls": 0
                if latest_result is None or self.paused
                else int(
                    latest_result[
                        "single_support_reacquisition_allocation_calls"
                    ]
                ),
                "wbc_single_support_reacquisition_allocated_bytes": 0
                if latest_result is None or self.paused
                else int(
                    latest_result["single_support_reacquisition_allocated_bytes"]
                ),
                # R312 measured landing is an evaluation-only Rust-owned
                # phase boundary.  Publish its latest diagnostic vector next
                # to the older single-support witness so a replay can tell
                # precontact, touchdown-normal, force-backed qualification,
                # and contact-mode promotion apart.  Pause/reset fail closed
                # rather than leaking the previous solve's state.
                "measured_landing_enabled": bool(
                    self.controller.measured_landing_enabled
                ),
                "measured_landing_active": bool(
                    self.controller.measured_landing_enabled
                    and published_contact_state
                    and latest_result is not None
                    and latest_result["measured_landing_active"]
                ),
                "measured_landing_precontact_active": bool(
                    self.controller.measured_landing_enabled
                    and published_contact_state
                    and latest_result is not None
                    and latest_result["measured_landing_precontact_active"]
                ),
                "measured_landing_touchdown_normal_active": bool(
                    self.controller.measured_landing_enabled
                    and published_contact_state
                    and latest_result is not None
                    and latest_result["measured_landing_touchdown_normal_active"]
                ),
                "measured_landing_reacquisition_qualified": bool(
                    self.controller.measured_landing_enabled
                    and published_contact_state
                    and latest_result is not None
                    and latest_result["measured_landing_reacquisition_qualified"]
                ),
                "measured_landing_diagnostics": (
                    self.controller.measured_landing_diagnostics.tolist()
                    if self.controller.measured_landing_enabled
                    and published_contact_state
                    else [0.0] * 25
                ),
                "measured_landing_contact_modes": (
                    self.controller.measured_landing_contact_modes.tolist()
                    if self.controller.measured_landing_enabled
                    and published_contact_state
                    else [0, 0]
                ),
                "measured_landing_step_us": 0.0
                if latest_result is None or self.paused
                else float(self.controller.measured_landing_step_ns) / 1.0e3,
                "measured_landing_allocation_calls": 0
                if latest_result is None or self.paused
                else int(self.controller.measured_landing_allocation_calls),
                "measured_landing_allocated_bytes": 0
                if latest_result is None or self.paused
                else int(self.controller.measured_landing_allocated_bytes),
                "wbc_observed_contact_available": latest_result is not None
                and not self.paused,
                "wbc_observed_contact_active": latest_observed_contact_active.tolist(),
                "wbc_observed_wheel_normal_force_n": (
                    latest_wbc_observed_wheel_normal_force_n.tolist()
                ),
                "physics_wheel_normal_force_n": (
                    latest_physics_wheel_normal_force_n.tolist()
                ),
                "wbc_predicted_normal_force_n": (
                    self.no_wheel_normal_force_n.tolist()
                    if latest_result is None or self.paused
                    else np.asarray(latest_result["normal_force"], np.float64).tolist()
                ),
                "wbc_debounced_contact_active": published_debounced_contact_active.tolist(),
                "wbc_hard_contact_active": published_hard_contact_active.tolist(),
                "wbc_hard_contact_executable": published_hard_contact_executable.tolist(),
                "wbc_support_active_count": 0
                if not published_contact_state
                else int(latest_result["support_active_count"]),
                "wbc_support_active_left": 0
                if not published_contact_state
                else int(latest_result["support_active_left"]),
                "wbc_support_active_right": 0
                if not published_contact_state
                else int(latest_result["support_active_right"]),
                "wbc_contact_observation_status": 0
                if latest_result is None
                else int(latest_result["contact_observation_status"]),
                "wbc_contact_observation_provenance": 0
                if latest_result is None
                else int(latest_result["contact_observation_provenance"]),
                "wbc_contact_observation_flags": 0
                if latest_result is None
                else int(latest_result["contact_observation_flags"]),
                "capture_pressure": peak_capture_pressure,
                "capture_error_m": 0.0
                if latest_result is None
                else float(latest_result["capture_error"]),
                "station_authority": minimum_station_authority,
                "station_error_m": 0.0
                if latest_result is None
                else float(latest_result["station_error"]),
                "torque_utilization": maximum_torque_utilization,
                "fall_safe_mode": maximum_fall_safe_mode,
                "fall_safe_risk": maximum_fall_safe_risk,
                "fall_safe_primary_authority": minimum_fall_safe_primary_authority,
                "fall_safe_fresh_command_authority": minimum_fall_safe_fresh_command_authority,
                "fall_safe_reason_flags": fall_safe_reason_flags,
                "controller_step_us": maximum_controller_step_ns / 1.0e3,
                "worker_step_us": (time.perf_counter_ns() - started) / 1.0e3,
                "root_tilt_rad": tilt,
                "root_height_m": float(root_position[2]),
                "contact_count": int(self.data.ncon),
                "ground_contact_count": len(ground_contacts),
                "total_ground_normal_force_n": total_ground_normal_force_n,
                "minimum_contact_distance_m": minimum_contact_distance_m,
                "maximum_penetration_m": max(-minimum_contact_distance_m, 0.0),
                "maximum_abs_joint_speed_rad_s": float(
                    np.max(np.abs(v), initial=0.0)
                ),
                "maximum_abs_actuator_effort_nm": float(
                    np.max(np.abs(actuator_effort_nm), initial=0.0)
                ),
                "maximum_actuator_effort_utilization": float(
                    np.max(actuator_effort_utilization, initial=0.0)
                ),
                "maximum_abs_actuator_mechanical_power_w": float(
                    np.max(np.abs(actuator_mechanical_power_w), initial=0.0)
                ),
                "maximum_abs_generalized_acceleration": float(
                    np.max(np.abs(generalized_acceleration), initial=0.0)
                ),
                "maximum_abs_constraint_force": float(
                    np.max(np.abs(constraint_generalized_force), initial=0.0)
                ),
                "maximum_abs_constraint_scalar_force": float(
                    np.max(np.abs(constraint_force), initial=0.0)
                ),
                "maximum_abs_constraint_position": float(
                    np.max(np.abs(constraint_position), initial=0.0)
                ),
                "maximum_abs_constraint_velocity": float(
                    np.max(np.abs(constraint_velocity), initial=0.0)
                ),
                "numeric_resets": self.numeric_resets,
                "fall_resets": self.fall_resets,
                "fallen": fallen,
                "command_phase": target_telemetry["phase"],
                "command_request_id": target_telemetry["request_id"],
                "command_frame": target_telemetry["frame"],
                "command_reason": target_telemetry["reason"],
                "command_progress": target_telemetry["progress"],
                "command_position_error_m": target_telemetry["position_error_m"],
                "command_target_position_world": target_telemetry[
                    "admitted_position_world"
                ],
                "command_measured_position_world": target_telemetry[
                    "measured_position_world"
                ],
                "command_task_rms": target_telemetry["task_rms"],
                "command_task_clipped": target_telemetry["task_clipped"],
                "command_intent_executable": target_telemetry[
                    "intent_executable"
                ],
                "command_intent_suppressed": target_telemetry[
                    "intent_suppressed"
                ],
                "command_rolling_zero_mean_residual": 0.0
                if latest_result is None
                else float(latest_result["command_rolling_zero_mean_residual"]),
                "command_rolling_common_residual": 0.0
                if latest_result is None
                else float(latest_result["command_rolling_common_residual"]),
                "command_rolling_protected_residual": 0.0
                if latest_result is None
                else float(latest_result["command_rolling_protected_residual"]),
                "command_rolling_maximum_differential_acceleration_m_s2": 0.0
                if latest_result is None
                else float(
                    np.max(
                        np.abs(
                            latest_result[
                                "command_rolling_differential_acceleration"
                            ]
                        )
                    )
                ),
                "command_rolling_maximum_protected_acceleration_rad_s2": 0.0
                if latest_result is None
                else float(
                    np.max(
                        np.abs(
                            latest_result[
                                "command_rolling_protected_acceleration"
                            ]
                        )
                    )
                ),
                "command_rolling_null_scale": 0.0
                if latest_result is None
                else float(latest_result["command_rolling_null_scale"]),
                "command_rolling_load_fade": 0.0
                if latest_result is None
                else float(latest_result["command_rolling_load_fade"]),
            },
        }
        return response


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=pathlib.Path)
    # The public worker remains 50 Hz WBC over a 250 Hz MuJoCo plant. Faster
    # inner-rate profiles are explicit evaluation overrides only.
    parser.add_argument("--stream-dt", type=float, default=STREAM_DT)
    parser.add_argument("--control-dt", type=float, default=CONTROL_DT)
    parser.add_argument("--physics-dt", type=float, default=PHYSICS_DT)
    return parser.parse_args()


def emit(value: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(value, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def main() -> int:
    args = parse_args()
    live = LiveUpkiePlant(
        args.model.resolve(),
        stream_dt=args.stream_dt,
        control_dt=args.control_dt,
        physics_dt=args.physics_dt,
    )
    emit(live.hello())
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if not isinstance(request, dict) or request.get("type") != "step":
                emit(
                    {
                        "type": "plant_error",
                        "message": "worker accepts only step requests",
                    }
                )
                continue
            emit(live.step(request))
        except Exception as error:  # Keep one bad command from killing the plant.
            emit({"type": "plant_error", "message": f"worker error: {error}"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
