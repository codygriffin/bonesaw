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
# live prototype.  It moves the floating root through the existing Rust WBC
# task stack; it never writes MuJoCo qpos/qvel and it never becomes a wrench.
COMMAND_DEFAULT_DURATION_MS = 3000
COMMAND_MIN_DURATION_MS = 1000
COMMAND_MAX_DURATION_MS = 5000
COMMAND_MAX_ROOT_X_DELTA_M = 0.12
COMMAND_MAX_ROOT_Y_DELTA_M = 0.10
COMMAND_MAX_ROOT_DOWN_DELTA_M = 0.045
COMMAND_MAX_ROOT_UP_DELTA_M = 0.05
COMMAND_MIN_ROOT_HEIGHT_M = 0.30
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
        root_body_id = self.body_by_name.get("base")
        if root_body_id is None:
            raise RuntimeError("live Upkie model has no base body")
        root_origin = np.asarray(self.data.xpos[root_body_id], dtype=np.float64)
        self.command_frame_root_offsets = {}
        for frame_name in ("base", "torso"):
            body_id = self.body_by_name.get(frame_name)
            if body_id is not None:
                self.command_frame_root_offsets[frame_name] = (
                    np.asarray(self.data.xpos[body_id], dtype=np.float64)
                    - root_origin
                )
        if "torso" not in self.command_frame_root_offsets:
            raise RuntimeError("live Upkie model has no torso command frame")
        root_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root"
        )
        self.command_root_qpos = int(self.model.jnt_qposadr[root_joint])
        self.command_leg_joint_dofs = []
        for side in ("left", "right"):
            dofs = []
            for name in (f"{side}_hip", f"{side}_knee", f"{side}_ankle"):
                joint = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
                dofs.append(int(self.model.jnt_dofadr[joint]))
            self.command_leg_joint_dofs.append(tuple(dofs))
        self.command_wheel_center_bodies = tuple(
            self.body_by_name[name]
            for name in ("left_wheel_center", "right_wheel_center")
        )
        self.command_joint_qpos = tuple(
            int(self.model.jnt_qposadr[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )])
            for name in plant.JOINT_ORDER
        )
        self.command_ik_data = mujoco.MjData(self.model)
        self.command_nominal_root_position = nominal_root.copy()
        self.command_nominal_joint_position = self.controller.nominal_joint_position.copy()
        self.command_phase = "idle"
        self.command_request_id: int | None = None
        self.command_frame: str | None = None
        self.command_reason = "no target committed"
        self.command_requested_root_position: np.ndarray | None = None
        self.command_target_root_position: np.ndarray | None = None
        self.command_start_root_position: np.ndarray | None = None
        self.command_elapsed_s = 0.0
        self.command_duration_s = 0.0
        self.command_plan: dict[str, Any] | None = None
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
        # Delayed, plant-owned external-load observations. The current command
        # is applied only after the WBC solve, so these are the last completed
        # MuJoCo load. Root-origin wrench drives the floating dynamics rows;
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
                "type": "plant_target_commit",
                "accepted_frames": sorted(self.command_frame_root_offsets),
                "target_frame": "world metres",
                "trajectory": "bounded quintic root position/velocity/acceleration",
                "default_duration_ms": COMMAND_DEFAULT_DURATION_MS,
                "duration_ms": [COMMAND_MIN_DURATION_MS, COMMAND_MAX_DURATION_MS],
                "root_down_limit_m": COMMAND_MAX_ROOT_DOWN_DELTA_M,
                "root_up_limit_m": COMMAND_MAX_ROOT_UP_DELTA_M,
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
                    "external_load.root_moment_world_nm, re-expressed about current root origin and consumed one 50 Hz solve later"
                ),
                "wbc_external_centroidal_moment_observation": (
                    "external_load.centroidal_moment_world_nm, re-expressed about current aggregate CoM and consumed one 50 Hz solve later"
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

    def _solve_command_joint_target(
        self,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        q: np.ndarray,
        target_root: np.ndarray,
        wheel_targets: np.ndarray,
    ) -> np.ndarray | None:
        """Solve a small measured-state IK splice that preserves wheel anchors."""
        data = self.command_ik_data
        data.qpos[:] = self.data.qpos
        data.qvel[:] = self.data.qvel
        data.qpos[self.command_root_qpos : self.command_root_qpos + 3] = target_root
        data.qpos[self.command_root_qpos + 3 : self.command_root_qpos + 7] = root_quaternion
        for coordinate, qpos_address in enumerate(self.command_joint_qpos):
            data.qpos[qpos_address] = q[coordinate]
        jacobian = np.zeros((3, self.model.nv), np.float64)
        angular_jacobian = np.zeros((3, self.model.nv), np.float64)
        for _ in range(32):
            mujoco.mj_forward(self.model, data)
            maximum_error = 0.0
            for leg, body_id in enumerate(self.command_wheel_center_bodies):
                error = wheel_targets[leg] - np.asarray(data.xpos[body_id])
                maximum_error = max(maximum_error, float(np.linalg.norm(error)))
                mujoco.mj_jacBody(
                    self.model,
                    data,
                    jacobian,
                    angular_jacobian,
                    body_id,
                )
                columns = self.command_leg_joint_dofs[leg]
                step = np.linalg.lstsq(
                    jacobian[:, columns], error, rcond=None
                )[0]
                for column, delta in zip(columns, step, strict=True):
                    joint_id = int(self.model.dof_jntid[column])
                    qpos_address = int(self.model.jnt_qposadr[joint_id])
                    lower, upper = self.model.jnt_range[joint_id]
                    value = data.qpos[qpos_address] + 0.75 * float(delta)
                    if self.model.jnt_limited[joint_id]:
                        value = float(np.clip(value, lower, upper))
                    data.qpos[qpos_address] = value
            if maximum_error < 1.0e-7:
                break
        mujoco.mj_forward(self.model, data)
        residual = max(
            float(
                np.linalg.norm(
                    wheel_targets[leg]
                    - np.asarray(data.xpos[body_id], dtype=np.float64)
                )
            )
            for leg, body_id in enumerate(self.command_wheel_center_bodies)
        )
        if not math.isfinite(residual) or residual > 2.0e-4:
            return None
        return np.asarray(
            [data.qpos[address] for address in self.command_joint_qpos],
            dtype=np.float64,
        )

    def _reject_target_command(self, command: object, reason: str) -> None:
        request_id = command.get("request_id") if isinstance(command, dict) else None
        self.command_request_id = (
            int(request_id) if isinstance(request_id, int) and not isinstance(request_id, bool) else None
        )
        self.command_frame = (
            str(command.get("frame")) if isinstance(command, dict) else None
        )
        self.command_phase = "rejected"
        self.command_reason = reason
        self.command_requested_root_position = None
        # Keep a previously admitted target as the physical hold point.  A
        # malformed replacement command must not jerk the measured plant back
        # to the nominal stance.
        if self.command_plan is None:
            root_position, _, _, _, _ = plant.read_state(self.model, self.data)
            self.command_start_root_position = root_position.copy()
            self.command_target_root_position = root_position.copy()
            self.command_elapsed_s = 0.0
            self.command_duration_s = 0.0

    def _accept_target_command(
        self,
        command: object,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        q: np.ndarray,
    ) -> None:
        if not isinstance(command, dict):
            self._reject_target_command(command, "target command must be an object")
            return
        request_id = command.get("request_id")
        if isinstance(request_id, bool) or not isinstance(request_id, int) or request_id <= 0:
            self._reject_target_command(command, "target command request_id must be a positive integer")
            return
        frame = command.get("frame")
        if frame not in self.command_frame_root_offsets:
            self._reject_target_command(
                command,
                "target frame must be the Upkie torso or base command handle",
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
            or not COMMAND_MIN_DURATION_MS <= int(duration_ms) <= COMMAND_MAX_DURATION_MS
        ):
            self._reject_target_command(
                command,
                f"target duration must be an integer in {COMMAND_MIN_DURATION_MS}..{COMMAND_MAX_DURATION_MS} ms",
            )
            return
        requested_root = target - self.command_frame_root_offsets[str(frame)]
        nominal = self.command_nominal_root_position
        target_root = requested_root.copy()
        target_root[0] = float(
            np.clip(
                target_root[0],
                nominal[0] - COMMAND_MAX_ROOT_X_DELTA_M,
                nominal[0] + COMMAND_MAX_ROOT_X_DELTA_M,
            )
        )
        target_root[1] = float(
            np.clip(
                target_root[1],
                nominal[1] - COMMAND_MAX_ROOT_Y_DELTA_M,
                nominal[1] + COMMAND_MAX_ROOT_Y_DELTA_M,
            )
        )
        minimum_height = max(
            COMMAND_MIN_ROOT_HEIGHT_M,
            nominal[2] - COMMAND_MAX_ROOT_DOWN_DELTA_M,
        )
        maximum_height = nominal[2] + COMMAND_MAX_ROOT_UP_DELTA_M
        target_root[2] = float(np.clip(target_root[2], minimum_height, maximum_height))
        clamped = not np.allclose(target_root, requested_root, atol=1.0e-12, rtol=0.0)
        wheel_targets = np.asarray(
            [
                self.data.xpos[body_id]
                for body_id in self.command_wheel_center_bodies
            ],
            dtype=np.float64,
        )
        target_joint = self._solve_command_joint_target(
            root_position,
            root_quaternion,
            q,
            target_root,
            wheel_targets,
        )
        if target_joint is None:
            self._reject_target_command(
                command,
                "target rejected because the measured wheel-anchor IK was infeasible",
            )
            return
        self.command_request_id = request_id
        self.command_frame = str(frame)
        self.command_reason = "target clamped to bounded squat envelope" if clamped else "target admitted"
        self.command_requested_root_position = requested_root.copy()
        self.command_target_root_position = target_root.copy()
        self.command_start_root_position = root_position.copy()
        self.command_elapsed_s = 0.0
        self.command_duration_s = float(duration_ms) / 1000.0
        self.command_plan = {
            "start": root_position.copy(),
            "target": target_root.copy(),
            "start_joint": q.copy(),
            "target_joint": target_joint,
            "duration_s": self.command_duration_s,
        }
        self.command_phase = "executing"

    def _sample_target_command(
        self, root_position: np.ndarray
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        float,
    ]:
        if self.command_plan is None or self.command_target_root_position is None:
            return (
                self.command_nominal_root_position.copy(),
                np.zeros(3, np.float64),
                np.zeros(3, np.float64),
                self.command_nominal_joint_position.copy(),
                np.zeros(6, np.float64),
                np.zeros(6, np.float64),
                0.0,
            )
        plan = self.command_plan
        if self.command_phase == "executing":
            desired, velocity, acceleration, progress = quintic_profile(
                plan["start"],
                plan["target"],
                float(plan["duration_s"]),
                self.command_elapsed_s,
            )
            joint, joint_velocity, joint_acceleration, _ = quintic_profile(
                np.asarray(plan["start_joint"], dtype=np.float64),
                np.asarray(plan["target_joint"], dtype=np.float64),
                float(plan["duration_s"]),
                self.command_elapsed_s,
            )
            return (
                desired,
                velocity,
                acceleration,
                joint,
                joint_velocity,
                joint_acceleration,
                progress,
            )
        return (
            np.asarray(plan["target"], dtype=np.float64).copy(),
            np.zeros(3, np.float64),
            np.zeros(3, np.float64),
            np.asarray(plan["target_joint"], dtype=np.float64).copy(),
            np.zeros(6, np.float64),
            np.zeros(6, np.float64),
            1.0,
        )

    def _apply_target_command(
        self,
        root_position: np.ndarray,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        float,
    ]:
        (
            desired,
            velocity,
            acceleration,
            joint,
            joint_velocity,
            joint_acceleration,
            progress,
        ) = self._sample_target_command(root_position)
        self.controller.nominal_root_position[:] = desired
        self.controller.nominal_joint_position[:] = joint
        return (
            desired,
            velocity,
            acceleration,
            joint,
            joint_velocity,
            joint_acceleration,
            progress,
        )

    def _advance_target_command(self) -> None:
        if self.command_phase != "executing" or self.command_plan is None:
            return
        self.command_elapsed_s = min(
            self.command_elapsed_s + self.control_dt,
            float(self.command_plan["duration_s"]),
        )
        if self.command_elapsed_s >= float(self.command_plan["duration_s"]) - 1.0e-12:
            self.command_phase = "holding"
            self.command_reason = "bounded target reached; holding measured endpoint"

    def _target_telemetry(
        self, root_position: np.ndarray
    ) -> dict[str, Any]:
        target = (
            self.command_target_root_position
            if self.command_target_root_position is not None
            else self.command_nominal_root_position
        )
        requested = (
            self.command_requested_root_position
            if self.command_requested_root_position is not None
            else target
        )
        error = float(np.linalg.norm(target - root_position))
        progress = (
            0.0
            if self.command_plan is None
            else float(
                np.clip(
                    self.command_elapsed_s / max(self.command_duration_s, 1.0e-9),
                    0.0,
                    1.0,
                )
            )
        )
        return {
            "phase": self.command_phase,
            "request_id": self.command_request_id,
            "frame": self.command_frame,
            "reason": self.command_reason,
            "progress": progress,
            "duration_s": self.command_duration_s,
            "requested_root_position": requested.tolist(),
            "target_root_position": target.tolist(),
            "start_root_position": (
                self.command_start_root_position.tolist()
                if self.command_start_root_position is not None
                else None
            ),
            "position_error_m": error,
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
        if isinstance(target_command, dict) and bool(
            target_command.get("active", True)
        ):
            target_request_id = target_command.get("request_id")
            if target_request_id != self.command_request_id:
                root_position, root_quaternion, _, q, _ = plant.read_state(
                    self.model, self.data
                )
                if self.paused:
                    self._reject_target_command(
                        target_command,
                        "target commit is disabled while MuJoCo is paused",
                    )
                else:
                    self._accept_target_command(
                        target_command, root_position, root_quaternion, q
                    )

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
            (
                command_root_position,
                command_root_velocity,
                command_root_acceleration,
                command_joint_position,
                command_joint_velocity,
                command_joint_acceleration,
                _command_progress,
            ) = self._apply_target_command(root_position)
            if self.last_external_wrench_valid:
                # Re-express the completed force at the current solve
                # boundary.  The core floating dynamics rows use the root
                # body origin; the optional centroidal task uses the current
                # aggregate CoM.  Storing point+force avoids using a stale
                # reference point after the plant moves during the 20 ms
                # delay.
                current_root_origin = np.asarray(
                    self.data.xpos[self.last_external_root_body_id],
                    dtype=np.float64,
                )
                self.last_external_wrench_world[:3] = np.cross(
                    self.last_external_application_point_world
                    - current_root_origin,
                    self.last_external_force_world,
                )
                self.last_external_wrench_world[3:] = self.last_external_force_world
                self.last_external_centroidal_moment_world[...] = np.cross(
                    self.last_external_application_point_world
                    - np.asarray(self.data.subtree_com[0], dtype=np.float64),
                    self.last_external_force_world,
                )
                np.copyto(
                    observed_external_wrench_world,
                    self.last_external_wrench_world,
                )
                np.copyto(
                    observed_external_centroidal_moment_world,
                    self.last_external_centroidal_moment_world,
                )
                observed_external_wrench_valid = True
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
                    self.last_external_wrench_world
                    if self.last_external_wrench_valid
                    else None
                ),
                observed_external_centroidal_moment_world=(
                    self.last_external_centroidal_moment_world
                    if self.last_external_wrench_valid
                    else None
                ),
                observed_contact_available=True,
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
                application_offset = point - self.data.xipos[body_id]
                application_offset_m = float(np.linalg.norm(application_offset))
                applied_moment_world = np.cross(application_offset, force)
                # Centroidal angular momentum is about the aggregate system
                # CoM, not the MuJoCo body inertial origin.  Keep this as a
                # separate observation so existing body-COM telemetry remains
                # compatible while Rust consumes the physically relevant r×F.
                applied_centroidal_moment_world = np.cross(
                    point - np.asarray(self.data.subtree_com[0], dtype=np.float64),
                    force,
                )
                root_body_id = int(self.model.body_rootid[body_id])
                applied_root_moment_world = np.cross(
                    point - np.asarray(self.data.xpos[root_body_id], dtype=np.float64),
                    force,
                )
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
            self._advance_target_command()

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
        target_telemetry = self._target_telemetry(root_position)
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
                "command_target_root_position": target_telemetry[
                    "target_root_position"
                ],
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
