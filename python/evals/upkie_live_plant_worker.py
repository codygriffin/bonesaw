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


def finite_vector(value: object, length: int) -> np.ndarray | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    result = np.asarray(value, np.float64)
    return result if np.all(np.isfinite(result)) else None


class LiveUpkiePlant:
    def __init__(self, model_path: pathlib.Path):
        self.model_path = model_path
        self.reset_epoch = 0
        self.tick = 0
        self.numeric_resets = 0
        self.fall_resets = 0
        self.pending_automatic_reset: str | None = None
        self._build()

    def _build(self) -> None:
        import bonesaw

        self.model, self.data = plant.make_plant(
            self.model_path, physics_dt=PHYSICS_DT
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
        self.controller = plant.RustWbcAdapter(
            self.model_path,
            nominal_root,
            self.target_ground_position,
            balance,
            "capture",
            0.2,
            fall_safe_enabled=True,
            fall_safe_primary_blend=False,
            control_dt=CONTROL_DT,
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
        self.body_by_name = {
            name: body
            for body in range(1, self.model.nbody)
            if (name := mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body
            ))
            is not None
        }
        self.ground_geom = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground"
        )
        self.zero_torque = np.zeros(3, np.float64)
        self.last_result: dict[str, Any] | None = None

    def reset(self, *, numeric: bool = False, fall: bool = False) -> None:
        self.reset_epoch += 1
        if numeric:
            self.numeric_resets += 1
        if fall:
            self.fall_resets += 1
        self._build()

    def hello(self) -> dict[str, Any]:
        return {
            "type": "plant_hello",
            "protocol": 1,
            "model": "upkie",
            "stream_hz": int(round(1.0 / STREAM_DT)),
            "control_hz": int(round(1.0 / CONTROL_DT)),
            "physics_hz": int(round(1.0 / PHYSICS_DT)),
            "physics_substeps_per_control": PHYSICS_STEPS_PER_CONTROL,
            "maximum_force_n": MAX_FORCE_N,
            "maximum_application_offset_m": MAX_APPLICATION_OFFSET_M,
            "command_ttl_ms": 140,
            "automatic_reset": {
                "fall_height_m": FALL_HEIGHT_M,
                "fall_tilt_rad": FALL_TILT_RAD,
                "policy": "report_fall_then_reset_on_next_stream_step",
            },
            "body_names": sorted(self.body_by_name),
            "boundary": "python_mujoco_plant__rust_capture_wbc",
            "simulator": {
                "backend": "MuJoCo",
                "version": mujoco.__version__,
                "integrator": "implicitfast",
                "ground_plane_z_m": 0.0,
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

    def step(self, request: dict[str, Any]) -> dict[str, Any]:
        requested_reset = bool(request.get("reset", False))
        automatic_reset_reason = (
            None if requested_reset else self.pending_automatic_reset
        )
        self.pending_automatic_reset = None
        if automatic_reset_reason == "fall":
            self.reset(fall=True)
        if requested_reset:
            self.reset()
        command = request.get("push")
        active = isinstance(command, dict) and bool(command.get("active", False))
        request_id = command.get("request_id") if active else None
        body_name = str(command.get("body", "base")) if active else "base"
        force = finite_vector(command.get("force_world"), 3) if active else None
        point = (
            finite_vector(command.get("application_point_world"), 3)
            if active
            else None
        )
        if active and (force is None or point is None):
            return {
                "type": "plant_error",
                "message": "active push requires finite force_world[3] and application_point_world[3]",
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
        latest_result: dict[str, Any] | None = None
        applied_moment_world = np.zeros(3, np.float64)
        application_offset_m = 0.0
        maximum_moment_nm = 0.0
        for _ in range(CONTROL_TICKS_PER_STREAM):
            root_position, root_quaternion, root_twist, q, v = plant.read_state(
                self.model, self.data
            )
            ground_position = float(np.mean(self.data.xpos[self.wheel_bodies, 0]))
            ground_height = float(np.mean(self.data.xpos[self.wheel_bodies, 2]))
            result = self.controller.solve(
                root_position,
                root_quaternion,
                root_twist,
                q,
                v,
                ground_position,
                ground_height,
            )
            latest_result = result
            self.data.ctrl[self.actuator_ids] = result["torque"]
            self.data.qfrc_applied.fill(0.0)
            self.data.xfrc_applied.fill(0.0)
            if active:
                application_offset = point - self.data.xipos[body_id]
                application_offset_m = float(np.linalg.norm(application_offset))
                applied_moment_world = np.cross(application_offset, force)
                maximum_moment_nm = max(
                    maximum_moment_nm, float(np.linalg.norm(applied_moment_world))
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
            for _ in range(PHYSICS_STEPS_PER_CONTROL):
                mujoco.mj_step(self.model, self.data)
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

        numeric_reset = False
        if not self._validate_state():
            self.reset(numeric=True)
            numeric_reset = True
            latest_result = None
        mujoco.mj_forward(self.model, self.data)
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
        ground_contacts = [contact for contact in contacts if contact["ground"]]
        minimum_contact_distance_m = min(
            (contact["distance_m"] for contact in contacts), default=0.0
        )
        solver_niter = np.asarray(getattr(self.data, "solver_niter", [0]))
        response = {
            "type": "plant_state",
            "tick": self.tick,
            "reset_epoch": self.reset_epoch,
            "numeric_reset": numeric_reset,
            "automatic_reset_reason": automatic_reset_reason,
            "automatic_reset_pending": self.pending_automatic_reset,
            "command_id": request.get("command_id"),
            "command_expired": bool(request.get("command_expired", False)),
            "frames": self._frames(),
            "root_position": root_position.tolist(),
            "root_quaternion_wxyz": root_quaternion.tolist(),
            "root_twist_world": root_twist.tolist(),
            "joint_positions": q.tolist(),
            "joint_velocities": v.tolist(),
            "contacts": contacts,
            "simulator": {
                "backend": "MuJoCo",
                "time_s": float(self.data.time),
                "physics_dt_s": PHYSICS_DT,
                "control_dt_s": CONTROL_DT,
                "physics_substeps": PHYSICS_STEPS_PER_CONTROL,
                "solver_iterations": int(np.max(solver_niter)),
                "ground_plane_z_m": 0.0,
            },
            "push": {
                "active": active,
                "request_id": request_id,
                "body": body_name if active else None,
                "force_world": force.tolist() if active else [0.0, 0.0, 0.0],
                "application_point_world": point.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "moment_world_nm": applied_moment_world.tolist()
                if active
                else [0.0, 0.0, 0.0],
                "application_offset_m": application_offset_m if active else 0.0,
                "maximum_moment_nm": maximum_moment_nm if active else 0.0,
            },
            "metrics": {
                "wbc_status": "unavailable"
                if latest_result is None
                else plant.STATUS_NAMES[int(latest_result["status"])],
                "wbc_admitted": latest_result is not None
                and int(latest_result["status"]) in (0, 1),
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
                "minimum_contact_distance_m": minimum_contact_distance_m,
                "maximum_penetration_m": max(-minimum_contact_distance_m, 0.0),
                "numeric_resets": self.numeric_resets,
                "fall_resets": self.fall_resets,
                "fallen": fallen,
            },
        }
        return response


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=pathlib.Path)
    return parser.parse_args()


def emit(value: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(value, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def main() -> int:
    args = parse_args()
    live = LiveUpkiePlant(args.model.resolve())
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
