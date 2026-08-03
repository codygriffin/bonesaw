#!/usr/bin/env python3
"""Closed-loop Upkie plant differential with MuJoCo outside Bonesaw.

Python owns model adaptation, contact response, integration, external wrench,
experiment sequencing, statistics, and artifacts. Rust receives each observed
floating state independently and returns bounded inverse-dynamics WBC effort.
"""

from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import json
import math
import pathlib
import platform
import resource
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "upkie-mujoco-plant-r128"
STATUS_NAMES = {
    0: "Solved",
    1: "SolvedWithSlack",
    2: "PrimalInfeasible",
    3: "NumericalOrInvalid",
    4: "MaxIterations",
}
JOINT_ORDER = (
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
)
CONTACT_FRAMES = ("left_wheel_center", "right_wheel_center")
ROLLING_COORDINATES = np.asarray([2, 5], dtype=np.int64)
ROLLING_COEFFICIENTS = np.asarray([-0.05, 0.05], dtype=np.float64)
CONTROL_DT = 0.005
PHYSICS_DT = 0.001
WHEEL_RADIUS_M = 0.05
INITIAL_CONTACT_PENETRATION_M = 1.0e-6
STARTUP_TRANSIENT_S = 0.025
# Declared Upkie root-origin height at which the upper body collision envelope
# first reaches the ground in the terminal-consequence audit. This is model
# adapter evidence, not inferred by the generic Rust objective.
UPKIE_ROOT_IMPACT_PLANE_M = 0.225


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--revision", default=REVISION)
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    parser.add_argument("--duration", type=float, default=4.5)
    parser.add_argument("--push-start", type=float, default=1.0)
    parser.add_argument("--push-duration", type=float, default=0.1)
    parser.add_argument("--push-force", type=float, default=4.0)
    parser.add_argument("--overload-force", type=float, default=6.0)
    parser.add_argument(
        "--balance-mode",
        choices=(
            "reference",
            "capture",
            "planar_capture",
            "viability_capture",
            "viability_verified",
            "viability_support_capture",
            "viability_coordinate",
        ),
        default="reference",
        help="exact upstream-style PI reference or rooted velocity-dependent capture composition",
    )
    parser.add_argument(
        "--capture-velocity-fraction",
        type=float,
        default=0.2,
        help="fraction of the DCM velocity offset presented to the reference PI loop",
    )
    parser.add_argument(
        "--contact-model",
        choices=("soft", "prescribed"),
        default="soft",
        help="MuJoCo soft collision or Bonesaw's solved rigid-contact wrench applied to MuJoCo",
    )
    parser.add_argument(
        "--output", default=f"benchmarks/results/{REVISION}"
    )
    parser.add_argument("--web-report", default="web/UPKIE_MUJOCO_PLANT_R128.html")
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def rss_bytes() -> int:
    for line in pathlib.Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) * 1024
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024


def effort_limits(model_path: pathlib.Path) -> dict[str, float]:
    root = ET.parse(model_path).getroot()
    limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("effort") is not None:
            limits[joint.get("name", "")] = float(limit.get("effort", "nan"))
    return {name: limits[name] for name in JOINT_ORDER}


def standing_posture() -> np.ndarray:
    return np.asarray([0.4, -0.625, 0.0, -0.4, 0.625, 0.0], np.float64)


def make_plant(
    model_path: pathlib.Path,
    *,
    sliding_friction: float = 1.0,
    physics_dt: float = PHYSICS_DT,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    if not math.isfinite(sliding_friction) or sliding_friction <= 0.0:
        raise ValueError("sliding_friction must be finite and positive")
    if not math.isfinite(physics_dt) or physics_dt <= 0.0:
        raise ValueError("physics_dt must be finite and positive")
    xml = model_path.read_text().replace(
        "package://upkie_description/", f"{model_path.parent.resolve()}/"
    )
    # Give the URDF an explicit floating parent before MuJoCo parses it. Adding
    # joints to the already-parsed root is too late: the URDF importer has
    # already fused fixed bodies into the world and their inertias cannot be
    # recovered by a later MjSpec edit.
    xml = xml.replace(
        '<robot name="upkie">',
        '<robot name="upkie">'
        '<link name="world"/>'
        '<joint name="root" type="floating">'
        '<parent link="world"/><child link="base"/>'
        '</joint>',
        1,
    )
    spec = mujoco.MjSpec.from_string(xml)
    # MuJoCo enables static-body fusion for URDF by default. Upkie has long
    # fixed-link chains with rotated inertial frames; retaining them gives the
    # same CoM, bias, and sagittal mass matrix as the independent Pinocchio
    # oracle, while fusion produces a visibly asymmetric nominal model.
    spec.compiler.fusestatic = False
    spec.option.timestep = physics_dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0.0, 0.0, 0.05],
        rgba=[0.18, 0.20, 0.23, 1.0],
        friction=[sliding_friction, 0.01, 0.001],
        solref=[0.008, 1.0],
        solimp=[0.95, 0.99, 0.001, 0.5, 2.0],
    )
    limits = effort_limits(model_path)
    for name in JOINT_ORDER:
        actuator = spec.add_actuator(name=f"{name}_motor")
        actuator.set_to_motor()
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.target = name
        actuator.ctrllimited = True
        actuator.ctrlrange = [-limits[name], limits[name]]
        actuator.forcelimited = True
        actuator.forcerange = [-limits[name], limits[name]]
    model = spec.compile()
    model.geom_solref[:, 0] = 0.008
    model.geom_solref[:, 1] = 1.0
    model.geom_solimp[:] = [0.99, 0.999, 0.0001, 0.5, 2.0]
    model.geom_friction[:] = [sliding_friction, 0.01, 0.001]
    data = mujoco.MjData(model)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    root_qpos = model.jnt_qposadr[root]
    data.qpos[root_qpos : root_qpos + 7] = [0.0, 0.0, 0.539, 1.0, 0.0, 0.0, 0.0]
    for name, value in zip(JOINT_ORDER, standing_posture(), strict=True):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)
    wheel_centers = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, frame)
        for frame in CONTACT_FRAMES
    ]
    root_height_correction = (
        WHEEL_RADIUS_M
        - INITIAL_CONTACT_PENETRATION_M
        - min(float(data.xpos[body, 2]) for body in wheel_centers)
    )
    data.qpos[root_qpos + 2] += root_height_correction
    mujoco.mj_forward(model, data)
    return model, data


def allocate_outputs(session: Any, ticks: int = 1) -> dict[str, np.ndarray]:
    if ticks < 1:
        raise ValueError("output batch must contain at least one tick")
    dof = session.dof
    generalized = dof + 6
    contacts = 2
    diagnostics = session.task_diagnostic_capacity
    priorities = 5
    return {
        "generalized_acceleration": np.empty((ticks, generalized), np.float64),
        "actuator_torque": np.empty((ticks, dof), np.float64),
        "contact_normal_force": np.empty((ticks, contacts), np.float64),
        "contact_force_basis": np.empty((ticks, contacts, 3), np.float64),
        "task_rms": np.empty((ticks, diagnostics), np.float64),
        "task_clipped": np.empty((ticks, diagnostics), np.uint8),
        "dynamics_residual": np.empty(ticks, np.float64),
        "contact_residual": np.empty(ticks, np.float64),
        "minimum_friction_margin": np.empty(ticks, np.float64),
        "minimum_support_margin": np.empty(ticks, np.float64),
        "minimum_torque_margin": np.empty(ticks, np.float64),
        "maximum_constraint_violation": np.empty(ticks, np.float64),
        "minimum_bound_margin": np.empty(ticks, np.float64),
        "minimum_joint_margin_rad": np.empty(ticks, np.float64),
        "minimum_joint_headroom_fraction": np.empty(ticks, np.float64),
        "limiting_joint": np.empty(ticks, np.uint16),
        "maximum_torque_utilization": np.empty(ticks, np.float64),
        "minimum_torque_headroom": np.empty(ticks, np.float64),
        "limiting_actuator": np.empty(ticks, np.uint16),
        "witness_acceleration_rms": np.empty(ticks, np.float64),
        "step_ns": np.empty(ticks, np.uint64),
        "status": np.empty(ticks, np.uint8),
        "task_pseudoinverse_calls": np.empty(ticks, np.uint16),
        "task_pseudoinverse_calls_by_priority": np.empty(
            (ticks, priorities), np.uint16
        ),
        "clipped_steps": np.empty(ticks, np.uint16),
        "clipped_steps_by_priority": np.empty((ticks, priorities), np.uint16),
        "task_jacobi_sweeps": np.empty(ticks, np.uint16),
        "task_jacobi_sweeps_by_priority": np.empty(
            (ticks, priorities), np.uint16
        ),
        "feasibility_projection_sweeps": np.empty(ticks, np.uint16),
        "feasibility_halfspace_projections": np.empty(ticks, np.uint32),
        "feasibility_polish_iterations": np.empty(ticks, np.uint16),
        "feasibility_seed_reused": np.empty(ticks, np.uint8),
        "feasibility_prefix_resumed": np.empty(ticks, np.uint8),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
    }


def quaternion_rotation_vector(quaternion_wxyz: np.ndarray) -> np.ndarray:
    quaternion = quaternion_wxyz / np.linalg.norm(quaternion_wxyz)
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    vector_norm = float(np.linalg.norm(quaternion[1:]))
    if vector_norm < 1.0e-12:
        return 2.0 * quaternion[1:]
    angle = 2.0 * math.atan2(vector_norm, float(quaternion[0]))
    return quaternion[1:] * (angle / vector_norm)


def wheel_contact_body_sets(
    model: mujoco.MjModel, wheel_roots: np.ndarray
) -> tuple[frozenset[int], ...]:
    """Resolve each wheel subtree once; contact identity is plant topology."""
    sets: list[frozenset[int]] = []
    for wheel_root in wheel_roots:
        members: set[int] = set()
        for body in range(model.nbody):
            cursor = body
            while cursor != 0 and cursor != int(wheel_root):
                cursor = int(model.body_parentid[cursor])
            if cursor == int(wheel_root):
                members.add(body)
        sets.append(frozenset(members))
    return tuple(sets)


def measured_wheel_ground_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_sets: tuple[frozenset[int], ...],
) -> np.ndarray:
    """Return exact MuJoCo wheel-to-world contact activity for this observation."""
    active = np.empty(len(body_sets), np.uint8)
    return measured_wheel_ground_contacts_into(model, data, body_sets, active)


def measured_wheel_ground_contacts_into(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_sets: tuple[frozenset[int], ...],
    active: np.ndarray,
) -> np.ndarray:
    """Write exact wheel-to-ground contact activity into caller-owned storage."""
    if active.shape != (len(body_sets),) or active.dtype != np.uint8:
        raise ValueError("active must be a uint8 vector matching body_sets")
    active.fill(0)
    for contact in data.contact[: data.ncon]:
        body_a = int(model.geom_bodyid[int(contact.geom[0])])
        body_b = int(model.geom_bodyid[int(contact.geom[1])])
        for wheel, members in enumerate(body_sets):
            if (body_a == 0 and body_b in members) or (
                body_b == 0 and body_a in members
            ):
                active[wheel] = 1
    return active


def measured_wheel_ground_normal_forces_into(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_sets: tuple[frozenset[int], ...],
    normal_force_n: np.ndarray,
    wrench_scratch: np.ndarray,
) -> np.ndarray:
    """Write per-wheel MuJoCo ground-normal load without allocating.

    The result is a plant measurement, not the WBC's contact-force decision.
    It therefore remains useful before the binary contact mask changes and
    makes loss-of-load a causal observer for support-preservation experiments.
    """
    if normal_force_n.shape != (len(body_sets),) or normal_force_n.dtype != np.float64:
        raise ValueError("normal_force_n must be float64 and match body_sets")
    if wrench_scratch.shape != (6,) or wrench_scratch.dtype != np.float64:
        raise ValueError("wrench_scratch must be a float64 vector of length six")
    normal_force_n.fill(0.0)
    for index, contact in enumerate(data.contact[: data.ncon]):
        body_a = int(model.geom_bodyid[int(contact.geom[0])])
        body_b = int(model.geom_bodyid[int(contact.geom[1])])
        if body_a != 0 and body_b != 0:
            continue
        for wheel, members in enumerate(body_sets):
            if (body_a == 0 and body_b in members) or (
                body_b == 0 and body_a in members
            ):
                mujoco.mj_contactForce(model, data, index, wrench_scratch)
                normal_force_n[wheel] += max(float(wrench_scratch[0]), 0.0)
                break
    return normal_force_n


class PythonViabilityCoordinatePlanner:
    """Python-owned bounded search over allocation-free exact Rust WBC queries.

    This is evaluation orchestration, not a controller hidden in the plant.
    Requests are ordered `[roll, lateral, yaw]` and are deltas on the existing
    capture objectives. The Rust supervisor below owns whether any candidate
    is fresh and executable; the final command is always solved again.
    """

    HORIZON_S = 0.25
    ROLL_BOUND_RAD = math.radians(45.0)
    LATERAL_CAPTURE_BOUND_M = 0.10
    ACTIVATION_PRESSURE = 0.10
    RELEASE_PRESSURE = 0.05
    AXIS_ORDER = (2, 0, 1)
    VALUES = (
        np.asarray([-250.0, -100.0, -40.0, 0.0, 40.0, 100.0, 250.0], np.float64),
        np.asarray([-250.0, -100.0, -40.0, 0.0, 40.0, 100.0, 250.0], np.float64),
        np.asarray([-80.0, -20.0, 0.0, 20.0, 80.0], np.float64),
    )

    def __init__(
        self,
        model_path: pathlib.Path,
        friction_coefficient: float,
        root_angular_task_weight: float,
        nominal_root_position: np.ndarray,
        activation_pressure: float = 0.10,
        release_pressure: float = 0.05,
        maximum_feasibility_iterations: int = 64,
        maximum_feasibility_projection_sweeps: int | None = None,
        feasibility_projection_continuation_violation_threshold: float | None = None,
        repair_feasibility_equalities_before_inequalities: bool = False,
        use_feasibility_row_spans: bool = False,
        reuse_identical_hard_feasibility_seed: bool = False,
    ) -> None:
        import bonesaw

        if (
            not math.isfinite(activation_pressure)
            or not math.isfinite(release_pressure)
            or release_pressure < 0.0
            or release_pressure >= activation_pressure
        ):
            raise ValueError("planner pressure thresholds must be finite and ordered")
        self.ACTIVATION_PRESSURE = activation_pressure
        self.RELEASE_PRESSURE = release_pressure

        self.session = bonesaw.FloatingWbcSession(
            str(model_path),
            maximum_contacts=2,
            friction_coefficient=friction_coefficient,
            maximum_acceleration=250.0,
            maximum_torque=2000.0,
            maximum_normal_force_multiple=3.0,
            maximum_feasibility_iterations=maximum_feasibility_iterations,
            maximum_feasibility_projection_sweeps=maximum_feasibility_projection_sweeps,
            feasibility_projection_continuation_violation_threshold=feasibility_projection_continuation_violation_threshold,
            repair_feasibility_equalities_before_inequalities=repair_feasibility_equalities_before_inequalities,
            use_feasibility_row_spans=use_feasibility_row_spans,
            reuse_identical_hard_feasibility_seed=reuse_identical_hard_feasibility_seed,
            joint_limit_braking=True,
            root_angular_task_weight=root_angular_task_weight,
            root_height_task_weight=10.0,
            root_horizontal_task_weight=10.0,
            root_horizontal_task_priority=1,
            joint_posture_weight=1.0,
            joint_posture_priority=1,
            center_of_mass_task_weight=0.0,
        )
        frames = list(self.session.frame_names)
        self.frame_ids = np.asarray(
            [frames.index(name) for name in CONTACT_FRAMES], np.int64
        )
        self.priorities = np.ones(2, np.uint8)
        self.weights = np.ones(2, np.float64)
        self.contact_modes = np.full(2, 3, np.uint8)
        self.rolling_gains = np.full(2, 2.0, np.float64)
        self.rolling_corrections = np.full(2, 3.0, np.float64)
        self.batches = {
            count: self._make_batch(count) for count in (1, 3, 5, 7)
        }
        self.request = np.zeros(3, np.float64)
        self.scores = np.empty(7, np.float64)
        self.rotation = np.empty(3, np.float64)
        self.zero_achieved = np.zeros(6, np.float64)
        self.query_count = 0
        self.step_ns = 0
        self.allocation_calls = 0
        self.allocated_bytes = 0
        self.feasibility_seed_reuses = 0
        self.feasibility_projection_sweeps = 0
        self.feasibility_halfspace_projections = 0
        self.zero_pressure = 0.0
        self.candidate_pressure = 0.0
        self.nominal_lateral = float(nominal_root_position[1])

    def _make_batch(self, ticks: int) -> dict[str, Any]:
        return {
            "request": np.zeros((ticks, 3), np.float64),
            "root_position": np.empty((ticks, 3), np.float64),
            "root_velocity": np.empty((ticks, 3), np.float64),
            "root_acceleration": np.empty((ticks, 3), np.float64),
            "root_quaternion": np.empty((ticks, 4), np.float64),
            "root_angular_velocity": np.empty((ticks, 3), np.float64),
            "root_angular_acceleration": np.empty((ticks, 3), np.float64),
            "q": np.empty((ticks, 6), np.float64),
            "v": np.empty((ticks, 6), np.float64),
            "joint_acceleration": np.empty((ticks, 6), np.float64),
            "contact_active": np.empty((ticks, 2), np.uint8),
            "contact_bases_world": np.empty((ticks, 2, 3, 3), np.float64),
            "out": allocate_outputs(self.session, ticks),
        }

    def _query(
        self,
        batch: dict[str, Any],
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        base_root_acceleration: np.ndarray,
        base_root_angular_acceleration: np.ndarray,
        joint_acceleration: np.ndarray,
        contact_active: np.ndarray,
        contact_bases_world: np.ndarray,
    ) -> dict[str, np.ndarray]:
        request = batch["request"]
        batch["root_position"][:] = root_position
        batch["root_velocity"][:] = root_twist[3:]
        batch["root_acceleration"][:] = base_root_acceleration
        batch["root_quaternion"][:] = root_quaternion
        batch["root_angular_velocity"][:] = root_twist[:3]
        batch["root_angular_acceleration"][:] = base_root_angular_acceleration
        batch["q"][:] = q
        batch["v"][:] = v
        batch["joint_acceleration"][:] = joint_acceleration
        batch["contact_active"][:] = contact_active
        batch["contact_bases_world"][:] = contact_bases_world
        batch["root_angular_acceleration"][:, 0] += request[:, 0]
        batch["root_acceleration"][:, 1] += request[:, 1]
        batch["root_angular_acceleration"][:, 2] += request[:, 2]
        out = batch["out"]
        self.session.run_oracle_trace(
            batch["root_position"],
            batch["root_velocity"],
            batch["root_acceleration"],
            batch["q"],
            batch["v"],
            batch["joint_acceleration"],
            self.frame_ids,
            batch["contact_active"],
            self.priorities,
            self.weights,
            0,
            0,
            out["generalized_acceleration"],
            out["actuator_torque"],
            out["contact_normal_force"],
            out["task_rms"],
            out["task_clipped"],
            out["dynamics_residual"],
            out["contact_residual"],
            out["minimum_friction_margin"],
            out["minimum_support_margin"],
            out["minimum_torque_margin"],
            out["maximum_constraint_violation"],
            out["minimum_bound_margin"],
            out["minimum_joint_margin_rad"],
            out["minimum_joint_headroom_fraction"],
            out["limiting_joint"],
            out["maximum_torque_utilization"],
            out["minimum_torque_headroom"],
            out["limiting_actuator"],
            out["witness_acceleration_rms"],
            out["step_ns"],
            out["status"],
            out["task_pseudoinverse_calls"],
            out["task_pseudoinverse_calls_by_priority"],
            out["clipped_steps"],
            out["clipped_steps_by_priority"],
            out["task_jacobi_sweeps"],
            out["task_jacobi_sweeps_by_priority"],
            out["feasibility_projection_sweeps"],
            out["feasibility_halfspace_projections"],
            out["feasibility_polish_iterations"],
            out["allocation_calls"],
            out["allocated_bytes"],
            contact_force_basis_out=out["contact_force_basis"],
            contact_modes=self.contact_modes,
            rolling_coordinates=ROLLING_COORDINATES,
            rolling_velocity_coefficients=ROLLING_COEFFICIENTS,
            rolling_velocity_stabilization_gains=self.rolling_gains,
            rolling_maximum_stabilization_accelerations=self.rolling_corrections,
            root_quaternions_wxyz=batch["root_quaternion"],
            root_angular_velocities_world=batch["root_angular_velocity"],
            root_angular_accelerations_world=batch["root_angular_acceleration"],
            contact_bases_world=batch["contact_bases_world"],
            feasibility_seed_reused_out=out["feasibility_seed_reused"],
            feasibility_prefix_resumed_out=out["feasibility_prefix_resumed"],
        )
        self.query_count += len(request)
        self.step_ns += int(np.sum(out["step_ns"]))
        self.allocation_calls += int(np.sum(out["allocation_calls"]))
        self.allocated_bytes += int(np.sum(out["allocated_bytes"]))
        self.feasibility_seed_reuses += int(np.sum(out["feasibility_seed_reused"]))
        self.feasibility_projection_sweeps += int(
            np.sum(out["feasibility_projection_sweeps"], dtype=np.uint64)
        )
        self.feasibility_halfspace_projections += int(
            np.sum(out["feasibility_halfspace_projections"], dtype=np.uint64)
        )
        return out

    def _pressure(self, achieved: np.ndarray) -> float:
        height = max(self.state_height, 0.05)
        omega = math.sqrt(9.81 / height)
        roll_acceleration = float(achieved[0])
        lateral_acceleration = float(achieved[4])
        predicted_roll = (
            self.state_roll
            + self.HORIZON_S * self.state_roll_rate
            + 0.5 * self.HORIZON_S**2 * roll_acceleration
        )
        predicted_roll_rate = self.state_roll_rate + self.HORIZON_S * roll_acceleration
        predicted_lateral = (
            self.state_lateral
            + self.HORIZON_S * self.state_lateral_rate
            + 0.5 * self.HORIZON_S**2 * lateral_acceleration
        )
        predicted_lateral_rate = (
            self.state_lateral_rate + self.HORIZON_S * lateral_acceleration
        )
        roll_capture = predicted_roll + predicted_roll_rate / omega
        lateral_capture = predicted_lateral + predicted_lateral_rate / omega
        return max(
            abs(roll_capture) / self.ROLL_BOUND_RAD,
            abs(lateral_capture) / self.LATERAL_CAPTURE_BOUND_M,
        )

    @staticmethod
    def _admitted(out: dict[str, np.ndarray], row: int) -> bool:
        return bool(
            int(out["status"][row]) in (0, 1)
            and float(out["maximum_constraint_violation"][row]) <= 1.0e-8
        )

    def plan(
        self,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        base_root_acceleration: np.ndarray,
        base_root_angular_acceleration: np.ndarray,
        joint_acceleration: np.ndarray,
        contact_active: np.ndarray,
        contact_bases_world: np.ndarray,
        was_active: bool,
        previous_request: np.ndarray | None = None,
    ) -> tuple[np.ndarray, float, bool]:
        self.query_count = 0
        self.step_ns = 0
        self.allocation_calls = 0
        self.allocated_bytes = 0
        self.feasibility_seed_reuses = 0
        self.feasibility_projection_sweeps = 0
        self.feasibility_halfspace_projections = 0
        self.request.fill(0.0)
        self.rotation[:] = quaternion_rotation_vector(root_quaternion)
        self.state_roll = float(self.rotation[0])
        self.state_roll_rate = float(root_twist[0])
        self.state_lateral = float(root_position[1]) - self.nominal_lateral
        self.state_lateral_rate = float(root_twist[4])
        self.state_height = float(root_position[2])

        one = self.batches[1]
        one["request"].fill(0.0)
        zero = self._query(
            one,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        if not self._admitted(zero, 0):
            self.zero_pressure = self._pressure(self.zero_achieved)
            self.candidate_pressure = self.zero_pressure
            return self.request, self.zero_pressure, False
        self.zero_pressure = self._pressure(zero["generalized_acceleration"][0, :6])
        self.candidate_pressure = self.zero_pressure
        should_search = self.zero_pressure > self.RELEASE_PRESSURE and (
            was_active or self.zero_pressure >= self.ACTIVATION_PRESSURE
        )
        if not should_search:
            return self.request, self.zero_pressure, False

        for _ in range(2):
            for axis in self.AXIS_ORDER:
                values = self.VALUES[axis]
                batch = self.batches[len(values)]
                batch["request"][:] = self.request
                batch["request"][:, axis] = values
                out = self._query(
                    batch,
                    root_position,
                    root_quaternion,
                    root_twist,
                    q,
                    v,
                    base_root_acceleration,
                    base_root_angular_acceleration,
                    joint_acceleration,
                    contact_active,
                    contact_bases_world,
                )
                best_row = -1
                best_pressure = math.inf
                for row in range(len(values)):
                    candidate_pressure = (
                        self._pressure(out["generalized_acceleration"][row, :6])
                        if self._admitted(out, row)
                        else math.inf
                    )
                    self.scores[row] = candidate_pressure
                    if candidate_pressure < best_pressure:
                        best_pressure = candidate_pressure
                        best_row = row
                if best_row < 0:
                    self.request.fill(0.0)
                    self.candidate_pressure = self.zero_pressure
                    return self.request, self.zero_pressure, False
                self.request[axis] = values[best_row]

        one["request"][0] = self.request
        verified = self._query(
            one,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        available = self._admitted(verified, 0)
        self.candidate_pressure = (
            self._pressure(verified["generalized_acceleration"][0, :6])
            if available
            else self.zero_pressure
        )
        available = bool(
            available and self.candidate_pressure < self.zero_pressure - 1.0e-12
        )
        return self.request, self.zero_pressure, available


class PythonBudgetedViabilityPlanner(PythonViabilityCoordinatePlanner):
    """One local coordinate query per tick with Rust-owned path scoring.

    Python retains experiment sequencing and exact-WBC batch orchestration.
    Rust evaluates every candidate at eight fixed forecast knots and keeps
    capture, support, rate, yaw, resource, action, and request-delta evidence
    separate. At most four exact WBC queries occur on one control tick.
    """

    # This is a one-tick trust region, not the supervisor's global emergency
    # bound. It keeps the forecast aligned with the request that can actually
    # become executable on this tick and prevents multi-tick edge walking.
    MAXIMUM_REQUEST = np.asarray([40.0, 40.0, 20.0], np.float64)
    LOCAL_STEP = np.asarray([40.0, 40.0, 20.0], np.float64)
    ACTIVATION_DIAGNOSTIC = "activation_pressure"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        import bonesaw

        self.forecast = bonesaw.UpkieBalanceSession(str(args[0]))
        self.forecast_names = tuple(self.forecast.viability_forecast_diagnostic_names)
        self.forecast_index = {
            name: index for index, name in enumerate(self.forecast_names)
        }
        self.forecast_state = np.empty(10, np.float64)
        self.forecast_achieved = {
            count: np.empty((count, 4), np.float64) for count in (1, 3)
        }
        self.forecast_torque_utilization = {
            count: np.empty(count, np.float64) for count in (1, 3)
        }
        self.forecast_joint_headroom = {
            count: np.empty(count, np.float64) for count in (1, 3)
        }
        self.forecast_diagnostics = {
            count: np.empty((count, len(self.forecast_names)), np.float64)
            for count in (1, 3)
        }
        self.axis_tick = 0
        self.baseline_forecast = np.zeros(len(self.forecast_names), np.float64)
        self.candidate_forecast = np.zeros(len(self.forecast_names), np.float64)
        self.candidate_forecast_path = np.zeros((8, 9), np.float64)
        self.candidate_forecast_path_valid = False
        self.candidate_achieved_acceleration = np.zeros(4, np.float64)
        self.candidate_normal_force = np.zeros(2, np.float64)

    def _score_forecast(
        self,
        out: dict[str, np.ndarray],
        requests: np.ndarray,
        previous_request: np.ndarray,
        rows: int,
    ) -> np.ndarray:
        achieved = self.forecast_achieved[rows]
        torque_utilization = self.forecast_torque_utilization[rows]
        joint_headroom = self.forecast_joint_headroom[rows]
        for row in range(rows):
            generalized = out["generalized_acceleration"][row]
            admitted = (
                self._admitted(out, row)
                and math.isfinite(float(generalized[0]))
                and math.isfinite(float(generalized[1]))
                and math.isfinite(float(generalized[2]))
                and math.isfinite(float(generalized[4]))
            )
            if admitted:
                achieved[row] = (
                    generalized[0],
                    generalized[4],
                    generalized[2],
                    generalized[1],
                )
                torque = float(out["maximum_torque_utilization"][row])
                joint = float(out["minimum_joint_headroom_fraction"][row])
                torque_utilization[row] = (
                    max(torque, 0.0) if math.isfinite(torque) else 1.0
                )
                joint_headroom[row] = (
                    min(max(joint, 0.0), 1.0)
                    if math.isfinite(joint)
                    else (1.0 if joint > 0.0 else 0.0)
                )
            else:
                # The scorer validates the complete fixed batch atomically.
                # Substitute finite worst-case resource evidence for a rejected
                # row; selection still excludes it through `_admitted` below.
                achieved[row].fill(0.0)
                torque_utilization[row] = 1.0
                joint_headroom[row] = 0.0
        diagnostics = self.forecast_diagnostics[rows]
        elapsed_ns, allocation_calls, allocated_bytes = (
            self.forecast.score_viability_forecast_batch(
                self.forecast_state,
                achieved,
                requests,
                previous_request,
                self.MAXIMUM_REQUEST,
                torque_utilization,
                joint_headroom,
                diagnostics,
            )
        )
        self.step_ns += int(elapsed_ns)
        self.allocation_calls += int(allocation_calls)
        self.allocated_bytes += int(allocated_bytes)
        return diagnostics

    def _query_local_candidates(
        self,
        previous: np.ndarray,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        base_root_acceleration: np.ndarray,
        base_root_angular_acceleration: np.ndarray,
        joint_acceleration: np.ndarray,
        contact_active: np.ndarray,
        contact_bases_world: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], int]:
        """Ask one three-point coordinate question at the current state."""
        axis = self.AXIS_ORDER[self.axis_tick % len(self.AXIS_ORDER)]
        self.axis_tick += 1
        batch = self.batches[3]
        batch["request"][:] = previous
        center = float(previous[axis])
        batch["request"][:, axis] = (
            max(-self.MAXIMUM_REQUEST[axis], center - self.LOCAL_STEP[axis]),
            center,
            min(self.MAXIMUM_REQUEST[axis], center + self.LOCAL_STEP[axis]),
        )
        out = self._query(
            batch,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        return batch, out, 3

    def plan(
        self,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        base_root_acceleration: np.ndarray,
        base_root_angular_acceleration: np.ndarray,
        joint_acceleration: np.ndarray,
        contact_active: np.ndarray,
        contact_bases_world: np.ndarray,
        was_active: bool,
        previous_request: np.ndarray | None = None,
    ) -> tuple[np.ndarray, float, bool]:
        self.query_count = 0
        self.step_ns = 0
        self.allocation_calls = 0
        self.allocated_bytes = 0
        self.feasibility_seed_reuses = 0
        self.feasibility_projection_sweeps = 0
        self.feasibility_halfspace_projections = 0
        self.request.fill(0.0)
        self.candidate_forecast_path.fill(0.0)
        self.candidate_forecast_path_valid = False
        self.candidate_achieved_acceleration.fill(0.0)
        self.candidate_normal_force.fill(0.0)
        previous = self.request if previous_request is None else previous_request
        self.rotation[:] = quaternion_rotation_vector(root_quaternion)
        self.forecast_state[:] = (
            self.rotation[0],
            root_twist[0],
            self.rotation[1],
            root_twist[1],
            root_position[1] - self.nominal_lateral,
            root_twist[4],
            self.rotation[2],
            root_twist[2],
            max(float(root_position[2]), 0.05),
            int(contact_active[0]) | (int(contact_active[1]) << 1),
        )

        one = self.batches[1]
        one["request"].fill(0.0)
        zero = self._query(
            one,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        if not self._admitted(zero, 0):
            self.zero_pressure = 0.0
            self.candidate_pressure = 0.0
            self.baseline_forecast.fill(0.0)
            self.candidate_forecast.fill(0.0)
            return self.request, 0.0, False
        baseline = self._score_forecast(zero, one["request"], previous, 1)[0]
        self.baseline_forecast[:] = baseline
        self.zero_pressure = float(baseline[self.forecast_index["total_score"]])
        activation_pressure = float(
            baseline[self.forecast_index[self.ACTIVATION_DIAGNOSTIC]]
        )
        self.candidate_pressure = self.zero_pressure
        self.candidate_forecast[:] = baseline
        should_search = activation_pressure > self.RELEASE_PRESSURE and (
            was_active or activation_pressure >= self.ACTIVATION_PRESSURE
        )
        if not should_search or int(self.forecast_state[9]) == 0:
            return self.request, activation_pressure, False

        batch, out, rows = self._query_local_candidates(
            previous,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        scores = self._score_forecast(out, batch["request"], previous, rows)
        best_row = -1
        best_score = math.inf
        score_column = self.forecast_index["total_score"]
        peak_sagittal_column = self.forecast_index["peak_sagittal_pressure"]
        terminal_sagittal_column = self.forecast_index[
            "terminal_sagittal_pressure"
        ]
        for row in range(rows):
            pitch_path_nonregressive = (
                float(scores[row, peak_sagittal_column])
                <= float(baseline[peak_sagittal_column]) + 1.0e-12
                and float(scores[row, terminal_sagittal_column])
                <= float(baseline[terminal_sagittal_column]) + 1.0e-12
            )
            score = (
                float(scores[row, score_column])
                if self._admitted(out, row) and pitch_path_nonregressive
                else math.inf
            )
            if score < best_score:
                best_score = score
                best_row = row
        minimum_improvement = max(0.01, 0.03 * self.zero_pressure)
        if best_row < 0 or best_score >= self.zero_pressure - minimum_improvement:
            return self.request, activation_pressure, False
        self.request[:] = batch["request"][best_row]
        self.candidate_pressure = best_score
        self.candidate_forecast[:] = scores[best_row]
        self.candidate_achieved_acceleration[:] = self.forecast_achieved[rows][best_row]
        self.candidate_normal_force[:] = out["contact_normal_force"][best_row]
        elapsed_ns, allocation_calls, allocated_bytes = (
            self.forecast.predict_viability_forecast_path(
                self.forecast_state,
                self.forecast_achieved[rows][best_row],
                self.request,
                previous,
                self.MAXIMUM_REQUEST,
                float(self.forecast_torque_utilization[rows][best_row]),
                float(self.forecast_joint_headroom[rows][best_row]),
                self.candidate_forecast_path,
            )
        )
        self.step_ns += int(elapsed_ns)
        self.allocation_calls += int(allocation_calls)
        self.allocated_bytes += int(allocated_bytes)
        self.candidate_forecast_path_valid = True
        return self.request, activation_pressure, True


class PythonLateralBudgetedViabilityPlanner(PythonBudgetedViabilityPlanner):
    """Wake only on roll/lateral capture that this request can influence.

    Sagittal, support, rate, yaw, resource, action, and action-change pressures
    remain independently visible and may veto a proposal. They cannot wake a
    planner whose request coordinates are only roll, lateral, and yaw.
    """

    ACTIVATION_DIAGNOSTIC = "current_capture_pressure"


class PythonAnytimeLateralBudgetedViabilityPlanner(
    PythonLateralBudgetedViabilityPlanner
):
    """Planner-only WBC polls with an eight-iteration feasibility budget."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        kwargs["maximum_feasibility_iterations"] = 8
        super().__init__(*args, **kwargs)


class PythonPairedLateralBudgetedViabilityPlanner(
    PythonLateralBudgetedViabilityPlanner
):
    """Two planner queries per active tick with a Rust-owned proposal phase.

    The same-state zero baseline remains mandatory. Rust emits one signed
    coordinate proposal, the exact WBC realizes it, and the normal forecast
    descent/pitch-veto rule decides whether it can reach the r148 supervisor.
    The final execution WBC remains a separate full-budget solve.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.poll_proposal = np.empty(3, np.float64)
        self.poll_diagnostics = np.empty(
            len(self.forecast.viability_poll_diagnostic_names), np.float64
        )

    def _query_local_candidates(
        self,
        previous: np.ndarray,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        base_root_acceleration: np.ndarray,
        base_root_angular_acceleration: np.ndarray,
        joint_acceleration: np.ndarray,
        contact_active: np.ndarray,
        contact_bases_world: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], int]:
        elapsed_ns, allocation_calls, allocated_bytes = (
            self.forecast.next_viability_poll(
                previous,
                self.MAXIMUM_REQUEST,
                self.LOCAL_STEP,
                self.poll_proposal,
                self.poll_diagnostics,
            )
        )
        self.step_ns += int(elapsed_ns)
        self.allocation_calls += int(allocation_calls)
        self.allocated_bytes += int(allocated_bytes)
        batch = self.batches[1]
        batch["request"][0] = self.poll_proposal
        out = self._query(
            batch,
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            base_root_acceleration,
            base_root_angular_acceleration,
            joint_acceleration,
            contact_active,
            contact_bases_world,
        )
        return batch, out, 1


class RustWbcAdapter:
    def __init__(
        self,
        model_path: pathlib.Path,
        nominal_root_position: np.ndarray,
        target_ground_position: float,
        balance_session: Any,
        balance_mode: str,
        capture_velocity_fraction: float,
        nominal_joint_position: np.ndarray | None = None,
        friction_coefficient: float = 0.8,
        root_angular_task_weight: float = 10.0,
        root_roll_stiffness: float = 24.0,
        root_roll_damping: float = 4.4,
        root_lateral_stiffness: float = 18.0,
        root_lateral_damping: float = 8.0,
        minimum_support_load_fraction: float = 0.0,
        support_load_guard_enabled: bool = False,
        fall_safe_enabled: bool = False,
        fall_safe_primary_blend: bool = True,
        execute_reduced_support: bool = True,
        support_contingency_enabled: bool = False,
        support_contingency_execute: bool = False,
        support_contingency_preserve_primary_support: bool = False,
        support_contingency_query_every_tick: bool = False,
        support_contingency_flight_only: bool = False,
        support_contingency_single_evaluation_lease: bool = False,
        support_contingency_forecast_guard: bool = False,
        support_contingency_project_primary: bool = False,
        support_contingency_realize_primary_torque: bool = False,
        support_contingency_config: tuple[float, ...] | None = None,
        contact_program_authority_ticks: int | None = None,
        contact_program_inexact_hold_ticks: int = 0,
        contact_program_inexact_hold_authority: float = 1.0,
        contact_program_inexact_hold_forecast_selector: bool = False,
        contact_program_inexact_hold_forecast_minimum_improvement: float = 0.0,
        contact_program_inexact_support_free_brake: bool = False,
        contact_program_inexact_terminal_chooser: bool = False,
        contact_program_inexact_terminal_zero_effort_baseline: bool = False,
        contact_program_inexact_terminal_support_hypothesis_envelope: bool = False,
        contact_program_inexact_terminal_maximum_component_regression: float = 0.0,
        contact_program_inexact_terminal_minimum_component_improvement: float = 0.01,
        contact_command_lease_ticks: int | None = None,
        contact_command_lease_fallback_to_freshness: bool = False,
        viability_planner_enabled: bool = False,
        viability_planner_activation_pressure: float = 0.10,
        viability_planner_release_pressure: float = 0.05,
        viability_support_requires_active_request: bool = False,
        viability_planner_strategy: str = "coordinate",
        viability_planner_update_period_ticks: int | None = None,
        viability_confirmation_updates: int = 1,
        execution_residual_veto: bool = False,
        maximum_feasibility_iterations: int = 64,
        maximum_feasibility_projection_sweeps: int | None = None,
        repair_feasibility_equalities_before_inequalities: bool = False,
        use_feasibility_row_spans: bool = False,
        viability_planner_maximum_feasibility_projection_sweeps: int | None = None,
        viability_planner_projection_continuation_violation_threshold: float | None = None,
        viability_planner_reuse_identical_hard_feasibility_seed: bool = False,
        viability_planner_transfer_hard_feasibility_witness: bool = False,
        control_dt: float = CONTROL_DT,
    ):
        import bonesaw

        if not math.isfinite(control_dt) or control_dt <= 0.0:
            raise ValueError("control_dt must be finite and positive")
        self.control_dt = control_dt

        self.session = bonesaw.FloatingWbcSession(
            str(model_path),
            maximum_contacts=2,
            friction_coefficient=friction_coefficient,
            maximum_acceleration=250.0,
            maximum_torque=2000.0,
            maximum_normal_force_multiple=3.0,
            maximum_feasibility_iterations=maximum_feasibility_iterations,
            maximum_feasibility_projection_sweeps=maximum_feasibility_projection_sweeps,
            repair_feasibility_equalities_before_inequalities=repair_feasibility_equalities_before_inequalities,
            use_feasibility_row_spans=use_feasibility_row_spans,
            reuse_identical_hard_feasibility_seed=viability_planner_transfer_hard_feasibility_witness,
            joint_limit_braking=True,
            root_angular_task_weight=root_angular_task_weight,
            root_height_task_weight=10.0,
            root_horizontal_task_weight=10.0,
            root_horizontal_task_priority=1,
            joint_posture_weight=1.0,
            joint_posture_priority=1,
            center_of_mass_task_weight=0.0,
            minimum_support_load_fraction=minimum_support_load_fraction,
        )
        self.support_contingency_enabled = support_contingency_enabled
        self.support_contingency_execute = support_contingency_execute
        self.support_contingency_preserve_primary_support = (
            support_contingency_preserve_primary_support
        )
        self.support_contingency_query_every_tick = (
            support_contingency_query_every_tick
        )
        self.support_contingency_flight_only = support_contingency_flight_only
        self.support_contingency_single_evaluation_lease = (
            support_contingency_single_evaluation_lease
        )
        self.support_contingency_forecast_guard = support_contingency_forecast_guard
        self.support_contingency_project_primary = support_contingency_project_primary
        self.support_contingency_realize_primary_torque = (
            support_contingency_realize_primary_torque
        )
        if support_contingency_project_primary and support_contingency_realize_primary_torque:
            raise ValueError(
                "primary-acceleration projection and fixed-primary-torque realization are mutually exclusive"
            )
        if support_contingency_execute and not support_contingency_enabled:
            raise ValueError("support contingency execution requires its shadow query")
        self.support_contingency_session = (
            bonesaw.FloatingWbcSession(
                str(model_path),
                maximum_contacts=2,
                friction_coefficient=friction_coefficient,
                maximum_acceleration=250.0,
                maximum_torque=2000.0,
                maximum_normal_force_multiple=3.0,
                maximum_feasibility_iterations=maximum_feasibility_iterations,
                maximum_feasibility_projection_sweeps=maximum_feasibility_projection_sweeps,
                repair_feasibility_equalities_before_inequalities=True,
                use_feasibility_row_spans=True,
                joint_limit_braking=True,
                root_angular_task_weight=root_angular_task_weight,
                root_height_task_weight=10.0,
                root_horizontal_task_weight=10.0,
                root_horizontal_task_priority=1,
                joint_posture_weight=1.0,
                joint_posture_priority=1,
                center_of_mass_task_weight=0.0,
                minimum_support_load_fraction=minimum_support_load_fraction,
            )
            if support_contingency_enabled
            else None
        )
        if tuple(self.session.joint_names) != JOINT_ORDER:
            raise RuntimeError("Upkie coordinate order changed")
        self.balance = balance_session
        self.balance.capture_velocity_fraction = capture_velocity_fraction
        if support_contingency_config is not None:
            if len(support_contingency_config) != 11:
                raise ValueError(
                    "support_contingency_config must contain exactly 11 values"
                )
            self.balance.configure_support_contingency(
                *support_contingency_config
            )
        if tuple(self.balance.coordinates) != tuple(ROLLING_COORDINATES):
            raise RuntimeError("Upkie balance coordinate order changed")
        frames = list(self.session.frame_names)
        self.frame_ids = np.asarray(
            [frames.index(name) for name in CONTACT_FRAMES], dtype=np.int64
        )
        self.out = allocate_outputs(self.session)
        self.support_contingency_out = (
            allocate_outputs(self.support_contingency_session)
            if self.support_contingency_session is not None
            else None
        )
        self.root_position = np.empty((1, 3), np.float64)
        self.root_velocity = np.empty((1, 3), np.float64)
        self.root_acceleration = np.empty((1, 3), np.float64)
        self.root_quaternion = np.empty((1, 4), np.float64)
        self.root_angular_velocity = np.empty((1, 3), np.float64)
        self.root_angular_acceleration = np.empty((1, 3), np.float64)
        self.q = np.empty((1, 6), np.float64)
        self.v = np.empty((1, 6), np.float64)
        self.joint_acceleration = np.empty((1, 6), np.float64)
        # A newly built adapter has no accepted plant observation yet. Keep
        # all contact witnesses fail-closed until the worker supplies a fresh
        # MuJoCo mask; the nominal stance is not measured support authority.
        self.contact_active = np.zeros((1, 2), np.uint8)
        self.observed_contact_active = np.zeros(2, np.uint8)
        self.contact_debounced = np.zeros(2, np.uint8)
        self.contact_observation_diagnostics = np.zeros(
            len(self.balance.contact_observation_diagnostic_names), np.int64
        )
        self.contact_observation_index = {
            name: index
            for index, name in enumerate(
                self.balance.contact_observation_diagnostic_names
            )
        }
        self.contact_observation_tick = 0
        self.contact_program_authority_enabled = (
            contact_program_authority_ticks is not None
        )
        self.contact_program_authority_tick = 0
        self.contact_program_authority_diagnostics = np.zeros(
            len(self.balance.contact_program_authority_diagnostic_names), np.int64
        )
        self.contact_program_authority_index = {
            name: index
            for index, name in enumerate(
                self.balance.contact_program_authority_diagnostic_names
            )
        }
        self.contact_program_command = np.zeros(6, np.float64)
        self.contact_program_inexact_hold_forecast_selector = bool(
            contact_program_inexact_hold_forecast_selector
        )
        self.contact_program_inexact_hold_ticks = int(
            contact_program_inexact_hold_ticks
        )
        self.contact_program_inexact_hold_forecast_minimum_improvement = float(
            contact_program_inexact_hold_forecast_minimum_improvement
        )
        self.contact_program_inexact_support_free_brake = bool(
            contact_program_inexact_support_free_brake
        )
        self.contact_program_inexact_terminal_chooser = bool(
            contact_program_inexact_terminal_chooser
        )
        self.contact_program_inexact_terminal_zero_effort_baseline = bool(
            contact_program_inexact_terminal_zero_effort_baseline
        )
        self.contact_program_inexact_terminal_support_hypothesis_envelope = bool(
            contact_program_inexact_terminal_support_hypothesis_envelope
        )
        self.contact_program_inexact_terminal_maximum_component_regression = float(
            contact_program_inexact_terminal_maximum_component_regression
        )
        self.contact_program_inexact_terminal_minimum_component_improvement = float(
            contact_program_inexact_terminal_minimum_component_improvement
        )
        self.inexact_observation_authority_selector_names = tuple(
            self.balance.inexact_observation_authority_selector_diagnostic_names
        )
        self.inexact_observation_authority_selector_index = {
            name: index
            for index, name in enumerate(
                self.inexact_observation_authority_selector_names
            )
        }
        self.inexact_observation_authority_selector_diagnostics = np.zeros(
            len(self.inexact_observation_authority_selector_names), np.float64
        )
        self.inexact_observation_authority_selector_state = np.zeros(9, np.float64)
        self.inexact_observation_authority_selector_step_ns = 0
        self.inexact_observation_authority_selector_allocation_calls = 0
        self.inexact_observation_authority_selector_allocated_bytes = 0
        self.terminal_impact_names = tuple(self.balance.terminal_impact_diagnostic_names)
        self.terminal_impact_index = {
            name: index for index, name in enumerate(self.terminal_impact_names)
        }
        self.terminal_impact_selection_names = tuple(
            self.balance.terminal_impact_selection_diagnostic_names
        )
        self.terminal_impact_selection_index = {
            name: index
            for index, name in enumerate(self.terminal_impact_selection_names)
        }
        self.inexact_observation_terminal_selector_state = np.zeros(6, np.float64)
        self.inexact_observation_terminal_selector_available = np.zeros(3, np.uint8)
        self.inexact_observation_terminal_selector_root_acceleration = np.zeros(
            (3, 2), np.float64
        )
        self.inexact_observation_terminal_selector_joint_acceleration = np.zeros(
            (3, 6), np.float64
        )
        self.inexact_observation_terminal_selector_effort_utilization = np.zeros(
            3, np.float64
        )
        self.inexact_observation_terminal_selector_diagnostics = np.zeros(
            (3, len(self.terminal_impact_names)), np.float64
        )
        self.inexact_observation_terminal_selector_selection = np.zeros(
            len(self.terminal_impact_selection_names), np.float64
        )
        self.inexact_observation_terminal_selector_joint_lower = np.asarray(
            self.session.joint_position_lower_limits, np.float64
        )
        self.inexact_observation_terminal_selector_joint_upper = np.asarray(
            self.session.joint_position_upper_limits, np.float64
        )
        self.inexact_observation_terminal_selector_joint_velocity_limit = np.asarray(
            self.session.joint_velocity_limits, np.float64
        )
        self.inexact_observation_terminal_selector_retained_joint_acceleration = (
            np.zeros(6, np.float64)
        )
        self.inexact_observation_terminal_zero_effort = np.zeros((1, 6), np.float64)
        self.inexact_observation_terminal_zero_effort_acceleration = np.zeros(
            12, np.float64
        )
        self.inexact_observation_terminal_zero_effort_available = False
        self.inexact_observation_terminal_zero_effort_step_ns = 0
        self.inexact_observation_terminal_zero_effort_allocation_calls = 0
        self.inexact_observation_terminal_zero_effort_allocated_bytes = 0
        self.inexact_observation_terminal_hypothesis_fixed_effort = np.zeros(
            (1, 6), np.float64
        )
        self.inexact_observation_terminal_hypothesis_acceleration = np.zeros(
            (4, 3, 12), np.float64
        )
        self.inexact_observation_terminal_hypothesis_available = np.zeros(
            (4, 3), np.uint8
        )
        self.inexact_observation_terminal_hypothesis_diagnostics = np.zeros(
            (3, 4, len(self.terminal_impact_names)), np.float64
        )
        self.inexact_observation_terminal_hypothesis_envelopes = np.zeros(
            (3, len(self.terminal_impact_names)), np.float64
        )
        self.inexact_observation_terminal_hypothesis_selection_scratch = np.zeros(
            len(self.terminal_impact_selection_names), np.float64
        )
        self.inexact_observation_terminal_hypothesis_torque = np.zeros(
            (3, 6), np.float64
        )
        self.inexact_observation_terminal_hypothesis_effort_utilization = np.zeros(
            3, np.float64
        )
        self.inexact_observation_terminal_hypothesis_query_step_ns = 0
        self.inexact_observation_terminal_hypothesis_query_allocation_calls = 0
        self.inexact_observation_terminal_hypothesis_query_allocated_bytes = 0
        self.inexact_observation_terminal_hypothesis_aggregate_step_ns = 0
        self.inexact_observation_terminal_hypothesis_aggregate_allocation_calls = 0
        self.inexact_observation_terminal_hypothesis_aggregate_allocated_bytes = 0
        self.inexact_observation_terminal_selector_queried = False
        self.inexact_observation_terminal_selector_step_ns = 0
        self.inexact_observation_terminal_selector_allocation_calls = 0
        self.inexact_observation_terminal_selector_allocated_bytes = 0
        self.primary_program_torque = np.zeros((1, 6), np.float64)
        self.primary_program_available = False
        self.primary_program_source_fresh = False
        self.contact_program_authority_has_primary_history = False
        self.contact_program_hard_contact = np.ones(2, np.uint8)
        if contact_program_authority_ticks is not None:
            if contact_program_authority_ticks < 0:
                raise ValueError(
                    "contact program authority hold ticks must be nonnegative"
                )
            if not support_contingency_enabled:
                raise ValueError(
                    "contact program authority requires the current-support contingency query"
                )
            if contact_program_inexact_hold_ticks < 0:
                raise ValueError(
                    "contact program inexact-observation hold ticks must be nonnegative"
                )
            if not math.isfinite(contact_program_inexact_hold_authority) or not (
                0.0 <= contact_program_inexact_hold_authority <= 1.0
            ):
                raise ValueError(
                    "contact program inexact-observation hold authority must be finite in [0, 1]"
                )
            self.balance.configure_contact_program_authority(
                contact_program_authority_ticks,
                support_contingency_realize_primary_torque,
                contact_program_inexact_hold_ticks,
                contact_program_inexact_hold_authority,
            )
            if (
                self.contact_program_inexact_hold_forecast_selector
                and contact_program_inexact_hold_ticks == 0
            ):
                raise ValueError(
                    "inexact-observation forecast selection requires a positive hold budget"
                )
            if not math.isfinite(
                self.contact_program_inexact_hold_forecast_minimum_improvement
            ) or self.contact_program_inexact_hold_forecast_minimum_improvement < 0.0:
                raise ValueError(
                    "inexact-observation forecast minimum improvement must be finite and nonnegative"
                )
            if (
                self.contact_program_inexact_hold_forecast_minimum_improvement != 0.0
                and not self.contact_program_inexact_hold_forecast_selector
            ):
                raise ValueError(
                    "inexact-observation forecast minimum improvement requires forecast selection"
                )
            if (
                self.contact_program_inexact_support_free_brake
                and not support_contingency_enabled
            ):
                raise ValueError(
                    "inexact-observation support-free brake requires its independent WBC session"
                )
            if self.contact_program_inexact_terminal_chooser:
                if not support_contingency_enabled:
                    raise ValueError(
                        "inexact-observation terminal chooser requires its independent WBC session"
                    )
                if contact_program_inexact_hold_ticks <= 0:
                    raise ValueError(
                        "inexact-observation terminal chooser requires a positive retained-command budget"
                    )
                if self.contact_program_inexact_hold_forecast_selector:
                    raise ValueError(
                        "terminal and retained-effort forecast selectors are mutually exclusive"
                    )
                if self.contact_program_inexact_support_free_brake:
                    raise ValueError(
                        "terminal chooser and unconditional support-free brake are mutually exclusive"
                    )
                terminal_values = (
                    self.contact_program_inexact_terminal_maximum_component_regression,
                    self.contact_program_inexact_terminal_minimum_component_improvement,
                )
                if not all(math.isfinite(value) for value in terminal_values):
                    raise ValueError(
                        "inexact-observation terminal chooser parameters must be finite"
                    )
                if any(value < 0.0 for value in terminal_values):
                    raise ValueError(
                        "invalid inexact-observation terminal chooser regression/improvement"
                    )
                if (
                    self.contact_program_inexact_terminal_zero_effort_baseline
                    and self.contact_program_inexact_terminal_support_hypothesis_envelope
                ):
                    raise ValueError(
                        "single zero-effort and support-hypothesis terminal baselines are mutually exclusive"
                    )
            elif self.contact_program_inexact_terminal_zero_effort_baseline:
                raise ValueError(
                    "terminal zero-effort baseline requires terminal chooser"
                )
            elif self.contact_program_inexact_terminal_support_hypothesis_envelope:
                raise ValueError(
                    "terminal support-hypothesis envelope requires terminal chooser"
                )
        elif contact_program_inexact_hold_ticks or contact_program_inexact_hold_authority != 1.0:
            raise ValueError(
                "contact program inexact-observation hold configuration requires program authority"
            )
        elif self.contact_program_inexact_hold_forecast_selector:
            raise ValueError(
                "inexact-observation forecast selection requires program authority"
            )
        elif self.contact_program_inexact_hold_forecast_minimum_improvement != 0.0:
            raise ValueError(
                "inexact-observation forecast minimum improvement requires forecast selection"
            )
        elif self.contact_program_inexact_support_free_brake:
            raise ValueError(
                "inexact-observation support-free brake requires program authority"
            )
        elif self.contact_program_inexact_terminal_chooser:
            raise ValueError(
                "inexact-observation terminal chooser requires program authority"
            )
        elif self.contact_program_inexact_terminal_zero_effort_baseline:
            raise ValueError(
                "terminal zero-effort baseline requires program authority"
            )
        elif self.contact_program_inexact_terminal_support_hypothesis_envelope:
            raise ValueError(
                "terminal support-hypothesis envelope requires program authority"
            )
        self.contact_command_lease_enabled = contact_command_lease_ticks is not None
        self.contact_command_lease_fallback_to_freshness = (
            contact_command_lease_fallback_to_freshness
        )
        self.contact_command_lease_tick = 0
        self.contact_command_lease_diagnostics = np.zeros(
            len(self.balance.contact_command_lease_diagnostic_names), np.int64
        )
        self.contact_command_lease_index = {
            name: index
            for index, name in enumerate(
                self.balance.contact_command_lease_diagnostic_names
            )
        }
        self.contact_command = np.zeros(6, np.float64)
        if contact_command_lease_ticks is not None:
            if self.contact_program_authority_enabled:
                raise ValueError(
                    "contact program authority and legacy command lease are mutually exclusive"
                )
            if contact_command_lease_ticks < 0:
                raise ValueError("contact command lease ticks must be nonnegative")
            self.balance.configure_contact_command_lease(contact_command_lease_ticks)
        self.priorities = np.ones(2, np.uint8)
        self.weights = np.ones(2, np.float64)
        self.contact_modes = np.full(2, 3, np.uint8)
        self.rolling_gains = np.full(2, 2.0, np.float64)
        self.rolling_corrections = np.full(2, 3.0, np.float64)
        self.wheel_acceleration = np.empty(2, np.float64)
        self.virtual_pitch = np.empty(1, np.float64)
        diagnostic_names = (
            self.balance.planar_capture_diagnostic_names
            if balance_mode
            in (
                "planar_capture",
                "viability_capture",
                "viability_verified",
                "viability_support_capture",
                "viability_coordinate",
            )
            else self.balance.capture_diagnostic_names
        )
        self.capture_diagnostics = np.zeros(len(diagnostic_names), np.float64)
        self.capture_index = {
            name: index
            for index, name in enumerate(diagnostic_names)
        }
        self.contact_bases_world = np.zeros((1, 2, 3, 3), np.float64)
        self.contact_bases_world[:, :, 0, 0] = 1.0
        self.contact_bases_world[:, :, 1, 1] = 1.0
        self.contact_bases_world[:, :, 2, 2] = 1.0
        self.zero_translation = np.zeros(3, np.float64)
        self.identity_quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], np.float64)
        self.last_admitted_torque = np.zeros(6, np.float64)
        self.last_admitted_contact_force = np.zeros((2, 3), np.float64)
        self.executed_torque = np.zeros(6, np.float64)
        self.executed_contact_force = np.zeros((2, 3), np.float64)
        self.command_age_steps = 0
        self.previous_solver_admitted = True
        self.has_admitted_command = False
        self.fall_safe_enabled = fall_safe_enabled
        self.fall_safe_primary_blend = fall_safe_primary_blend
        self.execute_reduced_support = execute_reduced_support
        self.support_contingency_diagnostics = np.zeros(
            len(self.balance.support_contingency_diagnostic_names), np.float64
        )
        self.support_contingency_root_angular_acceleration = np.zeros(
            (1, 3), np.float64
        )
        self.support_contingency_root_acceleration = np.zeros((1, 3), np.float64)
        self.support_contingency_joint_acceleration = np.zeros((1, 6), np.float64)
        self.support_contingency_candidate_generalized_acceleration = np.zeros(
            12, np.float64
        )
        self.support_contingency_candidate_torque = np.zeros(6, np.float64)
        self.support_contingency_root_twist = np.zeros(6, np.float64)
        self.support_contingency_author_step_ns = 0
        self.support_contingency_author_allocation_calls = 0
        self.support_contingency_author_allocated_bytes = 0
        self.support_contingency_admitted = False
        self.support_contingency_armed = False
        self.support_contingency_consumed = False
        self.support_contingency_requested = False
        self.support_contingency_selected = False
        self.support_free_inexact_brake_available = False
        self.support_contingency_support_mask = 3
        self.support_contingency_candidate_power_w = 0.0
        self.support_contingency_realization_fallback = False
        self.support_contingency_primary_power_w = 0.0
        self.support_contingency_incremental_power_w = 0.0
        self.support_contingency_forecast_guard_passed = True
        self.support_contingency_forecast_baseline_score = 0.0
        self.support_contingency_forecast_candidate_score = 0.0
        self.support_contingency_forecast_step_ns = 0
        self.support_contingency_forecast_allocation_calls = 0
        self.support_contingency_forecast_allocated_bytes = 0
        self.support_contingency_forecast_state = np.zeros(10, np.float64)
        self.support_contingency_forecast_achieved = np.zeros((2, 4), np.float64)
        self.support_contingency_forecast_requests = np.zeros((2, 3), np.float64)
        self.support_contingency_forecast_previous_request = np.zeros(3, np.float64)
        self.support_contingency_forecast_maximum_request = np.asarray(
            [250.0, 250.0, 80.0], np.float64
        )
        self.support_contingency_forecast_torque_utilization = np.zeros(2, np.float64)
        self.support_contingency_forecast_joint_headroom = np.ones(2, np.float64)
        self.support_contingency_forecast_diagnostics = np.zeros((2, 15), np.float64)
        self.fall_safe_diagnostics = np.zeros(
            len(self.balance.fall_safe_diagnostic_names), np.float64
        )
        self.fall_safe_index = {
            name: index
            for index, name in enumerate(self.balance.fall_safe_diagnostic_names)
        }
        self.contingency_root_angular_acceleration = np.zeros(3, np.float64)
        self.contingency_root_acceleration = np.zeros(3, np.float64)
        self.contingency_joint_acceleration = np.zeros(6, np.float64)
        self.contingency_root_angular_blend = np.zeros(3, np.float64)
        self.contingency_root_blend = np.zeros(3, np.float64)
        self.contingency_blend = np.zeros((1, 6), np.float64)
        self.nominal_root_position = nominal_root_position.copy()
        self.nominal_joint_position = (
            standing_posture().copy()
            if nominal_joint_position is None
            else np.asarray(nominal_joint_position, np.float64).copy()
        )
        if self.nominal_joint_position.shape != (6,) or not np.all(
            np.isfinite(self.nominal_joint_position)
        ):
            raise ValueError("nominal_joint_position must contain six finite values")
        self.target_ground_position = target_ground_position
        self.balance_mode = balance_mode
        self.root_roll_stiffness = root_roll_stiffness
        self.root_roll_damping = root_roll_damping
        self.root_lateral_stiffness = root_lateral_stiffness
        self.root_lateral_damping = root_lateral_damping
        self.support_load_guard_enabled = support_load_guard_enabled
        self.support_load_guard_active = False
        self.support_load_guard_authority = 0.0
        self.support_load_guard_release_ticks = 0
        self.viability_verification_scales = (1.0, 0.5, 0.25, 0.125, 0.0)
        self.viability_lateral_delta_x = 0.0
        self.viability_lateral_delta_y = 0.0
        self.viability_bank_delta_x = 0.0
        self.viability_bank_delta_y = 0.0
        self.viability_verified_scale = 1.0
        self.viability_verification_queries = 1
        self.viability_planner_enabled = (
            viability_planner_enabled or balance_mode == "viability_coordinate"
        )
        self.viability_support_requires_active_request = (
            viability_support_requires_active_request
        )
        if viability_planner_strategy not in (
            "coordinate",
            "multistep_budgeted",
            "lateral_multistep_budgeted",
            "lateral_anytime_multistep_budgeted",
            "lateral_paired_multistep_budgeted",
            "hybrid_confirmed_multistep_budgeted",
        ):
            raise ValueError(
                "viability_planner_strategy must be coordinate, multistep_budgeted, lateral_multistep_budgeted, lateral_anytime_multistep_budgeted, lateral_paired_multistep_budgeted, or hybrid_confirmed_multistep_budgeted"
            )
        self.viability_planner_strategy = viability_planner_strategy
        self.viability_hybrid_guard_enabled = (
            viability_planner_strategy == "hybrid_confirmed_multistep_budgeted"
        )
        if viability_confirmation_updates < 1:
            raise ValueError("viability_confirmation_updates must be at least one")
        if viability_confirmation_updates > 1 and viability_planner_strategy == "coordinate":
            raise ValueError(
                "viability confirmation is only defined for the budgeted forecast planners"
            )
        self.viability_confirmation_updates = viability_confirmation_updates
        self.viability_confirmation_enabled = viability_confirmation_updates > 1
        self.execution_residual_veto_enabled = execution_residual_veto
        default_planner_period = 1 if viability_planner_strategy != "coordinate" else 4
        if viability_planner_update_period_ticks is None:
            viability_planner_update_period_ticks = default_planner_period
        if not 1 <= viability_planner_update_period_ticks <= 16:
            raise ValueError("viability planner update period must be in 1..=16 ticks")
        self.viability_planner_update_period_ticks = (
            viability_planner_update_period_ticks
        )
        self.viability_request_tick = 0
        self.viability_candidate = np.zeros(3, np.float64)
        self.viability_request = np.zeros(3, np.float64)
        self.viability_request_diagnostics = np.zeros(
            len(self.balance.viability_request_diagnostic_names), np.float64
        )
        self.viability_request_index = {
            name: index
            for index, name in enumerate(
                self.balance.viability_request_diagnostic_names
            )
        }
        self.viability_confirmation_candidate = np.zeros(3, np.float64)
        self.viability_confirmation_shadow = np.zeros(3, np.float64)
        self.viability_confirmation_diagnostics = np.zeros(
            len(self.balance.viability_confirmation_diagnostic_names), np.float64
        )
        self.viability_confirmation_index = {
            name: index
            for index, name in enumerate(
                self.balance.viability_confirmation_diagnostic_names
            )
        }
        self.viability_hybrid_guard_diagnostics = np.zeros(
            len(self.balance.viability_hybrid_guard_diagnostic_names), np.float64
        )
        self.viability_hybrid_guard_index = {
            name: index
            for index, name in enumerate(
                self.balance.viability_hybrid_guard_diagnostic_names
            )
        }
        self.viability_candidate_normal_force = np.zeros(2, np.float64)
        self.viability_roll_capture_pressure = 0.0
        self.viability_planner_query_count = 0
        self.viability_planner_step_ns = 0
        self.viability_planner_feasibility_seed_reuses = 0
        self.viability_planner_feasibility_projection_sweeps = 0
        self.viability_planner_feasibility_halfspace_projections = 0
        self.viability_planner_transfer_hard_feasibility_witness = (
            viability_planner_transfer_hard_feasibility_witness
        )
        self.viability_hard_feasibility_witness_transferred = False
        self.final_feasibility_seed_reused = False
        self.final_feasibility_prefix_resumed = False
        self.viability_hard_feasibility_witness_transfer_diagnostics = np.zeros(
            3, np.uint64
        )
        self.viability_planner_zero_pressure = 0.0
        self.viability_planner_candidate_pressure = 0.0
        self.viability_forecast_names = tuple(
            self.balance.viability_forecast_diagnostic_names
        )
        self.viability_forecast_index = {
            name: index for index, name in enumerate(self.viability_forecast_names)
        }
        self.viability_forecast_diagnostics = np.zeros(
            len(self.viability_forecast_names), np.float64
        )
        self.viability_forecast_path = np.zeros((8, 9), np.float64)
        self.viability_forecast_path_valid = False
        # r157 calibration witness: unlike the planner path above, this path is
        # generated from the final exact WBC acceleration that is admitted for
        # execution on this tick.  Buffers are retained across ticks so the
        # measurement cannot hide Python allocation in the controller loop.
        self.execution_forecast_state = np.zeros(10, np.float64)
        self.execution_forecast_reduced_state = np.zeros(8, np.float64)
        self.execution_forecast_achieved = np.zeros(4, np.float64)
        self.execution_forecast_maximum_torque_utilization = 0.0
        self.execution_forecast_minimum_joint_headroom_fraction = 1.0
        self.execution_forecast_maximum_request = np.asarray(
            [250.0, 250.0, 80.0], np.float64
        )
        self.execution_forecast_path = np.zeros((8, 9), np.float64)
        self.execution_forecast_path_valid = False
        self.execution_forecast_support_mask = 0
        self.execution_forecast_step_ns = 0
        self.execution_forecast_allocation_calls = 0
        self.execution_forecast_allocated_bytes = 0
        self.execution_residual_state = np.zeros(8, np.float64)
        self.execution_residual_prediction = np.zeros(8, np.float64)
        self.execution_residual_prediction_valid = False
        self.execution_residual_prediction_support_mask = 0
        self.execution_residual_scale = np.asarray(
            [math.pi / 4.0, 4.0, math.pi / 4.0, 4.0, 0.10, 1.0, math.pi / 4.0, 4.0],
            np.float64,
        )
        self.execution_residual_minimum_bound = np.full(8, 1.0e-4, np.float64)
        self.execution_residual_maximum_bound = np.full(8, 0.25, np.float64)
        self.execution_residual_diagnostics = np.zeros(
            len(self.balance.viability_execution_monitor_diagnostic_names), np.float64
        )
        self.execution_residual_index = {
            name: index
            for index, name in enumerate(
                self.balance.viability_execution_monitor_diagnostic_names
            )
        }
        self.execution_residual_step_ns = 0
        self.execution_residual_allocation_calls = 0
        self.execution_residual_allocated_bytes = 0
        self.execution_residual_tick = 0
        if self.execution_residual_veto_enabled:
            self.balance.configure_viability_execution_monitor(
                8,
                2.0,
                self.execution_residual_minimum_bound,
                self.execution_residual_maximum_bound,
            )
        self.viability_planner = None
        if self.viability_planner_enabled:
            self.balance.configure_viability_request(
                viability_planner_activation_pressure,
                viability_planner_release_pressure,
                self.viability_planner_update_period_ticks,
                np.asarray([250.0, 250.0, 80.0], np.float64),
                np.asarray([40.0, 40.0, 20.0], np.float64),
            )
            if self.viability_confirmation_enabled:
                self.balance.configure_viability_confirmation(
                    viability_confirmation_updates,
                    self.viability_planner_update_period_ticks,
                    0.0,
                    0.5,
                    np.asarray([40.0, 40.0, 20.0], np.float64),
                )
            if self.viability_hybrid_guard_enabled:
                self.balance.configure_viability_hybrid_guard(
                    3,
                    0.05,
                    0.20,
                    np.asarray([40.0, 40.0, 20.0], np.float64),
                )
            planner_type = {
                "coordinate": PythonViabilityCoordinatePlanner,
                "multistep_budgeted": PythonBudgetedViabilityPlanner,
                "lateral_multistep_budgeted": PythonLateralBudgetedViabilityPlanner,
                "lateral_anytime_multistep_budgeted": PythonAnytimeLateralBudgetedViabilityPlanner,
                "lateral_paired_multistep_budgeted": PythonPairedLateralBudgetedViabilityPlanner,
                "hybrid_confirmed_multistep_budgeted": PythonBudgetedViabilityPlanner,
            }[viability_planner_strategy]
            self.viability_planner = planner_type(
                model_path,
                friction_coefficient,
                root_angular_task_weight,
                nominal_root_position,
                viability_planner_activation_pressure,
                viability_planner_release_pressure,
                maximum_feasibility_iterations,
                (
                    viability_planner_maximum_feasibility_projection_sweeps
                    if viability_planner_maximum_feasibility_projection_sweeps
                    is not None
                    else maximum_feasibility_projection_sweeps
                ),
                viability_planner_projection_continuation_violation_threshold,
                repair_feasibility_equalities_before_inequalities,
                use_feasibility_row_spans,
                viability_planner_reuse_identical_hard_feasibility_seed,
            )
        self.viability_support_active = False
        self.viability_support_pressure = 0.0
        self.viability_coordinate_tick = 0
        self.viability_coordinate_target = np.zeros(3, np.float64)
        self.viability_coordinate_request = np.zeros(3, np.float64)
        self.viability_coordinate_queries = 0
        self.viability_coordinate_score = 0.0

    def _run_wbc_query(self) -> None:
        out = self.out
        self.session.run_oracle_trace(
            self.root_position,
            self.root_velocity,
            self.root_acceleration,
            self.q,
            self.v,
            self.joint_acceleration,
            self.frame_ids,
            self.contact_active,
            self.priorities,
            self.weights,
            0,
            0,
            out["generalized_acceleration"],
            out["actuator_torque"],
            out["contact_normal_force"],
            out["task_rms"],
            out["task_clipped"],
            out["dynamics_residual"],
            out["contact_residual"],
            out["minimum_friction_margin"],
            out["minimum_support_margin"],
            out["minimum_torque_margin"],
            out["maximum_constraint_violation"],
            out["minimum_bound_margin"],
            out["minimum_joint_margin_rad"],
            out["minimum_joint_headroom_fraction"],
            out["limiting_joint"],
            out["maximum_torque_utilization"],
            out["minimum_torque_headroom"],
            out["limiting_actuator"],
            out["witness_acceleration_rms"],
            out["step_ns"],
            out["status"],
            out["task_pseudoinverse_calls"],
            out["task_pseudoinverse_calls_by_priority"],
            out["clipped_steps"],
            out["clipped_steps_by_priority"],
            out["task_jacobi_sweeps"],
            out["task_jacobi_sweeps_by_priority"],
            out["feasibility_projection_sweeps"],
            out["feasibility_halfspace_projections"],
            out["feasibility_polish_iterations"],
            out["allocation_calls"],
            out["allocated_bytes"],
            contact_force_basis_out=out["contact_force_basis"],
            contact_modes=self.contact_modes,
            rolling_coordinates=ROLLING_COORDINATES,
            rolling_velocity_coefficients=ROLLING_COEFFICIENTS,
            rolling_velocity_stabilization_gains=self.rolling_gains,
            rolling_maximum_stabilization_accelerations=self.rolling_corrections,
            root_quaternions_wxyz=self.root_quaternion,
            root_angular_velocities_world=self.root_angular_velocity,
            root_angular_accelerations_world=self.root_angular_acceleration,
            contact_bases_world=self.contact_bases_world,
            feasibility_seed_reused_out=out["feasibility_seed_reused"],
            feasibility_prefix_resumed_out=out["feasibility_prefix_resumed"],
        )

    def _run_support_contingency_query(
        self,
        observation_exact: bool,
        *,
        project_primary: bool = False,
        realize_primary_torque: bool = False,
        fixed_actuator_effort: np.ndarray | None = None,
    ) -> None:
        if self.support_contingency_session is None or self.support_contingency_out is None:
            return
        if realize_primary_torque and fixed_actuator_effort is not None:
            raise ValueError(
                "primary-torque and explicit fixed-effort realization are mutually exclusive"
            )
        if realize_primary_torque:
            self.support_contingency_realization_fallback = False
        support_mask = int(self.contact_active[0, 0]) | (
            int(self.contact_active[0, 1]) << 1
        )
        self.support_contingency_root_twist[:3] = self.root_angular_velocity[0]
        self.support_contingency_root_twist[3:] = self.root_velocity[0]
        if project_primary or realize_primary_torque:
            primary_acceleration = self.out["generalized_acceleration"][0]
            self.support_contingency_root_angular_acceleration[0] = (
                primary_acceleration[:3]
            )
            self.support_contingency_root_acceleration[0] = primary_acceleration[3:6]
            self.support_contingency_joint_acceleration[0] = primary_acceleration[6:]
            self.support_contingency_diagnostics.fill(0.0)
            self.support_contingency_diagnostics[0] = (
                0.0 if support_mask == 3 else 1.0 if support_mask in (1, 2) else 2.0
            )
            self.support_contingency_diagnostics[1] = support_mask
            self.support_contingency_author_step_ns = 0
            self.support_contingency_author_allocation_calls = 0
            self.support_contingency_author_allocated_bytes = 0
        else:
            (
                self.support_contingency_author_step_ns,
                self.support_contingency_author_allocation_calls,
                self.support_contingency_author_allocated_bytes,
            ) = self.balance.write_support_contingency_from_state(
                support_mask,
                self.root_position[0],
                self.root_quaternion[0],
                self.support_contingency_root_twist,
                self.q[0],
                self.v[0],
                self.support_contingency_diagnostics,
                self.support_contingency_root_angular_acceleration[0],
                self.support_contingency_root_acceleration[0],
                self.support_contingency_joint_acceleration[0],
            )
        out = self.support_contingency_out
        self.support_contingency_session.run_oracle_trace(
            self.root_position,
            self.root_velocity,
            self.support_contingency_root_acceleration,
            self.q,
            self.v,
            self.support_contingency_joint_acceleration,
            self.frame_ids,
            self.contact_active,
            self.priorities,
            self.weights,
            0,
            0,
            out["generalized_acceleration"],
            out["actuator_torque"],
            out["contact_normal_force"],
            out["task_rms"],
            out["task_clipped"],
            out["dynamics_residual"],
            out["contact_residual"],
            out["minimum_friction_margin"],
            out["minimum_support_margin"],
            out["minimum_torque_margin"],
            out["maximum_constraint_violation"],
            out["minimum_bound_margin"],
            out["minimum_joint_margin_rad"],
            out["minimum_joint_headroom_fraction"],
            out["limiting_joint"],
            out["maximum_torque_utilization"],
            out["minimum_torque_headroom"],
            out["limiting_actuator"],
            out["witness_acceleration_rms"],
            out["step_ns"],
            out["status"],
            out["task_pseudoinverse_calls"],
            out["task_pseudoinverse_calls_by_priority"],
            out["clipped_steps"],
            out["clipped_steps_by_priority"],
            out["task_jacobi_sweeps"],
            out["task_jacobi_sweeps_by_priority"],
            out["feasibility_projection_sweeps"],
            out["feasibility_halfspace_projections"],
            out["feasibility_polish_iterations"],
            out["allocation_calls"],
            out["allocated_bytes"],
            contact_force_basis_out=out["contact_force_basis"],
            contact_modes=self.contact_modes,
            rolling_coordinates=ROLLING_COORDINATES,
            rolling_velocity_coefficients=ROLLING_COEFFICIENTS,
            rolling_velocity_stabilization_gains=self.rolling_gains,
            rolling_maximum_stabilization_accelerations=self.rolling_corrections,
            root_quaternions_wxyz=self.root_quaternion,
            root_angular_velocities_world=self.root_angular_velocity,
            root_angular_accelerations_world=(
                self.support_contingency_root_angular_acceleration
            ),
            contact_bases_world=self.contact_bases_world,
            fixed_actuator_effort=(
                self.primary_program_torque
                if realize_primary_torque
                else fixed_actuator_effort
            ),
            realization_reference_acceleration=(
                self.out["generalized_acceleration"]
                if realize_primary_torque or fixed_actuator_effort is not None
                else None
            ),
            feasibility_seed_reused_out=out["feasibility_seed_reused"],
            feasibility_prefix_resumed_out=out["feasibility_prefix_resumed"],
        )
        self.support_contingency_candidate_generalized_acceleration[:] = out[
            "generalized_acceleration"
        ][0]
        self.support_contingency_candidate_torque[:] = out["actuator_torque"][0]
        if realize_primary_torque and (
            int(out["status"][0]) not in (0, 1)
            or float(out["maximum_constraint_violation"][0]) >= 1.0e-8
        ):
            self._run_support_contingency_query(observation_exact)
            self.support_contingency_realization_fallback = True

    def _run_terminal_support_hypothesis_queries(
        self,
        retained_available: bool,
    ) -> None:
        """Realize three typed efforts under four explicit support masks."""
        if self.support_contingency_out is None:
            return
        self.inexact_observation_terminal_hypothesis_acceleration.fill(0.0)
        self.inexact_observation_terminal_hypothesis_available.fill(0)
        self.inexact_observation_terminal_hypothesis_torque.fill(0.0)
        self.inexact_observation_terminal_hypothesis_effort_utilization.fill(0.0)
        self.inexact_observation_terminal_hypothesis_query_step_ns = 0
        self.inexact_observation_terminal_hypothesis_query_allocation_calls = 0
        self.inexact_observation_terminal_hypothesis_query_allocated_bytes = 0
        self.inexact_observation_terminal_hypothesis_torque[1] = (
            self.contact_program_command
        )
        self.inexact_observation_terminal_hypothesis_torque[2] = (
            self.support_contingency_out["actuator_torque"][0]
        )
        self.inexact_observation_terminal_hypothesis_effort_utilization[1] = (
            self.execution_forecast_maximum_torque_utilization
        )
        self.inexact_observation_terminal_hypothesis_effort_utilization[2] = float(
            self.support_contingency_out["maximum_torque_utilization"][0]
        )
        action_available = (
            True,
            retained_available,
            self.support_free_inexact_brake_available,
        )
        for support_mask in range(4):
            self.contact_active[0] = (
                support_mask & 1,
                (support_mask >> 1) & 1,
            )
            for action in range(3):
                if not action_available[action]:
                    continue
                self.inexact_observation_terminal_hypothesis_fixed_effort[0] = (
                    self.inexact_observation_terminal_hypothesis_torque[action]
                )
                self._run_support_contingency_query(
                    False,
                    fixed_actuator_effort=(
                        self.inexact_observation_terminal_hypothesis_fixed_effort
                    ),
                )
                self.inexact_observation_terminal_hypothesis_acceleration[
                    support_mask, action
                ] = self.support_contingency_candidate_generalized_acceleration
                admitted = bool(
                    int(self.support_contingency_out["status"][0]) in (0, 1)
                    and float(
                        self.support_contingency_out[
                            "maximum_constraint_violation"
                        ][0]
                    )
                    < 1.0e-8
                )
                self.inexact_observation_terminal_hypothesis_available[
                    support_mask, action
                ] = admitted
                self.inexact_observation_terminal_hypothesis_query_step_ns += (
                    self.support_contingency_author_step_ns
                    + int(self.support_contingency_out["step_ns"][0])
                )
                self.inexact_observation_terminal_hypothesis_query_allocation_calls += (
                    self.support_contingency_author_allocation_calls
                    + int(self.support_contingency_out["allocation_calls"][0])
                )
                self.inexact_observation_terminal_hypothesis_query_allocated_bytes += (
                    self.support_contingency_author_allocated_bytes
                    + int(self.support_contingency_out["allocated_bytes"][0])
                )
        self.contact_active.fill(1)
        # The bounded hypothesis total is accounted separately. Do not count
        # the final fixed-effort query again through the ordinary contingency
        # timing fields.
        self.support_contingency_author_step_ns = 0
        self.support_contingency_author_allocation_calls = 0
        self.support_contingency_author_allocated_bytes = 0
        self.support_contingency_out["step_ns"].fill(0)
        self.support_contingency_out["allocation_calls"].fill(0)
        self.support_contingency_out["allocated_bytes"].fill(0)
        self.support_contingency_candidate_torque[:] = (
            self.inexact_observation_terminal_hypothesis_torque[2]
        )

    def prime_contact_observation(
        self,
        observed_contact_active: np.ndarray,
        samples: int,
        age_ticks: int = 0,
    ) -> None:
        """Warm only causal contact evidence before actuator authority is enabled."""
        if observed_contact_active.shape != (2,):
            raise ValueError("observed_contact_active must have shape (2,)")
        if samples < 0:
            raise ValueError("contact observation prestart samples must be nonnegative")
        if age_ticks < 0:
            raise ValueError("contact observation prestart age must be nonnegative")
        np.copyto(self.observed_contact_active, observed_contact_active, casting="unsafe")
        # Advance the receiver clock before admitting the warm samples so a
        # steady delayed stream remains monotonic when actuator time begins.
        # Without this offset, switching from zero-age priming to an aged
        # runtime sample manufactures a backwards-timestamp fault.
        self.contact_observation_tick += age_ticks
        for _ in range(samples):
            self.contact_observation_tick += 1
            timestamp_ns = self.contact_observation_tick * int(self.control_dt * 1.0e9)
            mapped_timestamp_ns = timestamp_ns - age_ticks * int(
                self.control_dt * 1.0e9
            )
            self.balance.step_contact_observation_from_mask(
                timestamp_ns,
                True,
                mapped_timestamp_ns,
                self.contact_observation_tick,
                1,
                0,
                self.observed_contact_active,
                self.contact_debounced,
                self.contact_active[0],
                self.contact_observation_diagnostics,
            )

    def solve(
        self,
        root_position: np.ndarray,
        root_quaternion: np.ndarray,
        root_twist: np.ndarray,
        q: np.ndarray,
        v: np.ndarray,
        ground_position: float,
        ground_height: float,
        observed_contact_active: np.ndarray | None = None,
        observed_wheel_normal_force_n: np.ndarray | None = None,
        observed_contact_available: bool = True,
        observed_contact_age_ticks: int = 0,
        observed_contact_synchronization_uncertainty_ns: int = 0,
    ) -> dict[str, float | np.ndarray]:
        if observed_contact_age_ticks < 0:
            raise ValueError("observed contact age ticks must be nonnegative")
        if observed_contact_synchronization_uncertainty_ns < 0:
            raise ValueError(
                "observed contact synchronization uncertainty must be nonnegative"
            )
        if observed_wheel_normal_force_n is not None and (
            observed_wheel_normal_force_n.shape != (2,)
            or not np.all(np.isfinite(observed_wheel_normal_force_n))
            or np.any(observed_wheel_normal_force_n < 0.0)
        ):
            raise ValueError(
                "observed_wheel_normal_force_n must contain two finite nonnegative values"
            )
        if observed_contact_active is None:
            if not observed_contact_available or observed_contact_age_ticks:
                raise ValueError(
                    "contact availability/age requires an observed contact mask"
                )
            self.contact_active.fill(1)
            self.observed_contact_active.fill(1)
            self.contact_debounced.fill(1)
            self.contact_observation_diagnostics.fill(0)
        else:
            if observed_contact_active.shape != (2,):
                raise ValueError("observed_contact_active must have shape (2,)")
            np.copyto(self.observed_contact_active, observed_contact_active, casting="unsafe")
            self.contact_observation_tick += 1
            timestamp_ns = self.contact_observation_tick * int(self.control_dt * 1.0e9)
            sample_age_ns = observed_contact_age_ticks * int(self.control_dt * 1.0e9)
            mapped_timestamp_ns = max(timestamp_ns - sample_age_ns, 0)
            self.balance.step_contact_observation_from_mask(
                timestamp_ns,
                observed_contact_available,
                mapped_timestamp_ns,
                self.contact_observation_tick,
                1,
                observed_contact_synchronization_uncertainty_ns,
                self.observed_contact_active,
                self.contact_debounced,
                self.contact_active[0],
                self.contact_observation_diagnostics,
            )
        observation_exact = bool(
            observed_contact_active is not None
            and self.contact_observation_diagnostics[
                self.contact_observation_index["accepted"]
            ]
            == 1
            and self.contact_observation_diagnostics[
                self.contact_observation_index["provenance"]
            ]
            == 0
        )
        if self.fall_safe_enabled:
            self.balance.step_fall_safe_from_state(
                self.control_dt,
                root_position,
                root_quaternion,
                root_twist,
                v,
                self.previous_solver_admitted or not self.has_admitted_command,
                self.fall_safe_diagnostics,
                self.contingency_root_angular_acceleration,
                self.contingency_root_acceleration,
                self.contingency_joint_acceleration,
            )
        else:
            self.fall_safe_diagnostics.fill(0.0)
            self.fall_safe_diagnostics[self.fall_safe_index["mode"]] = 0.0
            self.fall_safe_diagnostics[
                self.fall_safe_index["primary_authority"]
            ] = 1.0
            self.fall_safe_diagnostics[
                self.fall_safe_index["fresh_command_authority"]
            ] = 1.0
        self.root_position[0] = root_position
        self.root_quaternion[0] = root_quaternion
        self.root_angular_velocity[0] = root_twist[:3]
        self.root_velocity[0] = root_twist[3:]
        rotation_error = quaternion_rotation_vector(root_quaternion)
        self.execution_residual_state[:] = (
            rotation_error[0],
            root_twist[0],
            rotation_error[1],
            root_twist[1],
            root_position[1] - self.nominal_root_position[1],
            root_twist[4],
            rotation_error[2],
            root_twist[2],
        )
        execution_support_mask = int(self.observed_contact_active[0]) | (
            int(self.observed_contact_active[1]) << 1
        )
        execution_prediction_available = bool(
            observation_exact
            and self.execution_residual_prediction_valid
            and self.execution_residual_prediction_support_mask
            == execution_support_mask
        )
        if self.execution_residual_veto_enabled:
            self.execution_residual_tick += 1
            (
                self.execution_residual_step_ns,
                self.execution_residual_allocation_calls,
                self.execution_residual_allocated_bytes,
            ) = self.balance.step_viability_execution_monitor(
                self.execution_residual_tick,
                observation_exact,
                execution_prediction_available,
                execution_support_mask,
                self.execution_residual_prediction,
                self.execution_residual_state,
                self.execution_residual_scale,
                self.execution_residual_diagnostics,
            )
        else:
            self.execution_residual_diagnostics.fill(0.0)
            self.execution_residual_step_ns = 0
            self.execution_residual_allocation_calls = 0
            self.execution_residual_allocated_bytes = 0
        execution_residual_veto_active = bool(
            self.execution_residual_veto_enabled
            and self.execution_residual_diagnostics[
                self.execution_residual_index["status"]
            ]
            == 2.0
        )
        if self.support_load_guard_enabled and observed_wheel_normal_force_n is not None:
            total_load = float(np.sum(observed_wheel_normal_force_n))
            minimum_load_fraction = (
                float(np.min(observed_wheel_normal_force_n)) / total_load
                if total_load > 1.0e-9
                else 0.0
            )
            if not self.support_load_guard_active:
                self.support_load_guard_active = bool(
                    observation_exact
                    and np.all(self.observed_contact_active != 0)
                    and total_load > 1.0
                    and minimum_load_fraction < 0.45
                )
            if self.support_load_guard_active:
                releasable = bool(
                    observation_exact
                    and np.all(self.observed_contact_active != 0)
                    and minimum_load_fraction >= 0.47
                    and abs(float(root_twist[0])) <= 0.05
                )
                self.support_load_guard_release_ticks = (
                    self.support_load_guard_release_ticks + 1 if releasable else 0
                )
                if self.support_load_guard_release_ticks >= 25:
                    self.support_load_guard_active = False
                    self.support_load_guard_release_ticks = 0
        guard_authority_target = 1.0 if self.support_load_guard_active else 0.0
        guard_authority_delta = np.clip(
            guard_authority_target - self.support_load_guard_authority,
            -0.04,
            0.20,
        )
        self.support_load_guard_authority += float(guard_authority_delta)
        guarded_roll_stiffness = (
            (1.0 - self.support_load_guard_authority) * self.root_roll_stiffness
            + self.support_load_guard_authority * 12.0
        )
        guarded_roll_damping = (
            (1.0 - self.support_load_guard_authority) * self.root_roll_damping
            + self.support_load_guard_authority * 20.0
        )
        self.root_angular_acceleration[0] = -24.0 * rotation_error - 4.4 * root_twist[:3]
        self.root_angular_acceleration[0, 0] = (
            -guarded_roll_stiffness * rotation_error[0]
            - guarded_roll_damping * root_twist[0]
        )
        baseline_yaw_acceleration = self.root_angular_acceleration[0, 2]
        position_error = self.nominal_root_position - root_position
        self.root_acceleration[0] = np.asarray(
            [
                10.0 * position_error[0] - 6.0 * root_twist[3],
                self.root_lateral_stiffness * position_error[1]
                - self.root_lateral_damping * root_twist[4],
                90.0 * position_error[2] - 18.0 * root_twist[5],
            ]
        )
        self.q[0] = q
        self.v[0] = v
        self.joint_acceleration[0] = (
            60.0 * (self.nominal_joint_position - q) - 12.0 * v
        )
        self.viability_lateral_delta_x = 0.0
        self.viability_lateral_delta_y = 0.0
        self.viability_bank_delta_x = 0.0
        self.viability_bank_delta_y = 0.0
        self.viability_verified_scale = 1.0
        self.viability_verification_queries = 1
        if self.balance_mode in (
            "planar_capture",
            "viability_capture",
            "viability_verified",
            "viability_support_capture",
            "viability_coordinate",
        ):
            self.balance.step_planar_capture_from_state(
                self.control_dt,
                self.target_ground_position,
                ground_height,
                self.zero_translation,
                self.identity_quaternion,
                self.zero_translation,
                self.identity_quaternion,
                root_position,
                root_quaternion,
                root_twist,
                q,
                v,
                self.wheel_acceleration,
                self.capture_diagnostics,
            )
            self.virtual_pitch[0] = self.capture_diagnostics[
                self.capture_index["virtual_pitch_rad"]
            ]
            heading = self.capture_diagnostics[self.capture_index["heading_world_rad"]]
            cosine = math.cos(heading)
            sine = math.sin(heading)
            steering_pressure = self.capture_diagnostics[
                self.capture_index["lateral_capture_pressure"]
            ]
            if steering_pressure > 0.0:
                self.contact_bases_world[:, :, 0] = (cosine, sine, 0.0)
                self.contact_bases_world[:, :, 1] = (-sine, cosine, 0.0)
                self.contact_bases_world[:, :, 2] = (0.0, 0.0, 1.0)
            else:
                self.contact_bases_world.fill(0.0)
                self.contact_bases_world[:, :, 0, 0] = 1.0
                self.contact_bases_world[:, :, 1, 1] = 1.0
                self.contact_bases_world[:, :, 2, 2] = 1.0
            steering_yaw_acceleration = self.capture_diagnostics[
                self.capture_index["commanded_yaw_acceleration_rad_s2"]
            ]
            self.root_angular_acceleration[0, 2] = (
                (1.0 - steering_pressure) * baseline_yaw_acceleration
                + steering_pressure * steering_yaw_acceleration
            )
            if self.balance_mode in (
                "viability_capture",
                "viability_verified",
                "viability_support_capture",
            ):
                viability_pressure = self.capture_diagnostics[
                    self.capture_index["viability_activation_pressure"]
                ]
                baseline_lateral_acceleration = (
                    -sine * self.root_acceleration[0, 0]
                    + cosine * self.root_acceleration[0, 1]
                )
                commanded_lateral_acceleration = self.capture_diagnostics[
                    self.capture_index["commanded_lateral_acceleration_m_s2"]
                ]
                lateral_delta = viability_pressure * (
                    commanded_lateral_acceleration - baseline_lateral_acceleration
                )
                self.viability_lateral_delta_x = -lateral_delta * sine
                self.viability_lateral_delta_y = lateral_delta * cosine
                baseline_bank_acceleration = (
                    cosine * self.root_angular_acceleration[0, 0]
                    + sine * self.root_angular_acceleration[0, 1]
                )
                commanded_roll_acceleration = self.capture_diagnostics[
                    self.capture_index["commanded_roll_acceleration_rad_s2"]
                ]
                bank_delta = viability_pressure * (
                    commanded_roll_acceleration - baseline_bank_acceleration
                )
                self.viability_bank_delta_x = bank_delta * cosine
                self.viability_bank_delta_y = bank_delta * sine
                if self.balance_mode in (
                    "viability_capture",
                    "viability_support_capture",
                ):
                    self.root_acceleration[0, 0] += self.viability_lateral_delta_x
                    self.root_acceleration[0, 1] += self.viability_lateral_delta_y
                    self.root_angular_acceleration[
                        0, 0
                    ] += self.viability_bank_delta_x
                    self.root_angular_acceleration[
                        0, 1
                    ] += self.viability_bank_delta_y
                if self.balance_mode == "viability_support_capture":
                    height = max(root_position[2], 0.05)
                    omega = math.sqrt(9.81 / height)
                    roll_capture = rotation_error[0] + root_twist[0] / omega
                    lateral_capture = (
                        root_position[1]
                        - self.nominal_root_position[1]
                        + root_twist[4] / omega
                    )
                    self.viability_support_pressure = max(
                        abs(roll_capture) / math.radians(45.0),
                        abs(lateral_capture) / 0.10,
                    )
                    if self.viability_support_active:
                        self.viability_support_active = (
                            self.viability_support_pressure > 0.05
                        )
                    else:
                        self.viability_support_active = (
                            self.viability_support_pressure >= 0.10
                        )
        elif self.balance_mode == "capture":
            self.balance.step_rooted_capture_from_state(
                self.control_dt,
                self.target_ground_position,
                ground_height,
                self.zero_translation,
                self.identity_quaternion,
                self.zero_translation,
                self.identity_quaternion,
                root_position,
                root_quaternion,
                root_twist,
                q,
                v,
                self.wheel_acceleration,
                self.capture_diagnostics,
            )
            self.virtual_pitch[0] = self.capture_diagnostics[
                self.capture_index["virtual_pitch_rad"]
            ]
        else:
            self.balance.step_from_state(
                self.control_dt,
                self.target_ground_position,
                ground_position,
                ground_height,
                root_position,
                root_quaternion,
                q,
                v,
                self.wheel_acceleration,
                self.virtual_pitch,
            )
            self.capture_diagnostics.fill(0.0)
            self.capture_diagnostics[self.capture_index["station_authority"]] = 1.0
        self.joint_acceleration[0, ROLLING_COORDINATES] = self.wheel_acceleration
        primary_authority = self.fall_safe_diagnostics[
            self.fall_safe_index["primary_authority"]
        ]
        if self.fall_safe_enabled and self.fall_safe_primary_blend:
            contingency_authority = 1.0 - primary_authority
            self.root_acceleration *= primary_authority
            np.multiply(
                self.contingency_root_acceleration,
                contingency_authority,
                out=self.contingency_root_blend,
            )
            self.root_acceleration[0] += self.contingency_root_blend
            self.root_angular_acceleration *= primary_authority
            np.multiply(
                self.contingency_root_angular_acceleration,
                contingency_authority,
                out=self.contingency_root_angular_blend,
            )
            self.root_angular_acceleration[0] += self.contingency_root_angular_blend
            self.joint_acceleration *= primary_authority
            np.multiply(
                self.contingency_joint_acceleration,
                contingency_authority,
                out=self.contingency_blend[0],
            )
            self.joint_acceleration += self.contingency_blend
        coordinate_step_ns = 0
        coordinate_allocation_calls = 0
        coordinate_allocated_bytes = 0
        self.viability_coordinate_queries = 0
        self.viability_planner_query_count = 0
        self.viability_planner_step_ns = 0
        self.viability_planner_feasibility_seed_reuses = 0
        self.viability_planner_feasibility_projection_sweeps = 0
        self.viability_planner_feasibility_halfspace_projections = 0
        self.viability_hard_feasibility_witness_transferred = False
        self.final_feasibility_seed_reused = False
        self.final_feasibility_prefix_resumed = False
        self.viability_hard_feasibility_witness_transfer_diagnostics.fill(0)
        self.viability_forecast_path.fill(0.0)
        self.viability_forecast_path_valid = False
        if self.viability_planner_enabled:
            self.viability_coordinate_tick += 1
            was_active = bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["active"]
                ]
            )
            if self.viability_support_requires_active_request:
                rotation = quaternion_rotation_vector(root_quaternion)
                height = max(float(root_position[2]), 0.05)
                omega = math.sqrt(9.81 / height)
                physical_pressure = max(
                    abs(float(rotation[0]) + float(root_twist[0]) / omega)
                    / self.viability_planner.ROLL_BOUND_RAD,
                    abs(
                        float(root_position[1] - self.nominal_root_position[1])
                        + float(root_twist[4]) / omega
                    )
                    / self.viability_planner.LATERAL_CAPTURE_BOUND_M,
                )
                if (
                    not was_active
                    and physical_pressure < self.viability_planner.ACTIVATION_PRESSURE
                ):
                    self.contact_active.fill(1)
            planner_update = (
                (self.viability_coordinate_tick - 1)
                % self.viability_planner_update_period_ticks
                == 0
            )
            pressure = 0.0
            candidate_available = False
            hybrid_guard_execution_permitted = True
            self.viability_roll_capture_pressure = 0.0
            self.viability_candidate.fill(0.0)
            self.viability_candidate_normal_force.fill(0.0)
            planner_previous_request = self.viability_request
            if (
                self.viability_confirmation_enabled
                and self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["has_shadow"]
                ]
                == 1.0
            ):
                planner_previous_request = self.viability_confirmation_shadow
            if planner_update and observation_exact:
                candidate, pressure, candidate_available = self.viability_planner.plan(
                    root_position,
                    root_quaternion,
                    root_twist,
                    q,
                    v,
                    self.root_acceleration[0],
                    self.root_angular_acceleration[0],
                    self.joint_acceleration[0],
                    self.contact_active[0],
                    self.contact_bases_world[0],
                    was_active,
                    planner_previous_request,
                )
                self.viability_candidate[:] = candidate
                if candidate_available and self.viability_hybrid_guard_enabled:
                    forecast_state = self.viability_planner.forecast_state
                    omega = math.sqrt(9.81 / max(float(forecast_state[8]), 0.05))
                    self.viability_roll_capture_pressure = float(
                        (forecast_state[0] + forecast_state[1] / omega)
                        / self.viability_planner.ROLL_BOUND_RAD
                    )
                if self.viability_planner_strategy != "coordinate":
                    self.viability_candidate_normal_force[:] = (
                        self.viability_planner.candidate_normal_force
                    )
                coordinate_step_ns = self.viability_planner.step_ns
                coordinate_allocation_calls = self.viability_planner.allocation_calls
                coordinate_allocated_bytes = self.viability_planner.allocated_bytes
                self.viability_coordinate_queries = self.viability_planner.query_count
                self.viability_planner_query_count = self.viability_planner.query_count
                self.viability_planner_step_ns = self.viability_planner.step_ns
                self.viability_planner_feasibility_seed_reuses = (
                    getattr(self.viability_planner, "feasibility_seed_reuses", 0)
                )
                self.viability_planner_feasibility_projection_sweeps = (
                    getattr(
                        self.viability_planner,
                        "feasibility_projection_sweeps",
                        0,
                    )
                )
                self.viability_planner_feasibility_halfspace_projections = (
                    getattr(
                        self.viability_planner,
                        "feasibility_halfspace_projections",
                        0,
                    )
                )
                self.viability_planner_zero_pressure = (
                    self.viability_planner.zero_pressure
                )
                self.viability_planner_candidate_pressure = (
                    self.viability_planner.candidate_pressure
                    if math.isfinite(self.viability_planner.candidate_pressure)
                    else 0.0
                )
                self.viability_coordinate_score = (
                    self.viability_planner_candidate_pressure
                )
                if self.viability_planner_transfer_hard_feasibility_witness:
                    self.viability_hard_feasibility_witness_transferred = bool(
                        self.session.import_hard_feasibility_witness_from(
                            self.viability_planner.session,
                            self.viability_hard_feasibility_witness_transfer_diagnostics,
                        )
                    )
                    coordinate_step_ns += int(
                        self.viability_hard_feasibility_witness_transfer_diagnostics[0]
                    )
                    coordinate_allocation_calls += int(
                        self.viability_hard_feasibility_witness_transfer_diagnostics[1]
                    )
                    coordinate_allocated_bytes += int(
                        self.viability_hard_feasibility_witness_transfer_diagnostics[2]
                    )
                if self.viability_planner_strategy != "coordinate":
                    self.viability_forecast_diagnostics[:] = (
                        self.viability_planner.candidate_forecast
                    )
                    self.viability_forecast_path_valid = bool(
                        candidate_available
                        and self.viability_planner.candidate_forecast_path_valid
                    )
                    if self.viability_forecast_path_valid:
                        self.viability_forecast_path[:] = (
                            self.viability_planner.candidate_forecast_path
                        )
            if self.viability_hybrid_guard_enabled:
                support_mask = int(self.observed_contact_active[0]) | (
                    int(self.observed_contact_active[1]) << 1
                )
                guard_ns, guard_allocations, guard_bytes = (
                    self.balance.step_viability_hybrid_guard(
                        self.viability_coordinate_tick,
                        observation_exact,
                        candidate_available,
                        support_mask,
                        self.viability_roll_capture_pressure,
                        self.viability_candidate,
                        self.viability_candidate_normal_force,
                        self.viability_hybrid_guard_diagnostics,
                    )
                )
                coordinate_step_ns += int(guard_ns)
                coordinate_allocation_calls += int(guard_allocations)
                coordinate_allocated_bytes += int(guard_bytes)
                hybrid_guard_execution_permitted = bool(
                    self.viability_hybrid_guard_diagnostics[
                        self.viability_hybrid_guard_index["executable"]
                    ]
                )
                candidate_available = bool(
                    self.viability_hybrid_guard_diagnostics[
                        self.viability_hybrid_guard_index["shadow_admissible"]
                    ]
                )
            if execution_residual_veto_active:
                candidate_available = False
            if self.viability_confirmation_enabled:
                support_mask = int(self.observed_contact_active[0]) | (
                    int(self.observed_contact_active[1]) << 1
                )
                confirmation_ns, confirmation_allocations, confirmation_bytes = (
                    self.balance.step_viability_confirmation(
                        self.viability_coordinate_tick,
                        observation_exact,
                        planner_update,
                        support_mask,
                        candidate_available,
                        self.viability_planner_zero_pressure,
                        self.viability_planner_candidate_pressure,
                        self.viability_candidate,
                        self.viability_confirmation_candidate,
                        self.viability_confirmation_diagnostics,
                    )
                )
                coordinate_step_ns += int(confirmation_ns)
                coordinate_allocation_calls += int(confirmation_allocations)
                coordinate_allocated_bytes += int(confirmation_bytes)
                self.viability_confirmation_shadow[:] = (
                    self.viability_confirmation_diagnostics[
                        self.viability_confirmation_index[
                            "shadow_roll_acceleration"
                        ]
                    ],
                    self.viability_confirmation_diagnostics[
                        self.viability_confirmation_index[
                            "shadow_lateral_acceleration"
                        ]
                    ],
                    self.viability_confirmation_diagnostics[
                        self.viability_confirmation_index[
                            "shadow_yaw_acceleration"
                        ]
                    ],
                )
                self.viability_candidate[:] = self.viability_confirmation_candidate
                candidate_available = bool(
                    self.viability_confirmation_diagnostics[
                        self.viability_confirmation_index["executable"]
                    ]
                    and hybrid_guard_execution_permitted
                )
            self.balance.step_viability_request(
                self.viability_coordinate_tick,
                observation_exact,
                planner_update,
                pressure,
                candidate_available,
                self.viability_candidate,
                self.viability_request,
                self.viability_request_diagnostics,
            )
            self.viability_support_active = bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["active"]
                ]
            )
            self.viability_support_pressure = float(
                self.viability_request_diagnostics[
                    self.viability_request_index["last_pressure"]
                ]
            )
            self.viability_coordinate_target[:] = (
                self.viability_request_diagnostics[
                    self.viability_request_index["target_roll_acceleration"]
                ],
                self.viability_request_diagnostics[
                    self.viability_request_index["target_lateral_acceleration"]
                ],
                self.viability_request_diagnostics[
                    self.viability_request_index["target_yaw_acceleration"]
                ],
            )
            self.viability_coordinate_request[:] = self.viability_request
            request_executable = bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["executable"]
                ]
            )
            if request_executable:
                self.root_angular_acceleration[0, 0] += (
                    primary_authority * self.viability_request[0]
                )
                self.root_acceleration[0, 1] += (
                    primary_authority * self.viability_request[1]
                )
                self.root_angular_acceleration[0, 2] += (
                    primary_authority * self.viability_request[2]
                )
            elif self.viability_support_requires_active_request:
                self.contact_active.fill(1)
        else:
            self.viability_request.fill(0.0)
            self.viability_request_diagnostics.fill(0.0)
            self.viability_confirmation_candidate.fill(0.0)
            self.viability_confirmation_shadow.fill(0.0)
            self.viability_confirmation_diagnostics.fill(0.0)
            self.viability_hybrid_guard_diagnostics.fill(0.0)
            self.viability_candidate_normal_force.fill(0.0)
            self.viability_forecast_path.fill(0.0)
            self.viability_forecast_path_valid = False
            self.viability_coordinate_target.fill(0.0)
            self.viability_coordinate_request.fill(0.0)
        if self.support_contingency_enabled:
            self.support_contingency_realization_fallback = False
            self.support_contingency_support_mask = int(
                self.contact_active[0, 0]
            ) | (int(self.contact_active[0, 1]) << 1)
            self.support_contingency_requested = bool(
                observation_exact
                and self.support_contingency_armed
                and self.support_contingency_support_mask != 3
                and (
                    not self.support_contingency_flight_only
                    or self.support_contingency_support_mask == 0
                )
            )
            if (
                self.support_contingency_requested
                or self.support_contingency_query_every_tick
            ) and not (
                self.support_contingency_project_primary
                or self.support_contingency_realize_primary_torque
            ):
                self._run_support_contingency_query(observation_exact)
            else:
                self.support_contingency_diagnostics.fill(0.0)
                self.support_contingency_diagnostics[0] = (
                    0.0
                    if self.support_contingency_support_mask == 3
                    else 1.0
                    if self.support_contingency_support_mask in (1, 2)
                    else 2.0
                )
                self.support_contingency_diagnostics[1] = (
                    self.support_contingency_support_mask
                )
                self.support_contingency_author_step_ns = 0
                self.support_contingency_author_allocation_calls = 0
                self.support_contingency_author_allocated_bytes = 0
                if self.support_contingency_out is not None:
                    self.support_contingency_candidate_generalized_acceleration.fill(0.0)
                    self.support_contingency_candidate_torque.fill(0.0)
                    self.support_contingency_out["step_ns"].fill(0)
                    self.support_contingency_out["allocation_calls"].fill(0)
                    self.support_contingency_out["allocated_bytes"].fill(0)
                    self.support_contingency_out["status"].fill(3)
                    self.support_contingency_out[
                        "maximum_constraint_violation"
                    ].fill(0.0)
            self.contact_program_hard_contact[:] = self.contact_active[0]
            if (
                self.support_contingency_preserve_primary_support
                or self.contact_program_authority_enabled
            ):
                self.contact_active.fill(1)
        else:
            self.support_contingency_diagnostics.fill(0.0)
            self.support_contingency_author_step_ns = 0
            self.support_contingency_author_allocation_calls = 0
            self.support_contingency_author_allocated_bytes = 0
            self.support_contingency_support_mask = 3
            self.support_contingency_requested = False
            self.contact_program_hard_contact[:] = self.contact_active[0]
        out = self.out
        base_root_x = self.root_acceleration[0, 0]
        base_root_y = self.root_acceleration[0, 1]
        base_bank_x = self.root_angular_acceleration[0, 0]
        base_bank_y = self.root_angular_acceleration[0, 1]
        verification_scales = (
            self.viability_verification_scales
            if self.balance_mode == "viability_verified"
            else (1.0,)
        )
        accumulated_step_ns = coordinate_step_ns
        accumulated_allocation_calls = coordinate_allocation_calls
        accumulated_allocated_bytes = coordinate_allocated_bytes
        for query_index, verification_scale in enumerate(verification_scales, 1):
            if self.balance_mode == "viability_verified":
                effective_scale = verification_scale * primary_authority
                self.root_acceleration[0, 0] = (
                    base_root_x + effective_scale * self.viability_lateral_delta_x
                )
                self.root_acceleration[0, 1] = (
                    base_root_y + effective_scale * self.viability_lateral_delta_y
                )
                self.root_angular_acceleration[0, 0] = (
                    base_bank_x + effective_scale * self.viability_bank_delta_x
                )
                self.root_angular_acceleration[0, 1] = (
                    base_bank_y + effective_scale * self.viability_bank_delta_y
                )
            self._run_wbc_query()
            accumulated_step_ns += int(out["step_ns"][0])
            accumulated_allocation_calls += int(out["allocation_calls"][0])
            accumulated_allocated_bytes += int(out["allocated_bytes"][0])
            self.viability_verified_scale = verification_scale
            self.viability_verification_queries = query_index
            if int(out["status"][0]) in (0, 1):
                break
        raw_primary_status = int(out["status"][0])
        primary_wbc_step_ns = accumulated_step_ns - coordinate_step_ns
        primary_task_pseudoinverse_calls = int(
            out["task_pseudoinverse_calls"][0]
        )
        primary_clipped_steps = int(out["clipped_steps"][0])
        primary_task_jacobi_sweeps = int(out["task_jacobi_sweeps"][0])
        primary_feasibility_projection_sweeps = int(
            out["feasibility_projection_sweeps"][0]
        )
        primary_feasibility_halfspace_projections = int(
            out["feasibility_halfspace_projections"][0]
        )
        primary_fresh_command_authority = float(
            self.fall_safe_diagnostics[
                self.fall_safe_index["fresh_command_authority"]
            ]
        )
        self.primary_program_source_fresh = raw_primary_status in (0, 1)
        if self.primary_program_source_fresh:
            self.primary_program_torque[0] = out["actuator_torque"][0]
            self.primary_program_available = True
        elif self.has_admitted_command and primary_fresh_command_authority > 0.0:
            np.multiply(
                self.last_admitted_torque,
                primary_fresh_command_authority,
                out=self.primary_program_torque[0],
            )
            self.primary_program_available = True
        else:
            self.primary_program_torque.fill(0.0)
            self.primary_program_available = False
        if (
            (
                self.support_contingency_project_primary
                or self.support_contingency_realize_primary_torque
            )
            and self.support_contingency_requested
        ):
            self.contact_active[0] = self.contact_program_hard_contact
            self._run_support_contingency_query(
                observation_exact,
                project_primary=self.support_contingency_project_primary,
                realize_primary_torque=(
                    self.support_contingency_realize_primary_torque
                ),
            )
            self.contact_active.fill(1)
        self.support_free_inexact_brake_available = False
        self.inexact_observation_terminal_zero_effort_available = False
        self.inexact_observation_terminal_zero_effort_acceleration.fill(0.0)
        self.inexact_observation_terminal_zero_effort_step_ns = 0
        self.inexact_observation_terminal_zero_effort_allocation_calls = 0
        self.inexact_observation_terminal_zero_effort_allocated_bytes = 0
        self.inexact_observation_terminal_hypothesis_acceleration.fill(0.0)
        self.inexact_observation_terminal_hypothesis_available.fill(0)
        self.inexact_observation_terminal_hypothesis_diagnostics.fill(0.0)
        self.inexact_observation_terminal_hypothesis_envelopes.fill(0.0)
        self.inexact_observation_terminal_hypothesis_query_step_ns = 0
        self.inexact_observation_terminal_hypothesis_query_allocation_calls = 0
        self.inexact_observation_terminal_hypothesis_query_allocated_bytes = 0
        self.inexact_observation_terminal_hypothesis_aggregate_step_ns = 0
        self.inexact_observation_terminal_hypothesis_aggregate_allocation_calls = 0
        self.inexact_observation_terminal_hypothesis_aggregate_allocated_bytes = 0
        if (
            self.contact_program_inexact_support_free_brake
            or self.contact_program_inexact_terminal_chooser
        ) and not observation_exact:
            self.contact_active.fill(0)
            if self.contact_program_inexact_terminal_zero_effort_baseline:
                self._run_support_contingency_query(
                    False,
                    fixed_actuator_effort=(
                        self.inexact_observation_terminal_zero_effort
                    ),
                )
                self.inexact_observation_terminal_zero_effort_acceleration[:] = (
                    self.support_contingency_candidate_generalized_acceleration
                )
                self.inexact_observation_terminal_zero_effort_available = bool(
                    self.support_contingency_out is not None
                    and int(self.support_contingency_out["status"][0]) in (0, 1)
                    and float(
                        self.support_contingency_out[
                            "maximum_constraint_violation"
                        ][0]
                    )
                    < 1.0e-8
                )
                self.inexact_observation_terminal_zero_effort_step_ns = (
                    self.support_contingency_author_step_ns
                    + int(self.support_contingency_out["step_ns"][0])
                )
                self.inexact_observation_terminal_zero_effort_allocation_calls = (
                    self.support_contingency_author_allocation_calls
                    + int(self.support_contingency_out["allocation_calls"][0])
                )
                self.inexact_observation_terminal_zero_effort_allocated_bytes = (
                    self.support_contingency_author_allocated_bytes
                    + int(self.support_contingency_out["allocated_bytes"][0])
                )
            self._run_support_contingency_query(False)
            self.contact_active.fill(1)
            self.support_free_inexact_brake_available = bool(
                self.support_contingency_out is not None
                and int(self.support_contingency_out["status"][0]) in (0, 1)
                and float(
                    self.support_contingency_out[
                        "maximum_constraint_violation"
                    ][0]
                )
                < 1.0e-8
            )
            if self.contact_program_inexact_terminal_support_hypothesis_envelope:
                previous_command_age = int(
                    self.contact_program_authority_diagnostics[
                        self.contact_program_authority_index["command_age_ticks"]
                    ]
                )
                retained_available = bool(
                    self.contact_program_authority_has_primary_history
                    and self.contact_program_inexact_hold_ticks > 0
                    and previous_command_age
                    < self.contact_program_inexact_hold_ticks
                )
                self._run_terminal_support_hypothesis_queries(retained_available)
        out["step_ns"][0] = accumulated_step_ns
        out["allocation_calls"][0] = accumulated_allocation_calls
        out["allocated_bytes"][0] = accumulated_allocated_bytes
        if self.support_contingency_out is not None:
            accumulated_step_ns += self.support_contingency_author_step_ns + int(
                self.support_contingency_out["step_ns"][0]
            )
            accumulated_allocation_calls += (
                self.support_contingency_author_allocation_calls
                + int(self.support_contingency_out["allocation_calls"][0])
            )
            accumulated_allocated_bytes += (
                self.support_contingency_author_allocated_bytes
                + int(self.support_contingency_out["allocated_bytes"][0])
            )
        accumulated_step_ns += self.inexact_observation_terminal_zero_effort_step_ns
        accumulated_allocation_calls += (
            self.inexact_observation_terminal_zero_effort_allocation_calls
        )
        accumulated_allocated_bytes += (
            self.inexact_observation_terminal_zero_effort_allocated_bytes
        )
        accumulated_step_ns += (
            self.inexact_observation_terminal_hypothesis_query_step_ns
        )
        accumulated_allocation_calls += (
            self.inexact_observation_terminal_hypothesis_query_allocation_calls
        )
        accumulated_allocated_bytes += (
            self.inexact_observation_terminal_hypothesis_query_allocated_bytes
        )
        self.final_feasibility_seed_reused = bool(out["feasibility_seed_reused"][0])
        self.final_feasibility_prefix_resumed = bool(
            out["feasibility_prefix_resumed"][0]
        )
        raw_status = raw_primary_status
        support_program_admitted = bool(
            observed_contact_active is None
            or self.execute_reduced_support
            or (self.contact_active[0, 0] != 0 and self.contact_active[0, 1] != 0)
        )
        primary_status = raw_status if support_program_admitted else 4
        support_mask = self.support_contingency_support_mask
        self.support_contingency_admitted = bool(
            self.support_contingency_requested
            and
            observation_exact
            and self.support_contingency_out is not None
            and int(self.support_contingency_out["status"][0]) in (0, 1)
            and float(
                self.support_contingency_out["maximum_constraint_violation"][0]
            )
            < 1.0e-8
            and not (
                self.support_contingency_realize_primary_torque
                and self.support_contingency_realization_fallback
            )
        )
        self.support_contingency_candidate_power_w = (
            float(
                np.dot(
                    self.support_contingency_out["actuator_torque"][0],
                    self.v[0],
                )
            )
            if self.support_contingency_requested
            and self.support_contingency_out is not None
            else 0.0
        )
        self.support_contingency_primary_power_w = (
            float(np.dot(out["actuator_torque"][0], self.v[0]))
            if self.support_contingency_requested
            else 0.0
        )
        self.support_contingency_incremental_power_w = (
            self.support_contingency_candidate_power_w
            - self.support_contingency_primary_power_w
        )
        self.support_contingency_forecast_guard_passed = True
        self.support_contingency_forecast_baseline_score = 0.0
        self.support_contingency_forecast_candidate_score = 0.0
        self.support_contingency_forecast_step_ns = 0
        self.support_contingency_forecast_allocation_calls = 0
        self.support_contingency_forecast_allocated_bytes = 0
        if (
            self.support_contingency_forecast_guard
            and self.support_contingency_requested
            and self.support_contingency_out is not None
        ):
            self.support_contingency_forecast_state[:] = (
                rotation_error[0],
                root_twist[0],
                rotation_error[1],
                root_twist[1],
                root_position[1] - self.nominal_root_position[1],
                root_twist[4],
                rotation_error[2],
                root_twist[2],
                max(float(root_position[2]), 0.05),
                support_mask,
            )
            self.support_contingency_forecast_achieved.fill(0.0)
            candidate_acceleration = self.support_contingency_candidate_generalized_acceleration
            self.support_contingency_forecast_achieved[1] = (
                candidate_acceleration[0],
                candidate_acceleration[4],
                candidate_acceleration[2],
                candidate_acceleration[1],
            )
            candidate_torque_utilization = float(
                self.support_contingency_out["maximum_torque_utilization"][0]
            )
            candidate_joint_headroom = float(
                self.support_contingency_out["minimum_joint_headroom_fraction"][0]
            )
            self.support_contingency_forecast_torque_utilization.fill(
                candidate_torque_utilization
            )
            self.support_contingency_forecast_joint_headroom.fill(
                candidate_joint_headroom
            )
            (
                self.support_contingency_forecast_step_ns,
                self.support_contingency_forecast_allocation_calls,
                self.support_contingency_forecast_allocated_bytes,
            ) = self.balance.score_viability_forecast_batch(
                self.support_contingency_forecast_state,
                self.support_contingency_forecast_achieved,
                self.support_contingency_forecast_requests,
                self.support_contingency_forecast_previous_request,
                self.support_contingency_forecast_maximum_request,
                self.support_contingency_forecast_torque_utilization,
                self.support_contingency_forecast_joint_headroom,
                self.support_contingency_forecast_diagnostics,
            )
            self.support_contingency_forecast_baseline_score = float(
                self.support_contingency_forecast_diagnostics[0, 0]
            )
            self.support_contingency_forecast_candidate_score = float(
                self.support_contingency_forecast_diagnostics[1, 0]
            )
            angular_acceleration_within_trust_region = bool(
                np.max(np.abs(candidate_acceleration[:3])) <= 40.0
            )
            self.support_contingency_forecast_guard_passed = bool(
                angular_acceleration_within_trust_region
                and self.support_contingency_forecast_candidate_score
                <= self.support_contingency_forecast_baseline_score - 0.10
            )
            accumulated_step_ns += self.support_contingency_forecast_step_ns
            accumulated_allocation_calls += (
                self.support_contingency_forecast_allocation_calls
            )
            accumulated_allocated_bytes += (
                self.support_contingency_forecast_allocated_bytes
            )
        if (
            observation_exact
            and support_mask == 3
            and primary_status in (0, 1)
            and not (
                self.support_contingency_single_evaluation_lease
                and self.support_contingency_consumed
            )
        ):
            self.support_contingency_armed = True
        self.support_contingency_selected = bool(
            not self.contact_program_authority_enabled
            and
            self.support_contingency_execute
            and self.support_contingency_requested
            and self.support_contingency_admitted
            and self.support_contingency_forecast_guard_passed
        )
        if (
            self.support_contingency_requested
            and self.support_contingency_single_evaluation_lease
        ):
            self.support_contingency_armed = False
            self.support_contingency_consumed = True
        command_out = (
            self.support_contingency_out
            if self.support_contingency_selected
            else out
        )
        if command_out is None:
            raise RuntimeError("support contingency selection has no WBC output")
        program_executable = False
        program_selection = 0
        if self.contact_program_authority_enabled:
            self.contact_program_authority_tick += 1
            self.inexact_observation_authority_selector_diagnostics.fill(0.0)
            self.inexact_observation_authority_selector_step_ns = 0
            self.inexact_observation_authority_selector_allocation_calls = 0
            self.inexact_observation_authority_selector_allocated_bytes = 0
            self.inexact_observation_terminal_selector_diagnostics.fill(0.0)
            self.inexact_observation_terminal_selector_selection.fill(0.0)
            self.inexact_observation_terminal_selector_available.fill(0)
            self.inexact_observation_terminal_selector_queried = False
            self.inexact_observation_terminal_selector_step_ns = 0
            self.inexact_observation_terminal_selector_allocation_calls = 0
            self.inexact_observation_terminal_selector_allocated_bytes = 0
            if (
                self.contact_program_inexact_hold_forecast_selector
                and not observation_exact
            ):
                self.inexact_observation_authority_selector_state[:] = (
                    rotation_error[0],
                    root_twist[0],
                    rotation_error[1],
                    root_twist[1],
                    root_position[1] - self.nominal_root_position[1],
                    root_twist[4],
                    rotation_error[2],
                    root_twist[2],
                    max(float(root_position[2]), 0.05),
                )
                (
                    self.inexact_observation_authority_selector_step_ns,
                    self.inexact_observation_authority_selector_allocation_calls,
                    self.inexact_observation_authority_selector_allocated_bytes,
                ) = self.balance.select_contact_program_inexact_observation_authority(
                    self.inexact_observation_authority_selector_state,
                    self.execution_forecast_achieved,
                    self.execution_forecast_maximum_torque_utilization,
                    self.execution_forecast_minimum_joint_headroom_fraction,
                    self.contact_program_inexact_hold_forecast_minimum_improvement,
                    self.inexact_observation_authority_selector_diagnostics,
                )
                accumulated_step_ns += (
                    self.inexact_observation_authority_selector_step_ns
                )
                accumulated_allocation_calls += (
                    self.inexact_observation_authority_selector_allocation_calls
                )
                accumulated_allocated_bytes += (
                    self.inexact_observation_authority_selector_allocated_bytes
                )
            if self.contact_program_inexact_terminal_chooser and not observation_exact:
                previous_command_age = int(
                    self.contact_program_authority_diagnostics[
                        self.contact_program_authority_index["command_age_ticks"]
                    ]
                )
                retained_available = bool(
                    self.contact_program_authority_has_primary_history
                    and
                    self.contact_program_inexact_hold_ticks > 0
                    and previous_command_age
                    < self.contact_program_inexact_hold_ticks
                )
                self.inexact_observation_terminal_selector_state[:] = (
                    max(
                        # Fixed world-z root plane for the Upkie scene. The
                        # current wheel-center height moves with the robot and
                        # therefore cannot define an impact surface. This is an
                        # upright torso-bottom proxy, not first mesh contact.
                        float(root_position[2] - UPKIE_ROOT_IMPACT_PLANE_M),
                        0.0,
                    ),
                    root_twist[5],
                    rotation_error[0],
                    rotation_error[1],
                    root_twist[0],
                    root_twist[1],
                )
                self.inexact_observation_terminal_selector_available[:] = (
                    (
                        self.inexact_observation_terminal_zero_effort_available
                        if self.contact_program_inexact_terminal_zero_effort_baseline
                        else True
                    ),
                    retained_available,
                    self.support_free_inexact_brake_available
                    and self.contact_program_authority_has_primary_history,
                )
                self.inexact_observation_terminal_selector_root_acceleration.fill(0.0)
                self.inexact_observation_terminal_selector_joint_acceleration.fill(0.0)
                self.inexact_observation_terminal_selector_effort_utilization.fill(0.0)
                if self.contact_program_inexact_terminal_zero_effort_baseline:
                    self.inexact_observation_terminal_selector_root_acceleration[0] = (
                        self.inexact_observation_terminal_zero_effort_acceleration[0],
                        self.inexact_observation_terminal_zero_effort_acceleration[1],
                    )
                    self.inexact_observation_terminal_selector_joint_acceleration[0] = (
                        self.inexact_observation_terminal_zero_effort_acceleration[6:]
                    )
                if not self.inexact_observation_terminal_selector_available[0]:
                    # The real action remains fail-closed zero effort, but an
                    # invalid model witness cannot participate in selection.
                    self.inexact_observation_terminal_selector_selection.fill(0.0)
                    self.balance.install_contact_program_terminal_impact_selection(0)
                    self.inexact_observation_terminal_selector_queried = False
                    self.inexact_observation_terminal_selector_root_acceleration.fill(0.0)
                    self.inexact_observation_terminal_selector_joint_acceleration.fill(0.0)
                    self.inexact_observation_terminal_selector_effort_utilization.fill(0.0)
                    # Skip the scorer: it requires an available baseline.
                    continue_terminal_selection = False
                else:
                    continue_terminal_selection = True
                self.inexact_observation_terminal_selector_root_acceleration[1] = (
                    self.execution_forecast_achieved[0],
                    self.execution_forecast_achieved[3],
                )
                self.inexact_observation_terminal_selector_joint_acceleration[1] = (
                    self.inexact_observation_terminal_selector_retained_joint_acceleration
                )
                self.inexact_observation_terminal_selector_effort_utilization[1] = (
                    self.execution_forecast_maximum_torque_utilization
                )
                if self.support_contingency_out is not None:
                    brake_acceleration = self.support_contingency_out[
                        "generalized_acceleration"
                    ][0]
                    self.inexact_observation_terminal_selector_root_acceleration[2] = (
                        brake_acceleration[0],
                        brake_acceleration[1],
                    )
                    self.inexact_observation_terminal_selector_joint_acceleration[2] = (
                        brake_acceleration[6:]
                    )
                    self.inexact_observation_terminal_selector_effort_utilization[2] = float(
                        self.support_contingency_out[
                            "maximum_torque_utilization"
                        ][0]
                    )
                if self.contact_program_inexact_terminal_support_hypothesis_envelope:
                    if not self.contact_program_authority_has_primary_history:
                        self.inexact_observation_terminal_hypothesis_available[
                            :, 2
                        ] = 0
                    baseline_hypotheses_available = bool(
                        np.all(
                            self.inexact_observation_terminal_hypothesis_available[
                                :, 0
                            ]
                            != 0
                        )
                    )
                    if baseline_hypotheses_available:
                        for support_mask in range(4):
                            hypothesis_acceleration = (
                                self.inexact_observation_terminal_hypothesis_acceleration[
                                    support_mask
                                ]
                            )
                            self.inexact_observation_terminal_selector_available[:] = (
                                self.inexact_observation_terminal_hypothesis_available[
                                    support_mask
                                ]
                            )
                            self.inexact_observation_terminal_selector_root_acceleration[:] = (
                                hypothesis_acceleration[:, :2]
                            )
                            self.inexact_observation_terminal_selector_joint_acceleration[:] = (
                                hypothesis_acceleration[:, 6:]
                            )
                            self.inexact_observation_terminal_selector_effort_utilization[:] = (
                                self.inexact_observation_terminal_hypothesis_effort_utilization
                            )
                            score_timing = self.balance.score_terminal_impact_candidates(
                                self.inexact_observation_terminal_selector_state,
                                q,
                                v,
                                self.inexact_observation_terminal_selector_joint_lower,
                                self.inexact_observation_terminal_selector_joint_upper,
                                self.inexact_observation_terminal_selector_joint_velocity_limit,
                                self.inexact_observation_terminal_selector_available,
                                self.inexact_observation_terminal_selector_root_acceleration,
                                self.inexact_observation_terminal_selector_joint_acceleration,
                                self.inexact_observation_terminal_selector_effort_utilization,
                                0,
                                self.contact_program_inexact_terminal_maximum_component_regression,
                                self.contact_program_inexact_terminal_minimum_component_improvement,
                                self.inexact_observation_terminal_selector_diagnostics,
                                self.inexact_observation_terminal_hypothesis_selection_scratch,
                            )
                            self.inexact_observation_terminal_selector_step_ns += int(
                                score_timing[0]
                            )
                            self.inexact_observation_terminal_selector_allocation_calls += int(
                                score_timing[1]
                            )
                            self.inexact_observation_terminal_selector_allocated_bytes += int(
                                score_timing[2]
                            )
                            self.inexact_observation_terminal_hypothesis_diagnostics[
                                :, support_mask
                            ] = self.inexact_observation_terminal_selector_diagnostics
                        (
                            self.inexact_observation_terminal_hypothesis_aggregate_step_ns,
                            self.inexact_observation_terminal_hypothesis_aggregate_allocation_calls,
                            self.inexact_observation_terminal_hypothesis_aggregate_allocated_bytes,
                        ) = self.balance.select_terminal_impact_hypothesis_envelopes(
                            self.inexact_observation_terminal_hypothesis_diagnostics,
                            0,
                            self.contact_program_inexact_terminal_maximum_component_regression,
                            self.contact_program_inexact_terminal_minimum_component_improvement,
                            self.inexact_observation_terminal_hypothesis_envelopes,
                            self.inexact_observation_terminal_selector_selection,
                        )
                        self.inexact_observation_terminal_selector_diagnostics[:] = (
                            self.inexact_observation_terminal_hypothesis_envelopes
                        )
                        self.inexact_observation_terminal_selector_available[:] = (
                            np.all(
                                self.inexact_observation_terminal_hypothesis_available[
                                    :, 0
                                ]
                                != 0
                            ),
                            np.all(
                                self.inexact_observation_terminal_hypothesis_available[
                                    :, 1
                                ]
                                != 0
                            ),
                            np.all(
                                self.inexact_observation_terminal_hypothesis_available[
                                    :, 2
                                ]
                                != 0
                            ),
                        )
                        representative = (
                            self.inexact_observation_terminal_hypothesis_acceleration[0]
                        )
                        self.inexact_observation_terminal_selector_root_acceleration[:] = (
                            representative[:, :2]
                        )
                        self.inexact_observation_terminal_selector_joint_acceleration[:] = (
                            representative[:, 6:]
                        )
                        self.inexact_observation_terminal_selector_effort_utilization[:] = (
                            self.inexact_observation_terminal_hypothesis_effort_utilization
                        )
                        selected_terminal_action = int(
                            self.inexact_observation_terminal_selector_selection[
                                self.terminal_impact_selection_index["selected_index"]
                            ]
                        )
                        self.balance.install_contact_program_terminal_impact_selection(
                            selected_terminal_action
                        )
                        self.inexact_observation_terminal_selector_queried = True
                        self.inexact_observation_terminal_selector_step_ns += (
                            self.inexact_observation_terminal_hypothesis_aggregate_step_ns
                        )
                        self.inexact_observation_terminal_selector_allocation_calls += (
                            self.inexact_observation_terminal_hypothesis_aggregate_allocation_calls
                        )
                        self.inexact_observation_terminal_selector_allocated_bytes += (
                            self.inexact_observation_terminal_hypothesis_aggregate_allocated_bytes
                        )
                        accumulated_step_ns += (
                            self.inexact_observation_terminal_selector_step_ns
                        )
                        accumulated_allocation_calls += (
                            self.inexact_observation_terminal_selector_allocation_calls
                        )
                        accumulated_allocated_bytes += (
                            self.inexact_observation_terminal_selector_allocated_bytes
                        )
                    else:
                        self.inexact_observation_terminal_selector_selection.fill(0.0)
                        self.balance.install_contact_program_terminal_impact_selection(0)
                    continue_terminal_selection = False
                if continue_terminal_selection:
                    (
                        self.inexact_observation_terminal_selector_step_ns,
                        self.inexact_observation_terminal_selector_allocation_calls,
                        self.inexact_observation_terminal_selector_allocated_bytes,
                    ) = self.balance.score_terminal_impact_candidates(
                        self.inexact_observation_terminal_selector_state,
                        q,
                        v,
                        self.inexact_observation_terminal_selector_joint_lower,
                        self.inexact_observation_terminal_selector_joint_upper,
                        self.inexact_observation_terminal_selector_joint_velocity_limit,
                        self.inexact_observation_terminal_selector_available,
                        self.inexact_observation_terminal_selector_root_acceleration,
                        self.inexact_observation_terminal_selector_joint_acceleration,
                        self.inexact_observation_terminal_selector_effort_utilization,
                        0,
                        self.contact_program_inexact_terminal_maximum_component_regression,
                        self.contact_program_inexact_terminal_minimum_component_improvement,
                        self.inexact_observation_terminal_selector_diagnostics,
                        self.inexact_observation_terminal_selector_selection,
                    )
                    selected_terminal_action = int(
                        self.inexact_observation_terminal_selector_selection[
                            self.terminal_impact_selection_index["selected_index"]
                        ]
                    )
                    self.balance.install_contact_program_terminal_impact_selection(
                        selected_terminal_action
                    )
                    self.inexact_observation_terminal_selector_queried = True
                    accumulated_step_ns += self.inexact_observation_terminal_selector_step_ns
                    accumulated_allocation_calls += (
                        self.inexact_observation_terminal_selector_allocation_calls
                    )
                    accumulated_allocated_bytes += (
                        self.inexact_observation_terminal_selector_allocated_bytes
                    )
            current_support_available = bool(
                self.support_contingency_execute
                and self.support_contingency_requested
                and self.support_contingency_admitted
                and self.support_contingency_forecast_guard_passed
            )
            current_support_command = (
                self.support_contingency_out["actuator_torque"][0]
                if self.support_contingency_out is not None
                else self.contact_program_command
            )
            terminal_support_free_selected = bool(
                self.contact_program_inexact_terminal_chooser
                and self.inexact_observation_terminal_selector_selection[
                    self.terminal_impact_selection_index["selected_index"]
                ]
                == 2.0
            )
            self.balance.step_contact_program_authority_from_masks(
                self.contact_program_authority_tick,
                observation_exact,
                self.observed_contact_active,
                self.contact_debounced,
                self.contact_program_hard_contact,
                self.primary_program_available,
                self.primary_program_torque[0],
                current_support_available,
                current_support_command,
                self.support_free_inexact_brake_available
                and (
                    self.contact_program_inexact_support_free_brake
                    or terminal_support_free_selected
                ),
                (
                    self.support_contingency_out["actuator_torque"][0]
                    if self.support_contingency_out is not None
                    else self.contact_program_command
                ),
                self.contact_program_command,
                self.contact_program_authority_diagnostics,
            )
            program_selection = int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["selection"]
                ]
            )
            program_executable = bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["executable"]
                ]
            )
            self.contact_program_authority_has_primary_history = program_executable
            self.support_contingency_selected = program_selection in (2, 5)
            if self.support_contingency_selected:
                command_out = self.support_contingency_out
                if command_out is None:
                    raise RuntimeError(
                        "current-support authority selected without a WBC output"
                    )
                status = int(command_out["status"][0])
            elif program_selection == 1:
                command_out = out
                status = primary_status
            elif program_selection in (3, 4):
                command_out = out
                status = 0
            else:
                command_out = out
                status = 4
            command_fresh = bool(
                program_selection in (1, 2)
                and (
                    not self.support_contingency_realize_primary_torque
                    or self.primary_program_source_fresh
                )
            )
        else:
            self.contact_program_authority_diagnostics.fill(0)
            self.contact_program_command.fill(0.0)
            status = (
                int(command_out["status"][0])
                if self.support_contingency_selected
                else primary_status
            )
            command_fresh = status in (0, 1)
        self.execution_forecast_path.fill(0.0)
        self.execution_forecast_path_valid = False
        self.execution_residual_prediction_valid = False
        self.execution_forecast_step_ns = 0
        self.execution_forecast_allocation_calls = 0
        self.execution_forecast_allocated_bytes = 0
        execution_hard_contact = (
            self.contact_program_hard_contact
            if self.contact_program_authority_enabled
            else self.contact_active[0]
        )
        self.execution_forecast_support_mask = int(execution_hard_contact[0]) | (
            int(execution_hard_contact[1]) << 1
        )
        self.execution_forecast_reduced_state[:] = (
            rotation_error[0],
            root_twist[0],
            rotation_error[1],
            root_twist[1],
            root_position[1] - self.nominal_root_position[1],
            root_twist[4],
            rotation_error[2],
            root_twist[2],
        )
        self.execution_forecast_state[:8] = self.execution_forecast_reduced_state
        self.execution_forecast_state[8] = max(float(root_position[2]), 0.05)
        self.execution_forecast_state[9] = self.execution_forecast_support_mask
        if command_fresh and observation_exact:
            generalized = command_out["generalized_acceleration"][0]
            self.inexact_observation_terminal_selector_retained_joint_acceleration[:] = (
                generalized[6:]
            )
            self.execution_forecast_achieved[:] = (
                generalized[0],
                generalized[4],
                generalized[2],
                generalized[1],
            )
            self.execution_forecast_maximum_torque_utilization = float(
                command_out["maximum_torque_utilization"][0]
            )
            self.execution_forecast_minimum_joint_headroom_fraction = float(
                command_out["minimum_joint_headroom_fraction"][0]
            )
            (
                self.execution_forecast_step_ns,
                self.execution_forecast_allocation_calls,
                self.execution_forecast_allocated_bytes,
            ) = self.balance.predict_viability_forecast_path(
                self.execution_forecast_state,
                self.execution_forecast_achieved,
                self.viability_request,
                self.viability_request,
                self.execution_forecast_maximum_request,
                float(command_out["maximum_torque_utilization"][0]),
                float(command_out["minimum_joint_headroom_fraction"][0]),
                self.execution_forecast_path,
            )
            self.execution_forecast_path_valid = True
            if self.execution_forecast_support_mask == execution_support_mask:
                self.execution_residual_prediction[:] = self.execution_residual_state
                component_acceleration = (
                    self.execution_forecast_achieved[0],
                    self.execution_forecast_achieved[3],
                    self.execution_forecast_achieved[1],
                    self.execution_forecast_achieved[2],
                )
                for pair, acceleration in enumerate(component_acceleration):
                    position = 2 * pair
                    velocity = position + 1
                    self.execution_residual_prediction[position] += (
                        CONTROL_DT * self.execution_residual_state[velocity]
                        + 0.5 * CONTROL_DT**2 * acceleration
                    )
                    self.execution_residual_prediction[velocity] += (
                        CONTROL_DT * acceleration
                    )
                self.execution_residual_prediction_support_mask = (
                    execution_support_mask
                )
                self.execution_residual_prediction_valid = True
        if command_fresh:
            self.has_admitted_command = True
            self.last_admitted_torque[:] = command_out["actuator_torque"][0]
            self.last_admitted_contact_force[:] = command_out["contact_force_basis"][0]
            self.command_age_steps = 0
        else:
            self.command_age_steps += 1
        fresh_command_authority = self.fall_safe_diagnostics[
            self.fall_safe_index["fresh_command_authority"]
        ]
        lease_executable = False
        if self.contact_command_lease_enabled:
            self.contact_command_lease_tick += 1
            observation_exact = bool(
                observed_contact_active is not None
                and self.contact_observation_diagnostics[
                    self.contact_observation_index["accepted"]
                ]
                == 1
                and self.contact_observation_diagnostics[
                    self.contact_observation_index["provenance"]
                ]
                == 0
            )
            self.balance.step_contact_command_lease_from_masks(
                self.contact_command_lease_tick,
                observation_exact,
                self.contact_debounced,
                self.contact_active[0],
                command_fresh,
                command_out["actuator_torque"][0],
                self.contact_command,
                self.contact_command_lease_diagnostics,
            )
            lease_executable = bool(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["executable"]
                ]
            )
        else:
            self.contact_command_lease_diagnostics.fill(0)
        if self.contact_program_authority_enabled:
            self.previous_solver_admitted = bool(
                program_executable
                if program_selection == 5
                else self.primary_program_source_fresh
                if self.support_contingency_realize_primary_torque
                else program_executable
            )
        else:
            self.previous_solver_admitted = command_fresh or lease_executable
        if self.contact_program_authority_enabled:
            if program_executable:
                self.executed_torque[:] = self.contact_program_command
                if command_fresh:
                    self.executed_contact_force[:] = command_out[
                        "contact_force_basis"
                    ][0]
                else:
                    self.executed_contact_force.fill(0.0)
            else:
                self.executed_torque.fill(0.0)
                self.executed_contact_force.fill(0.0)
        elif self.contact_command_lease_enabled:
            if lease_executable:
                self.executed_torque[:] = self.contact_command
                if command_fresh:
                    self.executed_contact_force[:] = self.last_admitted_contact_force
                else:
                    # A retained actuator command has no current-support force
                    # witness. The force remains diagnostic and non-executable.
                    self.executed_contact_force.fill(0.0)
            elif self.contact_command_lease_fallback_to_freshness:
                self.executed_torque[:] = self.last_admitted_torque
                self.executed_contact_force[:] = self.last_admitted_contact_force
                if status not in (0, 1) and self.fall_safe_enabled:
                    self.executed_torque *= fresh_command_authority
                    self.executed_contact_force *= fresh_command_authority
            else:
                self.executed_torque.fill(0.0)
                self.executed_contact_force.fill(0.0)
        else:
            self.executed_contact_force[:] = self.last_admitted_contact_force
            self.executed_torque[:] = self.last_admitted_torque
            if status not in (0, 1) and self.fall_safe_enabled:
                self.executed_torque *= fresh_command_authority
                self.executed_contact_force *= fresh_command_authority
        return {
            "torque": self.executed_torque,
            "generalized_acceleration": command_out[
                "generalized_acceleration"
            ][0].copy(),
            "normal_force": command_out["contact_normal_force"][0].copy(),
            "contact_force_basis": self.executed_contact_force,
            "status": status,
            "raw_wbc_status": raw_status,
            "primary_status": primary_status,
            "primary_wbc_step_ns": primary_wbc_step_ns,
            "primary_task_pseudoinverse_calls": (
                primary_task_pseudoinverse_calls
            ),
            "primary_clipped_steps": primary_clipped_steps,
            "primary_task_jacobi_sweeps": primary_task_jacobi_sweeps,
            "primary_feasibility_projection_sweeps": (
                primary_feasibility_projection_sweeps
            ),
            "primary_feasibility_halfspace_projections": (
                primary_feasibility_halfspace_projections
            ),
            "contact_program_authority_enabled": (
                self.contact_program_authority_enabled
            ),
            "contact_program_authority_selection": program_selection,
            "contact_program_authority_executable": program_executable,
            "contact_program_authority_transition_pending": bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["transition_pending"]
                ]
            ),
            "contact_program_authority_activation_pending": bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["activation_pending"]
                ]
            ),
            "contact_program_authority_deactivation_pending": bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["deactivation_pending"]
                ]
            ),
            "contact_program_authority_masks_consistent": bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["masks_consistent"]
                ]
            ),
            "contact_program_authority_lease_status": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["lease_status"]
                ]
            ),
            "contact_program_authority_lease_provenance": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["lease_provenance"]
                ]
            ),
            "contact_program_authority_authoring_mask": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["authoring_mask"]
                ]
            ),
            "contact_program_authority_stable_mask": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["stable_mask"]
                ]
            ),
            "contact_program_authority_hard_mask": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["hard_mask"]
                ]
            ),
            "contact_program_authority_transition_count": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["transition_count"]
                ]
            ),
            "contact_program_authority_transition_current_enabled": bool(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index[
                        "allow_transition_current_support"
                    ]
                ]
            ),
            "contact_program_authority_age_ticks": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["command_age_ticks"]
                ]
            ),
            "contact_program_authority_remaining_ticks": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["remaining_hold_ticks"]
                ]
            ),
            "contact_program_authority_flags": int(
                self.contact_program_authority_diagnostics[
                    self.contact_program_authority_index["flags"]
                ]
            ),
            "inexact_observation_authority_selector_queried": bool(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index["queried"]
                ]
            ),
            "inexact_observation_authority_selector_selected_index": int(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "selected_index"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_authority_q15": int(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "selected_authority_q15"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_selected_score": float(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "selected_score"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_zero_score": float(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "zero_authority_score"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_full_score": float(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "full_authority_score"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_improvement": float(
                self.inexact_observation_authority_selector_diagnostics[
                    self.inexact_observation_authority_selector_index[
                        "score_improvement_from_zero"
                    ]
                ]
            ),
            "inexact_observation_authority_selector_candidate_scores": (
                self.inexact_observation_authority_selector_diagnostics[7:12]
            ),
            "inexact_observation_authority_selector_step_ns": int(
                self.inexact_observation_authority_selector_step_ns
            ),
            "inexact_observation_authority_selector_allocation_calls": int(
                self.inexact_observation_authority_selector_allocation_calls
            ),
            "inexact_observation_authority_selector_allocated_bytes": int(
                self.inexact_observation_authority_selector_allocated_bytes
            ),
            "inexact_observation_terminal_selector_queried": bool(
                self.inexact_observation_terminal_selector_queried
            ),
            "inexact_observation_terminal_zero_effort_baseline_enabled": (
                self.contact_program_inexact_terminal_zero_effort_baseline
            ),
            "inexact_observation_terminal_zero_effort_available": (
                self.inexact_observation_terminal_zero_effort_available
            ),
            "inexact_observation_terminal_zero_effort_acceleration": (
                self.inexact_observation_terminal_zero_effort_acceleration
            ),
            "inexact_observation_terminal_zero_effort_step_ns": int(
                self.inexact_observation_terminal_zero_effort_step_ns
            ),
            "inexact_observation_terminal_zero_effort_allocation_calls": int(
                self.inexact_observation_terminal_zero_effort_allocation_calls
            ),
            "inexact_observation_terminal_zero_effort_allocated_bytes": int(
                self.inexact_observation_terminal_zero_effort_allocated_bytes
            ),
            "inexact_observation_terminal_support_hypothesis_envelope_enabled": (
                self.contact_program_inexact_terminal_support_hypothesis_envelope
            ),
            "inexact_observation_terminal_hypothesis_acceleration": (
                self.inexact_observation_terminal_hypothesis_acceleration
            ),
            "inexact_observation_terminal_hypothesis_available": (
                self.inexact_observation_terminal_hypothesis_available
            ),
            "inexact_observation_terminal_hypothesis_diagnostics": (
                self.inexact_observation_terminal_hypothesis_diagnostics
            ),
            "inexact_observation_terminal_hypothesis_envelopes": (
                self.inexact_observation_terminal_hypothesis_envelopes
            ),
            "inexact_observation_terminal_hypothesis_torque": (
                self.inexact_observation_terminal_hypothesis_torque
            ),
            "inexact_observation_terminal_hypothesis_effort_utilization": (
                self.inexact_observation_terminal_hypothesis_effort_utilization
            ),
            "inexact_observation_terminal_hypothesis_query_step_ns": int(
                self.inexact_observation_terminal_hypothesis_query_step_ns
            ),
            "inexact_observation_terminal_hypothesis_query_allocation_calls": int(
                self.inexact_observation_terminal_hypothesis_query_allocation_calls
            ),
            "inexact_observation_terminal_hypothesis_query_allocated_bytes": int(
                self.inexact_observation_terminal_hypothesis_query_allocated_bytes
            ),
            "inexact_observation_terminal_hypothesis_aggregate_step_ns": int(
                self.inexact_observation_terminal_hypothesis_aggregate_step_ns
            ),
            "inexact_observation_terminal_hypothesis_aggregate_allocation_calls": int(
                self.inexact_observation_terminal_hypothesis_aggregate_allocation_calls
            ),
            "inexact_observation_terminal_hypothesis_aggregate_allocated_bytes": int(
                self.inexact_observation_terminal_hypothesis_aggregate_allocated_bytes
            ),
            "inexact_observation_terminal_selector_action": int(
                self.inexact_observation_terminal_selector_selection[
                    self.terminal_impact_selection_index["selected_index"]
                ]
            ),
            "inexact_observation_terminal_selector_retained_available": bool(
                self.inexact_observation_terminal_selector_available[1]
            ),
            "inexact_observation_terminal_selector_support_free_available": bool(
                self.inexact_observation_terminal_selector_available[2]
            ),
            "inexact_observation_terminal_selector_time_to_impact_s": float(
                self.inexact_observation_terminal_selector_diagnostics[
                    0, self.terminal_impact_index["time_to_impact_s"]
                ]
            ),
            "inexact_observation_terminal_selector_vertical_specific_energy_j_kg": float(
                self.inexact_observation_terminal_selector_diagnostics[
                    0,
                    self.terminal_impact_index[
                        "vertical_specific_impact_energy_j_kg"
                    ],
                ]
            ),
            "inexact_observation_terminal_selector_candidate_diagnostics": (
                self.inexact_observation_terminal_selector_diagnostics
            ),
            "inexact_observation_terminal_selector_state": (
                self.inexact_observation_terminal_selector_state
            ),
            "inexact_observation_terminal_selector_root_acceleration": (
                self.inexact_observation_terminal_selector_root_acceleration
            ),
            "inexact_observation_terminal_selector_joint_acceleration": (
                self.inexact_observation_terminal_selector_joint_acceleration
            ),
            "inexact_observation_terminal_selector_effort_utilization": (
                self.inexact_observation_terminal_selector_effort_utilization
            ),
            "inexact_observation_terminal_selector_selection_diagnostics": (
                self.inexact_observation_terminal_selector_selection
            ),
            "inexact_observation_terminal_selector_maximum_harm_pressures": (
                self.inexact_observation_terminal_selector_diagnostics[
                    :, self.terminal_impact_index["maximum_terminal_harm_pressure"]
                ]
            ),
            "inexact_observation_terminal_selector_aggregate_scores": (
                self.inexact_observation_terminal_selector_diagnostics[
                    :, self.terminal_impact_index["aggregate_score"]
                ]
            ),
            "inexact_observation_terminal_selector_step_ns": int(
                self.inexact_observation_terminal_selector_step_ns
            ),
            "inexact_observation_terminal_selector_allocation_calls": int(
                self.inexact_observation_terminal_selector_allocation_calls
            ),
            "inexact_observation_terminal_selector_allocated_bytes": int(
                self.inexact_observation_terminal_selector_allocated_bytes
            ),
            "support_contingency_enabled": self.support_contingency_enabled,
            "support_contingency_preserve_primary_support": (
                self.support_contingency_preserve_primary_support
            ),
            "support_contingency_admitted": self.support_contingency_admitted,
            "support_contingency_armed": self.support_contingency_armed,
            "support_contingency_requested": self.support_contingency_requested,
            "support_contingency_selected": self.support_contingency_selected,
            "support_contingency_realization_fallback": (
                self.support_contingency_realization_fallback
            ),
            "support_contingency_candidate_power_w": (
                self.support_contingency_candidate_power_w
            ),
            "support_contingency_primary_torque": self.primary_program_torque[0],
            "support_contingency_primary_source_fresh": (
                self.primary_program_source_fresh
            ),
            "support_contingency_primary_power_w": (
                self.support_contingency_primary_power_w
            ),
            "support_contingency_incremental_power_w": (
                self.support_contingency_incremental_power_w
            ),
            "support_contingency_forecast_guard_passed": (
                self.support_contingency_forecast_guard_passed
            ),
            "support_contingency_forecast_baseline_score": (
                self.support_contingency_forecast_baseline_score
            ),
            "support_contingency_forecast_candidate_score": (
                self.support_contingency_forecast_candidate_score
            ),
            "support_contingency_forecast_step_ns": (
                self.support_contingency_forecast_step_ns
            ),
            "support_load_guard_enabled": self.support_load_guard_enabled,
            "support_load_guard_active": self.support_load_guard_active,
            "support_load_guard_authority": self.support_load_guard_authority,
            "support_contingency_diagnostics": self.support_contingency_diagnostics,
            "support_contingency_candidate_generalized_acceleration": (
                self.support_contingency_candidate_generalized_acceleration
            ),
            "support_contingency_candidate_torque": (
                self.support_contingency_candidate_torque
            ),
            "support_contingency_mode": int(
                self.support_contingency_diagnostics[0]
            ),
            "support_contingency_support_mask": int(
                self.support_contingency_diagnostics[1]
            ),
            "support_contingency_status": int(
                self.support_contingency_out["status"][0]
            ) if self.support_contingency_out is not None else 3,
            "support_contingency_maximum_constraint_violation": float(
                self.support_contingency_out["maximum_constraint_violation"][0]
            ) if self.support_contingency_out is not None else 0.0,
            "support_contingency_author_step_ns": int(
                self.support_contingency_author_step_ns
            ),
            "support_contingency_wbc_step_ns": int(
                self.support_contingency_out["step_ns"][0]
            ) if self.support_contingency_out is not None else 0,
            "support_program_admitted": support_program_admitted,
            "contact_command_lease_enabled": self.contact_command_lease_enabled,
            "contact_command_lease_status": int(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["status"]
                ]
            ),
            "contact_command_lease_provenance": int(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["provenance"]
                ]
            ),
            "contact_command_lease_executable": lease_executable,
            "contact_command_lease_age_ticks": int(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["command_age_ticks"]
                ]
            ),
            "contact_command_lease_remaining_ticks": int(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["remaining_hold_ticks"]
                ]
            ),
            "contact_command_lease_flags": int(
                self.contact_command_lease_diagnostics[
                    self.contact_command_lease_index["flags"]
                ]
            ),
            "maximum_constraint_violation": float(
                command_out["maximum_constraint_violation"][0]
            ),
            "minimum_bound_margin": float(command_out["minimum_bound_margin"][0]),
            "command_age_steps": self.command_age_steps,
            "support_active_count": int(
                self.contact_program_hard_contact[0]
                + self.contact_program_hard_contact[1]
                if self.contact_program_authority_enabled
                else self.contact_active[0, 0] + self.contact_active[0, 1]
            ),
            "support_active_left": int(
                self.contact_program_hard_contact[0]
                if self.contact_program_authority_enabled
                else self.contact_active[0, 0]
            ),
            "support_active_right": int(
                self.contact_program_hard_contact[1]
                if self.contact_program_authority_enabled
                else self.contact_active[0, 1]
            ),
            "support_transition_count": int(
                self.contact_observation_diagnostics[
                    self.contact_observation_index["transition_count"]
                ]
            ) if observed_contact_active is not None else 0,
            "contact_observation_status": int(
                self.contact_observation_diagnostics[
                    self.contact_observation_index["status"]
                ]
            ) if observed_contact_active is not None else 0,
            "contact_observation_provenance": int(
                self.contact_observation_diagnostics[
                    self.contact_observation_index["provenance"]
                ]
            ) if observed_contact_active is not None else 0,
            "contact_observation_flags": int(
                self.contact_observation_diagnostics[
                    self.contact_observation_index["flags"]
                ]
            ) if observed_contact_active is not None else 0,
            "contact_observation_age_ns": int(
                self.contact_observation_diagnostics[
                    self.contact_observation_index["age_ns"]
                ]
            ) if observed_contact_active is not None else 0,
            "contact_observation_available": bool(
                observed_contact_active is not None and observed_contact_available
            ),
            "fall_safe_mode": int(
                self.fall_safe_diagnostics[self.fall_safe_index["mode"]]
            ),
            "fall_safe_primary_authority": float(primary_authority),
            "fall_safe_fresh_command_authority": float(fresh_command_authority),
            "fall_safe_risk": float(
                self.fall_safe_diagnostics[self.fall_safe_index["raw_risk"]]
            ),
            "fall_safe_reason_flags": int(
                self.fall_safe_diagnostics[
                    self.fall_safe_index["limiting_reason_flags"]
                ]
            ),
            "step_ns": int(accumulated_step_ns),
            "viability_planner_step_ns": int(self.viability_planner_step_ns),
            "viability_planner_feasibility_seed_reuses": int(
                self.viability_planner_feasibility_seed_reuses
            ),
            "viability_planner_feasibility_projection_sweeps": int(
                self.viability_planner_feasibility_projection_sweeps
            ),
            "viability_planner_feasibility_halfspace_projections": int(
                self.viability_planner_feasibility_halfspace_projections
            ),
            "viability_hard_feasibility_witness_transferred": int(
                self.viability_hard_feasibility_witness_transferred
            ),
            "final_feasibility_seed_reused": int(
                self.final_feasibility_seed_reused
            ),
            "final_feasibility_prefix_resumed": int(
                self.final_feasibility_prefix_resumed
            ),
            "viability_hard_feasibility_witness_transfer_ns": int(
                self.viability_hard_feasibility_witness_transfer_diagnostics[0]
            ),
            "viability_hard_feasibility_witness_transfer_allocation_calls": int(
                self.viability_hard_feasibility_witness_transfer_diagnostics[1]
            ),
            "viability_hard_feasibility_witness_transfer_allocated_bytes": int(
                self.viability_hard_feasibility_witness_transfer_diagnostics[2]
            ),
            "task_pseudoinverse_calls": int(
                command_out["task_pseudoinverse_calls"][0]
            ),
            "clipped_steps": int(command_out["clipped_steps"][0]),
            "task_jacobi_sweeps": int(command_out["task_jacobi_sweeps"][0]),
            "feasibility_projection_sweeps": int(
                command_out["feasibility_projection_sweeps"][0]
            ),
            "feasibility_halfspace_projections": int(
                command_out["feasibility_halfspace_projections"][0]
            ),
            "feasibility_polish_iterations": int(
                command_out["feasibility_polish_iterations"][0]
            ),
            "allocation_calls": int(accumulated_allocation_calls),
            "allocated_bytes": int(accumulated_allocated_bytes),
            "dynamics_residual": float(command_out["dynamics_residual"][0]),
            "contact_residual": float(command_out["contact_residual"][0]),
            "torque_utilization": float(
                command_out["maximum_torque_utilization"][0]
            ),
            "minimum_torque_headroom": float(
                command_out["minimum_torque_headroom"][0]
            ),
            "witness_acceleration_rms": float(
                command_out["witness_acceleration_rms"][0]
            ),
            "virtual_pitch": float(self.virtual_pitch[0]),
            "capture_position": float(
                self.capture_diagnostics[
                    self.capture_index["capture_position_control_world_m"]
                ]
            ),
            "capture_error": float(
                self.capture_diagnostics[self.capture_index["capture_error_m"]]
            ),
            "capture_pressure": float(
                self.capture_diagnostics[self.capture_index["capture_pressure"]]
            ),
            "station_authority": float(
                self.capture_diagnostics[self.capture_index["station_authority"]]
            ),
            "station_error": float(
                self.capture_diagnostics[self.capture_index["station_error_m"]]
            ),
            "commanded_ground_velocity": float(
                self.capture_diagnostics[
                    self.capture_index["commanded_ground_velocity_mps"]
                ]
            ),
            "heading": float(
                self.capture_diagnostics[self.capture_index["heading_world_rad"]]
                if "heading_world_rad" in self.capture_index
                else 0.0
            ),
            "longitudinal_capture_error": float(
                self.capture_diagnostics[
                    self.capture_index["longitudinal_capture_error_m"]
                ]
                if "longitudinal_capture_error_m" in self.capture_index
                else self.capture_diagnostics[self.capture_index["capture_error_m"]]
            ),
            "lateral_capture_error": float(
                self.capture_diagnostics[
                    self.capture_index["lateral_capture_error_m"]
                ]
                if "lateral_capture_error_m" in self.capture_index
                else 0.0
            ),
            "planar_capture_pressure": float(
                self.capture_diagnostics[self.capture_index["planar_capture_pressure"]]
                if "planar_capture_pressure" in self.capture_index
                else self.capture_diagnostics[self.capture_index["capture_pressure"]]
            ),
            "commanded_yaw_rate": float(
                self.capture_diagnostics[self.capture_index["commanded_yaw_rate_rad_s"]]
                if "commanded_yaw_rate_rad_s" in self.capture_index
                else 0.0
            ),
            "commanded_yaw_acceleration": float(
                self.capture_diagnostics[
                    self.capture_index["commanded_yaw_acceleration_rad_s2"]
                ]
                if "commanded_yaw_acceleration_rad_s2" in self.capture_index
                else 0.0
            ),
            "lateral_support_margin": float(
                self.capture_diagnostics[self.capture_index["lateral_support_margin_m"]]
                if "lateral_support_margin_m" in self.capture_index
                else 0.0
            ),
            "lateral_dcm": float(
                self.capture_diagnostics[self.capture_index["lateral_dcm_m"]]
                if "lateral_dcm_m" in self.capture_index
                else 0.0
            ),
            "viability_margin": float(
                self.capture_diagnostics[self.capture_index["viability_margin_m"]]
                if "viability_margin_m" in self.capture_index
                else 0.0
            ),
            "commanded_zmp": float(
                self.capture_diagnostics[self.capture_index["commanded_zmp_m"]]
                if "commanded_zmp_m" in self.capture_index
                else 0.0
            ),
            "commanded_lateral_acceleration": float(
                self.capture_diagnostics[
                    self.capture_index["commanded_lateral_acceleration_m_s2"]
                ]
                if "commanded_lateral_acceleration_m_s2" in self.capture_index
                else 0.0
            ),
            "commanded_bank_angle": float(
                self.capture_diagnostics[self.capture_index["commanded_bank_angle_rad"]]
                if "commanded_bank_angle_rad" in self.capture_index
                else 0.0
            ),
            "commanded_roll_acceleration": float(
                self.capture_diagnostics[
                    self.capture_index["commanded_roll_acceleration_rad_s2"]
                ]
                if "commanded_roll_acceleration_rad_s2" in self.capture_index
                else 0.0
            ),
            "viability_activation_pressure": float(
                self.capture_diagnostics[
                    self.capture_index["viability_activation_pressure"]
                ]
                if "viability_activation_pressure" in self.capture_index
                else 0.0
            ),
            "viability_zmp_was_saturated": int(
                self.capture_diagnostics[
                    self.capture_index["viability_zmp_was_saturated"]
                ]
                if "viability_zmp_was_saturated" in self.capture_index
                else 0.0
            ),
            "viability_verified_scale": self.viability_verified_scale,
            "viability_verification_queries": self.viability_verification_queries,
            "viability_support_pressure": self.viability_support_pressure,
            "viability_support_active": self.viability_support_active,
            "viability_coordinate_queries": self.viability_coordinate_queries,
            "viability_coordinate_score": self.viability_coordinate_score,
            "viability_coordinate_target_roll": self.viability_coordinate_target[0],
            "viability_coordinate_target_lateral": self.viability_coordinate_target[1],
            "viability_coordinate_target_yaw": self.viability_coordinate_target[2],
            "viability_coordinate_request_roll": self.viability_coordinate_request[0],
            "viability_coordinate_request_lateral": self.viability_coordinate_request[1],
            "viability_coordinate_request_yaw": self.viability_coordinate_request[2],
            "viability_request_status": int(
                self.viability_request_diagnostics[
                    self.viability_request_index["status"]
                ]
            ),
            "viability_request_provenance": int(
                self.viability_request_diagnostics[
                    self.viability_request_index["provenance"]
                ]
            ),
            "viability_request_active": bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["active"]
                ]
            ),
            "viability_request_executable": bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["executable"]
                ]
            ),
            "viability_request_slew_limited": bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["request_was_slew_limited"]
                ]
            ),
            "viability_request_age_ticks": int(
                self.viability_request_diagnostics[
                    self.viability_request_index["candidate_age_ticks"]
                ]
            ),
            "viability_request_remaining_ticks": int(
                self.viability_request_diagnostics[
                    self.viability_request_index["remaining_fresh_ticks"]
                ]
            ),
            "viability_planner_update": bool(
                self.viability_request_diagnostics[
                    self.viability_request_index["planner_update"]
                ]
            ),
            "viability_request_flags": int(
                self.viability_request_diagnostics[
                    self.viability_request_index["flags"]
                ]
            ),
            "viability_confirmation_enabled": self.viability_confirmation_enabled,
            "viability_confirmation_status": int(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["status"]
                ]
            ),
            "viability_confirmation_executable": bool(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["executable"]
                ]
            ),
            "viability_confirmation_has_shadow": bool(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["has_shadow"]
                ]
            ),
            "viability_confirmation_support_mask": int(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["support_mask"]
                ]
            ),
            "viability_confirmation_consistent_updates": int(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["consistent_update_count"]
                ]
            ),
            "viability_confirmation_alignment": float(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["normalized_alignment"]
                ]
            ),
            "viability_confirmation_score_improvement": float(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["score_improvement"]
                ]
            ),
            "viability_confirmation_flags": int(
                self.viability_confirmation_diagnostics[
                    self.viability_confirmation_index["flags"]
                ]
            ),
            "viability_confirmation_shadow": self.viability_confirmation_shadow,
            "viability_hybrid_guard_enabled": self.viability_hybrid_guard_enabled,
            "viability_hybrid_guard_status": int(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["status"]
                ]
            ),
            "viability_hybrid_guard_executable": bool(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["executable"]
                ]
            ),
            "viability_hybrid_guard_shadow_admissible": bool(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["shadow_admissible"]
                ]
            ),
            "viability_hybrid_guard_support_age_ticks": int(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["support_age_ticks"]
                ]
            ),
            "viability_hybrid_guard_minimum_load_fraction": float(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["minimum_load_fraction"]
                ]
            ),
            "viability_hybrid_guard_signed_roll_capture_pressure": float(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index[
                        "signed_roll_capture_pressure"
                    ]
                ]
            ),
            "viability_hybrid_guard_flags": int(
                self.viability_hybrid_guard_diagnostics[
                    self.viability_hybrid_guard_index["flags"]
                ]
            ),
            "viability_planner_zero_pressure": (
                self.viability_planner.zero_pressure
                if self.viability_planner is not None
                else 0.0
            ),
            "viability_planner_candidate_pressure": self.viability_planner_candidate_pressure,
            "viability_planner_query_count": self.viability_planner_query_count,
            "viability_forecast_peak_capture_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["peak_capture_pressure"]
            ],
            "viability_forecast_peak_sagittal_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["peak_sagittal_pressure"]
            ],
            "viability_forecast_terminal_capture_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["terminal_capture_pressure"]
            ],
            "viability_forecast_terminal_sagittal_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["terminal_sagittal_pressure"]
            ],
            "viability_forecast_terminal_rate_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["terminal_rate_pressure"]
            ],
            "viability_forecast_yaw_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["peak_yaw_pressure"]
            ],
            "viability_forecast_resource_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["resource_pressure"]
            ],
            "viability_forecast_action_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["action_pressure"]
            ],
            "viability_forecast_action_delta_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["action_delta_pressure"]
            ],
            "viability_forecast_support_pressure": self.viability_forecast_diagnostics[
                self.viability_forecast_index["support_pressure"]
            ],
            "viability_forecast_minimum_capture_margin": self.viability_forecast_diagnostics[
                self.viability_forecast_index["minimum_capture_margin"]
            ],
            "viability_forecast_path_valid": self.viability_forecast_path_valid,
            "viability_forecast_path": self.viability_forecast_path,
            "execution_forecast_path_valid": self.execution_forecast_path_valid,
            "execution_forecast_support_mask": self.execution_forecast_support_mask,
            "execution_forecast_reduced_state": self.execution_forecast_reduced_state,
            "execution_forecast_achieved_acceleration": self.execution_forecast_achieved,
            "execution_forecast_path": self.execution_forecast_path,
            "execution_forecast_step_ns": self.execution_forecast_step_ns,
            "execution_forecast_allocation_calls": self.execution_forecast_allocation_calls,
            "execution_forecast_allocated_bytes": self.execution_forecast_allocated_bytes,
            "execution_residual_veto_enabled": self.execution_residual_veto_enabled,
            "execution_residual_veto_active": execution_residual_veto_active,
            "execution_residual_status": int(
                self.execution_residual_diagnostics[
                    self.execution_residual_index["status"]
                ]
            ),
            "execution_residual_certificate_valid": bool(
                self.execution_residual_diagnostics[
                    self.execution_residual_index["certificate_valid"]
                ]
            ),
            "execution_residual_sample_count": int(
                self.execution_residual_diagnostics[
                    self.execution_residual_index["sample_count"]
                ]
            ),
            "execution_residual_maximum_error_ratio": float(
                self.execution_residual_diagnostics[
                    self.execution_residual_index["maximum_error_ratio"]
                ]
            ),
            "execution_residual_flags": int(
                self.execution_residual_diagnostics[
                    self.execution_residual_index["flags"]
                ]
            ),
            "execution_residual_step_ns": self.execution_residual_step_ns,
            "execution_residual_allocation_calls": self.execution_residual_allocation_calls,
            "execution_residual_allocated_bytes": self.execution_residual_allocated_bytes,
            "viability_request": self.viability_request,
        }


def read_state(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    twist = np.empty(6, np.float64)
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, base, twist, 0
    )
    q = np.empty(6, np.float64)
    v = np.empty(6, np.float64)
    for coordinate, name in enumerate(JOINT_ORDER):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        q[coordinate] = data.qpos[model.jnt_qposadr[joint]]
        v[coordinate] = data.qvel[model.jnt_dofadr[joint]]
    return (
        data.xpos[base].copy(),
        data.xquat[base].copy(),
        twist,
        q,
        v,
    )


def run_case(
    model_path: pathlib.Path,
    *,
    duration: float,
    push_start: float,
    push_duration: float,
    push_force: float,
    contact_model: str,
    balance_mode: str,
    capture_velocity_fraction: float,
    push_force_world: tuple[float, float, float] | np.ndarray | None = None,
    sliding_friction: float = 1.0,
    wbc_friction_coefficient: float | None = None,
    terminate_on_fall: bool = False,
    root_angular_task_weight: float = 10.0,
    root_roll_stiffness: float = 24.0,
    root_roll_damping: float = 4.4,
    root_lateral_stiffness: float = 18.0,
    root_lateral_damping: float = 8.0,
    fall_safe_enabled: bool = False,
    fall_safe_primary_blend: bool = True,
    execute_reduced_support: bool = True,
    observe_measured_contact: bool = False,
    support_contingency_enabled: bool = False,
    support_contingency_execute: bool = False,
    support_contingency_preserve_primary_support: bool = False,
    support_contingency_query_every_tick: bool = False,
) -> dict[str, np.ndarray | int]:
    import bonesaw

    force_world = (
        np.asarray([push_force, 0.0, 0.0], np.float64)
        if push_force_world is None
        else np.asarray(push_force_world, np.float64)
    )
    if force_world.shape != (3,) or not np.all(np.isfinite(force_world)):
        raise ValueError("push_force_world must contain three finite values")
    if wbc_friction_coefficient is None:
        wbc_friction_coefficient = min(sliding_friction, 0.8)
    if (
        not math.isfinite(wbc_friction_coefficient)
        or wbc_friction_coefficient <= 0.0
    ):
        raise ValueError("wbc_friction_coefficient must be finite and positive")
    model, data = make_plant(model_path, sliding_friction=sliding_friction)
    balance_session = bonesaw.UpkieBalanceSession(str(model_path))
    root_position, _, _, q, _ = read_state(model, data)
    balanced_root = np.empty(3, np.float64)
    balanced_q = np.empty(6, np.float64)
    balance_error = balance_session.balanced_standing(
        root_position, q, balanced_root, balanced_q
    )
    if abs(balance_error) > 1.0e-6:
        raise RuntimeError(
            f"balanced standing projection retained {balance_error:.3e} m CoM error"
        )
    root_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    data.qpos[model.jnt_qposadr[root_joint] : model.jnt_qposadr[root_joint] + 3] = (
        balanced_root
    )
    for coordinate, name in enumerate(JOINT_ORDER):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = balanced_q[coordinate]
    mujoco.mj_forward(model, data)
    if contact_model == "prescribed":
        model.geom_contype[:] = 0
        model.geom_conaffinity[:] = 0
        mujoco.mj_forward(model, data)
    elif contact_model != "soft":
        raise ValueError(f"unsupported contact model: {contact_model}")
    nominal_root_position, _, _, _, _ = read_state(model, data)
    wheel_bodies = np.asarray(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in (
                "left_ankle_mj5208_rotor",
                "right_ankle_mj5208_rotor",
            )
        ],
        np.int64,
    )
    target_ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
    wheel_contact_bodies = wheel_contact_body_sets(model, wheel_bodies)
    controller = RustWbcAdapter(
        model_path=model_path,
        nominal_root_position=nominal_root_position,
        target_ground_position=target_ground_position,
        balance_session=balance_session,
        balance_mode=balance_mode,
        capture_velocity_fraction=capture_velocity_fraction,
        friction_coefficient=wbc_friction_coefficient,
        root_angular_task_weight=root_angular_task_weight,
        root_roll_stiffness=root_roll_stiffness,
        root_roll_damping=root_roll_damping,
        root_lateral_stiffness=root_lateral_stiffness,
        root_lateral_damping=root_lateral_damping,
        fall_safe_enabled=fall_safe_enabled,
        fall_safe_primary_blend=fall_safe_primary_blend,
        execute_reduced_support=execute_reduced_support,
        support_contingency_enabled=support_contingency_enabled,
        support_contingency_execute=support_contingency_execute,
        support_contingency_preserve_primary_support=(
            support_contingency_preserve_primary_support
        ),
        support_contingency_query_every_tick=support_contingency_query_every_tick,
    )
    ticks = int(round(duration / CONTROL_DT))
    substeps = int(round(CONTROL_DT / PHYSICS_DT))
    if abs(substeps * PHYSICS_DT - CONTROL_DT) > 1.0e-12:
        raise ValueError("control period must be an integer number of physics steps")
    traces: dict[str, np.ndarray] = {
        "time_s": np.arange(ticks, dtype=np.float64) * CONTROL_DT,
        "root_position": np.empty((ticks, 3), np.float64),
        "root_quaternion": np.empty((ticks, 4), np.float64),
        "root_twist": np.empty((ticks, 6), np.float64),
        "rotation_vector": np.empty((ticks, 3), np.float64),
        "virtual_pitch": np.empty(ticks, np.float64),
        "capture_position": np.empty(ticks, np.float64),
        "capture_error": np.empty(ticks, np.float64),
        "capture_pressure": np.empty(ticks, np.float64),
        "station_authority": np.empty(ticks, np.float64),
        "station_error": np.empty(ticks, np.float64),
        "commanded_ground_velocity": np.empty(ticks, np.float64),
        "q": np.empty((ticks, 6), np.float64),
        "v": np.empty((ticks, 6), np.float64),
        "torque": np.empty((ticks, 6), np.float64),
        "generalized_acceleration": np.empty((ticks, 12), np.float64),
        "wbc_normal_force": np.empty((ticks, 2), np.float64),
        "external_force_x": np.empty(ticks, np.float64),
        "external_force_world": np.empty((ticks, 3), np.float64),
        "contact_count": np.empty(ticks, np.uint16),
        "measured_wheel_contact_active": np.empty((ticks, 2), np.uint8),
        "status": np.empty(ticks, np.uint8),
        "primary_status": np.empty(ticks, np.uint8),
        "support_contingency_admitted": np.empty(ticks, np.uint8),
        "support_contingency_requested": np.empty(ticks, np.uint8),
        "support_contingency_selected": np.empty(ticks, np.uint8),
        "support_contingency_mode": np.empty(ticks, np.uint8),
        "support_contingency_support_mask": np.empty(ticks, np.uint8),
        "support_contingency_status": np.empty(ticks, np.uint8),
        "support_contingency_maximum_constraint_violation": np.empty(ticks, np.float64),
        "support_contingency_author_step_ns": np.empty(ticks, np.uint64),
        "support_contingency_wbc_step_ns": np.empty(ticks, np.uint64),
        "maximum_constraint_violation": np.empty(ticks, np.float64),
        "minimum_bound_margin": np.empty(ticks, np.float64),
        "command_age_steps": np.empty(ticks, np.uint32),
        "fall_safe_mode": np.empty(ticks, np.uint8),
        "fall_safe_primary_authority": np.empty(ticks, np.float64),
        "fall_safe_fresh_command_authority": np.empty(ticks, np.float64),
        "fall_safe_risk": np.empty(ticks, np.float64),
        "fall_safe_reason_flags": np.empty(ticks, np.uint32),
        "controller_step_ns": np.empty(ticks, np.uint64),
        "plant_step_ns": np.empty(ticks, np.uint64),
        "loop_ns": np.empty(ticks, np.uint64),
        "allocation_calls": np.empty(ticks, np.uint64),
        "allocated_bytes": np.empty(ticks, np.uint64),
        "dynamics_residual": np.empty(ticks, np.float64),
        "contact_residual": np.empty(ticks, np.float64),
        "torque_utilization": np.empty(ticks, np.float64),
        "minimum_torque_headroom": np.empty(ticks, np.float64),
        "witness_acceleration_rms": np.empty(ticks, np.float64),
        "kinetic_energy_j": np.empty(ticks, np.float64),
    }
    actuator_ids = np.asarray(
        [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_motor"
            )
            for name in JOINT_ORDER
        ],
        np.int64,
    )
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    contact_bodies = np.asarray(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in CONTACT_FRAMES
        ],
        np.int64,
    )
    rolling_dofs = np.asarray(
        [
            model.jnt_dofadr[
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    JOINT_ORDER[coordinate],
                )
            ]
            for coordinate in ROLLING_COORDINATES
        ],
        np.int64,
    )
    zero_contact_torque = np.zeros(3, np.float64)
    gc.collect()
    gc_before = np.asarray([item["collections"] for item in gc.get_stats()])
    rss_before = rss_bytes()
    completed_ticks = ticks
    fall_time_s: float | None = None
    for tick, timestamp in enumerate(traces["time_s"]):
        loop_started = time.perf_counter_ns()
        root_position, root_quaternion, root_twist, q, v = read_state(model, data)
        measured_contact_active = (
            np.ones(2, np.uint8)
            if contact_model == "prescribed"
            else measured_wheel_ground_contacts(model, data, wheel_contact_bodies)
        )
        ground_position = float(np.mean(data.xpos[wheel_bodies, 0]))
        ground_height = float(np.mean(data.xpos[wheel_bodies, 2]))
        result = controller.solve(
            root_position,
            root_quaternion,
            root_twist,
            q,
            v,
            ground_position,
            ground_height,
            measured_contact_active if observe_measured_contact else None,
        )
        mujoco.mj_energyVel(model, data)
        kinetic_energy_j = float(data.energy[1])
        data.ctrl[actuator_ids] = result["torque"]
        disturbed = push_start <= timestamp < push_start + push_duration
        data.qfrc_applied.fill(0.0)
        data.xfrc_applied.fill(0.0)
        if contact_model == "prescribed":
            for row, body in enumerate(contact_bodies):
                force = result["contact_force_basis"][row]
                mujoco.mj_applyFT(
                    model,
                    data,
                    force,
                    zero_contact_torque,
                    data.xpos[body],
                    body,
                    data.qfrc_applied,
                )
                data.qfrc_applied[rolling_dofs[row]] += (
                    ROLLING_COEFFICIENTS[row] * force[0]
                )
        if disturbed and np.any(force_world != 0.0):
            data.xfrc_applied[base, :3] = force_world
        plant_started = time.perf_counter_ns()
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        plant_elapsed = time.perf_counter_ns() - plant_started
        traces["root_position"][tick] = root_position
        traces["root_quaternion"][tick] = root_quaternion
        traces["root_twist"][tick] = root_twist
        traces["rotation_vector"][tick] = quaternion_rotation_vector(root_quaternion)
        traces["virtual_pitch"][tick] = result["virtual_pitch"]
        traces["capture_position"][tick] = result["capture_position"]
        traces["capture_error"][tick] = result["capture_error"]
        traces["capture_pressure"][tick] = result["capture_pressure"]
        traces["station_authority"][tick] = result["station_authority"]
        traces["station_error"][tick] = result["station_error"]
        traces["commanded_ground_velocity"][tick] = result[
            "commanded_ground_velocity"
        ]
        traces["q"][tick] = q
        traces["v"][tick] = v
        traces["torque"][tick] = result["torque"]
        traces["generalized_acceleration"][tick] = result[
            "generalized_acceleration"
        ]
        traces["wbc_normal_force"][tick] = result["normal_force"]
        traces["external_force_x"][tick] = force_world[0] if disturbed else 0.0
        traces["external_force_world"][tick] = force_world if disturbed else 0.0
        traces["contact_count"][tick] = data.ncon
        traces["measured_wheel_contact_active"][tick] = measured_contact_active
        traces["status"][tick] = result["status"]
        traces["primary_status"][tick] = result["primary_status"]
        traces["support_contingency_admitted"][tick] = result[
            "support_contingency_admitted"
        ]
        traces["support_contingency_requested"][tick] = result[
            "support_contingency_requested"
        ]
        traces["support_contingency_selected"][tick] = result[
            "support_contingency_selected"
        ]
        traces["support_contingency_mode"][tick] = result[
            "support_contingency_mode"
        ]
        traces["support_contingency_support_mask"][tick] = result[
            "support_contingency_support_mask"
        ]
        traces["support_contingency_status"][tick] = result[
            "support_contingency_status"
        ]
        traces["support_contingency_maximum_constraint_violation"][tick] = result[
            "support_contingency_maximum_constraint_violation"
        ]
        traces["support_contingency_author_step_ns"][tick] = result[
            "support_contingency_author_step_ns"
        ]
        traces["support_contingency_wbc_step_ns"][tick] = result[
            "support_contingency_wbc_step_ns"
        ]
        traces["maximum_constraint_violation"][tick] = result[
            "maximum_constraint_violation"
        ]
        traces["minimum_bound_margin"][tick] = result["minimum_bound_margin"]
        traces["command_age_steps"][tick] = result["command_age_steps"]
        traces["fall_safe_mode"][tick] = result["fall_safe_mode"]
        traces["fall_safe_primary_authority"][tick] = result[
            "fall_safe_primary_authority"
        ]
        traces["fall_safe_fresh_command_authority"][tick] = result[
            "fall_safe_fresh_command_authority"
        ]
        traces["fall_safe_risk"][tick] = result["fall_safe_risk"]
        traces["fall_safe_reason_flags"][tick] = result[
            "fall_safe_reason_flags"
        ]
        traces["controller_step_ns"][tick] = result["step_ns"]
        traces["plant_step_ns"][tick] = plant_elapsed
        traces["allocation_calls"][tick] = result["allocation_calls"]
        traces["allocated_bytes"][tick] = result["allocated_bytes"]
        traces["dynamics_residual"][tick] = result["dynamics_residual"]
        traces["contact_residual"][tick] = result["contact_residual"]
        traces["torque_utilization"][tick] = result["torque_utilization"]
        traces["minimum_torque_headroom"][tick] = result[
            "minimum_torque_headroom"
        ]
        traces["witness_acceleration_rms"][tick] = result[
            "witness_acceleration_rms"
        ]
        traces["kinetic_energy_j"][tick] = kinetic_energy_j
        traces["loop_ns"][tick] = time.perf_counter_ns() - loop_started
        if terminate_on_fall:
            post_step_root, post_step_quaternion, _, _, _ = read_state(model, data)
            post_step_rotation = quaternion_rotation_vector(post_step_quaternion)
            if (
                float(post_step_root[2]) < 0.35
                or float(np.linalg.norm(post_step_rotation[:2]))
                > math.radians(45.0)
            ):
                completed_ticks = tick + 1
                fall_time_s = float(timestamp + CONTROL_DT)
                break
    if completed_ticks != ticks:
        for field, value in tuple(traces.items()):
            if isinstance(value, np.ndarray) and value.shape[0] == ticks:
                traces[field] = value[:completed_ticks].copy()
    gc_after = np.asarray([item["collections"] for item in gc.get_stats()])
    traces["gc_collections"] = int(np.sum(gc_after - gc_before))
    traces["rss_delta_bytes"] = rss_bytes() - rss_before
    traces["configured_ticks"] = ticks
    traces["terminated_on_fall"] = int(fall_time_s is not None)
    traces["fall_time_s"] = fall_time_s
    return traces


def status_counts(status: np.ndarray) -> dict[str, int]:
    return {
        STATUS_NAMES[int(value)]: int(np.sum(status == value))
        for value in np.unique(status)
    }


def longest_true_run(mask: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in mask:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def summarize_case(trace: dict[str, np.ndarray | int], push_end: float) -> dict[str, Any]:
    time_s = np.asarray(trace["time_s"])
    after = time_s >= push_end
    rotation = np.asarray(trace["rotation_vector"])
    tilt = np.linalg.norm(rotation[:, :2], axis=1)
    angular_speed = np.linalg.norm(np.asarray(trace["root_twist"])[:, :3], axis=1)
    q = np.asarray(trace["q"])
    v = np.asarray(trace["v"])
    posture_coordinates = np.asarray([0, 1, 3, 4])
    posture_error = q[:, posture_coordinates] - standing_posture()[posture_coordinates]
    status = np.asarray(trace["status"])
    admitted = np.isin(status, [0, 1])
    nonadmitted = ~admitted
    contactless = np.asarray(trace["contact_count"]) == 0
    root_x = np.asarray(trace["root_position"])[:, 0]
    station_error = np.asarray(trace["station_error"])
    station_authority = np.asarray(trace["station_authority"])
    dynamics_residual = np.asarray(trace["dynamics_residual"])
    contact_residual = np.asarray(trace["contact_residual"])
    hard_violation = np.asarray(trace["maximum_constraint_violation"])
    command_age_steps = np.asarray(trace["command_age_steps"])
    fall_safe_mode = np.asarray(trace["fall_safe_mode"])
    fall_safe_primary = np.asarray(trace["fall_safe_primary_authority"])
    fall_safe_fresh = np.asarray(trace["fall_safe_fresh_command_authority"])
    settle_mask = after & (tilt < math.radians(2.0)) & (angular_speed < 0.15)
    recovery_time = None
    required = int(round(0.4 / CONTROL_DT))
    indexes = np.flatnonzero(after)
    for index in indexes:
        if index + required <= len(time_s) and np.all(settle_mask[index : index + required]):
            recovery_time = float(time_s[index] - push_end)
            break
    station_reentry_time = None
    station_settle_mask = (np.abs(station_error) < 0.05) & (station_authority > 0.99)
    for index in indexes:
        if index + required <= len(time_s) and np.all(
            station_settle_mask[index : index + required]
        ):
            station_reentry_time = float(time_s[index] - push_end)
            break
    return {
        "status_counts": status_counts(status),
        "nonadmitted_steps": int(np.sum(nonadmitted)),
        "maximum_consecutive_nonadmitted_steps": longest_true_run(nonadmitted),
        "last_nonadmitted_time_s": (
            None if not np.any(nonadmitted) else float(time_s[np.flatnonzero(nonadmitted)[-1]])
        ),
        "post_startup_nonadmitted_steps": int(
            np.sum(nonadmitted & (time_s > STARTUP_TRANSIENT_S))
        ),
        "maximum_command_age_steps": int(np.max(command_age_steps)),
        "maximum_command_age_s": float(np.max(command_age_steps) * CONTROL_DT),
        "maximum_hard_constraint_violation": float(np.max(hard_violation)),
        "maximum_admitted_hard_constraint_violation": float(
            np.max(hard_violation[admitted])
        ),
        "fall_safe_mode_counts": {
            str(int(mode)): int(np.sum(fall_safe_mode == mode))
            for mode in np.unique(fall_safe_mode)
        },
        "minimum_fall_safe_primary_authority": float(np.min(fall_safe_primary)),
        "minimum_fall_safe_fresh_command_authority": float(np.min(fall_safe_fresh)),
        "maximum_fall_safe_risk": float(np.max(np.asarray(trace["fall_safe_risk"]))),
        "controller_step_ns": distribution(np.asarray(trace["controller_step_ns"])),
        "plant_step_ns": distribution(np.asarray(trace["plant_step_ns"])),
        "loop_ns": distribution(np.asarray(trace["loop_ns"])),
        "maximum_tilt_rad": float(np.max(tilt)),
        "final_tilt_rad": float(tilt[-1]),
        "maximum_angular_speed_radps": float(np.max(angular_speed)),
        "maximum_virtual_pitch_rad": float(
            np.max(np.abs(np.asarray(trace["virtual_pitch"])))
        ),
        "maximum_capture_error_m": float(
            np.max(np.abs(np.asarray(trace["capture_error"])))
        ),
        "maximum_capture_pressure": float(np.max(np.asarray(trace["capture_pressure"]))),
        "minimum_station_authority": float(np.min(np.asarray(trace["station_authority"]))),
        "final_station_authority": float(station_authority[-1]),
        "maximum_station_error_m": float(np.max(np.abs(station_error))),
        "final_station_error_m": float(station_error[-1]),
        "station_reentry_time_s": station_reentry_time,
        "maximum_commanded_ground_velocity_mps": float(
            np.max(np.abs(np.asarray(trace["commanded_ground_velocity"])))
        ),
        "initial_root_x_m": float(root_x[0]),
        "maximum_root_excursion_m": float(np.max(np.abs(root_x - root_x[0]))),
        "final_root_displacement_m": float(root_x[-1] - root_x[0]),
        "minimum_root_height_m": float(np.min(np.asarray(trace["root_position"])[:, 2])),
        "maximum_posture_error_rad": float(np.max(np.abs(posture_error))),
        "posture_tracking_rms_rad": float(math.sqrt(np.mean(posture_error**2))),
        "maximum_joint_speed_radps": float(np.max(np.abs(v))),
        "maximum_torque_utilization": float(np.max(np.asarray(trace["torque_utilization"]))),
        "minimum_contact_count": int(np.min(np.asarray(trace["contact_count"]))),
        "contactless_steps": int(np.sum(contactless)),
        "maximum_consecutive_contactless_steps": longest_true_run(contactless),
        "maximum_contactless_duration_s": longest_true_run(contactless) * CONTROL_DT,
        "last_contactless_time_s": (
            None if not np.any(contactless) else float(time_s[np.flatnonzero(contactless)[-1]])
        ),
        "maximum_dynamics_residual": float(np.max(dynamics_residual)),
        "maximum_contact_residual": float(np.max(contact_residual)),
        "maximum_admitted_dynamics_residual": float(np.max(dynamics_residual[admitted])),
        "maximum_admitted_contact_residual": float(np.max(contact_residual[admitted])),
        "maximum_witness_acceleration_rms": float(
            np.max(np.asarray(trace["witness_acceleration_rms"]))
        ),
        "maximum_kinetic_energy_j": float(np.max(np.asarray(trace["kinetic_energy_j"]))),
        "terminal_kinetic_energy_j": float(np.asarray(trace["kinetic_energy_j"])[-1]),
        "controller_allocation_calls": int(np.sum(np.asarray(trace["allocation_calls"]))),
        "controller_allocated_bytes": int(np.sum(np.asarray(trace["allocated_bytes"]))),
        "python_gc_collections": int(trace["gc_collections"]),
        "rss_delta_bytes": int(trace["rss_delta_bytes"]),
        "execution_over_5ms_steps": int(
            np.sum(np.asarray(trace["loop_ns"]) > int(CONTROL_DT * 1e9))
        ),
        "recovery_time_s": recovery_time,
        "fell": bool(
            trace.get("terminated_on_fall", 0)
            or np.min(np.asarray(trace["root_position"])[:, 2]) < 0.35
            or np.max(tilt) > math.radians(45.0)
        ),
        "fall_time_s": trace.get("fall_time_s"),
    }


def write_trace(path: pathlib.Path, trace: dict[str, np.ndarray | int]) -> None:
    headers = [
        "time_s", "root_x_m", "root_y_m", "root_z_m", "roll_rad", "pitch_rad",
        "yaw_rad", "virtual_pitch_rad", "capture_position_m", "capture_error_m",
        "capture_pressure", "station_authority", "station_error_m",
        "commanded_ground_velocity_mps",
        "angular_speed_radps", "linear_speed_mps", "force_x_n",
        "contacts", "status", "command_age_steps", "maximum_constraint_violation",
        "fall_safe_mode", "fall_safe_primary_authority",
        "fall_safe_fresh_command_authority", "fall_safe_risk",
        "fall_safe_reason_flags",
        "controller_step_ns", "plant_step_ns", "loop_ns",
        "torque_utilization", "dynamics_residual", "contact_residual",
    ] + [f"q_{name}" for name in JOINT_ORDER] + [f"v_{name}" for name in JOINT_ORDER] + [f"tau_{name}" for name in JOINT_ORDER]
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        ticks = len(np.asarray(trace["time_s"]))
        for tick in range(ticks):
            twist = np.asarray(trace["root_twist"])[tick]
            writer.writerow([
                np.asarray(trace["time_s"])[tick],
                *np.asarray(trace["root_position"])[tick],
                *np.asarray(trace["rotation_vector"])[tick],
                np.asarray(trace["virtual_pitch"])[tick],
                np.asarray(trace["capture_position"])[tick],
                np.asarray(trace["capture_error"])[tick],
                np.asarray(trace["capture_pressure"])[tick],
                np.asarray(trace["station_authority"])[tick],
                np.asarray(trace["station_error"])[tick],
                np.asarray(trace["commanded_ground_velocity"])[tick],
                np.linalg.norm(twist[:3]), np.linalg.norm(twist[3:]),
                np.asarray(trace["external_force_x"])[tick],
                np.asarray(trace["contact_count"])[tick],
                STATUS_NAMES[int(np.asarray(trace["status"])[tick])],
                np.asarray(trace["command_age_steps"])[tick],
                np.asarray(trace["maximum_constraint_violation"])[tick],
                np.asarray(trace["fall_safe_mode"])[tick],
                np.asarray(trace["fall_safe_primary_authority"])[tick],
                np.asarray(trace["fall_safe_fresh_command_authority"])[tick],
                np.asarray(trace["fall_safe_risk"])[tick],
                np.asarray(trace["fall_safe_reason_flags"])[tick],
                np.asarray(trace["controller_step_ns"])[tick],
                np.asarray(trace["plant_step_ns"])[tick],
                np.asarray(trace["loop_ns"])[tick],
                np.asarray(trace["torque_utilization"])[tick],
                np.asarray(trace["dynamics_residual"])[tick],
                np.asarray(trace["contact_residual"])[tick],
                *np.asarray(trace["q"])[tick],
                *np.asarray(trace["v"])[tick],
                *np.asarray(trace["torque"])[tick],
            ])


def make_report(metrics: dict[str, Any]) -> str:
    nominal = metrics["nominal"]
    pushed = metrics["pushed"]
    overload = metrics["overload"]
    contact_model = metrics["contact_model"]
    balance_mode = metrics["balance_mode"]
    station_reentry_text = (
        "not observed"
        if pushed["station_reentry_time_s"] is None
        else f'{pushed["station_reentry_time_s"]:.3f} s after push end'
    )
    if balance_mode == "capture":
        reference_details = (
            f'Capture mode presents **{metrics["capture_velocity_fraction"]:.3f}×** '
            "of the full DCM velocity offset to that PI loop while the full offset "
            "remains the viability-pressure witness."
        )
        station_evidence = (
            f'- Station preference re-entry: **{station_reentry_text}**; final error '
            f'**{pushed["final_station_error_m"]:.6f} m** at authority '
            f'**{pushed["final_station_authority"]:.4f}**.'
        )
    else:
        reference_details = (
            "Exact reference mode does not execute the rooted capture/station layer; "
            "those semantics are scored only in capture mode."
        )
        station_evidence = (
            "- Rooted capture/station evidence: **not applicable in exact reference mode**."
        )
    recovery_text = (
        "does not recover"
        if pushed["recovery_time_s"] is None
        else f'recovers in **{pushed["recovery_time_s"]:.3f} s**'
    )
    overload_text = "falls" if overload["fell"] else "does not fall"
    timing_rows = []
    for label, case in (
        ("nominal", nominal),
        (f'{metrics["push"]["force_x_n"]:g} N recovery probe', pushed),
        (f'{metrics["overload_push"]["force_x_n"]:g} N overload probe', overload),
    ):
        timing_rows.append([
            label,
            f'{case["controller_step_ns"]["p50"] / 1e3:.1f}',
            f'{case["controller_step_ns"]["p95"] / 1e3:.1f}',
            f'{case["controller_step_ns"]["p99"] / 1e3:.1f}',
            f'{case["loop_ns"]["p99"] / 1e3:.1f}',
            case["execution_over_5ms_steps"],
        ])
    behavior_rows = []
    for label, case in (("nominal", nominal), ("recovery probe", pushed), ("overload", overload)):
        behavior_rows.append([
            label,
            f'{math.degrees(case["maximum_tilt_rad"]):.3f}',
            f'{case["maximum_root_excursion_m"] * 1000:.2f}',
            f'{case["minimum_root_height_m"]:.4f}',
            f'{case["posture_tracking_rms_rad"]:.5f}',
            "—" if case["recovery_time_s"] is None else f'{case["recovery_time_s"]:.3f}',
            case["fell"],
        ])
    lines = [
        f'# Upkie MuJoCo plant audit · {metrics["revision"]}', "",
        f'**Admission: {"PASS" if metrics["admission"] else "FAIL"}.** '
        f"This is a retained closed-loop plant differential using the **{contact_model}** contact model and **{balance_mode}** Rust balance composition. Python owns MuJoCo integration, the external torso wrench, and experiment orchestration. Rust receives the observed floating state and returns inverse-dynamics WBC effort plus its rigid-contact witness. In `soft` mode MuJoCo owns collision response; in `prescribed` mode the Rust solved wrench is applied to MuJoCo as an ideal integration/correspondence bridge. The browser preview and this plant are not conflated.", "",
        "## Boundary under test", "",
        "```text",
        "MuJoCo q/v/root ──observed state──> Bonesaw Rust WBC ──torque──> MuJoCo",
        "      ^                                                        │",
        "      └──────────── Python-applied torso wrench ────────────────┘",
        "```", "",
        f'The task-reference adapter is deterministic and not learned policy. A fingerprinted Rust tools session owns the Upkie PI state, clamps, signed rolling coordinates, balanced-standing morphology projection, and axle-to-CoM virtual-pitch observation. {reference_details} Python owns only MuJoCo state transfer, scenario timing, disturbance, and scoring. The policy-/physics-free state-local WBC corpus remains the solver-semantics gate.', "",
        "## Timing and jitter", "",
        *markdown_table(["case", "Rust p50 µs", "Rust p95 µs", "Rust p99 µs", "full loop p99 µs", ">5 ms"], timing_rows), "",
        "## Physical response", "",
        *markdown_table(["case", "peak tilt deg", "peak Δx mm", "min z m", "posture RMS rad", "recovery s", "fell"], behavior_rows), "",
        "## Admission gates", "",
        *markdown_table(["gate", "observed", "pass"], [[key, value["observed"], value["pass"]] for key, value in metrics["gates"].items()]), "",
        "## Resource and authority evidence", "",
        f'- Rust timed-region allocations: **{pushed["controller_allocation_calls"]} calls / {pushed["controller_allocated_bytes"]} bytes**.',
        f'- Python GC collections during pushed loop: **{pushed["python_gc_collections"]}**; RSS delta: **{pushed["rss_delta_bytes"] / 1_048_576:.2f} MiB**.',
        f'- Peak actuator effort utilization: **{pushed["maximum_torque_utilization"]:.4f}**.',
        f'- Peak capture error/pressure and minimum station authority: **{pushed["maximum_capture_error_m"]:.4f} m / {pushed["maximum_capture_pressure"]:.4f} / {pushed["minimum_station_authority"]:.4f}**.',
        station_evidence,
        f'- Maximum admitted dynamics/contact residual: **{pushed["maximum_admitted_dynamics_residual"]:.3e} / {pushed["maximum_admitted_contact_residual"]:.3e}**. Rejected diagnostic candidates are reported separately at **{pushed["maximum_dynamics_residual"]:.3e} / {pushed["maximum_contact_residual"]:.3e}** and are never executed.',
        f'- Recovery-probe solver status counts: **{pushed["status_counts"]}**. The fail-closed startup hold lasts **{pushed["maximum_consecutive_nonadmitted_steps"]} ticks**, ending at **{pushed["last_nonadmitted_time_s"]:.3f} s**; admitted effort is then continuous through the disturbance.',
        f'- Recovery-probe contactless interval: **{pushed["maximum_contactless_duration_s"] * 1000:.1f} ms**; last contactless sample **{pushed["last_contactless_time_s"] if pushed["last_contactless_time_s"] is not None else "none"}**. Temporary flight is not called rigid-contact admission.', "",
        "## Measured recovery boundary", "",
        f'The declared recovery probe is **{metrics["push"]["force_x_n"]:g} N for {metrics["push"]["duration_s"] * 1000:.0f} ms ({metrics["push"]["impulse_ns"]:.3f} N·s)** and {recovery_text}. The otherwise identical **{metrics["overload_push"]["force_x_n"]:g} N ({metrics["overload_push"]["impulse_ns"]:.3f} N·s)** probe {overload_text}. This brackets one forward-push envelope only when the recovery probe passes and the overload probe fails; it is never an all-direction robustness claim.', "",
        "## Deliberate limits", "",
        "This plant is an evaluation adapter, not a reference implementation of MuJoCo dynamics inside Bonesaw. An explicit floating root is inserted before URDF import, static-body fusion is disabled to preserve Upkie's rotated fixed-link inertias, and the plant uses a ground plane plus direct URDF-limited torque motors without invented armature or damping. MuJoCo wheel/ground contact and Bonesaw's two rigid rolling constraints are not identical; that mismatch is part of the test. There is no estimator delay/noise, motor bandwidth, thermal model, terrain change, contact-mode estimator, network, or hardware calibration in this checkpoint. Per-tick Python dictionary/array boundary traffic remains outside the Rust allocation witness even though Python GC is zero; replacing it with fixed caller-owned plant outputs is still open.", "",
        "The physical browser gesture remains disabled until this adapter is connected to the live WebSocket process. A green target must never be relabeled as a push. The retained CSV traces contain every control tick for independent review.", "",
    ]
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    model_path = pathlib.Path(args.model).resolve()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    push_end = args.push_start + args.push_duration
    nominal_trace = run_case(
        model_path,
        duration=args.duration,
        push_start=args.push_start,
        push_duration=args.push_duration,
        push_force=0.0,
        contact_model=args.contact_model,
        balance_mode=args.balance_mode,
        capture_velocity_fraction=args.capture_velocity_fraction,
    )
    pushed_trace = run_case(
        model_path,
        duration=args.duration,
        push_start=args.push_start,
        push_duration=args.push_duration,
        push_force=args.push_force,
        contact_model=args.contact_model,
        balance_mode=args.balance_mode,
        capture_velocity_fraction=args.capture_velocity_fraction,
    )
    overload_trace = run_case(
        model_path,
        duration=args.duration,
        push_start=args.push_start,
        push_duration=args.push_duration,
        push_force=args.overload_force,
        contact_model=args.contact_model,
        balance_mode=args.balance_mode,
        capture_velocity_fraction=args.capture_velocity_fraction,
    )
    nominal = summarize_case(nominal_trace, push_end)
    pushed = summarize_case(pushed_trace, push_end)
    overload = summarize_case(overload_trace, push_end)
    delta_tilt = pushed["maximum_tilt_rad"] - nominal["maximum_tilt_rad"]
    gates = {
        "finite trace": {
            "observed": bool(all(np.all(np.isfinite(value)) for value in pushed_trace.values() if isinstance(value, np.ndarray) and value.dtype.kind == "f")),
            "pass": True,
        },
        "physical disturbance is observable": {
            "observed": f"peak tilt delta {math.degrees(delta_tilt):.4f} deg",
            "pass": delta_tilt > math.radians(0.02),
        },
        "nominal standing remains stable": {
            "observed": f'peak {math.degrees(nominal["maximum_tilt_rad"]):.4f} deg; fall {nominal["fell"]}',
            "pass": not nominal["fell"] and nominal["maximum_tilt_rad"] < math.radians(0.1),
        },
        "no fall": {"observed": pushed["fell"], "pass": not pushed["fell"]},
        "post-disturbance recovery": {
            "observed": pushed["recovery_time_s"],
            "pass": pushed["recovery_time_s"] is not None,
        },
        "station preference re-enters after recovery": (
            {
                "observed": f'{pushed["station_reentry_time_s"]} s; final error {pushed["final_station_error_m"]:.6f} m; authority {pushed["final_station_authority"]:.6f}',
                "pass": pushed["station_reentry_time_s"] is not None
                and abs(pushed["final_station_error_m"]) < 0.05
                and pushed["final_station_authority"] > 0.99,
            }
            if args.balance_mode == "capture"
            else {
                "observed": "not applicable in exact reference mode",
                "pass": True,
            }
        ),
        "controller timed region allocation-free": {
            "observed": f'{pushed["controller_allocation_calls"]} calls / {pushed["controller_allocated_bytes"]} bytes',
            "pass": pushed["controller_allocation_calls"] == 0 and pushed["controller_allocated_bytes"] == 0,
        },
        "5 ms loop budget": {
            "observed": f'{pushed["execution_over_5ms_steps"]} overruns',
            "pass": pushed["execution_over_5ms_steps"] == 0,
        },
        "admitted hard equation residual": {
            "observed": f'{max(pushed["maximum_admitted_dynamics_residual"], pushed["maximum_admitted_contact_residual"]):.3e}',
            "pass": max(pushed["maximum_admitted_dynamics_residual"], pushed["maximum_admitted_contact_residual"]) < 1.0e-7,
        },
        "bounded fail-closed startup": {
            "observed": f'{pushed["nonadmitted_steps"]} ticks; longest {pushed["maximum_consecutive_nonadmitted_steps"]}; last {pushed["last_nonadmitted_time_s"]} s',
            "pass": pushed["maximum_consecutive_nonadmitted_steps"]
            <= int(round(STARTUP_TRANSIENT_S / CONTROL_DT))
            and pushed["last_nonadmitted_time_s"] is not None
            and pushed["last_nonadmitted_time_s"] <= STARTUP_TRANSIENT_S + 1.0e-12,
        },
        "solver admitted after startup": {
            "observed": f'{pushed["post_startup_nonadmitted_steps"]} nonadmitted ticks',
            "pass": pushed["post_startup_nonadmitted_steps"] == 0,
        },
        "contact is reacquired": {
            "observed": f'{pushed["maximum_contactless_duration_s"]:.3f} s maximum flight; last {pushed["last_contactless_time_s"]}',
            "pass": pushed["maximum_contactless_duration_s"] <= 0.25
            and (pushed["last_contactless_time_s"] is None or pushed["last_contactless_time_s"] < args.duration - 0.4),
        },
        "overload boundary is discriminating": {
            "observed": f'{args.overload_force:g} N fall {overload["fell"]}; {overload["post_startup_nonadmitted_steps"]} later nonadmitted ticks',
            "pass": overload["fell"] or overload["post_startup_nonadmitted_steps"] > 0,
        },
    }
    gates["finite trace"]["pass"] = bool(gates["finite trace"]["observed"])
    metrics = {
        "schema_version": 1,
        "revision": args.revision,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "host": {"platform": platform.platform(), "processor": platform.machine()},
        "mujoco_version": mujoco.__version__,
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "control_dt_s": CONTROL_DT,
        "physics_dt_s": PHYSICS_DT,
        "contact_model": args.contact_model,
        "balance_mode": args.balance_mode,
        "capture_velocity_fraction": args.capture_velocity_fraction,
        "duration_s": args.duration,
        "push": {"start_s": args.push_start, "duration_s": args.push_duration, "force_x_n": args.push_force, "body": "base"},
        "overload_push": {"start_s": args.push_start, "duration_s": args.push_duration, "force_x_n": args.overload_force, "body": "base"},
        "nominal": nominal,
        "pushed": pushed,
        "overload": overload,
        "gates": gates,
        "admission": all(item["pass"] for item in gates.values()),
    }
    metrics["push"]["impulse_ns"] = args.push_force * args.push_duration
    metrics["overload_push"]["impulse_ns"] = args.overload_force * args.push_duration
    write_trace(output / "nominal-trace.csv", nominal_trace)
    write_trace(output / "pushed-trace.csv", pushed_trace)
    write_trace(output / "overload-trace.csv", overload_trace)
    metrics_path = output / "upkie-mujoco-plant-metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path = output / "UPKIE_MUJOCO_PLANT_AUDIT.md"
    report_path.write_text(report)
    web_report = pathlib.Path(args.web_report)
    web_report.write_text(render_report_html(report, title="Upkie MuJoCo plant audit"))
    print(json.dumps({"admission": metrics["admission"], "metrics": str(metrics_path), "report": str(report_path), "web_report": str(web_report), "pushed": pushed}, indent=2))
    return 0 if metrics["admission"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
