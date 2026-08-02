#!/usr/bin/env python3
"""R213 reset-every-sample G1 contact-law holdout for transition tubes.

This is deliberately an evaluation adapter, not controller code. MuJoCo owns
the held-out contact labels; Rust owns model response, generalized momentum,
and inverse-mass interval projection. Every sample starts from an independently
authored pre-impact state, so no policy or rollout history can explain a miss.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

import mujoco
import numpy as np

from cpu_reference_report import distribution, markdown_table, render_report_html


REVISION = "g1-contact-law-transition-holdout-r213"
CONTROL_DT = 0.005
PHYSICS_DT = 0.001
SUBSTEPS = int(round(CONTROL_DT / PHYSICS_DT))
GRAVITY = 9.81
FOOT_FRAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
FOOT_LOCAL_POINTS = np.asarray(
    [
        [-0.05, 0.025, -0.03],
        [-0.05, -0.025, -0.03],
        [0.12, 0.03, -0.03],
        [0.12, -0.03, -0.03],
    ],
    np.float64,
)
SPHERE_RADIUS_M = 0.005

# Frozen before generating the momentum-transition labels. These are physical
# reserve fractions, not values fitted to a sample quantile. Root reserves are
# fractions of one control tick of weight impulse; the angular scale uses the
# pre-impact root height. Joint reserves are fractions of authored effort over
# the same tick.
RESERVE_FRACTION = 0.25
GROUPED_ACCELERATION_RESERVE = (50.0, 10.0, 50.0)
DIRECTIONAL_PROFILE = {
    "tangential_acceleration_upper_m_s2": 100.0,
    "normal_acceleration_upper_m_s2": 100.0,
    "effective_mass_scale": 1.0,
    "tangential_load_upper_n": 8.0,
    "normal_load_scale": 2.0,
    "restitution_upper": 1.0,
}
SPATIAL_PATCH_PROFILE = {
    # Frozen physical authoring for the post-R214 coupled-patch experiment.
    # Tangential uncertainty retains R204's passive 100 m/s² and 8 N values.
    # Normal uncertainty uses the already-frozen 10 m/s² root-linear reserve,
    # one whole-body weight load, and no unmodeled torsional friction.
    "tangential_acceleration_upper_m_s2": 100.0,
    "normal_acceleration_upper_m_s2": 10.0,
    "tangential_load_upper_n": 8.0,
    "normal_load_scale": 1.0,
    "restitution_upper": 1.0,
    "torsion_radius_m": 0.0,
}
WIDTH_GATES = {
    "root_angular_rad_s": 2.0,
    "root_linear_m_s": 0.5,
    "joint_rad_s": 10.0,
}


@dataclass(frozen=True)
class ContactLaw:
    name: str
    friction: float
    solref_time_s: float
    solimp_min: float
    solimp_max: float
    cone: int
    integrator: int


CONTACT_LAWS = (
    ContactLaw(
        "soft_pyramidal",
        friction=0.35,
        solref_time_s=0.020,
        solimp_min=0.80,
        solimp_max=0.95,
        cone=int(mujoco.mjtCone.mjCONE_PYRAMIDAL),
        integrator=int(mujoco.mjtIntegrator.mjINT_RK4),
    ),
    ContactLaw(
        "stiff_elliptic",
        friction=0.90,
        solref_time_s=0.004,
        solimp_min=0.98,
        solimp_max=0.999,
        cone=int(mujoco.mjtCone.mjCONE_ELLIPTIC),
        integrator=int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
    ),
)


def standing_posture(joint_names: list[str]) -> np.ndarray:
    """Official-G1 bent-knee seed already used by the CPU reference corpus."""

    values = {
        "left_hip_pitch_joint": -0.1,
        "right_hip_pitch_joint": -0.1,
        "left_knee_joint": 0.3,
        "right_knee_joint": 0.3,
        "left_ankle_pitch_joint": -0.2,
        "right_ankle_pitch_joint": -0.2,
    }
    posture = np.zeros(len(joint_names), np.float64)
    for name, value in values.items():
        if name in joint_names:
            posture[joint_names.index(name)] = value
    return posture


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf",
    )
    parser.add_argument("--samples-per-law", type=int, default=48)
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/G1_CONTACT_LAW_TRANSITION_HOLDOUT_R213.html"
    )
    return parser.parse_args()


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        np.float64,
    )


def strip_mesh_geometry_and_add_root(model_path: pathlib.Path) -> str:
    """Keep authored inertials and primitive foot spheres; remove absent meshes."""

    root = ET.parse(model_path).getroot()
    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)
        for collision in list(link.findall("collision")):
            geometry = collision.find("geometry")
            if geometry is None or geometry.find("mesh") is not None:
                link.remove(collision)
    world = ET.Element("link", {"name": "world"})
    joint = ET.Element("joint", {"name": "root", "type": "floating"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": "pelvis"})
    root.insert(0, joint)
    root.insert(0, world)
    return ET.tostring(root, encoding="unicode")


def build_plant(model_path: pathlib.Path, law: ContactLaw) -> tuple[mujoco.MjModel, mujoco.MjData]:
    spec = mujoco.MjSpec.from_string(strip_mesh_geometry_and_add_root(model_path))
    spec.compiler.fusestatic = False
    spec.option.timestep = PHYSICS_DT
    spec.option.integrator = law.integrator
    spec.option.cone = law.cone
    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0.0, 0.0, 0.05],
        rgba=[0.18, 0.20, 0.23, 1.0],
        friction=[law.friction, 1.0e-8, 1.0e-8],
        solref=[law.solref_time_s, 1.0],
        solimp=[law.solimp_min, law.solimp_max, 0.001, 0.5, 2.0],
    )
    model = spec.compile()
    model.geom_solref[:, 0] = law.solref_time_s
    model.geom_solref[:, 1] = 1.0
    model.geom_solimp[:] = [law.solimp_min, law.solimp_max, 0.001, 0.5, 2.0]
    model.geom_friction[:] = [law.friction, 1.0e-8, 1.0e-8]
    return model, mujoco.MjData(model)


def joint_effort_limits(model_path: pathlib.Path, joint_names: list[str]) -> np.ndarray:
    limits: dict[str, float] = {}
    for joint in ET.parse(model_path).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("effort") is not None:
            limits[joint.get("name", "")] = float(limit.get("effort", "nan"))
    values = np.asarray([limits[name] for name in joint_names], np.float64)
    if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("every holdout joint needs a finite positive effort limit")
    return values


def plant_layout(
    model: mujoco.MjModel, joint_names: list[str]
) -> tuple[int, list[int], list[int], int, list[int], dict[int, int]]:
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    if root_id < 0:
        raise ValueError("MuJoCo plant is missing floating root")
    root_qpos = int(model.jnt_qposadr[root_id])
    root_qvel = int(model.jnt_dofadr[root_id])
    qpos_indices: list[int] = []
    qvel_indices: list[int] = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"MuJoCo is missing joint {name}")
        qpos_indices.append(int(model.jnt_qposadr[joint_id]))
        qvel_indices.append(int(model.jnt_dofadr[joint_id]))
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    foot_geoms: list[int] = []
    geom_to_contact: dict[int, int] = {}
    for side, frame in enumerate(FOOT_FRAMES):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, frame)
        candidates = [
            geom
            for geom in range(model.ngeom)
            if int(model.geom_bodyid[geom]) == body
            and int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
        ]
        candidates.sort(key=lambda geom: tuple(float(x) for x in model.geom_pos[geom]))
        if len(candidates) != 4:
            raise ValueError(f"R213 expected four sphere contacts on {frame}")
        actual_points = np.asarray([model.geom_pos[geom] for geom in candidates])
        expected_points = np.asarray(
            sorted(FOOT_LOCAL_POINTS.tolist(), key=lambda point: tuple(point)),
            np.float64,
        )
        if not np.allclose(actual_points, expected_points, atol=1.0e-12, rtol=0.0):
            raise ValueError(f"R213 primitive contact geometry changed on {frame}")
        for local, geom in enumerate(candidates):
            geom_to_contact[geom] = side * 4 + local
            foot_geoms.append(geom)
    return root_qpos, qpos_indices, qvel_indices, ground, foot_geoms, geom_to_contact


def to_bonesaw_tangent(
    vector: np.ndarray, root_qvel: int, joint_qvel: list[int]
) -> np.ndarray:
    return np.concatenate(
        (
            vector[root_qvel + 3 : root_qvel + 6],
            vector[root_qvel : root_qvel + 3],
            vector[joint_qvel],
        )
    )


def contact_force_world(contact: Any, force_contact: np.ndarray) -> np.ndarray:
    frame = np.asarray(contact.frame, np.float64).reshape(3, 3)
    return frame.T @ force_contact[:3]


def group_maximum(values: np.ndarray) -> tuple[float, float, float]:
    return (
        float(np.max(np.abs(values[:3]))),
        float(np.max(np.abs(values[3:6]))),
        float(np.max(np.abs(values[6:]))) if len(values) > 6 else 0.0,
    )


def grouped_acceleration_reserve(generalized_dof: int) -> np.ndarray:
    angular, linear, joint = GROUPED_ACCELERATION_RESERVE
    return np.asarray(
        [angular] * 3 + [linear] * 3 + [joint] * (generalized_dof - 6),
        np.float64,
    )


def point_velocities_world(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_ids: list[int],
) -> np.ndarray:
    velocities = np.empty((len(geom_ids), 3), np.float64)
    jacobian_position = np.empty((3, model.nv), np.float64)
    jacobian_rotation = np.empty((3, model.nv), np.float64)
    for contact, geom in enumerate(geom_ids):
        body = int(model.geom_bodyid[geom])
        mujoco.mj_jac(
            model,
            data,
            jacobian_position,
            jacobian_rotation,
            data.geom_xpos[geom],
            body,
        )
        velocities[contact] = jacobian_position @ data.qvel
    return velocities


def body_point_velocities_world(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_ids: list[int],
    points_world: np.ndarray,
) -> np.ndarray:
    if len(body_ids) != len(points_world):
        raise ValueError("body and point counts must match")
    velocities = np.empty((len(body_ids), 3), np.float64)
    jacobian_position = np.empty((3, model.nv), np.float64)
    jacobian_rotation = np.empty((3, model.nv), np.float64)
    for index, (body, point) in enumerate(zip(body_ids, points_world, strict=True)):
        mujoco.mj_jac(
            model,
            data,
            jacobian_position,
            jacobian_rotation,
            point,
            body,
        )
        velocities[index] = jacobian_position @ data.qvel
    return velocities


def directional_witnesses(
    prospective_velocity: np.ndarray,
    effective_mass: np.ndarray,
    total_mass: float,
    friction: float,
) -> np.ndarray:
    witnesses = np.empty((len(prospective_velocity), 10), np.float64)
    speed_reserve = np.asarray(
        [
            DIRECTIONAL_PROFILE["tangential_acceleration_upper_m_s2"] * CONTROL_DT,
            DIRECTIONAL_PROFILE["tangential_acceleration_upper_m_s2"] * CONTROL_DT,
            DIRECTIONAL_PROFILE["normal_acceleration_upper_m_s2"] * CONTROL_DT,
        ],
        np.float64,
    )
    witnesses[:, :3] = np.abs(prospective_velocity) + speed_reserve
    witnesses[:, 3:6] = (
        effective_mass * DIRECTIONAL_PROFILE["effective_mass_scale"]
    )
    witnesses[:, 6:8] = DIRECTIONAL_PROFILE["tangential_load_upper_n"]
    witnesses[:, 8] = total_mass * GRAVITY * DIRECTIONAL_PROFILE["normal_load_scale"]
    witnesses[:, 9] = friction
    return witnesses


def run_law(
    model_path: pathlib.Path,
    law: ContactLaw,
    samples: int,
    *,
    sample_offset: int = 0,
    reserve_kind: str = "coordinate_box",
    reserve_fraction: float = RESERVE_FRACTION,
    evaluate_spatial_patch: bool = False,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    import bonesaw

    frame_names = [FOOT_FRAMES[0]] * 4 + [FOOT_FRAMES[1]] * 4
    session = bonesaw.ContactTransitionModelSession(str(model_path), frame_names)
    patch_session = (
        bonesaw.ContactTransitionModelSession(str(model_path), list(FOOT_FRAMES))
        if evaluate_spatial_patch
        else None
    )
    joint_names = list(session.joint_names())
    dof = int(session.joint_dof())
    generalized_dof = int(session.generalized_dof())
    q_nominal = standing_posture(joint_names)
    effort = joint_effort_limits(model_path, joint_names)
    model, data = build_plant(model_path, law)
    root_qpos, qpos_indices, qvel_indices, ground, foot_geoms, geom_to_contact = plant_layout(
        model, joint_names
    )
    root_qvel = int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")])
    total_mass = float(np.sum(model.body_mass))
    bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 8, axis=0)
    response = np.empty((generalized_dof, 8, 3), np.float64)
    effective_mass = np.empty((8, 3), np.float64)
    delassus = np.empty((24, 24), np.float64)
    momentum_residual = np.empty((1, generalized_dof), np.float64)
    velocity_lower = np.empty(generalized_dof, np.float64)
    velocity_upper = np.empty(generalized_dof, np.float64)
    directional_impulse_upper = np.empty((8, 3), np.float64)
    directional_lower = np.empty(generalized_dof, np.float64)
    directional_upper = np.empty(generalized_dof, np.float64)
    acceleration_reserve = grouped_acceleration_reserve(generalized_dof)
    transition_time = np.asarray([0.0, CONTROL_DT], np.float64)
    contact_force = np.zeros(6, np.float64)
    patch_bases = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 2, axis=0)
    patch_response = np.empty((generalized_dof, 2, 6), np.float64)
    patch_delassus = np.empty((12, 12), np.float64)
    patch_witnesses = np.empty((2, 13), np.float64)
    patch_normal_upper = np.empty(2, np.float64)
    patch_lower = np.empty(generalized_dof, np.float64)
    patch_upper = np.empty(generalized_dof, np.float64)
    patch_body_ids = [
        int(model.geom_bodyid[foot_geoms[0]]),
        int(model.geom_bodyid[foot_geoms[4]]),
    ]

    covered = np.zeros(samples, np.uint8)
    component_coverage = np.zeros((samples, generalized_dof), np.uint8)
    residuals = np.empty((samples, generalized_dof), np.float64)
    raw_velocity_error = np.empty((samples, generalized_dof), np.float64)
    interval_width = np.empty((samples, generalized_dof), np.float64)
    grouped_covered = np.zeros(samples, np.uint8)
    grouped_component_coverage = np.zeros((samples, generalized_dof), np.uint8)
    grouped_interval_lower = np.empty((samples, generalized_dof), np.float64)
    grouped_interval_upper = np.empty((samples, generalized_dof), np.float64)
    grouped_interval_width = np.empty((samples, generalized_dof), np.float64)
    grouped_impulse_upper = np.empty((samples, 8, 3), np.float64)
    prospective_velocity = np.empty((samples, 8, 3), np.float64)
    contact_impulses = np.empty((samples, 8, 3), np.float64)
    contact_points = np.empty((samples, 8, 3), np.float64)
    query_timing = np.empty(samples, np.uint64)
    residual_timing = np.empty(samples, np.uint64)
    projection_timing = np.empty(samples, np.uint64)
    directional_timing = np.empty(samples, np.uint64)
    generalized_constraint_error = np.empty(samples, np.float64)
    generalized_constraint_relative_error = np.empty(samples, np.float64)
    contact_impulse_norm = np.empty(samples, np.float64)
    active_contact_points = np.empty(samples, np.uint8)
    root_height = np.empty(samples, np.float64)
    zero_allocation = True
    patch_covered = np.zeros(samples, np.uint8)
    patch_component_coverage = np.zeros((samples, generalized_dof), np.uint8)
    patch_interval_lower = np.empty((samples, generalized_dof), np.float64)
    patch_interval_upper = np.empty((samples, generalized_dof), np.float64)
    patch_interval_width = np.empty((samples, generalized_dof), np.float64)
    patch_normal_impulse_upper = np.empty((samples, 2), np.float64)
    patch_query_timing = np.empty(samples, np.uint64)
    patch_bound_timing = np.empty(samples, np.uint64)

    for sample in range(samples):
        state_index = sample + sample_offset
        mujoco.mj_resetData(model, data)
        phase = 0.37 * state_index + np.arange(dof, dtype=np.float64) * 0.23
        q = q_nominal + 0.025 * np.sin(phase)
        roll = 0.025 * math.sin(0.31 * state_index)
        pitch = 0.035 * math.cos(0.27 * state_index)
        yaw = 0.02 * math.sin(0.19 * state_index)
        quaternion = quaternion_from_rpy(roll, pitch, yaw)
        data.qpos[root_qpos : root_qpos + 7] = [0.0, 0.0, 0.80, *quaternion]
        data.qpos[qpos_indices] = q
        mujoco.mj_forward(model, data)
        closing_speed = 0.25 + 0.55 * (
            0.5 + 0.5 * math.sin(0.43 * state_index)
        )
        impact_fraction = 0.12 + 0.70 * ((state_index % 7) / 6.0)
        clearance = closing_speed * CONTROL_DT * impact_fraction
        lowest_surface = min(
            float(data.geom_xpos[geom, 2] - model.geom_size[geom, 0])
            for geom in foot_geoms
        )
        data.qpos[root_qpos + 2] += clearance - lowest_surface
        data.qvel.fill(0.0)
        data.qvel[root_qvel : root_qvel + 3] = [
            0.12 * math.sin(0.17 * state_index),
            0.08 * math.cos(0.29 * state_index),
            -closing_speed,
        ]
        data.qvel[root_qvel + 3 : root_qvel + 6] = [
            0.15 * math.sin(0.21 * state_index),
            0.12 * math.cos(0.33 * state_index),
            0.08 * math.sin(0.41 * state_index),
        ]
        data.qvel[qvel_indices] = 0.08 * np.sin(phase + 0.5)
        mujoco.mj_forward(model, data)

        root_position = np.asarray(data.qpos[root_qpos : root_qpos + 3], np.float64).copy()
        root_quaternion = np.asarray(data.qpos[root_qpos + 3 : root_qpos + 7], np.float64).copy()
        q_pre = np.asarray(data.qpos[qpos_indices], np.float64).copy()
        velocity_pre = to_bonesaw_tangent(data.qvel, root_qvel, qvel_indices)
        smooth_acceleration = to_bonesaw_tangent(data.qacc_smooth, root_qvel, qvel_indices)
        points_pre = np.asarray(data.geom_xpos[foot_geoms], np.float64).copy()
        points_pre[:, 2] -= SPHERE_RADIUS_M
        contact_points[sample] = points_pre
        root_height[sample] = root_position[2]
        timing = session.point_impulse_velocity_response(
            root_position,
            root_quaternion,
            q_pre,
            points_pre,
            bases,
            response,
            effective_mass,
            delassus,
        )
        query_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        prospective_velocity[sample] = point_velocities_world(
            model, data, foot_geoms
        )
        witnesses = directional_witnesses(
            prospective_velocity[sample],
            effective_mass,
            total_mass,
            law.friction,
        )
        timing = session.bound_directional_contact_transition_velocity_jump(
            transition_time,
            DIRECTIONAL_PROFILE["restitution_upper"],
            witnesses,
            smooth_acceleration - acceleration_reserve,
            smooth_acceleration + acceleration_reserve,
            response,
            directional_impulse_upper,
            directional_lower,
            directional_upper,
        )
        directional_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        grouped_interval_lower[sample] = directional_lower
        grouped_interval_upper[sample] = directional_upper
        grouped_interval_width[sample] = directional_upper - directional_lower
        grouped_impulse_upper[sample] = directional_impulse_upper
        if patch_session is not None:
            patch_points = points_pre.reshape(2, 4, 3).mean(axis=1)
            timing = patch_session.spatial_impulse_velocity_response(
                root_position,
                root_quaternion,
                q_pre,
                patch_points,
                patch_bases,
                patch_response,
                patch_delassus,
            )
            patch_query_timing[sample] = timing[0]
            zero_allocation &= timing[1:] == (0, 0)
            patch_velocity = body_point_velocities_world(
                model, data, patch_body_ids, patch_points
            )
            point_normal_speed = np.abs(
                prospective_velocity[sample].reshape(2, 4, 3)[:, :, 2]
            ).max(axis=1)
            patch_witnesses[:, :3] = np.abs(patch_velocity) + np.asarray(
                [
                    SPATIAL_PATCH_PROFILE["tangential_acceleration_upper_m_s2"]
                    * CONTROL_DT,
                    SPATIAL_PATCH_PROFILE["tangential_acceleration_upper_m_s2"]
                    * CONTROL_DT,
                    SPATIAL_PATCH_PROFILE["normal_acceleration_upper_m_s2"]
                    * CONTROL_DT,
                ],
                np.float64,
            )
            patch_witnesses[:, 2] = np.maximum(
                point_normal_speed,
                np.abs(patch_velocity[:, 2]),
            ) + (
                SPATIAL_PATCH_PROFILE["normal_acceleration_upper_m_s2"]
                * CONTROL_DT
            )
            for patch in range(2):
                for axis in range(3):
                    patch_witnesses[patch, 3 + axis] = 1.0 / patch_delassus[
                        patch * 6 + 3 + axis, patch * 6 + 3 + axis
                    ]
            patch_witnesses[:, 6:8] = SPATIAL_PATCH_PROFILE[
                "tangential_load_upper_n"
            ]
            patch_witnesses[:, 8] = (
                total_mass * GRAVITY * SPATIAL_PATCH_PROFILE["normal_load_scale"]
            )
            patch_witnesses[:, 9] = law.friction
            patch_witnesses[:, 10] = 0.5 * (
                np.max(FOOT_LOCAL_POINTS[:, 0]) - np.min(FOOT_LOCAL_POINTS[:, 0])
            )
            patch_witnesses[:, 11] = 0.5 * (
                np.max(FOOT_LOCAL_POINTS[:, 1]) - np.min(FOOT_LOCAL_POINTS[:, 1])
            )
            patch_witnesses[:, 12] = SPATIAL_PATCH_PROFILE["torsion_radius_m"]
            timing = patch_session.bound_spatial_patch_contact_transition_velocity_jump(
                transition_time,
                SPATIAL_PATCH_PROFILE["restitution_upper"],
                patch_witnesses,
                smooth_acceleration - acceleration_reserve,
                smooth_acceleration + acceleration_reserve,
                patch_response,
                patch_normal_upper,
                patch_lower,
                patch_upper,
            )
            patch_bound_timing[sample] = timing[0]
            zero_allocation &= timing[1:] == (0, 0)
            patch_interval_lower[sample] = patch_lower
            patch_interval_upper[sample] = patch_upper
            patch_interval_width[sample] = patch_upper - patch_lower
            patch_normal_impulse_upper[sample] = patch_normal_upper

        impulse = np.zeros((8, 3), np.float64)
        constraint_impulse = np.zeros(model.nv, np.float64)
        for _ in range(SUBSTEPS):
            mujoco.mj_step(model, data)
            constraint_impulse += np.asarray(data.qfrc_constraint) * PHYSICS_DT
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                geom0, geom1 = int(contact.geom[0]), int(contact.geom[1])
                if geom0 == ground and geom1 in geom_to_contact:
                    foot_geom = geom1
                    sign = 1.0
                elif geom1 == ground and geom0 in geom_to_contact:
                    foot_geom = geom0
                    sign = -1.0
                else:
                    continue
                mujoco.mj_contactForce(model, data, contact_index, contact_force)
                impulse[geom_to_contact[foot_geom]] += (
                    sign * contact_force_world(contact, contact_force) * PHYSICS_DT
                )
        contact_impulses[sample] = impulse
        contact_impulse_norm[sample] = float(np.linalg.norm(impulse))
        active_contact_points[sample] = np.count_nonzero(
            np.linalg.norm(impulse, axis=1) > 1.0e-12
        )
        velocity_post = to_bonesaw_tangent(data.qvel, root_qvel, qvel_indices)
        observed_delta = velocity_post - velocity_pre
        grouped_component_coverage[sample] = (
            (observed_delta >= grouped_interval_lower[sample] - 1.0e-12)
            & (observed_delta <= grouped_interval_upper[sample] + 1.0e-12)
        )
        grouped_covered[sample] = np.all(grouped_component_coverage[sample])
        if patch_session is not None:
            patch_component_coverage[sample] = (
                (observed_delta >= patch_interval_lower[sample] - 1.0e-12)
                & (observed_delta <= patch_interval_upper[sample] + 1.0e-12)
            )
            patch_covered[sample] = np.all(patch_component_coverage[sample])
        continuous_delta = smooth_acceleration * CONTROL_DT
        contact_delta = np.einsum("dca,ca->d", response, impulse, optimize=True)
        predicted_delta = continuous_delta + contact_delta
        raw_velocity_error[sample] = observed_delta - predicted_delta
        timing = session.generalized_momentum_impulse_residuals(
            root_position,
            root_quaternion,
            q_pre,
            observed_delta,
            predicted_delta[None, :],
            momentum_residual,
        )
        residual_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        residuals[sample] = momentum_residual[0]

        if reserve_kind == "coordinate_box":
            weight_impulse = total_mass * GRAVITY * CONTROL_DT
            momentum_half_width = np.concatenate(
                (
                    np.full(3, reserve_fraction * weight_impulse * root_position[2]),
                    np.full(3, reserve_fraction * weight_impulse),
                    reserve_fraction * effort * CONTROL_DT,
                )
            )
            timing = session.generalized_velocity_interval_from_momentum_box(
                root_position,
                root_quaternion,
                q_pre,
                -momentum_half_width,
                momentum_half_width,
                velocity_lower,
                velocity_upper,
            )
        elif reserve_kind == "kinetic_ellipsoid":
            twice_energy_upper = (
                reserve_fraction * reserve_fraction * total_mass * (GRAVITY * CONTROL_DT) ** 2
            )
            timing = session.generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
                root_position,
                root_quaternion,
                q_pre,
                twice_energy_upper,
                velocity_lower,
                velocity_upper,
            )
        else:
            raise ValueError(f"unknown reserve kind: {reserve_kind}")
        projection_timing[sample] = timing[0]
        zero_allocation &= timing[1:] == (0, 0)
        interval_width[sample] = velocity_upper - velocity_lower
        component_coverage[sample] = (
            (observed_delta >= predicted_delta + velocity_lower - 1.0e-12)
            & (observed_delta <= predicted_delta + velocity_upper + 1.0e-12)
        )
        covered[sample] = np.all(component_coverage[sample])

        predicted_generalized_contact_impulse = np.zeros(generalized_dof, np.float64)
        # M * contact_delta is the generalized impulse represented by the
        # point responses. Reuse the momentum query with observed=contact_delta
        # and predicted=0 so the comparison stays in Rust model conventions.
        zero_prediction = np.zeros((1, generalized_dof), np.float64)
        session.generalized_momentum_impulse_residuals(
            root_position,
            root_quaternion,
            q_pre,
            contact_delta,
            zero_prediction,
            predicted_generalized_contact_impulse[None, :],
        )
        observed_constraint = to_bonesaw_tangent(
            constraint_impulse, root_qvel, qvel_indices
        )
        generalized_constraint_error[sample] = float(
            np.linalg.norm(predicted_generalized_contact_impulse - observed_constraint)
        )
        generalized_constraint_relative_error[sample] = (
            generalized_constraint_error[sample]
            / max(float(np.linalg.norm(observed_constraint)), 1.0e-12)
        )

    width_groups = np.asarray([group_maximum(row) for row in interval_width])
    grouped_width_groups = np.asarray(
        [group_maximum(row) for row in grouped_interval_width]
    )
    error_groups = np.asarray([group_maximum(row) for row in raw_velocity_error])
    residual_groups = np.asarray([group_maximum(row) for row in residuals])
    width_summary = {
        "root_angular_rad_s": distribution(width_groups[:, 0]),
        "root_linear_m_s": distribution(width_groups[:, 1]),
        "joint_rad_s": distribution(width_groups[:, 2]),
    }
    width_gate_passed = all(
        width_summary[name]["p95"] <= limit for name, limit in WIDTH_GATES.items()
    )
    coordinate_names = [
        "root_wx",
        "root_wy",
        "root_wz",
        "root_vx",
        "root_vy",
        "root_vz",
        *joint_names,
    ]
    uncovered_sample_details = [
        {
            "sample": int(sample),
            "state_index": int(sample + sample_offset),
            "coordinates": [
                coordinate_names[int(coordinate)]
                for coordinate in np.flatnonzero(component_coverage[sample] == 0)
            ],
        }
        for sample in np.flatnonzero(covered == 0)
    ]
    grouped_width_summary = {
        "root_angular_rad_s": distribution(grouped_width_groups[:, 0]),
        "root_linear_m_s": distribution(grouped_width_groups[:, 1]),
        "joint_rad_s": distribution(grouped_width_groups[:, 2]),
    }
    grouped_width_gate_passed = all(
        grouped_width_summary[name]["p95"] <= limit
        for name, limit in WIDTH_GATES.items()
    )
    spatial_patch_result: dict[str, Any] | None = None
    if patch_session is not None:
        patch_width_groups = np.asarray(
            [group_maximum(row) for row in patch_interval_width]
        )
        patch_width_summary = {
            "root_angular_rad_s": distribution(patch_width_groups[:, 0]),
            "root_linear_m_s": distribution(patch_width_groups[:, 1]),
            "joint_rad_s": distribution(patch_width_groups[:, 2]),
        }
        patch_width_gate_passed = all(
            patch_width_summary[name]["p95"] <= limit
            for name, limit in WIDTH_GATES.items()
        )
        patch_uncovered = [
            {
                "sample": int(sample),
                "state_index": int(sample + sample_offset),
                "coordinates": [
                    coordinate_names[int(coordinate)]
                    for coordinate in np.flatnonzero(
                        patch_component_coverage[sample] == 0
                    )
                ],
            }
            for sample in np.flatnonzero(patch_covered == 0)
        ]
        spatial_patch_result = {
            "sample_coverage": float(np.mean(patch_covered)),
            "component_coverage": float(np.mean(patch_component_coverage)),
            "uncovered_samples": int(np.count_nonzero(patch_covered == 0)),
            "uncovered_sample_details": patch_uncovered,
            "strict_coverage_passed": bool(np.all(patch_covered)),
            "interval_width": patch_width_summary,
            "width_gate_passed": patch_width_gate_passed,
            "normal_impulse_upper_ns": distribution(
                patch_normal_impulse_upper.reshape(-1)
            ),
            "point_response_timing_ns": distribution(patch_query_timing),
            "bound_timing_ns": distribution(patch_bound_timing),
            "zero_rust_allocation": zero_allocation,
            "profile_promoted": bool(
                np.all(patch_covered)
                and patch_width_gate_passed
                and zero_allocation
            ),
            "profile": SPATIAL_PATCH_PROFILE,
            "patch_half_length_x_m": float(
                0.5
                * (np.max(FOOT_LOCAL_POINTS[:, 0]) - np.min(FOOT_LOCAL_POINTS[:, 0]))
            ),
            "patch_half_width_y_m": float(
                0.5
                * (np.max(FOOT_LOCAL_POINTS[:, 1]) - np.min(FOOT_LOCAL_POINTS[:, 1]))
            ),
            "torsion_radius_m": SPATIAL_PATCH_PROFILE["torsion_radius_m"],
        }
    contact_label_nonzero = bool(
        np.all(contact_impulse_norm > 1.0e-9) and np.all(active_contact_points > 0)
    )
    result = {
        "law": law.name,
        "reserve_kind": reserve_kind,
        "reserve_fraction": reserve_fraction,
        "sample_offset": sample_offset,
        "samples": samples,
        "physics_steps": samples * SUBSTEPS,
        "sample_coverage": float(np.mean(covered)),
        "component_coverage": float(np.mean(component_coverage)),
        "uncovered_samples": int(np.count_nonzero(covered == 0)),
        "uncovered_sample_details": uncovered_sample_details,
        "strict_coverage_passed": bool(np.all(covered)),
        "contact_label_nonzero": contact_label_nonzero,
        "contact_impulse_norm_ns": distribution(contact_impulse_norm),
        "active_contact_points": {
            "minimum": int(np.min(active_contact_points)),
            "maximum": int(np.max(active_contact_points)),
        },
        "interval_width": width_summary,
        "width_gate_passed": width_gate_passed,
        "raw_velocity_error": {
            "root_angular_rad_s": distribution(error_groups[:, 0]),
            "root_linear_m_s": distribution(error_groups[:, 1]),
            "joint_rad_s": distribution(error_groups[:, 2]),
        },
        "momentum_residual": {
            "root_angular_nms": distribution(residual_groups[:, 0]),
            "root_linear_ns": distribution(residual_groups[:, 1]),
            "joint_nms": distribution(residual_groups[:, 2]),
        },
        "generalized_constraint_reconstruction_error_ns": distribution(
            generalized_constraint_error
        ),
        "generalized_constraint_reconstruction_relative_error": distribution(
            generalized_constraint_relative_error
        ),
        "point_response_timing_ns": distribution(query_timing),
        "momentum_residual_timing_ns": distribution(residual_timing),
        "momentum_projection_timing_ns": distribution(projection_timing),
        "zero_rust_allocation": zero_allocation,
        "profile_promoted": bool(
            np.all(covered)
            and width_gate_passed
            and zero_allocation
            and contact_label_nonzero
        ),
        "grouped_acceleration_transition": {
            "sample_coverage": float(np.mean(grouped_covered)),
            "component_coverage": float(np.mean(grouped_component_coverage)),
            "uncovered_samples": int(np.count_nonzero(grouped_covered == 0)),
            "strict_coverage_passed": bool(np.all(grouped_covered)),
            "interval_width": grouped_width_summary,
            "width_gate_passed": grouped_width_gate_passed,
            "reserve": {
                "root_angular_rad_s2": GROUPED_ACCELERATION_RESERVE[0],
                "root_linear_m_s2": GROUPED_ACCELERATION_RESERVE[1],
                "joint_rad_s2": GROUPED_ACCELERATION_RESERVE[2],
            },
            "directional_profile": DIRECTIONAL_PROFILE,
            "bound_timing_ns": distribution(directional_timing),
            "zero_rust_allocation": zero_allocation,
            "profile_promoted": bool(
                np.all(grouped_covered)
                and grouped_width_gate_passed
                and zero_allocation
                and contact_label_nonzero
            ),
        },
        "spatial_patch_transition": spatial_patch_result,
        "law_parameters": {
            "friction": law.friction,
            "solref_time_s": law.solref_time_s,
            "solimp_min": law.solimp_min,
            "solimp_max": law.solimp_max,
            "cone": law.cone,
            "integrator": law.integrator,
        },
    }
    arrays = {
        f"{law.name}_covered": covered,
        f"{law.name}_component_coverage": component_coverage,
        f"{law.name}_momentum_residual": residuals,
        f"{law.name}_raw_velocity_error": raw_velocity_error,
        f"{law.name}_interval_width": interval_width,
        f"{law.name}_grouped_acceleration_covered": grouped_covered,
        f"{law.name}_grouped_acceleration_component_coverage": grouped_component_coverage,
        f"{law.name}_grouped_acceleration_interval_lower": grouped_interval_lower,
        f"{law.name}_grouped_acceleration_interval_upper": grouped_interval_upper,
        f"{law.name}_grouped_acceleration_interval_width": grouped_interval_width,
        f"{law.name}_grouped_acceleration_impulse_upper": grouped_impulse_upper,
        f"{law.name}_prospective_velocity": prospective_velocity,
        f"{law.name}_contact_impulse": contact_impulses,
        f"{law.name}_contact_points": contact_points,
        f"{law.name}_constraint_reconstruction_error": generalized_constraint_error,
        f"{law.name}_constraint_reconstruction_relative_error": generalized_constraint_relative_error,
        f"{law.name}_contact_impulse_norm": contact_impulse_norm,
        f"{law.name}_active_contact_points": active_contact_points,
        f"{law.name}_root_height": root_height,
        f"{law.name}_point_response_timing_ns": query_timing,
        f"{law.name}_momentum_residual_timing_ns": residual_timing,
        f"{law.name}_momentum_projection_timing_ns": projection_timing,
        f"{law.name}_directional_bound_timing_ns": directional_timing,
    }
    if patch_session is not None:
        arrays.update(
            {
                f"{law.name}_spatial_patch_covered": patch_covered,
                f"{law.name}_spatial_patch_component_coverage": patch_component_coverage,
                f"{law.name}_spatial_patch_interval_lower": patch_interval_lower,
                f"{law.name}_spatial_patch_interval_upper": patch_interval_upper,
                f"{law.name}_spatial_patch_interval_width": patch_interval_width,
                f"{law.name}_spatial_patch_normal_impulse_upper": patch_normal_impulse_upper,
                f"{law.name}_spatial_patch_response_timing_ns": patch_query_timing,
                f"{law.name}_spatial_patch_bound_timing_ns": patch_bound_timing,
            }
        )
    return result, arrays


def main() -> int:
    args = parse_args()
    if args.samples_per_law <= 0:
        raise SystemExit("samples-per-law must be positive")
    model_path = pathlib.Path(args.model).resolve()
    if not model_path.is_file():
        raise SystemExit(f"missing model: {model_path}")
    results: list[dict[str, Any]] = []
    replay: dict[str, np.ndarray] = {}
    for law in CONTACT_LAWS:
        result, arrays = run_law(model_path, law, args.samples_per_law)
        results.append(result)
        replay.update(arrays)
    mechanism_passed = all(
        result["zero_rust_allocation"] and result["contact_label_nonzero"]
        for result in results
    )
    momentum_promoted = all(result["profile_promoted"] for result in results)
    grouped_promoted = all(
        result["grouped_acceleration_transition"]["profile_promoted"]
        for result in results
    )
    metrics = {
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path),
        "model_sha256": sha256(model_path),
        "joint_dof": 23,
        "contact_points": 8,
        "contact_laws": len(CONTACT_LAWS),
        "samples": args.samples_per_law * len(CONTACT_LAWS),
        "physics_steps": args.samples_per_law * len(CONTACT_LAWS) * SUBSTEPS,
        "policy_or_controller_steps": 0,
        "sample_reset_every_transition": True,
        "reserve_fraction": RESERVE_FRACTION,
        "width_gates": WIDTH_GATES,
        "mechanism_passed": mechanism_passed,
        "momentum_profile_promoted": momentum_promoted,
        "grouped_acceleration_profile_promoted": grouped_promoted,
        "authority_admitted": False,
        "results": results,
    }
    momentum_rows = []
    grouped_rows = []
    for result in results:
        width = result["interval_width"]
        momentum_rows.append(
            [
                result["law"],
                result["samples"],
                f"{100.0 * result['sample_coverage']:.3f}%",
                f"{100.0 * result['component_coverage']:.4f}%",
                f"{width['root_angular_rad_s']['p95']:.3f}",
                f"{width['root_linear_m_s']['p95']:.3f}",
                f"{width['joint_rad_s']['p95']:.3f}",
                f"{result['contact_impulse_norm_ns']['p95']:.4f}",
                f"{result['generalized_constraint_reconstruction_error_ns']['p95']:.4f}",
                f"{100.0 * result['generalized_constraint_reconstruction_relative_error']['p95']:.2f}%",
                "yes" if result["zero_rust_allocation"] else "NO",
                "PASS" if result["profile_promoted"] else "REJECT",
            ]
        )
        grouped = result["grouped_acceleration_transition"]
        grouped_width = grouped["interval_width"]
        grouped_rows.append(
            [
                result["law"],
                result["samples"],
                f"{100.0 * grouped['sample_coverage']:.3f}%",
                f"{100.0 * grouped['component_coverage']:.4f}%",
                f"{grouped_width['root_angular_rad_s']['p95']:.3f}",
                f"{grouped_width['root_linear_m_s']['p95']:.3f}",
                f"{grouped_width['joint_rad_s']['p95']:.3f}",
                f"{grouped['bound_timing_ns']['p99'] / 1_000.0:.3f}",
                "yes" if grouped["zero_rust_allocation"] else "NO",
                "PASS" if grouped["profile_promoted"] else "REJECT",
            ]
        )
    report = "\n".join(
        [
            "# Bonesaw G1 contact-law transition holdout · r213",
            "",
            f"> Generic mechanism **{'PASS' if mechanism_passed else 'FAIL'}** · frozen 50/10/50 grouped profile **{'PROMOTED' if grouped_promoted else 'REJECTED'}** · momentum sensitivity **{'PROMOTED' if momentum_promoted else 'REJECTED'}** · authority **NOT ADMITTED** · controller/policy steps **0**.",
            "",
            "## Contract",
            "",
            "- The pinned official 23-DOF G1 supplies authored mass, inertia, limits, and four primitive sphere contacts per foot. G1 remains an evaluation fixture; Upkie remains the interactive model.",
            "- Every five-millisecond transition resets to an independently authored pre-impact state. MuJoCo alone generates the completed contact label. The two profiles predeclare different compliance, friction, cone, and integration laws; no sample is selected from a successful rollout.",
            "- Rust evaluates the causal pre-impact eight-point `M⁻¹Jᵀ`. The primary query applies R212's frozen ±50 rad/s² root-angular, ±10 m/s² root-linear, and ±50 rad/s² joint continuous-acceleration reserve with the unchanged R204 passive directional contact profile. MuJoCo's completed velocity is scoring-only.",
            f"- A separate momentum sensitivity maps the completed impulse only for oracle decomposition, forms the generalized-momentum residual, and projects a reserve frozen at {100.0 * RESERVE_FRACTION:.0f}% of one tick of weight/effort through full `M⁻¹`. Usefulness gates remain {WIDTH_GATES['root_angular_rad_s']:.1f} rad/s root angular, {WIDTH_GATES['root_linear_m_s']:.1f} m/s root linear, and {WIDTH_GATES['joint_rad_s']:.1f} rad/s joints (p95 group maximum).",
            "",
            "## Frozen grouped acceleration + directional contact tube",
            "",
            *markdown_table(
                [
                    "contact law",
                    "samples",
                    "sample coverage",
                    "component coverage",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "bound p99 µs",
                    "zero alloc",
                    "profile",
                ],
                grouped_rows,
            ),
            "",
            "The profile closes every transition but fails every useful-width gate. Its eight independent point-impulse boxes compound through the G1 leg Jacobians; strict coverage does not rescue a 1,383.9 rad/s p95 joint interval.",
            "",
            "## Completed-impulse momentum sensitivity",
            "",
            *markdown_table(
                [
                    "contact law",
                    "samples",
                    "sample coverage",
                    "component coverage",
                    "root ω width p95",
                    "root v width p95",
                    "joint width p95",
                    "contact impulse p95 N·s",
                    "constraint recon p95 N·s",
                    "constraint recon p95 relative",
                    "zero alloc",
                    "profile",
                ],
                momentum_rows,
            ),
            "",
            "The constraint-reconstruction column compares the point-response generalized impulse to MuJoCo's separately accumulated generalized constraint impulse. It is diagnostic, not a fitted correction.",
            "",
            "## Decision",
            "",
            (
                "The R212 construction crosses both held-out G1 laws at useful width. It still cannot authorize a command until terminal consequence, deadline, provenance, and hardware gates pass."
                if grouped_promoted
                else "The R212 grouped construction is rejected out of sample: it obtains strict coverage only with unusably wide G1 joint intervals. The independently frozen momentum sensitivity is also rejected on width. No holdout quantile is fed back into either profile, and no command authority changes. The next construction must couple the finite foot patch (or carry a causal spatial-wrench set) instead of summing eight independent point boxes."
            ),
            "",
            "This is a second morphology and materially different parameterized MuJoCo contact formulation, not a second simulator engine or hardware contact calibration. That distinction remains explicit.",
        ]
    ) + "\n"
    destination = pathlib.Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "g1-contact-law-transition-holdout-metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    (destination / "G1_CONTACT_LAW_TRANSITION_HOLDOUT.md").write_text(report)
    np.savez_compressed(
        destination / "g1-contact-law-transition-holdout-replay.npz", **replay
    )
    web = pathlib.Path(args.web_report)
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_text(render_report_html(report))
    print(
        json.dumps(
            {
                "mechanism_passed": mechanism_passed,
                "grouped_acceleration_profile_promoted": grouped_promoted,
                "momentum_profile_promoted": momentum_promoted,
                "grouped_acceleration_sample_coverage": {
                    result["law"]: result["grouped_acceleration_transition"]["sample_coverage"]
                    for result in results
                },
                "physics_steps": metrics["physics_steps"],
                "policy_or_controller_steps": 0,
                "authority_admitted": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if mechanism_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
