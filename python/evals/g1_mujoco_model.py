"""Primitive MuJoCo model adapter for the Unitree G1 23-DoF reference.

The checked-in G1 asset is a URDF that references meshes which are intentionally
not part of this repository.  The CPU evaluations still need a real floating
MuJoCo plant, so this module performs a small, deterministic URDF-to-MJCF
translation using primitive geoms only.  The adapter keeps the URDF topology,
link inertials, revolute axes/ranges/effort limits, and fixed-link transforms;
it does not pretend that the primitive collision envelope is the manufacturer's
mesh model.

This is an evaluation fixture, not a runtime controller path.  Importing the
module does not require MuJoCo; :func:`load_g1_model` imports it lazily.
"""

from __future__ import annotations

import math
import pathlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable


DEFAULT_URDF = pathlib.Path("benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf")
ROOT_HEIGHT_M = 0.80
GROUND_GEOM_NAME = "ground"
FOOT_SPHERE_NAMES = (
    "left_foot_sphere_front",
    "left_foot_sphere_rear",
    "right_foot_sphere_front",
    "right_foot_sphere_rear",
)


@dataclass(frozen=True)
class _Link:
    name: str
    inertial: ET.Element | None
    material: str


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None


def default_urdf_path() -> pathlib.Path:
    """Return the repository-relative G1 URDF path as an absolute path."""

    return (pathlib.Path(__file__).resolve().parents[2] / DEFAULT_URDF).resolve()


def _float_attr(element: ET.Element | None, name: str, default: float) -> float:
    if element is None:
        return default
    value = element.get(name)
    if value is None:
        return default
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite URDF attribute {name!r}: {value!r}")
    return parsed


def _vector(value: str | None, *, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if value is None:
        return default
    parts = tuple(float(part) for part in value.split())
    if len(parts) != 3 or not all(math.isfinite(part) for part in parts):
        raise ValueError(f"expected a finite 3-vector, got {value!r}")
    return parts  # type: ignore[return-value]


def _parse_urdf(path: pathlib.Path) -> tuple[dict[str, _Link], tuple[_Joint, ...]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    root = ET.parse(path).getroot()
    links: dict[str, _Link] = {}
    for link in root.findall("link"):
        name = link.get("name")
        if not name:
            raise ValueError("G1 URDF contains a link without a name")
        visual = link.find("visual")
        material = "white"
        if visual is not None:
            material_element = visual.find("material")
            material = (material_element.get("name") if material_element is not None else None) or material
        links[name] = _Link(name, link.find("inertial"), material)

    joints: list[_Joint] = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        kind = joint.get("type")
        parent = joint.find("parent")
        child = joint.find("child")
        if not name or not kind or parent is None or child is None:
            raise ValueError("G1 URDF contains an incomplete joint")
        parent_name = parent.get("link")
        child_name = child.get("link")
        if not parent_name or not child_name:
            raise ValueError(f"joint {name!r} has no parent/child link")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        lower = upper = effort = velocity = None
        if limit is not None:
            lower = _float_attr(limit, "lower", 0.0) if limit.get("lower") is not None else None
            upper = _float_attr(limit, "upper", 0.0) if limit.get("upper") is not None else None
            effort = _float_attr(limit, "effort", 0.0) if limit.get("effort") is not None else None
            velocity = _float_attr(limit, "velocity", 0.0) if limit.get("velocity") is not None else None
        joints.append(
            _Joint(
                name=name,
                kind=kind,
                parent=parent_name,
                child=child_name,
                origin_xyz=_vector(origin.get("xyz") if origin is not None else None, default=(0.0, 0.0, 0.0)),
                origin_rpy=_vector(origin.get("rpy") if origin is not None else None, default=(0.0, 0.0, 0.0)),
                axis=_vector(axis.get("xyz") if axis is not None else None, default=(0.0, 0.0, 1.0)),
                lower=lower,
                upper=upper,
                effort=effort,
                velocity=velocity,
            )
        )
    if len(links) != 30 or sum(j.kind == "revolute" for j in joints) != 23:
        raise ValueError(
            f"unexpected G1 topology: {len(links)} links, "
            f"{sum(j.kind == 'revolute' for j in joints)} revolute joints"
        )
    return links, tuple(joints)


def _fmt(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _xml_attrs(**attrs: object) -> str:
    # XML names and values are all generated from fixed/validated URDF fields;
    # escaping names still keeps this helper safe for alternate local URDFs.
    from xml.sax.saxutils import quoteattr

    return " ".join(f"{key}={quoteattr(str(value))}" for key, value in attrs.items())


def _primitive_spec(link_name: str) -> tuple[str, tuple[float, ...]]:
    """Return ``(geom_type, dimensions)`` for a conservative fallback shell."""

    lowered = link_name.lower()
    if lowered == "pelvis":
        return "box", (0.14, 0.12, 0.10)
    if "torso" in lowered:
        return "box", (0.18, 0.14, 0.27)
    if "head" in lowered:
        return "box", (0.10, 0.10, 0.12)
    if "hip" in lowered or "knee" in lowered:
        return "capsule", (0.055, -0.115, 0.0, 0.035)
    if "ankle_pitch" in lowered:
        return "capsule", (0.040, -0.055, 0.0, 0.040)
    if "ankle_roll" in lowered:
        return "box", (0.085, 0.060, 0.045)
    if "shoulder" in lowered or "elbow" in lowered:
        return "capsule", (0.040, -0.060, 0.0, 0.035)
    if "wrist" in lowered or "hand" in lowered:
        return "box", (0.070, 0.045, 0.035)
    if "logo" in lowered:
        return "box", (0.015, 0.015, 0.008)
    # Sensor/fixed links without a physical inertial are not given a geom.
    return "box", (0.025, 0.025, 0.025)


def _inertial_xml(link: _Link) -> str:
    if link.inertial is None:
        return ""
    origin = link.inertial.find("origin")
    xyz = _vector(origin.get("xyz") if origin is not None else None, default=(0.0, 0.0, 0.0))
    mass = _float_attr(link.inertial.find("mass"), "value", 0.0)
    inertia = link.inertial.find("inertia")
    if mass <= 0.0 or inertia is None:
        return ""
    ixx = _float_attr(inertia, "ixx", 0.0)
    iyy = _float_attr(inertia, "iyy", 0.0)
    izz = _float_attr(inertia, "izz", 0.0)
    ixy = _float_attr(inertia, "ixy", 0.0)
    ixz = _float_attr(inertia, "ixz", 0.0)
    iyz = _float_attr(inertia, "iyz", 0.0)
    attrs = _xml_attrs(
        pos=_fmt(xyz),
        mass=f"{mass:.12g}",
        fullinertia=_fmt((ixx, iyy, izz, ixy, ixz, iyz)),
    )
    return f"<inertial {attrs} />"


def _geom_xml(link: _Link) -> str:
    if link.inertial is None:
        return ""
    geom_type, dimensions = _primitive_spec(link.name)
    color = "0.20 0.22 0.25 1" if link.material == "dark" else "0.72 0.74 0.78 1"
    common = _xml_attrs(
        name=f"{link.name}_collision_{geom_type}",
        rgba=color,
        # Primitive shells collide with the ground (mask 1) but are kept out
        # of one another and out of the explicit foot probes.  This avoids a
        # dense self-contact graph while retaining useful fallback geometry.
        contype="2",
        conaffinity="1",
        group="1",
    )
    if geom_type == "box":
        return f"<geom {common} type=\"box\" size=\"{_fmt(dimensions)}\" />"
    radius, z0, x1, z1 = dimensions
    return (
        f"<geom {common} type=\"capsule\" size=\"{radius:.12g}\" "
        f"fromto=\"0 0 {z0:.12g} {x1:.12g} 0 {z1:.12g}\" />"
    )


def _foot_spheres_xml(link_name: str) -> str:
    if link_name == "left_ankle_roll_link":
        side = "left"
    elif link_name == "right_ankle_roll_link":
        side = "right"
    else:
        return ""
    y = 0.0
    front_attrs = _xml_attrs(
        name=f"{side}_foot_sphere_front",
        type="sphere",
        pos=_fmt((-0.05, y, -0.03)),
        size="0.045",
        rgba="0.92 0.42 0.16 1",
        contype="4",
        conaffinity="1",
        group="2",
    )
    rear_attrs = _xml_attrs(
        name=f"{side}_foot_sphere_rear",
        type="sphere",
        pos=_fmt((0.12, y, -0.03)),
        size="0.045",
        rgba="0.92 0.42 0.16 1",
        contype="4",
        conaffinity="1",
        group="2",
    )
    return f"<geom {front_attrs} />\n<geom {rear_attrs} />"


def _body_xml(
    link_name: str,
    links: dict[str, _Link],
    children: dict[str, tuple[_Joint, ...]],
    by_child: dict[str, _Joint],
    *,
    root: bool = False,
) -> str:
    link = links[link_name]
    joint = by_child.get(link_name)
    attrs: dict[str, object] = {"name": link_name}
    if root:
        attrs["pos"] = _fmt((0.0, 0.0, ROOT_HEIGHT_M))
    elif joint is not None:
        attrs["pos"] = _fmt(joint.origin_xyz)
        attrs["euler"] = _fmt(joint.origin_rpy)
    pieces = [f"<body {_xml_attrs(**attrs)}>"]
    if root:
        pieces.append('<freejoint name="root" />')
    if joint is not None and joint.kind == "revolute":
        if joint.lower is None or joint.upper is None:
            raise ValueError(f"revolute joint {joint.name!r} has no lower/upper limit")
        jattrs: dict[str, object] = {
            "name": joint.name,
            "type": "hinge",
            "axis": _fmt(joint.axis),
            "range": _fmt((joint.lower, joint.upper)),
            "limited": "true",
            "damping": "0.5",
            "armature": "0.01",
        }
        # MuJoCo has no native per-joint velocity limit.  Retain the URDF
        # value in the first joint-user slot so evaluators can enforce the
        # source limit without losing it during the primitive conversion.
        if joint.velocity is not None and joint.velocity > 0.0:
            jattrs["user"] = f"{joint.velocity:.12g}"
        pieces.append(f"<joint {_xml_attrs(**jattrs)} />")
    inertial = _inertial_xml(link)
    if inertial:
        pieces.append(inertial)
    geom = _geom_xml(link)
    if geom and link_name not in ("left_ankle_roll_link", "right_ankle_roll_link"):
        pieces.append(geom)
    foot_spheres = _foot_spheres_xml(link_name)
    if foot_spheres:
        pieces.append(foot_spheres)
    for child_joint in children.get(link_name, ()):
        pieces.append(_body_xml(child_joint.child, links, children, by_child))
    pieces.append("</body>")
    return "\n".join(pieces)


def build_g1_mjcf(urdf_path: str | pathlib.Path | None = None) -> str:
    """Build a self-contained primitive MJCF string from the pinned G1 URDF.

    No mesh or package URI is emitted.  The generated model has one free root,
    23 bounded hinge joints, one effort-limited motor per hinge, a ground plane,
    primitive per-link collision shells, and exactly four foot spheres (two per
    foot) used as the robust contact probes for the first CPU plant gate.  The
    source URDF velocity limits are retained in ``model.jnt_user[:, 0]`` because
    MuJoCo does not expose a native per-joint velocity-limit attribute.
    """

    path = default_urdf_path() if urdf_path is None else pathlib.Path(urdf_path).resolve()
    links, joints = _parse_urdf(path)
    child_names = {joint.child for joint in joints}
    roots = [name for name in links if name not in child_names]
    if roots != ["pelvis"]:
        raise ValueError(f"expected pelvis as the sole root link, got {roots!r}")
    children: dict[str, list[_Joint]] = {}
    by_child: dict[str, _Joint] = {}
    for joint in joints:
        children.setdefault(joint.parent, []).append(joint)
        by_child[joint.child] = joint
    children_tuple = {parent: tuple(value) for parent, value in children.items()}
    body = _body_xml("pelvis", links, children_tuple, by_child, root=True)

    motors: list[str] = []
    for joint in joints:
        if joint.kind != "revolute":
            continue
        if joint.effort is None or joint.effort <= 0.0:
            raise ValueError(f"revolute joint {joint.name!r} has no positive effort limit")
        attrs = _xml_attrs(
            name=f"{joint.name}_motor",
            joint=joint.name,
            ctrlrange=_fmt((-joint.effort, joint.effort)),
            ctrllimited="true",
            gear="1",
        )
        motors.append(f"<motor {attrs} />")

    return (
        '<mujoco model="g1_23dof_primitive">\n'
        '  <compiler angle="radian" autolimits="false" />\n'
        '  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast" '
        'cone="elliptic" iterations="80" />\n'
        '  <size njmax="2000" nconmax="256" nuser_jnt="1" />\n'
        '  <default>\n'
        '    <joint limited="true" damping="0.5" armature="0.01" />\n'
        '    <geom density="0" friction="1 0.01 0.001" solref="0.008 1" '
        'solimp="0.95 0.99 0.001 0.5 2" />\n'
        '  </default>\n'
        '  <worldbody>\n'
        '    <geom name="ground" type="plane" size="0 0 0.05" rgba="0.18 0.20 0.23 1" '
        'friction="1 0.01 0.001" contype="1" conaffinity="7" />\n'
        f"{body}\n"
        '  </worldbody>\n'
        '  <actuator>\n'
        + "\n".join(motors)
        + '\n  </actuator>\n'
        '</mujoco>\n'
    )


def load_g1_model(
    urdf_path: str | pathlib.Path | None = None,
):
    """Return ``(mujoco.MjModel, mujoco.MjData)`` for the primitive fixture."""

    import mujoco

    model = mujoco.MjModel.from_xml_string(build_g1_mjcf(urdf_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


__all__ = [
    "DEFAULT_URDF",
    "FOOT_SPHERE_NAMES",
    "GROUND_GEOM_NAME",
    "ROOT_HEIGHT_M",
    "build_g1_mjcf",
    "default_urdf_path",
    "load_g1_model",
]
