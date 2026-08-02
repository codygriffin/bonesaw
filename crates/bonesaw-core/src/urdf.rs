use std::{collections::BTreeMap, fs, path::Path};

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion};
use serde::Deserialize;
use thiserror::Error;

use crate::{
    math::{Transform3, Vec3, parse_vec3},
    model::{
        BodyId, CollisionShape, CompiledModel, FrameId, JointId, JointKind, JointLimit, JointSpec,
        ModelError, RigidBodySpec, VisualShape,
    },
};

const DEFAULT_VISUAL_RGBA: [f64; 4] = [0.7, 0.7, 0.7, 1.0];

#[derive(Debug, Error)]
pub enum UrdfError {
    #[error("failed to read URDF: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid URDF XML: {0}")]
    Xml(#[from] quick_xml::DeError),
    #[error("invalid URDF: {0}")]
    Invalid(String),
    #[error(transparent)]
    Model(#[from] ModelError),
}

pub fn load_urdf_file(path: impl AsRef<Path>) -> Result<CompiledModel, UrdfError> {
    load_urdf(&fs::read_to_string(path)?)
}

pub fn load_urdf(source: &str) -> Result<CompiledModel, UrdfError> {
    let robot: RobotXml = quick_xml::de::from_str(source)?;
    let mut robot_links = Vec::new();
    let mut robot_joints = Vec::new();
    let mut robot_materials = BTreeMap::new();
    for child in robot.children {
        match child {
            RobotChildXml::Link(link) => robot_links.push(link),
            RobotChildXml::Joint(joint) => robot_joints.push(joint),
            RobotChildXml::Material(material) => {
                if let Some(color) = material.color {
                    robot_materials.insert(material.name, parse_rgba(&color.rgba)?);
                }
            }
            RobotChildXml::Other => {}
        }
    }
    if robot_links.is_empty() {
        return Err(UrdfError::Invalid("robot contains no links".into()));
    }

    let links: BTreeMap<_, _> = robot_links
        .into_iter()
        .map(|link| (link.name.clone(), link))
        .collect();
    let mut child_joints = BTreeMap::<String, JointXml>::new();
    let mut outgoing = BTreeMap::<String, Vec<String>>::new();
    for joint in robot_joints {
        if !links.contains_key(&joint.parent.link) {
            return Err(UrdfError::Invalid(format!(
                "joint {} has missing parent {}",
                joint.name, joint.parent.link
            )));
        }
        if !links.contains_key(&joint.child.link) {
            return Err(UrdfError::Invalid(format!(
                "joint {} has missing child {}",
                joint.name, joint.child.link
            )));
        }
        if child_joints
            .insert(joint.child.link.clone(), joint)
            .is_some()
        {
            return Err(UrdfError::Invalid(
                "a link has more than one parent joint".into(),
            ));
        }
    }
    for (child, joint) in &child_joints {
        outgoing
            .entry(joint.parent.link.clone())
            .or_default()
            .push(child.clone());
    }
    for children in outgoing.values_mut() {
        children.sort();
    }
    let roots: Vec<_> = links
        .keys()
        .filter(|name| !child_joints.contains_key(*name))
        .cloned()
        .collect();
    if roots.len() != 1 {
        return Err(UrdfError::Invalid(format!(
            "expected exactly one root link, found {}",
            roots.len()
        )));
    }

    let mut order = Vec::with_capacity(links.len());
    let mut stack = vec![roots[0].clone()];
    while let Some(name) = stack.pop() {
        order.push(name.clone());
        if let Some(children) = outgoing.get(&name) {
            // Push backwards to retain lexical traversal order.
            for child in children.iter().rev() {
                stack.push(child.clone());
            }
        }
    }
    if order.len() != links.len() {
        return Err(UrdfError::Invalid(
            "joint topology contains a cycle or disconnected component".into(),
        ));
    }

    let ids: BTreeMap<_, _> = order
        .iter()
        .enumerate()
        .map(|(index, name)| (name.clone(), BodyId(index)))
        .collect();
    let mut bodies = Vec::with_capacity(order.len());
    for (index, name) in order.iter().enumerate() {
        let link = &links[name];
        let (mass, com, inertia) = parse_inertial(link)?;
        let collisions = link
            .collisions
            .iter()
            .map(parse_collision)
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .collect();
        let visuals = link
            .visuals
            .iter()
            .map(|visual| parse_visual(visual, &robot_materials))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .collect();
        bodies.push(RigidBodySpec {
            id: BodyId(index),
            name: name.clone(),
            parent_joint: None,
            body_frame: FrameId(index),
            mass,
            com_in_body: com,
            inertia_about_com_in_body: inertia,
            collisions,
            visuals,
        });
    }

    let mut joints = Vec::with_capacity(child_joints.len());
    for child in order.iter().skip(1) {
        let joint = &child_joints[child];
        let kind = match joint.kind.as_str() {
            "fixed" => JointKind::Fixed,
            "revolute" => JointKind::Revolute,
            "continuous" => JointKind::Continuous,
            "prismatic" => JointKind::Prismatic,
            unsupported => {
                return Err(UrdfError::Invalid(format!(
                    "joint {} uses unsupported type {}",
                    joint.name, unsupported
                )));
            }
        };
        let axis = parse_vec3(joint.axis.as_ref().map_or("1 0 0", |axis| &axis.xyz))
            .ok_or_else(|| UrdfError::Invalid(format!("joint {} has invalid axis", joint.name)))?;
        if kind != JointKind::Fixed && axis.norm_squared() < 1e-16 {
            return Err(UrdfError::Invalid(format!(
                "joint {} has a zero axis",
                joint.name
            )));
        }
        let limit = parse_limit(kind, joint.limit.as_ref(), &joint.name)?;
        joints.push(JointSpec {
            id: JointId(joints.len()),
            name: joint.name.clone(),
            kind,
            parent: ids[&joint.parent.link],
            child: ids[&joint.child.link],
            parent_from_joint: parse_origin(joint.origin.as_ref())?,
            axis_in_joint: if kind == JointKind::Fixed {
                Vec3::x()
            } else {
                axis.normalize()
            },
            coordinate: None,
            limit,
        });
    }

    CompiledModel::new(robot.name, bodies, joints).map_err(Into::into)
}

fn parse_limit(
    kind: JointKind,
    source: Option<&LimitXml>,
    name: &str,
) -> Result<JointLimit, UrdfError> {
    if kind == JointKind::Fixed {
        return Ok(JointLimit::unbounded());
    }
    if kind == JointKind::Continuous {
        return Ok(JointLimit {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: source
                .and_then(|value| value.velocity)
                .unwrap_or(f64::INFINITY),
            effort: source
                .and_then(|value| value.effort)
                .unwrap_or(f64::INFINITY),
        });
    }
    let source =
        source.ok_or_else(|| UrdfError::Invalid(format!("joint {name} is missing limits")))?;
    if source.lower.is_none() && source.upper.is_none() {
        // A number of established robot descriptions encode wheel joints as
        // `revolute` with velocity/effort but no position bounds. Canonicalize
        // that representation to an unbounded coordinate.
        return Ok(JointLimit {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: source.velocity.unwrap_or(f64::INFINITY),
            effort: source.effort.unwrap_or(f64::INFINITY),
        });
    }
    let lower = source
        .lower
        .ok_or_else(|| UrdfError::Invalid(format!("joint {name} has no lower limit")))?;
    let upper = source
        .upper
        .ok_or_else(|| UrdfError::Invalid(format!("joint {name} has no upper limit")))?;
    if !lower.is_finite() || !upper.is_finite() || lower > upper {
        return Err(UrdfError::Invalid(format!(
            "joint {name} has invalid position limits"
        )));
    }
    Ok(JointLimit {
        lower,
        upper,
        velocity: source.velocity.unwrap_or(f64::INFINITY),
        effort: source.effort.unwrap_or(f64::INFINITY),
    })
}

fn parse_inertial(link: &LinkXml) -> Result<(f64, Vec3, Matrix3<f64>), UrdfError> {
    let Some(inertial) = &link.inertial else {
        // Authored links without inertia are retained as massless frames.
        return Ok((0.0, Vec3::zeros(), Matrix3::zeros()));
    };
    let pose = parse_origin(inertial.origin.as_ref())?;
    let values = &inertial.inertia;
    let matrix = Matrix3::new(
        values.ixx, values.ixy, values.ixz, values.ixy, values.iyy, values.iyz, values.ixz,
        values.iyz, values.izz,
    );
    let rotation = pose.rotation.to_rotation_matrix();
    let inertia_in_body = rotation.matrix() * matrix * rotation.matrix().transpose();
    Ok((
        inertial.mass.value,
        pose.translation.vector,
        inertia_in_body,
    ))
}

fn parse_collision(collision: &CollisionXml) -> Result<Option<CollisionShape>, UrdfError> {
    let pose = parse_origin(collision.origin.as_ref())?;
    parse_geometry(&collision.geometry, pose, "collision")
}

fn parse_visual(
    visual: &VisualXml,
    materials: &BTreeMap<String, [f64; 4]>,
) -> Result<Option<VisualShape>, UrdfError> {
    let pose = parse_origin(visual.origin.as_ref())?;
    let geometry = parse_geometry(&visual.geometry, pose, "visual")?;
    let rgba = match visual.material.as_ref() {
        Some(material) if material.color.is_some() => {
            parse_rgba(&material.color.as_ref().expect("checked above").rgba)?
        }
        Some(material) => materials
            .get(&material.name)
            .copied()
            .unwrap_or(DEFAULT_VISUAL_RGBA),
        None => DEFAULT_VISUAL_RGBA,
    };
    Ok(geometry.map(|geometry| VisualShape { geometry, rgba }))
}

fn parse_geometry(
    geometry: &GeometryXml,
    pose: Transform3,
    context: &str,
) -> Result<Option<CollisionShape>, UrdfError> {
    if let Some(sphere) = &geometry.sphere {
        return Ok(Some(CollisionShape::Sphere {
            radius: sphere.radius,
            body_from_shape: pose,
        }));
    }
    if let Some(cylinder) = &geometry.cylinder {
        return Ok(Some(CollisionShape::Cylinder {
            radius: cylinder.radius,
            half_length: 0.5 * cylinder.length,
            body_from_shape: pose,
        }));
    }
    if let Some(box_shape) = &geometry.box_shape {
        let size = parse_vec3(&box_shape.size)
            .ok_or_else(|| UrdfError::Invalid(format!("{context} box has invalid size")))?;
        return Ok(Some(CollisionShape::Box {
            half_extents: size * 0.5,
            body_from_shape: pose,
        }));
    }
    if let Some(mesh) = &geometry.mesh {
        let scale = mesh
            .scale
            .as_deref()
            .map(parse_vec3)
            .transpose_option()
            .ok_or_else(|| UrdfError::Invalid(format!("{context} mesh has invalid scale")))?
            .unwrap_or_else(|| Vec3::repeat(1.0));
        return Ok(Some(CollisionShape::Mesh {
            filename: mesh.filename.clone(),
            scale,
            body_from_shape: pose,
        }));
    }
    Ok(None)
}

fn parse_rgba(source: &str) -> Result<[f64; 4], UrdfError> {
    let values = source
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| UrdfError::Invalid("material has invalid rgba".into()))?;
    let rgba: [f64; 4] = values
        .try_into()
        .map_err(|_| UrdfError::Invalid("material rgba must have four values".into()))?;
    if rgba.iter().any(|value| !value.is_finite()) {
        return Err(UrdfError::Invalid("material rgba must be finite".into()));
    }
    Ok(rgba)
}

fn parse_origin(origin: Option<&OriginXml>) -> Result<Transform3, UrdfError> {
    let Some(origin) = origin else {
        return Ok(Transform3::identity());
    };
    let xyz = origin
        .xyz
        .as_deref()
        .map(parse_vec3)
        .transpose_option()
        .ok_or_else(|| UrdfError::Invalid("origin has invalid xyz".into()))?
        .unwrap_or_else(Vec3::zeros);
    let rpy = origin
        .rpy
        .as_deref()
        .map(parse_vec3)
        .transpose_option()
        .ok_or_else(|| UrdfError::Invalid("origin has invalid rpy".into()))?
        .unwrap_or_else(Vec3::zeros);
    Ok(Isometry3::from_parts(
        Translation3::from(xyz),
        UnitQuaternion::from_euler_angles(rpy.x, rpy.y, rpy.z),
    ))
}

trait TransposeOption<T> {
    fn transpose_option(self) -> Option<Option<T>>;
}

impl<T> TransposeOption<T> for Option<Option<T>> {
    fn transpose_option(self) -> Option<Option<T>> {
        match self {
            Some(Some(value)) => Some(Some(value)),
            Some(None) => None,
            None => Some(None),
        }
    }
}

#[derive(Debug, Deserialize)]
struct RobotXml {
    #[serde(rename = "@name")]
    name: String,
    #[serde(rename = "$value", default)]
    children: Vec<RobotChildXml>,
}

#[derive(Debug, Deserialize)]
enum RobotChildXml {
    #[serde(rename = "link")]
    Link(LinkXml),
    #[serde(rename = "joint")]
    Joint(JointXml),
    #[serde(rename = "material")]
    Material(MaterialXml),
    #[serde(other)]
    Other,
}

#[derive(Debug, Deserialize)]
struct LinkXml {
    #[serde(rename = "@name")]
    name: String,
    inertial: Option<InertialXml>,
    #[serde(rename = "collision", default)]
    collisions: Vec<CollisionXml>,
    #[serde(rename = "visual", default)]
    visuals: Vec<VisualXml>,
}

#[derive(Debug, Deserialize)]
struct MaterialXml {
    #[serde(rename = "@name", default)]
    name: String,
    color: Option<ColorXml>,
}

#[derive(Debug, Deserialize)]
struct ColorXml {
    #[serde(rename = "@rgba")]
    rgba: String,
}

#[derive(Debug, Deserialize)]
struct InertialXml {
    origin: Option<OriginXml>,
    mass: MassXml,
    inertia: InertiaXml,
}

#[derive(Debug, Deserialize)]
struct MassXml {
    #[serde(rename = "@value")]
    value: f64,
}

#[derive(Debug, Deserialize)]
struct InertiaXml {
    #[serde(rename = "@ixx")]
    ixx: f64,
    #[serde(rename = "@ixy")]
    ixy: f64,
    #[serde(rename = "@ixz")]
    ixz: f64,
    #[serde(rename = "@iyy")]
    iyy: f64,
    #[serde(rename = "@iyz")]
    iyz: f64,
    #[serde(rename = "@izz")]
    izz: f64,
}

#[derive(Debug, Deserialize)]
struct JointXml {
    #[serde(rename = "@name")]
    name: String,
    #[serde(rename = "@type")]
    kind: String,
    origin: Option<OriginXml>,
    parent: LinkNameXml,
    child: LinkNameXml,
    axis: Option<AxisXml>,
    limit: Option<LimitXml>,
}

#[derive(Debug, Deserialize)]
struct LinkNameXml {
    #[serde(rename = "@link")]
    link: String,
}

#[derive(Debug, Deserialize)]
struct AxisXml {
    #[serde(rename = "@xyz")]
    xyz: String,
}

#[derive(Debug, Deserialize)]
struct LimitXml {
    #[serde(rename = "@lower")]
    lower: Option<f64>,
    #[serde(rename = "@upper")]
    upper: Option<f64>,
    #[serde(rename = "@velocity")]
    velocity: Option<f64>,
    #[serde(rename = "@effort")]
    effort: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct OriginXml {
    #[serde(rename = "@xyz")]
    xyz: Option<String>,
    #[serde(rename = "@rpy")]
    rpy: Option<String>,
}

#[derive(Debug, Deserialize)]
struct CollisionXml {
    origin: Option<OriginXml>,
    geometry: GeometryXml,
}

#[derive(Debug, Deserialize)]
struct VisualXml {
    origin: Option<OriginXml>,
    geometry: GeometryXml,
    material: Option<MaterialXml>,
}

#[derive(Debug, Deserialize)]
struct GeometryXml {
    sphere: Option<SphereXml>,
    cylinder: Option<CylinderXml>,
    #[serde(rename = "box")]
    box_shape: Option<BoxXml>,
    mesh: Option<MeshXml>,
}

#[derive(Debug, Deserialize)]
struct SphereXml {
    #[serde(rename = "@radius")]
    radius: f64,
}

#[derive(Debug, Deserialize)]
struct CylinderXml {
    #[serde(rename = "@radius")]
    radius: f64,
    #[serde(rename = "@length")]
    length: f64,
}

#[derive(Debug, Deserialize)]
struct BoxXml {
    #[serde(rename = "@size")]
    size: String,
}

#[derive(Debug, Deserialize)]
struct MeshXml {
    #[serde(rename = "@filename")]
    filename: String,
    #[serde(rename = "@scale")]
    scale: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    const ARM: &str = r#"
    <robot name="arm">
      <link name="base">
        <inertial>
          <origin xyz="0 0 0"/>
          <mass value="1"/>
          <inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/>
        </inertial>
      </link>
      <link name="tip">
        <inertial>
          <origin xyz=".5 0 0"/>
          <mass value="2"/>
          <inertia ixx=".1" ixy="0" ixz="0" iyy=".2" iyz="0" izz=".2"/>
        </inertial>
        <collision><geometry><box size="1 .1 .1"/></geometry></collision>
      </link>
      <joint name="shoulder" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0 0 1"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" velocity="2" effort="5"/>
      </joint>
    </robot>
    "#;

    #[test]
    fn parses_canonical_model() {
        let model = load_urdf(ARM).unwrap();
        assert_eq!(model.dof, 1);
        assert_eq!(model.bodies.len(), 2);
        assert_eq!(model.bodies[1].collisions.len(), 1);
        assert_eq!(model.coordinate_names(), vec!["shoulder"]);
    }

    #[test]
    fn parses_visual_geometry_and_named_material() {
        let source = r#"
        <robot name="visual">
          <material name="wood"><color rgba="1 .5 .25 .8"/></material>
          <link name="base">
            <visual>
              <origin xyz="1 2 3"/>
              <geometry><mesh filename="package://demo/mesh.stl" scale="2 3 4"/></geometry>
              <material name="wood"/>
            </visual>
            <inertial>
              <mass value="1"/>
              <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
            </inertial>
          </link>
        </robot>
        "#;
        let model = load_urdf(source).unwrap();
        assert_eq!(model.bodies[0].visuals.len(), 1);
        assert_eq!(model.bodies[0].visuals[0].rgba, [1.0, 0.5, 0.25, 0.8]);
        let CollisionShape::Mesh {
            filename,
            scale,
            body_from_shape,
        } = &model.bodies[0].visuals[0].geometry
        else {
            panic!("expected visual mesh");
        };
        assert_eq!(filename, "package://demo/mesh.stl");
        assert_eq!(*scale, Vec3::new(2.0, 3.0, 4.0));
        assert_eq!(body_from_shape.translation.vector, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn parses_repository_humanoid() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = load_urdf(source).unwrap();
        assert_eq!(model.dof, 18);
        assert!(model.body_id("left_hand").is_some());
        assert!(model.body_id("right_foot").is_some());
        assert!(model.bodies.iter().map(|body| body.mass).sum::<f64>() > 40.0);
    }

    #[test]
    fn parses_pinned_upkie_reference() {
        let source = include_str!("../../../models/upkie/upkie.urdf");
        let model = load_urdf(source).unwrap();
        assert_eq!(model.dof, 6);
        assert_eq!(model.bodies.len(), 41);
        assert_eq!(
            model.coordinate_names(),
            vec![
                "left_hip",
                "left_knee",
                "left_wheel",
                "right_hip",
                "right_knee",
                "right_wheel"
            ]
        );
        assert!((model.bodies.iter().map(|body| body.mass).sum::<f64>() - 5.33922).abs() < 1e-12);
        assert_eq!(
            model
                .bodies
                .iter()
                .map(|body| body.visuals.len())
                .sum::<usize>(),
            41
        );
        assert_eq!(
            model
                .bodies
                .iter()
                .flat_map(|body| &body.visuals)
                .filter(|visual| matches!(visual.geometry, CollisionShape::Mesh { .. }))
                .count(),
            25
        );
    }
}
