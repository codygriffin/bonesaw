use std::collections::BTreeMap;

use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix3, Point3, Translation3, Unit, UnitQuaternion, Vector3,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::math::{Force6, Motion6, SpatialAcceleration6, Transform3, Vec3};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct JointId(pub usize);

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct BodyId(pub usize);

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct FrameId(pub usize);

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimit {
    #[serde(with = "serde_extended_f64")]
    pub lower: f64,
    #[serde(with = "serde_extended_f64")]
    pub upper: f64,
    #[serde(with = "serde_extended_f64")]
    pub velocity: f64,
    #[serde(with = "serde_extended_f64")]
    pub effort: f64,
}

pub(crate) mod serde_extended_f64 {
    use std::fmt;

    use serde::{
        Deserializer, Serializer,
        de::{Error, Visitor},
    };

    pub fn serialize<S>(value: &f64, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if value.is_finite() {
            serializer.serialize_f64(*value)
        } else if value.is_sign_positive() {
            serializer.serialize_str("Infinity")
        } else {
            serializer.serialize_str("-Infinity")
        }
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<f64, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ExtendedF64Visitor;

        impl<'de> Visitor<'de> for ExtendedF64Visitor {
            type Value = f64;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a number, \"Infinity\", or \"-Infinity\"")
            }

            fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E> {
                Ok(value)
            }

            fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E> {
                Ok(value as f64)
            }

            fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E> {
                Ok(value as f64)
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
            where
                E: Error,
            {
                match value {
                    "Infinity" => Ok(f64::INFINITY),
                    "-Infinity" => Ok(f64::NEG_INFINITY),
                    _ => Err(E::custom("invalid extended floating-point value")),
                }
            }
        }

        deserializer.deserialize_any(ExtendedF64Visitor)
    }
}

impl JointLimit {
    pub fn unbounded() -> Self {
        Self {
            lower: f64::NEG_INFINITY,
            upper: f64::INFINITY,
            velocity: f64::INFINITY,
            effort: f64::INFINITY,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointSpec {
    pub id: JointId,
    pub name: String,
    pub kind: JointKind,
    pub parent: BodyId,
    pub child: BodyId,
    #[serde(with = "crate::math::serde_transform3")]
    pub parent_from_joint: Transform3,
    pub axis_in_joint: Vec3,
    pub coordinate: Option<usize>,
    pub limit: JointLimit,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CollisionShape {
    Sphere {
        radius: f64,
        #[serde(with = "crate::math::serde_transform3")]
        body_from_shape: Transform3,
    },
    Capsule {
        radius: f64,
        half_length: f64,
        #[serde(with = "crate::math::serde_transform3")]
        body_from_shape: Transform3,
    },
    Box {
        half_extents: Vec3,
        #[serde(with = "crate::math::serde_transform3")]
        body_from_shape: Transform3,
    },
    Cylinder {
        radius: f64,
        half_length: f64,
        #[serde(with = "crate::math::serde_transform3")]
        body_from_shape: Transform3,
    },
    Mesh {
        filename: String,
        scale: Vec3,
        #[serde(with = "crate::math::serde_transform3")]
        body_from_shape: Transform3,
    },
}

/// Model-authored render geometry in body-local coordinates.
///
/// Visuals deliberately reuse the collision geometry vocabulary: URDF uses the
/// same primitive and mesh forms for both, while dynamics remains isolated from
/// material/color concerns.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VisualShape {
    pub geometry: CollisionShape,
    pub rgba: [f64; 4],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RigidBodySpec {
    pub id: BodyId,
    pub name: String,
    pub parent_joint: Option<JointId>,
    pub body_frame: FrameId,
    pub mass: f64,
    pub com_in_body: Vec3,
    pub inertia_about_com_in_body: Matrix3<f64>,
    pub collisions: Vec<CollisionShape>,
    /// Authored URDF visuals. Empty is omitted so schema-5 archives produced
    /// before visual ingestion retain their canonical fingerprint.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub visuals: Vec<VisualShape>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompiledModel {
    pub name: String,
    pub bodies: Vec<RigidBodySpec>,
    pub joints: Vec<JointSpec>,
    pub dof: usize,
    pub root: BodyId,
    #[serde(skip)]
    body_names: BTreeMap<String, BodyId>,
    #[serde(skip)]
    joint_names: BTreeMap<String, JointId>,
}

#[derive(Debug)]
pub struct RobotState {
    /// Pose of the canonical model root in the smooth `control_world` frame.
    pub control_world_from_root: Transform3,
    pub q: DVector<f64>,
    pub v: DVector<f64>,
}

/// Authoritative floating-base state.
///
/// The root tangent is expressed in `control_world` and ordered
/// `[angular; linear]`. Keeping it beside the pose avoids the ambiguous,
/// caller-local root velocity that otherwise appears in floating-base loops.
#[derive(Debug)]
pub struct FloatingRobotState {
    pub robot: RobotState,
    pub root_twist_world: Motion6,
}

impl Clone for FloatingRobotState {
    fn clone(&self) -> Self {
        Self {
            robot: self.robot.clone(),
            root_twist_world: self.root_twist_world,
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.robot.clone_from(&source.robot);
        self.root_twist_world = source.root_twist_world;
    }
}

impl FloatingRobotState {
    pub fn zeros(model: &CompiledModel) -> Self {
        Self {
            robot: RobotState::zeros(model),
            root_twist_world: Motion6::default(),
        }
    }

    pub fn validate(&self, model: &CompiledModel) -> Result<(), ModelError> {
        self.robot.validate(model)?;
        if !self
            .root_twist_world
            .0
            .iter()
            .all(|component| component.is_finite())
        {
            return Err(ModelError::NonFiniteState);
        }
        Ok(())
    }
}

impl Clone for RobotState {
    fn clone(&self) -> Self {
        Self {
            control_world_from_root: self.control_world_from_root,
            q: self.q.clone(),
            v: self.v.clone(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.control_world_from_root = source.control_world_from_root;
        if self.q.len() == source.q.len() {
            self.q.as_mut_slice().copy_from_slice(source.q.as_slice());
        } else {
            self.q.clone_from(&source.q);
        }
        if self.v.len() == source.v.len() {
            self.v.as_mut_slice().copy_from_slice(source.v.as_slice());
        } else {
            self.v.clone_from(&source.v);
        }
    }
}

impl RobotState {
    pub fn zeros(model: &CompiledModel) -> Self {
        Self {
            control_world_from_root: Transform3::identity(),
            q: DVector::zeros(model.dof),
            v: DVector::zeros(model.dof),
        }
    }

    pub fn validate(&self, model: &CompiledModel) -> Result<(), ModelError> {
        if self.q.len() != model.dof || self.v.len() != model.dof {
            return Err(ModelError::StateDimension {
                expected: model.dof,
                q: self.q.len(),
                v: self.v.len(),
            });
        }
        if !self.q.iter().chain(self.v.iter()).all(|x| x.is_finite()) {
            return Err(ModelError::NonFiniteState);
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ModelCache {
    pub world_from_body: Vec<Transform3>,
    pub world_from_joint: Vec<Transform3>,
    pub joint_axis_world: Vec<Vec3>,
    pub center_of_mass_world: Vec3,
    pub total_mass: f64,
}

impl ModelCache {
    pub fn new(model: &CompiledModel) -> Self {
        Self {
            world_from_body: vec![Transform3::identity(); model.bodies.len()],
            world_from_joint: vec![Transform3::identity(); model.joints.len()],
            joint_axis_world: vec![Vec3::zeros(); model.joints.len()],
            center_of_mass_world: Vec3::zeros(),
            total_mass: 0.0,
        }
    }
}

/// Caller-owned recursive dynamics workspace for one model evaluation.
#[derive(Clone, Debug)]
pub struct DynamicsCache {
    pub angular_velocity_world: Vec<Vec3>,
    pub angular_acceleration_world: Vec<Vec3>,
    pub linear_velocity_origin_world: Vec<Vec3>,
    pub linear_acceleration_origin_world: Vec<Vec3>,
    linear_jacobian: DMatrix<f64>,
    angular_jacobian: DMatrix<f64>,
    floating_linear_jacobian: DMatrix<f64>,
    floating_angular_jacobian: DMatrix<f64>,
}

impl DynamicsCache {
    pub fn new(model: &CompiledModel) -> Self {
        let bodies = model.bodies.len();
        Self {
            angular_velocity_world: vec![Vec3::zeros(); bodies],
            angular_acceleration_world: vec![Vec3::zeros(); bodies],
            linear_velocity_origin_world: vec![Vec3::zeros(); bodies],
            linear_acceleration_origin_world: vec![Vec3::zeros(); bodies],
            linear_jacobian: DMatrix::zeros(3, model.dof),
            angular_jacobian: DMatrix::zeros(3, model.dof),
            floating_linear_jacobian: DMatrix::zeros(3, model.dof + 6),
            floating_angular_jacobian: DMatrix::zeros(3, model.dof + 6),
        }
    }

    pub fn reset(&mut self) {
        self.angular_velocity_world.fill(Vec3::zeros());
        self.angular_acceleration_world.fill(Vec3::zeros());
        self.linear_velocity_origin_world.fill(Vec3::zeros());
        self.linear_acceleration_origin_world.fill(Vec3::zeros());
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FrameQuery {
    pub from: FrameId,
    pub to: FrameId,
}

#[derive(Clone, Debug)]
pub struct FrameEstimate {
    pub from_to: Transform3,
    pub twist_to_relative_from_expressed_in_from: Motion6,
    pub acceleration_to_relative_from_expressed_in_from: SpatialAcceleration6,
}

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("the model has no bodies")]
    Empty,
    #[error("body or joint name `{0}` is duplicated")]
    DuplicateName(String),
    #[error("model root/body topology is invalid: {0}")]
    Topology(String),
    #[error("body `{body}` has invalid mass/inertia: {reason}")]
    InvalidInertia { body: String, reason: String },
    #[error("state dimension mismatch: expected {expected}, q={q}, v={v}")]
    StateDimension { expected: usize, q: usize, v: usize },
    #[error("robot state contains NaN or infinity")]
    NonFiniteState,
    #[error("generalized acceleration dimension mismatch: expected {expected}, actual {actual}")]
    AccelerationDimension { expected: usize, actual: usize },
    #[error("mass matrix is singular or numerically indefinite")]
    SingularMassMatrix,
    #[error("unknown body/frame `{0}`")]
    UnknownFrame(String),
    #[error("frame id {0} is out of range")]
    FrameOutOfRange(usize),
}

impl CompiledModel {
    pub fn new(
        name: String,
        mut bodies: Vec<RigidBodySpec>,
        mut joints: Vec<JointSpec>,
    ) -> Result<Self, ModelError> {
        if bodies.is_empty() {
            return Err(ModelError::Empty);
        }

        let mut body_names = BTreeMap::new();
        for (index, body) in bodies.iter_mut().enumerate() {
            body.id = BodyId(index);
            body.body_frame = FrameId(index);
            if body_names.insert(body.name.clone(), body.id).is_some() {
                return Err(ModelError::DuplicateName(body.name.clone()));
            }
        }

        let mut joint_names = BTreeMap::new();
        let mut dof = 0;
        for (index, joint) in joints.iter_mut().enumerate() {
            joint.id = JointId(index);
            if joint_names.insert(joint.name.clone(), joint.id).is_some() {
                return Err(ModelError::DuplicateName(joint.name.clone()));
            }
            if joint.parent.0 >= bodies.len() || joint.child.0 >= bodies.len() {
                return Err(ModelError::Topology(format!(
                    "joint {} refers to a missing body",
                    joint.name
                )));
            }
            joint.coordinate = if joint.kind == JointKind::Fixed {
                None
            } else {
                let coordinate = dof;
                dof += 1;
                Some(coordinate)
            };
            bodies[joint.child.0].parent_joint = Some(joint.id);
        }

        fold_fixed_body_inertias(&mut bodies, &joints);
        for body in &bodies {
            validate_inertia(body)?;
        }

        let roots: Vec<_> = bodies
            .iter()
            .filter(|body| body.parent_joint.is_none())
            .map(|body| body.id)
            .collect();
        if roots.len() != 1 {
            return Err(ModelError::Topology(format!(
                "expected one root, found {}",
                roots.len()
            )));
        }

        // The URDF importer emits parents before children. Verify this invariant
        // because FK intentionally uses a flat bounded loop.
        for joint in &joints {
            if joint.parent.0 >= joint.child.0 {
                return Err(ModelError::Topology(format!(
                    "joint {} is not in parent-before-child order",
                    joint.name
                )));
            }
        }

        Ok(Self {
            name,
            bodies,
            joints,
            dof,
            root: roots[0],
            body_names,
            joint_names,
        })
    }

    pub fn rebuild_indexes(&mut self) {
        self.body_names = self
            .bodies
            .iter()
            .map(|body| (body.name.clone(), body.id))
            .collect();
        self.joint_names = self
            .joints
            .iter()
            .map(|joint| (joint.name.clone(), joint.id))
            .collect();
    }

    pub fn body_id(&self, name: &str) -> Option<BodyId> {
        self.body_names.get(name).copied()
    }

    pub fn frame_id(&self, name: &str) -> Option<FrameId> {
        self.body_id(name).map(|id| FrameId(id.0))
    }

    pub fn joint_id(&self, name: &str) -> Option<JointId> {
        self.joint_names.get(name).copied()
    }

    pub fn coordinate_names(&self) -> Vec<&str> {
        let mut names = vec![""; self.dof];
        for joint in &self.joints {
            if let Some(index) = joint.coordinate {
                names[index] = &joint.name;
            }
        }
        names
    }

    pub fn forward_kinematics(
        &self,
        state: &RobotState,
        cache: &mut ModelCache,
    ) -> Result<(), ModelError> {
        state.validate(self)?;
        cache.world_from_body[self.root.0] = state.control_world_from_root;

        for joint in &self.joints {
            let parent_pose = cache.world_from_body[joint.parent.0];
            let joint_pose = parent_pose * joint.parent_from_joint;
            cache.world_from_joint[joint.id.0] = joint_pose;
            cache.joint_axis_world[joint.id.0] =
                joint_pose.rotation.transform_vector(&joint.axis_in_joint);

            let motion = match (joint.kind, joint.coordinate) {
                (JointKind::Revolute | JointKind::Continuous, Some(index)) => {
                    let axis = Unit::new_normalize(joint.axis_in_joint);
                    Isometry3::from_parts(
                        Translation3::identity(),
                        UnitQuaternion::from_axis_angle(&axis, state.q[index]),
                    )
                }
                (JointKind::Prismatic, Some(index)) => Isometry3::from_parts(
                    Translation3::from(joint.axis_in_joint * state.q[index]),
                    UnitQuaternion::identity(),
                ),
                _ => Transform3::identity(),
            };
            cache.world_from_body[joint.child.0] = joint_pose * motion;
        }

        let mut weighted = Vec3::zeros();
        let mut total = 0.0;
        for body in &self.bodies {
            let com = cache.world_from_body[body.id.0]
                .transform_point(&Point3::from(body.com_in_body))
                .coords;
            weighted += body.mass * com;
            total += body.mass;
        }
        cache.total_mass = total;
        cache.center_of_mass_world = if total > 0.0 {
            weighted / total
        } else {
            Vec3::zeros()
        };
        Ok(())
    }

    pub fn frame_pose(&self, cache: &ModelCache, frame: FrameId) -> Result<Transform3, ModelError> {
        cache
            .world_from_body
            .get(frame.0)
            .copied()
            .ok_or(ModelError::FrameOutOfRange(frame.0))
    }

    pub fn query_frame(
        &self,
        state: &RobotState,
        cache: &ModelCache,
        query: FrameQuery,
    ) -> Result<FrameEstimate, ModelError> {
        let world_from_from = self.frame_pose(cache, query.from)?;
        let world_from_to = self.frame_pose(cache, query.to)?;
        let from_to = world_from_from.inverse() * world_from_to;

        let jac_to = self.frame_jacobian(cache, query.to)?;
        let jac_from = self.frame_jacobian(cache, query.from)?;
        let twist_to_world = jac_to * &state.v;
        let twist_from_world = jac_from * &state.v;
        let angular_from_world = Vector3::new(
            twist_from_world[0],
            twist_from_world[1],
            twist_from_world[2],
        );
        let angular_relative_world = Vector3::new(
            twist_to_world[0] - twist_from_world[0],
            twist_to_world[1] - twist_from_world[1],
            twist_to_world[2] - twist_from_world[2],
        );
        let origin_offset_world =
            world_from_to.translation.vector - world_from_from.translation.vector;
        let linear_relative_world = Vector3::new(
            twist_to_world[3] - twist_from_world[3],
            twist_to_world[4] - twist_from_world[4],
            twist_to_world[5] - twist_from_world[5],
        ) - angular_from_world.cross(&origin_offset_world);
        let rotation = world_from_from.rotation.inverse();
        let angular = rotation.transform_vector(&angular_relative_world);
        let linear = rotation.transform_vector(&linear_relative_world);
        Ok(FrameEstimate {
            from_to,
            twist_to_relative_from_expressed_in_from: Motion6(nalgebra::SVector::<f64, 6>::new(
                angular.x, angular.y, angular.z, linear.x, linear.y, linear.z,
            )),
            acceleration_to_relative_from_expressed_in_from: SpatialAcceleration6::default(),
        })
    }

    /// Geometric frame Jacobian, ordered `[angular; linear]`, expressed in world.
    pub fn frame_jacobian(
        &self,
        cache: &ModelCache,
        frame: FrameId,
    ) -> Result<DMatrix<f64>, ModelError> {
        let mut jacobian = DMatrix::zeros(6, self.dof);
        self.frame_jacobian_into(cache, frame, &mut jacobian)?;
        Ok(jacobian)
    }

    pub fn frame_jacobian_into(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        jacobian: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if frame.0 >= self.bodies.len() {
            return Err(ModelError::FrameOutOfRange(frame.0));
        }
        if jacobian.nrows() != 6 || jacobian.ncols() != self.dof {
            *jacobian = DMatrix::zeros(6, self.dof);
        } else {
            jacobian.fill(0.0);
        }
        let point = cache.world_from_body[frame.0].translation.vector;
        let mut body = BodyId(frame.0);

        while let Some(joint_id) = self.bodies[body.0].parent_joint {
            let joint = &self.joints[joint_id.0];
            if let Some(column) = joint.coordinate {
                let axis = cache.joint_axis_world[joint_id.0];
                let origin = cache.world_from_joint[joint_id.0].translation.vector;
                match joint.kind {
                    JointKind::Revolute | JointKind::Continuous => {
                        let linear = axis.cross(&(point - origin));
                        jacobian[(0, column)] = axis.x;
                        jacobian[(1, column)] = axis.y;
                        jacobian[(2, column)] = axis.z;
                        jacobian[(3, column)] = linear.x;
                        jacobian[(4, column)] = linear.y;
                        jacobian[(5, column)] = linear.z;
                    }
                    JointKind::Prismatic => {
                        jacobian[(3, column)] = axis.x;
                        jacobian[(4, column)] = axis.y;
                        jacobian[(5, column)] = axis.z;
                    }
                    JointKind::Fixed => {}
                }
            }
            body = joint.parent;
        }
        Ok(())
    }

    pub fn point_jacobian(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        point_in_frame: Vec3,
    ) -> Result<DMatrix<f64>, ModelError> {
        let mut result = DMatrix::zeros(3, self.dof);
        self.point_jacobian_into(cache, frame, point_in_frame, &mut result)?;
        Ok(result)
    }

    pub fn angular_jacobian_into(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        jacobian: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if frame.0 >= self.bodies.len() {
            return Err(ModelError::FrameOutOfRange(frame.0));
        }
        if jacobian.nrows() != 3 || jacobian.ncols() != self.dof {
            *jacobian = DMatrix::zeros(3, self.dof);
        } else {
            jacobian.fill(0.0);
        }
        let mut body = BodyId(frame.0);
        while let Some(joint_id) = self.bodies[body.0].parent_joint {
            let joint = &self.joints[joint_id.0];
            if let Some(column) = joint.coordinate
                && matches!(joint.kind, JointKind::Revolute | JointKind::Continuous)
            {
                let axis = cache.joint_axis_world[joint_id.0];
                jacobian[(0, column)] = axis.x;
                jacobian[(1, column)] = axis.y;
                jacobian[(2, column)] = axis.z;
            }
            body = joint.parent;
        }
        Ok(())
    }

    pub fn point_jacobian_into(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        point_in_frame: Vec3,
        result: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        let pose = self.frame_pose(cache, frame)?;
        let point_world = pose.transform_point(&Point3::from(point_in_frame)).coords;
        if result.nrows() != 3 || result.ncols() != self.dof {
            *result = DMatrix::zeros(3, self.dof);
        } else {
            result.fill(0.0);
        }
        self.accumulate_point_jacobian(cache, frame, point_world, 1.0, result);
        Ok(())
    }

    /// Point Jacobian for a floating root tangent ordered
    /// `[root angular; root linear; joint velocity]`, expressed in world.
    pub fn floating_point_jacobian_into(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        point_in_frame: Vec3,
        result: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        let pose = self.frame_pose(cache, frame)?;
        let point_world = pose.transform_point(&Point3::from(point_in_frame)).coords;
        let root_origin = cache.world_from_body[self.root.0].translation.vector;
        let columns = self.dof + 6;
        if result.nrows() != 3 || result.ncols() != columns {
            *result = DMatrix::zeros(3, columns);
        } else {
            result.fill(0.0);
        }
        for axis in 0..3 {
            let mut basis = Vec3::zeros();
            basis[axis] = 1.0;
            let linear = basis.cross(&(point_world - root_origin));
            for row in 0..3 {
                result[(row, axis)] = linear[row];
            }
            result[(axis, axis + 3)] = 1.0;
        }
        let mut body = BodyId(frame.0);
        while let Some(joint_id) = self.bodies[body.0].parent_joint {
            let joint = &self.joints[joint_id.0];
            if let Some(column) = joint.coordinate {
                let axis = cache.joint_axis_world[joint_id.0];
                match joint.kind {
                    JointKind::Revolute | JointKind::Continuous => {
                        let origin = cache.world_from_joint[joint_id.0].translation.vector;
                        let linear = axis.cross(&(point_world - origin));
                        for row in 0..3 {
                            result[(row, 6 + column)] = linear[row];
                        }
                    }
                    JointKind::Prismatic => {
                        for row in 0..3 {
                            result[(row, 6 + column)] = axis[row];
                        }
                    }
                    JointKind::Fixed => {}
                }
            }
            body = joint.parent;
        }
        Ok(())
    }

    pub fn floating_angular_jacobian_into(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        result: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if frame.0 >= self.bodies.len() {
            return Err(ModelError::FrameOutOfRange(frame.0));
        }
        let columns = self.dof + 6;
        if result.nrows() != 3 || result.ncols() != columns {
            *result = DMatrix::zeros(3, columns);
        } else {
            result.fill(0.0);
        }
        for axis in 0..3 {
            result[(axis, axis)] = 1.0;
        }
        let mut body = BodyId(frame.0);
        while let Some(joint_id) = self.bodies[body.0].parent_joint {
            let joint = &self.joints[joint_id.0];
            if let Some(column) = joint.coordinate
                && matches!(joint.kind, JointKind::Revolute | JointKind::Continuous)
            {
                let axis = cache.joint_axis_world[joint_id.0];
                for row in 0..3 {
                    result[(row, 6 + column)] = axis[row];
                }
            }
            body = joint.parent;
        }
        Ok(())
    }

    pub fn com_jacobian(&self, cache: &ModelCache) -> Result<DMatrix<f64>, ModelError> {
        let mut jacobian = DMatrix::zeros(3, self.dof);
        self.com_jacobian_into(cache, &mut jacobian)?;
        Ok(jacobian)
    }

    pub fn com_jacobian_into(
        &self,
        cache: &ModelCache,
        jacobian: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if jacobian.nrows() != 3 || jacobian.ncols() != self.dof {
            *jacobian = DMatrix::zeros(3, self.dof);
        } else {
            jacobian.fill(0.0);
        }
        if cache.total_mass <= 0.0 {
            return Ok(());
        }
        for body in &self.bodies {
            if body.mass == 0.0 {
                continue;
            }
            let point_world = cache.world_from_body[body.id.0]
                .transform_point(&Point3::from(body.com_in_body))
                .coords;
            self.accumulate_point_jacobian(
                cache,
                body.body_frame,
                point_world,
                body.mass / cache.total_mass,
                jacobian,
            );
        }
        Ok(())
    }

    /// Floating-root CoM Jacobian for tangent
    /// `[root angular; root linear; joint velocity]`, expressed in world.
    pub fn floating_com_jacobian_into(
        &self,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        jacobian: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        let generalized_dof = self.dof + 6;
        if jacobian.nrows() != 3 || jacobian.ncols() != generalized_dof {
            *jacobian = DMatrix::zeros(3, generalized_dof);
        } else {
            jacobian.fill(0.0);
        }
        if cache.total_mass <= 0.0 {
            return Ok(());
        }
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.floating_point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.floating_linear_jacobian,
            )?;
            let scale = body.mass / cache.total_mass;
            for row in 0..3 {
                for column in 0..generalized_dof {
                    jacobian[(row, column)] +=
                        scale * dynamics_cache.floating_linear_jacobian[(row, column)];
                }
            }
        }
        Ok(())
    }

    /// Kinematic `J̇v` term of the system CoM acceleration in world.
    ///
    /// The dynamics cache must have been populated for the current state and
    /// floating root twist with zero generalized acceleration, as done by
    /// `floating_bias_forces_into`.
    pub fn center_of_mass_bias_acceleration_world(
        &self,
        cache: &ModelCache,
        dynamics_cache: &DynamicsCache,
    ) -> Result<Vec3, ModelError> {
        if cache.total_mass <= 0.0 {
            return Ok(Vec3::zeros());
        }
        let mut acceleration = Vec3::zeros();
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            acceleration += body.mass
                * self.point_bias_acceleration_world(
                    body.body_frame,
                    body.com_in_body,
                    cache,
                    dynamics_cache,
                )?;
        }
        Ok(acceleration / cache.total_mass)
    }

    /// Centroidal momentum map, ordered `[angular momentum; linear momentum]`
    /// and expressed in control world about the system center of mass.
    pub fn centroidal_map(&self, cache: &ModelCache) -> Result<DMatrix<f64>, ModelError> {
        let mut map = DMatrix::zeros(6, self.dof);
        let mut dynamics_cache = DynamicsCache::new(self);
        self.centroidal_map_into(cache, &mut dynamics_cache, &mut map)?;
        Ok(map)
    }

    pub fn centroidal_map_into(
        &self,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        map: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if map.nrows() != 6 || map.ncols() != self.dof {
            *map = DMatrix::zeros(6, self.dof);
        } else {
            map.fill(0.0);
        }
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.linear_jacobian,
            )?;
            self.angular_jacobian_into(
                cache,
                body.body_frame,
                &mut dynamics_cache.angular_jacobian,
            )?;
            let body_pose = cache.world_from_body[body.id.0];
            let body_com_world = body_pose
                .transform_point(&Point3::from(body.com_in_body))
                .coords;
            let offset_from_system_com = body_com_world - cache.center_of_mass_world;
            let rotation = body_pose.rotation.to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            for coordinate in 0..self.dof {
                let linear_velocity_column = Vec3::new(
                    dynamics_cache.linear_jacobian[(0, coordinate)],
                    dynamics_cache.linear_jacobian[(1, coordinate)],
                    dynamics_cache.linear_jacobian[(2, coordinate)],
                );
                let angular_velocity_column = Vec3::new(
                    dynamics_cache.angular_jacobian[(0, coordinate)],
                    dynamics_cache.angular_jacobian[(1, coordinate)],
                    dynamics_cache.angular_jacobian[(2, coordinate)],
                );
                let linear_momentum = body.mass * linear_velocity_column;
                let angular_momentum = inertia_world * angular_velocity_column
                    + offset_from_system_com.cross(&linear_momentum);
                for axis in 0..3 {
                    map[(axis, coordinate)] += angular_momentum[axis];
                    map[(axis + 3, coordinate)] += linear_momentum[axis];
                }
            }
        }
        Ok(())
    }

    pub fn centroidal_momentum_into(
        &self,
        state: &RobotState,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        map: &mut DMatrix<f64>,
        momentum: &mut Force6,
    ) -> Result<(), ModelError> {
        state.validate(self)?;
        self.centroidal_map_into(cache, dynamics_cache, map)?;
        momentum.0.fill(0.0);
        for row in 0..6 {
            momentum.0[row] = (0..self.dof)
                .map(|coordinate| map[(row, coordinate)] * state.v[coordinate])
                .sum();
        }
        Ok(())
    }

    pub fn floating_centroidal_map_into(
        &self,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        map: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        let generalized_dof = self.dof + 6;
        if map.nrows() != 6 || map.ncols() != generalized_dof {
            *map = DMatrix::zeros(6, generalized_dof);
        } else {
            map.fill(0.0);
        }
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.floating_point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.floating_linear_jacobian,
            )?;
            self.floating_angular_jacobian_into(
                cache,
                body.body_frame,
                &mut dynamics_cache.floating_angular_jacobian,
            )?;
            let body_pose = cache.world_from_body[body.id.0];
            let body_com_world = body_pose
                .transform_point(&Point3::from(body.com_in_body))
                .coords;
            let offset_from_system_com = body_com_world - cache.center_of_mass_world;
            let rotation = body_pose.rotation.to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            for coordinate in 0..generalized_dof {
                let linear_velocity_column = Vec3::new(
                    dynamics_cache.floating_linear_jacobian[(0, coordinate)],
                    dynamics_cache.floating_linear_jacobian[(1, coordinate)],
                    dynamics_cache.floating_linear_jacobian[(2, coordinate)],
                );
                let angular_velocity_column = Vec3::new(
                    dynamics_cache.floating_angular_jacobian[(0, coordinate)],
                    dynamics_cache.floating_angular_jacobian[(1, coordinate)],
                    dynamics_cache.floating_angular_jacobian[(2, coordinate)],
                );
                let linear_momentum = body.mass * linear_velocity_column;
                let angular_momentum = inertia_world * angular_velocity_column
                    + offset_from_system_com.cross(&linear_momentum);
                for axis in 0..3 {
                    map[(axis, coordinate)] += angular_momentum[axis];
                    map[(axis + 3, coordinate)] += linear_momentum[axis];
                }
            }
        }
        Ok(())
    }

    fn accumulate_point_jacobian(
        &self,
        cache: &ModelCache,
        frame: FrameId,
        point_world: Vec3,
        scale: f64,
        jacobian: &mut DMatrix<f64>,
    ) {
        let mut body = BodyId(frame.0);
        while let Some(joint_id) = self.bodies[body.0].parent_joint {
            let joint = &self.joints[joint_id.0];
            if let Some(column) = joint.coordinate {
                let axis = cache.joint_axis_world[joint_id.0];
                match joint.kind {
                    JointKind::Revolute | JointKind::Continuous => {
                        let origin = cache.world_from_joint[joint_id.0].translation.vector;
                        let linear = axis.cross(&(point_world - origin)) * scale;
                        jacobian[(0, column)] += linear.x;
                        jacobian[(1, column)] += linear.y;
                        jacobian[(2, column)] += linear.z;
                    }
                    JointKind::Prismatic => {
                        jacobian[(0, column)] += scale * axis.x;
                        jacobian[(1, column)] += scale * axis.y;
                        jacobian[(2, column)] += scale * axis.z;
                    }
                    JointKind::Fixed => {}
                }
            }
            body = joint.parent;
        }
    }

    /// Joint-space mass matrix assembled from body CoM and angular Jacobians.
    ///
    /// This Jacobian form is the CPU reference path for the initial concept. It
    /// is asymptotically slower than CRBA but compact, transparent, and directly
    /// testable against the kinetic-energy identity.
    pub fn mass_matrix(&self, cache: &ModelCache) -> Result<DMatrix<f64>, ModelError> {
        let mut mass_matrix = DMatrix::zeros(self.dof, self.dof);
        let mut dynamics_cache = DynamicsCache::new(self);
        self.mass_matrix_into(cache, &mut dynamics_cache, &mut mass_matrix)?;
        Ok(mass_matrix)
    }

    pub fn mass_matrix_into(
        &self,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        mass_matrix: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        if mass_matrix.nrows() != self.dof || mass_matrix.ncols() != self.dof {
            *mass_matrix = DMatrix::zeros(self.dof, self.dof);
        } else {
            mass_matrix.fill(0.0);
        }
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.linear_jacobian,
            )?;
            self.angular_jacobian_into(
                cache,
                body.body_frame,
                &mut dynamics_cache.angular_jacobian,
            )?;
            let rotation = cache.world_from_body[body.id.0]
                .rotation
                .to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            for row in 0..self.dof {
                let linear_row = Vec3::new(
                    dynamics_cache.linear_jacobian[(0, row)],
                    dynamics_cache.linear_jacobian[(1, row)],
                    dynamics_cache.linear_jacobian[(2, row)],
                );
                let angular_row = Vec3::new(
                    dynamics_cache.angular_jacobian[(0, row)],
                    dynamics_cache.angular_jacobian[(1, row)],
                    dynamics_cache.angular_jacobian[(2, row)],
                );
                for column in 0..=row {
                    let linear_column = Vec3::new(
                        dynamics_cache.linear_jacobian[(0, column)],
                        dynamics_cache.linear_jacobian[(1, column)],
                        dynamics_cache.linear_jacobian[(2, column)],
                    );
                    let angular_column = Vec3::new(
                        dynamics_cache.angular_jacobian[(0, column)],
                        dynamics_cache.angular_jacobian[(1, column)],
                        dynamics_cache.angular_jacobian[(2, column)],
                    );
                    let value = body.mass * linear_row.dot(&linear_column)
                        + angular_row.dot(&(inertia_world * angular_column));
                    mass_matrix[(row, column)] += value;
                    if row != column {
                        mass_matrix[(column, row)] += value;
                    }
                }
            }
        }
        Ok(())
    }

    /// Full floating-root mass matrix for tangent
    /// `[root angular; root linear; joint velocity]`.
    pub fn floating_mass_matrix_into(
        &self,
        cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        mass_matrix: &mut DMatrix<f64>,
    ) -> Result<(), ModelError> {
        let generalized_dof = self.dof + 6;
        if mass_matrix.nrows() != generalized_dof || mass_matrix.ncols() != generalized_dof {
            *mass_matrix = DMatrix::zeros(generalized_dof, generalized_dof);
        } else {
            mass_matrix.fill(0.0);
        }
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.floating_point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.floating_linear_jacobian,
            )?;
            self.floating_angular_jacobian_into(
                cache,
                body.body_frame,
                &mut dynamics_cache.floating_angular_jacobian,
            )?;
            let rotation = cache.world_from_body[body.id.0]
                .rotation
                .to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            for row in 0..generalized_dof {
                let linear_row = Vec3::new(
                    dynamics_cache.floating_linear_jacobian[(0, row)],
                    dynamics_cache.floating_linear_jacobian[(1, row)],
                    dynamics_cache.floating_linear_jacobian[(2, row)],
                );
                let angular_row = Vec3::new(
                    dynamics_cache.floating_angular_jacobian[(0, row)],
                    dynamics_cache.floating_angular_jacobian[(1, row)],
                    dynamics_cache.floating_angular_jacobian[(2, row)],
                );
                for column in 0..=row {
                    let linear_column = Vec3::new(
                        dynamics_cache.floating_linear_jacobian[(0, column)],
                        dynamics_cache.floating_linear_jacobian[(1, column)],
                        dynamics_cache.floating_linear_jacobian[(2, column)],
                    );
                    let angular_column = Vec3::new(
                        dynamics_cache.floating_angular_jacobian[(0, column)],
                        dynamics_cache.floating_angular_jacobian[(1, column)],
                        dynamics_cache.floating_angular_jacobian[(2, column)],
                    );
                    let value = body.mass * linear_row.dot(&linear_column)
                        + angular_row.dot(&(inertia_world * angular_column));
                    mass_matrix[(row, column)] += value;
                    if row != column {
                        mass_matrix[(column, row)] += value;
                    }
                }
            }
        }
        Ok(())
    }

    /// Generalized gravity compensation torque for world gravity `[0, 0, -g]`.
    pub fn gravity_forces(
        &self,
        cache: &ModelCache,
        gravity_magnitude: f64,
    ) -> Result<DVector<f64>, ModelError> {
        let mut generalized = DVector::zeros(self.dof);
        let mut dynamics_cache = DynamicsCache::new(self);
        self.gravity_forces_into(
            cache,
            gravity_magnitude,
            &mut dynamics_cache,
            &mut generalized,
        )?;
        Ok(generalized)
    }

    pub fn gravity_forces_into(
        &self,
        cache: &ModelCache,
        gravity_magnitude: f64,
        dynamics_cache: &mut DynamicsCache,
        generalized: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        if generalized.len() != self.dof {
            *generalized = DVector::zeros(self.dof);
        } else {
            generalized.fill(0.0);
        }
        let upward = Vec3::new(0.0, 0.0, gravity_magnitude);
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            self.point_jacobian_into(
                cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.linear_jacobian,
            )?;
            for coordinate in 0..self.dof {
                generalized[coordinate] += body.mass
                    * (dynamics_cache.linear_jacobian[(0, coordinate)] * upward.x
                        + dynamics_cache.linear_jacobian[(1, coordinate)] * upward.y
                        + dynamics_cache.linear_jacobian[(2, coordinate)] * upward.z);
            }
        }
        Ok(())
    }

    /// Recursive fixed-base inverse dynamics in world coordinates.
    ///
    /// The body recursion computes origin/angular accelerations. Generalized
    /// forces are then accumulated from each body's inertial wrench through the
    /// same analytic Jacobians used by WBC task emission.
    pub fn inverse_dynamics(
        &self,
        state: &RobotState,
        generalized_acceleration: &DVector<f64>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
    ) -> Result<DVector<f64>, ModelError> {
        let mut torque = DVector::zeros(self.dof);
        self.inverse_dynamics_into(
            state,
            generalized_acceleration,
            gravity_world,
            model_cache,
            dynamics_cache,
            &mut torque,
        )?;
        Ok(torque)
    }

    pub fn inverse_dynamics_into(
        &self,
        state: &RobotState,
        generalized_acceleration: &DVector<f64>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        torque: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        state.validate(self)?;
        if generalized_acceleration.len() != self.dof {
            return Err(ModelError::AccelerationDimension {
                expected: self.dof,
                actual: generalized_acceleration.len(),
            });
        }
        if !generalized_acceleration
            .iter()
            .all(|value| value.is_finite())
            || !gravity_world.iter().all(|value| value.is_finite())
        {
            return Err(ModelError::NonFiniteState);
        }
        self.inverse_dynamics_impl(
            state,
            Some(generalized_acceleration),
            gravity_world,
            model_cache,
            dynamics_cache,
            torque,
        )
    }

    fn inverse_dynamics_impl(
        &self,
        state: &RobotState,
        generalized_acceleration: Option<&DVector<f64>>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        torque: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        if torque.len() != self.dof {
            *torque = DVector::zeros(self.dof);
        } else {
            torque.fill(0.0);
        }
        dynamics_cache.reset();

        for joint in &self.joints {
            let parent = joint.parent.0;
            let child = joint.child.0;
            let angular_parent = dynamics_cache.angular_velocity_world[parent];
            let angular_acceleration_parent = dynamics_cache.angular_acceleration_world[parent];
            let velocity_parent = dynamics_cache.linear_velocity_origin_world[parent];
            let acceleration_parent = dynamics_cache.linear_acceleration_origin_world[parent];
            let parent_origin = model_cache.world_from_body[parent].translation.vector;
            let joint_origin = model_cache.world_from_joint[joint.id.0].translation.vector;
            let parent_to_joint = joint_origin - parent_origin;
            let velocity_joint = velocity_parent + angular_parent.cross(&parent_to_joint);
            let acceleration_joint = acceleration_parent
                + angular_acceleration_parent.cross(&parent_to_joint)
                + angular_parent.cross(&angular_parent.cross(&parent_to_joint));

            let (position, velocity, acceleration) =
                joint.coordinate.map_or((0.0, 0.0, 0.0), |coordinate| {
                    (
                        state.q[coordinate],
                        state.v[coordinate],
                        generalized_acceleration.map_or(0.0, |value| value[coordinate]),
                    )
                });
            let axis = model_cache.joint_axis_world[joint.id.0];
            match joint.kind {
                JointKind::Revolute | JointKind::Continuous => {
                    dynamics_cache.angular_velocity_world[child] = angular_parent + axis * velocity;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent
                        + axis * acceleration
                        + angular_parent.cross(&(axis * velocity));
                    dynamics_cache.linear_velocity_origin_world[child] = velocity_joint;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint;
                }
                JointKind::Prismatic => {
                    let joint_to_child = axis * position;
                    dynamics_cache.angular_velocity_world[child] = angular_parent;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent;
                    dynamics_cache.linear_velocity_origin_world[child] =
                        velocity_joint + angular_parent.cross(&joint_to_child) + axis * velocity;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint
                        + angular_acceleration_parent.cross(&joint_to_child)
                        + angular_parent.cross(&angular_parent.cross(&joint_to_child))
                        + 2.0 * angular_parent.cross(&(axis * velocity))
                        + axis * acceleration;
                }
                JointKind::Fixed => {
                    dynamics_cache.angular_velocity_world[child] = angular_parent;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent;
                    dynamics_cache.linear_velocity_origin_world[child] = velocity_joint;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint;
                }
            }
        }

        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            let body_index = body.id.0;
            let body_pose = model_cache.world_from_body[body_index];
            let com_offset_world = body_pose.rotation.transform_vector(&body.com_in_body);
            let angular_velocity = dynamics_cache.angular_velocity_world[body_index];
            let angular_acceleration = dynamics_cache.angular_acceleration_world[body_index];
            let com_acceleration = dynamics_cache.linear_acceleration_origin_world[body_index]
                + angular_acceleration.cross(&com_offset_world)
                + angular_velocity.cross(&angular_velocity.cross(&com_offset_world));
            let force = body.mass * (com_acceleration - gravity_world);
            let rotation = body_pose.rotation.to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            let moment_at_com = inertia_world * angular_acceleration
                + angular_velocity.cross(&(inertia_world * angular_velocity));
            self.point_jacobian_into(
                model_cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.linear_jacobian,
            )?;
            self.angular_jacobian_into(
                model_cache,
                body.body_frame,
                &mut dynamics_cache.angular_jacobian,
            )?;
            for coordinate in 0..self.dof {
                torque[coordinate] += dynamics_cache.linear_jacobian[(0, coordinate)] * force.x
                    + dynamics_cache.linear_jacobian[(1, coordinate)] * force.y
                    + dynamics_cache.linear_jacobian[(2, coordinate)] * force.z
                    + dynamics_cache.angular_jacobian[(0, coordinate)] * moment_at_com.x
                    + dynamics_cache.angular_jacobian[(1, coordinate)] * moment_at_com.y
                    + dynamics_cache.angular_jacobian[(2, coordinate)] * moment_at_com.z;
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn floating_inverse_dynamics_into(
        &self,
        state: &RobotState,
        root_twist_world: Motion6,
        generalized_acceleration: &DVector<f64>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        generalized_force: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        let generalized_dof = self.dof + 6;
        if generalized_acceleration.len() != generalized_dof {
            return Err(ModelError::AccelerationDimension {
                expected: generalized_dof,
                actual: generalized_acceleration.len(),
            });
        }
        self.floating_inverse_dynamics_impl(
            state,
            root_twist_world,
            Some(generalized_acceleration),
            gravity_world,
            model_cache,
            dynamics_cache,
            generalized_force,
        )
    }

    pub fn floating_bias_forces_into(
        &self,
        state: &RobotState,
        root_twist_world: Motion6,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        generalized_force: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        self.floating_inverse_dynamics_impl(
            state,
            root_twist_world,
            None,
            gravity_world,
            model_cache,
            dynamics_cache,
            generalized_force,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn floating_inverse_dynamics_impl(
        &self,
        state: &RobotState,
        root_twist_world: Motion6,
        generalized_acceleration: Option<&DVector<f64>>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        generalized_force: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        state.validate(self)?;
        let generalized_dof = self.dof + 6;
        if generalized_acceleration.is_some_and(|value| value.len() != generalized_dof) {
            return Err(ModelError::AccelerationDimension {
                expected: generalized_dof,
                actual: generalized_acceleration.map_or(0, DVector::len),
            });
        }
        if !root_twist_world.0.iter().all(|value| value.is_finite())
            || !gravity_world.iter().all(|value| value.is_finite())
            || generalized_acceleration
                .is_some_and(|value| !value.iter().all(|component| component.is_finite()))
        {
            return Err(ModelError::NonFiniteState);
        }
        if generalized_force.len() != generalized_dof {
            *generalized_force = DVector::zeros(generalized_dof);
        } else {
            generalized_force.fill(0.0);
        }
        dynamics_cache.reset();
        let root = self.root.0;
        dynamics_cache.angular_velocity_world[root] = Vec3::new(
            root_twist_world.0[0],
            root_twist_world.0[1],
            root_twist_world.0[2],
        );
        dynamics_cache.linear_velocity_origin_world[root] = Vec3::new(
            root_twist_world.0[3],
            root_twist_world.0[4],
            root_twist_world.0[5],
        );
        if let Some(acceleration) = generalized_acceleration {
            dynamics_cache.angular_acceleration_world[root] =
                Vec3::new(acceleration[0], acceleration[1], acceleration[2]);
            dynamics_cache.linear_acceleration_origin_world[root] =
                Vec3::new(acceleration[3], acceleration[4], acceleration[5]);
        }

        for joint in &self.joints {
            let parent = joint.parent.0;
            let child = joint.child.0;
            let angular_parent = dynamics_cache.angular_velocity_world[parent];
            let angular_acceleration_parent = dynamics_cache.angular_acceleration_world[parent];
            let velocity_parent = dynamics_cache.linear_velocity_origin_world[parent];
            let acceleration_parent = dynamics_cache.linear_acceleration_origin_world[parent];
            let parent_origin = model_cache.world_from_body[parent].translation.vector;
            let joint_origin = model_cache.world_from_joint[joint.id.0].translation.vector;
            let parent_to_joint = joint_origin - parent_origin;
            let velocity_joint = velocity_parent + angular_parent.cross(&parent_to_joint);
            let acceleration_joint = acceleration_parent
                + angular_acceleration_parent.cross(&parent_to_joint)
                + angular_parent.cross(&angular_parent.cross(&parent_to_joint));
            let (position, velocity, acceleration) =
                joint.coordinate.map_or((0.0, 0.0, 0.0), |coordinate| {
                    (
                        state.q[coordinate],
                        state.v[coordinate],
                        generalized_acceleration.map_or(0.0, |value| value[6 + coordinate]),
                    )
                });
            let axis = model_cache.joint_axis_world[joint.id.0];
            match joint.kind {
                JointKind::Revolute | JointKind::Continuous => {
                    dynamics_cache.angular_velocity_world[child] = angular_parent + axis * velocity;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent
                        + axis * acceleration
                        + angular_parent.cross(&(axis * velocity));
                    dynamics_cache.linear_velocity_origin_world[child] = velocity_joint;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint;
                }
                JointKind::Prismatic => {
                    let joint_to_child = axis * position;
                    dynamics_cache.angular_velocity_world[child] = angular_parent;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent;
                    dynamics_cache.linear_velocity_origin_world[child] =
                        velocity_joint + angular_parent.cross(&joint_to_child) + axis * velocity;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint
                        + angular_acceleration_parent.cross(&joint_to_child)
                        + angular_parent.cross(&angular_parent.cross(&joint_to_child))
                        + 2.0 * angular_parent.cross(&(axis * velocity))
                        + axis * acceleration;
                }
                JointKind::Fixed => {
                    dynamics_cache.angular_velocity_world[child] = angular_parent;
                    dynamics_cache.angular_acceleration_world[child] = angular_acceleration_parent;
                    dynamics_cache.linear_velocity_origin_world[child] = velocity_joint;
                    dynamics_cache.linear_acceleration_origin_world[child] = acceleration_joint;
                }
            }
        }

        let root_origin = model_cache.world_from_body[root].translation.vector;
        for body in &self.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            let body_index = body.id.0;
            let body_pose = model_cache.world_from_body[body_index];
            let com_offset_world = body_pose.rotation.transform_vector(&body.com_in_body);
            let com_world = body_pose.translation.vector + com_offset_world;
            let angular_velocity = dynamics_cache.angular_velocity_world[body_index];
            let angular_acceleration = dynamics_cache.angular_acceleration_world[body_index];
            let com_acceleration = dynamics_cache.linear_acceleration_origin_world[body_index]
                + angular_acceleration.cross(&com_offset_world)
                + angular_velocity.cross(&angular_velocity.cross(&com_offset_world));
            let force = body.mass * (com_acceleration - gravity_world);
            let rotation = body_pose.rotation.to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            let moment_at_com = inertia_world * angular_acceleration
                + angular_velocity.cross(&(inertia_world * angular_velocity));
            let root_moment = moment_at_com + (com_world - root_origin).cross(&force);
            for axis in 0..3 {
                generalized_force[axis] += root_moment[axis];
                generalized_force[axis + 3] += force[axis];
            }

            self.point_jacobian_into(
                model_cache,
                body.body_frame,
                body.com_in_body,
                &mut dynamics_cache.linear_jacobian,
            )?;
            self.angular_jacobian_into(
                model_cache,
                body.body_frame,
                &mut dynamics_cache.angular_jacobian,
            )?;
            for coordinate in 0..self.dof {
                generalized_force[6 + coordinate] +=
                    dynamics_cache.linear_jacobian[(0, coordinate)] * force.x
                        + dynamics_cache.linear_jacobian[(1, coordinate)] * force.y
                        + dynamics_cache.linear_jacobian[(2, coordinate)] * force.z
                        + dynamics_cache.angular_jacobian[(0, coordinate)] * moment_at_com.x
                        + dynamics_cache.angular_jacobian[(1, coordinate)] * moment_at_com.y
                        + dynamics_cache.angular_jacobian[(2, coordinate)] * moment_at_com.z;
            }
        }
        Ok(())
    }

    pub fn bias_forces(
        &self,
        state: &RobotState,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
    ) -> Result<DVector<f64>, ModelError> {
        let mut bias = DVector::zeros(self.dof);
        self.bias_forces_into(state, gravity_world, model_cache, dynamics_cache, &mut bias)?;
        Ok(bias)
    }

    pub fn bias_forces_into(
        &self,
        state: &RobotState,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
        bias: &mut DVector<f64>,
    ) -> Result<(), ModelError> {
        state.validate(self)?;
        if !gravity_world.iter().all(|value| value.is_finite()) {
            return Err(ModelError::NonFiniteState);
        }
        self.inverse_dynamics_impl(
            state,
            None,
            gravity_world,
            model_cache,
            dynamics_cache,
            bias,
        )
    }

    pub fn point_bias_acceleration_world(
        &self,
        frame: FrameId,
        point_in_frame: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &DynamicsCache,
    ) -> Result<Vec3, ModelError> {
        if frame.0 >= self.bodies.len() {
            return Err(ModelError::FrameOutOfRange(frame.0));
        }
        let body_pose = model_cache.world_from_body[frame.0];
        let point_offset_world = body_pose.rotation.transform_vector(&point_in_frame);
        let angular_velocity = dynamics_cache.angular_velocity_world[frame.0];
        let angular_acceleration = dynamics_cache.angular_acceleration_world[frame.0];
        Ok(dynamics_cache.linear_acceleration_origin_world[frame.0]
            + angular_acceleration.cross(&point_offset_world)
            + angular_velocity.cross(&angular_velocity.cross(&point_offset_world)))
    }

    /// Kinematic `J̇v` contribution to a frame's world angular acceleration.
    pub fn angular_bias_acceleration_world(
        &self,
        frame: FrameId,
        dynamics_cache: &DynamicsCache,
    ) -> Result<Vec3, ModelError> {
        if frame.0 >= self.bodies.len() {
            return Err(ModelError::FrameOutOfRange(frame.0));
        }
        Ok(dynamics_cache.angular_acceleration_world[frame.0])
    }

    pub fn forward_dynamics(
        &self,
        state: &RobotState,
        generalized_force: &DVector<f64>,
        gravity_world: Vec3,
        model_cache: &ModelCache,
        dynamics_cache: &mut DynamicsCache,
    ) -> Result<DVector<f64>, ModelError> {
        if generalized_force.len() != self.dof {
            return Err(ModelError::AccelerationDimension {
                expected: self.dof,
                actual: generalized_force.len(),
            });
        }
        let bias = self.bias_forces(state, gravity_world, model_cache, dynamics_cache)?;
        self.mass_matrix(model_cache)?
            .lu()
            .solve(&(generalized_force - bias))
            .ok_or(ModelError::SingularMassMatrix)
    }

    pub fn kinetic_energy(
        &self,
        state: &RobotState,
        cache: &ModelCache,
    ) -> Result<f64, ModelError> {
        let mass_matrix = self.mass_matrix(cache)?;
        Ok(0.5 * state.v.dot(&(&mass_matrix * &state.v)))
    }

    pub fn potential_energy(&self, cache: &ModelCache, gravity_magnitude: f64) -> f64 {
        self.bodies
            .iter()
            .filter(|body| body.mass > 0.0)
            .map(|body| {
                let com = cache.world_from_body[body.id.0]
                    .transform_point(&Point3::from(body.com_in_body))
                    .coords;
                body.mass * gravity_magnitude * com.z
            })
            .sum()
    }

    pub fn integrate(&self, state: &mut RobotState, velocity: &DVector<f64>, dt: f64) {
        for joint in &self.joints {
            if let Some(index) = joint.coordinate {
                let mut q = state.q[index] + velocity[index] * dt;
                if joint.kind == JointKind::Continuous {
                    q = (q + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                        - std::f64::consts::PI;
                    state.v[index] = velocity[index];
                } else {
                    q = q.clamp(joint.limit.lower, joint.limit.upper);
                    state.v[index] = project_limit_velocity(
                        q,
                        velocity[index],
                        joint.limit.lower,
                        joint.limit.upper,
                    );
                }
                state.q[index] = q;
            }
        }
    }

    /// Integrate a floating state under constant world-expressed acceleration.
    ///
    /// This is allocation-free after state construction. The root rotation is
    /// advanced on SO(3) by left multiplication because both angular velocity
    /// and angular acceleration are expressed in `control_world`.
    pub fn integrate_floating(
        &self,
        state: &mut FloatingRobotState,
        generalized_acceleration: &DVector<f64>,
        dt: f64,
    ) -> Result<(), ModelError> {
        let generalized_dof = self.dof + 6;
        if generalized_acceleration.len() != generalized_dof {
            return Err(ModelError::AccelerationDimension {
                expected: generalized_dof,
                actual: generalized_acceleration.len(),
            });
        }
        state.validate(self)?;
        if !dt.is_finite()
            || dt < 0.0
            || !generalized_acceleration
                .iter()
                .all(|component| component.is_finite())
        {
            return Err(ModelError::NonFiniteState);
        }

        let dt_squared_half = 0.5 * dt * dt;
        let angular_step = Vec3::new(
            state.root_twist_world.0[0] * dt + generalized_acceleration[0] * dt_squared_half,
            state.root_twist_world.0[1] * dt + generalized_acceleration[1] * dt_squared_half,
            state.root_twist_world.0[2] * dt + generalized_acceleration[2] * dt_squared_half,
        );
        let linear_step = Vec3::new(
            state.root_twist_world.0[3] * dt + generalized_acceleration[3] * dt_squared_half,
            state.root_twist_world.0[4] * dt + generalized_acceleration[4] * dt_squared_half,
            state.root_twist_world.0[5] * dt + generalized_acceleration[5] * dt_squared_half,
        );
        state.robot.control_world_from_root.rotation =
            UnitQuaternion::from_scaled_axis(angular_step)
                * state.robot.control_world_from_root.rotation;
        state.robot.control_world_from_root.translation.vector += linear_step;

        for axis in 0..6 {
            state.root_twist_world.0[axis] += generalized_acceleration[axis] * dt;
        }
        for joint in &self.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let acceleration = generalized_acceleration[6 + index];
            let mut q =
                state.robot.q[index] + state.robot.v[index] * dt + acceleration * dt_squared_half;
            let next_velocity = state.robot.v[index] + acceleration * dt;
            if joint.kind == JointKind::Continuous {
                q = (q + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                    - std::f64::consts::PI;
                state.robot.v[index] = next_velocity;
            } else {
                q = q.clamp(joint.limit.lower, joint.limit.upper);
                state.robot.v[index] =
                    project_limit_velocity(q, next_velocity, joint.limit.lower, joint.limit.upper);
            }
            state.robot.q[index] = q;
        }
        Ok(())
    }
}

fn project_limit_velocity(position: f64, velocity: f64, lower: f64, upper: f64) -> f64 {
    if (position <= lower && velocity < 0.0) || (position >= upper && velocity > 0.0) {
        0.0
    } else {
        velocity
    }
}

/// Fold fixed-link inertia into the canonical parent body while leaving the
/// authored child body/frame in the semantic namespace.
fn fold_fixed_body_inertias(bodies: &mut [RigidBodySpec], joints: &[JointSpec]) {
    for joint in joints.iter().rev() {
        if joint.kind != JointKind::Fixed {
            continue;
        }
        let child_mass = bodies[joint.child.0].mass;
        if child_mass <= 0.0 {
            continue;
        }
        let child_com = joint
            .parent_from_joint
            .transform_point(&Point3::from(bodies[joint.child.0].com_in_body))
            .coords;
        let child_rotation = joint.parent_from_joint.rotation.to_rotation_matrix();
        let child_inertia = child_rotation.matrix()
            * bodies[joint.child.0].inertia_about_com_in_body
            * child_rotation.matrix().transpose();

        let parent_mass = bodies[joint.parent.0].mass;
        let parent_com = bodies[joint.parent.0].com_in_body;
        let total_mass = parent_mass + child_mass;
        let combined_com = if total_mass > 0.0 {
            (parent_mass * parent_com + child_mass * child_com) / total_mass
        } else {
            Vec3::zeros()
        };
        let parent_shift = parent_com - combined_com;
        let child_shift = child_com - combined_com;
        let combined_inertia = bodies[joint.parent.0].inertia_about_com_in_body
            + parallel_axis(parent_mass, parent_shift)
            + child_inertia
            + parallel_axis(child_mass, child_shift);

        bodies[joint.parent.0].mass = total_mass;
        bodies[joint.parent.0].com_in_body = combined_com;
        bodies[joint.parent.0].inertia_about_com_in_body = combined_inertia;
        bodies[joint.child.0].mass = 0.0;
        bodies[joint.child.0].com_in_body = Vec3::zeros();
        bodies[joint.child.0].inertia_about_com_in_body = Matrix3::zeros();
    }
}

fn parallel_axis(mass: f64, displacement: Vec3) -> Matrix3<f64> {
    mass * (displacement.norm_squared() * Matrix3::identity()
        - displacement * displacement.transpose())
}

fn validate_inertia(body: &RigidBodySpec) -> Result<(), ModelError> {
    if !body.mass.is_finite() || body.mass < 0.0 {
        return Err(ModelError::InvalidInertia {
            body: body.name.clone(),
            reason: "mass must be finite and non-negative".into(),
        });
    }
    if body.mass == 0.0 {
        return Ok(());
    }
    if !body.inertia_about_com_in_body.iter().all(|x| x.is_finite()) {
        return Err(ModelError::InvalidInertia {
            body: body.name.clone(),
            reason: "inertia contains NaN or infinity".into(),
        });
    }
    let asymmetry = body.inertia_about_com_in_body - body.inertia_about_com_in_body.transpose();
    if asymmetry.norm() > 1e-9 {
        return Err(ModelError::InvalidInertia {
            body: body.name.clone(),
            reason: "inertia is materially asymmetric".into(),
        });
    }
    let eigen = body.inertia_about_com_in_body.symmetric_eigen().eigenvalues;
    if eigen.iter().any(|value| *value <= 0.0) {
        return Err(ModelError::InvalidInertia {
            body: body.name.clone(),
            reason: "principal moments must be positive".into(),
        });
    }
    let mut moments = [eigen[0], eigen[1], eigen[2]];
    moments.sort_by(f64::total_cmp);
    if moments[2] > moments[0] + moments[1] + 1e-9 {
        return Err(ModelError::InvalidInertia {
            body: body.name.clone(),
            reason: "principal moments violate the triangle inequality".into(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_link() -> CompiledModel {
        let inertia = Matrix3::identity() * 0.01;
        CompiledModel::new(
            "two-link".into(),
            vec![
                RigidBodySpec {
                    id: BodyId(0),
                    name: "root".into(),
                    parent_joint: None,
                    body_frame: FrameId(0),
                    mass: 1.0,
                    com_in_body: Vec3::zeros(),
                    inertia_about_com_in_body: inertia,
                    collisions: vec![],
                    visuals: vec![],
                },
                RigidBodySpec {
                    id: BodyId(1),
                    name: "tip".into(),
                    parent_joint: None,
                    body_frame: FrameId(1),
                    mass: 1.0,
                    com_in_body: Vec3::new(0.5, 0.0, 0.0),
                    inertia_about_com_in_body: inertia,
                    collisions: vec![],
                    visuals: vec![],
                },
            ],
            vec![JointSpec {
                id: JointId(0),
                name: "hinge".into(),
                kind: JointKind::Revolute,
                parent: BodyId(0),
                child: BodyId(1),
                parent_from_joint: Transform3::translation(1.0, 0.0, 0.0),
                axis_in_joint: Vec3::z(),
                coordinate: None,
                limit: JointLimit::unbounded(),
            }],
        )
        .unwrap()
    }

    #[test]
    fn fk_and_jacobian_agree() {
        let model = two_link();
        let mut state = RobotState::zeros(&model);
        state.q[0] = 0.3;
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let jacobian = model
            .point_jacobian(&cache, FrameId(1), Vec3::new(1.0, 0.0, 0.0))
            .unwrap();

        let epsilon = 1e-7;
        let before = cache.world_from_body[1]
            .transform_point(&Point3::new(1.0, 0.0, 0.0))
            .coords;
        state.q[0] += epsilon;
        model.forward_kinematics(&state, &mut cache).unwrap();
        let after = cache.world_from_body[1]
            .transform_point(&Point3::new(1.0, 0.0, 0.0))
            .coords;
        let numerical = (after - before) / epsilon;
        assert!((numerical - jacobian.column(0)).norm() < 1e-6);
    }

    #[test]
    fn mass_matrix_is_symmetric_positive_and_gravity_is_energy_gradient() {
        let model = two_link();
        let mut state = RobotState::zeros(&model);
        state.q[0] = 0.4;
        state.v[0] = 0.7;
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let mass = model.mass_matrix(&cache).unwrap();
        assert!((&mass - mass.transpose()).norm() < 1e-12);
        assert!(mass[(0, 0)] > 0.0);
        assert!(model.kinetic_energy(&state, &cache).unwrap() > 0.0);

        let gravity = model.gravity_forces(&cache, 9.81).unwrap();
        let epsilon = 1e-7;
        let before = model.potential_energy(&cache, 9.81);
        state.q[0] += epsilon;
        model.forward_kinematics(&state, &mut cache).unwrap();
        let numerical = (model.potential_energy(&cache, 9.81) - before) / epsilon;
        assert!((numerical - gravity[0]).abs() < 1e-5);
    }

    #[test]
    fn inverse_and_forward_dynamics_round_trip() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for index in 0..model.dof {
            state.q[index] = 0.05 * (index as f64 * 0.7).sin();
            state.v[index] = 0.1 * (index as f64 * 0.4).cos();
        }
        let acceleration = DVector::from_iterator(
            model.dof,
            (0..model.dof).map(|index| 0.2 * (index as f64 * 0.3).sin()),
        );
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let mut model_cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut model_cache).unwrap();
        let mut dynamics_cache = DynamicsCache::new(&model);
        let torque = model
            .inverse_dynamics(
                &state,
                &acceleration,
                gravity,
                &model_cache,
                &mut dynamics_cache,
            )
            .unwrap();
        let recovered = model
            .forward_dynamics(&state, &torque, gravity, &model_cache, &mut dynamics_cache)
            .unwrap();
        assert!((&recovered - acceleration).norm() < 1e-8);
    }

    #[test]
    fn inverse_dynamics_columns_match_mass_matrix_at_zero_velocity() {
        let model = two_link();
        let mut state = RobotState::zeros(&model);
        state.q[0] = 0.2;
        let mut model_cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut model_cache).unwrap();
        let mass = model.mass_matrix(&model_cache).unwrap();
        let mut dynamics_cache = DynamicsCache::new(&model);
        let inverse = model
            .inverse_dynamics(
                &state,
                &DVector::from_element(1, 1.0),
                Vec3::zeros(),
                &model_cache,
                &mut dynamics_cache,
            )
            .unwrap();
        assert!((inverse[0] - mass[(0, 0)]).abs() < 1e-10);
    }

    #[test]
    fn centroidal_map_matches_summed_body_momentum() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for index in 0..model.dof {
            state.q[index] = 0.08 * (index as f64 * 0.41).sin();
            state.v[index] = 0.17 * (index as f64 * 0.29).cos();
        }
        let mut model_cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut model_cache).unwrap();
        let mut dynamics_cache = DynamicsCache::new(&model);
        let mut bias = DVector::zeros(model.dof);
        model
            .bias_forces_into(
                &state,
                Vec3::zeros(),
                &model_cache,
                &mut dynamics_cache,
                &mut bias,
            )
            .unwrap();
        let mut map = DMatrix::zeros(6, model.dof);
        let mut mapped = Force6::default();
        model
            .centroidal_momentum_into(
                &state,
                &model_cache,
                &mut dynamics_cache,
                &mut map,
                &mut mapped,
            )
            .unwrap();

        let mut direct_angular = Vec3::zeros();
        let mut direct_linear = Vec3::zeros();
        for body in &model.bodies {
            if body.mass <= 0.0 {
                continue;
            }
            let index = body.id.0;
            let pose = model_cache.world_from_body[index];
            let com_offset = pose.rotation.transform_vector(&body.com_in_body);
            let com_world = pose.translation.vector + com_offset;
            let angular_velocity = dynamics_cache.angular_velocity_world[index];
            let com_velocity = dynamics_cache.linear_velocity_origin_world[index]
                + angular_velocity.cross(&com_offset);
            let linear_momentum = body.mass * com_velocity;
            let rotation = pose.rotation.to_rotation_matrix();
            let inertia_world =
                rotation.matrix() * body.inertia_about_com_in_body * rotation.matrix().transpose();
            direct_linear += linear_momentum;
            direct_angular += inertia_world * angular_velocity
                + (com_world - model_cache.center_of_mass_world).cross(&linear_momentum);
        }
        let mapped_angular = Vec3::new(mapped.0[0], mapped.0[1], mapped.0[2]);
        let mapped_linear = Vec3::new(mapped.0[3], mapped.0[4], mapped.0[5]);
        assert!((mapped_angular - direct_angular).norm() < 1e-10);
        assert!((mapped_linear - direct_linear).norm() < 1e-10);

        let com_jacobian = model.com_jacobian(&model_cache).unwrap();
        let com_velocity = com_jacobian * &state.v;
        assert!((mapped_linear - model_cache.total_mass * com_velocity).norm() < 1e-10);
    }

    #[test]
    fn floating_inverse_dynamics_matches_full_mass_and_bias() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for index in 0..model.dof {
            state.q[index] = 0.06 * (index as f64 * 0.47).sin();
            state.v[index] = 0.11 * (index as f64 * 0.33).cos();
        }
        let root_twist = Motion6(nalgebra::SVector::<f64, 6>::new(
            0.12, -0.08, 0.05, 0.09, -0.04, 0.07,
        ));
        let generalized_dof = model.dof + 6;
        let acceleration = DVector::from_iterator(
            generalized_dof,
            (0..generalized_dof).map(|index| 0.2 * (index as f64 * 0.21).sin()),
        );
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let mut model_cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut model_cache).unwrap();
        let mut dynamics_cache = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&model_cache, &mut dynamics_cache, &mut mass)
            .unwrap();
        assert!((&mass - mass.transpose()).norm() < 1e-11);
        assert!(
            mass.clone()
                .symmetric_eigen()
                .eigenvalues
                .iter()
                .all(|value| *value > 0.0)
        );
        let mut bias = DVector::zeros(generalized_dof);
        model
            .floating_bias_forces_into(
                &state,
                root_twist,
                gravity,
                &model_cache,
                &mut dynamics_cache,
                &mut bias,
            )
            .unwrap();
        let mut inverse = DVector::zeros(generalized_dof);
        model
            .floating_inverse_dynamics_into(
                &state,
                root_twist,
                &acceleration,
                gravity,
                &model_cache,
                &mut dynamics_cache,
                &mut inverse,
            )
            .unwrap();
        assert!((inverse - (mass * acceleration + bias.clone())).norm() < 1e-9);

        let mut centroidal = DMatrix::zeros(6, generalized_dof);
        model
            .floating_centroidal_map_into(&model_cache, &mut dynamics_cache, &mut centroidal)
            .unwrap();
        let mut floating_com = DMatrix::zeros(3, generalized_dof);
        model
            .floating_com_jacobian_into(&model_cache, &mut dynamics_cache, &mut floating_com)
            .unwrap();
        for row in 0..3 {
            for column in 0..generalized_dof {
                assert!(
                    (model_cache.total_mass * floating_com[(row, column)]
                        - centroidal[(row + 3, column)])
                        .abs()
                        < 1e-10
                );
            }
        }
        model
            .floating_bias_forces_into(
                &state,
                root_twist,
                gravity,
                &model_cache,
                &mut dynamics_cache,
                &mut bias,
            )
            .unwrap();
        let com_bias = model
            .center_of_mass_bias_acceleration_world(&model_cache, &dynamics_cache)
            .unwrap();
        assert!(com_bias.iter().all(|value| value.is_finite()));
        for axis in 0..3 {
            for row in 0..3 {
                assert!(centroidal[(row, axis + 3)].abs() < 1e-10);
                let expected = if row == axis {
                    model_cache.total_mass
                } else {
                    0.0
                };
                assert!((centroidal[(row + 3, axis + 3)] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn floating_integration_advances_se3_and_joint_tangent_without_allocation() {
        let model = two_link();
        let mut state = FloatingRobotState::zeros(&model);
        state.root_twist_world.0[2] = 0.4;
        state.root_twist_world.0[3] = 1.0;
        state.robot.v[0] = 0.2;
        let mut acceleration = DVector::zeros(model.dof + 6);
        acceleration[2] = 0.2;
        acceleration[5] = -2.0;
        acceleration[6] = 0.6;

        model
            .integrate_floating(&mut state, &acceleration, 0.5)
            .unwrap();

        let expected_yaw = 0.4 * 0.5 + 0.5 * 0.2 * 0.5_f64.powi(2);
        assert!(
            (state.robot.control_world_from_root.rotation.scaled_axis().z - expected_yaw).abs()
                < 1e-12
        );
        assert!((state.robot.control_world_from_root.translation.vector.x - 0.5).abs() < 1e-12);
        assert!((state.robot.control_world_from_root.translation.vector.z + 0.25).abs() < 1e-12);
        assert!((state.root_twist_world.0[2] - 0.5).abs() < 1e-12);
        assert!((state.root_twist_world.0[5] + 1.0).abs() < 1e-12);
        assert!((state.robot.q[0] - 0.175).abs() < 1e-12);
        assert!((state.robot.v[0] - 0.5).abs() < 1e-12);

        let q_capacity = state.robot.q.data.as_vec().capacity();
        let v_capacity = state.robot.v.data.as_vec().capacity();
        model
            .integrate_floating(&mut state, &acceleration, 0.01)
            .unwrap();
        assert_eq!(state.robot.q.data.as_vec().capacity(), q_capacity);
        assert_eq!(state.robot.v.data.as_vec().capacity(), v_capacity);
    }

    #[test]
    fn integration_projects_outward_velocity_at_joint_limits() {
        let mut model = two_link();
        model.joints[0].limit.lower = -0.25;
        model.joints[0].limit.upper = 0.25;
        let mut state = FloatingRobotState::zeros(&model);
        state.robot.q[0] = 0.24;
        state.robot.v[0] = 1.0;
        let mut acceleration = DVector::zeros(model.dof + 6);
        acceleration[6] = 2.0;
        model
            .integrate_floating(&mut state, &acceleration, 0.1)
            .unwrap();
        assert_eq!(state.robot.q[0], 0.25);
        assert_eq!(state.robot.v[0], 0.0);

        state.robot.q[0] = -0.24;
        state.robot.v[0] = -1.0;
        acceleration[6] = -2.0;
        model
            .integrate_floating(&mut state, &acceleration, 0.1)
            .unwrap();
        assert_eq!(state.robot.q[0], -0.25);
        assert_eq!(state.robot.v[0], 0.0);

        state.robot.q[0] = 0.25;
        state.robot.v[0] = -0.1;
        acceleration[6] = 0.0;
        model
            .integrate_floating(&mut state, &acceleration, 0.1)
            .unwrap();
        assert!(state.robot.q[0] < 0.25);
        assert_eq!(state.robot.v[0], -0.1);
    }
}
