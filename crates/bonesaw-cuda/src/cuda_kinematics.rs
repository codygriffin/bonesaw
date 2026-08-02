//! Packed immutable model ABI for the CUDA FK/CoM stage.

use bonesaw_core::{JointKind, MotionProgram};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{BatchLayout, POSE_COMPONENTS};

pub const CUDA_FK_COM_SOURCE: &str = include_str!("kernels/fk_com_v1.cu");
pub const CUDA_JACOBIANS_SOURCE: &str = include_str!("kernels/jacobians_v1.cu");
pub const CUDA_DYNAMICS_SOURCE: &str = include_str!("kernels/dynamics_v1.cu");
pub const CUDA_POINT_QUERIES_SOURCE: &str = include_str!("kernels/point_queries_v1.cu");
pub const CUDA_EMISSION_SOURCE: &str = include_str!("kernels/emission_v1.cu");
pub const CUDA_SOLVE_SOURCE: &str = include_str!("kernels/solve_v1.cu");

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum CudaJointKind {
    Fixed = 0,
    Revolute = 1,
    Prismatic = 2,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum CudaKinematicsModelError {
    #[error("program does not match the CUDA batch layout")]
    ProgramMismatch,
    #[error("CUDA kinematics dimension exceeds the u32 ABI")]
    DimensionOverflow,
    #[error("model contains a value outside finite f32 range")]
    NonFiniteModel,
    #[error("joint {joint} parent body {parent} has not been produced in compiled order")]
    NonTopologicalJointOrder { joint: usize, parent: usize },
}

/// Fixed model constants uploaded once by a CUDA kinematics executor.
///
/// Joint records use AoS for the small fixed transform and axis because every
/// thread in a warp reads the same joint record together. Runtime state and
/// outputs remain agent-minor SoA.
#[derive(Clone, Debug)]
pub struct CudaKinematicsModel {
    program_hash: [u8; 32],
    root_body: u32,
    joint_parent: Vec<u32>,
    joint_child: Vec<u32>,
    joint_coordinate: Vec<u32>,
    joint_kind: Vec<u32>,
    joint_parent_from_joint: Vec<f32>,
    joint_axis: Vec<f32>,
    body_parent_joint: Vec<u32>,
    body_mass: Vec<f32>,
    body_com: Vec<f32>,
    body_inertia: Vec<f32>,
    hash: [u8; 32],
}

impl CudaKinematicsModel {
    pub fn compile(
        program: &MotionProgram,
        layout: &BatchLayout,
    ) -> Result<Self, CudaKinematicsModelError> {
        if layout.program_fingerprint != program.header.fingerprint_sha256
            || layout.coordinate_count != program.model.dof
            || layout.body_count != program.model.bodies.len()
            || layout.joint_count != program.model.joints.len()
        {
            return Err(CudaKinematicsModelError::ProgramMismatch);
        }
        let root_body = u32::try_from(program.model.root.0)
            .map_err(|_| CudaKinematicsModelError::DimensionOverflow)?;
        let mut produced = vec![false; program.model.bodies.len()];
        produced[program.model.root.0] = true;
        let mut joint_parent = Vec::with_capacity(program.model.joints.len());
        let mut joint_child = Vec::with_capacity(program.model.joints.len());
        let mut joint_coordinate = Vec::with_capacity(program.model.joints.len());
        let mut joint_kind = Vec::with_capacity(program.model.joints.len());
        let mut joint_parent_from_joint =
            Vec::with_capacity(program.model.joints.len() * POSE_COMPONENTS);
        let mut joint_axis = Vec::with_capacity(program.model.joints.len() * 3);
        for (index, joint) in program.model.joints.iter().enumerate() {
            if !produced[joint.parent.0] {
                return Err(CudaKinematicsModelError::NonTopologicalJointOrder {
                    joint: index,
                    parent: joint.parent.0,
                });
            }
            let kind = match joint.kind {
                JointKind::Fixed => CudaJointKind::Fixed,
                JointKind::Revolute | JointKind::Continuous => CudaJointKind::Revolute,
                JointKind::Prismatic => CudaJointKind::Prismatic,
            };
            let mut axis = [
                finite_f32(joint.axis_in_joint.x)?,
                finite_f32(joint.axis_in_joint.y)?,
                finite_f32(joint.axis_in_joint.z)?,
            ];
            if kind == CudaJointKind::Revolute {
                let norm = axis[0]
                    .mul_add(axis[0], axis[1].mul_add(axis[1], axis[2] * axis[2]))
                    .sqrt();
                if !norm.is_finite() || norm <= f32::MIN_POSITIVE {
                    return Err(CudaKinematicsModelError::NonFiniteModel);
                }
                for value in &mut axis {
                    *value /= norm;
                }
            }
            joint_parent.push(
                u32::try_from(joint.parent.0)
                    .map_err(|_| CudaKinematicsModelError::DimensionOverflow)?,
            );
            joint_child.push(
                u32::try_from(joint.child.0)
                    .map_err(|_| CudaKinematicsModelError::DimensionOverflow)?,
            );
            joint_coordinate.push(
                joint
                    .coordinate
                    .map(u32::try_from)
                    .transpose()
                    .map_err(|_| CudaKinematicsModelError::DimensionOverflow)?
                    .unwrap_or(u32::MAX),
            );
            joint_kind.push(kind as u32);
            let rotation = joint.parent_from_joint.rotation.to_rotation_matrix();
            for row in 0..3 {
                for column in 0..3 {
                    joint_parent_from_joint.push(finite_f32(rotation[(row, column)])?);
                }
            }
            for component in 0..3 {
                joint_parent_from_joint.push(finite_f32(
                    joint.parent_from_joint.translation.vector[component],
                )?);
            }
            joint_axis.extend(axis);
            produced[joint.child.0] = true;
        }
        if produced.iter().any(|value| !value) {
            return Err(CudaKinematicsModelError::NonTopologicalJointOrder {
                joint: program.model.joints.len(),
                parent: produced.iter().position(|value| !value).unwrap_or(0),
            });
        }
        let mut body_mass = Vec::with_capacity(program.model.bodies.len());
        let mut body_com = Vec::with_capacity(program.model.bodies.len() * 3);
        let mut body_inertia = Vec::with_capacity(program.model.bodies.len() * 9);
        let mut body_parent_joint = vec![u32::MAX; program.model.bodies.len()];
        for (joint_index, joint) in program.model.joints.iter().enumerate() {
            body_parent_joint[joint.child.0] = u32::try_from(joint_index)
                .map_err(|_| CudaKinematicsModelError::DimensionOverflow)?;
        }
        for body in &program.model.bodies {
            body_mass.push(finite_f32(body.mass)?);
            body_com.extend([
                finite_f32(body.com_in_body.x)?,
                finite_f32(body.com_in_body.y)?,
                finite_f32(body.com_in_body.z)?,
            ]);
            for row in 0..3 {
                for column in 0..3 {
                    body_inertia.push(finite_f32(body.inertia_about_com_in_body[(row, column)])?);
                }
            }
        }
        let mut model = Self {
            program_hash: program.header.fingerprint_sha256,
            root_body,
            joint_parent,
            joint_child,
            joint_coordinate,
            joint_kind,
            joint_parent_from_joint,
            joint_axis,
            body_parent_joint,
            body_mass,
            body_com,
            body_inertia,
            hash: [0; 32],
        };
        model.hash = model.compute_hash();
        Ok(model)
    }

    pub fn program_hash(&self) -> [u8; 32] {
        self.program_hash
    }
    pub fn root_body(&self) -> u32 {
        self.root_body
    }
    pub fn joint_parent(&self) -> &[u32] {
        &self.joint_parent
    }
    pub fn joint_child(&self) -> &[u32] {
        &self.joint_child
    }
    pub fn joint_coordinate(&self) -> &[u32] {
        &self.joint_coordinate
    }
    pub fn joint_kind(&self) -> &[u32] {
        &self.joint_kind
    }
    pub fn joint_parent_from_joint(&self) -> &[f32] {
        &self.joint_parent_from_joint
    }
    pub fn joint_axis(&self) -> &[f32] {
        &self.joint_axis
    }
    pub fn body_parent_joint(&self) -> &[u32] {
        &self.body_parent_joint
    }
    pub fn body_mass(&self) -> &[f32] {
        &self.body_mass
    }
    pub fn body_com(&self) -> &[f32] {
        &self.body_com
    }
    pub fn body_inertia(&self) -> &[f32] {
        &self.body_inertia
    }
    pub fn hash(&self) -> [u8; 32] {
        self.hash
    }

    fn compute_hash(&self) -> [u8; 32] {
        let mut digest = Sha256::new();
        digest.update(b"bonesaw-cuda-kinematics-model-v2");
        digest.update(self.program_hash);
        digest.update(self.root_body.to_le_bytes());
        for values in [
            &self.joint_parent,
            &self.joint_child,
            &self.joint_coordinate,
            &self.body_parent_joint,
        ] {
            digest.update((values.len() as u64).to_le_bytes());
            for value in values {
                digest.update(value.to_le_bytes());
            }
        }
        for values in [
            &self.joint_parent_from_joint,
            &self.joint_axis,
            &self.body_mass,
            &self.body_com,
            &self.body_inertia,
        ] {
            digest.update((values.len() as u64).to_le_bytes());
            for value in values {
                digest.update(value.to_bits().to_le_bytes());
            }
        }
        digest.finalize().into()
    }
}

pub fn cuda_fk_com_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_FK_COM_SOURCE.as_bytes()).into()
}

pub fn cuda_jacobians_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_JACOBIANS_SOURCE.as_bytes()).into()
}

pub fn cuda_dynamics_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_DYNAMICS_SOURCE.as_bytes()).into()
}

pub fn cuda_point_queries_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_POINT_QUERIES_SOURCE.as_bytes()).into()
}

pub fn cuda_emission_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_EMISSION_SOURCE.as_bytes()).into()
}

pub fn cuda_solve_source_sha256() -> [u8; 32] {
    Sha256::digest(CUDA_SOLVE_SOURCE.as_bytes()).into()
}

fn finite_f32(value: f64) -> Result<f32, CudaKinematicsModelError> {
    let value = value as f32;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(CudaKinematicsModelError::NonFiniteModel)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bonesaw_core::TimingSpec;

    #[test]
    fn toy_model_pack_is_stable_topological_and_f32_finite() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let layout = BatchLayout::compile(&program, 17, 32).unwrap();
        let first = CudaKinematicsModel::compile(&program, &layout).unwrap();
        let second = CudaKinematicsModel::compile(&program, &layout).unwrap();
        assert_eq!(first.hash(), second.hash());
        assert_eq!(first.joint_parent().len(), program.model.joints.len());
        assert_eq!(
            first.joint_parent_from_joint().len(),
            12 * program.model.joints.len()
        );
        assert_eq!(first.joint_axis().len(), 3 * program.model.joints.len());
        assert_eq!(first.body_parent_joint().len(), program.model.bodies.len());
        assert_eq!(first.body_mass().len(), program.model.bodies.len());
        assert_eq!(first.body_com().len(), 3 * program.model.bodies.len());
        assert_eq!(first.body_inertia().len(), 9 * program.model.bodies.len());
        assert!(
            first
                .joint_parent_from_joint()
                .iter()
                .chain(first.joint_axis())
                .chain(first.body_mass())
                .chain(first.body_com())
                .chain(first.body_inertia())
                .all(|value| value.is_finite())
        );
    }

    #[test]
    fn cuda_source_has_fixed_entry_no_allocation_and_no_atomics() {
        assert!(CUDA_FK_COM_SOURCE.contains("bonesaw_fk_com_v1"));
        assert!(!CUDA_FK_COM_SOURCE.contains("malloc"));
        assert!(!CUDA_FK_COM_SOURCE.contains("atomic"));
        assert_eq!(cuda_fk_com_source_sha256().len(), 32);
        assert!(CUDA_JACOBIANS_SOURCE.contains("bonesaw_jacobians_v1"));
        assert!(!CUDA_JACOBIANS_SOURCE.contains("malloc"));
        assert!(!CUDA_JACOBIANS_SOURCE.contains("atomic"));
        assert_eq!(cuda_jacobians_source_sha256().len(), 32);
        assert!(CUDA_DYNAMICS_SOURCE.contains("bonesaw_dynamics_v1"));
        assert!(!CUDA_DYNAMICS_SOURCE.contains("malloc"));
        assert!(!CUDA_DYNAMICS_SOURCE.contains("atomic"));
        assert_eq!(cuda_dynamics_source_sha256().len(), 32);
        assert!(CUDA_POINT_QUERIES_SOURCE.contains("bonesaw_point_queries_v1"));
        assert!(!CUDA_POINT_QUERIES_SOURCE.contains("malloc"));
        assert!(!CUDA_POINT_QUERIES_SOURCE.contains("atomic"));
        assert_eq!(cuda_point_queries_source_sha256().len(), 32);
        assert!(CUDA_EMISSION_SOURCE.contains("bonesaw_emission_v1"));
        assert!(!CUDA_EMISSION_SOURCE.contains("malloc"));
        assert!(!CUDA_EMISSION_SOURCE.contains("atomic"));
        assert_eq!(cuda_emission_source_sha256().len(), 32);
        assert!(CUDA_SOLVE_SOURCE.contains("bonesaw_solve_v1"));
        assert!(CUDA_SOLVE_SOURCE.contains("HARD_SWEEPS = 64"));
        assert!(CUDA_SOLVE_SOURCE.contains("TASK_SWEEPS = 32"));
        assert!(!CUDA_SOLVE_SOURCE.contains("malloc"));
        assert!(!CUDA_SOLVE_SOURCE.contains("atomic"));
        assert!(!CUDA_SOLVE_SOURCE.contains("__syncthreads"));
        assert_eq!(cuda_solve_source_sha256().len(), 32);
    }
}
