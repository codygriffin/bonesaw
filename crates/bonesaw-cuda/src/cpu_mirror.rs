use bonesaw_core::{JointKind, MotionProgram, Priority, TaskOp};
use nalgebra::{SMatrix, Vector3};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::layouts::TaskVectorStorage;
use crate::{
    AgentStatus, BackendFingerprint, BackendProfile, BatchLayout, BatchLayoutError,
    BatchProgramDescriptor, ContactKinematicMode, ContactLockDescriptor, DynamicsBatchInput,
    DynamicsBatchOutput, EmissionBatchInput, EmissionBatchOutput, FkBatchInput, FkBatchOutput,
    JacobianBatchOutput, KERNEL_ABI_VERSION, KernelManifest, MathFlags, MirrorAlgorithm,
    POSE_COMPONENTS, PointQueryBatchOutput, PointQueryDescriptor, PointTaskDescriptor,
    ROOT_TANGENT_COMPONENTS, ScalarFormat,
};

#[derive(Debug, Error)]
pub enum BatchExecutionError {
    #[error(transparent)]
    Layout(#[from] BatchLayoutError),
    #[error("input or output layout does not match the executor descriptor")]
    LayoutMismatch,
    #[error("compiled model contains a value outside finite f32 range")]
    NonFiniteModel,
    #[error("failed to fingerprint the batch kernel descriptor")]
    Fingerprint(#[from] serde_json::Error),
    #[error(
        "point query {stable_id} refers to body frame {frame_index}, outside {body_count} bodies"
    )]
    PointFrame {
        stable_id: u32,
        frame_index: usize,
        body_count: usize,
    },
    #[error("point query stable ID {0} is duplicated")]
    DuplicatePointStableId(u32),
    #[error("emission row stable ID {0} is duplicated")]
    DuplicateEmissionStableId(u32),
    #[error(
        "emission row {stable_id} refers to missing point query stable ID {point_query_stable_id}"
    )]
    MissingPointQuery {
        stable_id: u32,
        point_query_stable_id: u32,
    },
    #[error("point task {0} has an invalid weight or bandwidth")]
    InvalidPointTask(u32),
    #[error("contact kinematic-mode count does not match the contact declaration")]
    ContactModeCount,
    #[error("rigid patch {stable_id} must contain 3..=16 contiguous contact slots")]
    InvalidRigidPatchSize { stable_id: u32 },
    #[error("rigid patch {stable_id} contact range exceeds the contact declaration")]
    RigidPatchRange { stable_id: u32 },
    #[error("rigid patch {stable_id} overlaps another rigid patch at contact slot {slot}")]
    RigidPatchOverlap { stable_id: u32, slot: usize },
    #[error("rigid patch {stable_id} contact {contact_stable_id} has no point query")]
    RigidPatchMissingPoint {
        stable_id: u32,
        contact_stable_id: u32,
    },
    #[error("rigid patch {stable_id} spans multiple body frames")]
    RigidPatchMultipleFrames { stable_id: u32 },
    #[error("rigid patch {stable_id} point geometry is non-finite or rank deficient")]
    RigidPatchRankDeficient { stable_id: u32 },
}

/// Authoring-time fixed point slot for a batch executor. This is consumed only
/// at executor construction; runtime agents cannot change query topology.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointQuerySpec {
    pub stable_id: u32,
    pub frame_index: usize,
    pub point_in_frame: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointAttractorSpec {
    pub stable_id: u32,
    pub point_query_stable_id: u32,
    pub priority: Priority,
    pub weight: f64,
    pub bandwidth_hz: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContactLockSpec {
    pub stable_id: u32,
    pub point_query_stable_id: u32,
}

/// Construction-time request for a compiler-selected six-row rigid-body basis.
///
/// Points must be expressed in one body frame. Canonical frame X/Y/Z are the
/// tangent-X/tangent-Y/normal axes supplied to the runtime contact input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidPatchBasisSpec {
    pub stable_id: u32,
    pub first_contact_slot: usize,
    pub contact_count: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidPatchBasisDiagnostic {
    pub stable_id: u32,
    pub row_count: usize,
    pub rank: usize,
    pub minimum_singular_value: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RigidPatchBasisPlan {
    pub contact_modes: Vec<ContactKinematicMode>,
    pub patches: Vec<RigidPatchBasisDiagnostic>,
}

/// Derive a deterministic, best-conditioned six-row rigid-patch basis from
/// fixed point geometry. Construction may allocate; execution topology stays
/// fixed and the selected modes enter the ordinary kernel fingerprint.
pub fn derive_rigid_patch_contact_modes(
    point_queries: &[PointQuerySpec],
    contact_locks: &[ContactLockSpec],
    patches: &[RigidPatchBasisSpec],
) -> Result<RigidPatchBasisPlan, BatchExecutionError> {
    let mut contact_modes = vec![ContactKinematicMode::LockedPoint; contact_locks.len()];
    let mut claimed = vec![false; contact_locks.len()];
    let mut diagnostics = Vec::with_capacity(patches.len());
    for patch in patches {
        if !(3..=16).contains(&patch.contact_count) {
            return Err(BatchExecutionError::InvalidRigidPatchSize {
                stable_id: patch.stable_id,
            });
        }
        let end = patch
            .first_contact_slot
            .checked_add(patch.contact_count)
            .filter(|end| *end <= contact_locks.len())
            .ok_or(BatchExecutionError::RigidPatchRange {
                stable_id: patch.stable_id,
            })?;
        for slot in patch.first_contact_slot..end {
            if claimed[slot] {
                return Err(BatchExecutionError::RigidPatchOverlap {
                    stable_id: patch.stable_id,
                    slot,
                });
            }
            claimed[slot] = true;
        }
        let mut frame_index = None;
        let mut points = Vec::with_capacity(patch.contact_count);
        for contact in &contact_locks[patch.first_contact_slot..end] {
            let query = point_queries
                .iter()
                .find(|query| query.stable_id == contact.point_query_stable_id)
                .ok_or(BatchExecutionError::RigidPatchMissingPoint {
                    stable_id: patch.stable_id,
                    contact_stable_id: contact.stable_id,
                })?;
            if let Some(expected) = frame_index {
                if query.frame_index != expected {
                    return Err(BatchExecutionError::RigidPatchMultipleFrames {
                        stable_id: patch.stable_id,
                    });
                }
            } else {
                frame_index = Some(query.frame_index);
            }
            let point = Vector3::from(query.point_in_frame);
            if !point.iter().all(|value| value.is_finite()) {
                return Err(BatchExecutionError::RigidPatchRankDeficient {
                    stable_id: patch.stable_id,
                });
            }
            points.push(point);
        }
        let (selected, minimum_singular_value) = select_rigid_patch_modes(&points).ok_or(
            BatchExecutionError::RigidPatchRankDeficient {
                stable_id: patch.stable_id,
            },
        )?;
        contact_modes[patch.first_contact_slot..end].copy_from_slice(&selected);
        diagnostics.push(RigidPatchBasisDiagnostic {
            stable_id: patch.stable_id,
            row_count: 6,
            rank: 6,
            minimum_singular_value,
        });
    }
    Ok(RigidPatchBasisPlan {
        contact_modes,
        patches: diagnostics,
    })
}

fn select_rigid_patch_modes(points: &[Vector3<f64>]) -> Option<(Vec<ContactKinematicMode>, f64)> {
    const MODES: [ContactKinematicMode; 4] = [
        ContactKinematicMode::Disabled,
        ContactKinematicMode::NormalPoint,
        ContactKinematicMode::RollingPoint,
        ContactKinematicMode::LockedPoint,
    ];
    fn recurse(
        points: &[Vector3<f64>],
        index: usize,
        rows_left: usize,
        current: &mut [ContactKinematicMode],
        best: &mut Option<(Vec<ContactKinematicMode>, f64)>,
    ) {
        if index == points.len() {
            if rows_left != 0 {
                return;
            }
            let origin = points[0];
            let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
            let mut matrix = SMatrix::<f64, 6, 6>::zeros();
            let mut row = 0;
            for (point, mode) in points.iter().zip(current.iter().copied()) {
                let relative = point - origin;
                for (component, axis) in axes.iter().enumerate() {
                    if !mode.axis_enabled(component) {
                        continue;
                    }
                    let angular = relative.cross(axis);
                    matrix[(row, 0)] = angular.x;
                    matrix[(row, 1)] = angular.y;
                    matrix[(row, 2)] = angular.z;
                    matrix[(row, 3)] = axis.x;
                    matrix[(row, 4)] = axis.y;
                    matrix[(row, 5)] = axis.z;
                    row += 1;
                }
            }
            let singular_values = matrix.svd(false, false).singular_values;
            let maximum = singular_values.iter().copied().fold(0.0_f64, f64::max);
            let minimum = singular_values
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            if !minimum.is_finite() || minimum <= 1.0e-10 * maximum.max(1.0) {
                return;
            }
            if best
                .as_ref()
                .is_none_or(|(_, best_minimum)| minimum > *best_minimum + 1.0e-14)
            {
                *best = Some((current.to_vec(), minimum));
            }
            return;
        }
        let remaining_points = points.len() - index - 1;
        for mode in MODES {
            let rows = (0..3)
                .filter(|component| mode.axis_enabled(*component))
                .count();
            if rows > rows_left || rows_left - rows > 3 * remaining_points {
                continue;
            }
            current[index] = mode;
            recurse(points, index + 1, rows_left - rows, current, best);
        }
    }

    let mut current = vec![ContactKinematicMode::Disabled; points.len()];
    let mut best = None;
    recurse(points, 0, 6, &mut current, &mut best);
    best
}

#[derive(Clone, Copy, Debug)]
enum MirrorJointKind {
    Fixed,
    Revolute,
    Prismatic,
}

#[derive(Clone, Copy, Debug)]
struct MirrorJoint {
    parent: usize,
    child: usize,
    coordinate: Option<usize>,
    kind: MirrorJointKind,
    parent_from_joint: Pose,
    axis: [f32; 3],
}

#[derive(Clone, Copy, Debug)]
struct MirrorBody {
    mass: f32,
    com_in_body: [f32; 3],
    inertia_about_com_in_body: [f32; 9],
}

#[derive(Clone, Copy, Debug)]
struct Pose([f32; POSE_COMPONENTS]);

impl Pose {
    const IDENTITY: Self = Self([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

    fn rotation(self, row: usize, column: usize) -> f32 {
        self.0[row * 3 + column]
    }

    fn translation(self, component: usize) -> f32 {
        self.0[9 + component]
    }

    fn transform_point(self, point: [f32; 3]) -> [f32; 3] {
        let mut output = [0.0; 3];
        for (row, value) in output.iter_mut().enumerate() {
            let mut sum = self.rotation(row, 0) * point[0];
            sum = self.rotation(row, 1).mul_add(point[1], sum);
            sum = self.rotation(row, 2).mul_add(point[2], sum);
            *value = sum + self.translation(row);
        }
        output
    }

    fn transform_vector(self, vector: [f32; 3]) -> [f32; 3] {
        let mut output = [0.0; 3];
        for (row, value) in output.iter_mut().enumerate() {
            let mut sum = self.rotation(row, 0) * vector[0];
            sum = self.rotation(row, 1).mul_add(vector[1], sum);
            sum = self.rotation(row, 2).mul_add(vector[2], sum);
            *value = sum;
        }
        output
    }
}

/// Precompiled `CpuMirrorF32` executor for the first CUDA-mirror stages.
///
/// Construction may allocate and fingerprint. `execute_into` uses only fixed
/// loops, caller-owned batch buffers, and stack-local 3×3 transforms.
#[derive(Clone, Debug)]
pub struct CpuMirrorExecutor {
    descriptor: BatchProgramDescriptor,
    fingerprint: BackendFingerprint,
    root_body: usize,
    joints: Vec<MirrorJoint>,
    bodies: Vec<MirrorBody>,
    parent_joint_by_body: Vec<Option<usize>>,
}

impl CpuMirrorExecutor {
    pub fn compile(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
    ) -> Result<Self, BatchExecutionError> {
        let point_queries = program
            .tasks
            .ops()
            .iter()
            .filter_map(|operation| match *operation {
                TaskOp::Point {
                    stable_id,
                    frame,
                    point_in_frame,
                    ..
                } => Some(PointQuerySpec {
                    stable_id,
                    frame_index: frame.0,
                    point_in_frame: [point_in_frame.x, point_in_frame.y, point_in_frame.z],
                }),
                _ => None,
            })
            .collect::<Vec<_>>();
        let point_tasks = program
            .tasks
            .ops()
            .iter()
            .filter_map(|operation| match *operation {
                TaskOp::Point {
                    stable_id,
                    priority,
                    weight,
                    bandwidth_hz,
                    ..
                } => Some(PointAttractorSpec {
                    stable_id,
                    point_query_stable_id: stable_id,
                    priority,
                    weight,
                    bandwidth_hz,
                }),
                _ => None,
            })
            .collect::<Vec<_>>();
        Self::compile_with_emission_plan(
            program,
            agent_capacity,
            agent_alignment,
            &point_queries,
            &point_tasks,
            &[],
        )
    }

    pub fn compile_with_point_queries(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_queries: &[PointQuerySpec],
    ) -> Result<Self, BatchExecutionError> {
        Self::compile_with_emission_plan(
            program,
            agent_capacity,
            agent_alignment,
            point_queries,
            &[],
            &[],
        )
    }

    pub fn compile_with_emission_plan(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_queries: &[PointQuerySpec],
        point_tasks: &[PointAttractorSpec],
        contact_locks: &[ContactLockSpec],
    ) -> Result<Self, BatchExecutionError> {
        let modes = vec![ContactKinematicMode::LockedPoint; contact_locks.len()];
        Self::compile_with_contact_modes(
            program,
            agent_capacity,
            agent_alignment,
            point_queries,
            point_tasks,
            contact_locks,
            &modes,
        )
    }

    pub fn compile_with_contact_modes(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_queries: &[PointQuerySpec],
        point_tasks: &[PointAttractorSpec],
        contact_locks: &[ContactLockSpec],
        contact_kinematic_modes: &[ContactKinematicMode],
    ) -> Result<Self, BatchExecutionError> {
        if contact_locks.len() != contact_kinematic_modes.len() {
            return Err(BatchExecutionError::ContactModeCount);
        }
        let point_queries = point_queries
            .iter()
            .copied()
            .enumerate()
            .map(|(index, query)| {
                if query.frame_index >= program.model.bodies.len() {
                    return Err(BatchExecutionError::PointFrame {
                        stable_id: query.stable_id,
                        frame_index: query.frame_index,
                        body_count: program.model.bodies.len(),
                    });
                }
                if point_queries[..index]
                    .iter()
                    .any(|prior| prior.stable_id == query.stable_id)
                {
                    return Err(BatchExecutionError::DuplicatePointStableId(query.stable_id));
                }
                Ok(PointQueryDescriptor {
                    stable_id: query.stable_id,
                    frame_index: query.frame_index,
                    point_in_frame_bits: [
                        finite_f32(query.point_in_frame[0])?.to_bits(),
                        finite_f32(query.point_in_frame[1])?.to_bits(),
                        finite_f32(query.point_in_frame[2])?.to_bits(),
                    ],
                })
            })
            .collect::<Result<Vec<_>, BatchExecutionError>>()?;
        let point_tasks = point_tasks
            .iter()
            .copied()
            .enumerate()
            .map(|(index, task)| {
                if point_tasks[..index]
                    .iter()
                    .any(|prior| prior.stable_id == task.stable_id)
                    || contact_locks
                        .iter()
                        .any(|contact| contact.stable_id == task.stable_id)
                {
                    return Err(BatchExecutionError::DuplicateEmissionStableId(
                        task.stable_id,
                    ));
                }
                let point_query_slot = point_queries
                    .iter()
                    .position(|query| query.stable_id == task.point_query_stable_id)
                    .ok_or(BatchExecutionError::MissingPointQuery {
                        stable_id: task.stable_id,
                        point_query_stable_id: task.point_query_stable_id,
                    })?;
                let weight = finite_f32(task.weight)?;
                let bandwidth_hz = finite_f32(task.bandwidth_hz)?;
                if weight <= 0.0 || bandwidth_hz <= 0.0 {
                    return Err(BatchExecutionError::InvalidPointTask(task.stable_id));
                }
                Ok(PointTaskDescriptor {
                    stable_id: task.stable_id,
                    point_query_slot,
                    priority: task.priority,
                    weight_bits: weight.to_bits(),
                    bandwidth_hz_bits: bandwidth_hz.to_bits(),
                })
            })
            .collect::<Result<Vec<_>, BatchExecutionError>>()?;
        let contact_locks = contact_locks
            .iter()
            .copied()
            .enumerate()
            .map(|(index, contact)| {
                if contact_locks[..index]
                    .iter()
                    .any(|prior| prior.stable_id == contact.stable_id)
                {
                    return Err(BatchExecutionError::DuplicateEmissionStableId(
                        contact.stable_id,
                    ));
                }
                let point_query_slot = point_queries
                    .iter()
                    .position(|query| query.stable_id == contact.point_query_stable_id)
                    .ok_or(BatchExecutionError::MissingPointQuery {
                        stable_id: contact.stable_id,
                        point_query_stable_id: contact.point_query_stable_id,
                    })?;
                Ok(ContactLockDescriptor {
                    stable_id: contact.stable_id,
                    point_query_slot,
                    kinematic_mode: contact_kinematic_modes[index],
                })
            })
            .collect::<Result<Vec<_>, BatchExecutionError>>()?;
        let layout = BatchLayout::compile_with_plan_counts(
            program,
            agent_capacity,
            agent_alignment,
            point_queries.len(),
            point_tasks.len(),
            contact_locks.len(),
        )?;
        let descriptor = BatchProgramDescriptor {
            kernel_abi_version: KERNEL_ABI_VERSION,
            algorithm: MirrorAlgorithm::FixedTreeF32V1,
            layout,
            manifest: KernelManifest::emission_v1(),
            math_flags: MathFlags::MIRROR_F32,
            point_queries,
            point_tasks,
            contact_locks,
        };
        let joints = program
            .model
            .joints
            .iter()
            .map(|joint| {
                let mut axis = [
                    finite_f32(joint.axis_in_joint.x)?,
                    finite_f32(joint.axis_in_joint.y)?,
                    finite_f32(joint.axis_in_joint.z)?,
                ];
                let kind = match joint.kind {
                    JointKind::Fixed => MirrorJointKind::Fixed,
                    JointKind::Revolute | JointKind::Continuous => {
                        let norm = axis[0]
                            .mul_add(axis[0], axis[1].mul_add(axis[1], axis[2] * axis[2]))
                            .sqrt();
                        if !norm.is_finite() || norm <= f32::MIN_POSITIVE {
                            return Err(BatchExecutionError::NonFiniteModel);
                        }
                        for value in &mut axis {
                            *value /= norm;
                        }
                        MirrorJointKind::Revolute
                    }
                    JointKind::Prismatic => MirrorJointKind::Prismatic,
                };
                Ok(MirrorJoint {
                    parent: joint.parent.0,
                    child: joint.child.0,
                    coordinate: joint.coordinate,
                    kind,
                    parent_from_joint: pose_from_core(joint.parent_from_joint)?,
                    axis,
                })
            })
            .collect::<Result<Vec<_>, BatchExecutionError>>()?;
        let bodies = program
            .model
            .bodies
            .iter()
            .map(|body| {
                let mut inertia_about_com_in_body = [0.0_f32; 9];
                for row in 0..3 {
                    for column in 0..3 {
                        inertia_about_com_in_body[row * 3 + column] =
                            finite_f32(body.inertia_about_com_in_body[(row, column)])?;
                    }
                }
                Ok(MirrorBody {
                    mass: finite_f32(body.mass)?,
                    com_in_body: [
                        finite_f32(body.com_in_body.x)?,
                        finite_f32(body.com_in_body.y)?,
                        finite_f32(body.com_in_body.z)?,
                    ],
                    inertia_about_com_in_body,
                })
            })
            .collect::<Result<Vec<_>, BatchExecutionError>>()?;
        let mut parent_joint_by_body = vec![None; program.model.bodies.len()];
        for (joint_index, joint) in joints.iter().enumerate() {
            parent_joint_by_body[joint.child] = Some(joint_index);
        }
        let kernel_hash: [u8; 32] = Sha256::digest(serde_json::to_vec(&descriptor)?).into();
        let core_build_hash = mirror_build_hash();
        let fingerprint = BackendFingerprint {
            program_hash: program.header.fingerprint_sha256,
            core_build_hash,
            backend_profile: BackendProfile::CpuMirrorF32,
            scalar_format: ScalarFormat::F32,
            cpu_isa: Some(cpu_isa()),
            cuda_toolkit: None,
            cuda_driver: None,
            gpu_architecture: None,
            gpu_sm_count: None,
            kernel_hash,
            math_flags: MathFlags::MIRROR_F32,
        };
        Ok(Self {
            descriptor,
            fingerprint,
            root_body: program.model.root.0,
            joints,
            bodies,
            parent_joint_by_body,
        })
    }

    pub fn compile_with_auto_rigid_patches(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_queries: &[PointQuerySpec],
        point_tasks: &[PointAttractorSpec],
        contact_locks: &[ContactLockSpec],
        patches: &[RigidPatchBasisSpec],
    ) -> Result<Self, BatchExecutionError> {
        let plan = derive_rigid_patch_contact_modes(point_queries, contact_locks, patches)?;
        Self::compile_with_contact_modes(
            program,
            agent_capacity,
            agent_alignment,
            point_queries,
            point_tasks,
            contact_locks,
            &plan.contact_modes,
        )
    }

    pub fn descriptor(&self) -> &BatchProgramDescriptor {
        &self.descriptor
    }

    pub fn fingerprint(&self) -> &BackendFingerprint {
        &self.fingerprint
    }

    pub fn execute_into(
        &self,
        input: &FkBatchInput,
        output: &mut FkBatchOutput,
    ) -> Result<(), BatchExecutionError> {
        if input.layout() != &self.descriptor.layout || output.layout() != &self.descriptor.layout {
            return Err(BatchExecutionError::LayoutMismatch);
        }
        output.clear();
        for agent in 0..input.active_agents() {
            let Some(root_pose) = load_root_pose(input, agent) else {
                output.set_status(agent, AgentStatus::InvalidInput);
                continue;
            };
            if (0..self.descriptor.layout.coordinate_count).any(|coordinate| {
                !input.q_soa()[self.descriptor.layout.q_index(coordinate, agent)].is_finite()
            }) {
                output.set_status(agent, AgentStatus::InvalidInput);
                continue;
            }
            output.set_body_pose(agent, self.root_body, &root_pose.0);
            for joint in &self.joints {
                let parent = Pose(
                    output
                        .body_pose(agent, joint.parent)
                        .expect("compiled parent body"),
                );
                let joint_pose = compose(parent, joint.parent_from_joint);
                let motion = match joint.kind {
                    MirrorJointKind::Fixed => Pose::IDENTITY,
                    MirrorJointKind::Revolute => {
                        let coordinate = joint.coordinate.expect("revolute coordinate");
                        let value =
                            input.q_soa()[self.descriptor.layout.q_index(coordinate, agent)];
                        revolute_motion(joint.axis, value)
                    }
                    MirrorJointKind::Prismatic => {
                        let coordinate = joint.coordinate.expect("prismatic coordinate");
                        let value =
                            input.q_soa()[self.descriptor.layout.q_index(coordinate, agent)];
                        prismatic_motion(joint.axis, value)
                    }
                };
                let child = compose(joint_pose, motion);
                output.set_body_pose(agent, joint.child, &child.0);
            }
            let mut weighted = [0.0_f32; 3];
            let mut total_mass = 0.0_f32;
            for (body_index, body) in self.bodies.iter().enumerate() {
                let pose = Pose(
                    output
                        .body_pose(agent, body_index)
                        .expect("compiled body pose"),
                );
                let com = pose.transform_point(body.com_in_body);
                for component in 0..3 {
                    weighted[component] = body.mass.mul_add(com[component], weighted[component]);
                }
                total_mass += body.mass;
            }
            let center_of_mass = if total_mass > 0.0 {
                [
                    weighted[0] / total_mass,
                    weighted[1] / total_mass,
                    weighted[2] / total_mass,
                ]
            } else {
                [0.0; 3]
            };
            output.set_center_of_mass(agent, center_of_mass, total_mass);
            output.set_status(agent, AgentStatus::Ok);
        }
        Ok(())
    }

    /// Evaluate floating-root spatial frame Jacobians and the CoM Jacobian
    /// from a completed FK stage without allocation.
    ///
    /// Frame components are `[angular xyz; frame-origin linear xyz]`; tangent
    /// columns are `[root angular xyz; root linear xyz; joint coordinates]`.
    pub fn execute_jacobians_into(
        &self,
        fk_output: &FkBatchOutput,
        output: &mut JacobianBatchOutput,
    ) -> Result<(), BatchExecutionError> {
        if fk_output.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
        {
            return Err(BatchExecutionError::LayoutMismatch);
        }
        output.clear();
        for agent in 0..self.descriptor.layout.agent_capacity {
            let status = fk_output.agent_status()[agent];
            output.set_status(agent, status);
            if status != AgentStatus::Ok {
                continue;
            }
            let root_pose = Pose(
                fk_output
                    .body_pose(agent, self.root_body)
                    .expect("compiled root body"),
            );
            let root_origin = [
                root_pose.translation(0),
                root_pose.translation(1),
                root_pose.translation(2),
            ];
            for body in 0..self.bodies.len() {
                let body_pose = Pose(
                    fk_output
                        .body_pose(agent, body)
                        .expect("compiled body pose"),
                );
                let body_origin = [
                    body_pose.translation(0),
                    body_pose.translation(1),
                    body_pose.translation(2),
                ];
                for axis in 0..3 {
                    output.set_frame_jacobian_value(agent, body, axis, axis, 1.0);
                    output.set_frame_jacobian_value(agent, body, 3 + axis, 3 + axis, 1.0);
                    let mut basis = [0.0_f32; 3];
                    basis[axis] = 1.0;
                    let linear = cross(basis, subtract(body_origin, root_origin));
                    for (component, value) in linear.into_iter().enumerate() {
                        output.set_frame_jacobian_value(agent, body, 3 + component, axis, value);
                    }
                }
                let mut ancestor = body;
                while let Some(joint_index) = self.parent_joint_by_body[ancestor] {
                    let joint = self.joints[joint_index];
                    if let Some(coordinate) = joint.coordinate {
                        let joint_pose = self.joint_world_pose(fk_output, agent, joint);
                        let axis_world = joint_pose.transform_vector(joint.axis);
                        let generalized_coordinate = ROOT_TANGENT_COMPONENTS + coordinate;
                        match joint.kind {
                            MirrorJointKind::Revolute => {
                                let joint_origin = [
                                    joint_pose.translation(0),
                                    joint_pose.translation(1),
                                    joint_pose.translation(2),
                                ];
                                let linear = cross(axis_world, subtract(body_origin, joint_origin));
                                for component in 0..3 {
                                    output.set_frame_jacobian_value(
                                        agent,
                                        body,
                                        component,
                                        generalized_coordinate,
                                        axis_world[component],
                                    );
                                    output.set_frame_jacobian_value(
                                        agent,
                                        body,
                                        3 + component,
                                        generalized_coordinate,
                                        linear[component],
                                    );
                                }
                            }
                            MirrorJointKind::Prismatic => {
                                for (component, value) in axis_world.into_iter().enumerate() {
                                    output.set_frame_jacobian_value(
                                        agent,
                                        body,
                                        3 + component,
                                        generalized_coordinate,
                                        value,
                                    );
                                }
                            }
                            MirrorJointKind::Fixed => {}
                        }
                    }
                    ancestor = joint.parent;
                }
            }
            let total_mass = fk_output.total_mass()[agent];
            if total_mass <= 0.0 {
                continue;
            }
            for (body, body_model) in self.bodies.iter().copied().enumerate() {
                if body_model.mass <= 0.0 {
                    continue;
                }
                let body_pose = Pose(
                    fk_output
                        .body_pose(agent, body)
                        .expect("compiled body pose"),
                );
                let point_world = body_pose.transform_point(body_model.com_in_body);
                let scale = body_model.mass / total_mass;
                for axis in 0..3 {
                    let mut basis = [0.0_f32; 3];
                    basis[axis] = 1.0;
                    let linear = cross(basis, subtract(point_world, root_origin));
                    for (component, value) in linear.into_iter().enumerate() {
                        output.multiply_add_center_of_mass_jacobian_value(
                            agent, component, axis, scale, value,
                        );
                    }
                    output.multiply_add_center_of_mass_jacobian_value(
                        agent,
                        axis,
                        3 + axis,
                        scale,
                        1.0,
                    );
                }
                let mut ancestor = body;
                while let Some(joint_index) = self.parent_joint_by_body[ancestor] {
                    let joint = self.joints[joint_index];
                    if let Some(coordinate) = joint.coordinate {
                        let joint_pose = self.joint_world_pose(fk_output, agent, joint);
                        let axis_world = joint_pose.transform_vector(joint.axis);
                        let value = match joint.kind {
                            MirrorJointKind::Revolute => {
                                let joint_origin = [
                                    joint_pose.translation(0),
                                    joint_pose.translation(1),
                                    joint_pose.translation(2),
                                ];
                                cross(axis_world, subtract(point_world, joint_origin))
                            }
                            MirrorJointKind::Prismatic => axis_world,
                            MirrorJointKind::Fixed => [0.0; 3],
                        };
                        let generalized_coordinate = ROOT_TANGENT_COMPONENTS + coordinate;
                        for (component, value) in value.into_iter().enumerate() {
                            output.multiply_add_center_of_mass_jacobian_value(
                                agent,
                                component,
                                generalized_coordinate,
                                scale,
                                value,
                            );
                        }
                    }
                    ancestor = joint.parent;
                }
            }
        }
        Ok(())
    }

    /// Evaluate floating mass, bias, and centroidal products in fixed f32
    /// order from completed FK and Jacobian stages.
    pub fn execute_dynamics_into(
        &self,
        input: &DynamicsBatchInput,
        fk_output: &FkBatchOutput,
        jacobian_output: &JacobianBatchOutput,
        output: &mut DynamicsBatchOutput,
    ) -> Result<(), BatchExecutionError> {
        if input.layout() != &self.descriptor.layout
            || fk_output.layout() != &self.descriptor.layout
            || jacobian_output.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
        {
            return Err(BatchExecutionError::LayoutMismatch);
        }
        output.clear();
        let generalized_dof = self.descriptor.layout.generalized_coordinate_count;
        let stride = self.descriptor.layout.agent_stride;
        for agent in 0..self.descriptor.layout.agent_capacity {
            let prior_status = if fk_output.agent_status()[agent] == AgentStatus::Ok
                && jacobian_output.agent_status()[agent] == AgentStatus::Ok
                && agent < input.active_agents()
            {
                AgentStatus::Ok
            } else if fk_output.agent_status()[agent] == AgentStatus::InvalidInput
                || jacobian_output.agent_status()[agent] == AgentStatus::InvalidInput
            {
                AgentStatus::InvalidInput
            } else {
                AgentStatus::Inactive
            };
            let velocity_valid = (0..generalized_dof).all(|coordinate| {
                input.generalized_velocity_soa()
                    [self.descriptor.layout.generalized_index(coordinate, agent)]
                .is_finite()
            });
            let gravity_valid = (0..3)
                .all(|component| input.gravity_world_soa()[component * stride + agent].is_finite());
            let status = if prior_status == AgentStatus::Ok && (!velocity_valid || !gravity_valid) {
                AgentStatus::InvalidInput
            } else {
                prior_status
            };
            output.set_status(agent, status);
            if status != AgentStatus::Ok {
                continue;
            }
            let root_velocity = |coordinate| {
                input.generalized_velocity_soa()
                    [self.descriptor.layout.generalized_index(coordinate, agent)]
            };
            output.set_angular_velocity(
                self.root_body,
                agent,
                [root_velocity(0), root_velocity(1), root_velocity(2)],
            );
            output.set_linear_velocity_origin(
                self.root_body,
                agent,
                [root_velocity(3), root_velocity(4), root_velocity(5)],
            );
            for joint in &self.joints {
                let angular_parent = output.angular_velocity(joint.parent, agent);
                let angular_acceleration_parent = output.angular_acceleration(joint.parent, agent);
                let velocity_parent = output.linear_velocity_origin(joint.parent, agent);
                let acceleration_parent = output.linear_acceleration_origin(joint.parent, agent);
                let parent_pose = Pose(
                    fk_output
                        .body_pose(agent, joint.parent)
                        .expect("compiled parent body"),
                );
                let joint_pose = compose(parent_pose, joint.parent_from_joint);
                let parent_origin = pose_translation(parent_pose);
                let joint_origin = pose_translation(joint_pose);
                let parent_to_joint = subtract(joint_origin, parent_origin);
                let velocity_joint = add(velocity_parent, cross(angular_parent, parent_to_joint));
                let acceleration_joint = add(
                    add(
                        acceleration_parent,
                        cross(angular_acceleration_parent, parent_to_joint),
                    ),
                    cross(angular_parent, cross(angular_parent, parent_to_joint)),
                );
                let axis_world = joint_pose.transform_vector(joint.axis);
                let joint_velocity = joint.coordinate.map_or(0.0, |coordinate| {
                    input.generalized_velocity_soa()[self
                        .descriptor
                        .layout
                        .generalized_index(ROOT_TANGENT_COMPONENTS + coordinate, agent)]
                });
                match joint.kind {
                    MirrorJointKind::Revolute => {
                        let axis_velocity = scale(axis_world, joint_velocity);
                        output.set_angular_velocity(
                            joint.child,
                            agent,
                            add(angular_parent, axis_velocity),
                        );
                        output.set_angular_acceleration(
                            joint.child,
                            agent,
                            add(
                                angular_acceleration_parent,
                                cross(angular_parent, axis_velocity),
                            ),
                        );
                        output.set_linear_velocity_origin(joint.child, agent, velocity_joint);
                        output.set_linear_acceleration_origin(
                            joint.child,
                            agent,
                            acceleration_joint,
                        );
                    }
                    MirrorJointKind::Prismatic => {
                        let child_pose = Pose(
                            fk_output
                                .body_pose(agent, joint.child)
                                .expect("compiled child body"),
                        );
                        let joint_to_child = subtract(pose_translation(child_pose), joint_origin);
                        let axis_velocity = scale(axis_world, joint_velocity);
                        output.set_angular_velocity(joint.child, agent, angular_parent);
                        output.set_angular_acceleration(
                            joint.child,
                            agent,
                            angular_acceleration_parent,
                        );
                        output.set_linear_velocity_origin(
                            joint.child,
                            agent,
                            add(
                                add(velocity_joint, cross(angular_parent, joint_to_child)),
                                axis_velocity,
                            ),
                        );
                        output.set_linear_acceleration_origin(
                            joint.child,
                            agent,
                            add(
                                add(
                                    add(
                                        acceleration_joint,
                                        cross(angular_acceleration_parent, joint_to_child),
                                    ),
                                    cross(angular_parent, cross(angular_parent, joint_to_child)),
                                ),
                                scale(cross(angular_parent, axis_velocity), 2.0),
                            ),
                        );
                    }
                    MirrorJointKind::Fixed => {
                        output.set_angular_velocity(joint.child, agent, angular_parent);
                        output.set_angular_acceleration(
                            joint.child,
                            agent,
                            angular_acceleration_parent,
                        );
                        output.set_linear_velocity_origin(joint.child, agent, velocity_joint);
                        output.set_linear_acceleration_origin(
                            joint.child,
                            agent,
                            acceleration_joint,
                        );
                    }
                }
            }

            let system_com = fk_output.center_of_mass(agent).expect("active agent CoM");
            let gravity = [
                input.gravity_world_soa()[agent],
                input.gravity_world_soa()[stride + agent],
                input.gravity_world_soa()[2 * stride + agent],
            ];
            for (body_index, body) in self.bodies.iter().copied().enumerate() {
                if body.mass <= 0.0 {
                    continue;
                }
                let body_pose = Pose(
                    fk_output
                        .body_pose(agent, body_index)
                        .expect("compiled body pose"),
                );
                let com_offset = body_pose.transform_vector(body.com_in_body);
                let com_world = add(pose_translation(body_pose), com_offset);
                let inertia_world = inertia_world(body_pose, body.inertia_about_com_in_body);

                for coordinate in 0..generalized_dof {
                    let (angular, linear_origin) =
                        frame_column(jacobian_output, agent, body_index, coordinate);
                    let linear = add(linear_origin, cross(angular, com_offset));
                    let linear_momentum = scale(linear, body.mass);
                    let angular_momentum = add(
                        matrix_vector(inertia_world, angular),
                        cross(subtract(com_world, system_com), linear_momentum),
                    );
                    for component in 0..3 {
                        output.multiply_add_centroidal(
                            agent,
                            component,
                            coordinate,
                            angular_momentum[component],
                        );
                        output.multiply_add_centroidal(
                            agent,
                            3 + component,
                            coordinate,
                            linear_momentum[component],
                        );
                    }
                    for column in 0..=coordinate {
                        let (angular_column, linear_origin_column) =
                            frame_column(jacobian_output, agent, body_index, column);
                        let linear_column =
                            add(linear_origin_column, cross(angular_column, com_offset));
                        let value = body.mass.mul_add(
                            dot(linear, linear_column),
                            dot(angular, matrix_vector(inertia_world, angular_column)),
                        );
                        output.multiply_add_mass(agent, coordinate, column, value);
                        if coordinate != column {
                            output.multiply_add_mass(agent, column, coordinate, value);
                        }
                    }
                }

                let angular_velocity = output.angular_velocity(body_index, agent);
                let angular_acceleration = output.angular_acceleration(body_index, agent);
                let com_acceleration = add(
                    add(
                        output.linear_acceleration_origin(body_index, agent),
                        cross(angular_acceleration, com_offset),
                    ),
                    cross(angular_velocity, cross(angular_velocity, com_offset)),
                );
                let force = scale(subtract(com_acceleration, gravity), body.mass);
                let inertia_angular_velocity = matrix_vector(inertia_world, angular_velocity);
                let moment_at_com = add(
                    matrix_vector(inertia_world, angular_acceleration),
                    cross(angular_velocity, inertia_angular_velocity),
                );
                for coordinate in 0..generalized_dof {
                    let (angular, linear_origin) =
                        frame_column(jacobian_output, agent, body_index, coordinate);
                    let linear = add(linear_origin, cross(angular, com_offset));
                    for component in 0..3 {
                        output.multiply_add_bias(
                            agent,
                            coordinate,
                            linear[component],
                            force[component],
                        );
                    }
                    for component in 0..3 {
                        output.multiply_add_bias(
                            agent,
                            coordinate,
                            angular[component],
                            moment_at_com[component],
                        );
                    }
                }
            }
        }
        Ok(())
    }

    /// Evaluate every compiler-resolved point slot in fixed descriptor order.
    /// Outputs are point position, floating point Jacobian, and the kinematic
    /// bias acceleration `Jdot-v` for zero generalized acceleration.
    pub fn execute_point_queries_into(
        &self,
        fk_output: &FkBatchOutput,
        jacobian_output: &JacobianBatchOutput,
        dynamics_output: &DynamicsBatchOutput,
        output: &mut PointQueryBatchOutput,
    ) -> Result<(), BatchExecutionError> {
        if fk_output.layout() != &self.descriptor.layout
            || jacobian_output.layout() != &self.descriptor.layout
            || dynamics_output.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
        {
            return Err(BatchExecutionError::LayoutMismatch);
        }
        output.clear();
        let generalized_dof = self.descriptor.layout.generalized_coordinate_count;
        for agent in 0..self.descriptor.layout.agent_capacity {
            let status = if fk_output.agent_status()[agent] == AgentStatus::InvalidInput
                || jacobian_output.agent_status()[agent] == AgentStatus::InvalidInput
                || dynamics_output.agent_status()[agent] == AgentStatus::InvalidInput
            {
                AgentStatus::InvalidInput
            } else if fk_output.agent_status()[agent] == AgentStatus::Ok
                && jacobian_output.agent_status()[agent] == AgentStatus::Ok
                && dynamics_output.agent_status()[agent] == AgentStatus::Ok
            {
                AgentStatus::Ok
            } else {
                AgentStatus::Inactive
            };
            output.set_status(agent, status);
            if status != AgentStatus::Ok {
                continue;
            }
            for (slot, query) in self.descriptor.point_queries.iter().copied().enumerate() {
                let body_pose = Pose(
                    fk_output
                        .body_pose(agent, query.frame_index)
                        .expect("compiled point frame"),
                );
                let offset_world = body_pose.transform_vector(query.point_in_frame());
                let point_world = add(pose_translation(body_pose), offset_world);
                output.set_point_position(agent, slot, point_world);
                for coordinate in 0..generalized_dof {
                    let (angular, linear_origin) =
                        frame_column(jacobian_output, agent, query.frame_index, coordinate);
                    let linear = add(linear_origin, cross(angular, offset_world));
                    for (component, value) in linear.into_iter().enumerate() {
                        output.set_point_jacobian_value(agent, slot, component, coordinate, value);
                    }
                }
                let angular_velocity = dynamics_output.angular_velocity(query.frame_index, agent);
                let angular_acceleration =
                    dynamics_output.angular_acceleration(query.frame_index, agent);
                let bias_acceleration = add(
                    add(
                        dynamics_output.linear_acceleration_origin(query.frame_index, agent),
                        cross(angular_acceleration, offset_world),
                    ),
                    cross(angular_velocity, cross(angular_velocity, offset_world)),
                );
                output.set_point_bias_acceleration(agent, slot, bias_acceleration);
            }
        }
        Ok(())
    }

    /// Lower fixed point-attractor objectives and contact-lock equalities into
    /// stable rows over generalized acceleration. Inactive slots are exactly
    /// zero; metadata and provenance remain in the immutable descriptor.
    pub fn execute_emission_into(
        &self,
        input: &EmissionBatchInput,
        dynamics_input: &DynamicsBatchInput,
        point_output: &PointQueryBatchOutput,
        output: &mut EmissionBatchOutput,
    ) -> Result<(), BatchExecutionError> {
        if input.layout() != &self.descriptor.layout
            || dynamics_input.layout() != &self.descriptor.layout
            || point_output.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
        {
            return Err(BatchExecutionError::LayoutMismatch);
        }
        output.clear();
        let generalized_dof = self.descriptor.layout.generalized_coordinate_count;
        for agent in 0..self.descriptor.layout.agent_capacity {
            let prior_status = if point_output.agent_status()[agent] == AgentStatus::InvalidInput {
                AgentStatus::InvalidInput
            } else if point_output.agent_status()[agent] == AgentStatus::Ok
                && agent < input.active_agents()
                && agent < dynamics_input.active_agents()
            {
                AgentStatus::Ok
            } else {
                AgentStatus::Inactive
            };
            let task_inputs_valid =
                self.descriptor
                    .point_tasks
                    .iter()
                    .enumerate()
                    .all(|(slot, _)| {
                        let active = input.point_task_active_soa()
                            [self.descriptor.layout.task_active_index(slot, agent)];
                        active <= 1
                            && (active == 0
                                || input
                                    .point_target_position(slot, agent)
                                    .into_iter()
                                    .chain(input.point_target_velocity(slot, agent))
                                    .chain(input.point_target_acceleration(slot, agent))
                                    .all(f32::is_finite))
                    });
            let contact_inputs_valid =
                self.descriptor
                    .contact_locks
                    .iter()
                    .enumerate()
                    .all(|(slot, _)| {
                        let active = input.contact_lock_active_soa()
                            [self.descriptor.layout.contact_active_index(slot, agent)];
                        active <= 1
                            && (active == 0
                                || input
                                    .contact_desired_acceleration(slot, agent)
                                    .into_iter()
                                    .all(f32::is_finite))
                    });
            let status = if prior_status == AgentStatus::Ok
                && (!task_inputs_valid || !contact_inputs_valid)
            {
                AgentStatus::InvalidInput
            } else {
                prior_status
            };
            output.set_status(agent, status);
            if status != AgentStatus::Ok {
                continue;
            }
            for (slot, task) in self.descriptor.point_tasks.iter().copied().enumerate() {
                if input.point_task_active_soa()
                    [self.descriptor.layout.task_active_index(slot, agent)]
                    == 0
                {
                    continue;
                }
                output.set_task_active(slot, agent);
                let current_position = point_output
                    .point_position(agent, task.point_query_slot)
                    .expect("compiled point task query");
                let target_position = input.point_target_position(slot, agent);
                let target_velocity = input.point_target_velocity(slot, agent);
                let target_acceleration = input.point_target_acceleration(slot, agent);
                let bias = point_output
                    .point_bias_acceleration(agent, task.point_query_slot)
                    .expect("compiled point task bias");
                let mut current_velocity = [0.0_f32; 3];
                for (component, current_component) in current_velocity.iter_mut().enumerate() {
                    for coordinate in 0..generalized_dof {
                        let jacobian = point_output
                            .point_jacobian_value(
                                agent,
                                task.point_query_slot,
                                component,
                                coordinate,
                            )
                            .expect("compiled point task Jacobian");
                        let velocity = dynamics_input.generalized_velocity_soa()
                            [self.descriptor.layout.generalized_index(coordinate, agent)];
                        *current_component = jacobian.mul_add(velocity, *current_component);
                        output.set_task_jacobian(slot, component, coordinate, agent, jacobian);
                    }
                }
                let position_error = subtract(target_position, current_position);
                let velocity_error = subtract(target_velocity, current_velocity);
                let omega = std::f32::consts::TAU * task.bandwidth_hz();
                let velocity_gain = 2.0 * omega;
                let position_gain = omega * omega;
                let mut desired_acceleration = [0.0_f32; 3];
                for component in 0..3 {
                    desired_acceleration[component] = position_gain.mul_add(
                        position_error[component],
                        velocity_gain
                            .mul_add(velocity_error[component], target_acceleration[component]),
                    );
                }
                output.set_task_vector(
                    TaskVectorStorage::PositionError,
                    slot,
                    agent,
                    position_error,
                );
                output.set_task_vector(
                    TaskVectorStorage::VelocityError,
                    slot,
                    agent,
                    velocity_error,
                );
                output.set_task_vector(
                    TaskVectorStorage::DesiredAcceleration,
                    slot,
                    agent,
                    desired_acceleration,
                );
                output.set_task_vector(
                    TaskVectorStorage::RightHandSide,
                    slot,
                    agent,
                    subtract(desired_acceleration, bias),
                );
            }
            for (slot, contact) in self.descriptor.contact_locks.iter().copied().enumerate() {
                if input.contact_lock_active_soa()
                    [self.descriptor.layout.contact_active_index(slot, agent)]
                    == 0
                {
                    continue;
                }
                output.set_contact_active(slot, agent);
                for component in 0..3 {
                    if !contact.kinematic_mode.axis_enabled(component) {
                        continue;
                    }
                    for coordinate in 0..generalized_dof {
                        let jacobian = point_output
                            .point_jacobian_value(
                                agent,
                                contact.point_query_slot,
                                component,
                                coordinate,
                            )
                            .expect("compiled contact Jacobian");
                        output.set_contact_jacobian(slot, component, coordinate, agent, jacobian);
                    }
                }
                let desired = input.contact_desired_acceleration(slot, agent);
                let bias = point_output
                    .point_bias_acceleration(agent, contact.point_query_slot)
                    .expect("compiled contact bias");
                let mut rhs = subtract(desired, bias);
                for (component, value) in rhs.iter_mut().enumerate() {
                    if !contact.kinematic_mode.axis_enabled(component) {
                        *value = 0.0;
                    }
                }
                output.set_contact_rhs(slot, agent, rhs);
            }
        }
        Ok(())
    }

    fn joint_world_pose(
        &self,
        fk_output: &FkBatchOutput,
        agent: usize,
        joint: MirrorJoint,
    ) -> Pose {
        let parent = Pose(
            fk_output
                .body_pose(agent, joint.parent)
                .expect("compiled parent body"),
        );
        compose(parent, joint.parent_from_joint)
    }
}

fn mirror_build_hash() -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(
        format!(
            "bonesaw-core@{}|bonesaw-cuda@{}|kernel-abi={}",
            env!("CARGO_PKG_VERSION"),
            env!("CARGO_PKG_VERSION"),
            KERNEL_ABI_VERSION
        )
        .as_bytes(),
    );
    digest.update(include_bytes!("cpu_mirror.rs"));
    digest.update(include_bytes!("layouts.rs"));
    digest.update(include_bytes!("kernel_api.rs"));
    digest.finalize().into()
}

fn subtract(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn add(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn scale(vector: [f32; 3], scalar: f32) -> [f32; 3] {
    [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

fn dot(left: [f32; 3], right: [f32; 3]) -> f32 {
    left[2].mul_add(right[2], left[1].mul_add(right[1], left[0] * right[0]))
}

fn pose_translation(pose: Pose) -> [f32; 3] {
    [
        pose.translation(0),
        pose.translation(1),
        pose.translation(2),
    ]
}

fn matrix_vector(matrix: [f32; 9], vector: [f32; 3]) -> [f32; 3] {
    let mut output = [0.0; 3];
    for row in 0..3 {
        output[row] = matrix[row * 3 + 2].mul_add(
            vector[2],
            matrix[row * 3 + 1].mul_add(vector[1], matrix[row * 3] * vector[0]),
        );
    }
    output
}

fn inertia_world(pose: Pose, inertia_body: [f32; 9]) -> [f32; 9] {
    let mut intermediate = [0.0_f32; 9];
    let mut output = [0.0_f32; 9];
    for row in 0..3 {
        for column in 0..3 {
            intermediate[row * 3 + column] = pose.rotation(row, 2).mul_add(
                inertia_body[6 + column],
                pose.rotation(row, 1).mul_add(
                    inertia_body[3 + column],
                    pose.rotation(row, 0) * inertia_body[column],
                ),
            );
        }
    }
    for row in 0..3 {
        for column in 0..3 {
            output[row * 3 + column] = intermediate[row * 3 + 2].mul_add(
                pose.rotation(column, 2),
                intermediate[row * 3 + 1].mul_add(
                    pose.rotation(column, 1),
                    intermediate[row * 3] * pose.rotation(column, 0),
                ),
            );
        }
    }
    output
}

fn frame_column(
    jacobian: &JacobianBatchOutput,
    agent: usize,
    body: usize,
    coordinate: usize,
) -> ([f32; 3], [f32; 3]) {
    (
        [
            jacobian
                .frame_jacobian_value(agent, body, 0, coordinate)
                .expect("compiled Jacobian entry"),
            jacobian
                .frame_jacobian_value(agent, body, 1, coordinate)
                .expect("compiled Jacobian entry"),
            jacobian
                .frame_jacobian_value(agent, body, 2, coordinate)
                .expect("compiled Jacobian entry"),
        ],
        [
            jacobian
                .frame_jacobian_value(agent, body, 3, coordinate)
                .expect("compiled Jacobian entry"),
            jacobian
                .frame_jacobian_value(agent, body, 4, coordinate)
                .expect("compiled Jacobian entry"),
            jacobian
                .frame_jacobian_value(agent, body, 5, coordinate)
                .expect("compiled Jacobian entry"),
        ],
    )
}

fn cross(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1].mul_add(right[2], -(left[2] * right[1])),
        left[2].mul_add(right[0], -(left[0] * right[2])),
        left[0].mul_add(right[1], -(left[1] * right[0])),
    ]
}

fn finite_f32(value: f64) -> Result<f32, BatchExecutionError> {
    let converted = value as f32;
    if converted.is_finite() {
        Ok(converted)
    } else {
        Err(BatchExecutionError::NonFiniteModel)
    }
}

fn pose_from_core(transform: bonesaw_core::Transform3) -> Result<Pose, BatchExecutionError> {
    let rotation = transform.rotation.to_rotation_matrix();
    let matrix = rotation.matrix();
    let mut pose = [0.0; POSE_COMPONENTS];
    for row in 0..3 {
        for column in 0..3 {
            pose[row * 3 + column] = finite_f32(matrix[(row, column)])?;
        }
        pose[9 + row] = finite_f32(transform.translation.vector[row])?;
    }
    Ok(Pose(pose))
}

fn compose(left: Pose, right: Pose) -> Pose {
    let mut output = [0.0; POSE_COMPONENTS];
    for row in 0..3 {
        for column in 0..3 {
            let mut value = left.rotation(row, 0) * right.rotation(0, column);
            value = left
                .rotation(row, 1)
                .mul_add(right.rotation(1, column), value);
            value = left
                .rotation(row, 2)
                .mul_add(right.rotation(2, column), value);
            output[row * 3 + column] = value;
        }
        let mut translation = left.rotation(row, 0) * right.translation(0);
        translation = left
            .rotation(row, 1)
            .mul_add(right.translation(1), translation);
        translation = left
            .rotation(row, 2)
            .mul_add(right.translation(2), translation);
        output[9 + row] = translation + left.translation(row);
    }
    Pose(output)
}

fn revolute_motion(axis: [f32; 3], angle: f32) -> Pose {
    let (sine, cosine) = angle.sin_cos();
    let one_minus_cosine = 1.0 - cosine;
    let [x, y, z] = axis;
    let xx = x * x * one_minus_cosine;
    let yy = y * y * one_minus_cosine;
    let zz = z * z * one_minus_cosine;
    let xy = x * y * one_minus_cosine;
    let xz = x * z * one_minus_cosine;
    let yz = y * z * one_minus_cosine;
    let xs = x * sine;
    let ys = y * sine;
    let zs = z * sine;
    Pose([
        xx + cosine,
        xy - zs,
        xz + ys,
        xy + zs,
        yy + cosine,
        yz - xs,
        xz - ys,
        yz + xs,
        zz + cosine,
        0.0,
        0.0,
        0.0,
    ])
}

fn prismatic_motion(axis: [f32; 3], distance: f32) -> Pose {
    let mut pose = Pose::IDENTITY;
    for (component, value) in axis.into_iter().enumerate() {
        pose.0[9 + component] = value * distance;
    }
    pose
}

fn load_root_pose(input: &FkBatchInput, agent: usize) -> Option<Pose> {
    let mut pose = [0.0; POSE_COMPONENTS];
    for (component, value) in pose.iter_mut().enumerate() {
        *value = input.root_pose_soa()[input.layout().root_pose_index(component, agent)];
    }
    if !pose.iter().all(|value| value.is_finite()) || !rotation_is_valid(&pose) {
        return None;
    }
    Some(Pose(pose))
}

fn rotation_is_valid(pose: &[f32; POSE_COMPONENTS]) -> bool {
    let tolerance = 2.0e-3_f32;
    for row in 0..3 {
        let norm = pose[row * 3].mul_add(
            pose[row * 3],
            pose[row * 3 + 1].mul_add(pose[row * 3 + 1], pose[row * 3 + 2] * pose[row * 3 + 2]),
        );
        if (norm - 1.0).abs() > tolerance {
            return false;
        }
        for other in row + 1..3 {
            let dot = pose[row * 3].mul_add(
                pose[other * 3],
                pose[row * 3 + 1]
                    .mul_add(pose[other * 3 + 1], pose[row * 3 + 2] * pose[other * 3 + 2]),
            );
            if dot.abs() > tolerance {
                return false;
            }
        }
    }
    let determinant = pose[0].mul_add(
        pose[4].mul_add(pose[8], -(pose[5] * pose[7])),
        -pose[1].mul_add(
            pose[3].mul_add(pose[8], -(pose[5] * pose[6])),
            -(pose[2] * pose[3].mul_add(pose[7], -(pose[4] * pose[6]))),
        ),
    );
    (determinant - 1.0).abs() <= 4.0 * tolerance
}

fn cpu_isa() -> String {
    #[cfg(target_arch = "x86_64")]
    {
        return format!(
            "x86_64:fma={}:avx2={}",
            std::is_x86_feature_detected!("fma"),
            std::is_x86_feature_detected!("avx2")
        );
    }
    #[allow(unreachable_code)]
    std::env::consts::ARCH.to_owned()
}

#[cfg(test)]
mod tests {
    use std::hint::black_box;

    use bonesaw_core::{
        CompiledSignalProgram, CompiledTaskProgram, DynamicsCache, FrameId, ModelCache, Motion6,
        Priority, RobotState, SignalOp, SignalOutputSpec, TaskSpec, TimingSpec, Vec3, VectorJet,
    };
    use nalgebra::{DMatrix, DVector, Matrix3, Point3, SVector, Translation3, UnitQuaternion};

    use super::*;
    use crate::{KernelStage, allocation_sentinel};

    fn program(source: &str) -> MotionProgram {
        MotionProgram::compile_urdf(source, TimingSpec::default(), 17).unwrap()
    }

    #[test]
    fn rigid_patch_basis_is_geometry_derived_full_rank_and_fingerprinted() {
        let program = program(include_str!("../../../models/toy_humanoid.urdf"));
        let foot = program.model.frame_id("left_foot").unwrap().0;
        let offsets = [
            [-0.08, -0.04, -0.07],
            [-0.08, 0.04, -0.07],
            [0.12, -0.04, -0.07],
            [0.12, 0.04, -0.07],
        ];
        let queries = std::array::from_fn::<_, 4, _>(|slot| PointQuerySpec {
            stable_id: 9_001 + slot as u32,
            frame_index: foot,
            point_in_frame: offsets[slot],
        });
        let contacts = std::array::from_fn::<_, 4, _>(|slot| ContactLockSpec {
            stable_id: 9_201 + slot as u32,
            point_query_stable_id: 9_001 + slot as u32,
        });
        let patch = [RigidPatchBasisSpec {
            stable_id: 8_401,
            first_contact_slot: 0,
            contact_count: 4,
        }];
        let plan = derive_rigid_patch_contact_modes(&queries, &contacts, &patch).unwrap();
        assert_eq!(plan.patches.len(), 1);
        assert_eq!(plan.patches[0].row_count, 6);
        assert_eq!(plan.patches[0].rank, 6);
        assert!(plan.patches[0].minimum_singular_value > 0.04);
        assert_eq!(
            plan.contact_modes
                .iter()
                .map(|mode| (0..3).filter(|axis| mode.axis_enabled(*axis)).count())
                .sum::<usize>(),
            6
        );

        let translated = queries.map(|mut query| {
            for (value, shift) in query.point_in_frame.iter_mut().zip([3.0, -2.0, 1.0]) {
                *value += shift;
            }
            query
        });
        let translated_plan =
            derive_rigid_patch_contact_modes(&translated, &contacts, &patch).unwrap();
        assert_eq!(plan.contact_modes, translated_plan.contact_modes);
        assert!(
            (plan.patches[0].minimum_singular_value
                - translated_plan.patches[0].minimum_singular_value)
                .abs()
                < 1.0e-14
        );

        let automatic = CpuMirrorExecutor::compile_with_auto_rigid_patches(
            &program,
            2,
            2,
            &queries,
            &[],
            &contacts,
            &patch,
        )
        .unwrap();
        let all_locked =
            CpuMirrorExecutor::compile_with_emission_plan(&program, 2, 2, &queries, &[], &contacts)
                .unwrap();
        assert_eq!(
            automatic
                .descriptor()
                .contact_locks
                .iter()
                .map(|contact| contact.kinematic_mode)
                .collect::<Vec<_>>(),
            plan.contact_modes
        );
        assert_ne!(
            automatic.fingerprint().kernel_hash,
            all_locked.fingerprint().kernel_hash
        );

        let collinear = std::array::from_fn::<_, 4, _>(|slot| PointQuerySpec {
            stable_id: 9_001 + slot as u32,
            frame_index: foot,
            point_in_frame: [slot as f64 * 0.05, 0.0, 0.0],
        });
        assert!(matches!(
            derive_rigid_patch_contact_modes(&collinear, &contacts, &patch),
            Err(BatchExecutionError::RigidPatchRankDeficient { stable_id: 8_401 })
        ));
    }

    fn deterministic_states(program: &MotionProgram, count: usize) -> Vec<RobotState> {
        let mut random = 0xd248_3a95_16e7_c04b_u64;
        let mut sample = || {
            random ^= random << 13;
            random ^= random >> 7;
            random ^= random << 17;
            random as i64 as f64 / i64::MAX as f64
        };
        (0..count)
            .map(|agent| {
                let mut state = RobotState::zeros(&program.model);
                for joint in &program.model.joints {
                    if let Some(coordinate) = joint.coordinate {
                        let lower = if joint.limit.lower.is_finite() {
                            joint.limit.lower
                        } else {
                            -1.0
                        };
                        let upper = if joint.limit.upper.is_finite() {
                            joint.limit.upper
                        } else {
                            1.0
                        };
                        let blend = 0.5 + 0.45 * sample();
                        state.q[coordinate] = lower + blend * (upper - lower);
                        state.v[coordinate] = 0.5 * sample();
                    }
                }
                let rotation = UnitQuaternion::from_euler_angles(
                    0.15 * sample(),
                    0.15 * sample(),
                    0.3 * sample(),
                );
                state.control_world_from_root = bonesaw_core::Transform3::from_parts(
                    Translation3::new(
                        0.2 * agent as f64 + 0.05 * sample(),
                        0.1 * sample(),
                        0.8 + 0.05 * sample(),
                    ),
                    rotation,
                );
                state
            })
            .collect()
    }

    fn execute_states(
        executor: &CpuMirrorExecutor,
        program: &MotionProgram,
        states: &[RobotState],
    ) -> FkBatchOutput {
        let mut input =
            FkBatchInput::new(executor.descriptor().layout.clone(), states.len()).unwrap();
        for (agent, state) in states.iter().enumerate() {
            input
                .set_agent_from_state(&program.model, agent, state)
                .unwrap();
        }
        let mut output = FkBatchOutput::new(executor.descriptor().layout.clone());
        executor.execute_into(&input, &mut output).unwrap();
        output
    }

    fn assert_agent_bits_equal(
        left: &FkBatchOutput,
        left_agent: usize,
        right: &FkBatchOutput,
        right_agent: usize,
    ) {
        assert_eq!(
            left.agent_status()[left_agent],
            right.agent_status()[right_agent]
        );
        for body in 0..left.layout().body_count {
            let left_pose = left.body_pose(left_agent, body).unwrap();
            let right_pose = right.body_pose(right_agent, body).unwrap();
            assert!(
                left_pose
                    .iter()
                    .zip(right_pose)
                    .all(|(left, right)| left.to_bits() == right.to_bits())
            );
        }
        let left_com = left.center_of_mass(left_agent).unwrap();
        let right_com = right.center_of_mass(right_agent).unwrap();
        assert!(
            left_com
                .iter()
                .zip(right_com)
                .all(|(left, right)| left.to_bits() == right.to_bits())
        );
        assert_eq!(
            left.total_mass()[left_agent].to_bits(),
            right.total_mass()[right_agent].to_bits()
        );
    }

    fn execute_jacobians(
        executor: &CpuMirrorExecutor,
        fk_output: &FkBatchOutput,
    ) -> JacobianBatchOutput {
        let mut output = JacobianBatchOutput::new(executor.descriptor().layout.clone());
        executor
            .execute_jacobians_into(fk_output, &mut output)
            .unwrap();
        output
    }

    fn root_twist(agent: usize) -> Motion6 {
        let phase = agent as f64 + 1.0;
        Motion6(SVector::<f64, 6>::from_row_slice(&[
            0.11 * phase.sin(),
            -0.07 * (0.7 * phase).cos(),
            0.09 * (0.3 * phase).sin(),
            0.13 * (0.5 * phase).cos(),
            -0.08 * (0.9 * phase).sin(),
            0.06 * (0.4 * phase).cos(),
        ]))
    }

    fn dynamics_input(executor: &CpuMirrorExecutor, states: &[RobotState]) -> DynamicsBatchInput {
        let mut input =
            DynamicsBatchInput::new(executor.descriptor().layout.clone(), states.len()).unwrap();
        let stride = input.layout().agent_stride;
        for (agent, state) in states.iter().enumerate() {
            let root = root_twist(agent);
            for coordinate in 0..6 {
                let index = input.layout().generalized_index(coordinate, agent);
                input.generalized_velocity_soa_mut()[index] = root.0[coordinate] as f32;
            }
            for (coordinate, value) in state.v.iter().copied().enumerate() {
                let index = input.layout().generalized_index(6 + coordinate, agent);
                input.generalized_velocity_soa_mut()[index] = value as f32;
            }
            input.gravity_world_soa_mut()[agent] = 0.17;
            input.gravity_world_soa_mut()[stride + agent] = -0.09;
            input.gravity_world_soa_mut()[2 * stride + agent] = -9.73;
        }
        input
    }

    fn execute_dynamics(
        executor: &CpuMirrorExecutor,
        states: &[RobotState],
        fk_output: &FkBatchOutput,
        jacobian_output: &JacobianBatchOutput,
    ) -> DynamicsBatchOutput {
        let input = dynamics_input(executor, states);
        let mut output = DynamicsBatchOutput::new(executor.descriptor().layout.clone());
        executor
            .execute_dynamics_into(&input, fk_output, jacobian_output, &mut output)
            .unwrap();
        output
    }

    fn assert_dynamics_agent_bits_equal(
        left: &DynamicsBatchOutput,
        left_agent: usize,
        right: &DynamicsBatchOutput,
        right_agent: usize,
    ) {
        assert_eq!(
            left.agent_status()[left_agent],
            right.agent_status()[right_agent]
        );
        let generalized_dof = left.layout().generalized_coordinate_count;
        for row in 0..generalized_dof {
            assert_eq!(
                left.bias_force_value(left_agent, row).unwrap().to_bits(),
                right.bias_force_value(right_agent, row).unwrap().to_bits()
            );
            for column in 0..generalized_dof {
                assert_eq!(
                    left.mass_matrix_value(left_agent, row, column)
                        .unwrap()
                        .to_bits(),
                    right
                        .mass_matrix_value(right_agent, row, column)
                        .unwrap()
                        .to_bits()
                );
            }
        }
        for component in 0..6 {
            for coordinate in 0..generalized_dof {
                assert_eq!(
                    left.centroidal_map_value(left_agent, component, coordinate)
                        .unwrap()
                        .to_bits(),
                    right
                        .centroidal_map_value(right_agent, component, coordinate)
                        .unwrap()
                        .to_bits()
                );
            }
        }
    }

    fn assert_jacobian_agent_bits_equal(
        left: &JacobianBatchOutput,
        left_agent: usize,
        right: &JacobianBatchOutput,
        right_agent: usize,
    ) {
        assert_eq!(
            left.agent_status()[left_agent],
            right.agent_status()[right_agent]
        );
        for body in 0..left.layout().body_count {
            for component in 0..left.layout().spatial_components {
                for coordinate in 0..left.layout().generalized_coordinate_count {
                    assert_eq!(
                        left.frame_jacobian_value(left_agent, body, component, coordinate)
                            .unwrap()
                            .to_bits(),
                        right
                            .frame_jacobian_value(right_agent, body, component, coordinate,)
                            .unwrap()
                            .to_bits()
                    );
                }
            }
        }
        for component in 0..3 {
            for coordinate in 0..left.layout().generalized_coordinate_count {
                assert_eq!(
                    left.center_of_mass_jacobian_value(left_agent, component, coordinate)
                        .unwrap()
                        .to_bits(),
                    right
                        .center_of_mass_jacobian_value(right_agent, component, coordinate)
                        .unwrap()
                        .to_bits()
                );
            }
        }
    }

    fn assert_mirror_close(mirror: f32, exact: f64, label: &str) {
        let error = (mirror as f64 - exact).abs();
        let tolerance = 2.0e-5 + 2.0e-4 * (mirror as f64).abs().max(exact.abs());
        assert!(
            error <= tolerance,
            "{label}: {mirror} vs {exact}, error {error}"
        );
    }

    #[test]
    fn descriptor_freezes_soa_layout_manifest_and_fingerprint() {
        let program = program(include_str!("../../../models/toy_humanoid.urdf"));
        let first = CpuMirrorExecutor::compile(&program, 33, 32).unwrap();
        let second = CpuMirrorExecutor::compile(&program, 33, 32).unwrap();
        assert_eq!(first.descriptor(), second.descriptor());
        assert_eq!(first.fingerprint(), second.fingerprint());
        assert_eq!(first.descriptor().layout.agent_stride, 64);
        assert_eq!(first.descriptor().layout.pose_components, 12);
        assert_eq!(first.descriptor().layout.spatial_components, 6);
        assert_eq!(
            first.descriptor().layout.generalized_coordinate_count,
            program.model.dof + 6
        );
        assert!(
            first
                .descriptor()
                .manifest
                .supports(KernelStage::ForwardKinematics)
        );
        assert!(
            first
                .descriptor()
                .manifest
                .supports(KernelStage::CenterOfMass)
        );
        assert!(first.descriptor().manifest.supports(KernelStage::Jacobians));
        assert!(
            first
                .descriptor()
                .manifest
                .supports(KernelStage::RigidBodyDynamics)
        );
        assert!(
            first
                .descriptor()
                .manifest
                .supports(KernelStage::PointQueries)
        );
        assert_eq!(first.descriptor().layout.point_query_count, 0);
        assert!(first.descriptor().point_queries.is_empty());
        assert!(
            !first
                .descriptor()
                .manifest
                .supports(KernelStage::HierarchicalSolve)
        );
        assert_eq!(
            first.fingerprint().backend_profile,
            BackendProfile::CpuMirrorF32
        );
        assert_eq!(first.fingerprint().scalar_format, ScalarFormat::F32);
        assert_eq!(
            first.fingerprint().program_hash,
            program.header.fingerprint_sha256
        );
    }

    #[test]
    fn compile_resolves_program_point_tasks_into_stable_query_slots() {
        let base = program(include_str!("../../../models/toy_humanoid.urdf"));
        let signals = CompiledSignalProgram::compile(
            vec![SignalOp::ConstantVector {
                stable_id: 10,
                jet: VectorJet {
                    value: Vec3::new(0.2, 0.1, 1.0),
                    velocity: Vec3::zeros(),
                    acceleration: Vec3::zeros(),
                },
            }],
            vec![SignalOutputSpec {
                stable_id: 100,
                node: 0,
            }],
        )
        .unwrap();
        let with_signals = base.with_signals(signals).unwrap();
        let frame = with_signals.model.frame_id("left_hand").unwrap();
        let point = Vec3::new(0.04, -0.03, 0.02);
        let tasks = CompiledTaskProgram::compile(
            &with_signals.model,
            &with_signals.signals,
            vec![TaskSpec::Point {
                stable_id: 7001,
                frame,
                point_in_frame: point,
                target_signal: 100,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 2.0,
            }],
        )
        .unwrap();
        let program = with_signals.with_tasks(tasks).unwrap();
        let executor = CpuMirrorExecutor::compile(&program, 8, 8).unwrap();
        assert_eq!(executor.descriptor().layout.point_query_count, 1);
        assert_eq!(executor.descriptor().layout.point_task_count, 1);
        assert_eq!(executor.descriptor().point_queries[0].stable_id, 7001);
        assert_eq!(executor.descriptor().point_queries[0].frame_index, frame.0);
        assert_eq!(
            executor.descriptor().point_queries[0].point_in_frame(),
            [point.x as f32, point.y as f32, point.z as f32]
        );
        assert_eq!(executor.descriptor().point_tasks[0].stable_id, 7001);
        assert_eq!(executor.descriptor().point_tasks[0].point_query_slot, 0);
        assert_eq!(
            executor.descriptor().point_tasks[0].priority,
            Priority::Intent
        );
        assert_eq!(executor.descriptor().point_tasks[0].weight(), 1.0);
        assert_eq!(executor.descriptor().point_tasks[0].bandwidth_hz(), 2.0);
    }

    #[test]
    fn cpu_mirror_fk_and_com_meet_d3_tolerances() {
        for source in [
            include_str!("../../../models/toy_humanoid.urdf"),
            include_str!("../../../models/upkie/upkie.urdf"),
        ] {
            let program = program(source);
            let states = deterministic_states(&program, 32);
            let executor = CpuMirrorExecutor::compile(&program, states.len(), 32).unwrap();
            let output = execute_states(&executor, &program, &states);
            let mut exact_cache = ModelCache::new(&program.model);
            for (agent, state) in states.iter().enumerate() {
                program
                    .model
                    .forward_kinematics(state, &mut exact_cache)
                    .unwrap();
                assert_eq!(output.agent_status()[agent], AgentStatus::Ok);
                for body in 0..program.model.bodies.len() {
                    let mirror = output.body_pose(agent, body).unwrap();
                    let exact = exact_cache.world_from_body[body];
                    let translation_error = ((0..3)
                        .map(|component| {
                            let delta =
                                mirror[9 + component] as f64 - exact.translation.vector[component];
                            delta * delta
                        })
                        .sum::<f64>())
                    .sqrt();
                    assert!(
                        translation_error <= 5.0e-5,
                        "translation {translation_error}"
                    );
                    let mirror_rotation = Matrix3::from_row_slice(&mirror[..9]).cast::<f64>();
                    let exact_rotation = exact.rotation.to_rotation_matrix();
                    let relative = exact_rotation.matrix().transpose() * mirror_rotation;
                    let cosine = ((relative.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
                    let sine = 0.5
                        * ((relative[(2, 1)] - relative[(1, 2)]).powi(2)
                            + (relative[(0, 2)] - relative[(2, 0)]).powi(2)
                            + (relative[(1, 0)] - relative[(0, 1)]).powi(2))
                        .sqrt();
                    let angle = sine.atan2(cosine);
                    assert!(angle <= 5.0e-5, "rotation {angle}");
                    let orthogonality = (mirror_rotation.transpose() * mirror_rotation
                        - Matrix3::identity())
                    .norm();
                    assert!(orthogonality <= 5.0e-5, "orthogonality {orthogonality}");
                }
                let mirror_com = output.center_of_mass(agent).unwrap();
                let com_error = ((0..3)
                    .map(|component| {
                        let delta = mirror_com[component] as f64
                            - exact_cache.center_of_mass_world[component];
                        delta * delta
                    })
                    .sum::<f64>())
                .sqrt();
                assert!(com_error <= 5.0e-5, "com {com_error}");
            }
        }
    }

    #[test]
    fn cpu_mirror_frame_origin_and_com_jacobians_meet_d3_tolerances() {
        for source in [
            include_str!("../../../models/toy_humanoid.urdf"),
            include_str!("../../../models/upkie/upkie.urdf"),
        ] {
            let program = program(source);
            let states = deterministic_states(&program, 16);
            let executor = CpuMirrorExecutor::compile(&program, states.len(), 32).unwrap();
            let fk_output = execute_states(&executor, &program, &states);
            let output = execute_jacobians(&executor, &fk_output);
            let generalized_dof = program.model.dof + 6;
            let mut cache = ModelCache::new(&program.model);
            let mut dynamics_cache = DynamicsCache::new(&program.model);
            let mut angular = DMatrix::zeros(3, generalized_dof);
            let mut linear = DMatrix::zeros(3, generalized_dof);
            let mut com = DMatrix::zeros(3, generalized_dof);
            for (agent, state) in states.iter().enumerate() {
                program.model.forward_kinematics(state, &mut cache).unwrap();
                for body in 0..program.model.bodies.len() {
                    program
                        .model
                        .floating_angular_jacobian_into(&cache, FrameId(body), &mut angular)
                        .unwrap();
                    program
                        .model
                        .floating_point_jacobian_into(
                            &cache,
                            FrameId(body),
                            Vec3::zeros(),
                            &mut linear,
                        )
                        .unwrap();
                    for component in 0..3 {
                        for coordinate in 0..generalized_dof {
                            assert_mirror_close(
                                output
                                    .frame_jacobian_value(agent, body, component, coordinate)
                                    .unwrap(),
                                angular[(component, coordinate)],
                                "frame angular Jacobian",
                            );
                            assert_mirror_close(
                                output
                                    .frame_jacobian_value(agent, body, 3 + component, coordinate)
                                    .unwrap(),
                                linear[(component, coordinate)],
                                "frame-origin linear Jacobian",
                            );
                        }
                    }
                }
                program
                    .model
                    .floating_com_jacobian_into(&cache, &mut dynamics_cache, &mut com)
                    .unwrap();
                for component in 0..3 {
                    for coordinate in 0..generalized_dof {
                        assert_mirror_close(
                            output
                                .center_of_mass_jacobian_value(agent, component, coordinate)
                                .unwrap(),
                            com[(component, coordinate)],
                            "center-of-mass Jacobian",
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn cpu_mirror_dynamics_products_match_f64_core_and_physical_invariants() {
        for source in [
            include_str!("../../../models/toy_humanoid.urdf"),
            include_str!("../../../models/upkie/upkie.urdf"),
        ] {
            let program = program(source);
            let states = deterministic_states(&program, 12);
            let executor = CpuMirrorExecutor::compile(&program, states.len(), 32).unwrap();
            let fk_output = execute_states(&executor, &program, &states);
            let jacobian_output = execute_jacobians(&executor, &fk_output);
            let output = execute_dynamics(&executor, &states, &fk_output, &jacobian_output);
            let generalized_dof = program.model.dof + 6;
            let mut model_cache = ModelCache::new(&program.model);
            let mut dynamics_cache = DynamicsCache::new(&program.model);
            let mut exact_mass = DMatrix::zeros(generalized_dof, generalized_dof);
            let mut exact_bias = DVector::zeros(generalized_dof);
            let mut exact_centroidal = DMatrix::zeros(6, generalized_dof);
            let gravity = Vec3::new(0.17, -0.09, -9.73);
            for (agent, state) in states.iter().enumerate() {
                program
                    .model
                    .forward_kinematics(state, &mut model_cache)
                    .unwrap();
                program
                    .model
                    .floating_mass_matrix_into(&model_cache, &mut dynamics_cache, &mut exact_mass)
                    .unwrap();
                program
                    .model
                    .floating_bias_forces_into(
                        state,
                        root_twist(agent),
                        gravity,
                        &model_cache,
                        &mut dynamics_cache,
                        &mut exact_bias,
                    )
                    .unwrap();
                program
                    .model
                    .floating_centroidal_map_into(
                        &model_cache,
                        &mut dynamics_cache,
                        &mut exact_centroidal,
                    )
                    .unwrap();
                assert_eq!(output.agent_status()[agent], AgentStatus::Ok);
                for row in 0..generalized_dof {
                    assert_mirror_close(
                        output.bias_force_value(agent, row).unwrap(),
                        exact_bias[row],
                        "floating bias",
                    );
                    for column in 0..generalized_dof {
                        let mirror = output.mass_matrix_value(agent, row, column).unwrap();
                        assert_mirror_close(
                            mirror,
                            exact_mass[(row, column)],
                            "floating mass matrix",
                        );
                        assert_eq!(
                            mirror.to_bits(),
                            output
                                .mass_matrix_value(agent, column, row)
                                .unwrap()
                                .to_bits(),
                            "mass matrix symmetry"
                        );
                    }
                }
                for component in 0..6 {
                    for coordinate in 0..generalized_dof {
                        assert_mirror_close(
                            output
                                .centroidal_map_value(agent, component, coordinate)
                                .unwrap(),
                            exact_centroidal[(component, coordinate)],
                            "floating centroidal map",
                        );
                    }
                }
                let mut kinetic_energy_twice = 0.0_f64;
                for row in 0..generalized_dof {
                    let row_velocity = if row < 6 {
                        root_twist(agent).0[row]
                    } else {
                        state.v[row - 6]
                    };
                    for column in 0..generalized_dof {
                        let column_velocity = if column < 6 {
                            root_twist(agent).0[column]
                        } else {
                            state.v[column - 6]
                        };
                        kinetic_energy_twice += row_velocity
                            * output.mass_matrix_value(agent, row, column).unwrap() as f64
                            * column_velocity;
                    }
                }
                assert!(kinetic_energy_twice > 0.0);
            }
        }
    }

    #[test]
    fn repeated_dynamics_stage_is_bitwise_identical_and_allocation_free() {
        let program = program(include_str!("../../../models/upkie/upkie.urdf"));
        let states = deterministic_states(&program, 17);
        let executor = CpuMirrorExecutor::compile(&program, states.len(), 32).unwrap();
        let fk_output = execute_states(&executor, &program, &states);
        let jacobian_output = execute_jacobians(&executor, &fk_output);
        let input = dynamics_input(&executor, &states);
        let first = execute_dynamics(&executor, &states, &fk_output, &jacobian_output);
        let mut repeated = DynamicsBatchOutput::new(executor.descriptor().layout.clone());
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(executor.execute_dynamics_into(
                    black_box(&input),
                    black_box(&fk_output),
                    black_box(&jacobian_output),
                    black_box(&mut repeated),
                ))
                .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        for agent in 0..states.len() {
            assert_dynamics_agent_bits_equal(&first, agent, &repeated, agent);
        }
    }

    #[test]
    fn compiled_point_queries_match_f64_position_jacobian_and_jdot_v() {
        for source in [
            include_str!("../../../models/toy_humanoid.urdf"),
            include_str!("../../../models/upkie/upkie.urdf"),
        ] {
            let program = program(source);
            let last = program.model.bodies.len() - 1;
            let specs = [
                PointQuerySpec {
                    stable_id: 1001,
                    frame_index: program.model.root.0,
                    point_in_frame: [0.13, -0.07, 0.19],
                },
                PointQuerySpec {
                    stable_id: 1002,
                    frame_index: 1.min(last),
                    point_in_frame: [-0.04, 0.08, 0.11],
                },
                PointQuerySpec {
                    stable_id: 1003,
                    frame_index: last / 2,
                    point_in_frame: [0.03, -0.05, -0.09],
                },
                PointQuerySpec {
                    stable_id: 1004,
                    frame_index: last,
                    point_in_frame: [0.06, 0.02, -0.03],
                },
            ];
            let states = deterministic_states(&program, 12);
            let executor =
                CpuMirrorExecutor::compile_with_point_queries(&program, states.len(), 32, &specs)
                    .unwrap();
            assert_eq!(executor.descriptor().point_queries.len(), specs.len());
            assert_eq!(
                executor
                    .descriptor()
                    .point_queries
                    .iter()
                    .map(|query| query.stable_id)
                    .collect::<Vec<_>>(),
                specs
                    .iter()
                    .map(|query| query.stable_id)
                    .collect::<Vec<_>>()
            );
            let fk_output = execute_states(&executor, &program, &states);
            let jacobian_output = execute_jacobians(&executor, &fk_output);
            let dynamics_input = dynamics_input(&executor, &states);
            let mut dynamics_output =
                DynamicsBatchOutput::new(executor.descriptor().layout.clone());
            executor
                .execute_dynamics_into(
                    &dynamics_input,
                    &fk_output,
                    &jacobian_output,
                    &mut dynamics_output,
                )
                .unwrap();
            let mut point_output = PointQueryBatchOutput::new(executor.descriptor().layout.clone());
            executor
                .execute_point_queries_into(
                    &fk_output,
                    &jacobian_output,
                    &dynamics_output,
                    &mut point_output,
                )
                .unwrap();
            let generalized_dof = program.model.dof + 6;
            let mut model_cache = ModelCache::new(&program.model);
            let mut dynamics_cache = DynamicsCache::new(&program.model);
            let mut floating_jacobian = DMatrix::zeros(3, generalized_dof);
            let mut floating_bias = DVector::zeros(generalized_dof);
            for (agent, state) in states.iter().enumerate() {
                program
                    .model
                    .forward_kinematics(state, &mut model_cache)
                    .unwrap();
                program
                    .model
                    .floating_bias_forces_into(
                        state,
                        root_twist(agent),
                        Vec3::new(0.17, -0.09, -9.73),
                        &model_cache,
                        &mut dynamics_cache,
                        &mut floating_bias,
                    )
                    .unwrap();
                assert_eq!(point_output.agent_status()[agent], AgentStatus::Ok);
                for (slot, spec) in specs.iter().copied().enumerate() {
                    let frame = FrameId(spec.frame_index);
                    let point = Vec3::from_row_slice(&spec.point_in_frame);
                    let exact_position = model_cache.world_from_body[spec.frame_index]
                        .transform_point(&Point3::from(point))
                        .coords;
                    let mirror_position = point_output.point_position(agent, slot).unwrap();
                    for component in 0..3 {
                        assert_mirror_close(
                            mirror_position[component],
                            exact_position[component],
                            "compiled point position",
                        );
                    }
                    program
                        .model
                        .floating_point_jacobian_into(
                            &model_cache,
                            frame,
                            point,
                            &mut floating_jacobian,
                        )
                        .unwrap();
                    for component in 0..3 {
                        for coordinate in 0..generalized_dof {
                            assert_mirror_close(
                                point_output
                                    .point_jacobian_value(agent, slot, component, coordinate)
                                    .unwrap(),
                                floating_jacobian[(component, coordinate)],
                                "compiled point Jacobian",
                            );
                        }
                    }
                    let exact_bias = program
                        .model
                        .point_bias_acceleration_world(frame, point, &model_cache, &dynamics_cache)
                        .unwrap();
                    let mirror_bias = point_output.point_bias_acceleration(agent, slot).unwrap();
                    for component in 0..3 {
                        assert_mirror_close(
                            mirror_bias[component],
                            exact_bias[component],
                            "compiled point Jdot-v",
                        );
                    }
                }
            }
            let (_, calls, bytes) = allocation_sentinel::measure(|| {
                for _ in 0..100 {
                    black_box(executor.execute_point_queries_into(
                        black_box(&fk_output),
                        black_box(&jacobian_output),
                        black_box(&dynamics_output),
                        black_box(&mut point_output),
                    ))
                    .unwrap();
                }
            });
            assert_eq!((calls, bytes), (0, 0));
        }
    }

    #[test]
    fn fixed_point_tasks_and_contact_locks_emit_stable_masked_rows_without_allocation() {
        let program = program(include_str!("../../../models/upkie/upkie.urdf"));
        let last = program.model.bodies.len() - 1;
        let point_queries = [
            PointQuerySpec {
                stable_id: 1001,
                frame_index: 1,
                point_in_frame: [0.11, -0.04, 0.17],
            },
            PointQuerySpec {
                stable_id: 1002,
                frame_index: 20.min(last),
                point_in_frame: [0.02, 0.0, -0.01],
            },
            PointQuerySpec {
                stable_id: 1003,
                frame_index: 38.min(last),
                point_in_frame: [-0.02, 0.0, -0.01],
            },
        ];
        let point_tasks = [
            PointAttractorSpec {
                stable_id: 2001,
                point_query_stable_id: 1001,
                priority: Priority::Intent,
                weight: 1.5,
                bandwidth_hz: 2.25,
            },
            PointAttractorSpec {
                stable_id: 2002,
                point_query_stable_id: 1002,
                priority: Priority::Viability,
                weight: 0.75,
                bandwidth_hz: 1.5,
            },
        ];
        let contacts = [
            ContactLockSpec {
                stable_id: 3001,
                point_query_stable_id: 1002,
            },
            ContactLockSpec {
                stable_id: 3002,
                point_query_stable_id: 1003,
            },
        ];
        let states = deterministic_states(&program, 12);
        let executor = CpuMirrorExecutor::compile_with_emission_plan(
            &program,
            states.len(),
            32,
            &point_queries,
            &point_tasks,
            &contacts,
        )
        .unwrap();
        assert_eq!(executor.descriptor().layout.point_task_count, 2);
        assert_eq!(executor.descriptor().layout.contact_lock_count, 2);
        assert_eq!(executor.descriptor().point_tasks[0].stable_id, 2001);
        assert_eq!(
            executor.descriptor().point_tasks[0].priority,
            Priority::Intent
        );
        assert_eq!(executor.descriptor().point_tasks[0].weight(), 1.5);
        assert_eq!(executor.descriptor().contact_locks[1].stable_id, 3002);
        assert!(
            executor
                .descriptor()
                .manifest
                .supports(KernelStage::TaskEmission)
        );
        assert!(
            executor
                .descriptor()
                .manifest
                .supports(KernelStage::ConstraintEmission)
        );
        let fk_output = execute_states(&executor, &program, &states);
        let jacobian_output = execute_jacobians(&executor, &fk_output);
        let dynamics_input = dynamics_input(&executor, &states);
        let mut dynamics_output = DynamicsBatchOutput::new(executor.descriptor().layout.clone());
        executor
            .execute_dynamics_into(
                &dynamics_input,
                &fk_output,
                &jacobian_output,
                &mut dynamics_output,
            )
            .unwrap();
        let mut points = PointQueryBatchOutput::new(executor.descriptor().layout.clone());
        executor
            .execute_point_queries_into(&fk_output, &jacobian_output, &dynamics_output, &mut points)
            .unwrap();
        let layout = executor.descriptor().layout.clone();
        let mut input = EmissionBatchInput::new(layout.clone(), states.len()).unwrap();
        for agent in 0..states.len() {
            for slot in 0..2 {
                let active = slot == 0 || agent % 2 == 0;
                input.point_task_active_soa_mut()[layout.task_active_index(slot, agent)] =
                    u8::from(active);
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    if active {
                        let query_slot = executor.descriptor().point_tasks[slot].point_query_slot;
                        input.point_target_position_soa_mut()[index] =
                            points.point_position(agent, query_slot).unwrap()[component]
                                + 0.01 * (slot + component + 1) as f32;
                        input.point_target_velocity_soa_mut()[index] =
                            0.03 * (agent + component + 1) as f32;
                        input.point_target_acceleration_soa_mut()[index] =
                            -0.02 * (slot + component + 1) as f32;
                    } else {
                        input.point_target_position_soa_mut()[index] = f32::NAN;
                        input.point_target_velocity_soa_mut()[index] = f32::NAN;
                        input.point_target_acceleration_soa_mut()[index] = f32::NAN;
                    }
                }
            }
            for slot in 0..2 {
                let active = slot == 0 || agent % 3 == 0;
                input.contact_lock_active_soa_mut()[layout.contact_active_index(slot, agent)] =
                    u8::from(active);
                for component in 0..3 {
                    input.contact_desired_acceleration_soa_mut()
                        [layout.contact_vector_index(slot, component, agent)] = if active {
                        0.01 * (slot + component) as f32
                    } else {
                        f32::NAN
                    };
                }
            }
        }
        let mut output = EmissionBatchOutput::new(layout.clone());
        executor
            .execute_emission_into(&input, &dynamics_input, &points, &mut output)
            .unwrap();
        let generalized_dof = layout.generalized_coordinate_count;
        for agent in 0..states.len() {
            assert_eq!(output.agent_status()[agent], AgentStatus::Ok);
            for (slot, task) in executor
                .descriptor()
                .point_tasks
                .iter()
                .copied()
                .enumerate()
            {
                let active = slot == 0 || agent % 2 == 0;
                assert_eq!(
                    output.point_task_active_soa()[layout.task_active_index(slot, agent)],
                    u8::from(active)
                );
                for component in 0..3 {
                    let vector_index = layout.task_vector_index(slot, component, agent);
                    if active {
                        let position_error = output.point_task_position_error_soa()[vector_index];
                        assert_mirror_close(
                            position_error,
                            0.01 * (slot + component + 1) as f64,
                            "point task position error",
                        );
                        let desired = output.point_task_desired_acceleration_soa()[vector_index];
                        let rhs = output.point_task_rhs_soa()[vector_index];
                        let bias = points
                            .point_bias_acceleration(agent, task.point_query_slot)
                            .unwrap()[component];
                        assert_mirror_close(
                            rhs,
                            desired as f64 - bias as f64,
                            "point task Jdot-v subtraction",
                        );
                        for coordinate in 0..generalized_dof {
                            assert_eq!(
                                output.point_task_jacobian_soa()[layout
                                    .task_jacobian_index(slot, component, coordinate, agent,)]
                                .to_bits(),
                                points
                                    .point_jacobian_value(
                                        agent,
                                        task.point_query_slot,
                                        component,
                                        coordinate,
                                    )
                                    .unwrap()
                                    .to_bits()
                            );
                        }
                    } else {
                        assert_eq!(output.point_task_position_error_soa()[vector_index], 0.0);
                        assert_eq!(output.point_task_velocity_error_soa()[vector_index], 0.0);
                        assert_eq!(
                            output.point_task_desired_acceleration_soa()[vector_index],
                            0.0
                        );
                        assert_eq!(output.point_task_rhs_soa()[vector_index], 0.0);
                    }
                }
            }
            for (slot, contact) in executor
                .descriptor()
                .contact_locks
                .iter()
                .copied()
                .enumerate()
            {
                let active = slot == 0 || agent % 3 == 0;
                assert_eq!(
                    output.contact_lock_active_soa()[layout.contact_active_index(slot, agent)],
                    u8::from(active)
                );
                for component in 0..3 {
                    let vector_index = layout.contact_vector_index(slot, component, agent);
                    if active {
                        let desired = 0.01 * (slot + component) as f32;
                        let bias = points
                            .point_bias_acceleration(agent, contact.point_query_slot)
                            .unwrap()[component];
                        assert_mirror_close(
                            output.contact_lock_rhs_soa()[vector_index],
                            desired as f64 - bias as f64,
                            "contact Jdot-v subtraction",
                        );
                    } else {
                        assert_eq!(output.contact_lock_rhs_soa()[vector_index], 0.0);
                    }
                }
            }
        }
        let valid_output = output.clone();
        let poisoned_agent = 4;
        let neighbor = 5;
        let poisoned_index = layout.task_vector_index(0, 0, poisoned_agent);
        let original_target = input.point_target_position_soa_mut()[poisoned_index];
        input.point_target_position_soa_mut()[poisoned_index] = f32::NAN;
        executor
            .execute_emission_into(&input, &dynamics_input, &points, &mut output)
            .unwrap();
        assert_eq!(
            output.agent_status()[poisoned_agent],
            AgentStatus::InvalidInput
        );
        for slot in 0..layout.point_task_count {
            assert_eq!(
                output.point_task_active_soa()[layout.task_active_index(slot, poisoned_agent)],
                0
            );
            for component in 0..3 {
                assert_eq!(
                    output.point_task_rhs_soa()
                        [layout.task_vector_index(slot, component, poisoned_agent)],
                    0.0
                );
            }
        }
        assert_eq!(output.agent_status()[neighbor], AgentStatus::Ok);
        for slot in 0..layout.point_task_count {
            for component in 0..3 {
                let index = layout.task_vector_index(slot, component, neighbor);
                assert_eq!(
                    output.point_task_rhs_soa()[index].to_bits(),
                    valid_output.point_task_rhs_soa()[index].to_bits()
                );
            }
        }
        for slot in 0..layout.contact_lock_count {
            for component in 0..3 {
                let index = layout.contact_vector_index(slot, component, neighbor);
                assert_eq!(
                    output.contact_lock_rhs_soa()[index].to_bits(),
                    valid_output.contact_lock_rhs_soa()[index].to_bits()
                );
            }
        }
        input.point_target_position_soa_mut()[poisoned_index] = original_target;
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(executor.execute_emission_into(
                    black_box(&input),
                    black_box(&dynamics_input),
                    black_box(&points),
                    black_box(&mut output),
                ))
                .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
    }

    #[test]
    fn repeated_jacobian_stage_is_bitwise_identical_and_allocation_free() {
        let program = program(include_str!("../../../models/upkie/upkie.urdf"));
        let states = deterministic_states(&program, 17);
        let executor = CpuMirrorExecutor::compile(&program, states.len(), 32).unwrap();
        let fk_output = execute_states(&executor, &program, &states);
        let first = execute_jacobians(&executor, &fk_output);
        let mut repeated = JacobianBatchOutput::new(executor.descriptor().layout.clone());
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(
                    executor
                        .execute_jacobians_into(black_box(&fk_output), black_box(&mut repeated)),
                )
                .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        for agent in 0..states.len() {
            assert_jacobian_agent_bits_equal(&first, agent, &repeated, agent);
        }
    }

    #[test]
    fn repeated_batch_is_bitwise_identical_and_allocation_free() {
        let program = program(include_str!("../../../models/toy_humanoid.urdf"));
        let states = deterministic_states(&program, 16);
        let executor = CpuMirrorExecutor::compile(&program, 16, 32).unwrap();
        let mut input = FkBatchInput::new(executor.descriptor().layout.clone(), 16).unwrap();
        for (agent, state) in states.iter().enumerate() {
            input
                .set_agent_from_state(&program.model, agent, state)
                .unwrap();
        }
        let mut first = FkBatchOutput::new(executor.descriptor().layout.clone());
        let mut second = FkBatchOutput::new(executor.descriptor().layout.clone());
        executor.execute_into(&input, &mut first).unwrap();
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(executor.execute_into(black_box(&input), black_box(&mut second)))
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        for agent in 0..states.len() {
            assert_agent_bits_equal(&first, agent, &second, agent);
        }
    }

    #[test]
    fn permutation_padding_and_chunking_only_reindex_agents() {
        let program = program(include_str!("../../../models/toy_humanoid.urdf"));
        let states = deterministic_states(&program, 5);
        let compact_executor = CpuMirrorExecutor::compile(&program, 5, 1).unwrap();
        let compact = execute_states(&compact_executor, &program, &states);

        let permutation = [3, 0, 4, 1, 2];
        let permuted_states: Vec<_> = permutation
            .iter()
            .map(|index| states[*index].clone())
            .collect();
        let permuted = execute_states(&compact_executor, &program, &permuted_states);
        for (permuted_agent, source_agent) in permutation.into_iter().enumerate() {
            assert_agent_bits_equal(&compact, source_agent, &permuted, permuted_agent);
        }

        let padded_executor = CpuMirrorExecutor::compile(&program, 9, 8).unwrap();
        let mut padded_input =
            FkBatchInput::new(padded_executor.descriptor().layout.clone(), states.len()).unwrap();
        for (agent, state) in states.iter().enumerate() {
            padded_input
                .set_agent_from_state(&program.model, agent, state)
                .unwrap();
        }
        for coordinate in 0..padded_input.layout().coordinate_count {
            for agent in states.len()..padded_input.layout().agent_stride {
                let index = padded_input.layout().q_index(coordinate, agent);
                padded_input.q_soa_mut()[index] = f32::from_bits(0x7fc0_1234);
            }
        }
        let mut padded = FkBatchOutput::new(padded_executor.descriptor().layout.clone());
        padded_executor
            .execute_into(&padded_input, &mut padded)
            .unwrap();
        for agent in 0..states.len() {
            assert_agent_bits_equal(&compact, agent, &padded, agent);
        }
        assert!(
            padded.agent_status()[states.len()..]
                .iter()
                .all(|status| *status == AgentStatus::Inactive)
        );

        let chunk_executor = CpuMirrorExecutor::compile(&program, 3, 4).unwrap();
        let first_chunk = execute_states(&chunk_executor, &program, &states[..2]);
        let second_chunk = execute_states(&chunk_executor, &program, &states[2..]);
        for agent in 0..2 {
            assert_agent_bits_equal(&compact, agent, &first_chunk, agent);
        }
        for agent in 2..states.len() {
            assert_agent_bits_equal(&compact, agent, &second_chunk, agent - 2);
        }
    }

    #[test]
    fn invalid_agent_is_typed_and_cannot_poison_its_neighbor() {
        let program = program(include_str!("../../../models/toy_humanoid.urdf"));
        let states = deterministic_states(&program, 2);
        let executor = CpuMirrorExecutor::compile(&program, 2, 2).unwrap();
        let valid = execute_states(&executor, &program, &states);
        let mut input = FkBatchInput::new(executor.descriptor().layout.clone(), 2).unwrap();
        for (agent, state) in states.iter().enumerate() {
            input
                .set_agent_from_state(&program.model, agent, state)
                .unwrap();
        }
        let invalid_index = input.layout().q_index(0, 1);
        input.q_soa_mut()[invalid_index] = f32::NAN;
        let mut output = FkBatchOutput::new(executor.descriptor().layout.clone());
        executor.execute_into(&input, &mut output).unwrap();
        assert_agent_bits_equal(&valid, 0, &output, 0);
        assert_eq!(output.agent_status()[1], AgentStatus::InvalidInput);
        assert!(
            (0..program.model.bodies.len())
                .flat_map(|body| output.body_pose(1, body).unwrap())
                .all(|value| value == 0.0)
        );
    }
}
