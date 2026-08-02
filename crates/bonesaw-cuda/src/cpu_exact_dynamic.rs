//! Allocation-free fixed-layout batch adapter for the established floating
//! dynamic WBC.
//!
//! The semantic decision vector is `[qdd, generalized joint effort, point
//! contact force]`. Dynamics, kinematic contact, actuator-space effort,
//! unilateral force, and friction are hard constraints. This remains a CPU
//! exact reference path; it is not part of the `CpuMirrorF32` manifest.

use bonesaw_core::{
    ActuatorEffortInput, ContactMode, ContactSpec, DynamicWbcConfig, FLOATING_POINT_TASK_CAPACITY,
    FloatingDynamicWbc, FloatingDynamicWbcInput, FloatingDynamicWbcOutput,
    FloatingDynamicWbcScratch, FloatingPointAccelerationTask, FloatingTaskPriorities,
    FloatingTaskWeights, FrameId, Motion6, MotionProgram, RobotState, SupportPatchSpec, Transform3,
    Vec3, VelocityBounds,
};
use nalgebra::{DVector, Matrix3, Rotation3, Translation3, UnitQuaternion};
use thiserror::Error;

use crate::{
    AgentStatus, BatchInputError, BatchLayout, BatchProgramDescriptor, ContactKinematicMode,
    DynamicsBatchInput, EmissionBatchInput, EmissionBatchOutput, ExactSolveAgentStatus,
    FkBatchInput, JointEnvelopeAgentStatus, JointEnvelopeBatchOutput, ROOT_TANGENT_COMPONENTS,
};

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum ExactDynamicError {
    #[error("program does not match the dynamic batch descriptor")]
    ProgramMismatch,
    #[error("dynamic batch input or output layout mismatch")]
    LayoutMismatch,
    #[error("dynamic batch supports at most {FLOATING_POINT_TASK_CAPACITY} point tasks")]
    PointTaskCapacity,
    #[error("failed to construct the floating dynamic WBC")]
    InvalidConfiguration,
    #[error("support patch {0} is invalid for the fixed contact declaration")]
    InvalidSupportPatch(u32),
}

/// Construction-time topology for a finite support polygon in the exact CPU
/// batch path. Contact slots are fixed by the emission plan; only enablement
/// and inward margin vary per agent at runtime.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactDynamicSupportPatchSpec {
    pub stable_id: u32,
    pub first_contact_slot: usize,
    pub contact_count: usize,
}

#[derive(Clone, Copy, Debug)]
struct ResolvedSupportPatch {
    stable_id: u32,
    first_contact_order_index: usize,
    contact_count: usize,
}

#[derive(Clone, Debug)]
pub struct ExactDynamicBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    acceleration_lower_soa: Vec<f64>,
    acceleration_upper_soa: Vec<f64>,
    actuator_effort_lower_soa: Vec<f64>,
    actuator_effort_upper_soa: Vec<f64>,
    friction_coefficient_soa: Vec<f64>,
    minimum_normal_force_soa: Vec<f64>,
    maximum_normal_force_soa: Vec<f64>,
    nominal_normal_force_soa: Vec<f64>,
    support_patch_enabled_soa: Vec<u8>,
    support_patch_minimum_margin_soa: Vec<f64>,
}

impl ExactDynamicBatchInput {
    pub fn new(
        program: &MotionProgram,
        layout: BatchLayout,
        active_agents: usize,
    ) -> Result<Self, BatchInputError> {
        Self::new_with_support_patch_count(program, layout, active_agents, 0)
    }

    pub fn new_with_support_patch_count(
        program: &MotionProgram,
        layout: BatchLayout,
        active_agents: usize,
        support_patch_count: usize,
    ) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        let generalized_len = layout
            .generalized_velocity_len()
            .expect("validated batch layout");
        let actuator_len = program
            .actuation
            .actuators
            .len()
            .checked_mul(layout.agent_stride)
            .expect("validated batch layout");
        let contact_len = layout
            .contact_lock_count
            .checked_mul(layout.agent_stride)
            .expect("validated batch layout");
        let patch_len = support_patch_count
            .checked_mul(layout.agent_stride)
            .expect("validated batch layout");
        let mut input = Self {
            acceleration_lower_soa: vec![f64::NEG_INFINITY; generalized_len],
            acceleration_upper_soa: vec![f64::INFINITY; generalized_len],
            actuator_effort_lower_soa: vec![f64::NEG_INFINITY; actuator_len],
            actuator_effort_upper_soa: vec![f64::INFINITY; actuator_len],
            friction_coefficient_soa: vec![0.7; contact_len],
            minimum_normal_force_soa: vec![0.0; contact_len],
            maximum_normal_force_soa: vec![f64::INFINITY; contact_len],
            nominal_normal_force_soa: vec![0.0; contact_len],
            support_patch_enabled_soa: vec![0; patch_len],
            support_patch_minimum_margin_soa: vec![0.0; patch_len],
            layout,
            active_agents,
        };
        for (actuator, spec) in program.actuation.actuators.iter().enumerate() {
            for agent in 0..input.layout.agent_stride {
                let index = actuator * input.layout.agent_stride + agent;
                input.actuator_effort_lower_soa[index] = -spec.limits.effort;
                input.actuator_effort_upper_soa[index] = spec.limits.effort;
            }
        }
        Ok(input)
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub const fn active_agents(&self) -> usize {
        self.active_agents
    }
    pub fn acceleration_lower_soa(&self) -> &[f64] {
        &self.acceleration_lower_soa
    }
    pub fn acceleration_lower_soa_mut(&mut self) -> &mut [f64] {
        &mut self.acceleration_lower_soa
    }
    pub fn acceleration_upper_soa(&self) -> &[f64] {
        &self.acceleration_upper_soa
    }
    pub fn acceleration_upper_soa_mut(&mut self) -> &mut [f64] {
        &mut self.acceleration_upper_soa
    }
    pub fn actuator_effort_lower_soa(&self) -> &[f64] {
        &self.actuator_effort_lower_soa
    }
    pub fn actuator_effort_lower_soa_mut(&mut self) -> &mut [f64] {
        &mut self.actuator_effort_lower_soa
    }
    pub fn actuator_effort_upper_soa(&self) -> &[f64] {
        &self.actuator_effort_upper_soa
    }
    pub fn actuator_effort_upper_soa_mut(&mut self) -> &mut [f64] {
        &mut self.actuator_effort_upper_soa
    }
    pub fn friction_coefficient_soa(&self) -> &[f64] {
        &self.friction_coefficient_soa
    }
    pub fn friction_coefficient_soa_mut(&mut self) -> &mut [f64] {
        &mut self.friction_coefficient_soa
    }
    pub fn minimum_normal_force_soa(&self) -> &[f64] {
        &self.minimum_normal_force_soa
    }
    pub fn minimum_normal_force_soa_mut(&mut self) -> &mut [f64] {
        &mut self.minimum_normal_force_soa
    }
    pub fn maximum_normal_force_soa(&self) -> &[f64] {
        &self.maximum_normal_force_soa
    }
    pub fn maximum_normal_force_soa_mut(&mut self) -> &mut [f64] {
        &mut self.maximum_normal_force_soa
    }
    pub fn nominal_normal_force_soa(&self) -> &[f64] {
        &self.nominal_normal_force_soa
    }
    pub fn nominal_normal_force_soa_mut(&mut self) -> &mut [f64] {
        &mut self.nominal_normal_force_soa
    }
    pub fn support_patch_enabled_soa(&self) -> &[u8] {
        &self.support_patch_enabled_soa
    }
    pub fn support_patch_enabled_soa_mut(&mut self) -> &mut [u8] {
        &mut self.support_patch_enabled_soa
    }
    pub fn support_patch_minimum_margin_soa(&self) -> &[f64] {
        &self.support_patch_minimum_margin_soa
    }
    pub fn support_patch_minimum_margin_soa_mut(&mut self) -> &mut [f64] {
        &mut self.support_patch_minimum_margin_soa
    }
    pub fn support_patch_count(&self) -> usize {
        if self.layout.agent_stride == 0 {
            0
        } else {
            self.support_patch_enabled_soa.len() / self.layout.agent_stride
        }
    }
}

#[derive(Clone, Debug)]
pub struct ExactDynamicBatchOutput {
    layout: BatchLayout,
    actuator_count: usize,
    status: Vec<ExactSolveAgentStatus>,
    generalized_acceleration_soa: Vec<f64>,
    generalized_effort_soa: Vec<f64>,
    contact_force_basis_soa: Vec<f64>,
    contact_force_world_soa: Vec<f64>,
    active_contacts: Vec<u32>,
    dynamics_residual_linf: Vec<f64>,
    contact_residual_linf: Vec<f64>,
    minimum_friction_margin: Vec<f64>,
    minimum_support_margin_m: Vec<f64>,
    limiting_support_patch: Vec<u32>,
    active_support_patches: Vec<u32>,
    minimum_actuator_effort_margin: Vec<f64>,
    limiting_actuator: Vec<u32>,
    minimum_bound_margin: Vec<f64>,
    maximum_hard_violation: Vec<f64>,
    clipped_steps: Vec<u32>,
    minimum_joint_envelope_margin: Vec<f64>,
    limiting_joint_envelope_coordinate: Vec<u32>,
    task_l2_soa: Vec<f64>,
    task_clipped_soa: Vec<u8>,
}

impl ExactDynamicBatchOutput {
    pub fn new(layout: BatchLayout, actuator_count: usize) -> Self {
        let stride = layout.agent_stride;
        Self {
            status: vec![ExactSolveAgentStatus::Inactive; stride],
            generalized_acceleration_soa: vec![
                0.0;
                layout.generalized_velocity_len().expect("layout")
            ],
            generalized_effort_soa: vec![0.0; actuator_count * stride],
            contact_force_basis_soa: vec![0.0; layout.contact_lock_count * 3 * stride],
            contact_force_world_soa: vec![0.0; layout.contact_lock_count * 3 * stride],
            active_contacts: vec![0; stride],
            dynamics_residual_linf: vec![f64::INFINITY; stride],
            contact_residual_linf: vec![f64::INFINITY; stride],
            minimum_friction_margin: vec![f64::NEG_INFINITY; stride],
            minimum_support_margin_m: vec![f64::INFINITY; stride],
            limiting_support_patch: vec![u32::MAX; stride],
            active_support_patches: vec![0; stride],
            minimum_actuator_effort_margin: vec![f64::INFINITY; stride],
            limiting_actuator: vec![u32::MAX; stride],
            minimum_bound_margin: vec![f64::NEG_INFINITY; stride],
            maximum_hard_violation: vec![f64::INFINITY; stride],
            clipped_steps: vec![0; stride],
            minimum_joint_envelope_margin: vec![f64::INFINITY; stride],
            limiting_joint_envelope_coordinate: vec![u32::MAX; stride],
            task_l2_soa: vec![0.0; layout.point_task_count * stride],
            task_clipped_soa: vec![0; layout.point_task_count * stride],
            layout,
            actuator_count,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub const fn actuator_count(&self) -> usize {
        self.actuator_count
    }
    pub fn status(&self) -> &[ExactSolveAgentStatus] {
        &self.status
    }
    pub fn generalized_acceleration_soa(&self) -> &[f64] {
        &self.generalized_acceleration_soa
    }
    pub fn generalized_effort_soa(&self) -> &[f64] {
        &self.generalized_effort_soa
    }
    pub fn contact_force_basis_soa(&self) -> &[f64] {
        &self.contact_force_basis_soa
    }
    pub fn contact_force_world_soa(&self) -> &[f64] {
        &self.contact_force_world_soa
    }
    pub fn active_contacts(&self) -> &[u32] {
        &self.active_contacts
    }
    pub fn dynamics_residual_linf(&self) -> &[f64] {
        &self.dynamics_residual_linf
    }
    pub fn contact_residual_linf(&self) -> &[f64] {
        &self.contact_residual_linf
    }
    pub fn minimum_friction_margin(&self) -> &[f64] {
        &self.minimum_friction_margin
    }
    pub fn minimum_support_margin_m(&self) -> &[f64] {
        &self.minimum_support_margin_m
    }
    pub fn limiting_support_patch(&self) -> &[u32] {
        &self.limiting_support_patch
    }
    pub fn active_support_patches(&self) -> &[u32] {
        &self.active_support_patches
    }
    pub fn minimum_actuator_effort_margin(&self) -> &[f64] {
        &self.minimum_actuator_effort_margin
    }
    pub fn limiting_actuator(&self) -> &[u32] {
        &self.limiting_actuator
    }
    pub fn minimum_bound_margin(&self) -> &[f64] {
        &self.minimum_bound_margin
    }
    pub fn maximum_hard_violation(&self) -> &[f64] {
        &self.maximum_hard_violation
    }
    pub fn clipped_steps(&self) -> &[u32] {
        &self.clipped_steps
    }
    pub fn minimum_joint_envelope_margin(&self) -> &[f64] {
        &self.minimum_joint_envelope_margin
    }
    pub fn limiting_joint_envelope_coordinate(&self) -> &[u32] {
        &self.limiting_joint_envelope_coordinate
    }
    pub fn task_l2_soa(&self) -> &[f64] {
        &self.task_l2_soa
    }
    pub fn task_clipped_soa(&self) -> &[u8] {
        &self.task_clipped_soa
    }

    fn clear(&mut self) {
        self.status.fill(ExactSolveAgentStatus::Inactive);
        self.generalized_acceleration_soa.fill(0.0);
        self.generalized_effort_soa.fill(0.0);
        self.contact_force_basis_soa.fill(0.0);
        self.contact_force_world_soa.fill(0.0);
        self.active_contacts.fill(0);
        self.dynamics_residual_linf.fill(f64::INFINITY);
        self.contact_residual_linf.fill(f64::INFINITY);
        self.minimum_friction_margin.fill(f64::NEG_INFINITY);
        self.minimum_support_margin_m.fill(f64::INFINITY);
        self.limiting_support_patch.fill(u32::MAX);
        self.active_support_patches.fill(0);
        self.minimum_actuator_effort_margin.fill(f64::INFINITY);
        self.limiting_actuator.fill(u32::MAX);
        self.minimum_bound_margin.fill(f64::NEG_INFINITY);
        self.maximum_hard_violation.fill(f64::INFINITY);
        self.clipped_steps.fill(0);
        self.minimum_joint_envelope_margin.fill(f64::INFINITY);
        self.limiting_joint_envelope_coordinate.fill(u32::MAX);
        self.task_l2_soa.fill(0.0);
        self.task_clipped_soa.fill(0);
    }
}

#[derive(Debug)]
struct AgentScratch {
    state: RobotState,
    desired_generalized_acceleration: DVector<f64>,
    acceleration_bounds: VelocityBounds,
    torque_bounds: VelocityBounds,
    actuator_effort_bounds: VelocityBounds,
    point_tasks: Vec<FloatingPointAccelerationTask>,
    contacts: Vec<ContactSpec>,
    support_patches: Vec<SupportPatchSpec>,
    output: FloatingDynamicWbcOutput,
    scratch: FloatingDynamicWbcScratch,
}

pub struct CpuExactDynamicBatchSolver {
    descriptor: BatchProgramDescriptor,
    actuation: bonesaw_core::CompiledActuation,
    solver: FloatingDynamicWbc,
    point_task_order: Vec<usize>,
    contact_order: Vec<usize>,
    support_patches: Vec<ResolvedSupportPatch>,
    scratch: Vec<AgentScratch>,
}

impl CpuExactDynamicBatchSolver {
    pub fn new(
        program: &MotionProgram,
        descriptor: &BatchProgramDescriptor,
    ) -> Result<Self, ExactDynamicError> {
        Self::new_with_support_patches(program, descriptor, &[])
    }

    pub fn new_with_support_patches(
        program: &MotionProgram,
        descriptor: &BatchProgramDescriptor,
        support_patch_specs: &[ExactDynamicSupportPatchSpec],
    ) -> Result<Self, ExactDynamicError> {
        if descriptor.layout.program_fingerprint != program.header.fingerprint_sha256
            || descriptor.layout.coordinate_count != program.model.dof
            || program.actuation.actuators.len() != program.model.dof
        {
            return Err(ExactDynamicError::ProgramMismatch);
        }
        if descriptor.point_tasks.len() > FLOATING_POINT_TASK_CAPACITY {
            return Err(ExactDynamicError::PointTaskCapacity);
        }
        let solver = FloatingDynamicWbc::new(program.model.clone(), DynamicWbcConfig::default())
            .map_err(|_| ExactDynamicError::InvalidConfiguration)?;
        let mut point_task_order: Vec<_> = (0..descriptor.point_tasks.len()).collect();
        point_task_order.sort_unstable_by_key(|slot| descriptor.point_tasks[*slot].stable_id);
        let mut contact_order: Vec<_> = (0..descriptor.contact_locks.len()).collect();
        contact_order.sort_unstable_by_key(|slot| descriptor.contact_locks[*slot].stable_id);
        let mut support_patches = Vec::with_capacity(support_patch_specs.len());
        let mut previous_end = 0usize;
        let mut previous_stable_id = None;
        for spec in support_patch_specs.iter().copied() {
            let end = spec.first_contact_slot.saturating_add(spec.contact_count);
            let ordered_positions = contact_order
                .iter()
                .enumerate()
                .filter_map(|(position, slot)| {
                    (*slot >= spec.first_contact_slot && *slot < end).then_some(position)
                })
                .collect::<Vec<_>>();
            let contiguous = ordered_positions.len() == spec.contact_count
                && ordered_positions
                    .windows(2)
                    .all(|pair| pair[1] == pair[0] + 1);
            let first_order = ordered_positions.first().copied().unwrap_or(usize::MAX);
            if spec.stable_id > 0x00ff_ffff
                || spec.contact_count < 3
                || spec.contact_count > 16
                || end > descriptor.contact_locks.len()
                || !contiguous
                || first_order < previous_end
                || previous_stable_id.is_some_and(|id| id >= spec.stable_id)
            {
                return Err(ExactDynamicError::InvalidSupportPatch(spec.stable_id));
            }
            previous_end = first_order + spec.contact_count;
            previous_stable_id = Some(spec.stable_id);
            support_patches.push(ResolvedSupportPatch {
                stable_id: spec.stable_id,
                first_contact_order_index: first_order,
                contact_count: spec.contact_count,
            });
        }
        let dof = program.model.dof;
        let generalized_dof = dof + ROOT_TANGENT_COMPONENTS;
        let maximum_contacts = descriptor.contact_locks.len();
        let maximum_constraints = generalized_dof
            .saturating_add(maximum_contacts.saturating_mul(8))
            .saturating_add(
                support_patch_specs
                    .iter()
                    .map(|patch| patch.contact_count)
                    .sum::<usize>(),
            )
            .saturating_add(program.actuation.actuators.len());
        let mut scratch = Vec::with_capacity(descriptor.layout.agent_capacity);
        for _ in 0..descriptor.layout.agent_capacity {
            scratch.push(AgentScratch {
                state: RobotState::zeros(&program.model),
                desired_generalized_acceleration: DVector::zeros(generalized_dof),
                acceleration_bounds: VelocityBounds::unbounded(generalized_dof),
                torque_bounds: VelocityBounds::unbounded(dof),
                actuator_effort_bounds: VelocityBounds::unbounded(
                    program.actuation.actuators.len(),
                ),
                point_tasks: Vec::with_capacity(descriptor.point_tasks.len()),
                contacts: Vec::with_capacity(maximum_contacts),
                support_patches: Vec::with_capacity(support_patch_specs.len()),
                output: FloatingDynamicWbcOutput::workspace(
                    dof,
                    maximum_contacts,
                    maximum_constraints,
                ),
                scratch: FloatingDynamicWbcScratch::new_with_actuator_capacity(
                    &program.model,
                    maximum_contacts,
                    program.actuation.actuators.len(),
                ),
            });
        }
        Ok(Self {
            descriptor: descriptor.clone(),
            actuation: program.actuation.clone(),
            solver,
            point_task_order,
            contact_order,
            support_patches,
            scratch,
        })
    }

    pub fn execute_into(
        &mut self,
        state: &FkBatchInput,
        dynamics: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        emission: &EmissionBatchOutput,
        input: &ExactDynamicBatchInput,
        output: &mut ExactDynamicBatchOutput,
    ) -> Result<(), ExactDynamicError> {
        self.execute_impl(
            state,
            dynamics,
            emission_input,
            emission,
            input,
            None,
            output,
        )
    }

    pub fn execute_with_joint_envelope_into(
        &mut self,
        state: &FkBatchInput,
        dynamics: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        emission: &EmissionBatchOutput,
        input: &ExactDynamicBatchInput,
        joint_envelope: &JointEnvelopeBatchOutput,
        output: &mut ExactDynamicBatchOutput,
    ) -> Result<(), ExactDynamicError> {
        self.execute_impl(
            state,
            dynamics,
            emission_input,
            emission,
            input,
            Some(joint_envelope),
            output,
        )
    }

    fn execute_impl(
        &mut self,
        state: &FkBatchInput,
        dynamics: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        emission: &EmissionBatchOutput,
        input: &ExactDynamicBatchInput,
        joint_envelope: Option<&JointEnvelopeBatchOutput>,
        output: &mut ExactDynamicBatchOutput,
    ) -> Result<(), ExactDynamicError> {
        let layout = &self.descriptor.layout;
        if state.layout() != layout
            || dynamics.layout() != layout
            || emission_input.layout() != layout
            || emission.layout() != layout
            || input.layout() != layout
            || input.support_patch_count() != self.support_patches.len()
            || output.layout() != layout
            || output.actuator_count() != self.actuation.actuators.len()
            || joint_envelope.is_some_and(|envelope| envelope.layout() != layout)
        {
            return Err(ExactDynamicError::LayoutMismatch);
        }
        output.clear();
        let stride = layout.agent_stride;
        let dof = layout.coordinate_count;
        let generalized_dof = layout.generalized_coordinate_count;
        for agent in 0..layout.agent_capacity {
            if agent >= input.active_agents()
                || emission.agent_status()[agent] == AgentStatus::Inactive
            {
                continue;
            }
            if emission.agent_status()[agent] == AgentStatus::InvalidInput {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            if joint_envelope.is_some_and(|envelope| {
                matches!(
                    envelope.status()[agent],
                    JointEnvelopeAgentStatus::Inactive | JointEnvelopeAgentStatus::InvalidInput
                )
            }) {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            let Some(root_pose) = load_root_pose(state, agent) else {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            };
            let gravity_ok =
                [0.0_f32, 0.0, -9.81]
                    .into_iter()
                    .enumerate()
                    .all(|(axis, expected)| {
                        dynamics.gravity_world_soa()[axis * stride + agent] == expected
                    });
            if !gravity_ok {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            let scratch = &mut self.scratch[agent];
            scratch.state.control_world_from_root = root_pose;
            let mut valid = true;
            for coordinate in 0..dof {
                let q = state.q_soa()[layout.q_index(coordinate, agent)] as f64;
                let v = dynamics.generalized_velocity_soa()
                    [layout.generalized_index(ROOT_TANGENT_COMPONENTS + coordinate, agent)]
                    as f64;
                valid &= q.is_finite() && v.is_finite();
                scratch.state.q[coordinate] = q;
                scratch.state.v[coordinate] = v;
            }
            let mut root_twist = Motion6::default();
            for coordinate in 0..ROOT_TANGENT_COMPONENTS {
                let value = dynamics.generalized_velocity_soa()
                    [layout.generalized_index(coordinate, agent)]
                    as f64;
                valid &= value.is_finite();
                root_twist.0[coordinate] = value;
            }
            let mut invalid_problem = false;
            for coordinate in 0..generalized_dof {
                let index = layout.generalized_index(coordinate, agent);
                let mut lower = input.acceleration_lower_soa()[index];
                let mut upper = input.acceleration_upper_soa()[index];
                valid &= !lower.is_nan() && !upper.is_nan() && lower <= upper;
                if let Some(envelope) = joint_envelope
                    && envelope.status()[agent] == JointEnvelopeAgentStatus::Ok
                {
                    lower = lower.max(envelope.lower_soa()[index]);
                    upper = upper.min(envelope.upper_soa()[index]);
                    invalid_problem |= lower > upper;
                }
                scratch.acceleration_bounds.lower[coordinate] = lower;
                scratch.acceleration_bounds.upper[coordinate] = upper;
                scratch.desired_generalized_acceleration[coordinate] = 0.0;
            }
            for actuator in 0..self.actuation.actuators.len() {
                let index = actuator * stride + agent;
                let lower = input.actuator_effort_lower_soa()[index];
                let upper = input.actuator_effort_upper_soa()[index];
                valid &= !lower.is_nan() && !upper.is_nan() && lower <= upper;
                scratch.actuator_effort_bounds.lower[actuator] = lower;
                scratch.actuator_effort_bounds.upper[actuator] = upper;
            }
            if !valid {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            if invalid_problem {
                output.status[agent] = ExactSolveAgentStatus::InvalidProblem;
                continue;
            }
            scratch.point_tasks.clear();
            for &slot in &self.point_task_order {
                if emission.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0 {
                    continue;
                }
                let descriptor = self.descriptor.point_tasks[slot];
                let query = self.descriptor.point_queries[descriptor.point_query_slot];
                scratch.point_tasks.push(FloatingPointAccelerationTask {
                    stable_id: descriptor.stable_id,
                    frame: FrameId(query.frame_index),
                    point_in_frame: Vec3::from(query.point_in_frame().map(f64::from)),
                    desired_acceleration_world: Vec3::new(
                        emission.point_task_desired_acceleration_soa()
                            [layout.task_vector_index(slot, 0, agent)]
                            as f64,
                        emission.point_task_desired_acceleration_soa()
                            [layout.task_vector_index(slot, 1, agent)]
                            as f64,
                        emission.point_task_desired_acceleration_soa()
                            [layout.task_vector_index(slot, 2, agent)]
                            as f64,
                    ),
                    priority: descriptor.priority,
                    weight: descriptor.weight() as f64,
                });
            }
            scratch.contacts.clear();
            for &slot in &self.contact_order {
                if emission.contact_lock_active_soa()[layout.contact_active_index(slot, agent)] == 0
                {
                    continue;
                }
                let scalar = slot * stride + agent;
                let friction = input.friction_coefficient_soa()[scalar];
                let minimum = input.minimum_normal_force_soa()[scalar];
                let maximum = input.maximum_normal_force_soa()[scalar];
                let nominal = input.nominal_normal_force_soa()[scalar];
                if !friction.is_finite()
                    || friction < 0.0
                    || !minimum.is_finite()
                    || minimum < 0.0
                    || !maximum.is_finite()
                    || maximum < minimum
                    || !nominal.is_finite()
                    || nominal < minimum
                    || nominal > maximum
                {
                    valid = false;
                    break;
                }
                let descriptor = self.descriptor.contact_locks[slot];
                let query = self.descriptor.point_queries[descriptor.point_query_slot];
                let (mode, kinematic_enabled) = match descriptor.kinematic_mode {
                    ContactKinematicMode::Disabled => (ContactMode::LockedPoint, false),
                    ContactKinematicMode::LockedPoint => (ContactMode::LockedPoint, true),
                    ContactKinematicMode::NormalPoint => (ContactMode::NormalPoint, true),
                    ContactKinematicMode::RollingPoint => (ContactMode::RollingPoint, true),
                };
                let mut contact = ContactSpec::horizontal(
                    descriptor.stable_id,
                    FrameId(query.frame_index),
                    Vec3::from(query.point_in_frame().map(f64::from)),
                    mode,
                    friction,
                    maximum,
                    nominal,
                );
                contact.kinematic_enabled = kinematic_enabled;
                contact.minimum_normal_force = minimum;
                let desired = emission_input.contact_desired_acceleration(slot, agent);
                contact.desired_point_acceleration_world =
                    Vec3::new(desired[0] as f64, desired[1] as f64, desired[2] as f64);
                scratch.contacts.push(contact);
            }
            if !valid {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            scratch.support_patches.clear();
            for (patch_slot, patch) in self.support_patches.iter().copied().enumerate() {
                let scalar = patch_slot * stride + agent;
                if input.support_patch_enabled_soa()[scalar] == 0 {
                    continue;
                }
                let margin = input.support_patch_minimum_margin_soa()[scalar];
                let patch_end = patch.first_contact_order_index + patch.contact_count;
                let all_active = self.contact_order[patch.first_contact_order_index..patch_end]
                    .iter()
                    .all(|slot| {
                        emission.contact_lock_active_soa()
                            [layout.contact_active_index(*slot, agent)]
                            != 0
                    });
                if !margin.is_finite() || margin < 0.0 || !all_active {
                    valid = false;
                    break;
                }
                let first_contact = self.contact_order[..patch.first_contact_order_index]
                    .iter()
                    .filter(|slot| {
                        emission.contact_lock_active_soa()
                            [layout.contact_active_index(**slot, agent)]
                            != 0
                    })
                    .count();
                scratch.support_patches.push(SupportPatchSpec {
                    stable_id: patch.stable_id,
                    first_contact,
                    contact_count: patch.contact_count,
                    minimum_margin_m: margin,
                });
            }
            if !valid {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            let solve_input = FloatingDynamicWbcInput {
                state: &scratch.state,
                root_twist_world: root_twist,
                desired_generalized_acceleration: &scratch.desired_generalized_acceleration,
                task_priorities: FloatingTaskPriorities::default(),
                task_weights: FloatingTaskWeights {
                    root_angular: 0.0,
                    root_horizontal: 0.0,
                    root_height: 0.0,
                    joint_posture: 0.0,
                },
                joint_posture_weight: 0.0,
                joint_acceleration_task: None,
                center_of_mass_task: None,
                centroidal_angular_momentum_task: None,
                frame_angular_acceleration_tasks: &[],
                point_acceleration_tasks: &scratch.point_tasks,
                generalized_acceleration_bounds: &scratch.acceleration_bounds,
                torque_bounds: &scratch.torque_bounds,
                actuator_effort: Some(ActuatorEffortInput {
                    actuation: &self.actuation,
                    bounds: &scratch.actuator_effort_bounds,
                }),
                contacts: &scratch.contacts,
                support_patches: &scratch.support_patches,
            };
            if self
                .solver
                .solve_into(solve_input, &mut scratch.output, &mut scratch.scratch)
                .is_err()
            {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            copy_output(layout, agent, &self.descriptor, &scratch.output, output);
            output.active_support_patches[agent] = scratch.support_patches.len() as u32;
            if let Some(envelope) = joint_envelope
                && envelope.status()[agent] == JointEnvelopeAgentStatus::Ok
                && matches!(
                    output.status[agent],
                    ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
                )
            {
                for coordinate in ROOT_TANGENT_COMPONENTS..generalized_dof {
                    let index = layout.generalized_index(coordinate, agent);
                    let value = output.generalized_acceleration_soa[index];
                    let margin = (value - envelope.lower_soa()[index])
                        .min(envelope.upper_soa()[index] - value);
                    if margin < output.minimum_joint_envelope_margin[agent] {
                        output.minimum_joint_envelope_margin[agent] = margin;
                        output.limiting_joint_envelope_coordinate[agent] =
                            (coordinate - ROOT_TANGENT_COMPONENTS) as u32;
                    }
                }
            }
        }
        Ok(())
    }
}

fn load_root_pose(input: &FkBatchInput, agent: usize) -> Option<Transform3> {
    let layout = input.layout();
    let mut values = [0.0_f64; 12];
    for (component, value) in values.iter_mut().enumerate() {
        *value = input.root_pose_soa()[layout.root_pose_index(component, agent)] as f64;
        if !value.is_finite() {
            return None;
        }
    }
    let matrix = Matrix3::from_row_slice(&values[..9]);
    let should_be_identity = matrix.transpose() * matrix;
    if (should_be_identity - Matrix3::identity()).norm() > 1e-5 || matrix.determinant() <= 0.0 {
        return None;
    }
    Some(Transform3::from_parts(
        Translation3::new(values[9], values[10], values[11]),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(matrix)),
    ))
}

fn copy_output(
    layout: &BatchLayout,
    agent: usize,
    descriptor: &BatchProgramDescriptor,
    source: &FloatingDynamicWbcOutput,
    output: &mut ExactDynamicBatchOutput,
) {
    let stride = layout.agent_stride;
    output.status[agent] = source.status.into();
    for coordinate in 0..layout.generalized_coordinate_count {
        output.generalized_acceleration_soa[layout.generalized_index(coordinate, agent)] =
            source.generalized_acceleration[coordinate];
    }
    for coordinate in 0..layout.coordinate_count {
        output.generalized_effort_soa[coordinate * stride + agent] =
            source.actuator_torque[coordinate];
    }
    for slot in 0..layout.contact_lock_count {
        for component in 0..3 {
            let index = (slot * 3 + component) * stride + agent;
            output.contact_force_basis_soa[index] = source.contact_force_basis[(component, slot)];
            output.contact_force_world_soa[index] = source.contact_force_world[(component, slot)];
        }
    }
    output.active_contacts[agent] = source.active_contacts as u32;
    output.dynamics_residual_linf[agent] = source.dynamics_residual_linf;
    output.contact_residual_linf[agent] = source.contact_acceleration_residual_linf;
    output.minimum_friction_margin[agent] = source.minimum_friction_margin;
    output.minimum_support_margin_m[agent] = source.minimum_support_margin_m;
    output.limiting_support_patch[agent] = source.limiting_support_patch.unwrap_or(u32::MAX);
    output.minimum_actuator_effort_margin[agent] = source.minimum_actuator_effort_margin;
    output.limiting_actuator[agent] = source
        .limiting_actuator
        .map_or(u32::MAX, |value| value as u32);
    output.minimum_bound_margin[agent] = source.solve.minimum_bound_margin;
    output.maximum_hard_violation[agent] = source.solve.maximum_constraint_violation;
    output.clipped_steps[agent] = source.solve.clipped_steps as u32;
    for residual in source
        .task_residuals
        .iter()
        .filter(|residual| residual.active)
    {
        if let Some(slot) = descriptor
            .point_tasks
            .iter()
            .position(|task| task.stable_id == residual.stable_id)
        {
            output.task_l2_soa[slot * stride + agent] = residual.l2;
            output.task_clipped_soa[slot * stride + agent] = u8::from(residual.clipped);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::hint::black_box;

    use bonesaw_core::{Priority, TimingSpec};

    use super::*;
    use crate::{
        ContactLockSpec, CpuJointEnvelopeBatchExecutor, CpuMirrorExecutor, DynamicsBatchOutput,
        EmissionBatchInput, EmissionBatchOutput, FkBatchOutput, JacobianBatchOutput,
        JointEnvelopeBatchInput, JointEnvelopeBatchOutput, PointAttractorSpec,
        PointQueryBatchOutput, PointQuerySpec, allocation_sentinel,
    };

    #[test]
    fn dynamic_batch_enforces_dynamics_contacts_effort_and_friction_without_allocation() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/upkie/upkie.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let last = program.model.bodies.len() - 1;
        let queries = [
            PointQuerySpec {
                stable_id: 9001,
                frame_index: 1,
                point_in_frame: [0.0, 0.0, 0.0],
            },
            PointQuerySpec {
                stable_id: 9002,
                frame_index: 20.min(last),
                point_in_frame: [0.0, 0.0, 0.0],
            },
            PointQuerySpec {
                stable_id: 9003,
                frame_index: 38.min(last),
                point_in_frame: [0.0, 0.0, 0.0],
            },
        ];
        let tasks = [PointAttractorSpec {
            stable_id: 9101,
            point_query_stable_id: 9001,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 1.0,
        }];
        let contacts = [
            ContactLockSpec {
                stable_id: 9201,
                point_query_stable_id: 9002,
            },
            ContactLockSpec {
                stable_id: 9202,
                point_query_stable_id: 9003,
            },
        ];
        let executor = CpuMirrorExecutor::compile_with_emission_plan(
            &program, 4, 32, &queries, &tasks, &contacts,
        )
        .unwrap();
        let layout = executor.descriptor().layout.clone();
        let mut state_input = FkBatchInput::new(layout.clone(), 4).unwrap();
        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 4).unwrap();
        for agent in 0..4 {
            let mut state = RobotState::zeros(&program.model);
            state.control_world_from_root.translation.vector.z = 0.62;
            state_input
                .set_agent_from_state(&program.model, agent, &state)
                .unwrap();
            dynamics_input.gravity_world_soa_mut()[agent] = 0.0;
            dynamics_input.gravity_world_soa_mut()[layout.agent_stride + agent] = 0.0;
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.81;
        }
        let mut fk = FkBatchOutput::new(layout.clone());
        executor.execute_into(&state_input, &mut fk).unwrap();
        let mut jacobian = JacobianBatchOutput::new(layout.clone());
        executor.execute_jacobians_into(&fk, &mut jacobian).unwrap();
        let mut dynamics = DynamicsBatchOutput::new(layout.clone());
        executor
            .execute_dynamics_into(&dynamics_input, &fk, &jacobian, &mut dynamics)
            .unwrap();
        let mut points = PointQueryBatchOutput::new(layout.clone());
        executor
            .execute_point_queries_into(&fk, &jacobian, &dynamics, &mut points)
            .unwrap();
        let mut emission_input = EmissionBatchInput::new(layout.clone(), 4).unwrap();
        for agent in 0..4 {
            emission_input.point_task_active_soa_mut()[layout.task_active_index(0, agent)] = 1;
            for component in 0..3 {
                let index = layout.task_vector_index(0, component, agent);
                emission_input.point_target_position_soa_mut()[index] =
                    points.point_position(agent, 0).unwrap()[component];
            }
            for slot in 0..2 {
                emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = 1;
            }
        }
        let mut emission = EmissionBatchOutput::new(layout.clone());
        executor
            .execute_emission_into(&emission_input, &dynamics_input, &points, &mut emission)
            .unwrap();

        let mut input = ExactDynamicBatchInput::new(&program, layout.clone(), 4).unwrap();
        input.acceleration_lower_soa_mut().fill(-100.0);
        input.acceleration_upper_soa_mut().fill(100.0);
        input.maximum_normal_force_soa_mut().fill(1000.0);
        input.nominal_normal_force_soa_mut().fill(50.0);
        let mut output = ExactDynamicBatchOutput::new(layout.clone(), program.model.dof);
        let mut solver = CpuExactDynamicBatchSolver::new(&program, executor.descriptor()).unwrap();
        solver
            .execute_into(
                &state_input,
                &dynamics_input,
                &emission_input,
                &emission,
                &input,
                &mut output,
            )
            .unwrap();
        for agent in 0..4 {
            assert!(matches!(
                output.status()[agent],
                ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
            ));
            assert!(output.dynamics_residual_linf()[agent] <= 1e-8);
            assert!(output.contact_residual_linf()[agent] <= 1e-8);
            assert!(output.minimum_friction_margin()[agent] >= -1e-8);
            assert!(output.minimum_actuator_effort_margin()[agent] >= -1e-8);
            assert_eq!(output.active_contacts()[agent], 2);
            for slot in 0..2 {
                assert!(
                    output.contact_force_basis_soa()[(slot * 3 + 2) * layout.agent_stride + agent]
                        >= -1e-8
                );
            }
        }
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..50 {
                solver
                    .execute_into(
                        black_box(&state_input),
                        black_box(&dynamics_input),
                        black_box(&emission_input),
                        black_box(&emission),
                        black_box(&input),
                        black_box(&mut output),
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));

        let envelope_executor =
            CpuJointEnvelopeBatchExecutor::compile(&program, layout.clone()).unwrap();
        let mut envelope_input = JointEnvelopeBatchInput::new(layout.clone(), 4).unwrap();
        envelope_input.enabled_mut().fill(1);
        envelope_input.maximum_acceleration_soa_mut().fill(0.05);
        let mut envelope = JointEnvelopeBatchOutput::new(layout.clone());
        envelope_executor
            .execute_into(&envelope_input, &mut envelope)
            .unwrap();
        solver
            .execute_with_joint_envelope_into(
                &state_input,
                &dynamics_input,
                &emission_input,
                &emission,
                &input,
                &envelope,
                &mut output,
            )
            .unwrap();
        for agent in 0..4 {
            assert!(matches!(
                output.status()[agent],
                ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
            ));
            assert!(output.minimum_joint_envelope_margin()[agent].is_finite());
            assert!(output.minimum_joint_envelope_margin()[agent] >= -1e-8);
            assert_ne!(output.limiting_joint_envelope_coordinate()[agent], u32::MAX);
            for coordinate in ROOT_TANGENT_COMPONENTS..layout.generalized_coordinate_count {
                let index = layout.generalized_index(coordinate, agent);
                assert!(output.generalized_acceleration_soa()[index].abs() <= 0.05000001);
            }
        }
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..50 {
                envelope_executor
                    .execute_into(black_box(&envelope_input), black_box(&mut envelope))
                    .unwrap();
                solver
                    .execute_with_joint_envelope_into(
                        black_box(&state_input),
                        black_box(&dynamics_input),
                        black_box(&emission_input),
                        black_box(&emission),
                        black_box(&input),
                        black_box(&envelope),
                        black_box(&mut output),
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
    }

    #[test]
    fn finite_support_patch_is_fixed_declared_agent_masked_and_allocation_free() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let foot = program.model.frame_id("left_foot").unwrap().0;
        let offsets = [
            [-0.08, -0.04, -0.07],
            [-0.08, 0.04, -0.07],
            [0.12, -0.04, -0.07],
            [0.12, 0.04, -0.07],
        ];
        let queries = offsets.map(|point_in_frame| PointQuerySpec {
            stable_id: 9_001 + queries_index(point_in_frame, &offsets) as u32,
            frame_index: foot,
            point_in_frame,
        });
        let contacts = std::array::from_fn::<_, 4, _>(|slot| ContactLockSpec {
            stable_id: 9_201 + slot as u32,
            point_query_stable_id: 9_001 + slot as u32,
        });
        let contact_modes = [
            ContactKinematicMode::LockedPoint,
            ContactKinematicMode::NormalPoint,
            ContactKinematicMode::Disabled,
            ContactKinematicMode::RollingPoint,
        ];
        let executor = CpuMirrorExecutor::compile_with_contact_modes(
            &program,
            2,
            2,
            &queries,
            &[],
            &contacts,
            &contact_modes,
        )
        .unwrap();
        assert_eq!(
            executor
                .descriptor()
                .contact_locks
                .iter()
                .map(|contact| {
                    (0..3)
                        .filter(|component| contact.kinematic_mode.axis_enabled(*component))
                        .count()
                })
                .sum::<usize>(),
            6
        );
        let layout = executor.descriptor().layout.clone();
        let mut state = FkBatchInput::new(layout.clone(), 2).unwrap();
        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            let mut robot = RobotState::zeros(&program.model);
            robot.control_world_from_root.translation.vector.z = 0.98;
            state
                .set_agent_from_state(&program.model, agent, &robot)
                .unwrap();
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.81;
        }
        let mut fk = FkBatchOutput::new(layout.clone());
        executor.execute_into(&state, &mut fk).unwrap();
        let mut jacobian = JacobianBatchOutput::new(layout.clone());
        executor.execute_jacobians_into(&fk, &mut jacobian).unwrap();
        let mut dynamics = DynamicsBatchOutput::new(layout.clone());
        executor
            .execute_dynamics_into(&dynamics_input, &fk, &jacobian, &mut dynamics)
            .unwrap();
        let mut points = PointQueryBatchOutput::new(layout.clone());
        executor
            .execute_point_queries_into(&fk, &jacobian, &dynamics, &mut points)
            .unwrap();
        let mut emission_input = EmissionBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            for slot in 0..4 {
                emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = 1;
            }
        }
        let mut emission = EmissionBatchOutput::new(layout.clone());
        executor
            .execute_emission_into(&emission_input, &dynamics_input, &points, &mut emission)
            .unwrap();
        let patches = [ExactDynamicSupportPatchSpec {
            stable_id: 7,
            first_contact_slot: 0,
            contact_count: 4,
        }];
        let mut input = ExactDynamicBatchInput::new_with_support_patch_count(
            &program,
            layout.clone(),
            2,
            patches.len(),
        )
        .unwrap();
        input.acceleration_lower_soa_mut().fill(-200.0);
        input.acceleration_upper_soa_mut().fill(200.0);
        input.maximum_normal_force_soa_mut().fill(2_000.0);
        input.nominal_normal_force_soa_mut().fill(100.0);
        input.support_patch_enabled_soa_mut().fill(1);
        input.support_patch_minimum_margin_soa_mut().fill(0.02);
        let mut output = ExactDynamicBatchOutput::new(layout.clone(), program.model.dof);
        let mut solver = CpuExactDynamicBatchSolver::new_with_support_patches(
            &program,
            executor.descriptor(),
            &patches,
        )
        .unwrap();
        solver
            .execute_into(
                &state,
                &dynamics_input,
                &emission_input,
                &emission,
                &input,
                &mut output,
            )
            .unwrap();
        for agent in 0..2 {
            assert!(matches!(
                output.status()[agent],
                ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
            ));
            assert_eq!(output.active_support_patches()[agent], 1);
            assert_eq!(output.limiting_support_patch()[agent], 7);
            assert!(output.minimum_support_margin_m()[agent] >= 0.02 - 1e-8);
        }
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..20 {
                solver
                    .execute_into(
                        black_box(&state),
                        black_box(&dynamics_input),
                        black_box(&emission_input),
                        black_box(&emission),
                        black_box(&input),
                        black_box(&mut output),
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));

        emission_input.contact_lock_active_soa_mut()[layout.contact_active_index(2, 1)] = 0;
        executor
            .execute_emission_into(&emission_input, &dynamics_input, &points, &mut emission)
            .unwrap();
        solver
            .execute_into(
                &state,
                &dynamics_input,
                &emission_input,
                &emission,
                &input,
                &mut output,
            )
            .unwrap();
        assert!(matches!(
            output.status()[0],
            ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
        ));
        assert_eq!(output.status()[1], ExactSolveAgentStatus::InvalidInput);
        assert!(
            output
                .generalized_acceleration_soa()
                .iter()
                .skip(1)
                .step_by(layout.agent_stride)
                .all(|value| *value == 0.0)
        );
    }

    fn queries_index(point: [f64; 3], points: &[[f64; 3]; 4]) -> usize {
        points
            .iter()
            .position(|candidate| *candidate == point)
            .unwrap()
    }
}
