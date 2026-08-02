//! Allocation-free `CpuExactF64` consumer for the fixed r79 emission ABI.
//!
//! This is deliberately not a CUDA-mirror claim. It connects the batch row
//! contract to the established strict lexicographic CPU solver so task/contact
//! semantics, failure typing, and bounded-work diagnostics can be evaluated
//! before any device solver is admitted.

use bonesaw_core::{
    ConstraintBuffer, HierarchicalSolver, Priority, SolveResult, SolveStatus, SolverWorkspace,
    TaskBuffer, TaskKind, VelocityBounds,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    AgentStatus, BatchInputError, BatchLayout, BatchProgramDescriptor, EmissionBatchOutput,
    JointEnvelopeAgentStatus, JointEnvelopeBatchOutput, ROOT_TANGENT_COMPONENTS,
};

pub const PRIORITY_LEVEL_COUNT: usize = Priority::ALL.len();

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ExactSolveAgentStatus {
    Inactive = 0,
    Solved = 1,
    SolvedWithSlack = 2,
    MaxIterations = 3,
    PrimalInfeasible = 4,
    NumericalFailure = 5,
    InvalidProblem = 6,
    InvalidInput = 7,
}

impl From<SolveStatus> for ExactSolveAgentStatus {
    fn from(value: SolveStatus) -> Self {
        match value {
            SolveStatus::Solved => Self::Solved,
            SolveStatus::SolvedWithSlack => Self::SolvedWithSlack,
            SolveStatus::MaxIterations => Self::MaxIterations,
            SolveStatus::PrimalInfeasible => Self::PrimalInfeasible,
            SolveStatus::NumericalFailure => Self::NumericalFailure,
            SolveStatus::InvalidProblem => Self::InvalidProblem,
        }
    }
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum ExactSolveError {
    #[error("input or output layout does not match the exact solve plan")]
    LayoutMismatch,
    #[error("contact block stable ID {0} cannot encode three scalar row IDs")]
    ContactStableIdOverflow(u32),
}

/// Per-agent generalized-acceleration bounds for the exact solve bridge.
#[derive(Clone, Debug)]
pub struct ExactSolveBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    lower_soa: Vec<f64>,
    upper_soa: Vec<f64>,
}

impl ExactSolveBatchInput {
    pub fn new(layout: BatchLayout, active_agents: usize) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        let len = layout
            .generalized_velocity_len()
            .expect("validated batch layout");
        Ok(Self {
            lower_soa: vec![f64::NEG_INFINITY; len],
            upper_soa: vec![f64::INFINITY; len],
            layout,
            active_agents,
        })
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub const fn active_agents(&self) -> usize {
        self.active_agents
    }

    pub fn lower_soa(&self) -> &[f64] {
        &self.lower_soa
    }

    pub fn lower_soa_mut(&mut self) -> &mut [f64] {
        &mut self.lower_soa
    }

    pub fn upper_soa(&self) -> &[f64] {
        &self.upper_soa
    }

    pub fn upper_soa_mut(&mut self) -> &mut [f64] {
        &mut self.upper_soa
    }
}

/// Fixed diagnostics for a batch of strict lexicographic exact solves.
#[derive(Clone, Debug)]
pub struct ExactSolveBatchOutput {
    layout: BatchLayout,
    generalized_acceleration_soa: Vec<f64>,
    status: Vec<ExactSolveAgentStatus>,
    level_rows_soa: Vec<u32>,
    level_l2_soa: Vec<f64>,
    level_rank_soa: Vec<u32>,
    level_clipped_soa: Vec<u8>,
    minimum_bound_margin: Vec<f64>,
    maximum_hard_violation: Vec<f64>,
    active_constraint_count: Vec<u32>,
    task_pseudoinverse_calls: Vec<u32>,
    task_jacobi_sweeps: Vec<u32>,
    clipped_steps: Vec<u32>,
    feasibility_projection_sweeps: Vec<u32>,
    feasibility_halfspace_projections: Vec<u32>,
    feasibility_polish_iterations: Vec<u32>,
    minimum_joint_envelope_margin: Vec<f64>,
    limiting_joint_envelope_coordinate: Vec<u32>,
}

impl ExactSolveBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        let stride = layout.agent_stride;
        Self {
            generalized_acceleration_soa: vec![
                0.0;
                layout
                    .generalized_velocity_len()
                    .expect("validated batch layout")
            ],
            status: vec![ExactSolveAgentStatus::Inactive; stride],
            level_rows_soa: vec![0; PRIORITY_LEVEL_COUNT * stride],
            level_l2_soa: vec![0.0; PRIORITY_LEVEL_COUNT * stride],
            level_rank_soa: vec![0; PRIORITY_LEVEL_COUNT * stride],
            level_clipped_soa: vec![0; PRIORITY_LEVEL_COUNT * stride],
            minimum_bound_margin: vec![f64::INFINITY; stride],
            maximum_hard_violation: vec![0.0; stride],
            active_constraint_count: vec![0; stride],
            task_pseudoinverse_calls: vec![0; stride],
            task_jacobi_sweeps: vec![0; stride],
            clipped_steps: vec![0; stride],
            feasibility_projection_sweeps: vec![0; stride],
            feasibility_halfspace_projections: vec![0; stride],
            feasibility_polish_iterations: vec![0; stride],
            minimum_joint_envelope_margin: vec![f64::INFINITY; stride],
            limiting_joint_envelope_coordinate: vec![u32::MAX; stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub fn generalized_acceleration_soa(&self) -> &[f64] {
        &self.generalized_acceleration_soa
    }
    pub fn status(&self) -> &[ExactSolveAgentStatus] {
        &self.status
    }
    pub fn level_rows_soa(&self) -> &[u32] {
        &self.level_rows_soa
    }
    pub fn level_l2_soa(&self) -> &[f64] {
        &self.level_l2_soa
    }
    pub fn level_rank_soa(&self) -> &[u32] {
        &self.level_rank_soa
    }
    pub fn level_clipped_soa(&self) -> &[u8] {
        &self.level_clipped_soa
    }
    pub fn minimum_bound_margin(&self) -> &[f64] {
        &self.minimum_bound_margin
    }
    pub fn maximum_hard_violation(&self) -> &[f64] {
        &self.maximum_hard_violation
    }
    pub fn active_constraint_count(&self) -> &[u32] {
        &self.active_constraint_count
    }
    pub fn task_pseudoinverse_calls(&self) -> &[u32] {
        &self.task_pseudoinverse_calls
    }
    pub fn task_jacobi_sweeps(&self) -> &[u32] {
        &self.task_jacobi_sweeps
    }
    pub fn clipped_steps(&self) -> &[u32] {
        &self.clipped_steps
    }
    pub fn feasibility_projection_sweeps(&self) -> &[u32] {
        &self.feasibility_projection_sweeps
    }
    pub fn feasibility_halfspace_projections(&self) -> &[u32] {
        &self.feasibility_halfspace_projections
    }
    pub fn feasibility_polish_iterations(&self) -> &[u32] {
        &self.feasibility_polish_iterations
    }
    pub fn minimum_joint_envelope_margin(&self) -> &[f64] {
        &self.minimum_joint_envelope_margin
    }
    pub fn limiting_joint_envelope_coordinate(&self) -> &[u32] {
        &self.limiting_joint_envelope_coordinate
    }

    fn clear(&mut self) {
        self.generalized_acceleration_soa.fill(0.0);
        self.status.fill(ExactSolveAgentStatus::Inactive);
        self.level_rows_soa.fill(0);
        self.level_l2_soa.fill(0.0);
        self.level_rank_soa.fill(0);
        self.level_clipped_soa.fill(0);
        self.minimum_bound_margin.fill(f64::INFINITY);
        self.maximum_hard_violation.fill(0.0);
        self.active_constraint_count.fill(0);
        self.task_pseudoinverse_calls.fill(0);
        self.task_jacobi_sweeps.fill(0);
        self.clipped_steps.fill(0);
        self.feasibility_projection_sweeps.fill(0);
        self.feasibility_halfspace_projections.fill(0);
        self.feasibility_polish_iterations.fill(0);
        self.minimum_joint_envelope_margin.fill(f64::INFINITY);
        self.limiting_joint_envelope_coordinate.fill(u32::MAX);
    }
}

#[derive(Debug)]
struct AgentScratch {
    tasks: TaskBuffer,
    constraints: ConstraintBuffer,
    bounds: VelocityBounds,
    result: SolveResult,
    workspace: SolverWorkspace,
}

/// Preallocated strict `f64` solver over the fixed r79 emission rows.
#[derive(Debug)]
pub struct CpuExactBatchSolver {
    descriptor: BatchProgramDescriptor,
    solver: HierarchicalSolver,
    scratch: Vec<AgentScratch>,
}

impl CpuExactBatchSolver {
    pub fn new(descriptor: &BatchProgramDescriptor) -> Result<Self, ExactSolveError> {
        for contact in &descriptor.contact_locks {
            if contact.stable_id > (u32::MAX - 2) / 4 {
                return Err(ExactSolveError::ContactStableIdOverflow(contact.stable_id));
            }
        }
        let generalized_dof = descriptor.layout.generalized_coordinate_count;
        let maximum_tasks = descriptor.layout.point_task_count;
        let maximum_constraints = descriptor
            .layout
            .contact_lock_count
            .checked_mul(3)
            .expect("validated batch layout");
        let maximum_task_rows = maximum_tasks
            .checked_mul(3)
            .expect("validated batch layout");
        let maximum_feasibility_rows = maximum_constraints
            .checked_add(
                generalized_dof
                    .checked_mul(2)
                    .expect("validated batch layout"),
            )
            .and_then(|rows| rows.checked_add(maximum_constraints.checked_mul(2)?))
            .expect("validated batch layout");
        let workspace_rows = maximum_task_rows.max(maximum_feasibility_rows);
        let mut scratch = Vec::with_capacity(descriptor.layout.agent_capacity);
        for _ in 0..descriptor.layout.agent_capacity {
            scratch.push(AgentScratch {
                tasks: TaskBuffer::new(generalized_dof, maximum_tasks, 0),
                constraints: ConstraintBuffer::new(generalized_dof, maximum_constraints),
                bounds: VelocityBounds::unbounded(generalized_dof),
                result: SolveResult::workspace(generalized_dof, maximum_constraints),
                workspace: SolverWorkspace::new(
                    generalized_dof,
                    workspace_rows,
                    maximum_tasks,
                    maximum_constraints,
                ),
            });
        }
        Ok(Self {
            descriptor: descriptor.clone(),
            solver: HierarchicalSolver::default(),
            scratch,
        })
    }

    pub fn descriptor(&self) -> &BatchProgramDescriptor {
        &self.descriptor
    }

    pub fn execute_into(
        &mut self,
        emission: &EmissionBatchOutput,
        input: &ExactSolveBatchInput,
        output: &mut ExactSolveBatchOutput,
    ) -> Result<(), ExactSolveError> {
        self.execute_impl(emission, input, None, output)
    }

    pub fn execute_with_joint_envelope_into(
        &mut self,
        emission: &EmissionBatchOutput,
        input: &ExactSolveBatchInput,
        joint_envelope: &JointEnvelopeBatchOutput,
        output: &mut ExactSolveBatchOutput,
    ) -> Result<(), ExactSolveError> {
        self.execute_impl(emission, input, Some(joint_envelope), output)
    }

    fn execute_impl(
        &mut self,
        emission: &EmissionBatchOutput,
        input: &ExactSolveBatchInput,
        joint_envelope: Option<&JointEnvelopeBatchOutput>,
        output: &mut ExactSolveBatchOutput,
    ) -> Result<(), ExactSolveError> {
        if emission.layout() != &self.descriptor.layout
            || input.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
            || joint_envelope.is_some_and(|envelope| envelope.layout() != &self.descriptor.layout)
        {
            return Err(ExactSolveError::LayoutMismatch);
        }
        output.clear();
        let layout = &self.descriptor.layout;
        let generalized_dof = layout.generalized_coordinate_count;
        for agent in 0..layout.agent_capacity {
            if emission.agent_status()[agent] == AgentStatus::InvalidInput {
                output.status[agent] = ExactSolveAgentStatus::InvalidInput;
                continue;
            }
            if emission.agent_status()[agent] != AgentStatus::Ok || agent >= input.active_agents() {
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
            let scratch = &mut self.scratch[agent];
            scratch.tasks.begin();
            scratch.constraints.begin();
            for (slot, descriptor) in self.descriptor.point_tasks.iter().copied().enumerate() {
                if emission.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0 {
                    continue;
                }
                let task = scratch
                    .tasks
                    .push_three()
                    .expect("compiled point task capacity");
                task.stable_id = descriptor.stable_id;
                task.kind = TaskKind::Point;
                task.priority = descriptor.priority;
                task.weight = descriptor.weight() as f64;
                for component in 0..3 {
                    task.target_velocity[component] = emission.point_task_rhs_soa()
                        [layout.task_vector_index(slot, component, agent)]
                        as f64;
                    for coordinate in 0..generalized_dof {
                        task.jacobian[(component, coordinate)] = emission.point_task_jacobian_soa()
                            [layout.task_jacobian_index(slot, component, coordinate, agent)]
                            as f64;
                    }
                }
            }
            for (slot, descriptor) in self.descriptor.contact_locks.iter().copied().enumerate() {
                if emission.contact_lock_active_soa()[layout.contact_active_index(slot, agent)] == 0
                {
                    continue;
                }
                for component in 0..3 {
                    let constraint = scratch
                        .constraints
                        .push()
                        .expect("compiled contact constraint capacity");
                    constraint.stable_id = descriptor.stable_id * 4 + component as u32;
                    let rhs = emission.contact_lock_rhs_soa()
                        [layout.contact_vector_index(slot, component, agent)]
                        as f64;
                    constraint.lower = rhs;
                    constraint.upper = rhs;
                    for coordinate in 0..generalized_dof {
                        constraint.coefficients[coordinate] = emission.contact_lock_jacobian_soa()
                            [layout.contact_jacobian_index(slot, component, coordinate, agent)]
                            as f64;
                    }
                }
            }
            let mut invalid_bounds = false;
            for coordinate in 0..generalized_dof {
                let index = layout.generalized_index(coordinate, agent);
                let mut lower = input.lower_soa()[index];
                let mut upper = input.upper_soa()[index];
                if let Some(envelope) = joint_envelope
                    && envelope.status()[agent] == JointEnvelopeAgentStatus::Ok
                {
                    lower = lower.max(envelope.lower_soa()[index]);
                    upper = upper.min(envelope.upper_soa()[index]);
                }
                invalid_bounds |= lower.is_nan() || upper.is_nan() || lower > upper;
                scratch.bounds.lower[coordinate] = lower;
                scratch.bounds.upper[coordinate] = upper;
            }
            if invalid_bounds {
                output.status[agent] = ExactSolveAgentStatus::InvalidProblem;
                continue;
            }
            self.solver.solve_task_buffer_into(
                generalized_dof,
                &scratch.tasks,
                &scratch.bounds,
                scratch.constraints.active(),
                &mut scratch.result,
                &mut scratch.workspace,
            );
            let diagnostics = &scratch.result.diagnostics;
            output.status[agent] = diagnostics.status.into();
            for coordinate in 0..generalized_dof {
                output.generalized_acceleration_soa[layout.generalized_index(coordinate, agent)] =
                    scratch.result.velocity[coordinate];
            }
            for level in &diagnostics.level_residuals {
                let index = level.priority as usize * layout.agent_stride + agent;
                output.level_rows_soa[index] = level.rows.min(u32::MAX as usize) as u32;
                output.level_l2_soa[index] = level.l2;
            }
            for (level, rank) in diagnostics.rank_by_level.iter().copied().enumerate() {
                output.level_rank_soa[level * layout.agent_stride + agent] =
                    rank.min(u32::MAX as usize) as u32;
            }
            for priority in diagnostics.clipped_levels.iter().copied() {
                output.level_clipped_soa[priority as usize * layout.agent_stride + agent] = 1;
            }
            output.minimum_bound_margin[agent] = diagnostics.minimum_bound_margin;
            output.maximum_hard_violation[agent] = diagnostics.maximum_constraint_violation;
            output.active_constraint_count[agent] =
                diagnostics.active_constraints.len().min(u32::MAX as usize) as u32;
            output.task_pseudoinverse_calls[agent] =
                diagnostics.task_pseudoinverse_calls.min(u32::MAX as usize) as u32;
            output.task_jacobi_sweeps[agent] =
                diagnostics.task_jacobi_sweeps.min(u32::MAX as usize) as u32;
            output.clipped_steps[agent] = diagnostics.clipped_steps.min(u32::MAX as usize) as u32;
            output.feasibility_projection_sweeps[agent] = diagnostics
                .feasibility_projection_sweeps
                .min(u32::MAX as usize)
                as u32;
            output.feasibility_halfspace_projections[agent] = diagnostics
                .feasibility_halfspace_projections
                .min(u32::MAX as usize)
                as u32;
            output.feasibility_polish_iterations[agent] = diagnostics
                .feasibility_polish_iterations
                .min(u32::MAX as usize)
                as u32;
            if let Some(envelope) = joint_envelope
                && envelope.status()[agent] == JointEnvelopeAgentStatus::Ok
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

#[cfg(test)]
mod tests {
    use std::hint::black_box;

    use bonesaw_core::{MotionProgram, Priority, RobotState, TimingSpec, math::Transform3};

    use super::*;
    use crate::{
        ContactLockSpec, CpuMirrorExecutor, DynamicsBatchInput, DynamicsBatchOutput,
        EmissionBatchInput, FkBatchInput, FkBatchOutput, JacobianBatchOutput, PointAttractorSpec,
        PointQueryBatchOutput, PointQuerySpec, allocation_sentinel,
    };

    fn setup(
        agents: usize,
        contact_specs: &[ContactLockSpec],
    ) -> (
        CpuMirrorExecutor,
        DynamicsBatchInput,
        PointQueryBatchOutput,
        EmissionBatchInput,
        EmissionBatchOutput,
    ) {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/upkie/upkie.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let last = program.model.bodies.len() - 1;
        let queries = [
            PointQuerySpec {
                stable_id: 8001,
                frame_index: 1,
                point_in_frame: [0.11, -0.04, 0.17],
            },
            PointQuerySpec {
                stable_id: 8002,
                frame_index: 20.min(last),
                point_in_frame: [0.02, 0.0, -0.01],
            },
            PointQuerySpec {
                stable_id: 8003,
                frame_index: 38.min(last),
                point_in_frame: [-0.02, 0.0, -0.01],
            },
        ];
        let tasks = [
            PointAttractorSpec {
                stable_id: 8101,
                point_query_stable_id: 8001,
                priority: Priority::Viability,
                weight: 1.0,
                bandwidth_hz: 1.5,
            },
            PointAttractorSpec {
                stable_id: 8102,
                point_query_stable_id: 8001,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 2.0,
            },
        ];
        let executor = CpuMirrorExecutor::compile_with_emission_plan(
            &program,
            agents,
            32,
            &queries,
            &tasks,
            contact_specs,
        )
        .unwrap();
        let layout = executor.descriptor().layout.clone();
        let mut fk_input = FkBatchInput::new(layout.clone(), agents).unwrap();
        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), agents).unwrap();
        for agent in 0..agents {
            let mut state = RobotState::zeros(&program.model);
            state.control_world_from_root = Transform3::from_parts(
                nalgebra::Translation3::new(0.01 * agent as f64, -0.005 * agent as f64, 0.62),
                nalgebra::UnitQuaternion::identity(),
            );
            for coordinate in 0..program.model.dof {
                state.q[coordinate] = 0.02 * ((agent + coordinate) as f64 * 0.37).sin();
                state.v[coordinate] = 0.03 * ((agent + coordinate) as f64 * 0.29).cos();
            }
            fk_input
                .set_agent_from_state(&program.model, agent, &state)
                .unwrap();
            for coordinate in 0..layout.generalized_coordinate_count {
                dynamics_input.generalized_velocity_soa_mut()
                    [layout.generalized_index(coordinate, agent)] =
                    0.02 * ((agent + coordinate) as f32 * 0.17).sin();
            }
            dynamics_input.gravity_world_soa_mut()[agent] = 0.0;
            dynamics_input.gravity_world_soa_mut()[layout.agent_stride + agent] = 0.0;
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.81;
        }
        let mut fk = FkBatchOutput::new(layout.clone());
        executor.execute_into(&fk_input, &mut fk).unwrap();
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
        let mut emission_input = EmissionBatchInput::new(layout.clone(), agents).unwrap();
        for agent in 0..agents {
            for slot in 0..2 {
                emission_input.point_task_active_soa_mut()[layout.task_active_index(slot, agent)] =
                    1;
                for component in 0..3 {
                    let index = layout.task_vector_index(slot, component, agent);
                    let current = points.point_position(agent, 0).unwrap()[component];
                    emission_input.point_target_position_soa_mut()[index] =
                        current + (slot as f32 * 2.0 - 0.5) * 0.015;
                    emission_input.point_target_velocity_soa_mut()[index] = 0.0;
                    emission_input.point_target_acceleration_soa_mut()[index] = 0.0;
                }
            }
            for slot in 0..contact_specs.len() {
                emission_input.contact_lock_active_soa_mut()
                    [layout.contact_active_index(slot, agent)] = 1;
            }
        }
        let mut emission = EmissionBatchOutput::new(layout);
        executor
            .execute_emission_into(&emission_input, &dynamics_input, &points, &mut emission)
            .unwrap();
        (executor, dynamics_input, points, emission_input, emission)
    }

    #[test]
    fn exact_bridge_preserves_contacts_priority_and_zero_allocation() {
        let contacts = [ContactLockSpec {
            stable_id: 8201,
            point_query_stable_id: 8002,
        }];
        let (executor, _, _, _, emission) = setup(8, &contacts);
        let layout = executor.descriptor().layout.clone();
        let mut solver = CpuExactBatchSolver::new(executor.descriptor()).unwrap();
        let mut input = ExactSolveBatchInput::new(layout.clone(), 8).unwrap();
        for value in input.lower_soa_mut() {
            *value = -100.0;
        }
        for value in input.upper_soa_mut() {
            *value = 100.0;
        }
        let mut output = ExactSolveBatchOutput::new(layout.clone());
        solver.execute_into(&emission, &input, &mut output).unwrap();
        for agent in 0..8 {
            assert!(matches!(
                output.status()[agent],
                ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
            ));
            assert!(output.maximum_hard_violation()[agent] <= 1e-8);
            assert_eq!(
                output.level_rows_soa()[Priority::Viability as usize * layout.agent_stride + agent],
                3
            );
            assert_eq!(
                output.level_rows_soa()[Priority::Intent as usize * layout.agent_stride + agent],
                3
            );
            assert!(
                output.level_l2_soa()[Priority::Viability as usize * layout.agent_stride + agent]
                    <= 1e-7
            );
            assert!(
                output.level_l2_soa()[Priority::Intent as usize * layout.agent_stride + agent]
                    > 1e-3
            );
            for component in 0..3 {
                let mut achieved = 0.0;
                for coordinate in 0..layout.generalized_coordinate_count {
                    achieved += emission.contact_lock_jacobian_soa()
                        [layout.contact_jacobian_index(0, component, coordinate, agent)]
                        as f64
                        * output.generalized_acceleration_soa()
                            [layout.generalized_index(coordinate, agent)];
                }
                let rhs = emission.contact_lock_rhs_soa()
                    [layout.contact_vector_index(0, component, agent)]
                    as f64;
                assert!((achieved - rhs).abs() <= 1e-8);
            }
        }
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(solver.execute_into(
                    black_box(&emission),
                    black_box(&input),
                    black_box(&mut output),
                ))
                .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
    }

    #[test]
    fn exact_bridge_reports_continuous_soft_compromise_at_hard_bounds() {
        let (executor, _, _, _, emission) = setup(4, &[]);
        let layout = executor.descriptor().layout.clone();
        let mut solver = CpuExactBatchSolver::new(executor.descriptor()).unwrap();
        let mut input = ExactSolveBatchInput::new(layout.clone(), 4).unwrap();
        input.lower_soa_mut().fill(-0.01);
        input.upper_soa_mut().fill(0.01);
        let mut output = ExactSolveBatchOutput::new(layout.clone());
        solver.execute_into(&emission, &input, &mut output).unwrap();
        for agent in 0..4 {
            assert_eq!(
                output.status()[agent],
                ExactSolveAgentStatus::SolvedWithSlack
            );
            assert_eq!(output.minimum_bound_margin()[agent], 0.0);
            assert!(output.clipped_steps()[agent] > 0);
            assert!(
                output.level_l2_soa()[Priority::Viability as usize * layout.agent_stride + agent]
                    > 0.0
            );
            for coordinate in 0..layout.generalized_coordinate_count {
                let value = output.generalized_acceleration_soa()
                    [layout.generalized_index(coordinate, agent)];
                assert!((-0.01..=0.01).contains(&value));
            }
        }
    }

    #[test]
    fn exact_bridge_intersects_rust_joint_viability_envelopes_without_allocation() {
        let (executor, _, _, _, emission) = setup(4, &[]);
        let layout = executor.descriptor().layout.clone();
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/upkie/upkie.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let envelope_executor =
            crate::CpuJointEnvelopeBatchExecutor::compile(&program, layout.clone()).unwrap();
        let mut envelope_input = crate::JointEnvelopeBatchInput::new(layout.clone(), 4).unwrap();
        envelope_input.enabled_mut().fill(1);
        envelope_input.dt_seconds_mut().fill(0.02);
        envelope_input.maximum_acceleration_soa_mut().fill(0.01);
        let mut envelope = crate::JointEnvelopeBatchOutput::new(layout.clone());
        envelope_executor
            .execute_into(&envelope_input, &mut envelope)
            .unwrap();

        let mut solver = CpuExactBatchSolver::new(executor.descriptor()).unwrap();
        let mut input = ExactSolveBatchInput::new(layout.clone(), 4).unwrap();
        for agent in 0..4 {
            for coordinate in 0..ROOT_TANGENT_COMPONENTS {
                let index = layout.generalized_index(coordinate, agent);
                input.lower_soa_mut()[index] = 0.0;
                input.upper_soa_mut()[index] = 0.0;
            }
        }
        let mut output = ExactSolveBatchOutput::new(layout.clone());
        solver
            .execute_with_joint_envelope_into(&emission, &input, &envelope, &mut output)
            .unwrap();
        for agent in 0..4 {
            assert_eq!(
                output.status()[agent],
                ExactSolveAgentStatus::SolvedWithSlack
            );
            assert!(output.minimum_joint_envelope_margin()[agent].is_finite());
            assert!(
                (-1e-12..=0.0100000001).contains(&output.minimum_joint_envelope_margin()[agent])
            );
            assert_ne!(output.limiting_joint_envelope_coordinate()[agent], u32::MAX);
            for coordinate in ROOT_TANGENT_COMPONENTS..layout.generalized_coordinate_count {
                let index = layout.generalized_index(coordinate, agent);
                assert!(output.generalized_acceleration_soa()[index].abs() <= 0.0100000001);
            }
        }
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                solver
                    .execute_with_joint_envelope_into(
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
    fn contradictory_nonzero_contact_rows_report_budget_exhaustion_not_infeasibility() {
        let contacts = [
            ContactLockSpec {
                stable_id: 8201,
                point_query_stable_id: 8002,
            },
            ContactLockSpec {
                stable_id: 8202,
                point_query_stable_id: 8002,
            },
        ];
        let (executor, dynamics_input, points, mut emission_input, mut emission) =
            setup(1, &contacts);
        let layout = executor.descriptor().layout.clone();
        emission_input.contact_desired_acceleration_soa_mut()
            [layout.contact_vector_index(1, 0, 0)] = 1.0;
        executor
            .execute_emission_into(&emission_input, &dynamics_input, &points, &mut emission)
            .unwrap();
        let mut solver = CpuExactBatchSolver::new(executor.descriptor()).unwrap();
        let input = ExactSolveBatchInput::new(layout.clone(), 1).unwrap();
        let mut output = ExactSolveBatchOutput::new(layout);
        solver.execute_into(&emission, &input, &mut output).unwrap();
        assert_eq!(output.status()[0], ExactSolveAgentStatus::MaxIterations);
        assert!(output.maximum_hard_violation()[0] > 0.1);
        assert!(output.feasibility_projection_sweeps()[0] > 0);
        assert!(
            output
                .generalized_acceleration_soa()
                .iter()
                .all(|value| *value == 0.0)
        );
    }
}
