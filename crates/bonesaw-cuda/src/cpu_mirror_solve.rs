//! Fixed-iteration `CpuMirrorF32` hierarchy used to freeze the future CUDA
//! solver algorithm before any device implementation is admitted.
//!
//! This is intentionally `FixedLevelApproximate`, not the deployed strict
//! `CpuExactF64` solver. Hard contact rows and bounds are projected in stable
//! descriptor order. Each soft level then uses fixed row-action sweeps while
//! prior-level achieved rows are frozen as equalities. A budget-exhausted
//! candidate is retained for diagnostics, but the admitted command is zero.

use bonesaw_core::Priority;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{
    AgentStatus, BatchInputError, BatchLayout, BatchProgramDescriptor, EmissionBatchOutput,
    KernelManifest,
};

pub const MIRROR_HARD_PROJECTION_SWEEPS: u32 = 64;
pub const MIRROR_TASK_SWEEPS_PER_LEVEL: u32 = 32;
const MIRROR_RESTORE_SWEEPS: u32 = 2;
const HARD_TOLERANCE: f32 = 5.0e-4;
const SOFT_RESIDUAL_TOLERANCE: f32 = 5.0e-4;
const ROW_NORM_EPSILON: f32 = 1.0e-12;
const REGULARIZATION: f32 = 1.0e-6;
const RELAXATION: f32 = 0.85;
const ALGORITHM_ID: &str = "bonesaw-cpu-mirror-f32-fixed-level-v1|hard=64|task=32|restore=2|hard_tol=5e-4|soft_tol=5e-4|row_eps=1e-12|regularization=1e-6|relaxation=0.85|stable_descriptor_order|fma";

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MirrorSolveSemantics {
    FixedLevelApproximate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MirrorSolveAgentStatus {
    Inactive = 0,
    Solved = 1,
    SolvedWithResidual = 2,
    MaxIterations = 3,
    InvalidProblem = 4,
    InvalidInput = 5,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum MirrorSolveError {
    #[error("input or output layout does not match the mirror solve plan")]
    LayoutMismatch,
    #[error("mirror solve scratch size overflows usize")]
    SizeOverflow,
}

#[derive(Clone, Debug)]
pub struct MirrorSolveBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    lower_soa: Vec<f32>,
    upper_soa: Vec<f32>,
}

impl MirrorSolveBatchInput {
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
            lower_soa: vec![f32::NEG_INFINITY; len],
            upper_soa: vec![f32::INFINITY; len],
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

    pub fn lower_soa(&self) -> &[f32] {
        &self.lower_soa
    }

    pub fn lower_soa_mut(&mut self) -> &mut [f32] {
        &mut self.lower_soa
    }

    pub fn upper_soa(&self) -> &[f32] {
        &self.upper_soa
    }

    pub fn upper_soa_mut(&mut self) -> &mut [f32] {
        &mut self.upper_soa
    }

    pub fn resident_bytes(&self) -> usize {
        self.lower_soa.len() * size_of::<f32>() + self.upper_soa.len() * size_of::<f32>()
    }
}

#[derive(Clone, Debug)]
pub struct MirrorSolveBatchOutput {
    layout: BatchLayout,
    semantics: MirrorSolveSemantics,
    generalized_acceleration_soa: Vec<f32>,
    candidate_generalized_acceleration_soa: Vec<f32>,
    status: Vec<MirrorSolveAgentStatus>,
    level_rows_soa: Vec<u32>,
    level_rms_soa: Vec<f32>,
    level_preservation_drift_soa: Vec<f32>,
    initial_hard_violation: Vec<f32>,
    best_hard_violation: Vec<f32>,
    final_hard_violation: Vec<f32>,
    minimum_bound_margin: Vec<f32>,
    hard_projection_sweeps: Vec<u32>,
    task_sweeps: Vec<u32>,
    clipped_updates: Vec<u32>,
    active_hard_rows: Vec<u32>,
    active_soft_rows: Vec<u32>,
}

impl MirrorSolveBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        let vector_len = layout
            .generalized_velocity_len()
            .expect("validated batch layout");
        let level_len = Priority::ALL.len() * layout.agent_stride;
        let stride = layout.agent_stride;
        Self {
            generalized_acceleration_soa: vec![0.0; vector_len],
            candidate_generalized_acceleration_soa: vec![0.0; vector_len],
            status: vec![MirrorSolveAgentStatus::Inactive; stride],
            level_rows_soa: vec![0; level_len],
            level_rms_soa: vec![0.0; level_len],
            level_preservation_drift_soa: vec![0.0; level_len],
            initial_hard_violation: vec![0.0; stride],
            best_hard_violation: vec![0.0; stride],
            final_hard_violation: vec![0.0; stride],
            minimum_bound_margin: vec![f32::INFINITY; stride],
            hard_projection_sweeps: vec![0; stride],
            task_sweeps: vec![0; stride],
            clipped_updates: vec![0; stride],
            active_hard_rows: vec![0; stride],
            active_soft_rows: vec![0; stride],
            semantics: MirrorSolveSemantics::FixedLevelApproximate,
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub const fn semantics(&self) -> MirrorSolveSemantics {
        self.semantics
    }
    pub fn generalized_acceleration_soa(&self) -> &[f32] {
        &self.generalized_acceleration_soa
    }
    pub fn candidate_generalized_acceleration_soa(&self) -> &[f32] {
        &self.candidate_generalized_acceleration_soa
    }
    pub fn status(&self) -> &[MirrorSolveAgentStatus] {
        &self.status
    }
    pub fn level_rows_soa(&self) -> &[u32] {
        &self.level_rows_soa
    }
    pub fn level_rms_soa(&self) -> &[f32] {
        &self.level_rms_soa
    }
    pub fn level_preservation_drift_soa(&self) -> &[f32] {
        &self.level_preservation_drift_soa
    }
    pub fn initial_hard_violation(&self) -> &[f32] {
        &self.initial_hard_violation
    }
    pub fn best_hard_violation(&self) -> &[f32] {
        &self.best_hard_violation
    }
    pub fn final_hard_violation(&self) -> &[f32] {
        &self.final_hard_violation
    }
    pub fn minimum_bound_margin(&self) -> &[f32] {
        &self.minimum_bound_margin
    }
    pub fn hard_projection_sweeps(&self) -> &[u32] {
        &self.hard_projection_sweeps
    }
    pub fn task_sweeps(&self) -> &[u32] {
        &self.task_sweeps
    }
    pub fn clipped_updates(&self) -> &[u32] {
        &self.clipped_updates
    }
    pub fn active_hard_rows(&self) -> &[u32] {
        &self.active_hard_rows
    }
    pub fn active_soft_rows(&self) -> &[u32] {
        &self.active_soft_rows
    }

    pub fn resident_bytes(&self) -> usize {
        self.generalized_acceleration_soa.len() * size_of::<f32>()
            + self.candidate_generalized_acceleration_soa.len() * size_of::<f32>()
            + self.status.len() * size_of::<MirrorSolveAgentStatus>()
            + self.level_rows_soa.len() * size_of::<u32>()
            + self.level_rms_soa.len() * size_of::<f32>()
            + self.level_preservation_drift_soa.len() * size_of::<f32>()
            + self.initial_hard_violation.len() * size_of::<f32>()
            + self.best_hard_violation.len() * size_of::<f32>()
            + self.final_hard_violation.len() * size_of::<f32>()
            + self.minimum_bound_margin.len() * size_of::<f32>()
            + self.hard_projection_sweeps.len() * size_of::<u32>()
            + self.task_sweeps.len() * size_of::<u32>()
            + self.clipped_updates.len() * size_of::<u32>()
            + self.active_hard_rows.len() * size_of::<u32>()
            + self.active_soft_rows.len() * size_of::<u32>()
    }

    pub(crate) fn clear(&mut self) {
        self.generalized_acceleration_soa.fill(0.0);
        self.candidate_generalized_acceleration_soa.fill(0.0);
        self.status.fill(MirrorSolveAgentStatus::Inactive);
        self.level_rows_soa.fill(0);
        self.level_rms_soa.fill(0.0);
        self.level_preservation_drift_soa.fill(0.0);
        self.initial_hard_violation.fill(0.0);
        self.best_hard_violation.fill(0.0);
        self.final_hard_violation.fill(0.0);
        self.minimum_bound_margin.fill(f32::INFINITY);
        self.hard_projection_sweeps.fill(0);
        self.task_sweeps.fill(0);
        self.clipped_updates.fill(0);
        self.active_hard_rows.fill(0);
        self.active_soft_rows.fill(0);
    }

    pub(crate) fn generalized_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.generalized_acceleration_soa
    }
    pub(crate) fn candidate_generalized_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.candidate_generalized_acceleration_soa
    }
    pub(crate) fn set_status(&mut self, agent: usize, status: MirrorSolveAgentStatus) {
        self.status[agent] = status;
    }
    pub(crate) fn level_rows_soa_mut(&mut self) -> &mut [u32] {
        &mut self.level_rows_soa
    }
    pub(crate) fn level_rms_soa_mut(&mut self) -> &mut [f32] {
        &mut self.level_rms_soa
    }
    pub(crate) fn level_preservation_drift_soa_mut(&mut self) -> &mut [f32] {
        &mut self.level_preservation_drift_soa
    }
    pub(crate) fn initial_hard_violation_mut(&mut self) -> &mut [f32] {
        &mut self.initial_hard_violation
    }
    pub(crate) fn best_hard_violation_mut(&mut self) -> &mut [f32] {
        &mut self.best_hard_violation
    }
    pub(crate) fn final_hard_violation_mut(&mut self) -> &mut [f32] {
        &mut self.final_hard_violation
    }
    pub(crate) fn minimum_bound_margin_mut(&mut self) -> &mut [f32] {
        &mut self.minimum_bound_margin
    }
    pub(crate) fn hard_projection_sweeps_mut(&mut self) -> &mut [u32] {
        &mut self.hard_projection_sweeps
    }
    pub(crate) fn task_sweeps_mut(&mut self) -> &mut [u32] {
        &mut self.task_sweeps
    }
    pub(crate) fn clipped_updates_mut(&mut self) -> &mut [u32] {
        &mut self.clipped_updates
    }
    pub(crate) fn active_hard_rows_mut(&mut self) -> &mut [u32] {
        &mut self.active_hard_rows
    }
    pub(crate) fn active_soft_rows_mut(&mut self) -> &mut [u32] {
        &mut self.active_soft_rows
    }
}

#[derive(Debug)]
struct AgentScratch {
    x: Vec<f32>,
    best: Vec<f32>,
    locked_rows: Vec<f32>,
    locked_rhs: Vec<f32>,
    locked_reference_rhs: Vec<f32>,
    locked_priority: Vec<u8>,
    locked_count: usize,
}

impl AgentScratch {
    fn new(dof: usize, maximum_soft_rows: usize) -> Result<Self, MirrorSolveError> {
        let matrix_len = maximum_soft_rows
            .checked_mul(dof)
            .ok_or(MirrorSolveError::SizeOverflow)?;
        Ok(Self {
            x: vec![0.0; dof],
            best: vec![0.0; dof],
            locked_rows: vec![0.0; matrix_len],
            locked_rhs: vec![0.0; maximum_soft_rows],
            locked_reference_rhs: vec![0.0; maximum_soft_rows],
            locked_priority: vec![0; maximum_soft_rows],
            locked_count: 0,
        })
    }
}

#[derive(Debug)]
pub struct CpuMirrorBatchSolver {
    descriptor: BatchProgramDescriptor,
    scratch: Vec<AgentScratch>,
    algorithm_sha256: [u8; 32],
}

impl CpuMirrorBatchSolver {
    pub fn new(descriptor: &BatchProgramDescriptor) -> Result<Self, MirrorSolveError> {
        let maximum_soft_rows = descriptor
            .layout
            .point_task_count
            .checked_mul(3)
            .ok_or(MirrorSolveError::SizeOverflow)?;
        let mut scratch = Vec::with_capacity(descriptor.layout.agent_capacity);
        for _ in 0..descriptor.layout.agent_capacity {
            scratch.push(AgentScratch::new(
                descriptor.layout.generalized_coordinate_count,
                maximum_soft_rows,
            )?);
        }
        let mut digest = Sha256::new();
        digest.update(ALGORITHM_ID.as_bytes());
        digest.update(serde_json::to_vec(descriptor).expect("serializable descriptor"));
        let algorithm_sha256 = digest.finalize().into();
        Ok(Self {
            descriptor: descriptor.clone(),
            scratch,
            algorithm_sha256,
        })
    }

    pub fn descriptor(&self) -> &BatchProgramDescriptor {
        &self.descriptor
    }
    pub const fn semantics(&self) -> MirrorSolveSemantics {
        MirrorSolveSemantics::FixedLevelApproximate
    }
    pub const fn manifest(&self) -> KernelManifest {
        KernelManifest::solve_v1()
    }
    pub const fn algorithm_sha256(&self) -> [u8; 32] {
        self.algorithm_sha256
    }
    pub fn resident_bytes(&self) -> usize {
        self.scratch
            .iter()
            .map(|scratch| {
                (scratch.x.len() + scratch.best.len() + scratch.locked_rows.len())
                    * size_of::<f32>()
                    + (scratch.locked_rhs.len() + scratch.locked_reference_rhs.len())
                        * size_of::<f32>()
                    + scratch.locked_priority.len() * size_of::<u8>()
            })
            .sum()
    }

    pub fn execute_into(
        &mut self,
        emission: &EmissionBatchOutput,
        input: &MirrorSolveBatchInput,
        output: &mut MirrorSolveBatchOutput,
    ) -> Result<(), MirrorSolveError> {
        if emission.layout() != &self.descriptor.layout
            || input.layout() != &self.descriptor.layout
            || output.layout() != &self.descriptor.layout
        {
            return Err(MirrorSolveError::LayoutMismatch);
        }
        output.clear();
        let layout = &self.descriptor.layout;
        let dof = layout.generalized_coordinate_count;
        for agent in 0..layout.agent_capacity {
            if emission.agent_status()[agent] == AgentStatus::InvalidInput {
                output.status[agent] = MirrorSolveAgentStatus::InvalidInput;
                continue;
            }
            if emission.agent_status()[agent] != AgentStatus::Ok || agent >= input.active_agents() {
                continue;
            }
            let scratch = &mut self.scratch[agent];
            scratch.locked_count = 0;
            scratch.x.fill(0.0);
            scratch.best.fill(0.0);
            let mut invalid = false;
            for coordinate in 0..dof {
                let index = layout.generalized_index(coordinate, agent);
                let lower = input.lower_soa()[index];
                let upper = input.upper_soa()[index];
                invalid |= lower.is_nan() || upper.is_nan() || lower > upper;
                scratch.x[coordinate] = clamp_scalar(0.0, lower, upper);
            }
            if invalid {
                output.status[agent] = MirrorSolveAgentStatus::InvalidProblem;
                continue;
            }
            output.active_hard_rows[agent] =
                active_hard_rows(&self.descriptor, emission, agent) as u32;
            output.active_soft_rows[agent] =
                active_soft_rows(&self.descriptor, emission, agent) as u32;
            let initial =
                maximum_hard_violation(&self.descriptor, emission, input, agent, scratch, 0);
            output.initial_hard_violation[agent] = initial;
            let mut best_violation = initial;
            scratch.best.copy_from_slice(&scratch.x);
            let mut clipped = 0_u32;
            for _ in 0..MIRROR_HARD_PROJECTION_SWEEPS {
                clipped = clipped.saturating_add(project_feasible_sweep(
                    &self.descriptor,
                    emission,
                    input,
                    agent,
                    scratch,
                    0,
                ));
                let violation =
                    maximum_hard_violation(&self.descriptor, emission, input, agent, scratch, 0);
                if violation < best_violation {
                    best_violation = violation;
                    scratch.best.copy_from_slice(&scratch.x);
                }
            }
            output.hard_projection_sweeps[agent] = MIRROR_HARD_PROJECTION_SWEEPS;
            if best_violation > HARD_TOLERANCE || !best_violation.is_finite() {
                write_candidate(layout, agent, &scratch.best, output);
                output.best_hard_violation[agent] = best_violation;
                output.final_hard_violation[agent] =
                    maximum_hard_violation(&self.descriptor, emission, input, agent, scratch, 0);
                output.minimum_bound_margin[agent] =
                    minimum_bound_margin(input, layout, agent, &scratch.best);
                output.clipped_updates[agent] = clipped;
                output.status[agent] = MirrorSolveAgentStatus::MaxIterations;
                continue;
            }
            scratch.x.copy_from_slice(&scratch.best);

            for priority in Priority::ALL {
                let level_index = priority as usize * layout.agent_stride + agent;
                let rows = active_level_rows(&self.descriptor, emission, agent, priority);
                output.level_rows_soa[level_index] = rows as u32;
                if rows == 0 {
                    continue;
                }
                for _ in 0..MIRROR_TASK_SWEEPS_PER_LEVEL {
                    for (slot, task) in self.descriptor.point_tasks.iter().enumerate() {
                        if task.priority != priority
                            || emission.point_task_active_soa()
                                [layout.task_active_index(slot, agent)]
                                == 0
                        {
                            continue;
                        }
                        for component in 0..3 {
                            let norm =
                                task_row_norm_squared(layout, emission, slot, component, agent);
                            if norm <= ROW_NORM_EPSILON {
                                continue;
                            }
                            let target = emission.point_task_rhs_soa()
                                [layout.task_vector_index(slot, component, agent)];
                            let residual = target
                                - task_row_dot(
                                    layout, emission, slot, component, agent, &scratch.x,
                                );
                            let weight = task.weight().max(0.0);
                            let alpha =
                                RELAXATION * weight / (weight.mul_add(norm, REGULARIZATION));
                            for coordinate in 0..dof {
                                let coefficient = emission.point_task_jacobian_soa()[layout
                                    .task_jacobian_index(slot, component, coordinate, agent)];
                                scratch.x[coordinate] =
                                    (alpha * residual).mul_add(coefficient, scratch.x[coordinate]);
                            }
                            clipped = clipped.saturating_add(clamp_to_bounds(
                                input,
                                layout,
                                agent,
                                &mut scratch.x,
                            ));
                            for _ in 0..MIRROR_RESTORE_SWEEPS {
                                clipped = clipped.saturating_add(project_feasible_sweep(
                                    &self.descriptor,
                                    emission,
                                    input,
                                    agent,
                                    scratch,
                                    scratch.locked_count,
                                ));
                            }
                        }
                    }
                }
                output.task_sweeps[agent] =
                    output.task_sweeps[agent].saturating_add(MIRROR_TASK_SWEEPS_PER_LEVEL);
                output.level_rms_soa[level_index] =
                    level_rms(&self.descriptor, emission, agent, priority, &scratch.x);
                append_level_locks(&self.descriptor, emission, agent, priority, scratch);
            }

            for priority in Priority::ALL {
                let level_index = priority as usize * layout.agent_stride + agent;
                output.level_preservation_drift_soa[level_index] =
                    maximum_locked_drift(dof, scratch, priority as u8);
            }

            let final_hard = maximum_hard_violation(
                &self.descriptor,
                emission,
                input,
                agent,
                scratch,
                scratch.locked_count,
            );
            best_violation = best_violation.min(maximum_hard_violation(
                &self.descriptor,
                emission,
                input,
                agent,
                scratch,
                0,
            ));
            write_candidate(layout, agent, &scratch.x, output);
            output.best_hard_violation[agent] = best_violation;
            output.final_hard_violation[agent] = final_hard;
            output.minimum_bound_margin[agent] =
                minimum_bound_margin(input, layout, agent, &scratch.x);
            output.clipped_updates[agent] = clipped;
            if final_hard > HARD_TOLERANCE || !final_hard.is_finite() {
                output.status[agent] = MirrorSolveAgentStatus::MaxIterations;
                continue;
            }
            for coordinate in 0..dof {
                let index = layout.generalized_index(coordinate, agent);
                output.generalized_acceleration_soa[index] = scratch.x[coordinate];
            }
            let has_residual = Priority::ALL.into_iter().any(|priority| {
                output.level_rms_soa[priority as usize * layout.agent_stride + agent]
                    > SOFT_RESIDUAL_TOLERANCE
            });
            output.status[agent] = if has_residual || clipped > 0 {
                MirrorSolveAgentStatus::SolvedWithResidual
            } else {
                MirrorSolveAgentStatus::Solved
            };
        }
        Ok(())
    }
}

fn clamp_scalar(value: f32, lower: f32, upper: f32) -> f32 {
    value.max(lower).min(upper)
}

fn clamp_to_bounds(
    input: &MirrorSolveBatchInput,
    layout: &BatchLayout,
    agent: usize,
    x: &mut [f32],
) -> u32 {
    let mut clipped = 0_u32;
    for (coordinate, value) in x.iter_mut().enumerate() {
        let index = layout.generalized_index(coordinate, agent);
        let clamped = clamp_scalar(*value, input.lower_soa()[index], input.upper_soa()[index]);
        clipped = clipped.saturating_add(u32::from(clamped.to_bits() != value.to_bits()));
        *value = clamped;
    }
    clipped
}

fn contact_row_dot(
    layout: &BatchLayout,
    emission: &EmissionBatchOutput,
    slot: usize,
    component: usize,
    agent: usize,
    x: &[f32],
) -> f32 {
    let mut value = 0.0_f32;
    for (coordinate, x) in x.iter().copied().enumerate() {
        value = emission.contact_lock_jacobian_soa()
            [layout.contact_jacobian_index(slot, component, coordinate, agent)]
        .mul_add(x, value);
    }
    value
}

fn contact_row_norm_squared(
    layout: &BatchLayout,
    emission: &EmissionBatchOutput,
    slot: usize,
    component: usize,
    agent: usize,
) -> f32 {
    let mut value = 0.0_f32;
    for coordinate in 0..layout.generalized_coordinate_count {
        let coefficient = emission.contact_lock_jacobian_soa()
            [layout.contact_jacobian_index(slot, component, coordinate, agent)];
        value = coefficient.mul_add(coefficient, value);
    }
    value
}

fn locked_row_dot(dof: usize, scratch: &AgentScratch, row: usize, x: &[f32]) -> f32 {
    let mut value = 0.0_f32;
    let offset = row * dof;
    for coordinate in 0..dof {
        value = scratch.locked_rows[offset + coordinate].mul_add(x[coordinate], value);
    }
    value
}

fn locked_row_norm_squared(dof: usize, scratch: &AgentScratch, row: usize) -> f32 {
    let mut value = 0.0_f32;
    let offset = row * dof;
    for coordinate in 0..dof {
        let coefficient = scratch.locked_rows[offset + coordinate];
        value = coefficient.mul_add(coefficient, value);
    }
    value
}

fn project_feasible_sweep(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    input: &MirrorSolveBatchInput,
    agent: usize,
    scratch: &mut AgentScratch,
    locked_count: usize,
) -> u32 {
    let layout = &descriptor.layout;
    let dof = layout.generalized_coordinate_count;
    let mut clipped = 0_u32;
    for slot in 0..layout.contact_lock_count {
        if emission.contact_lock_active_soa()[layout.contact_active_index(slot, agent)] == 0 {
            continue;
        }
        for component in 0..3 {
            let norm = contact_row_norm_squared(layout, emission, slot, component, agent);
            if norm <= ROW_NORM_EPSILON {
                continue;
            }
            let rhs = emission.contact_lock_rhs_soa()
                [layout.contact_vector_index(slot, component, agent)];
            let residual =
                rhs - contact_row_dot(layout, emission, slot, component, agent, &scratch.x);
            let alpha = residual / norm;
            for coordinate in 0..dof {
                let coefficient = emission.contact_lock_jacobian_soa()
                    [layout.contact_jacobian_index(slot, component, coordinate, agent)];
                scratch.x[coordinate] = alpha.mul_add(coefficient, scratch.x[coordinate]);
            }
            clipped = clipped.saturating_add(clamp_to_bounds(input, layout, agent, &mut scratch.x));
        }
    }
    for row in 0..locked_count {
        let norm = locked_row_norm_squared(dof, scratch, row);
        if norm <= ROW_NORM_EPSILON {
            continue;
        }
        let residual = scratch.locked_rhs[row] - locked_row_dot(dof, scratch, row, &scratch.x);
        let alpha = residual / norm;
        let offset = row * dof;
        for coordinate in 0..dof {
            scratch.x[coordinate] = alpha.mul_add(
                scratch.locked_rows[offset + coordinate],
                scratch.x[coordinate],
            );
        }
        clipped = clipped.saturating_add(clamp_to_bounds(input, layout, agent, &mut scratch.x));
    }
    clipped
}

fn maximum_hard_violation(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    input: &MirrorSolveBatchInput,
    agent: usize,
    scratch: &AgentScratch,
    locked_count: usize,
) -> f32 {
    let layout = &descriptor.layout;
    let dof = layout.generalized_coordinate_count;
    let mut maximum = 0.0_f32;
    for slot in 0..layout.contact_lock_count {
        if emission.contact_lock_active_soa()[layout.contact_active_index(slot, agent)] == 0 {
            continue;
        }
        for component in 0..3 {
            let rhs = emission.contact_lock_rhs_soa()
                [layout.contact_vector_index(slot, component, agent)];
            maximum = maximum.max(
                (contact_row_dot(layout, emission, slot, component, agent, &scratch.x) - rhs).abs(),
            );
        }
    }
    for row in 0..locked_count {
        maximum = maximum
            .max((locked_row_dot(dof, scratch, row, &scratch.x) - scratch.locked_rhs[row]).abs());
    }
    for coordinate in 0..dof {
        let index = layout.generalized_index(coordinate, agent);
        maximum = maximum.max((input.lower_soa()[index] - scratch.x[coordinate]).max(0.0));
        maximum = maximum.max((scratch.x[coordinate] - input.upper_soa()[index]).max(0.0));
    }
    maximum
}

fn task_row_dot(
    layout: &BatchLayout,
    emission: &EmissionBatchOutput,
    slot: usize,
    component: usize,
    agent: usize,
    x: &[f32],
) -> f32 {
    let mut value = 0.0_f32;
    for (coordinate, x) in x.iter().copied().enumerate() {
        value = emission.point_task_jacobian_soa()
            [layout.task_jacobian_index(slot, component, coordinate, agent)]
        .mul_add(x, value);
    }
    value
}

fn task_row_norm_squared(
    layout: &BatchLayout,
    emission: &EmissionBatchOutput,
    slot: usize,
    component: usize,
    agent: usize,
) -> f32 {
    let mut value = 0.0_f32;
    for coordinate in 0..layout.generalized_coordinate_count {
        let coefficient = emission.point_task_jacobian_soa()
            [layout.task_jacobian_index(slot, component, coordinate, agent)];
        value = coefficient.mul_add(coefficient, value);
    }
    value
}

fn active_hard_rows(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    agent: usize,
) -> usize {
    descriptor
        .contact_locks
        .iter()
        .enumerate()
        .filter(|(slot, _)| {
            emission.contact_lock_active_soa()[descriptor.layout.contact_active_index(*slot, agent)]
                != 0
        })
        .map(|(_, contact)| {
            (0..3)
                .filter(|component| contact.kinematic_mode.axis_enabled(*component))
                .count()
        })
        .sum()
}

fn active_soft_rows(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    agent: usize,
) -> usize {
    descriptor
        .point_tasks
        .iter()
        .enumerate()
        .filter(|(slot, _)| {
            emission.point_task_active_soa()[descriptor.layout.task_active_index(*slot, agent)] != 0
        })
        .count()
        * 3
}

fn active_level_rows(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    agent: usize,
    priority: Priority,
) -> usize {
    descriptor
        .point_tasks
        .iter()
        .enumerate()
        .filter(|(slot, task)| {
            task.priority == priority
                && emission.point_task_active_soa()
                    [descriptor.layout.task_active_index(*slot, agent)]
                    != 0
        })
        .count()
        * 3
}

fn level_rms(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    agent: usize,
    priority: Priority,
    x: &[f32],
) -> f32 {
    let layout = &descriptor.layout;
    let mut squared = 0.0_f32;
    let mut rows = 0_usize;
    for (slot, task) in descriptor.point_tasks.iter().enumerate() {
        if task.priority != priority
            || emission.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0
        {
            continue;
        }
        for component in 0..3 {
            let target =
                emission.point_task_rhs_soa()[layout.task_vector_index(slot, component, agent)];
            let residual = task_row_dot(layout, emission, slot, component, agent, x) - target;
            squared = residual.mul_add(residual, squared);
            rows += 1;
        }
    }
    if rows == 0 {
        0.0
    } else {
        (squared / rows as f32).sqrt()
    }
}

fn append_level_locks(
    descriptor: &BatchProgramDescriptor,
    emission: &EmissionBatchOutput,
    agent: usize,
    priority: Priority,
    scratch: &mut AgentScratch,
) {
    let layout = &descriptor.layout;
    let dof = layout.generalized_coordinate_count;
    for (slot, task) in descriptor.point_tasks.iter().enumerate() {
        if task.priority != priority
            || emission.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0
        {
            continue;
        }
        for component in 0..3 {
            let row = scratch.locked_count;
            let offset = row * dof;
            for coordinate in 0..dof {
                scratch.locked_rows[offset + coordinate] = emission.point_task_jacobian_soa()
                    [layout.task_jacobian_index(slot, component, coordinate, agent)];
            }
            let achieved = task_row_dot(layout, emission, slot, component, agent, &scratch.x);
            scratch.locked_rhs[row] = achieved;
            scratch.locked_reference_rhs[row] = achieved;
            scratch.locked_priority[row] = priority as u8;
            scratch.locked_count += 1;
        }
    }
}

fn maximum_locked_drift(dof: usize, scratch: &AgentScratch, through_priority: u8) -> f32 {
    let mut maximum = 0.0_f32;
    for row in 0..scratch.locked_count {
        if scratch.locked_priority[row] <= through_priority {
            maximum = maximum.max(
                (locked_row_dot(dof, scratch, row, &scratch.x) - scratch.locked_reference_rhs[row])
                    .abs(),
            );
        }
    }
    maximum
}

fn minimum_bound_margin(
    input: &MirrorSolveBatchInput,
    layout: &BatchLayout,
    agent: usize,
    x: &[f32],
) -> f32 {
    let mut minimum = f32::INFINITY;
    for (coordinate, value) in x.iter().copied().enumerate() {
        let index = layout.generalized_index(coordinate, agent);
        minimum =
            minimum.min((value - input.lower_soa()[index]).min(input.upper_soa()[index] - value));
    }
    minimum
}

fn write_candidate(
    layout: &BatchLayout,
    agent: usize,
    candidate: &[f32],
    output: &mut MirrorSolveBatchOutput,
) {
    for (coordinate, value) in candidate.iter().copied().enumerate() {
        output.candidate_generalized_acceleration_soa
            [layout.generalized_index(coordinate, agent)] = value;
    }
}

#[cfg(test)]
mod tests {
    use std::hint::black_box;

    use bonesaw_core::{MotionProgram, TimingSpec};

    use super::*;
    use crate::{
        ContactLockSpec, CpuExactBatchSolver, CpuMirrorExecutor, ExactSolveAgentStatus,
        ExactSolveBatchInput, ExactSolveBatchOutput, PointAttractorSpec, PointQuerySpec,
        allocation_sentinel, layouts::TaskVectorStorage,
    };

    fn analytic_case() -> (
        CpuMirrorExecutor,
        EmissionBatchOutput,
        MirrorSolveBatchInput,
        ExactSolveBatchInput,
    ) {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let queries = [PointQuerySpec {
            stable_id: 97_001,
            frame_index: program.model.root.0,
            point_in_frame: [0.0, 0.0, 0.0],
        }];
        let tasks = [
            PointAttractorSpec {
                stable_id: 97_101,
                point_query_stable_id: 97_001,
                priority: Priority::Viability,
                weight: 1.0,
                bandwidth_hz: 1.0,
            },
            PointAttractorSpec {
                stable_id: 97_102,
                point_query_stable_id: 97_001,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 1.0,
            },
        ];
        let contacts = [
            ContactLockSpec {
                stable_id: 97_201,
                point_query_stable_id: 97_001,
            },
            ContactLockSpec {
                stable_id: 97_202,
                point_query_stable_id: 97_001,
            },
        ];
        let executor = CpuMirrorExecutor::compile_with_emission_plan(
            &program, 5, 8, &queries, &tasks, &contacts,
        )
        .unwrap();
        let layout = executor.descriptor().layout.clone();
        let mut emission = EmissionBatchOutput::new(layout.clone());
        for agent in [0, 1, 2, 4] {
            emission.set_status(agent, AgentStatus::Ok);
            for slot in 0..2 {
                emission.set_task_active(slot, agent);
                emission.set_task_jacobian(slot, 0, 1, agent, 1.0);
                emission.set_task_vector(
                    TaskVectorStorage::RightHandSide,
                    slot,
                    agent,
                    [if slot == 0 { 1.0 } else { -1.0 }, 0.0, 0.0],
                );
            }
            emission.set_contact_active(0, agent);
            emission.set_contact_jacobian(0, 0, 0, agent, 1.0);
            emission.set_contact_rhs(0, agent, [0.25, 0.0, 0.0]);
        }
        emission.set_contact_active(1, 1);
        emission.set_contact_jacobian(1, 0, 0, 1, 1.0);
        emission.set_contact_rhs(1, 1, [1.25, 0.0, 0.0]);
        emission.set_status(3, AgentStatus::InvalidInput);

        let mut mirror_input = MirrorSolveBatchInput::new(layout.clone(), 5).unwrap();
        mirror_input.lower_soa_mut().fill(-2.0);
        mirror_input.upper_soa_mut().fill(2.0);
        let x1_agent4 = layout.generalized_index(1, 4);
        mirror_input.lower_soa_mut()[x1_agent4] = -0.1;
        mirror_input.upper_soa_mut()[x1_agent4] = 0.1;
        let invalid_agent2 = layout.generalized_index(0, 2);
        mirror_input.lower_soa_mut()[invalid_agent2] = 1.0;
        mirror_input.upper_soa_mut()[invalid_agent2] = -1.0;

        let mut exact_input = ExactSolveBatchInput::new(layout.clone(), 5).unwrap();
        for coordinate in 0..layout.generalized_coordinate_count {
            for agent in 0..layout.agent_stride {
                let index = layout.generalized_index(coordinate, agent);
                exact_input.lower_soa_mut()[index] = mirror_input.lower_soa()[index] as f64;
                exact_input.upper_soa_mut()[index] = mirror_input.upper_soa()[index] as f64;
            }
        }
        (executor, emission, mirror_input, exact_input)
    }

    #[test]
    fn mirror_solver_preserves_priority_reports_budget_and_matches_exact_semantics() {
        let (executor, emission, mirror_input, exact_input) = analytic_case();
        let layout = executor.descriptor().layout.clone();
        let mut mirror_solver = CpuMirrorBatchSolver::new(executor.descriptor()).unwrap();
        let mut mirror = MirrorSolveBatchOutput::new(layout.clone());
        mirror_solver
            .execute_into(&emission, &mirror_input, &mut mirror)
            .unwrap();

        let mut exact_solver = CpuExactBatchSolver::new(executor.descriptor()).unwrap();
        let mut exact = ExactSolveBatchOutput::new(layout.clone());
        exact_solver
            .execute_into(&emission, &exact_input, &mut exact)
            .unwrap();

        assert_eq!(
            mirror.semantics(),
            MirrorSolveSemantics::FixedLevelApproximate
        );
        assert_eq!(
            mirror.status()[0],
            MirrorSolveAgentStatus::SolvedWithResidual
        );
        assert!(matches!(
            exact.status()[0],
            ExactSolveAgentStatus::Solved | ExactSolveAgentStatus::SolvedWithSlack
        ));
        let x0 = mirror.generalized_acceleration_soa()[layout.generalized_index(0, 0)];
        let x1 = mirror.generalized_acceleration_soa()[layout.generalized_index(1, 0)];
        assert!((x0 - 0.25).abs() <= HARD_TOLERANCE);
        assert!((x1 - 1.0).abs() <= 2.0e-3, "x1={x1}");
        assert!(
            mirror.level_rms_soa()[Priority::Viability as usize * layout.agent_stride] <= 2.0e-3
        );
        // RMS is normalized over all three Cartesian rows. Only x conflicts,
        // so a magnitude-two residual reports as 2/sqrt(3).
        assert!(mirror.level_rms_soa()[Priority::Intent as usize * layout.agent_stride] >= 1.1);
        assert!(
            mirror.level_preservation_drift_soa()
                [Priority::Viability as usize * layout.agent_stride]
                <= HARD_TOLERANCE
        );
        assert!(mirror.final_hard_violation()[0] <= HARD_TOLERANCE);
        assert!(
            (exact.generalized_acceleration_soa()[layout.generalized_index(0, 0)] - x0 as f64)
                .abs()
                <= 2.0e-3
        );
        assert!(
            (exact.level_l2_soa()[Priority::Viability as usize * layout.agent_stride] as f32
                - mirror.level_rms_soa()[Priority::Viability as usize * layout.agent_stride])
                .abs()
                <= 2.0e-3
        );

        assert_eq!(mirror.status()[1], MirrorSolveAgentStatus::MaxIterations);
        assert_eq!(exact.status()[1], ExactSolveAgentStatus::MaxIterations);
        assert!(mirror.best_hard_violation()[1] > 0.1);
        assert!(mirror.initial_hard_violation()[1].is_finite());
        assert!(mirror.final_hard_violation()[1].is_finite());
        assert!(
            mirror
                .generalized_acceleration_soa()
                .iter()
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride == 1)
                .all(|(_, value)| value.to_bits() == 0)
        );
        assert!(
            mirror
                .candidate_generalized_acceleration_soa()
                .iter()
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride == 1)
                .all(|(_, value)| value.is_finite())
        );

        assert_eq!(mirror.status()[2], MirrorSolveAgentStatus::InvalidProblem);
        assert_eq!(mirror.status()[3], MirrorSolveAgentStatus::InvalidInput);
        assert_eq!(
            mirror.status()[4],
            MirrorSolveAgentStatus::SolvedWithResidual
        );
        let bounded = mirror.generalized_acceleration_soa()[layout.generalized_index(1, 4)];
        assert!((-0.1..=0.1).contains(&bounded));
        assert_eq!(mirror.status()[5], MirrorSolveAgentStatus::Inactive);
        assert!(
            mirror
                .generalized_acceleration_soa()
                .iter()
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride >= 5)
                .all(|(_, value)| value.to_bits() == 0)
        );
    }

    #[test]
    fn mirror_solver_replays_bitwise_without_allocation_or_neighbor_poisoning() {
        let (executor, emission, input, _) = analytic_case();
        let layout = executor.descriptor().layout.clone();
        let mut solver = CpuMirrorBatchSolver::new(executor.descriptor()).unwrap();
        let mut first = MirrorSolveBatchOutput::new(layout.clone());
        let mut repeated = MirrorSolveBatchOutput::new(layout);
        solver.execute_into(&emission, &input, &mut first).unwrap();
        solver
            .execute_into(&emission, &input, &mut repeated)
            .unwrap();
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                black_box(solver.execute_into(
                    black_box(&emission),
                    black_box(&input),
                    black_box(&mut repeated),
                ))
                .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        assert_eq!(
            repeated.generalized_acceleration_soa(),
            first.generalized_acceleration_soa()
        );
        assert_eq!(
            repeated.candidate_generalized_acceleration_soa(),
            first.candidate_generalized_acceleration_soa()
        );
        assert_eq!(repeated.status(), first.status());
        assert_eq!(repeated.level_rms_soa(), first.level_rms_soa());
        assert_eq!(
            repeated.level_preservation_drift_soa(),
            first.level_preservation_drift_soa()
        );
        assert_eq!(
            repeated.initial_hard_violation(),
            first.initial_hard_violation()
        );
        assert_eq!(repeated.best_hard_violation(), first.best_hard_violation());
        assert_eq!(
            repeated.final_hard_violation(),
            first.final_hard_violation()
        );
    }
}
