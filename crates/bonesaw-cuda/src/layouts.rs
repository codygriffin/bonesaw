use bonesaw_core::{CompiledModel, MotionProgram, RobotState, TaskOp};
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const POSE_COMPONENTS: usize = 12;
pub const SPATIAL_COMPONENTS: usize = 6;
pub const ROOT_TANGENT_COMPONENTS: usize = 6;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct BatchLayout {
    pub program_fingerprint: [u8; 32],
    pub coordinate_count: usize,
    pub generalized_coordinate_count: usize,
    pub body_count: usize,
    pub joint_count: usize,
    pub point_query_count: usize,
    pub point_task_count: usize,
    pub contact_lock_count: usize,
    pub agent_capacity: usize,
    pub agent_stride: usize,
    pub pose_components: usize,
    pub spatial_components: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum AgentStatus {
    Inactive = 0,
    Ok = 1,
    InvalidInput = 2,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum BatchLayoutError {
    #[error("agent capacity and alignment must be positive")]
    EmptyCapacity,
    #[error("batch layout size overflows usize")]
    SizeOverflow,
}

#[derive(Debug, Error)]
pub enum BatchInputError {
    #[error("active agent count {actual} exceeds capacity {capacity}")]
    ActiveAgents { actual: usize, capacity: usize },
    #[error("agent {agent} is outside the active range {active_agents}")]
    AgentOutOfRange { agent: usize, active_agents: usize },
    #[error("state model does not match the compiled batch layout")]
    ModelMismatch,
    #[error(transparent)]
    InvalidState(#[from] bonesaw_core::model::ModelError),
}

impl BatchLayout {
    pub fn compile(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
    ) -> Result<Self, BatchLayoutError> {
        let point_query_count = program
            .tasks
            .ops()
            .iter()
            .filter(|operation| matches!(operation, TaskOp::Point { .. }))
            .count();
        Self::compile_with_plan_counts(
            program,
            agent_capacity,
            agent_alignment,
            point_query_count,
            point_query_count,
            0,
        )
    }

    pub fn compile_with_point_query_count(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_query_count: usize,
    ) -> Result<Self, BatchLayoutError> {
        Self::compile_with_plan_counts(
            program,
            agent_capacity,
            agent_alignment,
            point_query_count,
            0,
            0,
        )
    }

    pub fn compile_with_plan_counts(
        program: &MotionProgram,
        agent_capacity: usize,
        agent_alignment: usize,
        point_query_count: usize,
        point_task_count: usize,
        contact_lock_count: usize,
    ) -> Result<Self, BatchLayoutError> {
        if agent_capacity == 0 || agent_alignment == 0 {
            return Err(BatchLayoutError::EmptyCapacity);
        }
        let rounded = agent_capacity
            .checked_add(agent_alignment - 1)
            .ok_or(BatchLayoutError::SizeOverflow)?;
        let agent_stride = (rounded / agent_alignment)
            .checked_mul(agent_alignment)
            .ok_or(BatchLayoutError::SizeOverflow)?;
        let layout = Self {
            program_fingerprint: program.header.fingerprint_sha256,
            coordinate_count: program.model.dof,
            generalized_coordinate_count: program
                .model
                .dof
                .checked_add(ROOT_TANGENT_COMPONENTS)
                .ok_or(BatchLayoutError::SizeOverflow)?,
            body_count: program.model.bodies.len(),
            joint_count: program.model.joints.len(),
            point_query_count,
            point_task_count,
            contact_lock_count,
            agent_capacity,
            agent_stride,
            pose_components: POSE_COMPONENTS,
            spatial_components: SPATIAL_COMPONENTS,
        };
        layout.q_len()?;
        layout.root_pose_len()?;
        layout.body_pose_len()?;
        layout.frame_jacobian_len()?;
        layout.center_of_mass_jacobian_len()?;
        layout.generalized_velocity_len()?;
        layout.mass_matrix_len()?;
        layout.centroidal_map_len()?;
        layout.point_position_len()?;
        layout.point_jacobian_len()?;
        layout.point_bias_acceleration_len()?;
        layout.point_task_active_len()?;
        layout.point_task_vector_len()?;
        layout.point_task_jacobian_len()?;
        layout.contact_lock_active_len()?;
        layout.contact_lock_vector_len()?;
        layout.contact_lock_jacobian_len()?;
        Ok(layout)
    }

    pub fn q_len(&self) -> Result<usize, BatchLayoutError> {
        self.coordinate_count
            .checked_mul(self.agent_stride)
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn root_pose_len(&self) -> Result<usize, BatchLayoutError> {
        self.pose_components
            .checked_mul(self.agent_stride)
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn body_pose_len(&self) -> Result<usize, BatchLayoutError> {
        self.body_count
            .checked_mul(self.pose_components)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn frame_jacobian_len(&self) -> Result<usize, BatchLayoutError> {
        self.body_count
            .checked_mul(self.spatial_components)
            .and_then(|count| count.checked_mul(self.generalized_coordinate_count))
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn center_of_mass_jacobian_len(&self) -> Result<usize, BatchLayoutError> {
        3_usize
            .checked_mul(self.generalized_coordinate_count)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn generalized_velocity_len(&self) -> Result<usize, BatchLayoutError> {
        self.generalized_coordinate_count
            .checked_mul(self.agent_stride)
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn mass_matrix_len(&self) -> Result<usize, BatchLayoutError> {
        self.generalized_coordinate_count
            .checked_mul(self.generalized_coordinate_count)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn centroidal_map_len(&self) -> Result<usize, BatchLayoutError> {
        SPATIAL_COMPONENTS
            .checked_mul(self.generalized_coordinate_count)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn point_position_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_query_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn point_jacobian_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_query_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.generalized_coordinate_count))
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn point_bias_acceleration_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_position_len()
    }

    pub fn point_task_active_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_task_count
            .checked_mul(self.agent_stride)
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn point_task_vector_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_task_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn point_task_jacobian_len(&self) -> Result<usize, BatchLayoutError> {
        self.point_task_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.generalized_coordinate_count))
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn contact_lock_active_len(&self) -> Result<usize, BatchLayoutError> {
        self.contact_lock_count
            .checked_mul(self.agent_stride)
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn contact_lock_vector_len(&self) -> Result<usize, BatchLayoutError> {
        self.contact_lock_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub fn contact_lock_jacobian_len(&self) -> Result<usize, BatchLayoutError> {
        self.contact_lock_count
            .checked_mul(3)
            .and_then(|count| count.checked_mul(self.generalized_coordinate_count))
            .and_then(|count| count.checked_mul(self.agent_stride))
            .ok_or(BatchLayoutError::SizeOverflow)
    }

    pub const fn q_index(&self, coordinate: usize, agent: usize) -> usize {
        coordinate * self.agent_stride + agent
    }

    pub const fn root_pose_index(&self, component: usize, agent: usize) -> usize {
        component * self.agent_stride + agent
    }

    pub const fn body_pose_index(&self, body: usize, component: usize, agent: usize) -> usize {
        (body * self.pose_components + component) * self.agent_stride + agent
    }

    pub const fn com_index(&self, component: usize, agent: usize) -> usize {
        component * self.agent_stride + agent
    }

    pub const fn frame_jacobian_index(
        &self,
        body: usize,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        ((body * self.spatial_components + component) * self.generalized_coordinate_count
            + generalized_coordinate)
            * self.agent_stride
            + agent
    }

    pub const fn center_of_mass_jacobian_index(
        &self,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        (component * self.generalized_coordinate_count + generalized_coordinate) * self.agent_stride
            + agent
    }

    pub const fn generalized_index(&self, generalized_coordinate: usize, agent: usize) -> usize {
        generalized_coordinate * self.agent_stride + agent
    }

    pub const fn mass_matrix_index(&self, row: usize, column: usize, agent: usize) -> usize {
        (row * self.generalized_coordinate_count + column) * self.agent_stride + agent
    }

    pub const fn centroidal_map_index(
        &self,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        (component * self.generalized_coordinate_count + generalized_coordinate) * self.agent_stride
            + agent
    }

    pub const fn point_position_index(&self, slot: usize, component: usize, agent: usize) -> usize {
        (slot * 3 + component) * self.agent_stride + agent
    }

    pub const fn point_jacobian_index(
        &self,
        slot: usize,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        ((slot * 3 + component) * self.generalized_coordinate_count + generalized_coordinate)
            * self.agent_stride
            + agent
    }

    pub const fn task_active_index(&self, slot: usize, agent: usize) -> usize {
        slot * self.agent_stride + agent
    }

    pub const fn task_vector_index(&self, slot: usize, component: usize, agent: usize) -> usize {
        (slot * 3 + component) * self.agent_stride + agent
    }

    pub const fn task_jacobian_index(
        &self,
        slot: usize,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        ((slot * 3 + component) * self.generalized_coordinate_count + generalized_coordinate)
            * self.agent_stride
            + agent
    }

    pub const fn contact_active_index(&self, slot: usize, agent: usize) -> usize {
        slot * self.agent_stride + agent
    }

    pub const fn contact_vector_index(&self, slot: usize, component: usize, agent: usize) -> usize {
        (slot * 3 + component) * self.agent_stride + agent
    }

    pub const fn contact_jacobian_index(
        &self,
        slot: usize,
        component: usize,
        generalized_coordinate: usize,
        agent: usize,
    ) -> usize {
        ((slot * 3 + component) * self.generalized_coordinate_count + generalized_coordinate)
            * self.agent_stride
            + agent
    }
}

#[derive(Clone, Debug)]
pub struct FkBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    q_soa: Vec<f32>,
    root_pose_soa: Vec<f32>,
}

impl FkBatchInput {
    pub fn new(layout: BatchLayout, active_agents: usize) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        let mut input = Self {
            q_soa: vec![0.0; layout.q_len().expect("validated batch layout")],
            root_pose_soa: vec![0.0; layout.root_pose_len().expect("validated batch layout")],
            layout,
            active_agents,
        };
        for agent in 0..input.layout.agent_stride {
            for diagonal in [0, 4, 8] {
                let index = input.layout.root_pose_index(diagonal, agent);
                input.root_pose_soa[index] = 1.0;
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

    pub fn q_soa(&self) -> &[f32] {
        &self.q_soa
    }

    pub fn q_soa_mut(&mut self) -> &mut [f32] {
        &mut self.q_soa
    }

    pub fn root_pose_soa(&self) -> &[f32] {
        &self.root_pose_soa
    }

    pub fn root_pose_soa_mut(&mut self) -> &mut [f32] {
        &mut self.root_pose_soa
    }

    pub fn set_agent_from_state(
        &mut self,
        model: &CompiledModel,
        agent: usize,
        state: &RobotState,
    ) -> Result<(), BatchInputError> {
        if agent >= self.active_agents {
            return Err(BatchInputError::AgentOutOfRange {
                agent,
                active_agents: self.active_agents,
            });
        }
        if model.dof != self.layout.coordinate_count
            || model.bodies.len() != self.layout.body_count
            || model.joints.len() != self.layout.joint_count
        {
            return Err(BatchInputError::ModelMismatch);
        }
        state.validate(model)?;
        for coordinate in 0..self.layout.coordinate_count {
            let index = self.layout.q_index(coordinate, agent);
            self.q_soa[index] = state.q[coordinate] as f32;
        }
        let rotation = state.control_world_from_root.rotation.to_rotation_matrix();
        let matrix = rotation.matrix();
        for row in 0..3 {
            for column in 0..3 {
                let component = row * 3 + column;
                let index = self.layout.root_pose_index(component, agent);
                self.root_pose_soa[index] = matrix[(row, column)] as f32;
            }
        }
        for component in 0..3 {
            let index = self.layout.root_pose_index(9 + component, agent);
            self.root_pose_soa[index] =
                state.control_world_from_root.translation.vector[component] as f32;
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct FkBatchOutput {
    layout: BatchLayout,
    body_pose_soa: Vec<f32>,
    center_of_mass_soa: Vec<f32>,
    total_mass: Vec<f32>,
    agent_status: Vec<AgentStatus>,
}

impl FkBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        Self {
            body_pose_soa: vec![0.0; layout.body_pose_len().expect("validated batch layout")],
            center_of_mass_soa: vec![0.0; 3 * layout.agent_stride],
            total_mass: vec![0.0; layout.agent_stride],
            agent_status: vec![AgentStatus::Inactive; layout.agent_stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn body_pose_soa(&self) -> &[f32] {
        &self.body_pose_soa
    }

    pub fn body_pose_soa_mut(&mut self) -> &mut [f32] {
        &mut self.body_pose_soa
    }

    pub fn center_of_mass_soa(&self) -> &[f32] {
        &self.center_of_mass_soa
    }

    pub fn center_of_mass_soa_mut(&mut self) -> &mut [f32] {
        &mut self.center_of_mass_soa
    }

    pub fn total_mass(&self) -> &[f32] {
        &self.total_mass
    }

    pub(crate) fn total_mass_mut(&mut self) -> &mut [f32] {
        &mut self.total_mass
    }

    pub fn agent_status(&self) -> &[AgentStatus] {
        &self.agent_status
    }

    pub fn body_pose(&self, agent: usize, body: usize) -> Option<[f32; POSE_COMPONENTS]> {
        if agent >= self.layout.agent_capacity || body >= self.layout.body_count {
            return None;
        }
        let mut pose = [0.0; POSE_COMPONENTS];
        for (component, value) in pose.iter_mut().enumerate() {
            *value = self.body_pose_soa[self.layout.body_pose_index(body, component, agent)];
        }
        Some(pose)
    }

    pub fn center_of_mass(&self, agent: usize) -> Option<[f32; 3]> {
        if agent >= self.layout.agent_capacity {
            return None;
        }
        Some([
            self.center_of_mass_soa[self.layout.com_index(0, agent)],
            self.center_of_mass_soa[self.layout.com_index(1, agent)],
            self.center_of_mass_soa[self.layout.com_index(2, agent)],
        ])
    }

    pub(crate) fn clear(&mut self) {
        self.body_pose_soa.fill(0.0);
        self.center_of_mass_soa.fill(0.0);
        self.total_mass.fill(0.0);
        self.agent_status.fill(AgentStatus::Inactive);
    }

    pub(crate) fn set_status(&mut self, agent: usize, status: AgentStatus) {
        self.agent_status[agent] = status;
    }

    pub(crate) fn set_body_pose(
        &mut self,
        agent: usize,
        body: usize,
        pose: &[f32; POSE_COMPONENTS],
    ) {
        for (component, value) in pose.iter().copied().enumerate() {
            let index = self.layout.body_pose_index(body, component, agent);
            self.body_pose_soa[index] = value;
        }
    }

    pub(crate) fn set_center_of_mass(&mut self, agent: usize, value: [f32; 3], mass: f32) {
        for (component, value) in value.into_iter().enumerate() {
            let index = self.layout.com_index(component, agent);
            self.center_of_mass_soa[index] = value;
        }
        self.total_mass[agent] = mass;
    }
}

/// Fixed-layout floating-root frame and center-of-mass Jacobian stage output.
///
/// Frame components are `[angular xyz; frame-origin linear xyz]` and tangent
/// columns are `[root angular xyz; root linear xyz; joint coordinates]`, all
/// expressed in `control_world`. Storage remains
/// `[body][component][generalized_coordinate][agent]` SoA.
#[derive(Clone, Debug)]
pub struct JacobianBatchOutput {
    layout: BatchLayout,
    frame_jacobian_soa: Vec<f32>,
    center_of_mass_jacobian_soa: Vec<f32>,
    agent_status: Vec<AgentStatus>,
}

impl JacobianBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        Self {
            frame_jacobian_soa: vec![
                0.0;
                layout.frame_jacobian_len().expect("validated batch layout")
            ],
            center_of_mass_jacobian_soa: vec![
                0.0;
                layout
                    .center_of_mass_jacobian_len()
                    .expect("validated batch layout")
            ],
            agent_status: vec![AgentStatus::Inactive; layout.agent_stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn frame_jacobian_soa(&self) -> &[f32] {
        &self.frame_jacobian_soa
    }

    pub(crate) fn frame_jacobian_soa_mut(&mut self) -> &mut [f32] {
        &mut self.frame_jacobian_soa
    }

    pub fn center_of_mass_jacobian_soa(&self) -> &[f32] {
        &self.center_of_mass_jacobian_soa
    }

    pub(crate) fn center_of_mass_jacobian_soa_mut(&mut self) -> &mut [f32] {
        &mut self.center_of_mass_jacobian_soa
    }

    pub fn agent_status(&self) -> &[AgentStatus] {
        &self.agent_status
    }

    pub fn frame_jacobian_value(
        &self,
        agent: usize,
        body: usize,
        component: usize,
        generalized_coordinate: usize,
    ) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || body >= self.layout.body_count
            || component >= self.layout.spatial_components
            || generalized_coordinate >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(
            self.frame_jacobian_soa[self.layout.frame_jacobian_index(
                body,
                component,
                generalized_coordinate,
                agent,
            )],
        )
    }

    pub fn center_of_mass_jacobian_value(
        &self,
        agent: usize,
        component: usize,
        generalized_coordinate: usize,
    ) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || component >= 3
            || generalized_coordinate >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(
            self.center_of_mass_jacobian_soa[self.layout.center_of_mass_jacobian_index(
                component,
                generalized_coordinate,
                agent,
            )],
        )
    }

    pub(crate) fn clear(&mut self) {
        self.frame_jacobian_soa.fill(0.0);
        self.center_of_mass_jacobian_soa.fill(0.0);
        self.agent_status.fill(AgentStatus::Inactive);
    }

    pub(crate) fn set_status(&mut self, agent: usize, status: AgentStatus) {
        self.agent_status[agent] = status;
    }

    pub(crate) fn set_frame_jacobian_value(
        &mut self,
        agent: usize,
        body: usize,
        component: usize,
        generalized_coordinate: usize,
        value: f32,
    ) {
        let index =
            self.layout
                .frame_jacobian_index(body, component, generalized_coordinate, agent);
        self.frame_jacobian_soa[index] = value;
    }

    pub(crate) fn multiply_add_center_of_mass_jacobian_value(
        &mut self,
        agent: usize,
        component: usize,
        generalized_coordinate: usize,
        scale: f32,
        value: f32,
    ) {
        let index =
            self.layout
                .center_of_mass_jacobian_index(component, generalized_coordinate, agent);
        self.center_of_mass_jacobian_soa[index] =
            scale.mul_add(value, self.center_of_mass_jacobian_soa[index]);
    }
}

/// Velocity and gravity inputs for the floating dynamics-product stage.
#[derive(Clone, Debug)]
pub struct DynamicsBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    generalized_velocity_soa: Vec<f32>,
    gravity_world_soa: Vec<f32>,
}

impl DynamicsBatchInput {
    pub fn new(layout: BatchLayout, active_agents: usize) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        let mut input = Self {
            generalized_velocity_soa: vec![
                0.0;
                layout
                    .generalized_velocity_len()
                    .expect("validated batch layout")
            ],
            gravity_world_soa: vec![0.0; 3 * layout.agent_stride],
            layout,
            active_agents,
        };
        for agent in 0..input.layout.agent_stride {
            input.gravity_world_soa[2 * input.layout.agent_stride + agent] = -9.81;
        }
        Ok(input)
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub const fn active_agents(&self) -> usize {
        self.active_agents
    }

    pub fn generalized_velocity_soa(&self) -> &[f32] {
        &self.generalized_velocity_soa
    }

    pub fn generalized_velocity_soa_mut(&mut self) -> &mut [f32] {
        &mut self.generalized_velocity_soa
    }

    pub fn gravity_world_soa(&self) -> &[f32] {
        &self.gravity_world_soa
    }

    pub fn gravity_world_soa_mut(&mut self) -> &mut [f32] {
        &mut self.gravity_world_soa
    }
}

/// Fixed-layout floating mass, bias, and centroidal-map output.
#[derive(Clone, Debug)]
pub struct DynamicsBatchOutput {
    layout: BatchLayout,
    mass_matrix_soa: Vec<f32>,
    bias_force_soa: Vec<f32>,
    centroidal_map_soa: Vec<f32>,
    agent_status: Vec<AgentStatus>,
    angular_velocity_soa: Vec<f32>,
    angular_acceleration_soa: Vec<f32>,
    linear_velocity_origin_soa: Vec<f32>,
    linear_acceleration_origin_soa: Vec<f32>,
}

impl DynamicsBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        let body_vector_len = layout.body_count * 3 * layout.agent_stride;
        Self {
            mass_matrix_soa: vec![0.0; layout.mass_matrix_len().expect("validated batch layout")],
            bias_force_soa: vec![
                0.0;
                layout
                    .generalized_velocity_len()
                    .expect("validated batch layout")
            ],
            centroidal_map_soa: vec![
                0.0;
                layout.centroidal_map_len().expect("validated batch layout")
            ],
            agent_status: vec![AgentStatus::Inactive; layout.agent_stride],
            angular_velocity_soa: vec![0.0; body_vector_len],
            angular_acceleration_soa: vec![0.0; body_vector_len],
            linear_velocity_origin_soa: vec![0.0; body_vector_len],
            linear_acceleration_origin_soa: vec![0.0; body_vector_len],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn mass_matrix_soa(&self) -> &[f32] {
        &self.mass_matrix_soa
    }

    pub fn bias_force_soa(&self) -> &[f32] {
        &self.bias_force_soa
    }

    pub fn centroidal_map_soa(&self) -> &[f32] {
        &self.centroidal_map_soa
    }

    pub(crate) fn mass_matrix_soa_mut(&mut self) -> &mut [f32] {
        &mut self.mass_matrix_soa
    }

    pub(crate) fn bias_force_soa_mut(&mut self) -> &mut [f32] {
        &mut self.bias_force_soa
    }

    pub(crate) fn centroidal_map_soa_mut(&mut self) -> &mut [f32] {
        &mut self.centroidal_map_soa
    }

    pub fn agent_status(&self) -> &[AgentStatus] {
        &self.agent_status
    }

    pub fn mass_matrix_value(&self, agent: usize, row: usize, column: usize) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || row >= self.layout.generalized_coordinate_count
            || column >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(self.mass_matrix_soa[self.layout.mass_matrix_index(row, column, agent)])
    }

    pub fn bias_force_value(&self, agent: usize, coordinate: usize) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || coordinate >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(self.bias_force_soa[self.layout.generalized_index(coordinate, agent)])
    }

    pub fn centroidal_map_value(
        &self,
        agent: usize,
        component: usize,
        coordinate: usize,
    ) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || component >= SPATIAL_COMPONENTS
            || coordinate >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(
            self.centroidal_map_soa[self
                .layout
                .centroidal_map_index(component, coordinate, agent)],
        )
    }

    pub(crate) fn clear(&mut self) {
        self.mass_matrix_soa.fill(0.0);
        self.bias_force_soa.fill(0.0);
        self.centroidal_map_soa.fill(0.0);
        self.agent_status.fill(AgentStatus::Inactive);
        self.angular_velocity_soa.fill(0.0);
        self.angular_acceleration_soa.fill(0.0);
        self.linear_velocity_origin_soa.fill(0.0);
        self.linear_acceleration_origin_soa.fill(0.0);
    }

    pub(crate) fn set_status(&mut self, agent: usize, status: AgentStatus) {
        self.agent_status[agent] = status;
    }

    pub(crate) fn body_vector(&self, storage: &[f32], body: usize, agent: usize) -> [f32; 3] {
        [
            storage[(body * 3) * self.layout.agent_stride + agent],
            storage[(body * 3 + 1) * self.layout.agent_stride + agent],
            storage[(body * 3 + 2) * self.layout.agent_stride + agent],
        ]
    }

    pub(crate) fn set_body_vector(
        layout: &BatchLayout,
        storage: &mut [f32],
        body: usize,
        agent: usize,
        value: [f32; 3],
    ) {
        for component in 0..3 {
            storage[(body * 3 + component) * layout.agent_stride + agent] = value[component];
        }
    }

    pub(crate) fn angular_velocity(&self, body: usize, agent: usize) -> [f32; 3] {
        self.body_vector(&self.angular_velocity_soa, body, agent)
    }

    pub(crate) fn angular_acceleration(&self, body: usize, agent: usize) -> [f32; 3] {
        self.body_vector(&self.angular_acceleration_soa, body, agent)
    }

    pub(crate) fn linear_velocity_origin(&self, body: usize, agent: usize) -> [f32; 3] {
        self.body_vector(&self.linear_velocity_origin_soa, body, agent)
    }

    pub(crate) fn linear_acceleration_origin(&self, body: usize, agent: usize) -> [f32; 3] {
        self.body_vector(&self.linear_acceleration_origin_soa, body, agent)
    }

    pub(crate) fn set_angular_velocity(&mut self, body: usize, agent: usize, value: [f32; 3]) {
        Self::set_body_vector(
            &self.layout,
            &mut self.angular_velocity_soa,
            body,
            agent,
            value,
        );
    }

    pub(crate) fn set_angular_acceleration(&mut self, body: usize, agent: usize, value: [f32; 3]) {
        Self::set_body_vector(
            &self.layout,
            &mut self.angular_acceleration_soa,
            body,
            agent,
            value,
        );
    }

    pub(crate) fn set_linear_velocity_origin(
        &mut self,
        body: usize,
        agent: usize,
        value: [f32; 3],
    ) {
        Self::set_body_vector(
            &self.layout,
            &mut self.linear_velocity_origin_soa,
            body,
            agent,
            value,
        );
    }

    pub(crate) fn set_linear_acceleration_origin(
        &mut self,
        body: usize,
        agent: usize,
        value: [f32; 3],
    ) {
        Self::set_body_vector(
            &self.layout,
            &mut self.linear_acceleration_origin_soa,
            body,
            agent,
            value,
        );
    }

    pub(crate) fn multiply_add_mass(
        &mut self,
        agent: usize,
        row: usize,
        column: usize,
        value: f32,
    ) {
        let index = self.layout.mass_matrix_index(row, column, agent);
        self.mass_matrix_soa[index] += value;
    }

    pub(crate) fn multiply_add_bias(
        &mut self,
        agent: usize,
        coordinate: usize,
        left: f32,
        right: f32,
    ) {
        let index = self.layout.generalized_index(coordinate, agent);
        self.bias_force_soa[index] = left.mul_add(right, self.bias_force_soa[index]);
    }

    pub(crate) fn multiply_add_centroidal(
        &mut self,
        agent: usize,
        component: usize,
        coordinate: usize,
        value: f32,
    ) {
        let index = self
            .layout
            .centroidal_map_index(component, coordinate, agent);
        self.centroidal_map_soa[index] += value;
    }
}

/// Fixed compiler-resolved point position, floating Jacobian, and `Jdot-v`
/// bias output. Slots are immutable kernel-plan entries with stable IDs in the
/// batch descriptor; agents may vary only in state.
#[derive(Clone, Debug)]
pub struct PointQueryBatchOutput {
    layout: BatchLayout,
    point_position_soa: Vec<f32>,
    point_jacobian_soa: Vec<f32>,
    point_bias_acceleration_soa: Vec<f32>,
    agent_status: Vec<AgentStatus>,
}

impl PointQueryBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        Self {
            point_position_soa: vec![
                0.0;
                layout.point_position_len().expect("validated batch layout")
            ],
            point_jacobian_soa: vec![
                0.0;
                layout.point_jacobian_len().expect("validated batch layout")
            ],
            point_bias_acceleration_soa: vec![
                0.0;
                layout
                    .point_bias_acceleration_len()
                    .expect("validated batch layout")
            ],
            agent_status: vec![AgentStatus::Inactive; layout.agent_stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn point_position_soa(&self) -> &[f32] {
        &self.point_position_soa
    }

    pub fn point_jacobian_soa(&self) -> &[f32] {
        &self.point_jacobian_soa
    }

    pub fn point_bias_acceleration_soa(&self) -> &[f32] {
        &self.point_bias_acceleration_soa
    }

    pub fn agent_status(&self) -> &[AgentStatus] {
        &self.agent_status
    }

    pub(crate) fn point_position_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_position_soa
    }

    pub(crate) fn point_jacobian_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_jacobian_soa
    }

    pub(crate) fn point_bias_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_bias_acceleration_soa
    }

    pub fn point_position(&self, agent: usize, slot: usize) -> Option<[f32; 3]> {
        if agent >= self.layout.agent_capacity || slot >= self.layout.point_query_count {
            return None;
        }
        Some(std::array::from_fn(|component| {
            self.point_position_soa[self.layout.point_position_index(slot, component, agent)]
        }))
    }

    pub fn point_jacobian_value(
        &self,
        agent: usize,
        slot: usize,
        component: usize,
        generalized_coordinate: usize,
    ) -> Option<f32> {
        if agent >= self.layout.agent_capacity
            || slot >= self.layout.point_query_count
            || component >= 3
            || generalized_coordinate >= self.layout.generalized_coordinate_count
        {
            return None;
        }
        Some(
            self.point_jacobian_soa[self.layout.point_jacobian_index(
                slot,
                component,
                generalized_coordinate,
                agent,
            )],
        )
    }

    pub fn point_bias_acceleration(&self, agent: usize, slot: usize) -> Option<[f32; 3]> {
        if agent >= self.layout.agent_capacity || slot >= self.layout.point_query_count {
            return None;
        }
        Some(std::array::from_fn(|component| {
            self.point_bias_acceleration_soa
                [self.layout.point_position_index(slot, component, agent)]
        }))
    }

    pub(crate) fn clear(&mut self) {
        self.point_position_soa.fill(0.0);
        self.point_jacobian_soa.fill(0.0);
        self.point_bias_acceleration_soa.fill(0.0);
        self.agent_status.fill(AgentStatus::Inactive);
    }

    pub(crate) fn set_status(&mut self, agent: usize, status: AgentStatus) {
        self.agent_status[agent] = status;
    }

    pub(crate) fn set_point_position(&mut self, agent: usize, slot: usize, value: [f32; 3]) {
        for (component, value) in value.into_iter().enumerate() {
            let index = self.layout.point_position_index(slot, component, agent);
            self.point_position_soa[index] = value;
        }
    }

    pub(crate) fn set_point_jacobian_value(
        &mut self,
        agent: usize,
        slot: usize,
        component: usize,
        generalized_coordinate: usize,
        value: f32,
    ) {
        let index =
            self.layout
                .point_jacobian_index(slot, component, generalized_coordinate, agent);
        self.point_jacobian_soa[index] = value;
    }

    pub(crate) fn set_point_bias_acceleration(
        &mut self,
        agent: usize,
        slot: usize,
        value: [f32; 3],
    ) {
        for (component, value) in value.into_iter().enumerate() {
            let index = self.layout.point_position_index(slot, component, agent);
            self.point_bias_acceleration_soa[index] = value;
        }
    }
}

/// Per-agent target jets and activation masks for a fixed task/constraint
/// emission plan. Topology and metadata live in the batch descriptor.
#[derive(Clone, Debug)]
pub struct EmissionBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    point_task_active_soa: Vec<u8>,
    point_target_position_soa: Vec<f32>,
    point_target_velocity_soa: Vec<f32>,
    point_target_acceleration_soa: Vec<f32>,
    contact_lock_active_soa: Vec<u8>,
    contact_desired_acceleration_soa: Vec<f32>,
}

impl EmissionBatchInput {
    pub fn new(layout: BatchLayout, active_agents: usize) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        Ok(Self {
            point_task_active_soa: vec![
                0;
                layout
                    .point_task_active_len()
                    .expect("validated batch layout")
            ],
            point_target_position_soa: vec![
                0.0;
                layout
                    .point_task_vector_len()
                    .expect("validated batch layout")
            ],
            point_target_velocity_soa: vec![
                0.0;
                layout
                    .point_task_vector_len()
                    .expect("validated batch layout")
            ],
            point_target_acceleration_soa: vec![
                0.0;
                layout
                    .point_task_vector_len()
                    .expect("validated batch layout")
            ],
            contact_lock_active_soa: vec![
                0;
                layout
                    .contact_lock_active_len()
                    .expect("validated batch layout")
            ],
            contact_desired_acceleration_soa: vec![
                0.0;
                layout
                    .contact_lock_vector_len()
                    .expect("validated batch layout")
            ],
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

    pub fn point_task_active_soa(&self) -> &[u8] {
        &self.point_task_active_soa
    }

    pub fn point_task_active_soa_mut(&mut self) -> &mut [u8] {
        &mut self.point_task_active_soa
    }

    pub fn point_target_position_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_target_position_soa
    }

    pub(crate) fn point_target_position_soa(&self) -> &[f32] {
        &self.point_target_position_soa
    }

    pub fn point_target_velocity_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_target_velocity_soa
    }

    pub(crate) fn point_target_velocity_soa(&self) -> &[f32] {
        &self.point_target_velocity_soa
    }

    pub fn point_target_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_target_acceleration_soa
    }

    pub(crate) fn point_target_acceleration_soa(&self) -> &[f32] {
        &self.point_target_acceleration_soa
    }

    pub fn contact_lock_active_soa(&self) -> &[u8] {
        &self.contact_lock_active_soa
    }

    pub fn contact_lock_active_soa_mut(&mut self) -> &mut [u8] {
        &mut self.contact_lock_active_soa
    }

    pub fn contact_desired_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.contact_desired_acceleration_soa
    }

    pub(crate) fn contact_desired_acceleration_soa(&self) -> &[f32] {
        &self.contact_desired_acceleration_soa
    }

    pub(crate) fn point_target_position(&self, slot: usize, agent: usize) -> [f32; 3] {
        std::array::from_fn(|component| {
            self.point_target_position_soa[self.layout.task_vector_index(slot, component, agent)]
        })
    }

    pub(crate) fn point_target_velocity(&self, slot: usize, agent: usize) -> [f32; 3] {
        std::array::from_fn(|component| {
            self.point_target_velocity_soa[self.layout.task_vector_index(slot, component, agent)]
        })
    }

    pub(crate) fn point_target_acceleration(&self, slot: usize, agent: usize) -> [f32; 3] {
        std::array::from_fn(|component| {
            self.point_target_acceleration_soa
                [self.layout.task_vector_index(slot, component, agent)]
        })
    }

    pub(crate) fn contact_desired_acceleration(&self, slot: usize, agent: usize) -> [f32; 3] {
        std::array::from_fn(|component| {
            self.contact_desired_acceleration_soa
                [self.layout.contact_vector_index(slot, component, agent)]
        })
    }
}

/// Stable fixed-row point-task and contact-lock emission output.
#[derive(Clone, Debug)]
pub struct EmissionBatchOutput {
    layout: BatchLayout,
    point_task_active_soa: Vec<u8>,
    point_task_position_error_soa: Vec<f32>,
    point_task_velocity_error_soa: Vec<f32>,
    point_task_desired_acceleration_soa: Vec<f32>,
    point_task_jacobian_soa: Vec<f32>,
    point_task_rhs_soa: Vec<f32>,
    contact_lock_active_soa: Vec<u8>,
    contact_lock_jacobian_soa: Vec<f32>,
    contact_lock_rhs_soa: Vec<f32>,
    agent_status: Vec<AgentStatus>,
}

impl EmissionBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        let task_vector_len = layout
            .point_task_vector_len()
            .expect("validated batch layout");
        Self {
            point_task_active_soa: vec![
                0;
                layout
                    .point_task_active_len()
                    .expect("validated batch layout")
            ],
            point_task_position_error_soa: vec![0.0; task_vector_len],
            point_task_velocity_error_soa: vec![0.0; task_vector_len],
            point_task_desired_acceleration_soa: vec![0.0; task_vector_len],
            point_task_jacobian_soa: vec![
                0.0;
                layout
                    .point_task_jacobian_len()
                    .expect("validated batch layout")
            ],
            point_task_rhs_soa: vec![0.0; task_vector_len],
            contact_lock_active_soa: vec![
                0;
                layout
                    .contact_lock_active_len()
                    .expect("validated batch layout")
            ],
            contact_lock_jacobian_soa: vec![
                0.0;
                layout
                    .contact_lock_jacobian_len()
                    .expect("validated batch layout")
            ],
            contact_lock_rhs_soa: vec![
                0.0;
                layout
                    .contact_lock_vector_len()
                    .expect("validated batch layout")
            ],
            agent_status: vec![AgentStatus::Inactive; layout.agent_stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub fn point_task_active_soa(&self) -> &[u8] {
        &self.point_task_active_soa
    }
    pub fn point_task_position_error_soa(&self) -> &[f32] {
        &self.point_task_position_error_soa
    }
    pub fn point_task_velocity_error_soa(&self) -> &[f32] {
        &self.point_task_velocity_error_soa
    }
    pub fn point_task_desired_acceleration_soa(&self) -> &[f32] {
        &self.point_task_desired_acceleration_soa
    }
    pub fn point_task_jacobian_soa(&self) -> &[f32] {
        &self.point_task_jacobian_soa
    }
    pub fn point_task_rhs_soa(&self) -> &[f32] {
        &self.point_task_rhs_soa
    }
    pub fn contact_lock_active_soa(&self) -> &[u8] {
        &self.contact_lock_active_soa
    }
    pub fn contact_lock_jacobian_soa(&self) -> &[f32] {
        &self.contact_lock_jacobian_soa
    }
    pub fn contact_lock_rhs_soa(&self) -> &[f32] {
        &self.contact_lock_rhs_soa
    }
    pub fn agent_status(&self) -> &[AgentStatus] {
        &self.agent_status
    }

    pub(crate) fn point_task_active_soa_mut(&mut self) -> &mut [u8] {
        &mut self.point_task_active_soa
    }
    pub(crate) fn point_task_position_error_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_task_position_error_soa
    }
    pub(crate) fn point_task_velocity_error_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_task_velocity_error_soa
    }
    pub(crate) fn point_task_desired_acceleration_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_task_desired_acceleration_soa
    }
    pub(crate) fn point_task_jacobian_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_task_jacobian_soa
    }
    pub(crate) fn point_task_rhs_soa_mut(&mut self) -> &mut [f32] {
        &mut self.point_task_rhs_soa
    }
    pub(crate) fn contact_lock_active_soa_mut(&mut self) -> &mut [u8] {
        &mut self.contact_lock_active_soa
    }
    pub(crate) fn contact_lock_jacobian_soa_mut(&mut self) -> &mut [f32] {
        &mut self.contact_lock_jacobian_soa
    }
    pub(crate) fn contact_lock_rhs_soa_mut(&mut self) -> &mut [f32] {
        &mut self.contact_lock_rhs_soa
    }

    pub(crate) fn clear(&mut self) {
        self.point_task_active_soa.fill(0);
        self.point_task_position_error_soa.fill(0.0);
        self.point_task_velocity_error_soa.fill(0.0);
        self.point_task_desired_acceleration_soa.fill(0.0);
        self.point_task_jacobian_soa.fill(0.0);
        self.point_task_rhs_soa.fill(0.0);
        self.contact_lock_active_soa.fill(0);
        self.contact_lock_jacobian_soa.fill(0.0);
        self.contact_lock_rhs_soa.fill(0.0);
        self.agent_status.fill(AgentStatus::Inactive);
    }

    pub(crate) fn set_status(&mut self, agent: usize, status: AgentStatus) {
        self.agent_status[agent] = status;
    }

    pub(crate) fn set_task_active(&mut self, slot: usize, agent: usize) {
        let index = self.layout.task_active_index(slot, agent);
        self.point_task_active_soa[index] = 1;
    }

    pub(crate) fn set_task_vector(
        &mut self,
        storage: TaskVectorStorage,
        slot: usize,
        agent: usize,
        value: [f32; 3],
    ) {
        let target = match storage {
            TaskVectorStorage::PositionError => &mut self.point_task_position_error_soa,
            TaskVectorStorage::VelocityError => &mut self.point_task_velocity_error_soa,
            TaskVectorStorage::DesiredAcceleration => &mut self.point_task_desired_acceleration_soa,
            TaskVectorStorage::RightHandSide => &mut self.point_task_rhs_soa,
        };
        for (component, value) in value.into_iter().enumerate() {
            target[self.layout.task_vector_index(slot, component, agent)] = value;
        }
    }

    pub(crate) fn set_task_jacobian(
        &mut self,
        slot: usize,
        component: usize,
        coordinate: usize,
        agent: usize,
        value: f32,
    ) {
        let index = self
            .layout
            .task_jacobian_index(slot, component, coordinate, agent);
        self.point_task_jacobian_soa[index] = value;
    }

    pub(crate) fn set_contact_active(&mut self, slot: usize, agent: usize) {
        let index = self.layout.contact_active_index(slot, agent);
        self.contact_lock_active_soa[index] = 1;
    }

    pub(crate) fn set_contact_jacobian(
        &mut self,
        slot: usize,
        component: usize,
        coordinate: usize,
        agent: usize,
        value: f32,
    ) {
        let index = self
            .layout
            .contact_jacobian_index(slot, component, coordinate, agent);
        self.contact_lock_jacobian_soa[index] = value;
    }

    pub(crate) fn set_contact_rhs(&mut self, slot: usize, agent: usize, value: [f32; 3]) {
        for (component, value) in value.into_iter().enumerate() {
            let index = self.layout.contact_vector_index(slot, component, agent);
            self.contact_lock_rhs_soa[index] = value;
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum TaskVectorStorage {
    PositionError,
    VelocityError,
    DesiredAcceleration,
    RightHandSide,
}
