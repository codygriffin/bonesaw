//! Fixed-layout joint acceleration viability envelopes for batch CPU solves.
//!
//! This stage converts observed joint position/velocity plus explicit
//! acceleration authority into next-tick acceleration boxes. It deliberately
//! does not infer actuator effort feasibility: that requires the later
//! dynamic `[qdd, tau, contact force]` decision vector.

use bonesaw_core::{MotionProgram, dynamic_wbc::joint_acceleration_interval};
use thiserror::Error;

use crate::{BatchInputError, BatchLayout, ROOT_TANGENT_COMPONENTS};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum JointEnvelopeAgentStatus {
    Inactive = 0,
    Disabled = 1,
    Ok = 2,
    InvalidInput = 3,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum JointEnvelopeError {
    #[error("program does not match the batch layout")]
    ProgramMismatch,
    #[error("joint envelope input or output layout mismatch")]
    LayoutMismatch,
}

#[derive(Clone, Copy, Debug)]
struct JointLimit {
    lower_position: f64,
    upper_position: f64,
    maximum_velocity: f64,
}

#[derive(Clone, Debug)]
pub struct JointEnvelopeBatchInput {
    layout: BatchLayout,
    active_agents: usize,
    enabled: Vec<u8>,
    dt_seconds: Vec<f64>,
    position_soa: Vec<f64>,
    velocity_soa: Vec<f64>,
    maximum_acceleration_soa: Vec<f64>,
}

impl JointEnvelopeBatchInput {
    pub fn new(layout: BatchLayout, active_agents: usize) -> Result<Self, BatchInputError> {
        if active_agents > layout.agent_capacity {
            return Err(BatchInputError::ActiveAgents {
                actual: active_agents,
                capacity: layout.agent_capacity,
            });
        }
        let joint_len = layout.q_len().expect("validated batch layout");
        Ok(Self {
            enabled: vec![0; layout.agent_stride],
            dt_seconds: vec![0.02; layout.agent_stride],
            position_soa: vec![0.0; joint_len],
            velocity_soa: vec![0.0; joint_len],
            maximum_acceleration_soa: vec![f64::INFINITY; joint_len],
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
    pub fn enabled(&self) -> &[u8] {
        &self.enabled
    }
    pub fn enabled_mut(&mut self) -> &mut [u8] {
        &mut self.enabled
    }
    pub fn dt_seconds(&self) -> &[f64] {
        &self.dt_seconds
    }
    pub fn dt_seconds_mut(&mut self) -> &mut [f64] {
        &mut self.dt_seconds
    }
    pub fn position_soa(&self) -> &[f64] {
        &self.position_soa
    }
    pub fn position_soa_mut(&mut self) -> &mut [f64] {
        &mut self.position_soa
    }
    pub fn velocity_soa(&self) -> &[f64] {
        &self.velocity_soa
    }
    pub fn velocity_soa_mut(&mut self) -> &mut [f64] {
        &mut self.velocity_soa
    }
    pub fn maximum_acceleration_soa(&self) -> &[f64] {
        &self.maximum_acceleration_soa
    }
    pub fn maximum_acceleration_soa_mut(&mut self) -> &mut [f64] {
        &mut self.maximum_acceleration_soa
    }
}

#[derive(Clone, Debug)]
pub struct JointEnvelopeBatchOutput {
    layout: BatchLayout,
    lower_soa: Vec<f64>,
    upper_soa: Vec<f64>,
    status: Vec<JointEnvelopeAgentStatus>,
    minimum_position_headroom: Vec<f64>,
    minimum_velocity_headroom: Vec<f64>,
    limiting_position_coordinate: Vec<u32>,
    limiting_velocity_coordinate: Vec<u32>,
    recovery_coordinate_count: Vec<u32>,
}

impl JointEnvelopeBatchOutput {
    pub fn new(layout: BatchLayout) -> Self {
        let generalized_len = layout
            .generalized_velocity_len()
            .expect("validated batch layout");
        let stride = layout.agent_stride;
        Self {
            lower_soa: vec![f64::NEG_INFINITY; generalized_len],
            upper_soa: vec![f64::INFINITY; generalized_len],
            status: vec![JointEnvelopeAgentStatus::Inactive; stride],
            minimum_position_headroom: vec![f64::INFINITY; stride],
            minimum_velocity_headroom: vec![f64::INFINITY; stride],
            limiting_position_coordinate: vec![u32::MAX; stride],
            limiting_velocity_coordinate: vec![u32::MAX; stride],
            recovery_coordinate_count: vec![0; stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }
    pub fn lower_soa(&self) -> &[f64] {
        &self.lower_soa
    }
    pub fn upper_soa(&self) -> &[f64] {
        &self.upper_soa
    }
    pub fn status(&self) -> &[JointEnvelopeAgentStatus] {
        &self.status
    }
    pub fn minimum_position_headroom(&self) -> &[f64] {
        &self.minimum_position_headroom
    }
    pub fn minimum_velocity_headroom(&self) -> &[f64] {
        &self.minimum_velocity_headroom
    }
    pub fn limiting_position_coordinate(&self) -> &[u32] {
        &self.limiting_position_coordinate
    }
    pub fn limiting_velocity_coordinate(&self) -> &[u32] {
        &self.limiting_velocity_coordinate
    }
    pub fn recovery_coordinate_count(&self) -> &[u32] {
        &self.recovery_coordinate_count
    }

    fn clear(&mut self) {
        self.lower_soa.fill(f64::NEG_INFINITY);
        self.upper_soa.fill(f64::INFINITY);
        self.status.fill(JointEnvelopeAgentStatus::Inactive);
        self.minimum_position_headroom.fill(f64::INFINITY);
        self.minimum_velocity_headroom.fill(f64::INFINITY);
        self.limiting_position_coordinate.fill(u32::MAX);
        self.limiting_velocity_coordinate.fill(u32::MAX);
        self.recovery_coordinate_count.fill(0);
    }
}

#[derive(Clone, Debug)]
pub struct CpuJointEnvelopeBatchExecutor {
    layout: BatchLayout,
    limits: Vec<JointLimit>,
}

impl CpuJointEnvelopeBatchExecutor {
    pub fn compile(
        program: &MotionProgram,
        layout: BatchLayout,
    ) -> Result<Self, JointEnvelopeError> {
        if layout.program_fingerprint != program.header.fingerprint_sha256
            || layout.coordinate_count != program.model.dof
        {
            return Err(JointEnvelopeError::ProgramMismatch);
        }
        let mut limits = vec![
            JointLimit {
                lower_position: f64::NEG_INFINITY,
                upper_position: f64::INFINITY,
                maximum_velocity: f64::INFINITY,
            };
            program.model.dof
        ];
        for joint in &program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                limits[coordinate] = JointLimit {
                    lower_position: joint.limit.lower,
                    upper_position: joint.limit.upper,
                    maximum_velocity: joint.limit.velocity,
                };
            }
        }
        Ok(Self { layout, limits })
    }

    pub fn execute_into(
        &self,
        input: &JointEnvelopeBatchInput,
        output: &mut JointEnvelopeBatchOutput,
    ) -> Result<(), JointEnvelopeError> {
        if input.layout() != &self.layout || output.layout() != &self.layout {
            return Err(JointEnvelopeError::LayoutMismatch);
        }
        output.clear();
        let stride = self.layout.agent_stride;
        for agent in 0..input.active_agents() {
            if input.enabled()[agent] == 0 {
                output.status[agent] = JointEnvelopeAgentStatus::Disabled;
                continue;
            }
            let dt = input.dt_seconds()[agent];
            let mut valid = dt.is_finite() && dt > 0.0;
            for (coordinate, limit) in self.limits.iter().copied().enumerate() {
                let index = coordinate * stride + agent;
                let position = input.position_soa()[index];
                let velocity = input.velocity_soa()[index];
                let maximum_acceleration = input.maximum_acceleration_soa()[index];
                let interval = joint_acceleration_interval(
                    position,
                    velocity,
                    limit.lower_position,
                    limit.upper_position,
                    limit.maximum_velocity,
                    maximum_acceleration,
                    dt,
                );
                let Some((lower, upper)) = interval else {
                    valid = false;
                    break;
                };
                let generalized = ROOT_TANGENT_COMPONENTS + coordinate;
                let output_index = generalized * stride + agent;
                output.lower_soa[output_index] = lower;
                output.upper_soa[output_index] = upper;

                let position_headroom =
                    (position - limit.lower_position).min(limit.upper_position - position);
                if position_headroom < output.minimum_position_headroom[agent] {
                    output.minimum_position_headroom[agent] = position_headroom;
                    output.limiting_position_coordinate[agent] = coordinate as u32;
                }
                let velocity_headroom = limit.maximum_velocity - velocity.abs();
                if velocity_headroom < output.minimum_velocity_headroom[agent] {
                    output.minimum_velocity_headroom[agent] = velocity_headroom;
                    output.limiting_velocity_coordinate[agent] = coordinate as u32;
                }
                if position < limit.lower_position || position > limit.upper_position {
                    output.recovery_coordinate_count[agent] += 1;
                }
            }
            if valid {
                output.status[agent] = JointEnvelopeAgentStatus::Ok;
            } else {
                for coordinate in 0..self.layout.generalized_coordinate_count {
                    let index = coordinate * stride + agent;
                    output.lower_soa[index] = f64::NEG_INFINITY;
                    output.upper_soa[index] = f64::INFINITY;
                }
                output.minimum_position_headroom[agent] = f64::INFINITY;
                output.minimum_velocity_headroom[agent] = f64::INFINITY;
                output.limiting_position_coordinate[agent] = u32::MAX;
                output.limiting_velocity_coordinate[agent] = u32::MAX;
                output.recovery_coordinate_count[agent] = 0;
                output.status[agent] = JointEnvelopeAgentStatus::InvalidInput;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BatchLayout, allocation_sentinel};
    use bonesaw_core::{MotionProgram, TimingSpec};
    use std::hint::black_box;

    #[test]
    fn envelope_is_fixed_shape_isolated_and_allocation_free() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let layout = BatchLayout::compile_with_plan_counts(&program, 4, 32, 0, 0, 0).unwrap();
        let executor = CpuJointEnvelopeBatchExecutor::compile(&program, layout.clone()).unwrap();
        let mut input = JointEnvelopeBatchInput::new(layout.clone(), 4).unwrap();
        input.enabled_mut().fill(1);
        input.dt_seconds_mut().fill(0.02);
        input.maximum_acceleration_soa_mut().fill(30.0);
        let recovery_joint = program
            .model
            .joints
            .iter()
            .find(|joint| joint.coordinate.is_some() && joint.limit.upper.is_finite())
            .unwrap();
        let recovery_coordinate = recovery_joint.coordinate.unwrap();
        let recovery_index = recovery_coordinate * layout.agent_stride + 3;
        input.position_soa_mut()[recovery_index] = recovery_joint.limit.upper + 0.1;
        input.velocity_soa_mut()[recovery_index] = 1.0;
        let mut output = JointEnvelopeBatchOutput::new(layout.clone());
        executor.execute_into(&input, &mut output).unwrap();
        assert!(
            output
                .status()
                .iter()
                .take(4)
                .all(|status| *status == JointEnvelopeAgentStatus::Ok)
        );
        assert!(output.lower_soa()[ROOT_TANGENT_COMPONENTS * layout.agent_stride].is_finite());
        assert_eq!(output.recovery_coordinate_count()[3], 1);
        let recovery_output_index =
            (ROOT_TANGENT_COMPONENTS + recovery_coordinate) * layout.agent_stride + 3;
        assert_eq!(output.lower_soa()[recovery_output_index], -30.0);
        assert_eq!(output.upper_soa()[recovery_output_index], -30.0);
        input.maximum_acceleration_soa_mut()[2 * layout.agent_stride + 2] = f64::NAN;
        executor.execute_into(&input, &mut output).unwrap();
        assert_eq!(output.status()[2], JointEnvelopeAgentStatus::InvalidInput);
        assert_eq!(output.status()[1], JointEnvelopeAgentStatus::Ok);
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                executor
                    .execute_into(black_box(&input), black_box(&mut output))
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
    }
}
