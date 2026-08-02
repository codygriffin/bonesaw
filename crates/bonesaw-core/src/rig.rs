use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    dynamic_wbc::{FloatingCenterOfMassTask, FloatingTaskPriorities, FloatingTaskWeights},
    math::{Motion6, Transform3, Vec3},
    model::{CompiledModel, FrameId},
    signal::{CompiledSignalProgram, SignalJet, SignalKind, SignalOutputBuffer},
    solver::Priority,
};

/// Authoring-time task description. Signal outputs are named by stable ID and
/// resolved to fixed output slots during compilation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TaskSpec {
    Point {
        stable_id: u32,
        frame: FrameId,
        point_in_frame: Vec3,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    Orientation {
        stable_id: u32,
        frame: FrameId,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    DirectionAim {
        stable_id: u32,
        frame: FrameId,
        controlled_axis_in_frame: Vec3,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    CenterOfMass {
        stable_id: u32,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    FloatingRootOrientation {
        stable_id: u32,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
    },
    FloatingRootTranslation {
        stable_id: u32,
        target_signal: u32,
        horizontal_priority: Priority,
        height_priority: Priority,
        horizontal_weight: f64,
        height_weight: f64,
        horizontal_bandwidth_hz: f64,
        height_bandwidth_hz: f64,
        horizontal_damping_ratio: f64,
        height_damping_ratio: f64,
        maximum_horizontal_acceleration: f64,
        maximum_height_acceleration: f64,
    },
    FloatingCenterOfMass {
        stable_id: u32,
        target_signal: u32,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
    },
}

/// Canonical runtime operation with all names resolved to integer slots.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum TaskOp {
    Point {
        stable_id: u32,
        frame: FrameId,
        point_in_frame: Vec3,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    Orientation {
        stable_id: u32,
        frame: FrameId,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    DirectionAim {
        stable_id: u32,
        frame: FrameId,
        controlled_axis_in_frame: Vec3,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    CenterOfMass {
        stable_id: u32,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
    },
    FloatingRootOrientation {
        stable_id: u32,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
    },
    FloatingRootTranslation {
        stable_id: u32,
        target_output: usize,
        horizontal_priority: Priority,
        height_priority: Priority,
        horizontal_weight: f64,
        height_weight: f64,
        horizontal_bandwidth_hz: f64,
        height_bandwidth_hz: f64,
        horizontal_damping_ratio: f64,
        height_damping_ratio: f64,
        maximum_horizontal_acceleration: f64,
        maximum_height_acceleration: f64,
    },
    FloatingCenterOfMass {
        stable_id: u32,
        target_output: usize,
        priority: Priority,
        weight: f64,
        bandwidth_hz: f64,
        damping_ratio: f64,
        maximum_acceleration: f64,
    },
}

impl TaskOp {
    pub fn stable_id(self) -> u32 {
        match self {
            Self::Point { stable_id, .. }
            | Self::Orientation { stable_id, .. }
            | Self::DirectionAim { stable_id, .. }
            | Self::CenterOfMass { stable_id, .. }
            | Self::FloatingRootOrientation { stable_id, .. }
            | Self::FloatingRootTranslation { stable_id, .. }
            | Self::FloatingCenterOfMass { stable_id, .. } => stable_id,
        }
    }

    fn target_output(self) -> usize {
        match self {
            Self::Point { target_output, .. }
            | Self::Orientation { target_output, .. }
            | Self::DirectionAim { target_output, .. }
            | Self::CenterOfMass { target_output, .. }
            | Self::FloatingRootOrientation { target_output, .. }
            | Self::FloatingRootTranslation { target_output, .. }
            | Self::FloatingCenterOfMass { target_output, .. } => target_output,
        }
    }

    fn numeric_parameters_are_valid(self) -> bool {
        match self {
            Self::Point {
                point_in_frame,
                weight,
                bandwidth_hz,
                ..
            } => {
                point_in_frame.iter().all(|value| value.is_finite())
                    && weight.is_finite()
                    && weight > 0.0
                    && bandwidth_hz.is_finite()
                    && bandwidth_hz > 0.0
            }
            Self::DirectionAim {
                controlled_axis_in_frame,
                weight,
                bandwidth_hz,
                ..
            } => {
                controlled_axis_in_frame
                    .iter()
                    .all(|value| value.is_finite())
                    && (controlled_axis_in_frame.norm_squared() - 1.0).abs() <= 1e-12
                    && weight.is_finite()
                    && weight > 0.0
                    && bandwidth_hz.is_finite()
                    && bandwidth_hz > 0.0
            }
            Self::Orientation {
                weight,
                bandwidth_hz,
                ..
            }
            | Self::CenterOfMass {
                weight,
                bandwidth_hz,
                ..
            } => {
                weight.is_finite() && weight > 0.0 && bandwidth_hz.is_finite() && bandwidth_hz > 0.0
            }
            Self::FloatingRootOrientation {
                weight,
                bandwidth_hz,
                damping_ratio,
                maximum_acceleration,
                ..
            }
            | Self::FloatingCenterOfMass {
                weight,
                bandwidth_hz,
                damping_ratio,
                maximum_acceleration,
                ..
            } => {
                weight.is_finite()
                    && weight > 0.0
                    && bandwidth_hz.is_finite()
                    && bandwidth_hz > 0.0
                    && damping_ratio.is_finite()
                    && damping_ratio > 0.0
                    && maximum_acceleration.is_finite()
                    && maximum_acceleration > 0.0
            }
            Self::FloatingRootTranslation {
                horizontal_weight,
                height_weight,
                horizontal_bandwidth_hz,
                height_bandwidth_hz,
                horizontal_damping_ratio,
                height_damping_ratio,
                maximum_horizontal_acceleration,
                maximum_height_acceleration,
                ..
            } => [
                horizontal_weight,
                height_weight,
                horizontal_bandwidth_hz,
                height_bandwidth_hz,
                horizontal_damping_ratio,
                height_damping_ratio,
                maximum_horizontal_acceleration,
                maximum_height_acceleration,
            ]
            .into_iter()
            .all(|value| value.is_finite() && value > 0.0),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingTaskState {
    pub control_world_from_root: Transform3,
    pub root_twist_world: Motion6,
    pub center_of_mass_world: Vec3,
    pub center_of_mass_velocity_world: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingTaskCommand {
    pub root_acceleration_world: Motion6,
    pub task_priorities: FloatingTaskPriorities,
    pub task_weights: FloatingTaskWeights,
    pub center_of_mass_task: Option<FloatingCenterOfMassTask>,
}

impl Default for FloatingTaskCommand {
    fn default() -> Self {
        let task_priorities = FloatingTaskPriorities {
            joint_posture: Priority::Preference,
            ..FloatingTaskPriorities::default()
        };
        Self {
            root_acceleration_world: Motion6::default(),
            task_priorities,
            task_weights: FloatingTaskWeights::default(),
            center_of_mass_task: None,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CompiledTaskProgram {
    ops: Vec<TaskOp>,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum TaskCompileError {
    #[error("task stable ID {0} is duplicated")]
    DuplicateStableId(u32),
    #[error("task {task} refers to missing signal output stable ID {output}")]
    MissingSignalOutput { task: u32, output: u32 },
    #[error("task {task} requires {expected:?} but signal output is {actual:?}")]
    SignalType {
        task: u32,
        expected: SignalKind,
        actual: SignalKind,
    },
    #[error("task {task} refers to missing frame {frame:?}")]
    Frame { task: u32, frame: FrameId },
    #[error("task {0} contains an invalid numeric parameter")]
    InvalidParameter(u32),
    #[error("compiled task {task} refers to missing signal output slot {output}")]
    OutputSlot { task: u32, output: usize },
    #[error("compiled task plan has more than one {0} semantic slot")]
    DuplicateFloatingSemantic(&'static str),
}

impl CompiledTaskProgram {
    pub fn compile(
        model: &CompiledModel,
        signals: &CompiledSignalProgram,
        specs: Vec<TaskSpec>,
    ) -> Result<Self, TaskCompileError> {
        let mut ops = Vec::with_capacity(specs.len());
        for spec in specs {
            let op = match spec {
                TaskSpec::Point {
                    stable_id,
                    frame,
                    point_in_frame,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                } => TaskOp::Point {
                    stable_id,
                    frame,
                    point_in_frame,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Vector,
                    )?,
                    priority,
                    weight,
                    bandwidth_hz,
                },
                TaskSpec::Orientation {
                    stable_id,
                    frame,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                } => TaskOp::Orientation {
                    stable_id,
                    frame,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Rotation,
                    )?,
                    priority,
                    weight,
                    bandwidth_hz,
                },
                TaskSpec::DirectionAim {
                    stable_id,
                    frame,
                    controlled_axis_in_frame,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                } => {
                    let Some(controlled_axis_in_frame) =
                        controlled_axis_in_frame.try_normalize(1e-12)
                    else {
                        return Err(TaskCompileError::InvalidParameter(stable_id));
                    };
                    TaskOp::DirectionAim {
                        stable_id,
                        frame,
                        controlled_axis_in_frame,
                        target_output: resolve_output(
                            signals,
                            stable_id,
                            target_signal,
                            SignalKind::Vector,
                        )?,
                        priority,
                        weight,
                        bandwidth_hz,
                    }
                }
                TaskSpec::CenterOfMass {
                    stable_id,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                } => TaskOp::CenterOfMass {
                    stable_id,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Vector,
                    )?,
                    priority,
                    weight,
                    bandwidth_hz,
                },
                TaskSpec::FloatingRootOrientation {
                    stable_id,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                } => TaskOp::FloatingRootOrientation {
                    stable_id,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Rotation,
                    )?,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                },
                TaskSpec::FloatingRootTranslation {
                    stable_id,
                    target_signal,
                    horizontal_priority,
                    height_priority,
                    horizontal_weight,
                    height_weight,
                    horizontal_bandwidth_hz,
                    height_bandwidth_hz,
                    horizontal_damping_ratio,
                    height_damping_ratio,
                    maximum_horizontal_acceleration,
                    maximum_height_acceleration,
                } => TaskOp::FloatingRootTranslation {
                    stable_id,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Vector,
                    )?,
                    horizontal_priority,
                    height_priority,
                    horizontal_weight,
                    height_weight,
                    horizontal_bandwidth_hz,
                    height_bandwidth_hz,
                    horizontal_damping_ratio,
                    height_damping_ratio,
                    maximum_horizontal_acceleration,
                    maximum_height_acceleration,
                },
                TaskSpec::FloatingCenterOfMass {
                    stable_id,
                    target_signal,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                } => TaskOp::FloatingCenterOfMass {
                    stable_id,
                    target_output: resolve_output(
                        signals,
                        stable_id,
                        target_signal,
                        SignalKind::Vector,
                    )?,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                },
            };
            ops.push(op);
        }
        let program = Self { ops };
        program.validate(model, signals)?;
        Ok(program)
    }

    pub fn validate(
        &self,
        model: &CompiledModel,
        signals: &CompiledSignalProgram,
    ) -> Result<(), TaskCompileError> {
        for (index, op) in self.ops.iter().copied().enumerate() {
            let stable_id = op.stable_id();
            if self.ops[..index]
                .iter()
                .copied()
                .any(|prior| prior.stable_id() == stable_id)
            {
                return Err(TaskCompileError::DuplicateStableId(stable_id));
            }
            if !op.numeric_parameters_are_valid() {
                return Err(TaskCompileError::InvalidParameter(stable_id));
            }
            if let TaskOp::Point { frame, .. }
            | TaskOp::Orientation { frame, .. }
            | TaskOp::DirectionAim { frame, .. } = op
                && frame.0 >= model.bodies.len()
            {
                return Err(TaskCompileError::Frame {
                    task: stable_id,
                    frame,
                });
            }
            let output = op.target_output();
            let Some(kind) = signals.output_kind(output) else {
                return Err(TaskCompileError::OutputSlot {
                    task: stable_id,
                    output,
                });
            };
            let expected = match op {
                TaskOp::Orientation { .. } | TaskOp::FloatingRootOrientation { .. } => {
                    SignalKind::Rotation
                }
                TaskOp::Point { .. }
                | TaskOp::DirectionAim { .. }
                | TaskOp::CenterOfMass { .. }
                | TaskOp::FloatingRootTranslation { .. }
                | TaskOp::FloatingCenterOfMass { .. } => SignalKind::Vector,
            };
            if kind != expected {
                return Err(TaskCompileError::SignalType {
                    task: stable_id,
                    expected,
                    actual: kind,
                });
            }
        }
        self.validate_floating_semantics()?;
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.ops.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ops.is_empty()
    }

    pub fn three_row_slots(&self) -> usize {
        self.ops
            .iter()
            .filter(|op| {
                matches!(
                    op,
                    TaskOp::Point { .. }
                        | TaskOp::Orientation { .. }
                        | TaskOp::DirectionAim { .. }
                        | TaskOp::CenterOfMass { .. }
                )
            })
            .count()
    }

    pub fn floating_task_slots(&self) -> usize {
        self.ops.len() - self.three_row_slots()
    }

    /// Canonical stable-order task operations resolved by the compiler.
    ///
    /// Backends use this immutable plan to allocate fixed query and row slots;
    /// runtime execution must not rediscover task topology.
    pub fn ops(&self) -> &[TaskOp] {
        &self.ops
    }

    pub fn emit_floating_command_into(
        &self,
        signals: &SignalOutputBuffer,
        state: FloatingTaskState,
        command: &mut FloatingTaskCommand,
    ) -> Result<(), TaskEvaluationError> {
        *command = FloatingTaskCommand::default();
        for op in self.ops.iter().copied() {
            match op {
                TaskOp::FloatingRootOrientation {
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                    ..
                } => {
                    let Some(SignalJet::Rotation(target)) =
                        signals.values.get(target_output).copied()
                    else {
                        return Err(TaskEvaluationError::SignalOutput(target_output));
                    };
                    let local_error = state
                        .control_world_from_root
                        .rotation
                        .rotation_to(&target.value)
                        .scaled_axis();
                    let error_world = state
                        .control_world_from_root
                        .rotation
                        .transform_vector(&local_error);
                    let omega = angular_frequency(bandwidth_hz);
                    let current_velocity = Vec3::new(
                        state.root_twist_world.0[0],
                        state.root_twist_world.0[1],
                        state.root_twist_world.0[2],
                    );
                    let acceleration = target.angular_acceleration_world
                        + 2.0
                            * damping_ratio
                            * omega
                            * (target.angular_velocity_world - current_velocity)
                        + omega * omega * error_world;
                    for axis in 0..3 {
                        command.root_acceleration_world.0[axis] =
                            acceleration[axis].clamp(-maximum_acceleration, maximum_acceleration);
                    }
                    command.task_priorities.root_angular = priority;
                    command.task_weights.root_angular = weight;
                }
                TaskOp::FloatingRootTranslation {
                    target_output,
                    horizontal_priority,
                    height_priority,
                    horizontal_weight,
                    height_weight,
                    horizontal_bandwidth_hz,
                    height_bandwidth_hz,
                    horizontal_damping_ratio,
                    height_damping_ratio,
                    maximum_horizontal_acceleration,
                    maximum_height_acceleration,
                    ..
                } => {
                    let Some(SignalJet::Vector(target)) =
                        signals.values.get(target_output).copied()
                    else {
                        return Err(TaskEvaluationError::SignalOutput(target_output));
                    };
                    let current_position = state.control_world_from_root.translation.vector;
                    for axis in 0..3 {
                        let (bandwidth_hz, damping_ratio, maximum_acceleration) = if axis < 2 {
                            (
                                horizontal_bandwidth_hz,
                                horizontal_damping_ratio,
                                maximum_horizontal_acceleration,
                            )
                        } else {
                            (
                                height_bandwidth_hz,
                                height_damping_ratio,
                                maximum_height_acceleration,
                            )
                        };
                        let omega = angular_frequency(bandwidth_hz);
                        let acceleration = target.acceleration[axis]
                            + 2.0
                                * damping_ratio
                                * omega
                                * (target.velocity[axis] - state.root_twist_world.0[3 + axis])
                            + omega * omega * (target.value[axis] - current_position[axis]);
                        command.root_acceleration_world.0[3 + axis] =
                            acceleration.clamp(-maximum_acceleration, maximum_acceleration);
                    }
                    command.task_priorities.root_horizontal = horizontal_priority;
                    command.task_priorities.root_height = height_priority;
                    command.task_weights.root_horizontal = horizontal_weight;
                    command.task_weights.root_height = height_weight;
                }
                TaskOp::FloatingCenterOfMass {
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                    damping_ratio,
                    maximum_acceleration,
                    ..
                } => {
                    let Some(SignalJet::Vector(target)) =
                        signals.values.get(target_output).copied()
                    else {
                        return Err(TaskEvaluationError::SignalOutput(target_output));
                    };
                    let omega = angular_frequency(bandwidth_hz);
                    let acceleration = target.acceleration
                        + 2.0
                            * damping_ratio
                            * omega
                            * (target.velocity - state.center_of_mass_velocity_world)
                        + omega * omega * (target.value - state.center_of_mass_world);
                    command.center_of_mass_task = Some(FloatingCenterOfMassTask {
                        desired_acceleration_world: acceleration
                            .map(|value| value.clamp(-maximum_acceleration, maximum_acceleration)),
                        horizontal_only: false,
                        priority,
                        weight,
                    });
                }
                TaskOp::Point { .. }
                | TaskOp::Orientation { .. }
                | TaskOp::DirectionAim { .. }
                | TaskOp::CenterOfMass { .. } => {}
            }
        }
        Ok(())
    }

    fn validate_floating_semantics(&self) -> Result<(), TaskCompileError> {
        let mut root_orientation = None;
        let mut root_translation = None;
        let mut center_of_mass = None;
        for op in self.ops.iter().copied() {
            let slot = match op {
                TaskOp::FloatingRootOrientation { stable_id, .. } => {
                    (&mut root_orientation, stable_id, "root orientation")
                }
                TaskOp::FloatingRootTranslation { stable_id, .. } => {
                    (&mut root_translation, stable_id, "root translation")
                }
                TaskOp::FloatingCenterOfMass { stable_id, .. } => {
                    (&mut center_of_mass, stable_id, "floating CoM")
                }
                _ => continue,
            };
            if slot.0.replace(slot.1).is_some() {
                return Err(TaskCompileError::DuplicateFloatingSemantic(slot.2));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum TaskEvaluationError {
    #[error("compiled floating task refers to unavailable signal output slot {0}")]
    SignalOutput(usize),
}

fn angular_frequency(bandwidth_hz: f64) -> f64 {
    2.0 * std::f64::consts::PI * bandwidth_hz
}

fn resolve_output(
    signals: &CompiledSignalProgram,
    task: u32,
    stable_output: u32,
    expected: SignalKind,
) -> Result<usize, TaskCompileError> {
    let output =
        signals
            .output_index(stable_output)
            .ok_or(TaskCompileError::MissingSignalOutput {
                task,
                output: stable_output,
            })?;
    let actual = signals
        .output_kind(output)
        .ok_or(TaskCompileError::OutputSlot { task, output })?;
    if actual != expected {
        return Err(TaskCompileError::SignalType {
            task,
            expected,
            actual,
        });
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        signal::{
            CompiledSignalProgram, ScalarJet, SignalInputFrame, SignalMemory, SignalOp,
            SignalOutputSpec, SignalScratch, VectorJet,
        },
        urdf::load_urdf,
    };
    use nalgebra::UnitQuaternion;

    #[test]
    fn compiler_resolves_stable_signal_ids_and_rejects_scalar_targets() {
        let model = load_urdf(include_str!("../../../models/toy_humanoid.urdf")).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![
                SignalOp::ConstantVector {
                    stable_id: 1,
                    jet: VectorJet::default(),
                },
                SignalOp::ConstantScalar {
                    stable_id: 2,
                    jet: ScalarJet::default(),
                },
                SignalOp::ConstantRotation {
                    stable_id: 3,
                    jet: crate::RotationJet::default(),
                },
            ],
            vec![
                SignalOutputSpec {
                    stable_id: 10,
                    node: 0,
                },
                SignalOutputSpec {
                    stable_id: 11,
                    node: 1,
                },
                SignalOutputSpec {
                    stable_id: 12,
                    node: 2,
                },
            ],
        )
        .unwrap();
        let point = TaskSpec::Point {
            stable_id: 20,
            frame: model.frame_id("left_hand").unwrap(),
            point_in_frame: Vec3::zeros(),
            target_signal: 10,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 2.0,
        };
        let compiled = CompiledTaskProgram::compile(&model, &signals, vec![point]).unwrap();
        assert_eq!(compiled.three_row_slots(), 1);
        assert!(matches!(
            compiled.ops()[0],
            TaskOp::Point {
                target_output: 0,
                ..
            }
        ));

        let scalar_target = TaskSpec::CenterOfMass {
            stable_id: 21,
            target_signal: 11,
            priority: Priority::Viability,
            weight: 1.0,
            bandwidth_hz: 1.0,
        };
        assert_eq!(
            CompiledTaskProgram::compile(&model, &signals, vec![scalar_target]),
            Err(TaskCompileError::SignalType {
                task: 21,
                expected: SignalKind::Vector,
                actual: SignalKind::Scalar,
            })
        );

        let orientation = TaskSpec::Orientation {
            stable_id: 22,
            frame: model.frame_id("left_hand").unwrap(),
            target_signal: 12,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 2.0,
        };
        assert!(CompiledTaskProgram::compile(&model, &signals, vec![orientation]).is_ok());

        let direction = TaskSpec::DirectionAim {
            stable_id: 23,
            frame: model.frame_id("left_hand").unwrap(),
            controlled_axis_in_frame: Vec3::new(2.0, 0.0, 0.0),
            target_signal: 10,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 2.0,
        };
        let compiled = CompiledTaskProgram::compile(&model, &signals, vec![direction]).unwrap();
        assert!(matches!(
            compiled.ops()[0],
            TaskOp::DirectionAim {
                controlled_axis_in_frame,
                target_output: 0,
                ..
            } if controlled_axis_in_frame == Vec3::x()
        ));

        let zero_axis = TaskSpec::DirectionAim {
            stable_id: 23,
            frame: model.frame_id("left_hand").unwrap(),
            controlled_axis_in_frame: Vec3::zeros(),
            target_signal: 10,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 2.0,
        };
        assert_eq!(
            CompiledTaskProgram::compile(&model, &signals, vec![zero_axis]),
            Err(TaskCompileError::InvalidParameter(23))
        );

        let rotation_target = TaskSpec::DirectionAim {
            stable_id: 24,
            frame: model.frame_id("left_hand").unwrap(),
            controlled_axis_in_frame: Vec3::x(),
            target_signal: 12,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 2.0,
        };
        assert_eq!(
            CompiledTaskProgram::compile(&model, &signals, vec![rotation_target]),
            Err(TaskCompileError::SignalType {
                task: 24,
                expected: SignalKind::Vector,
                actual: SignalKind::Rotation,
            })
        );
    }

    #[test]
    fn compiled_floating_tasks_emit_bounded_semantic_accelerations() {
        let model = load_urdf(include_str!("../../../models/upkie/upkie.urdf")).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![
                SignalOp::ConstantRotation {
                    stable_id: 1,
                    jet: crate::RotationJet {
                        value: UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.2, 0.0)),
                        ..Default::default()
                    },
                },
                SignalOp::ConstantVector {
                    stable_id: 2,
                    jet: VectorJet {
                        value: Vec3::new(0.1, 0.0, 0.5),
                        ..Default::default()
                    },
                },
                SignalOp::ConstantVector {
                    stable_id: 3,
                    jet: VectorJet {
                        value: Vec3::new(0.0, 0.0, 0.6),
                        ..Default::default()
                    },
                },
            ],
            vec![
                SignalOutputSpec {
                    stable_id: 10,
                    node: 0,
                },
                SignalOutputSpec {
                    stable_id: 11,
                    node: 1,
                },
                SignalOutputSpec {
                    stable_id: 12,
                    node: 2,
                },
            ],
        )
        .unwrap();
        let tasks = CompiledTaskProgram::compile(
            &model,
            &signals,
            vec![
                TaskSpec::FloatingRootOrientation {
                    stable_id: 20,
                    target_signal: 10,
                    priority: Priority::Viability,
                    weight: 1.2,
                    bandwidth_hz: 1.0,
                    damping_ratio: 1.0,
                    maximum_acceleration: 2.0,
                },
                TaskSpec::FloatingRootTranslation {
                    stable_id: 21,
                    target_signal: 11,
                    horizontal_priority: Priority::Preference,
                    height_priority: Priority::Intent,
                    horizontal_weight: 0.8,
                    height_weight: 1.1,
                    horizontal_bandwidth_hz: 1.0,
                    height_bandwidth_hz: 1.0,
                    horizontal_damping_ratio: 1.0,
                    height_damping_ratio: 1.0,
                    maximum_horizontal_acceleration: 1.0,
                    maximum_height_acceleration: 1.5,
                },
                TaskSpec::FloatingCenterOfMass {
                    stable_id: 22,
                    target_signal: 12,
                    priority: Priority::Style,
                    weight: 0.7,
                    bandwidth_hz: 1.0,
                    damping_ratio: 1.0,
                    maximum_acceleration: 3.0,
                },
            ],
        )
        .unwrap();
        let memory = SignalMemory::new(&signals);
        let mut next_memory = SignalMemory::new(&signals);
        let mut output = SignalOutputBuffer::new(&signals);
        let mut scratch = SignalScratch::new(&signals);
        signals
            .evaluate_into(
                0.005,
                &SignalInputFrame::default(),
                &memory,
                &mut next_memory,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let mut command = FloatingTaskCommand::default();
        tasks
            .emit_floating_command_into(
                &output,
                FloatingTaskState {
                    control_world_from_root: Transform3::identity(),
                    root_twist_world: Motion6::default(),
                    center_of_mass_world: Vec3::zeros(),
                    center_of_mass_velocity_world: Vec3::zeros(),
                },
                &mut command,
            )
            .unwrap();

        assert_eq!(tasks.three_row_slots(), 0);
        assert_eq!(tasks.floating_task_slots(), 3);
        assert!((command.root_acceleration_world.0[1] - 2.0).abs() < 1e-12);
        assert!((command.root_acceleration_world.0[3] - 1.0).abs() < 1e-12);
        assert!((command.root_acceleration_world.0[5] - 1.5).abs() < 1e-12);
        assert_eq!(command.task_priorities.root_angular, Priority::Viability);
        assert_eq!(
            command.task_priorities.root_horizontal,
            Priority::Preference
        );
        assert_eq!(command.task_priorities.root_height, Priority::Intent);
        assert!((command.task_weights.root_angular - 1.2).abs() < 1e-12);
        let center_of_mass = command.center_of_mass_task.unwrap();
        assert_eq!(center_of_mass.priority, Priority::Style);
        assert!((center_of_mass.weight - 0.7).abs() < 1e-12);
        assert!((center_of_mass.desired_acceleration_world.z - 3.0).abs() < 1e-12);

        let duplicate = CompiledTaskProgram::compile(
            &model,
            &signals,
            vec![
                TaskSpec::FloatingRootOrientation {
                    stable_id: 30,
                    target_signal: 10,
                    priority: Priority::Viability,
                    weight: 1.0,
                    bandwidth_hz: 1.0,
                    damping_ratio: 1.0,
                    maximum_acceleration: 2.0,
                },
                TaskSpec::FloatingRootOrientation {
                    stable_id: 31,
                    target_signal: 10,
                    priority: Priority::Intent,
                    weight: 1.0,
                    bandwidth_hz: 1.0,
                    damping_ratio: 1.0,
                    maximum_acceleration: 2.0,
                },
            ],
        );
        assert_eq!(
            duplicate,
            Err(TaskCompileError::DuplicateFloatingSemantic(
                "root orientation"
            ))
        );
    }
}
