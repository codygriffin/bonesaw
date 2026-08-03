use nalgebra::{DVector, UnitQuaternion};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    collision::{
        CollisionAvoidanceConfig, CollisionError, CollisionEvaluationScratch,
        CompiledCollisionModel, DistanceSample,
    },
    math::{ControlTime, Vec3},
    model::{CompiledModel, DynamicsCache, FrameId, ModelCache, ModelError, RobotState},
    program::MotionProgram,
    rig::{CompiledTaskProgram, TaskOp},
    signal::{
        CompiledSignalProgram, SignalEvaluationError, SignalInputFrame, SignalJet, SignalMemory,
        SignalOutputBuffer, SignalScratch,
    },
    solver::{
        ConstraintBuffer, HierarchicalSolver, LinearConstraint, Priority, SolveDiagnostics,
        SolveResult, SolveStatus, SolverWorkspace, TaskBuffer, TaskKind, VelocityBounds,
    },
    trajectory::{
        ActuatorSampleBlock, QuinticSegment, SegmentExtrema, SegmentLimits, TrajectoryError,
    },
};

#[derive(Clone, Debug)]
pub struct FrameTarget {
    pub stable_id: u32,
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub target_position_world: Vec3,
    pub target_orientation_world: Option<UnitQuaternion<f64>>,
    pub feedforward_linear_velocity_world: Vec3,
    pub priority: Priority,
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub struct ControllerInput {
    pub tick_time_ns: ControlTime,
    pub state: RobotState,
    pub signal_inputs: SignalInputFrame,
    pub frame_targets: Vec<FrameTarget>,
    pub com_target_world: Option<Vec3>,
    pub posture_target: Option<DVector<f64>>,
    /// Precompiled velocity-space hard rows `lower <= A qdot <= upper`.
    pub hard_constraints: Vec<LinearConstraint>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct FrameState {
    pub id: usize,
    pub translation: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum StepStatus {
    Ok,
    Degraded,
    Contingency,
    Rejected,
}

#[derive(Clone, Debug)]
pub struct ControllerOutput {
    pub status: StepStatus,
    pub next_state: RobotState,
    pub commanded_velocity: DVector<f64>,
    /// Uniform scale applied to the hierarchy's velocity solution after exact
    /// trajectory validation. Zero denotes a selected contingency.
    pub trajectory_velocity_scale: f64,
    /// Number of fixed 0.5 backtracking steps used to admit the trajectory.
    pub trajectory_backtrack_steps: usize,
    pub primary_segment: QuinticSegment,
    pub contingency_segment: QuinticSegment,
    pub samples: ActuatorSampleBlock,
    pub frames: Vec<FrameState>,
    pub solve: SolveDiagnostics,
    pub segment_extrema: SegmentExtrema,
}

/// All semantic controller memory is caller-visible and double-bufferable.
#[derive(Clone, Debug)]
pub struct ControllerState {
    pub observed_state: RobotState,
    pub commanded_state: RobotState,
    pub previous_primary: QuinticSegment,
    pub previous_contingency: QuinticSegment,
    pub has_previous_plan: bool,
    pub last_tick_time_ns: Option<ControlTime>,
    pub tick_count: u64,
    pub program_epoch: u64,
    pub signal_memory: SignalMemory,
}

impl ControllerState {
    pub fn new(initial_state: RobotState, program_epoch: u64) -> Self {
        let dof = initial_state.q.len();
        let empty_segment = || QuinticSegment {
            start_time_ns: 0,
            duration_ns: 0,
            coefficients: vec![[0.0; 6]; dof],
        };
        Self {
            observed_state: initial_state.clone(),
            commanded_state: initial_state,
            previous_primary: empty_segment(),
            previous_contingency: empty_segment(),
            has_previous_plan: false,
            last_tick_time_ns: None,
            tick_count: 0,
            program_epoch,
            signal_memory: SignalMemory::default(),
        }
    }

    pub fn for_program(initial_state: RobotState, program: &MotionProgram) -> Self {
        let mut state = Self::new(initial_state, program.header.program_epoch);
        state.signal_memory = SignalMemory::new(&program.signals);
        state
    }
}

/// Caller-owned non-semantic workspace. Capacities and model caches survive
/// across ticks; changing scratch contents cannot change controller semantics.
#[derive(Clone, Debug)]
pub struct ControllerScratch {
    pub current_model: ModelCache,
    pub next_model: ModelCache,
    pub dynamics: DynamicsCache,
    collision_state: RobotState,
    collision_acceleration: DVector<f64>,
    zero: DVector<f64>,
    primary_endpoint: DVector<f64>,
    contingency_endpoint: DVector<f64>,
    bounds: VelocityBounds,
    segment_limits: Vec<SegmentLimits>,
    tasks: TaskBuffer,
    hard_constraints: ConstraintBuffer,
    collision_sample: DistanceSample,
    collision_evaluation: CollisionEvaluationScratch,
    solver_workspace: SolverWorkspace,
    solve_result: SolveResult,
    signal_scratch: SignalScratch,
    signal_output: SignalOutputBuffer,
    next_signal_memory: SignalMemory,
}

impl ControllerScratch {
    pub fn new(model: &CompiledModel, task_capacity: usize) -> Self {
        Self::with_signals(model, task_capacity, &CompiledSignalProgram::default())
    }

    pub fn for_program(program: &MotionProgram, additional_task_capacity: usize) -> Self {
        Self::with_signals(
            &program.model,
            program
                .tasks
                .three_row_slots()
                .saturating_add(additional_task_capacity),
            &program.signals,
        )
    }

    fn with_signals(
        model: &CompiledModel,
        task_capacity: usize,
        signals: &CompiledSignalProgram,
    ) -> Self {
        let collision = CompiledCollisionModel::compile(model);
        let collision_pairs = collision
            .self_pairs
            .len()
            .max(collision.validation_pairs.len());
        let collision_primitives = collision.validation_primitives.len();
        let collision_validation_pairs = collision.validation_pairs.len();
        let maximum_tasks = task_capacity
            .saturating_add(collision_pairs)
            .saturating_add(2);
        let maximum_task_rows = model
            .dof
            .saturating_add(task_capacity.saturating_mul(3))
            .saturating_add(collision_pairs);
        let maximum_constraints = model
            .dof
            .saturating_mul(2)
            .saturating_add(task_capacity)
            .saturating_add(collision_pairs);
        Self {
            current_model: ModelCache::new(model),
            next_model: ModelCache::new(model),
            dynamics: DynamicsCache::new(model),
            collision_state: RobotState::zeros(model),
            collision_acceleration: DVector::zeros(model.dof),
            zero: DVector::zeros(model.dof),
            primary_endpoint: DVector::zeros(model.dof),
            contingency_endpoint: DVector::zeros(model.dof),
            bounds: VelocityBounds::unbounded(model.dof),
            segment_limits: Vec::with_capacity(model.dof),
            tasks: TaskBuffer::new(model.dof, task_capacity, collision_pairs),
            hard_constraints: ConstraintBuffer::new(model.dof, maximum_constraints),
            collision_sample: DistanceSample::workspace(model.dof),
            collision_evaluation: CollisionEvaluationScratch::new(
                model.dof,
                collision_primitives,
                collision_validation_pairs,
            ),
            solver_workspace: SolverWorkspace::new(
                model.dof,
                maximum_task_rows,
                maximum_tasks,
                maximum_constraints,
            ),
            solve_result: SolveResult::workspace(model.dof, maximum_constraints),
            signal_scratch: SignalScratch::new(signals),
            signal_output: SignalOutputBuffer::new(signals),
            next_signal_memory: SignalMemory::new(signals),
        }
    }

    pub fn task_capacity(&self) -> usize {
        self.tasks.three_row_capacity()
    }
}

#[derive(Clone, Debug, Default)]
pub struct ControllerOutputBuffer {
    pub value: Option<ControllerOutput>,
}

impl ControllerOutputBuffer {
    pub fn take(&mut self) -> Option<ControllerOutput> {
        self.value.take()
    }

    /// Prewarm all output-owned variable-size buffers for a compiled layout.
    pub fn with_layout(
        model: &CompiledModel,
        control_horizon_ns: i64,
        sample_period_ns: i64,
    ) -> Self {
        Self {
            value: Some(ControllerOutput::workspace(
                model,
                control_horizon_ns,
                sample_period_ns,
            )),
        }
    }
}

impl ControllerOutput {
    fn workspace(model: &CompiledModel, control_horizon_ns: i64, sample_period_ns: i64) -> Self {
        let dof = model.dof;
        let sample_count = if sample_period_ns > 0 {
            (control_horizon_ns / sample_period_ns).max(0) as usize
        } else {
            0
        };
        let segment = || QuinticSegment {
            start_time_ns: 0,
            duration_ns: control_horizon_ns,
            coefficients: vec![[0.0; 6]; dof],
        };
        Self {
            status: StepStatus::Rejected,
            next_state: RobotState::zeros(model),
            commanded_velocity: DVector::zeros(dof),
            trajectory_velocity_scale: 0.0,
            trajectory_backtrack_steps: 0,
            primary_segment: segment(),
            contingency_segment: segment(),
            samples: ActuatorSampleBlock::with_capacity(sample_count, dof),
            frames: vec![
                FrameState {
                    id: 0,
                    translation: [0.0; 3],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                };
                model.bodies.len()
            ],
            solve: SolveDiagnostics {
                status: SolveStatus::InvalidProblem,
                level_residuals: Vec::with_capacity(Priority::ALL.len()),
                minimum_bound_margin: f64::NEG_INFINITY,
                maximum_bound_violation: f64::INFINITY,
                limiting_bound_coordinate: None,
                limiting_bound_is_upper: false,
                clipped_levels: Vec::with_capacity(Priority::ALL.len()),
                rank_by_level: Vec::with_capacity(Priority::ALL.len()),
                active_constraints: Vec::new(),
                maximum_linear_constraint_violation: f64::INFINITY,
                limiting_linear_constraint: None,
                limiting_linear_constraint_is_upper: false,
                maximum_constraint_violation: f64::INFINITY,
                task_pseudoinverse_calls: 0,
                task_pseudoinverse_calls_by_level: [0; Priority::ALL.len()],
                task_jacobi_sweeps: 0,
                task_jacobi_sweeps_by_level: [0; Priority::ALL.len()],
                clipped_steps: 0,
                clipped_steps_by_level: [0; Priority::ALL.len()],
                low_authority_budget_exhausted_level: None,
                equality_pseudoinverse_reused: false,
                feasibility_projection_sweeps: 0,
                feasibility_halfspace_projections: 0,
                feasibility_seed_reused: false,
                feasibility_prefix_resumed: false,
                feasibility_polish_iterations: 0,
                feasibility_polish_pseudoinverse_calls: 0,
                feasibility_polish_jacobi_sweeps: 0,
            },
            segment_extrema: SegmentExtrema {
                min_position: vec![0.0; dof],
                max_position: vec![0.0; dof],
                max_abs_velocity: vec![0.0; dof],
                max_abs_acceleration: vec![0.0; dof],
                max_abs_jerk: vec![0.0; dof],
            },
        }
    }
}

#[derive(Clone, Debug)]
pub struct ControllerConfig {
    pub control_horizon_ns: i64,
    pub sample_period_ns: i64,
    pub point_bandwidth_hz: f64,
    pub orientation_bandwidth_hz: f64,
    pub com_bandwidth_hz: f64,
    pub posture_bandwidth_hz: f64,
    pub max_acceleration: f64,
    pub max_jerk: f64,
    /// Optional conservative self-collision barriers and repellers. Disabled
    /// until a model-specific proxy/filter profile is compiled.
    pub collision_avoidance: Option<CollisionAvoidanceConfig>,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            control_horizon_ns: 20_000_000,
            sample_period_ns: 1_000_000,
            point_bandwidth_hz: 2.5,
            orientation_bandwidth_hz: 2.0,
            com_bandwidth_hz: 1.5,
            posture_bandwidth_hz: 0.8,
            // Prototype defaults are deliberately permissive; hardware
            // profiles must compile actuator-specific physical limits.
            max_acceleration: 5_000.0,
            max_jerk: 1_000_000.0,
            collision_avoidance: None,
        }
    }
}

#[derive(Debug, Error)]
pub enum ControllerError {
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error(transparent)]
    Trajectory(#[from] TrajectoryError),
    #[error("controller horizon and sample period must be positive and divide exactly")]
    Timing,
    #[error("a target contains an invalid frame id or non-finite value")]
    InvalidTarget,
    #[error("posture target dimension does not match the model")]
    PostureDimension,
    #[error("compiled controller task scratch capacity is too small")]
    TaskCapacity,
    #[error("compiled controller constraint scratch capacity is too small")]
    ConstraintCapacity,
    #[error("hard constraint dimension does not match the model")]
    ConstraintDimension,
    #[error(transparent)]
    Collision(#[from] CollisionError),
    #[error(transparent)]
    Signal(#[from] SignalEvaluationError),
    #[error("compiled task and signal layouts do not agree")]
    CompiledTaskLayout,
    #[error("tick time moved backwards relative to controller state")]
    NonMonotonicTime,
    #[error("controller acceleration and jerk limits must be finite and positive")]
    MotionLimits,
}

pub struct Controller {
    model: CompiledModel,
    config: ControllerConfig,
    solver: HierarchicalSolver,
    collision: CompiledCollisionModel,
    program_epoch: u64,
    signals: CompiledSignalProgram,
    tasks: CompiledTaskProgram,
}

impl Controller {
    pub fn new(model: CompiledModel, config: ControllerConfig) -> Result<Self, ControllerError> {
        if config.control_horizon_ns <= 0
            || config.sample_period_ns <= 0
            || config.control_horizon_ns % config.sample_period_ns != 0
        {
            return Err(ControllerError::Timing);
        }
        if !config.max_acceleration.is_finite()
            || config.max_acceleration <= 0.0
            || !config.max_jerk.is_finite()
            || config.max_jerk <= 0.0
        {
            return Err(ControllerError::MotionLimits);
        }
        if config
            .collision_avoidance
            .is_some_and(|collision| !collision.validate())
        {
            return Err(CollisionError::InvalidConfig.into());
        }
        let collision = CompiledCollisionModel::compile(&model);
        Ok(Self {
            model,
            config,
            solver: HierarchicalSolver::default(),
            collision,
            program_epoch: 0,
            signals: CompiledSignalProgram::default(),
            tasks: CompiledTaskProgram::default(),
        })
    }

    pub fn model(&self) -> &CompiledModel {
        &self.model
    }

    pub fn from_program(program: &MotionProgram) -> Result<Self, ControllerError> {
        let config = ControllerConfig {
            control_horizon_ns: program.timing.control_horizon_ns,
            sample_period_ns: program.timing.sample_period_ns,
            collision_avoidance: program.collision_avoidance,
            ..ControllerConfig::default()
        };
        let mut controller = Self::new(program.model.clone(), config)?;
        controller.program_epoch = program.header.program_epoch;
        controller.signals = program.signals.clone();
        controller.tasks = program.tasks.clone();
        Ok(controller)
    }

    pub fn advance(&self, input: ControllerInput) -> Result<ControllerOutput, ControllerError> {
        let segment_start = input.state.clone();
        let mut scratch = ControllerScratch::with_signals(
            &self.model,
            input
                .frame_targets
                .len()
                .saturating_mul(2)
                .saturating_add(self.tasks.three_row_slots())
                .saturating_add(2),
            &self.signals,
        );
        let signal_memory = SignalMemory::new(&self.signals);
        self.advance_with_scratch(
            &input,
            &segment_start,
            &signal_memory,
            self.config.control_horizon_ns as f64 * 1e-9,
            &mut scratch,
            None,
        )
    }

    /// Explicit state transition matching the library architecture. Observed
    /// state drives errors and constraints; the new segment splices from the
    /// prior commanded state carried by `state_in`.
    pub fn advance_into(
        &self,
        input: ControllerInput,
        state_in: &ControllerState,
        state_out: &mut ControllerState,
        output: &mut ControllerOutputBuffer,
        scratch: &mut ControllerScratch,
    ) -> Result<StepStatus, ControllerError> {
        self.advance_into_reusable(&input, state_in, state_out, output, scratch)
    }

    /// Allocation-conscious state transition borrowing all input storage.
    ///
    /// Callers may retain one `ControllerInput`, update its scalar values and
    /// refill its preallocated vectors between ticks. This is the preferred
    /// entry point for native batch loops and language bindings.
    pub fn advance_into_reusable(
        &self,
        input: &ControllerInput,
        state_in: &ControllerState,
        state_out: &mut ControllerState,
        output: &mut ControllerOutputBuffer,
        scratch: &mut ControllerScratch,
    ) -> Result<StepStatus, ControllerError> {
        if state_in.program_epoch != self.program_epoch {
            return Ok(StepStatus::Rejected);
        }
        let signal_timestep_seconds = match state_in.last_tick_time_ns {
            Some(last_tick) if input.tick_time_ns < last_tick => {
                return Err(ControllerError::NonMonotonicTime);
            }
            Some(last_tick) => (input.tick_time_ns - last_tick) as f64 * 1e-9,
            None => self.config.control_horizon_ns as f64 * 1e-9,
        };
        let reusable_output = output.value.take();
        let result = self.advance_with_scratch(
            input,
            &state_in.commanded_state,
            &state_in.signal_memory,
            signal_timestep_seconds,
            scratch,
            reusable_output,
        )?;
        state_out.observed_state.clone_from(&input.state);
        state_out.commanded_state.clone_from(&result.next_state);
        state_out
            .previous_primary
            .clone_from(&result.primary_segment);
        state_out
            .previous_contingency
            .clone_from(&result.contingency_segment);
        state_out.has_previous_plan = true;
        state_out.last_tick_time_ns = Some(
            result
                .samples
                .time(result.samples.len().saturating_sub(1))
                .map_or(0, |time_ns| time_ns - self.config.control_horizon_ns),
        );
        state_out.tick_count = state_in.tick_count.saturating_add(1);
        state_out.program_epoch = state_in.program_epoch;
        state_out
            .signal_memory
            .clone_from(&scratch.next_signal_memory);
        let status = result.status;
        output.value = Some(result);
        Ok(status)
    }

    fn advance_with_scratch(
        &self,
        input: &ControllerInput,
        segment_start: &RobotState,
        signal_memory_in: &SignalMemory,
        signal_timestep_seconds: f64,
        scratch: &mut ControllerScratch,
        reusable_output: Option<ControllerOutput>,
    ) -> Result<ControllerOutput, ControllerError> {
        input.state.validate(&self.model)?;
        segment_start.validate(&self.model)?;
        self.model
            .forward_kinematics(&input.state, &mut scratch.current_model)?;
        self.signals.evaluate_into(
            signal_timestep_seconds,
            &input.signal_inputs,
            signal_memory_in,
            &mut scratch.next_signal_memory,
            &mut scratch.signal_output,
            &mut scratch.signal_scratch,
        )?;
        let reusable_layout_matches = reusable_output.as_ref().is_some_and(|output| {
            output.next_state.q.len() == self.model.dof
                && output.frames.len() == self.model.bodies.len()
                && output.samples.dof == self.model.dof
        });
        let mut result = if reusable_layout_matches {
            reusable_output.expect("layout match requires an output")
        } else {
            ControllerOutput::workspace(
                &self.model,
                self.config.control_horizon_ns,
                self.config.sample_period_ns,
            )
        };
        scratch.tasks.begin();
        let tasks = &mut scratch.tasks;

        for op in self.tasks.ops().iter().copied() {
            match op {
                TaskOp::Point {
                    stable_id,
                    frame,
                    point_in_frame,
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                } => {
                    let Some(SignalJet::Vector(target)) =
                        scratch.signal_output.values.get(target_output).copied()
                    else {
                        return Err(ControllerError::CompiledTaskLayout);
                    };
                    let pose = self.model.frame_pose(&scratch.current_model, frame)?;
                    let current = pose
                        .transform_point(&nalgebra::Point3::from(point_in_frame))
                        .coords;
                    let desired =
                        target.velocity + bandwidth_gain(bandwidth_hz) * (target.value - current);
                    let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
                    task.stable_id = stable_id;
                    task.kind = TaskKind::Point;
                    task.priority = priority;
                    self.model.point_jacobian_into(
                        &scratch.current_model,
                        frame,
                        point_in_frame,
                        &mut task.jacobian,
                    )?;
                    task.target_velocity
                        .as_mut_slice()
                        .copy_from_slice(desired.as_slice());
                    task.weight = weight;
                }
                TaskOp::Orientation {
                    stable_id,
                    frame,
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                } => {
                    let Some(SignalJet::Rotation(target)) =
                        scratch.signal_output.values.get(target_output).copied()
                    else {
                        return Err(ControllerError::CompiledTaskLayout);
                    };
                    let pose = self.model.frame_pose(&scratch.current_model, frame)?;
                    let local_error = pose.rotation.rotation_to(&target.value).scaled_axis();
                    let error_world = pose.rotation.transform_vector(&local_error);
                    let angular_target =
                        target.angular_velocity_world + bandwidth_gain(bandwidth_hz) * error_world;
                    let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
                    task.stable_id = stable_id;
                    task.kind = TaskKind::Orientation;
                    task.priority = priority;
                    self.model.angular_jacobian_into(
                        &scratch.current_model,
                        frame,
                        &mut task.jacobian,
                    )?;
                    task.target_velocity
                        .as_mut_slice()
                        .copy_from_slice(angular_target.as_slice());
                    task.weight = weight;
                }
                TaskOp::DirectionAim {
                    stable_id,
                    frame,
                    controlled_axis_in_frame,
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                } => {
                    let Some(SignalJet::Vector(target)) =
                        scratch.signal_output.values.get(target_output).copied()
                    else {
                        return Err(ControllerError::CompiledTaskLayout);
                    };
                    let target_norm = target.value.norm();
                    if !target_norm.is_finite() || target_norm <= 1e-12 {
                        return Err(ControllerError::InvalidTarget);
                    }
                    let desired_axis_world = target.value / target_norm;
                    let desired_axis_velocity_world = (target.velocity
                        - desired_axis_world * desired_axis_world.dot(&target.velocity))
                        / target_norm;
                    if !desired_axis_velocity_world
                        .iter()
                        .all(|value| value.is_finite())
                    {
                        return Err(ControllerError::InvalidTarget);
                    }

                    let pose = self.model.frame_pose(&scratch.current_model, frame)?;
                    let current_axis_world =
                        pose.rotation.transform_vector(&controlled_axis_in_frame);
                    let mut angular_target = desired_axis_world.cross(&desired_axis_velocity_world)
                        + bandwidth_gain(bandwidth_hz)
                            * current_axis_world.cross(&desired_axis_world);
                    angular_target -= current_axis_world * current_axis_world.dot(&angular_target);

                    let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
                    task.stable_id = stable_id;
                    task.kind = TaskKind::Gaze;
                    task.priority = priority;
                    self.model.angular_jacobian_into(
                        &scratch.current_model,
                        frame,
                        &mut task.jacobian,
                    )?;
                    for column in 0..task.jacobian.ncols() {
                        let angular_column = Vec3::new(
                            task.jacobian[(0, column)],
                            task.jacobian[(1, column)],
                            task.jacobian[(2, column)],
                        );
                        let tangent_column = angular_column
                            - current_axis_world * current_axis_world.dot(&angular_column);
                        task.jacobian[(0, column)] = tangent_column.x;
                        task.jacobian[(1, column)] = tangent_column.y;
                        task.jacobian[(2, column)] = tangent_column.z;
                    }
                    task.target_velocity
                        .as_mut_slice()
                        .copy_from_slice(angular_target.as_slice());
                    task.weight = weight;
                }
                TaskOp::CenterOfMass {
                    stable_id,
                    target_output,
                    priority,
                    weight,
                    bandwidth_hz,
                } => {
                    let Some(SignalJet::Vector(target)) =
                        scratch.signal_output.values.get(target_output).copied()
                    else {
                        return Err(ControllerError::CompiledTaskLayout);
                    };
                    let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
                    task.stable_id = stable_id;
                    task.kind = TaskKind::CenterOfMass;
                    task.priority = priority;
                    self.model
                        .com_jacobian_into(&scratch.current_model, &mut task.jacobian)?;
                    let target_velocity = target.velocity
                        + bandwidth_gain(bandwidth_hz)
                            * (target.value - scratch.current_model.center_of_mass_world);
                    task.target_velocity
                        .as_mut_slice()
                        .copy_from_slice(target_velocity.as_slice());
                    task.weight = weight;
                }
                TaskOp::FloatingRootOrientation { .. }
                | TaskOp::FloatingRootTranslation { .. }
                | TaskOp::FloatingCenterOfMass { .. } => {}
            }
        }

        for target in &input.frame_targets {
            if target.frame.0 >= self.model.bodies.len()
                || !target
                    .target_position_world
                    .iter()
                    .chain(target.point_in_frame.iter())
                    .chain(target.feedforward_linear_velocity_world.iter())
                    .all(|x| x.is_finite())
                || !target.weight.is_finite()
            {
                return Err(ControllerError::InvalidTarget);
            }
            let pose = self
                .model
                .frame_pose(&scratch.current_model, target.frame)?;
            let current = pose
                .transform_point(&nalgebra::Point3::from(target.point_in_frame))
                .coords;
            let gain = bandwidth_gain(self.config.point_bandwidth_hz);
            let desired = target.feedforward_linear_velocity_world
                + gain * (target.target_position_world - current);
            let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
            task.stable_id = target.stable_id;
            task.kind = TaskKind::Point;
            task.priority = target.priority;
            self.model.point_jacobian_into(
                &scratch.current_model,
                target.frame,
                target.point_in_frame,
                &mut task.jacobian,
            )?;
            task.target_velocity
                .as_mut_slice()
                .copy_from_slice(desired.as_slice());
            task.weight = target.weight;

            if let Some(desired_orientation) = &target.target_orientation_world {
                let local_error = pose.rotation.rotation_to(desired_orientation).scaled_axis();
                let error = pose.rotation.transform_vector(&local_error);
                let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
                task.stable_id = target.stable_id.saturating_add(1_000_000);
                task.kind = TaskKind::Orientation;
                task.priority = target.priority;
                self.model.angular_jacobian_into(
                    &scratch.current_model,
                    target.frame,
                    &mut task.jacobian,
                )?;
                let angular_target = bandwidth_gain(self.config.orientation_bandwidth_hz) * error;
                task.target_velocity
                    .as_mut_slice()
                    .copy_from_slice(angular_target.as_slice());
                task.weight = target.weight;
            }
        }

        if let Some(target) = input.com_target_world {
            if !target.iter().all(|x| x.is_finite()) {
                return Err(ControllerError::InvalidTarget);
            }
            let task = tasks.push_three().ok_or(ControllerError::TaskCapacity)?;
            task.stable_id = 2_000_000;
            task.kind = TaskKind::CenterOfMass;
            task.priority = Priority::Viability;
            self.model
                .com_jacobian_into(&scratch.current_model, &mut task.jacobian)?;
            let target_velocity = bandwidth_gain(self.config.com_bandwidth_hz)
                * (target - scratch.current_model.center_of_mass_world);
            task.target_velocity
                .as_mut_slice()
                .copy_from_slice(target_velocity.as_slice());
            task.weight = 1.0;
        }

        if let Some(posture) = input.posture_target.as_ref() {
            if posture.len() != self.model.dof {
                return Err(ControllerError::PostureDimension);
            }
            let task = tasks.push_posture().ok_or(ControllerError::TaskCapacity)?;
            task.stable_id = 3_000_000;
            task.kind = TaskKind::Posture;
            task.priority = Priority::Preference;
            task.jacobian.fill(0.0);
            let gain = bandwidth_gain(self.config.posture_bandwidth_hz);
            for coordinate in 0..self.model.dof {
                task.jacobian[(coordinate, coordinate)] = 1.0;
                task.target_velocity[coordinate] =
                    gain * (posture[coordinate] - input.state.q[coordinate]);
            }
            task.weight = 0.2;
        }

        let horizon = self.config.control_horizon_ns as f64 * 1e-9;
        self.fill_velocity_bounds(&input.state, horizon, &mut scratch.bounds);
        let hard_constraints = if let Some(collision_config) = self.config.collision_avoidance {
            scratch.hard_constraints.begin();
            for constraint in &input.hard_constraints {
                if constraint.coefficients.len() != self.model.dof {
                    return Err(ControllerError::ConstraintDimension);
                }
                let row = scratch
                    .hard_constraints
                    .push()
                    .ok_or(ControllerError::ConstraintCapacity)?;
                row.stable_id = constraint.stable_id;
                row.coefficients.copy_from(&constraint.coefficients);
                row.lower = constraint.lower;
                row.upper = constraint.upper;
            }
            self.collision.emit_avoidance_rows_buffered(
                &self.model,
                &scratch.current_model,
                &input.state.v,
                collision_config,
                &mut scratch.hard_constraints,
                tasks,
                &mut scratch.collision_sample,
                &mut scratch.collision_evaluation,
            )?;
            scratch.hard_constraints.active()
        } else {
            input.hard_constraints.as_slice()
        };
        self.solver.solve_task_buffer_into(
            self.model.dof,
            tasks,
            &scratch.bounds,
            hard_constraints,
            &mut scratch.solve_result,
            &mut scratch.solver_workspace,
        );
        let solve = &scratch.solve_result;
        for index in 0..self.model.dof {
            scratch.zero[index] = 0.0;
            scratch.contingency_endpoint[index] =
                segment_start.q[index] + 0.5 * horizon * segment_start.v[index];
        }
        result.contingency_segment.set_boundary_conditions(
            input.tick_time_ns,
            self.config.control_horizon_ns,
            &segment_start.q,
            &segment_start.v,
            &scratch.zero,
            &scratch.contingency_endpoint,
            &scratch.zero,
            &scratch.zero,
        )?;
        self.fill_segment_limits(&mut scratch.segment_limits);
        // A target can legitimately ask for more velocity than a 20 ms
        // quintic can realize under the configured acceleration/jerk envelope.
        // Retrying the same rejected velocity forever latches the controller in
        // its braking contingency even after the target retreats. Uniformly
        // backtrack the solved velocity with fixed work until the complete
        // segment (including collision validation) is admissible. Scaling
        // preserves the solved direction and joint ratios; hard gates are
        // never weakened.
        const TRAJECTORY_BACKTRACK_STEPS: usize = 12;
        let mut use_primary = false;
        let mut accepted_velocity_scale = 0.0;
        let mut accepted_backtrack_steps = TRAJECTORY_BACKTRACK_STEPS + 1;
        for attempt in 0..=TRAJECTORY_BACKTRACK_STEPS {
            let velocity_scale = 0.5_f64.powi(attempt as i32);
            for index in 0..self.model.dof {
                scratch.collision_acceleration[index] = solve.velocity[index] * velocity_scale;
                scratch.primary_endpoint[index] =
                    segment_start.q[index] + horizon * scratch.collision_acceleration[index];
            }
            result.primary_segment.set_boundary_conditions(
                input.tick_time_ns,
                self.config.control_horizon_ns,
                &segment_start.q,
                &segment_start.v,
                &scratch.zero,
                &scratch.primary_endpoint,
                &scratch.collision_acceleration,
                &scratch.zero,
            )?;
            if result
                .primary_segment
                .validate_into(&scratch.segment_limits, &mut result.segment_extrema)
                .is_err()
            {
                continue;
            }
            let collision_clear = if let Some(collision_config) = self.config.collision_avoidance {
                self.collision
                    .validate_segment_on_grid_buffered(
                        &self.model,
                        &result.primary_segment,
                        segment_start.control_world_from_root,
                        self.config.sample_period_ns,
                        collision_config.hard_margin,
                        &mut scratch.collision_state,
                        &mut scratch.collision_acceleration,
                        &mut scratch.next_model,
                        &mut scratch.collision_sample,
                        &mut scratch.collision_evaluation,
                    )?
                    .is_clear()
            } else {
                true
            };
            if collision_clear {
                use_primary = true;
                accepted_velocity_scale = velocity_scale;
                accepted_backtrack_steps = attempt;
                break;
            }
        }
        let status = if use_primary {
            if solve.diagnostics.status == SolveStatus::Solved && accepted_velocity_scale == 1.0 {
                StepStatus::Ok
            } else {
                StepStatus::Degraded
            }
        } else {
            result
                .contingency_segment
                .extrema_into(&mut result.segment_extrema);
            StepStatus::Contingency
        };
        if use_primary {
            result
                .primary_segment
                .sample_block_into(self.config.sample_period_ns, &mut result.samples)?;
        } else {
            if result
                .contingency_segment
                .validate_into(&scratch.segment_limits, &mut result.segment_extrema)
                .is_err()
            {
                // The observed state arrived outside the controller's
                // one-step braking envelope. Do not publish a braking segment
                // that crosses a hard joint bound: hold the represented state
                // and zero its outgoing velocity as the explicit emergency
                // contingency. Normal controller-produced states are kept
                // inside this envelope by `fill_velocity_bounds` below.
                result.contingency_segment.set_boundary_conditions(
                    input.tick_time_ns,
                    self.config.control_horizon_ns,
                    &segment_start.q,
                    &scratch.zero,
                    &scratch.zero,
                    &segment_start.q,
                    &scratch.zero,
                    &scratch.zero,
                )?;
            }
            result
                .contingency_segment
                .sample_block_into(self.config.sample_period_ns, &mut result.samples)?;
        }
        let terminal_index = result.samples.len().saturating_sub(1);
        let terminal_position = result
            .samples
            .position(terminal_index)
            .expect("positive horizon and sample period produce at least one sample");
        let terminal_velocity = result
            .samples
            .velocity(terminal_index)
            .expect("positive horizon and sample period produce at least one sample");
        result.next_state.control_world_from_root = segment_start.control_world_from_root;
        result
            .next_state
            .q
            .as_mut_slice()
            .copy_from_slice(terminal_position);
        result
            .next_state
            .v
            .as_mut_slice()
            .copy_from_slice(terminal_velocity);
        if status == StepStatus::Contingency {
            result.commanded_velocity.fill(0.0);
            result.trajectory_velocity_scale = 0.0;
            result.trajectory_backtrack_steps = TRAJECTORY_BACKTRACK_STEPS + 1;
        } else {
            for index in 0..self.model.dof {
                result.commanded_velocity[index] = solve.velocity[index] * accepted_velocity_scale;
            }
            result.trajectory_velocity_scale = accepted_velocity_scale;
            result.trajectory_backtrack_steps = accepted_backtrack_steps;
        }
        self.model
            .forward_kinematics(&result.next_state, &mut scratch.next_model)?;
        for body in &self.model.bodies {
            let pose = scratch.next_model.world_from_body[body.id.0];
            let quaternion = pose.rotation.quaternion();
            result.frames[body.id.0] = FrameState {
                id: body.id.0,
                translation: [pose.translation.x, pose.translation.y, pose.translation.z],
                rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
            };
        }
        result.status = status;
        result.solve.clone_from(&solve.diagnostics);
        Ok(result)
    }

    fn fill_velocity_bounds(&self, state: &RobotState, horizon: f64, bounds: &mut VelocityBounds) {
        bounds.lower.fill(f64::NEG_INFINITY);
        bounds.upper.fill(f64::INFINITY);
        for joint in &self.model.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let velocity = joint.limit.velocity.abs();
            // Leave enough terminal headroom for a subsequent 20 ms braking
            // segment. The acceleration/jerk caps are conservative closed-form
            // bounds for the quintic splice; the exact candidate is still
            // validated after solving.
            let braking_velocity = velocity
                .min(self.config.max_acceleration * horizon / 4.0)
                .min(self.config.max_jerk * horizon * horizon / 20.0);
            bounds.lower[index] = -braking_velocity;
            bounds.upper[index] = braking_velocity;
            if joint.limit.lower.is_finite() {
                let headroom = (state.q[index] - joint.limit.lower).max(0.0);
                bounds.lower[index] = bounds.lower[index].max(-headroom / (2.0 * horizon));
            }
            if joint.limit.upper.is_finite() {
                let headroom = (joint.limit.upper - state.q[index]).max(0.0);
                bounds.upper[index] = bounds.upper[index].min(headroom / (2.0 * horizon));
            }
        }
    }

    fn fill_segment_limits(&self, limits: &mut Vec<SegmentLimits>) {
        limits.clear();
        limits.resize(
            self.model.dof,
            SegmentLimits {
                min_position: f64::NEG_INFINITY,
                max_position: f64::INFINITY,
                max_velocity: f64::INFINITY,
                max_acceleration: self.config.max_acceleration,
                max_jerk: self.config.max_jerk,
            },
        );
        for joint in &self.model.joints {
            if let Some(index) = joint.coordinate {
                limits[index].min_position = joint.limit.lower;
                limits[index].max_position = joint.limit.upper;
                // A quintic splice may have a small internal overshoot while
                // reconciling observed and commanded velocity.
                limits[index].max_velocity = joint.limit.velocity.abs() * 2.0;
            }
        }
    }
}

fn bandwidth_gain(hz: f64) -> f64 {
    (2.0 * std::f64::consts::PI * hz.max(0.0)).min(100.0)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, RowDVector};

    use super::*;
    use crate::{
        TimingSpec,
        collision::CompiledCollisionModel,
        model::{BodyId, FrameId, JointId, JointKind, JointLimit, JointSpec, RigidBodySpec},
        rig::{CompiledTaskProgram, TaskSpec},
        signal::{RotationJet, SignalOp, SignalOutputSpec, VectorJet},
        urdf::load_urdf,
    };

    fn arm() -> CompiledModel {
        let inertia = Matrix3::identity() * 0.01;
        CompiledModel::new(
            "arm".into(),
            vec![
                RigidBodySpec {
                    id: BodyId(0),
                    name: "base".into(),
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
                    name: "hand".into(),
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
                name: "joint".into(),
                kind: JointKind::Revolute,
                parent: BodyId(0),
                child: BodyId(1),
                parent_from_joint: nalgebra::Isometry3::translation(1.0, 0.0, 0.0),
                axis_in_joint: Vec3::z(),
                coordinate: None,
                limit: JointLimit {
                    lower: -2.0,
                    upper: 2.0,
                    velocity: 10.0,
                    effort: 10.0,
                },
            }],
        )
        .unwrap()
    }

    #[test]
    fn controller_reduces_reachable_point_error() {
        let controller = Controller::new(arm(), ControllerConfig::default()).unwrap();
        let state = RobotState::zeros(controller.model());
        let output = controller
            .advance(ControllerInput {
                tick_time_ns: 0,
                state,
                signal_inputs: SignalInputFrame::default(),
                frame_targets: vec![FrameTarget {
                    stable_id: 1,
                    frame: FrameId(1),
                    point_in_frame: Vec3::new(1.0, 0.0, 0.0),
                    target_position_world: Vec3::new(1.8, 0.3, 0.0),
                    target_orientation_world: None,
                    feedforward_linear_velocity_world: Vec3::zeros(),
                    priority: Priority::Intent,
                    weight: 1.0,
                }],
                com_target_world: None,
                posture_target: None,
                hard_constraints: vec![],
            })
            .unwrap();
        assert!(output.commanded_velocity[0] > 0.0);
        assert_eq!(output.samples.len(), 20);
    }

    #[test]
    fn trajectory_backtracking_admits_bounded_motion_instead_of_latching_brake() {
        let config = ControllerConfig {
            max_acceleration: 50.0,
            max_jerk: 10_000.0,
            ..ControllerConfig::default()
        };
        let controller = Controller::new(arm(), config.clone()).unwrap();
        let state = RobotState::zeros(controller.model());
        let output = controller
            .advance(ControllerInput {
                tick_time_ns: 0,
                state,
                signal_inputs: SignalInputFrame::default(),
                frame_targets: vec![FrameTarget {
                    stable_id: 1,
                    frame: FrameId(1),
                    point_in_frame: Vec3::new(1.0, 0.0, 0.0),
                    target_position_world: Vec3::new(0.0, 2.0, 0.0),
                    target_orientation_world: None,
                    feedforward_linear_velocity_world: Vec3::zeros(),
                    priority: Priority::Intent,
                    weight: 1.0,
                }],
                com_target_world: None,
                posture_target: None,
                hard_constraints: vec![],
            })
            .unwrap();
        assert_eq!(output.status, StepStatus::Degraded);
        assert!(output.commanded_velocity[0] > 0.0);
        assert!(output.commanded_velocity[0] < 1.0);
        assert!(output.segment_extrema.max_abs_acceleration[0] <= config.max_acceleration + 1e-9);
        assert!(output.segment_extrema.max_abs_jerk[0] <= config.max_jerk + 1e-9);
    }

    #[test]
    fn repeated_unreachable_target_respects_joint_limit_and_retreats() {
        let controller = Controller::new(arm(), ControllerConfig::default()).unwrap();
        let mut state = RobotState::zeros(controller.model());
        state.q[0] = 1.9;
        let point = Vec3::new(1.0, 0.0, 0.0);
        let outward = Vec3::new(-10.0, -3.0, 0.0);
        let retreat = Vec3::new(12.0, 0.0, 0.0);
        let mut maximum = state.q[0];
        for tick in 0..100 {
            let output = controller
                .advance(ControllerInput {
                    tick_time_ns: tick * 20_000_000,
                    state,
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![FrameTarget {
                        stable_id: 1,
                        frame: FrameId(1),
                        point_in_frame: point,
                        target_position_world: outward,
                        target_orientation_world: None,
                        feedforward_linear_velocity_world: Vec3::zeros(),
                        priority: Priority::Intent,
                        weight: 1.0,
                    }],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                })
                .unwrap();
            assert!(output.next_state.q[0] <= 2.0 + 1e-12);
            assert!(output.next_state.q[0] >= -2.0 - 1e-12);
            maximum = maximum.max(output.next_state.q[0]);
            state = output.next_state;
        }
        let before_retreat = state.q[0];
        for tick in 100..120 {
            let output = controller
                .advance(ControllerInput {
                    tick_time_ns: tick * 20_000_000,
                    state,
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![FrameTarget {
                        stable_id: 1,
                        frame: FrameId(1),
                        point_in_frame: point,
                        target_position_world: retreat,
                        target_orientation_world: None,
                        feedforward_linear_velocity_world: Vec3::zeros(),
                        priority: Priority::Intent,
                        weight: 1.0,
                    }],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                })
                .unwrap();
            assert!(output.next_state.q[0] <= 2.0 + 1e-12);
            assert!(output.next_state.q[0] >= -2.0 - 1e-12);
            state = output.next_state;
        }
        assert!(maximum > 1.9);
        assert!(state.q[0] < before_retreat - 1e-3);
    }

    #[test]
    fn controller_enforces_supplied_hard_velocity_row() {
        let controller = Controller::new(arm(), ControllerConfig::default()).unwrap();
        let state = RobotState::zeros(controller.model());
        let output = controller
            .advance(ControllerInput {
                tick_time_ns: 0,
                state,
                signal_inputs: SignalInputFrame::default(),
                frame_targets: vec![FrameTarget {
                    stable_id: 1,
                    frame: FrameId(1),
                    point_in_frame: Vec3::new(1.0, 0.0, 0.0),
                    target_position_world: Vec3::new(1.8, 0.3, 0.0),
                    target_orientation_world: None,
                    feedforward_linear_velocity_world: Vec3::zeros(),
                    priority: Priority::Intent,
                    weight: 1.0,
                }],
                com_target_world: None,
                posture_target: None,
                hard_constraints: vec![LinearConstraint {
                    stable_id: 8,
                    coefficients: RowDVector::from_row_slice(&[1.0]),
                    lower: f64::NEG_INFINITY,
                    upper: 0.0,
                }],
            })
            .unwrap();
        assert!(output.commanded_velocity[0] <= 1e-12);
        assert!(output.solve.maximum_constraint_violation <= 1e-12);
    }

    #[test]
    fn controller_collision_barrier_limits_approach_velocity() {
        let source = r#"
        <robot name="controlled_avoidance">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".2"/></geometry></collision></link>
          <link name="spacer"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".3"/></geometry></collision></link>
          <joint name="turn" type="revolute"><parent link="base"/><child link="spacer"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="2" effort="5"/></joint>
          <joint name="slide" type="prismatic"><parent link="spacer"/><child link="slider"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/><limit lower="-.4" upper=".4" velocity="2" effort="5"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let collision = CompiledCollisionModel::compile(&model);
        assert_eq!(collision.self_pairs.len(), 1);
        let collision_config = CollisionAvoidanceConfig {
            hard_margin: 0.4,
            influence_margin: 0.6,
            separation_gain: 2.0,
            maximum_separation_speed: 10.0,
            soft_weight: 0.0,
            ..CollisionAvoidanceConfig::default()
        };
        let controller = Controller::new(
            model.clone(),
            ControllerConfig {
                collision_avoidance: Some(collision_config),
                ..ControllerConfig::default()
            },
        )
        .unwrap();
        let state = RobotState::zeros(&model);
        let output = controller
            .advance(ControllerInput {
                tick_time_ns: 0,
                state: state.clone(),
                signal_inputs: SignalInputFrame::default(),
                frame_targets: vec![FrameTarget {
                    stable_id: 1,
                    frame: model.frame_id("slider").unwrap(),
                    point_in_frame: Vec3::zeros(),
                    target_position_world: Vec3::new(0.5, 0.0, 0.0),
                    target_orientation_world: None,
                    feedforward_linear_velocity_world: Vec3::zeros(),
                    priority: Priority::Intent,
                    weight: 1.0,
                }],
                com_target_world: None,
                posture_target: None,
                hard_constraints: vec![],
            })
            .unwrap();
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let distance = collision
            .evaluate_pair(&model, &cache, &state.v, collision.self_pairs[0])
            .unwrap();
        let achieved_rate = distance
            .jacobian_row
            .iter()
            .zip(output.commanded_velocity.iter())
            .map(|(coefficient, velocity)| coefficient * velocity)
            .sum::<f64>();
        let required_rate = -collision_config.separation_gain
            * (distance.signed_distance - collision_config.hard_margin);
        assert!(achieved_rate >= required_rate - 1e-10);
        assert!(
            output.solve.active_constraints.contains(
                &(collision_config.hard_stable_id_base + collision.self_pairs[0].stable_id)
            )
        );
    }

    #[test]
    fn explicit_transition_splices_from_commanded_not_lagging_observation() {
        let controller = Controller::new(arm(), ControllerConfig::default()).unwrap();
        let initial = RobotState::zeros(controller.model());
        let mut state_a = ControllerState::new(initial.clone(), 0);
        let mut state_b = state_a.clone();
        let mut state_c = state_a.clone();
        let mut scratch = ControllerScratch::new(controller.model(), 4);
        let mut output = ControllerOutputBuffer::default();
        let target = || FrameTarget {
            stable_id: 1,
            frame: FrameId(1),
            point_in_frame: Vec3::new(1.0, 0.0, 0.0),
            target_position_world: Vec3::new(1.95, 0.1, 0.0),
            target_orientation_world: None,
            feedforward_linear_velocity_world: Vec3::zeros(),
            priority: Priority::Intent,
            weight: 1.0,
        };

        controller
            .advance_into(
                ControllerInput {
                    tick_time_ns: 0,
                    state: initial.clone(),
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![target()],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                },
                &state_a,
                &mut state_b,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let first_commanded = state_b.commanded_state.q[0];
        assert!(first_commanded.abs() > 1e-6);

        // The observation deliberately remains at the original position.
        controller
            .advance_into(
                ControllerInput {
                    tick_time_ns: 20_000_000,
                    state: initial,
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![target()],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                },
                &state_b,
                &mut state_c,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let second = output.value.as_ref().unwrap();
        assert!((second.primary_segment.coefficients[0][0] - first_commanded).abs() < 1e-12);
        assert_eq!(state_c.tick_count, 2);
        assert_eq!(scratch.task_capacity(), 4);

        // Keep state_a mutable in this test to mirror caller-side double
        // buffering and prove no hidden controller mutation is required.
        state_a = state_c;
        assert_eq!(state_a.last_tick_time_ns, Some(20_000_000));
    }

    #[test]
    fn explicit_transition_rejects_program_epoch_mismatch() {
        let controller = Controller::new(arm(), ControllerConfig::default()).unwrap();
        let initial = RobotState::zeros(controller.model());
        let state_in = ControllerState::new(initial.clone(), 99);
        let mut state_out = state_in.clone();
        let mut scratch = ControllerScratch::new(controller.model(), 1);
        let mut output = ControllerOutputBuffer::default();
        let status = controller
            .advance_into(
                ControllerInput {
                    tick_time_ns: 0,
                    state: initial,
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                },
                &state_in,
                &mut state_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(status, StepStatus::Rejected);
        assert!(output.value.is_none());
        assert_eq!(state_out.tick_count, 0);
    }

    #[test]
    fn compiled_signal_output_drives_point_task_with_explicit_memory() {
        let source = r#"
        <robot name="compiled_signal_arm">
          <link name="base"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <link name="hand"><inertial><mass value="1"/><origin xyz=".5 0 0"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <joint name="shoulder" type="revolute">
            <parent link="base"/><child link="hand"/><origin xyz="1 0 0"/>
            <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="10" effort="10"/>
          </joint>
        </robot>"#;
        let base = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputVector {
                    stable_id: 10,
                    input: 0,
                },
                SignalOp::LowPass {
                    stable_id: 11,
                    source: 0,
                    bandwidth_hz: 8.0,
                },
                SignalOp::InputRotation {
                    stable_id: 12,
                    input: 0,
                },
                SignalOp::CriticallyDampedSpring {
                    stable_id: 13,
                    source: 2,
                    bandwidth_hz: 6.0,
                },
            ],
            vec![
                SignalOutputSpec {
                    stable_id: 100,
                    node: 1,
                },
                SignalOutputSpec {
                    stable_id: 101,
                    node: 3,
                },
            ],
        )
        .unwrap();
        let with_signals = base.with_signals(signals).unwrap();
        let tasks = CompiledTaskProgram::compile(
            &with_signals.model,
            &with_signals.signals,
            vec![
                TaskSpec::Point {
                    stable_id: 200,
                    frame: with_signals.model.frame_id("hand").unwrap(),
                    point_in_frame: Vec3::new(1.0, 0.0, 0.0),
                    target_signal: 100,
                    priority: Priority::Intent,
                    weight: 1.0,
                    bandwidth_hz: 2.5,
                },
                TaskSpec::Orientation {
                    stable_id: 201,
                    frame: with_signals.model.frame_id("hand").unwrap(),
                    target_signal: 101,
                    priority: Priority::Preference,
                    weight: 0.5,
                    bandwidth_hz: 2.0,
                },
            ],
        )
        .unwrap();
        let program = with_signals.with_tasks(tasks).unwrap();
        let controller = Controller::from_program(&program).unwrap();
        let initial = RobotState::zeros(&program.model);
        let state_in = ControllerState::for_program(initial.clone(), &program);
        let mut state_out = ControllerState::for_program(initial.clone(), &program);
        let mut scratch = ControllerScratch::for_program(&program, 0);
        let mut output = ControllerOutputBuffer::with_layout(
            &program.model,
            program.timing.control_horizon_ns,
            program.timing.sample_period_ns,
        );
        let mut signal_inputs = SignalInputFrame::with_full_layout(0, 1, 1);
        signal_inputs.vectors[0] = VectorJet {
            value: Vec3::new(1.8, 0.3, 0.0),
            velocity: Vec3::new(0.0, 0.05, 0.0),
            acceleration: Vec3::zeros(),
        };
        signal_inputs.rotations[0] = RotationJet {
            value: UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.0, 0.2)),
            angular_velocity_world: Vec3::new(0.0, 0.0, 0.05),
            angular_acceleration_world: Vec3::zeros(),
        };

        let status = controller
            .advance_into_reusable(
                &ControllerInput {
                    tick_time_ns: 0,
                    state: initial,
                    signal_inputs,
                    frame_targets: vec![],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                },
                &state_in,
                &mut state_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();

        assert!(matches!(status, StepStatus::Ok | StepStatus::Degraded));
        assert!(output.value.as_ref().unwrap().commanded_velocity[0] > 0.0);
        assert!(!state_in.signal_memory.bitwise_eq(&state_out.signal_memory));
    }

    #[test]
    fn compiled_direction_task_aims_without_constraining_roll() {
        let source = r#"
        <robot name="direction_arm">
          <link name="base"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <link name="roll"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <link name="pitch"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <link name="aim"><inertial><mass value="1"/><inertia ixx=".1" ixy="0" ixz="0" iyy=".1" iyz="0" izz=".1"/></inertial></link>
          <joint name="roll_joint" type="revolute">
            <parent link="base"/><child link="roll"/><axis xyz="1 0 0"/>
            <limit lower="-3" upper="3" velocity="10" effort="10"/>
          </joint>
          <joint name="pitch_joint" type="revolute">
            <parent link="roll"/><child link="pitch"/><axis xyz="0 1 0"/>
            <limit lower="-3" upper="3" velocity="10" effort="10"/>
          </joint>
          <joint name="yaw_joint" type="revolute">
            <parent link="pitch"/><child link="aim"/><axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" velocity="10" effort="10"/>
          </joint>
        </robot>"#;
        let base = MotionProgram::compile_urdf(source, TimingSpec::default(), 8).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![SignalOp::ConstantVector {
                stable_id: 10,
                jet: VectorJet {
                    value: Vec3::y(),
                    ..Default::default()
                },
            }],
            vec![SignalOutputSpec {
                stable_id: 100,
                node: 0,
            }],
        )
        .unwrap();
        let with_signals = base.with_signals(signals).unwrap();
        let tasks = CompiledTaskProgram::compile(
            &with_signals.model,
            &with_signals.signals,
            vec![TaskSpec::DirectionAim {
                stable_id: 200,
                frame: with_signals.model.frame_id("aim").unwrap(),
                controlled_axis_in_frame: Vec3::x(),
                target_signal: 100,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 2.0,
            }],
        )
        .unwrap();
        let program = with_signals.with_tasks(tasks).unwrap();
        let controller = Controller::from_program(&program).unwrap();
        let initial = RobotState::zeros(&program.model);
        let state_in = ControllerState::for_program(initial.clone(), &program);
        let mut state_out = state_in.clone();
        let mut scratch = ControllerScratch::for_program(&program, 0);
        let mut output = ControllerOutputBuffer::with_layout(
            &program.model,
            program.timing.control_horizon_ns,
            program.timing.sample_period_ns,
        );

        let status = controller
            .advance_into_reusable(
                &ControllerInput {
                    tick_time_ns: 0,
                    state: initial,
                    signal_inputs: SignalInputFrame::default(),
                    frame_targets: vec![],
                    com_target_world: None,
                    posture_target: None,
                    hard_constraints: vec![],
                },
                &state_in,
                &mut state_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();

        assert!(matches!(status, StepStatus::Ok | StepStatus::Degraded));
        let task = &scratch.tasks.tasks()[scratch.tasks.active_indices()[0]];
        assert_eq!(task.kind, TaskKind::Gaze);
        assert!(task.jacobian.column(0).norm() <= 1e-12);
        assert!(task.jacobian.column(1).norm() > 0.99);
        assert!(task.jacobian.column(2).norm() > 0.99);
        assert!(task.jacobian.column(1).dot(&task.jacobian.column(2)).abs() <= 1e-12);
        let commanded_velocity = &output.value.as_ref().unwrap().commanded_velocity;
        assert!(commanded_velocity[0].abs() <= 1e-12);
        assert!(commanded_velocity[2] > 0.0);
    }
}
