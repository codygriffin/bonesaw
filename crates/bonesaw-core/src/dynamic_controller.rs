//! Explicit floating-WBC to actuator-trajectory transaction.
//!
//! This module deliberately stops at command synthesis. It does not integrate
//! the floating robot state or imply a plant, policy, estimator, or contact
//! response. Observed state owns the physical WBC query; caller-owned command
//! state owns the exact trajectory splice.

use nalgebra::DVector;
use thiserror::Error;

use crate::{
    actuation::{ActuationError, CompiledActuation},
    collision::{
        CollisionError, CollisionEvaluationScratch, CollisionSweepReport, CompiledCollisionModel,
        DistanceSample,
    },
    dynamic_wbc::{
        DynamicWbcConfig, DynamicWbcError, FloatingDynamicWbc, FloatingDynamicWbcInput,
        FloatingDynamicWbcOutput, FloatingDynamicWbcScratch,
    },
    history::{
        RobotObservationErrorBound, RobotObservationEvidence, RobotObservationLimits,
        RobotObservationStamp, evaluate_robot_observation,
    },
    math::{ControlTime, Motion6, SpatialAcceleration6},
    model::{ModelCache, RobotState},
    program::MotionProgram,
    solver::SolveStatus,
    trajectory::{
        ActuatorSampleBlock, QuinticSegment, RootPosePredictionSegment, RootPredictionErrorGrowth,
        SegmentExtrema, SegmentLimits, TrajectoryError,
    },
    world_collision::{
        CompiledWorldCollisionModel, SdfOutsidePolicy, WorldCollisionContinuityReport,
        WorldCollisionError, WorldCollisionSweepReport, WorldCollisionSweepScratch,
        WorldSceneEvidence,
    },
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DynamicPlanSelection {
    Primary,
    Contingency,
    Rejected,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DynamicStepStatus {
    Ok,
    Degraded,
    Contingency,
    Rejected,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CommandTrackingAction {
    #[default]
    Nominal,
    Contingency,
    Rejected,
}

/// Two-stage observed-versus-commanded tracking envelope. Crossing a
/// contingency threshold withholds the next feed-forward candidate and asks
/// the independently validated brake to decelerate the commanded trajectory.
/// Crossing a reject threshold withholds both plans because neither command
/// segment is an adequate witness for the observed mechanism state.
#[derive(Clone, Copy, Debug, Default)]
pub struct CommandTrackingLimits {
    pub position_contingency: Option<f64>,
    pub position_reject: Option<f64>,
    pub velocity_contingency: Option<f64>,
    pub velocity_reject: Option<f64>,
}

impl CommandTrackingLimits {
    fn validate(self) -> bool {
        let limits = [
            self.position_contingency,
            self.position_reject,
            self.velocity_contingency,
            self.velocity_reject,
        ];
        limits
            .into_iter()
            .flatten()
            .all(|limit| limit.is_finite() && limit >= 0.0)
            && ordered_optional_pair(self.position_contingency, self.position_reject)
            && ordered_optional_pair(self.velocity_contingency, self.velocity_reject)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandTrackingEvidence {
    pub maximum_position_error: f64,
    pub maximum_velocity_error: f64,
    pub limiting_position_actuator: Option<usize>,
    pub limiting_velocity_actuator: Option<usize>,
    pub position_contingency_headroom: f64,
    pub position_reject_headroom: f64,
    pub velocity_contingency_headroom: f64,
    pub velocity_reject_headroom: f64,
    pub action: CommandTrackingAction,
}

impl Default for CommandTrackingEvidence {
    fn default() -> Self {
        Self {
            maximum_position_error: f64::INFINITY,
            maximum_velocity_error: f64::INFINITY,
            limiting_position_actuator: None,
            limiting_velocity_actuator: None,
            position_contingency_headroom: f64::NEG_INFINITY,
            position_reject_headroom: f64::NEG_INFINITY,
            velocity_contingency_headroom: f64::NEG_INFINITY,
            velocity_reject_headroom: f64::NEG_INFINITY,
            action: CommandTrackingAction::Rejected,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum UnknownCollisionGeometryPolicy {
    /// Collision-enabled construction fails if any authored shape lacks a
    /// high-rate proxy.
    #[default]
    RejectProgram,
    /// The caller knowingly limits admission to the represented subset.
    IgnoreExplicitly,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CollisionContinuityPolicy {
    /// Report and admit from the deterministic grid only.
    #[default]
    GridOnly,
    /// Require sampled clearance minus a conservative half-interval relative
    /// center-travel guard to remain above the configured clearance.
    ConservativeRateBound,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DynamicTrajectoryValidationConfig {
    /// `None` disables self-collision admission explicitly. A finite value
    /// enables deterministic dense-grid validation of both candidate segments.
    pub self_collision_clearance: Option<f64>,
    pub unknown_collision_geometry: UnknownCollisionGeometryPolicy,
    pub collision_continuity: CollisionContinuityPolicy,
    /// Maximum pair/interval midpoint-refinement depth after the base servo
    /// grid. Zero preserves the single-rate pairwise certificate.
    pub collision_max_subdivision_depth: u8,
    /// `None` disables world-segment admission explicitly. Enabling it requires
    /// a compiled immutable world field at controller construction.
    pub world_collision_clearance: Option<f64>,
    pub world_collision_continuity: CollisionContinuityPolicy,
    pub world_collision_max_subdivision_depth: u8,
    /// Deterministic estimator/model error-growth contract applied to both
    /// primary and braking floating-root collision predictions.
    pub root_prediction_error_growth: RootPredictionErrorGrowth,
    /// Optional epoch expected by this controller instance. A mismatch is
    /// typed scene evidence and withholds both primary and brake.
    pub expected_world_scene_epoch: Option<u64>,
    /// Maximum allowed age from the scene source timestamp at the tick.
    pub maximum_world_scene_age_ns: Option<i64>,
    /// Require the immutable snapshot validity window to cover the complete
    /// command horizon rather than only the current tick.
    pub require_world_scene_horizon_validity: bool,
    /// Optional observed-versus-commanded actuator tracking envelope. The
    /// default has no thresholds but still reports exact divergence evidence.
    pub command_tracking_limits: CommandTrackingLimits,
    /// Robot-state timestamp and clock-synchronization admission. The shell
    /// owns clock mapping; the core only checks the mapped evidence.
    pub robot_observation_limits: RobotObservationLimits,
}

impl DynamicTrajectoryValidationConfig {
    fn validate(self) -> bool {
        self.self_collision_clearance
            .is_none_or(|clearance| clearance.is_finite())
            && self.collision_max_subdivision_depth <= 12
            && self
                .world_collision_clearance
                .is_none_or(|clearance| clearance.is_finite())
            && self.world_collision_max_subdivision_depth <= 12
            && self.root_prediction_error_growth.is_valid()
            && self
                .maximum_world_scene_age_ns
                .is_none_or(|maximum_age_ns| maximum_age_ns >= 0)
            && self.command_tracking_limits.validate()
            && self.robot_observation_limits.validate()
    }
}

/// Fixed, composable admission evidence. Multiple causes remain visible; the
/// controller never collapses solver, expiry, derivative, and position-limit
/// failures into one health score.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DynamicAdmissionFlags(u32);

impl DynamicAdmissionFlags {
    pub const SOLVED_WITH_SLACK: u32 = 1 << 0;
    pub const SOLVER_NOT_ADMITTED: u32 = 1 << 1;
    pub const PREVIOUS_PLAN_EXPIRED: u32 = 1 << 2;
    pub const PRIMARY_ACTUATOR_LIMIT: u32 = 1 << 3;
    pub const PRIMARY_JOINT_POSITION_LIMIT: u32 = 1 << 4;
    pub const CONTINGENCY_ACTUATOR_LIMIT: u32 = 1 << 5;
    pub const CONTINGENCY_JOINT_POSITION_LIMIT: u32 = 1 << 6;
    pub const PRIMARY_COLLISION: u32 = 1 << 7;
    pub const CONTINGENCY_COLLISION: u32 = 1 << 8;
    pub const PRIMARY_CONTINUOUS_CLEARANCE: u32 = 1 << 9;
    pub const CONTINGENCY_CONTINUOUS_CLEARANCE: u32 = 1 << 10;
    pub const PRIMARY_WORLD_COLLISION: u32 = 1 << 11;
    pub const CONTINGENCY_WORLD_COLLISION: u32 = 1 << 12;
    pub const PRIMARY_WORLD_CONTINUOUS_CLEARANCE: u32 = 1 << 13;
    pub const CONTINGENCY_WORLD_CONTINUOUS_CLEARANCE: u32 = 1 << 14;
    pub const PRIMARY_WORLD_UNKNOWN: u32 = 1 << 15;
    pub const CONTINGENCY_WORLD_UNKNOWN: u32 = 1 << 16;
    pub const WORLD_SCENE_INVALID: u32 = 1 << 17;
    pub const COMMAND_TRACKING_CONTINGENCY: u32 = 1 << 18;
    pub const COMMAND_TRACKING_REJECTED: u32 = 1 << 19;
    pub const ROBOT_OBSERVATION_FUTURE: u32 = 1 << 20;
    pub const ROBOT_OBSERVATION_STALE: u32 = 1 << 21;
    pub const ROBOT_OBSERVATION_UNCERTAIN: u32 = 1 << 22;
    pub const PRIMARY_COLLISION_OBSERVATION_ERROR: u32 = 1 << 23;
    pub const CONTINGENCY_COLLISION_OBSERVATION_ERROR: u32 = 1 << 24;
    pub const PRIMARY_WORLD_OBSERVATION_ERROR: u32 = 1 << 25;
    pub const CONTINGENCY_WORLD_OBSERVATION_ERROR: u32 = 1 << 26;

    pub fn bits(self) -> u32 {
        self.0
    }

    pub fn contains(self, flag: u32) -> bool {
        self.0 & flag != 0
    }

    fn insert(&mut self, flag: u32) {
        self.0 |= flag;
    }
}

#[derive(Clone, Copy)]
pub struct FloatingDynamicControllerInput<'a> {
    pub tick_time_ns: ControlTime,
    pub program_epoch: u64,
    pub observation_stamp: RobotObservationStamp,
    pub wbc: FloatingDynamicWbcInput<'a>,
}

#[derive(Clone, Copy)]
pub struct FloatingDynamicControllerSolvedInput<'a> {
    pub tick_time_ns: ControlTime,
    pub program_epoch: u64,
    pub observation_stamp: RobotObservationStamp,
    pub observed: &'a RobotState,
    /// Observed floating-root twist in the canonical world convention. It is
    /// required because an already-solved WBC output does not retain its input.
    pub root_twist_world: Motion6,
    pub solved_wbc: &'a FloatingDynamicWbcOutput,
}

/// Semantic command memory. All evolving command state is caller-owned and
/// double-bufferable; the controller itself is immutable.
#[derive(Debug)]
pub struct FloatingDynamicControllerState {
    pub commanded_position: DVector<f64>,
    pub commanded_velocity: DVector<f64>,
    pub commanded_acceleration: DVector<f64>,
    pub previous_command: QuinticSegment,
    pub has_previous_command: bool,
    pub last_tick_time_ns: Option<ControlTime>,
    pub tick_count: u64,
    pub program_epoch: u64,
}

impl Clone for FloatingDynamicControllerState {
    fn clone(&self) -> Self {
        let mut previous_command = QuinticSegment {
            start_time_ns: self.previous_command.start_time_ns,
            duration_ns: self.previous_command.duration_ns,
            coefficients: Vec::with_capacity(self.previous_command.coefficients.capacity()),
        };
        previous_command
            .coefficients
            .extend_from_slice(&self.previous_command.coefficients);
        Self {
            commanded_position: self.commanded_position.clone(),
            commanded_velocity: self.commanded_velocity.clone(),
            commanded_acceleration: self.commanded_acceleration.clone(),
            previous_command,
            has_previous_command: self.has_previous_command,
            last_tick_time_ns: self.last_tick_time_ns,
            tick_count: self.tick_count,
            program_epoch: self.program_epoch,
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.commanded_position
            .as_mut_slice()
            .copy_from_slice(source.commanded_position.as_slice());
        self.commanded_velocity
            .as_mut_slice()
            .copy_from_slice(source.commanded_velocity.as_slice());
        self.commanded_acceleration
            .as_mut_slice()
            .copy_from_slice(source.commanded_acceleration.as_slice());
        self.previous_command.clone_from(&source.previous_command);
        self.has_previous_command = source.has_previous_command;
        self.last_tick_time_ns = source.last_tick_time_ns;
        self.tick_count = source.tick_count;
        self.program_epoch = source.program_epoch;
    }
}

impl FloatingDynamicControllerState {
    pub fn for_program(
        observed: &RobotState,
        program: &MotionProgram,
    ) -> Result<Self, DynamicControllerError> {
        observed.validate(&program.model)?;
        let actuator_count = program.actuation.actuators.len();
        let mut commanded_position = DVector::zeros(actuator_count);
        let mut commanded_velocity = DVector::zeros(actuator_count);
        program
            .actuation
            .write_actuator_velocity(&observed.q, &mut commanded_position)?;
        program
            .actuation
            .write_actuator_velocity(&observed.v, &mut commanded_velocity)?;
        Ok(Self {
            commanded_position,
            commanded_velocity,
            commanded_acceleration: DVector::zeros(actuator_count),
            previous_command: segment_workspace(actuator_count),
            has_previous_command: false,
            last_tick_time_ns: None,
            tick_count: 0,
            program_epoch: program.header.program_epoch,
        })
    }
}

#[derive(Debug)]
pub struct FloatingDynamicControllerOutput {
    pub status: DynamicStepStatus,
    pub selection: DynamicPlanSelection,
    pub admission_flags: DynamicAdmissionFlags,
    pub world_scene: WorldSceneEvidence,
    pub robot_observation: RobotObservationEvidence,
    pub command_tracking: CommandTrackingEvidence,
    /// Reconstruction error applied to command geometry. It is distinct from
    /// observed↔commanded tracking and floating-root forecast growth.
    pub observation_error: RobotObservationErrorBound,
    pub wbc: FloatingDynamicWbcOutput,
    /// Feed-forward effort is executable only when the primary plan is
    /// selected. It is exactly zero for contingency or rejected output.
    pub admitted_actuator_effort: DVector<f64>,
    /// The finite WBC effort candidate, retained separately for diagnostics.
    pub candidate_actuator_effort: DVector<f64>,
    /// Exact commanded boundary used to synthesize both new segments.
    pub splice_position: DVector<f64>,
    pub splice_velocity: DVector<f64>,
    pub splice_acceleration: DVector<f64>,
    pub primary_segment: QuinticSegment,
    pub contingency_segment: QuinticSegment,
    /// State-local floating-root prediction paired with the primary actuator
    /// segment. This is a collision witness, not an executable base command.
    pub primary_root_prediction: RootPosePredictionSegment,
    /// Independently braking/coasting root witness paired with contingency.
    pub contingency_root_prediction: RootPosePredictionSegment,
    pub samples: ActuatorSampleBlock,
    pub primary_extrema: SegmentExtrema,
    pub contingency_extrema: SegmentExtrema,
    /// Analytic extrema after mapping the actuator polynomial back through the
    /// exact generalized-from-actuator transmission.
    pub primary_joint_extrema: SegmentExtrema,
    pub contingency_joint_extrema: SegmentExtrema,
    pub primary_minimum_joint_position_headroom: f64,
    pub contingency_minimum_joint_position_headroom: f64,
    pub primary_valid: bool,
    pub contingency_valid: bool,
    pub primary_actuator_valid: bool,
    pub primary_joint_position_valid: bool,
    pub contingency_actuator_valid: bool,
    pub contingency_joint_position_valid: bool,
    pub primary_collision_valid: bool,
    pub contingency_collision_valid: bool,
    pub primary_collision: CollisionSweepReport,
    pub contingency_collision: CollisionSweepReport,
    pub primary_continuous_clearance_lower_bound: f64,
    pub contingency_continuous_clearance_lower_bound: f64,
    pub primary_robust_collision_minimum_clearance_m: f64,
    pub contingency_robust_collision_minimum_clearance_m: f64,
    pub primary_robust_continuous_clearance_lower_bound_m: f64,
    pub contingency_robust_continuous_clearance_lower_bound_m: f64,
    pub primary_continuous_limiting_pair_id: Option<u32>,
    pub contingency_continuous_limiting_pair_id: Option<u32>,
    pub primary_maximum_relative_speed_bound: f64,
    pub contingency_maximum_relative_speed_bound: f64,
    pub primary_continuity_leaf_interval_count: usize,
    pub contingency_continuity_leaf_interval_count: usize,
    pub primary_collision_refinement_pair_samples: usize,
    pub contingency_collision_refinement_pair_samples: usize,
    pub primary_continuity_unresolved_interval_count: usize,
    pub contingency_continuity_unresolved_interval_count: usize,
    pub primary_continuity_maximum_subdivision_depth: u8,
    pub contingency_continuity_maximum_subdivision_depth: u8,
    pub primary_world_collision_valid: bool,
    pub contingency_world_collision_valid: bool,
    pub primary_world_collision: WorldCollisionSweepReport,
    pub contingency_world_collision: WorldCollisionSweepReport,
    pub primary_world_continuity: WorldCollisionContinuityReport,
    pub contingency_world_continuity: WorldCollisionContinuityReport,
    pub primary_robust_world_minimum_clearance_m: f64,
    pub contingency_robust_world_minimum_clearance_m: f64,
    pub primary_robust_world_continuous_clearance_lower_bound_m: f64,
    pub contingency_robust_world_continuous_clearance_lower_bound_m: f64,
    pub previous_plan_expired: bool,
    pub maximum_observed_command_position_divergence: f64,
}

impl FloatingDynamicControllerOutput {
    pub fn workspace(
        program: &MotionProgram,
        maximum_contacts: usize,
    ) -> Result<Self, DynamicControllerError> {
        Self::workspace_with_world_probe_capacity(program, maximum_contacts, 0)
    }

    pub fn workspace_with_world_collision(
        program: &MotionProgram,
        maximum_contacts: usize,
        world_collision: &CompiledWorldCollisionModel,
    ) -> Result<Self, DynamicControllerError> {
        Self::workspace_with_world_probe_capacity(
            program,
            maximum_contacts,
            world_collision.probes.len(),
        )
    }

    fn workspace_with_world_probe_capacity(
        program: &MotionProgram,
        maximum_contacts: usize,
        world_probe_capacity: usize,
    ) -> Result<Self, DynamicControllerError> {
        validate_timing(program)?;
        let dof = program.model.dof;
        let actuators = program.actuation.actuators.len();
        let maximum_constraints = dof
            .saturating_add(6)
            .saturating_add(maximum_contacts.saturating_mul(8))
            .saturating_add(actuators)
            .saturating_add(
                CompiledCollisionModel::compile(&program.model)
                    .validation_pairs
                    .len(),
            )
            .saturating_add(world_probe_capacity);
        let sample_count =
            (program.timing.control_horizon_ns / program.timing.sample_period_ns) as usize;
        Ok(Self {
            status: DynamicStepStatus::Rejected,
            selection: DynamicPlanSelection::Rejected,
            admission_flags: DynamicAdmissionFlags::default(),
            world_scene: WorldSceneEvidence::timeless_valid(),
            robot_observation: RobotObservationEvidence::default(),
            command_tracking: CommandTrackingEvidence::default(),
            observation_error: RobotObservationErrorBound::default(),
            wbc: FloatingDynamicWbcOutput::workspace(dof, maximum_contacts, maximum_constraints),
            admitted_actuator_effort: DVector::zeros(actuators),
            candidate_actuator_effort: DVector::zeros(actuators),
            splice_position: DVector::zeros(actuators),
            splice_velocity: DVector::zeros(actuators),
            splice_acceleration: DVector::zeros(actuators),
            primary_segment: segment_workspace(actuators),
            contingency_segment: segment_workspace(actuators),
            primary_root_prediction: RootPosePredictionSegment::stationary(),
            contingency_root_prediction: RootPosePredictionSegment::stationary(),
            samples: ActuatorSampleBlock::with_capacity(sample_count, actuators),
            primary_extrema: extrema_workspace(actuators),
            contingency_extrema: extrema_workspace(actuators),
            primary_joint_extrema: extrema_workspace(dof),
            contingency_joint_extrema: extrema_workspace(dof),
            primary_minimum_joint_position_headroom: f64::NEG_INFINITY,
            contingency_minimum_joint_position_headroom: f64::NEG_INFINITY,
            primary_valid: false,
            contingency_valid: false,
            primary_actuator_valid: false,
            primary_joint_position_valid: false,
            contingency_actuator_valid: false,
            contingency_joint_position_valid: false,
            primary_collision_valid: false,
            contingency_collision_valid: false,
            primary_collision: empty_collision_report(),
            contingency_collision: empty_collision_report(),
            primary_continuous_clearance_lower_bound: f64::NEG_INFINITY,
            contingency_continuous_clearance_lower_bound: f64::NEG_INFINITY,
            primary_robust_collision_minimum_clearance_m: f64::NEG_INFINITY,
            contingency_robust_collision_minimum_clearance_m: f64::NEG_INFINITY,
            primary_robust_continuous_clearance_lower_bound_m: f64::NEG_INFINITY,
            contingency_robust_continuous_clearance_lower_bound_m: f64::NEG_INFINITY,
            primary_continuous_limiting_pair_id: None,
            contingency_continuous_limiting_pair_id: None,
            primary_maximum_relative_speed_bound: f64::INFINITY,
            contingency_maximum_relative_speed_bound: f64::INFINITY,
            primary_continuity_leaf_interval_count: 0,
            contingency_continuity_leaf_interval_count: 0,
            primary_collision_refinement_pair_samples: 0,
            contingency_collision_refinement_pair_samples: 0,
            primary_continuity_unresolved_interval_count: 0,
            contingency_continuity_unresolved_interval_count: 0,
            primary_continuity_maximum_subdivision_depth: 0,
            contingency_continuity_maximum_subdivision_depth: 0,
            primary_world_collision_valid: false,
            contingency_world_collision_valid: false,
            primary_world_collision: WorldCollisionSweepReport::empty(SdfOutsidePolicy::Reject),
            contingency_world_collision: WorldCollisionSweepReport::empty(SdfOutsidePolicy::Reject),
            primary_world_continuity: WorldCollisionContinuityReport::empty(),
            contingency_world_continuity: WorldCollisionContinuityReport::empty(),
            primary_robust_world_minimum_clearance_m: f64::NEG_INFINITY,
            contingency_robust_world_minimum_clearance_m: f64::NEG_INFINITY,
            primary_robust_world_continuous_clearance_lower_bound_m: f64::NEG_INFINITY,
            contingency_robust_world_continuous_clearance_lower_bound_m: f64::NEG_INFINITY,
            previous_plan_expired: false,
            maximum_observed_command_position_divergence: f64::INFINITY,
        })
    }
}

/// Caller-owned non-semantic storage for the complete transaction.
#[derive(Clone, Debug)]
pub struct FloatingDynamicControllerScratch {
    pub wbc: FloatingDynamicWbcScratch,
    joint_acceleration: DVector<f64>,
    observed_actuator_position: DVector<f64>,
    observed_actuator_velocity: DVector<f64>,
    start_position: DVector<f64>,
    start_velocity: DVector<f64>,
    start_acceleration: DVector<f64>,
    primary_position: DVector<f64>,
    primary_velocity: DVector<f64>,
    primary_acceleration: DVector<f64>,
    contingency_position: DVector<f64>,
    zero: DVector<f64>,
    segment_limits: Vec<SegmentLimits>,
    primary_joint_segment: QuinticSegment,
    contingency_joint_segment: QuinticSegment,
    joint_segment_limits: Vec<SegmentLimits>,
    collision_state: RobotState,
    collision_acceleration: DVector<f64>,
    collision_model: ModelCache,
    collision_sample: DistanceSample,
    collision_evaluation: CollisionEvaluationScratch,
    world_collision_evaluation: WorldCollisionSweepScratch,
}

impl FloatingDynamicControllerScratch {
    pub fn for_program(program: &MotionProgram, maximum_contacts: usize) -> Self {
        let world_probe_capacity = CompiledCollisionModel::compile(&program.model)
            .spheres
            .len();
        Self::for_program_with_world_probe_capacity(program, maximum_contacts, world_probe_capacity)
    }

    pub fn for_program_with_world_collision(
        program: &MotionProgram,
        maximum_contacts: usize,
        world_collision: &CompiledWorldCollisionModel,
    ) -> Self {
        Self::for_program_with_world_probe_capacity(
            program,
            maximum_contacts,
            world_collision.probes.len(),
        )
    }

    fn for_program_with_world_probe_capacity(
        program: &MotionProgram,
        maximum_contacts: usize,
        world_probe_capacity: usize,
    ) -> Self {
        let dof = program.model.dof;
        let actuators = program.actuation.actuators.len();
        let mut joint_segment_limits = vec![SegmentLimits::default(); dof];
        for joint in &program.model.joints {
            if let Some(coordinate) = joint.coordinate {
                joint_segment_limits[coordinate].min_position = joint.limit.lower;
                joint_segment_limits[coordinate].max_position = joint.limit.upper;
            }
        }
        Self {
            wbc: FloatingDynamicWbcScratch::new_with_collision_capacities(
                &program.model,
                maximum_contacts,
                actuators,
                CompiledCollisionModel::compile(&program.model)
                    .validation_pairs
                    .len(),
                world_probe_capacity,
            ),
            joint_acceleration: DVector::zeros(dof),
            observed_actuator_position: DVector::zeros(actuators),
            observed_actuator_velocity: DVector::zeros(actuators),
            start_position: DVector::zeros(actuators),
            start_velocity: DVector::zeros(actuators),
            start_acceleration: DVector::zeros(actuators),
            primary_position: DVector::zeros(actuators),
            primary_velocity: DVector::zeros(actuators),
            primary_acceleration: DVector::zeros(actuators),
            contingency_position: DVector::zeros(actuators),
            zero: DVector::zeros(actuators),
            segment_limits: program
                .actuation
                .actuators
                .iter()
                .map(|actuator| SegmentLimits {
                    min_position: f64::NEG_INFINITY,
                    max_position: f64::INFINITY,
                    max_velocity: actuator.limits.velocity,
                    max_acceleration: actuator.limits.acceleration,
                    max_jerk: actuator.limits.jerk,
                })
                .collect(),
            primary_joint_segment: segment_workspace(dof),
            contingency_joint_segment: segment_workspace(dof),
            joint_segment_limits,
            collision_state: RobotState::zeros(&program.model),
            collision_acceleration: DVector::zeros(dof),
            collision_model: ModelCache::new(&program.model),
            collision_sample: DistanceSample::workspace(dof),
            collision_evaluation: {
                let collision = CompiledCollisionModel::compile(&program.model);
                let grid_samples = (program.timing.control_horizon_ns
                    / program.timing.sample_period_ns) as usize
                    + 1;
                CollisionEvaluationScratch::with_grid_capacity(
                    dof,
                    collision.validation_primitives.len(),
                    collision.validation_pairs.len(),
                    grid_samples,
                )
            },
            world_collision_evaluation: {
                let grid_samples = (program.timing.control_horizon_ns
                    / program.timing.sample_period_ns) as usize
                    + 1;
                WorldCollisionSweepScratch::with_grid_capacity(
                    dof,
                    world_probe_capacity,
                    grid_samples,
                )
            },
        }
    }
}

pub struct FloatingDynamicController {
    wbc: FloatingDynamicWbc,
    actuation: CompiledActuation,
    program_epoch: u64,
    control_horizon_ns: i64,
    sample_period_ns: i64,
    collision: CompiledCollisionModel,
    world_collision: Option<CompiledWorldCollisionModel>,
    validation: DynamicTrajectoryValidationConfig,
}

struct DynamicCollisionAdmission {
    report: CollisionSweepReport,
    continuous_clearance_lower_bound: f64,
    robust_minimum_clearance_m: f64,
    robust_continuous_clearance_lower_bound_m: f64,
    continuous_limiting_pair_id: Option<u32>,
    maximum_relative_speed_bound: f64,
    continuity_leaf_interval_count: usize,
    collision_refinement_pair_samples: usize,
    continuity_unresolved_interval_count: usize,
    continuity_maximum_subdivision_depth: u8,
    observation_error_caused_failure: bool,
    valid: bool,
}

struct DynamicWorldCollisionAdmission {
    report: WorldCollisionSweepReport,
    continuity: WorldCollisionContinuityReport,
    robust_minimum_clearance_m: f64,
    robust_continuous_clearance_lower_bound_m: f64,
    observation_error_caused_failure: bool,
    valid: bool,
}

impl FloatingDynamicController {
    pub fn from_program(
        program: &MotionProgram,
        config: DynamicWbcConfig,
    ) -> Result<Self, DynamicControllerError> {
        Self::from_program_with_validation(
            program,
            config,
            DynamicTrajectoryValidationConfig::default(),
        )
    }

    pub fn from_program_with_validation(
        program: &MotionProgram,
        config: DynamicWbcConfig,
        validation: DynamicTrajectoryValidationConfig,
    ) -> Result<Self, DynamicControllerError> {
        Self::from_program_with_optional_world_collision(program, config, validation, None)
    }

    pub fn from_program_with_world_collision(
        program: &MotionProgram,
        config: DynamicWbcConfig,
        validation: DynamicTrajectoryValidationConfig,
        world_collision: CompiledWorldCollisionModel,
    ) -> Result<Self, DynamicControllerError> {
        Self::from_program_with_optional_world_collision(
            program,
            config,
            validation,
            Some(world_collision),
        )
    }

    fn from_program_with_optional_world_collision(
        program: &MotionProgram,
        config: DynamicWbcConfig,
        validation: DynamicTrajectoryValidationConfig,
        world_collision: Option<CompiledWorldCollisionModel>,
    ) -> Result<Self, DynamicControllerError> {
        validate_timing(program)?;
        if !validation.validate() {
            return Err(DynamicControllerError::InvalidValidationConfig);
        }
        program.actuation.validate(&program.model)?;
        if program.actuation.actuator_from_generalized.is_none() {
            return Err(DynamicControllerError::UnsupportedActuation);
        }
        let collision = CompiledCollisionModel::compile(&program.model);
        if validation.self_collision_clearance.is_some()
            && collision.unsupported_shape_count > 0
            && validation.unknown_collision_geometry
                == UnknownCollisionGeometryPolicy::RejectProgram
        {
            return Err(DynamicControllerError::UnsupportedCollisionGeometry {
                count: collision.unsupported_shape_count,
            });
        }
        if validation.world_collision_clearance.is_some() && world_collision.is_none() {
            return Err(DynamicControllerError::MissingWorldCollisionModel);
        }
        if let Some(world) = world_collision.as_ref() {
            if world.speed_coordinate_count != program.model.dof
                || world.probe_speed_coefficients.len()
                    != world.probes.len().saturating_mul(program.model.dof)
            {
                return Err(DynamicControllerError::WorldCollision(
                    WorldCollisionError::Dimension,
                ));
            }
            if validation.world_collision_clearance.is_some()
                && world.unsupported_shape_count > 0
                && validation.unknown_collision_geometry
                    == UnknownCollisionGeometryPolicy::RejectProgram
            {
                return Err(DynamicControllerError::UnsupportedWorldCollisionGeometry {
                    count: world.unsupported_shape_count,
                });
            }
        }
        let wbc = if config.floating_world_collision_barrier.is_some() {
            let world = world_collision
                .as_ref()
                .ok_or(DynamicControllerError::MissingWorldCollisionModel)?;
            FloatingDynamicWbc::new_with_world_collision(
                program.model.clone(),
                config,
                world.clone(),
            )?
        } else {
            FloatingDynamicWbc::new(program.model.clone(), config)?
        };
        Ok(Self {
            wbc,
            actuation: program.actuation.clone(),
            program_epoch: program.header.program_epoch,
            control_horizon_ns: program.timing.control_horizon_ns,
            sample_period_ns: program.timing.sample_period_ns,
            collision,
            world_collision,
            validation,
        })
    }

    pub fn advance_into(
        &self,
        input: FloatingDynamicControllerInput<'_>,
        state_in: &FloatingDynamicControllerState,
        state_out: &mut FloatingDynamicControllerState,
        output: &mut FloatingDynamicControllerOutput,
        scratch: &mut FloatingDynamicControllerScratch,
    ) -> Result<DynamicStepStatus, DynamicControllerError> {
        self.advance_with_observation_error_into(
            input,
            RobotObservationErrorBound::default(),
            state_in,
            state_out,
            output,
            scratch,
        )
    }

    /// Solve and admit a command while carrying canonical reconstruction
    /// error through both local WBC rows and primary/brake geometry.
    pub fn advance_with_observation_error_into(
        &self,
        input: FloatingDynamicControllerInput<'_>,
        observation_error: RobotObservationErrorBound,
        state_in: &FloatingDynamicControllerState,
        state_out: &mut FloatingDynamicControllerState,
        output: &mut FloatingDynamicControllerOutput,
        scratch: &mut FloatingDynamicControllerScratch,
    ) -> Result<DynamicStepStatus, DynamicControllerError> {
        if !observation_error.validate() {
            return Err(DynamicControllerError::InvalidObservationError);
        }
        self.validate_transition_layout(
            input.tick_time_ns,
            input.program_epoch,
            state_in,
            state_out,
            output,
            scratch,
        )?;
        self.wbc.solve_into_with_observation_error(
            input.wbc,
            observation_error,
            &mut output.wbc,
            &mut scratch.wbc,
        )?;
        self.advance_solved_output_into(
            input.tick_time_ns,
            input.observation_stamp,
            input.wbc.state,
            input.wbc.root_twist_world,
            observation_error,
            state_in,
            state_out,
            output,
            scratch,
        )
    }

    /// Apply the exact command-admission boundary to an already solved WBC
    /// query. This avoids repeating the physical solve in adapters that need
    /// both state-local authority and command-trajectory evidence.
    pub fn advance_solved_into(
        &self,
        input: FloatingDynamicControllerSolvedInput<'_>,
        state_in: &FloatingDynamicControllerState,
        state_out: &mut FloatingDynamicControllerState,
        output: &mut FloatingDynamicControllerOutput,
        scratch: &mut FloatingDynamicControllerScratch,
    ) -> Result<DynamicStepStatus, DynamicControllerError> {
        self.advance_solved_with_observation_error_into(
            input,
            RobotObservationErrorBound::default(),
            state_in,
            state_out,
            output,
            scratch,
        )
    }

    /// Apply command admission to a caller-solved WBC result while preserving
    /// the reconstruction error used by that solve as separate geometry
    /// authority. Tracking mismatch and root forecast growth are not folded
    /// into this value.
    pub fn advance_solved_with_observation_error_into(
        &self,
        input: FloatingDynamicControllerSolvedInput<'_>,
        observation_error: RobotObservationErrorBound,
        state_in: &FloatingDynamicControllerState,
        state_out: &mut FloatingDynamicControllerState,
        output: &mut FloatingDynamicControllerOutput,
        scratch: &mut FloatingDynamicControllerScratch,
    ) -> Result<DynamicStepStatus, DynamicControllerError> {
        if !observation_error.validate() {
            return Err(DynamicControllerError::InvalidObservationError);
        }
        self.validate_transition_layout(
            input.tick_time_ns,
            input.program_epoch,
            state_in,
            state_out,
            output,
            scratch,
        )?;
        if !output.wbc.copy_from_same_layout(input.solved_wbc) {
            return Err(DynamicControllerError::Layout);
        }
        self.advance_solved_output_into(
            input.tick_time_ns,
            input.observation_stamp,
            input.observed,
            input.root_twist_world,
            observation_error,
            state_in,
            state_out,
            output,
            scratch,
        )
    }

    fn validate_transition_layout(
        &self,
        tick_time_ns: ControlTime,
        program_epoch: u64,
        state_in: &FloatingDynamicControllerState,
        state_out: &FloatingDynamicControllerState,
        output: &FloatingDynamicControllerOutput,
        scratch: &FloatingDynamicControllerScratch,
    ) -> Result<(), DynamicControllerError> {
        if program_epoch != self.program_epoch || state_in.program_epoch != self.program_epoch {
            return Err(DynamicControllerError::ProgramEpochMismatch);
        }
        if state_in
            .last_tick_time_ns
            .is_some_and(|last| tick_time_ns < last)
        {
            return Err(DynamicControllerError::NonMonotonicTime);
        }
        let actuator_count = self.actuation.actuators.len();
        if !output_layout_matches(output, actuator_count)
            || !scratch_layout_matches(scratch, self.wbc.model().dof, actuator_count)
            || !state_layout_matches(state_in, actuator_count)
            || !state_layout_matches(state_out, actuator_count)
        {
            return Err(DynamicControllerError::Layout);
        }
        Ok(())
    }

    fn advance_solved_output_into(
        &self,
        tick_time_ns: ControlTime,
        observation_stamp: RobotObservationStamp,
        observed: &RobotState,
        root_twist_world: Motion6,
        observation_error: RobotObservationErrorBound,
        state_in: &FloatingDynamicControllerState,
        state_out: &mut FloatingDynamicControllerState,
        output: &mut FloatingDynamicControllerOutput,
        scratch: &mut FloatingDynamicControllerScratch,
    ) -> Result<DynamicStepStatus, DynamicControllerError> {
        let actuator_count = self.actuation.actuators.len();
        output.observation_error = observation_error;
        for coordinate in 0..scratch.joint_acceleration.len() {
            scratch.joint_acceleration[coordinate] =
                output.wbc.generalized_acceleration[6 + coordinate];
        }
        self.actuation.write_actuator_velocity(
            &scratch.joint_acceleration,
            &mut scratch.primary_acceleration,
        )?;
        self.actuation.write_actuator_effort(
            output.wbc.generalized_effort(),
            &mut output.candidate_actuator_effort,
        )?;
        output.admitted_actuator_effort.fill(0.0);

        self.actuation
            .write_actuator_velocity(&observed.q, &mut scratch.observed_actuator_position)?;
        self.actuation
            .write_actuator_velocity(&observed.v, &mut scratch.observed_actuator_velocity)?;
        output.previous_plan_expired = false;
        if state_in.has_previous_command {
            let previous_end = state_in
                .previous_command
                .start_time_ns
                .saturating_add(state_in.previous_command.duration_ns);
            output.previous_plan_expired = tick_time_ns > previous_end;
            state_in.previous_command.evaluate_into(
                tick_time_ns,
                scratch.start_position.as_mut_slice(),
                scratch.start_velocity.as_mut_slice(),
                scratch.start_acceleration.as_mut_slice(),
            )?;
        } else {
            scratch
                .start_position
                .copy_from(&state_in.commanded_position);
            scratch
                .start_velocity
                .copy_from(&state_in.commanded_velocity);
            scratch
                .start_acceleration
                .copy_from(&state_in.commanded_acceleration);
        }
        output.maximum_observed_command_position_divergence = scratch
            .observed_actuator_position
            .iter()
            .zip(scratch.start_position.iter())
            .map(|(observed, commanded)| (observed - commanded).abs())
            .fold(0.0, f64::max);
        output.command_tracking = command_tracking_evidence(
            &scratch.observed_actuator_position,
            &scratch.observed_actuator_velocity,
            &scratch.start_position,
            &scratch.start_velocity,
            self.validation.command_tracking_limits,
        );
        output.splice_position.copy_from(&scratch.start_position);
        output.splice_velocity.copy_from(&scratch.start_velocity);
        output
            .splice_acceleration
            .copy_from(&scratch.start_acceleration);

        let horizon = self.control_horizon_ns as f64 * 1e-9;
        let root_acceleration = SpatialAcceleration6(
            output
                .wbc
                .generalized_acceleration
                .fixed_rows::<6>(0)
                .into_owned(),
        );
        let primary_root_displacement =
            Motion6(root_twist_world.0 * horizon + root_acceleration.0 * (0.5 * horizon * horizon));
        let primary_root_end_twist = Motion6(root_twist_world.0 + root_acceleration.0 * horizon);
        output.primary_root_prediction.set_boundary_conditions(
            tick_time_ns,
            self.control_horizon_ns,
            observed.control_world_from_root,
            root_twist_world,
            root_acceleration,
            primary_root_displacement,
            primary_root_end_twist,
            root_acceleration,
        )?;
        let root_error_growth = root_prediction_growth_with_observation_error(
            self.validation.root_prediction_error_growth,
            observation_error,
        )?;
        output
            .primary_root_prediction
            .set_error_growth(root_error_growth)?;
        output.contingency_root_prediction.set_boundary_conditions(
            tick_time_ns,
            self.control_horizon_ns,
            observed.control_world_from_root,
            root_twist_world,
            SpatialAcceleration6::default(),
            Motion6(root_twist_world.0 * (0.5 * horizon)),
            Motion6::default(),
            SpatialAcceleration6::default(),
        )?;
        output
            .contingency_root_prediction
            .set_error_growth(root_error_growth)?;
        for actuator in 0..actuator_count {
            let acceleration = scratch.primary_acceleration[actuator];
            scratch.primary_position[actuator] = scratch.start_position[actuator]
                + horizon * scratch.start_velocity[actuator]
                + 0.5 * horizon * horizon * acceleration;
            scratch.primary_velocity[actuator] =
                scratch.start_velocity[actuator] + horizon * acceleration;
            scratch.contingency_position[actuator] =
                scratch.start_position[actuator] + 0.5 * horizon * scratch.start_velocity[actuator];
        }

        let solve_admitted = matches!(
            output.wbc.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        );
        output.admission_flags = DynamicAdmissionFlags::default();
        output.world_scene = self.world_scene_evidence(tick_time_ns);
        output.robot_observation = evaluate_robot_observation(
            tick_time_ns,
            observation_stamp,
            self.validation.robot_observation_limits,
        );
        if !output.world_scene.is_valid() {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::WORLD_SCENE_INVALID);
        }
        if !output.robot_observation.causal {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::ROBOT_OBSERVATION_FUTURE);
        }
        if !output.robot_observation.age_valid {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::ROBOT_OBSERVATION_STALE);
        }
        if !output.robot_observation.synchronization_valid {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::ROBOT_OBSERVATION_UNCERTAIN);
        }
        if output.wbc.status == SolveStatus::SolvedWithSlack {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::SOLVED_WITH_SLACK);
        } else if !solve_admitted {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::SOLVER_NOT_ADMITTED);
        }
        if output.previous_plan_expired {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::PREVIOUS_PLAN_EXPIRED);
        }
        match output.command_tracking.action {
            CommandTrackingAction::Nominal => {}
            CommandTrackingAction::Contingency => output
                .admission_flags
                .insert(DynamicAdmissionFlags::COMMAND_TRACKING_CONTINGENCY),
            CommandTrackingAction::Rejected => output
                .admission_flags
                .insert(DynamicAdmissionFlags::COMMAND_TRACKING_REJECTED),
        }
        output.primary_valid = false;
        output.primary_actuator_valid = false;
        output.primary_joint_position_valid = false;
        output.primary_collision_valid = false;
        output.primary_world_collision_valid = false;
        output.primary_minimum_joint_position_headroom = f64::NEG_INFINITY;
        if solve_admitted {
            output.primary_segment.set_boundary_conditions(
                tick_time_ns,
                self.control_horizon_ns,
                &scratch.start_position,
                &scratch.start_velocity,
                &scratch.start_acceleration,
                &scratch.primary_position,
                &scratch.primary_velocity,
                &scratch.primary_acceleration,
            )?;
            output.primary_actuator_valid = output
                .primary_segment
                .validate_into(&scratch.segment_limits, &mut output.primary_extrema)
                .is_ok();
            map_actuator_segment_into(
                &self.actuation,
                &output.primary_segment,
                &mut scratch.primary_joint_segment,
            );
            output.primary_joint_position_valid = scratch
                .primary_joint_segment
                .validate_into(
                    &scratch.joint_segment_limits,
                    &mut output.primary_joint_extrema,
                )
                .is_ok();
            output.primary_minimum_joint_position_headroom = minimum_position_headroom(
                &output.primary_joint_extrema,
                &scratch.joint_segment_limits,
            );
            let collision_admission = self.validate_collision(
                &scratch.primary_joint_segment,
                &output.primary_joint_extrema,
                observed,
                observation_error.represented_point_position_error_m,
                &mut scratch.collision_state,
                &mut scratch.collision_acceleration,
                &mut scratch.collision_model,
                &mut scratch.collision_sample,
                &mut scratch.collision_evaluation,
            )?;
            output.primary_collision = collision_admission.report;
            output.primary_continuous_clearance_lower_bound =
                collision_admission.continuous_clearance_lower_bound;
            output.primary_robust_collision_minimum_clearance_m =
                collision_admission.robust_minimum_clearance_m;
            output.primary_robust_continuous_clearance_lower_bound_m =
                collision_admission.robust_continuous_clearance_lower_bound_m;
            output.primary_continuous_limiting_pair_id =
                collision_admission.continuous_limiting_pair_id;
            output.primary_maximum_relative_speed_bound =
                collision_admission.maximum_relative_speed_bound;
            output.primary_continuity_leaf_interval_count =
                collision_admission.continuity_leaf_interval_count;
            output.primary_collision_refinement_pair_samples =
                collision_admission.collision_refinement_pair_samples;
            output.primary_continuity_unresolved_interval_count =
                collision_admission.continuity_unresolved_interval_count;
            output.primary_continuity_maximum_subdivision_depth =
                collision_admission.continuity_maximum_subdivision_depth;
            output.primary_collision_valid = collision_admission.valid;
            let world_admission = self.validate_world_collision(
                &scratch.primary_joint_segment,
                &output.primary_joint_extrema,
                &output.primary_root_prediction,
                output.world_scene,
                observation_error.represented_point_position_error_m,
                &mut scratch.collision_state,
                &mut scratch.collision_acceleration,
                &mut scratch.collision_model,
                &mut scratch.world_collision_evaluation,
            )?;
            output.primary_world_collision = world_admission.report;
            output.primary_world_continuity = world_admission.continuity;
            output.primary_robust_world_minimum_clearance_m =
                world_admission.robust_minimum_clearance_m;
            output.primary_robust_world_continuous_clearance_lower_bound_m =
                world_admission.robust_continuous_clearance_lower_bound_m;
            output.primary_world_collision_valid = world_admission.valid;
            output.primary_valid = output.robot_observation.is_valid()
                && output.command_tracking.action == CommandTrackingAction::Nominal
                && output.primary_actuator_valid
                && output.primary_joint_position_valid
                && output.primary_collision_valid
                && output.primary_world_collision_valid;
            if !output.primary_actuator_valid {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_ACTUATOR_LIMIT);
            }
            if !output.primary_joint_position_valid {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_JOINT_POSITION_LIMIT);
            }
            if !output.primary_collision.is_clear() {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_COLLISION);
            } else if collision_admission.robust_minimum_clearance_m
                >= self
                    .validation
                    .self_collision_clearance
                    .unwrap_or(f64::NEG_INFINITY)
                && !output.primary_collision_valid
            {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_CONTINUOUS_CLEARANCE);
            }
            if collision_admission.observation_error_caused_failure {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_COLLISION_OBSERVATION_ERROR);
            }
            if !output.primary_world_collision.is_known() {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_WORLD_UNKNOWN);
            } else if output.primary_world_collision.has_sampled_violation() {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_WORLD_COLLISION);
            } else if world_admission.robust_minimum_clearance_m
                >= self
                    .validation
                    .world_collision_clearance
                    .unwrap_or(f64::NEG_INFINITY)
                && !output.primary_world_collision_valid
            {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_WORLD_CONTINUOUS_CLEARANCE);
            }
            if world_admission.observation_error_caused_failure {
                output
                    .admission_flags
                    .insert(DynamicAdmissionFlags::PRIMARY_WORLD_OBSERVATION_ERROR);
            }
        } else {
            clear_extrema(&mut output.primary_extrema);
            clear_extrema(&mut output.primary_joint_extrema);
            output.primary_collision = empty_collision_report();
            output.primary_continuous_clearance_lower_bound = f64::NEG_INFINITY;
            output.primary_robust_collision_minimum_clearance_m = f64::NEG_INFINITY;
            output.primary_robust_continuous_clearance_lower_bound_m = f64::NEG_INFINITY;
            output.primary_continuous_limiting_pair_id = None;
            output.primary_maximum_relative_speed_bound = f64::INFINITY;
            output.primary_continuity_leaf_interval_count = 0;
            output.primary_collision_refinement_pair_samples = 0;
            output.primary_continuity_unresolved_interval_count = 0;
            output.primary_continuity_maximum_subdivision_depth = 0;
            output.primary_world_collision = self.empty_world_collision_report();
            output.primary_world_continuity = WorldCollisionContinuityReport::empty();
            output.primary_robust_world_minimum_clearance_m = f64::NEG_INFINITY;
            output.primary_robust_world_continuous_clearance_lower_bound_m = f64::NEG_INFINITY;
        }

        output.contingency_segment.set_boundary_conditions(
            tick_time_ns,
            self.control_horizon_ns,
            &scratch.start_position,
            &scratch.start_velocity,
            &scratch.start_acceleration,
            &scratch.contingency_position,
            &scratch.zero,
            &scratch.zero,
        )?;
        output.contingency_actuator_valid = output
            .contingency_segment
            .validate_into(&scratch.segment_limits, &mut output.contingency_extrema)
            .is_ok();
        map_actuator_segment_into(
            &self.actuation,
            &output.contingency_segment,
            &mut scratch.contingency_joint_segment,
        );
        output.contingency_joint_position_valid = scratch
            .contingency_joint_segment
            .validate_into(
                &scratch.joint_segment_limits,
                &mut output.contingency_joint_extrema,
            )
            .is_ok();
        output.contingency_minimum_joint_position_headroom = minimum_position_headroom(
            &output.contingency_joint_extrema,
            &scratch.joint_segment_limits,
        );
        let contingency_collision_admission = self.validate_collision(
            &scratch.contingency_joint_segment,
            &output.contingency_joint_extrema,
            observed,
            observation_error.represented_point_position_error_m,
            &mut scratch.collision_state,
            &mut scratch.collision_acceleration,
            &mut scratch.collision_model,
            &mut scratch.collision_sample,
            &mut scratch.collision_evaluation,
        )?;
        output.contingency_collision = contingency_collision_admission.report;
        output.contingency_continuous_clearance_lower_bound =
            contingency_collision_admission.continuous_clearance_lower_bound;
        output.contingency_robust_collision_minimum_clearance_m =
            contingency_collision_admission.robust_minimum_clearance_m;
        output.contingency_robust_continuous_clearance_lower_bound_m =
            contingency_collision_admission.robust_continuous_clearance_lower_bound_m;
        output.contingency_continuous_limiting_pair_id =
            contingency_collision_admission.continuous_limiting_pair_id;
        output.contingency_maximum_relative_speed_bound =
            contingency_collision_admission.maximum_relative_speed_bound;
        output.contingency_continuity_leaf_interval_count =
            contingency_collision_admission.continuity_leaf_interval_count;
        output.contingency_collision_refinement_pair_samples =
            contingency_collision_admission.collision_refinement_pair_samples;
        output.contingency_continuity_unresolved_interval_count =
            contingency_collision_admission.continuity_unresolved_interval_count;
        output.contingency_continuity_maximum_subdivision_depth =
            contingency_collision_admission.continuity_maximum_subdivision_depth;
        output.contingency_collision_valid = contingency_collision_admission.valid;
        let contingency_world_admission = self.validate_world_collision(
            &scratch.contingency_joint_segment,
            &output.contingency_joint_extrema,
            &output.contingency_root_prediction,
            output.world_scene,
            observation_error.represented_point_position_error_m,
            &mut scratch.collision_state,
            &mut scratch.collision_acceleration,
            &mut scratch.collision_model,
            &mut scratch.world_collision_evaluation,
        )?;
        output.contingency_world_collision = contingency_world_admission.report;
        output.contingency_world_continuity = contingency_world_admission.continuity;
        output.contingency_robust_world_minimum_clearance_m =
            contingency_world_admission.robust_minimum_clearance_m;
        output.contingency_robust_world_continuous_clearance_lower_bound_m =
            contingency_world_admission.robust_continuous_clearance_lower_bound_m;
        output.contingency_world_collision_valid = contingency_world_admission.valid;
        output.contingency_valid = output.robot_observation.is_valid()
            && output.command_tracking.action != CommandTrackingAction::Rejected
            && output.contingency_actuator_valid
            && output.contingency_joint_position_valid
            && output.contingency_collision_valid
            && output.contingency_world_collision_valid;
        if !output.contingency_actuator_valid {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_ACTUATOR_LIMIT);
        }
        if !output.contingency_joint_position_valid {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_JOINT_POSITION_LIMIT);
        }
        if !output.contingency_collision.is_clear() {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_COLLISION);
        } else if contingency_collision_admission.robust_minimum_clearance_m
            >= self
                .validation
                .self_collision_clearance
                .unwrap_or(f64::NEG_INFINITY)
            && !output.contingency_collision_valid
        {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_CONTINUOUS_CLEARANCE);
        }
        if contingency_collision_admission.observation_error_caused_failure {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_COLLISION_OBSERVATION_ERROR);
        }
        if !output.contingency_world_collision.is_known() {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_WORLD_UNKNOWN);
        } else if output.contingency_world_collision.has_sampled_violation() {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_WORLD_COLLISION);
        } else if contingency_world_admission.robust_minimum_clearance_m
            >= self
                .validation
                .world_collision_clearance
                .unwrap_or(f64::NEG_INFINITY)
            && !output.contingency_world_collision_valid
        {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_WORLD_CONTINUOUS_CLEARANCE);
        }
        if contingency_world_admission.observation_error_caused_failure {
            output
                .admission_flags
                .insert(DynamicAdmissionFlags::CONTINGENCY_WORLD_OBSERVATION_ERROR);
        }

        let primary_selected = output.primary_valid && !output.previous_plan_expired;
        if primary_selected {
            output.selection = DynamicPlanSelection::Primary;
            output.status = if output.wbc.status == SolveStatus::Solved {
                DynamicStepStatus::Ok
            } else {
                DynamicStepStatus::Degraded
            };
            output
                .admitted_actuator_effort
                .copy_from(&output.candidate_actuator_effort);
            output
                .primary_segment
                .sample_block_into(self.sample_period_ns, &mut output.samples)?;
        } else if output.contingency_valid {
            output.selection = DynamicPlanSelection::Contingency;
            output.status = DynamicStepStatus::Contingency;
            output
                .contingency_segment
                .sample_block_into(self.sample_period_ns, &mut output.samples)?;
        } else {
            output.selection = DynamicPlanSelection::Rejected;
            output.status = DynamicStepStatus::Rejected;
            clear_samples(&mut output.samples, actuator_count);
        }

        state_out.clone_from(state_in);
        if output.selection != DynamicPlanSelection::Rejected {
            let terminal = output.samples.len().saturating_sub(1);
            state_out.commanded_position.as_mut_slice().copy_from_slice(
                output
                    .samples
                    .position(terminal)
                    .expect("selected segment produces samples"),
            );
            state_out.commanded_velocity.as_mut_slice().copy_from_slice(
                output
                    .samples
                    .velocity(terminal)
                    .expect("selected segment produces samples"),
            );
            state_out
                .commanded_acceleration
                .as_mut_slice()
                .copy_from_slice(
                    output
                        .samples
                        .acceleration(terminal)
                        .expect("selected segment produces samples"),
                );
            let selected = if primary_selected {
                &output.primary_segment
            } else {
                &output.contingency_segment
            };
            state_out.previous_command.clone_from(selected);
            state_out.has_previous_command = true;
            state_out.last_tick_time_ns = Some(tick_time_ns);
            state_out.tick_count = state_in.tick_count.saturating_add(1);
        }
        Ok(output.status)
    }

    fn validate_collision(
        &self,
        joint_segment: &QuinticSegment,
        joint_extrema: &SegmentExtrema,
        observed: &RobotState,
        observation_point_error_m: f64,
        collision_state: &mut RobotState,
        collision_acceleration: &mut DVector<f64>,
        collision_model: &mut ModelCache,
        collision_sample: &mut DistanceSample,
        collision_evaluation: &mut CollisionEvaluationScratch,
    ) -> Result<DynamicCollisionAdmission, DynamicControllerError> {
        let Some(clearance) = self.validation.self_collision_clearance else {
            return Ok(DynamicCollisionAdmission {
                report: empty_collision_report(),
                continuous_clearance_lower_bound: f64::INFINITY,
                robust_minimum_clearance_m: f64::INFINITY,
                robust_continuous_clearance_lower_bound_m: f64::INFINITY,
                continuous_limiting_pair_id: None,
                maximum_relative_speed_bound: 0.0,
                continuity_leaf_interval_count: 0,
                collision_refinement_pair_samples: 0,
                continuity_unresolved_interval_count: 0,
                continuity_maximum_subdivision_depth: 0,
                observation_error_caused_failure: false,
                valid: true,
            });
        };
        let mut report = self.collision.validate_segment_on_grid_buffered(
            self.wbc.model(),
            joint_segment,
            observed.control_world_from_root,
            self.sample_period_ns,
            clearance,
            collision_state,
            collision_acceleration,
            collision_model,
            collision_sample,
            collision_evaluation,
        )?;
        let continuity =
            if self.validation.collision_max_subdivision_depth == 0 || !report.is_clear() {
                let half_interval_seconds = 0.5 * self.sample_period_ns as f64 * 1e-9;
                self.collision.pairwise_continuous_clearance_bound(
                    collision_evaluation,
                    &joint_extrema.max_abs_velocity,
                    half_interval_seconds,
                )
            } else {
                Some(
                    self.collision
                        .adaptive_pairwise_continuous_clearance_bound(
                            self.wbc.model(),
                            joint_segment,
                            observed.control_world_from_root,
                            self.sample_period_ns,
                            clearance,
                            self.validation.collision_max_subdivision_depth,
                            &joint_extrema.max_abs_velocity,
                            collision_state,
                            collision_acceleration,
                            collision_model,
                            collision_evaluation,
                            &mut report,
                        )?,
                )
            }
            .unwrap_or(crate::collision::CollisionContinuityReport {
                minimum_clearance_lower_bound: f64::NEG_INFINITY,
                limiting_pair_id: None,
                limiting_relative_speed_bound: f64::INFINITY,
                leaf_interval_count: 0,
                refinement_pair_samples_evaluated: 0,
                unresolved_interval_count: 0,
                maximum_subdivision_depth_reached: 0,
            });
        let continuous_clearance_lower_bound = continuity.minimum_clearance_lower_bound;
        let raw_continuity_valid = self.validation.collision_continuity
            == CollisionContinuityPolicy::GridOnly
            || continuous_clearance_lower_bound >= clearance;
        // Both members of a self-collision pair may independently move within the
        // represented-point reconstruction radius, so relative separation loses
        // twice that radius. Root-frame error is common-mode and cancels here.
        let observation_erosion_m = 2.0 * observation_point_error_m;
        let robust_minimum_clearance_m = report.minimum_signed_distance - observation_erosion_m;
        let robust_continuous_clearance_lower_bound_m =
            continuous_clearance_lower_bound - observation_erosion_m;
        let robust_sampled_valid = report.is_clear() && robust_minimum_clearance_m >= clearance;
        let robust_continuity_valid = self.validation.collision_continuity
            == CollisionContinuityPolicy::GridOnly
            || robust_continuous_clearance_lower_bound_m >= clearance;
        let raw_valid = report.is_clear() && raw_continuity_valid;
        let valid = robust_sampled_valid && robust_continuity_valid;
        Ok(DynamicCollisionAdmission {
            valid,
            report,
            continuous_clearance_lower_bound,
            robust_minimum_clearance_m,
            robust_continuous_clearance_lower_bound_m,
            continuous_limiting_pair_id: continuity.limiting_pair_id,
            maximum_relative_speed_bound: continuity.limiting_relative_speed_bound,
            continuity_leaf_interval_count: continuity.leaf_interval_count,
            collision_refinement_pair_samples: continuity.refinement_pair_samples_evaluated,
            continuity_unresolved_interval_count: continuity.unresolved_interval_count,
            continuity_maximum_subdivision_depth: continuity.maximum_subdivision_depth_reached,
            observation_error_caused_failure: raw_valid && !valid,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn validate_world_collision(
        &self,
        joint_segment: &QuinticSegment,
        joint_extrema: &SegmentExtrema,
        root_prediction: &RootPosePredictionSegment,
        scene: WorldSceneEvidence,
        observation_point_error_m: f64,
        collision_state: &mut RobotState,
        collision_acceleration: &mut DVector<f64>,
        collision_model: &mut ModelCache,
        world_evaluation: &mut WorldCollisionSweepScratch,
    ) -> Result<DynamicWorldCollisionAdmission, DynamicControllerError> {
        let Some(clearance) = self.validation.world_collision_clearance else {
            return Ok(DynamicWorldCollisionAdmission {
                report: self.empty_world_collision_report(),
                continuity: WorldCollisionContinuityReport::empty(),
                robust_minimum_clearance_m: f64::INFINITY,
                robust_continuous_clearance_lower_bound_m: f64::INFINITY,
                observation_error_caused_failure: false,
                valid: true,
            });
        };
        let world = self
            .world_collision
            .as_ref()
            .ok_or(DynamicControllerError::MissingWorldCollisionModel)?;
        if !scene.is_valid() {
            let mut report = self.empty_world_collision_report();
            report.scene = scene;
            return Ok(DynamicWorldCollisionAdmission {
                report,
                continuity: WorldCollisionContinuityReport::empty(),
                robust_minimum_clearance_m: f64::NEG_INFINITY,
                robust_continuous_clearance_lower_bound_m: f64::NEG_INFINITY,
                observation_error_caused_failure: false,
                valid: false,
            });
        }
        let mut report = world.validate_segment_on_grid_with_root_prediction_buffered(
            self.wbc.model(),
            joint_segment,
            root_prediction,
            self.sample_period_ns,
            clearance,
            collision_state,
            collision_acceleration,
            collision_model,
            world_evaluation,
        )?;
        report.scene = scene;
        let mut maximum_abs_root_twist = Motion6::default();
        root_prediction.maximum_abs_twist_between_into(
            joint_segment.start_time_ns,
            joint_segment
                .start_time_ns
                .saturating_add(joint_segment.duration_ns),
            &mut maximum_abs_root_twist,
        )?;
        let continuity = if report.is_clear() {
            if self.validation.world_collision_max_subdivision_depth == 0 {
                let half_interval_seconds = 0.5 * self.sample_period_ns as f64 * 1e-9;
                world
                    .continuous_clearance_bound_with_root_prediction(
                        world_evaluation,
                        &joint_extrema.max_abs_velocity,
                        root_prediction,
                        &maximum_abs_root_twist,
                        half_interval_seconds,
                    )
                    .ok_or(WorldCollisionError::Dimension)?
            } else {
                world.adaptive_continuous_clearance_bound_with_root_prediction(
                    self.wbc.model(),
                    joint_segment,
                    root_prediction,
                    self.sample_period_ns,
                    clearance,
                    self.validation.world_collision_max_subdivision_depth,
                    &joint_extrema.max_abs_velocity,
                    &maximum_abs_root_twist,
                    collision_state,
                    collision_acceleration,
                    collision_model,
                    world_evaluation,
                    &mut report,
                )?
            }
        } else {
            WorldCollisionContinuityReport {
                minimum_clearance_lower_bound_m: f64::NEG_INFINITY,
                limiting_probe_id: report.minimum_probe_id,
                limiting_body: report.minimum_body,
                limiting_distance_rate_bound_mps: f64::INFINITY,
                leaf_interval_count: 0,
                refinement_probe_samples_evaluated: 0,
                unresolved_interval_count: 0,
                maximum_subdivision_depth_reached: 0,
            }
        };
        let raw_continuity_valid = self.validation.world_collision_continuity
            == CollisionContinuityPolicy::GridOnly
            || continuity.minimum_clearance_lower_bound_m >= clearance;
        // Root reconstruction error is already part of the root prediction's
        // initial-radius witness. The remaining body-local point radius erodes
        // the signed-distance witness once, scaled by the field Lipschitz bound.
        let observation_erosion_m = world.field_lipschitz_bound * observation_point_error_m;
        let robust_minimum_clearance_m = report.minimum_signed_distance_m - observation_erosion_m;
        let robust_continuous_clearance_lower_bound_m =
            continuity.minimum_clearance_lower_bound_m - observation_erosion_m;
        let robust_sampled_valid = report.is_clear() && robust_minimum_clearance_m >= clearance;
        let robust_continuity_valid = self.validation.world_collision_continuity
            == CollisionContinuityPolicy::GridOnly
            || robust_continuous_clearance_lower_bound_m >= clearance;
        let raw_valid = report.is_clear() && raw_continuity_valid;
        let valid = robust_sampled_valid && robust_continuity_valid;
        Ok(DynamicWorldCollisionAdmission {
            valid,
            report,
            continuity,
            robust_minimum_clearance_m,
            robust_continuous_clearance_lower_bound_m,
            observation_error_caused_failure: raw_valid && !valid,
        })
    }

    fn empty_world_collision_report(&self) -> WorldCollisionSweepReport {
        let outside_policy = self
            .world_collision
            .as_ref()
            .map_or(SdfOutsidePolicy::Reject, |world| {
                world.field.outside_policy()
            });
        let mut report = WorldCollisionSweepReport::empty(outside_policy);
        if let Some(world) = self.world_collision.as_ref() {
            report.represented_probe_count = world.probes.len();
            report.unsupported_shape_count = world.unsupported_shape_count;
        }
        report
    }

    fn world_scene_evidence(&self, tick_time_ns: ControlTime) -> WorldSceneEvidence {
        self.world_collision
            .as_ref()
            .map_or_else(WorldSceneEvidence::timeless_valid, |world| {
                world.scene_evidence(
                    tick_time_ns,
                    self.control_horizon_ns,
                    self.validation.expected_world_scene_epoch,
                    self.validation.maximum_world_scene_age_ns,
                    self.validation.require_world_scene_horizon_validity,
                )
            })
    }
}

#[derive(Debug, Error)]
pub enum DynamicControllerError {
    #[error("program timing must be positive and sample period must divide the horizon")]
    InvalidTiming,
    #[error("dynamic trajectory validation configuration is invalid")]
    InvalidValidationConfig,
    #[error("collision-enabled program has {count} unsupported authored geometry shapes")]
    UnsupportedCollisionGeometry { count: usize },
    #[error("world-collision validation is enabled without a compiled world field")]
    MissingWorldCollisionModel,
    #[error("world-collision program has {count} unsupported authored geometry shapes")]
    UnsupportedWorldCollisionGeometry { count: usize },
    #[error("program epoch does not match controller command state")]
    ProgramEpochMismatch,
    #[error("tick time moved backwards")]
    NonMonotonicTime,
    #[error("actuation map has no unique generalized-to-actuator command mapping")]
    UnsupportedActuation,
    #[error("controller state, output, or scratch layout does not match the program")]
    Layout,
    #[error("robot observation reconstruction error bound is invalid")]
    InvalidObservationError,
    #[error(transparent)]
    DynamicWbc(#[from] DynamicWbcError),
    #[error(transparent)]
    Actuation(#[from] ActuationError),
    #[error(transparent)]
    Trajectory(#[from] TrajectoryError),
    #[error(transparent)]
    Collision(#[from] CollisionError),
    #[error(transparent)]
    WorldCollision(#[from] WorldCollisionError),
    #[error(transparent)]
    Model(#[from] crate::model::ModelError),
}

fn root_prediction_growth_with_observation_error(
    mut growth: RootPredictionErrorGrowth,
    observation_error: RobotObservationErrorBound,
) -> Result<RootPredictionErrorGrowth, DynamicControllerError> {
    growth.initial_translation_radius_m += observation_error.root_translation_error_m;
    growth.initial_rotation_radius_rad += observation_error.root_rotation_error_rad;
    if growth.is_valid() {
        Ok(growth)
    } else {
        Err(DynamicControllerError::InvalidObservationError)
    }
}

fn validate_timing(program: &MotionProgram) -> Result<(), DynamicControllerError> {
    if program.timing.control_horizon_ns <= 0
        || program.timing.sample_period_ns <= 0
        || program.timing.control_horizon_ns % program.timing.sample_period_ns != 0
    {
        Err(DynamicControllerError::InvalidTiming)
    } else {
        Ok(())
    }
}

fn segment_workspace(actuators: usize) -> QuinticSegment {
    QuinticSegment {
        start_time_ns: 0,
        duration_ns: 0,
        coefficients: Vec::with_capacity(actuators),
    }
}

fn extrema_workspace(actuators: usize) -> SegmentExtrema {
    SegmentExtrema {
        min_position: Vec::with_capacity(actuators),
        max_position: Vec::with_capacity(actuators),
        max_abs_velocity: Vec::with_capacity(actuators),
        max_abs_acceleration: Vec::with_capacity(actuators),
        max_abs_jerk: Vec::with_capacity(actuators),
    }
}

fn clear_extrema(extrema: &mut SegmentExtrema) {
    extrema.min_position.clear();
    extrema.max_position.clear();
    extrema.max_abs_velocity.clear();
    extrema.max_abs_acceleration.clear();
    extrema.max_abs_jerk.clear();
}

fn clear_samples(samples: &mut ActuatorSampleBlock, actuators: usize) {
    samples.times_ns.clear();
    samples.position.clear();
    samples.velocity.clear();
    samples.acceleration.clear();
    samples.dof = actuators;
}

fn empty_collision_report() -> CollisionSweepReport {
    CollisionSweepReport {
        samples_evaluated: 0,
        minimum_signed_distance: f64::INFINITY,
        minimum_pair_id: None,
        minimum_time_ns: None,
        first_violation_pair_id: None,
        first_violation_time_ns: None,
    }
}

fn map_actuator_segment_into(
    actuation: &CompiledActuation,
    source: &QuinticSegment,
    destination: &mut QuinticSegment,
) {
    destination.start_time_ns = source.start_time_ns;
    destination.duration_ns = source.duration_ns;
    destination.coefficients.clear();
    for coordinate in 0..actuation.generalized_from_actuator.nrows() {
        let mut coefficients = [0.0; 6];
        for actuator in 0..actuation.generalized_from_actuator.ncols() {
            let ratio = actuation.generalized_from_actuator[(coordinate, actuator)];
            for power in 0..6 {
                coefficients[power] += ratio * source.coefficients[actuator][power];
            }
        }
        destination.coefficients.push(coefficients);
    }
}

fn minimum_position_headroom(extrema: &SegmentExtrema, limits: &[SegmentLimits]) -> f64 {
    limits
        .iter()
        .enumerate()
        .flat_map(|(coordinate, limit)| {
            [
                extrema.min_position[coordinate] - limit.min_position,
                limit.max_position - extrema.max_position[coordinate],
            ]
        })
        .fold(f64::INFINITY, f64::min)
}

fn ordered_optional_pair(contingency: Option<f64>, reject: Option<f64>) -> bool {
    match (contingency, reject) {
        (Some(contingency), Some(reject)) => contingency <= reject,
        _ => true,
    }
}

fn tracking_headroom(limit: Option<f64>, value: f64) -> f64 {
    limit.map_or(f64::INFINITY, |limit| limit - value)
}

fn command_tracking_evidence(
    observed_position: &DVector<f64>,
    observed_velocity: &DVector<f64>,
    commanded_position: &DVector<f64>,
    commanded_velocity: &DVector<f64>,
    limits: CommandTrackingLimits,
) -> CommandTrackingEvidence {
    let mut maximum_position_error = 0.0_f64;
    let mut maximum_velocity_error = 0.0_f64;
    let mut limiting_position_actuator = None;
    let mut limiting_velocity_actuator = None;
    for actuator in 0..observed_position.len() {
        let position_error = (observed_position[actuator] - commanded_position[actuator]).abs();
        if position_error > maximum_position_error || limiting_position_actuator.is_none() {
            maximum_position_error = position_error;
            limiting_position_actuator = Some(actuator);
        }
        let velocity_error = (observed_velocity[actuator] - commanded_velocity[actuator]).abs();
        if velocity_error > maximum_velocity_error || limiting_velocity_actuator.is_none() {
            maximum_velocity_error = velocity_error;
            limiting_velocity_actuator = Some(actuator);
        }
    }
    let rejected = limits
        .position_reject
        .is_some_and(|limit| maximum_position_error > limit)
        || limits
            .velocity_reject
            .is_some_and(|limit| maximum_velocity_error > limit);
    let contingency = limits
        .position_contingency
        .is_some_and(|limit| maximum_position_error > limit)
        || limits
            .velocity_contingency
            .is_some_and(|limit| maximum_velocity_error > limit);
    CommandTrackingEvidence {
        maximum_position_error,
        maximum_velocity_error,
        limiting_position_actuator,
        limiting_velocity_actuator,
        position_contingency_headroom: tracking_headroom(
            limits.position_contingency,
            maximum_position_error,
        ),
        position_reject_headroom: tracking_headroom(limits.position_reject, maximum_position_error),
        velocity_contingency_headroom: tracking_headroom(
            limits.velocity_contingency,
            maximum_velocity_error,
        ),
        velocity_reject_headroom: tracking_headroom(limits.velocity_reject, maximum_velocity_error),
        action: if rejected {
            CommandTrackingAction::Rejected
        } else if contingency {
            CommandTrackingAction::Contingency
        } else {
            CommandTrackingAction::Nominal
        },
    }
}

fn output_layout_matches(output: &FloatingDynamicControllerOutput, actuators: usize) -> bool {
    output.admitted_actuator_effort.len() == actuators
        && output.candidate_actuator_effort.len() == actuators
        && output.splice_position.len() == actuators
        && output.splice_velocity.len() == actuators
        && output.splice_acceleration.len() == actuators
        && output.primary_segment.coefficients.capacity() >= actuators
        && output.contingency_segment.coefficients.capacity() >= actuators
        && output.samples.dof == actuators
}

fn state_layout_matches(state: &FloatingDynamicControllerState, actuators: usize) -> bool {
    state.commanded_position.len() == actuators
        && state.commanded_velocity.len() == actuators
        && state.commanded_acceleration.len() == actuators
        && state.previous_command.coefficients.capacity() >= actuators
}

fn scratch_layout_matches(
    scratch: &FloatingDynamicControllerScratch,
    dof: usize,
    actuators: usize,
) -> bool {
    scratch.joint_acceleration.len() == dof
        && scratch.observed_actuator_position.len() == actuators
        && scratch.observed_actuator_velocity.len() == actuators
        && scratch.start_position.len() == actuators
        && scratch.start_velocity.len() == actuators
        && scratch.start_acceleration.len() == actuators
        && scratch.primary_position.len() == actuators
        && scratch.primary_velocity.len() == actuators
        && scratch.primary_acceleration.len() == actuators
        && scratch.contingency_position.len() == actuators
        && scratch.zero.len() == actuators
        && scratch.segment_limits.len() == actuators
        && scratch.primary_joint_segment.coefficients.capacity() >= dof
        && scratch.contingency_joint_segment.coefficients.capacity() >= dof
        && scratch.joint_segment_limits.len() == dof
        && scratch.collision_state.q.len() == dof
        && scratch.collision_state.v.len() == dof
        && scratch.collision_acceleration.len() == dof
}

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, DVector, Translation3, UnitQuaternion};

    use super::*;
    use crate::{
        dynamic_wbc::{ContactMode, ContactSpec, FloatingTaskPriorities, FloatingTaskWeights},
        math::{Motion6, Transform3, Vec3},
        program::TimingSpec,
        solver::VelocityBounds,
        world_collision::{
            DenseSdfGrid, SdfOutsidePolicy, WorldSceneStamp, WorldSceneValidity, WorldSphereProbe,
        },
    };

    #[test]
    fn command_tracking_envelope_separates_nominal_contingency_and_reject() {
        let commanded_position = DVector::from_vec(vec![0.0, 0.0]);
        let commanded_velocity = DVector::from_vec(vec![0.0, 0.0]);
        let limits = CommandTrackingLimits {
            position_contingency: Some(0.01),
            position_reject: Some(0.03),
            velocity_contingency: Some(0.2),
            velocity_reject: Some(0.5),
        };
        let nominal = command_tracking_evidence(
            &DVector::from_vec(vec![0.005, -0.004]),
            &DVector::from_vec(vec![0.1, -0.05]),
            &commanded_position,
            &commanded_velocity,
            limits,
        );
        assert_eq!(nominal.action, CommandTrackingAction::Nominal);
        assert_eq!(nominal.limiting_position_actuator, Some(0));
        assert_eq!(nominal.limiting_velocity_actuator, Some(0));
        assert!((nominal.position_contingency_headroom - 0.005).abs() < 1e-15);

        let contingency = command_tracking_evidence(
            &DVector::from_vec(vec![0.02, 0.0]),
            &DVector::from_vec(vec![0.0, 0.0]),
            &commanded_position,
            &commanded_velocity,
            limits,
        );
        assert_eq!(contingency.action, CommandTrackingAction::Contingency);
        assert_eq!(contingency.limiting_position_actuator, Some(0));
        assert!(contingency.position_contingency_headroom < 0.0);
        assert!(contingency.position_reject_headroom > 0.0);

        let rejected = command_tracking_evidence(
            &DVector::from_vec(vec![0.0, 0.0]),
            &DVector::from_vec(vec![0.0, -0.6]),
            &commanded_position,
            &commanded_velocity,
            limits,
        );
        assert_eq!(rejected.action, CommandTrackingAction::Rejected);
        assert_eq!(rejected.limiting_velocity_actuator, Some(1));
        assert!(rejected.velocity_reject_headroom < 0.0);
    }

    #[test]
    fn robot_observation_admission_separates_causality_age_and_clock_uncertainty() {
        let limits = RobotObservationLimits {
            maximum_age_ns: Some(10_000_000),
            maximum_synchronization_uncertainty_ns: Some(2_000_000),
        };
        let exact = evaluate_robot_observation(
            20_000_000,
            RobotObservationStamp::exact_at(20_000_000, 7, 42),
            limits,
        );
        assert!(exact.is_valid());
        assert_eq!(exact.age_headroom_ns, 10_000_000);
        assert_eq!(exact.synchronization_uncertainty_headroom_ns, 2_000_000);
        assert_eq!(exact.stamp.source_id, 7);
        assert_eq!(exact.stamp.source_sequence, 42);

        let stale = evaluate_robot_observation(
            20_000_000,
            RobotObservationStamp {
                source_time_ns: 9_000_000,
                mapped_time_ns: 9_000_000,
                source_sequence: 43,
                source_id: 7,
                synchronization_uncertainty_ns: 1_000_000,
            },
            limits,
        );
        assert!(stale.causal);
        assert!(!stale.age_valid);
        assert!(stale.synchronization_valid);
        assert_eq!(stale.age_headroom_ns, -1_000_000);

        let future_uncertain = evaluate_robot_observation(
            20_000_000,
            RobotObservationStamp {
                source_time_ns: 21_000_000,
                mapped_time_ns: 21_000_000,
                source_sequence: 44,
                source_id: 7,
                synchronization_uncertainty_ns: 2_000_001,
            },
            limits,
        );
        assert!(!future_uncertain.causal);
        assert!(future_uncertain.age_valid);
        assert!(!future_uncertain.synchronization_valid);
        assert_eq!(future_uncertain.age_ns, -1_000_000);
        assert_eq!(future_uncertain.synchronization_uncertainty_headroom_ns, -1);
        assert!(!future_uncertain.is_valid());
    }

    #[test]
    fn dynamic_advance_splices_samples_and_expires_to_contingency_without_physics() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/upkie/upkie.urdf"),
            TimingSpec::default(),
            98,
        )
        .unwrap();
        let controller =
            FloatingDynamicController::from_program(&program, DynamicWbcConfig::default()).unwrap();
        let observed = RobotState::zeros(&program.model);
        let mut state_a = FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut state_b = state_a.clone();
        let mut output = FloatingDynamicControllerOutput::workspace(&program, 2).unwrap();
        let mut scratch = FloatingDynamicControllerScratch::for_program(&program, 2);
        let generalized_dof = program.model.dof + 6;
        let desired = DVector::zeros(generalized_dof);
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(generalized_dof, -100.0),
            upper: DVector::from_element(generalized_dof, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(program.model.dof, -1_000.0),
            upper: DVector::from_element(program.model.dof, 1_000.0),
        };
        let total_weight = program
            .model
            .bodies
            .iter()
            .map(|body| body.mass)
            .sum::<f64>()
            * 9.81;
        let contacts = [
            ContactSpec::horizontal(
                1,
                program.model.frame_id("left_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
            ContactSpec::horizontal(
                2,
                program.model.frame_id("right_wheel_center").unwrap(),
                Vec3::zeros(),
                ContactMode::RollingPoint,
                0.8,
                total_weight,
                0.5 * total_weight,
            ),
        ];
        let wbc = FloatingDynamicWbcInput {
            state: &observed,
            root_twist_world: Motion6::default(),
            desired_generalized_acceleration: &desired,
            task_priorities: FloatingTaskPriorities::default(),
            task_weights: FloatingTaskWeights::default(),
            joint_posture_weight: 1.0,
            joint_acceleration_task: None,
            center_of_mass_task: None,
            centroidal_angular_momentum_task: None,
            frame_angular_acceleration_tasks: &[],
            point_acceleration_tasks: &[],
            generalized_acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
            support_patches: &[],
        };

        let status = controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 98,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc,
                },
                &state_a,
                &mut state_b,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert!(matches!(
            status,
            DynamicStepStatus::Ok | DynamicStepStatus::Degraded
        ));
        assert_eq!(output.selection, DynamicPlanSelection::Primary);
        assert_eq!(output.samples.len(), 20);
        assert!(output.primary_valid);
        assert!(output.contingency_valid);
        assert!(output.wbc.dynamics_residual_linf < 1e-8);
        assert!(output.wbc.contact_acceleration_residual_linf < 1e-8);
        assert!(
            output
                .admitted_actuator_effort
                .iter()
                .all(|value| value.is_finite())
        );

        std::mem::swap(&mut state_a, &mut state_b);
        let prior_terminal = state_a.commanded_position.clone();
        controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 20_000_000,
                    program_epoch: 98,
                    observation_stamp: RobotObservationStamp::exact_at(20_000_000, 1, 2),
                    wbc,
                },
                &state_a,
                &mut state_b,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let mut splice_position = DVector::zeros(program.model.dof);
        let mut splice_velocity = DVector::zeros(program.model.dof);
        let mut splice_acceleration = DVector::zeros(program.model.dof);
        output
            .primary_segment
            .evaluate_into(
                20_000_000,
                splice_position.as_mut_slice(),
                splice_velocity.as_mut_slice(),
                splice_acceleration.as_mut_slice(),
            )
            .unwrap();
        assert_eq!(splice_position.as_slice(), prior_terminal.as_slice());

        std::mem::swap(&mut state_a, &mut state_b);
        let status = controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 50_000_001,
                    program_epoch: 98,
                    observation_stamp: RobotObservationStamp::exact_at(50_000_001, 1, 3),
                    wbc,
                },
                &state_a,
                &mut state_b,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(status, DynamicStepStatus::Contingency);
        assert_eq!(output.selection, DynamicPlanSelection::Contingency);
        assert!(output.previous_plan_expired);
        assert!(
            output
                .admitted_actuator_effort
                .iter()
                .all(|value| *value == 0.0)
        );
        // No robot state was integrated: this transaction emits commands only.
        assert_eq!(observed.q, RobotState::zeros(&program.model).q);

        // The primary may satisfy dynamics/contact yet still cross a joint
        // position limit inside its polynomial. Admission catches that
        // analytically after mapping the actuator segment back through the
        // transmission, while a viable braking segment remains executable.
        let mut observed_limit = RobotState::zeros(&program.model);
        observed_limit.q[0] = 1.2595;
        observed_limit.v[0] = 0.05;
        let mut desired_limit = DVector::zeros(generalized_dof);
        desired_limit[6] = 100.0;
        let state_limit_in =
            FloatingDynamicControllerState::for_program(&observed_limit, &program).unwrap();
        let mut state_limit_out = state_limit_in.clone();
        let limit_wbc = FloatingDynamicWbcInput {
            state: &observed_limit,
            root_twist_world: Motion6::default(),
            desired_generalized_acceleration: &desired_limit,
            task_priorities: FloatingTaskPriorities::default(),
            task_weights: FloatingTaskWeights::default(),
            joint_posture_weight: 1.0,
            joint_acceleration_task: None,
            center_of_mass_task: None,
            centroidal_angular_momentum_task: None,
            frame_angular_acceleration_tasks: &[],
            point_acceleration_tasks: &[],
            generalized_acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
            support_patches: &[],
        };
        let status = controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 98,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc: limit_wbc,
                },
                &state_limit_in,
                &mut state_limit_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(status, DynamicStepStatus::Contingency);
        assert_eq!(output.selection, DynamicPlanSelection::Contingency);
        assert!(!output.primary_valid);
        assert!(output.contingency_valid);
        assert!(
            output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_JOINT_POSITION_LIMIT)
        );
        assert!(
            !output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_ACTUATOR_LIMIT)
        );
        assert!(output.primary_minimum_joint_position_headroom < 0.0);
        assert!(output.contingency_minimum_joint_position_headroom >= 0.0);
    }

    #[test]
    fn coupled_transmission_position_gate_uses_mapped_joint_polynomial() {
        let source = r#"
        <robot name="coupled_position">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="middle"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <link name="tip"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <joint name="first" type="revolute"><parent link="base"/><child link="middle"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="10" effort="10"/></joint>
          <joint name="second" type="revolute"><parent link="middle"/><child link="tip"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="10" effort="10"/></joint>
        </robot>"#;
        let model = crate::load_urdf(source).unwrap();
        let mut actuation = CompiledActuation::identity_from_model(&model);
        actuation.generalized_from_actuator = DMatrix::from_row_slice(2, 2, &[0.5, 0.5, -1.0, 1.0]);
        actuation.actuator_from_generalized =
            Some(DMatrix::from_row_slice(2, 2, &[1.0, -0.5, 1.0, 0.5]));
        actuation.validate(&model).unwrap();

        let actuator_start = DVector::zeros(2);
        let actuator_end = DVector::from_vec(vec![2.0, 2.0]);
        let zero = DVector::zeros(2);
        let actuator_segment = QuinticSegment::new(
            0,
            1_000_000_000,
            &actuator_start,
            &zero,
            &zero,
            &actuator_end,
            &zero,
            &zero,
        )
        .unwrap();
        let mut joint_segment = segment_workspace(2);
        map_actuator_segment_into(&actuation, &actuator_segment, &mut joint_segment);
        let actuator_mid = actuator_segment.evaluate(500_000_000);
        let joint_mid = joint_segment.evaluate(500_000_000);
        let expected = &actuation.generalized_from_actuator
            * DVector::from_column_slice(&actuator_mid.position);
        assert_eq!(joint_mid.position.as_slice(), expected.as_slice());

        let limits = vec![
            SegmentLimits {
                min_position: -1.0,
                max_position: 1.0,
                ..SegmentLimits::default()
            };
            2
        ];
        assert!(matches!(
            joint_segment.validate(&limits),
            Err(TrajectoryError::Limit {
                actuator: 0,
                quantity: "position",
                ..
            })
        ));
    }

    #[test]
    fn dynamic_admission_sweeps_primary_and_braking_collision_geometry() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/collision_sweep_toy.urdf"),
            TimingSpec::default(),
            101,
        )
        .unwrap();
        let controller = FloatingDynamicController::from_program_with_validation(
            &program,
            DynamicWbcConfig::default(),
            DynamicTrajectoryValidationConfig {
                self_collision_clearance: Some(0.02),
                ..DynamicTrajectoryValidationConfig::default()
            },
        )
        .unwrap();
        assert_eq!(controller.collision.self_pairs.len(), 1);
        let mut observed = RobotState::zeros(&program.model);
        observed.q[0] = -0.56;
        observed.v[0] = -0.5;
        let state_in = FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut state_out = state_in.clone();
        let mut output = FloatingDynamicControllerOutput::workspace(&program, 0).unwrap();
        let mut scratch = FloatingDynamicControllerScratch::for_program(&program, 0);
        let mut desired = DVector::zeros(7);
        desired[6] = -100.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(7, -100.0),
            upper: DVector::from_element(7, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(1, -1_000.0),
            upper: DVector::from_element(1, 1_000.0),
        };
        let status = controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 101,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc: FloatingDynamicWbcInput {
                        state: &observed,
                        root_twist_world: Motion6::default(),
                        desired_generalized_acceleration: &desired,
                        task_priorities: FloatingTaskPriorities::default(),
                        task_weights: FloatingTaskWeights::default(),
                        joint_posture_weight: 1.0,
                        joint_acceleration_task: None,
                        center_of_mass_task: None,
                        centroidal_angular_momentum_task: None,
                        frame_angular_acceleration_tasks: &[],
                        point_acceleration_tasks: &[],
                        generalized_acceleration_bounds: &acceleration_bounds,
                        torque_bounds: &torque_bounds,
                        actuator_effort: None,
                        contacts: &[],
                        support_patches: &[],
                    },
                },
                &state_in,
                &mut state_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(status, DynamicStepStatus::Contingency);
        assert_eq!(output.selection, DynamicPlanSelection::Contingency);
        assert!(!output.primary_collision_valid);
        assert!(output.contingency_collision_valid);
        assert!(
            output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_COLLISION)
        );
        assert_eq!(output.primary_collision.samples_evaluated, 21);
        assert_eq!(output.primary_collision.first_violation_pair_id, Some(0));
        assert!(output.primary_collision.minimum_signed_distance < 0.02);
        assert!(output.contingency_collision.minimum_signed_distance >= 0.02);
        assert!(
            output
                .admitted_actuator_effort
                .iter()
                .all(|effort| *effort == 0.0)
        );

        let solved_state_in =
            FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut solved_state_out = solved_state_in.clone();
        let mut solved_output = FloatingDynamicControllerOutput::workspace(&program, 0).unwrap();
        controller
            .advance_solved_into(
                FloatingDynamicControllerSolvedInput {
                    tick_time_ns: 0,
                    program_epoch: 101,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    observed: &observed,
                    root_twist_world: Motion6::default(),
                    solved_wbc: &output.wbc,
                },
                &solved_state_in,
                &mut solved_state_out,
                &mut solved_output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(solved_output.selection, output.selection);
        assert_eq!(
            solved_output.admission_flags.bits(),
            output.admission_flags.bits()
        );
        assert_eq!(
            solved_output.primary_collision.minimum_signed_distance,
            output.primary_collision.minimum_signed_distance
        );
        assert_eq!(
            solved_output.primary_continuous_clearance_lower_bound,
            output.primary_continuous_clearance_lower_bound
        );

        // Reconstruction uncertainty is a separate admission witness: it does
        // not alter the raw trajectory report or tracking evidence, but each
        // independently uncertain member of a self-collision pair consumes one
        // represented-point radius. Here that alone invalidates the otherwise
        // clear brake.
        let point_error =
            0.5 * (output.contingency_collision.minimum_signed_distance - 0.02) + 1e-6;
        let observation_error = RobotObservationErrorBound {
            represented_point_position_error_m: point_error,
            root_translation_error_m: 0.003,
            root_rotation_error_rad: 0.004,
            ..RobotObservationErrorBound::default()
        };
        let robust_state_in =
            FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut robust_state_out = robust_state_in.clone();
        let mut robust_output = FloatingDynamicControllerOutput::workspace(&program, 0).unwrap();
        controller
            .advance_solved_with_observation_error_into(
                FloatingDynamicControllerSolvedInput {
                    tick_time_ns: 0,
                    program_epoch: 101,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    observed: &observed,
                    root_twist_world: Motion6::default(),
                    solved_wbc: &output.wbc,
                },
                observation_error,
                &robust_state_in,
                &mut robust_state_out,
                &mut robust_output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(robust_output.selection, DynamicPlanSelection::Rejected);
        assert_eq!(
            robust_output.contingency_collision.minimum_signed_distance,
            output.contingency_collision.minimum_signed_distance
        );
        assert_eq!(
            robust_output.contingency_robust_collision_minimum_clearance_m,
            output.contingency_collision.minimum_signed_distance - 2.0 * point_error
        );
        assert_eq!(
            robust_output.command_tracking.maximum_position_error,
            output.command_tracking.maximum_position_error
        );
        assert!(
            robust_output
                .admission_flags
                .contains(DynamicAdmissionFlags::CONTINGENCY_COLLISION_OBSERVATION_ERROR)
        );
        assert!(
            !robust_output
                .admission_flags
                .contains(DynamicAdmissionFlags::CONTINGENCY_CONTINUOUS_CLEARANCE)
        );
        assert_eq!(
            robust_output
                .primary_root_prediction
                .error_growth
                .initial_translation_radius_m,
            controller
                .validation
                .root_prediction_error_growth
                .initial_translation_radius_m
                + observation_error.root_translation_error_m
        );
        assert_eq!(
            robust_output
                .primary_root_prediction
                .error_growth
                .initial_rotation_radius_rad,
            controller
                .validation
                .root_prediction_error_growth
                .initial_rotation_radius_rad
                + observation_error.root_rotation_error_rad
        );

        // This nearby command is clear at every 1 ms sample, but there is not
        // enough sampled margin to prove clearance throughout each interval.
        // Grid-only admission may select it; the conservative rate certificate
        // must fall back to the braking segment and preserve the distinction in
        // the authority flags.
        let continuous_controller = FloatingDynamicController::from_program_with_validation(
            &program,
            DynamicWbcConfig::default(),
            DynamicTrajectoryValidationConfig {
                self_collision_clearance: Some(0.02),
                collision_continuity: CollisionContinuityPolicy::ConservativeRateBound,
                ..DynamicTrajectoryValidationConfig::default()
            },
        )
        .unwrap();
        observed.q[0] = -0.5503;
        let continuous_state_in =
            FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut continuous_state_out = continuous_state_in.clone();
        let mut continuous_output =
            FloatingDynamicControllerOutput::workspace(&program, 0).unwrap();
        let continuous_status = continuous_controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 101,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc: FloatingDynamicWbcInput {
                        state: &observed,
                        root_twist_world: Motion6::default(),
                        desired_generalized_acceleration: &desired,
                        task_priorities: FloatingTaskPriorities::default(),
                        task_weights: FloatingTaskWeights::default(),
                        joint_posture_weight: 1.0,
                        joint_acceleration_task: None,
                        center_of_mass_task: None,
                        centroidal_angular_momentum_task: None,
                        frame_angular_acceleration_tasks: &[],
                        point_acceleration_tasks: &[],
                        generalized_acceleration_bounds: &acceleration_bounds,
                        torque_bounds: &torque_bounds,
                        actuator_effort: None,
                        contacts: &[],
                        support_patches: &[],
                    },
                },
                &continuous_state_in,
                &mut continuous_state_out,
                &mut continuous_output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(continuous_status, DynamicStepStatus::Contingency);
        assert_eq!(
            continuous_output.selection,
            DynamicPlanSelection::Contingency
        );
        assert!(continuous_output.primary_collision.is_clear());
        assert!(continuous_output.primary_collision.minimum_signed_distance >= 0.02);
        assert!(continuous_output.primary_continuous_clearance_lower_bound < 0.02);
        assert!(continuous_output.contingency_continuous_clearance_lower_bound >= 0.02);
        assert!(
            continuous_output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_CONTINUOUS_CLEARANCE)
        );
        assert!(
            !continuous_output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_COLLISION)
        );
    }

    #[test]
    fn dynamic_admission_withholds_world_colliding_primary_and_admits_clear_brake() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/collision_sweep_toy.urdf"),
            TimingSpec::default(),
            109,
        )
        .unwrap();
        let dimensions = [21, 21, 21];
        let spacing = Vec3::repeat(0.1);
        let origin = Vec3::new(-0.5, -1.0, -1.0);
        let mut samples = Vec::new();
        for _z in 0..dimensions[2] {
            for _y in 0..dimensions[1] {
                for x in 0..dimensions[0] {
                    samples.push(origin.x + x as f64 * spacing.x - 0.2);
                }
            }
        }
        let field = DenseSdfGrid::new(
            Transform3::from_parts(Translation3::from(origin), UnitQuaternion::identity()),
            dimensions,
            spacing,
            samples,
            SdfOutsidePolicy::Reject,
        )
        .unwrap();
        let world = CompiledWorldCollisionModel::new(
            &program.model,
            field,
            vec![WorldSphereProbe {
                stable_id: 7,
                body: program.model.body_id("slider").unwrap(),
                body_from_sphere: Transform3::identity(),
                radius_m: 0.2,
                proxy_quality: crate::collision::DistanceQuality::ExactSphere,
            }],
            0,
        )
        .unwrap();
        let validation = DynamicTrajectoryValidationConfig {
            world_collision_clearance: Some(0.02),
            world_collision_continuity: CollisionContinuityPolicy::ConservativeRateBound,
            world_collision_max_subdivision_depth: 3,
            ..DynamicTrajectoryValidationConfig::default()
        };
        assert!(matches!(
            FloatingDynamicController::from_program_with_validation(
                &program,
                DynamicWbcConfig::default(),
                validation,
            ),
            Err(DynamicControllerError::MissingWorldCollisionModel)
        ));
        let controller = FloatingDynamicController::from_program_with_world_collision(
            &program,
            DynamicWbcConfig::default(),
            validation,
            world.clone(),
        )
        .unwrap();
        let mut observed = RobotState::zeros(&program.model);
        observed.q[0] = -0.56;
        observed.v[0] = -0.5;
        let state_in = FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut state_out = state_in.clone();
        let mut output =
            FloatingDynamicControllerOutput::workspace_with_world_collision(&program, 0, &world)
                .unwrap();
        let mut scratch =
            FloatingDynamicControllerScratch::for_program_with_world_collision(&program, 0, &world);
        let mut desired = DVector::zeros(7);
        desired[6] = -100.0;
        let acceleration_bounds = VelocityBounds {
            lower: DVector::from_element(7, -100.0),
            upper: DVector::from_element(7, 100.0),
        };
        let torque_bounds = VelocityBounds {
            lower: DVector::from_element(1, -1_000.0),
            upper: DVector::from_element(1, 1_000.0),
        };
        let status = controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 109,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc: FloatingDynamicWbcInput {
                        state: &observed,
                        root_twist_world: Motion6::default(),
                        desired_generalized_acceleration: &desired,
                        task_priorities: FloatingTaskPriorities::default(),
                        task_weights: FloatingTaskWeights::default(),
                        joint_posture_weight: 1.0,
                        joint_acceleration_task: None,
                        center_of_mass_task: None,
                        centroidal_angular_momentum_task: None,
                        frame_angular_acceleration_tasks: &[],
                        point_acceleration_tasks: &[],
                        generalized_acceleration_bounds: &acceleration_bounds,
                        torque_bounds: &torque_bounds,
                        actuator_effort: None,
                        contacts: &[],
                        support_patches: &[],
                    },
                },
                &state_in,
                &mut state_out,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(status, DynamicStepStatus::Contingency);
        assert_eq!(output.selection, DynamicPlanSelection::Contingency);
        assert!(!output.primary_world_collision_valid);
        assert!(output.contingency_world_collision_valid);
        assert!(
            output
                .admission_flags
                .contains(DynamicAdmissionFlags::PRIMARY_WORLD_COLLISION)
        );
        assert_eq!(output.primary_world_collision.samples_evaluated, 21);
        assert_eq!(
            output.primary_world_collision.first_violation_probe_id,
            Some(7)
        );
        assert_eq!(
            output.primary_world_collision.first_violation_body,
            Some(program.model.body_id("slider").unwrap())
        );
        assert!(output.primary_world_collision.minimum_signed_distance_m < 0.02);
        assert!(output.contingency_world_collision.minimum_signed_distance_m >= 0.02);
        assert!(
            output
                .contingency_world_continuity
                .minimum_clearance_lower_bound_m
                >= 0.02
        );
        assert!(
            output
                .admitted_actuator_effort
                .iter()
                .all(|effort| *effort == 0.0)
        );

        // World admission carries root reconstruction uncertainty through the
        // root-prediction radius and consumes the remaining body-local point
        // radius once through the field's Lipschitz bound. The raw/root-robust
        // sweep remains available as a distinct witness.
        let point_error = (output.contingency_world_collision.minimum_signed_distance_m - 0.02)
            / world.field_lipschitz_bound
            + 1e-6;
        let observation_error = RobotObservationErrorBound {
            represented_point_position_error_m: point_error,
            ..RobotObservationErrorBound::default()
        };
        let robust_state_in =
            FloatingDynamicControllerState::for_program(&observed, &program).unwrap();
        let mut robust_state_out = robust_state_in.clone();
        let mut robust_output =
            FloatingDynamicControllerOutput::workspace_with_world_collision(&program, 0, &world)
                .unwrap();
        controller
            .advance_solved_with_observation_error_into(
                FloatingDynamicControllerSolvedInput {
                    tick_time_ns: 0,
                    program_epoch: 109,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    observed: &observed,
                    root_twist_world: Motion6::default(),
                    solved_wbc: &output.wbc,
                },
                observation_error,
                &robust_state_in,
                &mut robust_state_out,
                &mut robust_output,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(robust_output.selection, DynamicPlanSelection::Rejected);
        assert_eq!(
            robust_output
                .contingency_world_collision
                .minimum_signed_distance_m,
            output.contingency_world_collision.minimum_signed_distance_m
        );
        assert_eq!(
            robust_output.contingency_robust_world_minimum_clearance_m,
            output.contingency_world_collision.minimum_signed_distance_m
                - world.field_lipschitz_bound * point_error
        );
        assert!(
            robust_output
                .admission_flags
                .contains(DynamicAdmissionFlags::CONTINGENCY_WORLD_OBSERVATION_ERROR)
        );
        assert!(
            !robust_output
                .admission_flags
                .contains(DynamicAdmissionFlags::CONTINGENCY_WORLD_CONTINUOUS_CLEARANCE)
        );

        let short_scene = world
            .clone()
            .with_scene_stamp(WorldSceneStamp {
                scene_epoch: 44,
                source_time_ns: 0,
                valid_from_ns: 0,
                valid_until_ns: 10_000_000,
            })
            .unwrap();
        let scene_controller = FloatingDynamicController::from_program_with_world_collision(
            &program,
            DynamicWbcConfig::default(),
            DynamicTrajectoryValidationConfig {
                world_collision_clearance: Some(0.02),
                world_collision_continuity: CollisionContinuityPolicy::ConservativeRateBound,
                world_collision_max_subdivision_depth: 3,
                expected_world_scene_epoch: Some(44),
                maximum_world_scene_age_ns: Some(5_000_000),
                require_world_scene_horizon_validity: true,
                ..DynamicTrajectoryValidationConfig::default()
            },
            short_scene.clone(),
        )
        .unwrap();
        let mut scene_output = FloatingDynamicControllerOutput::workspace_with_world_collision(
            &program,
            0,
            &short_scene,
        )
        .unwrap();
        let mut scene_scratch = FloatingDynamicControllerScratch::for_program_with_world_collision(
            &program,
            0,
            &short_scene,
        );
        let scene_status = scene_controller
            .advance_into(
                FloatingDynamicControllerInput {
                    tick_time_ns: 0,
                    program_epoch: 109,
                    observation_stamp: RobotObservationStamp::exact_at(0, 1, 1),
                    wbc: FloatingDynamicWbcInput {
                        state: &observed,
                        root_twist_world: Motion6::default(),
                        desired_generalized_acceleration: &desired,
                        task_priorities: FloatingTaskPriorities::default(),
                        task_weights: FloatingTaskWeights::default(),
                        joint_posture_weight: 1.0,
                        joint_acceleration_task: None,
                        center_of_mass_task: None,
                        centroidal_angular_momentum_task: None,
                        frame_angular_acceleration_tasks: &[],
                        point_acceleration_tasks: &[],
                        generalized_acceleration_bounds: &acceleration_bounds,
                        torque_bounds: &torque_bounds,
                        actuator_effort: None,
                        contacts: &[],
                        support_patches: &[],
                    },
                },
                &state_in,
                &mut state_out,
                &mut scene_output,
                &mut scene_scratch,
            )
            .unwrap();
        assert_eq!(scene_status, DynamicStepStatus::Rejected);
        assert_eq!(scene_output.selection, DynamicPlanSelection::Rejected);
        assert_eq!(
            scene_output.world_scene.validity,
            WorldSceneValidity::HorizonExpired
        );
        assert!(
            scene_output
                .admission_flags
                .contains(DynamicAdmissionFlags::WORLD_SCENE_INVALID)
        );
        assert!(!scene_output.primary_world_collision.is_known());
        assert!(!scene_output.contingency_world_collision.is_known());
    }

    #[test]
    fn collision_enabled_program_requires_explicit_unknown_geometry_policy() {
        let source = r#"
        <robot name="unknown_collision">
          <link name="base"><collision><geometry><mesh filename="package://fixture/unknown.stl"/></geometry></collision><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <link name="tip"><inertial><mass value=".1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <joint name="turn" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
        </robot>"#;
        let program = MotionProgram::compile_urdf(source, TimingSpec::default(), 102).unwrap();
        assert!(matches!(
            FloatingDynamicController::from_program_with_validation(
                &program,
                DynamicWbcConfig::default(),
                DynamicTrajectoryValidationConfig {
                    collision_max_subdivision_depth: 13,
                    ..DynamicTrajectoryValidationConfig::default()
                },
            ),
            Err(DynamicControllerError::InvalidValidationConfig)
        ));
        assert!(matches!(
            FloatingDynamicController::from_program_with_validation(
                &program,
                DynamicWbcConfig::default(),
                DynamicTrajectoryValidationConfig {
                    self_collision_clearance: Some(0.0),
                    ..DynamicTrajectoryValidationConfig::default()
                },
            ),
            Err(DynamicControllerError::UnsupportedCollisionGeometry { count: 1 })
        ));
        FloatingDynamicController::from_program_with_validation(
            &program,
            DynamicWbcConfig::default(),
            DynamicTrajectoryValidationConfig {
                self_collision_clearance: Some(0.0),
                unknown_collision_geometry: UnknownCollisionGeometryPolicy::IgnoreExplicitly,
                ..DynamicTrajectoryValidationConfig::default()
            },
        )
        .unwrap();
    }
}
